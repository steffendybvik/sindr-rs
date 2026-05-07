//! Newton-Raphson nonlinear solver.
//!
//! Orchestrates iterative solving for circuits containing nonlinear
//! elements (diodes, LEDs). Each iteration builds a fresh MNA system
//! with companion models evaluated at the current operating point,
//! solves it, applies voltage limiting, and checks convergence.

use nalgebra::DVector;

use sindr_devices::bjt::{BjtKind, BjtParams};
use sindr_devices::diode::{self, temperature_scale_is, DiodeParams, GMIN};

use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;
use crate::results::{self, SimulationResult};
use crate::stamp;

/// Maximum Newton-Raphson iterations before declaring non-convergence.
pub const MAX_NR_ITERATIONS: usize = 100;

/// Absolute voltage tolerance (V).
pub const V_ABSTOL: f64 = 1e-6;

/// Absolute current tolerance (A).
#[allow(dead_code)]
pub const I_ABSTOL: f64 = 1e-12;

/// Relative tolerance (dimensionless).
pub const RELTOL: f64 = 1e-3;

/// Solve a circuit containing nonlinear elements via Newton-Raphson iteration.
///
/// # Algorithm
///
/// 1. Start with all node voltages at zero.
/// 2. Each iteration: stamp all components (with companion models at v_prev),
///    add Gmin shunts, solve, apply voltage limiting, check convergence.
/// 3. Return results when converged, or error after MAX_NR_ITERATIONS.
///
pub fn solve_nonlinear(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
) -> Result<SimulationResult, SimError> {
    let (result, _) = nr_inner(circuit, node_map, num_nodes, num_vsources, None)?;
    Ok(result)
}

/// Inner NR iteration loop. Returns simulation result and final voltage vector.
fn nr_inner(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    v_init: Option<&DVector<f64>>,
) -> Result<(SimulationResult, DVector<f64>), SimError> {
    let size = num_nodes + num_vsources;
    let mut v_prev = match v_init {
        Some(v) => v.clone(),
        None => {
            let mut v = DVector::zeros(size);
            init_bjt_voltages(circuit, node_map, &mut v);
            v
        }
    };

    let diode_info = collect_diode_info(circuit, node_map);
    let debug_nr = std::env::var("DEBUG_NR").is_ok();
    for _iteration in 0..MAX_NR_ITERATIONS {
        let mut system = MnaSystem::new(num_nodes, num_vsources);
        stamp::stamp_circuit(circuit, &mut system, node_map, Some(&v_prev))?;

        // Add Gmin shunts
        add_gmin_shunts(&mut system, num_nodes);

        let v_new = system.solve()?;

        let v_limited = apply_voltage_limiting(&v_new, &v_prev, &diode_info);

        if debug_nr && _iteration < 30 {
            eprintln!(
                "NR iter {}: v_prev={:.4?} -> v_new={:.4?} -> v_limited={:.4?}",
                _iteration,
                v_prev.iter().take(num_nodes).collect::<Vec<_>>(),
                v_new.iter().take(num_nodes).collect::<Vec<_>>(),
                v_limited.iter().take(num_nodes).collect::<Vec<_>>()
            );
        }

        if converged(&v_prev, &v_limited, num_nodes) {
            let result = results::extract_results(circuit, node_map, &v_limited, num_nodes);
            return Ok((result, v_limited));
        }

        v_prev = v_limited;
    }

    Err(SimError::ConvergenceFailed)
}

/// Information about a diode/junction needed for voltage limiting.
pub(crate) struct DiodeLimitInfo {
    anode_idx: Option<usize>,
    cathode_idx: Option<usize>,
    v_crit: f64,
    n: f64,
    /// For BJT junctions: the non-base node index that should absorb
    /// voltage limiting corrections. When set, only this node is adjusted
    /// (the base node is left untouched to prevent B-E and B-C limiters
    /// from fighting each other on the shared base). None for regular diodes.
    correction_node: Option<CorrectionTarget>,
    /// Whether this is a BJT junction (enables bilateral step limiting).
    is_bjt: bool,
}

/// Which node to apply voltage limiting corrections to.
enum CorrectionTarget {
    /// Apply correction to the anode only (non-base node is the anode).
    Anode,
    /// Apply correction to the cathode only (non-base node is the cathode).
    Cathode,
}

/// Collect diode/LED node indices and parameters for voltage limiting.
pub(crate) fn collect_diode_info(circuit: &Circuit, node_map: &NodeMap) -> Vec<DiodeLimitInfo> {
    let mut info = Vec::new();

    for component in &circuit.components {
        match component {
            CircuitElement::Diode {
                nodes, temperature, ..
            } => {
                let mut params = DiodeParams::silicon();
                if (*temperature - 300.15).abs() > 1e-6 {
                    params.is = temperature_scale_is(params.is, *temperature, 300.15, 1.11, 2.0);
                }
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(params.is, params.n),
                    n: params.n,
                    correction_node: None,
                    is_bjt: false,
                });
            }
            CircuitElement::Led {
                nodes,
                color,
                temperature,
                ..
            } => {
                let mut params = DiodeParams::for_led_color(color);
                if (*temperature - 300.15).abs() > 1e-6 {
                    params.is = temperature_scale_is(params.is, *temperature, 300.15, 1.11, 2.0);
                }
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(params.is, params.n),
                    n: params.n,
                    correction_node: None,
                    is_bjt: false,
                });
            }
            CircuitElement::ZenerDiode { nodes, .. } => {
                // Forward region uses silicon Shockley model (n=1, IS=1e-14)
                let params = DiodeParams::silicon();
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(params.is, params.n),
                    n: params.n,
                    correction_node: None,
                    is_bjt: false,
                });
            }
            CircuitElement::SchottkyDiode { nodes, .. } => {
                let schottky_params = sindr_devices::schottky::SchottkyParams::default();
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(schottky_params.is, schottky_params.n),
                    n: schottky_params.n,
                    correction_node: None,
                    is_bjt: false,
                });
            }
            CircuitElement::Photodiode { nodes, .. } => {
                let photo_params = sindr_devices::photodiode::PhotodiodeParams::default();
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(photo_params.is, photo_params.n),
                    n: photo_params.n,
                    correction_node: None,
                    is_bjt: false,
                });
            }
            CircuitElement::Bjt {
                nodes,
                kind,
                bf,
                temperature,
                ..
            } => {
                let mut params = BjtParams::new(*bf);
                if (*temperature - 300.15).abs() > 1e-6 {
                    params.is = temperature_scale_is(params.is, *temperature, 300.15, 1.11, 3.0);
                }

                // B-E junction: standard diode limiting (splits between anode/cathode
                // or full delta to non-ground node). This allows the B-E limiter
                // to control the base voltage appropriately.
                let (be_anode, be_cathode) = match kind {
                    BjtKind::Npn => (&nodes[0], &nodes[2]),
                    BjtKind::Pnp => (&nodes[2], &nodes[0]),
                };
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(be_anode),
                    cathode_idx: node_map.index(be_cathode),
                    v_crit: diode::critical_voltage(params.is, params.nf),
                    n: params.nf,
                    correction_node: None,
                    is_bjt: true,
                });

                // B-C junction: correction goes ONLY to the collector node.
                // This prevents the B-C limiter from moving the base node,
                // avoiding fights with the B-E limiter on the shared base.
                let (bc_anode, bc_cathode) = match kind {
                    BjtKind::Npn => (&nodes[0], &nodes[1]),
                    BjtKind::Pnp => (&nodes[1], &nodes[0]),
                };
                let bc_correction = match kind {
                    BjtKind::Npn => CorrectionTarget::Cathode,
                    BjtKind::Pnp => CorrectionTarget::Anode,
                };
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(bc_anode),
                    cathode_idx: node_map.index(bc_cathode),
                    v_crit: diode::critical_voltage(params.is, params.nr),
                    n: params.nr,
                    correction_node: Some(bc_correction),
                    is_bjt: true,
                });
            }
            _ => {}
        }
    }

    info
}

/// Apply voltage limiting to all diode/junction node voltages.
///
/// For each diode/junction, compute the proposed voltage from v_new,
/// limit it, and adjust node voltages accordingly.
///
/// All junctions (standalone diodes and BJT junctions) use the same
/// limiting strategy: split the correction between anode and cathode,
/// or apply the full delta to the non-ground node when one terminal
/// is grounded. This prevents BJT junction limiters from fighting
/// each other on a shared base node.
pub(crate) fn apply_voltage_limiting(
    v_new: &DVector<f64>,
    v_prev: &DVector<f64>,
    diode_info: &[DiodeLimitInfo],
) -> DVector<f64> {
    let mut v_limited = v_new.clone();

    for info in diode_info {
        let v_anode_new = info.anode_idx.map_or(0.0, |i| v_limited[i]);
        let v_cathode_new = info.cathode_idx.map_or(0.0, |i| v_limited[i]);
        let v_d_new = v_anode_new - v_cathode_new;

        let v_anode_old = info.anode_idx.map_or(0.0, |i| v_prev[i]);
        let v_cathode_old = info.cathode_idx.map_or(0.0, |i| v_prev[i]);
        let v_d_old = v_anode_old - v_cathode_old;

        let mut v_d_limited = diode::limit_voltage(v_d_new, v_d_old, info.v_crit, info.n);

        // For BJT junctions, also apply bilateral step limiting.
        // The standard SPICE limiter only clamps forward-bias overshoots.
        // In saturation transitions, a junction voltage can swing far negative
        // (e.g., B-E goes from 0.7V to -0.3V), turning off the junction and
        // destabilizing the solver. Clamp the step to prevent this.
        if info.is_bjt {
            let max_step = 10.0 * info.n * sindr_devices::diode::V_T; // ~0.26V
            let step = v_d_limited - v_d_old;
            if step.abs() > max_step {
                v_d_limited = v_d_old + step.signum() * max_step;
            }
        }

        let delta = v_d_limited - v_d_new;

        if delta.abs() <= 1e-15 {
            continue;
        }

        match &info.correction_node {
            Some(CorrectionTarget::Anode) => {
                // BJT junction: apply full correction to anode (non-base node)
                if let Some(ai) = info.anode_idx {
                    v_limited[ai] += delta;
                }
            }
            Some(CorrectionTarget::Cathode) => {
                // BJT junction: apply full correction to cathode (non-base node)
                if let Some(ci) = info.cathode_idx {
                    v_limited[ci] -= delta;
                }
            }
            None => {
                // Standard diode: split adjustment between anode and cathode;
                // if one is ground, apply the full delta to the non-ground node.
                let both_present = info.anode_idx.is_some() && info.cathode_idx.is_some();
                let share = if both_present { 0.5 } else { 1.0 };
                if let Some(ai) = info.anode_idx {
                    v_limited[ai] += delta * share;
                }
                if let Some(ci) = info.cathode_idx {
                    v_limited[ci] -= delta * share;
                }
            }
        }
    }

    v_limited
}

/// Add Gmin conductance shunts to every node diagonal.
///
/// This prevents the Jacobian from becoming singular when diodes
/// are in reverse bias (near-zero conductance).
pub(crate) fn add_gmin_shunts(system: &mut MnaSystem, num_nodes: usize) {
    for i in 0..num_nodes {
        system.a[(i, i)] += GMIN;
    }
}

/// Check if Newton-Raphson has converged.
///
/// Uses SPICE-style convergence criterion: for each node voltage,
/// |v_new - v_prev| <= V_ABSTOL + RELTOL * max(|v_new|, |v_prev|)
pub(crate) fn converged(v_prev: &DVector<f64>, v_new: &DVector<f64>, num_nodes: usize) -> bool {
    for i in 0..num_nodes {
        let diff = (v_new[i] - v_prev[i]).abs();
        let tol = V_ABSTOL + RELTOL * v_new[i].abs().max(v_prev[i].abs());
        if diff > tol {
            return false;
        }
    }
    true
}

/// Set initial voltage guesses for BJT circuits.
///
/// Strategy: First solve the linear-only subcircuit (treating BJTs as open circuits)
/// to get reasonable node voltages from voltage sources and resistor networks.
/// Then overlay BJT junction voltage guesses (Vbe ~ 0.65V).
///
/// This dramatically improves convergence for saturation and PNP circuits
/// by starting the NR iteration near the physical solution.
fn init_bjt_voltages(circuit: &Circuit, node_map: &NodeMap, v_prev: &mut DVector<f64>) {
    const VBE_GUESS: f64 = 0.65;

    let has_nonlinear = circuit.components.iter().any(|c| {
        matches!(
            c,
            CircuitElement::Bjt { .. }
                | CircuitElement::Mosfet { .. }
                | CircuitElement::ZenerDiode { .. }
                | CircuitElement::SchottkyDiode { .. }
                | CircuitElement::Photodiode { .. }
        )
    });
    if !has_nonlinear {
        return;
    }

    // Step 1: Solve the linear subcircuit to get baseline node voltages.
    // This gives us the supply rail voltages and resistor divider biases.
    let num_nodes = node_map.num_nodes();
    let num_vsources = circuit.count_voltage_sources();
    let mut system = MnaSystem::new(num_nodes, num_vsources);

    // Stamp only linear components (skip nonlinear ones)
    let mut vsource_index: usize = 0;
    for component in &circuit.components {
        match component {
            CircuitElement::Resistor {
                nodes, resistance, ..
            } => {
                if *resistance > 0.0 {
                    stamp::stamp_resistor(&mut system, node_map, nodes, *resistance);
                }
            }
            CircuitElement::VoltageSource { nodes, voltage, .. } => {
                let branch = num_nodes + vsource_index;
                stamp::stamp_voltage_source(&mut system, node_map, nodes, *voltage, branch);
                vsource_index += 1;
            }
            CircuitElement::CurrentSource { nodes, current, .. } => {
                stamp::stamp_current_source(&mut system, node_map, nodes, *current);
            }
            _ => {} // Skip BJTs, diodes, etc.
        }
    }

    // Add Gmin shunts to prevent singularity for floating BJT nodes
    add_gmin_shunts(&mut system, num_nodes);

    if let Ok(linear_solution) = system.solve() {
        // Copy the linear solution as our initial guess
        for i in 0..v_prev.len().min(linear_solution.len()) {
            v_prev[i] = linear_solution[i];
        }
    }

    // Step 2: Adjust base voltages for BJT Vbe guess
    for component in &circuit.components {
        if let CircuitElement::Bjt { nodes, kind, .. } = component {
            let base_idx = node_map.index(&nodes[0]);
            let emitter_idx = node_map.index(&nodes[2]);

            let ve = emitter_idx.map_or(0.0, |i| v_prev[i]);
            match kind {
                BjtKind::Npn => {
                    if let Some(bi) = base_idx {
                        // Only adjust if current base voltage doesn't already
                        // provide reasonable Vbe (e.g. from resistor divider)
                        let current_vbe = v_prev[bi] - ve;
                        if !(0.3..=1.0).contains(&current_vbe) {
                            v_prev[bi] = ve + VBE_GUESS;
                        }
                    }
                }
                BjtKind::Pnp => {
                    if let Some(bi) = base_idx {
                        let current_vbe = v_prev[bi] - ve;
                        if !(-1.0..=-0.3).contains(&current_vbe) {
                            v_prev[bi] = ve - VBE_GUESS;
                        }
                    }
                }
            }
        }
    }

    // Step 3: Clamp ZenerDiode initial operating point to within realistic range.
    //
    // The linear solve treats zeners as open circuits, so the anode/cathode voltage
    // from the linear solve can be far outside the zener's operating range (e.g. 5V
    // forward when the actual forward drop is ~0.65V). This causes the companion model
    // to produce pathological currents and the NR solver to take tiny steps.
    //
    // Fix: set v_d_init = clamp(v_d_linear, -(vz + 0.1), +0.4) so the NR iteration
    // starts within a factor of ~2 of the actual solution. The voltage limiter handles
    // the rest.
    for component in &circuit.components {
        if let CircuitElement::ZenerDiode { nodes, vz, .. } = component {
            let anode_idx = node_map.index(&nodes[0]);
            let cathode_idx = node_map.index(&nodes[1]);

            let v_anode = anode_idx.map_or(0.0, |i| v_prev[i]);
            let v_cathode = cathode_idx.map_or(0.0, |i| v_prev[i]);
            let v_d = v_anode - v_cathode;

            // v_crit for silicon is approx 0.65V; use 0.4V as a safe initial point
            const V_INIT_FORWARD: f64 = 0.4;
            let v_crit_zener = vz + 0.1; // Allow slight overshoot past breakdown

            let v_d_clamped = v_d.clamp(-v_crit_zener, V_INIT_FORWARD);

            if (v_d_clamped - v_d).abs() > 1e-9 {
                // Adjust the anode node voltage; keep cathode fixed
                if let Some(ai) = anode_idx {
                    v_prev[ai] = v_cathode + v_d_clamped;
                } else if let Some(ci) = cathode_idx {
                    // Anode is ground; adjust cathode instead (opposite sign)
                    v_prev[ci] = -v_d_clamped;
                }
            }
        }
    }
}
