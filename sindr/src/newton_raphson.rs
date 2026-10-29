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
    solve_nonlinear_with_seeds(circuit, node_map, num_nodes, num_vsources, None)
}

fn solve_nonlinear_with_seeds(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    initial_voltages: Option<&std::collections::HashMap<String, f64>>,
) -> Result<SimulationResult, SimError> {
    // First attempt: plain Newton–Raphson with the default GMIN floor.
    match nr_inner(
        circuit,
        node_map,
        num_nodes,
        num_vsources,
        None,
        initial_voltages,
        0.0,
    ) {
        Ok((result, _)) => Ok(result),
        Err(SimError::ConvergenceFailed { .. }) => {
            // Fall back to gmin stepping: progressively softens the Jacobian
            // by adding extra conductance to ground on every node, then ramps
            // it down while warm-starting from the previous step's solution.
            gmin_stepping(circuit, node_map, num_nodes, num_vsources, initial_voltages)
        }
        // SingularMatrix and InvalidSolution are topology / numerical errors
        // that gmin stepping cannot fix — propagate immediately.
        Err(e) => Err(e),
    }
}

/// Gmin-stepping homotopy fallback for nonlinear DC.
///
/// Solves the circuit at a sequence of decreasing gmin shunts, warm-starting
/// each step from the previous solution. Recovers convergence on circuits
/// where plain Newton–Raphson fails to find a starting point — e.g. diodes
/// or BJTs whose default initial guess sits on the wrong branch of the I-V
/// curve.
fn gmin_stepping(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    initial_voltages: Option<&std::collections::HashMap<String, f64>>,
) -> Result<SimulationResult, SimError> {
    // Geometric ladder, ending at 0.0 (default GMIN floor).
    const LADDER: &[f64] = &[1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-8, 1e-10, 0.0];

    let mut warm_start: Option<DVector<f64>> = None;
    let mut last_err = SimError::ConvergenceFailed {
        iterations: 0,
        max_step_volts: f64::INFINITY,
    };

    for &gmin_extra in LADDER {
        let seeds = if warm_start.is_some() {
            None
        } else {
            initial_voltages
        };
        match nr_inner(
            circuit,
            node_map,
            num_nodes,
            num_vsources,
            warm_start.as_ref(),
            seeds,
            gmin_extra,
        ) {
            Ok((result, v)) => {
                if gmin_extra == 0.0 {
                    return Ok(result);
                }
                warm_start = Some(v);
            }
            Err(e) => {
                last_err = e;
                // Couldn't soften enough to converge, or ramp-down step
                // failed — bail with the diagnostic.
                return Err(last_err);
            }
        }
    }

    // Should be unreachable: the final ladder entry is 0.0 which returns Ok above.
    Err(last_err)
}

/// Like [`solve_nonlinear`] but seeds the Newton initial guess with the
/// supplied per-node voltages (V), keyed by node name. Equivalent to SPICE's
/// `.NODESET` directive — useful when a circuit has multiple operating
/// points or when default heuristics fail to converge. Unknown node names
/// are silently ignored; nodes not in the map fall back to the default
/// initialisation.
pub fn solve_nonlinear_with_initial_voltages(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    initial_voltages: &std::collections::HashMap<String, f64>,
) -> Result<SimulationResult, SimError> {
    solve_nonlinear_with_seeds(
        circuit,
        node_map,
        num_nodes,
        num_vsources,
        Some(initial_voltages),
    )
}

/// Inner NR iteration loop. Returns simulation result and final voltage vector.
fn nr_inner(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    v_init: Option<&DVector<f64>>,
    initial_voltages: Option<&std::collections::HashMap<String, f64>>,
    gmin_extra: f64,
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

    // Apply user-supplied initial node voltages on top of heuristic init.
    if let Some(seed) = initial_voltages {
        for (name, value) in seed {
            if let Some(idx) = node_map.index(name) {
                v_prev[idx] = *value;
            }
        }
    }

    let diode_info = collect_diode_info(circuit, node_map);
    let igbt_info = collect_igbt_info(circuit, node_map);
    let debug_nr = std::env::var("DEBUG_NR").is_ok();
    let mut last_step = f64::INFINITY;
    for _iteration in 0..MAX_NR_ITERATIONS {
        let mut system = MnaSystem::new(num_nodes, num_vsources);
        stamp::stamp_circuit(circuit, &mut system, node_map, Some(&v_prev))?;

        // Add Gmin shunts (default GMIN floor + any homotopy extra).
        add_gmin_shunts(&mut system, num_nodes);
        if gmin_extra > 0.0 {
            for i in 0..num_nodes {
                system.a[(i, i)] += gmin_extra;
            }
        }

        let v_new = system.solve()?;

        let mut v_limited = apply_voltage_limiting(&v_new, &v_prev, &diode_info);
        apply_igbt_limiting(&mut v_limited, &v_prev, &igbt_info);

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

        last_step = max_node_step(&v_prev, &v_limited, num_nodes);
        v_prev = v_limited;
    }

    Err(SimError::ConvergenceFailed {
        iterations: MAX_NR_ITERATIONS,
        max_step_volts: last_step,
    })
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
    /// Breakdown voltage for a Zener, which is also driven in reverse.
    zener_vz: Option<f64>,
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
                    zener_vz: None,
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
                    zener_vz: None,
                });
            }
            CircuitElement::ZenerDiode { nodes, vz, .. } => {
                // Forward region uses silicon Shockley model (n=1, IS=1e-14)
                let params = DiodeParams::silicon();
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(params.is, params.n),
                    n: params.n,
                    correction_node: None,
                    is_bjt: false,
                    zener_vz: Some(*vz),
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
                    zener_vz: None,
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
                    zener_vz: None,
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
                    zener_vz: None,
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
                    zener_vz: None,
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

    // Limiting one junction at a time can undo another. Two diodes in series
    // share a node: the second one's correction moves it after the first was
    // limited, and the first can end the step volts forward (the next stamp
    // then overflows). When that happens, fall back to a damped Newton step:
    // keep the raw step's direction and shorten it until every junction is
    // within what its limiter allows. Junction voltages are linear in the
    // node voltages, so one factor satisfies them all.
    let junction = |info: &DiodeLimitInfo, v: &DVector<f64>| {
        info.anode_idx.map_or(0.0, |i| v[i]) - info.cathode_idx.map_or(0.0, |i| v[i])
    };
    let overshoots = diode_info.iter().any(|info| {
        let (old, fin) = (junction(info, v_prev), junction(info, &v_limited));
        fin > diode::limit_voltage(fin, old, info.v_crit, info.n) + 1e-9
    });
    if overshoots {
        let mut alpha: f64 = 1.0;
        for info in diode_info {
            let (old, raw) = (junction(info, v_prev), junction(info, v_new));
            let allowed = diode::limit_voltage(raw, old, info.v_crit, info.n);
            if raw > allowed && allowed > old {
                alpha = alpha.min((allowed - old) / (raw - old));
            }
        }
        v_limited = v_prev + (v_new - v_prev) * alpha;
    }
    v_limited
}

/// Collector and emitter indices of an IGBT, for [`apply_igbt_limiting`].
pub(crate) struct IgbtLimitInfo {
    collector_idx: Option<usize>,
    emitter_idx: Option<usize>,
}

/// Collect the IGBTs' collector/emitter node indices for VCE limiting.
pub(crate) fn collect_igbt_info(circuit: &Circuit, node_map: &NodeMap) -> Vec<IgbtLimitInfo> {
    circuit
        .components
        .iter()
        .filter_map(|component| match component {
            CircuitElement::Igbt { nodes, .. } => Some(IgbtLimitInfo {
                collector_idx: node_map.index(&nodes[1]),
                emitter_idx: node_map.index(&nodes[2]),
            }),
            _ => None,
        })
        .collect()
}

/// Keep a forward-biased IGBT from stepping into reverse blocking in one
/// Newton step.
///
/// The IGBT carries no current and has zero conductance for `vce <= 0`, and
/// zero output conductance in saturation, so an unlimited Newton step can
/// bounce between the two without converging. As SPICE's `limvds` does for
/// MOSFETs, a step that would take `vce` from positive to `<= 0` lands at
/// `vce_old / 2` instead, which walks it down into the triode region where
/// Newton converges. The correction goes to the collector (the emitter if
/// the collector is ground).
pub(crate) fn apply_igbt_limiting(
    v_limited: &mut DVector<f64>,
    v_prev: &DVector<f64>,
    igbt_info: &[IgbtLimitInfo],
) {
    for info in igbt_info {
        let vce = |v: &DVector<f64>| {
            info.collector_idx.map_or(0.0, |i| v[i]) - info.emitter_idx.map_or(0.0, |i| v[i])
        };
        let (vce_old, vce_new) = (vce(v_prev), vce(v_limited));
        if vce_old <= 0.0 || vce_new > 0.0 {
            continue;
        }
        let delta = vce_old / 2.0 - vce_new;
        match (info.collector_idx, info.emitter_idx) {
            (Some(ci), _) => v_limited[ci] += delta,
            (None, Some(ei)) => v_limited[ei] -= delta,
            (None, None) => {}
        }
    }
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

/// Largest per-node voltage step `max_i |v_new[i] − v_prev[i]|` (V).
///
/// Used to populate [`SimError::ConvergenceFailed`] diagnostics when NR
/// runs out of iterations.
pub(crate) fn max_node_step(v_prev: &DVector<f64>, v_new: &DVector<f64>, num_nodes: usize) -> f64 {
    let mut max = 0.0f64;
    for i in 0..num_nodes {
        let diff = (v_new[i] - v_prev[i]).abs();
        if diff > max {
            max = diff;
        }
    }
    max
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
            } if *resistance > 0.0 => {
                stamp::stamp_resistor(&mut system, node_map, nodes, *resistance);
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

    // Step 3: Clamp each diode-family junction's starting point.
    //
    // The linear solve treats diodes as open circuits, so a junction can start
    // with the whole supply across it (e.g. 5 V forward when the real drop is
    // ~0.3 V). From there Newton on the exponential only moves down by about
    // n·V_T per iteration, and runs out of iterations long before it arrives.
    // Start forward-biased junctions at their critical voltage instead, as
    // SPICE does; the voltage limiter takes it from there. A Zener's reverse
    // side is held just past breakdown for the same reason.
    for info in collect_diode_info(circuit, node_map)
        .iter()
        .filter(|info| !info.is_bjt)
    {
        let v_anode = info.anode_idx.map_or(0.0, |i| v_prev[i]);
        let v_cathode = info.cathode_idx.map_or(0.0, |i| v_prev[i]);
        let v_d = v_anode - v_cathode;

        let v_min = info.zener_vz.map_or(f64::NEG_INFINITY, |vz| -(vz + 0.1));
        let v_d_clamped = v_d.clamp(v_min, info.v_crit);

        if (v_d_clamped - v_d).abs() > 1e-9 {
            // Move the anode and keep the cathode fixed; if the anode is
            // ground, move the cathode instead.
            if let Some(ai) = info.anode_idx {
                v_prev[ai] = v_cathode + v_d_clamped;
            } else if let Some(ci) = info.cathode_idx {
                v_prev[ci] = -v_d_clamped;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::CircuitElement;

    fn diode_resistor_circuit(vsource: f64) -> Circuit {
        Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: vsource,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 100.0,
                },
                CircuitElement::Diode {
                    id: "D1".into(),
                    nodes: ["n2".into(), "0".into()],
                    temperature: 300.15,
                },
            ],
        }
    }

    #[test]
    fn max_node_step_picks_largest_diff() {
        let prev = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0]);
        let new = DVector::from_vec(vec![1.05, 1.9, 3.5, 4.0]);
        // Diffs: 0.05, 0.1, 0.5, 0.0  → max = 0.5 over the first 4 nodes.
        let r = max_node_step(&prev, &new, 4);
        assert!((r - 0.5).abs() < 1e-12, "got {r}");
    }

    #[test]
    fn max_node_step_only_considers_node_range() {
        // num_nodes = 2 means branch unknowns at index 2,3 are ignored.
        let prev = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0]);
        let new = DVector::from_vec(vec![1.01, 2.01, 99.0, 99.0]);
        let r = max_node_step(&prev, &new, 2);
        assert!((r - 0.01).abs() < 1e-12, "got {r}");
    }

    /// Direct exercise of the gmin-stepping homotopy. Bypasses the
    /// plain-NR first attempt and verifies the homotopy alone produces
    /// the correct DC operating point on a diode-resistor divider.
    #[test]
    fn gmin_stepping_solves_diode_resistor_directly() {
        let circuit = diode_resistor_circuit(5.0);
        let node_map = NodeMap::from_circuit(&circuit);
        let num_nodes = node_map.num_nodes();
        let num_vsources = circuit.count_voltage_sources();

        let result = gmin_stepping(&circuit, &node_map, num_nodes, num_vsources, None)
            .expect("gmin stepping should converge on a diode-R circuit");

        // Forward-biased silicon diode: V(n2) ≈ 0.65 V, current ≈ 43 mA.
        let v_n2 = result.node_voltages["n2"];
        assert!(
            (0.5..=0.9).contains(&v_n2),
            "expected forward-bias diode drop, got V(n2) = {v_n2}"
        );

        // The standard solver should agree (modulo gmin bias).
        let plain = crate::solve_circuit(&circuit).expect("plain solve should converge");
        let v_n2_plain = plain.node_voltages["n2"];
        assert!(
            (v_n2 - v_n2_plain).abs() < 1e-3,
            "gmin path ({v_n2}) and plain NR ({v_n2_plain}) should agree to ~mV"
        );
    }

    /// gmin stepping with a wildly wrong initial seed must still converge.
    /// This is the "rescue" use case — plain NR with this seed often diverges.
    #[test]
    fn gmin_stepping_rescues_pathological_initial_seed() {
        let circuit = diode_resistor_circuit(5.0);
        let node_map = NodeMap::from_circuit(&circuit);
        let num_nodes = node_map.num_nodes();
        let num_vsources = circuit.count_voltage_sources();

        let mut bad_seed = std::collections::HashMap::new();
        bad_seed.insert("n1".to_string(), 1e6);
        bad_seed.insert("n2".to_string(), -1e6);

        let result = gmin_stepping(
            &circuit,
            &node_map,
            num_nodes,
            num_vsources,
            Some(&bad_seed),
        )
        .expect("gmin stepping should rescue absurd initial seed");

        let v_n2 = result.node_voltages["n2"];
        assert!(
            (0.5..=0.9).contains(&v_n2),
            "rescued solution should still be physical, got V(n2) = {v_n2}"
        );
    }

    /// Solver still produces correct results on a normal nonlinear
    /// circuit — the gmin fallback wrapper must not regress the easy path.
    #[test]
    fn solve_nonlinear_still_correct_for_easy_circuit() {
        let circuit = diode_resistor_circuit(5.0);
        let result = crate::solve_circuit(&circuit).expect("should converge");
        let v_n2 = result.node_voltages["n2"];
        assert!((0.5..=0.9).contains(&v_n2), "V(n2) = {v_n2}");
    }

    /// At very low forward bias the diode current is tiny and convergence
    /// is sensitive — exercises the homotopy + warm-start chain end-to-end.
    #[test]
    fn gmin_stepping_handles_low_bias() {
        let circuit = diode_resistor_circuit(0.3);
        let node_map = NodeMap::from_circuit(&circuit);
        let num_nodes = node_map.num_nodes();
        let num_vsources = circuit.count_voltage_sources();

        let result = gmin_stepping(&circuit, &node_map, num_nodes, num_vsources, None)
            .expect("low-bias diode should still converge via homotopy");
        let v_n2 = result.node_voltages["n2"];
        // 0.3 V applied across 100 Ω + diode → diode is below knee, V(n2) ≈ V(n1) ≈ 0.3 V
        assert!(
            (0.0..=0.35).contains(&v_n2),
            "low-bias diode should sit below knee, got V(n2) = {v_n2}"
        );
    }

    /// Forward drop of a junction (`IS`, `n`) fed from `vs` through `r`,
    /// solved directly from `V = n·V_T·ln((vs − V)/(r·IS) + 1)`.
    fn shockley_drop(vs: f64, r: f64, is: f64, n: f64) -> f64 {
        let nvt = n * sindr_devices::diode::V_T;
        let mut v = 0.5;
        for _ in 0..200 {
            v = nvt * ((vs - v) / (r * is) + 1.0).ln();
        }
        v
    }

    fn junction_behind(element: CircuitElement) -> Circuit {
        Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["in".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["in".into(), "out".into()],
                    resistance: 1_000.0,
                },
                element,
            ],
        }
    }

    #[test]
    fn forward_schottky_behind_5v_converges() {
        let circuit = junction_behind(CircuitElement::SchottkyDiode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
        });
        let result = crate::solve_circuit(&circuit).unwrap();
        let want = shockley_drop(5.0, 1_000.0, 1e-8, 1.0);
        assert!((result.node_voltages["out"] - want).abs() < 1e-5);
    }

    #[test]
    fn forward_photodiode_behind_5v_converges() {
        let circuit = junction_behind(CircuitElement::Photodiode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            irradiance: 0.0,
            temperature: 300.15,
        });
        let result = crate::solve_circuit(&circuit).unwrap();
        let want = shockley_drop(5.0, 1_000.0, 1e-12, 1.0);
        assert!((result.node_voltages["out"] - want).abs() < 1e-5);
    }

    #[test]
    fn silicon_diode_next_to_a_bjt_converges() {
        // The BJT sends the solver through the linear pre-solve, which used
        // to leave the diode with the full 5 V across it.
        let mut circuit = junction_behind(CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
        });
        circuit.components.extend([
            CircuitElement::Resistor {
                id: "RB".into(),
                nodes: ["in".into(), "b".into()],
                resistance: 470_000.0,
            },
            CircuitElement::Resistor {
                id: "RC".into(),
                nodes: ["in".into(), "c".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
            },
        ]);
        let result = crate::solve_circuit(&circuit).unwrap();
        let want = shockley_drop(5.0, 1_000.0, 1e-14, 1.0);
        assert!((result.node_voltages["out"] - want).abs() < 1e-5);
    }

    /// Current through `junctions` (IS, n) in series with `r` from `vs`,
    /// from `vs = I·r + Σ n·V_T·ln(I/IS + 1)` by bisection.
    fn series_current(vs: f64, r: f64, junctions: &[(f64, f64)]) -> f64 {
        let vt = sindr_devices::diode::V_T;
        let residual = |i: f64| {
            i * r
                + junctions
                    .iter()
                    .map(|&(is, n)| n * vt * (i / is + 1.0).ln())
                    .sum::<f64>()
                - vs
        };
        let (mut lo, mut hi) = (0.0, vs / r);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if residual(mid) > 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// `in` → R1 → chain of diodes to ground; node `out` after the resistor,
    /// then `m1`, `m2`, … between diodes.
    fn diode_chain(diodes: Vec<CircuitElement>) -> Circuit {
        let mut components = vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "out".into()],
                resistance: 1_000.0,
            },
        ];
        components.extend(diodes);
        Circuit {
            ground_node: "0".into(),
            components,
        }
    }

    fn silicon(id: &str, a: &str, k: &str) -> CircuitElement {
        CircuitElement::Diode {
            id: id.into(),
            nodes: [a.into(), k.into()],
            temperature: 300.15,
        }
    }

    #[test]
    fn two_silicon_diodes_in_series_converge() {
        let circuit = diode_chain(vec![silicon("D1", "out", "m1"), silicon("D2", "m1", "0")]);
        let result = crate::solve_circuit(&circuit).unwrap();
        let i = series_current(5.0, 1_000.0, &[(1e-14, 1.0), (1e-14, 1.0)]);
        assert!((result.node_voltages["out"] - (5.0 - 1_000.0 * i)).abs() < 1e-5);
        assert!((result.node_voltages["m1"] - (5.0 - 1_000.0 * i) / 2.0).abs() < 1e-5);
    }

    #[test]
    fn three_diodes_in_series_converge() {
        let circuit = diode_chain(vec![
            silicon("D1", "out", "m1"),
            silicon("D2", "m1", "m2"),
            silicon("D3", "m2", "0"),
        ]);
        let result = crate::solve_circuit(&circuit).unwrap();
        let i = series_current(5.0, 1_000.0, &[(1e-14, 1.0); 3]);
        assert!((result.node_voltages["out"] - (5.0 - 1_000.0 * i)).abs() < 1e-5);
    }

    #[test]
    fn schottky_over_silicon_in_series_converges() {
        let circuit = diode_chain(vec![
            CircuitElement::SchottkyDiode {
                id: "D1".into(),
                nodes: ["out".into(), "m1".into()],
                temperature: 300.15,
            },
            silicon("D2", "m1", "0"),
        ]);
        let result = crate::solve_circuit(&circuit).unwrap();
        let i = series_current(5.0, 1_000.0, &[(1e-8, 1.0), (1e-14, 1.0)]);
        let vt = sindr_devices::diode::V_T;
        assert!((result.node_voltages["m1"] - vt * (i / 1e-14 + 1.0).ln()).abs() < 1e-5);
        assert!((result.node_voltages["out"] - (5.0 - 1_000.0 * i)).abs() < 1e-5);
    }

    /// IGBT switched on into a resistive load sits in triode with a few mV
    /// across it. Its VCE solves (VCC − v)/R = K·(vov − v/2)·v, the smaller
    /// root of K/2·v² − (K·vov + 1/R)·v + VCC/R = 0.
    #[test]
    fn igbt_switch_matches_closed_form() {
        let params = sindr_devices::igbt::IgbtParams::default(); // vth=5, k=5
        let (vcc, r, vge) = (12.0, 100.0, 10.0);
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "Vg".into(),
                    nodes: ["g".into(), "0".into()],
                    voltage: vge,
                    waveform: None,
                },
                CircuitElement::VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: vcc,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R".into(),
                    nodes: ["vcc".into(), "c".into()],
                    resistance: r,
                },
                CircuitElement::Igbt {
                    id: "Q1".into(),
                    nodes: ["g".into(), "c".into(), "0".into()],
                    params: params.clone(),
                },
            ],
        };
        let vov = vge - params.vth;
        let b = params.k * vov + 1.0 / r;
        let expected = (b - (b * b - 2.0 * params.k * vcc / r).sqrt()) / params.k;

        let vce = crate::solve_circuit(&circuit).unwrap().node_voltages["c"];
        assert!(
            (vce - expected).abs() <= 1e-3 * expected,
            "VCE {vce} vs closed form {expected}"
        );
    }
}
