//! Newton-Raphson nonlinear solver.
//!
//! Orchestrates iterative solving for circuits containing nonlinear
//! elements (diodes, BJTs, MOSFETs, JFETs, IGBTs, ...). Each iteration builds a fresh MNA system
//! with companion models evaluated at the current operating point,
//! solves it, applies voltage limiting, and checks convergence.

use crate::element::{
    Behavioural, Bjt, Comparator, CurrentSource, Diode, Igbt, Led, Mosfet, OpAmp, Photodiode,
    SchottkyDiode, SwitchingRegulator, VoltageRegulator, VoltageSource, ZenerDiode,
};
use nalgebra::DVector;

use sindr_devices::bjt::BjtKind;
use sindr_devices::diode::{self, GMIN};

use crate::circuit::{Circuit, CircuitElement};
use crate::device_params;
use crate::element::behavioural::behavioural_finite_at;
use crate::element::layout::Layout;
use crate::element::voltage_regulator::{limit_regulator_step, regulator_pieces_settled};
use crate::error::{SimError, SolvePoint};
use crate::mna::MnaSystem;
use crate::node_map::{NodeMap, Terminals};
use crate::results::{self, ConvergenceAid, SimulationResult};
use crate::solver::SolverOptions;
use crate::stamp;

/// Solve a circuit containing nonlinear elements via Newton-Raphson iteration.
///
/// # Algorithm
///
/// 1. Start from the linear part of the circuit solved with the nonlinear
///    devices left out, then move each BJT base to a typical Vbe and cap each
///    forward-biased diode junction at its critical voltage, as SPICE does.
/// 2. Each iteration: stamp all components (with companion models at v_prev),
///    add Gmin shunts, solve, apply voltage limiting, check convergence.
///    When limiting one junction pushes a neighbour sharing its node past
///    its own limit (diodes in series), take a damped step that keeps every
///    junction within its limit instead.
/// 3. Converged when every node voltage has settled within
///    `vntol + reltol·|V|` and every diode, LED and BJT junction voltage
///    within `vntol + reltol·n·V_T`; error after `max_iterations`.
///
/// When plain Newton–Raphson fails, retries with gmin stepping and then
/// source stepping, as `opts` allows; [`SimulationResult::convergence_aid`]
/// says which one succeeded. `opts.nodeset` seeds the initial guess, as
/// SPICE's `.NODESET`.
///
/// # Errors
///
/// [`SimError::ConvergenceFailed`] if plain Newton–Raphson, gmin stepping
/// and source stepping all fail, or [`SimError::NotFinite`] when a
/// behavioural source's expression overflowed on the way; plus any stamping
/// or solve error.
pub fn solve_nonlinear(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    opts: &SolverOptions,
) -> Result<SimulationResult, SimError> {
    let initial_voltages = (!opts.nodeset.is_empty()).then_some(&opts.nodeset);
    // First attempt: plain Newton–Raphson with the default GMIN floor.
    match nr_inner(
        circuit,
        node_map,
        num_nodes,
        num_vsources,
        None,
        initial_voltages,
        0.0,
        opts,
    ) {
        Ok((result, _)) => Ok(result),
        // SingularMatrix and InvalidSolution are topology / numerical errors
        // that no convergence aid can fix: propagate them at once.
        Err(e) if !e.is_retryable() => Err(e),
        Err(first) => {
            // An expression that overflowed on the way is the better report
            // when every aid fails: it names the source and what it read.
            let (mut not_finite, mut last) = (None, None);
            let mut record = |e: SimError| match e {
                SimError::NotFinite { .. } if not_finite.is_none() => not_finite = Some(e),
                e => last = Some(e),
            };
            record(first);
            // Fall back to gmin stepping: progressively softens the Jacobian
            // by adding extra conductance to ground on every node, then ramps
            // it down while warm-starting from the previous step's solution.
            if !opts.gmin_steps.is_empty() {
                match gmin_stepping(
                    circuit,
                    node_map,
                    num_nodes,
                    num_vsources,
                    initial_voltages,
                    opts,
                ) {
                    Ok(mut result) => {
                        result.convergence_aid = Some(ConvergenceAid::GminStepping);
                        return Ok(result);
                    }
                    Err(e) if !e.is_retryable() => return Err(e),
                    Err(e) => record(e),
                }
            }
            if opts.source_stepping {
                // Then source stepping: ramp every source up from zero.
                match source_stepping(circuit, node_map, num_nodes, num_vsources, opts) {
                    Ok(mut result) => {
                        result.convergence_aid = Some(ConvergenceAid::SourceStepping);
                        return Ok(result);
                    }
                    Err(e) if !e.is_retryable() => return Err(e),
                    Err(e) => record(e),
                }
            }
            Err(not_finite
                .or(last)
                .expect("the first attempt's error is always recorded"))
        }
    }
}

/// Source-stepping homotopy, the fallback after gmin stepping.
///
/// Solves the circuit with every independent source (voltage and current
/// sources, regulator outputs) scaled by a factor that ramps from a small
/// value up to 1, warm-starting each solve from the last. With the sources
/// near zero every junction is near zero bias, where Newton–Raphson always
/// converges; each later step starts close to its answer. A step that fails
/// is retried with half the increment, down to `opts.min_source_step`.
fn source_stepping(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    opts: &SolverOptions,
) -> Result<SimulationResult, SimError> {
    let mut scale: f64 = 0.0;
    let mut increment: f64 = 0.1;
    let mut warm: Option<DVector<f64>> = None;
    loop {
        let next = (scale + increment).min(1.0);
        let scaled = scale_sources(circuit, next);
        match nr_inner(
            &scaled,
            node_map,
            num_nodes,
            num_vsources,
            warm.as_ref(),
            None,
            0.0,
            opts,
        ) {
            Ok((result, v)) => {
                if next >= 1.0 {
                    return Ok(result);
                }
                scale = next;
                warm = Some(v);
                increment = (increment * 2.0).min(0.25);
            }
            Err(e) if e.is_retryable() => {
                increment /= 2.0;
                if increment < opts.min_source_step {
                    return Err(e);
                }
            }
            Err(e) => return Err(e),
        }
    }
}

/// `circuit` with every independent source scaled by `factor`.
fn scale_sources(circuit: &Circuit, factor: f64) -> Circuit {
    let mut scaled = circuit.clone();
    for component in &mut scaled.components {
        match component {
            CircuitElement::VoltageSource(VoltageSource { voltage, .. })
            | CircuitElement::VoltageRegulator(VoltageRegulator { voltage, .. })
            | CircuitElement::SwitchingRegulator(SwitchingRegulator { voltage, .. }) => {
                *voltage *= factor
            }
            CircuitElement::CurrentSource(CurrentSource { current, .. }) => *current *= factor,
            _ => {}
        }
    }
    scaled
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
    opts: &SolverOptions,
) -> Result<SimulationResult, SimError> {
    // `opts.gmin_steps`, then a last step at 0.0 (the default GMIN floor).
    let ladder = opts.gmin_steps.iter().copied().chain(std::iter::once(0.0));

    let mut warm_start: Option<DVector<f64>> = None;
    let mut last_err = SimError::ConvergenceFailed {
        iterations: 0,
        max_step_volts: f64::INFINITY,
        node: None,
        components: Vec::new(),
        at: SolvePoint::OperatingPoint,
    };

    for gmin_extra in ladder {
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
            opts,
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

/// Inner NR iteration loop. Returns simulation result and final voltage vector.
#[allow(clippy::too_many_arguments)]
fn nr_inner(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    v_init: Option<&DVector<f64>>,
    initial_voltages: Option<&std::collections::HashMap<String, f64>>,
    gmin_extra: f64,
    opts: &SolverOptions,
) -> Result<(SimulationResult, DVector<f64>), SimError> {
    // A high-gain op-amp or comparator clamped to its rails gives Newton no
    // slope once an iterate saturates, so a feedback loop can bounce rail to rail. Solve
    // with the clamps lifted first: that is the answer whenever the op-amps
    // end up in their linear range, and a close start when they don't.
    if v_init.is_none() {
        if let Some(open) = without_op_amp_rails(circuit) {
            if let Ok((_, v)) = nr_inner(
                &open,
                node_map,
                num_nodes,
                num_vsources,
                None,
                initial_voltages,
                gmin_extra,
                opts,
            ) {
                if let Ok(solved) = nr_inner(
                    circuit,
                    node_map,
                    num_nodes,
                    num_vsources,
                    Some(&v),
                    None,
                    gmin_extra,
                    opts,
                ) {
                    return Ok(solved);
                }
            }
        }
    }
    let layout = Layout::new(circuit);
    let terminals = Terminals::new(circuit, node_map);
    let size = num_nodes + num_vsources;
    let mut v_prev = match v_init {
        Some(v) => v.clone(),
        None => {
            let mut v = DVector::zeros(size);
            init_bjt_voltages(circuit, node_map, (&layout, &terminals), &mut v);
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
    let mut last_step = (f64::INFINITY, None);
    // The iterate `v_prev` stepped from, to shorten a step that lands where
    // a behavioural expression isn't finite.
    let mut v_base: Option<DVector<f64>> = None;
    // The overflow of the full step, reported if halving doesn't help.
    let mut overflow: Option<SimError> = None;
    let mut recent: Option<SimError> = None;
    let mut halvings = 0;
    // One system, cleared for each iteration: no matrix to allocate.
    let mut system = MnaSystem::new(num_nodes, num_vsources);
    for _iteration in 0..opts.max_iterations {
        system.clear();
        let stamped = if v_base.is_none() {
            stamp_first(
                circuit,
                &mut system,
                node_map,
                (&layout, &terminals),
                &v_prev,
            )
        } else {
            stamp::stamp_circuit_with(
                circuit,
                &mut system,
                node_map,
                &layout,
                &terminals,
                Some(&v_prev),
            )
        };
        let solved = stamped.and_then(|()| {
            // Add Gmin shunts (default GMIN floor + any homotopy extra).
            add_gmin_shunts(&mut system, num_nodes);
            if gmin_extra > 0.0 {
                for i in 0..num_nodes {
                    system.a[(i, i)] += gmin_extra;
                }
            }
            system.solve()
        });
        let v_new = match (solved, &v_base) {
            (Ok(v), _) => {
                // A step that only stamped once shortened: the overflow is
                // what to report if Newton never settles.
                recent = if halvings > 0 { overflow.take() } else { None };
                (overflow, halvings) = (None, 0);
                v
            }
            // The step landed where a behavioural expression isn't finite,
            // or where the conductances it gives swamp the rest of the
            // matrix: shorten it toward the iterate it left.
            (Err(e @ (SimError::NotFinite { .. } | SimError::SingularMatrix)), Some(base)) => {
                let e = overflow.take().unwrap_or(e);
                if halvings == MAX_STEP_HALVINGS {
                    return Err(match e {
                        SimError::SingularMatrix => convergence_failed(
                            circuit,
                            node_map,
                            last_step,
                            SolvePoint::OperatingPoint,
                            opts.max_iterations,
                        ),
                        e => e,
                    });
                }
                halvings += 1;
                overflow = Some(e);
                v_prev = base + (&v_prev - base) * 0.5;
                continue;
            }
            (Err(e), _) => return Err(e),
        };

        let mut v_limited = apply_voltage_limiting(&v_new, &v_prev, &diode_info);
        let igbt_limited = apply_igbt_limiting(&mut v_limited, &v_prev, &igbt_info);
        let regulator_limited =
            limit_regulator_step(circuit, &terminals, num_nodes, &v_prev, &mut v_limited);

        if debug_nr && _iteration < 30 {
            eprintln!(
                "NR iter {}: v_prev={:.4?} -> v_new={:.4?} -> v_limited={:.4?}",
                _iteration,
                v_prev.iter().take(num_nodes).collect::<Vec<_>>(),
                v_new.iter().take(num_nodes).collect::<Vec<_>>(),
                v_limited.iter().take(num_nodes).collect::<Vec<_>>()
            );
        }

        if !igbt_limited
            && !regulator_limited
            && converged(&v_prev, &v_limited, num_nodes, &diode_info, opts)
            && regulator_pieces_settled(circuit, &terminals, num_nodes, &v_prev, &v_limited)
        {
            behavioural_finite_at(circuit, &terminals, &v_limited, 0.0)?;
            let result = results::extract_results(circuit, node_map, &v_limited, num_nodes);
            return Ok((result, v_limited));
        }

        last_step = max_node_step(&v_prev, &v_limited, num_nodes);
        v_base = Some(std::mem::replace(&mut v_prev, v_limited));
    }

    if let Some(e) = recent {
        return Err(e);
    }
    Err(convergence_failed(
        circuit,
        node_map,
        last_step,
        SolvePoint::OperatingPoint,
        opts.max_iterations,
    ))
}

/// Stamps `circuit` at the starting guess. A behavioural source whose
/// expression isn't finite there (`ln` of a node that starts at 0 V) is
/// left out of this first iteration, so the rest of the circuit moves its
/// inputs to where it is. Switching one off changes only its expression,
/// so `circuit`'s layout and terminals hold for the copy too.
fn stamp_first(
    circuit: &Circuit,
    system: &mut MnaSystem,
    node_map: &NodeMap,
    (layout, terminals): (&Layout, &Terminals),
    v: &DVector<f64>,
) -> Result<(), SimError> {
    let mut off: Option<Circuit> = None;
    loop {
        let stamped = stamp::stamp_circuit_with(
            off.as_ref().unwrap_or(circuit),
            system,
            node_map,
            layout,
            terminals,
            Some(v),
        );
        match stamped {
            Err(SimError::NotFinite { component, .. })
                if switch_off(off.get_or_insert_with(|| circuit.clone()), &component) =>
            {
                system.clear();
            }
            other => return other,
        }
    }
}

/// Sets behavioural source `id`'s expression to 0; false if it already is.
fn switch_off(circuit: &mut Circuit, id: &str) -> bool {
    let zero = crate::expression::Expression::parse("0").expect("0 parses");
    circuit.components.iter_mut().any(|c| match c {
        CircuitElement::Behavioural(Behavioural { id: this, expr, .. })
            if this == id && *expr != zero =>
        {
            *expr = zero.clone();
            true
        }
        _ => false,
    })
}

/// How many times one Newton step is halved, toward the iterate it left,
/// when it lands where a behavioural expression isn't finite: 2⁻²⁰ of a
/// 1 kV step is about a millivolt.
const MAX_STEP_HALVINGS: usize = 20;

/// Rails far enough out that the op-amp clamp never engages.
const UNCLAMPED_RAIL: f64 = 1e12;

/// `circuit` with every modelled op-amp's and comparator's rails moved out
/// of reach, or `None` when it has nothing clamped.
fn without_op_amp_rails(circuit: &Circuit) -> Option<Circuit> {
    let clamped = |c: &CircuitElement| {
        matches!(
            c,
            CircuitElement::OpAmp(OpAmp { model: Some(_), v_pos, .. })
                | CircuitElement::Comparator(Comparator { v_pos, .. })
                if *v_pos < UNCLAMPED_RAIL
        )
    };
    if !circuit.components.iter().any(clamped) {
        return None;
    }
    let mut open = circuit.clone();
    for c in &mut open.components {
        if let CircuitElement::OpAmp(OpAmp {
            v_pos,
            v_neg,
            model: Some(_),
            ..
        })
        | CircuitElement::Comparator(Comparator { v_pos, v_neg, .. }) = c
        {
            *v_pos = UNCLAMPED_RAIL;
            *v_neg = -UNCLAMPED_RAIL;
        }
    }
    Some(open)
}

/// Information about a diode/junction needed for voltage limiting.
pub(crate) struct DiodeLimitInfo {
    anode_idx: Option<usize>,
    cathode_idx: Option<usize>,
    v_crit: f64,
    n: f64,
    /// Thermal voltage kT/q at the junction temperature (V).
    vt: f64,
    /// For BJT junctions: the non-base node index that should absorb
    /// voltage limiting corrections. When set, only this node is adjusted
    /// (the base node is left untouched to prevent B-E and B-C limiters
    /// from fighting each other on the shared base). None for regular diodes.
    correction_node: Option<CorrectionTarget>,
    /// Whether this is a BJT junction (enables bilateral step limiting).
    is_bjt: bool,
    /// Breakdown voltage for a Zener, which is also driven in reverse.
    zener_vz: Option<f64>,
    /// BV of a diode whose breakdown is exponential: its reverse steps are
    /// limited past breakdown as forward ones are, mirrored about `−BV`.
    breakdown: Option<f64>,
    /// Whether the anode and the cathode are held by voltage sources, as
    /// [`held_nodes`] finds them. A held terminal is treated like ground
    /// when a limited step is split between the two.
    held: [bool; 2],
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
            CircuitElement::Diode(Diode {
                model,
                nodes,
                temperature,
                ..
            }) => {
                let params = device_params::diode(model.as_ref(), *temperature);
                let bv = params.breakdown.map(|b| b.bv);
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(params.is, params.n, params.vt()),
                    n: params.n,
                    vt: params.vt(),
                    correction_node: None,
                    is_bjt: false,
                    zener_vz: bv,
                    breakdown: bv,
                    held: [false; 2],
                });
            }
            CircuitElement::Led(Led {
                nodes,
                color,
                temperature,
                ..
            }) => {
                let params = device_params::led(color, *temperature);
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(params.is, params.n, params.vt()),
                    n: params.n,
                    vt: params.vt(),
                    correction_node: None,
                    is_bjt: false,
                    zener_vz: None,
                    breakdown: None,
                    held: [false; 2],
                });
            }
            CircuitElement::ZenerDiode(ZenerDiode {
                nodes,
                vz,
                temperature,
                ..
            }) => {
                let params = device_params::zener_diode(*vz, *temperature);
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(params.is, params.n, params.vt()),
                    n: params.n,
                    vt: params.vt(),
                    correction_node: None,
                    is_bjt: false,
                    zener_vz: Some(*vz),
                    breakdown: Some(*vz),
                    held: [false; 2],
                });
            }
            CircuitElement::SchottkyDiode(SchottkyDiode {
                nodes, temperature, ..
            }) => {
                let params = device_params::schottky(*temperature);
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(params.is, params.n, params.vt()),
                    n: params.n,
                    vt: params.vt(),
                    correction_node: None,
                    is_bjt: false,
                    zener_vz: None,
                    breakdown: None,
                    held: [false; 2],
                });
            }
            CircuitElement::Photodiode(Photodiode {
                nodes, temperature, ..
            }) => {
                let photo_params = device_params::photodiode(*temperature);
                info.push(DiodeLimitInfo {
                    anode_idx: node_map.index(&nodes[0]),
                    cathode_idx: node_map.index(&nodes[1]),
                    v_crit: diode::critical_voltage(
                        photo_params.is,
                        photo_params.n,
                        photo_params.vt(),
                    ),
                    n: photo_params.n,
                    vt: photo_params.vt(),
                    correction_node: None,
                    is_bjt: false,
                    zener_vz: None,
                    breakdown: None,
                    held: [false; 2],
                });
            }
            CircuitElement::Bjt(Bjt {
                model,
                nodes,
                kind,
                bf,
                temperature,
                ..
            }) => {
                let params = device_params::bjt(*bf, model.as_ref(), *temperature);

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
                    v_crit: diode::critical_voltage(params.is, params.nf, params.vt()),
                    n: params.nf,
                    vt: params.vt(),
                    correction_node: None,
                    is_bjt: true,
                    zener_vz: None,
                    breakdown: None,
                    held: [false; 2],
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
                    v_crit: diode::critical_voltage(params.is, params.nr, params.vt()),
                    n: params.nr,
                    vt: params.vt(),
                    correction_node: Some(bc_correction),
                    is_bjt: true,
                    zener_vz: None,
                    breakdown: None,
                    held: [false; 2],
                });
            }
            _ => {}
        }
    }

    let held = held_nodes(circuit, node_map);
    let is_held = |idx: Option<usize>| idx.is_some_and(|i| held[i]);
    for junction in &mut info {
        junction.held = [is_held(junction.anode_idx), is_held(junction.cathode_idx)];
    }
    info
}

/// Which nodes a voltage source holds: one whose other terminal is ground
/// or another held node, as a supply rail is. Newton can't move such a
/// node (the next linear solve puts it back where the sources say), so
/// limiting must not move it either.
fn held_nodes(circuit: &Circuit, node_map: &NodeMap) -> Vec<bool> {
    let mut held = vec![false; node_map.num_nodes()];
    let sources: Vec<[Option<usize>; 2]> = circuit
        .components
        .iter()
        .filter_map(|c| match c {
            CircuitElement::VoltageSource(VoltageSource { nodes, .. }) => {
                Some([node_map.index(&nodes[0]), node_map.index(&nodes[1])])
            }
            _ => None,
        })
        .collect();
    // Ground is `None`; each pass holds the far end of a source with one
    // end held, so a stack of n sources takes n passes.
    let fixed = |held: &[bool], idx: Option<usize>| idx.is_none_or(|i| held[i]);
    loop {
        let mut changed = false;
        for &[a, b] in &sources {
            for (near, far) in [(a, b), (b, a)] {
                if let Some(f) = far {
                    if !held[f] && fixed(&held, near) {
                        held[f] = true;
                        changed = true;
                    }
                }
            }
        }
        if !changed {
            return held;
        }
    }
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

        let mut v_d_limited = diode::limit_voltage(v_d_new, v_d_old, info.v_crit, info.n, info.vt);

        // Past breakdown the reverse current is exponential too: limit the
        // step in −(v + BV) the way the forward one is, as SPICE does.
        if let Some(bv) = info.breakdown {
            if v_d_limited < (-bv + 10.0 * info.n * info.vt).min(0.0) {
                let past = diode::limit_voltage(
                    -(v_d_limited + bv),
                    -(v_d_old + bv),
                    info.v_crit,
                    info.n,
                    info.vt,
                );
                v_d_limited = -(past + bv);
            }
        }

        // For BJT junctions, also apply bilateral step limiting.
        // The standard SPICE limiter only clamps forward-bias overshoots.
        // In saturation transitions, a junction voltage can swing far negative
        // (e.g., B-E goes from 0.7V to -0.3V), turning off the junction and
        // destabilizing the solver. Clamp the step to prevent this. A
        // reverse-biased junction moving toward zero and staying reverse
        // biased has no exponential to overshoot: clamping it there walks a
        // collector that starts at its rail down 0.26 V an iteration.
        let toward_zero_in_reverse = v_d_old < v_d_limited && v_d_limited <= 0.0;
        if info.is_bjt && !toward_zero_in_reverse {
            let max_step = 10.0 * info.n * info.vt; // ~0.26V
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
                // if one is ground or held by a source, apply the full delta
                // to the other. Moving a held node would linearise the
                // junction at a voltage the next solve can't keep, and leave
                // the free node only half corrected.
                let anode = info.anode_idx.filter(|_| !info.held[0]);
                let cathode = info.cathode_idx.filter(|_| !info.held[1]);
                let share = if anode.is_some() && cathode.is_some() {
                    0.5
                } else {
                    1.0
                };
                if let Some(ai) = anode {
                    v_limited[ai] += delta * share;
                }
                if let Some(ci) = cathode {
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
    // Only a junction that moved forward can overshoot: for a long step back
    // from forward bias `limit_voltage` returns `v_crit`, below where the
    // junction already is.
    let overshoots = diode_info.iter().any(|info| {
        let (old, fin) = (junction(info, v_prev), junction(info, &v_limited));
        fin > old && fin > diode::limit_voltage(fin, old, info.v_crit, info.n, info.vt) + 1e-9
    });
    if overshoots {
        let mut alpha: f64 = 1.0;
        for info in diode_info {
            let (old, raw) = (junction(info, v_prev), junction(info, v_new));
            let allowed = diode::limit_voltage(raw, old, info.v_crit, info.n, info.vt);
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
            CircuitElement::Igbt(Igbt { nodes, .. }) => Some(IgbtLimitInfo {
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
///
/// When the true solution is reverse-blocking, Newton keeps proposing
/// `vce <= 0` and halving alone would only approach 0 V; once `vce_old` is
/// within [`IGBT_VCE_FLOOR`] of it the step lands on 0 V instead, from where
/// the next step is unlimited. Returns whether any step was limited: a
/// limited iterate is not a Newton solution, so it must not count as
/// converged (SPICE's `noncon`).
pub(crate) fn apply_igbt_limiting(
    v_limited: &mut DVector<f64>,
    v_prev: &DVector<f64>,
    igbt_info: &[IgbtLimitInfo],
) -> bool {
    let mut limited = false;
    for info in igbt_info {
        let vce = |v: &DVector<f64>| {
            info.collector_idx.map_or(0.0, |i| v[i]) - info.emitter_idx.map_or(0.0, |i| v[i])
        };
        let (vce_old, vce_new) = (vce(v_prev), vce(v_limited));
        if vce_old <= 0.0 || vce_new > 0.0 {
            continue;
        }
        let target = if vce_old > IGBT_VCE_FLOOR {
            vce_old / 2.0
        } else {
            0.0
        };
        let delta = target - vce_new;
        match (info.collector_idx, info.emitter_idx) {
            (Some(ci), _) => v_limited[ci] += delta,
            (None, Some(ei)) => v_limited[ei] -= delta,
            (None, None) => continue,
        }
        limited = true;
    }
    limited
}

/// Below this `vce` (V), [`apply_igbt_limiting`] lands a step into reverse
/// blocking on 0 V rather than halving it again.
const IGBT_VCE_FLOOR: f64 = 10e-3;

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
/// Two criteria, both required:
/// - SPICE-style node voltages: |v_new - v_prev| <= vntol + reltol * max(|v_new|, |v_prev|)
/// - Junction voltages: |Δv_j| <= vntol + reltol * n * V_T for every
///   diode/LED/BJT junction. A junction's current changes by about
///   Δv_j / (n·V_T) relative, so this bounds the device current error at
///   reltol, as SPICE's device current check does. The node criterion alone
///   lets a junction between nodes near a 5 V rail move by ~5 mV, a ~20%
///   current error.
pub(crate) fn converged(
    v_prev: &DVector<f64>,
    v_new: &DVector<f64>,
    num_nodes: usize,
    diode_info: &[DiodeLimitInfo],
    opts: &SolverOptions,
) -> bool {
    for i in 0..num_nodes {
        let diff = (v_new[i] - v_prev[i]).abs();
        let tol = opts.vntol + opts.reltol * v_new[i].abs().max(v_prev[i].abs());
        if diff > tol {
            return false;
        }
    }
    let node = |v: &DVector<f64>, idx: Option<usize>| idx.map_or(0.0, |i| v[i]);
    diode_info.iter().all(|info| {
        let vj_prev = node(v_prev, info.anode_idx) - node(v_prev, info.cathode_idx);
        let vj_new = node(v_new, info.anode_idx) - node(v_new, info.cathode_idx);
        (vj_new - vj_prev).abs() <= opts.vntol + opts.reltol * info.n * info.vt
    })
}

/// Largest per-node voltage step `max_i |v_new[i] − v_prev[i]|` (V), and
/// the node index that took it (`None` when there are no nodes).
pub(crate) fn max_node_step(
    v_prev: &DVector<f64>,
    v_new: &DVector<f64>,
    num_nodes: usize,
) -> (f64, Option<usize>) {
    let mut max = (0.0f64, None);
    for i in 0..num_nodes {
        let diff = (v_new[i] - v_prev[i]).abs();
        if max.1.is_none() || diff > max.0 {
            max = (diff, Some(i));
        }
    }
    max
}

/// [`SimError::ConvergenceFailed`] for a solve of `circuit` whose last
/// Newton step was `step`: names the node that took it and the components
/// on that node.
pub(crate) fn convergence_failed(
    circuit: &Circuit,
    node_map: &NodeMap,
    step: (f64, Option<usize>),
    at: SolvePoint,
    iterations: usize,
) -> SimError {
    let node = step
        .1
        .and_then(|i| node_map.node_name(i))
        .map(str::to_string);
    let components = match &node {
        Some(name) => circuit
            .components
            .iter()
            .filter(|c| c.nodes().iter().any(|n| n == name))
            .map(|c| c.id().to_string())
            .collect(),
        None => Vec::new(),
    };
    SimError::ConvergenceFailed {
        iterations,
        max_step_volts: step.0,
        node,
        components,
        at,
    }
}

/// Set initial voltage guesses for BJT circuits.
///
/// Strategy: First solve the linear-only subcircuit (treating BJTs as open circuits)
/// to get reasonable node voltages from voltage sources and resistor networks.
/// Then overlay BJT junction voltage guesses (Vbe ~ 0.65V).
///
/// This dramatically improves convergence for saturation and PNP circuits
/// by starting the NR iteration near the physical solution.
fn init_bjt_voltages(
    circuit: &Circuit,
    node_map: &NodeMap,
    (layout, terminals): (&Layout, &Terminals),
    v_prev: &mut DVector<f64>,
) {
    const VBE_GUESS: f64 = 0.65;

    let has_nonlinear = circuit.components.iter().any(|c| {
        matches!(
            c,
            CircuitElement::Bjt(Bjt { .. })
                | CircuitElement::Mosfet(Mosfet { .. })
                | CircuitElement::ZenerDiode(ZenerDiode { .. })
                | CircuitElement::SchottkyDiode(SchottkyDiode { .. })
                | CircuitElement::Photodiode(Photodiode { .. })
        )
    });
    if !has_nonlinear {
        return;
    }

    // Step 1: Solve the circuit with its nonlinear devices left out (no
    // `v_prev`) to get the supply rails and resistor divider biases. Every
    // linear element stamps, with the branch rows the solve itself uses, so
    // a VCVS, op-amp or regulator doesn't leave an empty row that makes
    // this solve singular.
    let num_nodes = node_map.num_nodes();
    let mut system = MnaSystem::new(num_nodes, v_prev.len() - num_nodes);
    let linear = stamp::stamp_circuit_with(circuit, &mut system, node_map, layout, terminals, None)
        .and_then(|()| {
            // Gmin shunts keep a node only nonlinear devices reach from floating.
            add_gmin_shunts(&mut system, num_nodes);
            system.solve()
        });
    // No guess (a behavioural source not finite at 0 V, say) leaves the
    // all-zero start.
    if let Ok(linear_solution) = linear {
        v_prev.copy_from(&linear_solution);
    }

    // Step 2: Adjust base voltages for BJT Vbe guess
    for component in &circuit.components {
        if let CircuitElement::Bjt(Bjt { nodes, kind, .. }) = component {
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
    use crate::element::Resistor;

    fn diode_resistor_circuit(vsource: f64) -> Circuit {
        Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: vsource,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["n2".into(), "0".into()],
                    temperature: 300.15,
                    model: None,
                }),
            ],
            ..Default::default()
        }
    }

    /// A divider-biased NPN common-emitter stage on a 12 V rail, its
    /// collector buffered by a VCVS into a load. The VCVS's branch row has
    /// nothing in it unless the starting guess stamps it.
    fn biased_npn_with_buffer() -> Circuit {
        let r = |id: &str, a: &str, b: &str, resistance: f64| {
            CircuitElement::Resistor(Resistor {
                id: id.into(),
                nodes: [a.into(), b.into()],
                resistance,
            })
        };
        Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: 12.0,
                    waveform: None,
                }),
                r("R1", "vcc", "b", 47e3),
                r("R2", "b", "0", 10e3),
                r("RC", "vcc", "c", 4.7e3),
                r("RE", "e", "0", 1e3),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "e".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: None,
                }),
                CircuitElement::Vcvs(crate::element::Vcvs {
                    id: "E1".into(),
                    nodes: ["out".into(), "0".into()],
                    control_nodes: ["c".into(), "0".into()],
                    gain: 1.0,
                }),
                r("RL", "out", "0", 10e3),
            ],
        )
    }

    /// The first guess solves every linear element, VCVS included, with
    /// the BJT left out: the rail at its source value, the buffer's output
    /// following the collector, and each branch current in its own row.
    #[test]
    fn starting_guess_solves_the_linear_elements() {
        let circuit = biased_npn_with_buffer();
        let node_map = NodeMap::from_circuit(&circuit);
        let layout = Layout::new(&circuit);
        let n = node_map.num_nodes();
        let mut v = DVector::zeros(n + circuit.count_voltage_sources());
        init_bjt_voltages(
            &circuit,
            &node_map,
            (&layout, &Terminals::new(&circuit, &node_map)),
            &mut v,
        );
        let at = |node: &str| v[node_map.index(node).unwrap()];
        assert!((at("vcc") - 12.0).abs() < 1e-12, "vcc {}", at("vcc"));
        // The BJT is open, so only gmin pulls the collector off the rail.
        assert!((at("c") - 12.0).abs() < 1e-6, "c {}", at("c"));
        assert_eq!(at("out"), at("c"));
        // The base is nudged to one Vbe over the emitter.
        assert!((at("b") - at("e") - 0.65).abs() < 1e-12);
        // V1 feeds the divider (12 V / 57 kΩ), E1 the load (12 V / 10 kΩ).
        let v1 = v[n + layout.branch(0)];
        let e1 = v[n + layout.branch(6)];
        assert!((v1 + 12.0 / 57e3).abs() < 1e-9, "i(V1) {v1}");
        assert!((e1 + 12.0 / 10e3).abs() < 1e-9, "i(E1) {e1}");
    }

    /// With the rail and buffer in its first guess, the stage settles in a
    /// handful of iterations, buffered or not; it took 21 and 31.
    #[test]
    fn biased_npn_settles_quickly_with_or_without_a_buffer() {
        let buffered = biased_npn_with_buffer();
        let mut plain = buffered.clone();
        plain.components.truncate(6);
        let opts = SolverOptions {
            max_iterations: 6,
            ..SolverOptions::default()
        };
        for circuit in [&buffered, &plain] {
            let node_map = NodeMap::from_circuit(circuit);
            let n = node_map.num_nodes();
            let (result, _) = nr_inner(
                circuit,
                &node_map,
                n,
                circuit.count_voltage_sources(),
                None,
                None,
                0.0,
                &opts,
            )
            .expect("converges in 6 iterations");
            let reference = crate::solve_circuit(circuit).unwrap();
            for (node, v) in &reference.node_voltages {
                assert!((result.node_voltages[node] - v).abs() < 1e-6, "{node}");
            }
        }
    }

    /// A reverse-biased junction moving toward zero has no exponential to
    /// overshoot, so the BJT step clamp lets it go; one moving further into
    /// reverse is still clamped (`backward_bjt_step_keeps_its_step_clamp`).
    #[test]
    fn reverse_bjt_junction_moves_freely_toward_zero() {
        let n = 1.0;
        let info = [DiodeLimitInfo {
            anode_idx: Some(0),
            cathode_idx: Some(1),
            v_crit: diode::critical_voltage(1e-14, n, diode::V_T),
            n,
            vt: diode::V_T,
            correction_node: Some(CorrectionTarget::Cathode),
            is_bjt: true,
            zener_vz: None,
            breakdown: None,
            held: [false; 2],
        }];
        // Base-collector from −11.35 V to −3.8 V: unclamped.
        let v_prev = DVector::from_vec(vec![0.65, 12.0]);
        let v_new = DVector::from_vec(vec![2.0, 5.8]);
        assert_eq!(apply_voltage_limiting(&v_new, &v_prev, &info), v_new);
        // Into forward bias it is still clamped to 10·n·V_T.
        let v_new = DVector::from_vec(vec![2.0, 0.5]);
        let v = apply_voltage_limiting(&v_new, &v_prev, &info);
        assert!((v[0] - v[1] - (0.65 - 12.0 + 10.0 * n * diode::V_T)).abs() < 1e-12);
    }

    #[test]
    fn max_node_step_picks_largest_diff() {
        let prev = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0]);
        let new = DVector::from_vec(vec![1.05, 1.9, 3.5, 4.0]);
        // Diffs: 0.05, 0.1, 0.5, 0.0  → max = 0.5 over the first 4 nodes.
        let (r, at) = max_node_step(&prev, &new, 4);
        assert!((r - 0.5).abs() < 1e-12, "got {r}");
        assert_eq!(at, Some(2));
    }

    #[test]
    fn convergence_failed_names_the_node_and_its_components() {
        let diode = |id: &str| {
            CircuitElement::Diode(Diode {
                id: id.into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model: None,
            })
        };
        let circuit = Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["in".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["in".into(), "a".into()],
                    resistance: 1e3,
                }),
                diode("D1"),
                diode("D2"),
            ],
        );
        let node_map = NodeMap::from_circuit(&circuit);
        let a = node_map.index("a").unwrap();
        let err = convergence_failed(
            &circuit,
            &node_map,
            (0.25, Some(a)),
            SolvePoint::Transient { time: 2e-3 },
            100,
        );
        match &err {
            SimError::ConvergenceFailed {
                node,
                components,
                max_step_volts,
                at,
                ..
            } => {
                assert_eq!(node.as_deref(), Some("a"));
                assert_eq!(components, &["R1", "D1", "D2"]);
                assert_eq!(*max_step_volts, 0.25);
                assert_eq!(at, &SolvePoint::Transient { time: 2e-3 });
            }
            other => panic!("{other:?}"),
        }
        let msg = err.to_string();
        assert!(msg.contains("node `a` (on R1, D1, D2)"), "{msg}");

        // No iteration ran: no node to name.
        let none = convergence_failed(
            &circuit,
            &node_map,
            (f64::INFINITY, None),
            SolvePoint::OperatingPoint,
            100,
        );
        assert!(
            matches!(none, SimError::ConvergenceFailed { node: None, ref components, .. } if components.is_empty())
        );
    }

    #[test]
    fn max_node_step_only_considers_node_range() {
        // num_nodes = 2 means branch unknowns at index 2,3 are ignored.
        let prev = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0]);
        let new = DVector::from_vec(vec![1.01, 2.01, 99.0, 99.0]);
        let (r, at) = max_node_step(&prev, &new, 2);
        assert!((r - 0.01).abs() < 1e-12, "got {r}");
        assert!(matches!(at, Some(0 | 1)), "got {at:?}");
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

        let result = gmin_stepping(
            &circuit,
            &node_map,
            num_nodes,
            num_vsources,
            None,
            &SolverOptions::default(),
        )
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
            &SolverOptions::default(),
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

        let result = gmin_stepping(
            &circuit,
            &node_map,
            num_nodes,
            num_vsources,
            None,
            &SolverOptions::default(),
        )
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["in".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["in".into(), "out".into()],
                    resistance: 1_000.0,
                }),
                element,
            ],
            ..Default::default()
        }
    }

    /// An LED hanging from a 9 V rail: the limited step goes all to its
    /// cathode. Split between the two, half of it would land on the rail,
    /// which the next solve puts straight back at 9 V.
    #[test]
    fn limiting_leaves_a_rail_held_by_a_source_alone() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                vsource("V1", "vcc", 9.0),
                CircuitElement::Led(Led {
                    id: "D1".into(),
                    nodes: ["vcc".into(), "k".into()],
                    color: "red".into(),
                    temperature: 300.15,
                }),
                resistor("R1", "k", "0", 470.0),
            ],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let info = collect_diode_info(&circuit, &node_map);
        let (vcc, k) = (node_map.index("vcc").unwrap(), node_map.index("k").unwrap());
        let n = node_map.num_nodes();
        let mut v_prev = DVector::zeros(n + 1);
        v_prev[vcc] = 9.0;
        v_prev[k] = 8.0;
        let mut v_new = v_prev.clone();
        v_new[k] = 1.0;
        let v = apply_voltage_limiting(&v_new, &v_prev, &info);
        assert_eq!(v[vcc], 9.0);
        assert!(v[k] > 1.0 && v[k] < 8.0, "cathode {}", v[k]);
    }

    #[test]
    fn forward_schottky_behind_5v_converges() {
        let circuit = junction_behind(CircuitElement::SchottkyDiode(SchottkyDiode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
        }));
        let result = crate::solve_circuit(&circuit).unwrap();
        let want = shockley_drop(5.0, 1_000.0, 1e-8, 1.0);
        assert!((result.node_voltages["out"] - want).abs() < 1e-5);
    }

    #[test]
    fn forward_photodiode_behind_5v_converges() {
        let circuit = junction_behind(CircuitElement::Photodiode(Photodiode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            irradiance: 0.0,
            temperature: 300.15,
        }));
        let result = crate::solve_circuit(&circuit).unwrap();
        let want = shockley_drop(5.0, 1_000.0, 1e-12, 1.0);
        assert!((result.node_voltages["out"] - want).abs() < 1e-5);
    }

    #[test]
    fn silicon_diode_next_to_a_bjt_converges() {
        // The BJT sends the solver through the linear pre-solve, which used
        // to leave the diode with the full 5 V across it.
        let mut circuit = junction_behind(CircuitElement::Diode(Diode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
            model: None,
        }));
        circuit.components.extend([
            CircuitElement::Resistor(Resistor {
                id: "RB".into(),
                nodes: ["in".into(), "b".into()],
                resistance: 470_000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "RC".into(),
                nodes: ["in".into(), "c".into()],
                resistance: 1_000.0,
            }),
            CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }),
        ]);
        let result = crate::solve_circuit(&circuit).unwrap();
        let want = shockley_drop(5.0, 1_000.0, 1e-14, 1.0);
        assert!((result.node_voltages["out"] - want).abs() < 1e-5);
    }

    fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
        CircuitElement::Resistor(Resistor {
            id: id.into(),
            nodes: [a.into(), b.into()],
            resistance,
        })
    }

    fn vsource(id: &str, node: &str, voltage: f64) -> CircuitElement {
        CircuitElement::VoltageSource(VoltageSource {
            id: id.into(),
            nodes: [node.into(), "0".into()],
            voltage,
            waveform: None,
        })
    }

    fn assert_rel(actual: f64, expected: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= 1e-3 * expected.abs(),
            "{what}: {actual:e} vs {expected:e}"
        );
    }

    /// A diode whose terminals both sit near a 10 V rail: the node-voltage
    /// criterion alone allows ~10 mV of slack there, a large current error.
    /// The converged point must satisfy KCL against the diode equation.
    #[test]
    fn diode_far_from_ground_satisfies_kcl() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                vsource("V1", "vcc", 10.0),
                resistor("R1", "vcc", "a", 1_000.0),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["a".into(), "k".into()],
                    temperature: 300.15,
                    model: None,
                }),
                resistor("R2", "k", "0", 1_000.0),
            ],
            ..Default::default()
        };
        let r = crate::solve_circuit(&circuit).unwrap();
        let (va, vk) = (r.node_voltages["a"], r.node_voltages["k"]);
        let p = sindr_devices::diode::DiodeParams::silicon();
        let i_diode = p.is * (((va - vk) / (p.n * diode::V_T)).exp() - 1.0);
        assert_rel(i_diode, (10.0 - va) / 1_000.0, "diode vs R1 current");
        assert_rel(i_diode, vk / 1_000.0, "diode vs R2 current");
    }

    /// Saturated BJT stage. NPN: emitter side at ground. PNP: its mirror,
    /// emitter side at VCC, with every node near the rail.
    fn saturated_stage(kind: BjtKind) -> Circuit {
        let (vin, rc_to, re_to) = match kind {
            BjtKind::Npn => (5.0, "vcc", "0"),
            BjtKind::Pnp => (0.0, "0", "vcc"),
        };
        Circuit {
            ground_node: "0".into(),
            components: vec![
                vsource("VIN", "in", vin),
                vsource("VCC", "vcc", 5.0),
                resistor("RB", "in", "b", 10_000.0),
                resistor("RC", "c", rc_to, 1_000.0),
                resistor("RE", "e", re_to, 100.0),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "e".into()],
                    kind,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: None,
                }),
            ],
            ..Default::default()
        }
    }

    #[test]
    fn mirrored_bjt_stages_satisfy_kcl_and_agree() {
        let npn = crate::solve_circuit(&saturated_stage(BjtKind::Npn)).unwrap();
        let pnp = crate::solve_circuit(&saturated_stage(BjtKind::Pnp)).unwrap();

        for (r, rail_in, rail_c) in [(&npn, 5.0, 5.0), (&pnp, 0.0, 0.0)] {
            let v = &r.node_voltages;
            let q = &r.bjt_results[0];
            // Current the resistors feed into each terminal (negative for PNP).
            assert_rel((rail_in - v["b"]) / 10_000.0, q.ib, "base KCL");
            assert_rel((rail_c - v["c"]) / 1_000.0, q.ic, "collector KCL");
        }

        // The PNP circuit is the NPN one mirrored about 2.5 V.
        for node in ["b", "c", "e"] {
            let (vn, vp) = (npn.node_voltages[node], pnp.node_voltages[node]);
            assert!(
                (vn - (5.0 - vp)).abs() < 1e-4,
                "{node}: NPN {vn} vs mirrored PNP {}",
                5.0 - vp
            );
        }
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
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "out".into()],
                resistance: 1_000.0,
            }),
        ];
        components.extend(diodes);
        Circuit {
            ground_node: "0".into(),
            components,
            ..Default::default()
        }
    }

    fn silicon(id: &str, a: &str, k: &str) -> CircuitElement {
        CircuitElement::Diode(Diode {
            id: id.into(),
            nodes: [a.into(), k.into()],
            temperature: 300.15,
            model: None,
        })
    }

    /// A BJT junction stepping back from forward bias is not an overshoot:
    /// the series-junction fallback must not run and discard the BJT step
    /// clamp. Base on node 0 (emitter grounded), collector on node 1; the raw
    /// step pulls Vbe from 0.65 V to 0.17 V and throws the collector to 2 GV.
    #[test]
    fn backward_bjt_step_keeps_its_step_clamp() {
        let n = 1.0;
        let v_crit = diode::critical_voltage(1.5e-6, n, diode::V_T);
        let info = [
            DiodeLimitInfo {
                anode_idx: Some(0),
                cathode_idx: None,
                v_crit,
                n,
                vt: diode::V_T,
                correction_node: None,
                is_bjt: true,
                zener_vz: None,
                breakdown: None,
                held: [false; 2],
            },
            DiodeLimitInfo {
                anode_idx: Some(0),
                cathode_idx: Some(1),
                v_crit,
                n,
                vt: diode::V_T,
                correction_node: Some(CorrectionTarget::Cathode),
                is_bjt: true,
                zener_vz: None,
                breakdown: None,
                held: [false; 2],
            },
        ];
        let v_prev = DVector::from_vec(vec![0.65, 5.0]);
        let v_new = DVector::from_vec(vec![0.1688, 2.1e9]);
        let v = apply_voltage_limiting(&v_new, &v_prev, &info);

        let max_step = 10.0 * n * diode::V_T;
        let (vbe, vbc) = (v[0], v[0] - v[1]);
        assert!((vbe - (0.65 - max_step)).abs() < 1e-12, "Vbe {vbe}");
        assert!((vbc - (0.65 - 5.0 - max_step)).abs() < 1e-9, "Vbc {vbc}");
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
            CircuitElement::SchottkyDiode(SchottkyDiode {
                id: "D1".into(),
                nodes: ["out".into(), "m1".into()],
                temperature: 300.15,
            }),
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vg".into(),
                    nodes: ["g".into(), "0".into()],
                    voltage: vge,
                    waveform: None,
                }),
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: vcc,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R".into(),
                    nodes: ["vcc".into(), "c".into()],
                    resistance: r,
                }),
                CircuitElement::Igbt(Igbt {
                    id: "Q1".into(),
                    nodes: ["g".into(), "c".into(), "0".into()],
                    params: params.clone(),
                }),
            ],
            ..Default::default()
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

    /// A raw `ZenerDiode` reaching the solver (the analyses lower it to a
    /// `Diode` with `bv` first) is stamped, limited and reported as that
    /// diode: both solve to the same clamp.
    #[test]
    fn raw_zener_solves_like_its_lowered_diode() {
        let circuit = Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["in".into(), "0".into()],
                    voltage: 12.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["in".into(), "k".into()],
                    resistance: 1e3,
                }),
                CircuitElement::ZenerDiode(ZenerDiode {
                    id: "DZ1".into(),
                    nodes: ["0".into(), "k".into()],
                    vz: 5.1,
                    temperature: 300.15,
                }),
            ],
        );
        let node_map = NodeMap::from_circuit(&circuit);
        let raw = solve_nonlinear(
            &circuit,
            &node_map,
            node_map.num_nodes(),
            circuit.count_voltage_sources(),
            &SolverOptions::default(),
        )
        .unwrap();
        let lowered = crate::solve_circuit(&circuit).unwrap();
        let (v_raw, v_lowered) = (raw.node_voltages["k"], lowered.node_voltages["k"]);
        // Both converge to within vntol + reltol·|V| of the same root.
        assert!((v_raw - v_lowered).abs() < 1e-5, "{v_raw} vs {v_lowered}");
        assert!(v_raw > 5.0 && v_raw < 5.5, "clamped near vz: {v_raw}");
        // KCL at k: R1's current is the Zener's reverse current.
        let i_r = (12.0 - v_raw) / 1e3;
        let i_z = raw
            .component_results
            .iter()
            .find(|r| r.id == "DZ1")
            .unwrap()
            .current_through;
        assert!((i_r + i_z).abs() < 1e-6 * i_r, "{i_r} vs {i_z}");
    }
}
