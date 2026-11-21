//! Convergence fallbacks for nonlinear DC: which one a circuit needs, and
//! that the answer it gives is a real operating point.

use sindr::{
    solve_circuit, BjtKind, Circuit, CircuitElement, ConvergenceAid, SimError, SimulationResult,
    Solver, SolverOptions,
};

fn vsource(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn npn(id: &str, b: &str, c: &str, e: &str) -> CircuitElement {
    CircuitElement::Bjt {
        id: id.into(),
        nodes: [b.into(), c.into(), e.into()],
        kind: BjtKind::Npn,
        bf: 100.0,
        temperature: 300.15,
        parasitic_caps: None,
        model: None,
    }
}

fn bjt<'a>(result: &'a SimulationResult, id: &str) -> &'a sindr::BjtResult {
    result.bjt_results.iter().find(|b| b.id == id).unwrap()
}

/// A current mirror from 30 V with a 1 MΩ load: plain Newton–Raphson and gmin
/// stepping both end in a limit cycle at the BJT step clamp. Ramping the
/// sources up from zero finds the operating point: Q1 diode-connected near
/// 0.8 V carrying ~29 mA, Q2 asking for the same and saturated, since 1 MΩ
/// can only supply 30 µA.
#[test]
fn current_mirror_needs_source_stepping() {
    let mirror = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("VCC", "vcc", "0", 30.0),
            resistor("RREF", "vcc", "m", 1e3),
            npn("Q1", "m", "m", "0"),
            npn("Q2", "m", "out", "0"),
            resistor("RL", "vcc", "out", 1e6),
        ],
        ..Default::default()
    };
    let result = solve_circuit(&mirror).unwrap();
    assert_eq!(result.convergence_aid, Some(ConvergenceAid::SourceStepping));

    let (vm, vout) = (result.node_voltages["m"], result.node_voltages["out"]);
    assert!(vm > 0.6 && vm < 0.9, "V(m) = {vm}");
    assert!(
        vout > 0.0 && vout < 0.2,
        "Q2 should saturate, V(out) = {vout}"
    );

    // KCL at m: RREF's current feeds Q1's collector and both bases.
    let (q1, q2) = (bjt(&result, "Q1"), bjt(&result, "Q2"));
    let i_ref = (30.0 - vm) / 1e3;
    let i_in = q1.ic + q1.ib + q2.ib;
    assert!(
        (i_ref - i_in).abs() < 1e-6 * i_ref,
        "RREF {i_ref} A vs {i_in} A"
    );
    // KCL at out: RL's current is Q2's collector current.
    let i_load = (30.0 - vout) / 1e6;
    assert!(
        (i_load - q2.ic).abs() < 1e-9,
        "RL {i_load} A vs Ic2 {} A",
        q2.ic
    );
}

/// A circuit plain Newton–Raphson handles reports no fallback.
#[test]
fn easy_circuit_reports_no_aid() {
    let divider = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "in", "0", 5.0),
            resistor("R1", "in", "a", 1e3),
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model: None,
            },
        ],
        ..Default::default()
    };
    assert_eq!(solve_circuit(&divider).unwrap().convergence_aid, None);
}

fn diode(id: &str, anode: &str, cathode: &str) -> CircuitElement {
    CircuitElement::Diode {
        id: id.into(),
        nodes: [anode.into(), cathode.into()],
        temperature: 300.15,
        model: None,
    }
}

fn current(result: &SimulationResult, id: &str) -> f64 {
    result
        .component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap()
        .current_through
}

/// Two diodes in series, both reverse-biased: the node between them is held
/// only by picoamp leakage. With 12 iterations plain Newton–Raphson can't
/// settle it; the gmin ladder can, and says so.
fn reverse_pair() -> Circuit {
    Circuit::new(
        "0",
        vec![
            vsource("V1", "in", "0", 10.0),
            diode("D1", "m", "in"),
            diode("D2", "0", "m"),
        ],
    )
}

fn budget(iterations: usize, gmin: bool, source_stepping: bool) -> Solver {
    let mut options = SolverOptions::default();
    options.max_iterations = iterations;
    options.source_stepping = source_stepping;
    if !gmin {
        options.gmin_steps.clear();
    }
    Solver::new(options)
}

#[test]
fn leakage_held_node_needs_gmin_stepping() {
    let result = budget(12, true, false).circuit(&reverse_pair()).unwrap();
    assert_eq!(result.convergence_aid, Some(ConvergenceAid::GminStepping));
    let vm = result.node_voltages["m"];
    assert!(vm > 0.0 && vm < 10.0, "V(m) = {vm}");
    // KCL at m, currents reported anode to cathode: D2 brings i2 in, D1
    // takes i1 out, and the 1e-12 S floor every node has takes GMIN·V(m).
    // The node is set by femtoamps, so 1 % of D1's current is the check.
    let (i1, i2) = (current(&result, "D1"), current(&result, "D2"));
    assert!(i1 < 0.0 && i2 < 0.0, "both reverse: {i1}, {i2}");
    let residual = i2 - i1 - sindr_devices::diode::GMIN * vm;
    assert!(
        residual.abs() <= 1e-2 * i1.abs(),
        "{i1} vs {i2}: {residual}"
    );

    // The default budget needs no aid, and agrees on the node to within
    // the convergence tolerance.
    let plain = solve_circuit(&reverse_pair()).unwrap();
    assert_eq!(plain.convergence_aid, None);
    assert!((plain.node_voltages["m"] - vm).abs() < 1e-3);
}

/// With every fallback switched off the first failure is the answer.
#[test]
fn without_fallbacks_the_plain_failure_is_returned() {
    let err = budget(12, false, false)
        .circuit(&reverse_pair())
        .unwrap_err();
    assert!(
        matches!(err, SimError::ConvergenceFailed { iterations: 12, .. }),
        "{err:?}"
    );
}

/// A mirror fed from current sources only: source stepping ramps current
/// sources too. Q1 takes IREF less both base currents; Q2 asks for the
/// same but its load can only take IL, so it saturates.
#[test]
fn source_stepping_ramps_current_sources() {
    let isrc = |id: &str, to: &str, amps: f64| CircuitElement::CurrentSource {
        id: id.into(),
        nodes: ["0".into(), to.into()],
        current: amps,
        waveform: None,
    };
    let mirror = Circuit::new(
        "0",
        vec![
            isrc("IREF", "m", 10e-3),
            npn("Q1", "m", "m", "0"),
            npn("Q2", "m", "out", "0"),
            isrc("IL", "out", 30e-6),
            resistor("RL", "out", "0", 1e6),
        ],
    );
    let result = solve_circuit(&mirror).unwrap();
    assert_eq!(result.convergence_aid, Some(ConvergenceAid::SourceStepping));
    let (q1, q2) = (bjt(&result, "Q1"), bjt(&result, "Q2"));
    let i_in = q1.ic + q1.ib + q2.ib;
    assert!((i_in - 10e-3).abs() < 1e-6 * 10e-3, "KCL at m: {i_in} A");
    let vout = result.node_voltages["out"];
    let i_out = q2.ic + vout / 1e6;
    assert!((i_out - 30e-6).abs() < 1e-9, "KCL at out: {i_out} A");
    assert!(vout > 0.0 && vout < 0.2, "Q2 saturated: V(out) = {vout}");
}

/// Too few iterations for any rung of source stepping either: the ramp
/// halves its increment down to `min_source_step` and gives up with the
/// convergence failure.
#[test]
fn source_stepping_gives_up_below_its_minimum_step() {
    let err = budget(3, true, true).circuit(&reverse_pair()).unwrap_err();
    assert!(matches!(err, SimError::ConvergenceFailed { .. }), "{err:?}");
}
