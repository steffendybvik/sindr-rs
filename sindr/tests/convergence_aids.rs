//! Convergence fallbacks for nonlinear DC: which one a circuit needs, and
//! that the answer it gives is a real operating point.

use sindr::{solve_circuit, BjtKind, Circuit, CircuitElement, ConvergenceAid, SimulationResult};

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
            },
        ],
    };
    assert_eq!(solve_circuit(&divider).unwrap().convergence_aid, None);
}
