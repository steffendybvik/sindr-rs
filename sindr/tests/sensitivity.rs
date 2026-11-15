//! `.sens` against closed-form answers, and against finite differences of
//! full DC solves, which use only the DC model.

use approx::assert_relative_eq;
use sindr::{
    solve_circuit, solve_sensitivity, BjtKind, Circuit, CircuitElement, SensConfig, SensParameter,
    SensResult, SimError, TfOutput,
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

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
    }
}

fn v_out(node: &str) -> TfOutput {
    TfOutput::Voltage {
        node: node.into(),
        reference: None,
    }
}

fn sens(circuit: &Circuit, output: TfOutput) -> SensResult {
    solve_sensitivity(circuit, &SensConfig { output }).unwrap()
}

fn of(result: &SensResult, id: &str) -> f64 {
    result.get(id).unwrap().sensitivity
}

/// V volts over R1 and R2, out at their junction.
const V: f64 = 8.0;
const R1: f64 = 1e3;
const R2: f64 = 3e3;

fn divider() -> Circuit {
    circuit(vec![
        vsource("V1", "in", "0", V),
        resistor("R1", "in", "out", R1),
        resistor("R2", "out", "0", R2),
    ])
}

#[test]
fn divider_voltage_output() {
    let r = sens(&divider(), v_out("out"));
    let total = R1 + R2;
    assert_relative_eq!(of(&r, "V1"), R2 / total, max_relative = 1e-12);
    assert_relative_eq!(
        of(&r, "R1"),
        -V * R2 / (total * total),
        max_relative = 1e-12
    );
    assert_relative_eq!(of(&r, "R2"), V * R1 / (total * total), max_relative = 1e-12);
}

#[test]
fn entry_carries_value_and_per_percent() {
    let r = sens(&divider(), v_out("out"));
    let e = r.get("R2").unwrap();
    assert_eq!(e.parameter, SensParameter::Resistance);
    assert_eq!(e.value, R2);
    assert_relative_eq!(e.per_percent, e.sensitivity * R2 / 100.0);
    assert_eq!(r.entries.len(), 3);
}

#[test]
fn output_against_a_reference_node() {
    // V(in, out) = V·R1/(R1 + R2).
    let output = TfOutput::Voltage {
        node: "in".into(),
        reference: Some("out".into()),
    };
    let r = sens(&divider(), output);
    let total = R1 + R2;
    assert_relative_eq!(of(&r, "V1"), R1 / total, max_relative = 1e-12);
    assert_relative_eq!(of(&r, "R1"), V * R2 / (total * total), max_relative = 1e-12);
}

#[test]
fn current_output_through_a_voltage_source() {
    // I(V1), from + through the source, is −V/(R1 + R2).
    let output = TfOutput::Current {
        source_id: "V1".into(),
    };
    let r = sens(&divider(), output);
    let total = R1 + R2;
    assert_relative_eq!(of(&r, "V1"), -1.0 / total, max_relative = 1e-12);
    assert_relative_eq!(of(&r, "R1"), V / (total * total), max_relative = 1e-12);
}

#[test]
fn current_source_value() {
    // I1 pushes I into `a`, over R: V(a) = I·R.
    let c = circuit(vec![
        CircuitElement::CurrentSource {
            id: "I1".into(),
            nodes: ["0".into(), "a".into()],
            current: 2e-3,
            waveform: None,
        },
        resistor("R1", "a", "0", 5e3),
    ]);
    let r = sens(&c, v_out("a"));
    assert_relative_eq!(of(&r, "I1"), 5e3, max_relative = 1e-12);
    assert_relative_eq!(of(&r, "R1"), 2e-3, max_relative = 1e-12);
}

/// Central difference of `output` for a relative nudge of one parameter,
/// through full DC solves.
fn finite_difference(
    circuit: &Circuit,
    id: &str,
    value: f64,
    output: impl Fn(&sindr::SimulationResult) -> f64,
) -> f64 {
    let h = (value.abs() * 1e-5).max(1e-6);
    let with = |x: f64| {
        let mut c = circuit.clone();
        for e in &mut c.components {
            match e {
                CircuitElement::Resistor {
                    id: i, resistance, ..
                } if i == id => *resistance = x,
                CircuitElement::VoltageSource { id: i, voltage, .. } if i == id => *voltage = x,
                CircuitElement::CurrentSource { id: i, current, .. } if i == id => *current = x,
                CircuitElement::Vcvs { id: i, gain, .. } if i == id => *gain = x,
                CircuitElement::Vccs { id: i, gm, .. } if i == id => *gm = x,
                _ => {}
            }
        }
        output(&solve_circuit(&c).unwrap())
    };
    (with(value + h) - with(value - h)) / (2.0 * h)
}

/// Every entry of `.sens V(out)` against its finite difference.
fn assert_matches_finite_differences(name: &str, c: &Circuit, out: &str, tolerance: f64) {
    let r = sens(c, v_out(out));
    assert!(!r.entries.is_empty(), "{name}");
    for e in &r.entries {
        let fd = finite_difference(c, &e.component_id, e.value, |s| s.node_voltages[out]);
        assert_relative_eq!(e.sensitivity, fd, max_relative = tolerance, epsilon = 1e-9);
    }
}

#[test]
fn controlled_sources_match_finite_differences() {
    let c = circuit(vec![
        vsource("V1", "in", "0", 2.0),
        resistor("R1", "in", "x", 1e3),
        resistor("R2", "x", "0", 2e3),
        CircuitElement::Vcvs {
            id: "E1".into(),
            nodes: ["y".into(), "0".into()],
            control_nodes: ["x".into(), "0".into()],
            gain: 3.0,
        },
        resistor("R3", "y", "z", 500.0),
        resistor("R4", "z", "0", 1.5e3),
        CircuitElement::Vccs {
            id: "G1".into(),
            nodes: ["0".into(), "out".into()],
            control_nodes: ["z".into(), "0".into()],
            gm: 2e-3,
        },
        resistor("R5", "out", "0", 4e3),
    ]);
    assert_matches_finite_differences("controlled sources", &c, "out", 1e-7);
}

#[test]
fn diode_matches_finite_differences() {
    let c = circuit(vec![
        vsource("V1", "in", "0", 5.0),
        resistor("R1", "in", "out", 1e3),
        CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
        },
    ]);
    assert_matches_finite_differences("diode", &c, "out", 1e-4);
}

#[test]
fn common_emitter_stage_matches_finite_differences() {
    let c = circuit(vec![
        vsource("VCC", "vcc", "0", 12.0),
        vsource("V1", "in", "0", 2.0),
        resistor("RS", "in", "b", 1e3),
        resistor("RC", "vcc", "c", 2.2e3),
        resistor("RE", "e", "0", 470.0),
        CircuitElement::Bjt {
            id: "Q1".into(),
            nodes: ["b".into(), "c".into(), "e".into()],
            kind: BjtKind::Npn,
            bf: 100.0,
            temperature: 300.15,
            parasitic_caps: None,
        },
    ]);
    assert_matches_finite_differences("CE stage", &c, "c", 1e-4);
}

#[test]
fn sparse_system_uses_the_transpose_too() {
    // A ladder above the dense limit of 100 unknowns: N equal resistors from
    // V down to ground, out k resistors from the bottom. dV(out)/dR_j is
    // V·k/N²·(1/R) with a minus for a resistor above `out`, and V·(N−k)/N²/R
    // for one below it.
    const N: usize = 150;
    const K: usize = 40;
    let (r, v) = (1e3, 6.0);
    let name = |i: usize| {
        if i == 0 {
            "0".to_string()
        } else {
            format!("n{i}")
        }
    };
    let mut components = vec![vsource("V1", &name(N), "0", v)];
    for i in 0..N {
        components.push(resistor(&format!("R{i}"), &name(i + 1), &name(i), r));
    }
    let n = N as f64;
    let k = K as f64;
    let result = sens(&circuit(components), v_out(&name(K)));
    assert_relative_eq!(of(&result, "V1"), k / n, max_relative = 1e-9);
    let below = v * (n - k) / (n * n * r);
    let above = -v * k / (n * n * r);
    assert_relative_eq!(of(&result, "R0"), below, max_relative = 1e-9);
    assert_relative_eq!(of(&result, "R100"), above, max_relative = 1e-9);
}

#[test]
fn sparse_asymmetric_system_matches_finite_differences() {
    // Resistors and independent sources alone give a symmetric MNA matrix,
    // where the transpose changes nothing. A VCVS and a VCCS make it
    // asymmetric, and a ladder above 100 unknowns puts it in sparse storage.
    const N: usize = 150;
    let name = |i: usize| {
        if i == 0 {
            "0".to_string()
        } else {
            format!("n{i}")
        }
    };
    let mut components = vec![vsource("V1", &name(N), "0", 6.0)];
    for i in 0..N {
        components.push(resistor(&format!("R{i}"), &name(i + 1), &name(i), 1e3));
    }
    components.extend([
        CircuitElement::Vcvs {
            id: "E1".into(),
            nodes: ["e".into(), "0".into()],
            control_nodes: [name(90), name(30)],
            gain: 2.5,
        },
        resistor("RE", "e", "out", 2e3),
        CircuitElement::Vccs {
            id: "G1".into(),
            nodes: ["out".into(), "0".into()],
            control_nodes: [name(120), "0".into()],
            gm: 1e-4,
        },
        resistor("RL", "out", "0", 3e3),
    ]);
    let c = circuit(components);
    assert_matches_finite_differences("sparse ladder", &c, "out", 1e-6);
    assert_matches_finite_differences("sparse ladder", &c, &name(60), 1e-6);
}

#[test]
fn closed_relay_contact_matches_finite_differences() {
    // The coil sits at 12 V against a 5 V pickup, so DC closes the contact
    // and the path V2 → R2 → contact → R1 carries the signal.
    let c = circuit(vec![
        vsource("V1", "coil", "0", 12.0),
        vsource("V2", "src", "0", 10.0),
        resistor("R2", "src", "a", 1e3),
        CircuitElement::Relay {
            id: "K1".into(),
            nodes: ["coil".into(), "0".into(), "a".into(), "b".into()],
            coil_resistance: 500.0,
            pickup_voltage: 5.0,
            inductance: 0.0,
        },
        resistor("R1", "b", "0", 1e3),
    ]);
    let r = sens(&c, v_out("b"));
    assert_relative_eq!(of(&r, "V2"), 0.5, max_relative = 1e-4);
    assert_matches_finite_differences("closed relay", &c, "b", 1e-4);
}

#[test]
fn transformer_matches_closed_form() {
    // At DC each winding is a short, so V(m) = V1·R3/(R1 + R3) and the
    // secondary's loop carries no current. Closed form rather than finite
    // differences: `solve_circuit` runs a circuit with a transformer as a
    // transient, whose last step is only close to the DC point.
    let c = circuit(vec![
        vsource("V1", "in", "0", 4.0),
        resistor("R1", "in", "m", 1e3),
        resistor("R3", "m", "a", 3e3),
        CircuitElement::Transformer {
            id: "T1".into(),
            nodes: ["a".into(), "0".into(), "s".into(), "0".into()],
            l1: 1e-3,
            l2: 4e-3,
            k: 0.99,
        },
        resistor("RL", "s", "0", 100.0),
    ]);
    let r = sens(&c, v_out("m"));
    assert!(r.get("T1").is_none());
    let (v, r1, r3) = (4.0, 1e3, 3e3);
    let total = r1 + r3;
    assert_relative_eq!(of(&r, "V1"), r3 / total, max_relative = 1e-9);
    assert_relative_eq!(of(&r, "R1"), -v * r3 / (total * total), max_relative = 1e-9);
    assert_relative_eq!(of(&r, "R3"), v * r1 / (total * total), max_relative = 1e-9);
    assert_relative_eq!(of(&r, "RL"), 0.0, epsilon = 1e-12);
}

#[test]
fn reactive_parts_get_no_entry_and_leave_the_rest_right() {
    let mut c = divider();
    c.components[1] = resistor("R1", "in", "mid", R1);
    c.components.push(CircuitElement::Inductor {
        id: "L1".into(),
        nodes: ["mid".into(), "out".into()],
        inductance: 1e-3,
    });
    c.components.push(CircuitElement::Capacitor {
        id: "C1".into(),
        nodes: ["out".into(), "0".into()],
        capacitance: 1e-6,
    });
    let r = sens(&c, v_out("out"));
    assert!(r.get("L1").is_none());
    assert!(r.get("C1").is_none());
    assert_relative_eq!(of(&r, "V1"), R2 / (R1 + R2), max_relative = 1e-8);
    assert_relative_eq!(
        of(&r, "R2"),
        V * R1 / ((R1 + R2) * (R1 + R2)),
        max_relative = 1e-8
    );
}

#[test]
fn bad_output_is_an_error() {
    let c = divider();
    for output in [
        v_out("nowhere"),
        TfOutput::Current {
            source_id: "R1".into(),
        },
    ] {
        let err = solve_sensitivity(&c, &SensConfig { output });
        assert!(matches!(err, Err(SimError::InvalidComponent(_))), "{err:?}");
    }
}
