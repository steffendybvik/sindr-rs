//! `.tf` against closed-form answers for linear circuits, and against
//! finite differences of full DC solves for nonlinear ones.

use approx::assert_relative_eq;
use sindr::{
    solve_circuit, solve_tf, BjtKind, Circuit, CircuitElement, SimError, TfConfig, TfOutput,
};

fn vsource(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

fn isource(id: &str, from: &str, to: &str, current: f64) -> CircuitElement {
    CircuitElement::CurrentSource {
        id: id.into(),
        nodes: [from.into(), to.into()],
        current,
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

fn opamp(id: &str, plus: &str, minus: &str, out: &str) -> CircuitElement {
    CircuitElement::OpAmp {
        id: id.into(),
        nodes: [plus.into(), minus.into(), out.into()],
        v_pos: 15.0,
        v_neg: -15.0,
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

fn tf(circuit: &Circuit, output: TfOutput, input: &str) -> sindr::TfResult {
    solve_tf(
        circuit,
        &TfConfig {
            output,
            input_source: input.into(),
        },
    )
    .unwrap()
}

/// 1 kΩ over 3 kΩ: gain 3/4, Rin 4 kΩ, Rout 1k ‖ 3k = 750 Ω.
fn divider() -> Circuit {
    circuit(vec![
        vsource("V1", "in", "0", 2.0),
        resistor("R1", "in", "out", 1e3),
        resistor("R2", "out", "0", 3e3),
    ])
}

#[test]
fn divider_voltage_output() {
    let r = tf(&divider(), v_out("out"), "V1");
    assert_relative_eq!(r.gain, 0.75, max_relative = 1e-12);
    assert_relative_eq!(r.input_resistance, 4e3, max_relative = 1e-12);
    assert_relative_eq!(r.output_resistance, 750.0, max_relative = 1e-12);
}

#[test]
fn output_against_a_reference_node() {
    // V(in, out) is the drop across R1: gain 1/4. Between `in` (shorted to
    // ground through V1) and `out` sits 1k ‖ 3k.
    let output = TfOutput::Voltage {
        node: "in".into(),
        reference: Some("out".into()),
    };
    let r = tf(&divider(), output, "V1");
    assert_relative_eq!(r.gain, 0.25, max_relative = 1e-12);
    assert_relative_eq!(r.output_resistance, 750.0, max_relative = 1e-12);
}

#[test]
fn current_output_through_a_voltage_source() {
    // V1 delivers 1/4k A per volt, so the current from its + terminal
    // through it is −1/4k; the resistance in series with it is 4 kΩ.
    let output = TfOutput::Current {
        source_id: "V1".into(),
    };
    let r = tf(&divider(), output, "V1");
    assert_relative_eq!(r.gain, -1.0 / 4e3, max_relative = 1e-12);
    assert_relative_eq!(r.output_resistance, 4e3, max_relative = 1e-12);
}

#[test]
fn current_source_input() {
    // I1 pushes current from ground into `a`, which sees 2k ‖ (1k + 3k).
    let c = circuit(vec![
        isource("I1", "0", "a", 1e-3),
        resistor("R1", "a", "0", 2e3),
        resistor("R2", "a", "b", 1e3),
        resistor("R3", "b", "0", 3e3),
    ]);
    let r_a = 2e3 * 4e3 / 6e3;
    let r = tf(&c, v_out("b"), "I1");
    assert_relative_eq!(r.gain, r_a * 0.75, max_relative = 1e-12);
    assert_relative_eq!(r.input_resistance, r_a, max_relative = 1e-12);
    // At `b`: 3k ‖ (1k + 2k).
    assert_relative_eq!(r.output_resistance, 1.5e3, max_relative = 1e-12);
}

#[test]
fn inverting_amplifier_with_finite_gain() {
    // The op-amp is a VCVS of gain A: the closed-loop gain is
    // −(Rf/Ri) / (1 + (1 + Rf/Ri)/A), the input sees Ri plus the virtual
    // ground's Rf/(1 + A), and the ideal output has no resistance.
    let (a, ri, rf) = (1e5, 1e3, 10e3);
    let c = circuit(vec![
        vsource("V1", "in", "0", 0.1),
        resistor("Ri", "in", "minus", ri),
        resistor("Rf", "minus", "out", rf),
        opamp("U1", "0", "minus", "out"),
        resistor("RL", "out", "0", 2e3),
    ]);
    let r = tf(&c, v_out("out"), "V1");
    assert_relative_eq!(
        r.gain,
        -(rf / ri) / (1.0 + (1.0 + rf / ri) / a),
        max_relative = 1e-9
    );
    assert_relative_eq!(r.input_resistance, ri + rf / (1.0 + a), max_relative = 1e-9);
    assert!(r.output_resistance.abs() < 1e-9, "{}", r.output_resistance);
}

#[test]
fn source_drawing_no_current_has_infinite_input_resistance() {
    // A follower: V1 sees only the op-amp's input.
    let c = circuit(vec![
        vsource("V1", "in", "0", 1.0),
        opamp("U1", "in", "out", "out"),
        resistor("RL", "out", "0", 1e3),
        resistor("Rb", "in", "0", 1e12),
    ]);
    let c_no_bleed = circuit(c.components[..3].to_vec());
    let r = tf(&c_no_bleed, v_out("out"), "V1");
    assert_relative_eq!(r.gain, 1e5 / (1.0 + 1e5), max_relative = 1e-9);
    assert_eq!(r.input_resistance, f64::INFINITY);
    let r = tf(&c, v_out("out"), "V1");
    assert_relative_eq!(r.input_resistance, 1e12, max_relative = 1e-9);
}

#[test]
fn reactive_parts_take_their_dc_values() {
    // The inductor is a short and the capacitor open: the divider is
    // R1 over R2 with L in series, C across R2.
    let mut c = divider();
    c.components[1] = resistor("R1", "in", "mid", 1e3);
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
    let r = tf(&c, v_out("out"), "V1");
    // C1 is a 1 TΩ leak at DC, so the answers are the divider's to ~1e-9.
    assert_relative_eq!(r.gain, 0.75, max_relative = 1e-8);
    assert_relative_eq!(r.input_resistance, 4e3, max_relative = 1e-8);
    assert_relative_eq!(r.output_resistance, 750.0, max_relative = 1e-8);
}

/// `circuit` with source `V1` set to `volts`.
fn with_v1(circuit: &Circuit, volts: f64) -> Circuit {
    let mut c = circuit.clone();
    for e in &mut c.components {
        if let CircuitElement::VoltageSource { id, voltage, .. } = e {
            if id == "V1" {
                *voltage = volts;
            }
        }
    }
    c
}

/// Central differences of full DC solves around `V1 = v0`: the gain to
/// `out` and the resistance V1 sees, through `feed`, the one resistor V1
/// drives.
fn finite_difference(circuit: &Circuit, v0: f64, out: &str, feed: &str) -> (f64, f64) {
    const H: f64 = 1e-5;
    let lo = solve_circuit(&with_v1(circuit, v0 - H)).unwrap();
    let hi = solve_circuit(&with_v1(circuit, v0 + H)).unwrap();
    let gain = (hi.node_voltages[out] - lo.node_voltages[out]) / (2.0 * H);
    let current = |r: &sindr::SimulationResult| {
        r.component_results
            .iter()
            .find(|c| c.id == feed)
            .map(|c| c.current_through)
            .unwrap()
    };
    let rin = 2.0 * H / (current(&hi) - current(&lo));
    (gain, rin)
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
    for v0 in [0.3, 0.7, 5.0] {
        let at = with_v1(&c, v0);
        let r = tf(&at, v_out("out"), "V1");
        let (gain, rin) = finite_difference(&at, v0, "out", "R1");
        assert_relative_eq!(r.gain, gain, max_relative = 1e-5);
        // Near cut-off the difference of two tiny currents keeps fewer
        // digits than the gain's.
        assert_relative_eq!(r.input_resistance, rin, max_relative = 1e-4);
        // The diode's small-signal resistance in parallel with R1.
        let r_d = 1e3 * r.gain / (1.0 - r.gain);
        assert_relative_eq!(
            r.output_resistance,
            1e3 * r_d / (1e3 + r_d),
            max_relative = 1e-9
        );
    }
}

#[test]
fn common_emitter_stage_matches_finite_differences() {
    let c = circuit(vec![
        vsource("VCC", "vcc", "0", 12.0),
        // V1 biases the base through RS: Ve ≈ 1.3 V, Ic ≈ 2.8 mA, active.
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
    let r = tf(&c, v_out("c"), "V1");
    let (gain, rin) = finite_difference(&c, 2.0, "c", "RS");
    assert!(r.gain < -1.0, "a CE stage inverts with gain: {}", r.gain);
    assert_relative_eq!(r.gain, gain, max_relative = 1e-5);
    assert_relative_eq!(r.input_resistance, rin, max_relative = 1e-5);
}

#[test]
fn bad_input_or_output_is_an_error() {
    let c = divider();
    let run = |output: TfOutput, input: &str| {
        solve_tf(
            &c,
            &TfConfig {
                output,
                input_source: input.into(),
            },
        )
    };
    for err in [
        run(v_out("out"), "R1"),
        run(v_out("out"), "nope"),
        run(v_out("nowhere"), "V1"),
        run(
            TfOutput::Current {
                source_id: "R2".into(),
            },
            "V1",
        ),
    ] {
        assert!(matches!(err, Err(SimError::InvalidComponent(_))), "{err:?}");
    }
    // Ground is a valid output node: the gain to it is zero.
    let r = run(v_out("0"), "V1").unwrap();
    assert_eq!(r.gain, 0.0);
}
