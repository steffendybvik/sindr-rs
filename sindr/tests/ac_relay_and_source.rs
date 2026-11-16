//! AC and `.tf` linearise the circuit DC solved: a relay contact is closed
//! when its coil is above pickup at the bias point. And an AC source id that
//! names no voltage source is an error, not a sweep of zeros.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::transfer_function::{solve_tf, TfConfig, TfOutput};
use sindr::{Circuit, CircuitElement, SimError};

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

/// `v_coil` across a relay coil (pickup 5 V); V1 through 1 kΩ and the
/// contact into a 1 kΩ load. Closed, the gain to `out` is 0.5.
fn relay_divider(v_coil: f64) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("VCOIL", "coil", "0", v_coil),
            CircuitElement::Relay {
                id: "K1".into(),
                nodes: ["coil".into(), "0".into(), "a".into(), "out".into()],
                coil_resistance: 500.0,
                pickup_voltage: 5.0,
                inductance: 0.0,
            },
            vsource("V1", "in", "0", 1.0),
            resistor("R1", "in", "a", 1e3),
            resistor("RL", "out", "0", 1e3),
        ],
    }
}

fn ac(source_id: &str) -> AcConfig {
    AcConfig {
        f_start: 1e3,
        f_stop: 1e3,
        num_points: 1,
        spacing: FrequencySpacing::Linear,
        source_id: source_id.into(),
        ac_magnitude: 1.0,
    }
}

fn tf() -> TfConfig {
    TfConfig {
        output: TfOutput::Voltage {
            node: "out".into(),
            reference: None,
        },
        input_source: "V1".into(),
    }
}

#[test]
fn ac_and_tf_see_the_relay_contact_dc_closed() {
    let closed = relay_divider(12.0);
    let gain = solve_ac(&closed, &ac("V1")).unwrap().points[0].node_voltages["out"];
    assert!(
        (gain.re - 0.5).abs() < 1e-4 && gain.im.abs() < 1e-12,
        "AC {gain}"
    );
    let tf_gain = solve_tf(&closed, &tf()).unwrap().gain;
    assert!((tf_gain - 0.5).abs() < 1e-4, ".tf {tf_gain}");

    let open = relay_divider(1.0);
    let gain = solve_ac(&open, &ac("V1")).unwrap().points[0].node_voltages["out"];
    assert!(gain.norm() < 1e-5, "AC open {gain}");
    let tf_gain = solve_tf(&open, &tf()).unwrap().gain;
    assert!(tf_gain.abs() < 1e-5, ".tf open {tf_gain}");
}

#[test]
fn ac_rejects_a_source_id_that_is_not_a_voltage_source() {
    let mut circuit = relay_divider(12.0);
    circuit.components.push(CircuitElement::CurrentSource {
        id: "I1".into(),
        nodes: ["0".into(), "out".into()],
        current: 0.0,
        waveform: None,
    });
    for id in ["V9", "I1", "R1"] {
        match solve_ac(&circuit, &ac(id)) {
            Err(SimError::InvalidComponent(msg)) => {
                assert!(msg.contains(&format!("'{id}'")), "{msg}")
            }
            other => panic!("{id}: expected InvalidComponent, got {other:?}"),
        }
    }
}
