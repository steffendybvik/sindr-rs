//! Transient sources: a jump at a breakpoint belongs to the step after it,
//! snapshots report each source's waveform value, and initial conditions
//! that name nothing in the circuit are errors.

use std::collections::HashMap;
use std::f64::consts::PI;

use sindr::{solve_transient, Circuit, CircuitElement, SimError, TransientConfig, Waveform};

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn capacitor(id: &str, a: &str, b: &str, capacitance: f64) -> CircuitElement {
    CircuitElement::Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance,
    }
}

/// `waveform` into 1 kΩ / 100 nF (τ = 100 µs).
fn rc(waveform: Waveform) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(waveform),
            },
            resistor("R1", "in", "out", 1e3),
            capacitor("C1", "out", "0", 100e-9),
        ],
    }
}

fn square() -> Waveform {
    Waveform::Square {
        amplitude: 1.0,
        frequency: 1e3,
        offset: 0.0,
        duty: 0.5,
    }
}

/// A 1 kHz ±1 V square stepped at τ: the snapshot at the falling edge
/// (0.5 ms) still sees +1 V, with C charged to ≈ 1 V over 5τ, and the drop
/// shows up in the step after it.
#[test]
fn a_jump_lands_in_the_step_after_its_breakpoint() {
    let config = TransientConfig::new(1e-3, 100e-6).with_fixed_step();
    let tran = solve_transient(&rc(square()), &config)
        .unwrap()
        .transient
        .unwrap();
    let at = |t: f64| {
        tran.timesteps
            .iter()
            .find(|s| (s.time - t).abs() < 1e-12)
            .unwrap_or_else(|| panic!("no snapshot at {t}"))
    };
    let edge = at(0.5e-3);
    assert_eq!(edge.node_voltages["in"], 1.0);
    assert!(
        edge.node_voltages["out"] > 0.95,
        "out {}",
        edge.node_voltages["out"]
    );
    assert_eq!(at(0.6e-3).node_voltages["in"], -1.0);
}

/// A PULSE with zero rise and fall times: the step ending on each edge sees
/// the level before it.
#[test]
fn a_zero_rise_pulse_switches_in_the_step_after_each_edge() {
    let pulse = Waveform::Pulse {
        v1: 0.0,
        v2: 1.0,
        delay: 200e-6,
        rise_time: 0.0,
        fall_time: 0.0,
        pulse_width: 300e-6,
        period: 2e-3,
    };
    let config = TransientConfig::new(1e-3, 100e-6).with_fixed_step();
    let tran = solve_transient(&rc(pulse), &config)
        .unwrap()
        .transient
        .unwrap();
    let v_in = |t: f64| {
        tran.timesteps
            .iter()
            .find(|s| (s.time - t).abs() < 1e-12)
            .unwrap_or_else(|| panic!("no snapshot at {t}"))
            .node_voltages["in"]
    };
    assert_eq!(v_in(200e-6), 0.0);
    assert_eq!(v_in(300e-6), 1.0);
    assert_eq!(v_in(500e-6), 1.0);
    assert_eq!(v_in(600e-6), 0.0);
}

/// Snapshots report a waveform source's value at that time, not its DC
/// term, and power from it.
#[test]
fn sources_report_their_waveform_value() {
    let sine = Waveform::Sine {
        amplitude: 1.0,
        frequency: 1e3,
        offset: 0.0,
        phase: 0.0,
    };
    let config = TransientConfig::new(1e-3, 10e-6).with_fixed_step();
    let tran = solve_transient(&rc(sine), &config)
        .unwrap()
        .transient
        .unwrap();
    for s in &tran.timesteps {
        let v1 = s.component_results.iter().find(|c| c.id == "V1").unwrap();
        let expected = (2.0 * PI * 1e3 * s.time).sin();
        assert!(
            (v1.voltage_across - expected).abs() < 1e-12,
            "t = {}",
            s.time
        );
        assert!((v1.voltage_across - s.node_voltages["in"]).abs() < 1e-9);
        assert!((v1.power - v1.voltage_across * v1.current_through).abs() < 1e-15);
    }
    let i1 = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "out".into()],
                current: 0.0,
                waveform: Some(Waveform::Sine {
                    amplitude: 1e-3,
                    frequency: 1e3,
                    offset: 0.0,
                    phase: 0.0,
                }),
            },
            resistor("R1", "out", "0", 1e3),
            capacitor("C1", "out", "0", 100e-9),
        ],
    };
    let tran = solve_transient(&i1, &config).unwrap().transient.unwrap();
    for s in &tran.timesteps {
        let i = s.component_results.iter().find(|c| c.id == "I1").unwrap();
        let expected = 1e-3 * (2.0 * PI * 1e3 * s.time).sin();
        assert!(
            (i.current_through - expected).abs() < 1e-15,
            "t = {}",
            s.time
        );
    }
}

fn assert_rejected(circuit: &Circuit, config: TransientConfig, needle: &str) {
    match solve_transient(circuit, &config) {
        Err(SimError::InvalidComponent(msg)) => {
            assert!(msg.contains(needle), "{msg:?} lacks {needle:?}")
        }
        other => panic!("{needle}: expected InvalidComponent, got {other:?}"),
    }
}

#[test]
fn initial_conditions_that_name_nothing_are_errors() {
    let mut circuit = rc(square());
    circuit.components.push(CircuitElement::Inductor {
        id: "L1".into(),
        nodes: ["out".into(), "m".into()],
        inductance: 1e-3,
    });
    circuit.components.push(resistor("R2", "m", "0", 1e3));
    circuit.components.push(CircuitElement::Transformer {
        id: "T1".into(),
        nodes: ["out".into(), "0".into(), "s".into(), "0".into()],
        l1: 1e-3,
        l2: 1e-3,
        k: 0.99,
    });
    circuit.components.push(resistor("RS", "s", "0", 1e3));
    let base = || TransientConfig::new(1e-3, 10e-6);
    let volts = |node: &str, v: f64| HashMap::from([(node.to_string(), v)]);

    assert_rejected(
        &circuit,
        base().with_initial_voltages(volts("ot", 1.0)),
        "'ot'",
    );
    assert_rejected(
        &circuit,
        base().with_initial_voltages(volts("out", f64::NAN)),
        "'out'",
    );
    assert_rejected(
        &circuit,
        base().with_initial_voltages(volts("0", 1.0)),
        "'0'",
    );
    assert_rejected(&circuit, base().with_initial_current("L2", 1e-3), "'L2'");
    assert_rejected(
        &circuit,
        base().with_initial_current("L1", f64::INFINITY),
        "'L1'",
    );
    assert_rejected(
        &circuit,
        base().with_initial_current("T1", 1e-3),
        "transformer",
    );
    assert_rejected(&circuit, base().with_initial_current("R1", 1e-3), "'R1'");

    // The valid ones still run.
    let ok = base()
        .with_initial_voltages(volts("out", 0.5))
        .with_initial_current("L1", 1e-3);
    solve_transient(&circuit, &ok).unwrap();
}
