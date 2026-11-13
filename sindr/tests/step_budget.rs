//! Every path that steps or sweeps is bounded: a config that asks for more
//! than 10⁷ transient steps or sweep points returns an error instead of
//! hanging or failing to allocate.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{
    dc_sweep, solve_circuit, solve_transient, temperature_sweep, Circuit, CircuitElement, SimError,
    TransientConfig, Waveform,
};

fn vsource(id: &str, p: &str, n: &str, waveform: Option<Waveform>) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: 0.0,
        waveform,
    }
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn pulse(delay: f64, period: f64) -> Waveform {
    Waveform::Pulse {
        v1: 0.0,
        v2: 1.0,
        delay,
        rise_time: period / 10.0,
        fall_time: period / 10.0,
        pulse_width: period / 4.0,
        period,
    }
}

fn assert_invalid(result: Result<impl std::fmt::Debug, SimError>, needle: &str) {
    match result {
        Err(SimError::InvalidComponent(msg)) => {
            assert!(msg.contains(needle), "message {msg:?} lacks {needle:?}")
        }
        other => panic!("expected InvalidComponent, got {other:?}"),
    }
}

/// A 1 kHz sine into 1 kΩ with 1 µF behind a 1 GΩ leak: the automatic span
/// is 5000 s at a 10 µs step, 5·10⁸ steps. `solve_circuit` must refuse it as
/// `solve_transient` does, not run it.
#[test]
fn solve_circuit_rejects_an_automatic_config_over_the_step_cap() {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource(
                "V1",
                "in",
                "0",
                Some(Waveform::Sine {
                    amplitude: 1.0,
                    frequency: 1e3,
                    offset: 0.0,
                    phase: 0.0,
                }),
            ),
            resistor("R1", "in", "out", 1e3),
            resistor("RL", "out", "x", 1e9),
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["x".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    };
    assert_invalid(solve_circuit(&circuit), "automatic span");
}

/// Breakpoints force steps of their own: 9·10⁶ grid steps pass validation
/// alone, but a 1 ns pulse adds 4·10⁶ edges over 1 ms.
#[test]
fn breakpoints_count_against_the_step_cap() {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "in", "0", Some(pulse(0.0, 1e-9))),
            resistor("R1", "in", "0", 1e3),
        ],
    };
    let config = TransientConfig::new(1e-3, 1.1e-10);
    assert_invalid(solve_transient(&circuit, &config), "waveform breakpoints");
}

/// A source with more breakpoints than are kept is rejected by name, not
/// stepped over past the last one kept.
#[test]
fn a_source_with_too_many_breakpoints_is_named() {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("VFAST", "in", "0", Some(pulse(0.0, 1e-9))),
            resistor("R1", "in", "0", 1e3),
        ],
    };
    let config = TransientConfig::new(3e-3, 1e-5);
    assert_invalid(solve_transient(&circuit, &config), "'VFAST'");
}

fn divider() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 1.0,
                waveform: None,
            },
            resistor("R1", "in", "out", 1e3),
            resistor("R2", "out", "0", 1e3),
        ],
    }
}

#[test]
fn huge_sweeps_are_errors_not_panics() {
    let ac = AcConfig {
        f_start: 1.0,
        f_stop: 1e6,
        num_points: usize::MAX,
        spacing: FrequencySpacing::Logarithmic,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    };
    assert_invalid(solve_ac(&divider(), &ac), "num_points");
    assert_invalid(
        dc_sweep(&divider(), "V1", 0.0, 1.0, usize::MAX),
        "num_points",
    );
    assert_invalid(
        temperature_sweep(&divider(), 250.0, 400.0, usize::MAX),
        "num_steps",
    );
}
