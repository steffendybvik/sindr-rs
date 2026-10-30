//! `TransientConfig` and `solve_transient`: invalid configs are rejected, the
//! requested span and step are honoured, sources are sampled at the end of
//! each step, and SPICE `.tran` cards map onto the config.

use std::f64::consts::PI;

use sindr::{
    solve_circuit, solve_transient, Circuit, CircuitElement, SimError, TransientConfig, Waveform,
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

/// 1 V into 1 kΩ / 1 µF: τ = 1 ms.
fn rc() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "in", "0", 1.0),
            resistor("R1", "in", "out", 1e3),
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    }
}

#[test]
fn invalid_configs_are_rejected() {
    let nan = f64::NAN;
    let inf = f64::INFINITY;
    let cases = [
        ("tstop = 0", TransientConfig::new(0.0, 1e-6)),
        ("tstop < 0", TransientConfig::new(-1e-3, 1e-6)),
        ("tstop NaN", TransientConfig::new(nan, 1e-6)),
        ("tstop inf", TransientConfig::new(inf, 1e-6)),
        ("tstep = 0", TransientConfig::new(1e-3, 0.0)),
        ("tstep < 0", TransientConfig::new(1e-3, -1e-6)),
        ("tstep NaN", TransientConfig::new(1e-3, nan)),
        ("tstep > tstop", TransientConfig::new(1e-3, 2e-3)),
        ("tmax = 0", TransientConfig::new(1e-3, 1e-6).with_tmax(0.0)),
        ("tmax NaN", TransientConfig::new(1e-3, 1e-6).with_tmax(nan)),
        ("too many steps", TransientConfig::new(1.0, 1e-9)),
    ];
    for (case, config) in cases {
        match solve_transient(&rc(), &config) {
            Err(SimError::InvalidComponent(msg)) => {
                assert!(msg.starts_with("transient:"), "{case}: {msg}")
            }
            other => panic!("{case}: expected InvalidComponent, got {other:?}"),
        }
    }
}

/// `.tran`-style 10 µs steps to 2 ms: 200 snapshots on the step grid, the
/// last at exactly tstop, each the backward-Euler update
/// `v_n = (v_{n−1} + (dt/τ)·V) / (1 + dt/τ)`.
#[test]
fn requested_span_and_step_are_honoured() {
    let (tstop, tstep, tau) = (2e-3, 10e-6, 1e-3);
    let tran = solve_transient(&rc(), &TransientConfig::new(tstop, tstep))
        .unwrap()
        .transient
        .unwrap();
    assert_eq!(tran.timesteps.len(), 200);
    assert_eq!(tran.time_step, tstep);
    assert_eq!(tran.duration, tstop);
    assert_eq!(tran.timesteps.last().unwrap().time, tstop);

    let mut v = 0.0;
    for (i, s) in tran.timesteps.iter().enumerate() {
        assert!((s.time - (i + 1) as f64 * tstep).abs() < 1e-12 * tstop);
        v = (v + tstep / tau) / (1.0 + tstep / tau);
        assert!((s.node_voltages["out"] - v).abs() < 1e-12, "t = {}", s.time);
    }
}

/// A tstop that isn't a multiple of tstep ends on a short last step.
#[test]
fn last_step_lands_on_tstop() {
    let tran = solve_transient(&rc(), &TransientConfig::new(1.05e-3, 0.1e-3))
        .unwrap()
        .transient
        .unwrap();
    let times: Vec<f64> = tran.timesteps.iter().map(|s| s.time).collect();
    assert_eq!(times.len(), 11);
    assert!((times[9] - 1.0e-3).abs() < 1e-15);
    assert_eq!(times[10], 1.05e-3);
}

/// `tmax` below `tstep` caps the step.
#[test]
fn tmax_caps_the_step() {
    let tran = solve_transient(&rc(), &TransientConfig::new(1e-3, 100e-6).with_tmax(25e-6))
        .unwrap()
        .transient
        .unwrap();
    assert_eq!(tran.timesteps.len(), 40);
    assert!((tran.timesteps[0].time - 25e-6).abs() < 1e-18);
}

/// Backward Euler solves at the end of each step, so a resistive divider
/// fed by a sine reads the source's value at the snapshot's own time, with
/// no lag of one step.
#[test]
fn sources_are_sampled_at_the_end_of_each_step() {
    let (amplitude, frequency) = (2.0, 1e3);
    let mut divider = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "in", "0", 0.0),
            resistor("R1", "in", "out", 1e3),
            resistor("R2", "out", "0", 3e3),
        ],
    };
    if let CircuitElement::VoltageSource { waveform, .. } = &mut divider.components[0] {
        *waveform = Some(Waveform::Sine {
            amplitude,
            frequency,
            offset: 0.0,
            phase: 0.0,
        });
    }
    let tran = solve_transient(&divider, &TransientConfig::new(2e-3, 10e-6))
        .unwrap()
        .transient
        .unwrap();
    for s in &tran.timesteps {
        let want = 0.75 * amplitude * (2.0 * PI * frequency * s.time).sin();
        assert!(
            (s.node_voltages["out"] - want).abs() < 1e-12,
            "t = {}: {} vs {want}",
            s.time,
            s.node_voltages["out"]
        );
    }
}

/// `solve_circuit` runs the same stepper with `TransientConfig::auto`.
#[test]
fn solve_circuit_uses_the_auto_config() {
    let circuit = rc();
    let auto = TransientConfig::auto(&circuit);
    assert_eq!((auto.tstop, auto.tstep), (5e-3, 20e-6));
    let a = solve_circuit(&circuit).unwrap().transient.unwrap();
    let b = solve_transient(&circuit, &auto).unwrap().transient.unwrap();
    assert_eq!(a.timesteps.len(), 250);
    assert_eq!(a.timesteps.last().unwrap().time, 5e-3);
    for (x, y) in a.timesteps.iter().zip(&b.timesteps) {
        assert_eq!(x.node_voltages["out"], y.node_voltages["out"]);
    }
}

/// Half-wave rectifier into an RC load: a nonlinear transient that runs on
/// the requested grid and peaks one diode drop below the source.
#[test]
fn nonlinear_transient_honours_the_config() {
    let mut circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "in", "0", 0.0),
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["in".into(), "out".into()],
                temperature: 300.15,
            },
            resistor("RL", "out", "0", 10e3),
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    };
    if let CircuitElement::VoltageSource { waveform, .. } = &mut circuit.components[0] {
        *waveform = Some(Waveform::Sine {
            amplitude: 5.0,
            frequency: 50.0,
            offset: 0.0,
            phase: 0.0,
        });
    }
    let tran = solve_transient(&circuit, &TransientConfig::new(40e-3, 20e-6))
        .unwrap()
        .transient
        .unwrap();
    assert_eq!(tran.timesteps.len(), 2000);
    assert_eq!(tran.timesteps.last().unwrap().time, 40e-3);
    let peak = tran
        .timesteps
        .iter()
        .map(|s| s.node_voltages["out"])
        .fold(f64::MIN, f64::max);
    assert!(peak > 4.2 && peak < 4.5, "peak {peak}");
}

/// The solution doesn't depend on the order components are listed in, with
/// a voltage regulator ahead of a transformer and a diode making it
/// nonlinear (each takes MNA branch rows, so the order moves them).
#[test]
fn transient_ignores_component_order() {
    let components = vec![
        vsource("VIN", "in", "0", 12.0),
        CircuitElement::VoltageRegulator {
            id: "U1".into(),
            nodes: ["in".into(), "reg".into(), "0".into()],
            voltage: 5.0,
        },
        resistor("RREG", "reg", "0", 1e3),
        vsource("VAC", "p", "0", 0.0),
        CircuitElement::Transformer {
            id: "T1".into(),
            nodes: ["p".into(), "0".into(), "s".into(), "0".into()],
            l1: 10e-3,
            l2: 10e-3,
            k: 0.99,
        },
        CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["s".into(), "out".into()],
            temperature: 300.15,
        },
        resistor("RL", "out", "0", 100.0),
    ];
    let mut forward = Circuit {
        ground_node: "0".into(),
        components,
    };
    if let CircuitElement::VoltageSource { waveform, .. } = &mut forward.components[3] {
        *waveform = Some(Waveform::Sine {
            amplitude: 5.0,
            frequency: 1e3,
            offset: 0.0,
            phase: 0.0,
        });
    }
    let mut reversed = forward.clone();
    reversed.components.reverse();

    let config = TransientConfig::new(2e-3, 5e-6);
    let a = solve_transient(&forward, &config)
        .unwrap()
        .transient
        .unwrap();
    let b = solve_transient(&reversed, &config)
        .unwrap()
        .transient
        .unwrap();
    assert_eq!(a.timesteps.len(), b.timesteps.len());
    for (x, y) in a.timesteps.iter().zip(&b.timesteps) {
        for node in ["reg", "s", "out"] {
            let (u, w) = (x.node_voltages[node], y.node_voltages[node]);
            assert!((u - w).abs() < 1e-6, "t = {}, {node}: {u} vs {w}", x.time);
        }
    }
}
