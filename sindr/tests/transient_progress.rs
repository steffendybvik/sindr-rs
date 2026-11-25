//! The progress hook on a transient: what it reports, and stopping a run.

use sindr::circuit::CrystalParams;
use sindr::element::{Capacitor, Crystal, Resistor, VoltageSource};
use sindr::{
    solve_transient, solve_transient_with_progress, Circuit, CircuitElement, Flow, SimError,
    Solver, TimestepSnapshot, TransientConfig, Waveform,
};

/// A 1 kHz square wave into R = 1 kΩ, C = 1 µF.
fn rc_square() -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Pulse {
                    v1: 0.0,
                    v2: 5.0,
                    delay: 0.0,
                    rise_time: 1e-6,
                    fall_time: 1e-6,
                    pulse_width: 0.5e-3,
                    period: 1e-3,
                }),
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "out".into()],
                resistance: 1e3,
            }),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 1e-6,
            }),
        ],
    )
}

#[test]
fn the_hook_sees_every_step_the_result_holds() {
    let config = TransientConfig::new(5e-3, 10e-6);
    let mut times = Vec::new();
    let mut voltages = Vec::new();
    let mut counts = Vec::new();
    let result = solve_transient_with_progress(&rc_square(), &config, |p| {
        times.push(p.time);
        counts.push(p.accepted_steps);
        voltages.push(p.step.expect("recorded").node_voltages["out"]);
        assert_eq!(p.tstop, 5e-3);
        Flow::Continue
    })
    .expect("runs");
    let tran = result.transient.unwrap();

    assert_eq!(times.len(), tran.timesteps.len());
    assert_eq!(counts, (1..=times.len()).collect::<Vec<_>>());
    assert!(times.windows(2).all(|w| w[0] < w[1]));
    assert!((times.last().unwrap() - 5e-3).abs() < 1e-12);
    for (v, s) in voltages.iter().zip(&tran.timesteps) {
        assert_eq!(*v, s.node_voltages["out"]);
    }

    // The hook changes nothing about the run.
    let plain = solve_transient(&rc_square(), &config)
        .unwrap()
        .transient
        .unwrap();
    assert_eq!(plain.timesteps.len(), tran.timesteps.len());
    assert_eq!(plain.stats, tran.stats);
}

#[test]
fn stop_cancels_the_run_at_the_step_it_was_asked() {
    let config = TransientConfig::new(5e-3, 10e-6);
    let e = solve_transient_with_progress(&rc_square(), &config, |p| {
        if p.fraction() >= 0.4 {
            Flow::Stop
        } else {
            Flow::Continue
        }
    })
    .unwrap_err();
    let SimError::Cancelled { time } = e else {
        panic!("{e}")
    };
    assert!((2e-3..2.1e-3).contains(&time), "stopped at {time}");
    assert!(e.to_string().starts_with("cancelled at t = 2.0"), "{e}");
}

#[test]
fn steps_before_tstart_are_reported_without_a_snapshot() {
    let mut config = TransientConfig::new(2e-3, 10e-6);
    config.tstart = 1e-3;
    let (mut before, mut after) = (0, 0);
    let result = solve_transient_with_progress(&rc_square(), &config, |p| {
        match p.step {
            None => {
                assert!(p.time < 1e-3);
                before += 1;
            }
            Some(_) => after += 1,
        }
        Flow::Continue
    })
    .unwrap();
    assert!(before > 0);
    assert_eq!(after, result.transient.unwrap().timesteps.len());
}

#[test]
fn a_solver_with_options_takes_the_hook_too() {
    let mut last = 0.0;
    Solver::default()
        .transient_with_progress(&rc_square(), &TransientConfig::new(1e-3, 10e-6), |p| {
            last = p.fraction();
            Flow::Continue
        })
        .unwrap();
    assert_eq!(last, 1.0);
}

/// Each part's (id, voltage, current, power), sorted by id.
fn parts(step: &TimestepSnapshot) -> Vec<(String, f64, f64, f64)> {
    let mut parts: Vec<_> = step
        .component_results
        .iter()
        .map(|c| (c.id.clone(), c.voltage_across, c.current_through, c.power))
        .collect();
    parts.sort_by(|a, b| a.0.cmp(&b.0));
    parts
}

#[test]
fn the_hook_sees_a_crystal_folded_as_the_result_holds_it() {
    // A 1 MHz pulse through 50 Ω and a 16 MHz crystal into 50 Ω.
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Pulse {
                    v1: 0.0,
                    v2: 1.0,
                    delay: 0.0,
                    rise_time: 10e-9,
                    fall_time: 10e-9,
                    pulse_width: 0.5e-6,
                    period: 1e-6,
                }),
            }),
            CircuitElement::Resistor(Resistor {
                id: "RS".into(),
                nodes: ["in".into(), "a".into()],
                resistance: 50.0,
            }),
            CircuitElement::Crystal(Crystal {
                id: "X1".into(),
                nodes: ["a".into(), "out".into()],
                params: CrystalParams {
                    frequency: 16e6,
                    esr: 40.0,
                    c0: 5e-12,
                    cm: 20e-15,
                },
            }),
            CircuitElement::Resistor(Resistor {
                id: "RL".into(),
                nodes: ["out".into(), "0".into()],
                resistance: 50.0,
            }),
        ],
    );
    let config = TransientConfig::new(2e-6, 10e-9);
    let mut seen = Vec::new();
    let result = solve_transient_with_progress(&circuit, &config, |p| {
        seen.push(p.step.expect("recorded").clone());
        Flow::Continue
    })
    .expect("runs");
    let tran = result.transient.unwrap();

    assert_eq!(seen.len(), tran.timesteps.len());
    for (hooked, kept) in seen.iter().zip(&tran.timesteps) {
        assert!(hooked.current("X1").is_some());
        assert!(hooked.node_voltages.keys().all(|n| !n.contains('#')));
        assert_eq!(hooked.time, kept.time);
        assert_eq!(hooked.node_voltages, kept.node_voltages);
        assert_eq!(parts(hooked), parts(kept));
    }
}
