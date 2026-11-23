//! The progress hook on a transient: what it reports, and stopping a run.

use sindr::element::{Capacitor, Resistor, VoltageSource};
use sindr::{
    solve_transient, solve_transient_with_progress, Circuit, CircuitElement, Flow, SimError,
    Solver, TransientConfig, Waveform,
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
