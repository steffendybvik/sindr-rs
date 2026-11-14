//! Timestep control: steps land on waveform breakpoints, shrink where the
//! local truncation error demands, and grow on smooth stretches.

use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig, TransientData, Waveform};

/// 0 → 1 V pulse at 10 µs with 1 ns edges, 40 µs wide, into RC with τ = 1 µs.
const DELAY: f64 = 10e-6;
const EDGE: f64 = 1e-9;
const WIDTH: f64 = 40e-6;
const TAU: f64 = 1e-6;

fn pulse_rc() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Pulse {
                    v1: 0.0,
                    v2: 1.0,
                    delay: DELAY,
                    rise_time: EDGE,
                    fall_time: EDGE,
                    pulse_width: WIDTH,
                    period: 1.0,
                }),
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "out".into()],
                resistance: 1e3,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: TAU / 1e3,
            },
        ],
    }
}

/// The RC output, treating each 1 ns edge as an ideal step at its midpoint
/// (an error of about edge/2τ = 5e-4).
fn exact(t: f64) -> f64 {
    let rise = DELAY + EDGE / 2.0;
    let fall = DELAY + EDGE + WIDTH + EDGE / 2.0;
    if t < rise {
        0.0
    } else if t < fall {
        1.0 - (-(t - rise) / TAU).exp()
    } else {
        (1.0 - (-(fall - rise) / TAU).exp()) * (-(t - fall) / TAU).exp()
    }
}

fn run(config: &TransientConfig) -> TransientData {
    solve_transient(&pulse_rc(), config)
        .unwrap()
        .transient
        .unwrap()
}

fn has_snapshot_at(tran: &TransientData, t: f64) -> bool {
    tran.timesteps
        .iter()
        .any(|s| (s.time - t).abs() <= 1e-12 * t)
}

/// A 100 µs run with output every 10 µs: the 1 ns edges fall between grid
/// points, yet the stepper lands on all four corners of the pulse, follows
/// the RC response to within the tolerance, and does it in a small fraction
/// of the steps a fixed step fine enough for the edges would take.
#[test]
fn pulse_edges_land_on_breakpoints_and_are_resolved() {
    let tran = run(&TransientConfig::new(100e-6, 10e-6));

    for corner in [
        DELAY,
        DELAY + EDGE,
        DELAY + EDGE + WIDTH,
        DELAY + 2.0 * EDGE + WIDTH,
    ] {
        assert!(has_snapshot_at(&tran, corner), "no snapshot at {corner}");
    }
    for s in &tran.timesteps {
        let (got, want) = (s.node_voltages["out"], exact(s.time));
        assert!((got - want).abs() < 1e-2, "t = {}: {got} vs {want}", s.time);
    }

    // The shortest step taken is the 1 ns edge; a fixed step that short
    // would need 1e5 steps for the run.
    let shortest = tran
        .timesteps
        .windows(2)
        .map(|w| w[1].time - w[0].time)
        .fold(f64::MAX, f64::min);
    assert!(shortest <= EDGE * (1.0 + 1e-6), "shortest step {shortest}");
    let stats = tran.stats;
    assert_eq!(stats.accepted_steps, tran.timesteps.len());
    assert!(
        stats.accepted_steps < 1_000,
        "{} steps (fixed at the edge would be {})",
        stats.accepted_steps,
        (100e-6 / EDGE) as usize
    );
}

/// The step grows back after an edge: most of the flat stretch after the
/// response has settled is covered by the longest step allowed.
#[test]
fn step_grows_on_smooth_stretches() {
    let tran = run(&TransientConfig::new(100e-6, 10e-6));
    let long_steps = tran
        .timesteps
        .windows(2)
        .filter(|w| w[0].time > 70e-6 && w[1].time - w[0].time > 5e-6)
        .count();
    assert!(long_steps >= 2, "{long_steps} long steps after 70 µs");
}

/// With a step too coarse for the response, the error control rejects
/// steps and shortens them, and counts the rejections.
#[test]
fn lte_rejections_are_counted() {
    let tran = run(&TransientConfig::new(100e-6, 10e-6));
    assert!(tran.stats.lte_rejections > 0, "{:?}", tran.stats);
    assert_eq!(tran.stats.newton_rejections, 0);
}

/// Fixed-step runs still land on breakpoints, so the edges aren't skipped,
/// but take every other step at `tstep`.
#[test]
fn fixed_step_runs_land_on_breakpoints() {
    let tran = run(&TransientConfig::new(100e-6, 10e-6).with_fixed_step());
    for corner in [
        DELAY,
        DELAY + EDGE,
        DELAY + EDGE + WIDTH,
        DELAY + 2.0 * EDGE + WIDTH,
    ] {
        assert!(has_snapshot_at(&tran, corner), "no snapshot at {corner}");
    }
    assert_eq!(tran.stats.lte_rejections, 0);
}
