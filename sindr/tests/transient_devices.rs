//! Transient paths of particular devices and of the stepper's Newton loop:
//! a MOSFET's and a JFET's gate capacitance, a relay coil starting at its
//! DC current, and a step whose Newton iteration fails.

use sindr::circuit::{JfetParasiticCaps, MosfetParasiticCaps};
use sindr::{
    solve_transient, Circuit, CircuitElement, JfetKind, MosfetKind, SimError, SolvePoint, Solver,
    SolverOptions, TransientConfig, TransientStart, Waveform,
};
use sindr_devices::mosfet::MosfetParams;

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    }
}

fn v(id: &str, p: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), "0".into()],
        voltage: volts,
        waveform: None,
    }
}

/// An NMOS whose gate is driven through `RG` from `vg`, with only Cgs.
fn gate_rc(vg: f64) -> Circuit {
    Circuit::new(
        "0",
        vec![
            v("VG", "in", vg),
            r("RG", "in", "g", 10e3),
            v("VDD", "vdd", 10.0),
            r("RD", "vdd", "d", 1e3),
            CircuitElement::Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind: MosfetKind::Nmos,
                params: MosfetParams::default_nmos(),
                parasitic_caps: Some(MosfetParasiticCaps {
                    cgs: 1e-9,
                    cgd: 0.0,
                }),
                body_diode: None,
            },
        ],
    )
}

/// The gate draws no DC current, so switched on at zero it charges Cgs
/// through RG: V(g) = 5·(1 − e^(−t/τ)), τ = RG·Cgs = 10 µs.
#[test]
fn mosfet_gate_charges_through_its_cgs() {
    let tau = 10e3 * 1e-9;
    let result = solve_transient(
        &gate_rc(5.0),
        &TransientConfig::new(5.0 * tau, tau / 100.0).with_fixed_step(),
    )
    .unwrap();
    let tran = result.transient.unwrap();
    for s in tran.timesteps.iter().filter(|s| s.time > 0.0) {
        let want = 5.0 * (1.0 - (-s.time / tau).exp());
        // Gear-2 at h = τ/100: a global error of order (h/τ)² of the swing,
        // plus the backward-Euler start step.
        let got = s.node_voltages["g"];
        assert!(
            (got - want).abs() < 5e-3,
            "V(g) = {got} at t = {}, want {want}",
            s.time
        );
    }
}

/// Started at its operating point, a gate held at 3 V stays there: Cgs's
/// history starts at its DC voltage, not 0 V.
#[test]
fn mosfet_gate_starts_settled_at_the_operating_point() {
    let config = TransientConfig::new(50e-6, 1e-6).with_start(TransientStart::OperatingPoint);
    let tran = solve_transient(&gate_rc(3.0), &config)
        .unwrap()
        .transient
        .unwrap();
    // A history started at 0 V would pull V(g) about 3 V·h/τ = 3 mV down on
    // the first 10 ns step. What is left is the 1e-12 S floor on the gate
    // node across RG: 3 V · 10 kΩ · 1e-12 S = 3e-8 V.
    for s in &tran.timesteps {
        let got = s.node_voltages["g"];
        assert!((got - 3.0).abs() < 1e-7, "V(g) = {got} at t = {}", s.time);
    }
}

/// A common-source JFET (IDSS = 10 mA, |Vp| = 2 V, RD = 1 kΩ) whose gate is
/// driven through 100 kΩ from `vg` volts, stepping by `step` volts at 1 µs
/// when `step` isn't zero. The P-channel part is the N-channel one mirrored.
fn jfet_gate_rc(kind: JfetKind, vg: f64, step: f64, caps: JfetParasiticCaps) -> Circuit {
    let sign = match kind {
        JfetKind::NChannel => 1.0,
        JfetKind::PChannel => -1.0,
    };
    let source = CircuitElement::VoltageSource {
        id: "VG".into(),
        nodes: ["in".into(), "0".into()],
        // The waveform rides on `voltage`, so that is 0 when there is one.
        voltage: if step == 0.0 { sign * vg } else { 0.0 },
        waveform: (step != 0.0).then_some(Waveform::Pulse {
            v1: sign * vg,
            v2: sign * (vg + step),
            delay: 1e-6,
            rise_time: 1e-9,
            fall_time: 1e-9,
            pulse_width: 1.0,
            period: 0.0,
        }),
    };
    Circuit::new(
        "0",
        vec![
            source,
            r("RG", "in", "g", 100e3),
            v("VDD", "vdd", sign * 12.0),
            r("RD", "vdd", "d", 1e3),
            CircuitElement::Jfet {
                id: "J1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind,
                idss: 10e-3,
                vp: -sign * 2.0,
                parasitic_caps: Some(caps),
            },
        ],
    )
}

/// A 10 mV gate step charges CGS and the Miller-multiplied CGD through RG:
/// τ = RG·(CGS + CGD·(1 + gm·RD)). With M = 0 the junctions are constant
/// 10 pF and 2 pF; gm = 2·IDSS/|Vp|·(1 − Vgs/Vp) ≈ 5.03 mS about −0.995 V,
/// so τ ≈ 100 kΩ · 22.1 pF = 2.21 µs, against 1 µs from CGS alone and
/// 1.2 µs without the Miller gain.
#[test]
fn jfet_gate_step_charges_cgs_and_the_miller_cgd() {
    let caps = JfetParasiticCaps {
        cgs: 10e-12,
        cgd: 2e-12,
        m: 0.0,
        ..Default::default()
    };
    let gm = 2.0 * 10e-3 / 2.0 * (1.0 - 0.995 / 2.0);
    let tau = 100e3 * (10e-12 + 2e-12 * (1.0 + gm * 1e3));
    for kind in [JfetKind::NChannel, JfetKind::PChannel] {
        let sign = if kind == JfetKind::NChannel {
            1.0
        } else {
            -1.0
        };
        let config = TransientConfig::new(1e-6 + 5.0 * tau, tau / 200.0)
            .with_start(TransientStart::OperatingPoint)
            .with_fixed_step();
        let tran = solve_transient(&jfet_gate_rc(kind, -1.0, 0.01, caps), &config)
            .unwrap()
            .transient
            .unwrap();
        // Where the gate first passes 1 − 1/e of the step, past the delay.
        let level = sign * (-1.0 + 0.01 * (1.0 - (-1.0f64).exp()));
        let crossed = tran
            .timesteps
            .windows(2)
            .find(|w| {
                w[0].time >= 1e-6
                    && (w[0].node_voltages["g"] - level) * (w[1].node_voltages["g"] - level) <= 0.0
            })
            .expect("the gate reaches 63 % of its step");
        let (t0, t1) = (crossed[0].time, crossed[1].time);
        let (v0, v1) = (crossed[0].node_voltages["g"], crossed[1].node_voltages["g"]);
        let t = t0 + (level - v0) / (v1 - v0) * (t1 - t0) - 1e-6;
        assert!(
            (t - tau).abs() < 0.01 * tau,
            "{kind:?}: the gate took {t} s to 63 %, want τ = {tau} s"
        );
    }
}

/// Started at its operating point, a reverse-biased gate with graded (M =
/// 0.5) junctions stays put: each junction's charge history starts at its
/// bias charge, not at zero.
#[test]
fn jfet_gate_junctions_start_settled_at_the_operating_point() {
    let caps = JfetParasiticCaps {
        cgs: 10e-12,
        cgd: 2e-12,
        pb: 0.8,
        ..Default::default()
    };
    for kind in [JfetKind::NChannel, JfetKind::PChannel] {
        let config = TransientConfig::new(10e-6, 0.1e-6).with_start(TransientStart::OperatingPoint);
        let tran = solve_transient(&jfet_gate_rc(kind, -1.0, 0.0, caps), &config)
            .unwrap()
            .transient
            .unwrap();
        let sign = if kind == JfetKind::NChannel {
            1.0
        } else {
            -1.0
        };
        for s in &tran.timesteps {
            let got = s.node_voltages["g"];
            assert!(
                (got + sign).abs() < 1e-6,
                "{kind:?}: V(g) = {got} at t = {}",
                s.time
            );
        }
    }
}

/// A relay coil with inductance, started at the operating point, carries
/// its DC current V/R_coil from the first step.
#[test]
fn relay_coil_starts_with_its_dc_current() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "vcc", 5.0),
            CircuitElement::Relay {
                id: "K1".into(),
                nodes: ["vcc".into(), "0".into(), "c1".into(), "c2".into()],
                coil_resistance: 500.0,
                pickup_voltage: 3.0,
                inductance: 0.1,
            },
            v("V2", "c1", 1.0),
            r("RC", "c2", "0", 1e3),
        ],
    );
    let config = TransientConfig::new(1e-3, 10e-6).with_start(TransientStart::OperatingPoint);
    let tran = solve_transient(&circuit, &config)
        .unwrap()
        .transient
        .unwrap();
    for s in &tran.timesteps {
        let coil = s
            .component_results
            .iter()
            .find(|c| c.id == "K1")
            .unwrap()
            .current_through;
        assert!(
            (coil - 10e-3).abs() < 1e-9,
            "coil {coil} A at t = {}",
            s.time
        );
    }
}

fn diode_rc() -> Circuit {
    Circuit::new(
        "0",
        vec![
            v("V1", "in", 5.0),
            r("R1", "in", "a", 1e3),
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model: None,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["a".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    )
}

/// With two Newton iterations a step the diode can't settle in is halved
/// and retried, not failed: the run finishes, counts the retries, and
/// ends at the same clamp as a run with the default budget.
#[test]
fn newton_failures_halve_the_step_and_recover() {
    let config = TransientConfig::new(1e-3, 1e-5);
    let mut tight = SolverOptions::default();
    tight.max_iterations = 2;
    let short = Solver::new(tight).transient(&diode_rc(), &config).unwrap();
    let full = solve_transient(&diode_rc(), &config).unwrap();
    let stats = short.transient.as_ref().unwrap().stats;
    assert!(stats.newton_rejections > 0, "{stats:?}");
    let end = |r: &sindr::SimulationResult| {
        r.transient
            .as_ref()
            .unwrap()
            .timesteps
            .last()
            .unwrap()
            .node_voltages["a"]
    };
    // Both are converged to vntol + reltol·|V| ≈ 0.7 mV at the end.
    assert!(
        (end(&short) - end(&full)).abs() < 2e-3,
        "{} vs {}",
        end(&short),
        end(&full)
    );
}

/// With one iteration no step converges: halving runs into the minimum
/// step and the error names a transient time.
#[test]
fn newton_failure_below_the_minimum_step_is_reported_at_its_time() {
    let mut one = SolverOptions::default();
    one.max_iterations = 1;
    let err = Solver::new(one)
        .transient(&diode_rc(), &TransientConfig::new(1e-3, 1e-5))
        .unwrap_err();
    match &err {
        SimError::ConvergenceFailed {
            iterations,
            at: SolvePoint::Transient { time },
            ..
        } => {
            assert_eq!(*iterations, 1);
            assert!(*time > 0.0 && *time < 1e-5, "first step, t = {time}");
        }
        other => panic!("expected a transient ConvergenceFailed, got {other:?}"),
    }
    assert!(err.to_string().contains("transient"), "{err}");
}

fn relay(coil_volts: f64) -> Circuit {
    Circuit::new(
        "0",
        vec![
            v("V1", "vcc", coil_volts),
            CircuitElement::Relay {
                id: "K1".into(),
                nodes: ["vcc".into(), "0".into(), "c1".into(), "c2".into()],
                coil_resistance: 500.0,
                pickup_voltage: 3.0,
                inductance: 0.1,
            },
            v("V2", "c1", 1.0),
            r("RC", "c2", "0", 1e3),
        ],
    )
}

/// The contacts follow the coil: closed (0.01 Ω) at 5 V, open (1 GΩ) at
/// 2 V, under the 3 V pickup, as 1 V across the contacts and 1 kΩ shows.
#[test]
fn relay_contacts_follow_the_coil_in_a_transient() {
    let config = TransientConfig::new(1e-3, 10e-6).with_start(TransientStart::OperatingPoint);
    for (coil, want) in [(5.0, 1e3 / (1e3 + 0.01)), (2.0, 1e3 / (1e3 + 1e9))] {
        let result = solve_transient(&relay(coil), &config).unwrap();
        let tran = result.transient.unwrap();
        let last = tran.timesteps.last().unwrap();
        let got = last.node_voltages["c2"];
        assert!(
            (got - want).abs() <= 1e-9 * want.max(1e-6),
            "{coil} V coil: V(c2) = {got}, want {want}"
        );
    }
}

/// A coil current given at the operating point overrides its DC value and
/// relaxes to it through τ = L/R = 200 µs: 10 mA − 6 mA·e^(−t/τ).
#[test]
fn relay_coil_starts_at_a_given_current() {
    let tau = 0.1 / 500.0;
    let config = TransientConfig::new(5.0 * tau, tau / 100.0)
        .with_start(TransientStart::OperatingPoint)
        .with_initial_current("K1", 4e-3);
    let tran = solve_transient(&relay(5.0), &config)
        .unwrap()
        .transient
        .unwrap();
    for s in &tran.timesteps {
        let coil = s
            .component_results
            .iter()
            .find(|c| c.id == "K1")
            .unwrap()
            .current_through;
        let want = 10e-3 - 6e-3 * (-s.time / tau).exp();
        // Gear-2 at h = τ/100: well under 1 % of the 6 mA swing.
        assert!(
            (coil - want).abs() < 2e-5,
            "coil {coil} A at t = {}, want {want}",
            s.time
        );
    }
}

/// A photodiode between two live nodes: a transient with nothing to
/// charge sits at the DC operating point every step.
#[test]
fn photodiode_between_live_nodes_matches_dc() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "k", 5.0),
            v("V2", "m", 1.0),
            CircuitElement::Photodiode {
                id: "PD1".into(),
                nodes: ["a".into(), "k".into()],
                irradiance: 1.0,
                temperature: 300.15,
            },
            r("R1", "a", "m", 10e3),
        ],
    );
    let dc = sindr::solve_op(&circuit).unwrap().node_voltages["a"];
    // The photocurrent leaves the anode through R1, lifting it above 1 V.
    assert!(dc > 1.0, "V(a) = {dc}");
    let tran = solve_transient(&circuit, &TransientConfig::new(1e-4, 1e-5))
        .unwrap()
        .transient
        .unwrap();
    for s in &tran.timesteps {
        let got = s.node_voltages["a"];
        // Each is converged to vntol + reltol·|V| of the same root.
        assert!(
            (got - dc).abs() < 1e-5,
            "V(a) = {got} at t = {}, DC {dc}",
            s.time
        );
    }
}
