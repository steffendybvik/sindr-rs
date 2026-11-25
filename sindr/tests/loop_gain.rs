//! Loop gain and stability margins against closed-form op-amp loops.

use sindr::element::{Capacitor, OpAmp, Resistor, Vcvs, VoltageSource};
use std::f64::consts::PI;

use num_complex::Complex64;
use sindr::loop_gain::{solve_loop_gain, LoopGainConfig};
use sindr::{Circuit, CircuitElement, OpAmpModel, SimError};

const A0: f64 = 1e5;
const GBW: f64 = 1e6;

fn opamp(r_out: f64) -> CircuitElement {
    CircuitElement::OpAmp(OpAmp {
        id: "U1".into(),
        nodes: ["0".into(), "fb".into(), "out".into()],
        v_pos: 15.0,
        v_neg: -15.0,
        model: Some(OpAmpModel {
            gain: A0,
            gbw: Some(GBW),
            r_out,
            ..Default::default()
        }),
    })
}

fn probe(driven: &str, driving: &str) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: "Vprobe".into(),
        nodes: [driven.into(), driving.into()],
        voltage: 0.0,
        waveform: None,
    })
}

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn c(id: &str, a: &str, b: &str, farads: f64) -> CircuitElement {
    CircuitElement::Capacitor(Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance: farads,
    })
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
        ..Default::default()
    }
}

/// The open-loop pole's time constant, as the macromodel sets it.
fn tau() -> f64 {
    A0 / (2.0 * PI * GBW)
}

/// A root of `h` in `[lo, hi]` by bisection on a log scale.
fn root(mut lo: f64, mut hi: f64, h: impl Fn(f64) -> f64) -> f64 {
    let h_lo = h(lo);
    for _ in 0..200 {
        let mid = (lo * hi).sqrt();
        if (h(mid) > 0.0) == (h_lo > 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (lo * hi).sqrt()
}

#[test]
fn follower_phase_margin_matches_the_single_pole_model() {
    // Broken at the output, T = A0 / (1 + sτ): |T| = 1 at
    // f = GBW·√(1 − 1/A0²), where the phase margin is 180° − atan(√(A0² − 1)).
    let parts = vec![opamp(0.0), probe("fb", "out"), r("RL", "out", "0", 10e3)];
    let lg = solve_loop_gain(
        &circuit(parts),
        &LoopGainConfig::new("Vprobe", 1.0, 100e6, 161),
    )
    .unwrap();

    for p in &lg.points {
        let want = A0 / Complex64::new(1.0, 2.0 * PI * p.frequency * tau());
        assert!(
            (p.gain - want).norm() <= 1e-10 * want.norm(),
            "{} Hz: {} vs {want}",
            p.frequency,
            p.gain
        );
    }
    assert_eq!(lg.gain_crossovers.len(), 1);
    let fc = GBW * (1.0 - 1.0 / (A0 * A0)).sqrt();
    assert!(((lg.gain_crossovers[0] - fc) / fc).abs() < 1e-10);
    let pm = 180.0 - (A0 * A0 - 1.0).sqrt().atan().to_degrees();
    let got = lg.phase_margin_deg.unwrap();
    assert!((got - pm).abs() < 1e-8, "{got} vs {pm}");
    assert!(lg.phase_crossovers.is_empty() && lg.gain_margin_db.is_none());
}

#[test]
fn three_pole_loop_margins_through_a_loaded_output() {
    // A follower whose feedback runs through two RC sections, from an
    // output with 100 Ω of its own. The probe sits after the output
    // resistance, so the loop carries signal both ways through it and a
    // plain voltage injection would misread it; Tian's T is the exact
    // A(s)·H(s) with the output resistance in series with R1.
    let (r_out, r1, c1, r2, c2) = (100.0, 1e3, 100e-12, 1e3, 100e-12);
    let parts = vec![
        opamp(r_out),
        probe("x", "out"),
        r("R1", "x", "a", r1),
        c("C1", "a", "0", c1),
        r("R2", "a", "fb", r2),
        c("C2", "fb", "0", c2),
    ];
    let lg = solve_loop_gain(
        &circuit(parts),
        &LoopGainConfig::new("Vprobe", 10.0, 1e9, 321),
    )
    .unwrap();

    let r1 = r1 + r_out;
    let (a2, a1) = (r1 * c1 * r2 * c2, r1 * c1 + r2 * c2 + r1 * c2);
    let t = |f: f64| {
        let w = 2.0 * PI * f;
        A0 / Complex64::new(1.0, w * tau()) / Complex64::new(1.0 - w * w * a2, w * a1)
    };
    // The ladder's phase runs continuously from 0 to −180°.
    let phase = |f: f64| {
        let w = 2.0 * PI * f;
        -(w * tau()).atan().to_degrees() - (w * a1).atan2(1.0 - w * w * a2).to_degrees()
    };
    for p in &lg.points {
        let want = t(p.frequency);
        // T comes out of the difference of two responses, so far below
        // |T| = 1 its error is absolute, about 1e-15.
        assert!(
            (p.gain - want).norm() <= 1e-9 * want.norm() + 1e-14,
            "{} Hz: {} vs {want}",
            p.frequency,
            p.gain
        );
        let phase_tol = 1e-7 + (1e-14 / want.norm()).to_degrees();
        assert!(
            (p.phase_deg - phase(p.frequency)).abs() < phase_tol,
            "{} Hz: {}° vs {}°",
            p.frequency,
            p.phase_deg,
            phase(p.frequency)
        );
    }

    let fc = root(1e3, 1e8, |f| t(f).norm().ln());
    let fp = root(1e3, 1e9, |f| phase(f) + 180.0);
    assert_eq!(lg.gain_crossovers.len(), 1);
    assert_eq!(lg.phase_crossovers.len(), 1);
    assert!(((lg.gain_crossovers[0] - fc) / fc).abs() < 1e-9);
    assert!(((lg.phase_crossovers[0] - fp) / fp).abs() < 1e-9);

    let pm = 180.0 + phase(fc);
    let gm = -20.0 * t(fp).norm().log10();
    let (got_pm, got_gm) = (lg.phase_margin_deg.unwrap(), lg.gain_margin_db.unwrap());
    assert!((got_pm - pm).abs() < 1e-7, "{got_pm} vs {pm}");
    assert!((got_gm - gm).abs() < 1e-7, "{got_gm} vs {gm}");
    // Stable, with margins in the range this loop is designed for.
    assert!(pm > 10.0 && pm < 60.0, "{pm}");
    assert!(gm > 0.0, "{gm}");
}

/// A follower whose feedback runs through eight buffered RC sections at
/// 1 kHz: nine poles, so at the gain crossover (about 2 kHz) the phase is
/// past −540°. The margin is that phase plus 180° wrapped into one turn,
/// about −50° (unstable), not the −410° it read before it was wrapped.
#[test]
fn phase_margin_past_minus_540_degrees_wraps_into_one_turn() {
    const SECTIONS: usize = 8;
    let rc = 1.0 / (2.0 * PI * 1e3);
    let mut parts = vec![opamp(0.0), probe("x", "out")];
    let mut from = "x".to_string();
    for k in 1..=SECTIONS {
        let (a, b) = (format!("a{k}"), format!("b{k}"));
        let to = if k == SECTIONS { "fb".to_string() } else { b };
        parts.push(r(&format!("R{k}"), &from, &a, 1e3));
        parts.push(c(&format!("C{k}"), &a, "0", rc / 1e3));
        parts.push(CircuitElement::Vcvs(Vcvs {
            id: format!("E{k}"),
            nodes: [to.as_str().into(), "0".into()],
            control_nodes: [a.as_str().into(), "0".into()],
            gain: 1.0,
        }));
        from = to;
    }
    let lg = solve_loop_gain(
        &circuit(parts),
        &LoopGainConfig::new("Vprobe", 1.0, 1e5, 401),
    )
    .unwrap();

    let t = |f: f64| {
        let w = 2.0 * PI * f;
        A0 / Complex64::new(1.0, w * tau()) / Complex64::new(1.0, w * rc).powi(SECTIONS as i32)
    };
    let phase = |f: f64| {
        let w = 2.0 * PI * f;
        -(w * tau()).atan().to_degrees() - SECTIONS as f64 * (w * rc).atan().to_degrees()
    };
    let fc = root(1e2, 1e5, |f| t(f).norm().ln());
    assert_eq!(lg.gain_crossovers.len(), 1);
    assert!(((lg.gain_crossovers[0] - fc) / fc).abs() < 1e-9);
    assert!(phase(fc) < -540.0, "{}", phase(fc));

    let pm = 180.0 + phase(fc) + 360.0;
    assert!(pm > -180.0 && pm < 0.0, "{pm}");
    let got = lg.phase_margin_deg.unwrap();
    assert!((got - pm).abs() < 1e-6, "{got} vs {pm}");
    // The phase passes −180° and −540° below the crossover, where |T| > 1.
    assert_eq!(lg.phase_crossovers.len(), 2);
    assert!(lg.gain_margin_db.unwrap() < 0.0);
}

/// An AC-coupled loop crosses unity gain twice: low, with a phase lead
/// of nearly 90°, and high, with a lag of nearly 90°. Each is about 90°
/// from instability, so the margin is that, not the −90° that adding 180°
/// to the lead and wrapping it gives.
#[test]
fn a_band_pass_loop_has_its_margin_at_both_crossovers() {
    // High-pass at 1 Hz, gain −1000, low-pass at 1 kHz.
    let (r1, c1) = (1e3, 1.0 / (2.0 * PI * 1e3));
    let (r2, c2) = (1e3, 1.0 / (2.0 * PI * 1e6));
    let parts = vec![
        probe("fb", "drv"),
        c("C1", "fb", "hp", c1),
        r("R1", "hp", "0", r1),
        CircuitElement::Vcvs(Vcvs {
            id: "E1".into(),
            nodes: ["x".into(), "0".into()],
            control_nodes: ["hp".into(), "0".into()],
            gain: -1000.0,
        }),
        r("R2", "x", "lp", r2),
        c("C2", "lp", "0", c2),
        CircuitElement::Vcvs(Vcvs {
            id: "E2".into(),
            nodes: ["drv".into(), "0".into()],
            control_nodes: ["lp".into(), "0".into()],
            gain: 1.0,
        }),
    ];
    let lg = solve_loop_gain(
        &circuit(parts),
        &LoopGainConfig::new("Vprobe", 1e-5, 1e8, 1301),
    )
    .unwrap();
    assert_eq!(lg.gain_crossovers.len(), 2, "{:?}", lg.gain_crossovers);
    // T = 1000·(s·τ1)/(1 + s·τ1)/(1 + s·τ2); its phase at each crossover.
    let phase = |f: f64| {
        let (w1, w2) = (2.0 * PI * f * r1 * c1, 2.0 * PI * f * r2 * c2);
        90.0 - w1.atan().to_degrees() - w2.atan().to_degrees()
    };
    let margins: Vec<f64> = lg
        .gain_crossovers
        .iter()
        .map(|&f| 180.0 - phase(f).abs())
        .collect();
    let want = margins.iter().copied().fold(f64::INFINITY, f64::min);
    assert!((want - 90.06).abs() < 0.01, "{want}");
    let got = lg.phase_margin_deg.unwrap();
    assert!((got - want).abs() < 1e-6, "{got} vs {want}");
}

#[test]
fn probe_must_be_a_floating_voltage_source() {
    let parts = vec![opamp(0.0), probe("fb", "out"), r("RL", "out", "0", 10e3)];
    let config = LoopGainConfig::new("RL", 1.0, 1e6, 11);
    assert!(matches!(
        solve_loop_gain(&circuit(parts.clone()), &config),
        Err(SimError::InvalidConfig {
            config: sindr::ConfigKind::LoopGain,
            ..
        })
    ));

    let mut grounded = parts;
    grounded.push(CircuitElement::VoltageSource(VoltageSource {
        id: "Vg".into(),
        nodes: ["out".into(), "0".into()],
        voltage: 0.0,
        waveform: None,
    }));
    let config = LoopGainConfig::new("Vg", 1.0, 1e6, 11);
    assert!(matches!(
        solve_loop_gain(&circuit(grounded), &config),
        Err(SimError::InvalidConfig {
            config: sindr::ConfigKind::LoopGain,
            ..
        })
    ));
}

#[test]
fn linear_spacing_spaces_the_points_evenly_and_gain_db_reads_the_magnitude() {
    let parts = vec![opamp(0.0), probe("fb", "out"), r("RL", "out", "0", 10e3)];
    let config =
        LoopGainConfig::new("Vprobe", 1e3, 11e3, 11).with_spacing(sindr::FrequencySpacing::Linear);
    let lg = solve_loop_gain(&circuit(parts), &config).unwrap();

    let f: Vec<f64> = lg.points.iter().map(|p| p.frequency).collect();
    assert_eq!(f.len(), 11);
    for (i, fi) in f.iter().enumerate() {
        let want = 1e3 + 1e3 * i as f64;
        assert!((fi - want).abs() < 1e-9 * want, "point {i}: {fi} Hz");
    }
    for p in &lg.points {
        let want = A0 / Complex64::new(1.0, 2.0 * PI * p.frequency * tau());
        assert!((p.gain_db() - 20.0 * want.norm().log10()).abs() < 1e-9);
    }
}

#[test]
fn probe_with_its_driven_side_on_ground_is_an_error() {
    // The probe's first node is the side the loop is driven into; on
    // ground there is nothing to inject into.
    let parts = vec![opamp(0.0), probe("0", "out"), r("RL", "out", "fb", 10e3)];
    let err =
        solve_loop_gain(&circuit(parts), &LoopGainConfig::new("Vprobe", 1.0, 1e3, 3)).unwrap_err();
    assert!(
        matches!(&err, SimError::InvalidConfig { config: sindr::ConfigKind::LoopGain, reason } if reason.contains("Vprobe")),
        "{err:?}"
    );
}
