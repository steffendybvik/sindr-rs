//! Noise analysis beyond the ngspice goldens: flicker noise, the per-source
//! breakdown, and input-referred noise for a current input.

use sindr::ac_analysis::FrequencySpacing;
use sindr::noise::{solve_noise, NoiseConfig, NoiseKind, NoiseResult};
use sindr::{Circuit, CircuitElement, DiodeModel, SimError};

fn config(output: &str, input: &str, f: (f64, f64), points: usize) -> NoiseConfig {
    NoiseConfig {
        output: output.into(),
        reference: None,
        input_source: input.into(),
        f_start: f.0,
        f_stop: f.1,
        num_points: points,
        spacing: FrequencySpacing::Logarithmic,
    }
}

/// 5 V through 10 kΩ into a diode whose card has flicker noise.
fn biased_diode(kf: f64) -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "a".into()],
                resistance: 10e3,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model: Some(DiodeModel {
                    kf,
                    af: 1.0,
                    ..DiodeModel::default()
                }),
            },
        ],
    )
}

fn share(result: &NoiseResult, point: usize, component: &str, kind: NoiseKind) -> f64 {
    let k = result
        .sources
        .iter()
        .position(|s| s.component == component && s.kind == kind)
        .unwrap();
    result.points[point].contributions[k]
}

#[test]
fn flicker_noise_falls_as_one_over_f_and_meets_the_white_floor() {
    let noise = solve_noise(&biased_diode(1e-12), &config("a", "V1", (1.0, 1e6), 7)).unwrap();
    // A decade apart, flicker drops 10×, shot noise not at all.
    let f0 = share(&noise, 0, "D1", NoiseKind::Flicker);
    let f1 = share(&noise, 1, "D1", NoiseKind::Flicker);
    assert!((f0 / f1 - 10.0).abs() < 1e-9, "{}", f0 / f1);
    let s0 = share(&noise, 0, "D1", NoiseKind::Shot);
    let s1 = share(&noise, 1, "D1", NoiseKind::Shot);
    assert!((s0 / s1 - 1.0).abs() < 1e-12);
    // The breakdown adds up to the total.
    for p in &noise.points {
        let sum: f64 = p.contributions.iter().sum();
        assert!((sum / p.output_psd - 1.0).abs() < 1e-12);
    }
    // Without KF there's no flicker source at all.
    let quiet = solve_noise(&biased_diode(0.0), &config("a", "V1", (1.0, 1e6), 7)).unwrap();
    assert!(quiet.sources.iter().all(|s| s.kind != NoiseKind::Flicker));
    assert!(quiet.points[0].output_psd < noise.points[0].output_psd);
}

#[test]
fn current_input_refers_noise_in_amps() {
    // A current source into 1 kΩ: the gain is 1 kΩ, so the input-referred
    // noise is 4kT/R in A²/Hz.
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "out".into()],
                current: 0.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["out".into(), "0".into()],
                resistance: 1e3,
            },
        ],
    );
    let noise = solve_noise(&circuit, &config("out", "I1", (10.0, 1e3), 3)).unwrap();
    let p = &noise.points[0];
    let four_kt_over_r = 4.0 * 1.380_649e-23 * 300.15 / 1e3;
    assert!((p.gain - 1e3).abs() < 1e-9);
    assert!((p.input_psd / four_kt_over_r - 1.0).abs() < 1e-9);
}

#[test]
fn bad_configs_name_the_problem() {
    let c = biased_diode(0.0);
    let err = |cfg: NoiseConfig| match solve_noise(&c, &cfg) {
        Err(SimError::InvalidComponent(msg)) => msg,
        other => panic!("expected InvalidComponent, got {other:?}"),
    };
    assert!(err(config("nowhere", "V1", (1.0, 10.0), 2)).contains("nowhere"));
    assert!(err(config("a", "R1", (1.0, 10.0), 2)).contains("R1"));
    let mut dc = config("a", "V1", (1.0, 10.0), 2);
    dc.f_start = 0.0;
    dc.spacing = FrequencySpacing::Linear;
    assert!(err(dc).contains("f_start"));
}

/// A MOSFET's flicker parameters are checked like a diode's or a BJT's: a
/// negative KF would make a negative noise power.
#[test]
fn mosfet_flicker_parameters_are_validated() {
    use sindr_devices::mosfet::{MosfetKind, MosfetParams};
    let with = |kf: f64, af: f64| {
        Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["d".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Mosfet {
                    id: "M1".into(),
                    nodes: ["d".into(), "d".into(), "0".into()],
                    kind: MosfetKind::Nmos,
                    params: MosfetParams {
                        kf,
                        af,
                        ..MosfetParams::default_nmos()
                    },
                    parasitic_caps: None,
                    body_diode: None,
                },
            ],
        )
    };
    assert!(sindr::solve_circuit(&with(1e-12, 1.0)).is_ok());
    for (kf, af, field) in [(-1e-12, 1.0, "params.kf"), (1e-12, 0.0, "params.af")] {
        let err = sindr::solve_circuit(&with(kf, af)).unwrap_err().to_string();
        assert!(err.contains(field), "{err}");
    }
}
