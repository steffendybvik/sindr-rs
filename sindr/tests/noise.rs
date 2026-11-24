//! Noise analysis beyond the ngspice goldens: flicker noise, the per-source
//! breakdown, and input-referred noise for a current input.

use sindr::ac_analysis::FrequencySpacing;
use sindr::element::{
    Bjt, CurrentSource, Diode, Jfet, Led, Mosfet, Photodiode, Photoresistor, Potentiometer,
    Resistor, SchottkyDiode, Thermistor, VoltageSource, ZenerDiode,
};
use sindr::noise::{solve_noise, NoiseConfig, NoiseKind, NoiseResult};
use sindr::{
    solve_op, BjtKind, BjtModel, Circuit, CircuitElement, DiodeModel, JfetKind, MosfetKind,
    SimError,
};
use sindr_devices::mosfet::MosfetParams;

const K: f64 = 1.380_649e-23;
const Q: f64 = 1.602_176_634e-19;
const T_NOM: f64 = 300.15;

fn vsrc(id: &str, p: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), "0".into()],
        voltage: volts,
        waveform: None,
    })
}

fn res(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

/// `I1` pushing `amps` from ground into `node`.
fn isrc(node: &str, amps: f64) -> CircuitElement {
    CircuitElement::CurrentSource(CurrentSource {
        id: "I1".into(),
        nodes: ["0".into(), node.into()],
        current: amps,
        waveform: None,
    })
}

/// `x` within `rel` of `want`.
fn close(x: f64, want: f64, rel: f64) -> bool {
    (x / want - 1.0).abs() < rel
}

fn config(output: &str, input: &str, f: (f64, f64), points: usize) -> NoiseConfig {
    NoiseConfig::new(output, input, f.0, f.1, points)
}

/// 5 V through 10 kΩ into a diode whose card has flicker noise.
fn biased_diode(kf: f64) -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "a".into()],
                resistance: 10e3,
            }),
            CircuitElement::Diode(Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model: Some(DiodeModel {
                    kf,
                    af: 1.0,
                    ..DiodeModel::default()
                }),
            }),
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
            CircuitElement::CurrentSource(CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "out".into()],
                current: 0.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["out".into(), "0".into()],
                resistance: 1e3,
            }),
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

/// A missing reference node is named as the reference, not the output.
#[test]
fn a_missing_reference_is_named_as_the_reference() {
    let mut cfg = config("a", "V1", (1.0, 10.0), 2);
    cfg.reference = Some("nowhere".into());
    match solve_noise(&biased_diode(0.0), &cfg) {
        Err(SimError::InvalidComponent(msg)) => {
            assert!(msg.contains("reference node `nowhere`"), "{msg}");
            assert!(!msg.contains("output"), "{msg}");
        }
        other => panic!("expected InvalidComponent, got {other:?}"),
    }
}

/// V(out, ref) against ground named outright is V(out), and swapping the
/// two only flips the sign, which neither the PSD nor |gain| sees. Against
/// the input node no noise reaches the reference (the source holds it), but
/// the signal does: the gain is |V(a) − 1| = 1 − gain (the circuit is
/// resistive, so V(a) is real and below 1). Measured against itself the
/// output is silent and nothing reaches it from the input, so the
/// input-referred noise is infinite.
#[test]
fn the_reference_node_is_subtracted() {
    let c = biased_diode(1e-12);
    let run = |output: &str, reference: Option<&str>| {
        let mut cfg = config(output, "V1", (1.0, 1e3), 4);
        cfg.reference = reference.map(Into::into);
        solve_noise(&c, &cfg).unwrap()
    };
    let plain = run("a", None);
    for other in [run("a", Some("0")), run("0", Some("a"))] {
        for (p, q) in plain.points.iter().zip(&other.points) {
            assert!(close(q.output_psd, p.output_psd, 1e-12));
            assert!(close(q.gain, p.gain, 1e-12));
        }
    }
    let against_input = run("a", Some("in"));
    for (p, q) in plain.points.iter().zip(&against_input.points) {
        assert!(close(q.output_psd, p.output_psd, 1e-12));
        assert!(
            close(q.gain, 1.0 - p.gain, 1e-12),
            "{} vs {}",
            q.gain,
            p.gain
        );
    }
    let silent = run("a", Some("a"));
    for p in &silent.points {
        assert_eq!((p.output_psd, p.gain), (0.0, 0.0));
        assert_eq!(p.input_psd, f64::INFINITY);
    }
    assert_eq!(silent.total_output_noise, 0.0);
    assert_eq!(silent.total_input_noise, f64::INFINITY);
}

/// A potentiometer's halves are two resistors meeting at the wiper: with
/// the ends at AC ground and a load on the wiper, each `4kT/R` sees
/// `R_top ‖ R_bottom ‖ R_load`, and all three make `4kT` times that.
#[test]
fn potentiometer_halves_are_two_thermal_sources() {
    let circuit = Circuit::new(
        "0",
        vec![
            vsrc("V1", "in", 0.0),
            CircuitElement::Potentiometer(Potentiometer {
                id: "P1".into(),
                nodes: ["in".into(), "w".into(), "0".into()],
                resistance: 10e3,
                position: 0.3,
            }),
            res("RL", "w", "0", 10e3),
        ],
    );
    let noise = solve_noise(&circuit, &config("w", "V1", (10.0, 1e3), 3)).unwrap();
    let branches: Vec<_> = noise.sources.iter().map(|s| s.branch.as_str()).collect();
    assert_eq!(branches, ["top", "bottom", ""]);
    let (top, bottom, load) = (3e3, 7e3, 10e3);
    let par = 1.0 / (1.0 / top + 1.0 / bottom + 1.0 / load);
    let p = &noise.points[0];
    assert!(close(
        p.contributions[0],
        4.0 * K * T_NOM / top * par * par,
        1e-9
    ));
    assert!(close(
        p.contributions[1],
        4.0 * K * T_NOM / bottom * par * par,
        1e-9
    ));
    assert!(close(p.output_psd, 4.0 * K * T_NOM * par, 1e-9));
}

/// A thermistor and a photoresistor are resistors for noise: alone on a
/// current source their noise is `4kT·R`, R read back from the operating
/// point. The thermistor's T is its own temperature, the LDR's 27 °C.
#[test]
fn thermistor_and_photoresistor_noise_is_4ktr() {
    let cases = [
        (
            CircuitElement::Thermistor(Thermistor {
                id: "TH1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 330.0,
            }),
            330.0,
        ),
        (
            CircuitElement::Photoresistor(Photoresistor {
                id: "LDR1".into(),
                nodes: ["a".into(), "0".into()],
                light_level: 0.4,
            }),
            T_NOM,
        ),
    ];
    for (device, t) in cases {
        let circuit = Circuit::new("0", vec![isrc("a", 1e-3), device]);
        let r = solve_op(&circuit).unwrap().node_voltages["a"] / 1e-3;
        let noise = solve_noise(&circuit, &config("a", "I1", (10.0, 1e3), 3)).unwrap();
        assert_eq!(noise.sources[0].kind, NoiseKind::Thermal);
        let p = &noise.points[0];
        assert!(close(p.output_psd, 4.0 * K * t * r, 1e-9), "{t} K, {r} Ω");
        // Referred back to the current source: 4kT/R.
        assert!(close(p.input_psd, 4.0 * K * t / r, 1e-9));
    }
}

/// Every junction carries `2q·I` of shot noise. Driven by 1 mA from a
/// current source, the transfer from its own terminals to the output is
/// the analysis' gain, so its share is `2q·1 mA·gain²`. A Zener is solved
/// as its diode card, so it lands in the diode's arm.
#[test]
fn junction_shot_noise_is_2qi() {
    let nodes = || -> [sindr::NodeId; 2] { ["a".into(), "0".into()] };
    let devices = [
        CircuitElement::Led(Led {
            id: "D1".into(),
            nodes: nodes(),
            color: "red".into(),
            temperature: T_NOM,
        }),
        CircuitElement::SchottkyDiode(SchottkyDiode {
            id: "D1".into(),
            nodes: nodes(),
            temperature: T_NOM,
        }),
        CircuitElement::Photodiode(Photodiode {
            id: "D1".into(),
            nodes: nodes(),
            irradiance: 0.0,
            temperature: T_NOM,
        }),
        CircuitElement::ZenerDiode(ZenerDiode {
            id: "D1".into(),
            nodes: nodes(),
            vz: 5.1,
            temperature: T_NOM,
        }),
    ];
    for device in devices {
        let name = format!("{device:?}");
        let circuit = Circuit::new("0", vec![isrc("a", 1e-3), device]);
        let noise = solve_noise(&circuit, &config("a", "I1", (10.0, 1e3), 3)).unwrap();
        for p in &noise.points {
            let shot = share(&noise, 0, "D1", NoiseKind::Shot);
            assert!(
                close(shot, 2.0 * Q * 1e-3 * p.gain * p.gain, 1e-6),
                "{name}"
            );
        }
    }
}

/// A common-emitter stage whose card has flicker noise: it sits on the
/// base current, between the same nodes as the base shot noise, so the two
/// shares stand in the ratio of their PSDs, `KF·Ib^AF/f : 2q·Ib`.
#[test]
fn bjt_flicker_rides_on_the_base_current() {
    let (kf, af) = (1e-12, 1.2);
    let circuit = Circuit::new(
        "0",
        vec![
            vsrc("VCC", "vcc", 5.0),
            vsrc("VIN", "in", 5.0),
            res("RB", "in", "b", 430e3),
            res("RC", "vcc", "c", 1e3),
            CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: T_NOM,
                parasitic_caps: None,
                model: Some(BjtModel {
                    kf,
                    af,
                    ..BjtModel::default()
                }),
            }),
        ],
    );
    let ib = solve_op(&circuit).unwrap().bjt_results[0].ib;
    let noise = solve_noise(&circuit, &config("c", "VIN", (1.0, 1e4), 5)).unwrap();
    let kinds: Vec<_> = noise
        .sources
        .iter()
        .filter(|s| s.component == "Q1")
        .map(|s| (s.kind, s.branch.as_str()))
        .collect();
    assert_eq!(
        kinds,
        [
            (NoiseKind::Shot, "ic"),
            (NoiseKind::Shot, "ib"),
            (NoiseKind::Flicker, "ib")
        ]
    );
    for (i, p) in noise.points.iter().enumerate() {
        let shot_ib = p.contributions[noise.sources.iter().position(|s| s.branch == "ib").unwrap()];
        let flicker = share(&noise, i, "Q1", NoiseKind::Flicker);
        let want = kf * ib.abs().powf(af) / p.frequency / (2.0 * Q * ib.abs());
        assert!(close(flicker / shot_ib, want, 1e-9), "{} Hz", p.frequency);
    }
}

/// A FET's channel noise `(8/3)·kT·gm` and the drain resistor's `4kT/RD`
/// see the same impedance at the drain, so their shares stand at
/// `(2/3)·gm·RD`, and with no output conductance the stage's gain is
/// `gm·RD`: the ratio is `(2/3)·gain`.
fn assert_channel_noise(name: &str, circuit: &Circuit) -> NoiseResult {
    let noise = solve_noise(circuit, &config("d", "VIN", (10.0, 1e3), 3)).unwrap();
    for (i, p) in noise.points.iter().enumerate() {
        let channel = share(&noise, i, "M1", NoiseKind::Thermal);
        let rd = share(&noise, i, "RD", NoiseKind::Thermal);
        assert!(
            close(channel / rd, 2.0 / 3.0 * p.gain, 1e-6),
            "{name}: {} vs {}",
            channel / rd,
            2.0 / 3.0 * p.gain
        );
    }
    noise
}

#[test]
fn mosfet_channel_and_flicker_noise() {
    let rd = 10e3;
    let (kf, af) = (1e-20, 1.0);
    let stage = |kf: f64| {
        Circuit::new(
            "0",
            vec![
                vsrc("VDD", "vdd", 10.0),
                vsrc("VIN", "g", 1.2),
                res("RD", "vdd", "d", rd),
                CircuitElement::Mosfet(Mosfet {
                    id: "M1".into(),
                    nodes: ["g".into(), "d".into(), "0".into()],
                    kind: MosfetKind::Nmos,
                    params: MosfetParams {
                        lambda: 0.0,
                        kf,
                        af,
                        ..MosfetParams::default_nmos()
                    },
                    parasitic_caps: None,
                    body_diode: None,
                }),
            ],
        )
    };
    let quiet = assert_channel_noise("NMOS", &stage(0.0));
    assert!(quiet.sources.iter().all(|s| s.kind != NoiseKind::Flicker));

    let circuit = stage(kf);
    let op = solve_op(&circuit).unwrap();
    let id = (10.0 - op.node_voltages["d"]) / rd;
    let noise = assert_channel_noise("NMOS with KF", &circuit);
    for (i, p) in noise.points.iter().enumerate() {
        let gm = p.gain / rd;
        let flicker = share(&noise, i, "M1", NoiseKind::Flicker);
        let channel = share(&noise, i, "M1", NoiseKind::Thermal);
        let want = kf * id.powf(af) / p.frequency / (8.0 / 3.0 * K * T_NOM * gm);
        assert!(close(flicker / channel, want, 1e-6), "{} Hz", p.frequency);
    }
}

#[test]
fn jfet_channel_noise() {
    let circuit = Circuit::new(
        "0",
        vec![
            vsrc("VDD", "vdd", 12.0),
            vsrc("VIN", "g", -0.5),
            res("RD", "vdd", "d", 1e3),
            CircuitElement::Jfet(Jfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind: JfetKind::NChannel,
                idss: 10e-3,
                vp: -2.0,
                parasitic_caps: None,
            }),
        ],
    );
    let noise = assert_channel_noise("JFET", &circuit);
    assert!(noise.points[0].gain > 1.0, "the stage amplifies");
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["d".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Mosfet(Mosfet {
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
                }),
            ],
        )
    };
    assert!(sindr::solve_circuit(&with(1e-12, 1.0)).is_ok());
    for (kf, af, field) in [(-1e-12, 1.0, "params.kf"), (1e-12, 0.0, "params.af")] {
        let err = sindr::solve_circuit(&with(kf, af)).unwrap_err().to_string();
        assert!(err.contains(field), "{err}");
    }
}

#[test]
fn config_defaults_and_setters() {
    let config = NoiseConfig::new("out", "V1", 10.0, 1e5, 41);
    assert_eq!(config.output, "out");
    assert_eq!(config.input_source, "V1");
    assert_eq!(
        (config.f_start, config.f_stop, config.num_points),
        (10.0, 1e5, 41)
    );
    assert_eq!(config.reference, None);
    assert_eq!(config.spacing, FrequencySpacing::Logarithmic);

    let config = config
        .with_reference("ref")
        .with_spacing(FrequencySpacing::Linear);
    assert_eq!(config.reference.as_deref(), Some("ref"));
    assert_eq!(config.spacing, FrequencySpacing::Linear);
}
