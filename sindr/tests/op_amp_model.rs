//! Op-amp macromodel (`OpAmpModel`): closed-loop bandwidth from GBW, slew
//! rate, rail clamp, output resistance, offset and bias current.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{
    solve_circuit, solve_transient, Circuit, CircuitElement, OpAmpModel, SimError, TransientConfig,
    Waveform,
};

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    }
}

fn source(voltage: f64, waveform: Option<Waveform>) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: "VIN".into(),
        nodes: ["in".into(), "0".into()],
        voltage,
        waveform,
    }
}

fn op_amp(plus: &str, minus: &str, model: OpAmpModel) -> CircuitElement {
    CircuitElement::OpAmp {
        id: "U1".into(),
        nodes: [plus.into(), minus.into(), "out".into()],
        v_pos: 15.0,
        v_neg: -15.0,
        model: Some(model),
    }
}

/// Non-inverting amplifier of gain 1 + rf/rg, with a 10 kΩ load.
fn non_inverting(vin: CircuitElement, rf: f64, rg: f64, model: OpAmpModel) -> Circuit {
    Circuit::new(
        "0",
        vec![
            vin,
            op_amp("in", "fb", model),
            r("RF", "out", "fb", rf),
            r("RG", "fb", "0", rg),
            r("RL", "out", "0", 10e3),
        ],
    )
}

fn out(circuit: &Circuit) -> f64 {
    solve_circuit(circuit).unwrap().node_voltages["out"]
}

#[test]
fn closed_loop_dc_gain_holds_with_output_resistance() {
    let model = OpAmpModel {
        r_out: 100.0,
        ..OpAmpModel::default()
    };
    let v = out(&non_inverting(source(0.1, None), 9e3, 1e3, model));
    // Loop gain 1e4 shrinks r_out to ~10 mΩ and the gain error to ~1e-4.
    assert!((v - 1.0).abs() < 2e-4, "{v}");
}

#[test]
fn the_output_clamps_at_the_rails() {
    let model = OpAmpModel::default();
    let high = out(&non_inverting(source(2.0, None), 9e3, 1e3, model));
    assert!((high - 15.0).abs() < 0.02, "{high}");
    let low = out(&non_inverting(source(-2.0, None), 9e3, 1e3, model));
    assert!((low + 15.0).abs() < 0.02, "{low}");
}

#[test]
fn offset_and_bias_current_shift_the_output() {
    // Follower: the offset appears at the output.
    let follower = |model| {
        Circuit::new(
            "0",
            vec![
                source(0.0, None),
                op_amp("in", "out", model),
                r("RL", "out", "0", 10e3),
            ],
        )
    };
    let v = out(&follower(OpAmpModel {
        offset_voltage: 1e-3,
        ..OpAmpModel::default()
    }));
    assert!((v - 1e-3).abs() < 1e-6, "{v}");

    // Inverting input tied to ground through 1 MΩ, out to it through 1 MΩ:
    // the bias current drawn through the feedback resistor sets
    // v_out = ib · rf at zero input.
    let circuit = Circuit::new(
        "0",
        vec![
            source(0.0, None),
            op_amp(
                "in",
                "fb",
                OpAmpModel {
                    bias_current: 100e-9,
                    ..OpAmpModel::default()
                },
            ),
            r("RF", "out", "fb", 1e6),
            r("RG", "fb", "0", 1e6),
            r("RS", "in", "0", 1.0),
        ],
    );
    let v = out(&circuit);
    assert!((v - 0.1).abs() < 1e-3, "{v}");
}

#[test]
fn closed_loop_bandwidth_is_gbw_over_gain() {
    let model = OpAmpModel {
        gbw: Some(1e6),
        ..OpAmpModel::default()
    };
    let circuit = non_inverting(source(0.0, None), 9e3, 1e3, model);
    let ac = solve_ac(
        &circuit,
        &AcConfig {
            f_start: 1e3,
            f_stop: 1e6,
            num_points: 301,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "VIN".into(),
            ac_magnitude: 1.0,
        },
    )
    .unwrap();
    let gain = |p: &sindr::ac_analysis::AcPoint| p.node_voltages["out"].norm();
    assert!((gain(&ac.points[0]) - 10.0).abs() < 0.01);
    // −3 dB at gbw · (1 + gain·β) / gain ≈ 100 kHz.
    let f3db = ac
        .points
        .iter()
        .find(|p| gain(p) < 10.0 / 2f64.sqrt())
        .unwrap()
        .frequency;
    assert!((f3db / 100e3 - 1.0).abs() < 0.03, "f3db {f3db}");
}

#[test]
fn without_a_pole_ac_is_flat() {
    let model = OpAmpModel {
        gbw: None,
        slew_rate: None,
        ..OpAmpModel::default()
    };
    let circuit = non_inverting(source(0.0, None), 9e3, 1e3, model);
    let ac = solve_ac(
        &circuit,
        &AcConfig {
            f_start: 1e3,
            f_stop: 1e9,
            num_points: 2,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "VIN".into(),
            ac_magnitude: 1.0,
        },
    )
    .unwrap();
    for p in &ac.points {
        assert!((p.node_voltages["out"].norm() - 10.0).abs() < 0.01);
    }
}

/// A unity follower driven by a 0 → 5 V step at 1 µs.
fn step_response(model: OpAmpModel) -> Vec<(f64, f64)> {
    let step = Waveform::Pulse {
        v1: 0.0,
        v2: 5.0,
        delay: 1e-6,
        rise_time: 1e-9,
        fall_time: 1e-9,
        pulse_width: 1.0,
        period: 2.0,
    };
    let circuit = Circuit::new(
        "0",
        vec![
            source(0.0, Some(step)),
            op_amp("in", "out", model),
            r("RL", "out", "0", 10e3),
        ],
    );
    let config = TransientConfig::new(20e-6, 20e-9).with_fixed_step();
    solve_transient(&circuit, &config)
        .unwrap()
        .transient
        .unwrap()
        .timesteps
        .iter()
        .map(|s| (s.time, s.node_voltages["out"]))
        .collect()
}

/// Time `trace` first reaches `level`, interpolated.
fn crossing(trace: &[(f64, f64)], level: f64) -> f64 {
    let i = trace.iter().position(|&(_, v)| v >= level).unwrap();
    let ((t0, v0), (t1, v1)) = (trace[i - 1], trace[i]);
    t0 + (level - v0) / (v1 - v0) * (t1 - t0)
}

#[test]
fn a_large_step_rises_at_the_slew_rate() {
    let sr = 0.5e6; // 0.5 V/µs
    let trace = step_response(OpAmpModel {
        gbw: Some(10e6),
        slew_rate: Some(sr),
        ..OpAmpModel::default()
    });
    let slope = (4.0 - 1.0) / (crossing(&trace, 4.0) - crossing(&trace, 1.0));
    assert!((slope / sr - 1.0).abs() < 0.03, "slope {slope} V/s");
    let end = trace.last().unwrap().1;
    assert!((end - 5.0).abs() < 1e-3, "settles to {end}");
}

#[test]
fn without_a_slew_limit_a_step_follows_the_closed_loop_pole() {
    // Unity follower: τ_cl = 1 / (2π · gbw), so 10–90 % takes 2.2 τ_cl.
    let gbw = 1e6;
    let trace = step_response(OpAmpModel {
        gbw: Some(gbw),
        slew_rate: None,
        ..OpAmpModel::default()
    });
    let rise = crossing(&trace, 4.5) - crossing(&trace, 0.5);
    let expected = 2.2 / (2.0 * std::f64::consts::PI * gbw);
    assert!((rise / expected - 1.0).abs() < 0.05, "rise {rise} s");
}

#[test]
fn the_ideal_op_amp_is_unchanged() {
    let circuit = Circuit::new(
        "0",
        vec![
            source(2.0, None),
            CircuitElement::OpAmp {
                id: "U1".into(),
                nodes: ["in".into(), "fb".into(), "out".into()],
                v_pos: 15.0,
                v_neg: -15.0,
                model: None,
            },
            r("RF", "out", "fb", 9e3),
            r("RG", "fb", "0", 1e3),
        ],
    );
    // No rail clamp without a model: 2 V × 10, less the 1e5 gain error.
    let v = out(&circuit);
    assert!((v - 20.0).abs() < 0.01, "{v}");
}

#[test]
fn bad_model_parameters_name_the_field() {
    let cases = [
        (
            OpAmpModel {
                gain: 0.0,
                ..OpAmpModel::default()
            },
            "model.gain",
        ),
        (
            OpAmpModel {
                gbw: Some(-1.0),
                ..OpAmpModel::default()
            },
            "model.gbw",
        ),
        (
            OpAmpModel {
                gbw: None,
                ..OpAmpModel::default()
            },
            "needs model.gbw",
        ),
        (
            OpAmpModel {
                r_out: -1.0,
                ..OpAmpModel::default()
            },
            "model.r_out",
        ),
    ];
    for (model, field) in cases {
        match solve_circuit(&non_inverting(source(0.1, None), 9e3, 1e3, model)) {
            Err(SimError::InvalidComponent(msg)) => assert!(msg.contains(field), "{msg}"),
            other => panic!("{field}: expected InvalidComponent, got {other:?}"),
        }
    }
}

#[cfg(feature = "serde")]
#[test]
fn json_without_a_model_is_the_ideal_op_amp() {
    let json = r#"{"type":"op_amp","id":"U1","nodes":["a","b","c"]}"#;
    let el: CircuitElement = serde_json::from_str(json).unwrap();
    assert!(matches!(el, CircuitElement::OpAmp { model: None, .. }));
    let json = r#"{"type":"op_amp","id":"U1","nodes":["a","b","c"],"model":{"gbw":2e6}}"#;
    match serde_json::from_str::<CircuitElement>(json).unwrap() {
        CircuitElement::OpAmp { model: Some(m), .. } => {
            assert_eq!(m.gbw, Some(2e6));
            assert_eq!(m.gain, 1e5);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_saturated_stage_recovers_without_winding_up() {
    // A follower driven 5 V past its +15 V rail for 20 µs, then back to 0.
    let pulse = Waveform::Pulse {
        v1: 0.0,
        v2: 20.0,
        delay: 1e-6,
        rise_time: 1e-9,
        fall_time: 1e-9,
        pulse_width: 20e-6,
        period: 1.0,
    };
    let circuit = Circuit::new(
        "0",
        vec![
            source(0.0, Some(pulse)),
            op_amp(
                "in",
                "out",
                OpAmpModel {
                    gbw: Some(10e6),
                    slew_rate: None,
                    ..OpAmpModel::default()
                },
            ),
            r("RL", "out", "0", 10e3),
        ],
    );
    let config = TransientConfig::new(25e-6, 10e-9).with_fixed_step();
    let trace: Vec<(f64, f64)> = solve_transient(&circuit, &config)
        .unwrap()
        .transient
        .unwrap()
        .timesteps
        .iter()
        .map(|s| (s.time, s.node_voltages["out"]))
        .collect();
    let at = |t: f64| trace.iter().find(|&&(time, _)| time >= t).unwrap().1;
    assert!((at(15e-6) - 15.0).abs() < 0.05, "clamped at {}", at(15e-6));
    // Wound up by 5 V × 1e5 it would take ~0.8 s to return; it takes ~1 µs.
    assert!(at(23e-6).abs() < 0.05, "recovered to {}", at(23e-6));
}

#[test]
fn transfer_function_sees_the_model() {
    use sindr::{solve_tf, TfConfig, TfOutput};
    let model = OpAmpModel {
        r_out: 100.0,
        ..OpAmpModel::default()
    };
    let circuit = non_inverting(source(0.1, None), 9e3, 1e3, model);
    let tf = solve_tf(
        &circuit,
        &TfConfig {
            output: TfOutput::Voltage {
                node: "out".into(),
                reference: None,
            },
            input_source: "VIN".into(),
        },
    )
    .unwrap();
    assert!((tf.gain - 10.0).abs() < 2e-3, "gain {}", tf.gain);
    // r_out / (1 + loop gain) ≈ 100 Ω / 1e4, in parallel with the load.
    assert!(
        (tf.output_resistance - 0.01).abs() < 1e-3,
        "rout {}",
        tf.output_resistance
    );
}
