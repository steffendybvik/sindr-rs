//! Averaged switching regulator (buck, boost): power balance, current
//! limit, dropout and lockout in DC, its small-signal input, and a
//! transient through its lockout.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::element::{CurrentSource, Resistor, SwitchingRegulator, VoltageSource};
use sindr::waveform::Waveform;
use sindr::{
    check_ratings, solve_circuit, solve_transient, Circuit, CircuitElement, ConverterParams,
    ConverterTopology, Part, RatingKind, RatingStatus, Ratings, SimulationResult, TransientConfig,
};

use ConverterTopology::{Boost, Buck};

fn converter(
    topology: ConverterTopology,
    vin: f64,
    vout: f64,
    model: ConverterParams,
    load: CircuitElement,
) -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "VIN".into(),
                nodes: ["in".into(), "0".into()],
                voltage: vin,
                waveform: None,
            }),
            CircuitElement::SwitchingRegulator(SwitchingRegulator {
                id: "U1".into(),
                nodes: ["in".into(), "out".into(), "0".into()],
                topology,
                voltage: vout,
                model,
            }),
            load,
        ],
    )
}

fn load_r(ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: "RL".into(),
        nodes: ["out".into(), "0".into()],
        resistance: ohms,
    })
}

fn load_i(amps: f64) -> CircuitElement {
    CircuitElement::CurrentSource(CurrentSource {
        id: "IL".into(),
        nodes: ["out".into(), "0".into()],
        current: amps,
        waveform: None,
    })
}

fn u1(r: &SimulationResult) -> &sindr::ComponentResult {
    r.component("U1").unwrap()
}

fn vout(r: &SimulationResult) -> f64 {
    r.node_voltages["out"]
}

/// Current the input source delivers.
fn iin(r: &SimulationResult) -> f64 {
    -r.branch_currents["VIN"]
}

fn eta(efficiency: f64) -> ConverterParams {
    ConverterParams {
        efficiency,
        ..Default::default()
    }
}

#[test]
fn buck_balances_power() {
    let model = ConverterParams {
        quiescent_current: 2e-3,
        ..eta(0.88)
    };
    let r = solve_circuit(&converter(Buck, 12.0, 5.0, model, load_i(2.0))).unwrap();
    assert!((vout(&r) - 5.0).abs() < 1e-12, "{}", vout(&r));
    // Pin·η = Pout, on top of the quiescent draw.
    let p_in = 12.0 * (iin(&r) - 2e-3);
    assert!((p_in * 0.88 - 10.0).abs() < 1e-9, "{p_in}");
    let u = u1(&r);
    assert!((u.voltage_across - 5.0).abs() < 1e-12);
    assert!(
        (u.current_through - 2.0).abs() < 1e-9,
        "{}",
        u.current_through
    );
    assert!(
        (u.power - (12.0 * iin(&r) - 10.0)).abs() < 1e-9,
        "{}",
        u.power
    );
}

#[test]
fn boost_balances_power_into_a_resistor() {
    let r = solve_circuit(&converter(Boost, 3.3, 5.0, eta(0.9), load_r(10.0))).unwrap();
    assert!((vout(&r) - 5.0).abs() < 1e-12, "{}", vout(&r));
    // 2.5 W out takes 2.5 / 0.9 W in.
    assert!((iin(&r) - 2.5 / (0.9 * 3.3)).abs() < 1e-9, "{}", iin(&r));
}

#[test]
fn efficiency_over_load_is_followed() {
    let model = ConverterParams {
        efficiency_curve: vec![[0.01, 0.6], [0.1, 0.85], [1.0, 0.92], [3.0, 0.86]],
        ..Default::default()
    };
    for (load, expected) in [(0.005, 0.6), (0.55, 0.885), (2.0, 0.89), (5.0, 0.86)] {
        let r = solve_circuit(&converter(
            Buck,
            12.0,
            5.0,
            model.clone(),
            load_r(5.0 / load),
        ))
        .unwrap();
        let achieved = 5.0 * load / (12.0 * iin(&r));
        // To Newton's tolerance: the input current is linearised at the
        // iterate before the one it converged on.
        assert!((achieved - expected).abs() < 1e-8, "{load} A: {achieved}");
    }
}

#[test]
fn current_limit_holds_a_short_to_one_percent() {
    let model = ConverterParams {
        current_limit: Some(1.5),
        ..eta(0.9)
    };
    let r = solve_circuit(&converter(Buck, 12.0, 5.0, model, load_r(0.1))).unwrap();
    let i = u1(&r).current_through;
    assert!((i - 1.5).abs() < 0.015, "{i}");
    assert!((vout(&r) - 0.1 * i).abs() < 1e-9, "{}", vout(&r));
    // The input still carries only what the short dissipates.
    assert!((12.0 * iin(&r) * 0.9 - vout(&r) * i).abs() < 1e-9);
}

#[test]
fn boost_current_limit_holds_its_load_above_the_input() {
    let model = ConverterParams {
        current_limit: Some(1.0),
        ..eta(0.9)
    };
    // 3 Ω at 5 V would ask 1.67 A: the output falls to about 3 V instead.
    let r = solve_circuit(&converter(Boost, 2.5, 5.0, model, load_r(3.0))).unwrap();
    let i = u1(&r).current_through;
    assert!((i - 1.0).abs() < 0.01, "{i}");
    assert!((vout(&r) - 3.0 * i).abs() < 1e-9);
}

#[test]
fn buck_drops_out_and_follows_its_input_down() {
    let model = ConverterParams {
        dropout: 0.3,
        dropout_resistance: 0.1,
        ..eta(0.95)
    };
    // 5 V in can't make 5 V out: Vin − 0.3 − 0.1·Iout.
    let r = solve_circuit(&converter(Buck, 5.0, 5.0, model.clone(), load_i(1.0))).unwrap();
    assert!((vout(&r) - 4.6).abs() < 1e-9, "{}", vout(&r));
    // Enough headroom at 5.5 V.
    let r = solve_circuit(&converter(Buck, 5.5, 5.0, model, load_i(1.0))).unwrap();
    assert!((vout(&r) - 5.0).abs() < 1e-12, "{}", vout(&r));
}

#[test]
fn boost_passes_an_input_above_its_set_point_through() {
    let model = ConverterParams {
        dropout: 0.4,
        ..eta(0.95)
    };
    let r = solve_circuit(&converter(Boost, 6.0, 5.0, model, load_r(10.0))).unwrap();
    assert!((vout(&r) - 5.6).abs() < 1e-9, "{}", vout(&r));
}

#[test]
fn below_its_lockout_it_is_off() {
    let model = ConverterParams {
        uvlo: 4.5,
        quiescent_current: 1e-3,
        ..eta(0.9)
    };
    let r = solve_circuit(&converter(Buck, 4.0, 3.3, model.clone(), load_r(10.0))).unwrap();
    assert!(vout(&r).abs() < 1e-9, "{}", vout(&r));
    assert!(iin(&r).abs() < 1e-9, "{}", iin(&r));
    assert!(u1(&r).power.abs() < 1e-9);
    let r = solve_circuit(&converter(Buck, 5.0, 3.3, model, load_r(10.0))).unwrap();
    assert!((vout(&r) - 3.3).abs() < 1e-12, "{}", vout(&r));
}

#[test]
fn unpowered_converter_outputs_nothing() {
    let r = solve_circuit(&converter(Buck, 0.0, 5.0, eta(0.9), load_r(10.0))).unwrap();
    assert!(vout(&r).abs() < 1e-12 && iin(&r).abs() < 1e-12);
}

/// A converter on a constant-power load looks like a negative resistance
/// from its input, `−Pin/Vin²`: the input current falls as the input rises.
#[test]
fn small_signal_input_is_a_negative_resistance() {
    let (vin, rs, p_in) = (12.0, 1.0, 2.5 / 0.9);
    let mut c = converter(Buck, vin, 5.0, eta(0.9), load_r(10.0));
    // Feed it through 1 Ω so its input impedance shows.
    c.components[0] = CircuitElement::VoltageSource(VoltageSource {
        id: "VIN".into(),
        nodes: ["src".into(), "0".into()],
        voltage: vin,
        waveform: None,
    });
    c.components.push(CircuitElement::Resistor(Resistor {
        id: "RS".into(),
        nodes: ["src".into(), "in".into()],
        resistance: rs,
    }));
    let dc = solve_circuit(&c).unwrap();
    let v_bias = dc.node_voltages["in"];
    let g = -p_in / (v_bias * v_bias);
    let ac = solve_ac(
        &c,
        &AcConfig::new("VIN", 100.0, 100.0, 1).with_spacing(FrequencySpacing::Linear),
    )
    .unwrap();
    let point = &ac.points[0];
    let v_in = point.node_voltages["in"].re;
    assert!((v_in - 1.0 / (1.0 + rs * g)).abs() < 1e-9, "{v_in}");
    // Regulating, none of it reaches the output.
    assert!(point.node_voltages["out"].norm() < 1e-12);
}

#[test]
fn ratings_see_its_input_and_output() {
    let mut c = converter(Buck, 24.0, 5.0, eta(0.9), load_i(3.0));
    c.parts.insert(
        "U1".into(),
        Part {
            ratings: Ratings {
                voltage: Some(20.0),
                current: Some(3.5),
                ..Ratings::default()
            },
            ..Part::default()
        },
    );
    let r = solve_circuit(&c).unwrap();
    let report = check_ratings(&c, &r).unwrap();
    let check = |k| report.for_part("U1").find(|c| c.rating == k).unwrap();
    assert_eq!(check(RatingKind::Voltage).status, RatingStatus::Exceeded);
    assert!((check(RatingKind::Voltage).worst.unwrap() - 24.0).abs() < 1e-12);
    assert_eq!(check(RatingKind::Current).status, RatingStatus::Within);
}

/// The input ramps from 0 to 12 V through a 4.5 V lockout: the output
/// stays off, then holds 5 V, and every step balances power.
#[test]
fn transient_ramps_through_the_lockout() {
    let model = ConverterParams {
        uvlo: 4.5,
        ..eta(0.85)
    };
    let mut c = converter(Buck, 0.0, 5.0, model, load_r(5.0));
    if let CircuitElement::VoltageSource(v) = &mut c.components[0] {
        v.waveform = Some(Waveform::Pwl {
            points: vec![(0.0, 0.0), (1e-3, 12.0)],
        });
    }
    let config = TransientConfig::new(1.5e-3, 1e-5);
    let result = solve_transient(&c, &config).unwrap();
    let steps = &result.transient.as_ref().unwrap().timesteps;
    for step in steps {
        let vin = step.node_voltages["in"];
        let vout = step.node_voltages["out"];
        let u = step
            .component_results
            .iter()
            .find(|c| c.id == "U1")
            .unwrap();
        if vin < 4.5 - 1e-9 {
            assert!(vout.abs() < 1e-9, "t = {}: {vout} at {vin} V in", step.time);
        } else if vin > 5.0 {
            assert!((vout - 5.0).abs() < 1e-9, "t = {}: {vout}", step.time);
            // Loss = Pout·(1/η − 1).
            assert!(
                (u.power - 5.0 * (1.0 / 0.85 - 1.0)).abs() < 1e-9,
                "{}",
                u.power
            );
        }
    }
    let last = steps.last().unwrap();
    assert!((last.node_voltages["out"] - 5.0).abs() < 1e-9);
}

#[test]
fn zero_or_bad_settings_are_rejected() {
    let bad = |topology, voltage, model| {
        solve_circuit(&converter(topology, 12.0, voltage, model, load_r(10.0))).is_err()
    };
    assert!(bad(Buck, 0.0, ConverterParams::default()));
    assert!(bad(Buck, 5.0, eta(0.0)));
    assert!(bad(Boost, 5.0, eta(1.2)));
    assert!(bad(
        Buck,
        5.0,
        ConverterParams {
            efficiency_curve: vec![[1.0, 0.9], [0.5, 0.8]],
            ..Default::default()
        }
    ));
    assert!(bad(
        Buck,
        5.0,
        ConverterParams {
            current_limit: Some(0.0),
            ..Default::default()
        }
    ));
}

#[cfg(feature = "serde")]
#[test]
fn json_round_trips() {
    let c = converter(
        Boost,
        3.3,
        5.0,
        ConverterParams {
            current_limit: Some(2.0),
            efficiency_curve: vec![[0.1, 0.8], [1.0, 0.9]],
            switching_frequency: Some(1e6),
            inductance: Some(2.2e-6),
            ..eta(0.9)
        },
        load_r(10.0),
    );
    let json = serde_json::to_string(&c).unwrap();
    assert!(json.contains("\"switching_regulator\""), "{json}");
    assert!(json.contains("\"boost\""), "{json}");
    let back: Circuit = serde_json::from_str(&json).unwrap();
    assert_eq!(back, c);
    // An ideal buck needs only its pins and voltage.
    let minimal =
        r#"{"type":"switching_regulator","id":"U1","nodes":["in","out","0"],"voltage":5.0}"#;
    let e: CircuitElement = serde_json::from_str(minimal).unwrap();
    match e {
        CircuitElement::SwitchingRegulator(sw) => {
            assert_eq!(sw.topology, Buck);
            assert_eq!(sw.model, ConverterParams::default());
        }
        other => panic!("{other:?}"),
    }
}

/// Its loss heats it through the part's thermal data, as any part's does.
#[test]
fn its_loss_heats_it() {
    let mut c = converter(Buck, 12.0, 5.0, eta(0.8), load_i(2.0));
    c.parts.insert(
        "U1".into(),
        Part {
            thermal: Some(sindr::Thermal::junction_to_ambient(40.0)),
            ..Part::default()
        },
    );
    let r = solve_circuit(&c).unwrap();
    // 10 W out at 80 % loses 2.5 W.
    let p = u1(&r).power;
    assert!((p - 2.5).abs() < 1e-8, "{p}");
    let tj = r.junction_temperatures["U1"];
    assert!((tj - (300.15 + 2.5 * 40.0)).abs() < 0.01, "{tj}");
}
