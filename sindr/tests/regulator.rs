//! Linear regulator model against LM7805 and AMS1117 datasheet points.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::regulator::RegulatorParams;
use sindr::{
    check_ratings, solve_circuit, Circuit, CircuitElement, Part, RatingKind, RatingStatus, Ratings,
    SimulationResult, Thermal,
};

fn regulator(vin: f64, vout: f64, model: RegulatorParams, load: CircuitElement) -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "VIN".into(),
                nodes: ["in".into(), "0".into()],
                voltage: vin,
                waveform: None,
            },
            CircuitElement::VoltageRegulator {
                id: "U1".into(),
                nodes: ["in".into(), "out".into(), "0".into()],
                voltage: vout,
                model: Some(model),
            },
            load,
        ],
    )
}

fn load_r(ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: "RL".into(),
        nodes: ["out".into(), "0".into()],
        resistance: ohms,
    }
}

fn load_i(amps: f64) -> CircuitElement {
    CircuitElement::CurrentSource {
        id: "IL".into(),
        nodes: ["out".into(), "0".into()],
        current: amps,
        waveform: None,
    }
}

fn u1(r: &SimulationResult) -> &sindr::ComponentResult {
    r.component_results.iter().find(|c| c.id == "U1").unwrap()
}

fn vout(r: &SimulationResult) -> f64 {
    r.node_voltages["out"]
}

/// Current the input source delivers.
fn iin(r: &SimulationResult) -> f64 {
    -r.branch_currents["VIN"]
}

#[test]
fn lm7805_regulates_with_headroom() {
    let r = solve_circuit(&regulator(
        12.0,
        5.0,
        RegulatorParams::lm7805(),
        load_i(0.5),
    ))
    .unwrap();
    // 0.5 A through the 67 mΩ load-regulation resistance: about 33 mV low.
    assert!((vout(&r) - 5.0).abs() < 0.05, "{}", vout(&r));
    // The input carries the load plus 4.3 mA quiescent.
    assert!((iin(&r) - 0.5043).abs() < 1e-6, "{}", iin(&r));
}

#[test]
fn lm7805_drops_out_two_volts_below_its_input() {
    // 6 V in can't make 5 V out with a 2 V dropout: it follows the input.
    let r = solve_circuit(&regulator(6.0, 5.0, RegulatorParams::lm7805(), load_i(1.0))).unwrap();
    assert!((vout(&r) - 4.0).abs() < 1e-6, "{}", vout(&r));
    // Just enough headroom at 7 V (plus the load-regulation drop).
    let r = solve_circuit(&regulator(7.1, 5.0, RegulatorParams::lm7805(), load_i(1.0))).unwrap();
    assert!((vout(&r) - 5.0).abs() < 0.08, "{}", vout(&r));
}

#[test]
fn lm7805_line_and_load_regulation_match_the_datasheet() {
    let at = |vin, amps| {
        vout(
            &solve_circuit(&regulator(
                vin,
                5.0,
                RegulatorParams::lm7805(),
                load_i(amps),
            ))
            .unwrap(),
        )
    };
    // Line: 3 mV over 7–25 V (typical).
    let line = at(25.0, 0.5) - at(7.5, 0.5);
    assert!((line - 3e-3 * 17.5 / 18.0).abs() < 1e-4, "line {line}");
    // Load: 100 mV over 5 mA–1.5 A (maximum).
    let load = at(12.0, 0.005) - at(12.0, 1.5);
    assert!((load - 0.1).abs() < 1e-3, "load {load}");
}

#[test]
fn lm7805_current_limit_holds_a_short() {
    let r = solve_circuit(&regulator(
        12.0,
        5.0,
        RegulatorParams::lm7805(),
        load_r(1e-3),
    ))
    .unwrap();
    let i = u1(&r).current_through;
    assert!((i - 2.2).abs() < 0.03, "short-circuit current {i}");
    assert!(vout(&r) < 0.01);
}

#[test]
fn dissipation_reaches_the_ratings_and_thermal_model() {
    // 12 V → 5 V at 0.5 A in a TO-220 with no heatsink (Rθja ≈ 50 K/W).
    let mut c = regulator(12.0, 5.0, RegulatorParams::lm7805(), load_i(0.5));
    c.parts.insert(
        "U1".into(),
        Part {
            ratings: Ratings {
                voltage: Some(35.0),
                current: Some(1.5),
                power: Some(15.0),
                tj_max: Some(398.15), // 125 °C
                ..Ratings::default()
            },
            thermal: Some(Thermal::junction_to_ambient(50.0)),
            ..Part::default()
        },
    );
    let r = solve_circuit(&c).unwrap();
    let u = u1(&r);
    // (Vin − Vout)·Iout + Vin·Iq
    let expected = (12.0 - vout(&r)) * 0.5 + 12.0 * 4.3e-3;
    assert!(
        (u.power - expected).abs() < 1e-9,
        "{} vs {expected}",
        u.power
    );
    assert!((u.voltage_across - (12.0 - vout(&r))).abs() < 1e-12);
    assert!((u.current_through - 0.5).abs() < 1e-9);
    // 3.6 W × 50 K/W puts the junction near 480 K: past 125 °C.
    let tj = r.junction_temperatures["U1"];
    assert!((tj - (300.15 + u.power * 50.0)).abs() < 0.01, "{tj}");

    let report = check_ratings(&c, &r).unwrap();
    let status = |k| {
        report
            .for_part("U1")
            .find(|c| c.rating == k)
            .unwrap()
            .status
    };
    assert_eq!(status(RatingKind::Tj), RatingStatus::Exceeded);
    assert_eq!(status(RatingKind::Power), RatingStatus::Within);
    assert_eq!(status(RatingKind::Voltage), RatingStatus::Within);
    let vin = report
        .for_part("U1")
        .find(|c| c.rating == RatingKind::Voltage)
        .unwrap();
    assert!((vin.worst.unwrap() - 12.0).abs() < 1e-12, "input voltage");
}

#[test]
fn ams1117_from_usb_regulates_but_not_from_a_lithium_cell() {
    let at = |vin| {
        vout(
            &solve_circuit(&regulator(
                vin,
                3.3,
                RegulatorParams::ams1117(),
                load_i(0.8),
            ))
            .unwrap(),
        )
    };
    assert!((at(5.0) - 3.3).abs() < 0.02, "{}", at(5.0));
    // 1.1 V dropout at 0.8 A: 4.2 V in gives 3.1 V out.
    assert!((at(4.2) - 3.1).abs() < 1e-6, "{}", at(4.2));
}

#[test]
fn ams1117_current_limit() {
    let r = solve_circuit(&regulator(
        5.0,
        3.3,
        RegulatorParams::ams1117(),
        load_r(0.5),
    ))
    .unwrap();
    let i = u1(&r).current_through;
    assert!((i - 1.1).abs() < 0.02, "{i}");
}

#[test]
fn ideal_regulator_still_draws_its_output_from_the_input() {
    let mut c = regulator(12.0, 5.0, RegulatorParams::default(), load_r(100.0));
    if let CircuitElement::VoltageRegulator { model, .. } = &mut c.components[1] {
        *model = None;
    }
    let r = solve_circuit(&c).unwrap();
    assert!((vout(&r) - 5.0).abs() < 1e-9);
    assert!((iin(&r) - 0.05).abs() < 1e-9);
    assert!((u1(&r).power - 7.0 * 0.05).abs() < 1e-9);
    // And can't make more than it gets.
    let r = solve_circuit(&regulator(
        3.0,
        5.0,
        RegulatorParams::default(),
        load_r(100.0),
    ))
    .unwrap();
    assert!((vout(&r) - 3.0).abs() < 1e-9);
}

#[test]
fn input_ripple_reaches_the_output_through_line_regulation() {
    let ripple = |vin: f64| {
        let c = regulator(vin, 5.0, RegulatorParams::lm7805(), load_r(10.0));
        let ac = solve_ac(
            &c,
            &AcConfig::new("VIN", 100.0, 100.0, 1).with_spacing(FrequencySpacing::Linear),
        )
        .unwrap();
        ac.points[0].node_voltages["out"].norm()
    };
    // Regulating: only the line-regulation fraction gets through, divided
    // between the output resistance and the 10 Ω load.
    let p = RegulatorParams::lm7805();
    let expected = p.line_regulation / (1.0 + p.output_resistance / 10.0);
    assert!((ripple(12.0) - expected).abs() < 1e-9, "{}", ripple(12.0));
    // In dropout all of it does.
    assert!((ripple(6.0) - 1.0).abs() < 1e-6, "{}", ripple(6.0));
}

/// A 7905: the LM7805 characteristic set to −5 V, fed from −12 V. It sinks
/// its load current in at the output pin and regulates in DC and transient.
#[test]
fn negative_regulator_holds_minus_five_from_minus_twelve() {
    // A 0.5 A load pulled up to ground: current flows out of ground, through
    // the load and in at the output pin.
    let c = regulator(-12.0, -5.0, RegulatorParams::lm7805(), load_i(-0.5));
    let r = solve_circuit(&c).unwrap();
    assert!((vout(&r) + 5.0).abs() < 0.05, "{}", vout(&r));
    // The input carries the load plus the quiescent current, all negative.
    assert!((iin(&r) + 0.5043).abs() < 1e-6, "{}", iin(&r));
    let pass = u1(&r);
    assert!(pass.power > 0.0, "dissipation {}", pass.power);

    // Out of headroom at −6 V in: the output follows the input up by 2 V.
    let r = solve_circuit(&regulator(
        -6.0,
        -5.0,
        RegulatorParams::lm7805(),
        load_i(-1.0),
    ))
    .unwrap();
    assert!((vout(&r) + 4.0).abs() < 1e-6, "{}", vout(&r));

    // A resistive load in transient settles on the DC answer.
    let c = regulator(-12.0, -5.0, RegulatorParams::lm7805(), load_r(10.0));
    let dc = vout(&solve_circuit(&c).unwrap());
    assert!((dc + 5.0).abs() < 0.05, "{dc}");
    let tran = sindr::solve_transient(&c, &sindr::TransientConfig::new(1e-4, 1e-5))
        .unwrap()
        .transient
        .unwrap();
    for step in &tran.timesteps {
        let v = step.node_voltages["out"];
        assert!((v - dc).abs() < 1e-6, "t = {}: {v}", step.time);
    }
}

#[test]
fn negative_regulator_current_limit_holds_a_short() {
    let r = solve_circuit(&regulator(
        -12.0,
        -5.0,
        RegulatorParams::lm7805(),
        load_r(1e-3),
    ))
    .unwrap();
    let i = u1(&r).current_through;
    assert!((i + 2.2).abs() < 0.03, "short-circuit current {i}");
    assert!(vout(&r).abs() < 0.01);
}

#[test]
fn zero_regulator_voltage_is_rejected() {
    let err = solve_circuit(&regulator(
        12.0,
        0.0,
        RegulatorParams::lm7805(),
        load_r(10.0),
    ))
    .unwrap_err();
    assert!(err.to_string().contains("non-zero"), "{err}");
}

/// With no input the pass element is off: the output sits at ground rather
/// than below it, and nothing is drawn from a supply that isn't there.
#[test]
fn unpowered_regulator_outputs_nothing() {
    for vin in [0.0, 0.5, 1.0] {
        let r =
            solve_circuit(&regulator(vin, 5.0, RegulatorParams::lm7805(), load_r(1e3))).unwrap();
        let expect = (vin - RegulatorParams::lm7805().dropout).max(0.0);
        assert!(vout(&r) >= -1e-9, "vin {vin}: vout {}", vout(&r));
        assert!(
            (vout(&r) - expect).abs() < 1e-6,
            "vin {vin}: vout {}",
            vout(&r)
        );
        // No energy from nowhere: the input delivers at least what the load takes.
        let p_load = vout(&r).powi(2) / 1e3;
        assert!(iin(&r) * vin >= p_load - 1e-12, "vin {vin}");
    }
}

/// An input pulled to ground through a resistor stays at ground: the
/// quiescent current fades out with the input instead of pulling it negative.
#[test]
fn input_pulled_to_ground_stays_there() {
    let mut c = regulator(0.0, 5.0, RegulatorParams::lm7805(), load_r(1e3));
    c.components[0] = CircuitElement::Resistor {
        id: "RIN".into(),
        nodes: ["in".into(), "0".into()],
        resistance: 1e3,
    };
    let r = solve_circuit(&c).unwrap();
    assert!(
        r.node_voltages["in"].abs() < 1e-9,
        "vin {}",
        r.node_voltages["in"]
    );
    assert!(vout(&r).abs() < 1e-9, "vout {}", vout(&r));
    assert!(u1(&r).power.abs() < 1e-12);
}

/// The quiescent current ramps from 0 at no input to its full value at the
/// dropout voltage, and is full from there up.
#[test]
fn quiescent_current_ramps_in_below_dropout() {
    let p = RegulatorParams::lm7805();
    let iq = |vin: f64| {
        let r = solve_circuit(&regulator(vin, 5.0, p, load_i(0.0))).unwrap();
        iin(&r)
    };
    assert!(iq(0.0).abs() < 1e-12);
    assert!((iq(p.dropout / 2.0) - p.quiescent_current / 2.0).abs() < 1e-9);
    assert!((iq(p.dropout) - p.quiescent_current).abs() < 1e-9);
    assert!((iq(12.0) - p.quiescent_current).abs() < 1e-9);
}

/// From 0 V to well past regulation the output never steps: it is 0, then
/// follows the input down by the dropout voltage, then holds.
#[test]
fn output_is_continuous_and_monotonic_in_the_input() {
    let mut last = 0.0;
    for k in 0..=240 {
        let vin = k as f64 * 0.05;
        let r = solve_circuit(&regulator(
            vin,
            5.0,
            RegulatorParams::lm7805(),
            load_r(100.0),
        ))
        .unwrap();
        let v = vout(&r);
        assert!(v >= last - 1e-9, "vin {vin}: {v} after {last}");
        assert!(v - last < 0.05 + 1e-6, "vin {vin}: jumped {} V", v - last);
        last = v;
    }
    assert!((last - 5.0).abs() < 0.05);
}

/// The same holds for a negative regulator, mirrored through its ground pin.
#[test]
fn unpowered_negative_regulator_outputs_nothing() {
    let p = RegulatorParams::lm7805();
    for vin in [0.0, -0.5, -1.0] {
        let r = solve_circuit(&regulator(vin, -5.0, p, load_r(1e3))).unwrap();
        let expect = (vin + p.dropout).min(0.0);
        assert!(vout(&r) <= 1e-9, "vin {vin}: vout {}", vout(&r));
        assert!(
            (vout(&r) - expect).abs() < 1e-6,
            "vin {vin}: vout {}",
            vout(&r)
        );
    }
}

/// Leaving the 0 V floor on a ramped input, an output capacitor draws
/// C·dV/dt. Newton must not settle on the floor piece once the solution
/// is past the corner: within the node tolerance the output lags by a
/// millivolt, then catches up in one short step, a spike up to the
/// current limit. Here a 7805 ramps the input of an AMS1117.
#[test]
fn ramped_start_up_leaves_the_floor_without_a_current_spike() {
    let c = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "VIN".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(sindr::Waveform::Pwl {
                    points: vec![(0.0, 0.0), (1e-3, 12.0)],
                }),
            },
            CircuitElement::VoltageRegulator {
                id: "U1".into(),
                nodes: ["in".into(), "mid".into(), "0".into()],
                voltage: 5.0,
                model: Some(RegulatorParams::lm7805()),
            },
            CircuitElement::Capacitor {
                id: "CMID".into(),
                nodes: ["mid".into(), "0".into()],
                capacitance: 10e-6,
            },
            CircuitElement::VoltageRegulator {
                id: "U2".into(),
                nodes: ["mid".into(), "out".into(), "0".into()],
                voltage: 3.3,
                model: Some(RegulatorParams::ams1117()),
            },
            CircuitElement::Capacitor {
                id: "COUT".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 22e-6,
            },
            load_r(6.6),
        ],
    );
    let tran = sindr::solve_transient(&c, &sindr::TransientConfig::new(1e-3, 5e-6))
        .unwrap()
        .transient
        .unwrap();
    let inrush = 22e-6 * 12.0 / 1e-3;
    let peak = tran
        .timesteps
        .iter()
        .map(|s| {
            s.component_results
                .iter()
                .find(|r| r.id == "COUT")
                .unwrap()
                .current_through
        })
        .fold(f64::MIN, f64::max);
    // Gear-2 still overshoots one step at the corner in dV/dt, by about
    // 30 %; the stale piece gave the full 1.1 A limit.
    assert!(peak > 0.9 * inrush && peak < 1.35 * inrush, "peak {peak} A");
}
