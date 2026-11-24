//! Digital I/O pins: the output stage against hand calculations, the
//! clamps against the diode model, modes, read-back in a session, and
//! ratings.

use sindr::devices::diode::{diode_current, DiodeParams};
use sindr::devices::led::led_params_from_str;
use sindr::element::{
    Capacitor, DigitalPin, Led, LogicLevel, PinMode, PinModel, Pull, Resistor, VoltageSource,
};
use sindr::{
    check_ratings, solve_circuit, validate_circuit, Circuit, CircuitElement, DiodeModel, Edge,
    Event, Param, Part, RatingStatus, Ratings, SimError, SimulationResult, TransientConfig,
    TransientSession,
};

const VDD: f64 = 3.3;

fn vsource(id: &str, p: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), "0".into()],
        voltage: volts,
        waveform: None,
    })
}

fn resistor(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn pin(id: &str, pad: &str, mode: PinMode, level: bool) -> DigitalPin {
    DigitalPin {
        id: id.into(),
        nodes: [pad.into(), "vdd".into(), "0".into()],
        mode,
        level,
        pull: Pull::None,
        port: None,
        model: PinModel::default(),
    }
}

/// A 3.3 V supply and `parts`.
fn powered(parts: Vec<CircuitElement>) -> Circuit {
    let mut components = vec![vsource("VDD", "vdd", VDD)];
    components.extend(parts);
    Circuit::new("0", components)
}

fn result_of<'a>(result: &'a SimulationResult, id: &str) -> &'a sindr::ComponentResult {
    result
        .component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("no result for {id}"))
}

/// Output high into a resistor: a divider of `r_high` and the load.
#[test]
fn output_high_into_a_resistor_is_a_divider_with_r_high() {
    let circuit = powered(vec![
        CircuitElement::DigitalPin(pin("D13", "pad", PinMode::PushPull, true)),
        resistor("RL", "pad", "0", 1e3),
    ]);
    let result = solve_circuit(&circuit).unwrap();
    let v = VDD * 1e3 / (1e3 + 50.0);
    assert!((result.node_voltages["pad"] - v).abs() < 1e-9);
    let d13 = result_of(&result, "D13");
    assert!((d13.voltage_across - v).abs() < 1e-9);
    // It sources the load's current: negative into the pin.
    // The clamps' gmin shunts carry picoamps.
    assert!((d13.current_through + v / 1e3).abs() < 1e-10);
    // r_high dissipates (VDD − v)·I.
    assert!((d13.power - (VDD - v) * v / 1e3).abs() < 1e-10);
    // The parts and their nodes don't show.
    assert!(result.component_results.iter().all(|c| !c.id.contains('#')));
}

/// Output low sinking from the supply through a resistor.
#[test]
fn output_low_sinks_through_r_low() {
    let circuit = powered(vec![
        CircuitElement::DigitalPin(pin("D2", "pad", PinMode::PushPull, false)),
        resistor("RL", "vdd", "pad", 1e3),
    ]);
    let result = solve_circuit(&circuit).unwrap();
    let v = VDD * 50.0 / (1e3 + 50.0);
    assert!((result.node_voltages["pad"] - v).abs() < 1e-9);
    assert!((result_of(&result, "D2").current_through - (VDD - v) / 1e3).abs() < 1e-10);
}

/// The done-when check: a pin driving a red LED through 330 Ω, against
/// the hand calculation `I = (VDD − Vf(I)) / (330 Ω + r_high)` with the
/// LED's Shockley curve, solved by bisection.
#[test]
fn output_into_led_and_resistor_matches_hand_calculation() {
    let circuit = powered(vec![
        CircuitElement::DigitalPin(pin("D13", "pad", PinMode::PushPull, true)),
        resistor("R1", "pad", "a", 330.0),
        CircuitElement::Led(Led {
            id: "LED1".into(),
            nodes: ["a".into(), "0".into()],
            color: "red".into(),
            temperature: 300.15,
        }),
    ]);
    let result = solve_circuit(&circuit).unwrap();

    let led = led_params_from_str("red");
    let (mut lo, mut hi) = (0.0, VDD);
    for _ in 0..200 {
        let vf = 0.5 * (lo + hi);
        if diode_current(vf, &led) > (VDD - vf) / (330.0 + 50.0) {
            hi = vf;
        } else {
            lo = vf;
        }
    }
    let vf = 0.5 * (lo + hi);
    let i = (VDD - vf) / 380.0;
    assert!((result.node_voltages["a"] - vf).abs() < 1e-6, "Vf {vf}");
    assert!((result_of(&result, "LED1").current_through - i).abs() < 1e-8);
    assert!((result_of(&result, "D13").current_through + i).abs() < 1e-8);
    assert!((result.node_voltages["pad"] - (VDD - 50.0 * i)).abs() < 1e-6);
}

/// An input driven past the supply conducts into its power clamp exactly
/// as the clamp's diode model does, and past ground out of its ground
/// clamp.
#[test]
fn input_clamp_current_matches_the_diode_model_past_either_rail() {
    let clamp = DiodeModel::default();
    let params = DiodeParams {
        is: clamp.is,
        n: clamp.n,
        rs: 0.0,
        temperature: 300.15,
        breakdown: None,
    };
    for v_drive in [VDD + 0.6, VDD + 1.0, -0.6, -1.0] {
        let mut input = pin("A0", "pad", PinMode::Input, false);
        input.model.clamp = clamp;
        let circuit = powered(vec![
            CircuitElement::DigitalPin(input),
            vsource("VIN", "in", v_drive),
            resistor("RS", "in", "pad", 100.0),
        ]);
        let result = solve_circuit(&circuit).unwrap();
        let v_pad = result.node_voltages["pad"];
        // Into the pin: the power clamp's forward current above the rail,
        // less the ground clamp's below ground; the off stage leaks the rest.
        let expected = diode_current(v_pad - VDD, &params) - diode_current(-v_pad, &params)
            + (v_pad - VDD) / 1e12
            + v_pad / 1e12;
        let i = result_of(&result, "A0").current_through;
        assert!(
            // The clamps' gmin shunts carry picoamps the model leaves out.
            (i - expected).abs() < 1e-7 * expected.abs(),
            "{v_drive} V: {i} A against {expected} A"
        );
        assert!((i - (v_drive - v_pad) / 100.0).abs() < 1e-6 * i.abs());
        assert!(i.abs() > 1e-6, "{v_drive} V: the clamp conducts");
    }
}

/// An input with nothing driving it loads its net by only the off stage,
/// and a pull resistor sets it.
#[test]
fn input_pulls_and_high_z() {
    let load = |pull| {
        let mut p = pin("D4", "pad", PinMode::Input, false);
        p.pull = pull;
        let circuit = powered(vec![
            CircuitElement::DigitalPin(p),
            vsource("VS", "s", 1.0),
            resistor("RS", "s", "pad", 10e3),
        ]);
        solve_circuit(&circuit).unwrap().node_voltages["pad"]
    };
    assert!((load(Pull::None) - 1.0).abs() < 1e-6);
    // 1 V through 10 kΩ against 40 kΩ to 3.3 V, or to ground.
    let up = (1.0 / 10e3 + VDD / 40e3) / (1.0 / 10e3 + 1.0 / 40e3);
    assert!((load(Pull::Up) - up).abs() < 1e-6);
    assert!((load(Pull::Down) - 0.8).abs() < 1e-6);
}

/// Input leakage flows into the pin: 1 µA through 10 kΩ drops 10 mV.
#[test]
fn input_leakage_sinks_into_the_pin() {
    let mut p = pin("D4", "pad", PinMode::Input, false);
    p.model.leakage = 1e-6;
    let circuit = powered(vec![
        CircuitElement::DigitalPin(p),
        vsource("VS", "s", 1.0),
        resistor("RS", "s", "pad", 10e3),
    ]);
    let pad = solve_circuit(&circuit).unwrap().node_voltages["pad"];
    assert!((pad - 0.99).abs() < 1e-5, "pad at {pad} V");
}

/// Open drain pulls low, and released it's an input: the external pull-up
/// takes the net to the supply.
#[test]
fn open_drain_pulls_low_or_releases() {
    let level = |level| {
        let circuit = powered(vec![
            CircuitElement::DigitalPin(pin("SDA", "sda", PinMode::OpenDrain, level)),
            resistor("RPU", "vdd", "sda", 4.7e3),
        ]);
        solve_circuit(&circuit).unwrap().node_voltages["sda"]
    };
    assert!((level(false) - VDD * 50.0 / 4750.0).abs() < 1e-9);
    assert!((level(true) - VDD).abs() < 1e-6);
}

#[test]
fn reads_with_hysteresis_between_vil_and_vih() {
    let p = pin("A0", "pad", PinMode::Input, false);
    // 0.99 V and 2.31 V on 3.3 V.
    assert_eq!(p.reads(2.31, None), LogicLevel::High);
    assert_eq!(p.reads(0.99, None), LogicLevel::Low);
    assert_eq!(p.reads(1.5, Some(LogicLevel::High)), LogicLevel::High);
    assert_eq!(p.reads(1.5, Some(LogicLevel::Low)), LogicLevel::Low);
    assert_eq!(p.reads(1.7, None), LogicLevel::High);
    assert_eq!(p.reads(1.6, None), LogicLevel::Low);
}

#[test]
fn rejects_crossed_thresholds_and_bad_resistances() {
    let mut p = pin("A0", "pad", PinMode::Input, false);
    p.model.vih = 0.5;
    let circuit = powered(vec![
        CircuitElement::DigitalPin(p),
        resistor("R", "pad", "0", 1e3),
    ]);
    let err = solve_circuit(&circuit).unwrap_err();
    assert!(
        matches!(&err, SimError::InvalidComponent(m) if m.contains("A0") && m.contains("vih")),
        "{err}"
    );
    assert!(validate_circuit(&circuit).is_err());

    let mut p = pin("A1", "pad", PinMode::Input, false);
    p.model.r_high = 0.0;
    let err = solve_circuit(&powered(vec![CircuitElement::DigitalPin(p)])).unwrap_err();
    assert!(matches!(&err, SimError::InvalidComponent(m) if m.contains("r_high")));
}

/// Firmware toggling a pin into an RC through a session: the mode and the
/// level change between steps, the RC charges through `r_high` and the
/// series resistor, and a second pin reads the RC back with hysteresis.
#[test]
fn session_drives_a_pin_and_reads_another() {
    let mut out = pin("OUT", "pad", PinMode::PushPull, false);
    out.model.c_comp = 5e-12;
    let circuit = powered(vec![
        CircuitElement::DigitalPin(out),
        resistor("R1", "pad", "rc", 950.0),
        CircuitElement::Capacitor(Capacitor {
            id: "C1".into(),
            nodes: ["rc".into(), "0".into()],
            capacitance: 1e-6,
        }),
        CircuitElement::DigitalPin(pin("IN", "rc", PinMode::Input, false)),
    ]);
    // τ = (950 + 50) Ω · 1 µF = 1 ms.
    let tau = 1e-3;
    let mut sim = TransientSession::new(&circuit, &TransientConfig::new(10e-3, 10e-6)).unwrap();
    assert_eq!(sim.logic_level("IN"), Some(LogicLevel::Low));
    assert_eq!(sim.logic_level("R1"), None);

    sim.advance_to(1e-3).unwrap();
    assert!(sim.voltage("rc").unwrap().abs() < 1e-6);
    sim.set_param("OUT", Param::Level(true)).unwrap();

    // IN reads high once the RC reaches vih: τ·ln(VDD / (VDD − vih)).
    let vih = 0.7 * VDD;
    let hit = sim
        .run_until(10e-3, &[Event::crosses("rc", vih, Edge::Rising)])
        .unwrap()
        .expect("the RC reaches vih");
    let t_vih = 1e-3 + tau * (VDD / (VDD - vih)).ln();
    assert!(
        (hit.time - t_vih).abs() < 2e-5,
        "{} s against {t_vih} s",
        hit.time
    );
    assert_eq!(sim.logic_level("IN"), Some(LogicLevel::High));
    let i = sim.current("OUT").unwrap();
    let v_rc = sim.voltage("rc").unwrap();
    assert!((i + (VDD - v_rc) / 1e3).abs() < 1e-5, "sources {i} A");

    // Low again: through the hysteresis band IN holds high, and reads low
    // only at vil.
    sim.set_param("OUT", Param::Level(false)).unwrap();
    let vil = 0.3 * VDD;
    let mid = 0.5 * VDD;
    sim.run_until(10e-3, &[Event::crosses("rc", mid, Edge::Falling)])
        .unwrap()
        .expect("the RC falls past mid-rail");
    assert_eq!(sim.logic_level("IN"), Some(LogicLevel::High));
    sim.run_until(10e-3, &[Event::crosses("rc", vil, Edge::Falling)])
        .unwrap()
        .expect("the RC falls to vil");
    assert_eq!(sim.logic_level("IN"), Some(LogicLevel::Low));

    // Released to an input with its pull-down, the RC discharges through
    // 40 kΩ + 950 Ω instead.
    sim.set_param("OUT", Param::PinMode(PinMode::Input))
        .unwrap();
    sim.set_param("OUT", Param::Pull(Pull::Down)).unwrap();
    let v0 = sim.voltage("rc").unwrap();
    let t0 = sim.time();
    sim.advance_to(t0 + 1e-3).unwrap();
    let expected = v0 * (-1e-3_f64 / (40.95e3 * 1e-6)).exp();
    assert!((sim.voltage("rc").unwrap() - expected).abs() < 1e-3);

    let err = sim.set_param("OUT", Param::Resistance(1.0)).unwrap_err();
    assert!(matches!(err, SimError::InvalidComponent(_)));
    let err = sim.set_param("R1", Param::Level(true)).unwrap_err();
    assert!(matches!(err, SimError::InvalidComponent(_)));
}

/// A pin past its current rating, and a port past the sum its pins may
/// source.
#[test]
fn pin_and_port_current_ratings() {
    let mut circuit = powered(vec![]);
    for (k, load) in [(0, 68.0), (1, 150.0), (2, 1e3)] {
        let mut p = pin(&format!("D{k}"), &format!("p{k}"), PinMode::PushPull, true);
        p.port = Some("PORTB".into());
        circuit.components.push(CircuitElement::DigitalPin(p));
        circuit
            .components
            .push(resistor(&format!("R{k}"), &format!("p{k}"), "0", load));
        circuit.parts.insert(
            format!("R{k}"),
            Part {
                ratings: Ratings {
                    power: Some(1.0),
                    ..Ratings::default()
                },
                ..Part::default()
            },
        );
    }
    let current = |amps| Part {
        ratings: Ratings {
            current: Some(amps),
            ..Ratings::default()
        },
        ..Part::default()
    };
    for k in 0..3 {
        circuit.parts.insert(format!("D{k}"), current(0.020));
    }
    circuit.parts.insert("PORTB".into(), current(0.040));
    let result = solve_circuit(&circuit).unwrap();
    let report = check_ratings(&circuit, &result).unwrap();

    // D0 sources 3.3 V / 118 Ω = 28 mA.
    let exceeded: Vec<&str> = report.exceeded().iter().map(|c| c.part.as_str()).collect();
    assert_eq!(exceeded, ["D0", "PORTB"]);
    let port = report.for_part("PORTB").next().unwrap();
    let sum = VDD / 118.0 + VDD / 200.0 + VDD / 1050.0;
    assert!((port.worst.unwrap() - sum).abs() < 1e-9);
    assert!(report.unverified().is_empty());

    // Without the port's part its check is unverified.
    circuit.parts.remove("PORTB");
    let report = check_ratings(&circuit, &result).unwrap();
    let port = report.for_part("PORTB").next().unwrap();
    assert_eq!(port.status, RatingStatus::Unverified);
}

/// The pin round-trips through JSON with its defaults filled in.
#[cfg(feature = "serde")]
#[test]
fn deserialises_with_defaults() {
    let json = r#"{"type": "digital_pin", "id": "D13", "nodes": ["pad", "vdd", "0"],
        "mode": "push_pull", "level": true}"#;
    let element: CircuitElement = serde_json::from_str(json).unwrap();
    let expected = CircuitElement::DigitalPin(pin("D13", "pad", PinMode::PushPull, true));
    assert_eq!(element, expected);
    let back: CircuitElement =
        serde_json::from_str(&serde_json::to_string(&element).unwrap()).unwrap();
    assert_eq!(back, expected);
}

/// Each instance of a block keeps its own port: two MCUs' `PORTB`s are
/// checked apart, each against its own part.
#[test]
fn ports_are_scoped_per_instance() {
    use sindr::{Block, Instance};
    let mut p = pin("D0", "pad", PinMode::PushPull, true);
    p.port = Some("PORTB".into());
    let mut mcu = Block {
        ports: vec!["vdd".into(), "pad".into()],
        components: vec![CircuitElement::DigitalPin(p)],
        ..Block::default()
    };
    mcu.parts.insert(
        "PORTB".into(),
        Part {
            ratings: Ratings {
                current: Some(0.040),
                ..Ratings::default()
            },
            ..Part::default()
        },
    );
    let mut circuit = powered(vec![
        resistor("R1", "a", "0", 1e3),
        resistor("R2", "b", "0", 50.0),
    ]);
    circuit.blocks.insert("mcu".into(), mcu);
    circuit.instances = vec![
        Instance::new("U1", "mcu", ["vdd", "a"]),
        Instance::new("U2", "mcu", ["vdd", "b"]),
    ];
    let result = solve_circuit(&circuit).unwrap();
    let report = check_ratings(&circuit, &result).unwrap();
    let ports: Vec<(&str, RatingStatus)> = report
        .checks
        .iter()
        .filter(|c| c.part.ends_with("PORTB"))
        .map(|c| (c.part.as_str(), c.status))
        .collect();
    // U2's pin sources 3.3 V / 100 Ω = 33 mA, U1's 3.1 mA.
    assert_eq!(ports.len(), 2, "{ports:?}");
    assert!(
        ports.iter().all(|(_, s)| *s == RatingStatus::Within),
        "{ports:?}"
    );
    let u2 = report
        .checks
        .iter()
        .find(|c| c.part.starts_with("U2") && c.part.ends_with("PORTB"))
        .unwrap();
    assert!((u2.worst.unwrap() - VDD / 100.0).abs() < 1e-9);
}
