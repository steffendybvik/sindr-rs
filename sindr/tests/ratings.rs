//! Absolute-maximum ratings checks against DC and transient results.

use sindr::{
    check_ratings, solve_circuit, solve_transient, BjtKind, Block, Circuit, CircuitElement,
    Instance, Part, RatingKind, RatingStatus, Ratings, TransientConfig, TransientStart, Waveform,
};

fn vsrc(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    }
}

fn res(id: &str, p: &str, n: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [p.into(), n.into()],
        resistance: ohms,
    }
}

fn rated(ratings: Ratings) -> Part {
    Part {
        ratings,
        ..Part::default()
    }
}

#[test]
fn resistor_within_its_power_rating_passes_with_margin() {
    // 5 V across 1 kΩ: 25 mW against 250 mW, 90 % margin.
    let mut c = Circuit::new(
        "0",
        vec![vsrc("V1", "a", "0", 5.0), res("R1", "a", "0", 1e3)],
    );
    c.parts.insert(
        "R1".into(),
        rated(Ratings {
            power: Some(0.25),
            ..Ratings::default()
        }),
    );
    let report = check_ratings(&c, &solve_circuit(&c).unwrap()).unwrap();
    let check = report.for_part("R1").next().unwrap();
    assert_eq!(check.rating, RatingKind::Power);
    assert_eq!(check.status, RatingStatus::Within);
    assert_eq!(check.time, None);
    assert!((check.worst.unwrap() - 0.025).abs() < 1e-12);
    assert!((check.margin_percent.unwrap() - 90.0).abs() < 1e-9);
    assert!(report.is_safe());
}

#[test]
fn missing_rating_is_unverified_never_a_pass() {
    let c = Circuit::new(
        "0",
        vec![vsrc("V1", "a", "0", 5.0), res("R1", "a", "0", 1e3)],
    );
    let report = check_ratings(&c, &solve_circuit(&c).unwrap()).unwrap();
    let check = report.for_part("R1").next().unwrap();
    assert_eq!(check.status, RatingStatus::Unverified);
    assert_eq!(check.limit, None);
    // The value is still reported, so it can be compared by hand.
    assert!((check.worst.unwrap() - 0.025).abs() < 1e-12);
    assert!(!report.is_safe());
    // A voltage source needs no ratings, so it isn't listed.
    assert_eq!(report.for_part("V1").count(), 0);
}

#[test]
fn rating_the_part_cannot_measure_is_unverified() {
    let mut c = Circuit::new(
        "0",
        vec![vsrc("V1", "a", "0", 5.0), res("R1", "a", "0", 1e3)],
    );
    c.parts.insert(
        "R1".into(),
        rated(Ratings {
            power: Some(1.0),
            vds: Some(30.0),
            ..Ratings::default()
        }),
    );
    let report = check_ratings(&c, &solve_circuit(&c).unwrap()).unwrap();
    let vds = report
        .for_part("R1")
        .find(|c| c.rating == RatingKind::Vds)
        .unwrap();
    assert_eq!(vds.status, RatingStatus::Unverified);
    assert_eq!(vds.worst, None);
}

#[test]
fn inrush_peak_is_caught_in_the_transient() {
    // A 10 V step into a discharged 10 µF through 10 Ω: R1 takes 10 W at
    // the edge and almost nothing once the cap has charged. A 1 W rating
    // only fails on the peak, and the peak is right after the step.
    let mut c = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Pulse {
                    v1: 0.0,
                    v2: 10.0,
                    delay: 1e-5,
                    rise_time: 1e-7,
                    fall_time: 1e-7,
                    pulse_width: 1.0,
                    period: 0.0,
                }),
            },
            res("R1", "in", "out", 10.0),
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 10e-6,
            },
        ],
    );
    c.parts.insert(
        "R1".into(),
        rated(Ratings {
            power: Some(1.0),
            ..Ratings::default()
        }),
    );
    c.parts.insert(
        "C1".into(),
        rated(Ratings {
            voltage: Some(16.0),
            ..Ratings::default()
        }),
    );
    let result = solve_transient(&c, &TransientConfig::new(1e-3, 1e-6)).unwrap();
    // At the end the resistor is idle.
    let last = result
        .component_results
        .iter()
        .find(|r| r.id == "R1")
        .unwrap();
    assert!(last.power < 1e-3);

    let report = check_ratings(&c, &result).unwrap();
    let r1 = report.for_part("R1").next().unwrap();
    assert_eq!(r1.status, RatingStatus::Exceeded);
    assert!(r1.worst.unwrap() > 5.0, "peak {:?}", r1.worst);
    let t = r1.time.unwrap();
    assert!((1e-5..2e-5).contains(&t), "peak at {t}");
    assert!(r1.margin_percent.unwrap() < 0.0);

    let c1 = report.for_part("C1").next().unwrap();
    assert_eq!(c1.status, RatingStatus::Within);
    assert!((c1.worst.unwrap() - 10.0).abs() < 0.1);
}

#[test]
fn polarised_capacitor_reverse_bias_is_flagged() {
    // `+` is nodes[0]. C1 sits the right way round on +5 V; C2 is fitted
    // backwards, so its `+` is 5 V below its `-`.
    let cap = |id: &str, plus: &str, minus: &str| CircuitElement::Capacitor {
        id: id.into(),
        nodes: [plus.into(), minus.into()],
        capacitance: 100e-6,
    };
    let mut c = Circuit::new(
        "0",
        vec![
            vsrc("V1", "rail", "0", 5.0),
            res("R1", "rail", "p", 10.0),
            cap("C1", "p", "0"),
            cap("C2", "0", "p"),
        ],
    );
    let electrolytic = Ratings {
        voltage: Some(16.0),
        reverse_voltage: Some(1.0),
        ..Ratings::default()
    };
    c.parts.insert("C1".into(), rated(electrolytic));
    c.parts.insert("C2".into(), rated(electrolytic));
    let config = TransientConfig::new(1e-3, 1e-5).with_start(TransientStart::OperatingPoint);
    let report = check_ratings(&c, &solve_transient(&c, &config).unwrap()).unwrap();

    let get = |part: &str, kind| {
        report
            .for_part(part)
            .find(|c| c.rating == kind)
            .unwrap()
            .clone()
    };
    let c1 = get("C1", RatingKind::ReverseVoltage);
    assert_eq!(c1.status, RatingStatus::Within);
    assert_eq!(c1.worst, Some(0.0));
    let c2 = get("C2", RatingKind::ReverseVoltage);
    assert_eq!(c2.status, RatingStatus::Exceeded);
    assert!((c2.worst.unwrap() - 5.0).abs() < 1e-3, "{:?}", c2.worst);
    // Both see 5 V in magnitude, inside the 16 V rating.
    assert_eq!(get("C2", RatingKind::Voltage).status, RatingStatus::Within);
    let exceeded: Vec<_> = report.exceeded().iter().map(|c| &c.part).collect();
    assert_eq!(exceeded, ["C2"]);
}

#[test]
fn diode_reverse_voltage_and_forward_current() {
    // D1 reverse-biased by 30 V; D2 forward through 1 kΩ from 5 V.
    let diode = |id: &str, a: &str, k: &str| CircuitElement::Diode {
        id: id.into(),
        nodes: [a.into(), k.into()],
        temperature: 300.15,
        model: None,
    };
    let mut c = Circuit::new(
        "0",
        vec![
            vsrc("V1", "hv", "0", 30.0),
            res("R1", "hv", "k1", 1e3),
            diode("D1", "0", "k1"),
            vsrc("V2", "lv", "0", 5.0),
            res("R2", "lv", "a2", 1e3),
            diode("D2", "a2", "0"),
        ],
    );
    let diode_ratings = Ratings {
        reverse_voltage: Some(20.0),
        forward_current: Some(10e-3),
        power: Some(0.5),
        ..Ratings::default()
    };
    c.parts.insert("D1".into(), rated(diode_ratings));
    c.parts.insert("D2".into(), rated(diode_ratings));
    let report = check_ratings(&c, &solve_circuit(&c).unwrap()).unwrap();

    let get = |part: &str, kind| {
        report
            .for_part(part)
            .find(|c| c.rating == kind)
            .unwrap()
            .clone()
    };
    let d1_vr = get("D1", RatingKind::ReverseVoltage);
    assert_eq!(d1_vr.status, RatingStatus::Exceeded);
    assert!((d1_vr.worst.unwrap() - 30.0).abs() < 1e-3);
    assert_eq!(
        get("D1", RatingKind::ForwardCurrent).status,
        RatingStatus::Within
    );

    let d2_if = get("D2", RatingKind::ForwardCurrent);
    assert_eq!(d2_if.status, RatingStatus::Within);
    assert!((d2_if.worst.unwrap() - 4.3e-3).abs() < 0.3e-3);
    assert_eq!(get("D2", RatingKind::ReverseVoltage).worst, Some(0.0));

    let exceeded: Vec<_> = report.exceeded().iter().map(|c| &c.part).collect();
    assert_eq!(exceeded, ["D1"]);
}

#[test]
fn bjt_terminal_ratings() {
    // Common-emitter NPN: 12 V rail, 1 kΩ collector load, 100 kΩ base feed.
    let mut c = Circuit::new(
        "0",
        vec![
            vsrc("VCC", "vcc", "0", 12.0),
            res("RC", "vcc", "c", 1e3),
            res("RB", "vcc", "b", 100e3),
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            },
        ],
    );
    c.parts.insert(
        "Q1".into(),
        rated(Ratings {
            vce: Some(45.0),
            vcb: Some(50.0),
            ic: Some(5e-3), // well under the ~11 mA it carries
            power: Some(0.5),
            ..Ratings::default()
        }),
    );
    let result = solve_circuit(&c).unwrap();
    let report = check_ratings(&c, &result).unwrap();
    let q1: Vec<_> = report.for_part("Q1").collect();
    let kinds: Vec<_> = q1.iter().map(|c| c.rating).collect();
    assert_eq!(
        kinds,
        [
            RatingKind::Power,
            RatingKind::Vce,
            RatingKind::Vcb,
            RatingKind::Veb,
            RatingKind::Ic
        ]
    );
    let status = |k| q1.iter().find(|c| c.rating == k).unwrap().status;
    assert_eq!(status(RatingKind::Ic), RatingStatus::Exceeded);
    assert_eq!(status(RatingKind::Vce), RatingStatus::Within);
    assert_eq!(status(RatingKind::Veb), RatingStatus::Unverified);

    let bjt = &result.bjt_results[0];
    let vce = q1.iter().find(|c| c.rating == RatingKind::Vce).unwrap();
    assert!((vce.worst.unwrap() - bjt.vce.abs()).abs() < 1e-9);
    let vcb = q1.iter().find(|c| c.rating == RatingKind::Vcb).unwrap();
    assert!((vcb.worst.unwrap() - (bjt.vce - bjt.vbe).abs()).abs() < 1e-9);
}

#[test]
fn ratings_inside_blocks_follow_the_flattened_names() {
    let block = Block {
        ports: vec!["top".into(), "bot".into()],
        components: vec![res("R1", "top", "bot", 100.0)],
        parts: [(
            "R1".to_string(),
            rated(Ratings {
                power: Some(0.125),
                ..Ratings::default()
            }),
        )]
        .into(),
        ..Block::default()
    };
    let mut c = Circuit::new("0", vec![vsrc("V1", "a", "0", 12.0)]);
    c.blocks.insert("load".into(), block);
    c.instances.push(Instance::new("X1", "load", ["a", "0"]));
    let report = check_ratings(&c, &solve_circuit(&c).unwrap()).unwrap();
    let check = report.for_part("X1.R1").next().unwrap();
    assert_eq!(check.status, RatingStatus::Exceeded);
    assert!((check.worst.unwrap() - 1.44).abs() < 1e-9);
}

#[cfg(feature = "serde")]
#[test]
fn ratings_round_trip_through_json() {
    let part = rated(Ratings {
        vds: Some(60.0),
        id: Some(0.2),
        ..Ratings::default()
    });
    let json = serde_json::to_string(&part).unwrap();
    assert_eq!(json, r#"{"ratings":{"vds":60.0,"id":0.2}}"#);
    let back: Part = serde_json::from_str(&json).unwrap();
    assert_eq!(back, part);
    // A part with no ratings serialises as before.
    assert_eq!(serde_json::to_string(&Part::default()).unwrap(), "{}");
}
