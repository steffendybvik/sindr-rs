//! Absolute-maximum ratings checks against DC and transient results.

use sindr::element::{
    Bjt, Capacitor, Crystal, Diode, Igbt, Inductor, Led, Mosfet, Resistor, SchottkyDiode, Varactor,
    VoltageSource, ZenerDiode,
};
use sindr::{
    check_ratings, solve_circuit, solve_transient, BjtKind, Block, Circuit, CircuitElement,
    Instance, Part, RatingKind, RatingStatus, Ratings, TransientConfig, TransientStart, Waveform,
};

fn vsrc(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    })
}

fn res(id: &str, p: &str, n: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [p.into(), n.into()],
        resistance: ohms,
    })
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
            CircuitElement::VoltageSource(VoltageSource {
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
            }),
            res("R1", "in", "out", 10.0),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 10e-6,
            }),
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
    let cap = |id: &str, plus: &str, minus: &str| {
        CircuitElement::Capacitor(Capacitor {
            id: id.into(),
            nodes: [plus.into(), minus.into()],
            capacitance: 100e-6,
        })
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
    let diode = |id: &str, a: &str, k: &str| {
        CircuitElement::Diode(Diode {
            id: id.into(),
            nodes: [a.into(), k.into()],
            temperature: 300.15,
            model: None,
        })
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

/// An LED, a Schottky and a zener are junctions too: reverse voltage and
/// forward current read off their port, and an unrated LED or Schottky
/// needs all three of its ratings. A zener needs only power, but a part
/// may still rate the rest.
#[test]
fn led_schottky_and_zener_junction_ratings() {
    let mut c = Circuit::new(
        "0",
        vec![
            // LED forward through 330 Ω from 5 V.
            vsrc("V1", "lv", "0", 5.0),
            res("R1", "lv", "a1", 330.0),
            CircuitElement::Led(Led {
                id: "D1".into(),
                nodes: ["a1".into(), "0".into()],
                color: "red".into(),
                temperature: 300.15,
            }),
            // Schottky reverse-biased by 30 V through 1 kΩ.
            vsrc("V2", "hv", "0", 30.0),
            res("R2", "hv", "k2", 1e3),
            CircuitElement::SchottkyDiode(SchottkyDiode {
                id: "D2".into(),
                nodes: ["0".into(), "k2".into()],
                temperature: 300.15,
            }),
            // Zener in breakdown from 12 V through 1 kΩ.
            vsrc("V3", "zv", "0", 12.0),
            res("R3", "zv", "k3", 1e3),
            CircuitElement::ZenerDiode(ZenerDiode {
                id: "DZ1".into(),
                nodes: ["0".into(), "k3".into()],
                vz: 5.1,
                temperature: 300.15,
            }),
            // An unrated Schottky, forward.
            res("R4", "lv", "a4", 1e3),
            CircuitElement::SchottkyDiode(SchottkyDiode {
                id: "D4".into(),
                nodes: ["a4".into(), "0".into()],
                temperature: 300.15,
            }),
        ],
    );
    let junction = Ratings {
        reverse_voltage: Some(20.0),
        forward_current: Some(20e-3),
        power: Some(0.1),
        ..Ratings::default()
    };
    for id in ["D1", "D2", "DZ1"] {
        c.parts.insert(id.into(), rated(junction));
    }
    let result = solve_circuit(&c).unwrap();
    let report = check_ratings(&c, &result).unwrap();
    let get = |part: &str, kind| {
        report
            .for_part(part)
            .find(|c| c.rating == kind)
            .unwrap()
            .clone()
    };
    let port = |id: &str| {
        result
            .component_results
            .iter()
            .find(|r| r.id == id)
            .unwrap()
    };

    let led_if = get("D1", RatingKind::ForwardCurrent);
    assert_eq!(led_if.status, RatingStatus::Within);
    assert_eq!(led_if.worst, Some(port("D1").current_through));
    assert!((8e-3..10e-3).contains(&led_if.worst.unwrap()), "{led_if:?}");
    assert_eq!(get("D1", RatingKind::ReverseVoltage).worst, Some(0.0));

    let schottky_vr = get("D2", RatingKind::ReverseVoltage);
    assert_eq!(schottky_vr.status, RatingStatus::Exceeded);
    assert_eq!(schottky_vr.worst, Some(-port("D2").voltage_across));
    assert!((schottky_vr.worst.unwrap() - 30.0).abs() < 0.1);
    assert_eq!(get("D2", RatingKind::ForwardCurrent).worst, Some(0.0));

    // Held near vz, well inside 20 V, with no forward current.
    let zener_vr = get("DZ1", RatingKind::ReverseVoltage);
    assert_eq!(zener_vr.status, RatingStatus::Within);
    assert!((zener_vr.worst.unwrap() - 5.1).abs() < 0.2, "{zener_vr:?}");
    assert_eq!(get("DZ1", RatingKind::ForwardCurrent).worst, Some(0.0));

    let unverified: Vec<_> = report
        .unverified()
        .iter()
        .filter(|c| c.part == "D4")
        .map(|c| c.rating)
        .collect();
    assert_eq!(
        unverified,
        [
            RatingKind::Power,
            RatingKind::ReverseVoltage,
            RatingKind::ForwardCurrent
        ]
    );
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
            CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }),
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

#[test]
fn every_rating_kind_has_its_si_unit() {
    use RatingKind::*;
    for (kind, unit) in [
        (Power, "W"),
        (Voltage, "V"),
        (Current, "A"),
        (ReverseVoltage, "V"),
        (ForwardCurrent, "A"),
        (Vce, "V"),
        (Vcb, "V"),
        (Veb, "V"),
        (Ic, "A"),
        (Vds, "V"),
        (Vgs, "V"),
        (Id, "A"),
        (Tj, "K"),
    ] {
        assert_eq!(kind.unit(), unit, "{kind:?}");
    }
}

#[test]
fn mosfet_terminal_ratings() {
    // NMOS common source: gate at 3 V, 10 V rail through 1 kΩ.
    let mut c = Circuit::new(
        "0",
        vec![
            vsrc("VG", "g", "0", 3.0),
            vsrc("VDD", "vdd", "0", 10.0),
            res("RD", "vdd", "d", 1e3),
            CircuitElement::Mosfet(Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind: sindr::MosfetKind::Nmos,
                params: sindr_devices::mosfet::MosfetParams::default_nmos(),
                parasitic_caps: None,
                body_diode: None,
            }),
        ],
    );
    c.parts.insert(
        "M1".into(),
        rated(Ratings {
            vds: Some(60.0),
            vgs: Some(2.0), // under the 3 V on the gate
            id: Some(1.0),
            ..Ratings::default()
        }),
    );
    let result = solve_circuit(&c).unwrap();
    let report = check_ratings(&c, &result).unwrap();
    let m1: Vec<_> = report.for_part("M1").collect();
    let kinds: Vec<_> = m1.iter().map(|c| c.rating).collect();
    assert_eq!(
        kinds,
        [
            RatingKind::Power,
            RatingKind::Vds,
            RatingKind::Vgs,
            RatingKind::Id
        ]
    );
    let check = |k| *m1.iter().find(|c| c.rating == k).unwrap();
    let mos = &result.mosfet_results[0];
    assert!((check(RatingKind::Vds).worst.unwrap() - mos.vds).abs() < 1e-12);
    assert!((check(RatingKind::Vgs).worst.unwrap() - 3.0).abs() < 1e-12);
    assert!((check(RatingKind::Id).worst.unwrap() - mos.id_current).abs() < 1e-12);
    assert_eq!(check(RatingKind::Vgs).status, RatingStatus::Exceeded);
    assert_eq!(check(RatingKind::Vds).status, RatingStatus::Within);
    // No power rating set: needed, so unverified, and listed as such.
    assert_eq!(check(RatingKind::Power).status, RatingStatus::Unverified);
    let unverified: Vec<_> = report
        .unverified()
        .iter()
        .map(|c| (c.part.as_str(), c.rating))
        .collect();
    assert!(
        unverified.contains(&("M1", RatingKind::Power)),
        "{unverified:?}"
    );
    assert!(
        unverified.contains(&("RD", RatingKind::Power)),
        "{unverified:?}"
    );
    assert!(report
        .unverified()
        .iter()
        .all(|c| c.status == RatingStatus::Unverified));
}

/// A zero rating has no margin to divide by: anything above it is over,
/// and exactly zero is within.
#[test]
fn zero_rating_has_no_margin() {
    let mut c = Circuit::new(
        "0",
        vec![
            vsrc("V1", "a", "0", 1.0),
            res("R1", "a", "0", 1e3),
            // Both ends at 1 V: no current, no power.
            res("R2", "a", "a2", 1e3),
            vsrc("V2", "a2", "0", 1.0),
        ],
    );
    for id in ["R1", "R2"] {
        c.parts.insert(
            id.into(),
            rated(Ratings {
                power: Some(0.0),
                ..Ratings::default()
            }),
        );
    }
    let result = solve_circuit(&c).unwrap();
    let report = check_ratings(&c, &result).unwrap();
    let power = |id| report.for_part(id).next().unwrap();
    assert_eq!(power("R1").status, RatingStatus::Exceeded);
    assert_eq!(power("R1").margin_percent, None);
    assert_eq!(power("R2").status, RatingStatus::Within);
    assert_eq!(power("R2").margin_percent, None);
}

/// An inductor's needed rating is its current, checked at every step.
#[test]
fn inductor_current_rating_sees_the_settled_current() {
    let mut c = Circuit::new(
        "0",
        vec![
            vsrc("V1", "a", "0", 1.0),
            res("R1", "a", "b", 10.0),
            CircuitElement::Inductor(Inductor {
                id: "L1".into(),
                nodes: ["b".into(), "0".into()],
                inductance: 1e-3,
                saturation: None,
            }),
        ],
    );
    c.parts.insert(
        "L1".into(),
        rated(Ratings {
            current: Some(0.2),
            ..Ratings::default()
        }),
    );
    // τ = L/R = 100 µs; 2 ms is 20 τ, so the worst is the settled 100 mA.
    let result = solve_transient(&c, &TransientConfig::new(2e-3, 10e-6)).unwrap();
    let report = check_ratings(&c, &result).unwrap();
    let l1: Vec<_> = report.for_part("L1").collect();
    assert_eq!(l1.len(), 1);
    assert_eq!(l1[0].rating, RatingKind::Current);
    assert!((l1[0].worst.unwrap() - 0.1).abs() < 1e-6, "{:?}", l1[0]);
    assert!((l1[0].margin_percent.unwrap() - 50.0).abs() < 1e-3);
}

/// A crystal's needed rating is its drive level, its power.
#[test]
fn crystal_needs_a_drive_level_rating() {
    let c = Circuit::new(
        "0",
        vec![
            vsrc("V1", "a", "0", 1.0),
            res("R1", "a", "b", 1e3),
            CircuitElement::Crystal(Crystal {
                id: "X1".into(),
                nodes: ["b".into(), "0".into()],
                params: sindr::circuit::CrystalParams::from_datasheet(32768.0, 35e3, 1.5e-12),
            }),
            res("RB", "b", "0", 1e6),
        ],
    );
    let result = sindr::solve_op(&c).unwrap();
    let report = check_ratings(&c, &result).unwrap();
    let x1: Vec<_> = report.for_part("X1").map(|c| c.rating).collect();
    assert_eq!(x1, [RatingKind::Power]);
}

/// Which ratings a Zener, a varactor and an IGBT need, and what they read.
#[test]
fn zener_varactor_and_igbt_needed_ratings() {
    let c = Circuit::new(
        "0",
        vec![
            vsrc("V1", "in", "0", 12.0),
            res("R1", "in", "k", 1e3),
            CircuitElement::ZenerDiode(ZenerDiode {
                id: "DZ1".into(),
                nodes: ["0".into(), "k".into()],
                vz: 5.1,
                temperature: 300.15,
            }),
            // Reverse-biased 5 V through 1 MΩ.
            vsrc("V2", "c", "0", 5.0),
            res("R2", "c", "cv", 1e6),
            CircuitElement::Varactor(Varactor {
                id: "CV1".into(),
                nodes: ["0".into(), "cv".into()],
                params: sindr::VaractorParams::default(),
            }),
            // IGBT: gate at 6 V, 15 V through 100 Ω.
            vsrc("VG", "g", "0", 6.0),
            vsrc("VCC", "vcc", "0", 15.0),
            res("RL", "vcc", "col", 100.0),
            CircuitElement::Igbt(Igbt {
                id: "Z1".into(),
                nodes: ["g".into(), "col".into(), "0".into()],
                params: sindr::IgbtParams::default(),
            }),
        ],
    );
    let result = sindr::solve_op(&c).unwrap();
    let report = check_ratings(&c, &result).unwrap();
    let kinds = |id| report.for_part(id).map(|c| c.rating).collect::<Vec<_>>();
    assert_eq!(kinds("DZ1"), [RatingKind::Power]);
    assert_eq!(kinds("CV1"), [RatingKind::ReverseVoltage]);
    assert_eq!(
        kinds("Z1"),
        [
            RatingKind::Power,
            RatingKind::Vce,
            RatingKind::Ic,
            RatingKind::Vgs
        ]
    );
    let worst = |id, kind| {
        report
            .for_part(id)
            .find(|c| c.rating == kind)
            .unwrap()
            .worst
            .unwrap()
    };
    let port = |id: &str| {
        result
            .component_results
            .iter()
            .find(|r| r.id == id)
            .unwrap()
    };
    assert_eq!(worst("DZ1", RatingKind::Power), port("DZ1").power);
    // Open at DC: the whole 5 V, less the 1 TΩ shunt's 5 µV across 1 MΩ.
    assert!((worst("CV1", RatingKind::ReverseVoltage) - 5.0).abs() < 1e-4);
    assert_eq!(worst("Z1", RatingKind::Vgs), 6.0);
    let vce = result.node_voltages["col"];
    assert!((worst("Z1", RatingKind::Vce) - vce).abs() < 1e-12);
    assert_eq!(
        worst("Z1", RatingKind::Ic),
        port("Z1").current_through.abs()
    );
}
