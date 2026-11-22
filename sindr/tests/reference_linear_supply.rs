//! Reference design: a 12 V → 5 V → 3.3 V linear supply at 500 mA.
//!
//! # Reference designs
//!
//! A `reference_<design>.rs` suite builds one complete, realistic circuit
//! the way a designer would hand it over, then checks it end to end with
//! sindr's own tools:
//!
//! - **One builder** (`design()`) returns the nominal circuit with every
//!   part's datasheet data on its [`Part`]: absolute-maximum [`Ratings`]
//!   and, where the part runs hot, its [`Thermal`] path. Variants are made
//!   by editing that circuit, never by building a second one.
//! - **Nominal tests** compare the simulation against a hand calculation
//!   written out in the test, with the tolerance stated and the reason for
//!   any gap between the two (load regulation, quiescent current).
//! - **Broken variants** (`broken_*`) each break one thing a real board
//!   gets wrong and assert the *specific* violation sindr reports: which
//!   part, which rating or spec, and the value. A variant sindr can't
//!   flag says so in its doc and in `LIMITATIONS.md` rather than asserting
//!   something the model doesn't do.
//!
//! # This design
//!
//! ```text
//!  12 V ──┬── U1 7805 ──┬──┬── U2 AMS1117-3.3 ──┬── 3.3 V
//!  (V1)   C1           C2  C3                    C4   RL 6.6 Ω (500 mA)
//!         0.33 µF   0.1 µF 10 µF              22 µF
//! ```
//!
//! The load sits on the 3.3 V rail (6.6 Ω, 500 mA at 3.3 V), so both
//! regulators carry it. Capacitors are the datasheets' typical values: the
//! 7805's 0.33 µF in and 0.1 µF out, the AMS1117's 10 µF in and 22 µF
//! (tantalum) out. The 7805 is a TO-220 on a 10 K/W heatsink (pad
//! included), the AMS1117 a SOT-223 with its tab on about 1 in² of copper.
//! Ambient is 25 °C.
//!
//! What sindr's regulator model can't show: it is a static characteristic
//! with no control loop, so the output capacitor's value and ESR don't
//! affect stability (an AMS1117 without its output capacitor oscillates on
//! a real board, not here), and the load-step response is the same with
//! or without it. Start-up follows the input ramp with no soft-start.

use sindr::measure::{Signal, SpecStatus, Stat};
use sindr::regulator::RegulatorParams;
use sindr::{
    check_ratings, check_specs, solve_op, solve_transient, Circuit, CircuitElement, Measure,
    Measurement, Part, RatingKind, RatingStatus, Ratings, SimulationResult, Spec, Thermal,
    TransientConfig, TransientStart, Waveform,
};

/// 25 °C (K).
const AMBIENT: f64 = 298.15;
/// 125 °C (K): the operating junction limit of both regulators.
const TJ_MAX: f64 = 398.15;
/// The 3.3 V load (Ω): 500 mA at 3.3 V.
const R_LOAD: f64 = 6.6;

/// 7805 in TO-220: Rθjc 5 K/W.
const U1_RTH_JC: f64 = 5.0;
/// Heatsink plus interface pad, case to ambient (K/W).
const U1_RTH_CA: f64 = 10.0;
/// TO-220 free-standing, junction to ambient (K/W).
const U1_RTH_JA: f64 = 50.0;
/// AMS1117 in SOT-223: Rθjc 15 K/W.
const U2_RTH_JC: f64 = 15.0;
/// SOT-223 tab to ambient through about 1 in² of copper (K/W).
const U2_RTH_CA: f64 = 45.0;

fn cap(id: &str, node: &str, farads: f64) -> CircuitElement {
    CircuitElement::Capacitor {
        id: id.into(),
        nodes: [node.into(), "0".into()],
        capacitance: farads,
    }
}

fn regulator(
    id: &str,
    input: &str,
    output: &str,
    volts: f64,
    p: RegulatorParams,
) -> CircuitElement {
    CircuitElement::VoltageRegulator {
        id: id.into(),
        nodes: [input.into(), output.into(), "0".into()],
        voltage: volts,
        model: Some(p),
    }
}

fn rated(ratings: Ratings) -> Part {
    Part {
        ratings,
        ..Part::default()
    }
}

fn heated(ratings: Ratings, thermal: Thermal) -> Part {
    Part {
        ratings,
        thermal: Some(Thermal {
            ambient: AMBIENT,
            ..thermal
        }),
        ..Part::default()
    }
}

fn cap_rating(volts: f64) -> Part {
    rated(Ratings {
        voltage: Some(volts),
        ..Ratings::default()
    })
}

/// The nominal supply, fed a steady 12 V.
fn design() -> Circuit {
    let mut c = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["vin".into(), "0".into()],
                voltage: 12.0,
                waveform: None,
            },
            cap("C1", "vin", 0.33e-6),
            regulator("U1", "vin", "v5", 5.0, RegulatorParams::lm7805()),
            cap("C2", "v5", 0.1e-6),
            cap("C3", "v5", 10e-6),
            regulator("U2", "v5", "v3v3", 3.3, RegulatorParams::ams1117()),
            cap("C4", "v3v3", 22e-6),
            CircuitElement::Resistor {
                id: "RL".into(),
                nodes: ["v3v3".into(), "0".into()],
                resistance: R_LOAD,
            },
        ],
    );
    let parts = [
        // 35 V maximum input; output current internally limited at the
        // 2.2 A peak; 15 W in TO-220 with an infinite heatsink.
        (
            "U1",
            heated(
                Ratings {
                    voltage: Some(35.0),
                    current: Some(2.2),
                    power: Some(15.0),
                    tj_max: Some(TJ_MAX),
                    ..Ratings::default()
                },
                Thermal::on_heatsink(U1_RTH_JC, U1_RTH_CA),
            ),
        ),
        // 15 V maximum input, 1 A rated output, and the dissipation that
        // takes the junction to 125 °C from 25 °C through 60 K/W.
        (
            "U2",
            heated(
                Ratings {
                    voltage: Some(15.0),
                    current: Some(1.0),
                    power: Some((TJ_MAX - AMBIENT) / (U2_RTH_JC + U2_RTH_CA)),
                    tj_max: Some(TJ_MAX),
                    ..Ratings::default()
                },
                Thermal::on_heatsink(U2_RTH_JC, U2_RTH_CA),
            ),
        ),
        ("C1", cap_rating(50.0)),
        ("C2", cap_rating(50.0)),
        ("C3", cap_rating(16.0)),
        ("C4", cap_rating(10.0)),
        // 1.65 W in a 3 W resistor.
        (
            "RL",
            rated(Ratings {
                power: Some(3.0),
                ..Ratings::default()
            }),
        ),
    ];
    for (id, part) in parts {
        c.parts.insert(id.into(), part);
    }
    c
}

fn element_mut<'a>(c: &'a mut Circuit, id: &str) -> &'a mut CircuitElement {
    c.components.iter_mut().find(|e| e.id() == id).unwrap()
}

fn set_input(c: &mut Circuit, volts: f64, ramp: Option<Waveform>) {
    if let CircuitElement::VoltageSource {
        voltage, waveform, ..
    } = element_mut(c, "V1")
    {
        *voltage = volts;
        *waveform = ramp;
    }
}

fn without_capacitors(mut c: Circuit) -> Circuit {
    c.components
        .retain(|e| !matches!(e, CircuitElement::Capacitor { .. }));
    for id in ["C1", "C2", "C3", "C4"] {
        c.parts.remove(id);
    }
    c
}

fn port<'a>(r: &'a SimulationResult, id: &str) -> &'a sindr::ComponentResult {
    r.component_results.iter().find(|c| c.id == id).unwrap()
}

/// 7805 quiescent current (A).
fn iq_u1() -> f64 {
    RegulatorParams::lm7805().quiescent_current
}

/// AMS1117 quiescent current (A).
fn iq_u2() -> f64 {
    RegulatorParams::ams1117().quiescent_current
}

/// Each rail inside its regulator's output tolerance: 4.8–5.2 V for the
/// 7805, ±2 % for the AMS1117-3.3, at the end of the result.
fn rail_specs() -> [Spec; 2] {
    let rail = |name: &str, node: &str| {
        Measurement::new(name, Measure::stat(Signal::v(node), Stat::Final))
    };
    [
        Spec::new(rail("5v rail", "v5")).between(4.8, 5.2),
        Spec::new(rail("3v3 rail", "v3v3")).between(3.3 * 0.98, 3.3 * 1.02),
    ]
}

fn rating(c: &Circuit, r: &SimulationResult, part: &str, kind: RatingKind) -> sindr::RatingCheck {
    check_ratings(c, r)
        .unwrap()
        .for_part(part)
        .find(|k| k.rating == kind)
        .unwrap()
        .clone()
}

// ---------------------------------------------------------------------------
// Nominal design
// ---------------------------------------------------------------------------

#[test]
fn rails_regulate_and_every_rating_holds() {
    let c = design();
    let r = solve_op(&c).unwrap();
    let specs = check_specs(&r, &rail_specs()).with_ratings(check_ratings(&c, &r).unwrap());
    assert!(specs.passed(), "{specs:#?}");
    // 5 V less 0.5 A through the 7805's 67 mΩ load regulation, 3.3 V less
    // 0.5 A through the AMS1117's 16.5 mΩ.
    assert!(
        (r.node_voltages["v5"] - 4.967).abs() < 2e-3,
        "{}",
        r.node_voltages["v5"]
    );
    assert!(
        (r.node_voltages["v3v3"] - 3.292).abs() < 2e-3,
        "{}",
        r.node_voltages["v3v3"]
    );
    // The load draws 500 mA (less the 0.25 % the rail sits low).
    let i_load = port(&r, "RL").current_through;
    assert!((i_load - 0.5).abs() < 0.5 * 0.005, "{i_load}");
}

#[test]
fn every_regulator_has_its_input_and_output_capacitors() {
    // Datasheet minimums: 7805 0.33 µF in, 0.1 µF out; AMS1117 10 µF in,
    // 22 µF out. Capacitance to ground on each pin's node.
    let c = design();
    let to_ground = |node: &str| -> f64 {
        c.components
            .iter()
            .filter_map(|e| match e {
                CircuitElement::Capacitor {
                    nodes, capacitance, ..
                } if nodes[0] == node && nodes[1] == "0" => Some(*capacitance),
                _ => None,
            })
            .sum()
    };
    let mut seen = 0;
    for e in &c.components {
        if let CircuitElement::VoltageRegulator { id, nodes, .. } = e {
            let (cin, cout) = match id.as_str() {
                "U1" => (0.33e-6, 0.1e-6),
                "U2" => (10e-6, 22e-6),
                other => panic!("unexpected regulator {other}"),
            };
            // The 5 V rail's 10.1 µF counts as the 7805's output and the
            // AMS1117's input both.
            assert!(to_ground(&nodes[0]) >= cin, "{id} input");
            assert!(to_ground(&nodes[1]) >= cout, "{id} output");
            seen += 1;
        }
    }
    assert_eq!(seen, 2);
}

/// 7805: (12 − 5)·0.5 + 12·Iq = 3.55 W by hand. The simulated rail sits
/// 33 mV low and the 7805 also carries the AMS1117's 5 mA quiescent, so
/// the simulation lands about 1.2 % above; 2 % tolerance. On its heatsink
/// the junction runs at 25 °C + P·(Rθjc + Rθca) ≈ 79 °C.
#[test]
fn lm7805_dissipation_and_junction_on_its_heatsink() {
    let c = design();
    let r = solve_op(&c).unwrap();
    let u1 = port(&r, "U1");
    let hand = (12.0 - 5.0) * 0.5 + 12.0 * iq_u1();
    assert!(
        (u1.power - hand).abs() < 0.02 * hand,
        "{} vs {hand}",
        u1.power
    );
    // Exactly the dissipation formula on the simulated operating point.
    let exact = u1.voltage_across * u1.current_through + 12.0 * iq_u1();
    assert!((u1.power - exact).abs() < 1e-9, "{} vs {exact}", u1.power);

    let tj = r.junction_temperatures["U1"];
    let tj_hand = AMBIENT + hand * (U1_RTH_JC + U1_RTH_CA);
    assert!((tj - tj_hand).abs() < 1.0, "Tj {tj} K vs {tj_hand} K");
    assert!((tj - (AMBIENT + u1.power * 15.0)).abs() < 0.01, "{tj}");
    let check = rating(&c, &r, "U1", RatingKind::Tj);
    assert_eq!(check.status, RatingStatus::Within);
    // About 46 K of margin under 125 °C.
    assert!(check.margin_percent.unwrap() > 10.0, "{check:?}");
    assert_eq!(
        rating(&c, &r, "U1", RatingKind::Power).status,
        RatingStatus::Within
    );
}

/// AMS1117: (5 − 3.3)·0.5 + 5·Iq = 0.875 W by hand. The simulated 5 V
/// rail sits 33 mV low and the load draws 0.25 % under 500 mA, so the
/// simulation is about 1.8 % under; 3 % tolerance. Junction ≈ 25 °C +
/// 0.86 W·60 K/W ≈ 77 °C.
#[test]
fn ams1117_dissipation_and_junction_on_its_copper() {
    let c = design();
    let r = solve_op(&c).unwrap();
    let u2 = port(&r, "U2");
    let hand = (5.0 - 3.3) * 0.5 + 5.0 * iq_u2();
    assert!(
        (u2.power - hand).abs() < 0.03 * hand,
        "{} vs {hand}",
        u2.power
    );

    let tj = r.junction_temperatures["U2"];
    let tj_hand = AMBIENT + hand * (U2_RTH_JC + U2_RTH_CA);
    assert!((tj - tj_hand).abs() < 2.0, "Tj {tj} K vs {tj_hand} K");
    for kind in [
        RatingKind::Tj,
        RatingKind::Power,
        RatingKind::Voltage,
        RatingKind::Current,
    ] {
        let check = rating(&c, &r, "U2", kind);
        assert_eq!(check.status, RatingStatus::Within, "{check:?}");
    }
}

/// 12 V ramped up over 1 ms. Each regulator's output follows its input
/// down by the dropout until it regulates, so the rails rise at the input's
/// 12 V/ms and the output capacitors draw C·dV/dt: 22 µF × 12 V/ms =
/// 264 mA into C4, 10.1 µF × 12 V/ms = 121 mA into C2 + C3. Neither
/// regulator reaches its current limit, and no rating is crossed on the
/// way up.
///
/// The peak is checked too. Where the AMS1117 leaves 0 V its output must
/// not lag the corner and catch up in one short step, which reads as a
/// spike up to its 1.1 A limit. What's left is Gear-2 meeting a corner in
/// dV/dt: one step about 30 % over C·dV/dt.
#[test]
fn soft_ramp_inrush_is_c_dv_dt() {
    let mut c = design();
    set_input(
        &mut c,
        0.0,
        Some(Waveform::Pwl {
            points: vec![(0.0, 0.0), (1e-3, 12.0)],
        }),
    );
    let r = solve_transient(&c, &TransientConfig::new(2e-3, 5e-6)).unwrap();
    // Over the whole start-up, or over a window of it.
    let measure = |id: &str, stat: Stat, window: Option<(f64, f64)>| {
        let m = Measurement::new(id, Measure::stat(Signal::i(id), stat));
        match window {
            Some((from, to)) => m.window(from, to),
            None => m,
        }
        .on(&r)
        .unwrap()
    };
    let ramping = Some((0.3e-3, 0.5e-3));
    let dv_dt = 12.0 / 1e-3;
    // While both rails ramp (input 3.6–6 V): the steady charging current.
    let c4 = measure("C4", Stat::Avg, ramping);
    assert!(
        (c4 - 22e-6 * dv_dt).abs() < 0.02 * 22e-6 * dv_dt,
        "C4 {c4} A"
    );
    let c23 = measure("C2", Stat::Avg, ramping) + measure("C3", Stat::Avg, ramping);
    assert!(
        (c23 - 10.1e-6 * dv_dt).abs() < 0.02 * 10.1e-6 * dv_dt,
        "C2+C3 {c23} A"
    );
    // And the peak, corner included.
    let c4_peak = measure("C4", Stat::Max, None);
    assert!(c4_peak < 1.35 * 22e-6 * dv_dt, "C4 peak {c4_peak} A");
    // The 7805 carries the AMS1117's capacitor and load as well: about
    // 0.12 + 0.26 + 0.5 ≈ 0.89 A just before the 3.3 V rail regulates,
    // well under its 2.2 A limit.
    let u1 = measure("U1", Stat::Max, None);
    assert!((u1 - 0.89).abs() < 0.02, "U1 peak {u1} A");
    let u2 = measure("U2", Stat::Max, None);
    assert!(
        u2 < RegulatorParams::ams1117().current_limit.unwrap(),
        "U2 peak {u2} A"
    );

    // Settled in regulation, and safe all the way up.
    let specs = check_specs(&r, &rail_specs()).with_ratings(check_ratings(&c, &r).unwrap());
    assert!(specs.passed(), "{specs:#?}");
}

/// Hot-plugged: 12 V in 10 µs. C·dV/dt would be 26 A into C4 and 12 A into
/// C2 + C3, so each regulator's current limit binds instead: the AMS1117
/// holds at its 1.1 A and the 7805 at its 2.2 A.
#[test]
fn hot_plug_inrush_is_held_at_the_current_limits() {
    let mut c = design();
    set_input(
        &mut c,
        0.0,
        Some(Waveform::Pwl {
            points: vec![(0.0, 0.0), (10e-6, 12.0)],
        }),
    );
    let r = solve_transient(&c, &TransientConfig::new(400e-6, 1e-6)).unwrap();
    let tran = r.transient.as_ref().unwrap();
    let peak = |id: &str| {
        tran.timesteps
            .iter()
            .map(|s| port_in(s, id).current_through)
            .fold(f64::MIN, f64::max)
    };
    let u1_limit = RegulatorParams::lm7805().current_limit.unwrap();
    let u2_limit = RegulatorParams::ams1117().current_limit.unwrap();
    let (u1, u2) = (peak("U1"), peak("U2"));
    assert!((u1 - u1_limit).abs() < 0.02 * u1_limit, "U1 {u1} A");
    assert!((u2 - u2_limit).abs() < 0.02 * u2_limit, "U2 {u2} A");
    // The limited charge current takes the 3.3 V rail up in about
    // 22 µF × 3.3 V / (1.1 A − 0.5 A) ≈ 0.12 ms; by 0.4 ms it regulates.
    assert!(check_specs(&r, &rail_specs()).passed());
}

fn port_in<'a>(s: &'a sindr::TimestepSnapshot, id: &str) -> &'a sindr::ComponentResult {
    s.component_results.iter().find(|c| c.id == id).unwrap()
}

// ---------------------------------------------------------------------------
// Broken variants
// ---------------------------------------------------------------------------

/// The 7805 with no heatsink: Rθja 50 K/W puts its 3.6 W junction at about
/// 25 °C + 3.6 W × 50 K/W ≈ 205 °C, past 125 °C. Its power rating (15 W,
/// for an ideal heatsink) is still met: only the junction check catches it.
#[test]
fn broken_no_heatsink_overheats_the_7805() {
    let mut c = design();
    c.parts.get_mut("U1").unwrap().thermal = Some(Thermal {
        ambient: AMBIENT,
        ..Thermal::junction_to_ambient(U1_RTH_JA)
    });
    let r = solve_op(&c).unwrap();
    let report = check_ratings(&c, &r).unwrap();
    let exceeded = report.exceeded();
    assert_eq!(exceeded.len(), 1, "{exceeded:#?}");
    let tj = exceeded[0];
    assert_eq!((tj.part.as_str(), tj.rating), ("U1", RatingKind::Tj));
    let p = port(&r, "U1").power;
    let expected = AMBIENT + p * U1_RTH_JA;
    assert!((tj.worst.unwrap() - expected).abs() < 0.01, "{tj:?}");
    assert!((tj.worst.unwrap() - 478.0).abs() < 2.0, "{tj:?}");
    assert!(tj.margin_percent.unwrap() < -15.0, "{tj:?}");
    assert_eq!(
        rating(&c, &r, "U1", RatingKind::Power).status,
        RatingStatus::Within
    );
    // The rails themselves are fine: the model has no thermal shutdown.
    assert!(check_specs(&r, &rail_specs()).passed());
}

/// The AMS1117 wired straight to 12 V. Its 12 V input is inside the
/// 15 V absolute maximum (20 % margin), so the input-voltage check passes,
/// correctly. What kills it is heat: (12 − 3.3)·0.5 + 12·5 mA ≈ 4.4 W,
/// over the 1.67 W its copper can shed, with the junction near 290 °C.
#[test]
fn broken_ams1117_fed_12_v_overheats_but_is_inside_its_input_rating() {
    let mut c = design();
    if let CircuitElement::VoltageRegulator { nodes, .. } = element_mut(&mut c, "U2") {
        nodes[0] = "vin".into();
    }
    let r = solve_op(&c).unwrap();
    assert!((r.node_voltages["v3v3"] - 3.3).abs() < 0.02);

    let vin = rating(&c, &r, "U2", RatingKind::Voltage);
    assert_eq!(vin.status, RatingStatus::Within, "{vin:?}");
    assert!((vin.worst.unwrap() - 12.0).abs() < 1e-9, "{vin:?}");
    assert!((vin.margin_percent.unwrap() - 20.0).abs() < 1e-6, "{vin:?}");

    let power = rating(&c, &r, "U2", RatingKind::Power);
    assert_eq!(power.status, RatingStatus::Exceeded, "{power:?}");
    let hand = (12.0 - 3.3) * 0.5 + 12.0 * iq_u2();
    assert!(
        (power.worst.unwrap() - hand).abs() < 0.03 * hand,
        "{power:?}"
    );

    let tj = rating(&c, &r, "U2", RatingKind::Tj);
    assert_eq!(tj.status, RatingStatus::Exceeded, "{tj:?}");
    let tj_hand = AMBIENT + hand * (U2_RTH_JC + U2_RTH_CA);
    assert!(
        (tj.worst.unwrap() - tj_hand).abs() < 0.03 * (tj_hand - AMBIENT),
        "{tj:?}"
    );

    // A 16 V input would cross the input rating too.
    set_input(&mut c, 16.0, None);
    let r = solve_op(&c).unwrap();
    let vin = rating(&c, &r, "U2", RatingKind::Voltage);
    assert_eq!(vin.status, RatingStatus::Exceeded, "{vin:?}");
}

/// 6 V in: the 7805 needs 2 V of headroom, so it's in dropout and the
/// "5 V" rail is 6 − 2 = 4 V. The AMS1117 then needs 3.3 + 1.1 = 4.4 V
/// and has 4 V, so the 3.3 V rail drops out too, to 4 − 1.1 = 2.9 V. A
/// real 7805's dropout at 0.5 A is nearer 1.7 V, which still leaves the
/// AMS1117 short (4.3 V). Both rail specs fail; no rating is crossed.
#[test]
fn broken_6_v_input_drops_out_both_rails() {
    let mut c = design();
    set_input(&mut c, 6.0, None);
    let r = solve_op(&c).unwrap();
    let report = check_specs(&r, &rail_specs()).with_ratings(check_ratings(&c, &r).unwrap());
    assert!(!report.passed());
    let v5 = report.get("5v rail").unwrap();
    assert_eq!(v5.status, SpecStatus::Fail, "{v5:?}");
    assert!((v5.value.unwrap() - 4.0).abs() < 1e-6, "{v5:?}");
    let v3 = report.get("3v3 rail").unwrap();
    assert_eq!(v3.status, SpecStatus::Fail, "{v3:?}");
    assert!((v3.value.unwrap() - 2.9).abs() < 1e-6, "{v3:?}");
    assert!(check_ratings(&c, &r).unwrap().is_safe());

    // The 3.3 V rail is back in spec from about 6.35 V in (3.234 V + 1.1 V
    // + 2 V); the 5 V rail needs 6.8 V.
    set_input(&mut c, 6.5, None);
    let r = solve_op(&c).unwrap();
    let report = check_specs(&r, &rail_specs());
    assert_eq!(report.get("5v rail").unwrap().status, SpecStatus::Fail);
    assert_eq!(report.get("3v3 rail").unwrap().status, SpecStatus::Pass);
}

/// The input fed through a series blocking diode and cut at 50 µs. With
/// the capacitors fitted, their charge carries the load: C1 holds the 7805
/// in regulation for about 0.33 µF × 4 V / 0.5 A ≈ 3 µs, then C1–C3 hold
/// the 5 V rail above the AMS1117's 4.4 V dropout for another ≈ 12 µs,
/// and C4 adds a few µs more before the 3.3 V rail leaves its spec.
/// Without them the rails collapse the moment the input goes.
///
/// That hold-up is all the capacitors show here. sindr's regulator has no
/// control loop, so a missing output capacitor doesn't make it oscillate
/// and doesn't change its load-step response (see the module docs).
#[test]
fn broken_capacitors_missing_lose_the_hold_up() {
    let cut = 50e-6;
    let interrupted = |c: Circuit| {
        let mut c = c;
        set_input(
            &mut c,
            0.0,
            Some(Waveform::Pwl {
                points: vec![(0.0, 12.0), (cut, 12.0), (cut + 0.1e-6, 0.0)],
            }),
        );
        element_mut(&mut c, "V1").nodes_mut()[0] = "src".into();
        c.components.push(CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["src".into(), "vin".into()],
            temperature: 300.15,
            model: None,
        });
        let config =
            TransientConfig::new(cut + 50e-6, 0.1e-6).with_start(TransientStart::OperatingPoint);
        let tran = solve_transient(&c, &config).unwrap().transient.unwrap();
        // How long after the cut the 3.3 V rail stays in its ±2 % spec.
        tran.timesteps
            .iter()
            .find(|s| s.time > cut && s.node_voltages["v3v3"] < 3.3 * 0.98)
            .map(|s| s.time - cut)
            .unwrap()
    };
    let with_caps = interrupted(design());
    let without = interrupted(without_capacitors(design()));
    assert!(
        (15e-6..22e-6).contains(&with_caps),
        "with caps: {with_caps} s"
    );
    assert!(without < 0.5e-6, "without caps: {without} s");
}
