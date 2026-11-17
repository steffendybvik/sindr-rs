//! Self-heating: junction temperature from dissipation, fed back into the
//! solve.

use sindr::{
    check_ratings, solve_circuit, solve_transient, BjtKind, Circuit, CircuitElement, Part,
    RatingKind, RatingStatus, Ratings, SimError, Thermal, TransientConfig,
};

const AMBIENT: f64 = 300.15;

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

fn npn(id: &str, b: &str, c: &str, e: &str) -> CircuitElement {
    CircuitElement::Bjt {
        id: id.into(),
        nodes: [b.into(), c.into(), e.into()],
        kind: BjtKind::Npn,
        bf: 100.0,
        temperature: AMBIENT,
        parasitic_caps: None,
        model: None,
    }
}

fn heated(thermal: Thermal) -> Part {
    Part {
        thermal: Some(thermal),
        ..Part::default()
    }
}

/// 5 V through 150 Ω into a red LED.
fn led_circuit() -> Circuit {
    Circuit::new(
        "0",
        vec![
            vsrc("V1", "in", "0", 5.0),
            res("R1", "in", "a", 150.0),
            CircuitElement::Led {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                color: "red".into(),
                temperature: AMBIENT,
            },
        ],
    )
}

fn current(result: &sindr::SimulationResult, id: &str) -> f64 {
    result
        .component_results
        .iter()
        .find(|r| r.id == id)
        .unwrap()
        .current_through
}

#[test]
fn led_settles_at_a_raised_junction_temperature() {
    let cold = solve_circuit(&led_circuit()).unwrap();
    assert!(cold.junction_temperatures.is_empty());

    let mut c = led_circuit();
    c.parts
        .insert("D1".into(), heated(Thermal::junction_to_ambient(400.0)));
    let hot = solve_circuit(&c).unwrap();

    let tj = hot.junction_temperatures["D1"];
    let p = hot
        .component_results
        .iter()
        .find(|r| r.id == "D1")
        .unwrap()
        .power;
    // Self-consistent: the temperature is the one its own power gives.
    assert!((tj - (AMBIENT + p * 400.0)).abs() < 1e-2, "Tj {tj}, P {p}");
    assert!(tj > AMBIENT + 5.0, "Tj {tj}");
    // The device saw that temperature: the operating point moved.
    assert!((current(&hot, "D1") - current(&cold, "D1")).abs() > 1e-9);
}

/// A two-transistor mirror with no emitter degeneration. Q2's collector
/// sits on 20 V, so it dissipates 20 V × Iref ≈ 0.2 W while Q1, which sets
/// Vbe, stays cool. Heat raises Q2's IS at the fixed Vbe, so its current,
/// and with it its power, climbs.
fn mirror(q2_thermal: Thermal) -> Circuit {
    let mut c = Circuit::new(
        "0",
        vec![
            vsrc("VCC", "vcc", "0", 20.0),
            res("RREF", "vcc", "b", 2e3),
            npn("Q1", "b", "b", "0"),
            npn("Q2", "b", "vcc", "0"),
        ],
    );
    c.parts.insert("Q2".into(), heated(q2_thermal));
    c
}

#[test]
fn undegenerated_mirror_runs_away_without_a_heatsink() {
    // Free-standing small-signal package: ~500 K/W.
    let err = solve_circuit(&mirror(Thermal::junction_to_ambient(500.0))).unwrap_err();
    match err {
        SimError::ThermalRunaway { part, temperature } => {
            assert_eq!(part, "Q2");
            assert!(temperature > 400.0, "gave up at {temperature} K");
        }
        other => panic!("expected thermal runaway, got {other:?}"),
    }
}

#[test]
fn same_mirror_is_stable_on_a_heatsink() {
    let mut c = mirror(Thermal::on_heatsink(5.0, 10.0));
    c.parts.get_mut("Q2").unwrap().ratings = Ratings {
        tj_max: Some(423.15),
        ..Ratings::default()
    };
    let result = solve_circuit(&c).unwrap();
    let tj = result.junction_temperatures["Q2"];
    assert!(tj > AMBIENT + 1.0 && tj < AMBIENT + 20.0, "Tj {tj}");
    // It still runs hotter, and so carries more, than the cool reference.
    assert!(current(&result, "Q2") > current(&result, "RREF") * 0.95);

    let report = check_ratings(&c, &result).unwrap();
    let tj_check = report
        .for_part("Q2")
        .find(|c| c.rating == RatingKind::Tj)
        .unwrap();
    assert_eq!(tj_check.status, RatingStatus::Within);
    assert!((tj_check.worst.unwrap() - tj).abs() < 1e-12);
}

#[test]
fn tj_max_is_flagged_and_needed_once_a_part_is_heated() {
    // A resistor has no temperature in its model, so its junction (body)
    // temperature is exactly ambient + P·Rθ and changes nothing else.
    let mut c = Circuit::new(
        "0",
        vec![vsrc("V1", "a", "0", 10.0), res("R1", "a", "0", 100.0)],
    );
    c.parts
        .insert("R1".into(), heated(Thermal::junction_to_ambient(250.0)));
    let result = solve_circuit(&c).unwrap();
    // 1 W into 250 K/W.
    assert!((result.junction_temperatures["R1"] - (AMBIENT + 250.0)).abs() < 1e-9);
    assert!((current(&result, "R1") - 0.1).abs() < 1e-12);

    let report = check_ratings(&c, &result).unwrap();
    let tj = report
        .for_part("R1")
        .find(|c| c.rating == RatingKind::Tj)
        .unwrap();
    assert_eq!(tj.status, RatingStatus::Unverified);

    c.parts.get_mut("R1").unwrap().ratings.tj_max = Some(428.15); // 155 °C
    let report = check_ratings(&c, &result).unwrap();
    let tj = report
        .for_part("R1")
        .find(|c| c.rating == RatingKind::Tj)
        .unwrap();
    assert_eq!(tj.status, RatingStatus::Exceeded);
}

#[test]
fn transient_junction_follows_its_thermal_time_constant() {
    // Rθ·Cθ = 400 K/W × 2.5 mJ/K = 1 s.
    let mut c = led_circuit();
    c.parts.insert(
        "D1".into(),
        heated(Thermal {
            cth: Some(2.5e-3),
            ..Thermal::junction_to_ambient(400.0)
        }),
    );
    let steady = solve_circuit(&c).unwrap().junction_temperatures["D1"];

    let result = solve_transient(&c, &TransientConfig::new(6.0, 1e-2)).unwrap();
    let tran = result.transient.as_ref().unwrap();
    let at = |t: f64| {
        tran.timesteps
            .iter()
            .min_by(|a, b| (a.time - t).abs().total_cmp(&(b.time - t).abs()))
            .unwrap()
            .junction_temperatures["D1"]
    };
    let rise = steady - AMBIENT;
    assert!(at(0.0) - AMBIENT < 0.05 * rise, "starts cold");
    let one_tau = (at(1.0) - AMBIENT) / rise;
    assert!((one_tau - 0.632).abs() < 0.03, "{one_tau} of the rise at τ");
    // Six time constants: within 0.5 % of the DC balance.
    assert!((result.junction_temperatures["D1"] - steady).abs() < 0.005 * rise);
}

#[test]
fn thermal_without_a_resistance_is_rejected() {
    let mut c = led_circuit();
    c.parts.insert(
        "D1".into(),
        heated(Thermal {
            rth_jc: Some(5.0), // no case-to-ambient, no Rθja
            ..Thermal::default()
        }),
    );
    assert!(matches!(
        solve_circuit(&c),
        Err(SimError::InvalidComponent(msg)) if msg.contains("D1")
    ));
}
