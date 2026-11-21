//! Self-heating: junction temperature from dissipation, fed back into the
//! solve.

use sindr::{
    check_ratings, solve_ac, solve_circuit, solve_noise, solve_tf, solve_transient, AcConfig,
    BjtKind, Circuit, CircuitElement, FrequencySpacing, NoiseConfig, Part, RatingKind,
    RatingStatus, Ratings, SimError, TfConfig, TfOutput, Thermal, TransientConfig,
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

/// A 10 kΩ NTC fed 5 mA from a current source, 2000 K/W to ambient.
/// Heating lowers its resistance, so its power I²·R falls as it warms: a
/// stable, negative-feedback loop. Its loop gain at the balance is about
/// −1.8, so a plain `T ← ambient + P(T)·Rθ` iteration swings wider every
/// pass instead of settling.
fn fed_ntc(cth: Option<f64>) -> Circuit {
    let mut c = Circuit::new(
        "0",
        vec![
            CircuitElement::CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "a".into()],
                current: 5e-3,
                waveform: None,
            },
            CircuitElement::Thermistor {
                id: "TH1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: AMBIENT,
            },
        ],
    );
    c.parts.insert(
        "TH1".into(),
        heated(Thermal {
            cth,
            ..Thermal::junction_to_ambient(2000.0)
        }),
    );
    c
}

#[test]
fn ntc_with_strong_negative_feedback_balances_in_dc() {
    let dc = solve_circuit(&fed_ntc(None)).unwrap();
    let tj = dc.junction_temperatures["TH1"];
    let p = dc
        .component_results
        .iter()
        .find(|r| r.id == "TH1")
        .unwrap()
        .power;
    assert!((tj - (AMBIENT + p * 2000.0)).abs() < 1e-2, "Tj {tj}, P {p}");
    assert!(tj > AMBIENT + 30.0 && tj < AMBIENT + 80.0, "Tj {tj}");

    // A transient of the same part (Rθ·Cθ = 2 s) settles where DC balanced.
    let result = solve_transient(&fed_ntc(Some(1e-3)), &TransientConfig::new(20.0, 1e-2)).unwrap();
    let settled = result.junction_temperatures["TH1"];
    assert!(
        (settled - tj).abs() < 0.05,
        "transient {settled} K, DC {tj} K"
    );
}

/// Boltzmann constant over the elementary charge (V/K).
const K_OVER_Q: f64 = 1.380_649e-23 / 1.602_176_634e-19;
const RC: f64 = 1e3;

/// Common-emitter stage: base held at 0.67 V by `VIN`, emitter grounded,
/// collector through `RC` to 10 V, Q1 heated at 400 K/W. It settles near
/// Tj ≈ 309 K and Ic ≈ 3.1 mA, against about 1.7 mA cold, so `−gm·RC`
/// tells the heated bias point from the ambient one by far more than the
/// tolerances below.
fn heated_ce_stage() -> Circuit {
    let mut c = Circuit::new(
        "0",
        vec![
            vsrc("VIN", "b", "0", 0.67),
            vsrc("VCC", "vcc", "0", 10.0),
            res("RC", "vcc", "c", RC),
            npn("Q1", "b", "c", "0"),
        ],
    );
    c.parts
        .insert("Q1".into(), heated(Thermal::junction_to_ambient(400.0)));
    c
}

/// `circuit` without thermal data, Q1 fixed at `temperature`.
fn at_fixed_temperature(circuit: &Circuit, temperature: f64) -> Circuit {
    let mut c = circuit.clone();
    c.parts.clear();
    for e in &mut c.components {
        if let CircuitElement::Bjt { temperature: t, .. } = e {
            *t = temperature;
        }
    }
    c
}

/// The heated stage's junction temperature, collector current and the
/// midband gain `−gm·RC` they give, `gm = Ic/(kTj/q)`, from the plain DC
/// solve.
fn heated_bias() -> (f64, f64, f64) {
    let op = solve_circuit(&heated_ce_stage()).unwrap();
    let tj = op.junction_temperatures["Q1"];
    let ic = (10.0 - op.node_voltages["c"]) / RC;
    (tj, ic, -ic / (K_OVER_Q * tj) * RC)
}

fn ac_gain(circuit: &Circuit) -> f64 {
    let config = AcConfig {
        f_start: 1e3,
        f_stop: 1e3,
        num_points: 1,
        spacing: FrequencySpacing::Linear,
        source_id: "VIN".into(),
        ac_magnitude: 1.0,
        ac_phase_deg: 0.0,
    };
    let gain = solve_ac(circuit, &config).unwrap().points[0].node_voltages["c"];
    assert!(gain.im.abs() < 1e-9 * gain.re.abs(), "{gain}");
    gain.re
}

#[test]
fn ac_linearises_at_the_heated_bias_point() {
    let (tj, ic, expected) = heated_bias();
    assert!(tj > AMBIENT + 5.0, "Tj {tj}");
    let gain = ac_gain(&heated_ce_stage());
    assert!(
        (gain - expected).abs() < 1e-3 * expected.abs(),
        "AC gain {gain} vs −gm·Rc {expected} at Tj {tj} K, Ic {ic} A"
    );
    // Linearised at ambient instead, the gain is well off.
    let cold = ac_gain(&at_fixed_temperature(&heated_ce_stage(), AMBIENT));
    assert!((cold - gain).abs() > 0.2 * gain.abs(), "cold gain {cold}");
}

#[test]
fn tf_linearises_at_the_heated_bias_point() {
    let (tj, ic, expected) = heated_bias();
    let config = TfConfig {
        output: TfOutput::Voltage {
            node: "c".into(),
            reference: None,
        },
        input_source: "VIN".into(),
    };
    let tf = solve_tf(&heated_ce_stage(), &config).unwrap();
    assert!(
        (tf.gain - expected).abs() < 1e-3 * expected.abs(),
        ".tf gain {} vs −gm·Rc {expected} at Tj {tj} K, Ic {ic} A",
        tf.gain
    );
    // The base draws Ic/β, so the input sees rπ = β·kTj/(q·Ic).
    let r_pi = 100.0 * K_OVER_Q * tj / ic;
    assert!(
        (tf.input_resistance - r_pi).abs() < 1e-3 * r_pi,
        "Rin {} vs rπ {r_pi}",
        tf.input_resistance
    );
}

#[test]
fn noise_uses_the_heated_bias_point() {
    let config = NoiseConfig {
        output: "c".into(),
        reference: None,
        input_source: "VIN".into(),
        f_start: 1e3,
        f_stop: 1e5,
        num_points: 5,
        spacing: FrequencySpacing::Logarithmic,
    };
    let (tj, _, _) = heated_bias();
    let total = |c: &Circuit| solve_noise(c, &config).unwrap().total_output_noise;
    let heated = total(&heated_ce_stage());
    let fixed = total(&at_fixed_temperature(&heated_ce_stage(), tj));
    let cold = total(&at_fixed_temperature(&heated_ce_stage(), AMBIENT));
    assert!(
        (heated - fixed).abs() < 1e-3 * fixed,
        "heated {heated} vs fixed at Tj {fixed}"
    );
    assert!(
        (heated - cold).abs() > 0.1 * cold,
        "heated {heated} vs cold {cold}"
    );
}
