//! The brushed DC motor: its steady state, its step response, its
//! small-signal impedance and its datasheet constructor against their
//! closed forms.

use num_complex::Complex64;
use sindr::element::{DcMotor, DcMotorParams, Resistor, VoltageSource};
use sindr::session::Param;
use sindr::{
    check_ratings, solve_ac, solve_circuit, solve_op, solve_transient, AcConfig, Circuit,
    CircuitElement, Part, RatingKind, RatingStatus, Ratings, TransientConfig, TransientSession,
};

/// A small 12 V hobby motor: 2 Ω, 1 mH, k = 0.02, J = 1e-6, B = 1e-6.
fn params() -> DcMotorParams {
    DcMotorParams::new(2.0, 0.02)
        .with_inductance(1e-3)
        .with_inertia(1e-6)
        .with_friction(1e-6)
}

fn motor_on(volts: f64, params: DcMotorParams) -> Circuit {
    Circuit::new(
        "0",
        vec![
            VoltageSource::dc("V1", ["in", "0"], volts).into(),
            CircuitElement::DcMotor(DcMotor {
                id: "M1".into(),
                nodes: ["in".into(), "0".into()],
                params,
            }),
        ],
    )
}

fn close(a: f64, b: f64, rel: f64) {
    assert!(
        (a - b).abs() <= rel * b.abs().max(1e-12),
        "{a} is not within {rel} of {b}"
    );
}

#[test]
fn dc_speed_and_current_match_the_closed_form() {
    for load in [0.0, 1e-3, 5e-3] {
        let p = params().with_load(load);
        let result = solve_circuit(&motor_on(12.0, p)).unwrap();
        let (omega, i) = p.steady_state(12.0);
        let shaft = result.motors["M1"];
        close(shaft.speed, omega, 1e-9);
        close(shaft.torque, p.kt * i, 1e-9);
        close(shaft.mechanical_power, p.kt * i * omega, 1e-9);
        let port = result.component("M1").unwrap();
        close(port.current_through, i, 1e-9);
        close(port.voltage_across, 12.0, 1e-12);
        close(port.power, 12.0 * i, 1e-9);
        // Power in = armature loss + converted power.
        close(
            port.power,
            i * i * p.resistance + shaft.mechanical_power,
            1e-9,
        );
        // The internal parts and nodes are folded away.
        assert!(result.component_results.iter().all(|c| !c.id.contains('#')));
        assert!(result.node_voltages.keys().all(|n| !n.contains('#')));
    }
}

#[test]
fn a_motor_without_friction_runs_at_its_load_current() {
    let p = DcMotorParams::new(2.0, 0.02).with_load(4e-3);
    let result = solve_circuit(&motor_on(12.0, p)).unwrap();
    // kt·i = T_L, and V = R·i + ke·ω.
    let i = 4e-3 / 0.02;
    close(result.component("M1").unwrap().current_through, i, 1e-9);
    close(result.motors["M1"].speed, (12.0 - 2.0 * i) / 0.02, 1e-9);
}

#[test]
fn a_motor_held_still_draws_its_stall_current() {
    let p = DcMotorParams::new(2.0, 0.02).with_load(1.0);
    let stall = 12.0 / 2.0;
    // A load past the stall torque turns it backwards; at exactly kt·V/R it
    // stands still.
    let p = p.with_load(p.kt * stall);
    let result = solve_circuit(&motor_on(12.0, p)).unwrap();
    close(result.component("M1").unwrap().current_through, stall, 1e-9);
    assert!(result.motors["M1"].speed.abs() < 1e-9);
}

/// A motor is rated on its terminals: voltage, current and power, read off
/// the port the lowering folds its parts back into. Unrated, all three are
/// unverified. Checked at the operating point: `solve_circuit` would run
/// the power-on transient, and its inrush would be the worst current.
#[test]
fn a_motor_is_rated_on_its_terminals() {
    let mut circuit = motor_on(12.0, params().with_load(1e-3));
    let result = solve_op(&circuit).unwrap();
    let report = check_ratings(&circuit, &result).unwrap();
    let kinds: Vec<_> = report.unverified().iter().map(|c| c.rating).collect();
    for kind in [RatingKind::Voltage, RatingKind::Current, RatingKind::Power] {
        assert!(kinds.contains(&kind), "{kind:?} in {kinds:?}");
    }

    let i = result.component("M1").unwrap().current_through;
    circuit.parts.insert(
        "M1".into(),
        Part {
            ratings: Ratings {
                voltage: Some(24.0),
                current: Some(i / 2.0),
                power: Some(100.0),
                ..Ratings::default()
            },
            ..Part::default()
        },
    );
    let report = check_ratings(&circuit, &result).unwrap();
    let get = |kind| report.for_part("M1").find(|c| c.rating == kind).unwrap();
    assert_eq!(get(RatingKind::Voltage).status, RatingStatus::Within);
    close(get(RatingKind::Voltage).worst.unwrap(), 12.0, 1e-12);
    assert_eq!(get(RatingKind::Current).status, RatingStatus::Exceeded);
    close(get(RatingKind::Current).worst.unwrap(), i, 1e-12);
    assert_eq!(get(RatingKind::Power).status, RatingStatus::Within);
}

/// `x' = A·x + u` from rest for the 2×2 state `[i, ω]`, by the matrix
/// exponential: `x(t) = A⁻¹·(e^{At} − I)·u`.
fn two_pole(p: &DcMotorParams, volts: f64, t: f64) -> (f64, f64) {
    let (r, l, ke, kt, j, b) = (
        p.resistance,
        p.inductance,
        p.ke,
        p.kt,
        p.inertia,
        p.friction,
    );
    let a = [[-r / l, -ke / l], [kt / j, -b / j]];
    let u = [volts / l, -p.load_torque / j];
    // e^{At} for a 2×2 matrix, with complex eigenvalues allowed.
    let tr = a[0][0] + a[1][1];
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    let disc = Complex64::new(tr * tr / 4.0 - det, 0.0).sqrt();
    let (l1, l2) = (tr / 2.0 + disc, tr / 2.0 - disc);
    let (e1, e2) = ((l1 * t).exp(), (l2 * t).exp());
    // e^{At} = c0·I + c1·A.
    let c1 = (e1 - e2) / (l1 - l2);
    let c0 = e1 - c1 * l1;
    let exp = |row: usize, col: usize| {
        let id = if row == col { 1.0 } else { 0.0 };
        (c0 * id + c1 * a[row][col]).re
    };
    let m = [[exp(0, 0) - 1.0, exp(0, 1)], [exp(1, 0), exp(1, 1) - 1.0]];
    let y = [
        m[0][0] * u[0] + m[0][1] * u[1],
        m[1][0] * u[0] + m[1][1] * u[1],
    ];
    // Solve A·x = y.
    let x0 = (y[0] * a[1][1] - a[0][1] * y[1]) / det;
    let x1 = (a[0][0] * y[1] - a[1][0] * y[0]) / det;
    (x0, x1)
}

#[test]
fn the_step_response_matches_the_two_pole_solution() {
    let p = params().with_load(1e-3);
    let mut config = TransientConfig::new(0.2, 1e-4);
    config.tmax = Some(1e-4);
    let result = solve_transient(&motor_on(12.0, p), &config).unwrap();
    let steps = &result.transient.as_ref().unwrap().timesteps;
    // Within 1 % of the scale each moves on: the integrator's error at a
    // step a fifth of L/R, not the model's.
    let (omega_ss, i_ss) = p.steady_state(12.0);
    let stall = 12.0 / p.resistance;
    let mut peak = 0.0_f64;
    for step in steps.iter().filter(|s| s.time > 0.0) {
        let (i, omega) = two_pole(&p, 12.0, step.time);
        let shaft = step.motors["M1"];
        let current = step
            .component_results
            .iter()
            .find(|c| c.id == "M1")
            .unwrap()
            .current_through;
        assert!(
            (shaft.speed - omega).abs() < 0.01 * omega_ss,
            "t = {}: speed {} vs {omega}",
            step.time,
            shaft.speed
        );
        assert!(
            (current - i).abs() < 0.01 * stall,
            "t = {}: current {current} vs {i}",
            step.time
        );
        peak = peak.max(current);
    }
    // The start-up surge: well above the running current, near stall.
    assert!(peak > 0.8 * stall, "peak {peak}");
    let last = steps.last().unwrap();
    close(last.motors["M1"].speed, omega_ss, 1e-3);
    close(result.motors["M1"].speed, omega_ss, 1e-3);
    close(result.component("M1").unwrap().current_through, i_ss, 1e-2);
}

#[test]
fn the_impedance_is_the_armature_plus_the_reflected_shaft() {
    let p = params();
    let circuit = Circuit::new(
        "0",
        vec![
            VoltageSource::dc("V1", ["in", "0"], 0.0).into(),
            Resistor {
                id: "Rs".into(),
                nodes: ["in".into(), "a".into()],
                resistance: 1.0,
            }
            .into(),
            CircuitElement::DcMotor(DcMotor {
                id: "M1".into(),
                nodes: ["a".into(), "0".into()],
                params: p,
            }),
        ],
    );
    let result = solve_ac(&circuit, &AcConfig::new("V1", 1.0, 1e5, 21)).unwrap();
    for point in &result.points {
        let s = Complex64::new(0.0, std::f64::consts::TAU * point.frequency);
        let expected = p.resistance + s * p.inductance + p.ke * p.kt / (s * p.inertia + p.friction);
        let (vin, va) = (point.voltage("in").unwrap(), point.voltage("a").unwrap());
        let z = va / (vin - va);
        assert!(
            (z - expected).norm() < 1e-6 * expected.norm(),
            "f = {}: {z} vs {expected}",
            point.frequency
        );
    }
}

#[test]
fn the_datasheet_constructor_hits_no_load_and_stall() {
    let p = DcMotorParams::from_datasheet(12.0, 6000.0, 0.15, 0.05, 3.0);
    let (omega, i) = p.steady_state(12.0);
    close(omega * 60.0 / std::f64::consts::TAU, 6000.0, 1e-12);
    close(i, 0.15, 1e-12);
    let stalled = p.with_load(0.05);
    let (omega, i) = stalled.steady_state(12.0);
    assert!(omega.abs() < 1e-9, "{omega}");
    close(i, 3.0, 1e-12);
    // And the solver agrees.
    let result = solve_circuit(&motor_on(12.0, p)).unwrap();
    close(result.motors["M1"].rpm(), 6000.0, 1e-9);
}

#[test]
fn a_session_changes_the_load_torque() {
    let p = params();
    let mut config = TransientConfig::new(1.0, 1e-3);
    config.start = sindr::TransientStart::OperatingPoint;
    let mut session = TransientSession::new(&motor_on(12.0, p), &config).unwrap();
    session.advance_to(0.1).unwrap();
    let (omega_free, _) = p.steady_state(12.0);
    close(
        session.last_step().unwrap().motors["M1"].speed,
        omega_free,
        1e-6,
    );
    session.set_param("M1", Param::LoadTorque(5e-3)).unwrap();
    session.advance_to(0.6).unwrap();
    let (omega_loaded, i_loaded) = p.with_load(5e-3).steady_state(12.0);
    let step = session.last_step().unwrap();
    close(step.motors["M1"].speed, omega_loaded, 1e-3);
    close(session.current("M1").unwrap(), i_loaded, 1e-3);
    assert!(session.set_param("V1", Param::LoadTorque(1.0)).is_err());
    assert!(session
        .set_param("M1", Param::LoadTorque(f64::NAN))
        .is_err());
}

#[test]
fn bad_constants_are_refused() {
    for p in [
        DcMotorParams::new(0.0, 0.02),
        DcMotorParams::new(2.0, 0.0),
        DcMotorParams::new(2.0, 0.02).with_inertia(-1.0),
        DcMotorParams::new(2.0, 0.02).with_friction(f64::NAN),
    ] {
        let err = solve_circuit(&motor_on(12.0, p)).unwrap_err();
        assert!(err.to_string().contains("M1"), "{err}");
    }
}

#[cfg(feature = "spice")]
#[test]
fn the_exported_deck_reads_back_and_solves_the_same() {
    let p = params().with_load(2e-3);
    let circuit = motor_on(12.0, p);
    let deck = sindr::spice::to_netlist(&circuit, "motor").unwrap();
    for line in [
        "RM1_r",
        "LM1_l",
        "VM1_sense",
        "EM1_emf",
        "FM1_kt",
        "CM1_j",
        "RM1_b",
        "IM1_load",
    ] {
        assert!(deck.contains(line), "{line} missing from\n{deck}");
    }
    let back = sindr::spice::parse_str(&deck).unwrap().circuit;
    let ours = solve_circuit(&circuit).unwrap();
    let theirs = solve_circuit(&back).unwrap();
    // SPICE names are case-blind: the parser reads them lower-case.
    close(
        theirs.voltage("m1_w").unwrap(),
        ours.motors["M1"].speed,
        1e-9,
    );
}

#[cfg(feature = "serde")]
#[test]
fn a_motor_round_trips_through_json() {
    let circuit = motor_on(12.0, params().with_load(1e-3));
    let json = serde_json::to_string(&circuit).unwrap();
    assert!(json.contains("\"type\":\"dc_motor\""), "{json}");
    let back: Circuit = serde_json::from_str(&json).unwrap();
    assert_eq!(back, circuit);
    let result = solve_circuit(&circuit).unwrap();
    let json = serde_json::to_string(&result).unwrap();
    let back: sindr::SimulationResult = serde_json::from_str(&json).unwrap();
    assert_eq!(back.motors["M1"], result.motors["M1"]);
}
