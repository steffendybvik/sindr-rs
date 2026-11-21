//! The battery against closed forms of its equivalent circuit:
//! `v = OCV(SoC) − R0·i − v1`, `C1·dv1/dt = i − v1/R1`,
//! `dSoC/dt = −i/(3600·capacity)`.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{
    solve_circuit, solve_transient, validate_circuit, Circuit, CircuitElement, SimError,
    SimulationResult, TransientConfig, TransientStart,
};
use sindr_devices::battery::{open_circuit_voltage, BatteryParams, BatteryRc};

fn battery(params: BatteryParams, soc: f64) -> CircuitElement {
    CircuitElement::Battery {
        id: "B1".into(),
        nodes: ["p".into(), "0".into()],
        params,
        state_of_charge: soc,
    }
}

fn load(ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: "RL".into(),
        nodes: ["p".into(), "0".into()],
        resistance: ohms,
    }
}

fn sink(amps: f64) -> CircuitElement {
    CircuitElement::CurrentSource {
        id: "I1".into(),
        nodes: ["p".into(), "0".into()],
        current: amps,
        waveform: None,
    }
}

fn battery_result(r: &sindr::TimestepSnapshot) -> (f64, f64) {
    let b = r
        .component_results
        .iter()
        .find(|c| c.id == "B1")
        .expect("B1 result");
    (b.voltage_across, b.current_through)
}

fn rel(got: f64, want: f64, tol: f64, what: &str) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1e-12),
        "{what}: got {got}, want {want}"
    );
}

/// Without an RC pair `solve_circuit` stays in DC: OCV behind R0, into the
/// load. (With a pair, the operating point settles it to R0 + R1: see
/// `operating_point_start_is_settled`.)
#[test]
fn dc_is_the_open_circuit_voltage_behind_r0() {
    let params = BatteryParams::lithium_ion(1.0, 0.15);
    let circuit = Circuit::new("0", vec![battery(params, 0.5), load(10.0)]);
    let r: SimulationResult = solve_circuit(&circuit).expect("solves");
    assert!(r.transient.is_none(), "no RC pair, so no transient");
    let v = 3.84 * 10.0 / (10.0 + 0.15);
    rel(r.node_voltages["p"], v, 1e-12, "v(p)");
    let b = r.component_results.iter().find(|c| c.id == "B1").unwrap();
    rel(b.current_through, -v / 10.0, 1e-12, "B1 current_through");
    assert!(b.power < 0.0, "a discharging battery delivers power");
    assert_eq!(r.state_of_charge["B1"], 0.5);
}

/// A constant 0.5 A sink on a 1 Ah Li-ion cell: SoC falls linearly,
/// `SoC0 − I·t/3600`, and the terminal follows `OCV(SoC) − R0·I` down the
/// curve.
#[test]
fn constant_current_walks_down_the_ocv_curve() {
    let params = BatteryParams::lithium_ion(1.0, 0.1);
    let circuit = Circuit::new("0", vec![battery(params.clone(), 0.9), sink(0.5)]);
    // An hour at half an amp: 0.9 → 0.4.
    let config = TransientConfig::new(3600.0, 5.0).with_fixed_step();
    let result = solve_transient(&circuit, &config).expect("solves");
    let steps = &result.transient.as_ref().unwrap().timesteps;
    for step in steps.iter().step_by(60) {
        let soc = 0.9 - 0.5 * step.time / 3600.0;
        rel(step.state_of_charge["B1"], soc, 1e-9, "SoC");
        let (v, i) = battery_result(step);
        rel(i, -0.5, 1e-9, "current");
        // OCV is read at the start of each step: within one step's slide.
        let want = open_circuit_voltage(soc, &params) - 0.1 * 0.5;
        assert!(
            (v - want).abs() < 2e-3,
            "at {} s: v = {v}, want {want}",
            step.time
        );
    }
    rel(result.state_of_charge["B1"], 0.4, 1e-9, "final SoC");
}

/// A flat 3 V cell with R0 = 1 Ω and R1 = 2 Ω, C1 = 1 F, switched onto
/// 5 Ω from rest. With `R = R0 + RL` the pair's voltage rises as
/// `v1∞·(1 − e^(−t/τ))`, `v1∞ = R1·OCV/(R + R1)`,
/// `τ = C1·R1·R/(R + R1)`, and the terminal is `RL·(OCV − v1)/R`.
#[test]
fn rc_pair_sags_with_its_time_constant() {
    let mut params = BatteryParams::constant(3.0, 100.0, 1.0);
    params.rc = Some(BatteryRc { r: 2.0, c: 1.0 });
    let circuit = Circuit::new("0", vec![battery(params, 1.0), load(5.0)]);
    let (ocv, r, r1, c1, rl) = (3.0, 6.0, 2.0, 1.0, 5.0);
    let v1_inf = r1 * ocv / (r + r1);
    let tau = c1 * r1 * r / (r + r1);
    let config = TransientConfig::new(5.0 * tau, tau / 200.0);
    let result = solve_transient(&circuit, &config).expect("solves");
    for step in &result.transient.as_ref().unwrap().timesteps {
        let v1 = v1_inf * (1.0 - (-step.time / tau).exp());
        let want = rl * (ocv - v1) / r;
        let (v, _) = battery_result(step);
        assert!(
            (v - want).abs() < 2e-3 * want,
            "at {:.3} s: v = {v}, want {want}",
            step.time
        );
    }
}

/// From the operating point the pair is already settled: no sag.
#[test]
fn operating_point_start_is_settled() {
    let mut params = BatteryParams::constant(3.0, 100.0, 1.0);
    params.rc = Some(BatteryRc { r: 2.0, c: 1.0 });
    let circuit = Circuit::new("0", vec![battery(params, 1.0), load(5.0)]);
    let config = TransientConfig::new(5.0, 0.05).with_start(TransientStart::OperatingPoint);
    let result = solve_transient(&circuit, &config).expect("solves");
    let settled = 5.0 * 3.0 / 8.0;
    for step in &result.transient.as_ref().unwrap().timesteps {
        let (v, _) = battery_result(step);
        rel(v, settled, 1e-6, "terminal");
    }
}

/// `solve_circuit` takes a battery with an RC pair to transient, and
/// reports where its charge ended.
#[test]
fn solve_circuit_reports_state_of_charge() {
    let mut params = BatteryParams::constant(3.0, 1e-3, 1.0);
    params.rc = Some(BatteryRc { r: 1.0, c: 1.0 });
    let circuit = Circuit::new("0", vec![battery(params, 1.0), load(9.0)]);
    let result = solve_circuit(&circuit).expect("solves");
    assert!(result.transient.is_some());
    assert!(result.state_of_charge["B1"] < 1.0);
}

/// In AC the cell is `R0 + R1‖C1`: driven through 1 Ω, the node's gain is
/// `Z/(1 + Z)`.
#[test]
fn ac_impedance_is_r0_plus_r1_parallel_c1() {
    let mut params = BatteryParams::constant(3.0, 1.0, 0.5);
    params.rc = Some(BatteryRc { r: 2.0, c: 1e-3 });
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "Rs".into(),
                nodes: ["in".into(), "p".into()],
                resistance: 1.0,
            },
            battery(params, 1.0),
        ],
    );
    let result = solve_ac(
        &circuit,
        &AcConfig {
            f_start: 1.0,
            f_stop: 1e4,
            num_points: 9,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
            ac_phase_deg: 0.0,
        },
    )
    .expect("solves");
    for point in &result.points {
        let w = 2.0 * std::f64::consts::PI * point.frequency;
        let z = num_complex::Complex64::new(0.5, 0.0)
            + num_complex::Complex64::new(2.0, 0.0)
                / num_complex::Complex64::new(1.0, w * 2.0 * 1e-3);
        let want = z / (z + 1.0);
        let got = point.node_voltages["p"];
        assert!(
            (got - want).norm() < 1e-9,
            "at {} Hz: {got} vs {want}",
            point.frequency
        );
    }
}

#[test]
fn bad_parameters_are_refused() {
    let base = BatteryParams::lithium_ion(1.0, 0.1);
    let cases: Vec<(BatteryParams, f64, &str)> = vec![
        (
            BatteryParams {
                capacity: 0.0,
                ..base.clone()
            },
            0.5,
            "capacity",
        ),
        (
            BatteryParams {
                r_series: 0.0,
                ..base.clone()
            },
            0.5,
            "r_series",
        ),
        (
            BatteryParams {
                ocv: vec![],
                ..base.clone()
            },
            0.5,
            "params.ocv",
        ),
        (
            BatteryParams {
                ocv: vec![(0.5, 3.0), (0.2, 3.5)],
                ..base.clone()
            },
            0.5,
            "params.ocv",
        ),
        (
            BatteryParams {
                rc: Some(BatteryRc { r: 1.0, c: -1.0 }),
                ..base.clone()
            },
            0.5,
            "params.rc.c",
        ),
        (base.clone(), 1.5, "state_of_charge"),
        (base, f64::NAN, "state_of_charge"),
    ];
    for (params, soc, needle) in cases {
        let circuit = Circuit::new("0", vec![battery(params, soc), load(10.0)]);
        match validate_circuit(&circuit) {
            Err(SimError::InvalidComponent(m)) => assert!(m.contains(needle), "{m}"),
            other => panic!("expected InvalidComponent for {needle}, got {other:?}"),
        }
    }
}

#[cfg(feature = "serde")]
#[test]
fn round_trips_through_json() {
    let element = battery(BatteryParams::cr2032(), 0.8);
    let json = serde_json::to_string(&element).unwrap();
    assert!(json.contains("\"type\":\"battery\""), "{json}");
    let back: CircuitElement = serde_json::from_str(&json).unwrap();
    assert_eq!(back, element);
}
