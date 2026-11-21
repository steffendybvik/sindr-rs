//! A saturating inductor against closed forms of its own flux curve
//! `λ(i) = L_sat·i + (L0 − L_sat)·I_sat·tanh(i/I_sat)`.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{
    solve_transient, validate_circuit, Circuit, CircuitElement, IntegrationMethod, SimError,
    TransientConfig,
};
use sindr_devices::inductor::{flux, incremental_inductance, InductorSaturation};

const L0: f64 = 10e-3;
const SAT: InductorSaturation = InductorSaturation {
    i_sat: 1.0,
    l_sat: 1e-3,
};

fn inductor(a: &str, b: &str, saturation: Option<InductorSaturation>) -> CircuitElement {
    CircuitElement::Inductor {
        id: "L1".into(),
        nodes: [a.into(), b.into()],
        inductance: L0,
        saturation,
    }
}

fn vsource(p: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: "V1".into(),
        nodes: [p.into(), "0".into()],
        voltage: volts,
        waveform: None,
    }
}

fn current_at(tran: &sindr::TransientData, t: f64) -> f64 {
    let steps = &tran.timesteps;
    let k = steps
        .partition_point(|s| s.time < t)
        .clamp(1, steps.len() - 1);
    let (a, b) = (&steps[k - 1], &steps[k]);
    let i = |s: &sindr::TimestepSnapshot| {
        s.component_results
            .iter()
            .find(|c| c.id == "L1")
            .expect("L1")
            .current_through
    };
    i(a) + (i(b) - i(a)) * (t - a.time) / (b.time - a.time)
}

/// 1 V straight across the inductor: the flux is exactly `V·t` whatever the
/// integrator, so the current reaches `i` when `t = λ(i)/V`. The current
/// grows ever faster as the core saturates.
#[test]
fn constant_voltage_follows_the_flux_curve() {
    let circuit = Circuit::new("0", vec![vsource("a", 1.0), inductor("a", "0", Some(SAT))]);
    let t_end = flux(4.0, L0, &SAT);
    for method in [
        IntegrationMethod::Gear2,
        IntegrationMethod::Trapezoidal,
        IntegrationMethod::BackwardEuler,
    ] {
        let config = TransientConfig::new(t_end, t_end / 400.0).with_method(method);
        let result = solve_transient(&circuit, &config).expect("solves");
        let tran = result.transient.as_ref().expect("transient");
        for i in [0.2, 0.5, 1.0, 2.0, 3.0, 4.0] {
            let t = flux(i, L0, &SAT); // at 1 V
            let got = current_at(tran, t);
            assert!(
                (got - i).abs() <= 2e-3 * i,
                "{method:?}: at t = {t:.3e} s expected {i} A, got {got}"
            );
        }
    }
}

/// 10 V into 1 Ω and the inductor: `L(i)·di/dt = V − i·R`, so the time to
/// reach `i` is `∫₀ⁱ L(x)/(V − x·R) dx`, integrated here by Simpson's rule.
/// Saturation makes it faster than the linear inductor's
/// `−(L0/R)·ln(1 − i·R/V)`.
#[test]
fn rl_charge_matches_the_integrated_ode() {
    let (v, r) = (10.0, 1.0);
    let time_to = |i: f64| {
        let n = 20_000;
        let h = i / n as f64;
        let f = |x: f64| incremental_inductance(x, L0, &SAT) / (v - x * r);
        let mut sum = f(0.0) + f(i);
        for k in 1..n {
            sum += f(k as f64 * h) * if k % 2 == 1 { 4.0 } else { 2.0 };
        }
        sum * h / 3.0
    };
    let circuit = |saturation| {
        Circuit::new(
            "0",
            vec![
                vsource("a", v),
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["a".into(), "b".into()],
                    resistance: r,
                },
                inductor("b", "0", saturation),
            ],
        )
    };
    let t_end = time_to(9.0);
    let config = TransientConfig::new(t_end, t_end / 2000.0);
    let saturating = solve_transient(&circuit(Some(SAT)), &config).expect("solves");
    let linear = solve_transient(&circuit(None), &config).expect("solves");
    for i in [0.5, 1.0, 2.0, 5.0, 8.0, 9.0] {
        let t = time_to(i);
        let got = current_at(saturating.transient.as_ref().unwrap(), t);
        assert!(
            (got - i).abs() <= 5e-3 * i,
            "at t = {t:.4e} s expected {i} A, got {got}"
        );
        let slower = current_at(linear.transient.as_ref().unwrap(), t);
        assert!(
            slower < got,
            "the linear inductor should lag: {slower} vs {got}"
        );
        let linear_closed = v / r * (1.0 - (-t * r / L0).exp());
        assert!((slower - linear_closed).abs() <= 5e-3 * linear_closed);
    }
}

/// Biased at 2 A, an RL low-pass's corner sits at `R/(2π·L(2 A))`, the
/// incremental inductance there, not at `R/(2π·L0)`.
#[test]
fn ac_uses_the_incremental_inductance_at_the_bias() {
    let r = 10.0;
    let circuit = Circuit::new(
        "0",
        vec![
            vsource("a", 2.0 * r),
            inductor("a", "b", Some(SAT)),
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["b".into(), "0".into()],
                resistance: r,
            },
        ],
    );
    let l_inc = incremental_inductance(2.0, L0, &SAT);
    let corner = r / (2.0 * std::f64::consts::PI * l_inc);
    let result = solve_ac(
        &circuit,
        &AcConfig {
            f_start: corner,
            f_stop: corner,
            num_points: 1,
            spacing: FrequencySpacing::Linear,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
            ac_phase_deg: 0.0,
        },
    )
    .expect("solves");
    let gain = result.points[0].node_voltages["b"].norm();
    assert!(
        (gain - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-6,
        "gain at the incremental corner: {gain}"
    );
}

#[test]
fn saturation_parameters_are_validated() {
    for (sat, needle) in [
        (
            InductorSaturation {
                i_sat: 0.0,
                l_sat: 1e-3,
            },
            "i_sat",
        ),
        (
            InductorSaturation {
                i_sat: 1.0,
                l_sat: f64::NAN,
            },
            "l_sat",
        ),
        (
            InductorSaturation {
                i_sat: 1.0,
                l_sat: 2.0 * L0,
            },
            "must not exceed",
        ),
    ] {
        let circuit = Circuit::new(
            "0",
            vec![
                vsource("a", 1.0),
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["a".into(), "b".into()],
                    resistance: 1.0,
                },
                inductor("b", "0", Some(sat)),
            ],
        );
        match validate_circuit(&circuit) {
            Err(SimError::InvalidComponent(m)) => assert!(m.contains(needle), "{m}"),
            other => panic!("expected InvalidComponent for {needle}, got {other:?}"),
        }
    }
}

#[cfg(feature = "spice")]
#[test]
fn export_refuses_a_saturating_inductor() {
    let circuit = Circuit::new("0", vec![vsource("a", 1.0), inductor("a", "0", Some(SAT))]);
    let err = sindr::spice::to_netlist(&circuit, "t").unwrap_err();
    assert!(err.to_string().contains("saturating"), "{err}");
}

#[cfg(feature = "serde")]
#[test]
fn saturation_round_trips_through_json() {
    let element = inductor("a", "0", Some(SAT));
    let json = serde_json::to_string(&element).unwrap();
    assert!(json.contains("\"i_sat\""), "{json}");
    let back: CircuitElement = serde_json::from_str(&json).unwrap();
    assert_eq!(back, element);
    let linear = serde_json::to_string(&inductor("a", "0", None)).unwrap();
    assert!(!linear.contains("saturation"), "{linear}");
}
