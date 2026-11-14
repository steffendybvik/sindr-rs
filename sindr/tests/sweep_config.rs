//! Invalid sweep configs must return `Err`, never panic or yield NaN.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{dc_sweep, temperature_sweep, Circuit, CircuitElement, SimError};

/// V1 -> R1 -> out -> C1 -> gnd.
fn rc_lowpass() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "out".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    }
}

fn ac_config(f_start: f64, f_stop: f64, num_points: usize, spacing: FrequencySpacing) -> AcConfig {
    AcConfig {
        f_start,
        f_stop,
        num_points,
        spacing,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    }
}

fn assert_invalid<T: std::fmt::Debug>(result: Result<T, SimError>, case: &str) {
    match result {
        Err(SimError::InvalidComponent(_)) => {}
        other => panic!("{case}: expected InvalidComponent, got {other:?}"),
    }
}

#[test]
fn ac_rejects_invalid_configs() {
    use FrequencySpacing::{Linear, Logarithmic};
    let circuit = rc_lowpass();
    let cases = [
        ("zero points, log", ac_config(1.0, 1e3, 0, Logarithmic)),
        ("zero points, linear", ac_config(1.0, 1e3, 0, Linear)),
        ("f_start = 0, log", ac_config(0.0, 1e3, 10, Logarithmic)),
        ("f_start < 0, log", ac_config(-1.0, 1e3, 10, Logarithmic)),
        ("f_stop < 0, log", ac_config(1.0, -1e3, 10, Logarithmic)),
        ("f_start < 0, linear", ac_config(-1.0, 1e3, 10, Linear)),
        (
            "f_start > f_stop, log",
            ac_config(1e3, 1.0, 10, Logarithmic),
        ),
        ("f_start > f_stop, linear", ac_config(1e3, 1.0, 10, Linear)),
        ("NaN f_start", ac_config(f64::NAN, 1e3, 10, Logarithmic)),
        ("NaN f_stop", ac_config(1.0, f64::NAN, 10, Linear)),
        ("inf f_stop", ac_config(1.0, f64::INFINITY, 10, Logarithmic)),
        (
            "-inf f_start",
            ac_config(f64::NEG_INFINITY, 1.0, 10, Linear),
        ),
        (
            "NaN magnitude",
            AcConfig {
                ac_magnitude: f64::NAN,
                ..ac_config(1.0, 1e3, 10, Logarithmic)
            },
        ),
    ];
    for (case, config) in cases {
        assert_invalid(solve_ac(&circuit, &config), case);
    }
}

#[test]
fn ac_single_point_solves_f_start() {
    let circuit = rc_lowpass();
    for spacing in [FrequencySpacing::Logarithmic, FrequencySpacing::Linear] {
        let result = solve_ac(&circuit, &ac_config(159.155, 1e3, 1, spacing)).unwrap();
        assert_eq!(result.points.len(), 1);
        let point = &result.points[0];
        assert_eq!(point.frequency, 159.155);
        assert!((point.gain_db("out", 1.0).unwrap() + 3.0103).abs() < 1e-3);
    }
}

#[test]
fn ac_equal_endpoints_are_valid() {
    let circuit = rc_lowpass();
    for spacing in [FrequencySpacing::Logarithmic, FrequencySpacing::Linear] {
        let result = solve_ac(&circuit, &ac_config(100.0, 100.0, 5, spacing)).unwrap();
        assert!(result.points.iter().all(|p| p.frequency == 100.0));
    }
}

#[test]
fn ac_linear_sweep_from_zero_hz_is_finite() {
    let result = solve_ac(
        &rc_lowpass(),
        &ac_config(0.0, 1e3, 11, FrequencySpacing::Linear),
    )
    .unwrap();
    assert_eq!(result.points[0].frequency, 0.0);
    for point in &result.points {
        let v = point.node_voltages["out"];
        assert!(
            v.re.is_finite() && v.im.is_finite(),
            "at {} Hz: {v}",
            point.frequency
        );
    }
}

#[test]
fn dc_sweep_rejects_non_finite_endpoints() {
    let circuit = rc_lowpass();
    assert_invalid(dc_sweep(&circuit, "V1", f64::NAN, 1.0, 5), "NaN start");
    assert_invalid(dc_sweep(&circuit, "V1", 0.0, f64::INFINITY, 5), "inf stop");
    assert_invalid(dc_sweep(&circuit, "V1", 0.0, 1.0, 0), "zero points");
    assert_invalid(dc_sweep(&circuit, "V1", 0.0, 1.0, 1), "one point");
}

#[test]
fn temperature_sweep_rejects_invalid_temperatures() {
    let circuit = rc_lowpass();
    assert_invalid(temperature_sweep(&circuit, 0.0, 300.0, 5), "0 K start");
    assert_invalid(temperature_sweep(&circuit, 300.0, -10.0, 5), "negative end");
    assert_invalid(temperature_sweep(&circuit, f64::NAN, 300.0, 5), "NaN start");
    assert_invalid(
        temperature_sweep(&circuit, 300.0, f64::INFINITY, 5),
        "inf end",
    );
    assert_invalid(temperature_sweep(&circuit, 250.0, 350.0, 0), "zero steps");
}
