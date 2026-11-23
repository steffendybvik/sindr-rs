//! A transformer's saturating core: leakage on each winding and a shared
//! magnetising inductance on the primary that saturates, checked against
//! the closed form of its flux curve in transient and at its DC bias in AC.

use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::devices::inductor::InductorSaturation;
use sindr::element::{Resistor, Transformer, VoltageSource};
use sindr::{solve_transient, Circuit, CircuitElement, SimError, TransientConfig, Waveform};

const L1: f64 = 10e-3;
const L2: f64 = 2.5e-3;
const K: f64 = 0.99;

fn transformer(saturation: Option<InductorSaturation>) -> CircuitElement {
    CircuitElement::Transformer(Transformer {
        id: "t1".into(),
        nodes: ["p1".into(), "0".into(), "p2".into(), "0".into()],
        l1: L1,
        l2: L2,
        k: K,
        saturation,
    })
}

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn source(id: &str, node: &str, volts: f64, waveform: Option<Waveform>) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [node.into(), "0".into()],
        voltage: volts,
        waveform,
    })
}

/// The magnetising branch saturates at 1 A, keeping a twentieth of
/// k·L1 once it has.
fn core() -> InductorSaturation {
    InductorSaturation {
        i_sat: 1.0,
        l_sat: K * L1 / 20.0,
    }
}

/// The model's winding fluxes at winding currents `i1`, `i2`, written out
/// independently of the crate: leakage (1 − k)·L, magnetising flux of
/// `l_sat·im + (Lm − l_sat)·i_sat·tanh(im/i_sat)` on `im = i1 + n·i2`.
fn fluxes(i1: f64, i2: f64, sat: &InductorSaturation) -> (f64, f64) {
    let n = (L2 / L1).sqrt();
    let lm = K * L1;
    let im = i1 + n * i2;
    let flux_m = sat.l_sat * im + (lm - sat.l_sat) * sat.i_sat * (im / sat.i_sat).tanh();
    (
        (1.0 - K) * L1 * i1 + flux_m,
        (1.0 - K) * L2 * i2 + n * flux_m,
    )
}

/// A square wave of +10 V for 0.4 ms and −2.5 V for 0.1 ms, which never
/// resets the core: the volt-seconds left over each period walk it into
/// saturation, as a forward converter with a DC bias would.
fn offset_square_wave() -> Waveform {
    Waveform::Pulse {
        v1: -2.5,
        v2: 10.0,
        delay: 0.0,
        rise_time: 1e-7,
        fall_time: 1e-7,
        pulse_width: 0.4e-3,
        period: 0.5e-3,
    }
}

const R_LOAD: f64 = 1e3;

fn driven(saturation: Option<InductorSaturation>) -> Circuit {
    Circuit::new(
        "0",
        vec![
            source("v1", "p1", 0.0, Some(offset_square_wave())),
            transformer(saturation),
            r("rl", "p2", "0", R_LOAD),
        ],
    )
}

fn run(circuit: &Circuit) -> Vec<sindr::TimestepSnapshot> {
    solve_transient(circuit, &TransientConfig::new(5e-3, 1e-6))
        .unwrap()
        .transient
        .unwrap()
        .timesteps
}

fn primary_current(step: &sindr::TimestepSnapshot) -> f64 {
    step.component_results
        .iter()
        .find(|c| c.id == "t1")
        .unwrap()
        .current_through
}

/// Each winding's flux is the integral of its voltage (Faraday), and the
/// winding currents the solve found give exactly those fluxes on the
/// model's curve, all the way into saturation.
#[test]
fn fluxes_follow_the_curve_into_saturation() {
    let sat = core();
    let steps = run(&driven(Some(sat)));
    let (mut flux1, mut flux2) = (0.0, 0.0);
    let mut worst: f64 = 0.0;
    for w in steps.windows(2) {
        let dt = w[1].time - w[0].time;
        let v = |s: &sindr::TimestepSnapshot, n: &str| s.node_voltages[n];
        flux1 += 0.5 * dt * (v(&w[0], "p1") + v(&w[1], "p1"));
        flux2 += 0.5 * dt * (v(&w[0], "p2") + v(&w[1], "p2"));
        // The load carries the secondary's current out of p2.
        let i2 = -v(&w[1], "p2") / R_LOAD;
        let (model1, model2) = fluxes(primary_current(&w[1]), i2, &sat);
        worst = worst
            .max((model1 - flux1).abs())
            .max((model2 - flux2).abs());
    }
    // Each period leaves 4 − 0.25 = 3.75 mV·s, so ten end at 37.5 mV·s,
    // nearly five times the 7.8 mV·s at i_sat.
    let (lambda_sat, _) = fluxes(sat.i_sat, 0.0, &sat);
    close(flux1, 37.5e-3, 1e-3);
    assert!(flux1 > 4.0 * lambda_sat, "{flux1} vs {lambda_sat}");
    assert!(worst < 1e-5 * lambda_sat, "flux off the curve by {worst}");
}

/// The magnetising current spikes once the core saturates: in the first
/// pulse it rises at about V/L1 per second, and once saturated at exactly
/// V/(leakage + l_sat), sixteen times as fast.
#[test]
fn the_magnetising_current_spikes_once_saturated() {
    let steps = run(&driven(Some(core())));
    let slope_between = |from: f64, to: f64| {
        let at = |t: f64| {
            let s = steps
                .iter()
                .min_by(|a, b| (a.time - t).abs().total_cmp(&(b.time - t).abs()))
                .unwrap();
            (s.time, primary_current(s))
        };
        let (t0, i0) = at(from);
        let (t1, i1) = at(to);
        (i1 - i0) / (t1 - t0)
    };
    // First pulse: under 0.3 A, where the tanh has barely bent (4 %).
    let early = slope_between(0.1e-3, 0.3e-3);
    close(early, 10.0 / L1, 0.05);
    // Tenth pulse: some 45 A, deep in saturation.
    let late = slope_between(4.6e-3, 4.8e-3);
    let saturated = (1.0 - K) * L1 + core().l_sat;
    close(late, 10.0 / saturated, 1e-3);
    assert!(late > 15.0 * early, "{late} vs {early}");
}

/// A core that can't saturate (l_sat = k·L1) runs through the nonlinear
/// path, and matches the linear coupled inductors step for step.
#[test]
fn an_unsaturable_core_matches_the_linear_transformer() {
    let flat = InductorSaturation {
        i_sat: 1.0,
        l_sat: K * L1,
    };
    let linear = run(&driven(None));
    let nonlinear = run(&driven(Some(flat)));
    assert_eq!(linear.len(), nonlinear.len());
    for (a, b) in linear.iter().zip(&nonlinear) {
        close(primary_current(b), primary_current(a), 1e-6);
        close(b.node_voltages["p2"], a.node_voltages["p2"], 1e-6);
    }
}

/// With DC through the primary, AC sees the incremental inductance at that
/// bias: Ll1 + dλm/dim, far below L1 when the bias saturates the core.
#[test]
fn ac_sees_the_incremental_inductance_at_the_bias() {
    let sat = core();
    let r_series = 10.0;
    let bias = 20.0; // 2 A through the primary: twice i_sat.
    let circuit = Circuit::new(
        "0",
        vec![
            source("v1", "in", bias, None),
            r("rs", "in", "p1", r_series),
            transformer(Some(sat)),
            r("rl", "p2", "0", 1e9),
        ],
    );
    let f = 100.0;
    let ac = solve_ac(&circuit, &AcConfig::new("v1", f, f, 1)).unwrap();
    let got = ac.points[0].node_voltages["p1"].norm();
    let im = bias / r_series;
    let lm = K * L1;
    let sech = 1.0 / (im / sat.i_sat).cosh();
    let l_inc = (1.0 - K) * L1 + sat.l_sat + (lm - sat.l_sat) * sech * sech;
    let x = 2.0 * std::f64::consts::PI * f * l_inc;
    close(got, x / (r_series.powi(2) + x * x).sqrt(), 1e-4);
    // Without the bias it is the full L1.
    let mut unbiased = circuit.clone();
    unbiased.components[0] = source("v1", "in", 0.0, None);
    let ac = solve_ac(&unbiased, &AcConfig::new("v1", f, f, 1)).unwrap();
    let x = 2.0 * std::f64::consts::PI * f * L1;
    close(
        ac.points[0].node_voltages["p1"].norm(),
        x / (r_series.powi(2) + x * x).sqrt(),
        1e-4,
    );
}

#[test]
fn a_saturating_core_is_validated() {
    let check = |element: CircuitElement, field: &str| {
        let circuit = Circuit::new(
            "0",
            vec![
                source("v1", "p1", 1.0, None),
                element,
                r("rl", "p2", "0", 1e3),
            ],
        );
        match sindr::validate_circuit(&circuit) {
            Err(SimError::InvalidComponent(m)) => assert!(m.contains(field), "{m}"),
            other => panic!("{field}: {other:?}"),
        }
    };
    let with = |sat: InductorSaturation, k: f64| {
        CircuitElement::Transformer(Transformer {
            id: "t1".into(),
            nodes: ["p1".into(), "0".into(), "p2".into(), "0".into()],
            l1: L1,
            l2: L2,
            k,
            saturation: Some(sat),
        })
    };
    check(
        with(
            InductorSaturation {
                i_sat: 0.0,
                l_sat: 1e-4,
            },
            K,
        ),
        "i_sat",
    );
    check(
        with(
            InductorSaturation {
                i_sat: 1.0,
                l_sat: L1,
            },
            K,
        ),
        "magnetising",
    );
    check(with(core(), 0.0), "coupling");
}

#[cfg(feature = "spice")]
#[test]
fn spice_export_refuses_a_saturating_core() {
    let err = sindr::spice::to_netlist(&driven(Some(core())), "core").unwrap_err();
    assert!(err.to_string().contains("saturating core"), "{err}");
}

#[cfg(feature = "serde")]
#[test]
fn serde_carries_the_core_and_leaves_a_linear_one_out() {
    let json = serde_json::to_value(transformer(Some(core()))).unwrap();
    assert_eq!(json["saturation"]["i_sat"], 1.0);
    let back: CircuitElement = serde_json::from_value(json).unwrap();
    assert_eq!(back, transformer(Some(core())));
    let linear = serde_json::to_value(transformer(None)).unwrap();
    assert!(linear.get("saturation").is_none(), "{linear}");
}

fn close(got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1e-12),
        "got {got}, want {want}"
    );
}
