//! Circuits with exact solutions: DC sweeps against superposition and the
//! Shockley root, AC transfer functions against the analytic `H(jω)`, and
//! transient responses against both the exact solution and the
//! backward-Euler recurrence of the circuit's state equations.
//!
//! The expected values here come from the circuit equations alone, never from
//! sindr's stamps, so a wrong stamp or companion can't agree with them by
//! construction.

use std::f64::consts::PI;

use nalgebra::{DMatrix, DVector};
use num_complex::Complex64;
use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{
    dc_sweep, solve_transient, Circuit, CircuitElement, IntegrationMethod, TransientConfig,
    TransientData,
};

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
    }
}

fn vsource(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn capacitor(id: &str, a: &str, b: &str, capacitance: f64) -> CircuitElement {
    CircuitElement::Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance,
    }
}

fn inductor(id: &str, a: &str, b: &str, inductance: f64) -> CircuitElement {
    CircuitElement::Inductor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        inductance,
    }
}

// ----------------------------------------------------------------- DC sweep

/// Two sources into one node through R1 and R2, loaded by R3. By
/// superposition, v_out = (V1/R1 + V2/R2) / (1/R1 + 1/R2 + 1/R3) at every
/// value of the swept V1.
#[test]
fn dc_sweep_follows_superposition() {
    let (r1, r2, r3, v2) = (1e3, 2.2e3, 4.7e3, 3.0);
    let network = circuit(vec![
        vsource("V1", "a", "0", 0.0),
        vsource("V2", "b", "0", v2),
        resistor("R1", "a", "out", r1),
        resistor("R2", "b", "out", r2),
        resistor("R3", "out", "0", r3),
    ]);
    let sweep = dc_sweep(&network, "V1", -10.0, 10.0, 21).unwrap();
    assert_eq!(sweep.points.len(), 21);
    for p in &sweep.points {
        let v1 = p.sweep_value;
        let want = (v1 / r1 + v2 / r2) / (1.0 / r1 + 1.0 / r2 + 1.0 / r3);
        let got = p.result.node_voltages["out"];
        assert!(
            (got - want).abs() < 1e-12 * want.abs().max(1.0),
            "V1 = {v1}: {got} vs {want}"
        );
    }
}

/// Diode behind 1 kΩ, swept from reverse to well forward. Forward, the exact
/// drop is the root of `IS·(exp(V/V_T) − 1) = (Vs − V)/R` in `[0, Vs]`,
/// found here by bisection: the left side rises and the right falls.
#[test]
fn dc_sweep_diode_matches_shockley_root() {
    const V_T: f64 = 8.617_333_262e-5 * 300.15; // kT/q at 27 °C
    const IS: f64 = 1e-14;
    let r = 1e3;
    let network = circuit(vec![
        vsource("V1", "in", "0", 0.0),
        resistor("R1", "in", "out", r),
        CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
        },
    ]);
    let sweep = dc_sweep(&network, "V1", -2.0, 10.0, 25).unwrap();
    for p in &sweep.points {
        let vs = p.sweep_value;
        let got = p.result.node_voltages["out"];
        if vs <= 0.0 {
            // Reverse: the diode carries −IS, plus what sindr's 1e-12 S GMIN
            // shunts conduct, a few pA at these voltages.
            let i = (vs - got) / r;
            assert!(
                i <= 0.0 && i.abs() < IS + 3e-12 * vs.abs(),
                "Vs = {vs}: I = {i}"
            );
            continue;
        }
        let (mut lo, mut hi) = (0.0, vs);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if IS * ((mid / V_T).exp() - 1.0) > (vs - mid) / r {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        let v = 0.5 * (lo + hi);
        // Newton stops once a step is below RELTOL·V_T; the error left by
        // its quadratic tail is a few nV.
        assert!((got - v).abs() < 1e-7, "Vs = {vs}: sindr {got}, exact {v}");
    }
}

// ----------------------------------------------------------------------- AC

/// Log sweep of `circuit` from source `V1`, 10 points per decade.
fn ac_sweep(circuit: &Circuit, f_start: f64, f_stop: f64) -> sindr::ac_analysis::AcResult {
    let decades = (f_stop / f_start).log10().round() as usize;
    let config = AcConfig {
        f_start,
        f_stop,
        num_points: decades * 10 + 1,
        spacing: FrequencySpacing::Logarithmic,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    };
    solve_ac(circuit, &config).unwrap()
}

/// Every point of `node`'s response equals `h(ω)` to `rel` relative error,
/// compared as complex numbers (magnitude and phase together).
fn assert_transfer(
    circuit: &Circuit,
    node: &str,
    (f_start, f_stop): (f64, f64),
    h: impl Fn(f64) -> Complex64,
) {
    let result = ac_sweep(circuit, f_start, f_stop);
    for p in &result.points {
        let got = p.node_voltages[node];
        let want = h(2.0 * PI * p.frequency);
        assert!(
            (got - want).norm() <= 1e-9 * want.norm(),
            "f = {} Hz: sindr {got}, exact {want}",
            p.frequency
        );
    }
}

const J: Complex64 = Complex64::new(0.0, 1.0);

/// RC low-pass, 1 kΩ / 1 µF: H = 1 / (1 + jωRC).
#[test]
fn ac_rc_lowpass_matches_transfer_function() {
    let (r, c) = (1e3, 1e-6);
    let lowpass = circuit(vec![
        vsource("V1", "in", "0", 0.0),
        resistor("R1", "in", "out", r),
        capacitor("C1", "out", "0", c),
    ]);
    assert_transfer(&lowpass, "out", (1.0, 1e5), |w| 1.0 / (1.0 + J * w * r * c));

    // The corner: f_c = 1/(2πRC) = 159.155 Hz, −3.0103 dB, −45°.
    let f_c = 1.0 / (2.0 * PI * r * c);
    assert!((f_c - 159.155).abs() < 1e-3);
    let config = AcConfig {
        f_start: f_c,
        f_stop: f_c,
        num_points: 1,
        spacing: FrequencySpacing::Linear,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    };
    let at_corner = &solve_ac(&lowpass, &config).unwrap().points[0];
    assert!((at_corner.gain_db("out", 1.0).unwrap() + 3.010_299_957).abs() < 1e-8);
    assert!((at_corner.phase_deg("out").unwrap() + 45.0).abs() < 1e-8);
}

/// RC high-pass: H = jωRC / (1 + jωRC).
#[test]
fn ac_rc_highpass_matches_transfer_function() {
    let (r, c) = (2.2e3, 47e-9);
    let highpass = circuit(vec![
        vsource("V1", "in", "0", 0.0),
        capacitor("C1", "in", "out", c),
        resistor("R1", "out", "0", r),
    ]);
    assert_transfer(&highpass, "out", (10.0, 1e6), |w| {
        J * w * r * c / (1.0 + J * w * r * c)
    });
}

/// RL low-pass (series L, shunt R): H = R / (R + jωL).
#[test]
fn ac_rl_lowpass_matches_transfer_function() {
    let (r, l) = (100.0, 10e-3);
    let lowpass = circuit(vec![
        vsource("V1", "in", "0", 0.0),
        inductor("L1", "in", "out", l),
        resistor("R1", "out", "0", r),
    ]);
    assert_transfer(&lowpass, "out", (10.0, 1e6), |w| r / (r + J * w * l));
}

/// RL high-pass (series R, shunt L): H = jωL / (R + jωL).
#[test]
fn ac_rl_highpass_matches_transfer_function() {
    let (r, l) = (100.0, 10e-3);
    let highpass = circuit(vec![
        vsource("V1", "in", "0", 0.0),
        resistor("R1", "in", "out", r),
        inductor("L1", "out", "0", l),
    ]);
    assert_transfer(&highpass, "out", (10.0, 1e6), |w| {
        J * w * l / (r + J * w * l)
    });
}

/// Series RLC, output across R: H = R / (R + jωL + 1/(jωC)). At resonance,
/// f₀ = 1/(2π√(LC)), the reactances cancel: |H| = 1, phase 0.
#[test]
fn ac_series_rlc_resonance() {
    let (r, l, c) = (10.0, 1e-3, 1e-6);
    let series = circuit(vec![
        vsource("V1", "in", "0", 0.0),
        inductor("L1", "in", "a", l),
        capacitor("C1", "a", "out", c),
        resistor("R1", "out", "0", r),
    ]);
    let h = |w: f64| r / (r + J * w * l + 1.0 / (J * w * c));
    assert_transfer(&series, "out", (100.0, 1e6), h);

    let f0 = 1.0 / (2.0 * PI * (l * c).sqrt());
    let config = AcConfig {
        f_start: f0,
        f_stop: f0,
        num_points: 1,
        spacing: FrequencySpacing::Linear,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    };
    let at_f0 = solve_ac(&series, &config).unwrap().points[0].node_voltages["out"];
    assert!((at_f0 - Complex64::new(1.0, 0.0)).norm() < 1e-9, "{at_f0}");
}

/// Parallel RLC tank fed through R_s: H = Z / (R_s + Z), with
/// 1/Z = 1/R + jωC + 1/(jωL). At f₀ the tank is purely resistive.
#[test]
fn ac_parallel_rlc_tank() {
    let (rs, r, l, c) = (1e3, 10e3, 100e-6, 10e-9);
    let tank = circuit(vec![
        vsource("V1", "in", "0", 0.0),
        resistor("RS", "in", "out", rs),
        resistor("RP", "out", "0", r),
        inductor("L1", "out", "0", l),
        capacitor("C1", "out", "0", c),
    ]);
    let h = |w: f64| {
        let z = 1.0 / (1.0 / r + J * w * c + 1.0 / (J * w * l));
        z / (rs + z)
    };
    assert_transfer(&tank, "out", (1e3, 1e7), h);
}

/// Unity-gain Sallen-Key low-pass with an ideal buffer (a VCVS of gain 1):
/// H = 1 / (1 + jω·C2·(R1 + R2) − ω²·R1·R2·C1·C2), with C1 the feedback
/// capacitor and C2 the one to ground.
#[test]
fn ac_sallen_key_lowpass_matches_transfer_function() {
    let (r1, r2, c1, c2) = (10e3, 10e3, 22e-9, 10e-9);
    let sallen_key = circuit(vec![
        vsource("V1", "in", "0", 0.0),
        resistor("R1", "in", "a", r1),
        resistor("R2", "a", "b", r2),
        capacitor("C1", "a", "out", c1),
        capacitor("C2", "b", "0", c2),
        CircuitElement::Vcvs {
            id: "E1".into(),
            nodes: ["out".into(), "0".into()],
            control_nodes: ["b".into(), "0".into()],
            gain: 1.0,
        },
    ]);
    let h = |w: f64| 1.0 / (1.0 + J * w * c2 * (r1 + r2) - w * w * r1 * r2 * c1 * c2);
    assert_transfer(&sallen_key, "out", (10.0, 1e6), h);
}

// ---------------------------------------------------------------- transient

const METHODS: [IntegrationMethod; 3] = [
    IntegrationMethod::BackwardEuler,
    IntegrationMethod::Trapezoidal,
    IntegrationMethod::Gear2,
];

/// Transient of `circuit` with its automatic span and step, integrated by
/// `method`.
fn transient(circuit: &Circuit, method: IntegrationMethod) -> TransientData {
    let config = TransientConfig::auto(circuit).with_method(method);
    solve_transient(circuit, &config)
        .unwrap()
        .transient
        .expect("transient result")
}

/// `method` on `x' = A·x + b` from `x = 0`, for `steps` steps of `dt`, in
/// the textbook state-space forms (the first step is always backward Euler):
///
/// - backward Euler: `(I − dt·A)·x_n = x_{n−1} + dt·b`
/// - trapezoidal: `(I − dt/2·A)·x_n = (I + dt/2·A)·x_{n−1} + dt·b`
/// - BDF2: `(3/2·I − dt·A)·x_n = 2·x_{n−1} − 1/2·x_{n−2} + dt·b`
///
/// sindr builds its companions element by element; on a linear circuit they
/// must reproduce these recurrences exactly.
fn integrate(
    method: IntegrationMethod,
    a: &DMatrix<f64>,
    b: &DVector<f64>,
    dt: f64,
    steps: usize,
) -> Vec<DVector<f64>> {
    let n = b.len();
    let id = DMatrix::<f64>::identity(n, n);
    let mut out: Vec<DVector<f64>> = Vec::with_capacity(steps);
    let (mut x1, mut x2) = (DVector::zeros(n), DVector::zeros(n));
    for step in 0..steps {
        let (lhs, rhs) = match method {
            _ if step == 0 => (&id - a * dt, &x1 + b * dt),
            IntegrationMethod::BackwardEuler => (&id - a * dt, &x1 + b * dt),
            IntegrationMethod::Trapezoidal => {
                (&id - a * (dt / 2.0), (&id + a * (dt / 2.0)) * &x1 + b * dt)
            }
            IntegrationMethod::Gear2 => (&id * 1.5 - a * dt, &x1 * 2.0 - &x2 * 0.5 + b * dt),
            _ => unreachable!(),
        };
        let x = lhs.lu().solve(&rhs).unwrap();
        x2 = std::mem::replace(&mut x1, x.clone());
        out.push(x);
    }
    out
}

/// The current through component `id` at snapshot `s`.
fn current(s: &sindr::TimestepSnapshot, id: &str) -> f64 {
    s.component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap()
        .current_through
}

/// RC charging from 1 V: `v' = (V − v)/τ`. For every method, sindr's steps
/// match the method's recurrence to round-off, snapshot `i` being at
/// `t = (i + 1)·dt`.
#[test]
fn transient_rc_step_matches_each_methods_recurrence() {
    let (r, c, v) = (1e3, 1e-6, 1.0);
    let rc = circuit(vec![
        vsource("V1", "in", "0", v),
        resistor("R1", "in", "out", r),
        capacitor("C1", "out", "0", c),
    ]);
    let tau = r * c;
    let a = DMatrix::from_element(1, 1, -1.0 / tau);
    let b = DVector::from_element(1, v / tau);
    for method in METHODS {
        let tran = transient(&rc, method);
        let dt = tran.time_step;
        assert!((dt - tau / 50.0).abs() < 1e-15);
        let want = integrate(method, &a, &b, dt, tran.timesteps.len());
        for (i, (s, x)) in tran.timesteps.iter().zip(&want).enumerate() {
            assert!((s.time - (i + 1) as f64 * dt).abs() < 1e-12 * dt.max(s.time));
            let got = s.node_voltages["out"];
            assert!(
                (got - x[0]).abs() < 1e-12,
                "{method:?}, t = {}: sindr {got}, recurrence {}",
                s.time,
                x[0]
            );
            // And within truncation error of the exact 1 − e^(−t/τ).
            let exact = v * (1.0 - (-s.time / tau).exp());
            assert!((got - exact).abs() < 0.01 * v, "{method:?}, t = {}", s.time);
        }
    }
}

/// Series RLC step: `L·i' = V − R·i − v_C`, `C·v_C' = i`, underdamped
/// (ζ ≈ 0.016). The inductor current and the capacitor voltage follow each
/// method's recurrence, numerical damping included.
#[test]
fn transient_rlc_step_matches_each_methods_recurrence() {
    let (r, l, c, v) = (1.0, 1e-3, 1e-6, 1.0);
    let rlc = circuit(vec![
        vsource("V1", "in", "0", v),
        resistor("R1", "in", "a", r),
        inductor("L1", "a", "out", l),
        capacitor("C1", "out", "0", c),
    ]);
    // x = [i_L, v_C]
    let a = DMatrix::from_row_slice(2, 2, &[-r / l, -1.0 / l, 1.0 / c, 0.0]);
    let b = DVector::from_row_slice(&[v / l, 0.0]);
    let i_scale = v * (c / l).sqrt();
    for method in METHODS {
        let tran = transient(&rlc, method);
        let want = integrate(method, &a, &b, tran.time_step, tran.timesteps.len());
        for (s, x) in tran.timesteps.iter().zip(&want) {
            let (i_l, v_c) = (current(s, "L1"), s.node_voltages["out"]);
            assert!(
                (i_l - x[0]).abs() < 1e-9 * i_scale,
                "{method:?}, t = {}: i_L sindr {i_l}, recurrence {}",
                s.time,
                x[0]
            );
            assert!(
                (v_c - x[1]).abs() < 1e-9 * v,
                "{method:?}, t = {}: v_C sindr {v_c}, recurrence {}",
                s.time,
                x[1]
            );
        }
    }
}

/// Least-squares slope of `ln(err)` against `ln(dt)`.
fn log_log_slope(points: &[(f64, f64)]) -> f64 {
    let xs: Vec<f64> = points.iter().map(|p| p.0.ln()).collect();
    let ys: Vec<f64> = points.iter().map(|p| p.1.ln()).collect();
    let n = xs.len() as f64;
    let (mx, my) = (xs.iter().sum::<f64>() / n, ys.iter().sum::<f64>() / n);
    let sxy: f64 = xs.iter().zip(&ys).map(|(x, y)| (x - mx) * (y - my)).sum();
    let sxx: f64 = xs.iter().map(|x| (x - mx) * (x - mx)).sum();
    sxy / sxx
}

/// The final snapshot of a run of `circuit` from 0 to `tstop` in steps of
/// `dt`, integrated by `method`.
fn run_to(
    circuit: &Circuit,
    tstop: f64,
    dt: f64,
    method: IntegrationMethod,
) -> sindr::TimestepSnapshot {
    let config = TransientConfig::new(tstop, dt).with_method(method);
    let tran = solve_transient(circuit, &config)
        .unwrap()
        .transient
        .unwrap();
    let last = tran.timesteps.last().unwrap().clone();
    assert_eq!(last.time, tstop);
    last
}

/// Steps per time constant for the convergence tests: dt from τ/25 to τ/400.
const STEPS_PER_TAU: [f64; 5] = [25.0, 50.0, 100.0, 200.0, 400.0];

/// The order each method's error falls with dt.
fn order(method: IntegrationMethod) -> f64 {
    match method {
        IntegrationMethod::BackwardEuler => 1.0,
        _ => 2.0,
    }
}

/// On an RC step the error at t = τ falls as dt for backward Euler and as
/// dt² for trapezoidal and Gear-2.
#[test]
fn transient_rc_error_falls_with_each_methods_order() {
    let (r, c, v) = (1e3, 1e-6, 1.0);
    let tau = r * c;
    let rc = circuit(vec![
        vsource("V1", "in", "0", v),
        resistor("R1", "in", "out", r),
        capacitor("C1", "out", "0", c),
    ]);
    for method in METHODS {
        let errors: Vec<(f64, f64)> = STEPS_PER_TAU
            .iter()
            .map(|n| {
                let dt = tau / n;
                let got = run_to(&rc, tau, dt, method).node_voltages["out"];
                (dt, (got - v * (1.0 - (-1.0_f64).exp())).abs())
            })
            .collect();
        let slope = log_log_slope(&errors);
        assert!(
            (slope - order(method)).abs() < 0.05,
            "{method:?}: order {slope}, errors {errors:?}"
        );
    }
}

/// The same for an RL step: the inductor current at t = τ = L/R.
#[test]
fn transient_rl_error_falls_with_each_methods_order() {
    let (r, l, v) = (100.0, 10e-3, 1.0);
    let tau = l / r;
    let rl = circuit(vec![
        vsource("V1", "in", "0", v),
        resistor("R1", "in", "a", r),
        inductor("L1", "a", "0", l),
    ]);
    for method in METHODS {
        let errors: Vec<(f64, f64)> = STEPS_PER_TAU
            .iter()
            .map(|n| {
                let dt = tau / n;
                let got = current(&run_to(&rl, tau, dt, method), "L1");
                (dt, (got - v / r * (1.0 - (-1.0_f64).exp())).abs())
            })
            .collect();
        let slope = log_log_slope(&errors);
        assert!(
            (slope - order(method)).abs() < 0.05,
            "{method:?}: order {slope}, errors {errors:?}"
        );
    }
}

/// Undamped LC tank stepped from 0 to 1 V: v_C = 1 − cos(ω₀t), swinging
/// between 0 and 2 V forever. Over ten periods at 50 steps per period, the
/// trapezoidal rule keeps the swing (it conserves the tank's energy), Gear-2
/// loses a little, and backward Euler damps most of it away.
#[test]
fn lc_tank_amplitude_by_method() {
    let (l, c): (f64, f64) = (1e-3, 1e-6);
    let period = 2.0 * PI * (l * c).sqrt();
    let tank = circuit(vec![
        vsource("V1", "in", "0", 1.0),
        inductor("L1", "in", "out", l),
        capacitor("C1", "out", "0", c),
    ]);
    // Peak-to-peak swing of v_C over the tenth period.
    let swing = |method| {
        let config = TransientConfig::new(10.0 * period, period / 50.0).with_method(method);
        let tran = solve_transient(&tank, &config).unwrap().transient.unwrap();
        let last: Vec<f64> = tran
            .timesteps
            .iter()
            .filter(|s| s.time > 9.0 * period)
            .map(|s| s.node_voltages["out"])
            .collect();
        let max = last.iter().copied().fold(f64::MIN, f64::max);
        let min = last.iter().copied().fold(f64::MAX, f64::min);
        max - min
    };
    let trap = swing(IntegrationMethod::Trapezoidal);
    let gear = swing(IntegrationMethod::Gear2);
    let be = swing(IntegrationMethod::BackwardEuler);
    assert!((trap - 2.0).abs() < 0.02, "trapezoidal swing {trap}");
    assert!(gear < trap && gear > 1.5, "Gear-2 swing {gear}");
    assert!(be < 0.5, "backward Euler swing {be}");
}
