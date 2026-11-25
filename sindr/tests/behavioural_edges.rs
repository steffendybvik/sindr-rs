//! Behavioural sources where Newton has to work for the answer: far from
//! it at the start, across overflow, domain edges and kinks, in every
//! analysis. `behavioural.rs` has the closed forms near the answer.

use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::element::{Behavioural, Capacitor, Inductor, Resistor, VoltageSource};
use sindr::expression::Expression;
use sindr::DcSweepAxis;
use sindr::{
    dc_sweep, solve_op, solve_transient, BehaviouralOutput, Circuit, CircuitElement,
    ConvergenceAid, SimError, SimulationResult, Solver, SolverOptions, TransientConfig,
};

const V: BehaviouralOutput = BehaviouralOutput::Voltage;
const I: BehaviouralOutput = BehaviouralOutput::Current;
const VT: f64 = 0.02585;
const IS: f64 = 1e-14;

fn v(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    })
}

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn c(id: &str, a: &str, b: &str, farads: f64) -> CircuitElement {
    CircuitElement::Capacitor(Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance: farads,
    })
}

fn b(id: &str, p: &str, n: &str, output: BehaviouralOutput, expr: &str) -> CircuitElement {
    CircuitElement::Behavioural(Behavioural {
        id: id.into(),
        nodes: [p.into(), n.into()],
        output,
        expr: Expression::parse(expr).unwrap(),
    })
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit::new("0", components)
}

fn op_with(circuit: &Circuit, opts: SolverOptions) -> Result<SimulationResult, SimError> {
    Solver::new(opts).op(circuit)
}

fn seeded(seeds: &[(&str, f64)]) -> SolverOptions {
    SolverOptions::default().with_nodeset(seeds.iter().map(|(n, v)| (n.to_string(), *v)).collect())
}

/// Newton stops once a node moves less than `vntol + reltol·|V|` (1 µV +
/// 1e-3 relative) per step, so a node set through an exponential is good
/// to a few µV, not to rounding: those comparisons use [`SETTLED`].
const SETTLED: f64 = 1e-4;

fn close(got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1.0),
        "got {got}, want {want}"
    );
}

/// Root of `f` on `[lo, hi]` by bisection, for closed forms with no
/// algebraic solution.
fn bisect(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    let rising = f(hi) > f(lo);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if (f(mid) > 0.0) == rising {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    0.5 * (lo + hi)
}

fn diode_expr(node: &str) -> String {
    format!("{IS}*(exp(v({node})/{VT})-1)")
}

/// The junction voltage of `n` B-source diodes in series behind `ohms`.
fn diode_drop(supply: f64, ohms: f64, n: f64) -> f64 {
    bisect(
        |vd| (supply - n * vd) / ohms - IS * ((vd / VT).exp() - 1.0),
        0.0,
        1.0,
    )
}

// ------------------------------------------------- far from the answer

/// The first iterate puts the whole supply across the junction, and
/// `exp(supply/VT)` overflows from about 18 V; the solve still lands.
#[test]
fn an_exponential_solves_from_any_supply() {
    for supply in [1.0, 5.0, 20.0, 100.0, 1000.0] {
        let result = solve_op(&circuit(vec![
            v("v1", "in", "0", supply),
            r("r1", "in", "a", 1e3),
            b("b1", "a", "0", I, &diode_expr("a")),
        ]))
        .unwrap_or_else(|e| panic!("{supply} V: {e}"));
        close(
            result.node_voltages["a"],
            diode_drop(supply, 1e3, 1.0),
            SETTLED,
        );
    }
}

#[test]
fn exponentials_in_series_solve() {
    let result = solve_op(&circuit(vec![
        v("v1", "in", "0", 50.0),
        r("r1", "in", "a", 1e3),
        b("b1", "a", "m", I, "1e-14*(exp(v(a,m)/0.02585)-1)"),
        b("b2", "m", "0", I, &diode_expr("m")),
    ]))
    .unwrap();
    let vd = diode_drop(50.0, 1e3, 2.0);
    close(result.node_voltages["m"], vd, SETTLED);
    close(result.node_voltages["a"], 2.0 * vd, SETTLED);
}

/// Anti-parallel junctions conduct whichever way the supply points.
#[test]
fn anti_parallel_exponentials_solve_both_ways() {
    for supply in [-30.0, 30.0] {
        let result = solve_op(&circuit(vec![
            v("v1", "in", "0", supply),
            r("r1", "in", "a", 1e3),
            b("b1", "a", "0", I, &diode_expr("a")),
            b("b2", "0", "a", I, "1e-14*(exp(-v(a)/0.02585)-1)"),
        ]))
        .unwrap_or_else(|e| panic!("{supply} V: {e}"));
        let want = diode_drop(supply.abs(), 1e3, 1.0).copysign(supply);
        close(result.node_voltages["a"], want, SETTLED);
    }
}

#[test]
fn power_laws_solve_from_a_large_supply() {
    // 1e-3·v³ = (100 − v)/1k.
    let result = solve_op(&circuit(vec![
        v("v1", "in", "0", 100.0),
        r("r1", "in", "a", 1e3),
        b("b1", "a", "0", I, "1e-3*v(a)^3"),
    ]))
    .unwrap();
    let want = bisect(|x| (100.0 - x) / 1e3 - 1e-3 * x.powi(3), 0.0, 100.0);
    close(result.node_voltages["a"], want, 1e-8);
    // pwr keeps the sign: sgn(v)·|v|^1.5 mA, from a negative supply.
    let result = solve_op(&circuit(vec![
        v("v1", "in", "0", -100.0),
        r("r1", "in", "a", 1e3),
        b("b1", "a", "0", I, "1e-3*pwr(v(a),1.5)"),
    ]))
    .unwrap();
    let want = -bisect(|x| (100.0 - x) / 1e3 - 1e-3 * x.powf(1.5), 0.0, 100.0);
    close(result.node_voltages["a"], want, 1e-8);
}

/// A source that reads its own node at high gain: v = 10·tanh(v) + 1 has
/// three roots (near −9, −0.11 and 11), and Newton lands on one of them.
#[test]
fn a_source_reading_its_own_output_solves() {
    let result = solve_op(&circuit(vec![
        v("v1", "in", "0", 1.0),
        b("b1", "out", "0", V, "10*tanh(v(out)) + v(in)"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap();
    let out = result.node_voltages["out"];
    assert!((10.0 * out.tanh() + 1.0 - out).abs() < 1e-6, "{out}");
}

/// A nodeset on the wrong side of the answer still solves.
#[test]
fn a_nodeset_pointing_the_wrong_way_still_solves() {
    let circuit = circuit(vec![
        v("v1", "in", "0", 20.0),
        r("r1", "in", "a", 1e3),
        b("b1", "a", "0", I, &diode_expr("a")),
    ]);
    for seed in [-50.0, 50.0] {
        let result =
            op_with(&circuit, seeded(&[("a", seed)])).unwrap_or_else(|e| panic!("{seed}: {e}"));
        close(
            result.node_voltages["a"],
            diode_drop(20.0, 1e3, 1.0),
            SETTLED,
        );
    }
}

/// The aid that got the solve there is reported.
#[test]
fn the_rescuing_aid_is_reported() {
    let result = solve_op(&circuit(vec![
        v("v1", "in", "0", 100.0),
        r("r1", "in", "a", 1e3),
        b("b1", "a", "0", I, &diode_expr("a")),
    ]))
    .unwrap();
    // Gmin stepping still starts from 0 V and overflows; source stepping
    // brings the supply up from near 0 V, where exp stays finite.
    assert_eq!(result.convergence_aid, Some(ConvergenceAid::SourceStepping));
    // With every aid off, the overflow is the error, naming the source.
    let mut opts = SolverOptions::default();
    opts.gmin_steps = Vec::new();
    opts.source_stepping = false;
    let circuit = circuit(vec![
        v("v1", "in", "0", 100.0),
        r("r1", "in", "a", 1e3),
        b("b1", "a", "0", I, &diode_expr("a")),
    ]);
    match op_with(&circuit, opts) {
        Ok(result) => close(
            result.node_voltages["a"],
            diode_drop(100.0, 1e3, 1.0),
            SETTLED,
        ),
        Err(e) => assert!(
            matches!(&e, SimError::NotFinite { component, .. } if component == "b1")
                || matches!(e, SimError::ConvergenceFailed { .. }),
            "{e}"
        ),
    }
}

// --------------------------------------------- overflow and non-finite

#[test]
fn an_expression_never_finite_is_named_after_every_aid() {
    for expr in ["ln(-1-v(a)^2)", "sqrt(-1-v(a)^2)", "1/(v(a)-v(a))"] {
        let err = solve_op(&circuit(vec![
            v("v1", "a", "0", 1.0),
            b("b1", "out", "0", V, expr),
            r("r1", "out", "0", 1e3),
        ]))
        .unwrap_err();
        assert!(
            matches!(&err, SimError::NotFinite { component, expression, .. }
                if component == "b1" && !expression.is_empty()),
            "{expr}: {err}"
        );
    }
}

/// A quotient that is 0/0 at the starting point but finite at the answer.
#[test]
fn a_quotient_undefined_at_the_start_solves() {
    let result = solve_op(&circuit(vec![
        v("v1", "a", "0", 3.0),
        v("v2", "b", "0", 2.0),
        b("b1", "out", "0", V, "v(a)/v(b)"),
        r("r1", "out", "0", 1e3),
    ]));
    close(result.unwrap().node_voltages["out"], 1.5, 1e-12);
}

/// A slope that overflows while the value doesn't: the step is flat and
/// the solve carries on.
#[test]
fn an_overflowing_slope_alone_is_survived() {
    let result = solve_op(&circuit(vec![
        v("v1", "a", "0", 0.0),
        b("b1", "out", "0", V, "sqrt(v(a)) + 1"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["out"], 1.0, 1e-12);
}

/// A value that is finite but enormous is just a big answer: 1e300 A into
/// 1 kΩ.
#[test]
fn a_huge_finite_value_is_just_a_big_answer() {
    let result = solve_op(&circuit(vec![
        v("v1", "a", "0", 1.0),
        b("b1", "out", "0", I, "1e300*v(a)"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["out"], -1e303, 1e-8);
}

// --------------------------------------------------------- domain edges

/// sqrt and ln of a node that the first step may take negative, but that
/// settles positive.
#[test]
fn a_domain_left_on_the_way_but_not_at_the_answer_solves() {
    for (expr, want) in [("sqrt(v(m))", 0.5_f64.sqrt()), ("ln(v(m))", 0.5_f64.ln())] {
        let result = solve_op(&circuit(vec![
            v("v1", "a", "0", 1.0),
            r("r1", "a", "m", 1e3),
            r("r2", "m", "0", 1e3),
            b("b1", "out", "0", V, expr),
            r("r3", "out", "0", 1e3),
        ]))
        .unwrap_or_else(|e| panic!("{expr}: {e}"));
        close(result.node_voltages["out"], want, 1e-8);
    }
}

#[test]
fn pow_with_negative_bases_and_varying_exponents() {
    let op = |a: f64, bv: f64, expr: &str| {
        solve_op(&circuit(vec![
            v("va", "a", "0", a),
            v("vb", "b", "0", bv),
            r("rb", "b", "0", 1e3),
            b("b1", "out", "0", V, expr),
            r("r1", "out", "0", 1e3),
        ]))
    };
    // A negative base with an integer exponent is real.
    close(
        op(-2.0, 3.0, "v(a)^v(b)").unwrap().node_voltages["out"],
        -8.0,
        1e-12,
    );
    close(
        op(-2.0, 0.0, "pow(v(a),3)").unwrap().node_voltages["out"],
        -8.0,
        1e-12,
    );
    // Both varying, positive base.
    close(
        op(2.0, 0.5, "v(a)^v(b)").unwrap().node_voltages["out"],
        2.0_f64.sqrt(),
        1e-12,
    );
    // x^0 is 1 for every x, 0^0 included.
    close(
        op(0.0, 0.0, "v(a)^0").unwrap().node_voltages["out"],
        1.0,
        1e-12,
    );
    close(
        op(0.0, 0.0, "v(a)^v(b)").unwrap().node_voltages["out"],
        1.0,
        1e-12,
    );
    // A negative base with a fractional exponent, and 0^-1, aren't finite.
    for (a, bv, expr) in [(-2.0, 0.5, "v(a)^v(b)"), (0.0, 0.0, "v(a)^-1")] {
        let err = op(a, bv, expr).unwrap_err();
        assert!(matches!(err, SimError::NotFinite { .. }), "{expr}: {err}");
    }
}

#[test]
fn inverse_trig_at_and_past_its_edges() {
    let op = |a: f64, expr: &str| {
        solve_op(&circuit(vec![
            v("va", "a", "0", a),
            b("b1", "out", "0", V, expr),
            r("r1", "out", "0", 1e3),
        ]))
    };
    let pi = std::f64::consts::PI;
    close(
        op(1.0, "asin(v(a))").unwrap().node_voltages["out"],
        pi / 2.0,
        1e-12,
    );
    close(
        op(-1.0, "acos(v(a))").unwrap().node_voltages["out"],
        pi,
        1e-12,
    );
    for expr in ["asin(v(a))", "acos(v(a))"] {
        let err = op(1.0 + 1e-9, expr).unwrap_err();
        assert!(matches!(err, SimError::NotFinite { .. }), "{expr}: {err}");
    }
    // atan2(0, 0) is 0 in IEEE arithmetic; the solve doesn't choke on it.
    close(
        op(0.0, "atan2(v(a), v(a))").unwrap().node_voltages["out"],
        0.0,
        1e-12,
    );
    // tan just short of π/2 is huge but finite.
    let out = op(pi / 2.0 - 1e-6, "tan(v(a))").unwrap().node_voltages["out"];
    close(out, 1e6, 1e-6);
}

// ------------------------------------------------ kinks and switching

/// A precision clamp: out = min(v(in), 2) into a load; the answer at the
/// kink itself, and either side of it.
#[test]
fn min_max_limit_and_abs_settle_on_and_off_their_kinks() {
    for (vin, expr, want) in [
        (1.0, "min(v(in), 2)", 1.0),
        (2.0, "min(v(in), 2)", 2.0),
        (3.0, "min(v(in), 2)", 2.0),
        (-1.0, "max(v(in), 0)", 0.0),
        (0.0, "max(v(in), 0)", 0.0),
        (-3.0, "limit(v(in), -2, 2)", -2.0),
        (2.0, "limit(v(in), -2, 2)", 2.0),
        (0.0, "abs(v(in))", 0.0),
        (-4.0, "abs(v(in))", 4.0),
    ] {
        let result = solve_op(&circuit(vec![
            v("v1", "in", "0", vin),
            b("b1", "out", "0", V, expr),
            r("r1", "out", "0", 1e3),
        ]))
        .unwrap_or_else(|e| panic!("{expr} at {vin}: {e}"));
        close(result.node_voltages["out"], want, 1e-12);
    }
}

/// An ideal rectifier made with a kink, solved through a divider, where
/// its input is set by the circuit rather than by a source.
#[test]
fn an_ideal_rectifier_in_a_loop_converges() {
    for (supply, want) in [(10.0, 5.0), (-10.0, 0.0)] {
        let result = solve_op(&circuit(vec![
            v("v1", "in", "0", supply),
            r("r1", "in", "m", 1e3),
            r("r2", "m", "0", 1e3),
            b("b1", "out", "0", V, "uramp(v(m))"),
            r("r3", "out", "0", 1e3),
        ]))
        .unwrap();
        close(result.node_voltages["out"], want, 1e-8);
    }
}

/// A switch whose threshold sits right at its operating point.
#[test]
fn a_switch_threshold_at_the_operating_point() {
    for expr in ["v(in) > 1 ? 5 : 0", "5*u(v(in) - 1)", "5*(v(in) >= 1)"] {
        let result = solve_op(&circuit(vec![
            v("v1", "in", "0", 1.0),
            b("b1", "out", "0", V, expr),
            r("r1", "out", "0", 1e3),
        ]))
        .unwrap_or_else(|e| panic!("{expr}: {e}"));
        let out = result.node_voltages["out"];
        assert!(out == 0.0 || out == 5.0, "{expr}: {out}");
    }
}

/// floor has no slope anywhere: the node it drives is set by the source,
/// not by Newton's slope.
#[test]
fn a_slopeless_function_still_solves() {
    let result = solve_op(&circuit(vec![
        v("v1", "in", "0", 3.7),
        b("b1", "out", "0", V, "floor(v(in)) + ceil(v(in))"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["out"], 7.0, 1e-12);
}

/// A comparator with hysteresis has two valid answers: the nodeset picks.
#[test]
fn hysteresis_has_two_answers_and_the_nodeset_picks() {
    let circuit = circuit(vec![
        v("v1", "in", "0", 2.5),
        // Threshold 2.5 ± 0.5 V from the output: high out raises it.
        b("b1", "out", "0", V, "v(in) > (2 + v(fb)/5) ? 0 : 5"),
        r("r1", "out", "fb", 1e3),
        r("r2", "fb", "0", 1e9),
    ]);
    for seed in [0.0, 5.0] {
        let result = op_with(&circuit, seeded(&[("out", seed), ("fb", seed)]))
            .unwrap_or_else(|e| panic!("{seed}: {e}"));
        let out = result.node_voltages["out"];
        assert!(out == 0.0 || out == 5.0, "{seed}: {out}");
    }
}

/// An inverting comparator fed back on itself has no DC answer: a clear
/// convergence error, not a hang or a made-up point.
#[test]
fn an_oscillator_with_no_dc_answer_fails_cleanly() {
    let err = solve_op(&circuit(vec![
        v("v1", "in", "0", 1.0),
        r("r1", "in", "a", 1e3),
        b("b1", "out", "0", V, "v(a) > 0.5 ? 0 : 5"),
        r("r2", "out", "a", 1e3),
    ]))
    .unwrap_err();
    assert!(matches!(err, SimError::ConvergenceFailed { .. }), "{err}");
}

/// A DC sweep across a kink loses no points and stays monotonic.
#[test]
fn a_sweep_across_a_kink_keeps_every_point() {
    let circuit = circuit(vec![
        v("v1", "in", "0", 0.0),
        b("b1", "out", "0", V, "limit(2*v(in), -1, 1)"),
        r("r1", "out", "0", 1e3),
    ]);
    let sweep = dc_sweep(&circuit, &DcSweepAxis::new("v1", -2.0, 2.0, 401)).unwrap();
    assert_eq!(sweep.points.len(), 401);
    let outs: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["out"])
        .collect();
    assert!(outs.windows(2).all(|w| w[1] >= w[0] - 1e-12));
    for (p, out) in sweep.points.iter().zip(&outs) {
        close(
            *out,
            (2.0 * p.result.node_voltages["in"]).clamp(-1.0, 1.0),
            1e-9,
        );
    }
}

// ------------------------------------------------------------ transient

/// A square-law current charging a capacitor: dv/dt = (1 − v)²·k/C has
/// the closed form v = 1 − 1/(1 + k·t/C).
#[test]
fn a_nonlinear_source_charging_a_capacitor() {
    let k = 1e-3;
    let cap = 1e-6;
    let result = solve_transient(
        &circuit(vec![
            v("v1", "in", "0", 1.0),
            b("b1", "in", "a", I, "1e-3*(v(in)-v(a))^2"),
            c("c1", "a", "0", cap),
        ]),
        &TransientConfig::new(5e-3, 1e-6),
    )
    .unwrap();
    for step in result.transient.unwrap().timesteps.iter().skip(1) {
        let want = 1.0 - 1.0 / (1.0 + k * step.time / cap);
        close(step.node_voltages["a"], want, 2e-3);
    }
}

/// A time switch between grid points: the edge is resolved and the step
/// grows back afterwards.
#[test]
fn a_time_edge_off_the_grid_is_resolved() {
    let result = solve_transient(
        &circuit(vec![
            b("b1", "a", "0", V, "time > 0.3333m ? 1 : 0"),
            r("r1", "a", "m", 1e3),
            c("c1", "m", "0", 1e-9),
        ]),
        &TransientConfig::new(1e-3, 1e-4),
    )
    .unwrap();
    let steps = result.transient.unwrap().timesteps;
    // The RC (1 µs) settles long before the end, onto the switched value.
    close(steps.last().unwrap().node_voltages["m"], 1.0, 1e-6);
    // Before the edge it's still low.
    for s in steps.iter().filter(|s| s.time < 0.33e-3) {
        assert!(s.node_voltages["m"].abs() < 1e-9, "t = {}", s.time);
    }
    // Afterwards the steps grow back to the requested one.
    let tail: Vec<f64> = steps
        .windows(2)
        .rev()
        .take(3)
        .map(|w| w[1].time - w[0].time)
        .collect();
    assert!(tail.iter().all(|h| *h > 0.5e-4), "{tail:?}");
}

/// A B voltage source in a loop with an inductor, reading its own branch
/// current. i(b1) is the current into its + terminal, so the inductor
/// carries −i(b1), and v = 1 + 100·i(b1) is a 1 V source behind 100 Ω:
/// an RL with τ = L/100, switched on at t = 0 with no current.
#[test]
fn a_source_reading_its_own_branch_current_with_an_inductor() {
    let result = solve_transient(
        &circuit(vec![
            b("b1", "a", "0", V, "1 + 100*i(b1)"),
            CircuitElement::Inductor(Inductor {
                id: "l1".into(),
                nodes: ["a".into(), "0".into()],
                inductance: 1e-3,
                saturation: None,
            }),
        ]),
        &TransientConfig::new(50e-6, 0.1e-6),
    );
    let tau = 1e-3 / 100.0;
    for step in result.unwrap().transient.unwrap().timesteps.iter().skip(1) {
        close(step.node_voltages["a"], (-step.time / tau).exp(), 1e-2);
    }
}

/// Overflow mid-transient shortens the step rather than ending the run.
#[test]
fn an_overflow_mid_transient_shortens_the_step() {
    let result = solve_transient(
        &circuit(vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "v1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(sindr::Waveform::Pulse {
                    v1: 0.0,
                    v2: 50.0,
                    delay: 1e-6,
                    rise_time: 1e-9,
                    fall_time: 1e-9,
                    pulse_width: 1.0,
                    period: 2.0,
                }),
            }),
            r("r1", "in", "a", 1e3),
            b("b1", "a", "0", I, &diode_expr("a")),
            c("c1", "a", "0", 1e-12),
        ]),
        &TransientConfig::new(10e-6, 0.1e-6),
    )
    .unwrap();
    let last = result.transient.unwrap().timesteps.last().unwrap().clone();
    close(last.node_voltages["a"], diode_drop(50.0, 1e3, 1.0), 1e-4);
}

#[test]
fn a_long_run_stays_put() {
    let result = solve_transient(
        &circuit(vec![
            v("v1", "in", "0", 5.0),
            r("r1", "in", "a", 1e3),
            b("b1", "a", "0", I, &diode_expr("a")),
            c("c1", "a", "0", 1e-9),
        ]),
        &TransientConfig::new(10e-3, 0.1e-6),
    )
    .unwrap();
    let steps = result.transient.unwrap().timesteps;
    let want = diode_drop(5.0, 1e3, 1.0);
    let last = steps.last().unwrap().node_voltages["a"];
    close(last, want, SETTLED);
}

// ------------------------------------------------------- AC small-signal

/// The AC gain of a tanh stage is its slope at the bias point, at several
/// bias points, and matches a finite difference of the DC sweep.
#[test]
fn ac_gain_is_the_dc_slope_at_every_bias() {
    for bias in [-2.0, -0.5, 0.0, 0.3, 1.5] {
        let ckt = circuit(vec![
            v("v1", "in", "0", bias),
            b("b1", "out", "0", V, "3*tanh(v(in))"),
            r("r1", "out", "0", 1e3),
        ]);
        let ac = solve_ac(&ckt, &AcConfig::new("v1", 1e3, 1e3, 1)).unwrap();
        let gain = ac.points[0].node_voltages["out"].norm();
        let want = 3.0 / bias.cosh().powi(2);
        close(gain, want, 1e-9);
    }
}

/// At a kink the AC gain is the one-sided slope DC used.
#[test]
fn ac_gain_at_a_kink_is_the_one_sided_slope() {
    let ckt = circuit(vec![
        v("v1", "in", "0", 0.0),
        b("b1", "out", "0", V, "abs(v(in))"),
        r("r1", "out", "0", 1e3),
    ]);
    let ac = solve_ac(&ckt, &AcConfig::new("v1", 1e3, 1e3, 1)).unwrap();
    close(ac.points[0].node_voltages["out"].norm(), 1.0, 1e-12);
}

/// A source of time alone has no small-signal part.
#[test]
fn a_time_only_source_adds_nothing_to_ac() {
    let ckt = circuit(vec![
        v("v1", "in", "0", 0.0),
        r("r1", "in", "out", 1e3),
        b("b1", "out", "0", I, "1e-3*sin(1e3*time)"),
        r("r2", "out", "0", 1e3),
    ]);
    let ac = solve_ac(&ckt, &AcConfig::new("v1", 1e3, 1e3, 1)).unwrap();
    close(ac.points[0].node_voltages["out"].norm(), 0.5, 1e-9);
}

/// i() control in AC behaves as a CCVS: out = 2 kΩ · i(v1).
#[test]
fn current_control_in_ac_is_a_transresistance() {
    let ckt = circuit(vec![
        v("v1", "in", "0", 0.0),
        r("r1", "in", "0", 1e3),
        b("b1", "out", "0", V, "2e3*i(v1)"),
        r("r2", "out", "0", 1e3),
    ]);
    let ac = solve_ac(&ckt, &AcConfig::new("v1", 1e3, 1e3, 1)).unwrap();
    // 1 mA flows out of v1's + terminal, so i(v1) = −1 mA.
    close(ac.points[0].node_voltages["out"].norm(), 2.0, 1e-9);
}

// ------------------------------------------------ topology and validation

#[test]
fn a_behavioural_source_across_an_ideal_source_is_singular() {
    let err = solve_op(&circuit(vec![
        v("v1", "a", "0", 1.0),
        b("b1", "a", "0", V, "2"),
    ]))
    .unwrap_err();
    assert!(
        matches!(
            err,
            SimError::SingularMatrix | SimError::InvalidComponent { .. }
        ),
        "{err}"
    );
}

#[test]
fn a_loop_of_behavioural_voltage_sources_is_an_error() {
    let err = solve_op(&circuit(vec![
        b("b1", "a", "0", V, "1"),
        b("b2", "a", "0", V, "2"),
        r("r1", "a", "0", 1e3),
    ]))
    .unwrap_err();
    assert!(
        matches!(
            err,
            SimError::SingularMatrix | SimError::InvalidComponent { .. }
        ),
        "{err}"
    );
}

#[test]
fn i_of_what_isnt_voltage_defined_is_refused() {
    for target in ["r1", "nothing", "b2"] {
        let err = solve_op(&circuit(vec![
            v("v1", "a", "0", 1.0),
            r("r1", "a", "0", 1e3),
            b("b2", "a", "0", I, "1m"),
            b("b1", "out", "0", V, &format!("i({target})")),
            r("r2", "out", "0", 1e3),
        ]))
        .unwrap_err();
        assert!(err.to_string().contains(target), "{target}: {err}");
    }
}

/// A source reading its own branch current at DC: v = 1 − 100·i into 1 kΩ
/// gives i = 1/1100.
#[test]
fn a_source_reading_its_own_current_at_dc() {
    let result = solve_op(&circuit(vec![
        b("b1", "a", "0", V, "1 - 100*i(b1)"),
        r("r1", "a", "0", 1e3),
    ]))
    .unwrap();
    // The branch current flows out of + through the load: i(b1) = −v/1k.
    let want = bisect(|x| 1.0 + 100.0 * x / 1e3 - x, -10.0, 10.0);
    close(result.node_voltages["a"], want, 1e-9);
}

/// A node spelled in another case is the same node; a node named like a
/// function is a node.
#[test]
fn node_names_that_look_like_other_things() {
    let result = solve_op(&circuit(vec![
        v("v1", "exp", "0", 2.0),
        b("b1", "out", "0", V, "exp(v(exp))"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["out"], 2.0_f64.exp(), 1e-12);
}

#[test]
fn ground_inside_an_expression_is_zero() {
    let result = solve_op(&circuit(vec![
        v("v1", "a", "0", 2.0),
        b("b1", "out", "0", V, "v(a, 0) + v(0)"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["out"], 2.0, 1e-12);
}

// ------------------------------------------------------ parser round-trip

#[test]
fn precedence_corners() {
    let e = |s: &str| Expression::parse(s).unwrap().evaluate(&[]);
    assert_eq!(e("-2^2"), -4.0);
    assert_eq!(e("2^3^2"), 512.0);
    assert_eq!(e("2*-3"), -6.0);
    assert_eq!(e("1 - -1"), 2.0);
    assert_eq!(e("!0 + 1"), 2.0);
    assert_eq!(e("0 ? 1 : 2 ? 3 : 4"), 3.0);
    // Comparisons don't chain: `1 < 2 == 1` asks for parentheses.
    assert!(Expression::parse("1 < 2 == 1").is_err());
    assert_eq!(e("(1 < 2) == 1"), 1.0);
}

#[test]
fn suffixes_that_look_alike() {
    let e = |s: &str| Expression::parse(s).unwrap().evaluate(&[]);
    close(e("1m"), 1e-3, 1e-15);
    close(e("1meg"), 1e6, 1e-15);
    close(e("1mil"), 25.4e-6, 1e-15);
    close(e("1M"), 1e-3, 1e-15);
    close(e("5V"), 5.0, 1e-15);
    close(e("10kOhm"), 1e4, 1e-15);
    close(e("2.2uF"), 2.2e-6, 1e-15);
}

#[test]
fn display_round_trips_every_function_bit_for_bit() {
    let texts = [
        "abs(v(a))",
        "sqrt(v(a))",
        "exp(v(a))",
        "ln(v(a))",
        "log10(v(a))",
        "sin(v(a))",
        "cos(v(a))",
        "tan(v(a))",
        "asin(v(a)/4)",
        "acos(v(a)/4)",
        "atan(v(a))",
        "sinh(v(a))",
        "cosh(v(a))",
        "tanh(v(a))",
        "u(v(a))",
        "uramp(v(a))",
        "sgn(v(a))",
        "floor(v(a))",
        "ceil(v(a))",
        "atan2(v(a), v(b))",
        "pow(v(a), v(b))",
        "pwr(v(a), v(b))",
        "min(v(a), v(b))",
        "max(v(a), v(b))",
        "limit(v(a), -1, v(b))",
        "v(a) > v(b) ? -v(a)^2 : !v(b) && v(a) || 1e-12",
    ];
    for text in texts {
        let first = Expression::parse(text).unwrap();
        let again = Expression::parse(&first.to_string()).unwrap();
        assert_eq!(first.to_string(), again.to_string(), "{text}");
        let values = [1.7, 0.6];
        assert_eq!(
            first.evaluate(&values).to_bits(),
            again.evaluate(&values).to_bits(),
            "{text}"
        );
    }
}
