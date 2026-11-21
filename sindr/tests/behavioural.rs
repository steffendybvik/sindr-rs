//! Behavioural (`B`) sources: an expression of node voltages, branch
//! currents and time, as a voltage or a current, against closed forms in
//! every analysis.

use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::expression::Expression;
use sindr::{
    dc_sweep, solve_op, solve_transient, BehaviouralOutput, Block, Circuit, CircuitElement,
    Instance, SimError, TransientConfig,
};

fn v(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    }
}

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    }
}

fn b(id: &str, p: &str, n: &str, output: BehaviouralOutput, expr: &str) -> CircuitElement {
    CircuitElement::Behavioural {
        id: id.into(),
        nodes: [p.into(), n.into()],
        output,
        expr: Expression::parse(expr).unwrap(),
    }
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit::new("0", components)
}

/// `tol` is relative; 1e-8 where a node is set through a resistor, which
/// the solver's 1 pS GMIN shifts by about 1e-9.
fn close(got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1.0),
        "got {got}, want {want}"
    );
}

const V: BehaviouralOutput = BehaviouralOutput::Voltage;
const I: BehaviouralOutput = BehaviouralOutput::Current;

#[test]
fn multiplier_gives_the_product() {
    for (a, bv) in [(1.5, 2.0), (-3.0, 0.25), (0.0, 7.0)] {
        let result = solve_op(&circuit(vec![
            v("va", "a", "0", a),
            v("vb", "b", "0", bv),
            b("b1", "out", "0", V, "v(a)*v(b)"),
            r("rl", "out", "0", 1e3),
        ]))
        .unwrap();
        close(result.node_voltages["out"], a * bv, 1e-12);
        // Its branch current feeds the 1 kΩ load: into `out` from the source.
        close(result.branch_currents["b1"], -a * bv / 1e3, 1e-9);
    }
}

/// A current-output squarer into 1 kΩ, and the same law sensing its own
/// output: `I = k·v(x)²` into R with v(x) = R·I is a quadratic Newton
/// has to solve, not just evaluate.
#[test]
fn squarer_and_a_self_referencing_source_solve_to_the_closed_form() {
    let result = solve_op(&circuit(vec![
        v("vin", "in", "0", 3.0),
        b("b1", "0", "out", I, "1m*v(in)^2"),
        r("rl", "out", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["out"], 9.0, 1e-8);

    // I = 1m·(5 − v(x))² flows into x through 1 kΩ: v = (5 − v)², so
    // v = (11 − √21)/2.
    let result = solve_op(&circuit(vec![
        v("vs", "s", "0", 5.0),
        b("b1", "0", "x", I, "1m*(v(s)-v(x))^2"),
        r("rl", "x", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["x"], (11.0 - 21f64.sqrt()) / 2.0, 1e-9);
}

/// `i(V1)` reads another element's branch current: a current mirror with
/// gain 3, and a CCVS-like `V = 2k·i(V1)`.
#[test]
fn current_reads_a_branch() {
    let result = solve_op(&circuit(vec![
        v("v1", "a", "0", 2.0),
        r("ra", "a", "0", 1e3),
        // i(v1) = −2 mA (it delivers 2 mA).
        b("bm", "0", "m", I, "-3*i(v1)"),
        r("rm", "m", "0", 1e3),
        b("bh", "h", "0", V, "2k*i(v1)"),
        r("rh", "h", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["m"], 6.0, 1e-8);
    close(result.node_voltages["h"], -4.0, 1e-8);
}

/// DC sweep agrees with the closed form at every point, through tanh.
#[test]
fn dc_sweep_through_a_smooth_nonlinearity() {
    let c = circuit(vec![
        v("vin", "in", "0", 0.0),
        b("b1", "out", "0", V, "2*tanh(v(in)/0.5)"),
        r("rl", "out", "0", 1e3),
    ]);
    let sweep = dc_sweep(&c, "vin", -2.0, 2.0, 21).unwrap();
    for point in &sweep.points {
        let x = point.result.node_voltages["in"];
        close(
            point.result.node_voltages["out"],
            2.0 * (x / 0.5).tanh(),
            1e-9,
        );
    }
}

/// AC linearises at the bias point: the small-signal gain of
/// `V = v(in)²` at a bias of 1.5 V is d/dv = 3, and of a current
/// `I = 2m·v(in)³` into 1 kΩ, 2m·3·1.5²·1k.
#[test]
fn ac_gain_is_the_derivative_at_the_bias_point() {
    let c = circuit(vec![
        v("vin", "in", "0", 1.5),
        b("b1", "out", "0", V, "v(in)^2"),
        r("rl", "out", "0", 1e3),
        b("b2", "0", "i", I, "2m*v(in)^3"),
        r("ri", "i", "0", 1e3),
    ]);
    let ac = solve_ac(&c, &AcConfig::new("vin", 1.0, 1e3, 4)).unwrap();
    for point in &ac.points {
        close(point.node_voltages["out"].norm(), 3.0, 1e-9);
        close(
            point.node_voltages["i"].norm(),
            2e-3 * 3.0 * 1.5 * 1.5 * 1e3,
            1e-9,
        );
    }
    // The same slope by DC finite difference.
    let h = 1e-4;
    let at = |x: f64| {
        let mut c = c.clone();
        c.components[0] = v("vin", "in", "0", x);
        solve_op(&c).unwrap().node_voltages["out"]
    };
    close((at(1.5 + h) - at(1.5 - h)) / (2.0 * h), 3.0, 1e-6);
}

/// `time` in a transient: an expression-defined sine matches sin(2πft).
#[test]
fn transient_reads_time() {
    let c = circuit(vec![
        b("b1", "out", "0", V, "2*sin(2*pi*1k*time)"),
        r("rl", "out", "0", 1e3),
    ]);
    let result = solve_transient(&c, &TransientConfig::new(2e-3, 10e-6)).unwrap();
    let steps = result.transient.unwrap().timesteps;
    assert!(steps.len() > 100);
    for step in steps {
        let want = 2.0 * (2.0 * std::f64::consts::PI * 1e3 * step.time).sin();
        close(step.node_voltages["out"], want, 1e-9);
    }
}

/// Inside a block, the expression's nodes and sources are the instance's.
#[test]
fn hierarchy_renames_what_the_expression_reads() {
    let block = Block {
        ports: vec!["in".into(), "out".into()],
        components: vec![
            v("vref", "ref", "0", 2.0),
            r("rref", "ref", "0", 1e3),
            b("bx", "out", "0", V, "v(in)*v(ref) + 1k*i(vref)"),
        ],
        ..Default::default()
    };
    let c = Circuit {
        ground_node: "0".into(),
        components: vec![v("vin", "a", "0", 3.0), r("rl", "y", "0", 1e3)],
        blocks: [("dbl".to_string(), block)].into_iter().collect(),
        instances: vec![Instance::new("x1", "dbl", ["a", "y"])],
        ..Default::default()
    };
    // 3·2 + 1k·(−2 mA) = 4.
    close(solve_op(&c).unwrap().node_voltages["y"], 4.0, 1e-8);
}

#[test]
fn reading_what_isnt_there_is_an_error() {
    let err = solve_op(&circuit(vec![
        v("v1", "a", "0", 1.0),
        b("b1", "out", "0", V, "i(r1)"),
        r("r1", "out", "a", 1e3),
    ]))
    .unwrap_err();
    assert!(
        matches!(&err, SimError::InvalidComponent(m) if m.contains("i(r1)")),
        "{err}"
    );
    // A node nothing else touches floats.
    let err = solve_op(&circuit(vec![
        b("b1", "out", "0", V, "v(nowhere)"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap_err();
    assert!(matches!(err, SimError::FloatingNode(n) if n == "nowhere"));
}

/// sqrt starts at 0 on the first pass, where its slope is infinite; it
/// solves, and only a value that isn't finite is an error.
#[test]
fn an_infinite_slope_is_survived_and_a_non_finite_value_named() {
    let result = solve_op(&circuit(vec![
        v("v1", "a", "0", 4.0),
        b("b1", "out", "0", V, "sqrt(v(a))"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap();
    close(result.node_voltages["out"], 2.0, 1e-12);
}

#[test]
fn an_expression_that_isnt_finite_names_itself() {
    let err = solve_op(&circuit(vec![
        v("v1", "a", "0", -1.0),
        b("b1", "out", "0", V, "sqrt(v(a))"),
        r("r1", "out", "0", 1e3),
    ]))
    .unwrap_err();
    let message = err.to_string();
    assert!(
        message.contains("sqrt(v(a))") && message.contains("v(a) = -1"),
        "{message}"
    );
}

#[cfg(feature = "serde")]
#[test]
fn serde_carries_the_expression_as_text() {
    let element = b("b1", "out", "0", V, "v(a)*v(b)");
    let json = serde_json::to_value(&element).unwrap();
    assert_eq!(
        json,
        serde_json::json!({
            "type": "behavioural",
            "id": "b1",
            "nodes": ["out", "0"],
            "output": "voltage",
            "expr": "v(a)*v(b)",
        })
    );
    let back: CircuitElement = serde_json::from_value(json).unwrap();
    assert_eq!(back, element);
}
