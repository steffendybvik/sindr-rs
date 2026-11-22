//! SPICE's `B` lines and the `VALUE` form of `E`/`G`: what the parser
//! reads, and that `to_netlist` writes them back.
//!
//! Only compiled when the `spice` feature is enabled.
#![cfg(feature = "spice")]

use sindr::spice::{parse_str, to_netlist};
use sindr::{solve_op, BehaviouralOutput, CircuitElement};

fn op(deck: &str) -> sindr::SimulationResult {
    let netlist = parse_str(deck).unwrap_or_else(|e| panic!("{e}"));
    solve_op(&netlist.circuit).unwrap_or_else(|e| panic!("{e}"))
}

fn close(got: f64, want: f64) {
    assert!(
        (got - want).abs() <= 1e-8 * want.abs().max(1.0),
        "got {got}, want {want}"
    );
}

#[test]
fn b_lines_set_a_voltage_or_a_current() {
    let result = op("t\n\
        Va a 0 3\n\
        Vb b 0 2\n\
        B1 out 0 V={v(a)*v(b)}\n\
        R1 out 0 1k\n\
        B2 0 i I = 1m * v(a,b)\n\
        R2 i 0 1k\n\
        .end\n");
    close(result.node_voltages["out"], 6.0);
    close(result.node_voltages["i"], 1.0);
}

#[test]
fn value_forms_of_e_and_g_are_behavioural() {
    let netlist = parse_str(
        "t\n\
        Va a 0 3\n\
        E1 e 0 VALUE={2*v(a)}\n\
        Re e 0 1k\n\
        G1 0 g VALUE={v(a)^2/1k}\n\
        Rg g 0 1k\n\
        .end\n",
    )
    .unwrap();
    let outputs: Vec<_> = netlist
        .circuit
        .components
        .iter()
        .filter_map(|c| match c {
            CircuitElement::Behavioural { id, output, .. } => Some((id.as_str(), *output)),
            _ => None,
        })
        .collect();
    assert_eq!(
        outputs,
        [
            ("e1", BehaviouralOutput::Voltage),
            ("g1", BehaviouralOutput::Current)
        ]
    );
    let result = solve_op(&netlist.circuit).unwrap();
    close(result.node_voltages["e"], 6.0);
    close(result.node_voltages["g"], 9.0);
}

#[test]
fn params_resolve_inside_the_expression() {
    let result = op("t\n\
        .param gain=4 off=0.5\n\
        Va a 0 1.5\n\
        B1 out 0 V={gain*v(a) + {off}}\n\
        R1 out 0 1k\n\
        .end\n");
    close(result.node_voltages["out"], 6.5);
}

/// A subcircuit's B line reads its own nodes and `V` lines, per instance.
#[test]
fn subckt_expressions_read_the_instances_own_names() {
    let result = op("t\n\
        .subckt sq in out\n\
        Vs s 0 0.5\n\
        Rs s 0 1k\n\
        B1 out 0 V={v(in)^2 + 1k*i(vs)}\n\
        .ends\n\
        Va a 0 3\n\
        X1 a y sq\n\
        Ry y 0 1k\n\
        X2 y z sq\n\
        Rz z 0 1k\n\
        .end\n");
    // i(vs) = −0.5 mA, so each stage is v² − 0.5.
    close(result.node_voltages["y"], 8.5);
    close(result.node_voltages["z"], 8.5 * 8.5 - 0.5);
}

#[test]
fn malformed_b_lines_are_syntax_errors() {
    for line in ["B1 a 0 v(a)", "B1 a 0 X=1", "B1 a 0 V=", "B1 a 0 V={1+}"] {
        let deck = format!("t\nV1 a 0 1\nR1 a 0 1k\n{line}\n.end\n");
        assert!(parse_str(&deck).is_err(), "{line}");
    }
}

#[test]
fn exported_b_lines_read_back_and_solve_the_same() {
    let deck = "t\n\
        V1 a 0 2\n\
        R1 a 0 1k\n\
        B1 out 0 V={v(a)^2 - 1k*i(v1)}\n\
        R2 out 0 1k\n\
        B2 0 c I={tanh(v(out,a))*1m}\n\
        R3 c 0 1k\n\
        .end\n";
    let netlist = parse_str(deck).unwrap();
    let written = to_netlist(&netlist.circuit, "t").unwrap();
    assert!(
        written.contains("b1 out 0 V={v(a)^2-1000*i(v1)}"),
        "{written}"
    );
    assert!(
        written.contains("b2 0 c I={tanh(v(out,a))*0.001}"),
        "{written}"
    );
    let back = parse_str(&written).unwrap();
    let (a, b) = (
        solve_op(&netlist.circuit).unwrap(),
        solve_op(&back.circuit).unwrap(),
    );
    for node in ["out", "c"] {
        close(b.node_voltages[node], a.node_voltages[node]);
    }
}

/// Export writes the deck's names into the expression: ground as `0`,
/// and an element given its SPICE letter.
#[test]
fn export_renames_what_the_expression_reads() {
    use sindr::expression::Expression;
    let circuit = sindr::Circuit::new(
        "gnd",
        vec![
            CircuitElement::VoltageSource {
                id: "supply".into(),
                nodes: ["a".into(), "gnd".into()],
                voltage: 1.0,
                waveform: None,
            },
            CircuitElement::Behavioural {
                id: "mul".into(),
                nodes: ["out".into(), "gnd".into()],
                output: BehaviouralOutput::Voltage,
                expr: Expression::parse("v(a,gnd)*i(supply)").unwrap(),
            },
            CircuitElement::Resistor {
                id: "load".into(),
                nodes: ["out".into(), "gnd".into()],
                resistance: 1e3,
            },
        ],
    );
    let written = to_netlist(&circuit, "t").unwrap();
    assert!(
        written.contains("Bmul out 0 V={v(a,0)*i(Vsupply)}"),
        "{written}"
    );
}

#[test]
fn exporting_an_expression_that_reads_a_missing_source_is_an_error() {
    use sindr::expression::Expression;
    use sindr::spice::ExportError;
    let circuit = sindr::Circuit::new(
        "0",
        vec![
            CircuitElement::Behavioural {
                id: "b1".into(),
                nodes: ["out".into(), "0".into()],
                output: BehaviouralOutput::Current,
                expr: Expression::parse("2*i(ghost)").unwrap(),
            },
            CircuitElement::Resistor {
                id: "r1".into(),
                nodes: ["out".into(), "0".into()],
                resistance: 1e3,
            },
        ],
    );
    let err = to_netlist(&circuit, "t").unwrap_err();
    assert!(
        matches!(&err, ExportError::BadName { reason, .. } if reason.contains("i(ghost)")),
        "{err}"
    );
}

/// SPICE2's shorthand: a one-dimensional POLY with a single coefficient
/// is its linear term (ngspice refuses it; SPICE2 reads it).
#[test]
fn single_coefficient_poly_is_linear() {
    let result = op("t\nV1 a 0 2\nR1 a 0 1k\nE1 out 0 POLY(1) a 0 3\nR2 out 0 1k\n.end\n");
    close(result.node_voltages["out"], 6.0);
}

#[test]
fn poly_reads_params_and_separators() {
    let result = op("t\n\
        .param k=0.5\n\
        V1 a 0 2\n\
        R1 a 0 1k\n\
        E1 out 0 POLY(1) (a, 0) 1, {k}, 0.25\n\
        R2 out 0 1k\n\
        .end\n");
    close(result.node_voltages["out"], 1.0 + 0.5 * 2.0 + 0.25 * 4.0);
}

#[test]
fn malformed_poly_lines_are_syntax_errors() {
    for line in [
        "E1 out 0 POLY(0) a 0 1",
        "E1 out 0 POLY(x) a 0 1",
        "E1 out 0 POLY(2) a 0 1",
        "E1 out 0 POLY(1) a 0",
        "F1 out 0 POLY(1)",
    ] {
        let deck = format!("t\nV1 a 0 1\nR1 a 0 1k\n{line}\nR2 out 0 1k\n.end\n");
        assert!(parse_str(&deck).is_err(), "{line}");
    }
}
