//! Bad inputs must fail with a typed error, never a panic: bad topology,
//! values the models can't take, and malformed decks. Each topology case
//! runs through every analysis that takes a circuit, since each has its own
//! path into validation and the solver.

use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::element::{
    Capacitor, Cccs, Ccvs, CurrentSource, Inductor, OpAmp, Resistor, Transformer, Vccs,
    VoltageSource, ZenerDiode,
};
use sindr::DcSweepAxis;
use sindr::{
    dc_sweep, solve_circuit, solve_transient, validate_circuit, Circuit, CircuitElement, SimError,
    TransientConfig,
};

fn r(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    })
}

fn v(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    })
}

fn i(id: &str, from: &str, to: &str, current: f64) -> CircuitElement {
    CircuitElement::CurrentSource(CurrentSource {
        id: id.into(),
        nodes: [from.into(), to.into()],
        current,
        waveform: None,
    })
}

fn c(id: &str, a: &str, b: &str, capacitance: f64) -> CircuitElement {
    CircuitElement::Capacitor(Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance,
    })
}

fn l(id: &str, a: &str, b: &str, inductance: f64) -> CircuitElement {
    CircuitElement::Inductor(Inductor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        inductance,
        saturation: None,
    })
}

/// Runs `circuit` through DC, transient, AC and a DC sweep of `V1`, and
/// hands each error to `check`. A panic fails the test with the analysis
/// named; an `Ok` fails it too.
fn every_analysis_fails(circuit: &Circuit, check: impl Fn(&str, SimError)) {
    let ac = AcConfig::new("V1", 1.0, 1e3, 3);
    let runs: [(&str, Result<(), SimError>); 4] = [
        ("dc", solve_circuit(circuit).map(|_| ())),
        (
            "transient",
            solve_transient(circuit, &TransientConfig::new(1e-3, 1e-5)).map(|_| ()),
        ),
        ("ac", solve_ac(circuit, &ac).map(|_| ())),
        (
            "sweep",
            dc_sweep(circuit, &DcSweepAxis::new("V1", 0.0, 1.0, 3)).map(|_| ()),
        ),
    ];
    for (analysis, result) in runs {
        match result {
            Err(err) => check(analysis, err),
            Ok(()) => panic!("{analysis}: expected an error, got Ok"),
        }
    }
}

/// The component `id` refused, with a reason that starts with `field`.
fn expect_invalid_component(id: &'static str, field: &'static str) -> impl Fn(&str, SimError) {
    move |analysis, err| match err {
        SimError::InvalidComponent { id: got, reason } => {
            assert_eq!(got, id, "{analysis}");
            assert!(
                reason.starts_with(field),
                "{analysis}: {reason:?} lacks {field:?}"
            )
        }
        other => panic!("{analysis}: expected InvalidComponent, got {other:?}"),
    }
}

// ---------------------------------------------------------------- topology

#[test]
fn no_ground() {
    let circuit = Circuit::new("0", vec![v("V1", "a", "b", 1.0), r("R1", "a", "b", 1e3)]);
    every_analysis_fails(&circuit, |analysis, err| {
        assert!(matches!(err, SimError::NoGround), "{analysis}: {err:?}")
    });
}

#[test]
fn empty_circuit_has_no_ground() {
    let circuit = Circuit::new("0", vec![]);
    assert!(matches!(solve_circuit(&circuit), Err(SimError::NoGround)));
    assert!(matches!(
        validate_circuit(&circuit),
        Err(SimError::NoGround)
    ));
}

#[test]
fn floating_node() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "a", "0", 1.0),
            r("R1", "a", "0", 1e3),
            r("R2", "a", "dangling", 1e3),
        ],
    );
    every_analysis_fails(&circuit, |analysis, err| match err {
        SimError::FloatingNode(node) => assert_eq!(node, "dangling", "{analysis}"),
        other => panic!("{analysis}: expected FloatingNode, got {other:?}"),
    });
}

#[test]
fn island_with_no_path_to_ground() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "a", "0", 1.0),
            r("R1", "a", "0", 1e3),
            r("R2", "x", "y", 1e3),
            r("R3", "x", "y", 1e3),
        ],
    );
    every_analysis_fails(&circuit, |analysis, err| match err {
        SimError::DisconnectedNodes(nodes) => assert_eq!(nodes, ["x", "y"], "{analysis}"),
        other => panic!("{analysis}: expected DisconnectedNodes, got {other:?}"),
    });
}

#[test]
fn voltage_source_loop() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "a", "0", 5.0),
            v("V2", "a", "0", 3.0),
            r("R1", "a", "0", 1e3),
        ],
    );
    every_analysis_fails(&circuit, |analysis, err| {
        assert!(
            matches!(err, SimError::SingularMatrix),
            "{analysis}: {err:?}"
        )
    });
}

/// Two equal sources in parallel are still a loop: the split of current
/// between them is undetermined.
#[test]
fn voltage_source_loop_of_equal_sources() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "a", "0", 5.0),
            v("V2", "a", "0", 5.0),
            r("R1", "a", "0", 1e3),
        ],
    );
    every_analysis_fails(&circuit, |analysis, err| {
        assert!(
            matches!(err, SimError::SingularMatrix),
            "{analysis}: {err:?}"
        )
    });
}

#[test]
fn shorted_ideal_voltage_source() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "a", "a", 5.0),
            r("R1", "a", "0", 1e3),
            r("R2", "a", "0", 1e3),
        ],
    );
    every_analysis_fails(&circuit, |analysis, err| {
        assert!(
            matches!(err, SimError::SingularMatrix),
            "{analysis}: {err:?}"
        )
    });
}

/// Two current sources in series force different currents through the
/// same branch: no solution.
#[test]
fn current_sources_in_series() {
    let circuit = Circuit::new(
        "0",
        vec![
            i("I1", "0", "a", 1e-3),
            i("I2", "a", "0", 2e-3),
            r("R1", "b", "0", 1.0),
            r("R2", "b", "0", 1.0),
        ],
    );
    assert!(matches!(
        solve_circuit(&circuit),
        Err(SimError::SingularMatrix)
    ));
}

/// An inductor is a short at DC, so in parallel with a voltage source it
/// closes a loop for the operating point AC linearises around.
#[test]
fn voltage_source_across_an_inductor_has_no_ac_operating_point() {
    let circuit = Circuit::new("0", vec![v("V1", "a", "0", 5.0), l("L1", "a", "0", 1e-3)]);
    let ac = AcConfig::new("V1", 1.0, 1e3, 3);
    assert!(matches!(
        solve_ac(&circuit, &ac),
        Err(SimError::SingularMatrix)
    ));
}

#[test]
fn duplicate_ids() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "a", "0", 1.0),
            r("R1", "a", "0", 1.0),
            r("R1", "a", "0", 2.0),
        ],
    );
    every_analysis_fails(&circuit, |analysis, err| match err {
        SimError::DuplicateId { id, lowered: false } => assert_eq!(id, "R1", "{analysis}"),
        other => panic!("{analysis}: expected DuplicateId, got {other:?}"),
    });
}

#[test]
fn unknown_source_id() {
    let circuit = Circuit::new("0", vec![v("V1", "a", "0", 1.0), r("R1", "a", "0", 1.0)]);
    assert!(matches!(
        dc_sweep(&circuit, &DcSweepAxis::new("V9", 0.0, 1.0, 3)),
        Err(SimError::UnknownComponent(id)) if id == "V9"
    ));
}

// ------------------------------------------------------------------ values

fn divider_with(element: CircuitElement) -> Circuit {
    Circuit::new(
        "0",
        vec![v("V1", "a", "0", 1.0), r("R1", "a", "b", 1e3), element],
    )
}

#[test]
fn resistance_that_is_zero_negative_or_not_finite() {
    for resistance in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let circuit = divider_with(r("R2", "b", "0", resistance));
        every_analysis_fails(&circuit, |analysis, err| match err {
            SimError::InvalidComponent { id, reason } => {
                assert_eq!(id, "R2", "{analysis} at {resistance}");
                assert!(reason.starts_with("resistance"), "{reason}");
            }
            other => {
                panic!("{analysis} at {resistance}: expected InvalidComponent, got {other:?}")
            }
        });
    }
}

#[test]
fn source_values_that_are_not_finite() {
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let circuit = Circuit::new("0", vec![v("V1", "a", "0", value), r("R1", "a", "0", 1.0)]);
        assert!(
            matches!(solve_circuit(&circuit), Err(SimError::InvalidComponent { ref id, ref reason }) if id == "V1" && reason.starts_with("voltage")),
            "V1 = {value}"
        );
        let circuit = Circuit::new(
            "0",
            vec![
                v("V1", "a", "0", 1.0),
                r("R1", "a", "0", 1.0),
                i("I1", "0", "a", value),
            ],
        );
        every_analysis_fails(&circuit, expect_invalid_component("I1", "current"));
    }
}

#[test]
fn capacitance_that_is_negative_or_not_finite() {
    for capacitance in [-1e-6, f64::NAN, f64::INFINITY] {
        let circuit = divider_with(c("C1", "b", "0", capacitance));
        every_analysis_fails(&circuit, expect_invalid_component("C1", "capacitance"));
    }
}

/// A zero capacitor is an open circuit, as in SPICE.
#[test]
fn zero_capacitance_is_an_open() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "a", "0", 1.0),
            r("R1", "a", "b", 1e3),
            r("R2", "b", "0", 1e3),
            c("C1", "b", "0", 0.0),
        ],
    );
    let result = solve_circuit(&circuit).expect("a zero capacitor solves");
    assert!((result.node_voltages["b"] - 0.5).abs() < 1e-9);
}

#[test]
fn inductance_that_is_zero_negative_or_not_finite() {
    for inductance in [0.0, -1e-3, f64::NAN, f64::INFINITY] {
        let circuit = divider_with(l("L1", "b", "0", inductance));
        every_analysis_fails(&circuit, expect_invalid_component("L1", "inductance"));
    }
}

#[test]
fn controlled_source_gain_that_is_not_finite() {
    let circuit = divider_with(CircuitElement::Vccs(Vccs {
        id: "G1".into(),
        nodes: ["b".into(), "0".into()],
        control_nodes: ["a".into(), "0".into()],
        gm: f64::NAN,
    }));
    every_analysis_fails(&circuit, expect_invalid_component("G1", "gm"));
}

/// A CCCS and a CCVS driving `b`, each reading the current of `control`.
fn current_controlled(control: &str) -> [Circuit; 2] {
    let cccs = CircuitElement::Cccs(Cccs {
        id: "F1".into(),
        nodes: ["0".into(), "b".into()],
        control_source: control.into(),
        alpha: 2.0,
    });
    let ccvs = CircuitElement::Ccvs(Ccvs {
        id: "H1".into(),
        nodes: ["b".into(), "0".into()],
        control_source: control.into(),
        rm: 1e3,
    });
    [cccs, ccvs].map(|source| {
        Circuit::new(
            "0",
            vec![
                v("V1", "a", "0", 1.0),
                r("R1", "a", "0", 1e3),
                r("R2", "b", "0", 1e3),
                source,
            ],
        )
    })
}

/// A control source that isn't there used to solve as zero output.
#[test]
fn current_controlled_source_with_a_missing_control() {
    for (circuit, id) in current_controlled("V9").iter().zip(["F1", "H1"]) {
        every_analysis_fails(circuit, |analysis, err| match err {
            SimError::InvalidComponent { id: got, reason } => {
                assert_eq!(got, id, "{analysis}");
                assert!(reason.starts_with("control_source"), "{reason}");
                assert!(reason.contains("`V9` isn't in the circuit"), "{reason}");
            }
            other => panic!("{analysis}: expected InvalidComponent, got {other:?}"),
        });
    }
}

/// A resistor has no branch row, so there's no current to read from it.
#[test]
fn current_controlled_source_reading_a_resistor() {
    for (circuit, id) in current_controlled("R1").iter().zip(["F1", "H1"]) {
        every_analysis_fails(circuit, |analysis, err| match err {
            SimError::InvalidComponent { id: got, reason } => {
                assert_eq!(got, id, "{analysis}");
                assert!(reason.starts_with("control_source"), "{reason}");
                assert!(reason.contains("`R1` has no branch current"), "{reason}");
            }
            other => panic!("{analysis}: expected InvalidComponent, got {other:?}"),
        });
    }
}

/// With its control a voltage source, each solves. V1's branch current
/// is -1 mA (SPICE's sign: into its + terminal), so F1 pushes -2 mA into
/// `b` through R2 and H1 holds `b` at -1 V.
#[test]
fn current_controlled_source_reading_a_voltage_source() {
    for (circuit, want) in current_controlled("V1").iter().zip([-2.0, -1.0]) {
        validate_circuit(circuit).expect("V1 has a branch current");
        let vb = solve_circuit(circuit).expect("solves").node_voltages["b"];
        assert!((vb - want).abs() < 1e-9, "v(b) = {vb}, want {want}");
    }
}

#[test]
fn transformer_inductance_that_is_zero() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "a", "0", 1.0),
            r("R1", "a", "p", 1.0),
            CircuitElement::Transformer(Transformer {
                id: "T1".into(),
                nodes: ["p".into(), "0".into(), "s".into(), "0".into()],
                l1: 0.0,
                l2: 1.0,
                k: 0.999,
                saturation: None,
            }),
            r("R2", "s", "0", 100.0),
        ],
    );
    every_analysis_fails(&circuit, expect_invalid_component("T1", "l1"));
}

// -------------------------------------------------------------- bad decks

/// A Zener needs a positive breakdown voltage: zero, negative or NaN is
/// refused by name before it is lowered to a diode, in every analysis.
#[test]
fn zener_without_a_breakdown_voltage_is_rejected() {
    for vz in [0.0, -5.1, f64::NAN] {
        let circuit = Circuit::new(
            "0",
            vec![
                v("V1", "in", "0", 12.0),
                r("R1", "in", "k", 1e3),
                CircuitElement::ZenerDiode(ZenerDiode {
                    id: "DZ1".into(),
                    nodes: ["0".into(), "k".into()],
                    vz,
                    temperature: 300.15,
                }),
            ],
        );
        every_analysis_fails(&circuit, |analysis, err| match err {
            SimError::InvalidComponent { id, reason } => {
                assert_eq!(id, "DZ1", "{analysis}");
                assert_eq!(
                    reason,
                    format!("vz {vz} V must be a positive, finite number"),
                    "{analysis}"
                );
            }
            other => panic!("{analysis}: expected InvalidComponent, got {other:?}"),
        });
    }
}

/// A zener's temperature is checked on the zener itself, when the circuit
/// is validated, and on the diode it is lowered to, when it is solved:
/// both name the zener.
#[test]
fn zener_temperature_that_is_not_positive() {
    for temperature in [0.0, -10.0, f64::NAN] {
        let circuit = Circuit::new(
            "0",
            vec![
                v("V1", "in", "0", 12.0),
                r("R1", "in", "k", 1e3),
                CircuitElement::ZenerDiode(ZenerDiode {
                    id: "DZ1".into(),
                    nodes: ["0".into(), "k".into()],
                    vz: 5.1,
                    temperature,
                }),
            ],
        );
        let check = expect_invalid_component("DZ1", "temperature");
        check("validate", validate_circuit(&circuit).unwrap_err());
        every_analysis_fails(&circuit, &check);
    }
}

/// A modelled op-amp's rails must leave room between them.
#[test]
fn op_amp_rails_must_not_cross() {
    for (v_pos, v_neg) in [(5.0, 5.0), (-5.0, 5.0)] {
        let circuit = Circuit::new(
            "0",
            vec![
                v("V1", "in", "0", 1.0),
                CircuitElement::OpAmp(OpAmp {
                    id: "U1".into(),
                    nodes: ["in".into(), "out".into(), "out".into()],
                    v_pos,
                    v_neg,
                    model: Some(sindr::OpAmpModel::default()),
                }),
                r("RL", "out", "0", 1e3),
            ],
        );
        match validate_circuit(&circuit) {
            Err(SimError::InvalidComponent { id, reason }) => {
                assert_eq!(id, "U1");
                assert_eq!(
                    reason,
                    format!("v_pos {v_pos} V must be above v_neg {v_neg} V")
                );
            }
            other => panic!("expected InvalidComponent, got {other:?}"),
        }
    }
}

#[cfg(feature = "spice")]
mod decks {
    use sindr::spice::{parse_str, SpiceParseError};
    use sindr::{solve_circuit, solve_transient, ConfigKind, SimError};

    /// Each deck must be refused by the parser with an error, not a panic.
    #[test]
    fn malformed_decks_are_refused() {
        let decks = [
            ("control bytes", "\u{0}\u{1}\n\u{7f}"),
            ("element with no nodes", "* t\nR1\n.end\n"),
            ("element with one node", "* t\nR1 a\n.end\n"),
            ("resistor with no value", "* t\nR1 a 0\n.end\n"),
            (
                "value overflows f64",
                "* t\nV1 a 0 1e400\nR1 a 0 1k\n.end\n",
            ),
            ("non-numeric value", "* t\nV1 a 0 nan\nR1 a 0 1k\n.end\n"),
            (
                "unclosed .subckt",
                "* t\n.subckt foo a b\nR1 a b 1k\n.end\n",
            ),
            (
                "recursive .subckt",
                "* t\n.subckt foo a b\nX1 a b foo\n.ends\nX1 n 0 foo\nV1 n 0 1\n.end\n",
            ),
            ("unbalanced brace", "* t\n.param x={1+\nR1 a 0 {x}\n.end\n"),
            (
                "division by zero",
                "* t\n.param x={1/0}\nR1 a 0 {x}\nV1 a 0 1\n.end\n",
            ),
            ("undefined model", "* t\nV1 a 0 1\nD1 a 0 nomodel\n.end\n"),
            ("unclosed pulse", "* t\nV1 a 0 pulse(\nR1 a 0 1k\n.end\n"),
            ("odd pwl", "* t\nV1 a 0 pwl(0 0 1)\nR1 a 0 1k\n.end\n"),
            (
                "empty .meas",
                "* t\nV1 a 0 1\nR1 a 0 1k\n.meas tran x\n.end\n",
            ),
        ];
        for (what, deck) in decks {
            let result: Result<_, SpiceParseError> = parse_str(deck);
            assert!(result.is_err(), "{what}: parsed {deck:?}");
        }
    }

    /// A deck with nothing in it parses, then fails to solve for want of
    /// ground.
    #[test]
    fn empty_decks_have_no_ground() {
        for deck in ["", "* title only\n", ".end\n"] {
            let parsed = parse_str(deck).expect("an empty deck parses");
            assert!(
                matches!(solve_circuit(&parsed.circuit), Err(SimError::NoGround)),
                "{deck:?}"
            );
        }
    }

    #[test]
    fn duplicate_element_names() {
        let deck = "* t\nV1 a 0 1\nR1 a 0 1k\nR1 a 0 2k\n.op\n.end\n";
        let parsed = parse_str(deck).expect("parses");
        assert!(matches!(
            solve_circuit(&parsed.circuit),
            Err(SimError::DuplicateId { id, .. }) if id == "r1"
        ));
    }

    #[test]
    fn zero_resistor() {
        let parsed = parse_str("* t\nV1 a 0 1\nR1 a 0 0\n.op\n.end\n").expect("parses");
        assert!(matches!(
            solve_circuit(&parsed.circuit),
            Err(SimError::InvalidComponent { id, .. }) if id == "r1"
        ));
    }

    /// `.tran` settings the transient can't run parse, and are refused when
    /// the transient runs.
    #[test]
    fn tran_settings_that_cannot_run() {
        for (tran, needle) in [
            (".tran -1 1", "tstep"),
            (".tran 0 1m", "tstep"),
            (".tran 1u 0", "tstop"),
        ] {
            let deck = format!("* t\nV1 a 0 1\nR1 a 0 1k\n{tran}\n.end\n");
            let parsed = parse_str(&deck).expect("parses");
            let config = parsed
                .analyses
                .iter()
                .find_map(|a| a.transient_config())
                .expect("has a .tran");
            match solve_transient(&parsed.circuit, &config) {
                Err(SimError::InvalidConfig {
                    config: ConfigKind::Transient,
                    reason,
                }) => {
                    assert!(
                        reason.contains(needle),
                        "{tran}: {reason:?} lacks {needle:?}"
                    )
                }
                other => panic!("{tran}: expected InvalidConfig, got {other:?}"),
            }
        }
    }
}
