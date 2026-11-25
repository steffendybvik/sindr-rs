//! SPICE's linear controlled sources (`E`, `G`, `F`, `H`) and mutual
//! inductance (`K`): what the parser reads, and that `to_netlist`'s
//! output for them parses back to the same circuit.
//!
//! Only compiled when the `spice` feature is enabled.
#![cfg(feature = "spice")]

use sindr::element::{Cccs, Ccvs, Resistor, Transformer, Vccs, Vcvs, VoltageSource};
use sindr::spice::{parse_str, parse_str_with_options, to_netlist, ParseOptions};
use sindr::{solve_circuit, Circuit, CircuitElement};

fn resistor(id: &str, a: &str, b: &str, r: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: r,
    })
}

fn find<'a>(circuit: &'a Circuit, id: &str) -> &'a CircuitElement {
    circuit
        .components
        .iter()
        .find(|c| c.id() == id)
        .unwrap_or_else(|| panic!("no `{id}` in {:#?}", circuit.components))
}

/// Every controlled source and a transformer, in one circuit.
fn controlled_circuit() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "v1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 2.0,
                waveform: None,
            }),
            resistor("r1", "in", "a", 1e3),
            resistor("r2", "a", "0", 2e3),
            CircuitElement::Vcvs(Vcvs {
                id: "e1".into(),
                nodes: ["e".into(), "0".into()],
                control_nodes: ["a".into(), "0".into()],
                gain: 3.0,
            }),
            resistor("re", "e", "0", 1e3),
            CircuitElement::Vccs(Vccs {
                id: "g1".into(),
                nodes: ["0".into(), "g".into()],
                control_nodes: ["a".into(), "0".into()],
                gm: 1e-3,
            }),
            resistor("rg", "g", "0", 1e3),
            CircuitElement::Ccvs(Ccvs {
                id: "h1".into(),
                nodes: ["h".into(), "0".into()],
                control_source: "v1".into(),
                rm: 100.0,
            }),
            resistor("rh", "h", "0", 1e3),
            CircuitElement::Cccs(Cccs {
                id: "f1".into(),
                nodes: ["0".into(), "f".into()],
                control_source: "v1".into(),
                alpha: 2.0,
            }),
            resistor("rf", "f", "0", 1e3),
            CircuitElement::Transformer(Transformer {
                id: "t1".into(),
                nodes: ["e".into(), "0".into(), "s".into(), "0".into()],
                l1: 1e-3,
                l2: 4e-3,
                k: 0.99,
                saturation: None,
            }),
            resistor("rs", "s", "0", 1e3),
        ],
        ..Default::default()
    }
}

#[test]
fn exported_controlled_sources_and_transformer_parse_back() {
    let original = controlled_circuit();
    let deck = to_netlist(&original, "controlled").unwrap();
    let back = parse_str(&deck)
        .unwrap_or_else(|e| panic!("{e}\n{deck}"))
        .circuit;

    // Element for element: the same kinds, nodes and values.
    for element in &original.components {
        let id = element.id();
        let parsed = match element {
            // The transformer comes back under its `K` line's name.
            CircuitElement::Transformer(Transformer { .. }) => find(&back, "kt1"),
            _ => find(&back, id),
        };
        match (element, parsed) {
            (
                CircuitElement::Transformer(Transformer {
                    nodes, l1, l2, k, ..
                }),
                CircuitElement::Transformer(Transformer {
                    nodes: n,
                    l1: a,
                    l2: b,
                    k: c,
                    ..
                }),
            ) => assert_eq!((nodes, l1, l2, k), (n, a, b, c)),
            (a, b) => {
                let mut b = b.clone();
                *b.id_mut() = id.to_string();
                assert_eq!(a, &b);
            }
        }
    }
    assert_eq!(back.components.len(), original.components.len());

    let (a, b) = (
        solve_circuit(&original).unwrap(),
        solve_circuit(&back).unwrap(),
    );
    for (node, va) in &a.node_voltages {
        let vb = b.node_voltages[node];
        assert!(
            (va - vb).abs() <= 1e-9 * va.abs().max(1.0),
            "v({node}) {va} became {vb}"
        );
    }
}

const HAND_WRITTEN: &str = "\
hand-written controlled sources
V1 in 0 DC 1
R1 in a 1k
R2 a 0 1k
Eamp out 0 a 0 {2*5}
Rout out 0 1k
Gtc 0 g a 0 2m
Rg g 0 1k
Vsense in s 0
Rsense s 0 500
Fmir 0 f Vsense 3
Rf f 0 1k
Htr h 0 Vsense 1k
Rh h 0 1k
Lp p 0 10m
Ls x 0 40m
Kxfmr Lp Ls 0.95
Rp in p 50
Rx x 0 1k
.op
.end
";

#[test]
fn hand_written_deck_reads_each_card_in_strict_mode() {
    let parsed = parse_str_with_options(HAND_WRITTEN, ParseOptions::default().with_strict(true))
        .unwrap_or_else(|e| panic!("{e:?}"));
    let c = &parsed.circuit;
    assert!(matches!(
        find(c, "eamp"),
        CircuitElement::Vcvs(Vcvs { nodes, control_nodes, gain, .. })
            if nodes == &["out".to_string(), "0".to_string()]
                && control_nodes == &["a".to_string(), "0".to_string()]
                && *gain == 10.0
    ));
    assert!(matches!(find(c, "gtc"), CircuitElement::Vccs(Vccs { gm, .. }) if *gm == 2e-3));
    assert!(matches!(
        find(c, "fmir"),
        CircuitElement::Cccs(Cccs { control_source, alpha, .. })
            if control_source == "vsense" && *alpha == 3.0
    ));
    assert!(matches!(
        find(c, "htr"),
        CircuitElement::Ccvs(Ccvs { control_source, rm, .. })
            if control_source == "vsense" && *rm == 1e3
    ));
    assert!(matches!(
        find(c, "kxfmr"),
        CircuitElement::Transformer(Transformer { nodes, l1, l2, k, .. })
            if nodes == &["p".to_string(), "0".into(), "x".into(), "0".into()]
                && *l1 == 10e-3 && *l2 == 40e-3 && *k == 0.95
    ));
    // The coupled inductors became the transformer.
    assert!(!c.components.iter().any(|e| ["lp", "ls"].contains(&e.id())));

    let r = solve_circuit(c).unwrap();
    // a = 0.5 V, so E drives 5 V and G pushes 1 mA into g.
    assert!((r.node_voltages["out"] - 5.0).abs() < 1e-9);
    assert!((r.node_voltages["g"] - 1.0).abs() < 1e-9);
    // 2 mA flows through Vsense from + to −: F pushes 6 mA into f, H drives 2 V.
    assert!((r.node_voltages["f"] - 6.0).abs() < 1e-9);
    assert!((r.node_voltages["h"] - 2.0).abs() < 1e-9);
}

#[test]
fn coupling_and_control_resolve_inside_a_subckt() {
    let deck = "\
subckt scope
.subckt stage in out
Vs in m 0
Rm m 0 1k
Fo 0 out Vs 2
La in p 1m
Rp p 0 1k
Lb q 0 4m
Rq q 0 1k
Kab La Lb 0.9
.ends
V1 a 0 DC 1
X1 a b stage
X2 a c stage
Rb b 0 1k
Rc c 0 1k
.end
";
    let parsed = parse_str(deck).unwrap_or_else(|e| panic!("{e:?}"));
    let flat = parsed.circuit.flattened().unwrap().into_owned();
    for x in ["x1", "x2"] {
        assert!(matches!(
            find(&flat, &format!("{x}.fo")),
            CircuitElement::Cccs(Cccs { control_source, .. }) if *control_source == format!("{x}.vs")
        ));
        assert!(matches!(
            find(&flat, &format!("{x}.kab")),
            CircuitElement::Transformer(Transformer { .. })
        ));
    }
    let r = solve_circuit(&parsed.circuit).unwrap();
    // 1 mA through each Vs, doubled into Rb. (The transformer sends the
    // solve down the transient path, but `out` depends on no winding.)
    for node in ["b", "c"] {
        let v = r.node_voltages[node];
        assert!((v - 2.0).abs() < 1e-9, "{node} = {v}");
    }
}

fn error_of(deck: &str) -> String {
    match parse_str(deck) {
        Ok(_) => panic!("parsed:\n{deck}"),
        Err(e) => format!("{e:?}"),
    }
}

#[test]
fn a_coupling_needs_two_inductors_of_its_own() {
    let missing = "t\nL1 a 0 1m\nK1 L1 L2 0.9\nR1 a 0 1\n.end\n";
    assert!(error_of(missing).contains("`l2`"), "{}", error_of(missing));

    let itself = "t\nL1 a 0 1m\nK1 L1 L1 0.9\nR1 a 0 1\n.end\n";
    assert!(error_of(itself).contains("to itself"));

    let twice = "t\nL1 a 0 1m\nL2 b 0 1m\nL3 c 0 1m\nK1 L1 L2 0.9\nK2 L2 L3 0.9\n.end\n";
    assert!(error_of(twice).contains("already couples"));
}

#[test]
fn current_control_must_be_a_voltage_source() {
    let deck = "t\nV1 a 0 1\nR1 a 0 1k\nF1 0 b R1 2\nR2 b 0 1k\n.end\n";
    let e = error_of(deck);
    assert!(e.contains("isn't a `V` line"), "{e}");
}

#[test]
fn nonlinear_forms_are_unsupported_not_malformed() {
    let deck = "t\nV1 a 0 1\nR1 a 0 1k\nG1 0 b LAPLACE {v(a)} = {1/s}\nR2 b 0 1k\n.end\n";
    assert!(error_of(deck).contains("UnsupportedDevice"));
    let table = "t\nV1 a 0 1\nR1 a 0 1k\nE1 b 0 TABLE {V(a)} = (0,0) (1,1)\nR2 b 0 1k\n.end\n";
    assert!(error_of(table).contains("UnsupportedDevice"));

    // Lenient mode drops the line with a warning, as for other unsupported devices.
    let parsed = parse_str_with_options(deck, ParseOptions::default().with_strict(false)).unwrap();
    assert_eq!(parsed.warnings.len(), 1);
    assert!(parsed.circuit.components.iter().all(|c| c.id() != "g1"));
}
