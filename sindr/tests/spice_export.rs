//! `to_netlist` round trips: a circuit written as SPICE and read back by
//! `parse_str` solves the same as the original.
//!
//! Only compiled when the `spice` feature is enabled.
#![cfg(feature = "spice")]

use std::collections::HashMap;

use sindr::spice::{parse_str, to_netlist, ExportError};
use sindr::{
    get_examples, solve_circuit, BjtKind, BjtModel, Circuit, CircuitElement, DiodeModel,
    SimulationResult, Waveform,
};

/// Node voltages keyed as the deck names them: lowercase, ground as `0`.
fn as_spice_names(map: &HashMap<String, f64>, ground: &str) -> HashMap<String, f64> {
    map.iter()
        .map(|(k, v)| {
            let k = if k == ground {
                "0".to_string()
            } else {
                k.to_ascii_lowercase()
            };
            (k, *v)
        })
        .collect()
}

/// `a` solved `circuit`, `b` its round trip.
fn assert_same_solution(name: &str, circuit: &Circuit, a: &SimulationResult, b: &SimulationResult) {
    let (a, b) = (
        as_spice_names(&a.node_voltages, &circuit.ground_node),
        as_spice_names(&b.node_voltages, "0"),
    );
    assert_eq!(a.len(), b.len(), "{name}: node sets differ: {a:?} vs {b:?}");
    for (node, va) in &a {
        let vb = b
            .get(node)
            .unwrap_or_else(|| panic!("{name}: node {node} missing after the round trip"));
        assert!(
            (va - vb).abs() <= 1e-9 * va.abs().max(1.0),
            "{name}: v({node}) {va} became {vb}"
        );
    }
}

/// Whether sindr's parser reads every element line of `deck`.
fn parser_reads(deck: &str) -> bool {
    deck.lines().skip(1).all(|l| {
        l.starts_with('.')
            || l.starts_with([
                'R', 'r', 'L', 'l', 'C', 'c', 'V', 'v', 'I', 'i', 'D', 'd', 'Q', 'q',
            ])
    })
}

#[test]
fn examples_round_trip() {
    let mut round_tripped = Vec::new();
    for example in get_examples() {
        let Ok(original) = solve_circuit(&example.circuit) else {
            continue;
        };
        let deck = match to_netlist(&example.circuit, &example.name) {
            Ok(deck) => deck,
            Err(ExportError::Unsupported { .. }) => continue,
            Err(e) => panic!("{}: {e}", example.id),
        };
        if !parser_reads(&deck) {
            continue;
        }
        let parsed = parse_str(&deck).unwrap_or_else(|e| panic!("{}: {e}\n{deck}", example.id));
        let again = solve_circuit(&parsed.circuit)
            .unwrap_or_else(|e| panic!("{}: {e}\n{deck}", example.id));
        assert_same_solution(&example.id, &example.circuit, &original, &again);
        round_tripped.push(example.id);
    }
    println!("round-tripped {}: {round_tripped:?}", round_tripped.len());
    // Most examples are resistive, diode or BJT circuits; a drop here means
    // the exporter started refusing elements it used to write.
    assert!(
        round_tripped.len() >= 10,
        "only {} examples round-tripped: {round_tripped:?}",
        round_tripped.len()
    );
}

fn source_with(waveform: Waveform, dc: f64) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["a".into(), "0".into()],
                voltage: dc,
                waveform: Some(waveform),
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["a".into(), "0".into()],
                resistance: 1e3,
            },
        ],
        ..Default::default()
    }
}

/// The source's value at `t`, waveform plus static value, as the solver sums them.
fn source_value(circuit: &Circuit, t: f64) -> f64 {
    match &circuit.components[0] {
        CircuitElement::VoltageSource {
            voltage, waveform, ..
        } => voltage + waveform.as_ref().map_or(0.0, |w| w.evaluate(t)),
        other => panic!("expected a voltage source, got {other:?}"),
    }
}

#[test]
fn waveforms_round_trip_to_the_same_function_of_time() {
    let waves = [
        Waveform::Sine {
            amplitude: 2.0,
            frequency: 1e3,
            offset: 0.5,
            phase: 0.0,
        },
        Waveform::Sine {
            amplitude: 1.0,
            frequency: 50.0,
            offset: 0.0,
            phase: 1.2,
        },
        Waveform::Pulse {
            v1: 0.0,
            v2: 5.0,
            delay: 1e-4,
            rise_time: 1e-5,
            fall_time: 2e-5,
            pulse_width: 3e-4,
            period: 1e-3,
        },
        Waveform::Pulse {
            v1: -1.0,
            v2: 1.0,
            delay: 0.0,
            rise_time: 1e-5,
            fall_time: 1e-5,
            pulse_width: 1e-4,
            period: 0.0,
        },
        Waveform::Square {
            amplitude: 3.0,
            frequency: 1e3,
            offset: 1.0,
            duty: 0.3,
        },
        Waveform::Triangle {
            amplitude: 2.0,
            frequency: 500.0,
            offset: -0.5,
        },
        Waveform::Pwm {
            amplitude: 5.0,
            frequency: 2e3,
            duty: 0.25,
            offset: 0.2,
        },
        Waveform::Pwl {
            points: vec![(0.0, 0.0), (1e-3, 4.0), (2e-3, 4.0), (3e-3, -1.0)],
        },
    ];
    for wave in waves {
        let circuit = source_with(wave.clone(), 0.7);
        let deck = to_netlist(&circuit, "wave").unwrap();
        let back = parse_str(&deck)
            .unwrap_or_else(|e| panic!("{e}\n{deck}"))
            .circuit;
        // Times off the edges of every wave above: float rounding of the
        // period moves an edge by an ulp, not by these offsets.
        for i in 0..200 {
            let t = 1.37e-5 * i as f64 + 3.1e-7;
            let (want, got) = (source_value(&circuit, t), source_value(&back, t));
            assert!(
                (want - got).abs() <= 1e-9,
                "{wave:?} at {t} s: {want} became {got}\n{deck}"
            );
        }
    }
}

#[test]
fn models_round_trip() {
    let diode = DiodeModel {
        is: 2.52e-9,
        n: 1.752,
        rs: 0.568,
        kf: 1e-16,
        af: 1.2,
        ..DiodeModel::default()
    };
    let bjt = BjtModel {
        is: 6.734e-15,
        br: 0.7371,
        nf: 1.05,
        nr: 1.1,
        vaf: 74.03,
        kf: 3e-15,
        af: 1.1,
    };
    let circuit = Circuit {
        ground_node: "gnd".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "VCC".into(),
                nodes: ["vcc".into(), "gnd".into()],
                voltage: 9.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "RB".into(),
                nodes: ["vcc".into(), "b".into()],
                resistance: 1e6,
            },
            CircuitElement::Resistor {
                id: "RC".into(),
                nodes: ["vcc".into(), "c".into()],
                resistance: 2.2e3,
            },
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "gnd".into()],
                kind: BjtKind::Npn,
                bf: 416.4,
                temperature: 300.15,
                parasitic_caps: None,
                model: Some(bjt),
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["c".into(), "gnd".into()],
                temperature: 300.15,
                model: Some(diode),
            },
            CircuitElement::Diode {
                id: "D2".into(),
                nodes: ["c".into(), "gnd".into()],
                temperature: 300.15,
                model: Some(diode),
            },
        ],
        ..Default::default()
    };
    let deck = to_netlist(&circuit, "models").unwrap();
    // One card per distinct model.
    assert_eq!(deck.matches(".model").count(), 2, "{deck}");
    let back = parse_str(&deck).unwrap().circuit;
    match &back.components[3] {
        CircuitElement::Bjt { bf, model, .. } => {
            assert_eq!(*bf, 416.4);
            assert_eq!(*model, Some(bjt));
        }
        other => panic!("expected Bjt, got {other:?}"),
    }
    assert!(matches!(
        &back.components[4],
        CircuitElement::Diode { model: Some(m), .. } if *m == diode
    ));
    assert_same_solution(
        "models",
        &circuit,
        &solve_circuit(&circuit).unwrap(),
        &solve_circuit(&back).unwrap(),
    );
}

#[test]
fn diode_charge_and_breakdown_round_trip() {
    let model = DiodeModel {
        is: 31.7e-6,
        n: 1.373,
        rs: 0.051,
        cjo: 110e-12,
        vj: 0.3,
        m: 0.35,
        fc: 0.4,
        tt: 2e-9,
        bv: Some(40.0),
        ibv: 1e-3,
        ..DiodeModel::default()
    };
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["a".into(), "0".into()],
                voltage: 0.2,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["a".into(), "b".into()],
                resistance: 10.0,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["b".into(), "0".into()],
                temperature: 300.15,
                model: Some(model),
            },
        ],
        ..Default::default()
    };
    let deck = to_netlist(&circuit, "schottky").unwrap();
    assert!(
        deck.contains("CJO=1.1e-10 VJ=0.3 M=0.35 FC=0.4 TT=2e-9 BV=40"),
        "{deck}"
    );
    let back = parse_str(&deck).unwrap().circuit;
    assert!(matches!(
        &back.components[2],
        CircuitElement::Diode { model: Some(m), .. } if *m == model
    ));

    // A plain card stays as it was.
    let plain = DiodeModel {
        rs: 0.5,
        ..DiodeModel::default()
    };
    let mut circuit = circuit;
    if let CircuitElement::Diode { model, .. } = &mut circuit.components[2] {
        *model = Some(plain);
    }
    let deck = to_netlist(&circuit, "plain").unwrap();
    assert!(
        deck.contains(".model dmod1 D(IS=1e-14 N=1 RS=0.5)\n"),
        "{deck}"
    );
}

#[test]
fn names_get_their_spice_letter_and_ground_becomes_0() {
    let circuit = Circuit {
        ground_node: "gnd".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "supply".into(),
                nodes: ["in".into(), "gnd".into()],
                voltage: 1.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "r_load".into(),
                nodes: ["in".into(), "gnd".into()],
                resistance: 50.0,
            },
            CircuitElement::Cccs {
                id: "mirror".into(),
                nodes: ["in".into(), "gnd".into()],
                control_source: "supply".into(),
                alpha: 2.0,
            },
        ],
        ..Default::default()
    };
    let deck = to_netlist(&circuit, "names").unwrap();
    assert_eq!(
        deck,
        "names\nVsupply in 0 DC 1\nr_load in 0 50\nFmirror in 0 Vsupply 2\n.end\n"
    );
}

fn one_resistor(id: &str, a: &str, b: &str) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![CircuitElement::Resistor {
            id: id.into(),
            nodes: [a.into(), b.into()],
            resistance: 1.0,
        }],
        ..Default::default()
    }
}

#[test]
fn names_spice_cannot_hold_are_rejected() {
    for (id, a) in [
        ("R 1", "a"),
        ("R1", "a b"),
        ("R1", "f(x)"),
        ("R1", "GND"),
        ("R1", ".a"),
        ("", "a"),
    ] {
        let err = to_netlist(&one_resistor(id, a, "0"), "t").unwrap_err();
        assert!(
            matches!(err, ExportError::BadName { .. }),
            "{id:?}/{a:?}: {err}"
        );
    }

    let mut folded = one_resistor("R1", "Out", "0");
    folded.components.push(CircuitElement::Resistor {
        id: "R2".into(),
        nodes: ["out".into(), "0".into()],
        resistance: 1.0,
    });
    let err = to_netlist(&folded, "t").unwrap_err();
    assert!(err.to_string().contains("Out"), "{err}");

    // `1` becomes `R1`, which another resistor already is.
    let mut clash = one_resistor("1", "a", "0");
    clash.components.push(CircuitElement::Resistor {
        id: "r1".into(),
        nodes: ["a".into(), "0".into()],
        resistance: 1.0,
    });
    assert!(matches!(
        to_netlist(&clash, "t"),
        Err(ExportError::BadName { .. })
    ));
}

#[test]
fn what_spice_cannot_express_is_reported() {
    let mut hot = one_resistor("R1", "a", "0");
    hot.components.push(CircuitElement::Diode {
        id: "D1".into(),
        nodes: ["a".into(), "0".into()],
        temperature: 350.0,
        model: None,
    });
    let err = to_netlist(&hot, "t").unwrap_err();
    assert!(
        matches!(&err, ExportError::Unsupported { id, .. } if id == "D1"),
        "{err}"
    );

    let mut nan = one_resistor("R1", "a", "0");
    if let CircuitElement::Resistor { resistance, .. } = &mut nan.components[0] {
        *resistance = f64::NAN;
    }
    assert!(matches!(
        to_netlist(&nan, "t"),
        Err(ExportError::NonFinite {
            field: "resistance",
            ..
        })
    ));

    let mut orphan = one_resistor("R1", "a", "0");
    orphan.components.push(CircuitElement::Ccvs {
        id: "H1".into(),
        nodes: ["a".into(), "0".into()],
        control_source: "R1".into(),
        rm: 1.0,
    });
    assert!(matches!(
        to_netlist(&orphan, "t"),
        Err(ExportError::BadName { .. })
    ));
}

const NESTED: &str = "* nested\n\
                      V1 in 0 DC 1\n\
                      R9 out 0 1000\n\
                      X1 in out stage\n\
                      .subckt inner a b\n\
                      R1 a mid 1000\n\
                      R2 mid b 1000\n\
                      .ends inner\n\
                      .subckt stage p q\n\
                      R3 n q 1000\n\
                      X2 p n inner\n\
                      .ends stage\n\
                      .end\n";

#[test]
fn blocks_are_written_as_subckts() {
    let parsed = parse_str(NESTED).unwrap().circuit;
    let deck = to_netlist(&parsed, "nested").unwrap();
    // The parser reads names as lowercase, so they come back that way.
    let expected = NESTED.replacen("* nested", "nested", 1);
    assert_eq!(deck.to_ascii_lowercase(), expected.to_ascii_lowercase());
}

#[test]
fn hierarchy_round_trips() {
    let src = "* filter\n\
               Vin in 0 DC 5\n\
               X1 in mid rcfilter R=1k\n\
               X2 mid out rcfilter R=2k\n\
               X3 out end rcfilter R=1k\n\
               .subckt rcfilter a b R=10k\n\
               R1 a b {R}\n\
               C1 b 0 1u\n\
               .ends\n\
               .end\n";
    let parsed = parse_str(src).unwrap().circuit;
    assert_eq!(parsed.blocks.len(), 2);
    let deck = to_netlist(&parsed, "filter").unwrap();
    let back = parse_str(&deck).unwrap().circuit;
    assert_eq!(back, parsed, "deck:\n{deck}");
}

#[test]
fn a_broken_hierarchy_is_a_design_error() {
    let mut circuit = parse_str(NESTED).unwrap().circuit;
    circuit.blocks.remove("inner");
    assert!(matches!(
        to_netlist(&circuit, "t"),
        Err(ExportError::Design(_))
    ));
}

#[test]
fn a_ground_port_is_rejected() {
    let mut circuit = parse_str(NESTED).unwrap().circuit;
    circuit.blocks.get_mut("inner").unwrap().ports[1] = "0".into();
    assert!(matches!(
        to_netlist(&circuit, "t"),
        Err(ExportError::BadName { what: "node", .. })
    ));
}

#[test]
fn globals_round_trip() {
    let resistor = |id: &str, a: &str, b: &str, ohms: f64| CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    };
    // `pullup` reaches the `vcc` rail by name, not through a port.
    let pullup = sindr::Block {
        ports: vec!["out".into()],
        components: vec![resistor("r1", "vcc", "out", 1000.0)],
        ..Default::default()
    };
    let circuit = Circuit {
        ground_node: "0".into(),
        globals: vec!["vcc".into()],
        components: vec![
            CircuitElement::VoltageSource {
                id: "v1".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            resistor("r2", "a", "0", 1000.0),
            resistor("r3", "b", "0", 3000.0),
        ],
        instances: vec![
            sindr::Instance::new("x1", "pullup", ["a"]),
            sindr::Instance::new("x2", "pullup", ["b"]),
        ],
        blocks: [("pullup".to_string(), pullup)].into(),
        ..Default::default()
    };

    let deck = to_netlist(&circuit, "globals").unwrap();
    let back = parse_str(&deck).unwrap().circuit;
    assert_eq!(back, circuit, "deck:\n{deck}");

    let nets = |c: &Circuit| {
        let mut nets: Vec<String> = c
            .flatten()
            .unwrap()
            .components
            .iter()
            .flat_map(|e| e.nodes().to_vec())
            .collect();
        nets.sort();
        nets.dedup();
        nets
    };
    assert_eq!(nets(&back), ["0", "a", "b", "vcc"]);
    assert_eq!(nets(&back), nets(&circuit));

    let (a, b) = (
        solve_circuit(&circuit).unwrap(),
        solve_circuit(&back).unwrap(),
    );
    assert_same_solution("globals", &circuit, &a, &b);
    assert!((b.node_voltages["a"] - 2.5).abs() < 1e-9);
    assert!((b.node_voltages["b"] - 3.75).abs() < 1e-9);
}
