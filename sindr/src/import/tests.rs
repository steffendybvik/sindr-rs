use super::*;
use crate::ac_analysis::AcConfig;
use crate::element::{
    Battery, Bjt, Capacitor, Comparator, Crystal, CurrentSource, Diode, FerriteBead, Fuse, Igbt,
    Inductor, Jfet, Led, Mosfet, OpAmp, Photodiode, Photoresistor, Potentiometer, Pushbutton,
    Relay, Resistor, SchottkyDiode, Switch, Thermistor, Transformer, Varactor, VoltageRegulator,
    VoltageSource, ZenerDiode,
};
use crate::waveform::Waveform;
use crate::TransientConfig;

fn op(netlist: &Netlist, bench: &Bench) -> crate::SimulationResult {
    match simulate(netlist, bench).expect("simulates").result {
        Simulation::OperatingPoint(r) => r,
        other => panic!("not an operating point: {other:?}"),
    }
}

fn err(netlist: &Netlist, bench: &Bench) -> ImportError {
    netlist.to_circuit(bench).expect_err("should not build")
}

/// R1 from `in` to `out`, R2 from `out` to `gnd`.
fn divider(r1: &str, r2: &str) -> Netlist {
    Netlist {
        components: vec![
            Component::new("r1", ComponentKind::Resistor).with_value(r1),
            Component::new("r2", ComponentKind::Resistor).with_value(r2),
        ],
        nets: vec![
            Net::new("in", [("r1", "1")]),
            Net::new("out", [("r1", "2"), ("r2", "1")]),
            Net::new("gnd", [("r2", "2")]),
        ],
    }
}

#[test]
fn divider_with_text_values() {
    let bench = Bench::new("gnd").with_source(BenchSource::voltage("v", "in", 10.0));
    let r = op(&divider("4k7", "10 kΩ"), &bench);
    let want = 10.0 * 10e3 / 14.7e3;
    assert!((r.node_voltages["out"] - want).abs() < 1e-9);
}

#[test]
fn results_use_the_callers_ids() {
    let bench = Bench::new("gnd").with_source(BenchSource::voltage("v", "in", 5.0));
    let imported = simulate(&divider("1k", "1k"), &bench).unwrap();
    assert_eq!(imported.circuit.node("out"), Some("out"));
    let Simulation::OperatingPoint(r) = imported.result else {
        panic!()
    };
    let r1 = r.component_results.iter().find(|c| c.id == "r1").unwrap();
    assert!((r1.current_through - 2.5e-3).abs() < 1e-12);
}

#[test]
fn terminal_names_ignore_case() {
    let mut n = divider("1k", "1k");
    n.nets[1].pins[0].terminal = "B".into();
    n.nets[0].pins[0].terminal = "A".into();
    let bench = Bench::new("gnd").with_source(BenchSource::voltage("v", "in", 2.0));
    assert!((op(&n, &bench).node_voltages["out"] - 1.0).abs() < 1e-9);
}

#[test]
fn supply_through_a_connector_pin_and_a_display_name() {
    // A connector brings 9 V in; the bench attaches to its pin, and
    // grounds the net by its display name.
    let netlist = Netlist {
        components: vec![
            Component::new("j1", ComponentKind::Connector),
            Component::new("r1", ComponentKind::Resistor).with_value(1e3),
            Component::new("r2", ComponentKind::Resistor).with_value(2e3),
        ],
        nets: vec![
            Net::new("n-a", [("j1", "1"), ("r1", "1")]),
            Net::new("n-b", [("r1", "2"), ("r2", "1")]),
            Net::new("n-c", [("j1", "2"), ("r2", "2")]).with_name("GND"),
        ],
    };
    let bench =
        Bench::new("GND").with_source(BenchSource::voltage("supply", Pin::new("j1", "1"), 9.0));
    let r = op(&netlist, &bench);
    assert!((r.node_voltages["n-b"] - 6.0).abs() < 1e-9);
}

#[test]
fn current_source_drives_into_plus() {
    let netlist = Netlist {
        components: vec![Component::new("r", ComponentKind::Resistor).with_value("1k")],
        nets: vec![Net::new("top", [("r", "1")]), Net::new("gnd", [("r", "2")])],
    };
    let bench = Bench::new("gnd").with_source(BenchSource::current("i", "top", 1e-3));
    assert!((op(&netlist, &bench).node_voltages["top"] - 1.0).abs() < 1e-9);
}

#[test]
fn transistor_switch_lights_an_led() {
    // 5 V → 330 Ω → LED → NPN collector; base driven through 10 kΩ.
    let netlist = Netlist {
        components: vec![
            Component::new("rl", ComponentKind::Resistor).with_value("330"),
            Component::new("d1", ComponentKind::Led).with_param("color", "red"),
            Component::new("q1", ComponentKind::Npn).with_param("hfe", 200.0),
            Component::new("rb", ComponentKind::Resistor).with_value("10k"),
        ],
        nets: vec![
            Net::new("vcc", [("rl", "1"), ("rb", "1")]).with_name("5V"),
            Net::new("a", [("rl", "2"), ("d1", "A")]),
            Net::new("k", [("d1", "K"), ("q1", "C")]),
            Net::new("base", [("rb", "2"), ("q1", "B")]),
            Net::new("gnd", [("q1", "E")]).with_name("GND"),
        ],
    };
    let bench = Bench::new("GND").with_supplies_from_net_names(&netlist);
    assert_eq!(bench.sources.len(), 1);
    let r = op(&netlist, &bench);
    let led = r.component_results.iter().find(|c| c.id == "d1").unwrap();
    assert!(
        led.current_through > 5e-3,
        "LED current {}",
        led.current_through
    );
    assert!(
        r.node_voltages["k"] < 0.3,
        "saturated: {}",
        r.node_voltages["k"]
    );
}

#[test]
fn op_amp_rails_come_from_supplies_on_its_pins() {
    // A follower whose input asks for more than the 5 V rail gives.
    let netlist = Netlist {
        components: vec![
            Component::new("u1", ComponentKind::OpAmp)
                .with_model(Model::OpAmp(OpAmpModel::default())),
            Component::new("rl", ComponentKind::Resistor).with_value("10k"),
            Component::new("rin", ComponentKind::Resistor).with_value("1k"),
        ],
        nets: vec![
            Net::new("in", [("u1", "+"), ("rin", "1")]),
            Net::new("src", [("rin", "2")]),
            Net::new("out", [("u1", "OUT"), ("u1", "-"), ("rl", "1")]),
            Net::new("vdd", [("u1", "V+")]),
            Net::new("gnd", [("rl", "2"), ("u1", "V-")]),
        ],
    };
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("vsupply", "vdd", 5.0))
        .with_source(BenchSource::voltage("vin", "src", 8.0));
    let imported = netlist.to_circuit(&bench).unwrap();
    let Some(CircuitElement::OpAmp(OpAmp { v_pos, v_neg, .. })) =
        imported.circuit.components.iter().find(|c| c.id() == "u1")
    else {
        panic!("no op-amp")
    };
    assert_eq!((*v_pos, *v_neg), (5.0, 0.0));
}

#[test]
fn ties_join_nets_into_one_node() {
    let netlist = Netlist {
        components: vec![
            Component::new("r1", ComponentKind::Resistor).with_value(1e3),
            Component::new("r2", ComponentKind::Resistor).with_value(1e3),
        ],
        nets: vec![
            Net::new("in", [("r1", "1")]),
            Net::new("mid", [("r1", "2"), ("r2", "1")]),
            Net::new("gnda", [("r2", "2")]),
            Net::new("gnd", Vec::<(&str, &str)>::new()),
        ],
    };
    let bench = Bench::new("gnd")
        .with_tie("gnda", "gnd")
        .with_source(BenchSource::voltage("v", "in", 4.0));
    let imported = netlist.to_circuit(&bench).unwrap();
    assert_eq!(imported.node("gnda"), imported.node("gnd"));
    let r = op(&netlist, &bench);
    assert!((r.node_voltages["mid"] - 2.0).abs() < 1e-9);
}

#[test]
fn mosfet_body_on_its_source_is_fine_elsewhere_refused() {
    let netlist = |body_net: &str| Netlist {
        components: vec![
            Component::new("m1", ComponentKind::Nmos),
            Component::new("rd", ComponentKind::Resistor).with_value("10k"),
        ],
        nets: vec![
            Net::new("g", [("m1", "G")]),
            Net::new("vdd", [("rd", "1")]),
            Net::new("d", [("rd", "2"), ("m1", "D")]),
            Net::new("gnd", [("m1", "S")]),
            Net::new("other", Vec::<(&str, &str)>::new()),
        ]
        .into_iter()
        .map(|mut n| {
            if n.id == body_net {
                n.pins.push(Pin::new("m1", "B"));
            }
            n
        })
        .collect(),
    };
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("vdd", "vdd", 5.0))
        .with_source(BenchSource::voltage("vg", "g", 5.0));
    let r = op(&netlist("gnd"), &bench);
    assert!(r.node_voltages["d"] < 1.0);
    assert_eq!(
        err(&netlist("other"), &bench),
        ImportError::BodyNotAtSource {
            component: "m1".into()
        }
    );
}

#[test]
fn transient_charges_an_rc() {
    let netlist = Netlist {
        components: vec![
            Component::new("r", ComponentKind::Resistor).with_value("1k"),
            Component::new("c", ComponentKind::Capacitor).with_value("1uF"),
        ],
        nets: vec![
            Net::new("in", [("r", "1")]),
            Net::new("out", [("r", "2"), ("c", "1")]),
            Net::new("gnd", [("c", "2")]),
        ],
    };
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("v", "in", 1.0))
        .with_analysis(Analysis::Transient(TransientConfig::new(1e-3, 1e-5)));
    let Simulation::Transient(r) = simulate(&netlist, &bench).unwrap().result else {
        panic!()
    };
    check_rc(r);

    let pulsed = Bench::new("gnd")
        .with_source(
            BenchSource::voltage("v", "in", 0.0).with_waveform(Waveform::Pulse {
                v1: 0.0,
                v2: 1.0,
                delay: 0.0,
                rise_time: 1e-9,
                fall_time: 1e-9,
                pulse_width: 1.0,
                period: 0.0,
            }),
        )
        .with_analysis(Analysis::Transient(TransientConfig::new(1e-3, 1e-5)));
    let Simulation::Transient(r) = simulate(&netlist, &pulsed).unwrap().result else {
        panic!()
    };
    check_rc(r);
}

/// Starting uncharged, as the transient does by default, the RC's output
/// at τ = 1 ms is 1 − 1/e.
fn check_rc(r: crate::SimulationResult) {
    let at_tau = r
        .transient
        .unwrap()
        .timesteps
        .into_iter()
        .min_by(|a, b| (a.time - 1e-3).abs().total_cmp(&(b.time - 1e-3).abs()))
        .unwrap();
    let want = 1.0 - (-at_tau.time / 1e-3_f64).exp();
    assert!((at_tau.node_voltages["out"] - want).abs() < 5e-3);
}

#[test]
fn ac_and_dc_sweep_name_a_bench_source() {
    let netlist = divider("1k", "1k");
    let ac = Bench::new("gnd")
        .with_source(BenchSource::voltage("sig", "in", 0.0))
        .with_analysis(Analysis::Ac(AcConfig::new("sig", 10.0, 1e3, 3)));
    let Simulation::Ac(r) = simulate(&netlist, &ac).unwrap().result else {
        panic!()
    };
    assert!((r.points[0].node_voltages["out"].norm() - 0.5).abs() < 1e-9);

    let sweep = Bench::new("gnd")
        .with_source(BenchSource::voltage("sig", "in", 0.0))
        .with_analysis(Analysis::DcSweep(crate::DcSweepAxis::new(
            "sig", 0.0, 4.0, 5,
        )));
    let Simulation::DcSweep(r) = simulate(&netlist, &sweep).unwrap().result else {
        panic!()
    };
    let curve = r.node_voltage_curve("out");
    assert!((curve[4].1 - 2.0).abs() < 1e-9);
}

#[test]
fn supply_names_say_what_was_assumed_and_what_was_left() {
    let named = |id: &str, name: &str| Net::new(id, [("r", id)]).with_name(name);
    let netlist = Netlist {
        nets: vec![
            named("a", "3V3"),
            named("b", "+5V"),
            named("c", "VCC"),
            named("d", "3V3_A"),
            named("e", "GND"),
            named("f", "AGND"),
            named("g", "SDA"),
            named("h", "-12V"),
            named("i", "VBAT"),
            named("j", "+12V_IN"),
            named("k", "AVDD"),
            named("l", "V+"),
            named("m", "0V"),
            Net::new("n", [("r", "n")]),
        ],
        ..Default::default()
    };
    let names = supply_names(&netlist);
    let assumed: Vec<(&str, &str, f64)> = names
        .assumed
        .iter()
        .map(|s| (s.net.as_str(), s.name.as_str(), s.volts))
        .collect();
    assert_eq!(
        assumed,
        [("a", "3V3", 3.3), ("b", "+5V", 5.0), ("h", "-12V", -12.0)]
    );
    assert_eq!(names.unresolved, ["c", "d", "i", "j", "k", "l"]);
    assert_eq!(names.grounds, ["e", "f", "m"]);

    // The bench helper adds exactly the assumed supplies.
    let bench = Bench::new("e").with_supplies_from_net_names(&netlist);
    let sources: Vec<&str> = bench.sources.iter().map(|s| s.id.as_str()).collect();
    assert_eq!(sources, ["supply:a", "supply:b", "supply:h"]);
}

#[test]
fn power_names_that_state_a_voltage() {
    use super::bench::voltage_from_name;
    for (name, want) in [
        ("3V3", 3.3),
        ("+5V", 5.0),
        ("5V0", 5.0),
        ("12V", 12.0),
        ("-12V", -12.0),
        ("+3.3V", 3.3),
    ] {
        assert_eq!(voltage_from_name(name), Some(want), "{name}");
    }
    for name in ["VCC", "VDD", "GND", "VBAT", "3V3_A", "AGND", "5"] {
        assert_eq!(voltage_from_name(name), None, "{name}");
    }
}

#[test]
fn every_kind_has_a_name_that_reads_back() {
    for (name, kind) in KINDS {
        assert_eq!(ComponentKind::from_name(name), *kind);
        assert_eq!(kind.name(), *name);
    }
    assert_eq!(
        ComponentKind::from_name("mcu-qfn32"),
        ComponentKind::Other("mcu-qfn32".into())
    );
}

#[test]
fn passive_and_unknown_kinds_have_no_terminals() {
    for (_, kind) in KINDS {
        assert_eq!(
            kind.terminals().is_empty(),
            kind.is_passive() || *kind == ComponentKind::BlackBox,
            "{}",
            kind.name()
        );
        assert!(kind.optional_terminals() < kind.terminals().len().max(1));
    }
    assert!(ComponentKind::Other("mcu-qfn32".into())
        .terminals()
        .is_empty());
}

// --- errors name the caller's ids -------------------------------------

fn gnd_bench() -> Bench {
    Bench::new("gnd").with_source(BenchSource::voltage("v", "in", 1.0))
}

#[test]
fn unknown_kind_names_the_component() {
    let mut n = divider("1k", "1k");
    n.components[1].kind = ComponentKind::from_name("mcu-qfn32");
    assert_eq!(
        err(&n, &gnd_bench()),
        ImportError::UnsupportedKind {
            component: "r2".into(),
            kind: "mcu-qfn32".into()
        }
    );
}

#[test]
fn unknown_terminal_lists_the_names() {
    let mut n = divider("1k", "1k");
    n.nets[1].pins[1].terminal = "anode".into();
    let e = err(&n, &gnd_bench());
    assert!(matches!(&e, ImportError::UnknownTerminal { component, .. } if component == "r2"));
    assert!(e.to_string().contains("\"1\", \"2\""), "{e}");
}

#[test]
fn transistor_terminals_are_names_not_pin_numbers() {
    let n = Netlist {
        components: vec![Component::new("q1", ComponentKind::Npn)],
        nets: vec![Net::new("x", [("q1", "1")])],
    };
    assert!(matches!(
        err(&n, &Bench::new("x")),
        ImportError::UnknownTerminal { .. }
    ));
}

#[test]
fn terminal_on_two_nets() {
    let mut n = divider("1k", "1k");
    n.nets[2].pins.push(Pin::new("r1", "2"));
    assert_eq!(
        err(&n, &gnd_bench()),
        ImportError::TerminalOnTwoNets {
            component: "r1".into(),
            terminal: "2".into(),
            first: "out".into(),
            second: "gnd".into()
        }
    );
}

#[test]
fn unconnected_terminal() {
    let mut n = divider("1k", "1k");
    n.nets[2].pins.clear();
    assert_eq!(
        err(&n, &gnd_bench()),
        ImportError::Unconnected {
            component: "r2".into(),
            terminal: "2".into()
        }
    );
}

#[test]
fn net_naming_a_missing_component() {
    let mut n = divider("1k", "1k");
    n.nets[0].pins.push(Pin::new("r9", "1"));
    assert_eq!(
        err(&n, &gnd_bench()),
        ImportError::UnknownComponent {
            net: "in".into(),
            component: "r9".into()
        }
    );
}

#[test]
fn values_missing_or_not_numbers() {
    let mut n = divider("1k", "1k");
    n.components[0].value = None;
    assert!(matches!(
        err(&n, &gnd_bench()),
        ImportError::MissingValue { component, .. } if component == "r1"
    ));
    let n = divider("BC547", "1k");
    assert!(matches!(
        err(&n, &gnd_bench()),
        ImportError::BadValue { component, field, .. } if component == "r1" && field == "value"
    ));
}

#[test]
fn unknown_parameter_and_wrong_model() {
    let mut n = divider("1k", "1k");
    n.components[0] = n.components[0].clone().with_param("tolerance", "1%");
    assert!(matches!(
        err(&n, &gnd_bench()),
        ImportError::UnknownParameter { component, parameter, .. }
            if component == "r1" && parameter == "tolerance"
    ));
    let mut n = divider("1k", "1k");
    n.components[0] = n.components[0]
        .clone()
        .with_model(Model::Diode(DiodeModel::default()));
    assert!(matches!(
        err(&n, &gnd_bench()),
        ImportError::WrongModel { component, model, .. } if component == "r1" && model == "diode"
    ));
}

#[test]
fn bench_errors() {
    let n = divider("1k", "1k");
    assert!(matches!(
        err(&n, &Bench::new("nowhere")),
        ImportError::UnknownNet { item, .. } if item == "ground"
    ));
    let dup = gnd_bench().with_source(BenchSource::voltage("r1", "in", 1.0));
    assert_eq!(err(&n, &dup), ImportError::DuplicateId { id: "r1".into() });
    let mut named = divider("1k", "1k");
    named.nets[0].name = Some("X".into());
    named.nets[1].name = Some("X".into());
    assert!(matches!(
        err(&named, &Bench::new("gnd").with_source(BenchSource::voltage("v", "X", 1.0))),
        ImportError::UnknownNet { problem, .. } if problem.contains("several")
    ));
}

#[test]
fn solver_errors_come_through_with_the_net_id() {
    // A net with only one terminal on it floats.
    let mut n = divider("1k", "1k");
    n.components
        .push(Component::new("r3", ComponentKind::Resistor).with_value("1k"));
    n.nets[1].pins.push(Pin::new("r3", "1"));
    n.nets.push(Net::new("dangling", [("r3", "2")]));
    match simulate(&n, &gnd_bench()) {
        Err(BenchError::Sim(crate::SimError::FloatingNode(node))) => assert_eq!(node, "dangling"),
        other => panic!("{other:?}"),
    }
}

#[cfg(feature = "serde")]
#[test]
fn reads_json_and_round_trips() {
    let json = r#"{
        "components": [
            {"id": "c-1", "kind": "resistor", "value": "4k7"},
            {"id": "c-2", "kind": "led", "params": {"color": "green"}},
            {"id": "c-3", "kind": "npn", "params": {"bf": 150},
             "model": {"type": "bjt", "vaf": 80}},
            {"id": "c-4", "kind": "mcu-qfn32", "value": "MCU"},
            {"id": "c-5", "kind": "Crystal", "value": "16MHz"}
        ],
        "nets": [
            {"id": "n-1", "name": "3V3", "pins": [{"component": "c-1", "terminal": "1"}]}
        ]
    }"#;
    let netlist: Netlist = serde_json::from_str(json).unwrap();
    assert_eq!(
        netlist.components[3].kind,
        ComponentKind::Other("mcu-qfn32".into())
    );
    assert_eq!(netlist.components[4].kind, ComponentKind::Crystal);
    let Some(Model::Bjt(m)) = &netlist.components[2].model else {
        panic!()
    };
    assert_eq!(m.vaf, 80.0);
    let back: Netlist = serde_json::from_str(&serde_json::to_string(&netlist).unwrap()).unwrap();
    assert_eq!(back, netlist);

    let bench_json = r#"{
        "ground": {"net": "gnd"},
        "ties": [[{"net": "gnda"}, {"net": "gnd"}]],
        "sources": [
            {"id": "vin", "type": "voltage", "volts": 5, "plus": {"net": "3V3"}},
            {"id": "sig", "type": "current", "amps": 0.001,
             "plus": {"terminal": {"component": "j1", "terminal": "2"}}}
        ],
        "loads": [{"id": "rl", "kind": {"resistor": 1000}, "a": {"net": "out"}}],
        "analysis": {"type": "transient", "tstop": 0.001, "tstep": 1e-6}
    }"#;
    let bench: Bench = serde_json::from_str(bench_json).unwrap();
    assert_eq!(bench.sources[0].minus, None);
    assert!(matches!(&bench.analysis, Analysis::Transient(c) if c.adaptive && c.tstop == 1e-3));
    let back: Bench = serde_json::from_str(&serde_json::to_string(&bench).unwrap()).unwrap();
    assert_eq!(back, bench);
}

// --- every kind becomes its element -----------------------------------

/// Builds `component` alone: its terminal `terminals[i]` on net `n{i}`,
/// grounded at an empty `gnd` net. Returns its element.
fn build(component: Component, terminals: &[&str]) -> Result<CircuitElement, ImportError> {
    let id = component.id.clone();
    let mut nets: Vec<Net> = terminals
        .iter()
        .enumerate()
        .map(|(i, t)| Net::new(format!("n{i}"), [(id.as_str(), *t)]))
        .collect();
    nets.push(Net::new("gnd", Vec::<(&str, &str)>::new()));
    let netlist = Netlist {
        components: vec![component],
        nets,
    };
    let imported = netlist.to_circuit(&Bench::new("gnd"))?;
    Ok(imported
        .circuit
        .components
        .into_iter()
        .find(|e| e.id() == id || e.id().starts_with(&format!("{id}.")))
        .expect("the component's element"))
}

fn built(component: Component, terminals: &[&str]) -> CircuitElement {
    build(component, terminals).expect("builds")
}

/// `["n0", "n1", …]`, the nodes [`build`] puts the terminals on.
fn ns<const N: usize>() -> [String; N] {
    std::array::from_fn(|i| format!("n{i}"))
}

#[test]
fn two_terminal_passives_take_their_value() {
    assert_eq!(
        built(
            Component::new("c1", ComponentKind::CapacitorPolarized).with_value("10uF"),
            &["+", "NEG"]
        ),
        CircuitElement::Capacitor(Capacitor {
            id: "c1".into(),
            nodes: ns(),
            capacitance: 10e-6,
        })
    );
    assert_eq!(
        built(
            Component::new("l1", ComponentKind::Inductor).with_value(String::from("2.2mH")),
            &["a", "b"]
        ),
        CircuitElement::Inductor(Inductor {
            id: "l1".into(),
            nodes: ns(),
            inductance: 2.2e-3,
            saturation: None,
        })
    );
    assert_eq!(
        built(Component::new("t1", ComponentKind::Thermistor), &["1", "2"]),
        CircuitElement::Thermistor(Thermistor {
            id: "t1".into(),
            nodes: ns(),
            temperature: 298.15,
        })
    );
    assert_eq!(
        built(
            Component::new("p1", ComponentKind::Photoresistor).with_param("light_level", 0.4),
            &["1", "2"]
        ),
        CircuitElement::Photoresistor(Photoresistor {
            id: "p1".into(),
            nodes: ns(),
            light_level: 0.4,
        })
    );
    assert_eq!(
        built(
            Component::new("v1", ComponentKind::VoltageSource).with_value("3V3"),
            &["pos", "n"]
        ),
        CircuitElement::VoltageSource(VoltageSource {
            id: "v1".into(),
            nodes: ns(),
            voltage: 3.3,
            waveform: None,
        })
    );
    assert_eq!(
        built(
            Component::new("i1", ComponentKind::CurrentSource).with_value("2mA"),
            &["from", "to"]
        ),
        CircuitElement::CurrentSource(CurrentSource {
            id: "i1".into(),
            nodes: ns(),
            current: 2e-3,
            waveform: None,
        })
    );
}

#[test]
fn diodes_take_temperature_irradiance_and_breakdown() {
    let dm = DiodeModel::default();
    assert_eq!(
        built(
            Component::new("d1", ComponentKind::Diode)
                .with_param("temperature", 350.0)
                .with_model(Model::Diode(dm)),
            &["anode", "k"]
        ),
        CircuitElement::Diode(Diode {
            id: "d1".into(),
            nodes: ns(),
            temperature: 350.0,
            model: Some(dm),
        })
    );
    assert_eq!(
        built(
            Component::new("d2", ComponentKind::SchottkyDiode),
            &["a", "c"]
        ),
        CircuitElement::SchottkyDiode(SchottkyDiode {
            id: "d2".into(),
            nodes: ns(),
            temperature: NOMINAL_K,
        })
    );
    assert_eq!(
        built(
            Component::new("d3", ComponentKind::Photodiode).with_param("irradiance", "1m"),
            &["a", "k"]
        ),
        CircuitElement::Photodiode(Photodiode {
            id: "d3".into(),
            nodes: ns(),
            irradiance: 1e-3,
            temperature: NOMINAL_K,
        })
    );
    // The breakdown is the value, or `vz`, which wins over it.
    let zener = |c: Component| match built(c, &["a", "k"]) {
        CircuitElement::ZenerDiode(ZenerDiode { vz, .. }) => vz,
        other => panic!("{other:?}"),
    };
    assert_eq!(
        zener(Component::new("z", ComponentKind::ZenerDiode).with_value("5V1")),
        5.1
    );
    assert_eq!(
        zener(
            Component::new("z", ComponentKind::ZenerDiode)
                .with_value("5V1")
                .with_param("vz", 3.3)
        ),
        3.3
    );
    assert_eq!(
        built(Component::new("led", ComponentKind::Led), &["a", "k"]),
        CircuitElement::Led(Led {
            id: "led".into(),
            nodes: ns(),
            color: "red".into(),
            temperature: NOMINAL_K,
        })
    );
    let vp = VaractorParams {
        cj0: 50e-12,
        ..VaractorParams::default()
    };
    assert_eq!(
        built(
            Component::new("cv", ComponentKind::Varactor).with_model(Model::Varactor(vp.clone())),
            &["a", "k"]
        ),
        CircuitElement::Varactor(Varactor {
            id: "cv".into(),
            nodes: ns(),
            params: vp,
        })
    );
    assert_eq!(
        built(Component::new("cv", ComponentKind::Varactor), &["a", "k"]),
        CircuitElement::Varactor(Varactor {
            id: "cv".into(),
            nodes: ns(),
            params: VaractorParams::default(),
        })
    );
}

#[test]
fn transistors_take_their_defaults_and_models() {
    let bm = BjtModel {
        vaf: 50.0,
        ..BjtModel::default()
    };
    assert_eq!(
        built(
            Component::new("q1", ComponentKind::Pnp)
                .with_param("beta", 80.0)
                .with_model(Model::Bjt(bm)),
            &["b", "c", "e"]
        ),
        CircuitElement::Bjt(Bjt {
            id: "q1".into(),
            nodes: ns(),
            kind: BjtKind::Pnp,
            bf: 80.0,
            temperature: NOMINAL_K,
            parasitic_caps: None,
            model: Some(bm),
        })
    );
    match built(Component::new("q2", ComponentKind::Npn), &["b", "c", "e"]) {
        CircuitElement::Bjt(Bjt { bf, kind, .. }) => assert_eq!((bf, kind), (100.0, BjtKind::Npn)),
        other => panic!("{other:?}"),
    }

    // A MOSFET's body may be left off; it is tied to the source.
    assert_eq!(
        built(Component::new("m1", ComponentKind::Pmos), &["g", "d", "s"]),
        CircuitElement::Mosfet(Mosfet {
            id: "m1".into(),
            nodes: ns(),
            kind: MosfetKind::Pmos,
            params: MosfetParams::default_pmos(),
            parasitic_caps: None,
            body_diode: None,
        })
    );
    let mp = MosfetParams {
        vto: 1.5,
        ..MosfetParams::default_nmos()
    };
    match built(
        Component::new("m2", ComponentKind::Nmos).with_model(Model::Mosfet(mp.clone())),
        &["g", "d", "s"],
    ) {
        CircuitElement::Mosfet(Mosfet { params, kind, .. }) => {
            assert_eq!((params, kind), (mp, MosfetKind::Nmos))
        }
        other => panic!("{other:?}"),
    }

    // A JFET's pinch-off defaults to the sign of its channel.
    assert_eq!(
        built(Component::new("j1", ComponentKind::Njfet), &["g", "d", "s"]),
        CircuitElement::Jfet(Jfet {
            id: "j1".into(),
            nodes: ns(),
            kind: JfetKind::NChannel,
            idss: 10e-3,
            vp: -2.0,
            parasitic_caps: None,
        })
    );
    assert_eq!(
        built(
            Component::new("j2", ComponentKind::Pjfet).with_param("idss", "5mA"),
            &["g", "d", "s"]
        ),
        CircuitElement::Jfet(Jfet {
            id: "j2".into(),
            nodes: ns(),
            kind: JfetKind::PChannel,
            idss: 5e-3,
            vp: 2.0,
            parasitic_caps: None,
        })
    );
    let ip = IgbtParams {
        vth: 6.0,
        ..IgbtParams::default()
    };
    assert_eq!(
        built(
            Component::new("g1", ComponentKind::Igbt).with_model(Model::Igbt(ip.clone())),
            &["gate", "collector", "emitter"]
        ),
        CircuitElement::Igbt(Igbt {
            id: "g1".into(),
            nodes: ns(),
            params: ip,
        })
    );
    match built(Component::new("g2", ComponentKind::Igbt), &["g", "c", "e"]) {
        CircuitElement::Igbt(Igbt { params, .. }) => assert_eq!(params, IgbtParams::default()),
        other => panic!("{other:?}"),
    }
}

#[test]
fn comparator_rails_default_to_15_volts_or_take_parameters() {
    // Supply pins left off: ±15 V.
    assert_eq!(
        built(
            Component::new("u1", ComponentKind::Comparator),
            &["in+", "in-", "out"]
        ),
        CircuitElement::Comparator(Comparator {
            id: "u1".into(),
            nodes: ns(),
            v_pos: 15.0,
            v_neg: -15.0,
        })
    );
    // Parameters win over any supply on the pins.
    match built(
        Component::new("u2", ComponentKind::OpAmp)
            .with_param("v_pos", 3.3)
            .with_param("v_neg", 0.0),
        &["+", "-", "out", "v+", "v-"],
    ) {
        CircuitElement::OpAmp(OpAmp {
            v_pos,
            v_neg,
            model,
            ..
        }) => assert_eq!((v_pos, v_neg, model), (3.3, 0.0, None)),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_supply_not_returned_to_ground_is_no_rail() {
    // The op-amp's V+ net is driven 5 V above `vref`, not above ground,
    // so it is not a rail of known voltage and the default stands.
    let netlist = Netlist {
        components: vec![Component::new("u1", ComponentKind::OpAmp)],
        nets: vec![
            Net::new("in", [("u1", "+")]),
            Net::new("out", [("u1", "out"), ("u1", "-")]),
            Net::new("vdd", [("u1", "v+")]),
            Net::new("vref", Vec::<(&str, &str)>::new()),
            Net::new("gnd", [("u1", "v-")]),
        ],
    };
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("vs", "vdd", 5.0).with_minus("vref"))
        .with_source(BenchSource::voltage("vr", "vref", 1.0));
    let imported = netlist.to_circuit(&bench).unwrap();
    let Some(CircuitElement::OpAmp(OpAmp { v_pos, v_neg, .. })) =
        imported.circuit.components.iter().find(|c| c.id() == "u1")
    else {
        panic!("no op-amp")
    };
    assert_eq!((*v_pos, *v_neg), (15.0, 0.0));
    let vs = imported
        .circuit
        .components
        .iter()
        .find(|c| c.id() == "vs")
        .unwrap();
    assert_eq!(vs.nodes(), ["vdd", "vref"]);
}

#[test]
fn switches_fuses_relays_and_transformers() {
    assert_eq!(
        built(
            Component::new("s1", ComponentKind::Switch).with_param("closed", true),
            &["1", "2"]
        ),
        CircuitElement::Switch(Switch {
            id: "s1".into(),
            nodes: ns(),
            closed: true,
        })
    );
    assert_eq!(
        built(Component::new("s2", ComponentKind::PushButton), &["1", "2"]),
        CircuitElement::Pushbutton(Pushbutton {
            id: "s2".into(),
            nodes: ns(),
            closed: false,
        })
    );
    // A fuse's rating defaults to 1 A, and a part number in its value
    // is ignored rather than refused.
    assert_eq!(
        built(
            Component::new("f1", ComponentKind::Fuse)
                .with_value("MF-R050")
                .with_param("blown", true),
            &["1", "2"]
        ),
        CircuitElement::Fuse(Fuse {
            id: "f1".into(),
            nodes: ns(),
            rating: 1.0,
            blown: true,
        })
    );
    match built(
        Component::new("f2", ComponentKind::Fuse).with_value("500mA"),
        &["1", "2"],
    ) {
        CircuitElement::Fuse(Fuse { rating, blown, .. }) => {
            assert_eq!((rating, blown), (0.5, false))
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(
        built(
            Component::new("k1", ComponentKind::Relay).with_param("pickup_voltage", "3V5"),
            &["a1", "a2", "com", "no"]
        ),
        CircuitElement::Relay(Relay {
            id: "k1".into(),
            nodes: ns(),
            coil_resistance: 500.0,
            pickup_voltage: 3.5,
            inductance: 0.0,
        })
    );
    assert_eq!(
        built(
            Component::new("tr1", ComponentKind::Transformer)
                .with_param("l1", "10mH")
                .with_param("l2", "1mH"),
            &["p1", "q1", "sec+", "sec-"]
        ),
        CircuitElement::Transformer(Transformer {
            id: "tr1".into(),
            nodes: ns(),
            l1: 10e-3,
            l2: 1e-3,
            k: 0.999,
            saturation: None,
        })
    );
}

#[test]
fn regulators_and_batteries() {
    let rp = RegulatorParams::lm7805();
    assert_eq!(
        built(
            Component::new("u1", ComponentKind::Regulator)
                .with_value("LM7805")
                .with_param("voltage", 5.0)
                .with_model(Model::Regulator(rp)),
            &["in", "out", "gnd"]
        ),
        CircuitElement::VoltageRegulator(VoltageRegulator {
            id: "u1".into(),
            nodes: ns(),
            voltage: 5.0,
            model: Some(rp),
        })
    );
    match built(
        Component::new("u2", ComponentKind::Regulator).with_value("3V3"),
        &["vin", "vout", "com"],
    ) {
        CircuitElement::VoltageRegulator(VoltageRegulator { voltage, model, .. }) => {
            assert_eq!((voltage, model), (3.3, None))
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(
        built(
            Component::new("b1", ComponentKind::Battery)
                .with_value("9V")
                .with_param("capacity", 0.5)
                .with_param("state_of_charge", 0.8),
            &["+", "-"]
        ),
        CircuitElement::Battery(Battery {
            id: "b1".into(),
            nodes: ns(),
            params: BatteryParams::constant(9.0, 0.5, 0.1),
            state_of_charge: 0.8,
        })
    );
    // A model stands in for the value.
    let li = BatteryParams::lithium_ion(2.0, 0.05);
    assert_eq!(
        built(
            Component::new("b2", ComponentKind::Battery).with_model(Model::Battery(li.clone())),
            &["+", "-"]
        ),
        CircuitElement::Battery(Battery {
            id: "b2".into(),
            nodes: ns(),
            params: li,
            state_of_charge: 1.0,
        })
    );
}

#[test]
fn potentiometer_divides_by_its_position() {
    // Position 0 is the top, so at 0.25 a quarter of the 10 kΩ track
    // (2.5 kΩ) is above the wiper and 7.5 kΩ below it, here in parallel
    // with a 10 kΩ bench load.
    let netlist = Netlist {
        components: vec![Component::new("rv1", ComponentKind::Potentiometer)
            .with_value("10k")
            .with_param("position", 0.25)],
        nets: vec![
            Net::new("top", [("rv1", "cw")]),
            Net::new("wiper", [("rv1", "w")]),
            Net::new("gnd", [("rv1", "ccw")]),
        ],
    };
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("v", "top", 10.0))
        .with_load(BenchLoad::new("rl", LoadKind::Resistor(10e3), "wiper"));
    let lower = 7.5e3 * 10e3 / (7.5e3 + 10e3);
    let want = 10.0 * lower / (2.5e3 + lower);
    assert!((op(&netlist, &bench).node_voltages["wiper"] - want).abs() < 1e-6);
}

#[test]
fn battery_sags_across_its_series_resistance() {
    // 9 V behind 1 Ω into an 8 Ω bench load: 1 A, so 8 V at the terminal.
    let netlist = Netlist {
        components: vec![Component::new("b1", ComponentKind::Battery)
            .with_value(9.0)
            .with_param("r_series", 1.0)],
        nets: vec![
            Net::new("vbat", [("b1", "+")]),
            Net::new("gnd", [("b1", "-")]),
        ],
    };
    let bench = Bench::new("gnd").with_load(BenchLoad::new("rl", LoadKind::Resistor(8.0), "vbat"));
    assert!((op(&netlist, &bench).node_voltages["vbat"] - 8.0).abs() < 1e-6);
}

#[test]
fn ideal_regulator_holds_its_output() {
    let netlist = Netlist {
        components: vec![Component::new("u1", ComponentKind::Regulator).with_value("5V")],
        nets: vec![
            Net::new("vin", [("u1", "in")]),
            Net::new("vout", [("u1", "out")]),
            Net::new("gnd", [("u1", "gnd")]),
        ],
    };
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("v", "vin", 12.0))
        .with_load(BenchLoad::new("rl", LoadKind::Resistor(100.0), "vout"));
    let r = op(&netlist, &bench);
    assert!((r.node_voltages["vout"] - 5.0).abs() < 1e-6);
    let rl = r.component_results.iter().find(|c| c.id == "rl").unwrap();
    assert!((rl.current_through - 0.05).abs() < 1e-8);
}

#[test]
fn part_data_is_kept_under_the_component_id() {
    let part = Part {
        reference: Some("R1".into()),
        ..Part::default()
    };
    let mut n = divider("1k", "1k");
    n.components[0] = n.components[0].clone().with_part(part.clone());
    // A passive part's data is kept too, though it adds no element.
    n.components
        .push(Component::new("j1", ComponentKind::Connector).with_part(part.clone()));
    let imported = n.to_circuit(&gnd_bench()).unwrap();
    assert_eq!(imported.circuit.parts.get("r1"), Some(&part));
    assert_eq!(imported.circuit.parts.get("j1"), Some(&part));
    assert!(!imported.circuit.parts.contains_key("r2"));
    assert!(imported.circuit.components.iter().all(|c| c.id() != "j1"));
}

// --- bench loads and floating sources ---------------------------------

#[test]
fn loads_become_elements_between_their_nets() {
    // R1 = 1 kΩ from `in` to `out`; a 1 kΩ load across it and another to
    // ground: out = 10 V · 1k / (1k + 500) = 6.667 V.
    let netlist = Netlist {
        components: vec![Component::new("r1", ComponentKind::Resistor).with_value("1k")],
        nets: vec![
            Net::new("in", [("r1", "1")]),
            Net::new("out", [("r1", "2")]),
            Net::new("gnd", Vec::<(&str, &str)>::new()),
        ],
    };
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("v", "in", 10.0))
        .with_load(BenchLoad::new("rp", LoadKind::Resistor(1e3), "in").with_b("out"))
        .with_load(BenchLoad::new("rg", LoadKind::Resistor(1e3), "out"));
    let r = op(&netlist, &bench);
    assert!((r.node_voltages["out"] - 10.0 * 2.0 / 3.0).abs() < 1e-9);

    // At DC a capacitor load is open and an inductor load a short.
    let reactive = bench
        .clone()
        .with_load(BenchLoad::new("cl", LoadKind::Capacitor(1e-6), "out"))
        .with_load(BenchLoad::new(
            "ll",
            LoadKind::Inductor(1e-3),
            Pin::new("r1", "2"),
        ));
    let imported = netlist.to_circuit(&reactive).unwrap();
    let element = |id: &str| {
        imported
            .circuit
            .components
            .iter()
            .find(|c| c.id() == id)
            .unwrap()
            .clone()
    };
    assert_eq!(
        element("cl"),
        CircuitElement::Capacitor(Capacitor {
            id: "cl".into(),
            nodes: ["out".into(), "gnd".into()],
            capacitance: 1e-6,
        })
    );
    assert_eq!(
        element("ll"),
        CircuitElement::Inductor(Inductor {
            id: "ll".into(),
            nodes: ["out".into(), "gnd".into()],
            inductance: 1e-3,
            saturation: None,
        })
    );
    assert!(op(&netlist, &reactive).node_voltages["out"].abs() < 1e-6);
}

#[test]
fn a_source_stacks_on_its_minus_net() {
    // 3 V from `top` to `mid`, on 5 V from `mid` to ground: 8 V at the top.
    let netlist = Netlist {
        components: vec![Component::new("r1", ComponentKind::Resistor).with_value("1k")],
        nets: vec![
            Net::new("top", [("r1", "1")]),
            Net::new("mid", Vec::<(&str, &str)>::new()),
            Net::new("gnd", [("r1", "2")]),
        ],
    };
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("v1", "mid", 5.0))
        .with_source(BenchSource::voltage("v2", "top", 3.0).with_minus(String::from("mid")));
    let r = op(&netlist, &bench);
    assert!((r.node_voltages["top"] - 8.0).abs() < 1e-9);
}

// --- more errors that name the caller's ids ---------------------------

#[test]
fn duplicate_net_id() {
    let mut n = divider("1k", "1k");
    n.nets.push(Net::new("out", Vec::<(&str, &str)>::new()));
    assert_eq!(
        err(&n, &gnd_bench()),
        ImportError::DuplicateId { id: "out".into() }
    );
}

#[test]
fn an_unknown_kind_on_no_net_is_still_refused() {
    let mut n = divider("1k", "1k");
    n.components
        .push(Component::new("u1", ComponentKind::from_name("mcu-qfn32")));
    let e = err(&n, &gnd_bench());
    assert_eq!(
        e,
        ImportError::UnsupportedKind {
            component: "u1".into(),
            kind: "mcu-qfn32".into()
        }
    );
    assert_eq!(
        e.to_string(),
        "component \"u1\": sindr can't simulate a \"mcu-qfn32\""
    );
}

#[test]
fn values_of_the_wrong_type() {
    let bad = |c: Component, terminals: &[&str]| match build(c, terminals) {
        Err(ImportError::BadValue {
            field, expected, ..
        }) => (field, expected),
        other => panic!("{other:?}"),
    };
    let r = || Component::new("r", ComponentKind::Resistor);
    assert_eq!(
        bad(r().with_value(f64::NAN), &["1", "2"]),
        ("value".into(), "a number".into())
    );
    assert_eq!(
        bad(r().with_value(true), &["1", "2"]),
        ("value".into(), "a number".into())
    );
    assert_eq!(
        bad(
            Component::new("s", ComponentKind::Switch).with_param("closed", "yes"),
            &["1", "2"]
        ),
        ("closed".into(), "true or false".into())
    );
    assert_eq!(
        bad(
            Component::new("d", ComponentKind::Led).with_param("color", 1.0),
            &["a", "k"]
        ),
        ("color".into(), "text".into())
    );
    assert_eq!(
        bad(
            Component::new("q", ComponentKind::Npn).with_param("hfe", "high"),
            &["b", "c", "e"]
        ),
        ("hfe".into(), "a number".into())
    );
    let e = build(r().with_value(f64::INFINITY), &["1", "2"]).unwrap_err();
    assert_eq!(
        e.to_string(),
        "component \"r\": value Number(inf) isn't a number"
    );
}

#[test]
fn kinds_that_need_a_value_say_what() {
    let missing = |c: Component, terminals: &[&str]| match build(c, terminals) {
        Err(ImportError::MissingValue { component, what }) => {
            assert_eq!(component, "x");
            what
        }
        other => panic!("{other:?}"),
    };
    // A part number isn't a breakdown voltage.
    assert!(missing(
        Component::new("x", ComponentKind::ZenerDiode).with_value("BZX84"),
        &["a", "k"]
    )
    .contains("\"vz\""));
    assert!(missing(
        Component::new("x", ComponentKind::Regulator),
        &["in", "out", "gnd"]
    )
    .contains("\"voltage\""));
    assert_eq!(
        missing(
            Component::new("x", ComponentKind::Relay),
            &["coil+", "coil-", "com", "no"]
        ),
        "a pickup voltage (V) (parameter \"pickup_voltage\")"
    );
    assert_eq!(
        missing(
            Component::new("x", ComponentKind::Transformer).with_param("l1", 1e-3),
            &["p1", "q1", "p2", "q2"]
        ),
        "a secondary inductance (H) (parameter \"l2\")"
    );
    assert_eq!(
        missing(Component::new("x", ComponentKind::Battery), &["+", "-"]),
        "a voltage (V), or a battery model"
    );
    assert_eq!(
        missing(
            Component::new("x", ComponentKind::Potentiometer),
            &["1", "2", "3"]
        ),
        "an end-to-end resistance (Ω)"
    );
    for (kind, what) in [
        (ComponentKind::Inductor, "an inductance (H)"),
        (ComponentKind::VoltageSource, "a voltage (V)"),
        (ComponentKind::CurrentSource, "a current (A)"),
    ] {
        let terminals: Vec<&str> = kind.terminals().iter().map(|t| t[0]).collect();
        assert_eq!(missing(Component::new("x", kind), &terminals), what);
    }
}

#[test]
fn each_kind_takes_only_its_own_parameters() {
    // `irradiance` belongs to the photodiode, not a plain diode.
    assert_eq!(
        build(
            Component::new("d", ComponentKind::Diode).with_param("irradiance", 1.0),
            &["a", "k"]
        ),
        Err(ImportError::UnknownParameter {
            component: "d".into(),
            kind: "diode".into(),
            parameter: "irradiance".into(),
        })
    );
    // Every parameter a kind lists is accepted (each given a plausible
    // number, or the type its reader wants).
    for (_, kind) in KINDS {
        if kind.is_passive() || *kind == ComponentKind::BlackBox {
            continue;
        }
        let mut c = Component::new("x", kind.clone()).with_value(1.0);
        for &p in kind.params() {
            let v: Value = match p {
                "closed" | "blown" => true.into(),
                "color" | "colour" => "green".into(),
                "position" | "light_level" | "state_of_charge" | "k" => 0.5.into(),
                "vf" => 2.0.into(),
                "vf2" => 2.2.into(),
                "if" | "vce_sat_ic" | "hfe_ic" => 0.02.into(),
                "if2" => 0.05.into(),
                "iz" => 5e-3.into(),
                "vz" => 5.1.into(),
                "bf" | "beta" | "hfe" => 100.0.into(),
                "vce_sat" => 0.2.into(),
                "vce_sat_ib" => 2e-3.into(),
                "vth" => 2.0.into(),
                "vth_id" => 250e-6.into(),
                "rds_on" => 0.5.into(),
                "rds_on_vgs" => 10.0.into(),
                _ => 1.0.into(),
            };
            c = c.with_param(p, v);
        }
        let required = kind.terminals().len() - kind.optional_terminals();
        let terminals: Vec<&str> = kind.terminals()[..required].iter().map(|t| t[0]).collect();
        build(c, &terminals).unwrap_or_else(|e| panic!("{}: {e}", kind.name()));
    }
}

#[test]
fn every_model_is_refused_by_the_wrong_kind() {
    for (model, name) in [
        (Model::Diode(DiodeModel::default()), "diode"),
        (Model::Bjt(BjtModel::default()), "bjt"),
        (Model::Mosfet(MosfetParams::default_nmos()), "mosfet"),
        (Model::OpAmp(OpAmpModel::default()), "op_amp"),
        (Model::Regulator(RegulatorParams::default()), "regulator"),
        (
            Model::Battery(BatteryParams::lithium_ion(1.0, 0.1)),
            "battery",
        ),
        (Model::Varactor(VaractorParams::default()), "varactor"),
        (Model::Igbt(IgbtParams::default()), "igbt"),
    ] {
        let e = build(
            Component::new("r", ComponentKind::Resistor)
                .with_value(1.0)
                .with_model(model),
            &["1", "2"],
        )
        .unwrap_err();
        assert_eq!(
            e.to_string(),
            format!("component \"r\" (resistor) can't take a {name} model")
        );
    }
    // A comparator has no op-amp macromodel, nor a Schottky a diode model.
    assert!(matches!(
        build(
            Component::new("u", ComponentKind::Comparator)
                .with_model(Model::OpAmp(OpAmpModel::default())),
            &["+", "-", "out"]
        ),
        Err(ImportError::WrongModel { .. })
    ));
    assert!(matches!(
        build(
            Component::new("d", ComponentKind::SchottkyDiode)
                .with_model(Model::Diode(DiodeModel::default())),
            &["a", "k"]
        ),
        Err(ImportError::WrongModel { .. })
    ));
}

#[test]
fn bench_terminals_must_exist_and_be_connected() {
    let mut n = divider("1k", "1k");
    n.components
        .push(Component::new("tp1", ComponentKind::TestPoint));
    let on = |pin: Pin| Bench::new("gnd").with_source(BenchSource::voltage("v", pin, 1.0));
    assert_eq!(
        err(&n, &on(Pin::new("u9", "1"))),
        ImportError::UnknownComponent {
            net: "v".into(),
            component: "u9".into()
        }
    );
    let e = err(&n, &on(Pin::new("tp1", "1")));
    assert_eq!(
        e,
        ImportError::AttachedToUnconnected {
            item: "v".into(),
            component: "tp1".into(),
            terminal: "1".into()
        }
    );
    assert_eq!(
        e.to_string(),
        "v attaches to terminal \"1\" of component \"tp1\", which is on no net"
    );
    // A terminal the kind doesn't have is the same error as in a net.
    assert!(matches!(
        err(&n, &on(Pin::new("r1", "base"))),
        ImportError::UnknownTerminal { component, .. } if component == "r1"
    ));
    // Attached through a component's terminal, the source lands on its net.
    let imported = n.to_circuit(&on(Pin::new("r1", "1"))).unwrap();
    let v = imported
        .circuit
        .components
        .iter()
        .find(|c| c.id() == "v")
        .unwrap();
    assert_eq!(v.nodes(), ["in", "gnd"]);
}

#[test]
fn ties_name_the_missing_net() {
    let bench = gnd_bench().with_tie("gnd", "agnd");
    assert_eq!(
        err(&divider("1k", "1k"), &bench),
        ImportError::UnknownNet {
            item: "tie 1".into(),
            net: "agnd".into(),
            problem: "isn't in the netlist".into()
        }
    );
}

#[test]
fn loads_name_the_missing_net() {
    let bench = gnd_bench()
        .with_load(BenchLoad::new("rl", LoadKind::Resistor(1.0), "out").with_b("nowhere"));
    assert!(matches!(
        err(&divider("1k", "1k"), &bench),
        ImportError::UnknownNet { item, net, .. } if item == "rl" && net == "nowhere"
    ));
}

// --- netlists as an editor exports them -------------------------------

/// A board drawn in a schematic tool and exported as it stands: symbol ids
/// for kinds, terminals by id with each component's name table, text
/// values, and an IC sindr doesn't model on the supply.
///
/// A 10 V supply comes in on header J1 (1 = VCC, 2 = GND, 3 = OUT).
/// R1/R2 (10k/10k) set ref = 5 V; U2, an op-amp wired non-inverting with
/// Rf = 4k7 and Rg = 10k, gives out = 5 · (1 + 4.7/10) = 7.35 V. R3 (470)
/// and a green LED hang off out; Q1 sits on a 100k/1k bias; S1, an open
/// push-button, leaves its 10k pull-down at 0 V; U1 is the IC.
fn editor_board() -> (Netlist, ImportOptions) {
    let c = |id: &str, symbol: &str| Component::new(id, ComponentKind::from_name(symbol));
    let netlist = Netlist {
        components: vec![
            c("J1", "header-3"),
            c("R1", "resistor").with_value("10k"),
            c("R2", "resistor-iec").with_value("10k"),
            c("C1", "capacitor").with_value("100nF"),
            c("C2", "capacitor-polarized").with_value("4.7uF"),
            c("U2", "op-amp"),
            c("RF", "resistor").with_value("4k7"),
            c("RG", "resistor").with_value("10k"),
            c("R3", "resistor").with_value("470"),
            c("D1", "led").with_value("Green"),
            c("RB", "resistor").with_value("100k"),
            c("RE", "resistor").with_value("1k"),
            c("Q1", "npn").with_value("PMBT3904"),
            c("S1", "push-button"),
            c("RP", "resistor").with_value("10k"),
            c("TP1", "test-point"),
            c("U1", "mcu-qfn32").with_value("MCU"),
        ],
        nets: vec![
            Net::new(
                "net-vcc",
                [
                    ("J1", "1"),
                    ("R1", "1"),
                    ("C2", "1"),
                    ("U2", "V+"),
                    ("RB", "1"),
                    ("Q1", "3"),
                    ("S1", "1"),
                    ("U1", "1"),
                ],
            )
            .with_name("VCC"),
            Net::new(
                "net-gnd",
                [
                    ("J1", "2"),
                    ("R2", "2"),
                    ("C1", "2"),
                    ("C2", "2"),
                    ("U2", "V-"),
                    ("RG", "2"),
                    ("D1", "2"),
                    ("RE", "2"),
                    ("RP", "2"),
                    ("U1", "2"),
                ],
            )
            .with_name("GND"),
            Net::new(
                "net-ref",
                [("R1", "2"), ("R2", "1"), ("C1", "1"), ("U2", "+")],
            ),
            Net::new("net-fb", [("U2", "-"), ("RF", "2"), ("RG", "1")]),
            Net::new(
                "net-out",
                [
                    ("U2", "OUT"),
                    ("RF", "1"),
                    ("R3", "1"),
                    ("J1", "3"),
                    ("TP1", "1"),
                ],
            ),
            Net::new("net-led", [("R3", "2"), ("D1", "1")]),
            Net::new("net-base", [("RB", "2"), ("Q1", "1")]),
            Net::new("net-emitter", [("Q1", "2"), ("RE", "1")]),
            Net::new("net-btn", [("S1", "2"), ("RP", "1")]),
        ],
    };
    let options = ImportOptions::new()
        .with_unknown_kinds(UnknownKinds::BlackBox)
        .with_terminal_names("C2", [("1", "+"), ("2", "-")])
        .with_terminal_names(
            "U2",
            [
                ("+", "Non-inverting"),
                ("-", "Inverting"),
                ("OUT", "Output"),
                ("V+", "Positive supply"),
                ("V-", "Negative supply"),
            ],
        )
        .with_terminal_names("D1", [("1", "Anode"), ("2", "Cathode")])
        .with_terminal_names("Q1", [("1", "Base"), ("2", "Emitter"), ("3", "Collector")])
        .with_terminal_names("U1", [("1", "VDD"), ("2", "GND"), ("3", "PA0")]);
    (netlist, options)
}

fn editor_bench() -> Bench {
    Bench::new(Pin::new("J1", "2"))
        .with_source(BenchSource::voltage("vs", Pin::new("J1", "1"), 10.0))
        // A probe on the IC's supply pin, by its name.
        .with_load(BenchLoad::new(
            "probe",
            LoadKind::Resistor(1e6),
            Pin::new("U1", "vdd"),
        ))
}

#[test]
fn an_editor_export_simulates_to_its_hand_checked_operating_point() {
    let (netlist, options) = editor_board();
    let imported = simulate_with(&netlist, &editor_bench(), &options).expect("simulates");
    let Simulation::OperatingPoint(r) = &imported.result else {
        panic!()
    };
    let v = |net: &str| r.node_voltages[net];
    let current = |id: &str| {
        r.component_results
            .iter()
            .find(|c| c.id == id)
            .unwrap_or_else(|| panic!("{id}"))
            .current_through
    };

    assert!((v("net-vcc") - 10.0).abs() < 1e-9);
    // The solver's gmin shunt on each node pulls it off by nanovolts.
    assert!((v("net-ref") - 5.0).abs() < 1e-6, "ref {}", v("net-ref"));
    // Gain 1e5: out = A·ref / (1 + A·β), β = 10/14.7.
    assert!((v("net-out") - 7.35).abs() < 1e-3, "out {}", v("net-out"));
    assert!((v("net-fb") - 5.0).abs() < 1e-3);

    // The LED, by its Anode/Cathode names, conducts forward.
    let vf = v("net-led");
    assert!((1.5..3.0).contains(&vf), "LED forward voltage {vf}");
    assert!((current("D1") - (v("net-out") - vf) / 470.0).abs() < 1e-9);

    // Q1 by Base/Emitter/Collector: Ie = (bf + 1)·Ib with bf = 100.
    let vbe = v("net-base") - v("net-emitter");
    assert!((0.55..0.8).contains(&vbe), "Vbe {vbe}");
    let ib = (10.0 - v("net-base")) / 100e3;
    let ie = v("net-emitter") / 1e3;
    assert!((ie - 101.0 * ib).abs() < 1e-3 * ie, "Ie {ie}, Ib {ib}");

    // The open push-button: 10 V across 1 GΩ and 10 kΩ.
    assert!(v("net-btn").abs() < 1e-3);
    // The probe sees the supply through the IC's pin.
    assert!((current("probe") - 10e-6).abs() < 1e-12);

    // The IC simulated nothing, and is listed with the nets it's on.
    assert!(r.component_results.iter().all(|c| c.id != "U1"));
    assert_eq!(
        imported.circuit.black_boxes,
        vec![BlackBoxPart {
            component: "U1".into(),
            kind: "mcu-qfn32".into(),
            terminals: BTreeMap::from([
                ("1".into(), "net-vcc".into()),
                ("2".into(), "net-gnd".into()),
            ]),
        }]
    );
}

#[test]
fn a_strict_import_refuses_the_unknown_ic() {
    let (netlist, options) = editor_board();
    let strict = options.clone().with_unknown_kinds(UnknownKinds::Refuse);
    let e = match simulate_with(&netlist, &editor_bench(), &strict) {
        Err(BenchError::Import(e)) => e,
        other => panic!("{other:?}"),
    };
    assert_eq!(
        e,
        ImportError::UnsupportedKind {
            component: "U1".into(),
            kind: "mcu-qfn32".into()
        }
    );
    assert_eq!(
        e.to_string(),
        "component \"U1\": sindr can't simulate a \"mcu-qfn32\""
    );
    // Without the name table, the polarised capacitor's pin number is
    // refused first.
    assert!(matches!(
        simulate(&netlist, &editor_bench()),
        Err(BenchError::Import(ImportError::UnknownTerminal { component, .. })) if component == "C2"
    ));
}

#[test]
fn an_explicit_black_box_is_kept_by_a_strict_import() {
    let mut n = divider("1k", "1k");
    n.components.push(
        Component::new("u1", ComponentKind::BlackBox)
            .with_value("MCU")
            .with_param("anything", 1.0)
            .with_model(Model::Diode(DiodeModel::default()))
            .with_part(Part {
                reference: Some("U1".into()),
                ..Default::default()
            }),
    );
    n.components
        .push(Component::new("u2", ComponentKind::from_name("ic")));
    n.nets[0].pins.push(Pin::new("u1", "VCC"));
    n.nets[2].pins.push(Pin::new("u1", "GND"));
    let bench =
        Bench::new("gnd").with_source(BenchSource::voltage("v", Pin::new("u1", "VCC"), 4.0));
    let imported = simulate(&n, &bench).unwrap();
    let Simulation::OperatingPoint(r) = &imported.result else {
        panic!()
    };
    assert!((r.node_voltages["out"] - 2.0).abs() < 1e-9);
    let boxes = &imported.circuit.black_boxes;
    assert_eq!(boxes.len(), 2);
    assert_eq!(boxes[0].component, "u1");
    assert_eq!(boxes[0].kind, "black_box");
    assert_eq!(boxes[0].terminals["VCC"], "in");
    assert_eq!(boxes[0].terminals["GND"], "gnd");
    // On no net, and still listed.
    assert_eq!(boxes[1].component, "u2");
    assert!(boxes[1].terminals.is_empty());
    // Its part data is kept, as for any component.
    assert_eq!(
        imported.circuit.circuit.parts["u1"].reference.as_deref(),
        Some("U1")
    );
}

#[test]
fn an_unknown_kind_on_no_net_becomes_a_black_box_on_request() {
    let mut n = divider("1k", "1k");
    n.components
        .push(Component::new("u1", ComponentKind::from_name("Custom IC")));
    let options = ImportOptions::new().with_unknown_kinds(UnknownKinds::BlackBox);
    let imported = n.to_circuit_with(&gnd_bench(), &options).unwrap();
    assert_eq!(imported.black_boxes.len(), 1);
    assert_eq!(imported.black_boxes[0].kind, "Custom IC");
    assert_eq!(
        err(&n, &gnd_bench()),
        ImportError::UnsupportedKind {
            component: "u1".into(),
            kind: "Custom IC".into()
        }
    );
}

#[test]
fn a_black_box_terminal_by_an_ambiguous_name_is_taken_as_given() {
    // Two pins named GND: the name picks neither, so the bench's "GND"
    // is a terminal id, which is on no net.
    let mut n = divider("1k", "1k");
    n.components
        .push(Component::new("u1", ComponentKind::BlackBox));
    n.nets[2].pins.push(Pin::new("u1", "4"));
    n.nets[2].pins.push(Pin::new("u1", "9"));
    let options = ImportOptions::new().with_terminal_names("u1", [("4", "GND"), ("9", "GND")]);
    let bench = Bench::new(Pin::new("u1", "GND"));
    assert_eq!(
        n.to_circuit_with(&bench, &options),
        Err(ImportError::AttachedToUnconnected {
            item: "ground".into(),
            component: "u1".into(),
            terminal: "GND".into(),
        })
    );
    // By either id it's fine.
    let bench = Bench::new(Pin::new("u1", "9")).with_source(BenchSource::voltage("v", "in", 1.0));
    assert!(n.to_circuit_with(&bench, &options).is_ok());
}

#[test]
fn a_connector_pin_answers_to_its_name() {
    let netlist = Netlist {
        components: vec![
            Component::new("J1", ComponentKind::from_name("Header-2")),
            Component::new("R1", ComponentKind::Resistor).with_value("1k"),
        ],
        nets: vec![
            Net::new("a", [("J1", "1"), ("R1", "1")]),
            Net::new("b", [("J1", "2"), ("R1", "2")]),
        ],
    };
    let options = ImportOptions::new().with_terminal_names("J1", [("1", "VIN"), ("2", "GND")]);
    let bench = Bench::new(Pin::new("J1", "gnd")).with_source(BenchSource::current(
        "i",
        Pin::new("J1", "Vin"),
        1e-3,
    ));
    let Simulation::OperatingPoint(r) = simulate_with(&netlist, &bench, &options).unwrap().result
    else {
        panic!()
    };
    assert!((r.node_voltages["a"] - 1.0).abs() < 1e-9);
}

#[test]
fn symbol_ids_and_aliases_name_kinds() {
    use ComponentKind as K;
    for (name, kind) in [
        ("resistor", K::Resistor),
        ("Resistor", K::Resistor),
        ("resistor-iec", K::Resistor),
        ("resistor_us", K::Resistor),
        ("capacitor", K::Capacitor),
        ("capacitor-polarized", K::CapacitorPolarized),
        ("Capacitor Polarised", K::CapacitorPolarized),
        ("inductor", K::Inductor),
        ("led", K::Led),
        ("diode", K::Diode),
        ("npn", K::Npn),
        ("PNP", K::Pnp),
        ("op-amp", K::OpAmp),
        ("opamp", K::OpAmp),
        ("Op Amp", K::OpAmp),
        ("push-button", K::PushButton),
        ("pushbutton", K::PushButton),
        ("switch-spst", K::Switch),
        ("test-point", K::TestPoint),
        ("connector", K::Connector),
        ("header-3", K::Connector),
        ("header-4", K::Connector),
        ("header_2x5", K::Connector),
        ("zener-diode", K::ZenerDiode),
        ("schottky diode", K::SchottkyDiode),
        ("  ferrite--bead ", K::FerriteBead),
        ("crystal", K::Crystal),
        ("black-box", K::BlackBox),
    ] {
        assert_eq!(ComponentKind::from_name(name), kind, "{name}");
    }
    // Every alias names its kind, and the kind keeps its canonical name.
    for (alias, kind) in ALIASES {
        assert_eq!(ComponentKind::from_name(alias), *kind, "{alias}");
        assert!(KINDS.iter().any(|(n, _)| *n == kind.name()), "{alias}");
    }
    assert_eq!(ComponentKind::from_name("op-amp").name(), "op_amp");
    // Not a header of any size, and unknown names keep their spelling.
    for name in ["header-x", "header-", "headers", "Custom IC"] {
        assert_eq!(
            ComponentKind::from_name(name),
            K::Other(name.into()),
            "{name}"
        );
    }
}

#[test]
fn names_compare_without_case_or_separator_style() {
    assert_eq!(normalise("  Non-Inverting "), "non_inverting");
    assert_eq!(normalise("positive  supply"), "positive_supply");
    assert_eq!(normalise("push_-_button"), "push_button");
    assert_eq!(normalise("V-"), "v_");
    assert_eq!(normalise("-"), "_");
    assert_eq!(normalise("IN+"), "in+");
}

/// Builds `component` alone as [`build`] does, reading it with `options`.
fn build_with(
    component: Component,
    terminals: &[&str],
    options: &ImportOptions,
) -> Result<CircuitElement, ImportError> {
    let id = component.id.clone();
    let mut nets: Vec<Net> = terminals
        .iter()
        .enumerate()
        .map(|(i, t)| Net::new(format!("n{i}"), [(id.as_str(), *t)]))
        .collect();
    nets.push(Net::new("gnd", Vec::<(&str, &str)>::new()));
    let netlist = Netlist {
        components: vec![component],
        nets,
    };
    let imported = netlist.to_circuit_with(&Bench::new("gnd"), options)?;
    Ok(imported
        .circuit
        .components
        .into_iter()
        .find(|e| e.id() == id || e.id().starts_with(&format!("{id}.")))
        .expect("the component's element"))
}

#[test]
fn an_op_amp_answers_to_its_long_names() {
    let e = built(
        Component::new("u", ComponentKind::OpAmp),
        &[
            "Non-inverting",
            "Inverting",
            "Output",
            "Positive supply",
            "Negative supply",
        ],
    );
    let CircuitElement::OpAmp(OpAmp { nodes, .. }) = e else {
        panic!("{e:?}")
    };
    assert_eq!(nodes, ns::<3>());
    // Any separator style.
    assert!(build(
        Component::new("u", ComponentKind::Comparator),
        &[
            "non_inverting",
            "INVERTING",
            "out",
            "positive-supply",
            "negative_supply"
        ],
    )
    .is_ok());
}

#[test]
fn terminal_names_from_the_table_place_each_terminal() {
    // Pins by id, in the editor's order: Base, Emitter, Collector.
    let options = ImportOptions::new()
        .with_terminal_names("q", [("1", "Base"), ("2", "Emitter"), ("3", "Collector")]);
    let e = build_with(
        Component::new("q", ComponentKind::Npn),
        &["1", "2", "3"],
        &options,
    )
    .unwrap();
    let CircuitElement::Bjt(Bjt { nodes, .. }) = e else {
        panic!("{e:?}")
    };
    // Element order is base, collector, emitter.
    assert_eq!(nodes, ["n0", "n2", "n1"].map(String::from));
}

#[test]
fn a_terminal_name_wins_over_its_id() {
    // Pin "1" is the potentiometer's wiper on this symbol, not its top.
    let options = ImportOptions::new()
        .with_terminal_names("p", [("1", "Wiper"), ("2", "Top"), ("3", "Bottom")]);
    let e = build_with(
        Component::new("p", ComponentKind::Potentiometer).with_value("10k"),
        &["1", "2", "3"],
        &options,
    )
    .unwrap();
    let CircuitElement::Potentiometer(Potentiometer { nodes, .. }) = e else {
        panic!("{e:?}")
    };
    assert_eq!(nodes, ["n1", "n0", "n2"].map(String::from));
}

#[test]
fn a_name_that_isnt_a_terminal_falls_back_to_the_id() {
    let options = ImportOptions::new().with_terminal_names("r", [("1", "Pin one")]);
    assert!(build_with(
        Component::new("r", ComponentKind::Resistor).with_value(1.0),
        &["1", "2"],
        &options
    )
    .is_ok());
}

#[test]
fn a_pin_number_without_a_name_still_cant_pick_a_polarity() {
    for (kind, ids) in [
        (ComponentKind::Led, ["1", "2"]),
        (ComponentKind::from_name("capacitor-polarized"), ["1", "2"]),
    ] {
        let name = kind.name().to_string();
        let e = build(Component::new("x", kind.clone()).with_value(1e-6), &ids).unwrap_err();
        assert!(
            matches!(&e, ImportError::UnknownTerminal { terminal, kind, .. }
                if terminal == "1" && *kind == name),
            "{e:?}"
        );
        // A table with names for other parts doesn't help.
        let other = ImportOptions::new().with_terminal_names("y", [("1", "Anode")]);
        assert!(build_with(Component::new("x", kind), &ids, &other).is_err());
    }
    let e = build(Component::new("x", ComponentKind::Led), &["1", "2"]).unwrap_err();
    assert_eq!(
        e.to_string(),
        "component \"x\" (led) has no terminal \"1\"; it has \"anode\", \"cathode\""
    );
    // With its names, the polarised capacitor is a capacitor.
    let options = ImportOptions::new().with_terminal_names("x", [("1", "+"), ("2", "-")]);
    let e = build_with(
        Component::new("x", ComponentKind::CapacitorPolarized).with_value("4.7uF"),
        &["2", "1"],
        &options,
    )
    .unwrap();
    assert!(
        matches!(&e, CircuitElement::Capacitor(Capacitor { nodes, capacitance, .. })
            if *nodes == ["n1", "n0"].map(String::from) && (*capacitance - 4.7e-6).abs() < 1e-18),
        "{e:?}"
    );
}

#[test]
fn an_unknown_terminal_name_is_reported_with_its_id() {
    let options = ImportOptions::new().with_terminal_names("q", [("1", "Gate")]);
    let e = build_with(Component::new("q", ComponentKind::Npn), &["1"], &options).unwrap_err();
    assert_eq!(
        e,
        ImportError::UnknownTerminal {
            component: "q".into(),
            kind: "npn".into(),
            terminal: "1 (Gate)".into(),
            expected: "\"base\", \"collector\", \"emitter\"".into(),
        }
    );
    assert_eq!(
        e.to_string(),
        "component \"q\" (npn) has no terminal \"1 (Gate)\"; it has \"base\", \"collector\", \"emitter\""
    );
}

#[test]
fn crystals_and_ferrite_beads_import() {
    let e = built(
        Component::new("y", ComponentKind::from_name("crystal")).with_value("16MHz"),
        &["1", "2"],
    );
    assert_eq!(
        e,
        CircuitElement::Crystal(Crystal {
            id: "y".into(),
            nodes: ns(),
            params: CrystalParams::from_datasheet(16e6, 50.0, 5e-12),
        })
    );
    let e = built(
        Component::new("y", ComponentKind::Crystal)
            .with_value("X322516MLB4SI")
            .with_param("frequency", "32.768k")
            .with_param("esr", "70k")
            .with_param("c0", "1.3p"),
        &["a", "b"],
    );
    assert_eq!(
        e,
        CircuitElement::Crystal(Crystal {
            id: "y".into(),
            nodes: ns(),
            params: CrystalParams::from_datasheet(32.768e3, 70e3, 1.3e-12),
        })
    );
    let e = built(
        Component::new("fb", ComponentKind::from_name("ferrite-bead")).with_value("600"),
        &["1", "2"],
    );
    assert_eq!(
        e,
        CircuitElement::FerriteBead(FerriteBead {
            id: "fb".into(),
            nodes: ns(),
            params: FerriteBeadParams::from_datasheet(600.0, 0.1),
        })
    );
    let e = built(
        Component::new("fb", ComponentKind::FerriteBead)
            .with_param("z_100mhz", 120.0)
            .with_param("r_dc", 0.05),
        &["1", "2"],
    );
    assert_eq!(
        e,
        CircuitElement::FerriteBead(FerriteBead {
            id: "fb".into(),
            nodes: ns(),
            params: FerriteBeadParams::from_datasheet(120.0, 0.05),
        })
    );
}

#[test]
fn crystals_and_ferrite_beads_say_what_value_they_need() {
    assert_eq!(
        build(
            Component::new("y", ComponentKind::Crystal).with_value("HC49"),
            &["1", "2"]
        ),
        Err(ImportError::MissingValue {
            component: "y".into(),
            what: "a frequency (Hz), as the value or \"frequency\"".into(),
        })
    );
    let e = build(
        Component::new("fb", ComponentKind::FerriteBead),
        &["1", "2"],
    )
    .unwrap_err();
    assert_eq!(
        e.to_string(),
        "component \"fb\" needs an impedance at 100 MHz (Ω), as the value or \"z_100mhz\""
    );
}

#[test]
fn a_ferrite_bead_is_its_dc_resistance_at_dc() {
    // 5 V across a bead with 100 Ω DC resistance and a 100 Ω resistor.
    let netlist = Netlist {
        components: vec![
            Component::new("fb", ComponentKind::FerriteBead)
                .with_value("600")
                .with_param("r_dc", 100.0),
            Component::new("r", ComponentKind::Resistor).with_value(100.0),
        ],
        nets: vec![
            Net::new("in", [("fb", "1")]),
            Net::new("mid", [("fb", "2"), ("r", "1")]),
            Net::new("gnd", [("r", "2")]),
        ],
    };
    let bench = Bench::new("gnd").with_source(BenchSource::voltage("v", "in", 5.0));
    assert!((op(&netlist, &bench).node_voltages["mid"] - 2.5).abs() < 1e-6);
}

#[test]
fn default_options_read_as_to_circuit_does() {
    assert_eq!(ImportOptions::new(), ImportOptions::default());
    assert_eq!(ImportOptions::new().unknown_kinds, UnknownKinds::Refuse);
    let n = divider("1k", "2k");
    assert_eq!(
        n.to_circuit_with(&gnd_bench(), &ImportOptions::new()),
        n.to_circuit(&gnd_bench())
    );
    // Names add up across calls.
    let o = ImportOptions::new()
        .with_terminal_names("q", [("1", "Base")])
        .with_terminal_names("q", [("2", "Emitter")]);
    assert_eq!(o.terminal_names["q"].len(), 2);
}

#[cfg(feature = "serde")]
#[test]
fn options_read_from_json() {
    let json = r#"{
        "unknown_kinds": "black_box",
        "terminal_names": {"q1": {"1": "Base", "2": "Emitter", "3": "Collector"}}
    }"#;
    let options: ImportOptions = serde_json::from_str(json).unwrap();
    assert_eq!(options.unknown_kinds, UnknownKinds::BlackBox);
    assert_eq!(options.terminal_names["q1"]["3"], "Collector");
    let back: ImportOptions =
        serde_json::from_str(&serde_json::to_string(&options).unwrap()).unwrap();
    assert_eq!(back, options);
    let empty: ImportOptions = serde_json::from_str("{}").unwrap();
    assert_eq!(empty, ImportOptions::new());
    assert_eq!(
        serde_json::to_string(&empty).unwrap(),
        r#"{"unknown_kinds":"refuse"}"#
    );
}

#[test]
fn multi_unit_kinds_read_from_symbol_names() {
    use ComponentKind as K;
    for (name, kind) in [
        ("dual-op-amp", K::DualOpAmp),
        ("Op Amp Dual", K::DualOpAmp),
        ("dual_opamp", K::DualOpAmp),
        ("quad op-amp", K::QuadOpAmp),
        ("opamp-quad", K::QuadOpAmp),
        ("Dual Comparator", K::DualComparator),
        ("comparator-quad", K::QuadComparator),
    ] {
        assert_eq!(ComponentKind::from_name(name), kind, "{name}");
    }
    assert_eq!(K::DualOpAmp.units(), Some((K::OpAmp, 2)));
    assert_eq!(K::QuadComparator.units(), Some((K::Comparator, 4)));
    assert_eq!(K::OpAmp.units(), None);
    assert_eq!(K::DualOpAmp.terminals().len(), 8);
    assert_eq!(K::QuadOpAmp.terminals().len(), 14);
}

/// A dual op-amp on ±12 V: unit A a follower, unit B an inverting gain
/// of −2. Unit A's pins are named in three styles, and unit B's by pin
/// number through a terminal-name table.
#[test]
fn dual_op_amp_builds_one_element_per_unit_on_shared_supplies() {
    let netlist = Netlist {
        components: vec![
            Component::new("u1", ComponentKind::from_name("dual-op-amp"))
                .with_model(Model::OpAmp(OpAmpModel::default())),
            Component::new("rin", ComponentKind::Resistor).with_value("10k"),
            Component::new("rf", ComponentKind::Resistor).with_value("20k"),
            Component::new("rl", ComponentKind::Resistor).with_value("10k"),
            // Supply bleeders, so the rails aren't nets of one source alone.
            Component::new("rp", ComponentKind::Resistor).with_value("100k"),
            Component::new("rn", ComponentKind::Resistor).with_value("100k"),
        ],
        nets: vec![
            Net::new("in", [("u1", "1IN+"), ("rin", "1")]),
            Net::new("outa", [("u1", "OUT A"), ("u1", "in1-"), ("rl", "1")]),
            Net::new("sum", [("u1", "6"), ("rin", "2"), ("rf", "1")]),
            Net::new("outb", [("u1", "7"), ("rf", "2")]),
            Net::new("vcc", [("u1", "8"), ("rp", "1")]),
            Net::new("vee", [("u1", "4"), ("rn", "1")]),
            Net::new("gnd", [("u1", "5"), ("rl", "2"), ("rp", "2"), ("rn", "2")]),
        ],
    };
    let options = ImportOptions::new().with_terminal_names(
        "u1",
        [
            ("4", "V-"),
            ("5", "+IN B"),
            ("6", "-IN B"),
            ("7", "OUT B"),
            ("8", "V+"),
        ],
    );
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("vp", "vcc", 12.0))
        .with_source(BenchSource::voltage("vn", "vee", -12.0))
        .with_source(BenchSource::voltage("vin", "in", 1.5));
    let imported = netlist.to_circuit_with(&bench, &options).unwrap();
    for id in ["u1.a", "u1.b"] {
        let Some(CircuitElement::OpAmp(OpAmp {
            v_pos,
            v_neg,
            model,
            ..
        })) = imported.circuit.components.iter().find(|c| c.id() == id)
        else {
            panic!("no op-amp {id}")
        };
        assert_eq!((*v_pos, *v_neg), (12.0, -12.0), "{id}");
        assert!(model.is_some(), "{id}");
    }
    assert!(imported.circuit.components.iter().all(|c| c.id() != "u1"));

    let r = crate::solve_circuit(&imported.circuit).unwrap();
    let v = |net: &str| r.node_voltages[imported.node(net).unwrap()];
    assert!((v("outa") - 1.5).abs() < 1e-3, "follower: {}", v("outa"));
    assert!((v("outb") + 3.0).abs() < 1e-3, "inverting: {}", v("outb"));
}

#[test]
fn a_spare_unit_adds_no_element_and_a_half_wired_one_is_refused() {
    let quad = |nets: Vec<Net>| Netlist {
        components: vec![
            Component::new("u2", ComponentKind::QuadComparator),
            Component::new("rl", ComponentKind::Resistor).with_value("10k"),
            Component::new("rp", ComponentKind::Resistor).with_value("100k"),
        ],
        nets,
    };
    // Only unit C is wired: in+ to a 2 V input, in- to ground.
    let netlist = quad(vec![
        Net::new("in", [("u2", "INC+")]),
        Net::new("out", [("u2", "3OUT"), ("rl", "1")]),
        Net::new("vcc", [("u2", "VCC"), ("rp", "1")]),
        Net::new(
            "gnd",
            [("u2", "c-"), ("u2", "V-"), ("rl", "2"), ("rp", "2")],
        ),
    ]);
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("vp", "vcc", 5.0))
        .with_source(BenchSource::voltage("vin", "in", 2.0));
    let imported = netlist.to_circuit(&bench).unwrap();
    let units: Vec<&str> = imported
        .circuit
        .components
        .iter()
        .map(|c| c.id())
        .filter(|id| id.starts_with("u2"))
        .collect();
    assert_eq!(units, ["u2.c"]);
    // 2 V above 0 V: unit C drives its grounded load to the 5 V rail.
    let r = crate::solve_circuit(&imported.circuit).unwrap();
    assert!(r.node_voltages[imported.node("out").unwrap()] > 4.9);

    // Unit A with its output wired but not its inputs.
    let half = quad(vec![
        Net::new("out", [("u2", "1OUT"), ("rl", "1")]),
        Net::new("gnd", [("rl", "2"), ("rp", "1"), ("rp", "2")]),
    ]);
    assert_eq!(
        err(&half, &Bench::new("gnd")),
        ImportError::Unconnected {
            component: "u2.a".into(),
            terminal: "in+".into(),
        }
    );
}

#[test]
fn a_unit_id_that_clashes_with_a_component_is_refused() {
    let netlist = Netlist {
        components: vec![
            Component::new("u1", ComponentKind::DualOpAmp),
            Component::new("u1.a", ComponentKind::Resistor).with_value("1k"),
        ],
        nets: vec![
            Net::new("x", [("u1", "a_in+"), ("u1", "a_in-"), ("u1", "a_out")]),
            Net::new("gnd", [("u1.a", "1"), ("u1.a", "2")]),
        ],
    };
    assert_eq!(
        err(&netlist, &Bench::new("gnd")),
        ImportError::DuplicateId { id: "u1.a".into() }
    );
}

// --- models from part data --------------------------------------------

/// A two-transistor LED flasher: an astable multivibrator with an LED and
/// 470 Ω in each collector. `specs` gives the transistors hFE and
/// Vce(sat) figures; without, they carry only their part number.
fn flasher(specs: bool, led_value: &str) -> Netlist {
    let transistor = |id: &str| {
        let q = Component::new(id, ComponentKind::Npn).with_value("PMBT3904");
        if specs {
            q.with_param("hfe_min", 100.0)
                .with_param("hfe_max", 300.0)
                .with_param(
                    "hfe_ic",
                    Quantity::new(0.01, Unit::Ampere).with_text("10 mA"),
                )
                .with_param("vce_sat_max", "0.2 V")
                .with_param("vce_sat_ic", 10e-3)
                .with_param("vce_sat_ib", 1e-3)
        } else {
            q
        }
    };
    Netlist {
        components: vec![
            Component::new("d1", ComponentKind::Led).with_value(led_value),
            Component::new("d2", ComponentKind::Led).with_value(led_value),
            Component::new("r1", ComponentKind::Resistor).with_value("470"),
            Component::new("r2", ComponentKind::Resistor).with_value("470"),
            Component::new("rb1", ComponentKind::Resistor).with_value("47k"),
            Component::new("rb2", ComponentKind::Resistor).with_value("47k"),
            Component::new("c1", ComponentKind::CapacitorPolarized).with_value("4.7 µF"),
            Component::new("c2", ComponentKind::CapacitorPolarized).with_value("3.3 µF"),
            transistor("q1"),
            transistor("q2"),
        ],
        nets: vec![
            Net::new(
                "vcc",
                [("d1", "A"), ("d2", "A"), ("rb1", "1"), ("rb2", "1")],
            ),
            Net::new("a1", [("d1", "K"), ("r1", "1")]),
            Net::new("a2", [("d2", "K"), ("r2", "1")]),
            Net::new("c1n", [("r1", "2"), ("q1", "C"), ("c1", "+")]),
            Net::new("c2n", [("r2", "2"), ("q2", "C"), ("c2", "+")]),
            Net::new("b1", [("rb1", "2"), ("q1", "B"), ("c2", "-")]),
            Net::new("b2", [("rb2", "2"), ("q2", "B"), ("c1", "-")]),
            Net::new("gnd", [("q1", "E"), ("q2", "E")]),
        ],
    }
}

fn flasher_bench() -> Bench {
    Bench::new("gnd").with_source(BenchSource::voltage("vs", "vcc", 9.0))
}

#[test]
fn flasher_takes_its_models_from_part_data() {
    let netlist = flasher(true, "Amber");
    let imported = netlist.to_circuit(&flasher_bench()).unwrap();
    assert_eq!(imported.warnings, vec![], "no part fell back to a default");
    for led in ["d1", "d2"] {
        assert_eq!(imported.model_source(led), Some(ModelSource::Value));
    }
    for q in ["q1", "q2"] {
        assert_eq!(imported.model_source(q), Some(ModelSource::Datasheet));
    }
    assert_eq!(imported.model_source("r1"), Some(ModelSource::Value));
    assert_eq!(imported.defaulted().count(), 0);

    let element = |id: &str| {
        imported
            .circuit
            .components
            .iter()
            .find(|e| e.id() == id)
            .unwrap()
    };
    assert!(matches!(element("d1"), CircuitElement::Led(Led { color, .. }) if color == "amber"));
    let CircuitElement::Bjt(Bjt { bf, model, .. }) = element("q1") else {
        panic!()
    };
    // hFE 100–300 is 200 nominal; BR is fitted to Vce(sat).
    assert_eq!(*bf, 200.0);
    assert!(model.is_some_and(|m| m.br != BjtModel::default().br));

    // At its operating point both halves conduct, each LED near
    // (9 V − 2.05 V − Vce(sat)) / 470 Ω.
    let r = op(&netlist, &flasher_bench());
    for led in ["d1", "d2"] {
        let d = r.component_results.iter().find(|c| c.id == led).unwrap();
        assert!(
            (2.0..2.1).contains(&d.voltage_across),
            "{led}: {}",
            d.voltage_across
        );
        assert!(
            (13e-3..15e-3).contains(&d.current_through),
            "{led}: {}",
            d.current_through
        );
    }

    // From rest it flashes, through every edge: the period is near
    // ln 2 · 47 kΩ · (4.7 µF + 3.3 µF) = 261 ms, about 13 % short of it:
    // the amber LEDs keep the off collectors well below the rail, and the
    // edges start below Vbe = 0.6 V (see `tests/multivibrator.rs`).
    let bench = flasher_bench().with_analysis(Analysis::Transient(TransientConfig::new(1.2, 1e-4)));
    let Simulation::Transient(r) = simulate(&netlist, &bench).expect("simulates").result else {
        panic!("not a transient")
    };
    let steps = &r.transient.as_ref().unwrap().timesteps;
    let rises: Vec<f64> = steps
        .windows(2)
        .filter(|w| w[0].node_voltages["c2n"] < 4.0 && w[1].node_voltages["c2n"] >= 4.0)
        .map(|w| w[1].time)
        .collect();
    assert!(rises.len() >= 4, "{rises:?}");
    let period = (rises[rises.len() - 1] - rises[1]) / (rises.len() - 2) as f64;
    let closed_form = std::f64::consts::LN_2 * 47e3 * (4.7e-6 + 3.3e-6);
    eprintln!("PERIOD {period} {}", period / closed_form - 1.0);
    assert!(
        (0.8..0.95).contains(&(period / closed_form)),
        "period {period} s"
    );
}

#[test]
fn flasher_without_specs_warns_for_each_defaulted_part() {
    let imported = flasher(false, "LED").to_circuit(&flasher_bench()).unwrap();
    let warned: Vec<&str> = imported
        .warnings
        .iter()
        .map(|w| w.component.as_str())
        .collect();
    assert_eq!(warned, ["d1", "d2", "q1", "q2"]);
    assert_eq!(
        imported.defaulted().collect::<Vec<_>>(),
        ["d1", "d2", "q1", "q2"]
    );
    let q1 = &imported.warnings[2];
    assert!(q1.message.contains("PMBT3904"), "{q1}");
    assert!(q1.message.contains("bf = 100"), "{q1}");
    // Still a circuit: warnings don't stop the import.
    assert!(simulate(&flasher(false, "LED"), &flasher_bench()).is_ok());
}

#[test]
fn led_colour_from_any_case_and_from_the_value() {
    let led = |c: Component| {
        let id = c.id.clone();
        let netlist = Netlist {
            components: vec![c],
            nets: vec![
                Net::new("a", [(id.as_str(), "A")]),
                Net::new("gnd", [(id.as_str(), "K")]),
            ],
        };
        let imported = netlist.to_circuit(&Bench::new("gnd")).unwrap();
        let color = match &imported.circuit.components[0] {
            CircuitElement::Led(Led { color, .. }) => color.clone(),
            other => panic!("{other:?}"),
        };
        (
            color,
            imported.model_source(&id).unwrap(),
            imported.warnings,
        )
    };
    let (c, s, w) = led(Component::new("d", ComponentKind::Led).with_param("color", "Amber"));
    assert_eq!((c.as_str(), s, w.len()), ("amber", ModelSource::Params, 0));
    let (c, s, _) = led(Component::new("d", ComponentKind::Led).with_value("LED red 3mm"));
    assert_eq!((c.as_str(), s), ("red", ModelSource::Value));
    let (c, s, _) = led(Component::new("d", ComponentKind::Led).with_value("ORANGE"));
    assert_eq!((c.as_str(), s), ("orange", ModelSource::Value));
    let (c, s, w) = led(Component::new("d", ComponentKind::Led).with_param("color", "ultraviolet"));
    assert_eq!((c.as_str(), s), ("red", ModelSource::Default));
    assert!(w[0].message.contains("ultraviolet"));
}

#[test]
fn led_through_its_forward_voltage_is_a_fitted_diode() {
    // 2.05 V at 20 mA: through 350 Ω from 9 V, about 19.9 mA.
    let netlist = Netlist {
        components: vec![
            Component::new("d", ComponentKind::Led)
                .with_value("Amber")
                .with_param("vf", Quantity::new(2.05, Unit::Volt))
                .with_param("if", "20 mA"),
            Component::new("r", ComponentKind::Resistor).with_value(350.0),
        ],
        nets: vec![
            Net::new("vcc", [("r", "1")]),
            Net::new("a", [("r", "2"), ("d", "A")]),
            Net::new("gnd", [("d", "K")]),
        ],
    };
    let bench = Bench::new("gnd").with_source(BenchSource::voltage("v", "vcc", 9.0));
    let imported = simulate(&netlist, &bench).unwrap();
    assert_eq!(
        imported.circuit.model_source("d"),
        Some(ModelSource::Datasheet)
    );
    let Simulation::OperatingPoint(r) = imported.result else {
        panic!()
    };
    let v = r.node_voltages["a"];
    let i = (9.0 - v) / 350.0;
    // Near 20 mA the fitted LED sits within a few mV of its 2.05 V.
    assert!((v - 2.05).abs() < 5e-3, "Vf {v} at {i} A");
}

#[test]
fn zener_and_mosfet_from_datasheet_figures() {
    let netlist = Netlist {
        components: vec![
            Component::new("z", ComponentKind::ZenerDiode)
                .with_value("BZX84C5V1")
                .with_param("vz", 5.1)
                .with_param("iz", 5e-3),
            Component::new("m", ComponentKind::Nmos)
                .with_value("2N7002")
                .with_param("vth", 2.1)
                .with_param("vth_id", 250e-6)
                .with_param("rds_on_max", Quantity::new(5.0, Unit::Ohm))
                .with_param("rds_on_vgs", 10.0),
        ],
        nets: vec![
            Net::new("k", [("z", "K"), ("m", "G"), ("m", "D")]),
            Net::new("gnd", [("z", "A"), ("m", "S")]),
        ],
    };
    let imported = netlist.to_circuit(&Bench::new("gnd")).unwrap();
    assert!(imported.warnings.is_empty(), "{:?}", imported.warnings);
    assert_eq!(imported.model_source("z"), Some(ModelSource::Datasheet));
    assert_eq!(imported.model_source("m"), Some(ModelSource::Datasheet));
    let CircuitElement::Mosfet(Mosfet { params, .. }) = &imported.circuit.components[1] else {
        panic!()
    };
    assert!(params.vto > 0.0 && params.vto < 2.1, "{params:?}");

    // Half a figure is an error naming what's missing.
    let mut half = netlist.clone();
    half.components[1].params.remove("vth_id");
    assert!(matches!(
        half.to_circuit(&Bench::new("gnd")),
        Err(ImportError::MissingValue { component, .. }) if component == "m"
    ));
}

#[test]
fn quantities_carry_their_unit() {
    let netlist = |temperature: Value| Netlist {
        components: vec![Component::new("d", ComponentKind::Diode)
            .with_model(Model::Diode(DiodeModel::default()))
            .with_param("temperature", temperature)],
        nets: vec![Net::new("a", [("d", "A")]), Net::new("gnd", [("d", "K")])],
    };
    let kelvin = |v: Value| match &netlist(v).to_circuit(&Bench::new("gnd")) {
        Ok(imported) => match &imported.circuit.components[0] {
            CircuitElement::Diode(Diode { temperature, .. }) => Ok(*temperature),
            _ => unreachable!(),
        },
        Err(e) => Err(e.clone()),
    };
    assert_eq!(kelvin(Quantity::new(350.0, Unit::Kelvin).into()), Ok(350.0));
    assert_eq!(kelvin("300 K".into()), Ok(300.0));
    assert!(matches!(
        kelvin(Quantity::new(350.0, Unit::Volt).into()),
        Err(ImportError::BadValue { field, .. }) if field == "temperature"
    ));
    assert_eq!(Unit::from_symbol("Ω"), Unit::Ohm);
    assert_eq!(Unit::from_symbol("1"), Unit::Ratio);
    assert_eq!(Unit::from_symbol("k"), Unit::Other("k".into()));
}

#[cfg(feature = "serde")]
#[test]
fn quantities_in_json() {
    let c: Component = serde_json::from_str(
        r#"{"id": "q1", "kind": "npn", "value": "PMBT3904",
            "params": {"hfe": {"value": 200, "unit": "1", "text": "200"}}}"#,
    )
    .unwrap();
    assert_eq!(
        c.params["hfe"],
        Value::from(Quantity::new(200.0, Unit::Ratio).with_text("200"))
    );
    let back: Component = serde_json::from_str(&serde_json::to_string(&c).unwrap()).unwrap();
    assert_eq!(back, c);
    let source: ModelSource = serde_json::from_str(r#""datasheet""#).unwrap();
    assert_eq!(source, ModelSource::Datasheet);
}

// --- current-sink and timed-switch loads ------------------------------

/// An I²C-style bus on 3.3 V: two 4.7 kΩ pull-ups on `sda` (one per
/// device), one on `scl`, and an LED behind 220 Ω.
fn i2c_board() -> Netlist {
    Netlist {
        components: vec![
            Component::new("rp1", ComponentKind::Resistor).with_value("4k7"),
            Component::new("rp2", ComponentKind::Resistor).with_value("4k7"),
            Component::new("rp3", ComponentKind::Resistor).with_value("4k7"),
            Component::new("rled", ComponentKind::Resistor).with_value("220"),
            Component::new("d1", ComponentKind::Led).with_param("color", "green"),
        ],
        nets: vec![
            Net::new(
                "vdd",
                [("rp1", "1"), ("rp2", "1"), ("rp3", "1"), ("rled", "1")],
            ),
            Net::new("sda", [("rp1", "2"), ("rp2", "2")]),
            Net::new("scl", [("rp3", "2")]),
            Net::new("led_a", [("rled", "2"), ("d1", "anode")]),
            Net::new("gnd", [("d1", "cathode")]),
        ],
    }
}

const VDD: f64 = 3.3;
const RP: f64 = 4.7e3;
const C_BUS: f64 = 200e-12;
const R_ON: f64 = 25.0;
const SCL_LEAK: f64 = 1e6;
const I_SUPPLY: f64 = 5e-3;
const RELEASE: f64 = 1e-6;
const CLOSE: f64 = 4e-6;

/// The board on its bench: the supply, the IC's supply current, the bus
/// capacitance, `sda` held low until [`RELEASE`] and `scl` leaking to
/// ground through 1 MΩ until it is pulled low at [`CLOSE`].
fn i2c_bench(analysis: Analysis) -> Bench {
    Bench::new("gnd")
        .with_source(BenchSource::voltage("v1", "vdd", VDD))
        .with_load(BenchLoad::new(
            "u1",
            LoadKind::current_sink(I_SUPPLY),
            "vdd",
        ))
        .with_load(BenchLoad::new("c_sda", LoadKind::Capacitor(C_BUS), "sda"))
        .with_load(BenchLoad::new("c_scl", LoadKind::Capacitor(C_BUS), "scl"))
        .with_load(BenchLoad::new(
            "sda_drv",
            LoadKind::released_at(RELEASE, R_ON),
            "sda",
        ))
        .with_load(BenchLoad::new(
            "scl_drv",
            LoadKind::closed_at(CLOSE, R_ON).with_off_resistance(SCL_LEAK),
            "scl",
        ))
        .with_analysis(analysis)
}

fn current(r: &crate::SimulationResult, id: &str) -> f64 {
    r.component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("no result for {id}"))
        .current_through
}

/// The green LED's current behind `r` from `vdd`, solving
/// `I = (vdd − Vf(I)) / r` with `Vf(I) = 2.2 V + n·VT·ln(I / 20 mA)`, the
/// Shockley curve its model is fitted to (n = 2, 2.2 V at 20 mA).
fn led_current_by_hand(vdd: f64, r: f64) -> f64 {
    let vt = 1.380649e-23 * 300.15 / 1.602176634e-19;
    let mut i: f64 = 5e-3;
    for _ in 0..50 {
        i = (vdd - (2.2 + 2.0 * vt * (i / 20e-3).ln())) / r;
    }
    i
}

#[test]
fn i2c_bus_levels_sink_and_led_at_the_operating_point() {
    let r = op(&i2c_board(), &i2c_bench(Analysis::OperatingPoint));
    let v = |n: &str| r.node_voltages[n];

    // sda held low: R_ON under the two pull-ups in parallel.
    let rp_par = RP / 2.0;
    let sda_low = VDD * R_ON / (R_ON + rp_par);
    assert!((v("sda") - sda_low).abs() < 1e-6 * VDD, "sda {}", v("sda"));
    // scl released, a 1 MΩ leak under its pull-up.
    let scl_high = VDD * SCL_LEAK / (SCL_LEAK + RP);
    assert!((v("scl") - scl_high).abs() < 1e-6 * VDD, "scl {}", v("scl"));

    // The sink draws exactly its current, whatever the rail.
    assert!((current(&r, "u1").abs() - I_SUPPLY).abs() < 1e-12);

    // The LED, against the hand-solved Shockley curve: about 5.3 mA.
    let i_led = (VDD - v("led_a")) / 220.0;
    let by_hand = led_current_by_hand(VDD, 220.0);
    assert!((i_led / by_hand - 1.0).abs() < 1e-6, "{i_led} vs {by_hand}");
    assert!((current(&r, "d1").abs() - i_led).abs() < 1e-9);

    // The supply carries all of it.
    let budget = I_SUPPLY + i_led + (VDD - sda_low) / rp_par + (VDD - scl_high) / RP;
    assert!(
        (current(&r, "v1").abs() - budget).abs() < 1e-6 * budget,
        "{} vs {budget}",
        current(&r, "v1")
    );
}

/// The first time `v` crosses `level` rising after `from`, interpolated
/// between steps.
fn rising_crossing(r: &crate::SimulationResult, node: &str, level: f64, from: f64) -> f64 {
    let steps = &r.transient.as_ref().expect("transient").timesteps;
    steps
        .windows(2)
        .filter(|w| w[0].time >= from)
        .find_map(|w| {
            let (v0, v1) = (w[0].node_voltages[node], w[1].node_voltages[node]);
            (v0 < level && v1 >= level)
                .then(|| w[0].time + (level - v0) / (v1 - v0) * (w[1].time - w[0].time))
        })
        .unwrap_or_else(|| panic!("{node} never rises through {level}"))
}

#[test]
fn i2c_rise_time_after_the_switch_releases() {
    let config = TransientConfig::new(6e-6, 5e-9)
        .with_start(crate::transient::TransientStart::OperatingPoint);
    let netlist = i2c_board();
    let bench = i2c_bench(Analysis::Transient(config));
    let Simulation::Transient(r) = simulate(&netlist, &bench).unwrap().result else {
        unreachable!()
    };
    let steps = &r.transient.as_ref().unwrap().timesteps;
    let at = |t: f64, node: &str| {
        steps
            .iter()
            .find(|s| (s.time - t).abs() < 1e-15)
            .unwrap_or_else(|| panic!("no step at {t}"))
            .node_voltages[node]
    };

    // A step lands on each switching time, and still sees the old state.
    let rp_par = RP / 2.0;
    let sda_low = VDD * R_ON / (R_ON + rp_par);
    assert!((at(RELEASE, "sda") - sda_low).abs() < 1e-6 * VDD);
    let scl_high = VDD * SCL_LEAK / (SCL_LEAK + RP);
    assert!((at(CLOSE, "scl") - scl_high).abs() < 1e-3 * VDD);

    // Released, sda charges through the pull-ups in parallel: 30 % → 70 %
    // of the rail in ln(7/3)·R·C = 0.8473·R·C, 10 % → 90 % in ln 9·R·C =
    // 2.197·R·C, here 398 ns and 1.03 µs, both within 0.5 %. The 1 GΩ leak
    // moves the final level by under 1 ppm.
    let tau = rp_par * C_BUS;
    let t30 = rising_crossing(&r, "sda", 0.3 * VDD, RELEASE);
    let t70 = rising_crossing(&r, "sda", 0.7 * VDD, RELEASE);
    let t10 = rising_crossing(&r, "sda", 0.1 * VDD, RELEASE);
    let t90 = rising_crossing(&r, "sda", 0.9 * VDD, RELEASE);
    let rise_30_70 = t70 - t30;
    let rise_10_90 = t90 - t10;
    assert!(
        (rise_30_70 / ((7.0f64 / 3.0).ln() * tau) - 1.0).abs() < 0.005,
        "30-70 % {rise_30_70:e} s vs {:e} s",
        0.8473 * tau
    );
    assert!(
        (rise_10_90 / (9.0f64.ln() * tau) - 1.0).abs() < 0.005,
        "10-90 % {rise_10_90:e} s vs {:e} s",
        2.197 * tau
    );
    // The edge starts at the release, not before: from the held-low level
    // to 10 % takes R·C·ln((Vdd − V_low) / (0.9·Vdd)).
    let to_10 = tau * ((VDD - sda_low) / (0.9 * VDD)).ln();
    assert!(((t10 - RELEASE) / to_10 - 1.0).abs() < 0.005);

    // scl, pulled low at CLOSE through R_ON (τ = 5 ns), has settled by the
    // end; the IC's supply current is the same at every step.
    let scl_low = VDD * R_ON / (R_ON + RP);
    assert!((r.node_voltages["scl"] - scl_low).abs() < 1e-4);
    for s in steps {
        let sink = s.component_results.iter().find(|c| c.id == "u1").unwrap();
        assert!((sink.current_through.abs() - I_SUPPLY).abs() < 1e-12);
    }
}

#[test]
fn timed_switch_rejects_bad_values() {
    let bench = |kind| {
        Bench::new("gnd")
            .with_source(BenchSource::voltage("v", "in", 1.0))
            .with_load(BenchLoad::new("sw", kind, "out"))
    };
    for (kind, field) in [
        (LoadKind::released_at(1e-6, 0.0), "on_resistance"),
        (
            LoadKind::released_at(1e-6, 10.0).with_off_resistance(f64::NAN),
            "off_resistance",
        ),
        (LoadKind::closed_at(f64::INFINITY, 10.0), "at"),
    ] {
        match err(&divider("1k", "1k"), &bench(kind)) {
            ImportError::BadValue {
                component,
                field: f,
                ..
            } => assert_eq!((component.as_str(), f.as_str()), ("sw", field)),
            other => panic!("{other:?}"),
        }
    }
}

#[cfg(feature = "serde")]
#[test]
fn sink_and_timed_switch_loads_in_json() {
    let json = r#"[
        {"id": "u1", "kind": {"current_sink": {"amps": 0.005}}, "a": {"net": "vdd"}},
        {"id": "drv", "kind": {"timed_switch": {"on_resistance": 25, "at": 1e-6}},
         "a": {"net": "sda"}},
        {"id": "short", "kind": {"timed_switch": {"on_resistance": 0.1,
         "off_resistance": 1e6, "at": 2e-6, "starts_closed": false}},
         "a": {"net": "a"}, "b": {"net": "b"}}
    ]"#;
    let loads: Vec<BenchLoad> = serde_json::from_str(json).unwrap();
    assert_eq!(loads[0].kind, LoadKind::current_sink(5e-3));
    assert_eq!(loads[1].kind, LoadKind::released_at(1e-6, 25.0));
    assert_eq!(
        loads[2].kind,
        LoadKind::closed_at(2e-6, 0.1).with_off_resistance(1e6)
    );
    let back: Vec<BenchLoad> =
        serde_json::from_str(&serde_json::to_string(&loads).unwrap()).unwrap();
    assert_eq!(back, loads);
}

// --- part-data edge cases ---------------------------------------------

/// `component` alone, each terminal on its own net and the last on `gnd`.
/// A MOSFET's body is left off, so it sits on the source.
fn wired(component: Component) -> Netlist {
    let terminals: Vec<_> = component
        .kind
        .terminals()
        .iter()
        .filter(|names| names[0] != "body")
        .collect();
    let nets = terminals
        .iter()
        .enumerate()
        .map(|(i, names)| {
            let net = if i + 1 == terminals.len() {
                "gnd".to_string()
            } else {
                format!("n{i}")
            };
            Net::new(net, [(component.id.as_str(), names[0])])
        })
        .collect();
    Netlist {
        components: vec![component],
        nets,
    }
}

fn import(component: Component) -> Result<ImportedCircuit, ImportError> {
    wired(component).to_circuit(&Bench::new("gnd"))
}

#[test]
fn values_and_parameters_check_their_quantitys_unit() {
    use ComponentKind as K;
    let q = |v: f64, unit: Unit| Value::from(Quantity::new(v, unit));
    // Each kind with the field it reads, a usable number and its unit.
    let cases: Vec<(Component, &str, f64, Unit)> = vec![
        (Component::new("r", K::Resistor), "value", 1e3, Unit::Ohm),
        (
            Component::new("c", K::Capacitor),
            "value",
            1e-6,
            Unit::Farad,
        ),
        (Component::new("l", K::Inductor), "value", 1e-3, Unit::Henry),
        (Component::new("b", K::Battery), "value", 3.7, Unit::Volt),
        (Component::new("f", K::Fuse), "value", 0.5, Unit::Ampere),
        (Component::new("y", K::Crystal), "value", 8e6, Unit::Hertz),
        (
            Component::new("y", K::Crystal),
            "frequency",
            8e6,
            Unit::Hertz,
        ),
        (
            Component::new("y", K::Crystal).with_value(8e6),
            "c0",
            5e-12,
            Unit::Farad,
        ),
        (Component::new("q", K::Npn), "bf", 150.0, Unit::Ratio),
        (
            Component::new("k", K::Relay).with_param("pickup_voltage", 5.0),
            "inductance",
            0.1,
            Unit::Henry,
        ),
        (
            Component::new("t", K::Transformer).with_param("l2", 1e-3),
            "l1",
            1e-3,
            Unit::Henry,
        ),
    ];
    for (component, field, v, unit) in cases {
        let with = |value: Value| {
            let c = component.clone();
            if field == "value" {
                c.with_value(value)
            } else {
                c.with_param(field, value)
            }
        };
        let id = component.id.clone();
        let ok = import(with(q(v, unit.clone())));
        assert!(ok.is_ok(), "{id} {field}: {ok:?}");
        // A unit sindr doesn't know is taken as given.
        assert!(
            import(with(q(v, Unit::Other("x".into())))).is_ok(),
            "{id} {field}"
        );
        match import(with(q(v, Unit::Second))) {
            Err(ImportError::BadValue {
                component,
                field: f,
                expected,
                ..
            }) => {
                assert_eq!((component.as_str(), f.as_str()), (id.as_str(), field));
                assert_eq!(expected, format!("a quantity in {}", unit.symbol()));
            }
            other => panic!("{id} {field}: {other:?}"),
        }
    }
    // Some parameters aren't checked: an irradiance, a capacity in Ah.
    let pd = Component::new("pd", K::Photodiode).with_param("irradiance", q(10.0, Unit::Second));
    assert!(import(pd).is_ok());
    let b = Component::new("b", K::Battery)
        .with_value(3.7)
        .with_param("capacity", q(2.0, Unit::Second));
    assert!(import(b).is_ok());
}

#[test]
fn unit_symbols_round_trip() {
    for unit in [
        Unit::Ohm,
        Unit::Farad,
        Unit::Henry,
        Unit::Volt,
        Unit::Ampere,
        Unit::Watt,
        Unit::Hertz,
        Unit::Second,
        Unit::Metre,
        Unit::Kelvin,
        Unit::Ratio,
        Unit::Other("Ah".into()),
    ] {
        assert_eq!(Unit::from_symbol(unit.symbol()), unit, "{unit:?}");
    }
    assert_eq!(Unit::from_symbol(" hz "), Unit::Hertz);
    assert_eq!(Unit::from_symbol("ohms"), Unit::Ohm);
}

#[test]
fn a_bare_diode_warns_it_got_the_default_model() {
    let imported = import(Component::new("d", ComponentKind::Diode)).unwrap();
    assert_eq!(imported.model_source("d"), Some(ModelSource::Default));
    let [warning] = imported.warnings.as_slice() else {
        panic!("{:?}", imported.warnings)
    };
    assert_eq!(
        warning.to_string(),
        "component \"d\": no model data; using the built-in silicon diode (IS = 1e-14 A, N = 1)"
    );
    let CircuitElement::Diode(Diode { model, .. }) = &imported.circuit.components[0] else {
        panic!()
    };
    assert_eq!(*model, None);
}

#[test]
fn a_bjt_model_without_a_gain_gets_bf_100() {
    let q = Component::new("q", ComponentKind::Npn).with_model(Model::Bjt(BjtModel::default()));
    let imported = import(q).unwrap();
    assert_eq!(imported.model_source("q"), Some(ModelSource::Model));
    assert_eq!(imported.warnings.len(), 1);
    assert!(imported.warnings[0].message.contains("bf = 100"));
    let CircuitElement::Bjt(Bjt { bf, model, .. }) = &imported.circuit.components[0] else {
        panic!()
    };
    assert_eq!((*bf, *model), (100.0, Some(BjtModel::default())));
}

#[test]
fn figures_take_min_and_max_forms_only_where_they_are_figures() {
    // vf from its minimum and maximum: the fit uses the middle, 0.7 V.
    let ranged = Component::new("d", ComponentKind::Diode)
        .with_param("vf_min", 0.6)
        .with_param("vf_max", 0.8)
        .with_param("if", 10e-3);
    let typical = Component::new("d", ComponentKind::Diode)
        .with_param("vf", 0.7)
        .with_param("if", 10e-3);
    let model = |c: Component| {
        let imported = import(c).unwrap();
        assert_eq!(imported.model_source("d"), Some(ModelSource::Datasheet));
        match &imported.circuit.components[0] {
            CircuitElement::Diode(Diode { model, .. }) => model.unwrap(),
            other => panic!("{other:?}"),
        }
    };
    assert_eq!(model(ranged), model(typical));

    let z = Component::new("z", ComponentKind::ZenerDiode)
        .with_param("vz_max", 5.1)
        .with_param("iz", 5e-3);
    assert_eq!(
        import(z).unwrap().model_source("z"),
        Some(ModelSource::Datasheet)
    );

    // An op-amp's supply isn't a figure, so it has no `_max` form.
    let u = Component::new("u", ComponentKind::OpAmp).with_param("v_pos_max", 15.0);
    assert_eq!(
        import(u).unwrap_err(),
        ImportError::UnknownParameter {
            component: "u".into(),
            kind: "op_amp".into(),
            parameter: "v_pos_max".into(),
        }
    );
}

#[test]
fn figures_that_cant_be_fitted_name_the_parameter() {
    let missing = |c: Component| match import(c) {
        Err(ImportError::MissingValue { component, what }) => (component, what),
        other => panic!("expected MissingValue, got {other:?}"),
    };
    let bad = |c: Component| match import(c) {
        Err(ImportError::BadValue {
            component,
            field,
            given,
            ..
        }) => (component, field, given),
        other => panic!("expected BadValue, got {other:?}"),
    };

    // vf without the current it's given at.
    let d = Component::new("d", ComponentKind::Diode).with_param("vf", 0.7);
    let (c, what) = missing(d);
    assert_eq!(c, "d");
    assert!(what.contains("parameter \"if\""), "{what}");
    // A second point without its current.
    let d = Component::new("d", ComponentKind::Diode)
        .with_param("vf", 0.7)
        .with_param("if", 10e-3)
        .with_param("vf2", 0.8);
    assert_eq!(missing(d), ("d".into(), "datasheet figure \"if2\"".into()));
    // An on-resistance without the threshold it's fitted with.
    let m = Component::new("m", ComponentKind::Nmos)
        .with_param("rds_on", 1.0)
        .with_param("rds_on_vgs", 10.0);
    assert_eq!(missing(m), ("m".into(), "datasheet figure \"vth\"".into()));

    // A Zener's test current that isn't positive.
    let z = Component::new("z", ComponentKind::ZenerDiode)
        .with_param("vz", 5.1)
        .with_param("iz", -5e-3);
    assert_eq!(bad(z), ("z".into(), "iz".into(), Value::Number(-5e-3)));
    // An on-resistance given at a gate voltage below the threshold.
    let m = Component::new("m", ComponentKind::Nmos)
        .with_param("vth", 2.0)
        .with_param("vth_id", 250e-6)
        .with_param("rds_on", 1.0)
        .with_param("rds_on_vgs", 1.5);
    assert_eq!(
        bad(m),
        ("m".into(), "rds_on_vgs".into(), Value::Number(1.5))
    );
}

// --- nets left floating by unsimulated parts ---------------------------

/// A microcontroller and a sensor as black boxes, as a board with ICs
/// imports: an open-drain alert line and two I²C lines, each with only
/// its pull-up as an element; an LED driven from a pin the bench drives;
/// and a programming line that reaches only the MCU and a connector.
fn board_with_ics() -> Netlist {
    let bb = |id: &str| Component::new(id, ComponentKind::BlackBox);
    let r = |id: &str, v: &str| Component::new(id, ComponentKind::Resistor).with_value(v);
    Netlist {
        components: vec![
            bb("mcu"),
            bb("sensor"),
            Component::new("j1", ComponentKind::Connector),
            r("r_alert", "10k"),
            r("r_sda", "4k7"),
            r("r_scl", "4k7"),
            r("r_led", "1k"),
            Component::new("d_led", ComponentKind::Led).with_param("color", "green"),
        ],
        nets: vec![
            Net::new(
                "vdd",
                [
                    ("mcu", "vdd"),
                    ("sensor", "vdd"),
                    ("r_alert", "1"),
                    ("r_sda", "1"),
                    ("r_scl", "1"),
                ],
            ),
            Net::new("gnd", [("mcu", "gnd"), ("sensor", "gnd"), ("j1", "2")]),
            Net::new(
                "alert",
                [("mcu", "pa3"), ("sensor", "alert"), ("r_alert", "2")],
            ),
            Net::new("sda", [("mcu", "pa1"), ("sensor", "sda"), ("r_sda", "2")]),
            Net::new("scl", [("mcu", "pa2"), ("sensor", "scl"), ("r_scl", "2")]),
            Net::new("led_pin", [("mcu", "pa6"), ("r_led", "1")]),
            Net::new("led_k", [("r_led", "2"), ("d_led", "cathode")]),
            Net::new("led_a", [("d_led", "anode")]),
            Net::new("updi", [("mcu", "pa0"), ("j1", "1")]),
        ],
    }
}

#[test]
fn nets_that_only_reach_ic_pins_are_tied_high_impedance() {
    let netlist = board_with_ics();
    let bench = Bench::new("gnd")
        .with_source(BenchSource::voltage("vin", "vdd", 3.3))
        .with_source(BenchSource::voltage("pin_low", "led_pin", 0.0))
        .with_tie("led_a", "vdd");
    let imported = netlist.to_circuit(&bench).unwrap();

    let tied: Vec<&str> = imported
        .high_impedance
        .iter()
        .map(|h| h.node.as_str())
        .collect();
    assert_eq!(tied, ["alert", "scl", "sda"]);
    let alert = &imported.high_impedance[0];
    assert_eq!(alert.resistor, "alert.hi_z");
    assert_eq!(
        alert.pins,
        [Pin::new("mcu", "pa3"), Pin::new("sensor", "alert")]
    );
    // The tie is in the circuit as a resistor to ground.
    let tie = imported
        .circuit
        .components
        .iter()
        .find(|e| e.id() == "alert.hi_z")
        .unwrap();
    assert!(matches!(
        tie,
        CircuitElement::Resistor(Resistor { nodes, resistance, .. })
            if nodes == &["alert".to_string(), "gnd".to_string()] && *resistance == HIGH_IMPEDANCE_OHMS
    ));

    let r = op(&netlist, &bench);
    // The pull-ups hold their lines at the rail: 10 kΩ into 1 TΩ is 33 pV.
    for net in ["alert", "sda", "scl"] {
        let v = r.voltage(net).unwrap();
        assert!((v - 3.3).abs() < 1e-6, "{net}: {v}");
    }
    // The LED lights from the pin driven low: (3.3 − Vf) / 1 kΩ.
    let i = r.current("d_led").unwrap();
    assert!((1.0e-3..1.4e-3).contains(&i), "{i}");
}

#[test]
fn a_net_with_two_elements_or_none_is_left_alone() {
    let imported = board_with_ics()
        .to_circuit(
            &Bench::new("gnd")
                .with_source(BenchSource::voltage("vin", "vdd", 3.3))
                .with_load(BenchLoad::new("probe", LoadKind::Resistor(1e6), "alert")),
        )
        .unwrap();
    let tied: Vec<&str> = imported
        .high_impedance
        .iter()
        .map(|h| h.node.as_str())
        .collect();
    // `alert` has the probe as a second element, `updi` has none, and
    // `vdd` and `gnd` have many: only the I²C lines and the undriven LED
    // pin are tied.
    assert_eq!(tied, ["led_pin", "scl", "sda"]);
}

#[test]
fn a_resistor_between_two_ic_pins_is_held_to_ground() {
    // No path to ground but through the MCU: both ends tied, and it solves
    // at 0 V instead of failing as a disconnected island.
    let netlist = Netlist {
        components: vec![
            Component::new("mcu", ComponentKind::BlackBox),
            Component::new("r1", ComponentKind::Resistor).with_value("1k"),
            Component::new("r2", ComponentKind::Resistor).with_value("1k"),
        ],
        nets: vec![
            Net::new("a", [("mcu", "1"), ("r1", "1")]),
            Net::new("mid", [("r1", "2"), ("r2", "1")]),
            Net::new("b", [("mcu", "2"), ("r2", "2")]),
            Net::new("gnd", [("mcu", "3")]),
        ],
    };
    let bench = Bench::new("gnd");
    let tied: Vec<String> = netlist
        .to_circuit(&bench)
        .unwrap()
        .high_impedance
        .into_iter()
        .map(|h| h.node)
        .collect();
    assert_eq!(tied, ["a", "b"]);
    let r = op(&netlist, &bench);
    assert_eq!(r.voltage("mid"), Some(0.0));
}

#[test]
fn a_tie_never_takes_an_id_already_in_use() {
    let mut netlist = board_with_ics();
    netlist
        .components
        .push(Component::new("alert.hi_z", ComponentKind::TestPoint));
    netlist.nets.push(Net::new("tp", [("alert.hi_z", "1")]));
    let imported = netlist
        .to_circuit(&Bench::new("gnd").with_source(BenchSource::voltage("vin", "vdd", 3.3)))
        .unwrap();
    let alert = imported
        .high_impedance
        .iter()
        .find(|h| h.node == "alert")
        .unwrap();
    assert_eq!(alert.resistor, "alert.hi_z2");
}

#[test]
fn a_net_named_0v_is_a_ground_not_a_supply() {
    // Before, `0V` read as a 0 V supply, which on the bench's own ground
    // net put a source between ground and itself.
    let netlist = Netlist {
        components: vec![Component::new("r1", ComponentKind::Resistor).with_value("1k")],
        nets: vec![
            Net::new("top", [("r1", "1")]).with_name("5V"),
            Net::new("bottom", [("r1", "2")]).with_name("0V"),
        ],
    };
    let bench = Bench::new("bottom").with_supplies_from_net_names(&netlist);
    assert_eq!(bench.sources.len(), 1);
    let r = op(&netlist, &bench);
    assert!((r.current("r1").unwrap().abs() - 5e-3).abs() < 1e-12);
}

/// The divider, with r3 wired at one end only and u1 excluded from
/// simulation (a black box across `out` and `gnd`).
fn half_wired() -> Netlist {
    let mut n = divider("1k", "1k");
    n.components
        .push(Component::new("r3", ComponentKind::Resistor).with_value("1k"));
    n.components
        .push(Component::new("u1", ComponentKind::BlackBox));
    n.nets[1].pins.push(Pin::new("r3", "1"));
    n.nets[1].pins.push(Pin::new("u1", "1"));
    n.nets[2].pins.push(Pin::new("u1", "2"));
    n
}

#[test]
fn a_half_wired_part_is_refused_by_default() {
    assert_eq!(
        half_wired()
            .to_circuit_with(&gnd_bench(), &ImportOptions::new())
            .expect_err("refused"),
        ImportError::Unconnected {
            component: "r3".into(),
            terminal: "2".into()
        }
    );
}

#[test]
fn skip_leaves_a_half_wired_part_out_and_simulates_the_rest() {
    let options = ImportOptions::new().with_unconnected(UnconnectedTerminals::Skip);
    let out = simulate_with(&half_wired(), &gnd_bench(), &options).expect("simulates");
    let Simulation::OperatingPoint(r) = &out.result else {
        panic!()
    };
    // r3 carries nothing, so the divider is undisturbed.
    assert!((r.node_voltages["out"] - 0.5).abs() < 1e-9);
    assert!(r.component_results.iter().all(|c| c.id != "r3"));

    let imported = &out.circuit;
    assert_eq!(
        imported.skipped,
        vec![SkippedPart {
            component: "r3".into(),
            unconnected: vec!["2".into()],
        }]
    );
    assert_eq!(
        imported.warnings,
        vec![ImportWarning {
            component: "r3".into(),
            message: "left out of the simulation: terminal \"2\" on no net".into(),
        }]
    );
    // The excluded part is a black box, reported as one.
    assert_eq!(imported.black_boxes.len(), 1);
    assert_eq!(imported.black_boxes[0].component, "u1");
    assert!(imported.high_impedance.is_empty());
}

#[test]
fn a_part_with_no_terminals_wired_is_skipped_too() {
    let mut n = divider("1k", "1k");
    n.components.push(Component::new("q1", ComponentKind::Npn));
    let options = ImportOptions::new().with_unconnected(UnconnectedTerminals::Skip);
    let imported = n.to_circuit_with(&gnd_bench(), &options).expect("builds");
    assert_eq!(
        imported.skipped,
        vec![SkippedPart {
            component: "q1".into(),
            unconnected: vec!["base".into(), "collector".into(), "emitter".into()],
        }]
    );
    assert!(imported.warnings[0]
        .message
        .contains("terminals \"base\", \"collector\", \"emitter\""));
}

#[test]
fn a_skipped_part_leaves_its_net_to_the_high_impedance_tie() {
    // r3 hangs off its own net `tap`, which only it reaches: skipped, the
    // net would float, so it's tied as a black box's would be.
    let mut n = divider("1k", "1k");
    n.components
        .push(Component::new("r3", ComponentKind::Resistor).with_value("1k"));
    n.components.push(Component::new("q1", ComponentKind::Npn));
    n.nets.push(Net::new("tap", [("q1", "base"), ("r3", "1")]));
    n.nets[1].pins.push(Pin::new("r3", "2"));
    let options = ImportOptions::new().with_unconnected(UnconnectedTerminals::Skip);
    let out = simulate_with(&n, &gnd_bench(), &options).expect("simulates");
    assert_eq!(out.circuit.skipped[0].component, "q1");
    assert_eq!(out.circuit.high_impedance.len(), 1);
    assert_eq!(out.circuit.high_impedance[0].node, "tap");
}

#[test]
fn skip_leaves_out_a_half_wired_unit_and_keeps_the_other() {
    let n = Netlist {
        components: vec![
            Component::new("r1", ComponentKind::Resistor).with_value("1k"),
            Component::new("r2", ComponentKind::Resistor).with_value("1k"),
            Component::new("u1", ComponentKind::DualOpAmp),
        ],
        nets: vec![
            Net::new("in", [("r1", "1"), ("u1", "v+")]),
            Net::new(
                "mid",
                [("r1", "2"), ("r2", "1"), ("u1", "a_in+"), ("u1", "b_in+")],
            ),
            Net::new("buf", [("u1", "a_in-"), ("u1", "a_out")]),
            Net::new("gnd", [("r2", "2"), ("u1", "v-")]),
        ],
    };
    let bench = Bench::new("gnd").with_source(BenchSource::voltage("v", "in", 10.0));
    let options = ImportOptions::new().with_unconnected(UnconnectedTerminals::Skip);
    let out = simulate_with(&n, &bench, &options).expect("simulates");
    let Simulation::OperatingPoint(r) = &out.result else {
        panic!()
    };
    assert!(
        (r.node_voltages["buf"] - 5.0).abs() < 1e-3,
        "{}",
        r.node_voltages["buf"]
    );
    assert_eq!(
        out.circuit.skipped,
        vec![SkippedPart {
            component: "u1.b".into(),
            unconnected: vec!["in-".into(), "out".into()],
        }]
    );
    assert!(r.component_results.iter().any(|c| c.id == "u1.a"));
    assert!(r.component_results.iter().all(|c| c.id != "u1.b"));
    assert!(n.to_circuit_with(&bench, &ImportOptions::new()).is_err());
}

#[cfg(feature = "serde")]
#[test]
fn the_unconnected_option_reads_from_json() {
    let options: ImportOptions = serde_json::from_str(r#"{"unconnected":"skip"}"#).unwrap();
    assert_eq!(options.unconnected, UnconnectedTerminals::Skip);
    assert_eq!(
        ImportOptions::new().unconnected,
        UnconnectedTerminals::Error
    );
}

#[test]
fn skip_ties_open_ends_a_schematic_leaves() {
    // r3 runs from `out` to `stub`, a net nothing else reaches; r4 and r5
    // form an island not yet wired to anything.
    let mut n = divider("1k", "1k");
    for id in ["r3", "r4", "r5"] {
        n.components
            .push(Component::new(id, ComponentKind::Resistor).with_value("1k"));
    }
    n.nets[1].pins.push(Pin::new("r3", "1"));
    n.nets.push(Net::new("stub", [("r3", "2")]));
    n.nets.push(Net::new("x", [("r4", "1"), ("r5", "1")]));
    n.nets.push(Net::new("y", [("r4", "2"), ("r5", "2")]));
    assert!(simulate(&n, &gnd_bench()).is_err());

    let options = ImportOptions::new().with_unconnected(UnconnectedTerminals::Skip);
    let out = simulate_with(&n, &gnd_bench(), &options).expect("simulates");
    let Simulation::OperatingPoint(r) = &out.result else {
        panic!()
    };
    assert!((r.node_voltages["out"] - 0.5).abs() < 1e-9);
    let tied: Vec<&str> = out
        .circuit
        .high_impedance
        .iter()
        .map(|h| h.node.as_str())
        .collect();
    assert_eq!(tied, ["stub", "x", "y"]);
    assert!(out.circuit.high_impedance.iter().all(|h| h.pins.is_empty()));
    assert!(out.circuit.skipped.is_empty());
}

#[test]
fn skip_ties_a_group_whose_only_way_to_ground_is_a_capacitor() {
    // A second divider, half drawn: r3 and r4 from `top` down to `bot`,
    // which reaches ground only through the decoupling cap c1. Open at
    // DC, the cap leaves `mid` and `bot` floating in the operating point.
    let mut n = divider("1k", "1k");
    for id in ["r3", "r4"] {
        n.components
            .push(Component::new(id, ComponentKind::Resistor).with_value("1k"));
    }
    n.components
        .push(Component::new("c1", ComponentKind::Capacitor).with_value("100n"));
    n.nets.push(Net::new("top", [("r3", "1")]));
    n.nets.push(Net::new("mid", [("r3", "2"), ("r4", "1")]));
    n.nets.push(Net::new("bot", [("r4", "2"), ("c1", "1")]));
    n.nets[2].pins.push(Pin::new("c1", "2"));

    let options = ImportOptions::new().with_unconnected(UnconnectedTerminals::Skip);
    let out = simulate_with(&n, &gnd_bench(), &options).expect("simulates");
    let Simulation::OperatingPoint(r) = &out.result else {
        panic!()
    };
    assert!((r.node_voltages["out"] - 0.5).abs() < 1e-9);
    let tied: Vec<&str> = out
        .circuit
        .high_impedance
        .iter()
        .map(|h| h.node.as_str())
        .collect();
    assert_eq!(tied, ["bot", "mid", "top"]);
}

#[test]
fn skip_ties_a_gate_driven_only_through_a_capacitor() {
    // m1's gate is AC-coupled from `out` through c1: the gate draws no
    // current, so `g` has no DC path to ground and is tied.
    let mut n = divider("1k", "1k");
    n.components
        .push(Component::new("c1", ComponentKind::Capacitor).with_value("100n"));
    n.components.push(Component::new("m1", ComponentKind::Nmos));
    n.nets[1].pins.push(Pin::new("c1", "1"));
    n.nets.push(Net::new("g", [("c1", "2"), ("m1", "gate")]));
    n.nets[0].pins.push(Pin::new("m1", "drain"));
    n.nets[2].pins.push(Pin::new("m1", "source"));

    let options = ImportOptions::new().with_unconnected(UnconnectedTerminals::Skip);
    let out = simulate_with(&n, &gnd_bench(), &options).expect("simulates");
    let tied: Vec<&str> = out
        .circuit
        .high_impedance
        .iter()
        .map(|h| h.node.as_str())
        .collect();
    assert_eq!(tied, ["g"]);
}

#[cfg(feature = "spice")]
mod subckt_models {
    use super::*;

    /// A vendor-style op-amp: input resistance, a gain stage with a pole,
    /// output resistance and clamp diodes to the rails. Ports in+ in- out
    /// v+ v-.
    const OPAMP: &str = "
* vendor op-amp macromodel
.subckt OPX inp inn out vcc vee
Rin inp inn 2meg
G1 0 n1 inp inn 1m
R1 n1 0 100meg
C1 n1 0 15n
E1 n2 0 n1 0 1
Ro n2 out 50
D1 out vcc dclamp
D2 vee out dclamp
.model dclamp D(IS=1e-14)
.ends OPX
";

    /// A regulator: 3.3 V out, or the input less 1 V when that's lower.
    /// Ports in out gnd.
    const REG: &str = "
.subckt reg33 in out gnd
B1 out gnd V = min(V(in, gnd) - 1, 3.3)
.ends
";

    /// An inverting amplifier, gain −10, on ±12 V rails, built around U1.
    fn inverting_amp(model: SubcktModel) -> Netlist {
        Netlist {
            components: vec![
                Component::new("R1", ComponentKind::Resistor).with_value("10k"),
                Component::new("R2", ComponentKind::Resistor).with_value("100k"),
                Component::new("U1", ComponentKind::OpAmp).with_model(Model::Subckt(model)),
            ],
            nets: vec![
                Net::new("in", [("R1", "1")]),
                Net::new("inv", [("R1", "2"), ("R2", "1"), ("U1", "-")]),
                Net::new("out", [("R2", "2"), ("U1", "OUT")]),
                Net::new("vcc", [("U1", "V+")]),
                Net::new("vee", [("U1", "V-")]),
                Net::new("gnd", [("U1", "+")]),
            ],
        }
    }

    fn amp_bench() -> Bench {
        Bench::new("gnd")
            .with_source(BenchSource::voltage("vin", "in", 0.5))
            .with_source(BenchSource::voltage("vp", "vcc", 12.0))
            .with_source(BenchSource::voltage("vn", "vee", -12.0))
    }

    fn opamp_model() -> SubcktModel {
        SubcktModel::new(OPAMP, "opx", ["+", "-", "OUT", "V+", "V-"])
    }

    fn deck_op(deck: &str) -> crate::SimulationResult {
        let parsed = crate::spice::parse_str(deck).expect("deck parses");
        crate::solve_circuit(&parsed.circuit).expect("deck solves")
    }

    #[test]
    fn a_vendor_op_amp_matches_the_same_deck_run_as_spice() {
        let r = op(&inverting_amp(opamp_model()), &amp_bench());
        let deck = format!(
            "inverting amp\n{OPAMP}\nVIN in 0 0.5\nVP vcc 0 12\nVN vee 0 -12\n\
             R1 in inv 10k\nR2 inv out 100k\nXU1 0 inv out vcc vee OPX\n.end\n"
        );
        let spice = deck_op(&deck);
        // Two separate Newton solves: equal to within their tolerance.
        assert!((r.node_voltages["out"] - spice.node_voltages["out"]).abs() < 1e-6);
        assert!(
            (r.node_voltages["out"] + 5.0).abs() < 1e-2,
            "{}",
            r.node_voltages["out"]
        );
        // Internal nodes live under the component's id.
        assert!(
            r.node_voltages.contains_key("U1.n1"),
            "{:?}",
            r.node_voltages.keys()
        );
    }

    #[test]
    fn a_regulator_model_attaches_by_terminal_names() {
        let model = SubcktModel::new(REG, "REG33", ["VIN", "VOUT", "GND"]);
        let netlist = Netlist {
            components: vec![
                Component::new("U2", ComponentKind::Regulator).with_model(Model::Subckt(model)),
                Component::new("RL", ComponentKind::Resistor).with_value("100"),
            ],
            nets: vec![
                Net::new("vin", [("U2", "1")]),
                Net::new("vout", [("U2", "3"), ("RL", "1")]),
                Net::new("gnd", [("U2", "2"), ("RL", "2")]),
            ],
        };
        let options = ImportOptions::new()
            .with_terminal_names("U2", [("1", "VIN"), ("2", "GND"), ("3", "VOUT")]);
        for (vin, want) in [(5.0, 3.3), (3.8, 2.8)] {
            let bench = Bench::new("gnd").with_source(BenchSource::voltage("v", "vin", vin));
            let out = simulate_with(&netlist, &bench, &options).expect("simulates");
            let Simulation::OperatingPoint(r) = &out.result else {
                panic!()
            };
            let deck =
                format!("reg\n{REG}\nV1 vin 0 {vin}\nXU2 vin vout 0 reg33\nRL vout 0 100\n.end\n");
            let spice = deck_op(&deck);
            assert!((r.node_voltages["vout"] - want).abs() < 1e-6);
            assert!((r.node_voltages["vout"] - spice.node_voltages["vout"]).abs() < 1e-9);
            assert_eq!(out.circuit.model_source("U2"), Some(ModelSource::Model));
        }
    }

    /// Under `Skip` every net is checked for an open end. A subcircuit's
    /// pins are element terminals like any other: the regulator drives
    /// `vout`, so the net isn't tied to ground or reported as floating.
    #[test]
    fn a_net_a_subcircuit_drives_is_not_high_impedance() {
        let model = SubcktModel::new(REG, "REG33", ["VIN", "VOUT", "GND"]);
        let netlist = Netlist {
            components: vec![
                Component::new("U2", ComponentKind::Regulator).with_model(Model::Subckt(model)),
                Component::new("RL", ComponentKind::Resistor).with_value("100"),
            ],
            nets: vec![
                Net::new("vin", [("U2", "1")]),
                Net::new("vout", [("U2", "3"), ("RL", "1")]),
                Net::new("gnd", [("U2", "2"), ("RL", "2")]),
            ],
        };
        let options = ImportOptions::new()
            .with_terminal_names("U2", [("1", "VIN"), ("2", "GND"), ("3", "VOUT")])
            .with_unconnected(UnconnectedTerminals::Skip);
        let bench = Bench::new("gnd").with_source(BenchSource::voltage("v", "vin", 5.0));
        let out = simulate_with(&netlist, &bench, &options).expect("simulates");
        assert_eq!(out.circuit.high_impedance, vec![]);
        let Simulation::OperatingPoint(r) = &out.result else {
            panic!()
        };
        assert!((r.node_voltages["vout"] - 3.3).abs() < 1e-6);
    }

    #[test]
    fn a_pin_that_isnt_a_terminal_is_named_in_the_error() {
        let model = SubcktModel::new(OPAMP, "OPX", ["+", "-", "OUTPUT", "V+", "V-"]);
        assert_eq!(
            inverting_amp(model).to_circuit(&amp_bench()).unwrap_err(),
            ImportError::SubcktPin {
                component: "U1".into(),
                pin: "OUTPUT".into(),
                problem: "isn't a terminal of the component".into(),
            }
        );
        let twice = SubcktModel::new(OPAMP, "OPX", ["+", "-", "OUT", "V+", "+"]);
        assert!(matches!(
            inverting_amp(twice).to_circuit(&amp_bench()).unwrap_err(),
            ImportError::SubcktPin { pin, .. } if pin == "+"
        ));
    }

    #[test]
    fn a_map_of_the_wrong_length_or_name_is_refused() {
        let short = SubcktModel::new(OPAMP, "OPX", ["+", "-", "OUT", "V+"]);
        let e = inverting_amp(short).to_circuit(&amp_bench()).unwrap_err();
        assert_eq!(
            e.to_string(),
            "component \"U1\": its subcircuit model .subckt \"OPX\" has 5 ports, and the pin \
             map gives 4"
        );
        let missing = SubcktModel::new(OPAMP, "OPA2134", ["+", "-", "OUT", "V+", "V-"]);
        let e = inverting_amp(missing).to_circuit(&amp_bench()).unwrap_err();
        assert!(
            e.to_string()
                .contains("has no .subckt \"OPA2134\" (it has \"OPX\")"),
            "{e}"
        );
        let broken = SubcktModel::new(".subckt bad a b\nR1 a b\n.ends", "bad", ["+", "-"]);
        let e = inverting_amp(broken).to_circuit(&amp_bench()).unwrap_err();
        assert!(matches!(e, ImportError::SubcktModel { .. }), "{e}");
    }

    #[test]
    fn an_unconnected_pin_is_refused_or_skipped() {
        let mut n = inverting_amp(opamp_model());
        n.nets.retain(|net| net.id != "vee");
        assert_eq!(
            n.to_circuit(&amp_bench().with_source(BenchSource::voltage("vx", "in", 0.0)))
                .unwrap_err(),
            ImportError::UnknownNet {
                item: "vn".into(),
                net: "vee".into(),
                problem: "isn't in the netlist".into(),
            }
        );
        let bench = Bench::new("gnd")
            .with_source(BenchSource::voltage("vin", "in", 0.5))
            .with_source(BenchSource::voltage("vp", "vcc", 12.0));
        let options = ImportOptions::new().with_terminal_names(
            "U1",
            [
                ("+", "+"),
                ("-", "-"),
                ("OUT", "OUT"),
                ("V+", "V+"),
                ("V-", "V-"),
            ],
        );
        assert_eq!(
            n.to_circuit_with(&bench, &options).unwrap_err(),
            ImportError::Unconnected {
                component: "U1".into(),
                terminal: "V-".into(),
            }
        );
        let skip = options.with_unconnected(UnconnectedTerminals::Skip);
        let imported = n.to_circuit_with(&bench, &skip).expect("skips");
        assert_eq!(imported.skipped[0].component, "U1");
        assert!(imported.circuit.instances.is_empty());
    }

    #[test]
    fn two_parts_with_the_same_model_each_get_their_own_copy() {
        let mut n = inverting_amp(opamp_model());
        n.components.push(
            Component::new("U3", ComponentKind::OpAmp).with_model(Model::Subckt(opamp_model())),
        );
        n.nets.push(Net::new("buf", [("U3", "-"), ("U3", "OUT")]));
        for net in &mut n.nets {
            match net.id.as_str() {
                "out" => net.pins.push(Pin::new("U3", "+")),
                "vcc" => net.pins.push(Pin::new("U3", "V+")),
                "vee" => net.pins.push(Pin::new("U3", "V-")),
                _ => {}
            }
        }
        let r = op(&n, &amp_bench());
        assert!((r.node_voltages["buf"] - r.node_voltages["out"]).abs() < 1e-3);
        let imported = n.to_circuit(&amp_bench()).unwrap();
        assert_eq!(imported.circuit.blocks.len(), 2);
    }

    /// The op-amp model with its internal node `n1` named `to` instead.
    fn opamp_with_internal(to: &str) -> SubcktModel {
        let text = OPAMP.replace(" n1 ", &format!(" {to} "));
        SubcktModel::new(text, "opx", ["+", "-", "OUT", "V+", "V-"])
    }

    /// A model's internal node named like the circuit's ground net stays
    /// the model's own: the results are the model's with the node renamed.
    #[test]
    fn a_model_node_named_like_the_ground_net_is_not_ground() {
        let on_ground_one = |model: SubcktModel| {
            let mut n = inverting_amp(model);
            for net in &mut n.nets {
                if net.id == "gnd" {
                    net.id = "1".into();
                }
            }
            let bench = Bench::new("1")
                .with_source(BenchSource::voltage("vin", "in", 0.5))
                .with_source(BenchSource::voltage("vp", "vcc", 12.0))
                .with_source(BenchSource::voltage("vn", "vee", -12.0));
            op(&n, &bench).node_voltages["out"]
        };
        let named_one = on_ground_one(opamp_with_internal("1"));
        let by_hand = on_ground_one(opamp_with_internal("n1"));
        assert!(
            (named_one - by_hand).abs() < 1e-9,
            "{named_one} vs {by_hand}"
        );
        assert!((by_hand + 5.0).abs() < 1e-2, "{by_hand}");
    }

    /// A model's internal node named like another model's `.global` stays
    /// the model's own.
    #[test]
    fn a_model_node_named_like_another_models_global_is_not_that_global() {
        const BIAS: &str = "
.subckt bias p
.global vref
Ra p vref 1k
Rb vref 0 1k
.ends
";
        let with_bias = |model: SubcktModel| {
            let mut n = inverting_amp(model);
            n.components.push(
                Component::new("U2", ComponentKind::BlackBox)
                    .with_model(Model::Subckt(SubcktModel::new(BIAS, "bias", ["1"]))),
            );
            for net in &mut n.nets {
                if net.id == "vcc" {
                    net.pins.push(Pin::new("U2", "1"));
                }
            }
            let r = op(&n, &amp_bench());
            (r.node_voltages["out"], r.node_voltages["vref"])
        };
        let named_vref = with_bias(opamp_with_internal("vref"));
        let by_hand = with_bias(opamp_with_internal("n1"));
        assert!(
            (named_vref.0 - by_hand.0).abs() < 1e-9,
            "{named_vref:?} vs {by_hand:?}"
        );
        assert!(
            (named_vref.1 - by_hand.1).abs() < 1e-9,
            "{named_vref:?} vs {by_hand:?}"
        );
        assert!((by_hand.1 - 6.0).abs() < 1e-6, "{by_hand:?}");
    }

    #[cfg(feature = "serde")]
    #[test]
    fn a_subckt_model_reads_from_json() {
        let json =
            r#"{"type": "subckt", "text": ".subckt b a\n.ends", "name": "b", "pins": ["1"]}"#;
        let model: Model = serde_json::from_str(json).unwrap();
        assert_eq!(
            model,
            Model::Subckt(SubcktModel::new(".subckt b a\n.ends", "b", ["1"]))
        );
    }
}
