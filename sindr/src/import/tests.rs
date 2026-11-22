use super::*;
use crate::ac_analysis::AcConfig;
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
    let Some(CircuitElement::OpAmp { v_pos, v_neg, .. }) =
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
        .with_analysis(Analysis::DcSweep {
            source: "sig".into(),
            start: 0.0,
            stop: 4.0,
            points: 5,
        });
    let Simulation::DcSweep(r) = simulate(&netlist, &sweep).unwrap().result else {
        panic!()
    };
    let curve = r.node_voltage_curve("out");
    assert!((curve[4].1 - 2.0).abs() < 1e-9);
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
        ComponentKind::from_name("crystal"),
        ComponentKind::Other("crystal".into())
    );
}

#[test]
fn passive_and_unknown_kinds_have_no_terminals() {
    for (_, kind) in KINDS {
        assert_eq!(
            kind.terminals().is_empty(),
            kind.is_passive(),
            "{}",
            kind.name()
        );
        assert!(kind.optional_terminals() < kind.terminals().len().max(1));
    }
    assert!(ComponentKind::Other("crystal".into())
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
    n.components[1].kind = ComponentKind::from_name("crystal");
    assert_eq!(
        err(&n, &gnd_bench()),
        ImportError::UnsupportedKind {
            component: "r2".into(),
            kind: "crystal".into()
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
            {"id": "c-4", "kind": "crystal", "value": "16MHz"}
        ],
        "nets": [
            {"id": "n-1", "name": "3V3", "pins": [{"component": "c-1", "terminal": "1"}]}
        ]
    }"#;
    let netlist: Netlist = serde_json::from_str(json).unwrap();
    assert_eq!(
        netlist.components[3].kind,
        ComponentKind::Other("crystal".into())
    );
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
        .find(|e| e.id() == id)
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
        CircuitElement::Capacitor {
            id: "c1".into(),
            nodes: ns(),
            capacitance: 10e-6,
        }
    );
    assert_eq!(
        built(
            Component::new("l1", ComponentKind::Inductor).with_value(String::from("2.2mH")),
            &["a", "b"]
        ),
        CircuitElement::Inductor {
            id: "l1".into(),
            nodes: ns(),
            inductance: 2.2e-3,
            saturation: None,
        }
    );
    assert_eq!(
        built(Component::new("t1", ComponentKind::Thermistor), &["1", "2"]),
        CircuitElement::Thermistor {
            id: "t1".into(),
            nodes: ns(),
            temperature: 298.15,
        }
    );
    assert_eq!(
        built(
            Component::new("p1", ComponentKind::Photoresistor).with_param("light_level", 0.4),
            &["1", "2"]
        ),
        CircuitElement::Photoresistor {
            id: "p1".into(),
            nodes: ns(),
            light_level: 0.4,
        }
    );
    assert_eq!(
        built(
            Component::new("v1", ComponentKind::VoltageSource).with_value("3V3"),
            &["pos", "n"]
        ),
        CircuitElement::VoltageSource {
            id: "v1".into(),
            nodes: ns(),
            voltage: 3.3,
            waveform: None,
        }
    );
    assert_eq!(
        built(
            Component::new("i1", ComponentKind::CurrentSource).with_value("2mA"),
            &["from", "to"]
        ),
        CircuitElement::CurrentSource {
            id: "i1".into(),
            nodes: ns(),
            current: 2e-3,
            waveform: None,
        }
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
        CircuitElement::Diode {
            id: "d1".into(),
            nodes: ns(),
            temperature: 350.0,
            model: Some(dm),
        }
    );
    assert_eq!(
        built(
            Component::new("d2", ComponentKind::SchottkyDiode),
            &["a", "c"]
        ),
        CircuitElement::SchottkyDiode {
            id: "d2".into(),
            nodes: ns(),
            temperature: NOMINAL_K,
        }
    );
    assert_eq!(
        built(
            Component::new("d3", ComponentKind::Photodiode).with_param("irradiance", "1m"),
            &["a", "k"]
        ),
        CircuitElement::Photodiode {
            id: "d3".into(),
            nodes: ns(),
            irradiance: 1e-3,
            temperature: NOMINAL_K,
        }
    );
    // The breakdown is the value, or `vz`, which wins over it.
    let zener = |c: Component| match built(c, &["a", "k"]) {
        CircuitElement::ZenerDiode { vz, .. } => vz,
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
        CircuitElement::Led {
            id: "led".into(),
            nodes: ns(),
            color: "red".into(),
            temperature: NOMINAL_K,
        }
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
        CircuitElement::Varactor {
            id: "cv".into(),
            nodes: ns(),
            params: vp,
        }
    );
    assert_eq!(
        built(Component::new("cv", ComponentKind::Varactor), &["a", "k"]),
        CircuitElement::Varactor {
            id: "cv".into(),
            nodes: ns(),
            params: VaractorParams::default(),
        }
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
        CircuitElement::Bjt {
            id: "q1".into(),
            nodes: ns(),
            kind: BjtKind::Pnp,
            bf: 80.0,
            temperature: NOMINAL_K,
            parasitic_caps: None,
            model: Some(bm),
        }
    );
    match built(Component::new("q2", ComponentKind::Npn), &["b", "c", "e"]) {
        CircuitElement::Bjt { bf, kind, .. } => assert_eq!((bf, kind), (100.0, BjtKind::Npn)),
        other => panic!("{other:?}"),
    }

    // A MOSFET's body may be left off; it is tied to the source.
    assert_eq!(
        built(Component::new("m1", ComponentKind::Pmos), &["g", "d", "s"]),
        CircuitElement::Mosfet {
            id: "m1".into(),
            nodes: ns(),
            kind: MosfetKind::Pmos,
            params: MosfetParams::default_pmos(),
            parasitic_caps: None,
            body_diode: None,
        }
    );
    let mp = MosfetParams {
        vto: 1.5,
        ..MosfetParams::default_nmos()
    };
    match built(
        Component::new("m2", ComponentKind::Nmos).with_model(Model::Mosfet(mp.clone())),
        &["g", "d", "s"],
    ) {
        CircuitElement::Mosfet { params, kind, .. } => {
            assert_eq!((params, kind), (mp, MosfetKind::Nmos))
        }
        other => panic!("{other:?}"),
    }

    // A JFET's pinch-off defaults to the sign of its channel.
    assert_eq!(
        built(Component::new("j1", ComponentKind::Njfet), &["g", "d", "s"]),
        CircuitElement::Jfet {
            id: "j1".into(),
            nodes: ns(),
            kind: JfetKind::NChannel,
            idss: 10e-3,
            vp: -2.0,
        }
    );
    assert_eq!(
        built(
            Component::new("j2", ComponentKind::Pjfet).with_param("idss", "5mA"),
            &["g", "d", "s"]
        ),
        CircuitElement::Jfet {
            id: "j2".into(),
            nodes: ns(),
            kind: JfetKind::PChannel,
            idss: 5e-3,
            vp: 2.0,
        }
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
        CircuitElement::Igbt {
            id: "g1".into(),
            nodes: ns(),
            params: ip,
        }
    );
    match built(Component::new("g2", ComponentKind::Igbt), &["g", "c", "e"]) {
        CircuitElement::Igbt { params, .. } => assert_eq!(params, IgbtParams::default()),
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
        CircuitElement::Comparator {
            id: "u1".into(),
            nodes: ns(),
            v_pos: 15.0,
            v_neg: -15.0,
        }
    );
    // Parameters win over any supply on the pins.
    match built(
        Component::new("u2", ComponentKind::OpAmp)
            .with_param("v_pos", 3.3)
            .with_param("v_neg", 0.0),
        &["+", "-", "out", "v+", "v-"],
    ) {
        CircuitElement::OpAmp {
            v_pos,
            v_neg,
            model,
            ..
        } => assert_eq!((v_pos, v_neg, model), (3.3, 0.0, None)),
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
    let Some(CircuitElement::OpAmp { v_pos, v_neg, .. }) =
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
        CircuitElement::Switch {
            id: "s1".into(),
            nodes: ns(),
            closed: true,
        }
    );
    assert_eq!(
        built(Component::new("s2", ComponentKind::PushButton), &["1", "2"]),
        CircuitElement::Pushbutton {
            id: "s2".into(),
            nodes: ns(),
            closed: false,
        }
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
        CircuitElement::Fuse {
            id: "f1".into(),
            nodes: ns(),
            rating: 1.0,
            blown: true,
        }
    );
    match built(
        Component::new("f2", ComponentKind::Fuse).with_value("500mA"),
        &["1", "2"],
    ) {
        CircuitElement::Fuse { rating, blown, .. } => assert_eq!((rating, blown), (0.5, false)),
        other => panic!("{other:?}"),
    }
    assert_eq!(
        built(
            Component::new("k1", ComponentKind::Relay).with_param("pickup_voltage", "3V5"),
            &["a1", "a2", "com", "no"]
        ),
        CircuitElement::Relay {
            id: "k1".into(),
            nodes: ns(),
            coil_resistance: 500.0,
            pickup_voltage: 3.5,
            inductance: 0.0,
        }
    );
    assert_eq!(
        built(
            Component::new("tr1", ComponentKind::Transformer)
                .with_param("l1", "10mH")
                .with_param("l2", "1mH"),
            &["p1", "q1", "sec+", "sec-"]
        ),
        CircuitElement::Transformer {
            id: "tr1".into(),
            nodes: ns(),
            l1: 10e-3,
            l2: 1e-3,
            k: 0.999,
            saturation: None,
        }
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
        CircuitElement::VoltageRegulator {
            id: "u1".into(),
            nodes: ns(),
            voltage: 5.0,
            model: Some(rp),
        }
    );
    match built(
        Component::new("u2", ComponentKind::Regulator).with_value("3V3"),
        &["vin", "vout", "com"],
    ) {
        CircuitElement::VoltageRegulator { voltage, model, .. } => {
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
        CircuitElement::Battery {
            id: "b1".into(),
            nodes: ns(),
            params: BatteryParams::constant(9.0, 0.5, 0.1),
            state_of_charge: 0.8,
        }
    );
    // A model stands in for the value.
    let li = BatteryParams::lithium_ion(2.0, 0.05);
    assert_eq!(
        built(
            Component::new("b2", ComponentKind::Battery).with_model(Model::Battery(li.clone())),
            &["+", "-"]
        ),
        CircuitElement::Battery {
            id: "b2".into(),
            nodes: ns(),
            params: li,
            state_of_charge: 1.0,
        }
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
        CircuitElement::Capacitor {
            id: "cl".into(),
            nodes: ["out".into(), "gnd".into()],
            capacitance: 1e-6,
        }
    );
    assert_eq!(
        element("ll"),
        CircuitElement::Inductor {
            id: "ll".into(),
            nodes: ["out".into(), "gnd".into()],
            inductance: 1e-3,
            saturation: None,
        }
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
        .push(Component::new("y1", ComponentKind::from_name("crystal")));
    let e = err(&n, &gnd_bench());
    assert_eq!(
        e,
        ImportError::UnsupportedKind {
            component: "y1".into(),
            kind: "crystal".into()
        }
    );
    assert_eq!(
        e.to_string(),
        "component \"y1\": sindr can't simulate a \"crystal\""
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
        if kind.is_passive() {
            continue;
        }
        let mut c = Component::new("x", kind.clone()).with_value(1.0);
        for &p in kind.params() {
            let v: Value = match p {
                "closed" | "blown" => true.into(),
                "color" => "green".into(),
                "position" | "light_level" | "state_of_charge" | "k" => 0.5.into(),
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
