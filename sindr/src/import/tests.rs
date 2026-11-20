use super::*;
use crate::ac_analysis::{AcConfig, FrequencySpacing};
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
        .with_analysis(Analysis::Ac(AcConfig {
            f_start: 10.0,
            f_stop: 1e3,
            num_points: 3,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "sig".into(),
            ac_magnitude: 1.0,
        }));
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
