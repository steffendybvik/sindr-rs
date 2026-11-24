//! Logic gates at the analog level: truth tables, the DC transfer curve,
//! Schmitt hysteresis, the output stage against the pin model, and the
//! propagation delay in a ring oscillator.

use std::collections::HashMap;

use sindr::element::{GateKind, LogicGate, PinModel, Resistor, VoltageSource};
use sindr::{
    dc_sweep, solve_circuit, solve_transient, validate_circuit, Circuit, CircuitElement,
    DcSweepAxis, SimError, TransientConfig, TransientStart, Waveform,
};

const VDD: f64 = 5.0;

fn vsource(id: &str, p: &str, volts: f64, waveform: Option<Waveform>) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), "0".into()],
        voltage: volts,
        waveform,
    })
}

fn resistor(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

/// A 74HC-like gate on the 5 V rail `vdd`.
fn gate(id: &str, kind: GateKind, inputs: &[&str], out: &str) -> LogicGate {
    LogicGate::new(
        id,
        kind,
        inputs.iter().map(|&n| n.into()),
        [out.into(), "vdd".into(), "0".into()],
    )
    .with_model(PinModel::cmos(VDD))
}

/// The supply and `parts`.
fn powered(parts: Vec<CircuitElement>) -> Circuit {
    let mut components = vec![vsource("VDD", "vdd", VDD, None)];
    components.extend(parts);
    Circuit::new("0", components)
}

const KINDS: [GateKind; 8] = [
    GateKind::Buffer,
    GateKind::Not,
    GateKind::And,
    GateKind::Or,
    GateKind::Nand,
    GateKind::Nor,
    GateKind::Xor,
    GateKind::Xnor,
];

/// Every gate, two and three inputs where it takes them, matches its
/// truth table at the rails with nothing else on its output: no helper
/// loads.
#[test]
fn every_gate_matches_its_truth_table() {
    for kind in KINDS {
        let widths: &[usize] = if kind.is_single_input() {
            &[1]
        } else {
            &[2, 3]
        };
        for &n in widths {
            for bits in 0..1u32 << n {
                let levels: Vec<bool> = (0..n).map(|k| bits >> k & 1 == 1).collect();
                let names: Vec<String> = (0..n).map(|k| format!("in{k}")).collect();
                let mut parts: Vec<CircuitElement> = levels
                    .iter()
                    .zip(&names)
                    .map(|(&high, name)| {
                        vsource(
                            &format!("V{name}"),
                            name,
                            if high { VDD } else { 0.0 },
                            None,
                        )
                    })
                    .collect();
                let refs: Vec<&str> = names.iter().map(String::as_str).collect();
                parts.push(gate("U1", kind, &refs, "y").into());
                let result = solve_circuit(&powered(parts)).unwrap();
                let y = result.node_voltages["y"];
                let want = if kind.eval(&levels) { VDD } else { 0.0 };
                assert!(
                    (y - want).abs() < 1e-3,
                    "{kind:?} {levels:?}: y = {y} V, want {want} V"
                );
            }
        }
    }
}

/// An inverter's DC transfer curve: full swing outside the thresholds,
/// the switch centred between them, and falling all the way.
#[test]
fn inverter_transfer_curve() {
    let model = PinModel::cmos(VDD);
    let circuit = powered(vec![
        vsource("VIN", "a", 0.0, None),
        gate("U1", GateKind::Not, &["a"], "y").into(),
    ]);
    let sweep = dc_sweep(&circuit, &DcSweepAxis::new("VIN", 0.0, VDD, 101)).unwrap();
    let curve = sweep.node_voltage_curve("y");
    for pair in curve.windows(2) {
        assert!(pair[1].1 <= pair[0].1 + 1e-9, "rises at {:?}", pair);
    }
    let at = |v: f64| {
        curve
            .iter()
            .min_by(|a, b| (a.0 - v).abs().total_cmp(&(b.0 - v).abs()))
            .unwrap()
            .1
    };
    // Fully switched at each threshold, half way at the midpoint.
    assert!(at(model.vil) > 0.999 * VDD, "{}", at(model.vil));
    assert!(at(model.vih) < 0.001 * VDD, "{}", at(model.vih));
    assert!((at(0.5 * (model.vil + model.vih)) - 0.5 * VDD).abs() < 0.05);
}

/// A Schmitt inverter on a slow triangle switches low near `vih` on the
/// way up and high near `vil` on the way down.
#[test]
fn schmitt_input_has_hysteresis() {
    let model = PinModel::cmos(VDD);
    let triangle = Waveform::Pwl {
        points: vec![(0.0, 0.0), (1e-3, VDD), (2e-3, 0.0)],
    };
    let circuit = powered(vec![
        vsource("VIN", "a", 0.0, Some(triangle)),
        gate("U1", GateKind::Not, &["a"], "y").with_schmitt().into(),
    ]);
    let tran = solve_transient(&circuit, &TransientConfig::new(2e-3, 2e-6))
        .unwrap()
        .transient
        .unwrap();
    let mut edges = Vec::new();
    for pair in tran.timesteps.windows(2) {
        let (y0, y1) = (pair[0].node_voltages["y"], pair[1].node_voltages["y"]);
        if (y0 - 0.5 * VDD) * (y1 - 0.5 * VDD) < 0.0 {
            edges.push((pair[1].time, pair[1].node_voltages["a"], y1 > y0));
        }
    }
    assert_eq!(edges.len(), 2, "{edges:?}");
    let (_, falling_at, rose) = edges[0];
    assert!(!rose && (falling_at - model.vih).abs() < 0.1, "{edges:?}");
    let (_, rising_at, rose) = edges[1];
    assert!(rose && (rising_at - model.vil).abs() < 0.1, "{edges:?}");
}

/// A gate holds its input between the thresholds where a Schmitt input
/// last left it: at the midpoint, the DC solve starts low.
#[test]
fn schmitt_input_reads_low_from_the_start_between_thresholds() {
    let circuit = powered(vec![
        vsource("VIN", "a", 0.5 * VDD, None),
        gate("U1", GateKind::Buffer, &["a"], "y")
            .with_schmitt()
            .into(),
    ]);
    let y = solve_circuit(&circuit).unwrap().node_voltages["y"];
    assert!(y < 0.01, "{y}");
}

/// Driving high into a load, the output stage is `r_high` to the supply,
/// as a push-pull pin's is; its result is the current into the output.
#[test]
fn output_stage_is_the_pin_model() {
    let model = PinModel::cmos(VDD);
    for (input, load_to, want) in [
        (0.0, "0", VDD * 1e3 / (1e3 + model.r_high)),
        (VDD, "vdd", VDD * model.r_low / (1e3 + model.r_low)),
    ] {
        let circuit = powered(vec![
            vsource("VIN", "a", input, None),
            gate("U1", GateKind::Not, &["a"], "y").into(),
            resistor("RL", "y", load_to, 1e3),
        ]);
        let result = solve_circuit(&circuit).unwrap();
        let y = result.node_voltages["y"];
        assert!((y - want).abs() < 1e-3, "y = {y}, want {want}");
        let load: f64 = if load_to == "0" {
            y / 1e3
        } else {
            -(VDD - y) / 1e3
        };
        let u1 = result
            .component_results
            .iter()
            .find(|c| c.id == "U1")
            .expect("the gate has a result");
        assert!(
            (u1.current_through + load).abs() < 1e-6,
            "into the output {} A, load {load} A",
            u1.current_through
        );
        assert!((u1.voltage_across - y).abs() < 1e-12);
        assert!(u1.power > 0.0);
        assert!(
            result.component_results.iter().all(|c| !c.id.contains('#')),
            "parts leak into the result"
        );
    }
}

/// Three inverters in a ring oscillate at about 1 / (6 · tpd).
#[test]
fn ring_of_three_oscillates_at_one_over_six_tpd() {
    let tpd = 10e-9;
    let nodes = ["n0", "n1", "n2"];
    let mut parts = Vec::new();
    for k in 0..3 {
        parts.push(
            gate(
                &format!("U{k}"),
                GateKind::Not,
                &[nodes[k]],
                nodes[(k + 1) % 3],
            )
            .with_tpd(tpd)
            .with_c_in(2e-12)
            .into(),
        );
    }
    let config = TransientConfig::new(600e-9, 0.5e-9)
        .with_initial_voltages(HashMap::from([
            ("n0".to_string(), 0.0),
            ("n1".to_string(), VDD),
            ("n2".to_string(), 0.0),
        ]))
        .with_start(TransientStart::InitialConditions);
    let tran = solve_transient(&powered(parts), &config)
        .unwrap()
        .transient
        .unwrap();
    // Rising through half way, counted only after n0 has been below 20 %.
    let mut rising = Vec::new();
    let mut armed = false;
    for p in tran.timesteps.windows(2) {
        let (a, b) = (p[0].node_voltages["n0"], p[1].node_voltages["n0"]);
        armed |= b < 0.2 * VDD;
        if armed && a < 0.5 * VDD && b >= 0.5 * VDD {
            // Interpolate the crossing between the two steps.
            let t = p[0].time + (0.5 * VDD - a) / (b - a) * (p[1].time - p[0].time);
            rising.push(t);
            armed = false;
        }
    }
    assert!(rising.len() >= 4, "{rising:?}");
    // Skip the start-up; average the settled periods.
    let settled = &rising[1..];
    let period = (settled[settled.len() - 1] - settled[0]) / (settled.len() - 1) as f64;
    let ratio = period / (6.0 * tpd);
    assert!(
        (0.8..1.25).contains(&ratio),
        "period {period:e} s, {ratio} × 6 tpd"
    );
}

/// A gate given the wrong number of inputs, or a negative delay, is
/// refused naming the field.
#[test]
fn bad_gates_are_refused() {
    let one_input_nand = gate("U1", GateKind::Nand, &["a"], "y");
    let two_input_not = gate("U2", GateKind::Not, &["a", "b"], "y");
    let mut slow = gate("U3", GateKind::Not, &["a"], "y");
    slow.tpd = -1e-9;
    let mut flat = gate("U4", GateKind::Not, &["a"], "y");
    flat.model.vih = flat.model.vil;
    for (g, needle) in [
        (one_input_nand, "two or more inputs"),
        (two_input_not, "one input"),
        (slow, "tpd"),
        (flat, "vih"),
    ] {
        let circuit = powered(vec![
            vsource("VA", "a", 0.0, None),
            vsource("VB", "b", 0.0, None),
            resistor("RB", "b", "0", 1e3),
            g.into(),
            resistor("RL", "y", "0", 1e3),
        ]);
        let err = validate_circuit(&circuit).unwrap_err();
        assert!(err.to_string().contains(needle), "{err}");
        assert!(solve_circuit(&circuit).is_err());
    }
    // An element the analyses lower can't be stamped as itself.
    let _ = SimError::NoGround;
}

/// The gate round-trips through JSON with its defaults filled in.
#[cfg(feature = "serde")]
#[test]
fn serde_round_trip_and_defaults() {
    let element: CircuitElement = gate("U1", GateKind::Xor, &["a", "b"], "y")
        .with_tpd(5e-9)
        .into();
    let json = serde_json::to_string(&element).unwrap();
    assert!(json.contains("\"type\":\"logic_gate\""), "{json}");
    assert!(json.contains("\"kind\":\"xor\""), "{json}");
    let back: CircuitElement = serde_json::from_str(&json).unwrap();
    assert_eq!(back, element);
    let minimal: CircuitElement = serde_json::from_str(
        r#"{"type":"logic_gate","id":"U1","kind":"not","nodes":["a","y","vdd","0"]}"#,
    )
    .unwrap();
    let CircuitElement::LogicGate(g) = minimal else {
        panic!("not a gate")
    };
    assert_eq!((g.tpd, g.c_in, g.schmitt), (0.0, 0.0, false));
    assert_eq!(g.model, PinModel::default());
}

/// It exports as its parts, behavioural sources included, and the deck
/// reads back to the same truth table.
#[cfg(feature = "spice")]
#[test]
fn exports_as_its_parts() {
    let circuit = powered(vec![
        vsource("VA", "a", VDD, None),
        vsource("VB", "b", 0.0, None),
        gate("U1", GateKind::Xor, &["a", "b"], "y")
            .with_tpd(5e-9)
            .into(),
    ]);
    let deck = sindr::spice::to_netlist(&circuit, "xor").unwrap();
    assert!(deck.contains("V={"), "{deck}");
    let parsed = sindr::spice::parse_str(&deck).unwrap();
    let y = solve_circuit(&parsed.circuit).unwrap().node_voltages["y"];
    assert!((y - VDD).abs() < 1e-3, "{y}\n{deck}");
}

/// A sharp input edge reaches the output's half-way point `tpd` later,
/// both ways.
#[test]
fn a_sharp_edge_is_delayed_by_tpd() {
    let tpd = 20e-9;
    let pulse = Waveform::Pulse {
        v1: 0.0,
        v2: VDD,
        delay: 50e-9,
        rise_time: 0.1e-9,
        fall_time: 0.1e-9,
        pulse_width: 100e-9,
        period: 1.0,
    };
    let circuit = powered(vec![
        vsource("VIN", "a", 0.0, Some(pulse)),
        gate("U1", GateKind::Buffer, &["a"], "y")
            .with_tpd(tpd)
            .into(),
    ]);
    let tran = solve_transient(&circuit, &TransientConfig::new(250e-9, 0.2e-9))
        .unwrap()
        .transient
        .unwrap();
    let crossing = |rising: bool| {
        tran.timesteps
            .windows(2)
            .find_map(|p| {
                let (a, b) = (p[0].node_voltages["y"], p[1].node_voltages["y"]);
                let crossed = if rising {
                    a < 0.5 * VDD && b >= 0.5 * VDD
                } else {
                    a > 0.5 * VDD && b <= 0.5 * VDD
                };
                crossed.then(|| p[0].time + (0.5 * VDD - a) / (b - a) * (p[1].time - p[0].time))
            })
            .unwrap()
    };
    let rise = crossing(true) - 50.05e-9;
    let fall = crossing(false) - 150.15e-9;
    for delay in [rise, fall] {
        assert!((delay / tpd - 1.0).abs() < 0.05, "{rise:e} {fall:e}");
    }
}
