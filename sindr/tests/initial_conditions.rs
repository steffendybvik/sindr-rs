//! `.ic` initial conditions for transient analysis and `.nodeset` seeds,
//! from the Rust API and from SPICE decks.

use std::collections::HashMap;

use sindr::ac_analysis::{solve_ac, solve_ac_with_initial_voltages, AcConfig, FrequencySpacing};
use sindr::{
    solve_transient, BjtKind, Circuit, CircuitElement, IntegrationMethod, TransientConfig,
};

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn last(circuit: &Circuit, config: &TransientConfig) -> sindr::TimestepSnapshot {
    solve_transient(circuit, config)
        .unwrap()
        .transient
        .unwrap()
        .timesteps
        .last()
        .unwrap()
        .clone()
}

/// A capacitor charged to 5 V through `.ic` discharges as 5·e^(−t/τ), and
/// one between two nodes is charged to their difference.
#[test]
fn capacitor_starts_at_its_initial_voltage() {
    let rc = Circuit {
        ground_node: "0".into(),
        components: vec![
            resistor("R1", "a", "b", 1e3),
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["a".into(), "b".into()],
                capacitance: 1e-6,
            },
            resistor("R2", "b", "0", 1e-3),
        ],
    };
    // τ ≈ 1 ms; V(a) − V(b) starts at 5 − 2 = 3 V.
    let ics = HashMap::from([("a".to_string(), 5.0), ("b".to_string(), 2.0)]);
    let config = TransientConfig::new(2e-3, 5e-6).with_initial_voltages(ics);
    let tran = solve_transient(&rc, &config).unwrap().transient.unwrap();
    for s in &tran.timesteps[5..] {
        let v = s.node_voltages["a"] - s.node_voltages["b"];
        let want = 3.0 * (-s.time / (1e3 + 1e-3) / 1e-6).exp();
        assert!(
            (v - want).abs() < 3e-3 * 3.0,
            "t = {}: {v} vs {want}",
            s.time
        );
    }
}

/// An inductor started at 10 mA into a resistor decays as I₀·e^(−tR/L).
#[test]
fn inductor_starts_at_its_initial_current() {
    let rl = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::Inductor {
                id: "L1".into(),
                nodes: ["a".into(), "0".into()],
                inductance: 10e-3,
            },
            resistor("R1", "a", "0", 100.0),
        ],
    };
    let tau = 10e-3 / 100.0;
    let config = TransientConfig::new(tau, tau / 200.0).with_initial_current("L1", 10e-3);
    let s = last(&rl, &config);
    let i = s
        .component_results
        .iter()
        .find(|c| c.id == "L1")
        .unwrap()
        .current_through;
    let want = 10e-3 * (-1.0_f64).exp();
    assert!((i - want).abs() < 1e-3 * 10e-3, "{i} vs {want}");
}

/// An LC tank with its capacitor at 1 V and no source rings between ±1 V
/// (trapezoidal keeps the amplitude).
#[test]
fn lc_tank_rings_from_its_initial_charge() {
    let (l, c): (f64, f64) = (1e-3, 1e-6);
    let period = 2.0 * std::f64::consts::PI * (l * c).sqrt();
    let tank = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::Inductor {
                id: "L1".into(),
                nodes: ["a".into(), "0".into()],
                inductance: l,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["a".into(), "0".into()],
                capacitance: c,
            },
        ],
    };
    let config = TransientConfig::new(3.0 * period, period / 100.0)
        .with_method(IntegrationMethod::Trapezoidal)
        .with_fixed_step()
        .with_initial_voltages(HashMap::from([("a".to_string(), 1.0)]));
    let tran = solve_transient(&tank, &config).unwrap().transient.unwrap();
    for s in &tran.timesteps[1..] {
        let want = (2.0 * std::f64::consts::PI * s.time / period).cos();
        assert!(
            (s.node_voltages["a"] - want).abs() < 2e-2,
            "t = {}: {} vs {want}",
            s.time,
            s.node_voltages["a"]
        );
    }
}

/// A deck's `.ic` and `.nodeset` cards land in the parsed netlist, take
/// `.param` values, and drive the analyses.
#[cfg(feature = "spice")]
#[test]
fn spice_ic_and_nodeset_are_parsed_and_applied() {
    let deck = "\
* rc discharge
.param vinit=5
R1 a 0 1k
C1 a 0 1u
.ic V(a)={vinit}
.nodeset v(a)=2.5
.tran 10u 1m
.end
";
    let netlist = sindr::spice::parse_str(deck).unwrap();
    assert_eq!(
        netlist.initial_conditions,
        HashMap::from([("a".into(), 5.0)])
    );
    assert_eq!(netlist.nodesets, HashMap::from([("a".into(), 2.5)]));

    let config = netlist.analyses[0]
        .transient_config()
        .unwrap()
        .with_initial_voltages(netlist.initial_conditions.clone());
    let v = last(&netlist.circuit, &config).node_voltages["a"];
    let want = 5.0 * (-1.0_f64).exp();
    assert!((v - want).abs() < 1e-2, "{v} vs {want}");
}

#[cfg(feature = "spice")]
#[test]
fn malformed_ic_is_rejected() {
    for card in [".ic", ".ic a=1", ".ic V(a)", ".ic V(a)=", ".nodeset I(a)=1"] {
        let deck = format!("* bad\nR1 a 0 1k\n{card}\n.end\n");
        assert!(
            sindr::spice::parse_str(&deck).is_err(),
            "{card} should fail"
        );
    }
}

/// AC needs the DC bias point; a nodeset seeds it and changes nothing else.
#[test]
fn ac_bias_point_takes_a_nodeset() {
    let stage = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "VIN".into(),
                nodes: ["b".into(), "0".into()],
                voltage: 0.7,
                waveform: None,
            },
            CircuitElement::VoltageSource {
                id: "VCC".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            resistor("RC", "vcc", "c", 1e3),
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
            },
        ],
    };
    let config = AcConfig {
        f_start: 1e3,
        f_stop: 1e3,
        num_points: 1,
        spacing: FrequencySpacing::Linear,
        source_id: "VIN".into(),
        ac_magnitude: 1.0,
    };
    let plain = solve_ac(&stage, &config).unwrap().points[0].node_voltages["c"];
    let seed = HashMap::from([("c".to_string(), 5.0)]);
    let seeded = solve_ac_with_initial_voltages(&stage, &config, &seed)
        .unwrap()
        .points[0]
        .node_voltages["c"];
    assert!(
        (plain - seeded).norm() < 1e-6 * plain.norm(),
        "{plain} vs {seeded}"
    );
}
