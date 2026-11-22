//! Accessors on the result types: node voltages, component currents and
//! device results by name, inside block instances by path, and the AC,
//! transient and sweep curves.

use approx::assert_relative_eq;
use sindr::{
    dc_sweep, solve_ac, solve_circuit, solve_transient, temperature_sweep, AcConfig, BjtKind,
    Block, Circuit, CircuitElement, Instance, MosfetKind, TransientConfig,
};
use sindr_devices::mosfet::MosfetParams;

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    }
}

fn v(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    }
}

fn c(id: &str, a: &str, b: &str, farads: f64) -> CircuitElement {
    CircuitElement::Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance: farads,
    }
}

/// 10 V across 1 kΩ + 4 kΩ: 8 V at `out`, 2 mA.
fn divider() -> Circuit {
    Circuit::new(
        "gnd",
        vec![
            v("V1", "in", "gnd", 10.0),
            r("R1", "in", "out", 1e3),
            r("R2", "out", "gnd", 4e3),
        ],
    )
}

#[test]
fn voltage_reads_nodes_and_ground() {
    let result = solve_circuit(&divider()).unwrap();
    assert_relative_eq!(result.voltage("out").unwrap(), 8.0, epsilon = 1e-9);
    assert_relative_eq!(result.voltage("in").unwrap(), 10.0, epsilon = 1e-9);
    assert_eq!(result.voltage("gnd"), Some(0.0));
    assert_eq!(result.voltage("missing"), None);
    // Names are exact: no case folding, no aliasing of ground.
    assert_eq!(result.voltage("OUT"), None);
    assert_eq!(result.voltage("0"), None);
}

#[test]
fn current_component_and_branch_current() {
    let result = solve_circuit(&divider()).unwrap();
    assert_relative_eq!(result.current("R1").unwrap(), 2e-3, epsilon = 1e-12);
    assert_relative_eq!(result.current("R2").unwrap(), 2e-3, epsilon = 1e-12);
    // The source's component current is its MNA branch current: negative
    // while it delivers power.
    assert_relative_eq!(result.current("V1").unwrap(), -2e-3, epsilon = 1e-12);
    assert_relative_eq!(result.branch_current("V1").unwrap(), -2e-3, epsilon = 1e-12);
    // A resistor has no current unknown.
    assert_eq!(result.branch_current("R1"), None);

    let r2 = result.component("R2").unwrap();
    assert_relative_eq!(r2.voltage_across, 8.0, epsilon = 1e-9);
    assert_relative_eq!(r2.power, 16e-3, epsilon = 1e-12);

    assert_eq!(result.current("R9"), None);
    assert!(result.component("R9").is_none());
    assert_eq!(result.branch_current("V9"), None);
}

#[test]
fn device_results_by_id() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("VCC", "vcc", "0", 10.0),
            v("VB", "vb", "0", 0.7),
            r("RB", "vb", "b", 1e3),
            r("RC", "vcc", "c", 1e3),
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            },
            v("VG", "g", "0", 3.0),
            r("RD", "vcc", "d", 1e3),
            CircuitElement::Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind: MosfetKind::Nmos,
                params: MosfetParams::default_nmos(),
                parasitic_caps: None,
                body_diode: None,
            },
            CircuitElement::OpAmp {
                id: "U1".into(),
                nodes: ["vb".into(), "out".into(), "out".into()],
                v_pos: 15.0,
                v_neg: -15.0,
                model: None,
            },
            r("RL", "out", "0", 1e4),
            CircuitElement::Relay {
                id: "K1".into(),
                nodes: ["vcc".into(), "0".into(), "k_a".into(), "k_b".into()],
                coil_resistance: 500.0,
                pickup_voltage: 5.0,
                inductance: 0.0,
            },
            r("RK1", "k_a", "0", 1e3),
            r("RK2", "k_b", "0", 1e3),
        ],
    );
    let result = solve_circuit(&circuit).unwrap();

    let q1 = result.bjt("Q1").unwrap();
    assert_eq!(q1.id, "Q1");
    assert_eq!(result.current("Q1"), Some(q1.ic));
    assert!(result.bjt("M1").is_none());

    let m1 = result.mosfet("M1").unwrap();
    assert_eq!(result.current("M1"), Some(m1.id_current));
    assert!(result.mosfet("Q1").is_none());

    let u1 = result.op_amp("U1").unwrap();
    assert_relative_eq!(u1.v_out, 0.7, epsilon = 1e-4);
    assert!(result.op_amp("U2").is_none());

    assert!(result.relay("K1").unwrap().contact_closed);
    assert!(result.relay("K2").is_none());
}

/// `outer` holds a resistor and an instance of `inner`, a 1k–1k divider
/// with an internal node `mid` and a block-local `node` and ground.
fn nested() -> Circuit {
    let inner = Block {
        ports: vec!["a".into()],
        components: vec![r("R1", "a", "mid", 1e3), r("R2", "mid", "0", 1e3)],
        ..Block::default()
    };
    let outer = Block {
        ports: vec!["p".into()],
        components: vec![r("R3", "p", "n", 1e3)],
        instances: vec![Instance::new("X2", "inner", ["n"])],
        ..Block::default()
    };
    let mut circuit = Circuit::new("0", vec![v("V1", "in", "0", 3.0)]);
    circuit.blocks.insert("inner".into(), inner);
    circuit.blocks.insert("outer".into(), outer);
    circuit.instances.push(Instance::new("X1", "outer", ["in"]));
    circuit
}

#[test]
fn instance_reads_local_nets_and_components() {
    let result = solve_circuit(&nested()).unwrap();
    // 3 V across 3 kΩ: 1 mA everywhere, 2 V at X1.n, 1 V at X1.X2.mid.
    let x1 = result.instance("X1");
    assert_eq!(x1.path(), "X1");
    assert_relative_eq!(x1.voltage("n").unwrap(), 2.0, epsilon = 1e-9);
    assert_relative_eq!(x1.current("R3").unwrap(), 1e-3, epsilon = 1e-12);
    assert_relative_eq!(
        x1.component("R3").unwrap().voltage_across,
        1.0,
        epsilon = 1e-9
    );
    assert_eq!(x1.name("n"), "X1.n");

    // A port is the caller's net, and ground keeps its own name.
    assert_eq!(x1.voltage("p"), None);
    assert_eq!(x1.voltage("0"), None);
    assert_eq!(result.voltage("0"), Some(0.0));
    // Nothing at a name the block doesn't have.
    assert_eq!(x1.voltage("mid"), None);
    assert_eq!(x1.current("R1"), None);
}

#[test]
fn nested_instances_by_chain_or_path() {
    let result = solve_circuit(&nested()).unwrap();
    let chained = result.instance("X1").instance("X2");
    let direct = result.instance("X1.X2");
    assert_eq!(chained.path(), "X1.X2");
    for x2 in [&chained, &direct] {
        assert_relative_eq!(x2.voltage("mid").unwrap(), 1.0, epsilon = 1e-9);
        assert_relative_eq!(x2.current("R2").unwrap(), 1e-3, epsilon = 1e-12);
        assert!(x2.component("R1").is_some());
    }
    assert_eq!(
        result.voltage("X1.X2.mid"),
        result.instance("X1.X2").voltage("mid")
    );
    // The flat name reads AC, sweep and transient results.
    assert_eq!(chained.name("mid"), "X1.X2.mid");
}

#[test]
fn missing_instance_answers_none() {
    let result = solve_circuit(&nested()).unwrap();
    let ghost = result.instance("X9");
    assert_eq!(ghost.voltage("n"), None);
    assert_eq!(ghost.current("R3"), None);
    assert_eq!(ghost.branch_current("R3"), None);
    assert!(ghost.component("R3").is_none());
    assert!(ghost.bjt("Q1").is_none());
    assert!(ghost.mosfet("M1").is_none());
    assert!(ghost.op_amp("U1").is_none());
    assert!(ghost.relay("K1").is_none());
    assert_eq!(result.instance("X1").instance("X9").voltage("mid"), None);
}

#[test]
fn empty_instance_path_is_the_top_level() {
    let result = solve_circuit(&divider()).unwrap();
    let top = result.instance("");
    assert_eq!(top.voltage("out"), result.voltage("out"));
    assert_eq!(top.name("out"), "out");
}

fn rc() -> Circuit {
    Circuit::new(
        "0",
        vec![
            v("V1", "in", "0", 5.0),
            r("R1", "in", "out", 1e3),
            c("C1", "out", "0", 1e-6),
        ],
    )
}

#[test]
fn transient_curves_and_values_at_a_time() {
    let result = solve_transient(&rc(), &TransientConfig::new(5e-3, 1e-5)).unwrap();
    let tran = result.transient.as_ref().unwrap();

    let wave = tran.node_voltage_curve("out");
    assert_eq!(wave.len(), tran.timesteps.len());
    assert_eq!(wave[0].0, tran.timesteps[0].time);
    assert!(tran.node_voltage_curve("missing").is_empty());

    let ground = tran.node_voltage_curve("0");
    assert!(ground.iter().all(|&(_, v)| v == 0.0));

    // V(out) = 5(1 − e^(−t/τ)), I(C1) = (5/R)·e^(−t/τ), τ = 1 ms.
    let v_tau = tran.voltage_at("out", 1e-3).unwrap();
    assert_relative_eq!(v_tau, 5.0 * (1.0 - (-1.0f64).exp()), epsilon = 0.02);
    let i_tau = tran.current_at("R1", 1e-3).unwrap();
    assert_relative_eq!(i_tau, 5e-3 * (-1.0f64).exp(), epsilon = 2e-5);
    assert_eq!(tran.component_current_curve("R1").len(), wave.len());
    assert!(tran.component_current_curve("R9").is_empty());

    assert_eq!(tran.voltage_at("out", -1.0), None);
    assert_eq!(tran.voltage_at("out", 1.0), None);
    assert_eq!(tran.voltage_at("missing", 1e-3), None);
    assert_eq!(tran.current_at("R9", 1e-3), None);

    // A step's own values, and the end state on the result.
    let last = tran.timesteps.last().unwrap();
    assert_eq!(tran.voltage_at("out", last.time), last.voltage("out"));
    assert_eq!(last.voltage("0"), Some(0.0));
    assert_eq!(last.voltage("missing"), None);
    assert_eq!(
        last.current("R1"),
        last.component("R1").map(|c| c.current_through)
    );
    assert_eq!(last.current("R9"), None);
    assert!(last.component("R9").is_none());
    assert_eq!(result.voltage("out"), last.voltage("out"));
}

#[test]
fn ac_points_and_curves() {
    let ac = solve_ac(&rc(), &AcConfig::new("V1", 1.0, 1e5, 41)).unwrap();
    // Corner at 1/(2π·RC): |H| = 1/√2, −45°.
    let f_c = 1.0 / (2.0 * std::f64::consts::PI * 1e-3);
    let corner = ac
        .points
        .iter()
        .min_by(|a, b| {
            (a.frequency - f_c)
                .abs()
                .partial_cmp(&(b.frequency - f_c).abs())
                .unwrap()
        })
        .unwrap();
    let h = 1.0 / (1.0 + (corner.frequency / f_c).powi(2)).sqrt();
    assert_relative_eq!(corner.magnitude("out").unwrap(), h, epsilon = 1e-6);
    let vout = corner.voltage("out").unwrap();
    assert_relative_eq!(vout.norm(), h, epsilon = 1e-6);
    assert_relative_eq!(
        vout.arg().to_degrees(),
        corner.phase_deg("out").unwrap(),
        epsilon = 1e-12
    );
    assert_eq!(corner.voltage("0").map(|v| v.norm()), Some(0.0));
    assert_eq!(corner.voltage("missing"), None);
    assert_eq!(corner.magnitude("missing"), None);

    let curve = ac.node_voltage_curve("out");
    let mags = ac.magnitude_curve("out");
    assert_eq!(curve.len(), 41);
    assert_eq!(mags.len(), 41);
    for ((f, v), (fm, m)) in curve.iter().zip(&mags) {
        assert_eq!(f, fm);
        assert_eq!(v.norm(), *m);
    }
    for ((_, m), (_, db)) in mags.iter().zip(ac.gain_curve("out")) {
        assert_relative_eq!(20.0 * m.log10(), db, epsilon = 1e-9);
    }
    assert!(ac.node_voltage_curve("missing").is_empty());
    assert!(ac.magnitude_curve("missing").is_empty());
}

#[test]
fn sweep_points_use_the_same_accessors() {
    let sweep = dc_sweep(&divider(), "V1", 0.0, 10.0, 11).unwrap();
    let curve = sweep.node_voltage_curve("out");
    assert_eq!(curve.len(), 11);
    for (p, (x, vout)) in sweep.points.iter().zip(&curve) {
        assert_eq!(*x, p.sweep_value);
        assert_eq!(p.result.voltage("out"), Some(*vout));
        assert_relative_eq!(*vout, 0.8 * x, epsilon = 1e-9);
    }
    assert!(sweep.node_voltage_curve("missing").is_empty());

    let temps = temperature_sweep(&divider(), 250.0, 350.0, 3).unwrap();
    let curve = temps.node_voltage_curve("out");
    assert_eq!(curve.len(), temps.points.len());
    assert_eq!(curve[0].0, 250.0);
    for &(_, vout) in &curve {
        assert_relative_eq!(vout, 8.0, epsilon = 1e-9);
    }
    let currents = temps.component_current_curve("R1");
    assert!(currents.iter().all(|&(_, i)| (i - 2e-3).abs() < 1e-12));
    assert!(temps.node_voltage_curve("missing").is_empty());
    assert!(temps.component_current_curve("R9").is_empty());
}

/// A parsed deck's names are lowercase, and nested `X` lines give the same
/// instance paths as a design hierarchy.
#[cfg(feature = "spice")]
#[test]
fn spice_subckt_instances() {
    let netlist = sindr::spice::parse_str(
        "* nested
V1 in 0 3
X1 in stage
.subckt inner a
R1 a mid 1k
R2 mid 0 1k
.ends
.subckt stage p
R3 p n 1k
X2 n inner
.ends
.end
",
    )
    .unwrap();
    let result = solve_circuit(&netlist.circuit).unwrap();
    let x2 = result.instance("x1").instance("x2");
    assert_relative_eq!(x2.voltage("mid").unwrap(), 1.0, epsilon = 1e-9);
    assert_relative_eq!(x2.current("r1").unwrap(), 1e-3, epsilon = 1e-12);
    assert_relative_eq!(
        result.instance("x1").voltage("n").unwrap(),
        2.0,
        epsilon = 1e-9
    );
    // The source map names the same nodes.
    let path = &netlist.source_map["x1.x2.mid"];
    let by_path = result
        .instance(&path.instance_path.join("."))
        .voltage(&path.original_node);
    assert_eq!(by_path, result.voltage("x1.x2.mid"));
    assert_eq!(result.instance("X1").voltage("n"), None);
}
