//! Solve-level integration tests for circuit elements whose MNA stamping was
//! previously only unit-tested or untested end-to-end: the four controlled
//! sources, the ideal voltage regulator, fuse, switch, and potentiometer.
//!
//! Each test builds a small circuit, runs `solve_circuit`, and checks the
//! element's defining behaviour against hand-computed values. Where a current
//! direction depends on MNA branch-sign conventions the assertion is on the
//! magnitude, which is what the device's defining equation pins down.

use approx::assert_relative_eq;
use sindr::{solve_circuit, Circuit, CircuitElement, SimError, SimulationResult};

fn vsrc(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn node(r: &SimulationResult, n: &str) -> f64 {
    *r.node_voltages
        .get(n)
        .unwrap_or_else(|| panic!("no node `{n}` in solution"))
}

fn comp_current(r: &SimulationResult, id: &str) -> f64 {
    r.component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("no component `{id}` in results"))
        .current_through
}

// ----- Controlled sources -------------------------------------------------

#[test]
fn vcvs_output_is_gain_times_control_voltage() {
    // V(c) = 2 V, gain = 4  ->  V(out) = 8 V.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "c", "0", 2.0),
            CircuitElement::Vcvs {
                id: "E1".into(),
                nodes: ["out".into(), "0".into()],
                control_nodes: ["c".into(), "0".into()],
                gain: 4.0,
            },
            resistor("Rload", "out", "0", 1_000.0),
        ],
    };
    let r = solve_circuit(&circuit).expect("solve vcvs");
    assert_relative_eq!(node(&r, "c"), 2.0, epsilon = 1e-9);
    assert_relative_eq!(node(&r, "out"), 8.0, epsilon = 1e-6);
    // Defining law holds on the solved solution.
    assert_relative_eq!(node(&r, "out"), 4.0 * node(&r, "c"), epsilon = 1e-6);
}

#[test]
fn vccs_load_current_is_gm_times_control_voltage() {
    // I_out = gm * V(c) = 0.002 S * 2 V = 4 mA, across a 500 Ω load -> 2 V.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "c", "0", 2.0),
            CircuitElement::Vccs {
                id: "G1".into(),
                nodes: ["out".into(), "0".into()],
                control_nodes: ["c".into(), "0".into()],
                gm: 0.002,
            },
            resistor("Rload", "out", "0", 500.0),
        ],
    };
    let r = solve_circuit(&circuit).expect("solve vccs");
    assert_relative_eq!(comp_current(&r, "Rload").abs(), 0.002 * 2.0, epsilon = 1e-6);
    assert_relative_eq!(node(&r, "out").abs(), 0.002 * 2.0 * 500.0, epsilon = 1e-6);
}

#[test]
fn ccvs_output_is_rm_times_control_current() {
    // Controlling source V1 carries 1 V / 100 Ω = 10 mA. rm = 200 Ω -> |V_out| = 2 V.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "c", "0", 1.0),
            resistor("Rsense", "c", "0", 100.0),
            CircuitElement::Ccvs {
                id: "H1".into(),
                nodes: ["out".into(), "0".into()],
                control_source: "V1".into(),
                rm: 200.0,
            },
            resistor("Rload", "out", "0", 1_000.0),
        ],
    };
    let r = solve_circuit(&circuit).expect("solve ccvs");
    let i_ctrl = r.branch_currents["V1"].abs();
    assert_relative_eq!(i_ctrl, 0.01, epsilon = 1e-6);
    assert_relative_eq!(node(&r, "out").abs(), 200.0 * i_ctrl, epsilon = 1e-6);
}

#[test]
fn cccs_output_current_is_alpha_times_control_current() {
    // I_ctrl = 10 mA, alpha = 5 -> I_out = 50 mA across a 40 Ω load -> |V_out| = 2 V.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "c", "0", 1.0),
            resistor("Rsense", "c", "0", 100.0),
            CircuitElement::Cccs {
                id: "F1".into(),
                nodes: ["out".into(), "0".into()],
                control_source: "V1".into(),
                alpha: 5.0,
            },
            resistor("Rload", "out", "0", 40.0),
        ],
    };
    let r = solve_circuit(&circuit).expect("solve cccs");
    assert_relative_eq!(comp_current(&r, "Rload").abs(), 5.0 * 0.01, epsilon = 1e-6);
    assert_relative_eq!(node(&r, "out").abs(), 5.0 * 0.01 * 40.0, epsilon = 1e-6);
}

// ----- Voltage regulator --------------------------------------------------

#[test]
fn voltage_regulator_holds_output_independent_of_input() {
    let make = |vin: f64| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("Vin", "in", "0", vin),
            CircuitElement::VoltageRegulator {
                id: "U1".into(),
                nodes: ["in".into(), "out".into(), "0".into()],
                voltage: 5.0,
            },
            resistor("Rload", "out", "0", 1_000.0),
        ],
    };
    for vin in [9.0, 12.0, 15.0] {
        let r = solve_circuit(&make(vin)).expect("solve regulator");
        assert_relative_eq!(node(&r, "out"), 5.0, epsilon = 1e-6);
    }
}

// ----- Switch -------------------------------------------------------------

#[test]
fn switch_closed_shorts_open_blocks() {
    // 10 V -> 1 kΩ -> node `n` -> switch -> gnd.
    let make = |closed: bool| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            resistor("R1", "t", "n", 1_000.0),
            CircuitElement::Switch {
                id: "S1".into(),
                nodes: ["n".into(), "0".into()],
                closed,
            },
        ],
    };
    let closed = solve_circuit(&make(true)).expect("closed");
    let open = solve_circuit(&make(false)).expect("open");
    assert!(
        node(&closed, "n") < 0.01,
        "closed switch should pull `n` near 0 V, got {}",
        node(&closed, "n")
    );
    assert!(
        node(&open, "n") > 9.99,
        "open switch should leave `n` near the 10 V supply, got {}",
        node(&open, "n")
    );
}

// ----- Fuse ---------------------------------------------------------------

#[test]
fn fuse_intact_conducts_blown_opens() {
    let make = |blown: bool| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            resistor("R1", "t", "n", 1_000.0),
            CircuitElement::Fuse {
                id: "F1".into(),
                nodes: ["n".into(), "0".into()],
                rating: 1.0,
                blown,
            },
        ],
    };
    let intact = solve_circuit(&make(false)).expect("intact");
    let blown = solve_circuit(&make(true)).expect("blown");
    assert!(
        node(&intact, "n") < 0.02,
        "intact fuse should pull `n` near 0 V, got {}",
        node(&intact, "n")
    );
    assert!(
        node(&blown, "n") > 9.99,
        "blown fuse should leave `n` near the 10 V supply, got {}",
        node(&blown, "n")
    );
}

// ----- Potentiometer ------------------------------------------------------

#[test]
fn potentiometer_loaded_wiper_divides_voltage() {
    // top = 10 V, bottom = gnd, R = 1 kΩ, with a 750 Ω load on the wiper.
    // At position p the upper half is R*p (top→wiper) and the lower half is
    // R*(1-p) (wiper→gnd), which sits in parallel with the load.
    //   p = 0.25: r_top = 250, r_bot = 750 ∥ 750 = 375  -> V_w = 10·375/625 = 6.00 V
    //   p = 0.50: r_top = 500, r_bot = 500 ∥ 750 = 300  -> V_w = 10·300/800 = 3.75 V
    let make = |pos: f64| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            CircuitElement::Potentiometer {
                id: "P1".into(),
                nodes: ["t".into(), "w".into(), "0".into()],
                resistance: 1_000.0,
                position: pos,
            },
            resistor("Rload", "w", "0", 750.0),
        ],
    };
    let r25 = solve_circuit(&make(0.25)).expect("p=0.25");
    let r50 = solve_circuit(&make(0.50)).expect("p=0.50");
    assert_relative_eq!(node(&r25, "w"), 6.0, epsilon = 1e-6);
    assert_relative_eq!(node(&r50, "w"), 3.75, epsilon = 1e-6);
}

#[test]
fn potentiometer_unloaded_wiper_is_rejected_as_floating() {
    // Known limitation: validation counts component-terminal incidences per
    // node, so a potentiometer whose wiper has no external connection looks
    // like a degree-1 (floating) node — even though the wiper is internally
    // connected to both ends through the two resistor halves and the circuit
    // is solvable. Locked in as a regression test; if validation learns about
    // a pot's internal wiper connection this should flip to a solve.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            CircuitElement::Potentiometer {
                id: "P1".into(),
                nodes: ["t".into(), "w".into(), "0".into()],
                resistance: 1_000.0,
                position: 0.5,
            },
        ],
    };
    match solve_circuit(&circuit) {
        Err(SimError::FloatingNode(node)) => assert_eq!(node, "w"),
        other => panic!("expected FloatingNode(\"w\"), got {other:?}"),
    }
}
