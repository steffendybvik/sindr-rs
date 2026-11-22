//! Terminals on ground, and elements wired the other way round. Each stamp
//! drops the rows and columns of a grounded terminal, so a terminal that is
//! usually live (a controlled source's `out−`, a JFET's source, a
//! transformer's dotted end) is checked here against a closed-form answer
//! with it on ground or with the element reversed.

use sindr::{
    solve_circuit, solve_transient, validate_circuit, Circuit, CircuitElement, JfetKind,
    MosfetKind, RegulatorParams, SimError, SimulationResult, Solver, SolverOptions,
    TransientConfig, Waveform,
};
use sindr_devices::igbt::IgbtParams;
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

fn current(result: &SimulationResult, id: &str) -> f64 {
    result
        .component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("no result for {id}"))
        .current_through
}

fn close(got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol, "{got} vs {want} (±{tol})");
}

/// A VCVS with `out+` on ground drives `out−` to `−gain·Vc`, and one sensing
/// with `ctrl+` on ground sees `−V(in)`.
#[test]
fn vcvs_with_either_output_or_control_terminal_on_ground() {
    let vcvs = |id: &str, nodes: [&str; 2], control: [&str; 2]| CircuitElement::Vcvs {
        id: id.into(),
        nodes: nodes.map(Into::into),
        control_nodes: control.map(Into::into),
        gain: 2.0,
    };
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "in", "0", 1.5),
            r("RIN", "in", "0", 1e3),
            vcvs("E1", ["0", "a"], ["in", "0"]),
            r("RA", "a", "0", 1e3),
            vcvs("E2", ["b", "0"], ["0", "in"]),
            r("RB", "b", "0", 1e3),
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    // The branch rows are exact linear equations: only round-off remains.
    close(result.node_voltages["a"], -3.0, 1e-12);
    close(result.node_voltages["b"], -3.0, 1e-12);
}

/// A VCCS pushes `gm·Vc` from `nodes[0]` to `nodes[1]` through itself, so
/// with either end on ground it sets `±gm·Vc·R` across a load.
#[test]
fn vccs_with_either_output_terminal_on_ground() {
    let vccs = |id: &str, nodes: [&str; 2], control: [&str; 2]| CircuitElement::Vccs {
        id: id.into(),
        nodes: nodes.map(Into::into),
        control_nodes: control.map(Into::into),
        gm: 1e-3,
    };
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "in", "0", 2.0),
            r("RIN", "in", "0", 1e3),
            // Into `a`: +1e-3·2·2k = +4 V.
            vccs("G1", ["0", "a"], ["in", "0"]),
            r("RA", "a", "0", 2e3),
            // Out of `b`, sensing −V(in): pushed back in, +4 V.
            vccs("G2", ["b", "0"], ["0", "in"]),
            r("RB", "b", "0", 2e3),
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    close(result.node_voltages["a"], 4.0, 1e-12);
    close(result.node_voltages["b"], 4.0, 1e-12);
}

/// A VCCS between two live nodes sensing a differential voltage: gm·(V(in)
/// − V(half)) = 1 mA into `c`.
#[test]
fn vccs_with_a_differential_control_between_live_nodes() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "in", "0", 2.0),
            r("R1", "in", "half", 1e3),
            r("R2", "half", "0", 1e3),
            v("V2", "ref", "0", 1.0),
            CircuitElement::Vccs {
                id: "G1".into(),
                nodes: ["ref".into(), "c".into()],
                control_nodes: ["in".into(), "half".into()],
                gm: 1e-3,
            },
            r("RC", "c", "0", 2e3),
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    close(result.node_voltages["c"], 2.0, 1e-12);
    // The same 1 mA is drawn out of `ref`: V2 sources it from its +
    // terminal, which SPICE counts as −1 mA.
    close(result.branch_currents["V2"], -1e-3, 1e-15);
}

/// Current-controlled sources read the controlling source's branch current
/// whichever way their own output is wired.
#[test]
fn current_controlled_sources_with_an_output_on_ground() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("V1", "in", "0", 1.0),
            r("R1", "in", "0", 1e3),
            CircuitElement::Ccvs {
                id: "H1".into(),
                nodes: ["0".into(), "a".into()],
                control_source: "V1".into(),
                rm: 2e3,
            },
            r("RA", "a", "0", 1e3),
            CircuitElement::Cccs {
                id: "F1".into(),
                nodes: ["0".into(), "b".into()],
                control_source: "V1".into(),
                alpha: 3.0,
            },
            r("RB", "b", "0", 1e3),
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    let i_ctrl = result.branch_currents["V1"];
    // 1 mA out of V1's + terminal, which SPICE counts as −1 mA.
    close(i_ctrl, -1e-3, 1e-15);
    // V(0) − V(a) = rm·I, so V(a) = −rm·I = +2 V.
    close(result.node_voltages["a"], -2e3 * i_ctrl, 1e-12);
    // α·I flows from ground into b through F1: V(b) = α·I·RB = −3 V.
    close(result.node_voltages["b"], 3.0 * i_ctrl * 1e3, 1e-12);
}

/// An N-channel JFET with gate and source on ground runs at Vgs = 0, so in
/// saturation it carries exactly Idss.
#[test]
fn jfet_with_gate_and_source_on_ground_carries_idss() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("VDD", "vdd", "0", 20.0),
            r("RD", "vdd", "d", 1e3),
            CircuitElement::Jfet {
                id: "J1".into(),
                nodes: ["0".into(), "d".into(), "0".into()],
                kind: JfetKind::NChannel,
                idss: 10e-3,
                vp: -2.0,
            },
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    // Vds = 10 V > Vgs − Vp = 2 V: saturated, with gds only the 1e-12 S
    // floor, so V(d) = 20 − 1k·10 mA to well under a µV.
    close(result.node_voltages["d"], 10.0, 1e-6);
}

/// A P-channel JFET with its gate tied to its source (Vgs = 0) and its
/// drain on ground carries Idss from source to drain once saturated.
#[test]
fn p_jfet_with_its_drain_on_ground_carries_idss() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("VS", "vs", "0", 10.0),
            r("RS", "vs", "s", 100.0),
            CircuitElement::Jfet {
                id: "J1".into(),
                nodes: ["s".into(), "0".into(), "s".into()],
                kind: JfetKind::PChannel,
                idss: 10e-3,
                vp: 2.0,
            },
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    // Vsd = 9 V > |Vp|: saturated, 10 mA through 100 Ω.
    close(result.node_voltages["s"], 9.0, 1e-6);
}

/// A battery with its + terminal on ground is a negative supply.
#[test]
fn battery_with_its_positive_terminal_on_ground() {
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::Battery {
                id: "B1".into(),
                nodes: ["0".into(), "n".into()],
                params: sindr::battery::BatteryParams::constant(3.0, 1.0, 0.1),
                state_of_charge: 1.0,
            },
            r("RL", "n", "0", 1e3),
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    // 3 V behind 0.1 Ω into 1 kΩ, upside down.
    close(result.node_voltages["n"], -3.0 * 1e3 / (1e3 + 0.1), 1e-12);
}

/// A PMOS whose drain is on ground: the current its model reports is the
/// current its source resistor carries.
#[test]
fn pmos_with_its_drain_on_ground_balances_kcl() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("VDD", "vdd", "0", 5.0),
            r("RS", "vdd", "s", 1e3),
            CircuitElement::Mosfet {
                id: "M1".into(),
                nodes: ["0".into(), "0".into(), "s".into()],
                kind: MosfetKind::Pmos,
                params: MosfetParams::default_pmos(),
                parasitic_caps: None,
                body_diode: None,
            },
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    let vs = result.node_voltages["s"];
    let i_rs = (5.0 - vs) / 1e3;
    assert!(i_rs > 1e-4, "M1 should conduct: {i_rs} A");
    let m1 = result.mosfet_results.iter().find(|m| m.id == "M1").unwrap();
    // A PMOS's drain current is negative: it flows out of the drain.
    assert!(
        (i_rs + m1.id_current).abs() <= 1e-6 * i_rs,
        "RS {i_rs} A vs Id {} A",
        m1.id_current
    );
}

/// An IGBT whose collector is on ground, its emitter pulled to −15 V: the
/// collector current is the emitter resistor's.
///
/// Solved at a tight `reltol`: convergence is judged on node voltages, and
/// with gm ≈ 0.3 S the default 1e-3 (5 mV on V(e)) leaves the reported
/// current about 0.6 % off the resistor's.
#[test]
fn igbt_with_its_collector_on_ground_balances_kcl() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("VEE", "vee", "0", -15.0),
            r("RE", "e", "vee", 1e3),
            CircuitElement::Igbt {
                id: "Z1".into(),
                nodes: ["0".into(), "0".into(), "e".into()],
                params: IgbtParams::default(),
            },
        ],
    );
    let mut options = SolverOptions::default();
    options.reltol = 1e-9;
    options.vntol = 1e-12;
    let result = Solver::new(options).circuit(&circuit).unwrap();
    let ve = result.node_voltages["e"];
    let i_re = (ve + 15.0) / 1e3;
    assert!(i_re > 1e-4, "Z1 should conduct: {i_re} A");
    assert!(
        (i_re - current(&result, "Z1")).abs() <= 1e-8 * i_re,
        "RE {i_re} A vs Ic {} A",
        current(&result, "Z1")
    );
}

/// Swapping a transformer's primary terminals (dotted end on ground)
/// inverts its secondary: the circuit is linear, so step for step the
/// secondary is exactly the negated one.
#[test]
fn reversed_primary_inverts_the_secondary() {
    let transformer = |primary: [&str; 2]| {
        Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["in".into(), "0".into()],
                    voltage: 0.0,
                    waveform: Some(Waveform::Sine {
                        amplitude: 1.0,
                        frequency: 1e3,
                        offset: 0.0,
                        phase: 0.0,
                    }),
                },
                r("R1", "in", "p", 10.0),
                CircuitElement::Transformer {
                    id: "T1".into(),
                    nodes: [primary[0].into(), primary[1].into(), "s".into(), "0".into()],
                    l1: 10e-3,
                    l2: 40e-3,
                    k: 0.99,
                    saturation: None,
                },
                r("RL", "s", "0", 1e3),
            ],
        )
    };
    let config = TransientConfig::new(2e-3, 10e-6);
    let normal = solve_transient(&transformer(["p", "0"]), &config).unwrap();
    let reversed = solve_transient(&transformer(["0", "p"]), &config).unwrap();
    let (normal, reversed) = (normal.transient.unwrap(), reversed.transient.unwrap());
    assert_eq!(normal.timesteps.len(), reversed.timesteps.len());
    let peak = normal
        .timesteps
        .iter()
        .map(|s| s.node_voltages["s"].abs())
        .fold(0.0, f64::max);
    // Turns ratio √(L2/L1) = 2 on a 1 V drive: a secondary near 2 V.
    assert!(peak > 1.0, "secondary peak {peak} V");
    for (a, b) in normal.timesteps.iter().zip(&reversed.timesteps) {
        close(b.node_voltages["s"], -a.node_voltages["s"], 1e-9 * peak);
    }
}

/// A regulator's ground pin lifted 1 V puts its output 1 V higher, and the
/// quiescent current is all that flows in at that pin.
#[test]
fn regulator_with_a_lifted_ground_pin() {
    let lifted = |model| {
        Circuit::new(
            "0",
            vec![
                v("VIN", "in", "0", 12.0),
                v("VG", "g", "0", 1.0),
                CircuitElement::VoltageRegulator {
                    id: "U1".into(),
                    nodes: ["in".into(), "out".into(), "g".into()],
                    voltage: 5.0,
                    model,
                },
                r("RL", "out", "0", 100.0),
            ],
        )
    };
    let ideal = solve_circuit(&lifted(None)).unwrap();
    close(ideal.node_voltages["out"], 6.0, 1e-12);
    // Nothing but the 1e-12 S floor every node gets: 1 V · 1e-12 S.
    close(ideal.branch_currents["VG"], 0.0, 2e-12);

    let lm7805 = RegulatorParams::lm7805();
    let real = solve_circuit(&lifted(Some(lm7805))).unwrap();
    // Line and load regulation move it by millivolts, not the lift.
    close(real.node_voltages["out"], 6.0, 0.05);
    // KCL at g: VG takes in exactly the regulator's quiescent current.
    close(
        real.branch_currents["VG"].abs(),
        lm7805.quiescent_current,
        1e-9,
    );
}

/// A regulator whose input pin is connected to nothing is a floating node,
/// rejected before any solve: it never quietly outputs 0 V.
#[test]
fn regulator_with_a_floating_input_is_rejected() {
    for model in [None, Some(RegulatorParams::lm7805())] {
        let circuit = Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageRegulator {
                    id: "U1".into(),
                    nodes: ["vin".into(), "out".into(), "0".into()],
                    voltage: 5.0,
                    model,
                },
                r("RL", "out", "0", 1e3),
            ],
        );
        for err in [
            validate_circuit(&circuit).unwrap_err(),
            solve_circuit(&circuit).unwrap_err(),
        ] {
            match err {
                SimError::FloatingNode(node) => assert_eq!(node, "vin"),
                other => panic!("expected FloatingNode(vin), got {other:?}"),
            }
        }
    }
}
