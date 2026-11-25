//! `solve_op` and the DC sweeps solve SPICE's operating point: capacitors
//! open, inductors shorted and each source at its t = 0 value, where
//! `solve_circuit` runs a transient for any circuit with C, L or a waveform.

use approx::assert_relative_eq;
use sindr::dc_sweep::DcSweepAxis;
use sindr::element::{Capacitor, Inductor, Resistor, Transformer, VoltageSource};
use sindr::{
    dc_sweep, dc_sweep_nested, solve_circuit, solve_op, Circuit, CircuitElement, Solver,
    SolverOptions, Waveform,
};

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
        ..Default::default()
    }
}

fn v(id: &str, a: &str, b: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [a.into(), b.into()],
        voltage: volts,
        waveform: None,
    })
}

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn l(id: &str, a: &str, b: &str, henries: f64) -> CircuitElement {
    CircuitElement::Inductor(Inductor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        inductance: henries,
        saturation: None,
    })
}

fn c(id: &str, a: &str, b: &str, farads: f64) -> CircuitElement {
    CircuitElement::Capacitor(Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance: farads,
    })
}

/// 1 V through 1 mH into 1 kΩ: the inductor is a short at DC.
fn rl() -> Circuit {
    circuit(vec![
        v("V1", "in", "0", 1.0),
        l("L1", "in", "mid", 1e-3),
        r("R1", "mid", "0", 1e3),
    ])
}

#[test]
fn op_shorts_an_inductor_where_solve_circuit_runs_a_transient() {
    let op = solve_op(&rl()).unwrap();
    assert_eq!(op.node_voltages["mid"], 1.0);
    assert_relative_eq!(op.branch_currents["L1"], 1e-3, max_relative = 1e-12);
    assert!(op.transient.is_none());
    // solve_circuit stops 5 τ into the transient, short of the bias point.
    let tran = solve_circuit(&rl()).unwrap();
    assert!(tran.node_voltages["mid"] < 0.999);
}

#[test]
fn op_opens_a_capacitor() {
    let op = solve_op(&circuit(vec![
        v("V1", "in", "0", 3.0),
        r("R1", "in", "b", 1e3),
        r("R2", "b", "0", 2e3),
        c("C1", "b", "0", 1e-6),
    ]))
    .unwrap();
    assert_relative_eq!(op.node_voltages["b"], 2.0, max_relative = 1e-9);
}

#[test]
fn op_takes_a_waveform_source_at_t_zero() {
    let mut pulse = v("V1", "in", "0", 0.5);
    if let CircuitElement::VoltageSource(VoltageSource { waveform, .. }) = &mut pulse {
        *waveform = Some(Waveform::Pulse {
            v1: 2.0,
            v2: 5.0,
            delay: 1e-6,
            rise_time: 1e-9,
            fall_time: 1e-9,
            pulse_width: 1e-6,
            period: 2e-6,
        });
    }
    let op = solve_op(&circuit(vec![pulse, r("R1", "in", "0", 1e3)])).unwrap();
    // The DC offset plus the pulse's value at t = 0.
    assert_relative_eq!(op.node_voltages["in"], 2.5, max_relative = 1e-12);
}

#[test]
fn op_reports_no_internal_transformer_branch() {
    let op = solve_op(&circuit(vec![
        v("V1", "in", "0", 1.0),
        r("R1", "in", "p", 10.0),
        CircuitElement::Transformer(Transformer {
            id: "T1".into(),
            nodes: ["p".into(), "0".into(), "s".into(), "0".into()],
            l1: 1e-3,
            l2: 4e-3,
            k: 0.99,
            saturation: None,
        }),
        r("R2", "s", "0", 100.0),
    ]))
    .unwrap();
    assert!(op.branch_currents.keys().all(|id| !id.contains('#')));
    assert!(op.component_results.iter().all(|c| !c.id.contains('#')));
    // A shorted primary: the whole 1 V sits across R1.
    assert_relative_eq!(op.node_voltages["p"], 0.0, epsilon = 1e-12);
}

#[test]
fn solver_op_uses_its_options() {
    let solver = Solver::new(SolverOptions::default());
    let op = solver.op(&rl()).unwrap();
    assert_eq!(op.node_voltages["mid"], 1.0);
}

#[test]
fn op_rejects_an_invalid_circuit() {
    let mut bad = rl();
    bad.ground_node = "nowhere".into();
    assert!(solve_op(&bad).is_err());
}

#[test]
fn dc_sweep_solves_operating_points() {
    let sweep = dc_sweep(&rl(), &DcSweepAxis::new("V1", 0.0, 2.0, 5)).unwrap();
    for point in &sweep.points {
        assert_relative_eq!(
            point.result.node_voltages["mid"],
            point.sweep_value,
            epsilon = 1e-12
        );
        assert!(point.result.transient.is_none());
    }
}

/// A swept source is its sweep value alone, even with a waveform, on
/// either axis of a nested sweep.
#[test]
fn swept_sources_drop_their_waveforms() {
    let sine = |id: &str, node: &str| {
        let mut s = v(id, node, "0", 0.0);
        if let CircuitElement::VoltageSource(VoltageSource { waveform, .. }) = &mut s {
            *waveform = Some(Waveform::Sine {
                amplitude: 1.0,
                frequency: 1e3,
                offset: 3.0,
                phase: 0.0,
            });
        }
        s
    };
    let deck = circuit(vec![
        sine("VA", "a"),
        sine("VB", "b"),
        r("R1", "a", "m", 1e3),
        r("R2", "b", "m", 1e3),
        c("C1", "m", "0", 1e-6),
    ]);
    let nested = dc_sweep_nested(
        &deck,
        &DcSweepAxis::new("VA", 0.0, 1.0, 2),
        &DcSweepAxis::new("VB", 0.0, 2.0, 2),
    )
    .unwrap();
    for curve in &nested.curves {
        for point in &curve.sweep.points {
            let want = (point.sweep_value + curve.outer_value) / 2.0;
            assert_relative_eq!(point.result.node_voltages["m"], want, epsilon = 1e-9);
        }
    }
}
