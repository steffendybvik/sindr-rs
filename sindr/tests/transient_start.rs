//! Where a transient starts: from the DC operating point
//! (`TransientStart::OperatingPoint`, SPICE's `.tran` without `UIC`) or from
//! the initial conditions alone (`TransientStart::InitialConditions`, `UIC`).

use sindr::element::{Capacitor, Inductor, OpAmp, Resistor, Transformer, VoltageSource};
use std::collections::HashMap;

use sindr::{
    solve_transient, Circuit, CircuitElement, OpAmpModel, TransientConfig, TransientData,
    TransientStart, Waveform,
};

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn v(id: &str, a: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [a.into(), "0".into()],
        voltage: volts,
        waveform: None,
    })
}

fn c(id: &str, a: &str, farads: f64) -> CircuitElement {
    CircuitElement::Capacitor(Capacitor {
        id: id.into(),
        nodes: [a.into(), "0".into()],
        capacitance: farads,
    })
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
        ..Default::default()
    }
}

fn run(circuit: &Circuit, config: &TransientConfig) -> TransientData {
    solve_transient(circuit, config).unwrap().transient.unwrap()
}

fn at_op(tstop: f64, tstep: f64) -> TransientConfig {
    TransientConfig::new(tstop, tstep).with_start(TransientStart::OperatingPoint)
}

fn current(tran: &TransientData, step: usize, id: &str) -> f64 {
    tran.timesteps[step]
        .component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap()
        .current_through
}

/// Every snapshot of `node` is within `tol` of `want`.
fn flat(tran: &TransientData, node: &str, want: f64, tol: f64) {
    for s in &tran.timesteps {
        let got = s.node_voltages[node];
        assert!(
            (got - want).abs() < tol,
            "{node} = {got} at t = {}, want {want}",
            s.time
        );
    }
}

#[test]
fn the_default_is_switched_on_at_zero() {
    assert_eq!(
        TransientConfig::new(1.0, 0.1).start,
        TransientStart::InitialConditions
    );
}

/// A charged RC divider stays where it is; switched on, it charges.
#[test]
fn a_settled_rc_stays_settled() {
    let rc = circuit(vec![
        v("V1", "in", 10.0),
        r("R1", "in", "a", 1e3),
        r("R2", "a", "0", 1e3),
        c("C1", "a", 1e-6),
    ]);
    flat(&run(&rc, &at_op(2e-3, 1e-5)), "a", 5.0, 1e-6);

    let switched_on = run(&rc, &TransientConfig::new(2e-3, 1e-5));
    assert!(switched_on.timesteps[0].node_voltages["a"] < 0.1);
}

/// An inductor starts with its operating-point current, V/R.
#[test]
fn an_inductor_starts_with_its_dc_current() {
    let rl = circuit(vec![
        v("V1", "in", 1.0),
        r("R1", "in", "a", 10.0),
        CircuitElement::Inductor(Inductor {
            id: "L1".into(),
            nodes: ["a".into(), "0".into()],
            inductance: 10e-3,
            saturation: None,
        }),
    ]);
    let tran = run(&rl, &at_op(2e-3, 1e-5));
    for k in 0..tran.timesteps.len() {
        let i = current(&tran, k, "L1");
        assert!((i - 0.1).abs() < 1e-9, "i(L1) = {i} at step {k}");
    }
    flat(&tran, "a", 0.0, 1e-9);
}

/// `initial_currents` holds an inductor at its current while the operating
/// point is solved: here 50 mA, rising to V/R = 100 mA with τ = L/R = 1 ms.
#[test]
fn an_inductor_current_is_held_at_the_operating_point() {
    let rl = circuit(vec![
        v("V1", "in", 1.0),
        r("R1", "in", "a", 10.0),
        CircuitElement::Inductor(Inductor {
            id: "L1".into(),
            nodes: ["a".into(), "0".into()],
            inductance: 10e-3,
            saturation: None,
        }),
    ]);
    let tran = run(&rl, &at_op(1e-3, 1e-6).with_initial_current("L1", 0.05));
    // Held at 50 mA, the 10 Ω drops 0.5 V, so `a` starts at 0.5 V.
    let first = &tran.timesteps[0];
    assert!((current(&tran, 0, "L1") - 0.05).abs() < 1e-3);
    assert!((first.node_voltages["a"] - 0.5).abs() < 1e-2);
    let last = tran.timesteps.len() - 1;
    let want = 0.1 - 0.05 * (-1.0_f64).exp();
    let got = current(&tran, last, "L1");
    assert!((got - want).abs() < 1e-4, "i(L1) = {got}, want {want}");
}

/// A node in `.ic` is held at its value for the operating point, and the
/// run relaxes from there: 2 V towards the divider's 5 V, τ = 0.5 ms.
#[test]
fn an_ic_node_is_held_at_the_operating_point() {
    let rc = circuit(vec![
        v("V1", "in", 10.0),
        r("R1", "in", "a", 1e3),
        r("R2", "a", "0", 1e3),
        c("C1", "a", 1e-6),
    ]);
    let ic = HashMap::from([("a".to_string(), 2.0)]);
    let tran = run(&rc, &at_op(0.5e-3, 1e-6).with_initial_voltages(ic));
    assert!((tran.timesteps[0].node_voltages["a"] - 2.0).abs() < 0.01);
    let got = tran.timesteps.last().unwrap().node_voltages["a"];
    let want = 5.0 - 3.0 * (-1.0_f64).exp();
    assert!((got - want).abs() < 1e-3, "v(a) = {got}, want {want}");
}

/// The operating point takes each source at t = 0: a pulse's low value,
/// here 1 V until the pulse at 1 ms.
#[test]
fn a_waveform_source_starts_at_its_t0_value() {
    let pulse = circuit(vec![
        CircuitElement::VoltageSource(VoltageSource {
            id: "V1".into(),
            nodes: ["in".into(), "0".into()],
            voltage: 0.0,
            waveform: Some(Waveform::Pulse {
                v1: 1.0,
                v2: 5.0,
                delay: 1e-3,
                rise_time: 1e-6,
                fall_time: 1e-6,
                pulse_width: 10e-3,
                period: 20e-3,
            }),
        }),
        r("R1", "in", "a", 1e3),
        c("C1", "a", 1e-6),
    ]);
    let tran = run(&pulse, &at_op(1e-3, 1e-5));
    flat(&tran, "a", 1.0, 1e-6);
}

/// Transformer windings carry their operating-point currents: a DC current
/// through the primary induces nothing in the secondary. Switched on, the
/// current's rise does.
#[test]
fn transformer_windings_start_with_their_dc_currents() {
    let xfmr = circuit(vec![
        v("V1", "in", 5.0),
        r("R1", "in", "p", 10.0),
        CircuitElement::Transformer(Transformer {
            id: "T1".into(),
            nodes: ["p".into(), "0".into(), "s".into(), "0".into()],
            l1: 10e-3,
            l2: 10e-3,
            k: 0.99,
            saturation: None,
        }),
        r("RL", "s", "0", 100.0),
    ]);
    let tran = run(&xfmr, &at_op(1e-3, 1e-6));
    flat(&tran, "s", 0.0, 1e-6);
    for k in 0..tran.timesteps.len() {
        let i = current(&tran, k, "T1");
        assert!((i - 0.5).abs() < 1e-6, "i(T1) = {i} at step {k}");
    }

    let switched_on = run(&xfmr, &TransientConfig::new(1e-3, 1e-6));
    let peak = switched_on
        .timesteps
        .iter()
        .map(|s| s.node_voltages["s"].abs())
        .fold(0.0, f64::max);
    assert!(peak > 0.1, "secondary peak {peak} V");
}

/// An op amp's gain stage starts settled, so a buffer's output is at its
/// input from the first step rather than slewing up from 0 V.
#[test]
fn an_op_amp_stage_starts_settled() {
    let buffer = circuit(vec![
        v("V1", "in", 2.0),
        CircuitElement::OpAmp(OpAmp {
            id: "U1".into(),
            nodes: ["in".into(), "out".into(), "out".into()],
            v_pos: 15.0,
            v_neg: -15.0,
            model: Some(OpAmpModel::default()),
        }),
        r("RL", "out", "0", 10e3),
    ]);
    flat(&run(&buffer, &at_op(10e-6, 1e-7)), "out", 2.0, 1e-3);

    // Switched on, a 0.5 V/µs slew takes about 4 µs to reach 2 V.
    let switched_on = run(&buffer, &TransientConfig::new(10e-6, 1e-7));
    assert!(switched_on.timesteps[0].node_voltages["out"] < 0.5);
}

#[cfg(feature = "spice")]
mod spice {
    use super::*;
    use sindr::spice::{parse_str, AnalysisRequest};

    const RC: &str = "* rc\nV1 in 0 10\nR1 in a 1k\nR2 a 0 1k\nC1 a 0 1u\n";

    fn config(deck: &str) -> TransientConfig {
        let netlist = parse_str(deck).unwrap();
        netlist.analyses[0].transient_config().unwrap()
    }

    /// `.tran` starts from the operating point, as in SPICE; `uic` from the
    /// initial conditions.
    #[test]
    fn tran_starts_at_the_operating_point_unless_uic() {
        let plain = config(&format!("{RC}.tran 10u 2m\n.end\n"));
        assert_eq!(plain.start, TransientStart::OperatingPoint);
        assert_eq!(plain.tmax, None);

        let uic = config(&format!("{RC}.TRAN 10u 2m 0 1u UIC\n.end\n"));
        assert_eq!(uic.start, TransientStart::InitialConditions);
        assert_eq!(uic.tmax, Some(1e-6));

        let netlist = parse_str(&format!("{RC}.tran 10u 2m\n.end\n")).unwrap();
        flat(&run(&netlist.circuit, &plain), "a", 5.0, 1e-6);
        assert!(run(&netlist.circuit, &uic).timesteps[0].node_voltages["a"] < 0.1);
    }

    #[test]
    fn tran_reads_tmax_and_uic() {
        let netlist = parse_str(&format!("{RC}.tran 1u 2m 0 1u uic\n.end\n")).unwrap();
        assert_eq!(
            netlist.analyses,
            vec![AnalysisRequest::Tran {
                tstep: 1e-6,
                tstop: 2e-3,
                tstart: Some(0.0),
                tmax: Some(1e-6),
                uic: true,
            }]
        );
        assert!(parse_str(&format!("{RC}.tran 10u 2m sometimes\n.end\n")).is_err());
    }

    /// The deck's `.tran` carries its `tstart` and, through
    /// `ParsedNetlist::transient_config`, its `.ic` cards.
    #[test]
    fn deck_config_carries_tstart_and_ic() {
        let netlist =
            parse_str(&format!("{RC}.ic v(a)=8\n.op\n.tran 10u 2m 0.5m\n.end\n")).unwrap();
        let config = netlist.transient_config().unwrap();
        assert_eq!(config.tstart, 0.5e-3);
        assert_eq!(config.initial_voltages, netlist.initial_conditions);
        assert_eq!(config.start, TransientStart::OperatingPoint);
        // Without the .ic the analysis's own config starts `a` at the
        // divider's 5 V; with it, C1 starts at 8 V and has decayed most of
        // the way back by 0.5 ms (τ = 0.5 ms).
        let tran = run(&netlist.circuit, &config);
        let v0 = tran.timesteps[0].node_voltages["a"];
        assert!((v0 - (5.0 + 3.0 * (-1.0_f64).exp())).abs() < 1e-2, "{v0}");

        assert!(parse_str(&format!("{RC}.op\n.end\n"))
            .unwrap()
            .transient_config()
            .is_none());
    }
}
