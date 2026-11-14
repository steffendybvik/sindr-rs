//! The IGBT's VCE limiter must not let Newton stop at a point that breaks
//! KCL. When the true solution is reverse-blocking (vce < 0) but Newton
//! starts from vce > 0, the limiter walks vce down; a limited step must
//! never count as converged.

use std::collections::HashMap;
use std::f64::consts::PI;

use sindr::{
    solve_circuit_with_initial_voltages, solve_transient, Circuit, CircuitElement, IgbtParams,
    TransientConfig, Waveform,
};

fn vsource(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
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

/// `vs` through 1 kΩ into the collector of an IGBT whose gate is held at
/// 15 V and whose emitter is at ground.
fn reverse_igbt(vs: f64) -> Circuit {
    Circuit {
        components: vec![
            vsource("V1", "s", "0", vs),
            vsource("VG", "g", "0", 15.0),
            resistor("R1", "s", "c", 1e3),
            CircuitElement::Igbt {
                id: "Q1".into(),
                nodes: ["g".into(), "c".into(), "e".into()],
                params: IgbtParams::default(),
            },
            resistor("RE", "e", "0", 1e-3),
        ],
        ground_node: "0".into(),
    }
}

/// Reverse-blocking from a forward nodeset: the collector sits at the
/// source, −5 V, with no current through R1.
#[test]
fn reverse_blocking_from_a_forward_nodeset() {
    let nodeset = HashMap::from([("c".to_string(), 1.0)]);
    let result = solve_circuit_with_initial_voltages(&reverse_igbt(-5.0), &nodeset).unwrap();
    let vc = result.node_voltages["c"];
    assert!((vc + 5.0).abs() < 1e-6, "Vc = {vc}, expected −5 V");
}

/// A sine through zero: once the source is negative, the collector follows
/// it, with no current through R1.
#[test]
fn transient_follows_the_source_below_zero() {
    let mut circuit = reverse_igbt(0.0);
    if let CircuitElement::VoltageSource { waveform, .. } = &mut circuit.components[0] {
        *waveform = Some(Waveform::Sine {
            amplitude: 5.0,
            frequency: 1e3,
            offset: 0.0,
            phase: 0.0,
        });
    }
    let config = TransientConfig::new(1e-3, 1e-5).with_fixed_step();
    let tran = solve_transient(&circuit, &config)
        .unwrap()
        .transient
        .unwrap();
    for s in &tran.timesteps {
        let vs = 5.0 * (2.0 * PI * 1e3 * s.time).sin();
        if vs < 0.0 {
            let vc = s.node_voltages["c"];
            assert!(
                (vc - vs).abs() < 1e-6 + 1e-3 * vs.abs(),
                "t = {}: Vc = {vc}, Vs = {vs}",
                s.time
            );
        }
    }
}
