//! Solves under `wasm32-unknown-unknown`, where `std::time::Instant`,
//! threads and the file system trap or fail at run time rather than at
//! build time. CI runs it in Node:
//!
//! ```text
//! cargo test -p sindr --all-features --target wasm32-unknown-unknown --test wasm
//! ```
//!
//! with `CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_RUNNER=wasm-bindgen-test-runner`.
#![cfg(target_arch = "wasm32")]

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{solve_circuit, solve_transient, Circuit, CircuitElement, TransientConfig};
use wasm_bindgen_test::wasm_bindgen_test;

/// 5 V through 1 kΩ into a diode, with 1 µF across it.
fn circuit() -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "a".into()],
                resistance: 1e3,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model: None,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["a".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    )
}

#[wasm_bindgen_test]
fn operating_point_ac_and_transient() {
    let c = circuit();
    let op = solve_circuit(&c).unwrap();
    let a = op.node_voltages["a"];
    assert!((0.5..0.8).contains(&a), "diode drop {a}");

    let ac = solve_ac(
        &c,
        &AcConfig {
            f_start: 10.0,
            f_stop: 1e6,
            num_points: 11,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
            ac_phase_deg: 0.0,
        },
    )
    .unwrap();
    assert_eq!(ac.points.len(), 11);

    let tran = solve_transient(&c, &TransientConfig::new(1e-3, 1e-5)).unwrap();
    assert!(tran.transient.unwrap().timesteps.len() > 10);
}

#[cfg(feature = "spice")]
#[wasm_bindgen_test]
fn spice_deck() {
    let deck = "divider\nV1 in 0 9\nR1 in out 1k\nR2 out 0 2k\n.op\n.end\n";
    let netlist = sindr::spice::parse_str(deck).unwrap();
    let op = solve_circuit(&netlist.circuit).unwrap();
    assert!((op.node_voltages["out"] - 6.0).abs() < 1e-9);
}
