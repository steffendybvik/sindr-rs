// Seeding the Newton–Raphson starting point (SPICE `.nodeset`)
//
// Topology: V1 (5 V) → R1 (1 kΩ) → D1 → D2 → GND (two diodes in series)
//
// Run: cargo run -p sindr --example initial_voltages
//
// Expected output: both solves land on the same operating point
// (~1.38 V across the pair); the seed only changes where Newton starts.
// Seeds apply to nonlinear DC only, and node names that don't exist are
// ignored.

use std::collections::HashMap;

use sindr::{solve_circuit, solve_circuit_with_initial_voltages, Circuit, CircuitElement};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "top".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["top".into(), "mid".into()],
                temperature: 300.15,
            },
            CircuitElement::Diode {
                id: "D2".into(),
                nodes: ["mid".into(), "0".into()],
                temperature: 300.15,
            },
        ],
    };

    let plain = solve_circuit(&circuit)?;

    // Start Newton near the expected answer: ~0.65 V per diode.
    let seed = HashMap::from([("top".to_string(), 1.3), ("mid".to_string(), 0.65)]);
    let seeded = solve_circuit_with_initial_voltages(&circuit, &seed)?;

    for node in ["top", "mid"] {
        println!(
            "V({node}): plain {:.6} V, seeded {:.6} V",
            plain.node_voltages[node], seeded.node_voltages[node]
        );
    }

    Ok(())
}
