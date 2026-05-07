use sindr::{Circuit, CircuitElement, solve_circuit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2_000.0,
            },
        ],
    };

    let result = solve_circuit(&circuit)?;

    println!("=== Voltage Divider DC Operating Point ===");
    println!("Node n1 = {:.4} V", result.node_voltages["n1"]);
    // V_n2 = 10 * 2000 / (1000 + 2000) ≈ 6.6667 V
    println!("Node n2 = {:.4} V", result.node_voltages["n2"]);
    println!("Node 0  = {:.4} V", result.node_voltages["0"]);

    Ok(())
}
