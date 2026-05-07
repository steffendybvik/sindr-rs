use sindr::{examples, solve_circuit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = examples::voltage_divider();
    let result = solve_circuit(&circuit)?;

    println!("=== Voltage Divider DC Operating Point ===");
    let mut nodes: Vec<_> = result.node_voltages.iter().collect();
    nodes.sort_by_key(|(k, _)| (*k).clone());
    for (node, v) in nodes {
        println!("Node {:<4} = {:.4} V", node, v);
    }

    Ok(())
}
