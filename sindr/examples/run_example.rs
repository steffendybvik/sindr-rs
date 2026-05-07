use std::env;

use sindr::{get_examples, solve_circuit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let id = env::args().nth(1).ok_or(
        "usage: cargo run --example run_example -- <id>\n\
         (run `cargo run --example list_examples` to see available ids)",
    )?;

    let example = get_examples()
        .into_iter()
        .find(|ex| ex.id == id)
        .ok_or_else(|| format!("no example with id '{id}'"))?;

    println!("=== {} ===", example.name);
    println!("{}\n", example.description);

    let result = solve_circuit(&example.circuit)?;

    let mut nodes: Vec<_> = result.node_voltages.iter().collect();
    nodes.sort_by_key(|(k, _)| (*k).clone());
    println!("Node voltages:");
    for (node, v) in nodes {
        println!("  {:<10} = {:>10.4} V", node, v);
    }

    if !result.bjt_results.is_empty() {
        println!("\nBJT operating points:");
        for bjt in &result.bjt_results {
            println!(
                "  {:<6} Vbe={:.3}V Vce={:.3}V Ic={:.3}mA region={}",
                bjt.id,
                bjt.vbe,
                bjt.vce,
                bjt.ic * 1e3,
                bjt.region
            );
        }
    }

    if let Some(transient) = result.transient {
        let n = transient.timesteps.len();
        if n > 0 {
            let first = &transient.timesteps[0];
            let last = &transient.timesteps[n - 1];
            println!(
                "\nTransient: {n} timesteps, t = {:.6}s → {:.6}s",
                first.time, last.time
            );
        }
    }

    Ok(())
}
