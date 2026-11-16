use sindr::{examples, solve_sensitivity, SensConfig, TfOutput};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = examples::npn_common_emitter();

    // Which part moves the collector voltage most?
    let config = SensConfig {
        output: TfOutput::Voltage {
            node: "collector".into(),
            reference: None,
        },
    };
    let sens = solve_sensitivity(&circuit, &config)?;

    println!("=== NPN Common Emitter — .sens V(collector) ===");
    println!("{:>6}\t{:>12}\t{:>14}", "part", "value", "dV per +1%");
    for entry in &sens.entries {
        println!(
            "{:>6}\t{:>12.1}\t{:>11.2} mV",
            entry.component_id,
            entry.value,
            entry.per_percent * 1e3
        );
    }

    Ok(())
}
