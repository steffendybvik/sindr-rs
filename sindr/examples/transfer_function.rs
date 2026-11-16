use sindr::{examples, solve_tf, TfConfig, TfOutput};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = examples::npn_common_emitter();

    // How much supply ripple reaches the collector, what the supply sees,
    // and how stiff the collector node is.
    let config = TfConfig {
        output: TfOutput::Voltage {
            node: "collector".into(),
            reference: None,
        },
        input_source: "Vcc".into(),
    };
    let tf = solve_tf(&circuit, &config)?;

    println!("=== NPN Common Emitter — .tf V(collector) Vcc ===");
    println!("gain               = {:.4} V/V", tf.gain);
    println!("input resistance   = {:.1} Ω", tf.input_resistance);
    println!("output resistance  = {:.1} Ω", tf.output_resistance);

    Ok(())
}
