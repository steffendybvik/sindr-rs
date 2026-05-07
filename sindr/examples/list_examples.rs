use sindr::get_examples;

fn main() {
    let examples = get_examples();
    println!("=== Built-in example circuits ({}) ===", examples.len());
    for ex in &examples {
        println!("\n  {}", ex.id);
        println!("    {}", ex.name);
        println!("    {}", ex.description);
    }
    println!("\nRun any with: cargo run --example run_example -- <id>");
}
