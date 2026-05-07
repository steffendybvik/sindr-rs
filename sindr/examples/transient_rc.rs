use sindr::{examples, solve_circuit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = examples::rc_lowpass_filter();
    let result = solve_circuit(&circuit)?;
    let transient = result
        .transient
        .ok_or("expected transient data — circuit should contain a capacitor")?;

    println!("=== RC Low-Pass Filter — Step Response ===");
    println!("time(s)\t\tV_out(V)");

    let steps = &transient.timesteps;
    let stride = (steps.len() / 25).max(1);
    for (i, snap) in steps.iter().enumerate() {
        if i % stride == 0 {
            let v_out = snap.node_voltages.get("n2").copied().unwrap_or(0.0);
            println!("{:.6}\t{:.4}", snap.time, v_out);
        }
    }
    if let Some(last) = steps.last() {
        let v_out = last.node_voltages.get("n2").copied().unwrap_or(0.0);
        println!("{:.6}\t{:.4}", last.time, v_out);
    }

    Ok(())
}
