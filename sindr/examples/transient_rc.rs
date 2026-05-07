use sindr::{Circuit, CircuitElement, solve_circuit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None, // DC step: constant 5 V
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["n2".into(), "0".into()],
                capacitance: 100e-6, // tau = RC = 0.1 s
            },
        ],
    };

    let result = solve_circuit(&circuit)?;
    let transient = result
        .transient
        .ok_or("expected transient data for RC circuit")?;

    println!("=== RC Charging Transient (tau = 0.1 s, V_final = 5 V) ===");
    println!("time(s)\t\tV_out(V)");

    let steps = &transient.timesteps;
    let len = steps.len();

    for (i, snap) in steps.iter().enumerate() {
        if i % 10 == 0 {
            let v_out = snap.node_voltages.get("n2").copied().unwrap_or(0.0);
            println!("{:.6}\t{:.4}", snap.time, v_out);
        }
    }

    // Always print the final timestep so the asymptote toward 5 V is visible
    if len > 0 && (len - 1) % 10 != 0 {
        let last = &steps[len - 1];
        let v_out = last.node_voltages.get("n2").copied().unwrap_or(0.0);
        println!("{:.6}\t{:.4}", last.time, v_out);
    }

    Ok(())
}
