// Temperature sweep of a forward-biased silicon diode
//
// Topology: V1 (5 V) → R1 (1 kΩ) → D1 (diode) → GND
//
// Run: cargo run -p sindr --example temperature_sweep
//
// Expected output:
//   T(°C)    V_diode(V)
//   The forward drop falls by about 1.7 mV/K as the junction warms
//   (~0.77 V at −23 °C down to ~0.53 V at 127 °C).

use sindr::{temperature_sweep, Circuit, CircuitElement};

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
                nodes: ["in".into(), "out".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["out".into(), "0".into()],
                temperature: 300.15, // overridden at each sweep point
            },
        ],
    };

    // 250 K to 400 K (−23 °C to 127 °C) in 7 points
    let sweep = temperature_sweep(&circuit, 250.0, 400.0, 7)?;

    println!("T(°C)\tV_diode(V)");
    for point in &sweep.points {
        println!(
            "{:>6.1}\t{:.4}",
            point.temperature_kelvin - 273.15,
            point.result.node_voltages["out"]
        );
    }

    Ok(())
}
