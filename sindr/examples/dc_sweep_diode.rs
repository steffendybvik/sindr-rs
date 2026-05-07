// DC sweep of a series-resistor + diode circuit (I-V curve)
//
// Topology: V1 (swept) → R_series (100 Ω) → D1 (diode) → GND
//
// Run: cargo run -p sindr --example dc_sweep_diode
//
// Expected output:
//   V_applied(V)    I_diode(A)
//   Reverse-bias (V < 0): near-zero current
//   Forward-bias (V > ~0.5): exponential current turn-on

use sindr::{Circuit, CircuitElement, dc_sweep};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 0.0, // initial value irrelevant; dc_sweep overrides it
                waveform: None,
            },
            // Series resistor required (Pitfall 2): diode directly across V-source is degenerate
            CircuitElement::Resistor {
                id: "R_series".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 100.0,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["n2".into(), "0".into()], // anode: n2, cathode: GND
                temperature: 300.15,
            },
        ],
    };

    // Sweep V1 from -1 V to +1 V in 101 points
    let sweep = dc_sweep(&circuit, "V1", -1.0, 1.0, 101)?;

    println!("V_applied(V)\tI_diode(A)");
    for (v, i) in sweep.component_current_curve("D1") {
        println!("{:>8.3}\t{:>12.6e}", v, i);
    }

    Ok(())
}
