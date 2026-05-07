// EX-05: Common-emitter NPN amplifier nonlinear DC operating point
//
// Topology: Vcc (10V) → Rb (470kΩ) → base; Vcc → Rc (1kΩ) → collector; emitter → GND
//
// Run: cargo run -p sindr --example bjt_amplifier
//
// Expected approximate values (Rb=470kΩ, Vcc=10V, bf=100):
//   Ib ≈ (10-0.7)/470k ≈ 19.8 µA
//   Ic ≈ 100 × Ib     ≈ 1.98 mA
//   Vce ≈ 10 - 1.98×1k ≈ 8.02 V
//   Region: active

use sindr::{Circuit, CircuitElement, BjtKind, solve_circuit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "Vcc".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            // Base bias resistor from Vcc to base
            CircuitElement::Resistor {
                id: "Rb".into(),
                nodes: ["vcc".into(), "base".into()],
                resistance: 470_000.0,
            },
            // Collector load resistor from Vcc to collector
            CircuitElement::Resistor {
                id: "Rc".into(),
                nodes: ["vcc".into(), "collector".into()],
                resistance: 1_000.0,
            },
            // NPN BJT: nodes order is [base, collector, emitter]
            // BjtKind imported from sindr::BjtKind (not sindr_devices — Pitfall 4)
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["base".into(), "collector".into(), "0".into()], // emitter to GND
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
            },
        ],
    };

    let result = solve_circuit(&circuit)?;

    let bjt = result
        .bjt_results
        .iter()
        .find(|b| b.id == "Q1")
        .ok_or("Q1 not found in bjt_results")?;

    println!("=== BJT Operating Point (Q1) ===");
    println!("Vbe    = {:.4} V",  bjt.vbe);
    println!("Vce    = {:.4} V",  bjt.vce);
    println!("Ib     = {:.4} mA", bjt.ib * 1e3);
    println!("Ic     = {:.4} mA", bjt.ic * 1e3);
    println!("Ie     = {:.4} mA", bjt.ie * 1e3);
    println!("Region : {}",       bjt.region);
    println!("Power  = {:.4} mW", bjt.power * 1e3);

    Ok(())
}
