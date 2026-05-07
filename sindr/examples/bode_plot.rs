// RC low-pass Bode plot: R=1kΩ, C=1µF → corner frequency ≈ 159 Hz (1 / (2π·R·C))
// Expect -3 dB gain near 159 Hz and -90° phase at high frequencies.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{Circuit, CircuitElement};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            // AC stimulus is set via AcConfig.ac_magnitude; DC voltage field is 0.0
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n_in".into(), "0".into()],
                voltage: 0.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n_in".into(), "n_out".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["n_out".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    };

    let config = AcConfig {
        f_start: 10.0,
        f_stop: 100_000.0,
        num_points: 50,
        spacing: FrequencySpacing::Logarithmic,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    };

    let result = solve_ac(&circuit, &config)?;

    println!("=== RC Low-Pass Bode Plot (corner ~159 Hz) ===");
    println!("{:>10}\t{:>7}\t{:>7}", "freq(Hz)", "gain(dB)", "phase(deg)");

    for point in result.points.iter() {
        let gain = point
            .gain_db("n_out", config.ac_magnitude)
            .unwrap_or(f64::NAN);
        let phase = point.phase_deg("n_out").unwrap_or(f64::NAN);
        println!("{:>10.2}\t{:>7.2}\t{:>7.2}", point.frequency, gain, phase);
    }

    Ok(())
}
