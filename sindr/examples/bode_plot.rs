use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::examples;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = examples::rc_lowpass_filter();

    let config = AcConfig {
        f_start: 10.0,
        f_stop: 100_000.0,
        num_points: 50,
        spacing: FrequencySpacing::Logarithmic,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    };

    let result = solve_ac(&circuit, &config)?;

    println!("=== RC Low-Pass Filter — Bode Plot ===");
    println!("{:>10}\t{:>7}\t{:>7}", "freq(Hz)", "gain(dB)", "phase(deg)");

    for point in result.points.iter() {
        let gain = point.gain_db("n2", config.ac_magnitude).unwrap_or(f64::NAN);
        let phase = point.phase_deg("n2").unwrap_or(f64::NAN);
        println!("{:>10.2}\t{:>7.2}\t{:>7.2}", point.frequency, gain, phase);
    }

    Ok(())
}
