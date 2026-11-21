use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::examples;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = examples::rc_lowpass_filter();

    let config = AcConfig::new("V1", 10.0, 100_000.0, 50);

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
