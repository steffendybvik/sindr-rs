//! Harmonics, THD and a windowed FFT of a clipped sine: a 2 V, 1 kHz sine
//! through 1 kΩ into antiparallel diodes.
//!
//! Run with `cargo run --example fourier`.

use sindr::fourier::{Fft, Fourier, Window};
use sindr::measure::Signal;
use sindr::solve_transient;
use sindr::spice::parse_str;

const DECK: &str = "\
clipper
V1 in 0 SIN(0 2 1k)
R1 in out 1k
D1 out 0 dmod
D2 0 out dmod
.model dmod D(IS=1e-14 N=1)
.tran 5u 5m 0 5u
.end
";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let netlist = parse_str(DECK)?;
    let config = netlist.transient_config().ok_or("the deck has a .tran")?;
    let result = solve_transient(&netlist.circuit, &config)?;
    let out = Signal::v("out");

    // .four 1k v(out)
    let four = Fourier::new(1e3).on(&result, &out)?;
    println!("Fourier analysis of {out}: THD {:.4} %", four.thd_percent);
    println!("harmonic  frequency (Hz)  magnitude (V)  phase (°)");
    println!("{:>8}  {:>14}  {:>13.6}  {:>9}", 0, 0, four.dc, 0);
    for h in &four.harmonics {
        println!(
            "{:>8}  {:>14}  {:>13.6}  {:>9.4}",
            h.number, h.frequency, h.magnitude, h.phase
        );
    }

    // FFT of the last four periods, Hann-windowed: 250 Hz bins.
    let spectrum = Fft::new()
        .window(Window::Hann)
        .points(1024)
        .span(1e-3, 5e-3)
        .on(&result, &out)?;
    println!(
        "\nFFT, {:?} window, {} points, {} Hz bins:",
        spectrum.window, spectrum.points, spectrum.resolution
    );
    for bin in spectrum.bins.iter().filter(|b| b.magnitude > 1e-2) {
        println!("{:>8.0} Hz  {:.4} V", bin.frequency, bin.magnitude);
    }
    Ok(())
}
