//! Parse a SPICE3 netlist and solve it.
//!
//! Run with the `spice` feature enabled:
//!
//! ```text
//! cargo run --example parse_netlist --features spice
//! ```
//!
//! Demonstrates the full `sindr::spice` flow: parse a deck into a
//! `Circuit`, dispatch the deck's analysis directives to the matching
//! solver routine, and render a parse error with miette's fancy reporter.

use sindr::spice::{parse_str, parse_str_with_options, AnalysisRequest, ParseOptions};

// A resistor divider plus `.dc` sweep, `.tf` and `.sens` directives. V1 = 9 V across
// R1 = 1k / R2 = 2k, so node `n2` sits at 6 V at the nominal source value.
const DECK: &str = "* resistor divider\n\
                    V1 n1 0 DC 9\n\
                    R1 n1 n2 1k\n\
                    R2 n2 0 2k\n\
                    .dc V1 0 9 0.9\n\
                    .tf V(n2) V1\n\
                    .sens V(n2)\n\
                    .end\n";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Parse (strict mode). The result is a plain `sindr::Circuit`
    //    plus the deck's analysis directives.
    let netlist = parse_str(DECK)?;
    println!(
        "parsed `{}` — {} components, {} analyses",
        netlist.title,
        netlist.circuit.components.len(),
        netlist.analyses.len()
    );

    // 2. Solve the operating point directly.
    let op = sindr::solve_circuit(&netlist.circuit)?;
    println!("V(n2) = {:.3} V", op.node_voltages["n2"]);

    // 3. Dispatch each parsed directive to the matching sindr routine.
    for analysis in &netlist.analyses {
        match analysis {
            AnalysisRequest::Op => {
                sindr::solve_op(&netlist.circuit)?;
            }
            AnalysisRequest::Dc { source, .. } => {
                let (axis, _outer) = analysis.dc_axes().ok_or("not a .dc")??;
                let sweep = sindr::dc_sweep(
                    &netlist.circuit,
                    &axis.source_id,
                    axis.start,
                    axis.stop,
                    axis.num_points,
                )?;
                let curve = sweep.node_voltage_curve("n2");
                println!(
                    "dc sweep of {source}: {} points, V(n2) {:.3}..{:.3} V",
                    curve.len(),
                    curve.first().map(|p| p.1).unwrap_or(0.0),
                    curve.last().map(|p| p.1).unwrap_or(0.0),
                );
            }
            AnalysisRequest::Tran { tstop, tstep, .. } => {
                let config = analysis.transient_config().ok_or("not a .tran")?;
                let result = sindr::solve_transient(&netlist.circuit, &config)?;
                let steps = result.transient.map_or(0, |t| t.timesteps.len());
                println!("transient to {tstop} s in {tstep} s steps: {steps} snapshots");
            }
            AnalysisRequest::Ac { fstart, fstop, .. } => match netlist.ac_config() {
                Some(Ok(config)) => {
                    let ac = sindr::ac_analysis::solve_ac(&netlist.circuit, &config)?;
                    println!("ac sweep {fstart}..{fstop} Hz: {} points", ac.points.len());
                }
                // No `AC` spec on a V line: the stimulus has to be set by hand.
                _ => println!("ac sweep {fstart}..{fstop} Hz (no AC source in the deck)"),
            },
            AnalysisRequest::Sens { .. } => {
                let config = analysis.sens_config().ok_or("not a .sens")?;
                let sens = sindr::solve_sensitivity(&netlist.circuit, &config)?;
                println!("sens: {} parameters", sens.entries.len());
            }
            AnalysisRequest::Noise { .. } => {
                let config = analysis.noise_config().ok_or("not a .noise")?;
                let noise = sindr::solve_noise(&netlist.circuit, &config)?;
                println!("noise: {} points", noise.points.len());
            }
            AnalysisRequest::Tf { .. } => {
                let config = analysis.tf_config().ok_or("not a .tf")?;
                let tf = sindr::solve_tf(&netlist.circuit, &config)?;
                println!(
                    "tf: gain {:.4}, input {:.1} Ω, output {:.1} Ω",
                    tf.gain, tf.input_resistance, tf.output_resistance
                );
            }
            // Analyses added in later releases.
            other => println!("not run here: {other:?}"),
        }
    }

    // 4. Lenient mode: an unsupported element (B = behavioural source)
    //    becomes a collected warning instead of a hard error.
    let with_behavioural = format!("{DECK}B1 n2 0 V=1\n");
    let opts = ParseOptions::default().with_strict(false);
    let lenient = parse_str_with_options(&with_behavioural, opts)?;
    for warning in &lenient.warnings {
        println!("warning: {warning:?}");
    }

    // 5. Strict mode on a bad deck: render the diagnostic with miette's
    //    fancy reporter for an annotated, underlined span.
    if let Err(e) = parse_str("* bad\nZ1 a b 1\n.end\n") {
        eprintln!("{:?}", miette::Report::new(e));
    }

    Ok(())
}
