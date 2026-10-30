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

// A resistor divider plus `.dc` sweep and `.tf` directives. V1 = 9 V across
// R1 = 1k / R2 = 2k, so node `n2` sits at 6 V at the nominal source value.
const DECK: &str = "* resistor divider\n\
                    V1 n1 0 DC 9\n\
                    R1 n1 n2 1k\n\
                    R2 n2 0 2k\n\
                    .dc V1 0 9 0.9\n\
                    .tf V(n2) V1\n\
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
                sindr::solve_circuit(&netlist.circuit)?;
            }
            AnalysisRequest::Dc {
                source,
                start,
                stop,
                step,
            } => {
                let points = ((stop - start) / step).abs() as usize + 1;
                let sweep = sindr::dc_sweep(&netlist.circuit, source, *start, *stop, points)?;
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
            AnalysisRequest::Ac { fstart, fstop, .. } => {
                println!("ac sweep {fstart}..{fstop} Hz (build an AcConfig for solve_ac)");
            }
            AnalysisRequest::Tf { .. } => {
                let config = analysis.tf_config().ok_or("not a .tf")?;
                let tf = sindr::solve_tf(&netlist.circuit, &config)?;
                println!(
                    "tf: gain {:.4}, input {:.1} Ω, output {:.1} Ω",
                    tf.gain, tf.input_resistance, tf.output_resistance
                );
            }
        }
    }

    // 4. Lenient mode: an unsupported element (M = MOSFET) becomes a
    //    collected warning instead of a hard error.
    let with_mosfet = format!("{DECK}M1 n2 n1 0 nmod\n");
    let opts = ParseOptions {
        strict: false,
        include_search_path: None,
    };
    let lenient = parse_str_with_options(&with_mosfet, opts)?;
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
