//! Arbitrary text into the SPICE parser. Nothing may panic: a bad deck is a
//! `SpiceParseError`. A deck that parses is also validated and written back
//! out with `to_netlist`, and the written deck must parse again.

#![no_main]

use libfuzzer_sys::fuzz_target;
use sindr::spice::{parse_str, parse_str_with_options, to_netlist, ParseOptions};

fuzz_target!(|data: &[u8]| {
    let Some((&mode, rest)) = data.split_first() else {
        return;
    };
    let Ok(src) = std::str::from_utf8(rest) else {
        return;
    };
    // `.include` and `.lib` with an absolute path would read the machine's
    // files; that's the caller's choice in real use, and noise here.
    let reads_files = src.lines().any(|line| {
        let line = line.trim_start().to_ascii_lowercase();
        (line.starts_with(".inc") || line.starts_with(".lib"))
            && (line.contains('/') || line.contains('\\'))
    });
    if reads_files {
        return;
    }
    let opts = ParseOptions {
        strict: mode & 1 == 0,
        ..ParseOptions::default()
    };
    let Ok(parsed) = parse_str_with_options(src, opts) else {
        return;
    };
    let _ = sindr::validate_circuit(&parsed.circuit);
    for analysis in &parsed.analyses {
        let _ = analysis.transient_config();
    }
    if let Ok(deck) = to_netlist(&parsed.circuit, "fuzz") {
        if let Err(err) = parse_str(&deck) {
            panic!("exported deck doesn't parse: {err}\n{deck}");
        }
    }
});
