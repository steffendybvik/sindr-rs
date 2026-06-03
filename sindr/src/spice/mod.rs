//! SPICE3 netlist parser for the sindr circuit simulator.
//!
//! Parses a practical subset of Berkeley SPICE3f5 netlists into a
//! [`crate::Circuit`] plus structured analysis directives. The supported
//! device set tracks what `sindr` can solve: R, L, C, V, I, D, Q (NPN/PNP),
//! and X (subcircuit instance). Subcircuits declared with `.subckt` /
//! `.ends` are flattened with `.`-separated hierarchical node naming, and
//! a [`SourceMap`] is returned so callers can recover the original
//! instance path for any flattened node.
//!
//! Enable the optional `spice` feature to pull this module in
//! (`sindr = { version = "0.1", features = ["spice"] }`); it is off by
//! default so the parser dependencies stay out of the baseline build.
//!
//! # Quick start
//!
//! Parse a deck, hand the flattened [`Circuit`](crate::Circuit) to the
//! solver, then dispatch each parsed analysis directive to the matching
//! `sindr` routine:
//!
//! ```no_run
//! use sindr::spice::{parse_file, AnalysisRequest};
//!
//! let netlist = parse_file("amp.cir")?;
//! println!("parsed `{}` ({} components)", netlist.title, netlist.circuit.components.len());
//!
//! // `netlist.analyses` carries the deck's `.op` / `.tran` / `.dc` / `.ac`
//! // directives, in source order. The parser does not run them — you do.
//! for analysis in &netlist.analyses {
//!     match analysis {
//!         AnalysisRequest::Op => {
//!             let result = sindr::solve_circuit(&netlist.circuit)?;
//!             println!("op: {} node voltages", result.node_voltages.len());
//!         }
//!         AnalysisRequest::Dc { source, start, stop, step } => {
//!             let points = ((stop - start) / step).abs() as usize + 1;
//!             let sweep = sindr::dc_sweep(&netlist.circuit, source, *start, *stop, points)?;
//!             println!("dc sweep of {source}: {} points", sweep.points.len());
//!         }
//!         AnalysisRequest::Tran { tstop, .. } => {
//!             // Reactive elements / waveform sources make `solve_circuit`
//!             // take the transient path automatically.
//!             let _ = sindr::solve_circuit(&netlist.circuit)?;
//!             println!("transient to {tstop} s");
//!         }
//!         AnalysisRequest::Ac { fstart, fstop, .. } => {
//!             // Build an `AcConfig` and call `sindr::ac_analysis::solve_ac`.
//!             println!("ac sweep {fstart}..{fstop} Hz");
//!         }
//!     }
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! # Strictness and warnings
//!
//! By default the parser runs in **strict** mode: any unsupported card,
//! device, model type, or grammar failure aborts with a
//! [`SpiceParseError`] carrying a labelled span. Set
//! [`ParseOptions::strict`] to `false` to downgrade the recoverable cases
//! (unsupported devices/cards) to [`ParseWarning`]s collected in
//! [`ParsedNetlist::warnings`], so a deck containing an unsupported element
//! still parses:
//!
//! ```no_run
//! use sindr::spice::{parse_str_with_options, ParseOptions};
//! # let deck = "* deck\nV1 n1 0 1\nR1 n1 0 1k\n.end\n";
//!
//! let opts = ParseOptions { strict: false, include_search_path: None };
//! let netlist = parse_str_with_options(deck, opts)?;
//! for warning in &netlist.warnings {
//!     eprintln!("warning: {warning:?}");
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! # Diagnostics
//!
//! [`SpiceParseError`] implements [`miette::Diagnostic`] and embeds the
//! source text with labelled spans. Add `miette` with its `fancy` feature
//! and render the error through a [`miette::Report`] for annotated,
//! underlined output pointing at the offending line:
//!
//! ```ignore
//! match sindr::spice::parse_file("bad.cir") {
//!     Ok(netlist) => { /* ... */ }
//!     Err(e) => eprintln!("{:?}", miette::Report::new(e)),
//! }
//! ```
//!
//! See the crate `README` for the exact supported subset and known limits.

// SpiceParseError variants embed `NamedSource<String>` so callers using
// miette's fancy reporter get pretty diagnostics for free. The trade-off is
// a ~140-byte error enum, which clippy flags as `result_large_err`. We
// prefer the rich diagnostic over boxing every parse Result.
#![allow(clippy::result_large_err)]

mod analysis;
mod ast;
mod build;
mod error;
mod flatten;
mod param_eval;
mod parser;
mod preprocess;
mod si;
mod source_map;

pub use analysis::{AcSweep, AnalysisRequest};
pub use error::{ParseWarning, SpiceParseError};
pub use source_map::{HierarchyPath, SourceMap};

use std::path::Path;

/// Successful result of parsing a SPICE netlist.
///
/// All hierarchy is already flattened into [`Self::circuit`]; use
/// [`Self::source_map`] to translate flattened node names back to their
/// original `(instance_path, node)` pair.
#[derive(Debug, Clone)]
pub struct ParsedNetlist {
    /// Flattened circuit ready for the sindr solver.
    pub circuit: crate::Circuit,
    /// Analysis directives declared in the netlist (`.tran`, `.dc`, `.ac`, `.op`).
    ///
    /// Order matches declaration order in the source.
    pub analyses: Vec<AnalysisRequest>,
    /// Non-fatal warnings collected in lenient mode. Always empty when
    /// [`ParseOptions::strict`] is `true`.
    pub warnings: Vec<ParseWarning>,
    /// Hierarchy recovery: flattened node name → original instance path + node.
    pub source_map: SourceMap,
    /// Title line of the netlist (first non-blank line). May be empty if the
    /// source has no title.
    pub title: String,
}

/// Knobs controlling parser strictness and include-path resolution.
///
/// Constructed via [`Default`] for the strict / no-override defaults, then
/// mutated field-by-field — this is `#[non_exhaustive]`-spirit but kept
/// open for now since the type is alpha.
#[derive(Debug, Clone)]
pub struct ParseOptions {
    /// If `true` (default), any unsupported card/device or syntax error is
    /// fatal. If `false`, recoverable unsupported items are skipped and
    /// recorded as [`ParseWarning`]s instead.
    pub strict: bool,
    /// Override the base directory for `.include` / `.lib` resolution.
    ///
    /// `None` = derive from the input path (when using [`parse_file`]) or
    /// the process working directory (when using [`parse_str`]). Setting
    /// this is required if you call [`parse_str`] on a netlist that
    /// contains relative `.include`s.
    pub include_search_path: Option<std::path::PathBuf>,
}

impl Default for ParseOptions {
    fn default() -> Self {
        Self {
            strict: true,
            include_search_path: None,
        }
    }
}

/// Parse a SPICE netlist from an in-memory string in strict mode.
///
/// Use [`parse_str_with_options`] if you need lenient mode or a custom
/// include search path.
///
/// # Errors
///
/// Returns [`SpiceParseError`] on the first grammar / lookup / unsupported
/// construct.
pub fn parse_str(src: &str) -> Result<ParsedNetlist, SpiceParseError> {
    parse_str_with_options(src, ParseOptions::default())
}

/// Parse a SPICE netlist from disk in strict mode.
///
/// `.include` and `.lib` directives resolve relative to `path`'s parent
/// directory.
///
/// # Errors
///
/// Returns [`SpiceParseError`] on read failure, grammar failure, or any
/// unsupported construct.
pub fn parse_file(path: impl AsRef<Path>) -> Result<ParsedNetlist, SpiceParseError> {
    parse_file_with_options(path, ParseOptions::default())
}

/// Parse a SPICE netlist from an in-memory string with explicit options.
///
/// # Errors
///
/// Returns [`SpiceParseError`] in strict mode on any unsupported construct;
/// in lenient mode, only fatal grammar / lookup failures error.
pub fn parse_str_with_options(
    src: &str,
    opts: ParseOptions,
) -> Result<ParsedNetlist, SpiceParseError> {
    let mut warnings: Vec<ParseWarning> = Vec::new();
    let base_dir = opts.include_search_path.as_deref();
    let pre = preprocess::preprocess_str(src, "<string>", base_dir, opts.strict, &mut warnings)?;
    let title = pre.title.clone();
    let cards = parser::parse_with_options(&pre, opts.strict, &mut warnings)?;
    let flat = flatten::flatten(cards, opts.strict, &mut warnings)?;
    build::build_netlist(flat, title, opts.strict, &mut warnings)
}

/// Parse a SPICE netlist from disk with explicit options.
///
/// `.include` and `.lib` directives resolve relative to `path`'s parent
/// directory unless overridden via [`ParseOptions::include_search_path`].
///
/// # Errors
///
/// Returns [`SpiceParseError`] on read failure or fatal parse error.
pub fn parse_file_with_options(
    path: impl AsRef<Path>,
    opts: ParseOptions,
) -> Result<ParsedNetlist, SpiceParseError> {
    let path = path.as_ref();
    let mut warnings: Vec<ParseWarning> = Vec::new();
    // ParseOptions::include_search_path overrides the path's parent dir if set.
    let pre = if let Some(base) = opts.include_search_path.as_deref() {
        let src = std::fs::read_to_string(path).map_err(|source| SpiceParseError::IncludeIo {
            path: path.to_path_buf(),
            source,
        })?;
        preprocess::preprocess_str(
            &src,
            &path.display().to_string(),
            Some(base),
            opts.strict,
            &mut warnings,
        )?
    } else {
        preprocess::preprocess_file(path, opts.strict, &mut warnings)?
    };
    let title = pre.title.clone();
    let cards = parser::parse_with_options(&pre, opts.strict, &mut warnings)?;
    let flat = flatten::flatten(cards, opts.strict, &mut warnings)?;
    build::build_netlist(flat, title, opts.strict, &mut warnings)
}
