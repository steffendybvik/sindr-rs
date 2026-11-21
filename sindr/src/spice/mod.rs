//! SPICE3 netlist parser for the sindr circuit simulator.
//!
//! Parses a practical subset of Berkeley SPICE3f5 netlists into a
//! [`crate::Circuit`] plus structured analysis directives. The supported
//! device set tracks what `sindr` can solve: R, L, C, V, I, D, Q (NPN/PNP),
//! the linear controlled sources E, G, F and H, K (two coupled `L` lines,
//! read as one [`Transformer`](crate::CircuitElement::Transformer)) and X
//! (subcircuit instance). Subcircuits declared with `.subckt` /
//! `.ends` become [`Block`](crate::Block)s and X lines
//! [`Instance`](crate::Instance)s; the solver flattens them with
//! `.`-separated hierarchical naming ([`Circuit::flatten`](crate::Circuit::flatten)),
//! and a [`SourceMap`] maps each flattened node back to its instance path.
//! [`to_netlist`] writes blocks back as `.subckt`s.
//! `.global` nets, declared anywhere in the deck (inside a `.subckt` too),
//! land in [`Circuit::globals`](crate::Circuit::globals) and keep their name
//! inside every instance instead of taking its prefix.
//!
//! `.ic` cards land in [`ParsedNetlist::initial_conditions`], which
//! [`ParsedNetlist::transient_config`] passes on. `.options` (`RELTOL`,
//! `VNTOL`, `TRTOL`, `ITL1`) and `.nodeset` cards land in
//! [`ParsedNetlist::options`]; [`ParsedNetlist::solver`] runs the deck's
//! analyses with them.
//!
//! Enable the optional `spice` feature to pull this module in
//! (`sindr = { version = "0.1", features = ["spice"] }`); it is off by
//! default so the parser dependencies stay out of the baseline build.
//!
//! # Quick start
//!
//! Parse a deck from a string and solve its operating point:
//!
//! ```
//! let deck = "\
//! * 10 V across a 1k / 2k divider
//! V1 in 0 10
//! R1 in out 1k
//! R2 out 0 2k
//! .op
//! .end
//! ";
//! let netlist = sindr::spice::parse_str(deck)?;
//! let result = sindr::solve_circuit(&netlist.circuit)?;
//! assert!((result.node_voltages["out"] - 20.0 / 3.0).abs() < 1e-9);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! For a deck on disk, parse it, hand the [`Circuit`](crate::Circuit) to the
//! solver, then dispatch each parsed analysis directive to the matching
//! `sindr` routine (`no_run`: `amp.cir` doesn't exist where doctests run):
//!
//! ```no_run
//! use sindr::spice::{parse_file, AnalysisRequest};
//!
//! let netlist = parse_file("amp.cir")?;
//! println!("parsed `{}` ({} components)", netlist.title, netlist.circuit.components.len());
//!
//! // `netlist.analyses` carries the deck's `.op` / `.tran` / `.dc` / `.ac` / `.tf` / `.sens` / `.noise`
//! // directives, in source order. The parser does not run them — you do.
//! for analysis in &netlist.analyses {
//!     match analysis {
//!         AnalysisRequest::Op => {
//!             let result = sindr::solve_circuit(&netlist.circuit)?;
//!             println!("op: {} node voltages", result.node_voltages.len());
//!         }
//!         AnalysisRequest::Dc { source, .. } => {
//!             match analysis.dc_axes().unwrap()? {
//!                 (inner, Some(outer)) => {
//!                     let sweep = sindr::dc_sweep_nested(&netlist.circuit, &inner, &outer)?;
//!                     println!("dc sweep of {source}: {} curves", sweep.curves.len());
//!                 }
//!                 (axis, None) => {
//!                     let sweep = sindr::dc_sweep(
//!                         &netlist.circuit, &axis.source_id, axis.start, axis.stop, axis.num_points,
//!                     )?;
//!                     println!("dc sweep of {source}: {} points", sweep.points.len());
//!                 }
//!             }
//!         }
//!         AnalysisRequest::Tran { tstop, .. } => {
//!             let config = analysis.transient_config().unwrap();
//!             let _ = sindr::solve_transient(&netlist.circuit, &config)?;
//!             println!("transient to {tstop} s");
//!         }
//!         AnalysisRequest::Ac { fstart, fstop, .. } => {
//!             // Build an `AcConfig` and call `sindr::ac_analysis::solve_ac`.
//!             println!("ac sweep {fstart}..{fstop} Hz");
//!         }
//!         AnalysisRequest::Sens { .. } => {
//!             let sens = sindr::solve_sensitivity(&netlist.circuit, &analysis.sens_config().unwrap())?;
//!             println!("sens: {} parameters", sens.entries.len());
//!         }
//!         AnalysisRequest::Noise { .. } => {
//!             let noise = sindr::solve_noise(&netlist.circuit, &analysis.noise_config().unwrap())?;
//!             println!("noise: {} V rms at the output", noise.total_output_noise);
//!         }
//!         AnalysisRequest::Tf { .. } => {
//!             let tf = sindr::solve_tf(&netlist.circuit, &analysis.tf_config().unwrap())?;
//!             println!("tf: gain {}, rin {} Ω, rout {} Ω", tf.gain, tf.input_resistance, tf.output_resistance);
//!         }
//!     }
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! # Model cards
//!
//! A `.model` card's parameters are read where sindr's models have them:
//!
//! | Card | Parameters | Into |
//! |---|---|---|
//! | `D` | `IS`, `N`, `RS`, `CJO` (or `CJ0`, `CJ`), `VJ` (or `PB`), `M` (or `MJ`), `FC`, `TT`, `BV`, `IBV`, `KF`, `AF` | [`DiodeModel`](crate::DiodeModel) |
//! | `NPN`, `PNP` | `BF` | the element's `bf` |
//! | `NPN`, `PNP` | `IS`, `BR`, `NF`, `NR`, `VAF` (or `VA`), `KF`, `AF` | [`BjtModel`](crate::BjtModel) |
//!
//! sindr's BJT is Ebers–Moll, so its Early effect isn't SPICE's
//! Gummel–Poon base charge: with `VAF` set, results differ slightly from
//! SPICE's. Any other parameter (`EG`, `XTI`, `CJE`, `VAR`, `IKF`, …) is an
//! error in strict mode and a warning per card in lenient mode, naming them.
//! Part annotations that don't affect the simulation (`MFG`, `TYPE`,
//! `VCEO`, `ICRATING`, `IAVE`, `VPK`) are dropped without a word.
//!
//! # Controlled sources and coupling
//!
//! | Line | Into |
//! |---|---|
//! | `Exxx n+ n- nc+ nc- gain` | [`Vcvs`](crate::CircuitElement::Vcvs) |
//! | `Gxxx n+ n- nc+ nc- gm` | [`Vccs`](crate::CircuitElement::Vccs) |
//! | `Fxxx n+ n- vname alpha` | [`Cccs`](crate::CircuitElement::Cccs) |
//! | `Hxxx n+ n- vname rm` | [`Ccvs`](crate::CircuitElement::Ccvs) |
//! | `Kxxx lname1 lname2 k` | [`Transformer`](crate::CircuitElement::Transformer) |
//!
//! Nodes and current directions are SPICE's, so they map across as
//! written. An `F` or `H` line's `vname` must be a `V` line in the same
//! scope (the top level, or the same `.subckt` body). A `K` line replaces
//! its two `L` lines, which must be in its scope, with one transformer named
//! after the `K` line: primary from `lname1`, secondary from `lname2`, the
//! first node of each its dot. Each `L` can be coupled once; sindr has no
//! three-winding transformer. The nonlinear forms (`POLY`, `VALUE`,
//! `TABLE`, `LAPLACE`) are unsupported devices: an error in strict mode, a
//! warning (and the line dropped) in lenient mode.
//!
//! # Values
//!
//! Numbers take SPICE's case-insensitive suffixes: `t` (1e12), `g` (1e9),
//! `meg` (1e6), `k` (1e3), `m` (1e-3), `mil` (25.4e-6), `u` (1e-6),
//! `n` (1e-9), `p` (1e-12), `f` (1e-15) and `a` (1e-18). Letters after the
//! suffix are ignored, so `4.7uF` is 4.7e-6. `{…}` expressions may nest
//! at most 256 levels deep. A value that evaluates to NaN or ±∞ (`1e400`,
//! `{1/0}`) is an error at its line.
//!
//! Analysis cards are checked as they are read: a `.dc` must sweep one or
//! two different voltage or current sources the deck defines, each with a
//! non-zero step that moves from start towards stop; an `.ac` needs `fstart <= fstop`, and
//! `fstart > 0` for `dec`/`oct`.
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
//! ```
//! use sindr::spice::{parse_str_with_options, ParseOptions};
//! # let deck = "* deck\nV1 n1 0 1\nR1 n1 0 1k\n.end\n";
//!
//! let opts = ParseOptions::default().with_strict(false);
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
//! underlined output pointing at the offending line (`no_run`: `bad.cir`
//! doesn't exist where doctests run):
//!
//! ```no_run
//! match sindr::spice::parse_file("bad.cir") {
//!     Ok(netlist) => { /* ... */ }
//!     Err(e) => eprintln!("{:?}", miette::Report::new(e)),
//! }
//! ```
//!
//! # Export
//!
//! [`to_netlist`] writes a [`Circuit`](crate::Circuit) as a SPICE deck that
//! ngspice runs; its docs list what each element is written as. [`parse_str`]
//! reads every line it writes back, so an exported deck parses to a circuit
//! that solves the same (a transformer comes back named after its `K` line).
//!
//! Known gaps against full SPICE3 are listed in the repository's
//! [`LIMITATIONS.md`](https://github.com/steffendybvik/sindr-rs/blob/main/LIMITATIONS.md).

// SpiceParseError variants embed `NamedSource<String>` so callers using
// miette's fancy reporter get pretty diagnostics for free. The trade-off is
// a ~140-byte error enum, which clippy flags as `result_large_err`. We
// prefer the rich diagnostic over boxing every parse Result.
#![allow(clippy::result_large_err)]

mod analysis;
mod ast;
mod build;
mod error;
mod export;
mod flatten;
mod param_eval;
mod parser;
mod preprocess;
mod si;
mod source_map;

pub use analysis::{AcSweep, AnalysisRequest, DcSweepSpec, MeasureAnalysis, MeasureRequest};
pub use error::{ParseWarning, SpiceParseError};
pub use export::{to_netlist, ExportError};
pub use source_map::{HierarchyPath, SourceMap};

use std::collections::HashMap;
use std::path::Path;

/// Successful result of parsing a SPICE netlist.
///
/// [`Self::circuit`] keeps the `.subckt` hierarchy as blocks and
/// instances. Every analysis flattens it; use [`Self::source_map`] to
/// translate flattened node names back to their
/// original `(instance_path, node)` pair.
#[derive(Debug, Clone)]
pub struct ParsedNetlist {
    /// The circuit, with each `.subckt` as a [`Block`](crate::Block) and each
    /// X line as an [`Instance`](crate::Instance). A subckt whose instances
    /// bind different `.param` values gives one block per distinct body,
    /// named `name`, `name_2`, …. The solver takes it as is;
    /// [`Circuit::flatten`](crate::Circuit::flatten) gives the flat element
    /// list.
    pub circuit: crate::Circuit,
    /// Analysis directives declared in the netlist (`.op`, `.tran`, `.dc`,
    /// `.ac`, `.tf`, `.sens`).
    ///
    /// Order matches declaration order in the source.
    pub analyses: Vec<AnalysisRequest>,
    /// `.ic` node voltages (V), keyed by node name: where a transient
    /// starts, or with no `uic` what its operating point holds them at.
    /// [`ParsedNetlist::transient_config`] passes them to
    /// [`TransientConfig::with_initial_voltages`](crate::TransientConfig::with_initial_voltages).
    /// Only top-level cards are read.
    pub initial_conditions: HashMap<String, f64>,
    /// `.nodeset` node voltages (V), keyed by node name: a starting guess for
    /// the DC operating point. [`Self::options`] carries them too, as its
    /// [`nodeset`](crate::SolverOptions::nodeset). Only top-level cards are
    /// read.
    pub nodesets: HashMap<String, f64>,
    /// The deck's `.options` (`reltol`, `vntol`, `trtol`, `itl1`) over the
    /// defaults, with its `.nodeset` cards as the nodeset. Run the deck's
    /// analyses through [`Self::solver`] to use them. Only top-level cards
    /// are read.
    pub options: crate::SolverOptions,
    /// `.meas` / `.measure` cards, in source order. Only top-level cards are
    /// read.
    pub measurements: Vec<MeasureRequest>,
    /// Non-fatal warnings collected in lenient mode. Always empty when
    /// [`ParseOptions::strict`] is `true`.
    pub warnings: Vec<ParseWarning>,
    /// Hierarchy recovery: flattened node name → original instance path + node.
    pub source_map: SourceMap,
    /// Title line of the netlist (first non-blank line). May be empty if the
    /// source has no title.
    pub title: String,
}

impl ParsedNetlist {
    /// A [`Solver`](crate::Solver) with the deck's [`options`](Self::options).
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// let deck = "* opts\nV1 in 0 5\nR1 in a 1k\nD1 a 0 dmod\n.model dmod D\n\
    ///             .options reltol=1e-6 itl1=200\n.nodeset v(a)=0.7\n.op\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// assert_eq!(netlist.options.max_iterations, 200);
    /// let op = netlist.solver().circuit(&netlist.circuit)?;
    /// assert!((op.node_voltages["a"] - 0.7).abs() < 0.1);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn solver(&self) -> crate::Solver {
        crate::Solver::new(self.options.clone())
    }

    /// The [`TransientConfig`](crate::TransientConfig) for the deck's first
    /// `.tran`, with its `.ic` cards as
    /// [`initial_voltages`](crate::TransientConfig::initial_voltages), so
    /// the run starts as ngspice's does: from the operating point with the
    /// `.ic` nodes held, or with `uic` from the `.ic` values alone. `None`
    /// without a `.tran`. See [`AnalysisRequest::transient_config`] for the
    /// rest of the card.
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// // .ic holds `a` at 3 V while the operating point is solved; then
    /// // the capacitor discharges towards the divider's 0.5 V.
    /// let deck = "* ic\nV1 in 0 1\nR1 in a 1k\nR2 a 0 1k\nC1 a 0 1u\n\
    ///             .ic v(a)=3\n.tran 10u 5m\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// let config = netlist.transient_config().unwrap();
    /// let tran = sindr::solve_transient(&netlist.circuit, &config)?.transient.unwrap();
    /// assert!(tran.timesteps[0].node_voltages["a"] > 2.9);
    /// assert!((tran.timesteps.last().unwrap().node_voltages["a"] - 0.5).abs() < 1e-3);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn transient_config(&self) -> Option<crate::TransientConfig> {
        self.analyses
            .iter()
            .find_map(AnalysisRequest::transient_config)
            .map(|config| config.with_initial_voltages(self.initial_conditions.clone()))
    }
}

/// Knobs controlling parser strictness and include-path resolution.
///
/// Start from [`Default`] (strict, no include-path override) and set what
/// you need with [`with_strict`](Self::with_strict) and
/// [`with_include_search_path`](Self::with_include_search_path). It is
/// `#[non_exhaustive]`, so new options can come without breaking callers.
#[derive(Debug, Clone)]
#[non_exhaustive]
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

impl ParseOptions {
    /// Set [`strict`](Self::strict): `false` turns recoverable unsupported
    /// items into [`ParseWarning`]s instead of errors.
    #[must_use]
    pub fn with_strict(mut self, strict: bool) -> Self {
        self.strict = strict;
        self
    }

    /// Set [`include_search_path`](Self::include_search_path), the base
    /// directory for relative `.include` / `.lib` paths.
    #[must_use]
    pub fn with_include_search_path(mut self, path: impl Into<std::path::PathBuf>) -> Self {
        self.include_search_path = Some(path.into());
        self
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fixtures() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
    }

    fn component_ids(netlist: &ParsedNetlist) -> Vec<String> {
        let mut ids: Vec<String> = netlist
            .circuit
            .components
            .iter()
            .map(|c| c.id().to_string())
            .collect();
        ids.sort();
        ids
    }

    #[test]
    fn default_options_are_strict_with_no_search_path() {
        let opts = ParseOptions::default();
        assert!(opts.strict);
        assert!(opts.include_search_path.is_none());
    }

    #[test]
    fn title_is_the_first_line() {
        let netlist = parse_str("my amplifier\nV1 a 0 1\nR1 a 0 1k\n.end\n").unwrap();
        assert_eq!(netlist.title, "my amplifier");
        assert!(netlist.warnings.is_empty());
    }

    #[test]
    fn parse_str_and_parse_file_agree() {
        let path = fixtures().join("rc.cir");
        let from_file = parse_file(&path).unwrap();
        let from_str = parse_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(component_ids(&from_file), component_ids(&from_str));
        assert_eq!(from_file.analyses, from_str.analyses);
        assert_eq!(from_file.title, from_str.title);
    }

    #[test]
    fn parse_str_resolves_includes_from_the_search_path() {
        let deck = "main\nR1 a b 1k\n.include 'include_sub.cir'\n.end\n";
        let opts = ParseOptions {
            include_search_path: Some(fixtures()),
            ..ParseOptions::default()
        };
        let netlist = parse_str_with_options(deck, opts).unwrap();
        assert_eq!(component_ids(&netlist), ["r1", "rincl"]);
    }

    #[test]
    fn parse_file_search_path_overrides_the_file_directory() {
        // Copy the main deck somewhere its include can't be found by default.
        let dir = std::env::temp_dir().join(format!("sindr-spice-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let main = dir.join("include_main.cir");
        std::fs::copy(fixtures().join("include_main.cir"), &main).unwrap();

        assert!(
            parse_file(&main).is_err(),
            "include should not resolve next to the copy"
        );

        let opts = ParseOptions {
            include_search_path: Some(fixtures()),
            ..ParseOptions::default()
        };
        let netlist = parse_file_with_options(&main, opts).unwrap();
        assert_eq!(component_ids(&netlist), ["r1", "rincl"]);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn missing_file_is_an_io_error() {
        let missing = fixtures().join("does_not_exist.cir");
        for result in [
            parse_file(&missing),
            parse_file_with_options(
                &missing,
                ParseOptions {
                    include_search_path: Some(fixtures()),
                    ..ParseOptions::default()
                },
            ),
        ] {
            match result {
                Err(SpiceParseError::IncludeIo { path, .. }) => assert_eq!(path, missing),
                other => panic!("expected IncludeIo, got {other:?}"),
            }
        }
    }

    #[test]
    fn include_cycle_is_an_error_not_a_hang() {
        match parse_file(fixtures().join("cycle_a.cir")) {
            Err(SpiceParseError::Syntax { message, .. }) => {
                assert!(message.contains("include cycle"), "{message}")
            }
            other => panic!("expected an include-cycle error, got {other:?}"),
        }
    }
}
