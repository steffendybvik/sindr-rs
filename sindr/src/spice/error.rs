//! Error and warning types for the SPICE parser.
//!
//! [`SpiceParseError`] is a [`miette::Diagnostic`]: every span-bearing variant
//! carries the source text and a labelled span so callers using
//! `miette = { features = ["fancy"] }` get pretty terminal output for free.
//!
//! `unused_assignments` is suppressed module-wide because miette's
//! `Diagnostic` derive expands to a chain of `let mut x = ...; x = ...;`
//! per field that fires the lint for every variant.
#![allow(unused_assignments)]

use std::path::PathBuf;

use miette::{Diagnostic, NamedSource, SourceSpan};
use thiserror::Error;

/// Fatal errors raised while parsing a SPICE netlist.
///
/// Every variant that points at a location in source carries both the
/// [`NamedSource`] and a [`SourceSpan`] so it renders nicely under
/// `miette`'s fancy reporter.
#[derive(Error, Debug, Diagnostic)]
pub enum SpiceParseError {
    /// First character of an element line was not a recognised device prefix.
    #[error("unknown device prefix '{prefix}'")]
    #[diagnostic(
        code(sindr::spice::unknown_device),
        help("supported device prefixes: R, L, C, V, I, D, Q, X")
    )]
    UnknownDevice {
        /// The unrecognised prefix character.
        prefix: char,
        /// Source text the error was found in.
        #[source_code]
        src: NamedSource<String>,
        /// Span pointing at the offending prefix.
        #[label("unknown prefix")]
        bad_span: SourceSpan,
    },

    /// A `.<card>` directive that the parser does not recognise.
    #[error("unknown control card '.{card}'")]
    #[diagnostic(
        code(sindr::spice::unknown_card),
        help("supported cards: .subckt/.ends, .model, .param, .include, .lib, .tran, .dc, .ac, .op, .end")
    )]
    UnknownCard {
        /// The card name without its leading `.`.
        card: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the card token.
        #[label("unknown card")]
        bad_span: SourceSpan,
    },

    /// Device prefix is recognised by SPICE but not implemented by the parser.
    #[error("unsupported device '{device}'")]
    #[diagnostic(
        code(sindr::spice::unsupported_device),
        help("the parser currently supports: R, L, C, V, I, D, Q, X. M (MOSFET), J (JFET), and B (behavioural) are not implemented yet.")
    )]
    UnsupportedDevice {
        /// Element id that triggered the error.
        device: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the unsupported element.
        #[label("unsupported device")]
        bad_span: SourceSpan,
    },

    /// `.model` declares a model type the parser does not implement.
    #[error("unsupported model type '{kind}' on model '{name}'")]
    #[diagnostic(
        code(sindr::spice::unsupported_model_type),
        help("sindr currently supports model types: D, NPN, PNP")
    )]
    UnsupportedModelType {
        /// Model type token (e.g. `NMOS`).
        kind: String,
        /// Model name as declared.
        name: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the model type token.
        #[label("unsupported model type")]
        bad_span: SourceSpan,
    },

    /// `X<inst>` references a `.subckt` definition that was never declared.
    #[error("undefined subcircuit '{name}'")]
    #[diagnostic(code(sindr::spice::undefined_subckt))]
    UndefinedSubckt {
        /// Referenced subcircuit name.
        name: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the offending instance.
        #[label("undefined subckt")]
        bad_span: SourceSpan,
    },

    /// A device references a `.model` that was never declared.
    #[error("undefined model '{name}'")]
    #[diagnostic(code(sindr::spice::undefined_model))]
    UndefinedModel {
        /// Referenced model name.
        name: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the offending reference.
        #[label("undefined model")]
        bad_span: SourceSpan,
    },

    /// A `.param` expression references a name that was never bound.
    #[error("undefined parameter '{name}'")]
    #[diagnostic(code(sindr::spice::undefined_param))]
    UndefinedParam {
        /// Referenced parameter name.
        name: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the unresolved reference.
        #[label("undefined parameter")]
        bad_span: SourceSpan,
    },

    /// Parameter expressions form a cycle (`a = b * 2; b = a + 1`).
    #[error("circular parameter reference: {}", cycle.join(" -> "))]
    #[diagnostic(
        code(sindr::spice::circular_param),
        help("break the cycle by substituting a literal value for one parameter")
    )]
    CircularParam {
        /// Cycle of parameter names, in dependency order.
        cycle: Vec<String>,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the first parameter in the cycle.
        #[label("part of cycle")]
        bad_span: SourceSpan,
    },

    /// A device or directive received the wrong number of node/argument tokens.
    #[error("arity mismatch on '{card}': expected {expected}, got {got}")]
    #[diagnostic(code(sindr::spice::arity_mismatch))]
    ArityMismatch {
        /// Card or device id whose arity is wrong.
        card: String,
        /// Human-readable description of the expected arity.
        expected: String,
        /// Number of tokens actually supplied.
        got: usize,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the offending line.
        #[label("wrong number of arguments")]
        bad_span: SourceSpan,
    },

    /// A numeric literal could not be parsed as an `f64`.
    #[error("invalid number literal '{raw}'")]
    #[diagnostic(code(sindr::spice::bad_number))]
    BadNumber {
        /// Raw token text.
        raw: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the bad literal.
        #[label("not a number")]
        bad_span: SourceSpan,
    },

    /// A SPICE SI/engineering suffix could not be recognised.
    #[error("unknown SI suffix '{suffix}'")]
    #[diagnostic(
        code(sindr::spice::bad_si_suffix),
        help("supported suffixes: T, G, MEG, K, M, U, N, P, F, MIL")
    )]
    BadSiSuffix {
        /// Suffix tail of the literal.
        suffix: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the suffix.
        #[label("unknown suffix")]
        bad_span: SourceSpan,
    },

    /// Generic grammar failure (catch-all from the winnow parser).
    #[error("syntax error: {message}")]
    #[diagnostic(code(sindr::spice::syntax))]
    Syntax {
        /// Human-readable description of the failure.
        message: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the offending tokens.
        #[label("here")]
        bad_span: SourceSpan,
    },

    /// A `.include` referred to a file that doesn't exist on disk.
    #[error("included file not found: {}", path.display())]
    #[diagnostic(code(sindr::spice::include_not_found))]
    IncludeNotFound {
        /// Path that was searched for.
        path: PathBuf,
        /// Source text of the file containing the `.include`.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the `.include` directive.
        #[label("not found")]
        bad_span: SourceSpan,
    },

    /// A `.include`d file was found but failed to read.
    #[error("I/O error reading included file {}", path.display())]
    #[diagnostic(code(sindr::spice::include_io))]
    IncludeIo {
        /// Path that failed to read.
        path: PathBuf,
        /// Underlying I/O error.
        #[source]
        source: std::io::Error,
    },

    /// `parse_str` saw a relative `.include` but has no base directory to resolve against.
    #[error("relative .include '{include}' is not allowed when parsing from a string")]
    #[diagnostic(
        code(sindr::spice::relative_include_in_string),
        help("use parse_file or parse_str_with_options with an explicit include_search_path")
    )]
    RelativeIncludeInString {
        /// The relative include path as written.
        include: String,
        /// Source text.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the directive.
        #[label("relative include")]
        bad_span: SourceSpan,
    },

    /// `.lib file section` referenced a section name that doesn't exist in the file.
    #[error("section '{section}' not found in library {}", file.display())]
    #[diagnostic(code(sindr::spice::lib_section_not_found))]
    LibSectionNotFound {
        /// Section name as referenced.
        section: String,
        /// Library file searched.
        file: PathBuf,
        /// Source text of the calling file.
        #[source_code]
        src: NamedSource<String>,
        /// Span of the `.lib` directive.
        #[label("missing section")]
        bad_span: SourceSpan,
    },
}

/// Non-fatal warning collected when [`crate::spice::ParseOptions::strict`] is `false`.
///
/// Plain data — callers decide how (or whether) to render. The parser fills
/// `span` and `file` whenever the warning is locatable.
#[derive(Debug, Clone)]
pub struct ParseWarning {
    /// Human-readable warning message.
    pub message: String,
    /// Span in the originating file, if known.
    pub span: Option<SourceSpan>,
    /// File the warning originated in, if known.
    pub file: Option<String>,
}
