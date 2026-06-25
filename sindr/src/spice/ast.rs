//! Internal AST produced by the lexer/parser before lowering to [`crate::Circuit`].
//!
//! Everything in here is `pub(crate)`. These types are the contract between
//! the parser stage (plans 02-03) and the lowering stage (plans 03-04). They
//! are deliberately broader than what `sindr` accepts so that we can produce
//! good error messages on unsupported constructs rather than failing the
//! grammar.

#![allow(dead_code)] // some AST fields are not yet read by the lowering pass

use std::path::PathBuf;
use std::sync::Arc;

/// Byte-offset span attached to AST nodes. Converted to [`miette::SourceSpan`]
/// at error-report time.
#[derive(Debug, Clone)]
pub(crate) struct Span {
    pub start: usize,
    pub end: usize,
    pub file: Arc<str>,
}

/// One top-level entry in a netlist (after the title line).
///
/// Subcircuit bodies recursively contain `RawCard`s of their own.
#[derive(Debug, Clone)]
pub(crate) enum RawCard {
    Element(RawElement),
    Model(RawModel),
    Subckt(RawSubckt),
    Param(Vec<(String, ParamExpr)>),
    Include(PathBuf),
    Lib {
        file: PathBuf,
        section: Option<String>,
    },
    Analysis(RawAnalysis),
    /// `.ends [name]` marker; only meaningful inside a subckt body.
    Ends(Option<String>),
}

/// A device-instance line: `R1 a b 1k`, `Q2 c b e qmod`, `X1 a b nameddiv R=10k`, etc.
#[derive(Debug, Clone)]
pub(crate) struct RawElement {
    pub id: String,
    pub prefix: char,
    pub nodes: Vec<String>,
    pub body: RawElementBody,
    pub span: Span,
}

/// Element-specific tail of a [`RawElement`].
#[derive(Debug, Clone)]
pub(crate) enum RawElementBody {
    /// R / L / C — a single value expression.
    Passive { value: ParamExpr },
    /// V / I — DC value plus optional time-domain waveform.
    Source {
        kind: SourceKind,
        dc: Option<ParamExpr>,
        waveform: Option<RawWaveform>,
    },
    /// D — references a `.model` declaration.
    Diode { model: String },
    /// Q — references a `.model` declaration; optional substrate node.
    Bjt {
        model: String,
        substrate: Option<String>,
    },
    /// X — subcircuit instance with optional parameter overrides.
    Subckt {
        name: String,
        params: Vec<(String, ParamExpr)>,
    },
}

/// Whether a `RawElementBody::Source` is a voltage or current source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SourceKind {
    Voltage,
    Current,
}

/// Time-domain waveform clause attached to V/I sources.
#[derive(Debug, Clone)]
pub(crate) enum RawWaveform {
    Pulse(Vec<ParamExpr>),
    Sin(Vec<ParamExpr>),
    Pwl(Vec<(ParamExpr, ParamExpr)>),
}

/// `.model NAME TYPE (k=v ...)` line.
#[derive(Debug, Clone)]
pub(crate) struct RawModel {
    pub name: String,
    /// Model type as written: `D`, `NPN`, `PNP`, or other (yields `UnsupportedModelType` on lower).
    pub kind: String,
    pub params: Vec<(String, ParamExpr)>,
    pub span: Span,
}

/// `.subckt NAME ports... [params...]` block.
#[derive(Debug, Clone)]
pub(crate) struct RawSubckt {
    pub name: String,
    pub ports: Vec<String>,
    pub defaults: Vec<(String, ParamExpr)>,
    pub body: Vec<RawCard>,
    pub span: Span,
}

/// Analysis directive in raw (unevaluated) form.
#[derive(Debug, Clone)]
pub(crate) enum RawAnalysis {
    Op,
    Tran {
        args: Vec<ParamExpr>,
    },
    Dc {
        source: String,
        args: Vec<ParamExpr>,
    },
    Ac {
        sweep: AcSweepKind,
        points: ParamExpr,
        fstart: ParamExpr,
        fstop: ParamExpr,
    },
}

/// AC sweep style keyword.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AcSweepKind {
    Dec,
    Oct,
    Lin,
}

/// Parameter expression: numbers, references, and arithmetic.
#[derive(Debug, Clone)]
pub(crate) enum ParamExpr {
    Number(f64),
    Ref(String),
    BinOp(Box<ParamExpr>, BinOp, Box<ParamExpr>),
    Pow(Box<ParamExpr>, Box<ParamExpr>),
    Neg(Box<ParamExpr>),
}

/// Binary arithmetic operator inside [`ParamExpr`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BinOp {
    Add,
    Sub,
    Mul,
    Div,
}
