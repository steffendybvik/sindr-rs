//! Internal AST produced by the lexer/parser before lowering to [`crate::Circuit`].
//!
//! Everything in here is `pub(crate)`. These types are the contract between
//! the parser stage (plans 02-03) and the lowering stage (plans 03-04). They
//! are deliberately broader than what `sindr` accepts so that we can produce
//! good error messages on unsupported constructs rather than failing the
//! grammar.

use std::sync::Arc;

/// Byte-offset span attached to AST nodes. Converted to [`miette::SourceSpan`]
/// at error-report time.
#[derive(Debug, Clone)]
pub(crate) struct Span {
    pub start: usize,
    pub end: usize,
    pub file: Arc<str>,
    /// Text of `file`, so errors raised after parsing can still render the
    /// offending line. Empty for spans not tied to source yet.
    pub src: Arc<String>,
}

impl Span {
    /// A placeholder span with no source attached; the parser driver
    /// replaces it with the card's source line.
    pub(crate) fn detached() -> Self {
        Span {
            start: 0,
            end: 0,
            file: Arc::from(""),
            src: Arc::default(),
        }
    }
}

/// One top-level entry in a netlist (after the title line).
///
/// Subcircuit bodies recursively contain `RawCard`s of their own.
#[derive(Debug, Clone)]
pub(crate) enum RawCard {
    Element(RawElement),
    Model(RawModel),
    Subckt(RawSubckt),
    /// `.param` bindings and the card's source line.
    Param(Vec<(String, ParamExpr)>, Span),
    /// An analysis directive and its source line.
    Analysis(RawAnalysis, Span),
    /// `.ic` / `.nodeset`: `V(node)=value` pairs and the card's source line.
    NodeValues(NodeValueKind, Vec<(String, ParamExpr)>, Span),
    /// `.meas` / `.measure` and the card's source line.
    Measure(RawMeasure, Span),
    /// `.options`: `name=value` pairs and bare flags (`None`), and the
    /// card's source line.
    Options(Vec<(String, Option<ParamExpr>)>, Span),
    /// `.global`: nets shared by every subckt body under the same name.
    Global(Vec<String>),
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
    /// V / I — DC value, optional AC small-signal spec and optional
    /// time-domain waveform, written in any order.
    Source {
        kind: SourceKind,
        dc: Option<ParamExpr>,
        ac: Option<RawAcSpec>,
        waveform: Option<RawWaveform>,
    },
    /// D — references a `.model` declaration.
    Diode { model: String },
    /// Q — references a `.model` declaration; optional substrate node.
    Bjt {
        model: String,
        substrate: Option<String>,
    },
    /// E / G — `n+ n- nc+ nc- value`: the four nodes, output pair first.
    VoltageControlled { value: ParamExpr },
    /// F / H — `n+ n- vname value`: the two output nodes and the voltage
    /// source whose current controls it.
    CurrentControlled { source: String, value: ParamExpr },
    /// K — `lname1 lname2 k`: couples two `L` lines of the same scope into
    /// a transformer. Has no nodes of its own.
    Coupling {
        inductors: [String; 2],
        k: ParamExpr,
    },
    /// X — subcircuit instance with optional parameter overrides.
    Subckt {
        name: String,
        params: Vec<(String, ParamExpr)>,
    },
}

/// `AC [mag [phase]]` on a V/I line. A bare `AC` is magnitude 1 at 0°, as
/// in ngspice; `phase` is in degrees.
#[derive(Debug, Clone)]
pub(crate) struct RawAcSpec {
    pub magnitude: Option<ParamExpr>,
    pub phase: Option<ParamExpr>,
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
}

/// Which card a [`RawCard::NodeValues`] came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum NodeValueKind {
    /// `.ic`: initial node voltages for transient analysis.
    Ic,
    /// `.nodeset`: a starting guess for the DC operating point.
    Nodeset,
}

/// Analysis directive in raw (unevaluated) form.
#[derive(Debug, Clone)]
pub(crate) enum RawAnalysis {
    Op,
    Tran {
        args: Vec<ParamExpr>,
        /// The card ends in `uic`.
        uic: bool,
    },
    Dc {
        source: String,
        args: Vec<ParamExpr>,
        /// A second, outer source: `.dc SRC1 … SRC2 start stop step`.
        outer: Option<(String, Vec<ParamExpr>)>,
    },
    Ac {
        sweep: AcSweepKind,
        points: ParamExpr,
        fstart: ParamExpr,
        fstop: ParamExpr,
    },
    Tf {
        output: crate::TfOutput,
        input: String,
    },
    Sens {
        output: crate::TfOutput,
    },
    Noise {
        node: String,
        reference: Option<String>,
        input: String,
        sweep: AcSweepKind,
        points: ParamExpr,
        fstart: ParamExpr,
        fstop: ParamExpr,
    },
}

/// `.meas <analysis> <name> …` in raw (unevaluated) form.
#[derive(Debug, Clone)]
pub(crate) struct RawMeasure {
    pub analysis: crate::spice::MeasureAnalysis,
    pub name: String,
    pub kind: RawMeasureKind,
    pub from: Option<ParamExpr>,
    pub to: Option<ParamExpr>,
}

/// What a `.meas` card measures.
#[derive(Debug, Clone)]
pub(crate) enum RawMeasureKind {
    /// `AVG|RMS|MIN|MAX|PP|INTEG <signal>`.
    Stat(crate::measure::Signal, crate::measure::Stat),
    /// `FIND <signal> AT=<x>`.
    Find(crate::measure::Signal, ParamExpr),
    /// `WHEN <signal>=<level> [RISE|FALL|CROSS=n|LAST]`.
    When(RawCrossing),
    /// `TRIG … TARG …`.
    Delay(RawCrossing, RawCrossing),
}

/// A crossing in a `.meas` card, its level not yet evaluated.
#[derive(Debug, Clone)]
pub(crate) struct RawCrossing {
    pub signal: crate::measure::Signal,
    pub level: ParamExpr,
    pub edge: crate::measure::Edge,
    pub occurrence: crate::measure::Occurrence,
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
