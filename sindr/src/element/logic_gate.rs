//! A logic gate at the analog level: input thresholds, the output stage
//! of an I/O pin, and a propagation delay. The analyses replace it with
//! its parts.

use crate::error::SimError;
use crate::handle::NodeId;

use super::digital_pin::{validate_model, PinModel};
use super::{stamped_as_parts, AcStamp, Check, DcStamp, Device, Report, TranStamp};
#[cfg(feature = "spice")]
use crate::spice::export::{Deck, ExportError};

/// The function a [`LogicGate`] computes.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum GateKind {
    /// One input, passed through (a 74HC125 enabled, a 74HC7034).
    Buffer,
    /// One input, inverted (74HC04; 74HC14 with
    /// [`schmitt`](LogicGate::schmitt)).
    Not,
    /// High when every input is high (74HC08).
    And,
    /// High when any input is high (74HC32).
    Or,
    /// Low when every input is high (74HC00; 74HC132 with
    /// [`schmitt`](LogicGate::schmitt)).
    Nand,
    /// Low when any input is high (74HC02).
    Nor,
    /// High when an odd number of inputs are high (74HC86).
    Xor,
    /// High when an even number of inputs are high (74HC266).
    Xnor,
}

impl GateKind {
    /// Whether the gate takes exactly one input; the others take two or
    /// more.
    pub fn is_single_input(self) -> bool {
        matches!(self, GateKind::Buffer | GateKind::Not)
    }

    /// The gate's output for logic inputs, as a truth table gives it.
    pub fn eval(self, inputs: &[bool]) -> bool {
        let all = inputs.iter().all(|&x| x);
        let any = inputs.iter().any(|&x| x);
        let odd = inputs.iter().filter(|&&x| x).count() % 2 == 1;
        match self {
            GateKind::Buffer => any,
            GateKind::Not => !any,
            GateKind::And => all,
            GateKind::Nand => !all,
            GateKind::Or => any,
            GateKind::Nor => !any,
            GateKind::Xor => odd,
            GateKind::Xnor => !odd,
        }
    }
}

/// A logic gate as a CMOS part behaves electrically: input thresholds,
/// the output stage of an I/O pin, and a propagation delay. Not digital
/// simulation: the output is a voltage the circuit loads, and the inputs
/// read whatever voltage they're given.
///
/// `nodes: [inputs…, out, vdd, gnd]`: one input for a
/// [`Buffer`](GateKind::Buffer) or [`Not`](GateKind::Not), two or more
/// for the others, then the output, and the supply and ground its output
/// stage returns to.
///
/// - **Inputs** read [`model.vil`](PinModel::vil) and
///   [`model.vih`](PinModel::vih) from the ground terminal. A plain input
///   switches steeply but smoothly at their midpoint, across a tenth of
///   the band between them, so it reads fully low at `vil` and fully high
///   at `vih` with margin, as a CMOS input does. A
///   [`schmitt`](Self::schmitt) input has hysteresis instead: it turns
///   high at `vih` and back low at `vil`, through an internal state that
///   settles in about 1 ns, so a Schmitt gate makes its circuit reactive.
///   Each input loads its net with [`c_in`](Self::c_in) to ground.
/// - **The output stage** is the [`DigitalPin`](super::DigitalPin)'s, in
///   push-pull: `model.r_high` to the supply driving high, `model.r_low`
///   to ground driving low, the clamp diodes to both rails and
///   `model.c_comp`. The output's level follows the inputs through a
///   continuous blend, so Newton sees no step. The pin's pull resistor
///   and leakage don't apply.
/// - **The delay** [`tpd`](Self::tpd) sits between the inputs and the
///   output stage: once the inputs switch, the output's level slews to its
///   new value at a fixed rate and passes half way `tpd` later, however
///   recently it last switched. A ring of three inverters oscillates near
///   1 / (6 · `tpd`). At 0 the gate has no delay, and it makes its circuit
///   reactive only through `c_in`, `c_comp` and Schmitt inputs.
///
/// Its result's `voltage_across` is `v(out) − v(gnd)`, and its
/// `current_through` is the current into the output from the circuit:
/// positive while it sinks, negative while it sources. `power` is what
/// the output stage dissipates.
///
/// ```
/// use sindr::element::{GateKind, LogicGate, PinModel};
///
/// // A 74HC00 NAND on 5 V with a 9 ns delay.
/// let nand = LogicGate::new(
///     "U1A",
///     GateKind::Nand,
///     ["a".into(), "b".into()],
///     ["y".into(), "vcc".into(), "0".into()],
/// )
/// .with_model(PinModel::cmos(5.0))
/// .with_tpd(9e-9);
/// assert_eq!(nand.inputs().len(), 2);
/// ```
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct LogicGate {
    /// Unique component identifier.
    pub id: String,
    /// What the gate computes.
    pub kind: GateKind,
    /// Terminals `[inputs…, out, vdd, gnd]`.
    pub nodes: Vec<NodeId>,
    /// Input thresholds and the output stage. Default
    /// [`PinModel::cmos(3.3)`](PinModel::cmos).
    #[cfg_attr(feature = "serde", serde(default))]
    pub model: PinModel,
    /// Inputs with hysteresis, turning high at `vih` and low at `vil`, as
    /// a Schmitt-trigger part's do. Default off.
    #[cfg_attr(feature = "serde", serde(default))]
    pub schmitt: bool,
    /// Propagation delay (s), zero or positive. Default 0.
    #[cfg_attr(feature = "serde", serde(default))]
    pub tpd: f64,
    /// Capacitance from each input to ground (F), zero or positive.
    /// Default 0.
    #[cfg_attr(feature = "serde", serde(default))]
    pub c_in: f64,
}

impl LogicGate {
    /// A `kind` gate reading `inputs` and driving `out`, powered from
    /// `[out, vdd, gnd]`, with the defaults: a 3.3 V CMOS model, no
    /// hysteresis, no delay and no input capacitance.
    pub fn new(
        id: impl Into<String>,
        kind: GateKind,
        inputs: impl IntoIterator<Item = NodeId>,
        [out, vdd, gnd]: [NodeId; 3],
    ) -> Self {
        let mut nodes: Vec<NodeId> = inputs.into_iter().collect();
        nodes.extend([out, vdd, gnd]);
        Self {
            id: id.into(),
            kind,
            nodes,
            model: PinModel::default(),
            schmitt: false,
            tpd: 0.0,
            c_in: 0.0,
        }
    }

    /// With `model` for the thresholds and the output stage.
    pub fn with_model(mut self, model: PinModel) -> Self {
        self.model = model;
        self
    }

    /// With Schmitt-trigger inputs.
    pub fn with_schmitt(mut self) -> Self {
        self.schmitt = true;
        self
    }

    /// With a propagation delay of `tpd` (s).
    pub fn with_tpd(mut self, tpd: f64) -> Self {
        self.tpd = tpd;
        self
    }

    /// With `c_in` (F) from each input to ground.
    pub fn with_c_in(mut self, c_in: f64) -> Self {
        self.c_in = c_in;
        self
    }

    /// The input terminals.
    pub fn inputs(&self) -> &[NodeId] {
        &self.nodes[..self.nodes.len().saturating_sub(3)]
    }

    /// The error for an expression of its parts that doesn't parse: a bug
    /// in the lowering, reported rather than panicking.
    pub(crate) fn internal_error(&self, e: impl std::fmt::Display) -> SimError {
        Check::new(&self.id).fail(format_args!("internal expression: {e}"))
    }

    /// The output, supply and ground terminals.
    pub fn outputs(&self) -> [&NodeId; 3] {
        let n = self.nodes.len();
        [&self.nodes[n - 3], &self.nodes[n - 2], &self.nodes[n - 1]]
    }
}

impl Device for LogicGate {
    device_identity!();

    /// The output to each rail; the inputs are open at DC.
    fn dc_paths<'a>(&'a self, _: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        if self.nodes.len() < 3 {
            return Vec::new();
        }
        let [out, vdd, gnd] = self.outputs();
        vec![[out, vdd], [out, gnd]]
    }

    /// Lowered to its parts before any solve, and they report.
    fn report(&self, _: &mut Report<'_>) {}

    /// Nothing: AC analysis stamps its parts.
    fn stamp_ac(&self, _: &mut AcStamp<'_>) {}

    /// As its parts: the input stage, the delay and the output stage.
    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        deck.parts(&crate::CircuitElement::LogicGate(self.clone()), name)
    }

    fn stamp_dc(&self, _: &mut DcStamp<'_>) -> Result<(), SimError> {
        Err(stamped_as_parts(&self.id))
    }

    /// Nothing: the transient steps its parts.
    fn stamp_transient(&self, _: &mut TranStamp<'_>) -> Result<(), SimError> {
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        let inputs = self.nodes.len().saturating_sub(3);
        let fits = if self.kind.is_single_input() {
            inputs == 1
        } else {
            inputs >= 2
        };
        if !fits {
            let want = if self.kind.is_single_input() {
                "one input"
            } else {
                "two or more inputs"
            };
            return Err(check.fail(format_args!(
                "a {:?} gate takes {want} before [out, vdd, gnd], not {} nodes in all",
                self.kind,
                self.nodes.len()
            )));
        }
        validate_model(&check, &self.model)?;
        check.non_negative("tpd", self.tpd, " s")?;
        check.non_negative("c_in", self.c_in, " F")?;
        Ok(())
    }
}
