//! A comparator: the op-amp macromodel with no pole, its output clamped.

use crate::error::SimError;
use crate::handle::NodeId;
use crate::op_amp::{OpAmpStamp, Pins, OP_AMP_STAGE};

use super::{AcStamp, Check, DcStamp, Device, Report, TranStamp};
use crate::results::OpAmpResult;

#[cfg(feature = "spice")]
use crate::spice::export::{unsupported, Deck, ExportError};

/// Comparator, output referred to ground: gain 1e5 from `v₊ − v₋` with
/// no frequency response, its output clamped to `[v_neg, v_pos]`. It is
/// an op-amp with an [`OpAmpModel`] of that gain and no pole, so it makes
/// the circuit nonlinear (Newton). No hysteresis, no open-collector output.
///
/// [`OpAmpModel`]: crate::circuit::OpAmpModel
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Comparator {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[in_plus, in_minus, out]`.
    pub nodes: [NodeId; 3], // [in_plus, in_minus, out]
    /// Output high level (V): the output's upper limit. Default 15.
    #[cfg_attr(feature = "serde", serde(default = "super::default_v_pos"))]
    pub v_pos: f64,
    /// Output low level (V): the output's lower limit, below
    /// `v_pos`. Default -15.
    #[cfg_attr(feature = "serde", serde(default = "super::default_v_neg"))]
    pub v_neg: f64,
}

impl Device for Comparator {
    device_identity!();

    /// Its output: `Vout` and the output current, and after a DC solve
    /// its input pins.
    fn report(&self, r: &mut Report<'_>) {
        let v_in_plus = r.v(0);
        let v_in_minus = r.v(1);
        let v_out = r.v(2);
        let i = r.branch_current(0);
        r.branch_current_of(&self.id, i);
        r.port(&self.id, v_out, i);
        r.op_amp(|| OpAmpResult {
            id: self.id.clone(),
            v_in_plus,
            v_in_minus,
            v_differential: v_in_plus - v_in_minus,
            v_out,
        });
    }

    /// Linearised at the bias point, as a modelled op-amp (see
    /// [`OpAmpStamp::stamp_ac`]).
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        OpAmpStamp::of_comparator(self, s.branch).stamp_ac(s);
    }

    /// The output to the circuit's ground; the inputs only sense.
    fn dc_paths<'a>(&'a self, ground: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        vec![[&self.nodes[2], ground]]
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let pins: Pins = s.nodes();
        OpAmpStamp::of_comparator(self, s.branch).stamp_dc_at(s.system, &pins, s.v_prev);
        Ok(())
    }

    /// The macromodel's step, from its gain stage's history: with no pole,
    /// its DC rows at the iterate.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let pins: Pins = s.nodes();
        let history = s.history(&self.id, OP_AMP_STAGE);
        OpAmpStamp::of_comparator(self, s.branch)
            .stamp_transient(s.system, &pins, s.integ, &history, s.v_prev);
        Ok(())
    }

    /// Its output current, then its gain-stage voltage, as a modelled
    /// op-amp.
    fn branches(&self) -> usize {
        2
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.finite("v_pos", self.v_pos, " V")?;
        check.finite("v_neg", self.v_neg, " V")?;
        check.rails_ordered(self.v_pos, self.v_neg)
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, _: &mut Deck<'_>, _: &str) -> Result<(), ExportError> {
        Err(unsupported(
            &self.id,
            "the saturating macromodel has no SPICE primitive",
        ))
    }
}
