//! An op-amp: a near-ideal VCVS, or a macromodel with an [`OpAmpModel`].
//!
//! [`OpAmpModel`]: crate::circuit::OpAmpModel

use crate::circuit::OpAmpModel;
use crate::error::SimError;
use crate::op_amp::{OpAmpStamp, Pins};
use crate::stamp::stamp_vcvs_at;

use super::{Check, DcStamp, Device};

/// Op-amp, output referred to ground.
///
/// With `model: None` it is the near-ideal VCVS sindr has always used:
/// gain 1e5, no frequency response, no output resistance, and `v_pos` /
/// `v_neg` not enforced. With an [`OpAmpModel`] it gets a dominant pole
/// from the gain-bandwidth product, slew-rate limiting, output
/// resistance, input offset and bias current, and an output clamped to
/// `v_pos`/`v_neg`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct OpAmp {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[in_plus, in_minus, out]`.
    pub nodes: [String; 3], // [in_plus, in_minus, out]
    /// Positive supply rail (V): the output's upper limit with a
    /// `model`. Default 15.
    #[cfg_attr(feature = "serde", serde(default = "super::default_v_pos"))]
    pub v_pos: f64,
    /// Negative supply rail (V): the output's lower limit with a
    /// `model`. Default -15.
    #[cfg_attr(feature = "serde", serde(default = "super::default_v_neg"))]
    pub v_neg: f64,
    /// Macromodel parameters; `None` for the near-ideal VCVS.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<OpAmpModel>,
}

impl Device for OpAmp {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let nodes = s.nodes();
        let ground = s.node_map.index(&s.circuit.ground_node);
        match OpAmpStamp::of_op_amp(self, &s.circuit.ground_node, s.branch) {
            // Clamped to its rails.
            Some(op) => op.stamp_dc_at(s.system, &Pins { nodes, ground }, s.v_prev),
            // An ideal high-gain VCVS (gain 1e5), its output from the
            // circuit's ground.
            None => {
                let [plus, minus, out] = nodes;
                stamp_vcvs_at(s.system, [out, ground], [plus, minus], 1e5, s.branch);
            }
        }
        Ok(())
    }

    /// The macromodel adds its gain-stage voltage after the output current.
    fn branches(&self) -> usize {
        if self.model.is_some() {
            2
        } else {
            1
        }
    }

    /// The macromodel clamps its output to the rails.
    fn is_nonlinear(&self) -> bool {
        self.model.is_some()
    }

    /// Without a model the rails aren't enforced, so they need only be
    /// finite.
    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.finite("v_pos", self.v_pos, " V")?;
        check.finite("v_neg", self.v_neg, " V")?;
        let Some(model) = &self.model else {
            return Ok(());
        };
        check.rails_ordered(self.v_pos, self.v_neg)?;
        check.positive("model.gain", model.gain, "")?;
        if let Some(gbw) = model.gbw {
            check.positive("model.gbw", gbw, " Hz")?;
        }
        if let Some(sr) = model.slew_rate {
            check.positive("model.slew_rate", sr, " V/s")?;
            if model.gbw.is_none() {
                return Err(check.fail(
                    "model.slew_rate needs model.gbw: the slew limit acts on the gain stage's pole",
                ));
            }
        }
        check.non_negative("model.r_out", model.r_out, " Ω")?;
        check.finite("model.offset_voltage", model.offset_voltage, " V")?;
        check.finite("model.bias_current", model.bias_current, " A")
    }
}
