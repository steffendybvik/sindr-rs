//! An op-amp: a near-ideal VCVS, or a macromodel with an [`OpAmpModel`].
//!
//! [`OpAmpModel`]: crate::circuit::OpAmpModel

use crate::circuit::OpAmpModel;
use crate::error::SimError;
use crate::handle::NodeId;
use crate::op_amp::{OpAmpStamp, Pins, OP_AMP_STAGE};
use crate::stamp::stamp_vcvs_at;

use super::{AcStamp, Check, DcStamp, Device, TranStamp};
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::spice::export::{unsupported, Deck, ExportError};

/// Op-amp, output referred to ground.
///
/// With `model: None` it is the near-ideal VCVS sindr has always used:
/// gain 1e5, no frequency response, no output resistance, and `v_pos` /
/// `v_neg` not enforced. With an [`OpAmpModel`] it gets a dominant pole
/// from the gain-bandwidth product, slew-rate limiting, output
/// resistance, input offset and bias current, and an output clamped to
/// `v_pos`/`v_neg`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct OpAmp {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[in_plus, in_minus, out]`.
    pub nodes: [NodeId; 3], // [in_plus, in_minus, out]
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

    /// A modelled op-amp linearised at the bias point (see
    /// [`OpAmpStamp::stamp_ac`]); without a model, a VCVS of gain 10⁵.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        if let Some(op) = OpAmpStamp::of_op_amp(self, s.branch) {
            op.stamp_ac(s);
            return;
        }
        let branch = s.branch;
        let [in_p, in_m, out] = s.nodes();
        let one = Complex64::new(1.0, 0.0);
        let gain = Complex64::new(1e5, 0.0);
        // The output is referred to ground.
        if let Some(o) = out {
            s.a[(o, branch)] += one;
            s.a[(branch, o)] += one;
        }
        if let Some(p) = in_p {
            s.a[(branch, p)] -= gain;
        }
        if let Some(m) = in_m {
            s.a[(branch, m)] += gain;
        }
    }

    /// The output to the circuit's ground; the inputs only sense (a
    /// modelled op-amp's bias current is a current source).
    fn dc_paths<'a>(&'a self, ground: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        vec![[&self.nodes[2], ground]]
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let nodes = s.nodes();
        match OpAmpStamp::of_op_amp(self, s.branch) {
            // Clamped to its rails.
            Some(op) => op.stamp_dc_at(s.system, &nodes, s.v_prev),
            // An ideal high-gain VCVS (gain 1e5), its output from the
            // circuit's ground.
            None => {
                let [plus, minus, out] = nodes;
                stamp_vcvs_at(s.system, [out, None], [plus, minus], 1e5, s.branch);
            }
        }
        Ok(())
    }

    /// The macromodel's gain stage over the step, from its history; the
    /// near-ideal VCVS as at DC.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let Some(op) = OpAmpStamp::of_op_amp(self, s.branch) else {
            return self.stamp_dc(&mut s.dc());
        };
        let pins: Pins = s.nodes();
        let history = s.history(&self.id, OP_AMP_STAGE);
        op.stamp_transient(s.system, &pins, s.integ, &history, s.v_prev);
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

    #[cfg(feature = "spice")]
    fn write_spice(&self, _: &mut Deck<'_>, _: &str) -> Result<(), ExportError> {
        Err(unsupported(
            &self.id,
            "the saturating macromodel has no SPICE primitive",
        ))
    }
}
