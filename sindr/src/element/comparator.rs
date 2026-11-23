//! A comparator: the op-amp macromodel with no pole, its output clamped.

use crate::error::SimError;
use crate::op_amp::{OpAmpStamp, Pins, OP_AMP_STAGE};

use super::{Check, DcStamp, Device, TranStamp};

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
    pub nodes: [String; 3], // [in_plus, in_minus, out]
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

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let pins = Pins {
            nodes: s.nodes(),
            ground: s.node_map.index(&s.circuit.ground_node),
        };
        OpAmpStamp::of_comparator(self, &s.circuit.ground_node, s.branch)
            .stamp_dc_at(s.system, &pins, s.v_prev);
        Ok(())
    }

    /// The macromodel's step, from its gain stage's history: with no pole,
    /// its DC rows at the iterate.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let pins = Pins {
            nodes: s.nodes(),
            ground: s.node_map.index(&s.circuit.ground_node),
        };
        let history = s.history(&self.id, OP_AMP_STAGE);
        OpAmpStamp::of_comparator(self, &s.circuit.ground_node, s.branch)
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
}
