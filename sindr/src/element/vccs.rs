//! A voltage-controlled current source, SPICE's `G`.

use crate::error::SimError;

use super::{Check, Device};

/// Voltage-Controlled Current Source: I_out = gm * V_control.
/// nodes: [out_from, out_to], control_nodes: [ctrl+, ctrl-]
///
/// Like [`CircuitElement::CurrentSource`] and SPICE's `G n+ n-`, a
/// positive `I_out` flows through the source from `nodes[0]` to
/// `nodes[1]`: it is drawn out of `nodes[0]` and pushed into `nodes[1]`.
///
/// [`CircuitElement::CurrentSource`]: crate::CircuitElement::CurrentSource
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Vccs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out_from, out_to]`.
    pub nodes: [String; 2],
    /// Sensing terminals `[ctrl+, ctrl-]`.
    pub control_nodes: [String; 2],
    /// Transconductance gm (A/V): `I_out = gm * V_control`.
    pub gm: f64,
}

impl Device for Vccs {
    device_identity!();

    fn all_nodes(&self) -> Vec<&String> {
        let [a, b] = &self.nodes;
        let [c, d] = &self.control_nodes;
        vec![a, b, c, d]
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("gm", self.gm, " S")
    }
}
