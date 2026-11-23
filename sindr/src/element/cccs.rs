//! A current-controlled current source, SPICE's `F`.

use crate::error::SimError;

use super::{Check, Device};

/// Current-Controlled Current Source: I_out = alpha * I_control.
/// nodes: [out_from, out_to], control_source: id of controlling voltage source
///
/// Like [`CircuitElement::CurrentSource`] and SPICE's `F n+ n-`, a
/// positive `I_out` flows through the source from `nodes[0]` to
/// `nodes[1]`. `I_control` is the controlling source's branch current,
/// positive from its `+` terminal through it to `-`, as in SPICE.
///
/// [`CircuitElement::CurrentSource`]: crate::CircuitElement::CurrentSource
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Cccs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out_from, out_to]`.
    pub nodes: [String; 2],
    /// Id of the voltage source whose current is sensed.
    pub control_source: String,
    /// Current gain α (A/A): `I_out = alpha * I_control`.
    pub alpha: f64,
}

impl Device for Cccs {
    device_identity!();

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("alpha", self.alpha, "")
    }
}
