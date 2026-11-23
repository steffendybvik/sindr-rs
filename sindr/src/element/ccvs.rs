//! A current-controlled voltage source, SPICE's `H`.

use crate::error::SimError;

use super::{Check, Device};

/// Current-Controlled Voltage Source: V_out = rm * I_control.
/// nodes: [out+, out-], control_source: id of controlling voltage source
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Ccvs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out+, out-]`.
    pub nodes: [String; 2],
    /// Id of the voltage source whose current is sensed.
    pub control_source: String,
    /// Transresistance rm (Ω): `V_out = rm * I_control`.
    pub rm: f64,
}

impl Device for Ccvs {
    device_identity!();

    fn branches(&self) -> usize {
        1
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("rm", self.rm, " Ω")
    }
}
