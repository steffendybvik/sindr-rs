//! A ferrite bead. The analyses replace it with its equivalent circuit
//! before solving.

use crate::circuit::FerriteBeadParams;
use crate::error::SimError;

use super::{stamped_as_parts, DcStamp, Device};

/// Ferrite bead ([`FerriteBeadParams`]): its DC resistance at DC,
/// lossy and inductive at high frequency. Its result's `power` is what
/// its two resistances dissipate. `nodes: [a, b]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct FerriteBead {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// The equivalent circuit.
    pub params: FerriteBeadParams,
}

impl Device for FerriteBead {
    device_identity!();

    fn stamp_dc(&self, _: &mut DcStamp<'_>) -> Result<(), SimError> {
        Err(stamped_as_parts(&self.id))
    }
}
