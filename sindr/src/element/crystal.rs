//! A quartz crystal: the Butterworth–Van Dyke model. The analyses replace
//! it with its equivalent circuit before solving.

use crate::circuit::CrystalParams;
use crate::error::SimError;

use super::{stamped_as_parts, AcStamp, DcStamp, Device, TranStamp};

/// Quartz crystal: the Butterworth–Van Dyke model ([`CrystalParams`]).
/// An open circuit at DC. Its result's `power` is what the motional
/// resistance dissipates: the drive level a datasheet limits.
/// `nodes: [a, b]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Crystal {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// The equivalent circuit.
    pub params: CrystalParams,
}

impl Device for Crystal {
    device_identity!();

    /// Split into its parts by `solver_circuit` first.
    fn stamp_ac(&self, _: &mut AcStamp<'_>) {}

    fn stamp_dc(&self, _: &mut DcStamp<'_>) -> Result<(), SimError> {
        Err(stamped_as_parts(&self.id))
    }

    /// Nothing: the transient steps its parts.
    fn stamp_transient(&self, _: &mut TranStamp<'_>) -> Result<(), SimError> {
        Ok(())
    }
}
