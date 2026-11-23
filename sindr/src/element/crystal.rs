//! A quartz crystal: the Butterworth–Van Dyke model. The analyses replace
//! it with its equivalent circuit before solving.

use crate::circuit::CrystalParams;

use super::Device;

/// Quartz crystal: the Butterworth–Van Dyke model ([`CrystalParams`]).
/// An open circuit at DC. Its result's `power` is what the motional
/// resistance dissipates: the drive level a datasheet limits.
/// `nodes: [a, b]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
}
