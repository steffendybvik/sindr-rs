//! A ferrite bead. The analyses replace it with its equivalent circuit
//! before solving.

use crate::circuit::FerriteBeadParams;

use super::Device;

/// Ferrite bead ([`FerriteBeadParams`]): its DC resistance at DC,
/// lossy and inductive at high frequency. Its result's `power` is what
/// its two resistances dissipate. `nodes: [a, b]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
}
