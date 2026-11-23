//! A single-pole double-throw switch. The analyses split it into two
//! switches before solving.

use crate::circuit::SpdtPosition;

use super::Device;

/// Single-pole double-throw switch: `nodes: [common, a, b]`. The common
/// terminal connects to `a` or `b` as [`Switch`] does
/// (0.01 Ω), and the other throw is open (1 GΩ). Its result is the path
/// in use: `voltage_across` from common to the selected throw and the
/// current from common into it.
///
/// [`Switch`]: super::Switch
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct SwitchSpdt {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[common, a, b]`.
    pub nodes: [String; 3],
    /// Which throw the common terminal is switched to.
    #[cfg_attr(feature = "serde", serde(default))]
    pub position: SpdtPosition,
}

impl Device for SwitchSpdt {
    device_identity!();
}
