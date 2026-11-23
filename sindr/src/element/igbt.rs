//! An IGBT: MOSFET gate control driving a BJT output stage.

use super::Device;

/// IGBT: MOSFET gate control + BJT output characteristics.
/// Nonlinear element — requires Newton-Raphson iteration.
/// nodes: [gate, collector, emitter]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Igbt {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[gate, collector, emitter]`.
    pub nodes: [String; 3],
    /// Model parameters (threshold voltage, transconductance, etc.).
    #[cfg_attr(feature = "serde", serde(default))]
    pub params: sindr_devices::igbt::IgbtParams,
}

impl Device for Igbt {
    device_identity!();

    fn is_nonlinear(&self) -> bool {
        true
    }
}
