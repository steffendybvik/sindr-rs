//! An independent voltage source, DC plus an optional waveform.

use crate::error::SimError;
use crate::waveform::Waveform;

use super::{Check, Device};

/// Independent voltage source. `nodes[0]` is the positive terminal,
/// `nodes[1]` is the negative terminal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct VoltageSource {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[positive, negative]`.
    pub nodes: [String; 2],
    /// DC voltage (V); also the offset added to any `waveform`.
    pub voltage: f64,
    /// Optional time-varying waveform added on top of `voltage`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub waveform: Option<Waveform>,
}

impl Device for VoltageSource {
    device_identity!();

    fn branches(&self) -> usize {
        1
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("voltage", self.voltage, " V")
    }
}
