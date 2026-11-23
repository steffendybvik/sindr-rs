//! An independent current source, DC plus an optional waveform.

use crate::error::SimError;
use crate::waveform::Waveform;

use super::{Check, Device};

/// Independent current source. Current flows through it from `nodes[0]`
/// to `nodes[1]`: drawn out of `nodes[0]` and pushed into `nodes[1]`, as
/// SPICE's `I`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct CurrentSource {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[from, to]`; current flows from `nodes[0]` to `nodes[1]`.
    pub nodes: [String; 2],
    /// DC current (A); also the offset added to any `waveform`.
    pub current: f64,
    /// Optional time-varying waveform added on top of `current`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub waveform: Option<Waveform>,
}

impl Device for CurrentSource {
    device_identity!();

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("current", self.current, " A")
    }
}
