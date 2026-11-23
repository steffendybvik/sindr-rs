//! A light-dependent resistor.

use crate::error::SimError;

use super::{Check, Device};

/// Photoresistor (LDR): resistance varies logarithmically with light.
/// light_level = 0.0 → ~1 MΩ (dark), 1.0 → ~1 kΩ (bright).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Photoresistor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Incident light level, 0.0 (dark, ~1 MΩ) to 1.0 (bright, ~1 kΩ).
    pub light_level: f64,
}

impl Device for Photoresistor {
    device_identity!();

    /// The model clamps the light level into range, so only a non-finite
    /// value is refused.
    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("light_level", self.light_level, "")
    }
}
