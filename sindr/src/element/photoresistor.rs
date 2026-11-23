//! A light-dependent resistor.

use sindr_devices::photoresistor::{ldr_resistance as rs_ldr_resistance, PhotoresistorParams};

use crate::error::SimError;
use crate::stamp::stamp_resistor_at;

use super::{Check, DcStamp, Device};

/// Photoresistor (LDR): resistance varies logarithmically with light.
/// light_level = 0.0 → ~1 MΩ (dark), 1.0 → ~1 kΩ (bright).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let nodes = s.nodes();
        stamp_resistor_at(s.system, nodes, ldr_resistance(self.light_level));
        Ok(())
    }

    /// The model clamps the light level into range, so only a non-finite
    /// value is refused.
    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("light_level", self.light_level, "")
    }
}

/// Compute LDR (photoresistor) resistance from light level.
pub(crate) fn ldr_resistance(light_level: f64) -> f64 {
    rs_ldr_resistance(light_level, &PhotoresistorParams::default())
}
