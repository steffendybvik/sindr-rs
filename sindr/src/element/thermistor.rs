//! An NTC thermistor: a resistance set by its temperature.

use crate::error::SimError;
use crate::stamp::stamp_resistor_at;

use super::{Check, DcStamp, Device};

/// NTC thermistor: resistance varies with temperature (passive, no NR).
/// nodes: [n1, n2]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Thermistor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[n1, n2]`.
    pub nodes: [String; 2],
    /// Temperature in Kelvin, positive and finite. Default: 298.15 K (25°C).
    #[cfg_attr(feature = "serde", serde(default = "default_temperature"))]
    pub temperature: f64,
}

impl Device for Thermistor {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let r = crate::device_params::thermistor_resistance(self.temperature);
        let nodes = s.nodes();
        stamp_resistor_at(s.system, nodes, r);
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).positive("temperature", self.temperature, " K")
    }
}

#[cfg(feature = "serde")]
fn default_temperature() -> f64 {
    298.15
}
