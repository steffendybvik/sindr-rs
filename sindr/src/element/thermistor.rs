//! An NTC thermistor: a resistance set by its temperature.

use crate::error::SimError;

use super::{Check, Device};

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

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).positive("temperature", self.temperature, " K")
    }
}

#[cfg(feature = "serde")]
fn default_temperature() -> f64 {
    298.15
}
