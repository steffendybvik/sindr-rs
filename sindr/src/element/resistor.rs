//! A linear resistor.

use crate::error::SimError;

use super::Device;

/// Resistor between two nodes.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Resistor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Resistance (Ω).
    pub resistance: f64,
}

impl Device for Resistor {
    device_identity!();

    fn validate(&self) -> Result<(), SimError> {
        if self.resistance.is_finite() && self.resistance > 0.0 {
            Ok(())
        } else {
            Err(SimError::InvalidResistance(self.id.clone()))
        }
    }
}
