//! A photodiode: a junction diode in parallel with a photocurrent.

use crate::error::SimError;

use super::{Check, Device};

/// Photodiode: diode + photocurrent source driven by irradiance.
/// nodes: [anode, cathode]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Photodiode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Incident irradiance in W (optical power, not W/m²).
    /// 0.0 = dark, 0.1 = ~50mA photocurrent with default responsivity.
    #[cfg_attr(feature = "serde", serde(default))]
    pub irradiance: f64,
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(
        feature = "serde",
        serde(default = "super::default_junction_temperature")
    )]
    pub temperature: f64,
}

impl Device for Photodiode {
    device_identity!();

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).positive("temperature", self.temperature, " K")
    }
}
