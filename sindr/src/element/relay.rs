//! A relay: a coil whose voltage opens or closes a contact pair.

use crate::error::SimError;

use super::{Check, Device};

/// Relay: 4-terminal component with a coil and a contact pair.
/// nodes: [coil+, coil-, contact1, contact2]
/// Contact closes when |V_coil| >= pickup_voltage. AC and `.tf` take the
/// contact as open or closed at the DC bias point.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Relay {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[coil+, coil-, contact1, contact2]`.
    pub nodes: [String; 4],
    /// Coil resistance (Ω). Default 500.
    #[cfg_attr(feature = "serde", serde(default = "default_relay_coil_resistance"))]
    pub coil_resistance: f64,
    /// Coil voltage magnitude at which the contact closes (V).
    pub pickup_voltage: f64,
    /// Coil inductance in Henry, in series with `coil_resistance` in
    /// transient. 0.0 = purely resistive coil.
    #[cfg_attr(feature = "serde", serde(default))]
    pub inductance: f64,
}

impl Device for Relay {
    device_identity!();

    fn inductor_histories(&self) -> usize {
        usize::from(self.inductance > 0.0)
    }

    /// The contact switches on the coil voltage, which takes Newton
    /// iterations to settle.
    fn is_nonlinear(&self) -> bool {
        true
    }

    fn is_reactive(&self) -> bool {
        self.inductance > 0.0
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.positive("coil_resistance", self.coil_resistance, " Ω")?;
        check.positive("pickup_voltage", self.pickup_voltage, " V")?;
        check.non_negative("inductance", self.inductance, " H")
    }
}

#[cfg(feature = "serde")]
fn default_relay_coil_resistance() -> f64 {
    500.0
}
