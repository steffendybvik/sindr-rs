//! A linear capacitor.

use crate::error::SimError;

use super::{Check, Device};

/// Capacitor. Open-circuit in DC analysis; charges/discharges in
/// transient. `voltage_across` is reported as `nodes[0] − nodes[1]`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Capacitor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Capacitance (F).
    pub capacitance: f64,
}

impl Device for Capacitor {
    device_identity!();

    fn capacitor_histories(&self) -> usize {
        1
    }

    fn is_reactive(&self) -> bool {
        true
    }

    /// A zero capacitor is an open circuit, as in SPICE, so only a
    /// negative or non-finite capacitance is refused.
    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).non_negative("capacitance", self.capacitance, " F")
    }
}
