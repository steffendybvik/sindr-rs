//! A linear capacitor.

use crate::error::SimError;

use crate::stamp::stamp_companion_at;

use super::{Check, DcStamp, Device, TranStamp};

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

    fn stamp_dc(&self, _: &mut DcStamp<'_>) -> Result<(), SimError> {
        // Open at DC.
        Ok(())
    }

    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let (g, i_hist) = s.integ.capacitor(self.capacitance, s.capacitor(0));
        let nodes = s.nodes();
        stamp_companion_at(s.system, nodes, g, i_hist);
        Ok(())
    }

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
