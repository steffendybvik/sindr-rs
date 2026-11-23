//! A varactor: a diode used for its voltage-dependent junction capacitance.

use crate::error::SimError;
use crate::stamp::stamp_resistor_at;

use super::{DcStamp, Device};

/// Varactor diode: voltage-dependent junction capacitance.
/// Open circuit in DC (a 1 TΩ leak); voltage-dependent capacitor in
/// transient; in AC, its capacitance at the DC bias point.
/// nodes: [anode, cathode]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Varactor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Model parameters (zero-bias capacitance, grading, etc.).
    #[cfg_attr(feature = "serde", serde(default))]
    pub params: sindr_devices::varactor::VaractorParams,
}

impl Device for Varactor {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        // Open at DC: a 1e12 Ω shunt, open for practical purposes.
        let nodes = s.nodes();
        stamp_resistor_at(s.system, nodes, 1e12);
        Ok(())
    }

    fn capacitor_histories(&self) -> usize {
        1
    }

    fn is_reactive(&self) -> bool {
        true
    }
}
