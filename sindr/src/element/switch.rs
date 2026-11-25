//! A two-terminal switch, open or closed.

use crate::error::SimError;
use crate::handle::NodeId;
use crate::stamp::{stamp_resistor_at, SWITCH_R_CLOSED, SWITCH_R_OPEN};

use super::{AcStamp, DcStamp, Device, Report};

#[cfg(feature = "spice")]
use crate::spice::export::{num, switch_resistance, Deck, ExportError};

/// Switch between two nodes. Modeled as a resistor with very low (closed)
/// or very high (open) resistance for DC analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Switch {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [NodeId; 2],
    /// When true the switch conducts (closed); when false it is open.
    pub closed: bool,
}

impl Device for Switch {
    device_identity!();

    fn report(&self, r: &mut Report<'_>) {
        let resistance = if self.closed {
            SWITCH_R_CLOSED
        } else {
            SWITCH_R_OPEN
        };
        let v = r.across(0, 1);
        r.port(&self.id, v, v / resistance);
    }

    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let r = if self.closed { 0.01 } else { 1e9 };
        let nodes = s.nodes();
        s.conductance(nodes, 1.0 / r);
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let resistance = if self.closed {
            SWITCH_R_CLOSED
        } else {
            SWITCH_R_OPEN
        };
        let nodes = s.nodes();
        stamp_resistor_at(s.system, nodes, resistance);
        Ok(())
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        let r = switch_resistance(self.closed);
        deck.two_terminal(name, &self.nodes, num(&self.id, "resistance", r)?)
    }
}
