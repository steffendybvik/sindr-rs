//! A two-terminal switch, open or closed.

use crate::error::SimError;
use crate::stamp::{stamp_resistor_at, SWITCH_R_CLOSED, SWITCH_R_OPEN};

use super::{AcStamp, DcStamp, Device};

/// Switch between two nodes. Modeled as a resistor with very low (closed)
/// or very high (open) resistance for DC analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Switch {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// When true the switch conducts (closed); when false it is open.
    pub closed: bool,
}

impl Device for Switch {
    device_identity!();

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
}
