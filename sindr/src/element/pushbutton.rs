//! A momentary pushbutton, pressed or released.

use crate::error::SimError;
use crate::stamp::{stamp_resistor_at, SWITCH_R_CLOSED, SWITCH_R_OPEN};

use super::{DcStamp, Device};

/// Pushbutton (momentary switch): identical behaviour to Switch.
/// When closed = true, current flows; when false, open circuit.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Pushbutton {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// When true the button conducts (pressed); when false it is open.
    pub closed: bool,
}

impl Device for Pushbutton {
    device_identity!();

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
