//! A potentiometer: two resistors split at the wiper.

use crate::error::SimError;
use crate::stamp::stamp_resistor_at;

use super::{Check, DcStamp, Device};

/// Potentiometer: 3-terminal voltage divider.
/// nodes: [top, wiper, bottom]
/// position = 0.0 → wiper at top, 1.0 → wiper at bottom.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Potentiometer {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[top, wiper, bottom]`.
    pub nodes: [String; 3],
    /// End-to-end resistance (Ω).
    pub resistance: f64,
    /// Wiper position, 0.0 (at top) to 1.0 (at bottom).
    pub position: f64,
}

impl Device for Potentiometer {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let pos = self.position.clamp(0.001, 0.999);
        let r_top = self.resistance * pos;
        let r_bot = self.resistance * (1.0 - pos);
        let [top, wiper, bottom] = s.nodes();
        stamp_resistor_at(s.system, [top, wiper], r_top);
        stamp_resistor_at(s.system, [wiper, bottom], r_bot);
        Ok(())
    }

    /// The model clamps the wiper position into range, so only a
    /// non-finite position is refused.
    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.positive("resistance", self.resistance, " Ω")?;
        check.finite("position", self.position, "")
    }
}
