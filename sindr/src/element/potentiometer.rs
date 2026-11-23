//! A potentiometer: two resistors split at the wiper.

use crate::error::SimError;

use super::{Check, Device};

/// Potentiometer: 3-terminal voltage divider.
/// nodes: [top, wiper, bottom]
/// position = 0.0 → wiper at top, 1.0 → wiper at bottom.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

    /// The model clamps the wiper position into range, so only a
    /// non-finite position is refused.
    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.positive("resistance", self.resistance, " Ω")?;
        check.finite("position", self.position, "")
    }
}
