//! A linear resistor.

use crate::error::SimError;
use crate::stamp::stamp_resistor_at;

use super::{AcStamp, DcStamp, Device};
use crate::noise::NoiseSources;
use crate::ratings::RatingKind;

/// Resistor between two nodes.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Resistor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Resistance (Ω).
    pub resistance: f64,
}

impl Device for Resistor {
    device_identity!();

    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let nodes = s.nodes();
        s.conductance(nodes, 1.0 / self.resistance);
    }

    fn noise(&self, n: &mut NoiseSources<'_>) {
        let [a, b] = &self.nodes;
        n.thermal_27c(&self.id, "", [a, b], self.resistance);
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        &[RatingKind::Power]
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if self.resistance <= 0.0 {
            return Err(SimError::InvalidResistance(self.id.clone()));
        }
        let nodes = s.nodes();
        stamp_resistor_at(s.system, nodes, self.resistance);
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        if self.resistance.is_finite() && self.resistance > 0.0 {
            Ok(())
        } else {
            Err(SimError::InvalidResistance(self.id.clone()))
        }
    }
}
