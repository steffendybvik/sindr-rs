//! A linear resistor.

use crate::error::SimError;
use crate::handle::NodeId;
use crate::stamp::stamp_resistor_at;

use super::{AcStamp, DcStamp, Device};
use crate::noise::NoiseSources;
use crate::ratings::RatingKind;

#[cfg(feature = "spice")]
use crate::spice::export::{num, Deck, ExportError};

/// Resistor between two nodes.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Resistor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [NodeId; 2],
    /// Resistance (Ω).
    pub resistance: f64,
}

impl Resistor {
    /// The error for a resistance that is zero, negative or not finite.
    fn bad_resistance(&self) -> SimError {
        SimError::component(
            &self.id,
            format!(
                "resistance {} Ω must be a positive, finite number",
                self.resistance
            ),
        )
    }
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
            return Err(self.bad_resistance());
        }
        let nodes = s.nodes();
        stamp_resistor_at(s.system, nodes, self.resistance);
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        if self.resistance.is_finite() && self.resistance > 0.0 {
            Ok(())
        } else {
            Err(self.bad_resistance())
        }
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        let value = num(&self.id, "resistance", self.resistance)?;
        deck.two_terminal(name, &self.nodes, value)
    }
}
