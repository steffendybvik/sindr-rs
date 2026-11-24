//! A fuse: a small resistance, or open once blown.

use crate::error::SimError;
use crate::stamp::{stamp_resistor_at, SWITCH_R_OPEN};

use super::{AcStamp, Check, DcStamp, Device};

#[cfg(feature = "spice")]
use crate::spice::export::{num, Deck, ExportError};

/// Fuse between two nodes.
///
/// Intact (blown=false): stamped as 0.001 Ω (1 mΩ) — negligible voltage drop.
/// Blown (blown=true): stamped as 1e9 Ω — effectively open circuit.
///
/// NOTE: Do NOT use 0 Ω — that would cause a singular MNA matrix.
/// `rating` is stored for display only: the fuse never blows by itself;
/// set `blown` to open it. To flag an overcurrent, put
/// [`Ratings::current`](crate::Ratings::current) on its part.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Fuse {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Current rating (A), for display only: the fuse never blows by itself.
    #[cfg_attr(feature = "serde", serde(default = "default_fuse_rating"))]
    pub rating: f64,
    /// When true, fuse is blown (open circuit). User-settable, like switch.closed.
    #[cfg_attr(feature = "serde", serde(default))]
    pub blown: bool,
}

/// An intact fuse's resistance (Ω).
const FUSE_R_INTACT: f64 = 0.001;

impl Fuse {
    /// The resistance it's stamped and exported as (Ω): 1 mΩ intact,
    /// open once blown.
    pub(crate) fn resistance(&self) -> f64 {
        if self.blown {
            SWITCH_R_OPEN
        } else {
            FUSE_R_INTACT
        }
    }
}

impl Device for Fuse {
    device_identity!();

    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let r = if self.blown { 1e9_f64 } else { 0.001_f64 };
        let nodes = s.nodes();
        s.conductance(nodes, 1.0 / r);
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let resistance = self.resistance();
        let nodes = s.nodes();
        stamp_resistor_at(s.system, nodes, resistance);
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).positive("rating", self.rating, " A")
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        let r = num(&self.id, "resistance", self.resistance())?;
        deck.two_terminal(name, &self.nodes, r)
    }
}

#[cfg(feature = "serde")]
fn default_fuse_rating() -> f64 {
    1.0
}
