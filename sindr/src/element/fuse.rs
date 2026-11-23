//! A fuse: a small resistance, or open once blown.

use crate::error::SimError;
use crate::stamp::{stamp_resistor_at, SWITCH_R_OPEN};

use super::{Check, DcStamp, Device};

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

impl Device for Fuse {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        // Intact (blown=false): 0.001 Ω (1 mΩ) — negligible drop.
        // Blown (blown=true):  1e9 Ω — effectively open circuit.
        let resistance = if self.blown { SWITCH_R_OPEN } else { 0.001 };
        let nodes = s.nodes();
        stamp_resistor_at(s.system, nodes, resistance);
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).positive("rating", self.rating, " A")
    }
}

#[cfg(feature = "serde")]
fn default_fuse_rating() -> f64 {
    1.0
}
