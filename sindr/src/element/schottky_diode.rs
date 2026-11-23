//! A Schottky diode: a junction diode with a low forward drop.

use crate::device_params;
use crate::error::SimError;
use crate::stamp::stamp_diode_companion_at;

use super::{Check, DcStamp, Device};

/// Schottky diode: lower forward voltage (~0.3V) than silicon diode.
/// nodes: [anode, cathode]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct SchottkyDiode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(
        feature = "serde",
        serde(default = "super::default_junction_temperature")
    )]
    pub temperature: f64,
}

impl Device for SchottkyDiode {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp) = s.v_prev {
            let params = device_params::schottky(self.temperature);
            let nodes = s.nodes();
            stamp_diode_companion_at(s.system, nodes, vp, &params);
        }
        Ok(())
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).positive("temperature", self.temperature, " K")
    }
}
