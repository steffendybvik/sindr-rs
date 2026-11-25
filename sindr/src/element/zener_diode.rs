//! A zener diode: a silicon diode with reverse breakdown at `vz`.

use crate::device_params;
use crate::error::SimError;
use crate::handle::NodeId;
use crate::stamp::stamp_diode_companion_at;

use super::{AcStamp, Check, DcStamp, Device, Report};
use crate::ratings::{RatingKind, RatingProbe};

#[cfg(feature = "spice")]
use crate::spice::export::{unsupported, Deck, ExportError};

/// Zener diode between two nodes: a silicon diode with SPICE reverse
/// breakdown, `BV = vz` at `IBV` = 5 mA and 1 Ω of series resistance
/// (the card `D(IS=1e-14 N=1 RS=1 BV=vz IBV=5m)`), so it holds about
/// `vz` at a small zener's 5 mA test current and rises 1 V per amp past
/// it. For another knee, use a [`CircuitElement::Diode`] with a
/// [`DiodeModel`] setting `bv` and `ibv`.
/// nodes: [anode, cathode]
///
/// [`CircuitElement::Diode`]: crate::CircuitElement::Diode
/// [`DiodeModel`]: crate::circuit::DiodeModel
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct ZenerDiode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [NodeId; 2],
    /// Reverse breakdown (Zener) voltage (V).
    pub vz: f64,
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(
        feature = "serde",
        serde(default = "super::default_junction_temperature")
    )]
    pub temperature: f64,
}

impl Device for ZenerDiode {
    device_identity!();

    fn report(&self, r: &mut Report<'_>) {
        let params = crate::device_params::zener_diode(self.vz, self.temperature);
        let v = r.across(0, 1);
        r.port(&self.id, v, sindr_devices::diode::diode_current(v, &params));
    }

    /// Nothing: AC and noise run on the diode it is lowered to.
    fn stamp_ac(&self, _: &mut AcStamp<'_>) {}

    fn needed_ratings(&self) -> &'static [RatingKind] {
        &[RatingKind::Power]
    }

    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        m.junction_rating(kind)
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp) = s.v_prev {
            let params = device_params::zener_diode(self.vz, self.temperature);
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

    #[cfg(feature = "spice")]
    fn write_spice(&self, _: &mut Deck<'_>, _: &str) -> Result<(), ExportError> {
        Err(unsupported(
            &self.id,
            "sindr's breakdown clamp isn't SPICE's BV/IBV breakdown",
        ))
    }
}
