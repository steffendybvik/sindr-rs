//! A linear capacitor.

use crate::error::SimError;
use crate::handle::NodeId;

use crate::stamp::stamp_companion_at;

use super::{AcStamp, Check, DcStamp, Device, Report, TranStamp};
use crate::ratings::{RatingKind, RatingProbe};

#[cfg(feature = "spice")]
use crate::spice::export::{num, Deck, ExportError};

/// Capacitor. Open-circuit in DC analysis; charges/discharges in
/// transient. `voltage_across` is reported as `nodes[0] − nodes[1]`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Capacitor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [NodeId; 2],
    /// Capacitance (F).
    pub capacitance: f64,
}

impl Device for Capacitor {
    device_identity!();

    /// Open at DC, where it reports nothing across or through it.
    fn report(&self, r: &mut Report<'_>) {
        let Some(step) = r.step() else {
            r.component(&self.id, 0.0, 0.0, 0.0);
            return;
        };
        let v = r.across(0, 1);
        let history = r.capacitor(0);
        let (g, i_hist) = step.integ.capacitor(self.capacitance, &history);
        let i = g * v + i_hist;
        r.set_capacitor(0, history.push(v, i));
        r.port(&self.id, v, i);
    }

    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let y = s.s * self.capacitance;
        let nodes = s.nodes();
        s.admittance(nodes, y);
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        &[RatingKind::Voltage]
    }

    /// A polarised capacitor's + terminal is `nodes[0]`.
    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        match kind {
            RatingKind::ReverseVoltage => Some((-m.port()?.voltage_across).max(0.0)),
            _ => m.port_rating(kind),
        }
    }

    /// None: open at DC.
    fn dc_paths<'a>(&'a self, _: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        Vec::new()
    }

    fn stamp_dc(&self, _: &mut DcStamp<'_>) -> Result<(), SimError> {
        // Open at DC.
        Ok(())
    }

    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let (g, i_hist) = s.integ.capacitor(self.capacitance, s.capacitor(0));
        let nodes = s.nodes();
        stamp_companion_at(s.system, nodes, g, i_hist);
        Ok(())
    }

    fn capacitor_histories(&self) -> usize {
        1
    }

    fn is_reactive(&self) -> bool {
        true
    }

    /// A zero capacitor is an open circuit, as in SPICE, so only a
    /// negative or non-finite capacitance is refused.
    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).non_negative("capacitance", self.capacitance, " F")
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'C'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        let value = num(&self.id, "capacitance", self.capacitance)?;
        deck.two_terminal(name, &self.nodes, value)
    }
}
