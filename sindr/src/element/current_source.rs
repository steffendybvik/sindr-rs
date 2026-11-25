//! An independent current source, DC plus an optional waveform.

use crate::error::SimError;
use crate::handle::NodeId;
use crate::stamp::stamp_current_source_at;
use crate::waveform::Waveform;

use super::{AcStamp, Check, DcStamp, Device, Report, TranStamp};

#[cfg(feature = "spice")]
use crate::spice::export::{Deck, ExportError};

/// Independent current source. Current flows through it from `nodes[0]`
/// to `nodes[1]`: drawn out of `nodes[0]` and pushed into `nodes[1]`, as
/// SPICE's `I`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct CurrentSource {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[from, to]`; current flows from `nodes[0]` to `nodes[1]`.
    pub nodes: [NodeId; 2],
    /// DC current (A); also the offset added to any `waveform`.
    pub current: f64,
    /// Optional time-varying waveform added on top of `current`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub waveform: Option<Waveform>,
}

impl Device for CurrentSource {
    device_identity!();

    /// At a transient step the current includes the waveform's value.
    fn report(&self, r: &mut Report<'_>) {
        let v = r.across(&self.nodes[0], &self.nodes[1]);
        let i = match r.step() {
            Some(step) => {
                self.current
                    + self
                        .waveform
                        .as_ref()
                        .map_or(0.0, |w| w.evaluate(step.source_time))
            }
            None => self.current,
        };
        r.port(&self.id, v, i);
    }

    /// Zero in AC: open.
    fn stamp_ac(&self, _: &mut AcStamp<'_>) {}

    /// None: a current source fixes no voltage.
    fn dc_paths<'a>(&'a self, _: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        Vec::new()
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let nodes = s.nodes();
        stamp_current_source_at(s.system, nodes, self.current);
        Ok(())
    }

    /// The DC current plus the waveform at the step's end.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let i = match &self.waveform {
            Some(w) => self.current + w.evaluate(s.t),
            None => self.current,
        };
        let nodes = s.nodes();
        stamp_current_source_at(s.system, nodes, i);
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("current", self.current, " A")
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'I'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        deck.source(
            &self.id,
            name,
            &self.nodes,
            self.current,
            self.waveform.as_ref(),
        )
    }
}
