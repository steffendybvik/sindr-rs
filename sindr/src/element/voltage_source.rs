//! An independent voltage source, DC plus an optional waveform.

use crate::error::SimError;
use crate::handle::NodeId;
use crate::stamp::stamp_voltage_source_at;
use crate::waveform::Waveform;

use super::{AcStamp, Check, DcStamp, Device, Report, TranStamp};

#[cfg(feature = "spice")]
use crate::spice::export::{Deck, ExportError};

/// Independent voltage source. `nodes[0]` is the positive terminal,
/// `nodes[1]` is the negative terminal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct VoltageSource {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[positive, negative]`.
    pub nodes: [NodeId; 2],
    /// DC voltage (V); also the offset added to any `waveform`.
    pub voltage: f64,
    /// Optional time-varying waveform added on top of `voltage`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub waveform: Option<Waveform>,
}

impl VoltageSource {
    /// A DC source of `voltage` (V) with no waveform, `nodes[0]` positive.
    ///
    /// ```
    /// use sindr::element::VoltageSource;
    ///
    /// let v1 = VoltageSource::dc("V1", ["in", "0"], 5.0);
    /// assert_eq!(v1.nodes[0], "in");
    /// assert!(v1.waveform.is_none());
    /// ```
    pub fn dc<N: Into<NodeId>>(id: impl Into<String>, nodes: [N; 2], voltage: f64) -> Self {
        Self {
            id: id.into(),
            nodes: nodes.map(Into::into),
            voltage,
            waveform: None,
        }
    }
}

impl Device for VoltageSource {
    device_identity!();

    /// At a transient step the voltage includes the waveform's value.
    fn report(&self, r: &mut Report<'_>) {
        let v = match r.step() {
            Some(step) => {
                self.voltage
                    + self
                        .waveform
                        .as_ref()
                        .map_or(0.0, |w| w.evaluate(step.source_time))
            }
            None => self.voltage,
        };
        let i = r.branch_current(0);
        r.branch_current_of(&self.id, i);
        r.port(&self.id, v, i);
    }

    /// Zero in AC (a short), with its branch row where an AC sweep puts
    /// its excitation.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let (branch, nodes) = (s.branch, s.nodes());
        s.branch_pair(branch, nodes);
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let nodes = s.nodes();
        stamp_voltage_source_at(s.system, nodes, self.voltage, s.branch);
        Ok(())
    }

    /// The DC voltage plus the waveform at the step's end.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let v = match &self.waveform {
            Some(w) => self.voltage + w.evaluate(s.t),
            None => self.voltage,
        };
        let nodes = s.nodes();
        stamp_voltage_source_at(s.system, nodes, v, s.branch);
        Ok(())
    }

    fn branches(&self) -> usize {
        1
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("voltage", self.voltage, " V")
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'V'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        deck.source(
            &self.id,
            name,
            &self.nodes,
            self.voltage,
            self.waveform.as_ref(),
        )
    }
}
