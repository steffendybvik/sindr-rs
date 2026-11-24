//! An independent voltage source, DC plus an optional waveform.

use crate::error::SimError;
use crate::stamp::stamp_voltage_source_at;
use crate::waveform::Waveform;

use super::{AcStamp, Check, DcStamp, Device, TranStamp};

/// Independent voltage source. `nodes[0]` is the positive terminal,
/// `nodes[1]` is the negative terminal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct VoltageSource {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[positive, negative]`.
    pub nodes: [String; 2],
    /// DC voltage (V); also the offset added to any `waveform`.
    pub voltage: f64,
    /// Optional time-varying waveform added on top of `voltage`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub waveform: Option<Waveform>,
}

impl Device for VoltageSource {
    device_identity!();

    /// Zero in AC (a short), with its branch row where an AC sweep puts
    /// its excitation.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let (branch, nodes) = (s.branch, s.nodes());
        s.branch_pair(branch, nodes);
        s.expose_vsource(&self.id);
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
}
