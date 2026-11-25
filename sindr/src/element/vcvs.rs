//! A voltage-controlled voltage source, SPICE's `E`.

use crate::error::SimError;
use crate::handle::NodeId;
use crate::node_map::{Read, Sensed};
use crate::stamp::stamp_vcvs_at;

use super::{AcStamp, Check, DcStamp, Device, Report};
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::spice::export::{num, Deck, ExportError};

/// Voltage-Controlled Voltage Source: V_out = mu * V_control.
/// nodes: [out+, out-], control_nodes: [ctrl+, ctrl-]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Vcvs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out+, out-]`.
    pub nodes: [NodeId; 2],
    /// Sensing terminals `[ctrl+, ctrl-]`.
    pub control_nodes: [NodeId; 2],
    /// Voltage gain μ (V/V): `V_out = gain * V_control`.
    pub gain: f64, // mu
}

impl Device for Vcvs {
    device_identity!();

    fn report(&self, r: &mut Report<'_>) {
        let v = r.across(0, 1);
        let i = r.branch_current(0);
        r.branch_current_of(&self.id, i);
        r.port(&self.id, v, i);
    }

    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let (branch, nodes) = (s.branch, s.nodes());
        s.branch_pair(branch, nodes);
        let g = Complex64::new(self.gain, 0.0);
        let [cp, cq] = Sensed::control_pair(s.sensed());
        if let Some(cp) = cp {
            s.a[(branch, cp)] -= g;
        }
        if let Some(cq) = cq {
            s.a[(branch, cq)] += g;
        }
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let control = Sensed::control_pair(s.sensed());
        let nodes = s.nodes();
        stamp_vcvs_at(s.system, nodes, control, self.gain, s.branch);
        Ok(())
    }

    fn all_nodes(&self) -> Vec<&NodeId> {
        let [a, b] = &self.nodes;
        let [c, d] = &self.control_nodes;
        vec![a, b, c, d]
    }

    fn sensed(&self) -> Vec<Read<'_>> {
        let [p, q] = &self.control_nodes;
        vec![Read::Voltage(p, Some(q))]
    }

    fn branches(&self) -> usize {
        1
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("gain", self.gain, "")
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'E'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        let value = num(&self.id, "gain", self.gain)?;
        deck.controlled(name, &self.nodes, &self.control_nodes, value)
    }
}
