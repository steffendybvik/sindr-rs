//! A voltage-controlled current source, SPICE's `G`.

use crate::error::SimError;
use crate::handle::NodeId;
use crate::mna::MnaSystem;
use crate::node_map::{Read, Sensed};

use super::{AcStamp, Check, DcStamp, Device, Report};
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::spice::export::{num, Deck, ExportError};

/// Voltage-Controlled Current Source: I_out = gm * V_control.
/// nodes: [out_from, out_to], control_nodes: [ctrl+, ctrl-]
///
/// Like [`CircuitElement::CurrentSource`] and SPICE's `G n+ n-`, a
/// positive `I_out` flows through the source from `nodes[0]` to
/// `nodes[1]`: it is drawn out of `nodes[0]` and pushed into `nodes[1]`.
///
/// [`CircuitElement::CurrentSource`]: crate::CircuitElement::CurrentSource
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Vccs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out_from, out_to]`.
    pub nodes: [NodeId; 2],
    /// Sensing terminals `[ctrl+, ctrl-]`.
    pub control_nodes: [NodeId; 2],
    /// Transconductance gm (A/V): `I_out = gm * V_control`.
    pub gm: f64,
}

impl Device for Vccs {
    device_identity!();

    fn report(&self, r: &mut Report<'_>) {
        let i = self.gm * r.across(&self.control_nodes[0], &self.control_nodes[1]);
        let v = r.across(&self.nodes[0], &self.nodes[1]);
        r.port(&self.id, v, i);
    }

    /// The same direction as the DC stamp: out of `from`, into `to`.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let [from, to] = s.nodes();
        let [cp, cq] = Sensed::control_pair(s.sensed());
        let g = Complex64::new(self.gm, 0.0);
        s.add(from, cp, g);
        s.add(from, cq, -g);
        s.add(to, cp, -g);
        s.add(to, cq, g);
    }

    /// None: its output is a current, and it only senses its control
    /// pair.
    fn dc_paths<'a>(&'a self, _: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        Vec::new()
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let control = Sensed::control_pair(s.sensed());
        let nodes = s.nodes();
        stamp_vccs_at(s.system, nodes, control, self.gm);
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

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("gm", self.gm, " S")
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'G'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        // VCCS and CCCS share SPICE's G and F direction, so their nodes
        // go out as they are. Checked against ngspice by the
        // `op_exported_controlled` golden.
        let value = num(&self.id, "gm", self.gm)?;
        deck.controlled(name, &self.nodes, &self.control_nodes, value)
    }
}

/// Stamp a Voltage-Controlled Current Source (VCCS).
///
/// I_out = gm * V_control
/// nodes: [from, to] (current flows from->to), control_nodes: [ctrl+, ctrl-]
///
/// Terminals as matrix indices (`None` is ground).
pub(crate) fn stamp_vccs_at(
    system: &mut MnaSystem,
    [from, to]: [Option<usize>; 2],
    [cp, cq]: [Option<usize>; 2],
    gm: f64,
) {
    // I = gm * (V(cp) - V(cq)) flows through the source from `from` to
    // `to`, as in SPICE's `G n+ n-` and sindr's `CurrentSource`: it leaves
    // node `from` (+ on its KCL row) and enters node `to` (−).
    if let (Some(fi), Some(cpi)) = (from, cp) {
        system.a[(fi, cpi)] += gm;
    }
    if let (Some(fi), Some(cqi)) = (from, cq) {
        system.a[(fi, cqi)] -= gm;
    }
    if let (Some(ti), Some(cpi)) = (to, cp) {
        system.a[(ti, cpi)] -= gm;
    }
    if let (Some(ti), Some(cqi)) = (to, cq) {
        system.a[(ti, cqi)] += gm;
    }
}
