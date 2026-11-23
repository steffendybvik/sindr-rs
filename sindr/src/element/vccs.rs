//! A voltage-controlled current source, SPICE's `G`.

use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;

use super::{Check, DcStamp, Device};

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
    pub nodes: [String; 2],
    /// Sensing terminals `[ctrl+, ctrl-]`.
    pub control_nodes: [String; 2],
    /// Transconductance gm (A/V): `I_out = gm * V_control`.
    pub gm: f64,
}

impl Device for Vccs {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let [cp, cq] = &self.control_nodes;
        let control = [s.node_map.index(cp), s.node_map.index(cq)];
        let nodes = s.nodes();
        stamp_vccs_at(s.system, nodes, control, self.gm);
        Ok(())
    }

    fn all_nodes(&self) -> Vec<&String> {
        let [a, b] = &self.nodes;
        let [c, d] = &self.control_nodes;
        vec![a, b, c, d]
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("gm", self.gm, " S")
    }
}

/// Stamp a Voltage-Controlled Current Source (VCCS).
///
/// I_out = gm * V_control
/// nodes: [from, to] (current flows from->to), control_nodes: [ctrl+, ctrl-]
pub(crate) fn stamp_vccs(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    control_nodes: &[String; 2],
    gm: f64,
) {
    stamp_vccs_at(
        system,
        [node_map.index(&nodes[0]), node_map.index(&nodes[1])],
        [
            node_map.index(&control_nodes[0]),
            node_map.index(&control_nodes[1]),
        ],
        gm,
    );
}

/// [`stamp_vccs`] between matrix indices (`None` is ground).
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
