//! A current-controlled current source, SPICE's `F`.

use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;

use super::{Check, DcStamp, Device};

/// Current-Controlled Current Source: I_out = alpha * I_control.
/// nodes: [out_from, out_to], control_source: id of controlling voltage source
///
/// Like [`CircuitElement::CurrentSource`] and SPICE's `F n+ n-`, a
/// positive `I_out` flows through the source from `nodes[0]` to
/// `nodes[1]`. `I_control` is the controlling source's branch current,
/// positive from its `+` terminal through it to `-`, as in SPICE.
///
/// [`CircuitElement::CurrentSource`]: crate::CircuitElement::CurrentSource
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Cccs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out_from, out_to]`.
    pub nodes: [String; 2],
    /// Id of the voltage source whose current is sensed.
    pub control_source: String,
    /// Current gain α (A/A): `I_out = alpha * I_control`.
    pub alpha: f64,
}

impl Device for Cccs {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(control) = s.source_branch(&self.control_source) {
            let nodes = s.nodes();
            stamp_cccs_at(s.system, nodes, self.alpha, control);
        }
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("alpha", self.alpha, "")
    }
}

/// Stamp a Current-Controlled Current Source (CCCS).
///
/// I_out = alpha * I_control
/// nodes: [from, to] (current flows from->to)
///
/// The controlling current is the branch current of another voltage source.
pub(crate) fn stamp_cccs(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    alpha: f64,
    ctrl_branch: usize,
) {
    stamp_cccs_at(
        system,
        [node_map.index(&nodes[0]), node_map.index(&nodes[1])],
        alpha,
        ctrl_branch,
    );
}

/// [`stamp_cccs`] between matrix indices (`None` is ground).
pub(crate) fn stamp_cccs_at(
    system: &mut MnaSystem,
    [from, to]: [Option<usize>; 2],
    alpha: f64,
    ctrl_branch: usize,
) {
    // I_out = alpha * I_ctrl flows through the source from `from` to `to`,
    // as in SPICE's `F n+ n-`: it leaves `from` and enters `to`.
    if let Some(fi) = from {
        system.a[(fi, ctrl_branch)] += alpha;
    }
    if let Some(ti) = to {
        system.a[(ti, ctrl_branch)] -= alpha;
    }
}
