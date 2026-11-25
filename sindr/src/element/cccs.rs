//! A current-controlled current source, SPICE's `F`.

use crate::error::SimError;
use crate::handle::{ComponentId, NodeId};
use crate::mna::MnaSystem;

use super::{AcStamp, Check, DcStamp, Device, Report};
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::spice::export::{num, Deck, ExportError};

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
    pub nodes: [NodeId; 2],
    /// Id of the voltage source whose current is sensed.
    pub control_source: ComponentId,
    /// Current gain α (A/A): `I_out = alpha * I_control`.
    pub alpha: f64,
}

impl Device for Cccs {
    device_identity!();

    /// With its control source missing, it reads no current.
    fn report(&self, r: &mut Report<'_>) {
        let i = self.alpha * r.control_current().unwrap_or(0.0);
        let v = r.across(0, 1);
        r.port(&self.id, v, i);
    }

    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let Some(control) = s.source_branch(&self.control_source) else {
            return;
        };
        let [from, to] = s.nodes();
        let gain = Complex64::new(self.alpha, 0.0);
        s.add(from, Some(control), gain);
        s.add(to, Some(control), -gain);
    }

    /// None: its output is a current.
    fn dc_paths<'a>(&'a self, _: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        Vec::new()
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(control) = s.control_branch() {
            let nodes = s.nodes();
            stamp_cccs_at(s.system, nodes, self.alpha, control);
        }
        Ok(())
    }

    fn control_source(&self) -> Option<&str> {
        Some(&self.control_source)
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("alpha", self.alpha, "")
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'F'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        let value = num(&self.id, "alpha", self.alpha)?;
        deck.current_controlled(name, &self.id, &self.nodes, &self.control_source, value)
    }
}

/// Stamp a Current-Controlled Current Source (CCCS).
///
/// I_out = alpha * I_control
/// nodes: [from, to] (current flows from->to)
///
/// The controlling current is the branch current of another voltage source.
///
/// Terminals as matrix indices (`None` is ground).
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
