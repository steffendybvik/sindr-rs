//! A current-controlled voltage source, SPICE's `H`.

use crate::error::SimError;
use crate::mna::MnaSystem;

use super::{Check, DcStamp, Device};

/// Current-Controlled Voltage Source: V_out = rm * I_control.
/// nodes: [out+, out-], control_source: id of controlling voltage source
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Ccvs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out+, out-]`.
    pub nodes: [String; 2],
    /// Id of the voltage source whose current is sensed.
    pub control_source: String,
    /// Transresistance rm (Ω): `V_out = rm * I_control`.
    pub rm: f64,
}

impl Device for Ccvs {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(control) = s.source_branch(&self.control_source) {
            let nodes = s.nodes();
            stamp_ccvs_at(s.system, nodes, self.rm, s.branch, control);
        }
        Ok(())
    }

    fn branches(&self) -> usize {
        1
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("rm", self.rm, " Ω")
    }
}

/// Stamp a Current-Controlled Voltage Source (CCVS).
///
/// V_out = rm * I_control
/// nodes: [out+, out-], rm = transresistance
///
/// The controlling current is the branch current of another voltage source.
///
/// Terminals as matrix indices (`None` is ground).
pub(crate) fn stamp_ccvs_at(
    system: &mut MnaSystem,
    [p, q]: [Option<usize>; 2],
    rm: f64,
    branch: usize,
    ctrl_branch: usize,
) {
    // B submatrix
    if let Some(pi) = p {
        system.a[(pi, branch)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(qi, branch)] -= 1.0;
    }

    // C submatrix: V(p) - V(q) = rm * I_ctrl
    // branch row: V(p) - V(q) - rm*I_ctrl = 0
    if let Some(pi) = p {
        system.a[(branch, pi)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(branch, qi)] -= 1.0;
    }
    system.a[(branch, ctrl_branch)] -= rm;
}
