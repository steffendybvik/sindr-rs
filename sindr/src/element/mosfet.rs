//! A MOSFET, its body tied to its source, with an optional body diode.

use nalgebra::DVector;
use sindr_devices::mosfet::{MosfetKind, MosfetParams};

use crate::circuit::{DiodeModel, MosfetParasiticCaps};
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;

use super::{Check, DcStamp, Device};

/// MOSFET transistor (3 terminals: gate, drain, source; the body is tied
/// to the source).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Mosfet {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[gate, drain, source]`.
    pub nodes: [String; 3], // [gate, drain, source]
    /// Channel type (NMOS or PMOS).
    pub kind: MosfetKind,
    /// Model parameters (threshold voltage, transconductance, etc.).
    #[cfg_attr(feature = "serde", serde(default))]
    pub params: MosfetParams,
    /// Optional parasitic capacitances (Cgs, Cgd). None = no parasitic caps (default).
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub parasitic_caps: Option<MosfetParasiticCaps>,
    /// A power MOSFET's body diode, as a SPICE diode card: anode at the
    /// source and cathode at the drain for NMOS, the other way round for
    /// PMOS. It conducts when the drain is pulled past the source, as
    /// in a bridge's freewheeling interval. Every analysis solves it as
    /// a [`CircuitElement::Diode`] with id `<id>.body`, which is where
    /// its current is reported. `None` (default) is no body diode.
    ///
    /// [`CircuitElement::Diode`]: crate::CircuitElement::Diode
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub body_diode: Option<DiodeModel>,
}

impl Device for Mosfet {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp) = s.v_prev {
            let nodes = s.nodes();
            stamp_mosfet_companion_at(s.system, nodes, vp, &self.params, self.kind);
        }
        Ok(())
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn is_reactive(&self) -> bool {
        self.parasitic_caps
            .as_ref()
            .is_some_and(|caps| caps.cgs > 0.0 || caps.cgd > 0.0)
    }

    /// Only the noise fields: the rest of a MOSFET's parameters aren't
    /// checked.
    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.non_negative("params.kf", self.params.kf, "")?;
        check.positive("params.af", self.params.af, "")
    }
}

/// Stamp a MOSFET companion model into the MNA system.
///
/// Linearizes the Level 1 MOSFET model at the current operating point
/// and stamps the resulting conductances + current sources.
///
/// nodes: [gate, drain, source].
/// PMOS is handled via polarity flip on terminal voltages. The body is tied
/// to the source.
pub(crate) fn stamp_mosfet_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3],
    v_prev: &DVector<f64>,
    params: &MosfetParams,
    kind: MosfetKind,
) {
    let [g, d, s] = nodes.each_ref().map(|n| node_map.index(n));
    stamp_mosfet_companion_at(system, [g, d, s], v_prev, params, kind);
}

/// [`stamp_mosfet_companion`] between matrix indices `[gate, drain,
/// source]` (`None` is ground).
pub(crate) fn stamp_mosfet_companion_at(
    system: &mut MnaSystem,
    [g, d, s]: [Option<usize>; 3],
    v_prev: &DVector<f64>,
    params: &MosfetParams,
    kind: MosfetKind,
) {
    let vg = g.map_or(0.0, |i| v_prev[i]);
    let vd = d.map_or(0.0, |i| v_prev[i]);
    let vs = s.map_or(0.0, |i| v_prev[i]);

    // The NMOS-frame voltages the companion was linearised at.
    let (vgs, vds) = kind.nmos_frame(vg, vd, vs);
    let comp = crate::device_params::mosfet_op(kind, params, vg, vd, vs);

    // Helper closure
    let stamp_g = |system: &mut MnaSystem, row: Option<usize>, col: Option<usize>, val: f64| {
        if let (Some(r), Some(c)) = (row, col) {
            system.a[(r, c)] += val;
        }
    };

    // For NMOS: Id flows drain->source, controlled by Vgs and Vds. The body
    // is tied to the source, so Vbs = 0 and the body effect (gmb) drops out.
    // Linearized: Id = Id0 + gm*(vgs - Vgs0) + gds*(vds - Vds0)
    //
    // G-matrix stamps (3x3 Jacobian):
    // dId/dVg = gm, dId/dVd = gds, dId/dVs = -(gm + gds)
    // For drain row (current entering drain):
    stamp_g(system, d, g, comp.gm);
    stamp_g(system, d, d, comp.gds);
    stamp_g(system, d, s, -(comp.gm + comp.gds));

    // Source row (KCL: Is = -Id, so dIs/dX = -dId/dX):
    stamp_g(system, s, g, -comp.gm);
    stamp_g(system, s, d, -comp.gds);
    stamp_g(system, s, s, comp.gm + comp.gds);

    // Gate row: Ig = 0 (no gate current in MOSFET), no stamps needed

    // RHS: companion current sources
    // I_eq = Id0 - gm*Vgs0 - gds*Vds0
    let i_eq = comp.id - comp.gm * vgs - comp.gds * vds;

    // Apply PMOS sign correction: physical current flows source->drain
    let sign = match kind {
        MosfetKind::Nmos => 1.0,
        MosfetKind::Pmos => -1.0,
    };

    let i_rhs = sign * i_eq;

    // Stamp RHS: current enters drain, leaves source
    if let Some(di) = d {
        system.b[di] -= i_rhs;
    }
    if let Some(si) = s {
        system.b[si] += i_rhs;
    }
}
