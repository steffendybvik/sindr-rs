//! An IGBT: MOSFET gate control driving a BJT output stage.

use crate::handle::NodeId;
use nalgebra::DVector;

use crate::error::SimError;
use crate::mna::MnaSystem;

use super::{AcBias, AcStamp, Conductance, DcStamp, Device, Report};
use crate::ratings::{RatingKind, RatingProbe};

#[cfg(feature = "spice")]
use crate::spice::export::{unsupported, Deck, ExportError};

/// IGBT: MOSFET gate control + BJT output characteristics.
/// Nonlinear element — requires Newton-Raphson iteration.
/// nodes: [gate, collector, emitter]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Igbt {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[gate, collector, emitter]`.
    pub nodes: [NodeId; 3],
    /// Model parameters (threshold voltage, transconductance, etc.).
    #[cfg_attr(feature = "serde", serde(default))]
    pub params: sindr_devices::igbt::IgbtParams,
}

impl Device for Igbt {
    device_identity!();

    /// Its collector: `Vce`, `Ic` and the dissipation.
    fn report(&self, r: &mut Report<'_>) {
        let vg = r.v(&self.nodes[0]);
        let vc = r.v(&self.nodes[1]);
        let ve = r.v(&self.nodes[2]);
        let vce = vc - ve;
        let comp = sindr_devices::igbt::igbt_companion(vg - ve, vce, &self.params);
        r.component(&self.id, vce, comp.ids, (comp.ids * vce).abs());
    }

    /// gm and g_ce of the same companion as the DC stamp.
    fn linearise(&self, b: &AcBias<'_>) -> Vec<Conductance> {
        let [g, c, e] = b.nodes();
        let v = |n| b.voltage(n);
        let comp = sindr_devices::igbt::igbt_companion(v(g) - v(e), v(c) - v(e), &self.params);
        super::mosfet::fet_conductances([g, c, e], comp.gm, comp.g_ce)
    }

    /// All of it is in [`linearise`](Device::linearise).
    fn stamp_ac(&self, _: &mut AcStamp<'_>) {}

    fn needed_ratings(&self) -> &'static [RatingKind] {
        use RatingKind::*;
        &[Vce, Vgs, Ic, Power]
    }

    /// Terminal voltages from `[gate, collector, emitter]`.
    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        match kind {
            RatingKind::Vce => m.between(1, 2),
            RatingKind::Vgs => m.between(0, 2),
            RatingKind::Ic => Some(m.port()?.current_through.abs()),
            _ => m.port_rating(kind),
        }
    }

    /// Collector to emitter; the gate draws no current.
    fn dc_paths<'a>(&'a self, _: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        let [_, c, e] = &self.nodes;
        vec![[c, e]]
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp) = s.v_prev {
            let nodes = s.nodes();
            stamp_igbt_companion_at(s.system, nodes, vp, &self.params);
        }
        Ok(())
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, _: &mut Deck<'_>, _: &str) -> Result<(), ExportError> {
        Err(unsupported(&self.id, "SPICE has no IGBT primitive"))
    }
}

/// [`stamp_igbt_companion_at`] by node name.
#[cfg(test)]
pub(crate) fn stamp_igbt_companion(
    system: &mut MnaSystem,
    node_map: &crate::node_map::NodeMap,
    nodes: &[NodeId; 3],
    v_prev: &DVector<f64>,
    params: &sindr_devices::igbt::IgbtParams,
) {
    let [g, c, e] = nodes.each_ref().map(|n| node_map.index(n));
    stamp_igbt_companion_at(system, [g, c, e], v_prev, params);
}

/// Stamp an IGBT companion model (nonlinear, called each NR iteration).
///
/// Linearizes IGBT (MOSFET gate + BJT output) at operating point v_prev:
/// gm as a gate-controlled current source, g_ce as a C–E conductance.
/// nodes: [gate, collector, emitter]
///
/// Terminals as matrix indices (`None` is ground).
pub(crate) fn stamp_igbt_companion_at(
    system: &mut MnaSystem,
    [g, c, e]: [Option<usize>; 3],
    v_prev: &DVector<f64>,
    params: &sindr_devices::igbt::IgbtParams,
) {
    let vg = g.map_or(0.0, |i| v_prev[i]);
    let vc = c.map_or(0.0, |i| v_prev[i]);
    let ve = e.map_or(0.0, |i| v_prev[i]);
    let vge = vg - ve;
    let vce = vc - ve;

    let comp = sindr_devices::igbt::igbt_companion(vge, vce, params);

    let stamp_g = |system: &mut MnaSystem, row: Option<usize>, col: Option<usize>, val: f64| {
        if let (Some(r), Some(c)) = (row, col) {
            system.a[(r, c)] += val;
        }
    };

    // Ic = gm·vge + g_ce·vce + i_eq, entering the collector, leaving the emitter.
    // Collector row: dIc/dVg = gm, dIc/dVc = g_ce, dIc/dVe = −(gm + g_ce)
    stamp_g(system, c, g, comp.gm);
    stamp_g(system, c, c, comp.g_ce);
    stamp_g(system, c, e, -(comp.gm + comp.g_ce));
    // Emitter row (Ie = −Ic)
    stamp_g(system, e, g, -comp.gm);
    stamp_g(system, e, c, -comp.g_ce);
    stamp_g(system, e, e, comp.gm + comp.g_ce);

    // Stamp companion current source: I_eq from emitter to collector
    // i_eq = ids - gm*vge - g_ce*vce (companion source value)
    if let Some(ci) = c {
        system.b[ci] -= comp.i_eq;
    }
    if let Some(ei) = e {
        system.b[ei] += comp.i_eq;
    }
}
