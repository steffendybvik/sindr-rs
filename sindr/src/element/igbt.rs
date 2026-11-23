//! An IGBT: MOSFET gate control driving a BJT output stage.

use nalgebra::DVector;

use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;

use super::{DcStamp, Device};

/// IGBT: MOSFET gate control + BJT output characteristics.
/// Nonlinear element — requires Newton-Raphson iteration.
/// nodes: [gate, collector, emitter]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Igbt {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[gate, collector, emitter]`.
    pub nodes: [String; 3],
    /// Model parameters (threshold voltage, transconductance, etc.).
    #[cfg_attr(feature = "serde", serde(default))]
    pub params: sindr_devices::igbt::IgbtParams,
}

impl Device for Igbt {
    device_identity!();

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
}

/// Stamp an IGBT companion model (nonlinear, called each NR iteration).
///
/// Linearizes IGBT (MOSFET gate + BJT output) at operating point v_prev:
/// gm as a gate-controlled current source, g_ce as a C–E conductance.
/// nodes: [gate, collector, emitter]
pub(crate) fn stamp_igbt_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3],
    v_prev: &DVector<f64>,
    params: &sindr_devices::igbt::IgbtParams,
) {
    let [g, c, e] = nodes.each_ref().map(|n| node_map.index(n));
    stamp_igbt_companion_at(system, [g, c, e], v_prev, params);
}

/// [`stamp_igbt_companion`] between matrix indices `[gate, collector,
/// emitter]` (`None` is ground).
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
