//! A junction FET: the Shockley square law, with optional gate junctions.

use nalgebra::DVector;
use sindr_devices::jfet::JfetKind;

use crate::circuit::JfetParasiticCaps;
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;

use super::{Check, DcStamp, Device};

/// N-channel or P-channel JFET (Shockley square-law model).
/// nodes: [gate, drain, source]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Jfet {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[gate, drain, source]`.
    pub nodes: [String; 3], // [gate, drain, source]
    /// Channel type (N-channel or P-channel).
    pub kind: JfetKind,
    /// Drain saturation current at Vgs=0 (A). Typical: 1–20 mA.
    #[cfg_attr(feature = "serde", serde(default = "default_jfet_idss"))]
    pub idss: f64,
    /// Pinch-off voltage (V, negative for N-channel, e.g. -2.0).
    #[cfg_attr(feature = "serde", serde(default = "default_jfet_vp"))]
    pub vp: f64,
    /// Optional gate capacitances (CGS, CGD as SPICE junctions). None =
    /// no capacitance (default).
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub parasitic_caps: Option<JfetParasiticCaps>,
}

impl Device for Jfet {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp_prev) = s.v_prev {
            let nodes = s.nodes();
            stamp_jfet_companion_at(s.system, nodes, self.kind, self.idss, self.vp, vp_prev);
        }
        Ok(())
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn is_reactive(&self) -> bool {
        self.parasitic_caps
            .as_ref()
            .is_some_and(JfetParasiticCaps::stores_charge)
    }

    fn validate(&self) -> Result<(), SimError> {
        let Some(caps) = &self.parasitic_caps else {
            return Ok(());
        };
        let check = Check::new(&self.id);
        check.non_negative("parasitic_caps.cgs", caps.cgs, " F")?;
        check.non_negative("parasitic_caps.cgd", caps.cgd, " F")?;
        check.positive("parasitic_caps.pb", caps.pb, " V")?;
        check.within("parasitic_caps.m", caps.m, 0.9)?;
        check.within("parasitic_caps.fc", caps.fc, 0.95)
    }
}

#[cfg(feature = "serde")]
fn default_jfet_idss() -> f64 {
    10e-3 // 10 mA typical
}

#[cfg(feature = "serde")]
fn default_jfet_vp() -> f64 {
    -2.0 // -2V pinch-off (N-channel default)
}

/// Stamp a JFET companion model into the MNA system.
///
/// Linearizes the Shockley square-law JFET model at the current operating point.
/// nodes: [gate, drain, source].
/// P-channel: `jfet_companion` works in the mirrored N-channel frame, and the
/// Norton current is flipped back to the physical one here.
pub(crate) fn stamp_jfet_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3], // [gate, drain, source]
    kind: JfetKind,
    idss: f64,
    vp: f64,
    v_prev: &DVector<f64>,
) {
    let [g, d, s] = nodes.each_ref().map(|n| node_map.index(n));
    stamp_jfet_companion_at(system, [g, d, s], kind, idss, vp, v_prev);
}

/// [`stamp_jfet_companion`] between matrix indices `[gate, drain, source]`
/// (`None` is ground).
pub(crate) fn stamp_jfet_companion_at(
    system: &mut MnaSystem,
    [g, d, s]: [Option<usize>; 3],
    kind: JfetKind,
    idss: f64,
    vp: f64,
    v_prev: &DVector<f64>,
) {
    let vg = g.map_or(0.0, |i| v_prev[i]);
    let vd = d.map_or(0.0, |i| v_prev[i]);
    let vs = s.map_or(0.0, |i| v_prev[i]);

    let vgs = vg - vs;
    let vds = vd - vs;

    let c = sindr_devices::jfet::jfet_companion(vgs, vds, kind, idss, vp);
    // For P-channel, jfet_companion works in the mirrored N-channel frame
    // (vgs' = −vgs, vds' = −vds) and the physical drain current is −Id'.
    // Mirroring both the voltages and the current leaves gm and gds as they
    // are but flips the sign of the Norton term.
    let i_eq = match kind {
        JfetKind::NChannel => c.i_eq,
        JfetKind::PChannel => -c.i_eq,
    };

    // Stamp drain-source conductance gds
    if let Some(di) = d {
        system.a[(di, di)] += c.gds;
    }
    if let Some(si) = s {
        system.a[(si, si)] += c.gds;
    }
    if let (Some(di), Some(si)) = (d, s) {
        system.a[(di, si)] -= c.gds;
        system.a[(si, di)] -= c.gds;
    }

    // Stamp VCCS: gm * Vgs = gm * (Vg - Vs)
    // Current flows drain->source (exits drain, enters source)
    if let (Some(di), Some(gi)) = (d, g) {
        system.a[(di, gi)] += c.gm;
    }
    if let (Some(di), Some(si)) = (d, s) {
        system.a[(di, si)] -= c.gm;
    }
    if let (Some(si), Some(gi)) = (s, g) {
        system.a[(si, gi)] -= c.gm;
    }
    if let (Some(si2), Some(si)) = (s, s) {
        system.a[(si2, si)] += c.gm;
    }

    // Stamp equivalent current source i_eq (flows from source to drain in MNA)
    // Id = gm*Vgs + gds*Vds + i_eq
    if let Some(di) = d {
        system.b[di] -= i_eq;
    }
    if let Some(si) = s {
        system.b[si] += i_eq;
    }
}
