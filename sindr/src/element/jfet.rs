//! A junction FET: the Shockley square law, with optional gate junctions.

use nalgebra::DVector;
use sindr_devices::jfet::JfetKind;

use crate::circuit::JfetParasiticCaps;
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::stamp::stamp_companion_at;

use super::{AcBias, AcStamp, Check, Conductance, DcStamp, Device, TranStamp};
use crate::noise::NoiseSources;
use crate::ratings::{RatingKind, RatingProbe};

/// N-channel or P-channel JFET (Shockley square-law model).
/// nodes: [gate, drain, source]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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

    /// gm and gds at the bias point, the same in the physical frame for
    /// both channel types (see `stamp_jfet_companion`).
    fn linearise(&self, b: &AcBias<'_>) -> Vec<Conductance> {
        let [g, d, s] = b.nodes();
        let v = |n| b.voltage(n);
        let c = sindr_devices::jfet::jfet_companion(
            v(g) - v(s),
            v(d) - v(s),
            self.kind,
            self.idss,
            self.vp,
        );
        super::mosfet::fet_conductances([g, d, s], c.gm, c.gds)
    }

    /// Each gate junction's capacitance at the DC bias; the rest is in
    /// [`linearise`](Device::linearise).
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let Some(pc) = &self.parasitic_caps else {
            return;
        };
        for (_, c0, p, q) in jfet_junctions(&self.nodes, pc) {
            let (p, q) = (s.node_map.index(&p), s.node_map.index(&q));
            let (_, c) = jfet_gate_charge(s.voltage(p) - s.voltage(q), self.kind, pc, c0);
            let y = s.s * c;
            s.admittance([p, q], y);
        }
    }

    /// Channel thermal noise.
    fn noise(&self, n: &mut NoiseSources<'_>) {
        let [g, d, s] = &self.nodes;
        let (vg, vd, vs) = (n.voltage(g), n.voltage(d), n.voltage(s));
        let comp =
            sindr_devices::jfet::jfet_companion(vg - vs, vd - vs, self.kind, self.idss, self.vp);
        n.channel(&self.id, [d, s], comp.gm);
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        use RatingKind::*;
        &[Vds, Vgs, Id, Power]
    }

    /// Terminal voltages from `[gate, drain, source]`.
    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        super::mosfet::fet_rating(kind, m)
    }

    /// Drain to source; the model draws no gate current.
    fn dc_paths<'a>(&'a self, _: &'a String) -> Vec<[&'a String; 2]> {
        let [_, d, s] = &self.nodes;
        vec![[d, s]]
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp_prev) = s.v_prev {
            let nodes = s.nodes();
            stamp_jfet_companion_at(s.system, nodes, self.kind, self.idss, self.vp, vp_prev);
        }
        Ok(())
    }

    /// Its DC model at the iterate, and its gate junctions' charge over
    /// the step.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        self.stamp_dc(&mut s.dc())?;
        if let Some(pc) = &self.parasitic_caps {
            for (junction, c0, a, b) in jfet_junctions(&s.nodes(), pc) {
                let v = s.voltage(a) - s.voltage(b);
                let (q, c) = jfet_gate_charge(v, self.kind, pc, c0);
                let history = s.history(&self.id, junction);
                let (g, i_hist) = s.integ.charge(q, c, v, &history);
                stamp_companion_at(s.system, [a, b], g, i_hist);
            }
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

/// A JFET's gate junctions as `(junction, zero-bias C, terminal,
/// terminal)`, from its terminals `[gate, drain, source]` (names or matrix
/// indices): gate–source and gate–drain, each only if its capacitance is
/// positive. Their histories hold charges (C), not voltages.
pub(crate) fn jfet_junctions<T: Clone>(
    nodes: &[T; 3],
    pc: &JfetParasiticCaps,
) -> Vec<(&'static str, f64, T, T)> {
    let [g, d, s] = nodes.clone();
    [("gs", pc.cgs, g.clone(), s), ("gd", pc.cgd, g, d)]
        .into_iter()
        .filter(|j| j.1 > 0.0)
        .collect()
}

/// Charge (C) on a JFET gate junction of zero-bias capacitance `c0` (F),
/// gate side positive, and its capacitance (F) at `v` (V), gate minus the
/// other terminal. A P-channel part's gate junction points the other way,
/// so its junction voltage and charge are the negatives; the capacitance
/// is the same.
pub(crate) fn jfet_gate_charge(
    v: f64,
    kind: JfetKind,
    pc: &JfetParasiticCaps,
    c0: f64,
) -> (f64, f64) {
    let charge = sindr_devices::diode::JunctionCharge {
        cjo: c0,
        vj: pc.pb,
        m: pc.m,
        fc: pc.fc,
        tt: 0.0,
    };
    let sign = match kind {
        JfetKind::NChannel => 1.0,
        JfetKind::PChannel => -1.0,
    };
    let (q, c) = charge.charge(sign * v, 0.0, 0.0);
    (sign * q, c)
}

/// [`stamp_jfet_companion_at`] by node name.
#[cfg(test)]
pub(crate) fn stamp_jfet_companion(
    system: &mut MnaSystem,
    node_map: &crate::node_map::NodeMap,
    nodes: &[String; 3], // [gate, drain, source]
    kind: JfetKind,
    idss: f64,
    vp: f64,
    v_prev: &DVector<f64>,
) {
    let [g, d, s] = nodes.each_ref().map(|n| node_map.index(n));
    stamp_jfet_companion_at(system, [g, d, s], kind, idss, vp, v_prev);
}

/// Stamp a JFET companion model into the MNA system.
///
/// Linearizes the Shockley square-law JFET model at the current operating point.
/// nodes: [gate, drain, source].
/// P-channel: `jfet_companion` works in the mirrored N-channel frame, and the
/// Norton current is flipped back to the physical one here.
///
/// Terminals as matrix indices (`None` is ground).
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
