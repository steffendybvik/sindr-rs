//! A MOSFET, its body tied to its source, with an optional body diode.

use nalgebra::DVector;
use sindr_devices::mosfet::{MosfetKind, MosfetParams};

use crate::circuit::{DiodeModel, MosfetParasiticCaps};
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::stamp::stamp_companion_at;

use super::{AcBias, AcStamp, Check, Conductance, DcStamp, Device, TranStamp};
use crate::noise::NoiseSources;
use crate::ratings::{RatingKind, RatingProbe};

#[cfg(feature = "spice")]
use crate::spice::export::{num, unsupported, Deck, ExportError};

/// MOSFET transistor (3 terminals: gate, drain, source; the body is tied
/// to the source).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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

    /// gm and gds at the bias point. The body is tied to the source, so
    /// there is no gmb term.
    fn linearise(&self, b: &AcBias<'_>) -> Vec<Conductance> {
        let [g, d, s] = b.nodes();
        let comp = crate::device_params::mosfet_op(
            self.kind,
            &self.params,
            b.voltage(g),
            b.voltage(d),
            b.voltage(s),
        );
        fet_conductances([g, d, s], comp.gm, comp.gds)
    }

    /// Its parasitic capacitances; the rest is in
    /// [`linearise`](Device::linearise).
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let Some(pc) = &self.parasitic_caps else {
            return;
        };
        for (_, c, p, q) in mosfet_parasitics(&self.nodes, pc) {
            let y = s.s * c;
            let pq = [s.node_map.index(&p), s.node_map.index(&q)];
            s.admittance(pq, y);
        }
    }

    /// Channel thermal noise, and flicker noise when the model has KF.
    fn noise(&self, n: &mut NoiseSources<'_>) {
        let [g, d, s] = &self.nodes;
        let comp = crate::device_params::mosfet_op(
            self.kind,
            &self.params,
            n.voltage(g),
            n.voltage(d),
            n.voltage(s),
        );
        n.channel(&self.id, [d, s], comp.gm);
        if self.params.kf > 0.0 {
            n.flicker(
                &self.id,
                "",
                [d, s],
                comp.id,
                self.params.kf,
                self.params.af,
            );
        }
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        use RatingKind::*;
        &[Vds, Vgs, Id, Power]
    }

    /// Terminal voltages from `[gate, drain, source]`.
    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        super::mosfet::fet_rating(kind, m)
    }

    /// Drain to source; the gate draws no current.
    fn dc_paths<'a>(&'a self, _: &'a String) -> Vec<[&'a String; 2]> {
        let [_, d, s] = &self.nodes;
        vec![[d, s]]
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp) = s.v_prev {
            let nodes = s.nodes();
            stamp_mosfet_companion_at(s.system, nodes, vp, &self.params, self.kind);
        }
        Ok(())
    }

    /// Its DC model at the iterate, and its parasitic capacitors over the
    /// step.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        self.stamp_dc(&mut s.dc())?;
        if let Some(pc) = &self.parasitic_caps {
            for (junction, c, a, b) in mosfet_parasitics(&s.nodes(), pc) {
                let (g, i_hist) = s.integ.capacitor(c, &s.history(&self.id, junction));
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
            .is_some_and(|caps| caps.cgs > 0.0 || caps.cgd > 0.0)
    }

    /// Only the noise fields: the rest of a MOSFET's parameters aren't
    /// checked.
    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.non_negative("params.kf", self.params.kf, "")?;
        check.positive("params.af", self.params.af, "")
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'M'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        let id = &self.id;
        let params = &self.params;
        if params.kf != 0.0 {
            return Err(unsupported(
                id,
                "a level-1 card's flicker noise needs a gate oxide sindr doesn't model",
            ));
        }
        let caps = self.parasitic_caps.clone().unwrap_or_default();
        // The card's drain–bulk junction holds a body diode with
        // nothing SPICE's junction lacks; any other is its own D line.
        let plain = DiodeModel {
            is: 0.0,
            cjo: 0.0,
            vj: 0.8,
            m: 0.5,
            fc: 0.5,
            ..DiodeModel::default()
        };
        let (junction, separate) = match &self.body_diode {
            None => (plain, None),
            Some(d)
                if *d
                    == DiodeModel {
                        is: d.is,
                        cjo: d.cjo,
                        vj: d.vj,
                        m: d.m,
                        fc: d.fc,
                        ..DiodeModel::default()
                    } =>
            {
                (*d, None)
            }
            Some(d) => (plain, Some(*d)),
        };
        let card = format!(
            "{}(LEVEL=1 KP={} VTO={} LAMBDA={} GAMMA={} PHI={} CGSO={} CGDO={} IS={} CBD={} PB={} MJ={} FC={})",
            match self.kind {
                MosfetKind::Nmos => "NMOS",
                MosfetKind::Pmos => "PMOS",
            },
            num(id, "kp", params.kp)?,
            num(id, "vto", params.vto)?,
            num(id, "lambda", params.lambda)?,
            num(id, "gamma", params.gamma)?,
            num(id, "phi", params.phi)?,
            num(id, "cgs", caps.cgs)?,
            num(id, "cgd", caps.cgd)?,
            num(id, "is", junction.is)?,
            num(id, "cjo", junction.cjo)?,
            num(id, "vj", junction.vj)?,
            num(id, "m", junction.m)?,
            num(id, "fc", junction.fc)?,
        );
        let model_name = deck.fet_model("mmod", card);
        // SPICE's M line is [drain, gate, source, bulk]; sindr's is
        // [g, d, s], its body tied to the source. W = L = 1 m makes
        // KP and CGSO/CGDO sindr's values as they are.
        let [g, d, s] = self.nodes.clone();
        let line_nodes = deck.nodes(&[d.clone(), g, s.clone(), s.clone()])?;
        deck.line(format!("{name} {line_nodes} {model_name} W=1 L=1"));
        if let Some(diode) = separate {
            let diode_name = deck.extra_name(id, format!("D{name}_body"))?;
            let (anode, cathode) = match self.kind {
                MosfetKind::Nmos => (s, d),
                MosfetKind::Pmos => (d, s),
            };
            deck.diode(id, &diode_name, &[anode, cathode], diode)?;
        }
        Ok(())
    }
}

/// A MOSFET's parasitic capacitors as `(junction, C, terminal, terminal)`,
/// from its terminals `[gate, drain, source]` (names or matrix indices):
/// Cgs gate–source and Cgd gate–drain, each only if > 0.
pub(crate) fn mosfet_parasitics<T: Clone>(
    nodes: &[T; 3],
    pc: &MosfetParasiticCaps,
) -> Vec<(&'static str, f64, T, T)> {
    let [g, d, s] = nodes.clone();
    [("gs", pc.cgs, g.clone(), s), ("gd", pc.cgd, g, d)]
        .into_iter()
        .filter(|cap| cap.1 > 0.0)
        .collect()
}

/// [`stamp_mosfet_companion_at`] by node name.
#[cfg(test)]
pub(crate) fn stamp_mosfet_companion(
    system: &mut MnaSystem,
    node_map: &crate::node_map::NodeMap,
    nodes: &[String; 3],
    v_prev: &DVector<f64>,
    params: &MosfetParams,
    kind: MosfetKind,
) {
    let [g, d, s] = nodes.each_ref().map(|n| node_map.index(n));
    stamp_mosfet_companion_at(system, [g, d, s], v_prev, params, kind);
}

/// Stamp a MOSFET companion model into the MNA system.
///
/// Linearizes the Level 1 MOSFET model at the current operating point
/// and stamps the resulting conductances + current sources.
///
/// nodes: [gate, drain, source].
/// PMOS is handled via polarity flip on terminal voltages. The body is tied
/// to the source.
///
/// Terminals as matrix indices (`None` is ground).
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

/// A three-terminal transconductor's small-signal conductances: `gm` from
/// the control terminal `g` into the channel `d` → `s`, and `gds` across
/// the channel. A MOSFET, JFET and IGBT all linearise to it.
pub(crate) fn fet_conductances(
    [g, d, s]: [Option<usize>; 3],
    gm: f64,
    gds: f64,
) -> Vec<super::Conductance> {
    vec![
        (d, g, gm),
        (d, s, -gm),
        (s, g, -gm),
        (s, s, gm),
        (d, d, gds),
        (s, s, gds),
        (d, s, -gds),
        (s, d, -gds),
    ]
}

/// A FET's ratings, from `[gate, drain, source]`: a MOSFET's and a JFET's.
pub(crate) fn fet_rating(kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
    match kind {
        RatingKind::Vds => m.between(1, 2),
        RatingKind::Vgs => m.between(0, 2),
        RatingKind::Id => Some(m.port()?.current_through.abs()),
        _ => m.port_rating(kind),
    }
}
