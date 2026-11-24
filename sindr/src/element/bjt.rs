//! A bipolar junction transistor: Ebers–Moll with the Early effect.

use nalgebra::DVector;
use sindr_devices::bjt::{self, BjtKind, BjtParams};

use crate::circuit::{BjtModel, BjtParasiticCaps};
use crate::device_params;
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::stamp::stamp_companion_at;

use super::{AcBias, AcStamp, Check, Conductance, DcStamp, Device, TranStamp};
use crate::noise::NoiseSources;

/// BJT transistor (3 terminals): Ebers–Moll with Early effect, solved in
/// DC, transient and AC.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Bjt {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[base, collector, emitter]`.
    pub nodes: [String; 3], // [base, collector, emitter]
    /// Polarity (NPN or PNP).
    pub kind: BjtKind,
    /// Forward current gain β (unitless), positive and finite. Default 100.
    #[cfg_attr(feature = "serde", serde(default = "default_bf"))]
    pub bf: f64,
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(
        feature = "serde",
        serde(default = "super::default_junction_temperature")
    )]
    pub temperature: f64,
    /// Optional parasitic capacitances (Cbe, Cbc). None = no parasitic caps (default).
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub parasitic_caps: Option<BjtParasiticCaps>,
    /// Ebers–Moll parameters besides `bf` (IS, BR, NF, NR, VAF). `None`
    /// is the built-in transistor, the same as `Some(BjtModel::default())`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<BjtModel>,
}

impl Device for Bjt {
    device_identity!();

    /// The full terminal Jacobian `∂(Ib, Ic, Ie)/∂(Vb, Vc, Ve)`, with the
    /// same parameters as the DC solve: gπ, gm, gμ and g_o in every
    /// region. The entries are the same for NPN and PNP (the polarity sign
    /// cancels).
    fn linearise(&self, s: &AcBias<'_>) -> Vec<Conductance> {
        let terminals: [Option<usize>; 3] = s.nodes();
        let [vb, vc, ve] = terminals.map(|n| s.voltage(n));
        let sign = match self.kind {
            BjtKind::Npn => 1.0,
            BjtKind::Pnp => -1.0,
        };
        let params = device_params::bjt(self.bf, self.model.as_ref(), self.temperature);
        let jac = bjt::bjt_terminal_jacobian(sign * (vb - ve), sign * (vb - vc), &params);
        let m = jac.terminal_matrix();
        let mut conds = Vec::with_capacity(9);
        for (r, &row) in terminals.iter().enumerate() {
            for (k, &col) in terminals.iter().enumerate() {
                conds.push((row, col, m[r][k]));
            }
        }
        conds
    }

    /// Its parasitic capacitances; the rest is in
    /// [`linearise`](Device::linearise).
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let Some(pc) = &self.parasitic_caps else {
            return;
        };
        for (_, c, p, q) in bjt_parasitics(&self.nodes, pc) {
            let y = s.s * c;
            let pq = [s.node_map.index(&p), s.node_map.index(&q)];
            s.admittance(pq, y);
        }
    }

    /// Collector and base shot noise at the operating point, and base
    /// flicker noise when the model has KF.
    fn noise(&self, n: &mut NoiseSources<'_>) {
        let Some(op) = n.dc.bjt_results.iter().find(|b| b.id == self.id) else {
            return;
        };
        let (ic, ib) = (op.ic, op.ib);
        let [b, c, e] = &self.nodes;
        n.shot(&self.id, "ic", [c, e], ic);
        n.shot(&self.id, "ib", [b, e], ib);
        if let Some(m) = self.model.filter(|m| m.kf > 0.0) {
            n.flicker(&self.id, "ib", [b, e], ib, m.kf, m.af);
        }
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp) = s.v_prev {
            let params = device_params::bjt(self.bf, self.model.as_ref(), self.temperature);
            let nodes = s.nodes();
            stamp_bjt_companion_at(s.system, nodes, vp, &params, self.kind);
        }
        Ok(())
    }

    /// Its DC model at the iterate, and its parasitic capacitors over the
    /// step.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        self.stamp_dc(&mut s.dc())?;
        if let Some(pc) = &self.parasitic_caps {
            for (junction, c, a, b) in bjt_parasitics(&s.nodes(), pc) {
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
            .is_some_and(|caps| caps.cbe > 0.0 || caps.cbc > 0.0)
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.positive("temperature", self.temperature, " K")?;
        check.positive("bf", self.bf, "")?;
        if let Some(model) = &self.model {
            check.positive("model.is", model.is, " A")?;
            check.positive("model.br", model.br, "")?;
            check.positive("model.nf", model.nf, "")?;
            check.positive("model.nr", model.nr, "")?;
            check.non_negative("model.vaf", model.vaf, " V")?;
            check.non_negative("model.kf", model.kf, "")?;
            check.positive("model.af", model.af, "")?;
            check.positive("model.eg", model.eg, " eV")?;
            check.non_negative("model.xti", model.xti, "")?;
        }
        Ok(())
    }
}

#[cfg(feature = "serde")]
fn default_bf() -> f64 {
    100.0
}

/// A BJT's parasitic capacitors as `(junction, C, terminal, terminal)`,
/// from its terminals `[base, collector, emitter]` (names or matrix
/// indices): Cbe base–emitter and Cbc base–collector, each only if > 0.
pub(crate) fn bjt_parasitics<T: Clone>(
    nodes: &[T; 3],
    pc: &BjtParasiticCaps,
) -> Vec<(&'static str, f64, T, T)> {
    let [b, c, e] = nodes.clone();
    [("be", pc.cbe, b.clone(), e), ("bc", pc.cbc, b, c)]
        .into_iter()
        .filter(|cap| cap.1 > 0.0)
        .collect()
}

/// [`stamp_bjt_companion_at`] by node name.
#[cfg(test)]
pub(crate) fn stamp_bjt_companion(
    system: &mut MnaSystem,
    node_map: &crate::node_map::NodeMap,
    nodes: &[String; 3],
    v_prev: &DVector<f64>,
    params: &BjtParams,
    kind: BjtKind,
) {
    let [b, c, e] = nodes.each_ref().map(|n| node_map.index(n));
    stamp_bjt_companion_at(system, [b, c, e], v_prev, params, kind);
}

/// Stamp a BJT Ebers-Moll companion model into the MNA system.
///
/// The exact Newton linearisation at the current operating point (from
/// v_prev): the 3×3 terminal Jacobian from `bjt_terminal_jacobian` (Early
/// effect included) plus 3 RHS entries.
///
/// nodes: [base, collector, emitter].
/// PNP is handled via polarity multiplier on junction voltages and output currents.
///
/// Terminals as matrix indices (`None` is ground).
pub(crate) fn stamp_bjt_companion_at(
    system: &mut MnaSystem,
    [b, c, e]: [Option<usize>; 3],
    v_prev: &DVector<f64>,
    params: &BjtParams,
    kind: BjtKind,
) {
    // Get node voltages from previous iteration
    let vb = b.map_or(0.0, |i| v_prev[i]);
    let vc = c.map_or(0.0, |i| v_prev[i]);
    let ve = e.map_or(0.0, |i| v_prev[i]);

    // Apply polarity for PNP
    let sign = match kind {
        BjtKind::Npn => 1.0,
        BjtKind::Pnp => -1.0,
    };
    let vbe_eff = sign * (vb - ve);
    let vbc_eff = sign * (vb - vc);

    // Exact Newton linearisation: the currents and their Jacobian come
    // from the same large-signal model (Early effect included), so the
    // stamp is consistent at any bias. The terminal matrix is the same for
    // NPN and PNP; the polarity only flips the currents.
    let (ib, ic) = bjt::bjt_terminal_currents(vbe_eff, vbc_eff, params);
    let jac = bjt::bjt_terminal_jacobian(vbe_eff, vbc_eff, params);
    let matrix = jac.terminal_matrix();
    let terminals = [b, c, e];
    for (row, &r) in terminals.iter().enumerate() {
        for (col, &k) in terminals.iter().enumerate() {
            if let (Some(r), Some(k)) = (r, k) {
                system.a[(r, k)] += matrix[row][col];
            }
        }
    }

    // Norton terms: I(v0) − J·v0 per terminal, in the effective frame,
    // then back to physical currents.
    let ib_eq = ib - jac.dib_dvbe * vbe_eff - jac.dib_dvbc * vbc_eff;
    let ic_eq = ic - jac.dic_dvbe * vbe_eff - jac.dic_dvbc * vbc_eff;
    let ie_eq = -(ib_eq + ic_eq);
    for (node, i_eq) in [(b, ib_eq), (c, ic_eq), (e, ie_eq)] {
        if let Some(n) = node {
            system.b[n] -= sign * i_eq;
        }
    }
}
