//! A junction diode with a SPICE diode model.

use crate::circuit::DiodeModel;
use crate::device_params;
use crate::error::SimError;
use crate::handle::NodeId;
use crate::stamp::{stamp_companion_at, stamp_diode_companion_at};

use super::{AcBias, AcStamp, Check, Conductance, DcStamp, Device, TranStamp};
use crate::noise::NoiseSources;
use crate::ratings::{RatingKind, RatingProbe};
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::spice::export::{nominal, Deck, ExportError};

/// Diode (Shockley model, silicon unless `model` says otherwise).
/// `nodes: [anode, cathode]`. Forward current flows from anode → cathode;
/// positive `voltage_across` corresponds to forward bias.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Diode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [NodeId; 2],
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(
        feature = "serde",
        serde(default = "super::default_junction_temperature")
    )]
    pub temperature: f64,
    /// Model parameters (IS, N, RS). `None` is the built-in silicon diode,
    /// the same as `Some(DiodeModel::default())`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<DiodeModel>,
}

/// Key suffix, in the transient's per-element history map, of a diode's
/// stored charge (CJO/TT). Its history holds charges (C), not voltages.
pub(crate) const DIODE_CHARGE: &str = "q";

impl Device for Diode {
    device_identity!();

    fn linearise(&self, b: &AcBias<'_>) -> Vec<Conductance> {
        let params = device_params::diode(self.model.as_ref(), self.temperature);
        let (g_d, _) = sindr_devices::diode::diode_companion(b.across(), &params);
        b.two_terminal(g_d)
    }

    /// A diode's stored charge. The junction, `g_j + sC_j`, sits behind
    /// RS; [`linearise`](Device::linearise) already stamps its `s = 0`
    /// conductance `g_d`, so this adds the rest of
    /// `1/(RS + 1/(g_j + sC_j))`.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let Some(charge) = device_params::diode_charge(self.model.as_ref()) else {
            return;
        };
        let params = device_params::diode(self.model.as_ref(), self.temperature);
        let v_d = s.across();
        let (g_j, c_j) = sindr_devices::diode::junction_small_signal(v_d, &params, &charge);
        let (g_d, _) = sindr_devices::diode::diode_companion(v_d, &params);
        let junction = Complex64::new(g_j, 0.0) + s.s * c_j;
        let [p, q] = s.nodes();
        if s.pencil && params.rs > 0.0 {
            // RS from the anode to an internal node `m`, the junction from
            // `m` to the cathode, less the g_d the linearisation stamps
            // across the diode.
            let m = Some(s.aux());
            let g_rs = Complex64::new(1.0 / params.rs, 0.0);
            let g_d = Complex64::new(g_d, 0.0);
            for (x, y, v) in [(p, m, g_rs), (m, q, junction), (p, q, -g_d)] {
                s.admittance([x, y], v);
            }
            return;
        }
        let y = if params.rs > 0.0 {
            Complex64::new(1.0, 0.0) / (params.rs + 1.0 / junction)
        } else {
            junction
        } - g_d;
        let y = if y.is_finite() {
            y
        } else {
            Complex64::new(0.0, 0.0)
        };
        s.admittance([p, q], y);
    }

    /// An internal node behind RS for a diode that stores charge.
    fn pencil_rows(&self) -> usize {
        let stores_charge = device_params::diode_charge(self.model.as_ref()).is_some();
        if stores_charge && device_params::diode(self.model.as_ref(), self.temperature).rs > 0.0 {
            1
        } else {
            0
        }
    }

    fn noise(&self, n: &mut NoiseSources<'_>) {
        let [a, b] = &self.nodes;
        let i = n.current(&self.id);
        n.shot(&self.id, "", [a, b], i);
        if let Some(m) = self.model.filter(|m| m.kf > 0.0) {
            n.flicker(&self.id, "", [a, b], i, m.kf, m.af);
        }
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        use RatingKind::*;
        &[ReverseVoltage, ForwardCurrent, Power]
    }

    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        m.junction_rating(kind)
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp) = s.v_prev {
            let params = device_params::diode(self.model.as_ref(), self.temperature);
            let nodes = s.nodes();
            stamp_diode_companion_at(s.system, nodes, vp, &params);
        }
        Ok(())
    }

    /// The junction at the iterate, and with a CJO or TT its stored charge
    /// over the step.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let params = device_params::diode(self.model.as_ref(), self.temperature);
        let nodes = s.nodes();
        stamp_diode_companion_at(s.system, nodes, s.v_prev, &params);
        if let Some(charge) = device_params::diode_charge(self.model.as_ref()) {
            let [anode, cathode] = nodes;
            let v = s.voltage(anode) - s.voltage(cathode);
            let (q, c) = sindr_devices::diode::diode_charge(v, &params, &charge);
            let history = s.history(&self.id, DIODE_CHARGE);
            let (g, i_hist) = s.integ.charge(q, c, v, &history);
            stamp_companion_at(s.system, nodes, g, i_hist);
        }
        Ok(())
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn is_reactive(&self) -> bool {
        self.model.as_ref().is_some_and(DiodeModel::stores_charge)
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.positive("temperature", self.temperature, " K")?;
        if let Some(model) = &self.model {
            check.positive("model.is", model.is, " A")?;
            check.positive("model.n", model.n, "")?;
            check.non_negative("model.rs", model.rs, " Ω")?;
            check.non_negative("model.cjo", model.cjo, " F")?;
            check.positive("model.vj", model.vj, " V")?;
            check.within("model.m", model.m, 0.9)?;
            check.within("model.fc", model.fc, 0.95)?;
            check.non_negative("model.tt", model.tt, " s")?;
            if let Some(bv) = model.bv {
                check.positive("model.bv", bv, " V")?;
                check.positive("model.ibv", model.ibv, " A")?;
            }
            check.non_negative("model.kf", model.kf, "")?;
            check.positive("model.af", model.af, "")?;
            check.positive("model.eg", model.eg, " eV")?;
            check.non_negative("model.xti", model.xti, "")?;
        }
        Ok(())
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'D'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        nominal(&self.id, self.temperature)?;
        let model = self.model.unwrap_or_default();
        deck.diode(&self.id, name, &self.nodes, model)
    }
}
