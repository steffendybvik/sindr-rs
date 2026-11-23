//! A junction diode with a SPICE diode model.

use crate::circuit::DiodeModel;
use crate::device_params;
use crate::error::SimError;
use crate::stamp::stamp_diode_companion_at;

use super::{Check, DcStamp, Device};

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
    pub nodes: [String; 2],
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

impl Device for Diode {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        if let Some(vp) = s.v_prev {
            let params = device_params::diode(self.model.as_ref(), self.temperature);
            let nodes = s.nodes();
            stamp_diode_companion_at(s.system, nodes, vp, &params);
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
}
