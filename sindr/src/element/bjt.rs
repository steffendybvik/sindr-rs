//! A bipolar junction transistor: Ebers–Moll with the Early effect.

use sindr_devices::bjt::BjtKind;

use crate::circuit::{BjtModel, BjtParasiticCaps};
use crate::error::SimError;

use super::{Check, Device};

/// BJT transistor (3 terminals): Ebers–Moll with Early effect, solved in
/// DC, transient and AC.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
