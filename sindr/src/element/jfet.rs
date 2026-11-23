//! A junction FET: the Shockley square law, with optional gate junctions.

use sindr_devices::jfet::JfetKind;

use crate::circuit::JfetParasiticCaps;
use crate::error::SimError;

use super::{Check, Device};

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
