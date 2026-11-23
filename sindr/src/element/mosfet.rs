//! A MOSFET, its body tied to its source, with an optional body diode.

use sindr_devices::mosfet::{MosfetKind, MosfetParams};

use crate::circuit::{DiodeModel, MosfetParasiticCaps};
use crate::error::SimError;

use super::{Check, Device};

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
}
