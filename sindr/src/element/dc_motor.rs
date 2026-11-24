//! A brushed DC motor. The analyses replace it with its equivalent
//! circuit before solving: the armature as R, L and a back-EMF source, and
//! the shaft as an analogue network whose node voltage is its speed.

use crate::error::SimError;
use crate::handle::NodeId;

use super::{stamped_as_parts, AcStamp, Check, DcStamp, Device, Report, TranStamp};
use crate::ratings::RatingKind;

#[cfg(feature = "spice")]
use crate::spice::export::{Deck, ExportError};

/// A brushed DC motor's electrical and mechanical constants, in SI units.
///
/// The armature: `V = R·i + L·di/dt + ke·ω`. The shaft:
/// `J·dω/dt = kt·i − B·ω − load_torque`. Speed `ω` is in rad/s.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct DcMotorParams {
    /// Armature resistance `R` (Ω), positive.
    pub resistance: f64,
    /// Armature inductance `L` (H), zero or positive. Zero leaves it out.
    #[cfg_attr(feature = "serde", serde(default))]
    pub inductance: f64,
    /// Back-EMF constant `ke` (V·s/rad), positive.
    pub ke: f64,
    /// Torque constant `kt` (N·m/A), positive. Equal to `ke` for an ideal
    /// motor in SI units.
    pub kt: f64,
    /// Rotor and load inertia `J` (kg·m²), zero or positive. Zero leaves the
    /// shaft without inertia: its speed follows the current at once.
    #[cfg_attr(feature = "serde", serde(default))]
    pub inertia: f64,
    /// Viscous friction `B` (N·m·s/rad), zero or positive.
    #[cfg_attr(feature = "serde", serde(default))]
    pub friction: f64,
    /// Load torque on the shaft (N·m), opposing rotation when positive. A
    /// [`TransientSession`](crate::TransientSession) changes it with
    /// [`Param::LoadTorque`](crate::session::Param::LoadTorque).
    #[cfg_attr(feature = "serde", serde(default))]
    pub load_torque: f64,
}

impl DcMotorParams {
    /// Constants from the armature and the motor constant, with `kt = ke =
    /// k`, no inductance, inertia, friction or load.
    pub fn new(resistance: f64, k: f64) -> Self {
        Self {
            resistance,
            inductance: 0.0,
            ke: k,
            kt: k,
            inertia: 0.0,
            friction: 0.0,
            load_torque: 0.0,
        }
    }

    /// Constants from a datasheet's figures at `rated_voltage` (V): the
    /// no-load speed (rpm) and current (A), and the stall torque (N·m) and
    /// current (A).
    ///
    /// `R = V/I_stall`, `kt = T_stall/I_stall`, `ke = (V − R·I_0)/ω_0` and
    /// `B = kt·I_0/ω_0`, so the motor runs at exactly the no-load speed and
    /// current with no load, and stalls at exactly the stall current and
    /// torque. Inductance and inertia aren't on most datasheets: set them
    /// after.
    pub fn from_datasheet(
        rated_voltage: f64,
        no_load_rpm: f64,
        no_load_current: f64,
        stall_torque: f64,
        stall_current: f64,
    ) -> Self {
        let resistance = rated_voltage / stall_current;
        let omega_0 = no_load_rpm * std::f64::consts::TAU / 60.0;
        let kt = stall_torque / stall_current;
        Self {
            resistance,
            inductance: 0.0,
            ke: (rated_voltage - resistance * no_load_current) / omega_0,
            kt,
            inertia: 0.0,
            friction: kt * no_load_current / omega_0,
            load_torque: 0.0,
        }
    }

    /// This, with armature inductance `l` (H).
    pub fn with_inductance(mut self, l: f64) -> Self {
        self.inductance = l;
        self
    }

    /// This, with inertia `j` (kg·m²).
    pub fn with_inertia(mut self, j: f64) -> Self {
        self.inertia = j;
        self
    }

    /// This, with viscous friction `b` (N·m·s/rad).
    pub fn with_friction(mut self, b: f64) -> Self {
        self.friction = b;
        self
    }

    /// This, with load torque `t` (N·m).
    pub fn with_load(mut self, t: f64) -> Self {
        self.load_torque = t;
        self
    }

    /// The steady speed (rad/s) and armature current (A) at `voltage` (V):
    /// the closed form the solver's DC answer matches.
    pub fn steady_state(&self, voltage: f64) -> (f64, f64) {
        let Self {
            resistance: r,
            ke,
            kt,
            friction: b,
            load_torque: tl,
            ..
        } = *self;
        // V = R·i + ke·ω and kt·i = B·ω + T_L.
        let omega = (kt * voltage - r * tl) / (r * b + ke * kt);
        let current = (b * omega + tl) / kt;
        (omega, current)
    }
}

/// Brushed DC motor ([`DcMotorParams`]). `nodes: [a, b]`: current into `a`
/// turns it forwards.
///
/// Its result's `power` is the electrical power it takes, `V·I`. Its
/// speed, torque and shaft power are in
/// [`SimulationResult::motors`](crate::SimulationResult::motors) and each
/// transient step's [`motors`](crate::TimestepSnapshot::motors).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct DcMotor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [NodeId; 2],
    /// Electrical and mechanical constants.
    pub params: DcMotorParams,
}

impl Device for DcMotor {
    device_identity!();

    /// Lowered to its parts before any solve, and they report.
    fn report(&self, _: &mut Report<'_>) {}

    /// Split into its parts by `solver_circuit` first.
    fn stamp_ac(&self, _: &mut AcStamp<'_>) {}

    fn needed_ratings(&self) -> &'static [RatingKind] {
        use RatingKind::*;
        &[Voltage, Current, Power]
    }

    fn stamp_dc(&self, _: &mut DcStamp<'_>) -> Result<(), SimError> {
        Err(stamped_as_parts(&self.id))
    }

    /// Nothing: the transient steps its parts.
    fn stamp_transient(&self, _: &mut TranStamp<'_>) -> Result<(), SimError> {
        Ok(())
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        let p = &self.params;
        check.positive("params.resistance", p.resistance, " Ω")?;
        check.non_negative("params.inductance", p.inductance, " H")?;
        check.positive("params.ke", p.ke, " V·s/rad")?;
        check.positive("params.kt", p.kt, " N·m/A")?;
        check.non_negative("params.inertia", p.inertia, " kg·m²")?;
        check.non_negative("params.friction", p.friction, " N·m·s/rad")?;
        check.finite("params.load_torque", p.load_torque, " N·m")?;
        Ok(())
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        deck.parts(&crate::CircuitElement::DcMotor(self.clone()), name)
    }
}
