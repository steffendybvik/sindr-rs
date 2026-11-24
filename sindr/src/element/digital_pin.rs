//! A digital I/O pin: the electrical side of a microcontroller or logic-IC
//! pin, after the IBIS I/O buffer. The analyses replace it with its parts.

use crate::circuit::DiodeModel;
use crate::error::SimError;

use super::{stamped_as_parts, AcStamp, Check, DcStamp, Device, TranStamp};
use crate::ratings::RatingKind;
#[cfg(feature = "spice")]
use crate::spice::export::{Deck, ExportError};

/// What a [`DigitalPin`]'s output stage does.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum PinMode {
    /// High-impedance input: the output stage is off, leaving the clamps,
    /// the leakage and the pull resistor, if one is on. A pin out of reset.
    #[default]
    Input,
    /// Push-pull output: `r_high` to the supply when the level is high,
    /// `r_low` to ground when it is low.
    PushPull,
    /// Open-drain output: `r_low` to ground when the level is low,
    /// released (as an input) when it is high.
    OpenDrain,
}

/// A [`DigitalPin`]'s weak pull resistor, `r_pull`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum Pull {
    /// No pull resistor.
    #[default]
    None,
    /// `r_pull` to the supply.
    Up,
    /// `r_pull` to ground.
    Down,
}

/// What a [`DigitalPin`]'s input buffer reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LogicLevel {
    /// At or below `vil`, or between `vil` and `vih` after reading low.
    Low,
    /// At or above `vih`, or between `vil` and `vih` after reading high.
    High,
}

impl From<LogicLevel> for bool {
    fn from(level: LogicLevel) -> Self {
        level == LogicLevel::High
    }
}

/// A [`DigitalPin`]'s electrical model: IBIS's parametric I/O buffer, as
/// a datasheet's DC characteristics give it.
///
/// The output stage is a resistance to each rail, IBIS's pull-up and
/// pull-down. Take each from the datasheet's output levels:
/// `r_high = (VDD − VOH) / IOH`, `r_low = VOL / IOL`. The power and ground
/// clamps are diodes from the pin to each rail, so a pin driven past a rail
/// conducts as its ESD diodes do; `c_comp` is the pad's die capacitance.
///
/// [`Default`] is [`PinModel::cmos(3.3)`](Self::cmos).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, PartialEq)]
pub struct PinModel {
    /// Output resistance driving high (Ω), positive. Default 50 Ω:
    /// VOH = VDD − 0.4 V at 8 mA.
    pub r_high: f64,
    /// Output resistance driving low (Ω), positive. Default 50 Ω:
    /// VOL = 0.4 V at 8 mA.
    pub r_low: f64,
    /// The weak pull resistor's resistance (Ω), positive. Default 40 kΩ.
    pub r_pull: f64,
    /// Both clamp diodes. Default: IS 10⁻¹⁴ A, N 1, RS 10 Ω, so a pin
    /// driven 1 V past a rail conducts about 30 mA.
    pub clamp: DiodeModel,
    /// Die capacitance from pin to ground (F), zero or positive. Default
    /// 0, which keeps a pin from making its circuit reactive; a few pF
    /// gives an output its edges.
    pub c_comp: f64,
    /// Input leakage into the pin, to ground (A), finite. Default 0.
    pub leakage: f64,
    /// Input low threshold (V, from the ground terminal): at or below it
    /// the pin reads low.
    pub vil: f64,
    /// Input high threshold (V, from the ground terminal), above `vil`: at
    /// or above it the pin reads high. Between the two the reading holds,
    /// as a Schmitt trigger's does.
    pub vih: f64,
}

impl PinModel {
    /// A CMOS pin on a `vdd` volt supply: thresholds at 30 % and 70 % of
    /// it, and the other defaults on [`PinModel`].
    pub fn cmos(vdd: f64) -> Self {
        Self {
            r_high: 50.0,
            r_low: 50.0,
            r_pull: 40e3,
            clamp: DiodeModel {
                rs: 10.0,
                ..DiodeModel::default()
            },
            c_comp: 0.0,
            leakage: 0.0,
            vil: 0.3 * vdd,
            vih: 0.7 * vdd,
        }
    }
}

impl Default for PinModel {
    fn default() -> Self {
        Self::cmos(3.3)
    }
}

/// Digital I/O pin, after the IBIS I/O buffer: an output stage to each
/// rail, power and ground clamp diodes, leakage, die capacitance and a
/// weak pull resistor ([`PinModel`]).
/// `nodes: [pin, vdd, gnd]`: the pad, and the supply and ground the pin's
/// buffer returns to.
///
/// [`mode`](Self::mode), [`level`](Self::level) and [`pull`](Self::pull)
/// are what firmware sets; a [`TransientSession`](crate::TransientSession)
/// changes them between steps with [`Param::PinMode`](crate::Param),
/// [`Param::Level`](crate::Param) and [`Param::Pull`](crate::Param), and
/// reads the input back with
/// [`logic_level`](crate::TransientSession::logic_level).
///
/// Its result's `voltage_across` is `v(pin) − v(gnd)` and its
/// `current_through` the current into the pin from the circuit: positive
/// while it sinks, negative while it sources. `power` is what the pin's
/// buffer dissipates. A [`Ratings::current`](crate::Ratings) on its part
/// limits the pin's current, and one on a part named for its
/// [`port`](Self::port) the port's.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct DigitalPin {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[pin, vdd, gnd]`.
    pub nodes: [String; 3],
    /// The output stage. Default [`PinMode::Input`].
    #[cfg_attr(feature = "serde", serde(default))]
    pub mode: PinMode,
    /// The level an output drives, `true` for high. Open drain releases
    /// the pin when it's high. Default low.
    #[cfg_attr(feature = "serde", serde(default))]
    pub level: bool,
    /// The weak pull resistor. Default none.
    #[cfg_attr(feature = "serde", serde(default))]
    pub pull: Pull,
    /// The port the pin belongs to (`"PORTB"`), whose current rating sums
    /// its pins. Scoped like an id in a block.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub port: Option<String>,
    /// The electrical model. Default [`PinModel::cmos(3.3)`](PinModel::cmos).
    #[cfg_attr(feature = "serde", serde(default))]
    pub model: PinModel,
}

/// The resistance of an output stage or pull that is off (Ω): leakage at
/// the solver's gmin, so it doesn't load an input but keeps each node's
/// DC path.
pub(crate) const R_OFF: f64 = 1e12;

impl DigitalPin {
    /// What the input buffer reads at `volts` (from the ground terminal),
    /// having read `previous`: high at or above `vih`, low at or below
    /// `vil`, and `previous` between. With no previous reading, between
    /// the thresholds reads as the nearer one, high from their midpoint up.
    pub fn reads(&self, volts: f64, previous: Option<LogicLevel>) -> LogicLevel {
        let PinModel { vil, vih, .. } = self.model;
        if volts >= vih {
            LogicLevel::High
        } else if volts <= vil {
            LogicLevel::Low
        } else {
            previous.unwrap_or(if volts >= 0.5 * (vil + vih) {
                LogicLevel::High
            } else {
                LogicLevel::Low
            })
        }
    }

    /// The resistances from the pin to the supply and to ground, as the
    /// mode, level and pull leave them: what's on in parallel, or
    /// [`R_OFF`] when nothing is.
    pub(crate) fn stage(&self) -> (f64, f64) {
        let m = &self.model;
        let (drive_high, drive_low) = match self.mode {
            PinMode::Input => (false, false),
            PinMode::PushPull => (self.level, !self.level),
            PinMode::OpenDrain => (false, !self.level),
        };
        let r = |drive: bool, r_drive: f64, pull: bool| {
            let g =
                if drive { 1.0 / r_drive } else { 0.0 } + if pull { 1.0 / m.r_pull } else { 0.0 };
            if g > 0.0 {
                1.0 / g
            } else {
                R_OFF
            }
        };
        (
            r(drive_high, m.r_high, self.pull == Pull::Up),
            r(drive_low, m.r_low, self.pull == Pull::Down),
        )
    }
}

impl Device for DigitalPin {
    device_identity!();

    /// Nothing: AC analysis stamps its parts.
    fn stamp_ac(&self, _: &mut AcStamp<'_>) {}

    fn needed_ratings(&self) -> &'static [RatingKind] {
        &[RatingKind::Current]
    }

    /// As its parts: drivers, pull, clamps and pin capacitance.
    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        deck.parts(&crate::CircuitElement::DigitalPin(self.clone()), name)
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
        let m = &self.model;
        check.positive("model.r_high", m.r_high, " Ω")?;
        check.positive("model.r_low", m.r_low, " Ω")?;
        check.positive("model.r_pull", m.r_pull, " Ω")?;
        check.non_negative("model.c_comp", m.c_comp, " F")?;
        check.finite("model.leakage", m.leakage, " A")?;
        check.finite("model.vil", m.vil, " V")?;
        check.finite("model.vih", m.vih, " V")?;
        if m.vih <= m.vil {
            return Err(check.fail(format_args!(
                "model.vih {} V must be above model.vil {} V",
                m.vih, m.vil
            )));
        }
        let c = &m.clamp;
        check.positive("model.clamp.is", c.is, " A")?;
        check.positive("model.clamp.n", c.n, "")?;
        check.non_negative("model.clamp.rs", c.rs, " Ω")?;
        check.non_negative("model.clamp.cjo", c.cjo, " F")?;
        check.positive("model.clamp.vj", c.vj, " V")?;
        check.within("model.clamp.m", c.m, 0.9)?;
        check.within("model.clamp.fc", c.fc, 0.95)?;
        check.non_negative("model.clamp.tt", c.tt, " s")?;
        Ok(())
    }
}
