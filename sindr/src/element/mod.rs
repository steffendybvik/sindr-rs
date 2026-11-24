//! The circuit elements: one struct per [`CircuitElement`] variant.
//!
//! [`CircuitElement`] wraps each of them, so a resistor is
//! `CircuitElement::Resistor(Resistor { .. })`. The structs carry the
//! element's fields and serialise as the enum's variant did, inside the
//! enum's `type`-tagged object.
//!
//! [`CircuitElement`]: crate::CircuitElement

use std::fmt::Display;

use crate::circuit::CircuitElement;
use crate::error::SimError;
use crate::ratings::{RatingKind, RatingProbe};

#[cfg(feature = "spice")]
use crate::spice::export::{Deck, ExportError};

use crate::noise::NoiseSources;

pub(crate) use ac_stamp::{AcBias, AcStamp, Conductance};
use dc_stamp::stamped_as_parts;
pub(crate) use dc_stamp::DcStamp;
pub(crate) use tran_stamp::{parasitic_history, TranStamp};

/// What sindr knows about a circuit element: its identity and terminals,
/// the extra unknowns and history it takes, the values its models refuse,
/// and its DC and transient stamps.
///
/// Every element struct implements it, and [`CircuitElement::device`]
/// reaches the one inside an element. The provided methods describe a
/// plain linear, memoryless element; a device overrides only where it
/// differs.
pub(crate) trait Device {
    /// The component's id, unique within its circuit.
    fn id(&self) -> &str;

    /// The component's id, for renaming.
    fn id_mut(&mut self) -> &mut String;

    /// The terminals, in the order the element's docs give them.
    fn nodes(&self) -> &[String];

    /// The terminals, for renaming nets.
    fn nodes_mut(&mut self) -> &mut [String];

    /// Every net the element touches: its terminals, then any it senses
    /// without connecting to (a controlled source's control pair, the
    /// nodes a behavioural expression reads).
    fn all_nodes(&self) -> Vec<&String> {
        self.nodes().iter().collect()
    }

    /// The pairs of nets its DC stamp joins with a conductance or a
    /// voltage constraint, so that one net's voltage fixes the other's.
    /// `ground` is the circuit's ground node, for an element whose output
    /// is referred to it.
    ///
    /// The provided method joins every terminal to the first, which is
    /// right for anything that conducts between all its terminals. An
    /// element that is open at DC between some of them (a capacitor, a
    /// current source, a MOSFET's gate) overrides it. A tiny shunt the
    /// stamp adds, such as a junction's gmin or a varactor's 1 TΩ, still
    /// counts: the matrix isn't singular with it.
    fn dc_paths<'a>(&'a self, ground: &'a String) -> Vec<[&'a String; 2]> {
        let _ = ground;
        let nodes = self.nodes();
        nodes.iter().skip(1).map(|n| [&nodes[0], n]).collect()
    }

    /// MNA unknowns the element adds after the node voltages, its branch
    /// rows.
    fn branches(&self) -> usize {
        0
    }

    /// Whether its first branch row is a source current that other
    /// elements read by the element's id: a CCVS's or CCCS's control, or
    /// `i(id)` in a behavioural expression.
    fn exposes_branch_current(&self) -> bool {
        self.branches() > 0
    }

    /// The id of the element whose branch current it reads, for a CCVS or
    /// CCCS; `None` for the others.
    fn control_source(&self) -> Option<&str> {
        None
    }

    /// Capacitor histories the transient keeps for it, step to step.
    fn capacitor_histories(&self) -> usize {
        0
    }

    /// Inductor histories the transient keeps for it, step to step.
    fn inductor_histories(&self) -> usize {
        0
    }

    /// Whether its stamp depends on the solution, so the DC operating
    /// point needs Newton iterations.
    fn is_nonlinear(&self) -> bool {
        false
    }

    /// Whether it stores energy, so its circuit's behaviour over time
    /// needs a transient.
    fn is_reactive(&self) -> bool {
        false
    }

    /// Writes the element's DC model into `s.system`, linearised at
    /// `s.v_prev` when it depends on the solution.
    ///
    /// # Errors
    ///
    /// What the device's model rejects at stamp time: a resistor's
    /// non-positive resistance, a behavioural expression that isn't finite,
    /// an element that only stamps as its parts.
    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError>;

    /// Writes the element's model for one transient step into `s.system`:
    /// its companion over the step, from its histories at the step's
    /// start, with sources at `s.t`, linearised at `s.v_prev`.
    ///
    /// The provided method is the DC stamp at `s.v_prev`, which is only
    /// right for an element that stores no energy and doesn't change with
    /// time. Anything with a history, a waveform or a time-dependent
    /// expression overrides it.
    ///
    /// # Errors
    ///
    /// As [`stamp_dc`](Self::stamp_dc).
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        debug_assert!(
            !self.is_reactive()
                && self.capacitor_histories() == 0
                && self.inductor_histories() == 0,
            "`{}` stores energy, so its transient stamp isn't its DC one",
            self.id()
        );
        self.stamp_dc(&mut s.dc())
    }

    /// The element's conductances, linearised at the DC bias point: what a
    /// nonlinear device adds to the small-signal system at every frequency,
    /// after the [`stamp_ac`](Self::stamp_ac) of every element. Taken once
    /// per bias point; empty for a linear device.
    fn linearise(&self, _: &AcBias<'_>) -> Vec<Conductance> {
        Vec::new()
    }

    /// Writes the element's small-signal model at `s.s` into `s.a`, past
    /// its [`linearise`](Self::linearise) conductances: admittances,
    /// branch rows, the reactive part a nonlinear device's bias leaves it.
    /// Independent sources stamp their branch and no excitation; a current
    /// source stamps nothing.
    fn stamp_ac(&self, s: &mut AcStamp<'_>);

    /// Extra unknowns the element takes in the small-signal pencil form,
    /// where a stamp that isn't affine in `s` (an inductor's `1/(sL)`)
    /// becomes a branch.
    fn pencil_rows(&self) -> usize {
        0
    }

    /// The element's noise generators at the DC operating point, pushed
    /// onto `n`. None for a noiseless element (sources, reactances,
    /// controlled sources).
    fn noise(&self, _: &mut NoiseSources<'_>) {}

    /// The ratings [`check_ratings`](crate::check_ratings) checks on it
    /// even when its part sets none, so a missing one shows as unverified.
    fn needed_ratings(&self) -> &'static [RatingKind] {
        &[]
    }

    /// The value of a rating of `kind` on it at one instant, from `m`:
    /// `None` when it can't be measured on this element. Junction
    /// temperature and power are measured the same way on every element,
    /// before this; the provided method rates the port's voltage and
    /// current.
    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        m.port_rating(kind)
    }

    /// The SPICE letter it's written under. The provided `R` is for an
    /// element written as resistors, or not at all.
    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'R'
    }

    /// Writes its line, under SPICE name `name`, and any model card and
    /// extra lines it needs, into `deck`.
    ///
    /// # Errors
    ///
    /// [`ExportError::Unsupported`] for an element SPICE3 has no primitive
    /// for, and what the deck refuses: a value that isn't finite, a name
    /// SPICE can't hold.
    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError>;

    /// Rejects a value the models can't use, naming the component, the
    /// field and the value. Left alone, a 0 K junction divides by zero and
    /// fails much later as a bare [`SimError::InvalidSolution`], and a BJT's
    /// β ≤ 0 as that or a convergence failure.
    ///
    /// # Errors
    ///
    /// [`SimError::InvalidResistance`] for a resistor's resistance,
    /// otherwise [`SimError::InvalidComponent`].
    fn validate(&self) -> Result<(), SimError> {
        Ok(())
    }
}

/// The four [`Device`] methods every element implements the same way,
/// from its `id` and `nodes` fields.
macro_rules! device_identity {
    () => {
        fn id(&self) -> &str {
            &self.id
        }

        fn id_mut(&mut self) -> &mut String {
            &mut self.id
        }

        fn nodes(&self) -> &[String] {
            &self.nodes
        }

        fn nodes_mut(&mut self) -> &mut [String] {
            &mut self.nodes
        }
    };
}

mod ac_stamp;
mod battery;
pub(crate) mod behavioural;
pub(crate) mod bjt;
mod capacitor;
pub(crate) mod cccs;
pub(crate) mod ccvs;
mod comparator;
mod crystal;
mod current_source;
mod dc_stamp;
pub(crate) mod diode;
mod ferrite_bead;
mod fuse;
pub(crate) mod igbt;
mod inductor;
pub(crate) mod jfet;
pub(crate) mod layout;
mod led;
pub(crate) mod mosfet;
mod op_amp;
mod photodiode;
pub(crate) mod photoresistor;
mod potentiometer;
mod pushbutton;
pub(crate) mod relay;
mod resistor;
mod schottky_diode;
mod switch;
mod switch_spdt;
mod thermistor;
mod tran_stamp;
pub(crate) mod transformer;
mod varactor;
pub(crate) mod vccs;
mod vcvs;
pub(crate) mod voltage_regulator;
mod voltage_source;
mod zener_diode;

pub use battery::Battery;
pub use behavioural::Behavioural;
pub use bjt::Bjt;
pub use capacitor::Capacitor;
pub use cccs::Cccs;
pub use ccvs::Ccvs;
pub use comparator::Comparator;
pub use crystal::Crystal;
pub use current_source::CurrentSource;
pub use diode::Diode;
pub use ferrite_bead::FerriteBead;
pub use fuse::Fuse;
pub use igbt::Igbt;
pub use inductor::Inductor;
pub use jfet::Jfet;
pub use led::Led;
pub use mosfet::Mosfet;
pub use op_amp::OpAmp;
pub use photodiode::Photodiode;
pub use photoresistor::Photoresistor;
pub use potentiometer::Potentiometer;
pub use pushbutton::Pushbutton;
pub use relay::Relay;
pub use resistor::Resistor;
pub use schottky_diode::SchottkyDiode;
pub use switch::Switch;
pub use switch_spdt::SwitchSpdt;
pub use thermistor::Thermistor;
pub use transformer::Transformer;
pub use varactor::Varactor;
pub use vccs::Vccs;
pub use vcvs::Vcvs;
pub use voltage_regulator::VoltageRegulator;
pub use voltage_source::VoltageSource;
pub use zener_diode::ZenerDiode;

impl CircuitElement {
    /// The element's [`Device`].
    pub(crate) fn device(&self) -> &dyn Device {
        match self {
            CircuitElement::Resistor(d) => d,
            CircuitElement::VoltageSource(d) => d,
            CircuitElement::CurrentSource(d) => d,
            CircuitElement::Switch(d) => d,
            CircuitElement::Capacitor(d) => d,
            CircuitElement::Battery(d) => d,
            CircuitElement::Inductor(d) => d,
            CircuitElement::Diode(d) => d,
            CircuitElement::Led(d) => d,
            CircuitElement::Bjt(d) => d,
            CircuitElement::Mosfet(d) => d,
            CircuitElement::Jfet(d) => d,
            CircuitElement::Vcvs(d) => d,
            CircuitElement::Vccs(d) => d,
            CircuitElement::Ccvs(d) => d,
            CircuitElement::Cccs(d) => d,
            CircuitElement::Pushbutton(d) => d,
            CircuitElement::Relay(d) => d,
            CircuitElement::Photoresistor(d) => d,
            CircuitElement::Potentiometer(d) => d,
            CircuitElement::ZenerDiode(d) => d,
            CircuitElement::OpAmp(d) => d,
            CircuitElement::Comparator(d) => d,
            CircuitElement::SchottkyDiode(d) => d,
            CircuitElement::Thermistor(d) => d,
            CircuitElement::Photodiode(d) => d,
            CircuitElement::Varactor(d) => d,
            CircuitElement::Igbt(d) => d,
            CircuitElement::Transformer(d) => d,
            CircuitElement::Fuse(d) => d,
            CircuitElement::VoltageRegulator(d) => d,
            CircuitElement::SwitchSpdt(d) => d,
            CircuitElement::Crystal(d) => d,
            CircuitElement::FerriteBead(d) => d,
            CircuitElement::Behavioural(d) => d,
        }
    }

    /// The element's [`Device`], for renaming its id and nets.
    pub(crate) fn device_mut(&mut self) -> &mut dyn Device {
        match self {
            CircuitElement::Resistor(d) => d,
            CircuitElement::VoltageSource(d) => d,
            CircuitElement::CurrentSource(d) => d,
            CircuitElement::Switch(d) => d,
            CircuitElement::Capacitor(d) => d,
            CircuitElement::Battery(d) => d,
            CircuitElement::Inductor(d) => d,
            CircuitElement::Diode(d) => d,
            CircuitElement::Led(d) => d,
            CircuitElement::Bjt(d) => d,
            CircuitElement::Mosfet(d) => d,
            CircuitElement::Jfet(d) => d,
            CircuitElement::Vcvs(d) => d,
            CircuitElement::Vccs(d) => d,
            CircuitElement::Ccvs(d) => d,
            CircuitElement::Cccs(d) => d,
            CircuitElement::Pushbutton(d) => d,
            CircuitElement::Relay(d) => d,
            CircuitElement::Photoresistor(d) => d,
            CircuitElement::Potentiometer(d) => d,
            CircuitElement::ZenerDiode(d) => d,
            CircuitElement::OpAmp(d) => d,
            CircuitElement::Comparator(d) => d,
            CircuitElement::SchottkyDiode(d) => d,
            CircuitElement::Thermistor(d) => d,
            CircuitElement::Photodiode(d) => d,
            CircuitElement::Varactor(d) => d,
            CircuitElement::Igbt(d) => d,
            CircuitElement::Transformer(d) => d,
            CircuitElement::Fuse(d) => d,
            CircuitElement::VoltageRegulator(d) => d,
            CircuitElement::SwitchSpdt(d) => d,
            CircuitElement::Crystal(d) => d,
            CircuitElement::FerriteBead(d) => d,
            CircuitElement::Behavioural(d) => d,
        }
    }
}

/// Value checks for [`Device::validate`], each failing with a
/// [`SimError::InvalidComponent`] that names the component, the field and
/// its value.
struct Check<'a> {
    id: &'a str,
}

impl<'a> Check<'a> {
    fn new(id: &'a str) -> Self {
        Self { id }
    }

    /// An error naming the component, then `message`.
    fn fail(&self, message: impl Display) -> SimError {
        SimError::InvalidComponent(format!("`{}`: {message}", self.id))
    }

    /// `value` (in `unit`) is positive and finite.
    fn positive(&self, field: &str, value: f64, unit: &str) -> Result<(), SimError> {
        if value.is_finite() && value > 0.0 {
            Ok(())
        } else {
            Err(self.fail(format_args!(
                "{field} {value}{unit} must be a positive, finite number"
            )))
        }
    }

    /// `value` (in `unit`) is zero or positive, and finite.
    fn non_negative(&self, field: &str, value: f64, unit: &str) -> Result<(), SimError> {
        if value.is_finite() && value >= 0.0 {
            Ok(())
        } else {
            Err(self.fail(format_args!(
                "{field} {value}{unit} must be zero or a positive, finite number"
            )))
        }
    }

    /// `value` (in `unit`) is finite.
    fn finite(&self, field: &str, value: f64, unit: &str) -> Result<(), SimError> {
        if value.is_finite() {
            Ok(())
        } else {
            Err(self.fail(format_args!(
                "{field} {value}{unit} must be a finite number"
            )))
        }
    }

    /// A dimensionless coefficient in `[0, max]`, as SPICE bounds M and FC:
    /// at 1 the junction charge divides by zero.
    fn within(&self, field: &str, value: f64, max: f64) -> Result<(), SimError> {
        if (0.0..=max).contains(&value) {
            Ok(())
        } else {
            Err(self.fail(format_args!("{field} {value} must be between 0 and {max}")))
        }
    }

    /// An op-amp's or comparator's output limits leave it room to swing.
    fn rails_ordered(&self, v_pos: f64, v_neg: f64) -> Result<(), SimError> {
        if v_pos <= v_neg {
            return Err(self.fail(format_args!(
                "v_pos {v_pos} V must be above v_neg {v_neg} V"
            )));
        }
        Ok(())
    }
}

/// Default junction temperature for IS-bearing devices (K).
/// 300.15 K = 27°C, the SPICE reference temperature for IS.
#[cfg(feature = "serde")]
fn default_junction_temperature() -> f64 {
    300.15
}

#[cfg(feature = "serde")]
fn default_v_pos() -> f64 {
    15.0
}

#[cfg(feature = "serde")]
fn default_v_neg() -> f64 {
    -15.0
}
