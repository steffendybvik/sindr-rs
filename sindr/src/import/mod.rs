//! Import a schematic tool's netlist, plus a test bench, as a [`Circuit`].
//!
//! A schematic netlist names parts and wires, not physics: there's a
//! resistor `R3` valued `"4k7"`, and its terminal `1` sits on net `VOUT`.
//! It has no stimulus either, since a power port names a net (`3V3`)
//! without driving it. This module takes that shape in, in the caller's
//! own ids, and gives back a [`Circuit`] sindr can solve:
//!
//! - A [`Netlist`]: [`Component`]s, each with an id, a [`ComponentKind`], a
//!   value (a number or text such as `"4k7"`), named parameters and
//!   optional model and part data; and [`Net`]s, each an id and the
//!   [`Pin`]s (component and terminal name) on it.
//! - A [`Bench`]: what the schematic leaves out. The ground net, sources
//!   and loads attached to nets or terminals, nets tied together, and the
//!   [`Analysis`] to run.
//!
//! [`Netlist::to_circuit`] builds the circuit; [`simulate`] builds it and
//! runs the bench's analysis. Ids carry through: each element's id is the
//! component's id, and each node's name is a net id (see
//! [`ImportedCircuit::node`]), so results and errors point back at the
//! caller's own parts and nets. Every [`ImportError`] names the component,
//! net or bench item it is about.
//!
//! Terminals are matched by name, never by pin number, since pin numbers
//! depend on the package. Names are compared without case; each kind lists
//! the names it takes in [`ComponentKind::terminals`]. Unpolarised
//! two-terminal parts also take `1` and `2`.
//!
//! # Example
//!
//! ```
//! use sindr::import::{simulate, Analysis, Bench, BenchSource, Component,
//!                     ComponentKind, Net, Netlist, Simulation};
//!
//! let netlist = Netlist {
//!     components: vec![
//!         Component::new("c1", ComponentKind::Resistor).with_value("10k"),
//!         Component::new("c2", ComponentKind::Resistor).with_value("4k7"),
//!     ],
//!     nets: vec![
//!         Net::new("vin", [("c1", "1")]),
//!         Net::new("vout", [("c1", "2"), ("c2", "1")]),
//!         Net::new("gnd", [("c2", "2")]),
//!     ],
//! };
//! let bench = Bench::new("gnd")
//!     .with_source(BenchSource::voltage("supply", "vin", 5.0))
//!     .with_analysis(Analysis::OperatingPoint);
//!
//! let Simulation::OperatingPoint(op) = simulate(&netlist, &bench).unwrap().result else {
//!     unreachable!()
//! };
//! let vout = op.node_voltages["vout"];
//! assert!((vout - 5.0 * 4.7 / 14.7).abs() < 1e-9);
//! ```

mod bench;
mod value;

pub use bench::{
    simulate, Analysis, Attach, Bench, BenchError, BenchLoad, BenchSource, Imported, LoadKind,
    Simulation, SourceKind,
};
pub use value::parse_value;

use std::collections::{BTreeMap, BTreeSet};

use crate::circuit::{BjtModel, Circuit, CircuitElement, DiodeModel, OpAmpModel};
use crate::design::Part;
use sindr_devices::battery::BatteryParams;
use sindr_devices::bjt::BjtKind;
use sindr_devices::igbt::IgbtParams;
use sindr_devices::jfet::JfetKind;
use sindr_devices::mosfet::{MosfetKind, MosfetParams};
use sindr_devices::regulator::RegulatorParams;
use sindr_devices::varactor::VaractorParams;

/// A schematic's components and the nets joining them.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Netlist {
    /// The components, in any order.
    pub components: Vec<Component>,
    /// The nets, in any order. A terminal on no net is unconnected.
    pub nets: Vec<Net>,
}

/// One component of a [`Netlist`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Component {
    /// The caller's id, unique in the netlist. It becomes the element's id,
    /// so results are keyed by it.
    pub id: String,
    /// What the component is.
    pub kind: ComponentKind,
    /// The main value: resistance (Ω), capacitance (F), inductance (H), a
    /// fuse's rating (A), a zener's or regulator's voltage (V), a
    /// potentiometer's end-to-end resistance (Ω), a crystal's frequency
    /// (Hz). Text such as `"4k7"` goes through [`parse_value`]. Text that
    /// isn't a value (a part number) is ignored where a parameter or model
    /// can stand in, and an error where the kind needs a value.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub value: Option<Value>,
    /// Further parameters by name, such as `bf` for a transistor's gain or
    /// `color` for an LED. Each kind's are listed on [`ComponentKind`];
    /// any other name is an [`ImportError::UnknownParameter`].
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "BTreeMap::is_empty")
    )]
    pub params: BTreeMap<String, Value>,
    /// Device model, for the kinds that take one.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<Model>,
    /// Part data (ratings, thermal path, reference, MPN …), copied into
    /// [`Circuit::parts`] under the component's id.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub part: Option<Part>,
}

impl Component {
    /// A component with no value, parameters, model or part data.
    pub fn new(id: impl Into<String>, kind: ComponentKind) -> Self {
        Self {
            id: id.into(),
            kind,
            value: None,
            params: BTreeMap::new(),
            model: None,
            part: None,
        }
    }

    /// Sets [`value`](Self::value).
    pub fn with_value(mut self, value: impl Into<Value>) -> Self {
        self.value = Some(value.into());
        self
    }

    /// Adds a parameter.
    pub fn with_param(mut self, name: impl Into<String>, value: impl Into<Value>) -> Self {
        self.params.insert(name.into(), value.into());
        self
    }

    /// Sets [`model`](Self::model).
    pub fn with_model(mut self, model: Model) -> Self {
        self.model = Some(model);
        self
    }

    /// Sets [`part`](Self::part).
    pub fn with_part(mut self, part: Part) -> Self {
        self.part = Some(part);
        self
    }
}

/// A value or parameter: a number in SI base units, text, or a flag.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(untagged))]
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// A number in SI base units (Ω, F, H, V, A, Hz, K).
    Number(f64),
    /// A flag, such as a switch's `closed`.
    Bool(bool),
    /// Text: a value as written (`"4k7"`, read by [`parse_value`]) or a
    /// name (an LED's `"red"`).
    Text(String),
}

impl From<f64> for Value {
    fn from(v: f64) -> Self {
        Value::Number(v)
    }
}

impl From<bool> for Value {
    fn from(v: bool) -> Self {
        Value::Bool(v)
    }
}

impl From<&str> for Value {
    fn from(v: &str) -> Self {
        Value::Text(v.to_string())
    }
}

impl From<String> for Value {
    fn from(v: String) -> Self {
        Value::Text(v)
    }
}

/// A device model for a [`Component`] whose kind takes one.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "type", rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
pub enum Model {
    /// For a diode (`.model D`).
    Diode(DiodeModel),
    /// For an NPN or PNP transistor (`.model NPN`/`PNP`), besides `bf`.
    Bjt(BjtModel),
    /// For an NMOS or PMOS transistor.
    Mosfet(MosfetParams),
    /// For an op-amp: its macromodel.
    OpAmp(OpAmpModel),
    /// For a linear regulator: dropout, quiescent current and limits.
    Regulator(RegulatorParams),
    /// For a battery: its equivalent circuit.
    Battery(BatteryParams),
    /// For a varactor.
    Varactor(VaractorParams),
    /// For an IGBT.
    Igbt(IgbtParams),
}

impl Model {
    fn name(&self) -> &'static str {
        match self {
            Model::Diode(_) => "diode",
            Model::Bjt(_) => "bjt",
            Model::Mosfet(_) => "mosfet",
            Model::OpAmp(_) => "op_amp",
            Model::Regulator(_) => "regulator",
            Model::Battery(_) => "battery",
            Model::Varactor(_) => "varactor",
            Model::Igbt(_) => "igbt",
        }
    }
}

/// What a [`Component`] is. Serialised as its snake_case name
/// (`"resistor"`, `"npn"`); a name sindr doesn't know reads as
/// [`Other`](Self::Other), which [`Netlist::to_circuit`] refuses with the
/// component's id.
///
/// Parameters each kind takes, besides [`Component::value`] (numbers in SI
/// units, or text for [`parse_value`]):
///
/// | Kind | Value | Parameters | Model |
/// |---|---|---|---|
/// | `resistor` | Ω, required | | |
/// | `capacitor`, `capacitor_polarized` | F, required | | |
/// | `inductor` | H, required | | |
/// | `diode`, `schottky_diode`, `photodiode` | | `temperature` (K); photodiode `irradiance` (W) | diode only: `Diode` |
/// | `zener_diode` | V (breakdown) or `vz` | `vz`, `temperature` | |
/// | `led` | | `color` (`"red"`, `"green"`, `"blue"`, …; default red), `temperature` | |
/// | `varactor` | | | `Varactor` |
/// | `npn`, `pnp` | | `bf` (or `beta`, `hfe`), `temperature` | `Bjt` |
/// | `nmos`, `pmos` | | | `Mosfet` |
/// | `njfet`, `pjfet` | | `idss` (A), `vp` (V) | |
/// | `igbt` | | | `Igbt` |
/// | `op_amp`, `comparator` | | `v_pos`, `v_neg` (V; default from bench supplies on its supply pins, else ±15) | op-amp: `OpAmp` |
/// | `potentiometer` | Ω, required | `position` (0 top to 1 bottom, default 0.5) | |
/// | `switch`, `push_button` | | `closed` (default false) | |
/// | `fuse` | A (rating) | `blown` (default false) | |
/// | `relay` | | `pickup_voltage` (V, required), `coil_resistance` (Ω), `inductance` (H) | |
/// | `transformer` | | `l1`, `l2` (H, required), `k` | |
/// | `regulator` | V (output) or `voltage` | `voltage` | `Regulator` |
/// | `battery` | V, unless a model | `state_of_charge` (default 1), `r_series` (Ω, default 0.1), `capacity` (Ah, default 1) | `Battery` |
/// | `thermistor` | | `temperature` (K) | |
/// | `photoresistor` | | `light_level` (0 dark to 1 bright) | |
/// | `voltage_source`, `current_source` | V or A, required | | |
/// | `connector`, `test_point`, `net_tie`, `mounting_hole` | | | |
///
/// The last row are [passive](Self::is_passive): they add no element, and
/// any terminal name is fine. A bench attaches sources and loads to their
/// terminals.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(from = "String", into = "String"))]
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[allow(missing_docs)]
pub enum ComponentKind {
    Resistor,
    Capacitor,
    CapacitorPolarized,
    Inductor,
    Diode,
    SchottkyDiode,
    ZenerDiode,
    Led,
    Photodiode,
    Varactor,
    Npn,
    Pnp,
    Nmos,
    Pmos,
    Njfet,
    Pjfet,
    Igbt,
    OpAmp,
    Comparator,
    Potentiometer,
    Switch,
    PushButton,
    Fuse,
    Relay,
    Transformer,
    Regulator,
    Battery,
    Thermistor,
    Photoresistor,
    VoltageSource,
    CurrentSource,
    Connector,
    TestPoint,
    NetTie,
    MountingHole,
    /// A kind sindr doesn't simulate, by the caller's name for it.
    Other(String),
}

const KINDS: &[(&str, ComponentKind)] = &[
    ("resistor", ComponentKind::Resistor),
    ("capacitor", ComponentKind::Capacitor),
    ("capacitor_polarized", ComponentKind::CapacitorPolarized),
    ("inductor", ComponentKind::Inductor),
    ("diode", ComponentKind::Diode),
    ("schottky_diode", ComponentKind::SchottkyDiode),
    ("zener_diode", ComponentKind::ZenerDiode),
    ("led", ComponentKind::Led),
    ("photodiode", ComponentKind::Photodiode),
    ("varactor", ComponentKind::Varactor),
    ("npn", ComponentKind::Npn),
    ("pnp", ComponentKind::Pnp),
    ("nmos", ComponentKind::Nmos),
    ("pmos", ComponentKind::Pmos),
    ("njfet", ComponentKind::Njfet),
    ("pjfet", ComponentKind::Pjfet),
    ("igbt", ComponentKind::Igbt),
    ("op_amp", ComponentKind::OpAmp),
    ("comparator", ComponentKind::Comparator),
    ("potentiometer", ComponentKind::Potentiometer),
    ("switch", ComponentKind::Switch),
    ("push_button", ComponentKind::PushButton),
    ("fuse", ComponentKind::Fuse),
    ("relay", ComponentKind::Relay),
    ("transformer", ComponentKind::Transformer),
    ("regulator", ComponentKind::Regulator),
    ("battery", ComponentKind::Battery),
    ("thermistor", ComponentKind::Thermistor),
    ("photoresistor", ComponentKind::Photoresistor),
    ("voltage_source", ComponentKind::VoltageSource),
    ("current_source", ComponentKind::CurrentSource),
    ("connector", ComponentKind::Connector),
    ("test_point", ComponentKind::TestPoint),
    ("net_tie", ComponentKind::NetTie),
    ("mounting_hole", ComponentKind::MountingHole),
];

impl ComponentKind {
    /// The kind with this snake_case name, or [`Other`](Self::Other).
    pub fn from_name(name: &str) -> Self {
        KINDS
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, k)| k.clone())
            .unwrap_or_else(|| ComponentKind::Other(name.to_string()))
    }

    /// The kind's snake_case name; an [`Other`](Self::Other) kind's own.
    pub fn name(&self) -> &str {
        match self {
            ComponentKind::Other(name) => name,
            kind => KINDS.iter().find(|(_, k)| k == kind).map_or("", |(n, _)| n),
        }
    }

    /// Whether the kind adds no element: connectors, test points, net ties
    /// and mounting holes.
    pub fn is_passive(&self) -> bool {
        matches!(
            self,
            ComponentKind::Connector
                | ComponentKind::TestPoint
                | ComponentKind::NetTie
                | ComponentKind::MountingHole
        )
    }

    /// The kind's terminals in element order: each a list of the names it
    /// answers to, lowercase, canonical first. Required terminals come
    /// first; [`optional_terminals`](Self::optional_terminals) says how
    /// many at the end may be left off. Empty for passive and unknown
    /// kinds.
    pub fn terminals(&self) -> &'static [&'static [&'static str]] {
        use ComponentKind as K;
        match self {
            K::Resistor
            | K::Capacitor
            | K::Inductor
            | K::Switch
            | K::PushButton
            | K::Fuse
            | K::Thermistor
            | K::Photoresistor => &[&["1", "a"], &["2", "b"]],
            K::CapacitorPolarized | K::Battery | K::VoltageSource => &[
                &["+", "positive", "pos", "p"],
                &["-", "negative", "neg", "n"],
            ],
            K::CurrentSource => &[&["from", "+", "in"], &["to", "-", "out"]],
            K::Diode | K::SchottkyDiode | K::ZenerDiode | K::Led | K::Photodiode | K::Varactor => {
                &[&["anode", "a"], &["cathode", "k", "c"]]
            }
            K::Npn | K::Pnp => &[&["base", "b"], &["collector", "c"], &["emitter", "e"]],
            K::Nmos | K::Pmos => &[
                &["gate", "g"],
                &["drain", "d"],
                &["source", "s"],
                &["body", "bulk", "b"],
            ],
            K::Njfet | K::Pjfet => &[&["gate", "g"], &["drain", "d"], &["source", "s"]],
            K::Igbt => &[&["gate", "g"], &["collector", "c"], &["emitter", "e"]],
            K::OpAmp | K::Comparator => &[
                &["in+", "+", "inp", "non_inverting", "in_plus"],
                &["in-", "-", "inn", "inverting", "in_minus"],
                &["out", "output", "vout"],
                &["v+", "vcc", "vdd", "vs+", "v_pos"],
                &["v-", "vee", "vss", "vs-", "v_neg"],
            ],
            K::Potentiometer => &[
                &["1", "top", "cw"],
                &["2", "wiper", "w"],
                &["3", "bottom", "ccw"],
            ],
            K::Relay => &[
                &["coil+", "a1", "coil1"],
                &["coil-", "a2", "coil2"],
                &["com", "common"],
                &["no", "normally_open"],
            ],
            K::Transformer => &[
                &["p1", "pri+", "primary+"],
                &["q1", "pri-", "primary-"],
                &["p2", "sec+", "secondary+"],
                &["q2", "sec-", "secondary-"],
            ],
            K::Regulator => &[
                &["in", "vin", "input"],
                &["out", "vout", "output"],
                &["gnd", "ground", "com"],
            ],
            K::Connector | K::TestPoint | K::NetTie | K::MountingHole | K::Other(_) => &[],
        }
    }

    /// How many of [`terminals`](Self::terminals), at the end, may be left
    /// unconnected: a MOSFET's body (tied to its source), an op-amp's two
    /// supply pins.
    pub fn optional_terminals(&self) -> usize {
        match self {
            ComponentKind::Nmos | ComponentKind::Pmos => 1,
            ComponentKind::OpAmp | ComponentKind::Comparator => 2,
            _ => 0,
        }
    }

    fn params(&self) -> &'static [&'static str] {
        use ComponentKind as K;
        match self {
            K::Diode | K::SchottkyDiode => &["temperature"],
            K::Photodiode => &["temperature", "irradiance"],
            K::ZenerDiode => &["vz", "temperature"],
            K::Led => &["color", "temperature"],
            K::Npn | K::Pnp => &["bf", "beta", "hfe", "temperature"],
            K::Njfet | K::Pjfet => &["idss", "vp"],
            K::OpAmp | K::Comparator => &["v_pos", "v_neg"],
            K::Potentiometer => &["position"],
            K::Switch | K::PushButton => &["closed"],
            K::Fuse => &["blown"],
            K::Relay => &["pickup_voltage", "coil_resistance", "inductance"],
            K::Transformer => &["l1", "l2", "k"],
            K::Regulator => &["voltage"],
            K::Battery => &["state_of_charge", "r_series", "capacity"],
            K::Thermistor => &["temperature"],
            K::Photoresistor => &["light_level"],
            _ => &[],
        }
    }

    fn takes_model(&self, model: &Model) -> bool {
        use ComponentKind as K;
        matches!(
            (self, model),
            (K::Diode, Model::Diode(_))
                | (K::Npn | K::Pnp, Model::Bjt(_))
                | (K::Nmos | K::Pmos, Model::Mosfet(_))
                | (K::OpAmp, Model::OpAmp(_))
                | (K::Regulator, Model::Regulator(_))
                | (K::Battery, Model::Battery(_))
                | (K::Varactor, Model::Varactor(_))
                | (K::Igbt, Model::Igbt(_))
        )
    }
}

impl From<String> for ComponentKind {
    fn from(name: String) -> Self {
        ComponentKind::from_name(&name)
    }
}

impl From<ComponentKind> for String {
    fn from(kind: ComponentKind) -> Self {
        kind.name().to_string()
    }
}

/// A net: the terminals wired together.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Net {
    /// The caller's id, unique in the netlist. It becomes the node's name.
    pub id: String,
    /// Display name (`"VOUT"`, `"3V3"`). A bench may refer to the net by
    /// it when it is unique.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub name: Option<String>,
    /// The terminals on the net.
    pub pins: Vec<Pin>,
}

impl Net {
    /// A net with no display name.
    pub fn new<C, T>(id: impl Into<String>, pins: impl IntoIterator<Item = (C, T)>) -> Self
    where
        C: Into<String>,
        T: Into<String>,
    {
        Self {
            id: id.into(),
            name: None,
            pins: pins.into_iter().map(|(c, t)| Pin::new(c, t)).collect(),
        }
    }

    /// Sets [`name`](Self::name).
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }
}

/// A component's terminal, by the component's id and the terminal's name.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Pin {
    /// The component's id.
    pub component: String,
    /// The terminal's name (`"anode"`, `"B"`, `"1"`).
    pub terminal: String,
}

impl Pin {
    /// The terminal `terminal` of component `component`.
    pub fn new(component: impl Into<String>, terminal: impl Into<String>) -> Self {
        Self {
            component: component.into(),
            terminal: terminal.into(),
        }
    }
}

/// Why a netlist and bench can't become a circuit. Each variant names the
/// caller's component, net or bench item.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ImportError {
    /// Two components, nets or bench items share an id.
    #[error("id \"{id}\" is used twice")]
    DuplicateId {
        /// The id.
        id: String,
    },
    /// A kind sindr doesn't simulate.
    #[error("component \"{component}\": sindr can't simulate a \"{kind}\"")]
    UnsupportedKind {
        /// The component's id.
        component: String,
        /// Its kind's name.
        kind: String,
    },
    /// A net names a component that isn't in the netlist.
    #[error("net \"{net}\" names component \"{component}\", which isn't in the netlist")]
    UnknownComponent {
        /// The net's id, or the bench item's.
        net: String,
        /// The missing component's id.
        component: String,
    },
    /// A terminal name the component's kind doesn't have.
    #[error(
        "component \"{component}\" ({kind}) has no terminal \"{terminal}\"; it has {expected}"
    )]
    UnknownTerminal {
        /// The component's id.
        component: String,
        /// Its kind's name.
        kind: String,
        /// The terminal as given.
        terminal: String,
        /// The names it takes.
        expected: String,
    },
    /// A terminal sits on two nets.
    #[error("terminal \"{terminal}\" of component \"{component}\" is on two nets, \"{first}\" and \"{second}\"")]
    TerminalOnTwoNets {
        /// The component's id.
        component: String,
        /// The terminal.
        terminal: String,
        /// One net's id.
        first: String,
        /// The other's.
        second: String,
    },
    /// A terminal the element needs is on no net.
    #[error("terminal \"{terminal}\" of component \"{component}\" isn't connected")]
    Unconnected {
        /// The component's id.
        component: String,
        /// The terminal's canonical name.
        terminal: String,
    },
    /// The kind needs a value and the component has none, or one that
    /// isn't a number.
    #[error("component \"{component}\" needs {what}")]
    MissingValue {
        /// The component's id.
        component: String,
        /// What it needs, with its unit.
        what: String,
    },
    /// A value or parameter that isn't a value of the right type.
    #[error("component \"{component}\": {field} {given:?} isn't {expected}")]
    BadValue {
        /// The component's id.
        component: String,
        /// `"value"` or the parameter's name.
        field: String,
        /// What was given.
        given: Value,
        /// What it should be.
        expected: String,
    },
    /// A parameter the kind doesn't take.
    #[error("component \"{component}\" ({kind}) takes no parameter \"{parameter}\"")]
    UnknownParameter {
        /// The component's id.
        component: String,
        /// Its kind's name.
        kind: String,
        /// The parameter's name.
        parameter: String,
    },
    /// A model of the wrong sort for the kind.
    #[error("component \"{component}\" ({kind}) can't take a {model} model")]
    WrongModel {
        /// The component's id.
        component: String,
        /// Its kind's name.
        kind: String,
        /// The model's sort.
        model: String,
    },
    /// A MOSFET's body is on a different net from its source; sindr's
    /// MOSFET ties the two.
    #[error("component \"{component}\": the body must be on the source's net")]
    BodyNotAtSource {
        /// The component's id.
        component: String,
    },
    /// The bench names a net that isn't in the netlist, or a display name
    /// several nets share.
    #[error("{item} names net \"{net}\", which {problem}")]
    UnknownNet {
        /// The bench item: `"ground"`, a source's id …
        item: String,
        /// The net as named.
        net: String,
        /// `"isn't in the netlist"` or `"is the name of several nets"`.
        problem: String,
    },
    /// The bench attaches to a terminal that's on no net.
    #[error("{item} attaches to terminal \"{terminal}\" of component \"{component}\", which is on no net")]
    AttachedToUnconnected {
        /// The bench item.
        item: String,
        /// The component's id.
        component: String,
        /// The terminal.
        terminal: String,
    },
}

/// A [`Circuit`] built from a [`Netlist`], with the map from the caller's
/// net ids to its node names.
#[derive(Debug, Clone, PartialEq)]
pub struct ImportedCircuit {
    /// The circuit. Element ids are component and bench ids; node names
    /// are net ids.
    pub circuit: Circuit,
    /// Node name by net id. Nets the bench ties together share one node,
    /// named after one of them; every other net's node is its own id.
    pub nodes: BTreeMap<String, String>,
}

impl ImportedCircuit {
    /// The node name of net `net_id`, for looking up results.
    pub fn node(&self, net_id: &str) -> Option<&str> {
        self.nodes.get(net_id).map(String::as_str)
    }
}

impl Netlist {
    /// Builds the circuit: the netlist's components, with `bench`'s ground,
    /// ties, sources and loads. The bench's analysis isn't run; see
    /// [`simulate`].
    ///
    /// # Errors
    ///
    /// An [`ImportError`] naming the component, net or bench item that
    /// can't be built. Solver checks (a floating node, no path to ground)
    /// come later, from the analysis.
    pub fn to_circuit(&self, bench: &Bench) -> Result<ImportedCircuit, ImportError> {
        let mut ids = BTreeSet::new();
        for id in self
            .components
            .iter()
            .map(|c| &c.id)
            .chain(bench.sources.iter().map(|s| &s.id))
            .chain(bench.loads.iter().map(|l| &l.id))
        {
            if !ids.insert(id.as_str()) {
                return Err(ImportError::DuplicateId { id: id.clone() });
            }
        }
        let mut net_ids = BTreeSet::new();
        for net in &self.nets {
            if !net_ids.insert(net.id.as_str()) {
                return Err(ImportError::DuplicateId { id: net.id.clone() });
            }
        }

        let components: BTreeMap<&str, &Component> =
            self.components.iter().map(|c| (c.id.as_str(), c)).collect();

        // Net of each (component, terminal slot), checking every pin.
        let mut pin_net: BTreeMap<(&str, Slot), &str> = BTreeMap::new();
        for net in &self.nets {
            for pin in &net.pins {
                let component = components.get(pin.component.as_str()).ok_or_else(|| {
                    ImportError::UnknownComponent {
                        net: net.id.clone(),
                        component: pin.component.clone(),
                    }
                })?;
                let slot = slot_of(component, &pin.terminal)?;
                if let Some(first) = pin_net.insert((component.id.as_str(), slot), &net.id) {
                    return Err(ImportError::TerminalOnTwoNets {
                        component: component.id.clone(),
                        terminal: pin.terminal.clone(),
                        first: first.to_string(),
                        second: net.id.clone(),
                    });
                }
            }
        }

        let resolver = Resolver {
            nets: &self.nets,
            components: &components,
            pin_net: &pin_net,
        };

        // Ties: union-find over net ids, each set named after its least id
        // so the result doesn't depend on order.
        let mut parent: BTreeMap<&str, &str> = self
            .nets
            .iter()
            .map(|n| (n.id.as_str(), n.id.as_str()))
            .collect();
        for (i, [a, b]) in bench.ties.iter().enumerate() {
            let item = format!("tie {}", i + 1);
            let a = resolver.net(a, &item)?;
            let b = resolver.net(b, &item)?;
            let (ra, rb) = (find(&parent, a), find(&parent, b));
            let (root, child) = if ra <= rb { (ra, rb) } else { (rb, ra) };
            parent.insert(child, root);
        }
        let nodes: BTreeMap<String, String> = self
            .nets
            .iter()
            .map(|n| (n.id.clone(), find(&parent, &n.id).to_string()))
            .collect();
        let node = |net: &str| nodes.get(net).cloned().unwrap_or_else(|| net.to_string());

        let ground_net = resolver.net(&bench.ground, "ground")?;
        let mut circuit = Circuit::new(node(ground_net), Vec::new());

        // Supply voltage on each node from the bench's DC voltage sources
        // to ground, for op-amp rails.
        let side = |attach: &Option<Attach>, item: &str| -> Result<String, ImportError> {
            match attach {
                Some(a) => Ok(node(resolver.net(a, item)?)),
                None => Ok(node(ground_net)),
            }
        };
        let mut supply = BTreeMap::from([(circuit.ground_node.clone(), 0.0)]);
        for source in &bench.sources {
            if let SourceKind::Voltage { volts, .. } = source.kind {
                let plus = node(resolver.net(&source.plus, &source.id)?);
                if side(&source.minus, &source.id)? == circuit.ground_node {
                    supply.insert(plus, volts);
                }
            }
        }

        for component in &self.components {
            let nodes_of = |slot: usize| -> Option<String> {
                pin_net
                    .get(&(component.id.as_str(), Slot::Index(slot)))
                    .map(|net| node(net))
            };
            if let Some(element) = build_element(component, nodes_of, &supply)? {
                circuit.components.push(element);
            }
            if let Some(part) = &component.part {
                circuit.parts.insert(component.id.clone(), part.clone());
            }
        }

        for source in &bench.sources {
            let nodes = [
                node(resolver.net(&source.plus, &source.id)?),
                side(&source.minus, &source.id)?,
            ];
            circuit.components.push(source.element(nodes));
        }
        for load in &bench.loads {
            let nodes = [
                node(resolver.net(&load.a, &load.id)?),
                side(&load.b, &load.id)?,
            ];
            circuit.components.push(load.element(nodes));
        }

        Ok(ImportedCircuit { circuit, nodes })
    }
}

fn find<'a>(parent: &BTreeMap<&'a str, &'a str>, mut id: &'a str) -> &'a str {
    while let Some(&p) = parent.get(id) {
        if p == id {
            break;
        }
        id = p;
    }
    id
}

/// A terminal of a component: an element slot, or a passive part's own
/// name for it.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Slot {
    Index(usize),
    Named(String),
}

fn slot_of(component: &Component, terminal: &str) -> Result<Slot, ImportError> {
    let kind = &component.kind;
    if kind.is_passive() {
        return Ok(Slot::Named(terminal.to_string()));
    }
    if let ComponentKind::Other(name) = kind {
        return Err(ImportError::UnsupportedKind {
            component: component.id.clone(),
            kind: name.clone(),
        });
    }
    let wanted = terminal.trim().to_lowercase();
    kind.terminals()
        .iter()
        .position(|names| names.contains(&wanted.as_str()))
        .map(Slot::Index)
        .ok_or_else(|| ImportError::UnknownTerminal {
            component: component.id.clone(),
            kind: kind.name().to_string(),
            terminal: terminal.to_string(),
            expected: kind
                .terminals()
                .iter()
                .map(|names| format!("\"{}\"", names[0]))
                .collect::<Vec<_>>()
                .join(", "),
        })
}

struct Resolver<'a> {
    nets: &'a [Net],
    components: &'a BTreeMap<&'a str, &'a Component>,
    pin_net: &'a BTreeMap<(&'a str, Slot), &'a str>,
}

impl<'a> Resolver<'a> {
    /// The id of the net `attach` names.
    fn net(&self, attach: &Attach, item: &str) -> Result<&'a str, ImportError> {
        match attach {
            Attach::Net(name) => {
                if let Some(net) = self.nets.iter().find(|n| &n.id == name) {
                    return Ok(&net.id);
                }
                let mut named = self.nets.iter().filter(|n| n.name.as_ref() == Some(name));
                match (named.next(), named.next()) {
                    (Some(net), None) => Ok(&net.id),
                    (first, _) => Err(ImportError::UnknownNet {
                        item: item.to_string(),
                        net: name.clone(),
                        problem: if first.is_some() {
                            "is the name of several nets".into()
                        } else {
                            "isn't in the netlist".into()
                        },
                    }),
                }
            }
            Attach::Terminal(pin) => {
                let component = self.components.get(pin.component.as_str()).ok_or_else(|| {
                    ImportError::UnknownComponent {
                        net: item.to_string(),
                        component: pin.component.clone(),
                    }
                })?;
                let slot = slot_of(component, &pin.terminal)?;
                self.pin_net
                    .get(&(component.id.as_str(), slot))
                    .copied()
                    .ok_or_else(|| ImportError::AttachedToUnconnected {
                        item: item.to_string(),
                        component: pin.component.clone(),
                        terminal: pin.terminal.clone(),
                    })
            }
        }
    }
}

/// Reads a component's value and parameters.
struct Reader<'a> {
    component: &'a Component,
}

impl Reader<'_> {
    fn bad(&self, field: &str, given: &Value, expected: &str) -> ImportError {
        ImportError::BadValue {
            component: self.component.id.clone(),
            field: field.to_string(),
            given: given.clone(),
            expected: expected.to_string(),
        }
    }

    fn number(&self, field: &str, value: &Value) -> Result<f64, ImportError> {
        match value {
            Value::Number(n) if n.is_finite() => Ok(*n),
            Value::Text(t) => parse_value(t).ok_or_else(|| self.bad(field, value, "a number")),
            _ => Err(self.bad(field, value, "a number")),
        }
    }

    /// The first of `names` that is set, as a number.
    fn param(&self, names: &[&str]) -> Result<Option<f64>, ImportError> {
        for name in names {
            if let Some(v) = self.component.params.get(*name) {
                return self.number(name, v).map(Some);
            }
        }
        Ok(None)
    }

    fn param_or(&self, name: &str, default: f64) -> Result<f64, ImportError> {
        Ok(self.param(&[name])?.unwrap_or(default))
    }

    fn flag(&self, name: &str) -> Result<bool, ImportError> {
        match self.component.params.get(name) {
            None => Ok(false),
            Some(Value::Bool(b)) => Ok(*b),
            Some(v) => Err(self.bad(name, v, "true or false")),
        }
    }

    fn text(&self, name: &str) -> Result<Option<String>, ImportError> {
        match self.component.params.get(name) {
            None => Ok(None),
            Some(Value::Text(t)) => Ok(Some(t.clone())),
            Some(v) => Err(self.bad(name, v, "text")),
        }
    }

    /// The value as a number; `None` when unset or text that isn't a value
    /// (a part number, say).
    fn value(&self) -> Result<Option<f64>, ImportError> {
        match &self.component.value {
            None => Ok(None),
            Some(Value::Text(t)) => Ok(parse_value(t)),
            Some(v) => self.number("value", v).map(Some),
        }
    }

    /// The value, which the kind needs: `what` says what, with its unit.
    fn required_value(&self, what: &str) -> Result<f64, ImportError> {
        match &self.component.value {
            Some(v) => self.number("value", v),
            None => Err(ImportError::MissingValue {
                component: self.component.id.clone(),
                what: what.to_string(),
            }),
        }
    }

    /// A parameter the kind needs.
    fn required_param(&self, name: &str, what: &str) -> Result<f64, ImportError> {
        self.param(&[name])?
            .ok_or_else(|| ImportError::MissingValue {
                component: self.component.id.clone(),
                what: format!("{what} (parameter \"{name}\")"),
            })
    }
}

const NOMINAL_K: f64 = 300.15;

/// The element for `component`, or `None` for a passive one. `node(i)` is
/// the node of terminal slot `i`, if connected.
fn build_element(
    component: &Component,
    node: impl Fn(usize) -> Option<String>,
    supply: &BTreeMap<String, f64>,
) -> Result<Option<CircuitElement>, ImportError> {
    use ComponentKind as K;
    let kind = &component.kind;
    if let K::Other(name) = kind {
        return Err(ImportError::UnsupportedKind {
            component: component.id.clone(),
            kind: name.clone(),
        });
    }
    if kind.is_passive() {
        return Ok(None);
    }
    for name in component.params.keys() {
        if !kind.params().contains(&name.as_str()) {
            return Err(ImportError::UnknownParameter {
                component: component.id.clone(),
                kind: kind.name().to_string(),
                parameter: name.clone(),
            });
        }
    }
    if let Some(model) = &component.model {
        if !kind.takes_model(model) {
            return Err(ImportError::WrongModel {
                component: component.id.clone(),
                kind: kind.name().to_string(),
                model: model.name().to_string(),
            });
        }
    }

    let terminals = kind.terminals();
    let required = terminals.len() - kind.optional_terminals();
    let mut nodes = Vec::with_capacity(required);
    for (i, names) in terminals.iter().enumerate().take(required) {
        nodes.push(node(i).ok_or_else(|| ImportError::Unconnected {
            component: component.id.clone(),
            terminal: names[0].to_string(),
        })?);
    }
    let n2 = || [nodes[0].clone(), nodes[1].clone()];
    let n3 = || [nodes[0].clone(), nodes[1].clone(), nodes[2].clone()];
    let n4 = || {
        [
            nodes[0].clone(),
            nodes[1].clone(),
            nodes[2].clone(),
            nodes[3].clone(),
        ]
    };

    let r = Reader { component };
    let id = component.id.clone();
    let model = component.model.as_ref();
    let temperature = || r.param_or("temperature", NOMINAL_K);

    let element = match kind {
        K::Resistor => CircuitElement::Resistor {
            id,
            nodes: n2(),
            resistance: r.required_value("a resistance (Ω)")?,
        },
        K::Capacitor | K::CapacitorPolarized => CircuitElement::Capacitor {
            id,
            nodes: n2(),
            capacitance: r.required_value("a capacitance (F)")?,
        },
        K::Inductor => CircuitElement::Inductor {
            id,
            nodes: n2(),
            inductance: r.required_value("an inductance (H)")?,
            saturation: None,
        },
        K::Diode => CircuitElement::Diode {
            id,
            nodes: n2(),
            temperature: temperature()?,
            model: match model {
                Some(Model::Diode(m)) => Some(*m),
                _ => None,
            },
        },
        K::SchottkyDiode => CircuitElement::SchottkyDiode {
            id,
            nodes: n2(),
            temperature: temperature()?,
        },
        K::Photodiode => CircuitElement::Photodiode {
            id,
            nodes: n2(),
            irradiance: r.param_or("irradiance", 0.0)?,
            temperature: temperature()?,
        },
        K::ZenerDiode => CircuitElement::ZenerDiode {
            id,
            nodes: n2(),
            vz: match r.param(&["vz"])? {
                Some(vz) => vz,
                None => r.value()?.ok_or_else(|| ImportError::MissingValue {
                    component: component.id.clone(),
                    what: "a breakdown voltage (V), as the value or \"vz\"".into(),
                })?,
            },
            temperature: temperature()?,
        },
        K::Led => CircuitElement::Led {
            id,
            nodes: n2(),
            color: r.text("color")?.unwrap_or_else(|| "red".into()),
            temperature: temperature()?,
        },
        K::Varactor => CircuitElement::Varactor {
            id,
            nodes: n2(),
            params: match model {
                Some(Model::Varactor(p)) => p.clone(),
                _ => VaractorParams::default(),
            },
        },
        K::Npn | K::Pnp => CircuitElement::Bjt {
            id,
            nodes: n3(),
            kind: if *kind == K::Npn {
                BjtKind::Npn
            } else {
                BjtKind::Pnp
            },
            bf: r.param(&["bf", "beta", "hfe"])?.unwrap_or(100.0),
            temperature: temperature()?,
            parasitic_caps: None,
            model: match model {
                Some(Model::Bjt(m)) => Some(*m),
                _ => None,
            },
        },
        K::Nmos | K::Pmos => {
            if let Some(body) = node(3) {
                if body != nodes[2] {
                    return Err(ImportError::BodyNotAtSource {
                        component: component.id.clone(),
                    });
                }
            }
            let nmos = *kind == K::Nmos;
            CircuitElement::Mosfet {
                id,
                nodes: n3(),
                kind: if nmos {
                    MosfetKind::Nmos
                } else {
                    MosfetKind::Pmos
                },
                params: match model {
                    Some(Model::Mosfet(p)) => p.clone(),
                    _ if nmos => MosfetParams::default_nmos(),
                    _ => MosfetParams::default_pmos(),
                },
                parasitic_caps: None,
            }
        }
        K::Njfet | K::Pjfet => {
            let n = *kind == K::Njfet;
            CircuitElement::Jfet {
                id,
                nodes: n3(),
                kind: if n {
                    JfetKind::NChannel
                } else {
                    JfetKind::PChannel
                },
                idss: r.param_or("idss", 10e-3)?,
                vp: r.param_or("vp", if n { -2.0 } else { 2.0 })?,
            }
        }
        K::Igbt => CircuitElement::Igbt {
            id,
            nodes: n3(),
            params: match model {
                Some(Model::Igbt(p)) => p.clone(),
                _ => IgbtParams::default(),
            },
        },
        K::OpAmp | K::Comparator => {
            let rail = |name: &str, slot: usize, default: f64| -> Result<f64, ImportError> {
                Ok(match r.param(&[name])? {
                    Some(v) => v,
                    None => node(slot)
                        .and_then(|n| supply.get(&n).copied())
                        .unwrap_or(default),
                })
            };
            let v_pos = rail("v_pos", 3, 15.0)?;
            let v_neg = rail("v_neg", 4, -15.0)?;
            if *kind == K::OpAmp {
                CircuitElement::OpAmp {
                    id,
                    nodes: n3(),
                    v_pos,
                    v_neg,
                    model: match model {
                        Some(Model::OpAmp(m)) => Some(*m),
                        _ => None,
                    },
                }
            } else {
                CircuitElement::Comparator {
                    id,
                    nodes: n3(),
                    v_pos,
                    v_neg,
                }
            }
        }
        K::Potentiometer => CircuitElement::Potentiometer {
            id,
            nodes: n3(),
            resistance: r.required_value("an end-to-end resistance (Ω)")?,
            position: r.param_or("position", 0.5)?,
        },
        K::Switch => CircuitElement::Switch {
            id,
            nodes: n2(),
            closed: r.flag("closed")?,
        },
        K::PushButton => CircuitElement::Pushbutton {
            id,
            nodes: n2(),
            closed: r.flag("closed")?,
        },
        K::Fuse => CircuitElement::Fuse {
            id,
            nodes: n2(),
            rating: r.value()?.unwrap_or(1.0),
            blown: r.flag("blown")?,
        },
        K::Relay => CircuitElement::Relay {
            id,
            nodes: n4(),
            coil_resistance: r.param_or("coil_resistance", 500.0)?,
            pickup_voltage: r.required_param("pickup_voltage", "a pickup voltage (V)")?,
            inductance: r.param_or("inductance", 0.0)?,
        },
        K::Transformer => CircuitElement::Transformer {
            id,
            nodes: n4(),
            l1: r.required_param("l1", "a primary inductance (H)")?,
            l2: r.required_param("l2", "a secondary inductance (H)")?,
            k: r.param_or("k", 0.999)?,
        },
        K::Regulator => CircuitElement::VoltageRegulator {
            id,
            nodes: n3(),
            voltage: match r.param(&["voltage"])? {
                Some(v) => v,
                None => r.value()?.ok_or_else(|| ImportError::MissingValue {
                    component: component.id.clone(),
                    what: "an output voltage (V), as the value or \"voltage\"".into(),
                })?,
            },
            model: match model {
                Some(Model::Regulator(p)) => Some(*p),
                _ => None,
            },
        },
        K::Battery => CircuitElement::Battery {
            id,
            nodes: n2(),
            params: match model {
                Some(Model::Battery(p)) => p.clone(),
                _ => BatteryParams::constant(
                    r.required_value("a voltage (V), or a battery model")?,
                    r.param_or("capacity", 1.0)?,
                    r.param_or("r_series", 0.1)?,
                ),
            },
            state_of_charge: r.param_or("state_of_charge", 1.0)?,
        },
        K::Thermistor => CircuitElement::Thermistor {
            id,
            nodes: n2(),
            temperature: r.param_or("temperature", 298.15)?,
        },
        K::Photoresistor => CircuitElement::Photoresistor {
            id,
            nodes: n2(),
            light_level: r.param_or("light_level", 0.0)?,
        },
        K::VoltageSource => CircuitElement::VoltageSource {
            id,
            nodes: n2(),
            voltage: r.required_value("a voltage (V)")?,
            waveform: None,
        },
        K::CurrentSource => CircuitElement::CurrentSource {
            id,
            nodes: n2(),
            current: r.required_value("a current (A)")?,
            waveform: None,
        },
        K::Connector | K::TestPoint | K::NetTie | K::MountingHole | K::Other(_) => return Ok(None),
    };
    Ok(Some(element))
}

#[cfg(test)]
mod tests;
