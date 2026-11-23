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
//! depend on the package. Names are compared without case, with `-`, `_`
//! and spaces alike (`Non-inverting`, `non_inverting`); each kind lists
//! the names it takes in [`ComponentKind::terminals`]. Unpolarised
//! two-terminal parts also take `1` and `2`. Kind names read the same way,
//! so a schematic tool's symbol id (`resistor-iec`, `op-amp`, `header-4`)
//! names its kind.
//!
//! # Netlists as an editor exports them
//!
//! An editor's nets often name a terminal by its id (`"1"`) and keep the
//! terminal's name (`"Base"`) in a table on the component. Pass that table
//! in [`ImportOptions::with_terminal_names`] and import with
//! [`Netlist::to_circuit_with`] or [`simulate_with`]: a terminal with a
//! name is matched by its name, one without by its id. A bare pin number
//! still never picks a polarised terminal.
//!
//! A part sindr can't simulate, such as an IC, is a
//! [`ComponentKind::BlackBox`]: it adds no element, but a bench can still
//! attach to its terminals. Any other kind sindr doesn't know is refused
//! by default; [`UnknownKinds::BlackBox`] makes those black boxes too.
//! Either way each one is listed in [`ImportedCircuit::black_boxes`].
//! A net a black box leaves floating (a pull-up into an MCU input, a
//! resistor between two IC pins) is tied to ground through
//! [`HIGH_IMPEDANCE_OHMS`], the pin's leakage, so the rest of the circuit
//! solves; each is listed in [`ImportedCircuit::high_impedance`].
//!
//! An editor's "exclude from simulation" attribute maps onto the same
//! thing: send that part as a [`ComponentKind::BlackBox`], and its nets
//! stay for the bench without an element.
//!
//! # Schematics still being drawn
//!
//! A required terminal on no net refuses the import with
//! [`ImportError::Unconnected`]. A schematic being drawn always has loose
//! ends, so [`UnconnectedTerminals::Skip`] leaves such a component out
//! instead: it adds no element, its connected nets stay, an
//! [`ImportWarning`] names it, and it is listed in
//! [`ImportedCircuit::skipped`], so the finished part of the design still
//! simulates.
//!
//! # Models from part data
//!
//! A part's model comes from its [`Component::model`], its value (an LED's
//! `"Amber"`), its parameters, or datasheet figures an editor's parts
//! library holds (`vf` at `if`, `hfe`, `vce_sat`, `vth`, `rds_on`), fitted
//! by the constructors in [`crate::datasheet`]; [`ComponentKind`] lists
//! what each kind reads. When none of them says anything, sindr's default
//! model is used, and never silently: [`ImportedCircuit::models`] records
//! each component's [`ModelSource`], and [`ImportedCircuit::warnings`]
//! carries an [`ImportWarning`] for each one that fell back to the default.
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
    simulate, simulate_with, supply_names, Analysis, AssumedSupply, Attach, Bench, BenchError,
    BenchLoad, BenchSource, Imported, LoadKind, Simulation, SourceKind, SupplyNames,
};
pub use value::parse_value;

use std::collections::{BTreeMap, BTreeSet};

use crate::circuit::{
    BjtModel, Circuit, CircuitElement, CrystalParams, DiodeModel, FerriteBeadParams, OpAmpModel,
};
use crate::datasheet::{
    BjtDatasheet, DatasheetError, DatasheetValue, DiodeDatasheet, MosfetDatasheet, ZenerDatasheet,
};
use crate::design::Part;
use sindr_devices::battery::BatteryParams;
use sindr_devices::bjt::BjtKind;
use sindr_devices::igbt::IgbtParams;
use sindr_devices::jfet::JfetKind;
use sindr_devices::led;
use sindr_devices::mosfet::{MosfetKind, MosfetParams};
use sindr_devices::regulator::RegulatorParams;
use sindr_devices::varactor::VaractorParams;

/// A schematic's components and the nets joining them.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
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
    /// Further parameters by name, such as `bf` for a transistor's gain,
    /// `color` for an LED, or datasheet figures such as `vf` and `if`. Each
    /// kind's are listed on [`ComponentKind`] (figures also take `_min`,
    /// `_typ` and `_max`); any other name is an
    /// [`ImportError::UnknownParameter`].
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

/// A value or parameter: a number in SI base units, text, a flag, or a
/// typed quantity.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(untagged))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Value {
    /// A number in SI base units (Ω, F, H, V, A, Hz, K).
    Number(f64),
    /// A flag, such as a switch's `closed`.
    Bool(bool),
    /// Text: a value as written (`"4k7"`, read by [`parse_value`]) or a
    /// name (an LED's `"red"`).
    Text(String),
    /// A typed quantity, as a parts library stores it: a number in base
    /// units with its unit, and the text it was shown as. In JSON,
    /// `{"value": 300, "unit": "K", "text": "300 K"}`. A quantity whose
    /// unit isn't the one a value or parameter is in is an
    /// [`ImportError::BadValue`]. Boxed, so a `Value` stays as small as
    /// its text.
    Quantity(Box<Quantity>),
}

/// A number in base units with its unit, as a parts library stores a
/// typed spec: `Quantity::new(0.02, Unit::Ampere)` for 20 mA.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// or build it with [`new`](Self::new) and [`with_text`](Self::with_text).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Quantity {
    /// The number, in the unit's base unit (A, not mA), exact as the
    /// library stores it.
    pub value: f64,
    /// The unit.
    pub unit: Unit,
    /// The quantity as the editor shows it (`"20 mA"`), for messages; the
    /// number is [`value`](Self::value), not this text.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub text: Option<String>,
}

impl Quantity {
    /// `value` in `unit`, with no text.
    pub fn new(value: f64, unit: Unit) -> Self {
        Self {
            value,
            unit,
            text: None,
        }
    }

    /// Sets [`text`](Self::text).
    pub fn with_text(mut self, text: impl Into<String>) -> Self {
        self.text = Some(text.into());
        self
    }
}

/// The unit of a [`Quantity`]: the base units a parts library stores
/// specs in. Serialised as its symbol: `"Ohm"`, `"F"`, `"H"`, `"V"`,
/// `"A"`, `"W"`, `"Hz"`, `"s"`, `"m"`, `"K"`, and `"1"` for a plain ratio
/// (a gain, a position). [`from_symbol`](Self::from_symbol) also reads
/// `Ω`; any other symbol is [`Other`](Self::Other).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(from = "String", into = "String"))]
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Unit {
    /// Ohm (Ω).
    Ohm,
    /// Farad.
    Farad,
    /// Henry.
    Henry,
    /// Volt.
    Volt,
    /// Ampere.
    Ampere,
    /// Watt.
    Watt,
    /// Hertz.
    Hertz,
    /// Second.
    Second,
    /// Metre.
    Metre,
    /// Kelvin.
    Kelvin,
    /// A plain ratio, unit `1`: a gain, a coupling, a position.
    #[default]
    Ratio,
    /// A unit sindr doesn't know, by its symbol. Accepted where any unit is.
    Other(String),
}

const UNIT_SYMBOLS: &[(&str, Unit)] = &[
    ("Ohm", Unit::Ohm),
    ("F", Unit::Farad),
    ("H", Unit::Henry),
    ("V", Unit::Volt),
    ("A", Unit::Ampere),
    ("W", Unit::Watt),
    ("Hz", Unit::Hertz),
    ("s", Unit::Second),
    ("m", Unit::Metre),
    ("K", Unit::Kelvin),
    ("1", Unit::Ratio),
];

impl Unit {
    /// The unit with this symbol: `"Ohm"` (or `"Ω"`, `"ohm"`), `"F"`,
    /// `"H"`, `"V"`, `"A"`, `"W"`, `"Hz"`, `"s"`, `"m"`, `"K"`, `"1"`.
    /// Case matters except for `Ohm` and `Hz`, since `m` and `K` aren't
    /// `M` and `k`.
    pub fn from_symbol(symbol: &str) -> Self {
        let symbol = symbol.trim();
        if ["\u{3a9}", "\u{2126}"].contains(&symbol)
            || symbol.eq_ignore_ascii_case("ohm")
            || symbol.eq_ignore_ascii_case("ohms")
        {
            return Unit::Ohm;
        }
        if symbol.eq_ignore_ascii_case("hz") {
            return Unit::Hertz;
        }
        UNIT_SYMBOLS
            .iter()
            .find(|(s, _)| *s == symbol)
            .map_or_else(|| Unit::Other(symbol.to_string()), |(_, u)| u.clone())
    }

    /// The unit's symbol, as [`from_symbol`](Self::from_symbol) reads it.
    pub fn symbol(&self) -> &str {
        match self {
            Unit::Other(s) => s,
            unit => UNIT_SYMBOLS
                .iter()
                .find(|(_, u)| u == unit)
                .map_or("", |(s, _)| s),
        }
    }
}

impl From<String> for Unit {
    fn from(symbol: String) -> Self {
        Unit::from_symbol(&symbol)
    }
}

impl From<Unit> for String {
    fn from(unit: Unit) -> Self {
        unit.symbol().to_string()
    }
}

impl From<Quantity> for Value {
    fn from(q: Quantity) -> Self {
        Value::Quantity(Box::new(q))
    }
}

/// Where an imported component's model came from. Recorded for every
/// component that becomes an element, in
/// [`ImportedCircuit::models`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ModelSource {
    /// [`Component::model`].
    Model,
    /// [`Component::value`]: a resistance, a zener's voltage, an LED's
    /// colour named in the value text.
    Value,
    /// Named parameters that set the model directly: `bf`, `color`,
    /// `idss`.
    Params,
    /// Datasheet figures (`vf` at `if`, `hfe`, `vce_sat`, `vth`,
    /// `rds_on` …), fitted by the `from_datasheet` constructors in
    /// [`crate::datasheet`].
    Datasheet,
    /// Nothing the component gave: sindr's built-in model for the kind.
    /// Each such component has an [`ImportWarning`].
    Default,
}

/// Something about an import worth checking, though it isn't an error:
/// mostly a component that got sindr's default model because nothing in
/// its data said otherwise.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct ImportWarning {
    /// The component's id.
    pub component: String,
    /// What happened, and what sindr used instead.
    pub message: String,
}

impl std::fmt::Display for ImportWarning {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "component \"{}\": {}", self.component, self.message)
    }
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
#[non_exhaustive]
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
/// (`"resistor"`, `"npn"`). [`from_name`](Self::from_name) also reads a
/// schematic tool's symbol ids (`"resistor-iec"`, `"op-amp"`); a name
/// sindr doesn't know reads as [`Other`](Self::Other), which
/// [`Netlist::to_circuit`] refuses with the component's id unless
/// [`UnknownKinds::BlackBox`] says to keep it as a black box.
///
/// Parameters each kind takes, besides [`Component::value`] (numbers in SI
/// units, or text for [`parse_value`]):
///
/// | Kind | Value | Parameters | Model |
/// |---|---|---|---|
/// | `resistor` | Ω, required | | |
/// | `capacitor`, `capacitor_polarized` | F, required | | |
/// | `inductor` | H, required | | |
/// | `diode` | | `vf` (V) at `if` (A), and `vf2` at `if2`; `temperature` (K) | `Diode` |
/// | `schottky_diode`, `photodiode` | | `temperature` (K); photodiode `irradiance` (W) | |
/// | `zener_diode` | V (breakdown) or `vz` | `vz` at `iz` (A); `temperature` | |
/// | `led` | colour named in the text (`"Amber"`, `"LED red 3mm"`) | `color` or `colour` (`red`, `orange`, `amber`, `yellow`, `green`, `blue`, `white`, any case); `vf` (V) at `if` (A), and `vf2` at `if2`; `temperature` | |
/// | `varactor` | | | `Varactor` |
/// | `npn`, `pnp` | | `bf` (or `beta`); `hfe` at `hfe_ic` (A); `vce_sat` (V) at `vce_sat_ic` and `vce_sat_ib` (A); `temperature` | `Bjt` |
/// | `nmos`, `pmos` | | `vth` (V) at `vth_id` (A); `rds_on` (Ω) at `rds_on_vgs` (V), optionally at `rds_on_id` (A) | `Mosfet` |
/// | `njfet`, `pjfet` | | `idss` (A), `vp` (V) | |
/// | `igbt` | | | `Igbt` |
/// | `op_amp`, `comparator` | | `v_pos`, `v_neg` (V; default from bench supplies on its supply pins, else ±15) | op-amp: `OpAmp` |
/// | `dual_op_amp`, `quad_op_amp`, `dual_comparator`, `quad_comparator` | | as `op_amp` and `comparator`, for every unit | op-amps: `OpAmp` |
/// | `potentiometer` | Ω, required | `position` (0 top to 1 bottom, default 0.5) | |
/// | `switch`, `push_button` | | `closed` (default false) | |
/// | `fuse` | A (rating) | `blown` (default false) | |
/// | `relay` | | `pickup_voltage` (V, required), `coil_resistance` (Ω), `inductance` (H) | |
/// | `transformer` | | `l1`, `l2` (H, required), `k` | |
/// | `crystal` | Hz (series resonance) or `frequency` | `frequency`, `esr` (Ω, default 50), `c0` (F, default 5 pF) | |
/// | `ferrite_bead` | Ω (impedance at 100 MHz) or `z_100mhz` | `z_100mhz`, `r_dc` (Ω, default 0.1) | |
/// | `regulator` | V (output) or `voltage` | `voltage` | `Regulator` |
/// | `battery` | V, unless a model | `state_of_charge` (default 1), `r_series` (Ω, default 0.1), `capacity` (Ah, default 1) | `Battery` |
/// | `thermistor` | | `temperature` (K) | |
/// | `photoresistor` | | `light_level` (0 dark to 1 bright) | |
/// | `voltage_source`, `current_source` | V or A, required | | |
/// | `connector`, `test_point`, `net_tie`, `mounting_hole` | | | |
/// | `black_box` | | | |
///
/// # Part data
///
/// `vf`, `vf2`, `vz`, `hfe`, `vce_sat`, `vth` and `rds_on` are datasheet
/// figures: each also takes `_min`, `_typ` and `_max` forms (`hfe_min`,
/// `vce_sat_max`), and a fit uses the typical value, else the middle of
/// minimum and maximum, else whichever is given
/// ([`DatasheetValue::nominal`](crate::datasheet::DatasheetValue::nominal)).
/// The other names are test conditions, read as given. A figure needs its
/// condition (`vf` needs `if`, `vth` needs `vth_id` and `rds_on`), or the
/// import is an [`ImportError::MissingValue`]. What each fit does:
///
/// - **Diode**: [`DiodeModel::from_datasheet`]: IS through `vf` at `if`
///   (N = 1), or IS and N through both points.
/// - **LED** with `vf`: [`DiodeModel::led_from_datasheet`], imported as a
///   [`CircuitElement::Diode`] with that model, since the LED element knows
///   only its colour's preset. Without `vf`, the colour sets the preset; a
///   colour from the `color` parameter wins over one in the value text.
/// - **Zener** with `iz`: [`DiodeModel::zener_from_datasheet`] sets the
///   element's `vz` so that it carries `iz` at the datasheet's Vz.
/// - **NPN, PNP**: `bf` is `hfe` (or `bf`/`beta`, which win); with
///   `vce_sat`, [`BjtModel::from_datasheet`] fits BR through it. `hfe_ic`
///   is kept for the record: the model's gain doesn't change with current.
/// - **NMOS, PMOS**: [`MosfetParams::from_datasheet`] fits KP and VTO.
///
/// [`ImportedCircuit::models`] says, for every component, which of model,
/// value, parameters, datasheet figures or sindr's default the model came
/// from, and [`ImportedCircuit::warnings`] has one warning for every
/// component that got the default (a transistor whose value is only a part
/// number, an LED with no colour). Values and parameters may be typed
/// quantities ([`Value::Quantity`]), whose unit is checked against the
/// parameter's.
///
/// # Other kinds
///
/// The last two rows add no element, and any terminal name is fine. A
/// bench attaches sources and loads to their terminals. The connectors and
/// the like are [passive](Self::is_passive); a black box stands for a part
/// sindr can't simulate (an IC), takes any value, parameters and model and
/// ignores them, and is listed in [`ImportedCircuit::black_boxes`].
///
/// A dual or quad part is one package with two or four units sharing its
/// supply pins. Each unit becomes its own element, `<id>.a`, `<id>.b`
/// and so on, so results are keyed by those; [`Component::part`] stays
/// under the package's id. A unit's terminals are an op-amp's with the
/// unit's letter or number on them: `a_in+`, `in+ a`, `1IN+`, `IN1+`,
/// `INA+`, `-IN B`, `OUT B`, `2OUT`, `Non-inverting A`. The supplies are `v+` and
/// `v-` (or `vcc`, `vee`, `positive supply` …), without a unit. A unit
/// with none of its terminals connected (a spare half) adds no element;
/// one connected in part is an [`ImportError::Unconnected`] as for a
/// single op-amp (or skipped, under [`UnconnectedTerminals::Skip`]).
///
/// Names besides the canonical ones: `resistor_iec`, `resistor_us`, `res`;
/// `cap`, `capacitor_polarised`, `capacitor_electrolytic`, `electrolytic`;
/// `zener`, `schottky`; `opamp`, `operational_amplifier`;
/// `op_amp_dual`, `dual_opamp`, `op_amp_quad`, `quad_opamp`,
/// `comparator_dual`, `comparator_quad`; `pot`,
/// `trimmer`; `pushbutton`, `button`; `switch_spst`, `spst`;
/// `xtal`, `quartz`; `ferrite`, `bead`; `ldo`, `voltage_regulator`;
/// `cell`; `ntc`; `ldr`; `testpoint`; `header`, `header_<n>`,
/// `pin_header`, `terminal_block`, `conn` for a connector; `ic` for a
/// black box.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(from = "String", into = "String"))]
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[allow(missing_docs)]
#[non_exhaustive]
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
    /// Two op-amps in one package, sharing its supply pins.
    DualOpAmp,
    /// Four op-amps in one package, sharing its supply pins.
    QuadOpAmp,
    /// Two comparators in one package, sharing its supply pins.
    DualComparator,
    /// Four comparators in one package, sharing its supply pins.
    QuadComparator,
    Potentiometer,
    Switch,
    PushButton,
    Fuse,
    Relay,
    Transformer,
    Regulator,
    Battery,
    /// A quartz crystal ([`CircuitElement::Crystal`]).
    Crystal,
    /// A ferrite bead ([`CircuitElement::FerriteBead`]).
    FerriteBead,
    /// A part sindr doesn't simulate, kept for its terminals: it adds no
    /// element, and a bench may attach to its terminals.
    BlackBox,
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
    ("dual_op_amp", ComponentKind::DualOpAmp),
    ("quad_op_amp", ComponentKind::QuadOpAmp),
    ("dual_comparator", ComponentKind::DualComparator),
    ("quad_comparator", ComponentKind::QuadComparator),
    ("potentiometer", ComponentKind::Potentiometer),
    ("switch", ComponentKind::Switch),
    ("push_button", ComponentKind::PushButton),
    ("fuse", ComponentKind::Fuse),
    ("relay", ComponentKind::Relay),
    ("transformer", ComponentKind::Transformer),
    ("regulator", ComponentKind::Regulator),
    ("crystal", ComponentKind::Crystal),
    ("ferrite_bead", ComponentKind::FerriteBead),
    ("black_box", ComponentKind::BlackBox),
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

/// Other names for kinds, normalised (see [`normalise`]).
const ALIASES: &[(&str, ComponentKind)] = &[
    ("resistor_iec", ComponentKind::Resistor),
    ("resistor_us", ComponentKind::Resistor),
    ("res", ComponentKind::Resistor),
    ("cap", ComponentKind::Capacitor),
    ("capacitor_polarised", ComponentKind::CapacitorPolarized),
    ("capacitor_electrolytic", ComponentKind::CapacitorPolarized),
    ("electrolytic", ComponentKind::CapacitorPolarized),
    ("zener", ComponentKind::ZenerDiode),
    ("schottky", ComponentKind::SchottkyDiode),
    ("opamp", ComponentKind::OpAmp),
    ("operational_amplifier", ComponentKind::OpAmp),
    ("op_amp_dual", ComponentKind::DualOpAmp),
    ("dual_opamp", ComponentKind::DualOpAmp),
    ("opamp_dual", ComponentKind::DualOpAmp),
    ("op_amp_quad", ComponentKind::QuadOpAmp),
    ("quad_opamp", ComponentKind::QuadOpAmp),
    ("opamp_quad", ComponentKind::QuadOpAmp),
    ("comparator_dual", ComponentKind::DualComparator),
    ("comparator_quad", ComponentKind::QuadComparator),
    ("pot", ComponentKind::Potentiometer),
    ("trimmer", ComponentKind::Potentiometer),
    ("pushbutton", ComponentKind::PushButton),
    ("button", ComponentKind::PushButton),
    ("switch_spst", ComponentKind::Switch),
    ("spst", ComponentKind::Switch),
    ("xtal", ComponentKind::Crystal),
    ("quartz", ComponentKind::Crystal),
    ("ferrite", ComponentKind::FerriteBead),
    ("bead", ComponentKind::FerriteBead),
    ("ldo", ComponentKind::Regulator),
    ("voltage_regulator", ComponentKind::Regulator),
    ("cell", ComponentKind::Battery),
    ("ntc", ComponentKind::Thermistor),
    ("ldr", ComponentKind::Photoresistor),
    ("testpoint", ComponentKind::TestPoint),
    ("header", ComponentKind::Connector),
    ("pin_header", ComponentKind::Connector),
    ("terminal_block", ComponentKind::Connector),
    ("conn", ComponentKind::Connector),
    ("ic", ComponentKind::BlackBox),
];

/// An op-amp's supply pins, by the names they answer to.
const V_POS: &[&str] = &["v+", "vcc", "vdd", "vs+", "v_pos", "positive_supply"];
const V_NEG: &[&str] = &["v-", "vee", "vss", "vs-", "v_neg", "negative_supply"];

/// The terminals of a multi-unit op-amp or comparator: each unit's
/// `in+`, `in-` and `out`, tagged with its letter and number, then the
/// shared supplies.
macro_rules! unit_terminals {
    ($(($x:literal, $d:literal)),*) => {
        &[
            $(
                &[
                    concat!($x, "_in+"), concat!("in+_", $x), concat!($d, "in+"),
                    concat!("in", $d, "+"), concat!("in", $x, "+"), concat!($x, "+"), concat!("+in_", $x),
                    concat!("non_inverting_", $x), concat!($x, "_non_inverting"),
                ],
                &[
                    concat!($x, "_in-"), concat!("in-_", $x), concat!($d, "in-"),
                    concat!("in", $d, "-"), concat!("in", $x, "-"), concat!($x, "-"), concat!("-in_", $x),
                    concat!("inverting_", $x), concat!($x, "_inverting"),
                ],
                &[
                    concat!($x, "_out"), concat!("out_", $x), concat!($d, "out"),
                    concat!("out", $d), concat!("out", $x),
                    concat!("output_", $x), concat!($x, "_output"),
                ],
            )*
            V_POS,
            V_NEG,
        ]
    };
}

const DUAL_UNIT_TERMINALS: &[&[&str]] = unit_terminals!(("a", "1"), ("b", "2"));
const QUAD_UNIT_TERMINALS: &[&[&str]] =
    unit_terminals!(("a", "1"), ("b", "2"), ("c", "3"), ("d", "4"));

/// A name as it's compared: trimmed, lowercase, and each run of `-`, `_`
/// and whitespace one `_`. A separator is kept rather than dropped so a
/// minus sign (`-`, `in-`) still means something.
fn normalise(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    let mut in_separator = false;
    for c in name.trim().chars() {
        if c == '-' || c == '_' || c.is_whitespace() {
            if !in_separator {
                out.push('_');
            }
            in_separator = true;
        } else {
            out.extend(c.to_lowercase());
            in_separator = false;
        }
    }
    out
}

impl ComponentKind {
    /// The kind with this name, or [`Other`](Self::Other) with the name as
    /// given. Case is ignored and `-`, `_` and spaces are alike, so a
    /// symbol id such as `"Op-Amp"` or `"push button"` reads too; the
    /// aliases on [`ComponentKind`] (`resistor-iec`, `header-4`, `xtal`)
    /// name kinds as well.
    pub fn from_name(name: &str) -> Self {
        let key = normalise(name);
        if let Some((_, kind)) = KINDS.iter().chain(ALIASES).find(|(n, _)| *n == key) {
            return kind.clone();
        }
        // `header_3`, `header_2x5`: a pin header of any size.
        if let Some(size) = key.strip_prefix("header_") {
            if size.starts_with(|c: char| c.is_ascii_digit())
                && size.chars().all(|c| c.is_ascii_digit() || c == 'x')
            {
                return ComponentKind::Connector;
            }
        }
        ComponentKind::Other(name.to_string())
    }

    /// The kind's snake_case name; an [`Other`](Self::Other) kind's own.
    pub fn name(&self) -> &str {
        match self {
            ComponentKind::Other(name) => name,
            kind => KINDS.iter().find(|(_, k)| k == kind).map_or("", |(n, _)| n),
        }
    }

    /// Whether the kind is a connector, test point, net tie or mounting
    /// hole: no element, any terminal name. A [`BlackBox`](Self::BlackBox)
    /// adds no element either, but isn't passive.
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
    /// answers to, lowercase, canonical first. A terminal given as
    /// `Non-inverting` or `positive supply` matches `non_inverting` and
    /// `positive_supply`: case is ignored and `-`, `_` and spaces are
    /// alike. Required terminals come first;
    /// [`optional_terminals`](Self::optional_terminals) says how many at
    /// the end may be left off. Empty for passive, black-box and unknown
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
            | K::Photoresistor
            | K::Crystal
            | K::FerriteBead => &[&["1", "a"], &["2", "b"]],
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
                V_POS,
                V_NEG,
            ],
            K::DualOpAmp | K::DualComparator => DUAL_UNIT_TERMINALS,
            K::QuadOpAmp | K::QuadComparator => QUAD_UNIT_TERMINALS,
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
            K::Connector
            | K::TestPoint
            | K::NetTie
            | K::MountingHole
            | K::BlackBox
            | K::Other(_) => &[],
        }
    }

    /// How many of [`terminals`](Self::terminals), at the end, may be left
    /// unconnected: a MOSFET's body (tied to its source), an op-amp's two
    /// supply pins (a multi-unit part's too).
    pub fn optional_terminals(&self) -> usize {
        match self {
            ComponentKind::Nmos | ComponentKind::Pmos => 1,
            ComponentKind::OpAmp | ComponentKind::Comparator => 2,
            ComponentKind::DualOpAmp
            | ComponentKind::QuadOpAmp
            | ComponentKind::DualComparator
            | ComponentKind::QuadComparator => 2,
            _ => 0,
        }
    }

    /// For a multi-unit part, the kind of each unit and how many there
    /// are: a dual op-amp is two [`OpAmp`](Self::OpAmp)s.
    pub fn units(&self) -> Option<(ComponentKind, usize)> {
        use ComponentKind as K;
        match self {
            K::DualOpAmp => Some((K::OpAmp, 2)),
            K::QuadOpAmp => Some((K::OpAmp, 4)),
            K::DualComparator => Some((K::Comparator, 2)),
            K::QuadComparator => Some((K::Comparator, 4)),
            _ => None,
        }
    }

    fn params(&self) -> &'static [&'static str] {
        use ComponentKind as K;
        match self {
            K::Diode => &["vf", "if", "vf2", "if2", "temperature"],
            K::SchottkyDiode => &["temperature"],
            K::Photodiode => &["temperature", "irradiance"],
            K::ZenerDiode => &["vz", "iz", "temperature"],
            K::Led => &["color", "colour", "vf", "if", "vf2", "if2", "temperature"],
            K::Npn | K::Pnp => &[
                "bf",
                "beta",
                "hfe",
                "hfe_ic",
                "vce_sat",
                "vce_sat_ic",
                "vce_sat_ib",
                "temperature",
            ],
            K::Nmos | K::Pmos => &["vth", "vth_id", "rds_on", "rds_on_vgs", "rds_on_id"],
            K::Njfet | K::Pjfet => &["idss", "vp"],
            K::OpAmp
            | K::Comparator
            | K::DualOpAmp
            | K::QuadOpAmp
            | K::DualComparator
            | K::QuadComparator => &["v_pos", "v_neg"],
            K::Potentiometer => &["position"],
            K::Switch | K::PushButton => &["closed"],
            K::Fuse => &["blown"],
            K::Relay => &["pickup_voltage", "coil_resistance", "inductance"],
            K::Transformer => &["l1", "l2", "k"],
            K::Regulator => &["voltage"],
            K::Crystal => &["frequency", "esr", "c0"],
            K::FerriteBead => &["z_100mhz", "r_dc"],
            K::Battery => &["state_of_charge", "r_series", "capacity"],
            K::Thermistor => &["temperature"],
            K::Photoresistor => &["light_level"],
            _ => &[],
        }
    }

    /// The parameters that are datasheet figures, which also take `_min`,
    /// `_typ` and `_max` forms.
    fn figures(&self) -> &'static [&'static str] {
        use ComponentKind as K;
        match self {
            K::Diode | K::Led => &["vf", "vf2"],
            K::ZenerDiode => &["vz"],
            K::Npn | K::Pnp => &["hfe", "vce_sat"],
            K::Nmos | K::Pmos => &["vth", "rds_on"],
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
                | (K::OpAmp | K::DualOpAmp | K::QuadOpAmp, Model::OpAmp(_))
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
        /// The terminal as given, followed by its name in parentheses when
        /// [`ImportOptions::with_terminal_names`] gave it one.
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

/// What to do with a component of a kind sindr doesn't know
/// ([`ComponentKind::Other`]).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum UnknownKinds {
    /// Refuse the import with [`ImportError::UnsupportedKind`], naming the
    /// first such component. The default, and what
    /// [`Netlist::to_circuit`] does.
    #[default]
    Refuse,
    /// Import it as a [`ComponentKind::BlackBox`]: no element, terminals
    /// kept for the bench, listed in [`ImportedCircuit::black_boxes`].
    BlackBox,
}

/// What to do with a component that has a required terminal on no net.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum UnconnectedTerminals {
    /// Refuse the import with [`ImportError::Unconnected`], naming the
    /// first such terminal. The default, and what [`Netlist::to_circuit`]
    /// does.
    #[default]
    Error,
    /// Leave the component out: no element, its connected nets kept, an
    /// [`ImportWarning`] naming it, and listed in
    /// [`ImportedCircuit::skipped`]. For a schematic still being drawn.
    Skip,
}

impl UnconnectedTerminals {
    #[cfg(feature = "serde")]
    fn is_error(&self) -> bool {
        *self == Self::Error
    }
}

/// How [`Netlist::to_circuit_with`] and [`simulate_with`] read a netlist:
/// what to do with unknown kinds and unconnected terminals, and each
/// component's terminal names.
///
/// Built with [`new`](Self::new) and the `with_*` setters. The default
/// reads a netlist as [`Netlist::to_circuit`] does.
///
/// ```
/// use sindr::import::{ImportOptions, UnknownKinds};
///
/// let options = ImportOptions::new()
///     .with_unknown_kinds(UnknownKinds::BlackBox)
///     .with_terminal_names("q1", [("1", "Base"), ("2", "Emitter"), ("3", "Collector")]);
/// assert_eq!(options.terminal_names["q1"]["1"], "Base");
/// ```
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Default, PartialEq)]
#[non_exhaustive]
pub struct ImportOptions {
    /// What to do with a component of a kind sindr doesn't know.
    pub unknown_kinds: UnknownKinds,
    /// What to do with a component that has a required terminal on no net.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "UnconnectedTerminals::is_error")
    )]
    pub unconnected: UnconnectedTerminals,
    /// Terminal names by component id, then terminal id: the table a
    /// schematic tool keeps on each component (`"1"` is `"Base"`). A pin
    /// whose terminal has a name here is matched by that name, then by its
    /// id; one without is matched by the id alone, so a bare pin number
    /// still can't pick a polarised terminal. On a passive or black-box
    /// part, a bench may attach to a terminal by its name. Names for a
    /// component that isn't in the netlist are unused.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "BTreeMap::is_empty"))]
    pub terminal_names: BTreeMap<String, BTreeMap<String, String>>,
}

impl ImportOptions {
    /// Options that read a netlist as [`Netlist::to_circuit`] does.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets [`unknown_kinds`](Self::unknown_kinds).
    pub fn with_unknown_kinds(mut self, unknown_kinds: UnknownKinds) -> Self {
        self.unknown_kinds = unknown_kinds;
        self
    }

    /// Sets [`unconnected`](Self::unconnected).
    pub fn with_unconnected(mut self, unconnected: UnconnectedTerminals) -> Self {
        self.unconnected = unconnected;
        self
    }

    /// Adds names for `component`'s terminals, as `(terminal id, name)`
    /// pairs. Leave out terminals that have no name.
    pub fn with_terminal_names<T, N>(
        mut self,
        component: impl Into<String>,
        names: impl IntoIterator<Item = (T, N)>,
    ) -> Self
    where
        T: Into<String>,
        N: Into<String>,
    {
        self.terminal_names
            .entry(component.into())
            .or_default()
            .extend(names.into_iter().map(|(t, n)| (t.into(), n.into())));
        self
    }

    /// Whether `component` is imported as a black box; an error for an
    /// unknown kind when those are refused.
    fn black_box(&self, component: &Component) -> Result<bool, ImportError> {
        match &component.kind {
            ComponentKind::BlackBox => Ok(true),
            ComponentKind::Other(_) if self.unknown_kinds == UnknownKinds::BlackBox => Ok(true),
            ComponentKind::Other(name) => Err(ImportError::UnsupportedKind {
                component: component.id.clone(),
                kind: name.clone(),
            }),
            _ => Ok(false),
        }
    }
}

/// A component left out under [`UnconnectedTerminals::Skip`] because a
/// terminal its element needs is on no net.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct SkippedPart {
    /// The component's id, or a dual or quad unit's (`<id>.a`).
    pub component: String,
    /// The required terminals on no net, by canonical name, in terminal
    /// order.
    pub unconnected: Vec<String>,
}

/// A component imported as a black box: a [`ComponentKind::BlackBox`],
/// or an unknown kind under [`UnknownKinds::BlackBox`]. It simulates
/// nothing.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct BlackBoxPart {
    /// The component's id.
    pub component: String,
    /// Its kind's name: `"black_box"`, or the unknown kind as given.
    pub kind: String,
    /// Net id by terminal id, for the terminals that are on a net.
    pub terminals: BTreeMap<String, String>,
}

/// The resistance an import ties a floating net to ground through: an IC
/// input's leakage, and SPICE's gmin (10⁻¹² S). A 10 kΩ pull-up into it
/// sits 33 pV below its rail.
pub const HIGH_IMPEDANCE_OHMS: f64 = 1e12;

/// A net whose only connections, besides at most one element terminal,
/// are pins of parts sindr doesn't simulate: an MCU input with a pull-up,
/// an I²C line between two black boxes, a resistor between two IC pins.
/// Electrically those pins are high impedance, so the net is held where its
/// one element puts it; for the solver it would be floating. The import
/// ties it to ground through [`HIGH_IMPEDANCE_OHMS`], as element
/// [`resistor`](Self::resistor), so the rest of the circuit solves.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct HighImpedanceNet {
    /// The node, by its name in the circuit (a net id; see
    /// [`ImportedCircuit::node`]).
    pub node: String,
    /// The id of the tie's resistor in [`ImportedCircuit::circuit`].
    pub resistor: String,
    /// The pins of unsimulated parts on the net, in netlist order.
    pub pins: Vec<Pin>,
}

/// Ties every node an unsimulated pin leaves floating (fewer than two
/// element terminals, or no path to ground but through such pins) to ground
/// through [`HIGH_IMPEDANCE_OHMS`].
fn tie_high_impedance_nets(
    circuit: &mut Circuit,
    unsimulated: BTreeMap<String, Vec<Pin>>,
    ids: &BTreeSet<&str>,
) -> Vec<HighImpedanceNet> {
    let ground = circuit.ground_node.clone();
    let floating: Vec<(String, Vec<Pin>)> = {
        let mut degree: BTreeMap<&str, usize> = BTreeMap::new();
        let mut adjacent: BTreeMap<&str, BTreeSet<&str>> = BTreeMap::new();
        for element in &circuit.components {
            let nodes = element.all_nodes();
            for n in &nodes {
                *degree.entry(n.as_str()).or_default() += 1;
                for m in &nodes {
                    adjacent.entry(n.as_str()).or_default().insert(m.as_str());
                }
            }
        }
        let mut grounded = BTreeSet::from([ground.as_str()]);
        let mut stack = vec![ground.as_str()];
        while let Some(n) = stack.pop() {
            for &m in adjacent.get(n).into_iter().flatten() {
                if grounded.insert(m) {
                    stack.push(m);
                }
            }
        }
        unsimulated
            .into_iter()
            .filter(|(n, _)| *n != ground)
            .filter(|(n, _)| {
                let d = degree.get(n.as_str()).copied().unwrap_or(0);
                d == 1 || (d > 1 && !grounded.contains(n.as_str()))
            })
            .collect()
    };
    let mut tied = Vec::new();
    for (node, pins) in floating {
        let mut resistor = format!("{node}.hi_z");
        let mut k = 2;
        while ids.contains(resistor.as_str())
            || circuit.components.iter().any(|e| e.id() == resistor)
        {
            resistor = format!("{node}.hi_z{k}");
            k += 1;
        }
        circuit.components.push(CircuitElement::Resistor {
            id: resistor.clone(),
            nodes: [node.clone(), ground.clone()],
            resistance: HIGH_IMPEDANCE_OHMS,
        });
        tied.push(HighImpedanceNet {
            node,
            resistor,
            pins,
        });
    }
    tied
}

/// A [`Circuit`] built from a [`Netlist`], with the map from the caller's
/// net ids to its node names.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct ImportedCircuit {
    /// The circuit. Element ids are component and bench ids; node names
    /// are net ids.
    pub circuit: Circuit,
    /// Node name by net id. Nets the bench ties together share one node,
    /// named after one of them; every other net's node is its own id.
    pub nodes: BTreeMap<String, String>,
    /// Every component imported as a black box, in netlist order. None of
    /// them is in [`circuit`](Self::circuit), so a result has nothing for
    /// them; check this list before calling a design done.
    pub black_boxes: Vec<BlackBoxPart>,
    /// Where each component's model came from, by component id, for every
    /// component that became an element (a dual or quad part once, under
    /// its package's id).
    pub models: BTreeMap<String, ModelSource>,
    /// Warnings, in netlist order: one for every component whose model is
    /// [`ModelSource::Default`], and any other part data sindr couldn't use.
    pub warnings: Vec<ImportWarning>,
    /// Nets left floating by parts that simulate nothing, each tied to
    /// ground through [`HIGH_IMPEDANCE_OHMS`] so the rest of the circuit
    /// solves. See [`HighImpedanceNet`].
    pub high_impedance: Vec<HighImpedanceNet>,
    /// Components left out under [`UnconnectedTerminals::Skip`], in
    /// netlist order. Like a black box, none of them is in
    /// [`circuit`](Self::circuit).
    pub skipped: Vec<SkippedPart>,
}

impl ImportedCircuit {
    /// The node name of net `net_id`, for looking up results.
    pub fn node(&self, net_id: &str) -> Option<&str> {
        self.nodes.get(net_id).map(String::as_str)
    }

    /// Where component `id`'s model came from; `None` for a component
    /// that isn't an element (passive, black box) or isn't in the netlist.
    pub fn model_source(&self, id: &str) -> Option<ModelSource> {
        self.models.get(id).copied()
    }

    /// The ids of the components that got sindr's default model, in id
    /// order.
    pub fn defaulted(&self) -> impl Iterator<Item = &str> {
        self.models
            .iter()
            .filter(|(_, s)| **s == ModelSource::Default)
            .map(|(id, _)| id.as_str())
    }
}

impl Netlist {
    /// Builds the circuit: the netlist's components, with `bench`'s ground,
    /// ties, sources and loads. The bench's analysis isn't run; see
    /// [`simulate`]. Unknown kinds are refused; [`to_circuit_with`]
    /// can keep them as black boxes.
    ///
    /// # Errors
    ///
    /// An [`ImportError`] naming the component, net or bench item that
    /// can't be built. Solver checks (a floating node, no path to ground)
    /// come later, from the analysis.
    ///
    /// [`to_circuit_with`]: Self::to_circuit_with
    pub fn to_circuit(&self, bench: &Bench) -> Result<ImportedCircuit, ImportError> {
        self.to_circuit_with(bench, &ImportOptions::default())
    }

    /// [`to_circuit`](Self::to_circuit), reading the netlist as `options`
    /// say: with each component's terminal names, and unknown kinds
    /// refused or kept as black boxes.
    ///
    /// ```
    /// use sindr::import::{Bench, BenchSource, Component, ComponentKind, ImportOptions,
    ///                     Net, Netlist, UnknownKinds};
    ///
    /// // A transistor whose nets name terminals by id, and an IC sindr
    /// // doesn't know on the supply.
    /// let netlist = Netlist {
    ///     components: vec![
    ///         Component::new("q1", ComponentKind::from_name("npn")),
    ///         Component::new("r1", ComponentKind::from_name("resistor-iec")).with_value("100k"),
    ///         Component::new("u1", ComponentKind::from_name("mcu-qfn32")).with_value("MCU"),
    ///     ],
    ///     nets: vec![
    ///         Net::new("vcc", [("r1", "1"), ("q1", "3"), ("u1", "8")]),
    ///         Net::new("base", [("r1", "2"), ("q1", "1")]),
    ///         Net::new("gnd", [("q1", "2"), ("u1", "1")]),
    ///     ],
    /// };
    /// let options = ImportOptions::new()
    ///     .with_unknown_kinds(UnknownKinds::BlackBox)
    ///     .with_terminal_names("q1", [("1", "Base"), ("2", "Emitter"), ("3", "Collector")]);
    /// let bench = Bench::new("gnd").with_source(BenchSource::voltage("vs", "vcc", 5.0));
    ///
    /// let imported = netlist.to_circuit_with(&bench, &options).unwrap();
    /// assert_eq!(imported.black_boxes[0].component, "u1");
    /// assert_eq!(imported.black_boxes[0].terminals["8"], "vcc");
    /// assert!(netlist.to_circuit(&bench).is_err());
    /// ```
    ///
    /// # Errors
    ///
    /// As [`to_circuit`](Self::to_circuit); with
    /// [`UnknownKinds::BlackBox`], never [`ImportError::UnsupportedKind`].
    pub fn to_circuit_with(
        &self,
        bench: &Bench,
        options: &ImportOptions,
    ) -> Result<ImportedCircuit, ImportError> {
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
                let slot = slot_of(component, &pin.terminal, options)?;
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
            options,
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

        let mut black_boxes = Vec::new();
        // Node by node, the pins of parts that simulate nothing (black
        // boxes, connectors, test points): what an IC pin leaves a net.
        let mut unsimulated: BTreeMap<String, Vec<Pin>> = BTreeMap::new();
        let pins_of = |id: &str| -> Vec<(String, Pin)> {
            self.nets
                .iter()
                .flat_map(|net| {
                    net.pins
                        .iter()
                        .filter(|p| p.component == id)
                        .map(|p| (node(&net.id), p.clone()))
                })
                .collect()
        };
        let mut models = BTreeMap::new();
        let mut warnings = Vec::new();
        let mut skipped = Vec::new();
        // Under `Skip`, a component (or unit) with a required terminal on
        // no net is listed and warned about; its connected pins are left
        // to the high-impedance tie as a black box's are. `true` when it
        // was skipped.
        let mut skip = |id: &str,
                        kind: &ComponentKind,
                        connected: &dyn Fn(usize) -> bool,
                        warnings: &mut Vec<ImportWarning>|
         -> bool {
            if options.unconnected != UnconnectedTerminals::Skip {
                return false;
            }
            let unconnected = missing_terminals(kind, connected);
            if unconnected.is_empty() {
                return false;
            }
            let list = unconnected
                .iter()
                .map(|t| format!("\"{t}\""))
                .collect::<Vec<_>>()
                .join(", ");
            warnings.push(ImportWarning {
                component: id.to_string(),
                message: format!(
                    "left out of the simulation: terminal{} {list} on no net",
                    if unconnected.len() == 1 { "" } else { "s" }
                ),
            });
            skipped.push(SkippedPart {
                component: id.to_string(),
                unconnected,
            });
            true
        };
        let record = |warnings: &mut Vec<ImportWarning>,
                      models: &mut BTreeMap<String, ModelSource>,
                      id: &str,
                      built: &Built| {
            if let Some(source) = built.source {
                models.entry(id.to_string()).or_insert(source);
            }
            for message in &built.warnings {
                let warning = ImportWarning {
                    component: id.to_string(),
                    message: message.clone(),
                };
                if !warnings.contains(&warning) {
                    warnings.push(warning);
                }
            }
        };
        for component in &self.components {
            if let Some(part) = &component.part {
                circuit.parts.insert(component.id.clone(), part.clone());
            }
            if options.black_box(component)? {
                let terminals = pin_net
                    .iter()
                    .filter(|((c, _), _)| *c == component.id)
                    .filter_map(|((_, slot), net)| match slot {
                        Slot::Named(t) => Some((t.clone(), net.to_string())),
                        Slot::Index(_) => None,
                    })
                    .collect();
                black_boxes.push(BlackBoxPart {
                    component: component.id.clone(),
                    kind: component.kind.name().to_string(),
                    terminals,
                });
                for (n, pin) in pins_of(&component.id) {
                    unsimulated.entry(n).or_default().push(pin);
                }
                continue;
            }
            let nodes_of = |slot: usize| -> Option<String> {
                pin_net
                    .get(&(component.id.as_str(), Slot::Index(slot)))
                    .map(|net| node(net))
            };
            let Some((unit_kind, count)) = component.kind.units() else {
                let connected = |i: usize| nodes_of(i).is_some();
                if skip(&component.id, &component.kind, &connected, &mut warnings) {
                    for (n, pin) in pins_of(&component.id) {
                        unsimulated.entry(n).or_default().push(pin);
                    }
                    continue;
                }
                let built = build_element(component, nodes_of, &supply)?;
                record(&mut warnings, &mut models, &component.id, &built);
                if built.element.is_none() {
                    for (n, pin) in pins_of(&component.id) {
                        unsimulated.entry(n).or_default().push(pin);
                    }
                }
                circuit.components.extend(built.element);
                continue;
            };
            // Each unit as its own component: its in+, in- and out, then
            // the package's shared supplies.
            for u in 0..count {
                let slot = |i: usize| if i < 3 { 3 * u + i } else { 3 * count + i - 3 };
                if (0..3).all(|i| nodes_of(slot(i)).is_none()) {
                    continue;
                }
                let unit = Component {
                    id: format!("{}.{}", component.id, char::from(b'a' + u as u8)),
                    kind: unit_kind.clone(),
                    part: None,
                    ..component.clone()
                };
                if ids.contains(unit.id.as_str()) {
                    return Err(ImportError::DuplicateId { id: unit.id });
                }
                let connected = |i: usize| nodes_of(slot(i)).is_some();
                if skip(&unit.id, &unit.kind, &connected, &mut warnings) {
                    // The package's other pins may still serve the other
                    // units; only this unit's own nets lose a pin.
                    let own: Vec<String> = (0..3).filter_map(|i| nodes_of(slot(i))).collect();
                    for (n, pin) in pins_of(&component.id) {
                        if own.contains(&n) {
                            unsimulated.entry(n).or_default().push(pin);
                        }
                    }
                    continue;
                }
                let built = build_element(&unit, |i| nodes_of(slot(i)), &supply)?;
                record(&mut warnings, &mut models, &component.id, &built);
                circuit.components.extend(built.element);
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
            circuit.components.push(load.element(nodes)?);
        }

        let high_impedance = tie_high_impedance_nets(&mut circuit, unsimulated, &ids);

        Ok(ImportedCircuit {
            circuit,
            nodes,
            black_boxes,
            models,
            warnings,
            high_impedance,
            skipped,
        })
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

fn slot_of(
    component: &Component,
    terminal: &str,
    options: &ImportOptions,
) -> Result<Slot, ImportError> {
    let kind = &component.kind;
    let names = options.terminal_names.get(&component.id);
    if kind.is_passive() || options.black_box(component)? {
        // The terminal's id: as given, or the one terminal by that name.
        if let Some(names) = names.filter(|n| !n.contains_key(terminal)) {
            let wanted = normalise(terminal);
            let mut ids = names.iter().filter(|(_, n)| normalise(n) == wanted);
            if let (Some((id, _)), None) = (ids.next(), ids.next()) {
                return Ok(Slot::Named(id.clone()));
            }
        }
        return Ok(Slot::Named(terminal.to_string()));
    }
    let index = |given: &str| {
        let wanted = normalise(given);
        kind.terminals()
            .iter()
            .position(|aliases| aliases.iter().any(|a| normalise(a) == wanted))
    };
    // A terminal's name says what it is; its id may be only a pin number.
    let name = names.and_then(|n| n.get(terminal));
    if let Some(slot) = name.and_then(|n| index(n)).or_else(|| index(terminal)) {
        return Ok(Slot::Index(slot));
    }
    Err(ImportError::UnknownTerminal {
        component: component.id.clone(),
        kind: kind.name().to_string(),
        terminal: match name {
            Some(name) => format!("{terminal} ({name})"),
            None => terminal.to_string(),
        },
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
    options: &'a ImportOptions,
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
                let slot = slot_of(component, &pin.terminal, self.options)?;
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
            Value::Quantity(q) if q.value.is_finite() => {
                match unit_of(&self.component.kind, field) {
                    Some(want) if !matches!(q.unit, Unit::Other(_)) && q.unit != want => {
                        Err(self.bad(field, value, &format!("a quantity in {}", want.symbol())))
                    }
                    _ => Ok(q.value),
                }
            }
            _ => Err(self.bad(field, value, "a number")),
        }
    }

    /// A datasheet figure: `name` (typical), `name_typ`, `name_min` and
    /// `name_max`. `None` when none of them is set.
    fn spec(&self, name: &str) -> Result<Option<DatasheetValue>, ImportError> {
        let typ = match self.param(&[name])? {
            Some(v) => Some(v),
            None => self.param(&[&format!("{name}_typ")])?,
        };
        let spec = DatasheetValue {
            min: self.param(&[&format!("{name}_min")])?,
            typ,
            max: self.param(&[&format!("{name}_max")])?,
        };
        Ok((!spec.is_empty()).then_some(spec))
    }

    /// A datasheet fit's error, as the parameter it's about.
    fn datasheet(&self, e: DatasheetError) -> ImportError {
        match e {
            DatasheetError::Missing { figure } => ImportError::MissingValue {
                component: self.component.id.clone(),
                what: format!("datasheet figure \"{figure}\""),
            },
            DatasheetError::Invalid { figure, reason } => ImportError::BadValue {
                component: self.component.id.clone(),
                field: figure.to_string(),
                given: self
                    .component
                    .params
                    .get(figure)
                    .cloned()
                    .unwrap_or(Value::Text(String::new())),
                expected: format!("a figure that fits: it {reason}"),
            },
            _ => ImportError::BadValue {
                component: self.component.id.clone(),
                field: "datasheet".into(),
                given: Value::Text(e.to_string()),
                expected: "figures sindr can fit".into(),
            },
        }
    }

    /// The value as written, when it's text that isn't a number: a part
    /// number, a description.
    fn value_text(&self) -> Option<&str> {
        match &self.component.value {
            Some(Value::Text(t)) if parse_value(t).is_none() => Some(t),
            _ => None,
        }
    }

    /// `" (value \"PMBT3904\" isn't one sindr reads)"`, or nothing.
    fn ignored_value(&self) -> String {
        self.value_text()
            .map(|t| format!(" (value {t:?} isn't one sindr reads)"))
            .unwrap_or_default()
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

    /// The first of `names` that is set, as text.
    fn text(&self, names: &[&str]) -> Result<Option<String>, ImportError> {
        for name in names {
            match self.component.params.get(*name) {
                None => continue,
                Some(Value::Text(t)) => return Ok(Some(t.clone())),
                Some(v) => return Err(self.bad(name, v, "text")),
            }
        }
        Ok(None)
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

/// A component as built: its element (`None` for a passive one), where its
/// model came from, and what to warn about.
#[derive(Default)]
struct Built {
    element: Option<CircuitElement>,
    source: Option<ModelSource>,
    warnings: Vec<String>,
}

/// The canonical names of `kind`'s required terminals that aren't
/// connected, in terminal order.
fn missing_terminals(kind: &ComponentKind, connected: &dyn Fn(usize) -> bool) -> Vec<String> {
    if kind.is_passive() {
        return Vec::new();
    }
    let terminals = kind.terminals();
    let required = terminals.len() - kind.optional_terminals();
    terminals
        .iter()
        .enumerate()
        .take(required)
        .filter(|(i, _)| !connected(*i))
        .map(|(_, names)| names[0].to_string())
        .collect()
}

/// The element for `component`, or `None` for a passive one. `node(i)` is
/// the node of terminal slot `i`, if connected.
fn build_element(
    component: &Component,
    node: impl Fn(usize) -> Option<String>,
    supply: &BTreeMap<String, f64>,
) -> Result<Built, ImportError> {
    use ComponentKind as K;
    let kind = &component.kind;
    if kind.is_passive() {
        return Ok(Built::default());
    }
    for name in component.params.keys() {
        let figure = ["_min", "_typ", "_max"]
            .iter()
            .find_map(|suffix| name.strip_suffix(suffix))
            .filter(|base| kind.figures().contains(base));
        if !kind.params().contains(&name.as_str()) && figure.is_none() {
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

    // Most kinds' model is their value and parameters; the device kinds
    // set `source` (and warn on a default) as they read theirs.
    let mut source = if component.params.is_empty() {
        ModelSource::Value
    } else {
        ModelSource::Params
    };
    let mut warnings = Vec::new();
    let mut defaulted = |what: String| {
        warnings.push(format!("no model data{}; using {what}", r.ignored_value()));
        ModelSource::Default
    };
    let from_model = |given: bool, otherwise: ModelSource| {
        if given {
            ModelSource::Model
        } else {
            otherwise
        }
    };

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
        K::Diode => {
            let model = match model {
                Some(Model::Diode(m)) => {
                    source = ModelSource::Model;
                    Some(*m)
                }
                _ => match diode_datasheet(&r)? {
                    Some(ds) => {
                        source = ModelSource::Datasheet;
                        Some(DiodeModel::from_datasheet(&ds).map_err(|e| r.datasheet(e))?)
                    }
                    None => {
                        source =
                            defaulted("the built-in silicon diode (IS = 1e-14 A, N = 1)".into());
                        None
                    }
                },
            };
            CircuitElement::Diode {
                id,
                nodes: n2(),
                temperature: temperature()?,
                model,
            }
        }
        K::SchottkyDiode => {
            source =
                defaulted("the built-in Schottky diode; sindr reads no model for this kind".into());
            CircuitElement::SchottkyDiode {
                id,
                nodes: n2(),
                temperature: temperature()?,
            }
        }
        K::Photodiode => {
            source =
                defaulted("the built-in photodiode; sindr reads no model for this kind".into());
            CircuitElement::Photodiode {
                id,
                nodes: n2(),
                irradiance: r.param_or("irradiance", 0.0)?,
                temperature: temperature()?,
            }
        }
        K::ZenerDiode => {
            let (vz, from) = match r.spec("vz")? {
                Some(vz) => (vz, ModelSource::Params),
                None => match r.value()? {
                    Some(v) => (DatasheetValue::typical(v), ModelSource::Value),
                    None => {
                        return Err(ImportError::MissingValue {
                            component: component.id.clone(),
                            what: "a breakdown voltage (V), as the value or \"vz\"".into(),
                        })
                    }
                },
            };
            let vz = match r.param(&["iz"])? {
                Some(iz) => {
                    source = ModelSource::Datasheet;
                    let fit = DiodeModel::zener_from_datasheet(&ZenerDatasheet { vz, iz })
                        .map_err(|e| r.datasheet(e))?;
                    fit.bv.unwrap_or_default()
                }
                None => {
                    source = from;
                    vz.require("vz").map_err(|e| r.datasheet(e))?
                }
            };
            CircuitElement::ZenerDiode {
                id,
                nodes: n2(),
                vz,
                temperature: temperature()?,
            }
        }
        K::Led => match diode_datasheet(&r)? {
            // An LED through its own Vf is a diode with that model: the LED
            // element knows only its colour's preset.
            Some(ds) => {
                source = ModelSource::Datasheet;
                CircuitElement::Diode {
                    id,
                    nodes: n2(),
                    temperature: temperature()?,
                    model: Some(DiodeModel::led_from_datasheet(&ds).map_err(|e| r.datasheet(e))?),
                }
            }
            None => {
                let given = r.text(&["color", "colour"])?;
                let color = match &given {
                    Some(c) => led::color_in(c).inspect(|_| source = ModelSource::Params),
                    None => None,
                };
                let color = color.or_else(|| {
                    let c = match &component.value {
                        Some(Value::Text(t)) => led::color_in(t),
                        _ => None,
                    };
                    c.inspect(|_| source = ModelSource::Value)
                });
                let color = match color {
                    Some(c) => c,
                    None => {
                        source = match &given {
                            Some(c) => {
                                warnings.push(format!(
                                    "colour {c:?} isn't one sindr knows; using a red LED (1.8 V at 20 mA)"
                                ));
                                ModelSource::Default
                            }
                            None => defaulted("a red LED (1.8 V at 20 mA)".into()),
                        };
                        "red"
                    }
                };
                CircuitElement::Led {
                    id,
                    nodes: n2(),
                    color: color.to_string(),
                    temperature: temperature()?,
                }
            }
        },
        K::Varactor => {
            source = match model {
                Some(Model::Varactor(_)) => ModelSource::Model,
                _ => defaulted("the built-in varactor".into()),
            };
            CircuitElement::Varactor {
                id,
                nodes: n2(),
                params: match model {
                    Some(Model::Varactor(p)) => p.clone(),
                    _ => VaractorParams::default(),
                },
            }
        }
        K::Npn | K::Pnp => {
            let npn = *kind == K::Npn;
            let gain = r.param(&["bf", "beta"])?;
            let hfe = r.spec("hfe")?;
            let vce_sat = r.spec("vce_sat")?;
            let given = match model {
                Some(Model::Bjt(m)) => Some(*m),
                _ => None,
            };
            let (bf, bjt_model) = if vce_sat.is_some() && given.is_none() {
                let ds = BjtDatasheet {
                    hfe: gain
                        .map(DatasheetValue::typical)
                        .or(hfe)
                        .unwrap_or_default(),
                    hfe_ic: r.param(&["hfe_ic"])?.unwrap_or(0.0),
                    vce_sat,
                    vce_sat_ic: r.param(&["vce_sat_ic"])?,
                    vce_sat_ib: r.param(&["vce_sat_ib"])?,
                };
                let (bf, m) = BjtModel::from_datasheet(&ds).map_err(|e| r.datasheet(e))?;
                source = ModelSource::Datasheet;
                (bf, Some(m))
            } else {
                let hfe = hfe
                    .map(|h| h.require("hfe"))
                    .transpose()
                    .map_err(|e| r.datasheet(e))?;
                let bf = match (gain, hfe) {
                    (Some(bf), _) => {
                        source = from_model(given.is_some(), ModelSource::Params);
                        bf
                    }
                    (None, Some(hfe)) => {
                        source = from_model(given.is_some(), ModelSource::Datasheet);
                        hfe
                    }
                    (None, None) if given.is_some() => {
                        source = ModelSource::Model;
                        warnings.push("the model gives no gain; using bf = 100".into());
                        100.0
                    }
                    (None, None) => {
                        source = defaulted(format!(
                            "the default {} (bf = 100)",
                            if npn { "NPN" } else { "PNP" }
                        ));
                        100.0
                    }
                };
                (bf, given)
            };
            CircuitElement::Bjt {
                id,
                nodes: n3(),
                kind: if npn { BjtKind::Npn } else { BjtKind::Pnp },
                bf,
                temperature: temperature()?,
                parasitic_caps: None,
                model: bjt_model,
            }
        }
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
                    Some(Model::Mosfet(p)) => {
                        source = ModelSource::Model;
                        p.clone()
                    }
                    _ => {
                        let mos_kind = if nmos {
                            MosfetKind::Nmos
                        } else {
                            MosfetKind::Pmos
                        };
                        match mosfet_datasheet(&r)? {
                            Some(ds) => {
                                source = ModelSource::Datasheet;
                                MosfetParams::from_datasheet(mos_kind, &ds)
                                    .map_err(|e| r.datasheet(e))?
                            }
                            None if nmos => {
                                source = defaulted(
                                    "the default NMOS (VTO = 0.7 V, KP = 200 µA/V²)".into(),
                                );
                                MosfetParams::default_nmos()
                            }
                            None => {
                                source = defaulted(
                                    "the default PMOS (VTO = −0.7 V, KP = 100 µA/V²)".into(),
                                );
                                MosfetParams::default_pmos()
                            }
                        }
                    }
                },
                parasitic_caps: None,
                body_diode: None,
            }
        }
        K::Njfet | K::Pjfet => {
            let n = *kind == K::Njfet;
            if component.params.is_empty() {
                source = defaulted(format!(
                    "the default JFET (idss = 10 mA, vp = {} V)",
                    if n { "−2" } else { "2" }
                ));
            }
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
                parasitic_caps: None,
            }
        }
        K::Igbt => {
            source = match model {
                Some(Model::Igbt(_)) => ModelSource::Model,
                _ => defaulted("the built-in IGBT".into()),
            };
            CircuitElement::Igbt {
                id,
                nodes: n3(),
                params: match model {
                    Some(Model::Igbt(p)) => p.clone(),
                    _ => IgbtParams::default(),
                },
            }
        }
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
            source = match (kind, model) {
                (K::OpAmp, Some(Model::OpAmp(_))) => ModelSource::Model,
                (K::OpAmp, _) => defaulted(
                    "an ideal op-amp (gain 1e5, no bandwidth or slew limit, rails not enforced)"
                        .into(),
                ),
                _ => defaulted("the built-in comparator (gain 1e5, clamped to its rails)".into()),
            };
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
            saturation: None,
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
                Some(Model::Regulator(p)) => {
                    source = ModelSource::Model;
                    Some(*p)
                }
                _ => {
                    source = defaulted("an ideal regulator (no dropout, no current limit)".into());
                    None
                }
            },
        },
        K::Crystal => CircuitElement::Crystal {
            id,
            nodes: n2(),
            params: CrystalParams::from_datasheet(
                match r.param(&["frequency"])? {
                    Some(f) => f,
                    None => r.value()?.ok_or_else(|| ImportError::MissingValue {
                        component: component.id.clone(),
                        what: "a frequency (Hz), as the value or \"frequency\"".into(),
                    })?,
                },
                r.param_or("esr", 50.0)?,
                r.param_or("c0", 5e-12)?,
            ),
        },
        K::FerriteBead => CircuitElement::FerriteBead {
            id,
            nodes: n2(),
            params: FerriteBeadParams::from_datasheet(
                match r.param(&["z_100mhz"])? {
                    Some(z) => z,
                    None => r.value()?.ok_or_else(|| ImportError::MissingValue {
                        component: component.id.clone(),
                        what: "an impedance at 100 MHz (Ω), as the value or \"z_100mhz\"".into(),
                    })?,
                },
                r.param_or("r_dc", 0.1)?,
            ),
        },
        K::Battery => CircuitElement::Battery {
            id,
            nodes: n2(),
            params: match model {
                Some(Model::Battery(p)) => {
                    source = ModelSource::Model;
                    p.clone()
                }
                _ => BatteryParams::constant(
                    r.required_value("a voltage (V), or a battery model")?,
                    r.param_or("capacity", 1.0)?,
                    r.param_or("r_series", 0.1)?,
                ),
            },
            state_of_charge: r.param_or("state_of_charge", 1.0)?,
        },
        K::Thermistor => {
            source = defaulted(
                "the built-in 10 kΩ NTC (β = 3950); sindr reads no model for this kind".into(),
            );
            CircuitElement::Thermistor {
                id,
                nodes: n2(),
                temperature: r.param_or("temperature", 298.15)?,
            }
        }
        K::Photoresistor => {
            source =
                defaulted("the built-in photoresistor; sindr reads no model for this kind".into());
            CircuitElement::Photoresistor {
                id,
                nodes: n2(),
                light_level: r.param_or("light_level", 0.0)?,
            }
        }
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
        K::Connector | K::TestPoint | K::NetTie | K::MountingHole | K::BlackBox | K::Other(_) => {
            return Ok(Built::default())
        }
        // A multi-unit package is built unit by unit, as its unit kind.
        K::DualOpAmp | K::QuadOpAmp | K::DualComparator | K::QuadComparator => {
            return Ok(Built::default())
        }
    };
    Ok(Built {
        element: Some(element),
        source: Some(source),
        warnings,
    })
}

/// A diode's or LED's forward figures, `vf` at `if` (and `vf2` at `if2`),
/// if `vf` is given.
fn diode_datasheet(r: &Reader<'_>) -> Result<Option<DiodeDatasheet>, ImportError> {
    let Some(vf) = r.spec("vf")? else {
        return Ok(None);
    };
    let missing = |what: &str| ImportError::MissingValue {
        component: r.component.id.clone(),
        what: what.to_string(),
    };
    Ok(Some(DiodeDatasheet {
        vf,
        if_: r
            .param(&["if"])?
            .ok_or_else(|| missing("the current vf is given at (A, parameter \"if\")"))?,
        vf2: r.spec("vf2")?,
        if2: r.param(&["if2"])?,
    }))
}

/// A MOSFET's threshold and on-resistance figures, if either is given.
fn mosfet_datasheet(r: &Reader<'_>) -> Result<Option<MosfetDatasheet>, ImportError> {
    let vth = r.spec("vth")?;
    let rds_on = r.spec("rds_on")?;
    if vth.is_none() && rds_on.is_none() {
        return Ok(None);
    }
    let need = |name: &str, what: &str| -> Result<f64, ImportError> {
        r.param(&[name])?.ok_or_else(|| ImportError::MissingValue {
            component: r.component.id.clone(),
            what: format!("{what} (parameter \"{name}\")"),
        })
    };
    let missing = |name: &str| ImportError::MissingValue {
        component: r.component.id.clone(),
        what: format!("datasheet figure \"{name}\""),
    };
    Ok(Some(MosfetDatasheet {
        vth: vth.ok_or_else(|| missing("vth"))?,
        vth_id: need("vth_id", "the drain current vth is given at (A)")?,
        rds_on: rds_on.ok_or_else(|| missing("rds_on"))?,
        rds_on_vgs: need("rds_on_vgs", "the gate voltage rds_on is given at (V)")?,
        rds_on_id: r.param(&["rds_on_id"])?,
    }))
}

/// The unit a value or parameter is in, for checking a [`Quantity`]'s.
/// `None` where sindr doesn't check (a capacity in Ah, an irradiance).
fn unit_of(kind: &ComponentKind, field: &str) -> Option<Unit> {
    use ComponentKind as K;
    let base = ["_min", "_typ", "_max"]
        .iter()
        .find_map(|s| field.strip_suffix(s))
        .unwrap_or(field);
    Some(match base {
        "value" => match kind {
            K::Resistor | K::Potentiometer | K::FerriteBead => Unit::Ohm,
            K::Capacitor | K::CapacitorPolarized => Unit::Farad,
            K::Inductor => Unit::Henry,
            K::ZenerDiode | K::Regulator | K::Battery | K::VoltageSource => Unit::Volt,
            K::Fuse | K::CurrentSource => Unit::Ampere,
            K::Crystal => Unit::Hertz,
            _ => return None,
        },
        "temperature" => Unit::Kelvin,
        "vz" | "vf" | "vf2" | "vce_sat" | "vth" | "rds_on_vgs" | "vp" | "v_pos" | "v_neg"
        | "pickup_voltage" | "voltage" => Unit::Volt,
        "iz" | "if" | "if2" | "hfe_ic" | "vce_sat_ic" | "vce_sat_ib" | "vth_id" | "rds_on_id"
        | "idss" => Unit::Ampere,
        "rds_on" | "coil_resistance" | "esr" | "z_100mhz" | "r_dc" | "r_series" => Unit::Ohm,
        "bf" | "beta" | "hfe" | "position" | "k" | "state_of_charge" | "light_level" => Unit::Ratio,
        "inductance" | "l1" | "l2" => Unit::Henry,
        "frequency" => Unit::Hertz,
        "c0" => Unit::Farad,
        _ => return None,
    })
}

#[cfg(test)]
mod tests;
