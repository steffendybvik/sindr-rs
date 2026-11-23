//! The circuit elements: one struct per [`CircuitElement`] variant.
//!
//! [`CircuitElement`] wraps each of them, so a resistor is
//! `CircuitElement::Resistor(Resistor { .. })`. The structs carry the
//! element's fields and serialise as the enum's variant did, inside the
//! enum's `type`-tagged object.
//!
//! [`CircuitElement`]: crate::CircuitElement

use sindr_devices::bjt::BjtKind;
use sindr_devices::jfet::JfetKind;
use sindr_devices::mosfet::{MosfetKind, MosfetParams};

use crate::circuit::{
    BehaviouralOutput, BjtModel, BjtParasiticCaps, CrystalParams, DiodeModel, FerriteBeadParams,
    JfetParasiticCaps, MosfetParasiticCaps, OpAmpModel, SpdtPosition,
};
use crate::expression::Expression;
use crate::waveform::Waveform;

/// Resistor between two nodes.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Resistor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Resistance (Ω).
    pub resistance: f64,
}

/// Independent voltage source. `nodes[0]` is the positive terminal,
/// `nodes[1]` is the negative terminal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct VoltageSource {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[positive, negative]`.
    pub nodes: [String; 2],
    /// DC voltage (V); also the offset added to any `waveform`.
    pub voltage: f64,
    /// Optional time-varying waveform added on top of `voltage`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub waveform: Option<Waveform>,
}

/// Independent current source. Current flows through it from `nodes[0]`
/// to `nodes[1]`: drawn out of `nodes[0]` and pushed into `nodes[1]`, as
/// SPICE's `I`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct CurrentSource {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[from, to]`; current flows from `nodes[0]` to `nodes[1]`.
    pub nodes: [String; 2],
    /// DC current (A); also the offset added to any `waveform`.
    pub current: f64,
    /// Optional time-varying waveform added on top of `current`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub waveform: Option<Waveform>,
}

/// Switch between two nodes. Modeled as a resistor with very low (closed)
/// or very high (open) resistance for DC analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Switch {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// When true the switch conducts (closed); when false it is open.
    pub closed: bool,
}

/// Capacitor. Open-circuit in DC analysis; charges/discharges in
/// transient. `voltage_across` is reported as `nodes[0] − nodes[1]`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Capacitor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Capacitance (F).
    pub capacitance: f64,
}

/// A battery: open-circuit voltage by state of charge behind a series
/// resistance and an optional RC pair ([`BatteryParams`]). At DC it is
/// its open-circuit voltage at `state_of_charge` behind `R0 + R1`. In
/// transient the RC pair charges and the state of charge falls with the
/// charge drawn, reported per step in `state_of_charge`. In AC it is
/// the impedance `R0 + R1‖C1`. `current_through` flows
/// `nodes[0] → nodes[1]` inside the cell, so it is negative while the
/// battery discharges, as for a voltage source.
///
/// [`BatteryParams`]: sindr_devices::battery::BatteryParams
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Battery {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[positive, negative]`.
    pub nodes: [String; 2],
    /// The cell's equivalent circuit.
    pub params: sindr_devices::battery::BatteryParams,
    /// State of charge at the start, 0 (empty) to 1 (full).
    pub state_of_charge: f64,
}

/// Inductor. Stamped as **open-circuit in pure DC analysis** (sindr
/// solves DC as a single algebraic step, not as t→∞ of a transient);
/// integrates `dI/dt` properly during transient. The DC bias point that
/// AC, `.tf` and `.sens` linearise around shorts it instead. `voltage_across` is
/// reported as `nodes[0] − nodes[1]`, `current_through` flows
/// `nodes[0] → nodes[1]`. To get steady-state current, run a
/// transient long enough to settle.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Inductor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`; `current_through` flows `nodes[0] → nodes[1]`.
    pub nodes: [String; 2],
    /// Inductance (H): the small-signal value, at zero current.
    pub inductance: f64,
    /// Core saturation: the inductance falls towards `l_sat` as the
    /// current passes `i_sat`. `None` is a linear inductor.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub saturation: Option<sindr_devices::inductor::InductorSaturation>,
}

/// Diode (Shockley model, silicon unless `model` says otherwise).
/// `nodes: [anode, cathode]`. Forward current flows from anode → cathode;
/// positive `voltage_across` corresponds to forward bias.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Diode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
    pub temperature: f64,
    /// Model parameters (IS, N, RS). `None` is the built-in silicon diode,
    /// the same as `Some(DiodeModel::default())`.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<DiodeModel>,
}

/// LED. `nodes: [anode, cathode]`. Forward voltage drop depends on
/// `color` (e.g. `"red"` ≈ 1.8 V, `"blue"` ≈ 3.2 V).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Led {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Emission colour, any case: `"red"`, `"orange"`, `"amber"`,
    /// `"yellow"`, `"green"`, `"blue"` or `"white"`
    /// ([`led::COLORS`](sindr_devices::led::COLORS)); sets the forward
    /// voltage drop. Any other name is red.
    pub color: String,
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
    pub temperature: f64,
}

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
    #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
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

/// MOSFET transistor (3 terminals: gate, drain, source; the body is tied
/// to the source).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

/// Voltage-Controlled Voltage Source: V_out = mu * V_control.
/// nodes: [out+, out-], control_nodes: [ctrl+, ctrl-]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Vcvs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out+, out-]`.
    pub nodes: [String; 2],
    /// Sensing terminals `[ctrl+, ctrl-]`.
    pub control_nodes: [String; 2],
    /// Voltage gain μ (V/V): `V_out = gain * V_control`.
    pub gain: f64, // mu
}

/// Voltage-Controlled Current Source: I_out = gm * V_control.
/// nodes: [out_from, out_to], control_nodes: [ctrl+, ctrl-]
///
/// Like [`CircuitElement::CurrentSource`] and SPICE's `G n+ n-`, a
/// positive `I_out` flows through the source from `nodes[0]` to
/// `nodes[1]`: it is drawn out of `nodes[0]` and pushed into `nodes[1]`.
///
/// [`CircuitElement::CurrentSource`]: crate::CircuitElement::CurrentSource
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Vccs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out_from, out_to]`.
    pub nodes: [String; 2],
    /// Sensing terminals `[ctrl+, ctrl-]`.
    pub control_nodes: [String; 2],
    /// Transconductance gm (A/V): `I_out = gm * V_control`.
    pub gm: f64,
}

/// Current-Controlled Voltage Source: V_out = rm * I_control.
/// nodes: [out+, out-], control_source: id of controlling voltage source
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Ccvs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out+, out-]`.
    pub nodes: [String; 2],
    /// Id of the voltage source whose current is sensed.
    pub control_source: String,
    /// Transresistance rm (Ω): `V_out = rm * I_control`.
    pub rm: f64,
}

/// Current-Controlled Current Source: I_out = alpha * I_control.
/// nodes: [out_from, out_to], control_source: id of controlling voltage source
///
/// Like [`CircuitElement::CurrentSource`] and SPICE's `F n+ n-`, a
/// positive `I_out` flows through the source from `nodes[0]` to
/// `nodes[1]`. `I_control` is the controlling source's branch current,
/// positive from its `+` terminal through it to `-`, as in SPICE.
///
/// [`CircuitElement::CurrentSource`]: crate::CircuitElement::CurrentSource
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Cccs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out_from, out_to]`.
    pub nodes: [String; 2],
    /// Id of the voltage source whose current is sensed.
    pub control_source: String,
    /// Current gain α (A/A): `I_out = alpha * I_control`.
    pub alpha: f64,
}

/// Pushbutton (momentary switch): identical behaviour to Switch.
/// When closed = true, current flows; when false, open circuit.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Pushbutton {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// When true the button conducts (pressed); when false it is open.
    pub closed: bool,
}

/// Relay: 4-terminal component with a coil and a contact pair.
/// nodes: [coil+, coil-, contact1, contact2]
/// Contact closes when |V_coil| >= pickup_voltage. AC and `.tf` take the
/// contact as open or closed at the DC bias point.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Relay {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[coil+, coil-, contact1, contact2]`.
    pub nodes: [String; 4],
    /// Coil resistance (Ω). Default 500.
    #[cfg_attr(feature = "serde", serde(default = "default_relay_coil_resistance"))]
    pub coil_resistance: f64,
    /// Coil voltage magnitude at which the contact closes (V).
    pub pickup_voltage: f64,
    /// Coil inductance in Henry, in series with `coil_resistance` in
    /// transient. 0.0 = purely resistive coil.
    #[cfg_attr(feature = "serde", serde(default))]
    pub inductance: f64,
}

/// Photoresistor (LDR): resistance varies logarithmically with light.
/// light_level = 0.0 → ~1 MΩ (dark), 1.0 → ~1 kΩ (bright).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Photoresistor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Incident light level, 0.0 (dark, ~1 MΩ) to 1.0 (bright, ~1 kΩ).
    pub light_level: f64,
}

/// Potentiometer: 3-terminal voltage divider.
/// nodes: [top, wiper, bottom]
/// position = 0.0 → wiper at top, 1.0 → wiper at bottom.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Potentiometer {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[top, wiper, bottom]`.
    pub nodes: [String; 3],
    /// End-to-end resistance (Ω).
    pub resistance: f64,
    /// Wiper position, 0.0 (at top) to 1.0 (at bottom).
    pub position: f64,
}

/// Zener diode between two nodes: a silicon diode with SPICE reverse
/// breakdown, `BV = vz` at `IBV` = 5 mA and 1 Ω of series resistance
/// (the card `D(IS=1e-14 N=1 RS=1 BV=vz IBV=5m)`), so it holds about
/// `vz` at a small zener's 5 mA test current and rises 1 V per amp past
/// it. For another knee, use a [`CircuitElement::Diode`] with a
/// [`DiodeModel`] setting `bv` and `ibv`.
/// nodes: [anode, cathode]
///
/// [`CircuitElement::Diode`]: crate::CircuitElement::Diode
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct ZenerDiode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Reverse breakdown (Zener) voltage (V).
    pub vz: f64,
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
    pub temperature: f64,
}

/// Op-amp, output referred to ground.
///
/// With `model: None` it is the near-ideal VCVS sindr has always used:
/// gain 1e5, no frequency response, no output resistance, and `v_pos` /
/// `v_neg` not enforced. With an [`OpAmpModel`] it gets a dominant pole
/// from the gain-bandwidth product, slew-rate limiting, output
/// resistance, input offset and bias current, and an output clamped to
/// `v_pos`/`v_neg`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct OpAmp {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[in_plus, in_minus, out]`.
    pub nodes: [String; 3], // [in_plus, in_minus, out]
    /// Positive supply rail (V): the output's upper limit with a
    /// `model`. Default 15.
    #[cfg_attr(feature = "serde", serde(default = "default_v_pos"))]
    pub v_pos: f64,
    /// Negative supply rail (V): the output's lower limit with a
    /// `model`. Default -15.
    #[cfg_attr(feature = "serde", serde(default = "default_v_neg"))]
    pub v_neg: f64,
    /// Macromodel parameters; `None` for the near-ideal VCVS.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<OpAmpModel>,
}

/// Comparator, output referred to ground: gain 1e5 from `v₊ − v₋` with
/// no frequency response, its output clamped to `[v_neg, v_pos]`. It is
/// an op-amp with an [`OpAmpModel`] of that gain and no pole, so it makes
/// the circuit nonlinear (Newton). No hysteresis, no open-collector output.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Comparator {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[in_plus, in_minus, out]`.
    pub nodes: [String; 3], // [in_plus, in_minus, out]
    /// Output high level (V): the output's upper limit. Default 15.
    #[cfg_attr(feature = "serde", serde(default = "default_v_pos"))]
    pub v_pos: f64,
    /// Output low level (V): the output's lower limit, below
    /// `v_pos`. Default -15.
    #[cfg_attr(feature = "serde", serde(default = "default_v_neg"))]
    pub v_neg: f64,
}

/// Schottky diode: lower forward voltage (~0.3V) than silicon diode.
/// nodes: [anode, cathode]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct SchottkyDiode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
    pub temperature: f64,
}

/// NTC thermistor: resistance varies with temperature (passive, no NR).
/// nodes: [n1, n2]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Thermistor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[n1, n2]`.
    pub nodes: [String; 2],
    /// Temperature in Kelvin, positive and finite. Default: 298.15 K (25°C).
    #[cfg_attr(feature = "serde", serde(default = "default_temperature"))]
    pub temperature: f64,
}

/// Photodiode: diode + photocurrent source driven by irradiance.
/// nodes: [anode, cathode]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Photodiode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Incident irradiance in W (optical power, not W/m²).
    /// 0.0 = dark, 0.1 = ~50mA photocurrent with default responsivity.
    #[cfg_attr(feature = "serde", serde(default))]
    pub irradiance: f64,
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
    pub temperature: f64,
}

/// Varactor diode: voltage-dependent junction capacitance.
/// Open circuit in DC (a 1 TΩ leak); voltage-dependent capacitor in
/// transient; in AC, its capacitance at the DC bias point.
/// nodes: [anode, cathode]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Varactor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Model parameters (zero-bias capacitance, grading, etc.).
    #[cfg_attr(feature = "serde", serde(default))]
    pub params: sindr_devices::varactor::VaractorParams,
}

/// IGBT: MOSFET gate control + BJT output characteristics.
/// Nonlinear element — requires Newton-Raphson iteration.
/// nodes: [gate, collector, emitter]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Igbt {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[gate, collector, emitter]`.
    pub nodes: [String; 3],
    /// Model parameters (threshold voltage, transconductance, etc.).
    #[cfg_attr(feature = "serde", serde(default))]
    pub params: sindr_devices::igbt::IgbtParams,
}

/// Ideal transformer with coupled inductors.
///
/// nodes: [p1, q1, p2, q2] where:
///   - [p1, q1]: primary winding (+ and - terminals)
///   - [p2, q2]: secondary winding (+ and - terminals)
///
/// Coupling: M = k * sqrt(L1 * L2) where k ∈ [0, 1).
/// k=1 is ideal (no leakage), but mathematically singular — use k<=0.999.
///
/// In DC analysis: both windings stamped as short circuits; the bias point
/// for AC, `.tf` and `.sens` shorts each with a 0 V source.
/// In AC: the coupled pair `V = jω·[[L1, M], [M, L2]]·I`.
/// In transient: coupled-inductor companion with 2 branch current unknowns,
/// each winding `V = L·dI/dt` plus the mutual term. The windings start at
/// 0 A; an initial current on a transformer is rejected.
///
/// With `saturation`, the core is modelled as a leakage inductance
/// `(1 − k)·L` on each winding and a magnetising inductance `k·L1` on the
/// primary carrying the shared core flux, driven by the magnetising
/// current `i1 + n·i2` (n = √(L2/L1), the turns ratio). That branch
/// saturates as a saturating [`Inductor`] does, so a
/// winding driven past its volt-second rating, or with a DC bias, draws
/// a magnetising current that climbs once the core saturates. Without
/// it the pair is linear and identical to the coupled inductors above.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Transformer {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[p1, q1, p2, q2]`: primary `[p1, q1]`, secondary `[p2, q2]`.
    pub nodes: [String; 4], // [p1, q1, p2, q2]
    /// Primary winding inductance (H).
    pub l1: f64, // Primary inductance (H)
    /// Secondary winding inductance (H).
    pub l2: f64, // Secondary inductance (H)
    /// Coupling coefficient (unitless) in `[0, 0.999]`. Default 0.999 (near-ideal).
    #[cfg_attr(feature = "serde", serde(default = "default_coupling"))]
    pub k: f64, // Coupling coefficient [0, 0.999]. Default 0.999 (near-ideal).
    /// Core saturation, on the magnetising branch: `i_sat` is the
    /// magnetising current referred to the primary, and `l_sat` at
    /// most the magnetising inductance `k·l1`. `None` is a linear core.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub saturation: Option<sindr_devices::inductor::InductorSaturation>,
}

/// Fuse between two nodes.
///
/// Intact (blown=false): stamped as 0.001 Ω (1 mΩ) — negligible voltage drop.
/// Blown (blown=true): stamped as 1e9 Ω — effectively open circuit.
///
/// NOTE: Do NOT use 0 Ω — that would cause a singular MNA matrix.
/// `rating` is stored for display only: the fuse never blows by itself;
/// set `blown` to open it. To flag an overcurrent, put
/// [`Ratings::current`](crate::Ratings::current) on its part.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Fuse {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// Current rating (A), for display only: the fuse never blows by itself.
    #[cfg_attr(feature = "serde", serde(default = "default_fuse_rating"))]
    pub rating: f64,
    /// When true, fuse is blown (open circuit). User-settable, like switch.closed.
    #[cfg_attr(feature = "serde", serde(default))]
    pub blown: bool,
}

/// Linear voltage regulator: holds the output at `voltage` above its
/// ground pin while it has headroom, and draws its output current plus
/// its quiescent current from the input.
///
/// Piecewise linear (see `sindr_devices::regulator`): out of headroom
/// the output follows the input down by the dropout; past the current
/// limit it holds the current. `model` sets dropout, quiescent current,
/// current limit and line and load regulation;
/// `RegulatorParams::lm7805()` and `::ams1117()` are datasheet presets.
/// Its result is the pass element: `Vin − Vout`, the output current,
/// and the power it dissipates.
///
/// `nodes: [input, output, gnd]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct VoltageRegulator {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[input, output, gnd]`.
    pub nodes: [String; 3], // [input, output, gnd]
    /// Regulated output voltage (V) from the ground pin. E.g. 5.0 for a
    /// 7805, or −5.0 for a 7905, whose input sits below ground and whose
    /// output current flows in at the output pin.
    pub voltage: f64,
    /// Dropout, quiescent current, limit and regulation. `None` is an
    /// ideal regulator that still can't put out more than it gets in.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<sindr_devices::regulator::RegulatorParams>,
}

/// Single-pole double-throw switch: `nodes: [common, a, b]`. The common
/// terminal connects to `a` or `b` as [`Switch`] does
/// (0.01 Ω), and the other throw is open (1 GΩ). Its result is the path
/// in use: `voltage_across` from common to the selected throw and the
/// current from common into it.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct SwitchSpdt {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[common, a, b]`.
    pub nodes: [String; 3],
    /// Which throw the common terminal is switched to.
    #[cfg_attr(feature = "serde", serde(default))]
    pub position: SpdtPosition,
}

/// Quartz crystal: the Butterworth–Van Dyke model ([`CrystalParams`]).
/// An open circuit at DC. Its result's `power` is what the motional
/// resistance dissipates: the drive level a datasheet limits.
/// `nodes: [a, b]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Crystal {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// The equivalent circuit.
    pub params: CrystalParams,
}

/// Ferrite bead ([`FerriteBeadParams`]): its DC resistance at DC,
/// lossy and inductive at high frequency. Its result's `power` is what
/// its two resistances dissipate. `nodes: [a, b]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct FerriteBead {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// The equivalent circuit.
    pub params: FerriteBeadParams,
}

/// Behavioural source, SPICE's `B n+ n- V={expr}` or `I={expr}`: a
/// voltage or current set by an [`Expression`] of node voltages,
/// branch currents and time.
///
/// A voltage output holds `V(nodes[0]) − V(nodes[1])` at the
/// expression, and like a [`CircuitElement::VoltageSource`] has a
/// branch current others can read with `i(id)`. A current output
/// drives the expression's value through the source from `nodes[0]` to
/// `nodes[1]`, as a [`CircuitElement::CurrentSource`] does. Newton
/// stamps it with the expression's exact gradient, AC analysis
/// linearises it at the bias point, and `time` reads the transient's
/// time (0 elsewhere).
///
/// ```
/// use sindr::{BehaviouralOutput, CircuitElement};
/// use sindr::expression::Expression;
/// use sindr::element::Behavioural;
///
/// let multiplier = CircuitElement::Behavioural(Behavioural {
///     id: "b1".into(),
///     nodes: ["out".into(), "0".into()],
///     output: BehaviouralOutput::Voltage,
///     expr: Expression::parse("v(a)*v(b)")?,
/// });
/// # Ok::<(), sindr::expression::ExpressionError>(())
/// ```
///
/// [`CircuitElement::VoltageSource`]: crate::CircuitElement::VoltageSource
/// [`CircuitElement::CurrentSource`]: crate::CircuitElement::CurrentSource
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Behavioural {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[n+, n-]`.
    pub nodes: [String; 2],
    /// Whether the expression is a voltage or a current.
    pub output: BehaviouralOutput,
    /// The value, in volts or amperes.
    pub expr: Expression,
}

#[cfg(feature = "serde")]
fn default_bf() -> f64 {
    100.0
}

#[cfg(feature = "serde")]
fn default_temperature() -> f64 {
    298.15
}

/// Default junction temperature for IS-bearing devices (K).
/// 300.15 K = 27°C, the SPICE reference temperature for IS.
#[cfg(feature = "serde")]
fn default_junction_temperature() -> f64 {
    300.15
}

#[cfg(feature = "serde")]
fn default_relay_coil_resistance() -> f64 {
    500.0
}

#[cfg(feature = "serde")]
fn default_v_pos() -> f64 {
    15.0
}

#[cfg(feature = "serde")]
fn default_v_neg() -> f64 {
    -15.0
}

#[cfg(feature = "serde")]
fn default_coupling() -> f64 {
    0.999
}

#[cfg(feature = "serde")]
fn default_fuse_rating() -> f64 {
    1.0
}

#[cfg(feature = "serde")]
fn default_jfet_idss() -> f64 {
    10e-3 // 10 mA typical
}

#[cfg(feature = "serde")]
fn default_jfet_vp() -> f64 {
    -2.0 // -2V pinch-off (N-channel default)
}
