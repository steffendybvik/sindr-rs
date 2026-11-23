//! Circuit input format.
//!
//! [`Circuit`] is the root struct sindr's analyses take: a list of
//! [`CircuitElement`]s plus the name of the ground node, and optionally a
//! design hierarchy (see [`crate::design`]). Components share a node by
//! referencing the same node-name string; there are no separate "wire" or
//! "net" objects.
//!
//! [`CircuitElement`] is the open-ended enum of supported component types
//! (resistors, sources, semiconductors, magnetics, …). Each variant wraps a
//! struct from [`crate::element`] carrying its parameters and a fixed-size
//! `nodes` array; sign conventions are documented per struct.

use std::collections::BTreeMap;

use crate::design::{Block, Instance, Part};
use crate::element::{
    Battery, Behavioural, Bjt, Capacitor, Cccs, Ccvs, Comparator, Crystal, CurrentSource, Diode,
    FerriteBead, Fuse, Igbt, Inductor, Jfet, Led, Mosfet, OpAmp, Photodiode, Photoresistor,
    Potentiometer, Pushbutton, Relay, Resistor, SchottkyDiode, Switch, SwitchSpdt, Thermistor,
    Transformer, Varactor, Vccs, Vcvs, VoltageRegulator, VoltageSource, ZenerDiode,
};
use crate::expression::Expression;

/// BJT parasitic capacitances. All values in Farads. Default 0.0 disables each cap.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BjtParasiticCaps {
    /// Base-Emitter junction capacitance (F). Typical: 5–50 pF.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cbe: f64,
    /// Base-Collector junction capacitance (F). Typical: 1–10 pF.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cbc: f64,
}

/// Parameters of a [`CircuitElement::Diode`], as a SPICE `.model D` card
/// gives them: Shockley (IS, N, RS), charge storage (CJO, VJ, M, FC, TT) and
/// reverse breakdown (BV, IBV). Values are at the nominal 27 °C; the
/// element's `temperature` scales `is` from there with `eg` and `xti`, as
/// SPICE's `IS(T)` does, and only `is`.
///
/// The default is sindr's built-in silicon diode. With serde, a missing field
/// takes its default, so `{"rs": 0.5}` is a silicon diode with 0.5 Ω in series.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiodeModel {
    /// Saturation current IS (A), positive. Default 1e-14.
    pub is: f64,
    /// Emission coefficient N, positive. Default 1.
    pub n: f64,
    /// Series resistance RS (Ω), zero or positive. Default 0.
    pub rs: f64,
    /// Zero-bias depletion capacitance CJO (F), zero or positive. Default 0.
    pub cjo: f64,
    /// Junction potential VJ (V), positive. Default 1.
    pub vj: f64,
    /// Grading coefficient M, in `[0, 0.9]`. Default 0.5.
    pub m: f64,
    /// Forward-bias depletion capacitance coefficient FC, in `[0, 0.95]`.
    /// Default 0.5.
    pub fc: f64,
    /// Transit time TT (s), zero or positive: the diffusion charge is
    /// `TT·I_d`, which sets reverse recovery. Default 0.
    pub tt: f64,
    /// Reverse breakdown voltage BV (V), positive. Default `None`: no
    /// breakdown, as in SPICE without `BV`.
    pub bv: Option<f64>,
    /// Current at `−BV`, IBV (A), positive. Default 1 mA.
    pub ibv: f64,
    /// Flicker-noise coefficient KF, zero or positive: noise analysis puts
    /// `KF·|Id|^AF / f` (A²/Hz) across the junction. Default 0.
    pub kf: f64,
    /// Flicker-noise exponent AF, positive. Default 1.
    pub af: f64,
    /// Bandgap EG (eV), positive: how fast `is` rises with temperature.
    /// Default 1.11, silicon; a Schottky junction is about 0.69.
    pub eg: f64,
    /// Saturation-current temperature exponent XTI, zero or positive.
    /// Default 3, a pn junction; a Schottky junction is 2.
    pub xti: f64,
}

impl Default for DiodeModel {
    fn default() -> Self {
        Self {
            is: 1e-14,
            n: 1.0,
            rs: 0.0,
            cjo: 0.0,
            vj: 1.0,
            m: 0.5,
            fc: 0.5,
            tt: 0.0,
            bv: None,
            ibv: 1e-3,
            kf: 0.0,
            af: 1.0,
            eg: 1.11,
            xti: 3.0,
        }
    }
}

impl DiodeModel {
    /// Whether the diode stores charge (CJO or TT set), which makes the
    /// circuit reactive.
    pub fn stores_charge(&self) -> bool {
        self.cjo > 0.0 || self.tt > 0.0
    }
}

/// Ebers–Moll parameters of a [`CircuitElement::Bjt`], as a SPICE
/// `.model NPN`/`PNP` card gives them. The forward gain is the element's own
/// `bf`. Values are at the nominal 27 °C; the element's `temperature` scales
/// `is` from there with `eg` and `xti`, as SPICE's `IS(T)` does.
///
/// The default is sindr's built-in transistor. With serde, a missing field
/// takes its default.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BjtModel {
    /// Transport saturation current IS (A), positive. Default 1e-14.
    pub is: f64,
    /// Reverse current gain BR, positive. Default 1.
    pub br: f64,
    /// Forward emission coefficient NF, positive. Default 1.
    pub nf: f64,
    /// Reverse emission coefficient NR, positive. Default 1.
    pub nr: f64,
    /// Forward Early voltage VAF (V), zero or positive; 0 turns the Early
    /// effect off. Default 0. sindr applies it as `ic·(1 + vce/VAF)`, not
    /// through SPICE's Gummel–Poon base charge, so results differ slightly
    /// from SPICE's when it is set.
    pub vaf: f64,
    /// Flicker-noise coefficient KF, zero or positive: noise analysis puts
    /// `KF·|Ib|^AF / f` (A²/Hz) across base–emitter. Default 0.
    pub kf: f64,
    /// Flicker-noise exponent AF, positive. Default 1.
    pub af: f64,
    /// Bandgap EG (eV), positive: how fast `is` rises with temperature.
    /// Default 1.11, silicon.
    pub eg: f64,
    /// Saturation-current temperature exponent XTI, zero or positive.
    /// Default 3.
    pub xti: f64,
}

impl Default for BjtModel {
    fn default() -> Self {
        Self {
            is: 1e-14,
            br: 1.0,
            nf: 1.0,
            nr: 1.0,
            vaf: 0.0,
            kf: 0.0,
            af: 1.0,
            eg: 1.11,
            xti: 3.0,
        }
    }
}

/// MOSFET gate parasitic capacitances. All values in Farads.
/// Op-amp macromodel parameters, for [`CircuitElement::OpAmp`]'s `model`.
///
/// The gain stage is a first-order lag towards `gain · (v₊ − v₋ + offset)`,
/// clamped to the rails, with time constant `gain / (2π · gbw)`: open-loop
/// gain `gain / (1 + s·τ)`, so a non-inverting stage of gain G has its
/// −3 dB point near `gbw / G`. Its rate is limited to `slew_rate`. The
/// output is that stage's voltage behind `r_out`.
///
/// The pole and the slew limit act in transient and AC analyses. At DC the
/// stage sits at its clamped target. The op-amp's state doesn't count as
/// reactive, so [`solve_circuit`]-style automatic routing still picks DC
/// for a circuit whose only dynamics are op-amps; run a transient
/// explicitly to see them. Like capacitors, the stage starts from rest
/// (0 V) at t = 0.
///
/// [`solve_circuit`]: https://docs.rs/sindr/latest/sindr/fn.solve_circuit.html
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpAmpModel {
    /// Open-loop DC gain (V/V), positive. Default 1e5 (100 dB).
    pub gain: f64,
    /// Gain-bandwidth product (Hz), positive; `None` for no pole. Default
    /// 1 MHz.
    pub gbw: Option<f64>,
    /// Slew rate (V/s), positive; `None` for no limit. Needs `gbw`.
    /// Default 0.5 V/µs.
    pub slew_rate: Option<f64>,
    /// Output resistance (Ω), zero or positive. Default 0.
    pub r_out: f64,
    /// Input offset voltage (V), added to `v₊ − v₋`. Default 0.
    pub offset_voltage: f64,
    /// Input bias current (A) drawn by each input, to ground. Default 0.
    pub bias_current: f64,
}

impl Default for OpAmpModel {
    fn default() -> Self {
        Self {
            gain: 1e5,
            gbw: Some(1e6),
            slew_rate: Some(0.5e6),
            r_out: 0.0,
            offset_voltage: 0.0,
            bias_current: 0.0,
        }
    }
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
/// Optional gate capacitances of a MOSFET, for transient and AC.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MosfetParasiticCaps {
    /// Gate-Source capacitance (F). Typical: 100–500 pF for power MOSFETs.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cgs: f64,
    /// Gate-Drain capacitance (Miller capacitance) (F). Typical: 10–100 pF.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cgd: f64,
}

/// Gate capacitances of a [`CircuitElement::Jfet`], for transient and AC, as
/// a SPICE `.model NJF`/`PJF` card's `CGS`, `CGD`, `PB`, `M` and `FC` give
/// them.
///
/// Each gate junction is a depletion capacitance
/// `C = C0 / (1 − v/PB)^M` at junction voltage `v` (`vgs` and `vgd` for an
/// N-channel part, their negatives for a P-channel one), continued linearly
/// above `FC·PB` so it stays finite in forward bias, as SPICE's junction
/// charge. Transient integrates the charge, so it is conserved; AC uses the
/// capacitance at the bias point. Values are at the nominal 27 °C.
///
/// The default is no capacitance with SPICE's `PB = 1`, `M = 0.5` and
/// `FC = 0.5`. With serde, a missing field takes its default.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JfetParasiticCaps {
    /// Zero-bias gate–source capacitance CGS (F), zero or positive. Default 0.
    pub cgs: f64,
    /// Zero-bias gate–drain capacitance CGD (F), zero or positive; the
    /// Miller capacitance of a common-source stage. Default 0.
    pub cgd: f64,
    /// Gate junction potential PB (V), positive. Default 1.
    pub pb: f64,
    /// Grading coefficient M, in `[0, 0.9]`. Default 0.5 (an abrupt
    /// junction, the only value ngspice's JFET has).
    pub m: f64,
    /// Forward-bias depletion coefficient FC, in `[0, 0.95]`: above `FC·PB`
    /// the capacitance is continued linearly. Default 0.5.
    pub fc: f64,
}

impl Default for JfetParasiticCaps {
    fn default() -> Self {
        Self {
            cgs: 0.0,
            cgd: 0.0,
            pb: 1.0,
            m: 0.5,
            fc: 0.5,
        }
    }
}

impl JfetParasiticCaps {
    /// Whether either gate junction has a capacitance, which makes the
    /// circuit reactive.
    pub fn stores_charge(&self) -> bool {
        self.cgs > 0.0 || self.cgd > 0.0
    }
}

/// What a [`CircuitElement::Behavioural`] source's expression sets.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BehaviouralOutput {
    /// The voltage across the source, `n+` over `n-` (V): SPICE's `V=`.
    Voltage,
    /// The current through the source from `n+` to `n-` (A): SPICE's `I=`.
    Current,
}

/// Which throw of a [`CircuitElement::SwitchSpdt`] the common terminal
/// connects to.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SpdtPosition {
    /// Common to `nodes[1]`.
    #[default]
    A,
    /// Common to `nodes[2]`.
    B,
}

/// A quartz crystal's Butterworth–Van Dyke model, for
/// [`CircuitElement::Crystal`]: a motional arm `Rm`–`Lm`–`Cm` in series,
/// in parallel with the shunt capacitance `C0`. The series resonance is
/// `frequency`, so `Lm = 1 / ((2π·frequency)²·Cm)`.
///
/// Datasheets give the frequency, the ESR (`Rm`) and the shunt capacitance;
/// the motional capacitance is often only in an application note. The load
/// capacitance a datasheet names isn't part of the crystal: it's the two
/// capacitors on the board.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrystalParams {
    /// Series-resonant frequency (Hz), positive.
    pub frequency: f64,
    /// Equivalent series resistance `Rm` (Ω), positive.
    pub esr: f64,
    /// Shunt capacitance `C0` (F), zero or positive.
    pub c0: f64,
    /// Motional capacitance `Cm` (F), positive.
    pub cm: f64,
}

impl CrystalParams {
    /// From a datasheet's frequency, ESR and shunt capacitance, with a
    /// motional capacitance of `c0 / 250`, typical of AT-cut crystals in
    /// the megahertz range.
    pub fn from_datasheet(frequency: f64, esr: f64, c0: f64) -> Self {
        Self {
            frequency,
            esr,
            c0,
            cm: c0 / 250.0,
        }
    }

    /// Motional inductance `Lm` (H), from the frequency and `Cm`.
    pub fn lm(&self) -> f64 {
        let w = 2.0 * std::f64::consts::PI * self.frequency;
        1.0 / (w * w * self.cm)
    }
}

/// A ferrite bead's circuit, for [`CircuitElement::FerriteBead`]: `r_dc`
/// in series with `l ∥ r_ac` (and `c`, when set). Below its peak the bead
/// is inductive, at the peak it's `r_ac`, and above it `c` takes over.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FerriteBeadParams {
    /// DC resistance (Ω), positive.
    pub r_dc: f64,
    /// Inductance (H), positive.
    pub l: f64,
    /// Parallel loss resistance (Ω), positive: about the impedance at the
    /// peak.
    pub r_ac: f64,
    /// Parallel capacitance (F); `None` for none.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub c: Option<f64>,
}

impl FerriteBeadParams {
    /// From a datasheet's impedance at 100 MHz and DC resistance. Takes the
    /// bead as inductive up to 100 MHz, `l = z_100mhz / (2π·100 MHz)`, with
    /// its loss resistance at the same impedance: a rough fit that matches
    /// at 100 MHz and below. Fit `l`, `r_ac` and `c` to the impedance curve
    /// when the peak matters.
    pub fn from_datasheet(z_100mhz: f64, r_dc: f64) -> Self {
        Self {
            r_dc,
            l: z_100mhz / (2.0 * std::f64::consts::PI * 100e6),
            r_ac: z_100mhz,
            c: None,
        }
    }
}

/// A single circuit component.
///
/// Each variant carries an `id` (unique per circuit, used to look up results)
/// and a `nodes` array naming its terminals. Node names are arbitrary
/// strings — components share a node by referencing the same string.
///
/// # Sign conventions
///
/// - **Two-terminal components** (resistor, source, capacitor, …):
///   `nodes[0]` is the positive / first terminal, `nodes[1]` is the
///   negative / second.
/// - **Polarised components** (voltage/current sources, diodes, BJTs, …):
///   exact terminal order is documented per variant.
/// - **Currents** are reported as flowing from `nodes[0]` to `nodes[1]` for
///   two-terminal components.
///
/// # Serde
///
/// With the default `serde` feature enabled, `CircuitElement` serialises
/// using a `type` discriminator: `{"type": "resistor", "id": "R1", ...}`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", serde(tag = "type"))]
#[non_exhaustive]
pub enum CircuitElement {
    /// Resistor between two nodes.
    #[cfg_attr(feature = "serde", serde(rename = "resistor"))]
    Resistor(Resistor),

    /// Independent voltage source.
    #[cfg_attr(feature = "serde", serde(rename = "voltage_source"))]
    VoltageSource(VoltageSource),

    /// Independent current source.
    #[cfg_attr(feature = "serde", serde(rename = "current_source"))]
    CurrentSource(CurrentSource),

    /// Switch between two nodes.
    #[cfg_attr(feature = "serde", serde(rename = "switch"))]
    Switch(Switch),

    /// Capacitor.
    #[cfg_attr(feature = "serde", serde(rename = "capacitor"))]
    Capacitor(Capacitor),

    /// A battery: open-circuit voltage by state of charge behind a series
    /// resistance and an optional RC pair
    /// ([`BatteryParams`](sindr_devices::battery::BatteryParams)).
    #[cfg_attr(feature = "serde", serde(rename = "battery"))]
    Battery(Battery),

    /// Inductor.
    #[cfg_attr(feature = "serde", serde(rename = "inductor"))]
    Inductor(Inductor),

    /// Diode (Shockley model, silicon unless `model` says otherwise).
    #[cfg_attr(feature = "serde", serde(rename = "diode"))]
    Diode(Diode),

    /// LED.
    #[cfg_attr(feature = "serde", serde(rename = "led"))]
    Led(Led),

    /// BJT transistor (3 terminals): Ebers–Moll with Early effect, solved in
    /// DC, transient and AC.
    #[cfg_attr(feature = "serde", serde(rename = "bjt"))]
    Bjt(Bjt),

    /// MOSFET transistor (3 terminals: gate, drain, source; the body is tied to
    /// the source).
    #[cfg_attr(feature = "serde", serde(rename = "mosfet"))]
    Mosfet(Mosfet),

    /// N-channel or P-channel JFET (Shockley square-law model).
    #[cfg_attr(feature = "serde", serde(rename = "jfet"))]
    Jfet(Jfet),

    /// Voltage-Controlled Voltage Source: V_out = mu * V_control.
    #[cfg_attr(feature = "serde", serde(rename = "vcvs"))]
    Vcvs(Vcvs),

    /// Voltage-Controlled Current Source: I_out = gm * V_control.
    #[cfg_attr(feature = "serde", serde(rename = "vccs"))]
    Vccs(Vccs),

    /// Current-Controlled Voltage Source: V_out = rm * I_control.
    #[cfg_attr(feature = "serde", serde(rename = "ccvs"))]
    Ccvs(Ccvs),

    /// Current-Controlled Current Source: I_out = alpha * I_control.
    #[cfg_attr(feature = "serde", serde(rename = "cccs"))]
    Cccs(Cccs),

    /// Pushbutton (momentary switch): identical behaviour to Switch.
    #[cfg_attr(feature = "serde", serde(rename = "pushbutton"))]
    Pushbutton(Pushbutton),

    /// Relay: 4-terminal component with a coil and a contact pair.
    #[cfg_attr(feature = "serde", serde(rename = "relay"))]
    Relay(Relay),

    /// Photoresistor (LDR): resistance varies logarithmically with light.
    #[cfg_attr(feature = "serde", serde(rename = "photoresistor"))]
    Photoresistor(Photoresistor),

    /// Potentiometer: 3-terminal voltage divider.
    #[cfg_attr(feature = "serde", serde(rename = "potentiometer"))]
    Potentiometer(Potentiometer),

    /// Zener diode between two nodes: a silicon diode with SPICE reverse
    /// breakdown, `BV = vz` at `IBV` = 5 mA and 1 Ω of series resistance (the
    /// card `D(IS=1e-14 N=1 RS=1 BV=vz IBV=5m)`), so it holds about `vz` at a
    /// small zener's 5 mA test current and rises 1 V per amp past it.
    #[cfg_attr(feature = "serde", serde(rename = "zener_diode"))]
    ZenerDiode(ZenerDiode),

    /// Op-amp, output referred to ground.
    #[cfg_attr(feature = "serde", serde(rename = "op_amp"))]
    OpAmp(OpAmp),

    /// Comparator, output referred to ground: gain 1e5 from `v₊ − v₋` with no
    /// frequency response, its output clamped to `[v_neg, v_pos]`.
    #[cfg_attr(feature = "serde", serde(rename = "comparator"))]
    Comparator(Comparator),

    /// Schottky diode: lower forward voltage (~0.3V) than silicon diode.
    #[cfg_attr(feature = "serde", serde(rename = "schottky_diode"))]
    SchottkyDiode(SchottkyDiode),

    /// NTC thermistor: resistance varies with temperature (passive, no NR).
    #[cfg_attr(feature = "serde", serde(rename = "thermistor"))]
    Thermistor(Thermistor),

    /// Photodiode: diode + photocurrent source driven by irradiance.
    #[cfg_attr(feature = "serde", serde(rename = "photodiode"))]
    Photodiode(Photodiode),

    /// Varactor diode: voltage-dependent junction capacitance.
    #[cfg_attr(feature = "serde", serde(rename = "varactor"))]
    Varactor(Varactor),

    /// IGBT: MOSFET gate control + BJT output characteristics.
    #[cfg_attr(feature = "serde", serde(rename = "igbt"))]
    Igbt(Igbt),

    /// Ideal transformer with coupled inductors.
    #[cfg_attr(feature = "serde", serde(rename = "transformer"))]
    Transformer(Transformer),

    /// Fuse between two nodes.
    #[cfg_attr(feature = "serde", serde(rename = "fuse"))]
    Fuse(Fuse),

    /// Linear voltage regulator: holds the output at `voltage` above its ground
    /// pin while it has headroom, and draws its output current plus its
    /// quiescent current from the input.
    #[cfg_attr(feature = "serde", serde(rename = "voltage_regulator"))]
    VoltageRegulator(VoltageRegulator),

    /// Single-pole double-throw switch: `nodes: [common, a, b]`.
    #[cfg_attr(feature = "serde", serde(rename = "switch_spdt"))]
    SwitchSpdt(SwitchSpdt),

    /// Quartz crystal: the Butterworth–Van Dyke model ([`CrystalParams`]).
    #[cfg_attr(feature = "serde", serde(rename = "crystal"))]
    Crystal(Crystal),

    /// Ferrite bead ([`FerriteBeadParams`]): its DC resistance at DC, lossy and
    /// inductive at high frequency.
    #[cfg_attr(feature = "serde", serde(rename = "ferrite_bead"))]
    FerriteBead(FerriteBead),

    /// Behavioural source, SPICE's `B n+ n- V={expr}` or `I={expr}`: a voltage
    /// or current set by an [`Expression`] of node voltages, branch currents
    /// and time.
    #[cfg_attr(feature = "serde", serde(rename = "behavioural"))]
    Behavioural(Behavioural),
}

impl CircuitElement {
    /// Returns a reference to the node array for this component.
    pub fn nodes(&self) -> &[String] {
        self.device().nodes()
    }

    /// Returns all node names referenced by this component, including control nodes.
    pub fn all_nodes(&self) -> Vec<&String> {
        self.device().all_nodes()
    }

    /// Returns the component identifier.
    pub fn id(&self) -> &str {
        self.device().id()
    }

    /// Extra MNA unknowns the element adds after the node voltages: one
    /// branch current for a voltage-type source, two for a transformer
    /// (one per winding), and two for an op-amp with an [`OpAmpModel`] or a
    /// comparator (its output current, then its internal gain-stage voltage).
    pub fn branch_slots(&self) -> usize {
        self.device().branches()
    }

    /// Mutable [`nodes`](Self::nodes), for renaming nets.
    pub fn nodes_mut(&mut self) -> &mut [String] {
        self.device_mut().nodes_mut()
    }

    /// Mutable [`id`](Self::id), for renaming.
    pub fn id_mut(&mut self) -> &mut String {
        self.device_mut().id_mut()
    }

    /// The control-net pair of a VCVS or VCCS.
    pub fn control_nodes_mut(&mut self) -> Option<&mut [String; 2]> {
        match self {
            CircuitElement::Vcvs(Vcvs { control_nodes, .. })
            | CircuitElement::Vccs(Vccs { control_nodes, .. }) => Some(control_nodes),
            _ => None,
        }
    }

    /// The controlling source id of a CCVS or CCCS.
    pub fn control_source_mut(&mut self) -> Option<&mut String> {
        match self {
            CircuitElement::Ccvs(Ccvs { control_source, .. })
            | CircuitElement::Cccs(Cccs { control_source, .. }) => Some(control_source),
            _ => None,
        }
    }

    /// The expression of a behavioural source, for renaming the nodes and
    /// sources it reads.
    pub(crate) fn expression_mut(&mut self) -> Option<&mut Expression> {
        match self {
            CircuitElement::Behavioural(Behavioural { expr, .. }) => Some(expr),
            _ => None,
        }
    }
}

/// A circuit: components, nets, and optionally a design hierarchy.
///
/// The top level is a list of [`CircuitElement`]s plus the name of the
/// ground net. Components share a net by referencing the same net-name
/// string. A design can also place [`Block`]s through [`Instance`]s and
/// carry [`Part`] data (reference, symbol, footprint) that the simulator
/// ignores. [`Circuit::flatten`] expands the hierarchy; sindr's analyses
/// do that themselves.
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement};
/// use sindr::element::{Resistor, VoltageSource};
///
/// let circuit = Circuit::new(
///     "0",
///     vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["n1".into(), "0".into()],
///             voltage: 9.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R1".into(),
///             nodes: ["n1".into(), "0".into()],
///             resistance: 1_000.0,
///         }),
///     ],
/// );
/// assert!(circuit.is_flat());
/// ```
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Circuit {
    /// Top-level components. Order has no semantic meaning.
    pub components: Vec<CircuitElement>,
    /// Name of the reference (0 V) net. Must match the net string used by
    /// at least one component (typically `"0"` or `"gnd"`). It is the same
    /// net inside every block.
    pub ground_node: String,
    /// Top-level block instances.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Vec::is_empty")
    )]
    pub instances: Vec<Instance>,
    /// Block definitions, by name.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "BTreeMap::is_empty")
    )]
    pub blocks: BTreeMap<String, Block>,
    /// Part data for top-level components and instances, keyed by their id.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "BTreeMap::is_empty")
    )]
    pub parts: BTreeMap<String, Part>,
    /// Nets visible inside every block under the same name (supply rails),
    /// besides ground. Never prefixed when flattening.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Vec::is_empty")
    )]
    pub globals: Vec<String>,
}

impl Circuit {
    /// A flat circuit: `components` referencing `ground_node`, no hierarchy.
    pub fn new(ground_node: impl Into<String>, components: Vec<CircuitElement>) -> Self {
        Self {
            components,
            ground_node: ground_node.into(),
            ..Self::default()
        }
    }

    /// `true` when there are no instances to expand.
    pub fn is_flat(&self) -> bool {
        self.instances.is_empty()
    }

    /// Every net named at the top level: component and instance nodes plus
    /// ground, sorted and without repeats. Nets local to block instances
    /// show up only after [`flatten`](Self::flatten).
    pub fn nets(&self) -> Vec<&str> {
        let mut nets: Vec<&str> = std::iter::once(self.ground_node.as_str())
            .chain(
                self.components
                    .iter()
                    .flat_map(|c| c.all_nodes().into_iter().map(String::as_str)),
            )
            .chain(
                self.instances
                    .iter()
                    .flat_map(|i| i.nodes.iter().map(String::as_str)),
            )
            .collect();
        nets.sort_unstable();
        nets.dedup();
        nets
    }

    /// Count the number of independent voltage sources in the circuit.
    /// Also counts VCVS, CCVS, OpAmp, and Comparator which need branch currents.
    /// Transformer adds 2 branch current unknowns (one per winding).
    pub fn count_voltage_sources(&self) -> usize {
        self.components
            .iter()
            .map(CircuitElement::branch_slots)
            .sum()
    }

    /// Returns true if the circuit contains any nonlinear elements.
    pub fn has_nonlinear_elements(&self) -> bool {
        self.components.iter().any(|c| c.device().is_nonlinear())
    }

    /// Returns true if the circuit contains any reactive elements (Capacitor, Inductor, or
    /// Relay with inductance > 0, or Transformer, or BJT/MOSFET/JFET with non-zero parasitic caps,
    /// or a diode whose model stores charge).
    pub fn has_reactive_elements(&self) -> bool {
        self.components.iter().any(|c| c.device().is_reactive())
    }

    /// Returns true if any source has a time-varying waveform.
    pub fn has_waveform_sources(&self) -> bool {
        self.components.iter().any(|c| match c {
            CircuitElement::VoltageSource(VoltageSource { waveform, .. })
            | CircuitElement::CurrentSource(CurrentSource { waveform, .. }) => waveform.is_some(),
            _ => false,
        })
    }

    /// Find the branch index for a voltage source by its ID.
    /// Returns None if not found.
    pub fn vsource_branch_index(&self, source_id: &str) -> Option<usize> {
        let mut idx = 0;
        for component in &self.components {
            let device = component.device();
            if device.exposes_branch_current() && device.id() == source_id {
                return Some(idx);
            }
            idx += device.branches();
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sindr_devices::bjt::BjtKind;
    use sindr_devices::jfet::JfetKind;
    use sindr_devices::mosfet::{MosfetKind, MosfetParams};

    #[test]
    fn has_reactive_elements_true_with_capacitor() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["n1".into(), "0".into()],
                capacitance: 100e-6,
            })],
            ..Default::default()
        };
        assert!(circuit.has_reactive_elements());
    }

    #[test]
    fn has_reactive_elements_true_with_inductor() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Inductor(Inductor {
                id: "L1".into(),
                nodes: ["n1".into(), "0".into()],
                inductance: 10e-3,
                saturation: None,
            })],
            ..Default::default()
        };
        assert!(circuit.has_reactive_elements());
    }

    #[test]
    fn has_reactive_elements_false_resistor_only() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1000.0,
            })],
            ..Default::default()
        };
        assert!(!circuit.has_reactive_elements());
    }

    #[test]
    fn diode_storing_charge_is_reactive() {
        let diode = |model: Option<DiodeModel>| Circuit {
            components: vec![CircuitElement::Diode(Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model,
            })],
            ..Default::default()
        };
        assert!(!diode(None).has_reactive_elements());
        assert!(!diode(Some(DiodeModel {
            bv: Some(100.0),
            ..DiodeModel::default()
        }))
        .has_reactive_elements());
        assert!(diode(Some(DiodeModel {
            tt: 1e-9,
            ..DiodeModel::default()
        }))
        .has_reactive_elements());
        assert!(diode(Some(DiodeModel {
            cjo: 1e-12,
            ..DiodeModel::default()
        }))
        .has_reactive_elements());
    }

    #[test]
    fn bjt_with_parasitic_caps_is_reactive() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "e".into()],
                kind: sindr_devices::bjt::BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: Some(BjtParasiticCaps {
                    cbe: 10e-12,
                    cbc: 0.0,
                }),
                model: None,
            })],
            ..Default::default()
        };
        assert!(
            circuit.has_reactive_elements(),
            "BJT with cbe>0 should be reactive"
        );
    }

    #[test]
    fn bjt_without_parasitic_caps_not_reactive() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "e".into()],
                kind: sindr_devices::bjt::BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            })],
            ..Default::default()
        };
        // BJT without parasitic caps is not reactive (nonlinear, but not reactive)
        assert!(
            !circuit.has_reactive_elements(),
            "BJT without parasitic caps should not be reactive"
        );
    }

    #[test]
    fn mosfet_with_parasitic_caps_is_reactive() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Mosfet(Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "s".into()],
                kind: sindr_devices::mosfet::MosfetKind::Nmos,
                params: sindr_devices::mosfet::MosfetParams::default(),
                parasitic_caps: Some(MosfetParasiticCaps {
                    cgs: 100e-12,
                    cgd: 50e-12,
                }),
                body_diode: None,
            })],
            ..Default::default()
        };
        assert!(
            circuit.has_reactive_elements(),
            "MOSFET with cgs>0 should be reactive"
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn bjt_serde_roundtrip() {
        let json = r#"{
            "components": [
                {"type": "voltage_source", "id": "V1", "nodes": ["n1", "0"], "voltage": 10.0},
                {"type": "resistor", "id": "RC", "nodes": ["n1", "n2"], "resistance": 1000.0},
                {"type": "resistor", "id": "RB", "nodes": ["n1", "n3"], "resistance": 100000.0},
                {"type": "bjt", "id": "Q1", "nodes": ["n3", "n2", "0"], "kind": "npn", "bf": 100}
            ],
            "ground_node": "0"
        }"#;
        let circuit: Circuit = serde_json::from_str(json).unwrap();
        assert_eq!(circuit.components.len(), 4);
        let bjt = &circuit.components[3];
        assert_eq!(bjt.id(), "Q1");
        assert_eq!(bjt.nodes().len(), 3);
        assert_eq!(bjt.nodes()[0], "n3"); // base
        assert_eq!(bjt.nodes()[1], "n2"); // collector
        assert_eq!(bjt.nodes()[2], "0"); // emitter
    }

    #[cfg(feature = "serde")]
    #[test]
    fn bjt_default_bf() {
        let json = r#"{
            "components": [
                {"type": "bjt", "id": "Q1", "nodes": ["b", "c", "e"], "kind": "npn"}
            ],
            "ground_node": "0"
        }"#;
        let circuit: Circuit = serde_json::from_str(json).unwrap();
        match &circuit.components[0] {
            CircuitElement::Bjt(Bjt { bf, .. }) => assert_eq!(*bf, 100.0),
            _ => panic!("Expected Bjt variant"),
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn device_models_are_optional_and_partial_in_json() {
        let json = r#"{
            "components": [
                {"type": "diode", "id": "D0", "nodes": ["a", "0"]},
                {"type": "diode", "id": "D1", "nodes": ["a", "0"], "model": {"rs": 0.5}},
                {"type": "bjt", "id": "Q1", "nodes": ["b", "c", "e"], "kind": "pnp",
                 "model": {"is": 1e-15, "vaf": 50}}
            ],
            "ground_node": "0"
        }"#;
        let circuit: Circuit = serde_json::from_str(json).unwrap();
        assert!(matches!(
            &circuit.components[0],
            CircuitElement::Diode(Diode { model: None, .. })
        ));
        match &circuit.components[1] {
            CircuitElement::Diode(Diode { model, .. }) => assert_eq!(
                *model,
                Some(DiodeModel {
                    rs: 0.5,
                    ..DiodeModel::default()
                })
            ),
            other => panic!("expected Diode, got {other:?}"),
        }
        match &circuit.components[2] {
            CircuitElement::Bjt(Bjt { model, .. }) => assert_eq!(
                *model,
                Some(BjtModel {
                    is: 1e-15,
                    vaf: 50.0,
                    ..BjtModel::default()
                })
            ),
            other => panic!("expected Bjt, got {other:?}"),
        }

        let out = serde_json::to_string(&circuit.components[0]).unwrap();
        assert!(!out.contains("model"), "{out}");
    }

    #[test]
    fn has_nonlinear_elements_true_with_bjt() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "e".into()],
                kind: sindr_devices::bjt::BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            })],
            ..Default::default()
        };
        assert!(circuit.has_nonlinear_elements());
    }

    #[test]
    fn relay_has_nonlinear_returns_true() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Relay(Relay {
                id: "K1".into(),
                nodes: ["coil_p".into(), "0".into(), "c1".into(), "c2".into()],
                coil_resistance: 500.0,
                pickup_voltage: 5.0,
                inductance: 0.0,
            })],
            ..Default::default()
        };
        assert!(circuit.has_nonlinear_elements());
    }

    #[test]
    fn relay_with_inductance_has_reactive_elements() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Relay(Relay {
                id: "K1".into(),
                nodes: ["coil_p".into(), "0".into(), "c1".into(), "c2".into()],
                coil_resistance: 500.0,
                pickup_voltage: 5.0,
                inductance: 0.1,
            })],
            ..Default::default()
        };
        assert!(circuit.has_reactive_elements());
    }

    #[test]
    fn relay_without_inductance_not_reactive() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Relay(Relay {
                id: "K1".into(),
                nodes: ["coil_p".into(), "0".into(), "c1".into(), "c2".into()],
                coil_resistance: 500.0,
                pickup_voltage: 5.0,
                inductance: 0.0,
            })],
            ..Default::default()
        };
        assert!(!circuit.has_reactive_elements());
    }

    #[test]
    fn pushbutton_has_nonlinear_returns_false() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Pushbutton(Pushbutton {
                id: "PB1".into(),
                nodes: ["n1".into(), "0".into()],
                closed: true,
            })],
            ..Default::default()
        };
        assert!(!circuit.has_nonlinear_elements());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn relay_default_coil_resistance() {
        let json = r#"{
            "components": [
                {"type": "relay", "id": "K1", "nodes": ["cp", "cn", "c1", "c2"], "pickup_voltage": 5.0}
            ],
            "ground_node": "0"
        }"#;
        let circuit: Circuit = serde_json::from_str(json).unwrap();
        match &circuit.components[0] {
            CircuitElement::Relay(Relay {
                coil_resistance, ..
            }) => {
                assert_eq!(*coil_resistance, 500.0)
            }
            _ => panic!("Expected Relay variant"),
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn potentiometer_serde_roundtrip() {
        let json = r#"{
            "components": [
                {"type": "potentiometer", "id": "P1", "nodes": ["top", "wiper", "bot"], "resistance": 10000.0, "position": 0.5}
            ],
            "ground_node": "0"
        }"#;
        let circuit: Circuit = serde_json::from_str(json).unwrap();
        assert_eq!(circuit.components.len(), 1);
        let p1 = &circuit.components[0];
        assert_eq!(p1.id(), "P1");
        assert_eq!(p1.nodes().len(), 3);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn zener_diode_serde_roundtrip() {
        let json = r#"{
            "components": [
                {"type": "zener_diode", "id": "Z1", "nodes": ["n1", "0"], "vz": 5.1}
            ],
            "ground_node": "0"
        }"#;
        let circuit: Circuit = serde_json::from_str(json).unwrap();
        assert_eq!(circuit.components.len(), 1);
        match &circuit.components[0] {
            CircuitElement::ZenerDiode(ZenerDiode { id, vz, .. }) => {
                assert_eq!(id, "Z1");
                assert_eq!(*vz, 5.1);
            }
            _ => panic!("Expected ZenerDiode variant"),
        }
        assert!(circuit.has_nonlinear_elements());
    }

    #[test]
    fn op_amp_in_count_voltage_sources() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::OpAmp(OpAmp {
                    id: "U1".into(),
                    nodes: ["in_p".into(), "in_m".into(), "out".into()],
                    v_pos: 15.0,
                    v_neg: -15.0,
                    model: None,
                }),
                CircuitElement::Comparator(Comparator {
                    id: "U2".into(),
                    nodes: ["in_p".into(), "in_m".into(), "out2".into()],
                    v_pos: 5.0,
                    v_neg: 0.0,
                }),
            ],
            ..Default::default()
        };
        // The op-amp is one VCVS branch; the comparator, clamped to its
        // rails, adds its gain-stage voltage too and needs Newton.
        assert_eq!(circuit.count_voltage_sources(), 3);
        assert!(circuit.has_nonlinear_elements());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn op_amp_default_supply_rails() {
        let json = r#"{
            "components": [
                {"type": "op_amp", "id": "U1", "nodes": ["inp", "inm", "out"]}
            ],
            "ground_node": "0"
        }"#;
        let circuit: Circuit = serde_json::from_str(json).unwrap();
        match &circuit.components[0] {
            CircuitElement::OpAmp(OpAmp { v_pos, v_neg, .. }) => {
                assert_eq!(*v_pos, 15.0);
                assert_eq!(*v_neg, -15.0);
            }
            _ => panic!("Expected OpAmp variant"),
        }
    }

    /// One of every element, with nets `<id>.0`, `<id>.1`, …, so a test can
    /// tell which field an accessor reached.
    fn every_element() -> Vec<CircuitElement> {
        fn n<const N: usize>(id: &str) -> [String; N] {
            std::array::from_fn(|k| format!("{id}.{k}"))
        }
        vec![
            CircuitElement::Resistor(Resistor {
                id: "R".into(),
                nodes: n("R"),
                resistance: 1.0,
            }),
            CircuitElement::VoltageSource(VoltageSource {
                id: "V".into(),
                nodes: n("V"),
                voltage: 1.0,
                waveform: None,
            }),
            CircuitElement::CurrentSource(CurrentSource {
                id: "I".into(),
                nodes: n("I"),
                current: 1.0,
                waveform: None,
            }),
            CircuitElement::Switch(Switch {
                id: "S".into(),
                nodes: n("S"),
                closed: true,
            }),
            CircuitElement::Capacitor(Capacitor {
                id: "C".into(),
                nodes: n("C"),
                capacitance: 1.0,
            }),
            CircuitElement::Battery(Battery {
                id: "B".into(),
                nodes: n("B"),
                params: sindr_devices::battery::BatteryParams::constant(3.0, 1.0, 0.1),
                state_of_charge: 1.0,
            }),
            CircuitElement::Inductor(Inductor {
                id: "L".into(),
                nodes: n("L"),
                inductance: 1.0,
                saturation: None,
            }),
            CircuitElement::Diode(Diode {
                id: "D".into(),
                nodes: n("D"),
                temperature: 300.15,
                model: None,
            }),
            CircuitElement::Led(Led {
                id: "LED".into(),
                nodes: n("LED"),
                color: "red".into(),
                temperature: 300.15,
            }),
            CircuitElement::Bjt(Bjt {
                id: "Q".into(),
                nodes: n("Q"),
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }),
            CircuitElement::Mosfet(Mosfet {
                id: "M".into(),
                nodes: n("M"),
                kind: MosfetKind::Nmos,
                params: MosfetParams::default_nmos(),
                parasitic_caps: None,
                body_diode: None,
            }),
            CircuitElement::Jfet(Jfet {
                id: "J".into(),
                nodes: n("J"),
                kind: JfetKind::NChannel,
                idss: 1e-3,
                vp: -2.0,
                parasitic_caps: None,
            }),
            CircuitElement::Vcvs(Vcvs {
                id: "E".into(),
                nodes: n("E"),
                control_nodes: ["E.c0".into(), "E.c1".into()],
                gain: 1.0,
            }),
            CircuitElement::Vccs(Vccs {
                id: "G".into(),
                nodes: n("G"),
                control_nodes: ["G.c0".into(), "G.c1".into()],
                gm: 1.0,
            }),
            CircuitElement::Ccvs(Ccvs {
                id: "H".into(),
                nodes: n("H"),
                control_source: "V".into(),
                rm: 1.0,
            }),
            CircuitElement::Cccs(Cccs {
                id: "F".into(),
                nodes: n("F"),
                control_source: "V".into(),
                alpha: 1.0,
            }),
            CircuitElement::Pushbutton(Pushbutton {
                id: "PB".into(),
                nodes: n("PB"),
                closed: false,
            }),
            CircuitElement::Relay(Relay {
                id: "K".into(),
                nodes: n("K"),
                coil_resistance: 500.0,
                pickup_voltage: 3.0,
                inductance: 0.0,
            }),
            CircuitElement::Photoresistor(Photoresistor {
                id: "LDR".into(),
                nodes: n("LDR"),
                light_level: 0.5,
            }),
            CircuitElement::Potentiometer(Potentiometer {
                id: "POT".into(),
                nodes: n("POT"),
                resistance: 1e3,
                position: 0.5,
            }),
            CircuitElement::ZenerDiode(ZenerDiode {
                id: "DZ".into(),
                nodes: n("DZ"),
                vz: 5.1,
                temperature: 300.15,
            }),
            CircuitElement::OpAmp(OpAmp {
                id: "U".into(),
                nodes: n("U"),
                v_pos: 15.0,
                v_neg: -15.0,
                model: None,
            }),
            CircuitElement::Comparator(Comparator {
                id: "CMP".into(),
                nodes: n("CMP"),
                v_pos: 5.0,
                v_neg: 0.0,
            }),
            CircuitElement::SchottkyDiode(SchottkyDiode {
                id: "DS".into(),
                nodes: n("DS"),
                temperature: 300.15,
            }),
            CircuitElement::Thermistor(Thermistor {
                id: "TH".into(),
                nodes: n("TH"),
                temperature: 298.15,
            }),
            CircuitElement::Photodiode(Photodiode {
                id: "PD".into(),
                nodes: n("PD"),
                irradiance: 1.0,
                temperature: 300.15,
            }),
            CircuitElement::Varactor(Varactor {
                id: "CV".into(),
                nodes: n("CV"),
                params: sindr_devices::varactor::VaractorParams::default(),
            }),
            CircuitElement::Igbt(Igbt {
                id: "Z".into(),
                nodes: n("Z"),
                params: sindr_devices::igbt::IgbtParams::default(),
            }),
            CircuitElement::Transformer(Transformer {
                id: "T".into(),
                nodes: n("T"),
                l1: 1.0,
                l2: 1.0,
                k: 0.99,
                saturation: None,
            }),
            CircuitElement::Fuse(Fuse {
                id: "FU".into(),
                nodes: n("FU"),
                rating: 1.0,
                blown: false,
            }),
            CircuitElement::VoltageRegulator(VoltageRegulator {
                id: "REG".into(),
                nodes: n("REG"),
                voltage: 5.0,
                model: None,
            }),
            CircuitElement::SwitchSpdt(SwitchSpdt {
                id: "SW".into(),
                nodes: n("SW"),
                position: SpdtPosition::default(),
            }),
            CircuitElement::Crystal(Crystal {
                id: "X".into(),
                nodes: n("X"),
                params: CrystalParams::from_datasheet(32768.0, 35e3, 1.5e-12),
            }),
            CircuitElement::FerriteBead(FerriteBead {
                id: "FB".into(),
                nodes: n("FB"),
                params: FerriteBeadParams::from_datasheet(600.0, 0.1),
            }),
        ]
    }

    /// `nodes_mut` and `id_mut` reach the very fields `nodes` and `id`
    /// read, for every element: renaming nets and ids (as flattening a
    /// hierarchy does) can't miss a terminal or rename the wrong field.
    #[test]
    fn mutable_accessors_reach_the_fields_the_readers_read() {
        let elements = every_element();
        assert_eq!(elements.len(), 34, "one of every CircuitElement");
        for mut element in elements {
            let id = element.id().to_string();
            let nodes = element.nodes().to_vec();
            assert!(
                nodes
                    .iter()
                    .enumerate()
                    .all(|(k, n)| *n == format!("{id}.{k}")),
                "{id}: nodes() is {nodes:?}"
            );
            assert_eq!(element.nodes_mut().to_vec(), nodes, "{id}");
            for net in element.nodes_mut() {
                net.insert_str(0, "top/");
            }
            let renamed: Vec<String> = nodes.iter().map(|n| format!("top/{n}")).collect();
            assert_eq!(element.nodes(), renamed.as_slice(), "{id}");
            // all_nodes() adds only the control pair of a VCVS or VCCS.
            let extra = element.all_nodes().len() - element.nodes().len();
            assert_eq!(extra, element.control_nodes_mut().map_or(0, |_| 2), "{id}");

            element.id_mut().push_str("_x");
            assert_eq!(element.id(), format!("{id}_x"));
        }
    }

    /// A branch current's index counts every earlier element's branch
    /// slots: one per voltage-type source, two for a transformer or a
    /// modelled op-amp.
    #[test]
    fn vsource_branch_index_counts_earlier_branch_slots() {
        let v = |id: &str| {
            CircuitElement::VoltageSource(VoltageSource {
                id: id.into(),
                nodes: ["a".into(), "0".into()],
                voltage: 1.0,
                waveform: None,
            })
        };
        let circuit = Circuit::new(
            "0",
            vec![
                v("V1"),
                CircuitElement::OpAmp(OpAmp {
                    id: "U1".into(),
                    nodes: ["a".into(), "b".into(), "c".into()],
                    v_pos: 15.0,
                    v_neg: -15.0,
                    model: Some(OpAmpModel::default()),
                }),
                CircuitElement::Transformer(Transformer {
                    id: "T1".into(),
                    nodes: ["a".into(), "0".into(), "b".into(), "0".into()],
                    l1: 1.0,
                    l2: 1.0,
                    k: 0.99,
                    saturation: None,
                }),
                v("V2"),
            ],
        );
        assert_eq!(circuit.vsource_branch_index("V1"), Some(0));
        assert_eq!(circuit.vsource_branch_index("U1"), Some(1));
        assert_eq!(circuit.vsource_branch_index("V2"), Some(5));
        // A transformer's winding currents aren't a source's current.
        assert_eq!(circuit.vsource_branch_index("T1"), None);
        assert_eq!(circuit.vsource_branch_index("V9"), None);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn jfet_gate_caps_left_out_of_json_take_spice_defaults() {
        let json = r#"{"type": "jfet", "id": "J1", "nodes": ["g", "d", "s"],
                       "kind": "pchannel", "parasitic_caps": {"cgs": 5e-12}}"#;
        let CircuitElement::Jfet(Jfet { parasitic_caps, .. }) = serde_json::from_str(json).unwrap()
        else {
            panic!("not a JFET");
        };
        assert_eq!(
            parasitic_caps,
            Some(JfetParasiticCaps {
                cgs: 5e-12,
                ..Default::default()
            })
        );
        let caps = parasitic_caps.unwrap();
        assert_eq!((caps.cgd, caps.pb, caps.m, caps.fc), (0.0, 1.0, 0.5, 0.5));
    }

    #[cfg(feature = "serde")]
    #[test]
    fn fields_left_out_of_json_take_their_documented_defaults() {
        let json = r#"{
            "ground_node": "0",
            "components": [
                {"type": "thermistor", "id": "TH1", "nodes": ["a", "0"]},
                {"type": "transformer", "id": "T1", "nodes": ["a", "0", "b", "0"],
                 "l1": 1e-3, "l2": 4e-3},
                {"type": "fuse", "id": "F1", "nodes": ["a", "b"]},
                {"type": "jfet", "id": "J1", "nodes": ["g", "d", "s"], "kind": "nchannel"},
                {"type": "mosfet", "id": "M1", "nodes": ["g", "d", "s"], "kind": "nmos",
                 "params": {"kp": 2e-4, "vto": 1.0, "lambda": 0.02, "gamma": 0.4, "phi": 0.6}}
            ]
        }"#;
        let circuit: Circuit = serde_json::from_str(json).unwrap();
        let expected = [
            CircuitElement::Thermistor(Thermistor {
                id: "TH1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 298.15,
            }),
            CircuitElement::Transformer(Transformer {
                id: "T1".into(),
                nodes: ["a".into(), "0".into(), "b".into(), "0".into()],
                l1: 1e-3,
                l2: 4e-3,
                k: 0.999,
                saturation: None,
            }),
            CircuitElement::Fuse(Fuse {
                id: "F1".into(),
                nodes: ["a".into(), "b".into()],
                rating: 1.0,
                blown: false,
            }),
            CircuitElement::Jfet(Jfet {
                id: "J1".into(),
                nodes: ["g".into(), "d".into(), "s".into()],
                kind: JfetKind::NChannel,
                idss: 10e-3,
                vp: -2.0,
                parasitic_caps: None,
            }),
            // No flicker noise unless asked: kf 0, af 1.
            CircuitElement::Mosfet(Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "s".into()],
                kind: MosfetKind::Nmos,
                params: MosfetParams {
                    kp: 2e-4,
                    vto: 1.0,
                    lambda: 0.02,
                    gamma: 0.4,
                    phi: 0.6,
                    kf: 0.0,
                    af: 1.0,
                },
                parasitic_caps: None,
                body_diode: None,
            }),
        ];
        assert_eq!(circuit.components, expected);
    }
}
