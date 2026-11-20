//! Circuit input format.
//!
//! [`Circuit`] is the root struct sindr's analyses take: a list of
//! [`CircuitElement`]s plus the name of the ground node, and optionally a
//! design hierarchy (see [`crate::design`]). Components share a node by
//! referencing the same node-name string; there are no separate "wire" or
//! "net" objects.
//!
//! [`CircuitElement`] is the open-ended enum of supported component types
//! (resistors, sources, semiconductors, magnetics, …). Each variant carries
//! its own parameters and a fixed-size `nodes` array; sign conventions are
//! documented per variant.

use sindr_devices::bjt::BjtKind;
use sindr_devices::jfet::JfetKind;
use sindr_devices::mosfet::{MosfetKind, MosfetParams};

use std::collections::BTreeMap;

use crate::design::{Block, Instance, Part};
use crate::waveform::Waveform;

/// BJT parasitic capacitances. All values in Farads. Default 0.0 disables each cap.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
/// element's `temperature` scales `is` from there, and only `is`.
///
/// The default is sindr's built-in silicon diode. With serde, a missing field
/// takes its default, so `{"rs": 0.5}` is a silicon diode with 0.5 Ω in series.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
/// `is` from there.
///
/// The default is sindr's built-in transistor. With serde, a missing field
/// takes its default.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
/// Optional gate capacitances of a MOSFET, for transient and AC.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MosfetParasiticCaps {
    /// Gate-Source capacitance (F). Typical: 100–500 pF for power MOSFETs.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cgs: f64,
    /// Gate-Drain capacitance (Miller capacitance) (F). Typical: 10–100 pF.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cgd: f64,
}

/// Which throw of a [`CircuitElement::SwitchSpdt`] the common terminal
/// connects to.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", serde(tag = "type"))]
pub enum CircuitElement {
    /// Resistor between two nodes.
    #[cfg_attr(feature = "serde", serde(rename = "resistor"))]
    Resistor {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`.
        nodes: [String; 2],
        /// Resistance (Ω).
        resistance: f64,
    },

    /// Independent voltage source. `nodes[0]` is the positive terminal,
    /// `nodes[1]` is the negative terminal.
    #[cfg_attr(feature = "serde", serde(rename = "voltage_source"))]
    VoltageSource {
        /// Unique component identifier.
        id: String,
        /// Terminals `[positive, negative]`.
        nodes: [String; 2],
        /// DC voltage (V); also the offset added to any `waveform`.
        voltage: f64,
        /// Optional time-varying waveform added on top of `voltage`.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        waveform: Option<Waveform>,
    },

    /// Independent current source. Current flows from `nodes[0]` toward
    /// `nodes[1]`.
    #[cfg_attr(feature = "serde", serde(rename = "current_source"))]
    CurrentSource {
        /// Unique component identifier.
        id: String,
        /// Terminals `[from, to]`; current flows from `nodes[0]` to `nodes[1]`.
        nodes: [String; 2],
        /// DC current (A); also the offset added to any `waveform`.
        current: f64,
        /// Optional time-varying waveform added on top of `current`.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        waveform: Option<Waveform>,
    },

    /// Switch between two nodes. Modeled as a resistor with very low (closed)
    /// or very high (open) resistance for DC analysis.
    #[cfg_attr(feature = "serde", serde(rename = "switch"))]
    Switch {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`.
        nodes: [String; 2],
        /// When true the switch conducts (closed); when false it is open.
        closed: bool,
    },

    /// Capacitor. Open-circuit in DC analysis; charges/discharges in
    /// transient. `voltage_across` is reported as `nodes[0] − nodes[1]`.
    #[cfg_attr(feature = "serde", serde(rename = "capacitor"))]
    Capacitor {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`.
        nodes: [String; 2],
        /// Capacitance (F).
        capacitance: f64,
    },

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
    #[cfg_attr(feature = "serde", serde(rename = "battery"))]
    Battery {
        /// Unique component identifier.
        id: String,
        /// Terminals `[positive, negative]`.
        nodes: [String; 2],
        /// The cell's equivalent circuit.
        params: sindr_devices::battery::BatteryParams,
        /// State of charge at the start, 0 (empty) to 1 (full).
        state_of_charge: f64,
    },

    /// Inductor. Stamped as **open-circuit in pure DC analysis** (sindr
    /// solves DC as a single algebraic step, not as t→∞ of a transient);
    /// integrates `dI/dt` properly during transient. The DC bias point that
    /// AC, `.tf` and `.sens` linearise around shorts it instead. `voltage_across` is
    /// reported as `nodes[0] − nodes[1]`, `current_through` flows
    /// `nodes[0] → nodes[1]`. To get steady-state current, run a
    /// transient long enough to settle.
    #[cfg_attr(feature = "serde", serde(rename = "inductor"))]
    Inductor {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`; `current_through` flows `nodes[0] → nodes[1]`.
        nodes: [String; 2],
        /// Inductance (H): the small-signal value, at zero current.
        inductance: f64,
        /// Core saturation: the inductance falls towards `l_sat` as the
        /// current passes `i_sat`. `None` is a linear inductor.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        saturation: Option<sindr_devices::inductor::InductorSaturation>,
    },

    /// Diode (Shockley model, silicon unless `model` says otherwise).
    /// `nodes: [anode, cathode]`. Forward current flows from anode → cathode;
    /// positive `voltage_across` corresponds to forward bias.
    #[cfg_attr(feature = "serde", serde(rename = "diode"))]
    Diode {
        /// Unique component identifier.
        id: String,
        /// Terminals `[anode, cathode]`.
        nodes: [String; 2],
        /// Junction temperature (K), positive and finite. Default 300.15 K.
        /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
        /// Model parameters (IS, N, RS). `None` is the built-in silicon diode,
        /// the same as `Some(DiodeModel::default())`.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        model: Option<DiodeModel>,
    },

    /// LED. `nodes: [anode, cathode]`. Forward voltage drop depends on
    /// `color` (e.g. `"red"` ≈ 1.8 V, `"blue"` ≈ 3.2 V).
    #[cfg_attr(feature = "serde", serde(rename = "led"))]
    Led {
        /// Unique component identifier.
        id: String,
        /// Terminals `[anode, cathode]`.
        nodes: [String; 2],
        /// Emission colour (e.g. `"red"`, `"blue"`); sets the forward voltage drop.
        color: String,
        /// Junction temperature (K), positive and finite. Default 300.15 K.
        /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// BJT transistor (3 terminals): Ebers–Moll with Early effect, solved in
    /// DC, transient and AC.
    #[cfg_attr(feature = "serde", serde(rename = "bjt"))]
    Bjt {
        /// Unique component identifier.
        id: String,
        /// Terminals `[base, collector, emitter]`.
        nodes: [String; 3], // [base, collector, emitter]
        /// Polarity (NPN or PNP).
        kind: BjtKind,
        /// Forward current gain β (unitless), positive and finite. Default 100.
        #[cfg_attr(feature = "serde", serde(default = "default_bf"))]
        bf: f64,
        /// Junction temperature (K), positive and finite. Default 300.15 K.
        /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
        /// Optional parasitic capacitances (Cbe, Cbc). None = no parasitic caps (default).
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        parasitic_caps: Option<BjtParasiticCaps>,
        /// Ebers–Moll parameters besides `bf` (IS, BR, NF, NR, VAF). `None`
        /// is the built-in transistor, the same as `Some(BjtModel::default())`.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        model: Option<BjtModel>,
    },

    /// MOSFET transistor (3 terminals: gate, drain, source; the body is tied
    /// to the source).
    #[cfg_attr(feature = "serde", serde(rename = "mosfet"))]
    Mosfet {
        /// Unique component identifier.
        id: String,
        /// Terminals `[gate, drain, source]`.
        nodes: [String; 3], // [gate, drain, source]
        /// Channel type (NMOS or PMOS).
        kind: MosfetKind,
        /// Model parameters (threshold voltage, transconductance, etc.).
        #[cfg_attr(feature = "serde", serde(default))]
        params: MosfetParams,
        /// Optional parasitic capacitances (Cgs, Cgd). None = no parasitic caps (default).
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        parasitic_caps: Option<MosfetParasiticCaps>,
        /// A power MOSFET's body diode, as a SPICE diode card: anode at the
        /// source and cathode at the drain for NMOS, the other way round for
        /// PMOS. It conducts when the drain is pulled past the source, as
        /// in a bridge's freewheeling interval. Every analysis solves it as
        /// a [`CircuitElement::Diode`] with id `<id>.body`, which is where
        /// its current is reported. `None` (default) is no body diode.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        body_diode: Option<DiodeModel>,
    },

    /// N-channel or P-channel JFET (Shockley square-law model).
    /// nodes: [gate, drain, source]
    #[cfg_attr(feature = "serde", serde(rename = "jfet"))]
    Jfet {
        /// Unique component identifier.
        id: String,
        /// Terminals `[gate, drain, source]`.
        nodes: [String; 3], // [gate, drain, source]
        /// Channel type (N-channel or P-channel).
        kind: JfetKind,
        /// Drain saturation current at Vgs=0 (A). Typical: 1–20 mA.
        #[cfg_attr(feature = "serde", serde(default = "default_jfet_idss"))]
        idss: f64,
        /// Pinch-off voltage (V, negative for N-channel, e.g. -2.0).
        #[cfg_attr(feature = "serde", serde(default = "default_jfet_vp"))]
        vp: f64,
    },

    /// Voltage-Controlled Voltage Source: V_out = mu * V_control.
    /// nodes: [out+, out-], control_nodes: [ctrl+, ctrl-]
    #[cfg_attr(feature = "serde", serde(rename = "vcvs"))]
    Vcvs {
        /// Unique component identifier.
        id: String,
        /// Output terminals `[out+, out-]`.
        nodes: [String; 2],
        /// Sensing terminals `[ctrl+, ctrl-]`.
        control_nodes: [String; 2],
        /// Voltage gain μ (V/V): `V_out = gain * V_control`.
        gain: f64, // mu
    },

    /// Voltage-Controlled Current Source: I_out = gm * V_control.
    /// nodes: [out_from, out_to], control_nodes: [ctrl+, ctrl-]
    #[cfg_attr(feature = "serde", serde(rename = "vccs"))]
    Vccs {
        /// Unique component identifier.
        id: String,
        /// Output terminals `[out_from, out_to]`.
        nodes: [String; 2],
        /// Sensing terminals `[ctrl+, ctrl-]`.
        control_nodes: [String; 2],
        /// Transconductance gm (A/V): `I_out = gm * V_control`.
        gm: f64,
    },

    /// Current-Controlled Voltage Source: V_out = rm * I_control.
    /// nodes: [out+, out-], control_source: id of controlling voltage source
    #[cfg_attr(feature = "serde", serde(rename = "ccvs"))]
    Ccvs {
        /// Unique component identifier.
        id: String,
        /// Output terminals `[out+, out-]`.
        nodes: [String; 2],
        /// Id of the voltage source whose current is sensed.
        control_source: String,
        /// Transresistance rm (Ω): `V_out = rm * I_control`.
        rm: f64,
    },

    /// Current-Controlled Current Source: I_out = alpha * I_control.
    /// nodes: [out_from, out_to], control_source: id of controlling voltage source
    #[cfg_attr(feature = "serde", serde(rename = "cccs"))]
    Cccs {
        /// Unique component identifier.
        id: String,
        /// Output terminals `[out_from, out_to]`.
        nodes: [String; 2],
        /// Id of the voltage source whose current is sensed.
        control_source: String,
        /// Current gain α (A/A): `I_out = alpha * I_control`.
        alpha: f64,
    },

    /// Pushbutton (momentary switch): identical behaviour to Switch.
    /// When closed = true, current flows; when false, open circuit.
    #[cfg_attr(feature = "serde", serde(rename = "pushbutton"))]
    Pushbutton {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`.
        nodes: [String; 2],
        /// When true the button conducts (pressed); when false it is open.
        closed: bool,
    },

    /// Relay: 4-terminal component with a coil and a contact pair.
    /// nodes: [coil+, coil-, contact1, contact2]
    /// Contact closes when |V_coil| >= pickup_voltage. AC and `.tf` take the
    /// contact as open or closed at the DC bias point.
    #[cfg_attr(feature = "serde", serde(rename = "relay"))]
    Relay {
        /// Unique component identifier.
        id: String,
        /// Terminals `[coil+, coil-, contact1, contact2]`.
        nodes: [String; 4],
        /// Coil resistance (Ω). Default 500.
        #[cfg_attr(feature = "serde", serde(default = "default_relay_coil_resistance"))]
        coil_resistance: f64,
        /// Coil voltage magnitude at which the contact closes (V).
        pickup_voltage: f64,
        /// Coil inductance in Henry, in series with `coil_resistance` in
        /// transient. 0.0 = purely resistive coil.
        #[cfg_attr(feature = "serde", serde(default))]
        inductance: f64,
    },

    /// Photoresistor (LDR): resistance varies logarithmically with light.
    /// light_level = 0.0 → ~1 MΩ (dark), 1.0 → ~1 kΩ (bright).
    #[cfg_attr(feature = "serde", serde(rename = "photoresistor"))]
    Photoresistor {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`.
        nodes: [String; 2],
        /// Incident light level, 0.0 (dark, ~1 MΩ) to 1.0 (bright, ~1 kΩ).
        light_level: f64,
    },

    /// Potentiometer: 3-terminal voltage divider.
    /// nodes: [top, wiper, bottom]
    /// position = 0.0 → wiper at top, 1.0 → wiper at bottom.
    #[cfg_attr(feature = "serde", serde(rename = "potentiometer"))]
    Potentiometer {
        /// Unique component identifier.
        id: String,
        /// Terminals `[top, wiper, bottom]`.
        nodes: [String; 3],
        /// End-to-end resistance (Ω).
        resistance: f64,
        /// Wiper position, 0.0 (at top) to 1.0 (at bottom).
        position: f64,
    },

    /// Zener diode between two nodes: a silicon diode with SPICE reverse
    /// breakdown, `BV = vz` at `IBV` = 5 mA and 1 Ω of series resistance
    /// (the card `D(IS=1e-14 N=1 RS=1 BV=vz IBV=5m)`), so it holds about
    /// `vz` at a small zener's 5 mA test current and rises 1 V per amp past
    /// it. For another knee, use a [`CircuitElement::Diode`] with a
    /// [`DiodeModel`] setting `bv` and `ibv`.
    /// nodes: [anode, cathode]
    #[cfg_attr(feature = "serde", serde(rename = "zener_diode"))]
    ZenerDiode {
        /// Unique component identifier.
        id: String,
        /// Terminals `[anode, cathode]`.
        nodes: [String; 2],
        /// Reverse breakdown (Zener) voltage (V).
        vz: f64,
        /// Junction temperature (K), positive and finite. Default 300.15 K.
        /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// Op-amp, output referred to ground.
    ///
    /// With `model: None` it is the near-ideal VCVS sindr has always used:
    /// gain 1e5, no frequency response, no output resistance, and `v_pos` /
    /// `v_neg` not enforced. With an [`OpAmpModel`] it gets a dominant pole
    /// from the gain-bandwidth product, slew-rate limiting, output
    /// resistance, input offset and bias current, and an output clamped to
    /// `v_pos`/`v_neg`.
    #[cfg_attr(feature = "serde", serde(rename = "op_amp"))]
    OpAmp {
        /// Unique component identifier.
        id: String,
        /// Terminals `[in_plus, in_minus, out]`.
        nodes: [String; 3], // [in_plus, in_minus, out]
        /// Positive supply rail (V): the output's upper limit with a
        /// `model`. Default 15.
        #[cfg_attr(feature = "serde", serde(default = "default_v_pos"))]
        v_pos: f64,
        /// Negative supply rail (V): the output's lower limit with a
        /// `model`. Default -15.
        #[cfg_attr(feature = "serde", serde(default = "default_v_neg"))]
        v_neg: f64,
        /// Macromodel parameters; `None` for the near-ideal VCVS.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        model: Option<OpAmpModel>,
    },

    /// Comparator, output referred to ground: gain 1e5 from `v₊ − v₋` with
    /// no frequency response, its output clamped to `[v_neg, v_pos]`. It is
    /// an op-amp with an [`OpAmpModel`] of that gain and no pole, so it makes
    /// the circuit nonlinear (Newton). No hysteresis, no open-collector output.
    #[cfg_attr(feature = "serde", serde(rename = "comparator"))]
    Comparator {
        /// Unique component identifier.
        id: String,
        /// Terminals `[in_plus, in_minus, out]`.
        nodes: [String; 3], // [in_plus, in_minus, out]
        /// Output high level (V): the output's upper limit. Default 15.
        #[cfg_attr(feature = "serde", serde(default = "default_v_pos"))]
        v_pos: f64,
        /// Output low level (V): the output's lower limit, below
        /// `v_pos`. Default -15.
        #[cfg_attr(feature = "serde", serde(default = "default_v_neg"))]
        v_neg: f64,
    },

    /// Schottky diode: lower forward voltage (~0.3V) than silicon diode.
    /// nodes: [anode, cathode]
    #[cfg_attr(feature = "serde", serde(rename = "schottky_diode"))]
    SchottkyDiode {
        /// Unique component identifier.
        id: String,
        /// Terminals `[anode, cathode]`.
        nodes: [String; 2],
        /// Junction temperature (K), positive and finite. Default 300.15 K.
        /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// NTC thermistor: resistance varies with temperature (passive, no NR).
    /// nodes: [n1, n2]
    #[cfg_attr(feature = "serde", serde(rename = "thermistor"))]
    Thermistor {
        /// Unique component identifier.
        id: String,
        /// Terminals `[n1, n2]`.
        nodes: [String; 2],
        /// Temperature in Kelvin, positive and finite. Default: 298.15 K (25°C).
        #[cfg_attr(feature = "serde", serde(default = "default_temperature"))]
        temperature: f64,
    },

    /// Photodiode: diode + photocurrent source driven by irradiance.
    /// nodes: [anode, cathode]
    #[cfg_attr(feature = "serde", serde(rename = "photodiode"))]
    Photodiode {
        /// Unique component identifier.
        id: String,
        /// Terminals `[anode, cathode]`.
        nodes: [String; 2],
        /// Incident irradiance in W (optical power, not W/m²).
        /// 0.0 = dark, 0.1 = ~50mA photocurrent with default responsivity.
        #[cfg_attr(feature = "serde", serde(default))]
        irradiance: f64,
        /// Junction temperature (K), positive and finite. Default 300.15 K.
        /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// Varactor diode: voltage-dependent junction capacitance.
    /// Open circuit in DC (a 1 TΩ leak); voltage-dependent capacitor in
    /// transient; in AC, its capacitance at the DC bias point.
    /// nodes: [anode, cathode]
    #[cfg_attr(feature = "serde", serde(rename = "varactor"))]
    Varactor {
        /// Unique component identifier.
        id: String,
        /// Terminals `[anode, cathode]`.
        nodes: [String; 2],
        /// Model parameters (zero-bias capacitance, grading, etc.).
        #[cfg_attr(feature = "serde", serde(default))]
        params: sindr_devices::varactor::VaractorParams,
    },

    /// IGBT: MOSFET gate control + BJT output characteristics.
    /// Nonlinear element — requires Newton-Raphson iteration.
    /// nodes: [gate, collector, emitter]
    #[cfg_attr(feature = "serde", serde(rename = "igbt"))]
    Igbt {
        /// Unique component identifier.
        id: String,
        /// Terminals `[gate, collector, emitter]`.
        nodes: [String; 3],
        /// Model parameters (threshold voltage, transconductance, etc.).
        #[cfg_attr(feature = "serde", serde(default))]
        params: sindr_devices::igbt::IgbtParams,
    },

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
    #[cfg_attr(feature = "serde", serde(rename = "transformer"))]
    Transformer {
        /// Unique component identifier.
        id: String,
        /// Terminals `[p1, q1, p2, q2]`: primary `[p1, q1]`, secondary `[p2, q2]`.
        nodes: [String; 4], // [p1, q1, p2, q2]
        /// Primary winding inductance (H).
        l1: f64, // Primary inductance (H)
        /// Secondary winding inductance (H).
        l2: f64, // Secondary inductance (H)
        /// Coupling coefficient (unitless) in `[0, 0.999]`. Default 0.999 (near-ideal).
        #[cfg_attr(feature = "serde", serde(default = "default_coupling"))]
        k: f64, // Coupling coefficient [0, 0.999]. Default 0.999 (near-ideal).
    },

    /// Fuse between two nodes.
    ///
    /// Intact (blown=false): stamped as 0.001 Ω (1 mΩ) — negligible voltage drop.
    /// Blown (blown=true): stamped as 1e9 Ω — effectively open circuit.
    ///
    /// NOTE: Do NOT use 0 Ω — that would cause a singular MNA matrix.
    /// rating is stored for display purposes only (v1 — no auto-blow logic).
    #[cfg_attr(feature = "serde", serde(rename = "fuse"))]
    Fuse {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`.
        nodes: [String; 2],
        /// Current rating in Amperes (display only — no auto-blow logic in v1).
        #[cfg_attr(feature = "serde", serde(default = "default_fuse_rating"))]
        rating: f64,
        /// When true, fuse is blown (open circuit). User-settable, like switch.closed.
        #[cfg_attr(feature = "serde", serde(default))]
        blown: bool,
    },

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
    #[cfg_attr(feature = "serde", serde(rename = "voltage_regulator"))]
    VoltageRegulator {
        /// Unique component identifier.
        id: String,
        /// Terminals `[input, output, gnd]`.
        nodes: [String; 3], // [input, output, gnd]
        /// Regulated output voltage (V) from the ground pin. E.g. 5.0 for a
        /// 7805, or −5.0 for a 7905, whose input sits below ground and whose
        /// output current flows in at the output pin.
        voltage: f64,
        /// Dropout, quiescent current, limit and regulation. `None` is an
        /// ideal regulator that still can't put out more than it gets in.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        model: Option<sindr_devices::regulator::RegulatorParams>,
    },
    /// Single-pole double-throw switch: `nodes: [common, a, b]`. The common
    /// terminal connects to `a` or `b` as [`Switch`](Self::Switch) does
    /// (0.01 Ω), and the other throw is open (1 GΩ). Its result is the path
    /// in use: `voltage_across` from common to the selected throw and the
    /// current from common into it.
    #[cfg_attr(feature = "serde", serde(rename = "switch_spdt"))]
    SwitchSpdt {
        /// Unique component identifier.
        id: String,
        /// Terminals `[common, a, b]`.
        nodes: [String; 3],
        /// Which throw the common terminal is switched to.
        #[cfg_attr(feature = "serde", serde(default))]
        position: SpdtPosition,
    },

    /// Quartz crystal: the Butterworth–Van Dyke model ([`CrystalParams`]).
    /// An open circuit at DC. Its result's `power` is what the motional
    /// resistance dissipates: the drive level a datasheet limits.
    /// `nodes: [a, b]`
    #[cfg_attr(feature = "serde", serde(rename = "crystal"))]
    Crystal {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`.
        nodes: [String; 2],
        /// The equivalent circuit.
        params: CrystalParams,
    },

    /// Ferrite bead ([`FerriteBeadParams`]): its DC resistance at DC,
    /// lossy and inductive at high frequency. Its result's `power` is what
    /// its two resistances dissipate. `nodes: [a, b]`
    #[cfg_attr(feature = "serde", serde(rename = "ferrite_bead"))]
    FerriteBead {
        /// Unique component identifier.
        id: String,
        /// Terminals `[a, b]`.
        nodes: [String; 2],
        /// The equivalent circuit.
        params: FerriteBeadParams,
    },
}

fn default_bf() -> f64 {
    100.0
}

fn default_temperature() -> f64 {
    298.15
}

/// Default junction temperature for IS-bearing devices (K).
/// 300.15 K = 27°C, the SPICE reference temperature for IS.
fn default_junction_temperature() -> f64 {
    300.15
}

fn default_relay_coil_resistance() -> f64 {
    500.0
}

fn default_v_pos() -> f64 {
    15.0
}

fn default_v_neg() -> f64 {
    -15.0
}

fn default_coupling() -> f64 {
    0.999
}

fn default_fuse_rating() -> f64 {
    1.0
}

fn default_jfet_idss() -> f64 {
    10e-3 // 10 mA typical
}

fn default_jfet_vp() -> f64 {
    -2.0 // -2V pinch-off (N-channel default)
}

impl CircuitElement {
    /// Returns a reference to the node array for this component.
    pub fn nodes(&self) -> &[String] {
        match self {
            CircuitElement::Resistor { nodes, .. } => nodes,
            CircuitElement::VoltageSource { nodes, .. } => nodes,
            CircuitElement::CurrentSource { nodes, .. } => nodes,
            CircuitElement::Switch { nodes, .. } => nodes,
            CircuitElement::Capacitor { nodes, .. } => nodes,
            CircuitElement::Inductor { nodes, .. } => nodes,
            CircuitElement::Diode { nodes, .. } => nodes,
            CircuitElement::Led { nodes, .. } => nodes,
            CircuitElement::Bjt { nodes, .. } => nodes,
            CircuitElement::Mosfet { nodes, .. } => nodes,
            CircuitElement::Jfet { nodes, .. } => nodes,
            CircuitElement::Vcvs { nodes, .. } => nodes,
            CircuitElement::Vccs { nodes, .. } => nodes,
            CircuitElement::Ccvs { nodes, .. } => nodes,
            CircuitElement::Cccs { nodes, .. } => nodes,
            CircuitElement::Pushbutton { nodes, .. } => nodes,
            CircuitElement::Relay { nodes, .. } => nodes,
            CircuitElement::Photoresistor { nodes, .. } => nodes,
            CircuitElement::Potentiometer { nodes, .. } => nodes,
            CircuitElement::ZenerDiode { nodes, .. } => nodes,
            CircuitElement::OpAmp { nodes, .. } => nodes,
            CircuitElement::Comparator { nodes, .. } => nodes,
            CircuitElement::SchottkyDiode { nodes, .. } => nodes,
            CircuitElement::Thermistor { nodes, .. } => nodes,
            CircuitElement::Battery { nodes, .. } => nodes,
            CircuitElement::Photodiode { nodes, .. } => nodes,
            CircuitElement::Varactor { nodes, .. } => nodes,
            CircuitElement::Igbt { nodes, .. } => nodes,
            CircuitElement::Transformer { nodes, .. } => nodes,
            CircuitElement::Fuse { nodes, .. } => nodes,
            CircuitElement::VoltageRegulator { nodes, .. } => nodes,
            CircuitElement::SwitchSpdt { nodes, .. } => nodes,
            CircuitElement::Crystal { nodes, .. } => nodes,
            CircuitElement::FerriteBead { nodes, .. } => nodes,
        }
    }

    /// Returns all node names referenced by this component, including control nodes.
    pub fn all_nodes(&self) -> Vec<&String> {
        match self {
            CircuitElement::Vcvs {
                nodes,
                control_nodes,
                ..
            } => {
                vec![&nodes[0], &nodes[1], &control_nodes[0], &control_nodes[1]]
            }
            CircuitElement::Vccs {
                nodes,
                control_nodes,
                ..
            } => {
                vec![&nodes[0], &nodes[1], &control_nodes[0], &control_nodes[1]]
            }
            CircuitElement::Ccvs { nodes, .. } => vec![&nodes[0], &nodes[1]],
            CircuitElement::Cccs { nodes, .. } => vec![&nodes[0], &nodes[1]],
            _ => self.nodes().iter().collect(),
        }
    }

    /// Returns the component identifier.
    pub fn id(&self) -> &str {
        match self {
            CircuitElement::Resistor { id, .. } => id,
            CircuitElement::VoltageSource { id, .. } => id,
            CircuitElement::CurrentSource { id, .. } => id,
            CircuitElement::Switch { id, .. } => id,
            CircuitElement::Capacitor { id, .. } => id,
            CircuitElement::Inductor { id, .. } => id,
            CircuitElement::Diode { id, .. } => id,
            CircuitElement::Led { id, .. } => id,
            CircuitElement::Bjt { id, .. } => id,
            CircuitElement::Mosfet { id, .. } => id,
            CircuitElement::Jfet { id, .. } => id,
            CircuitElement::Vcvs { id, .. } => id,
            CircuitElement::Vccs { id, .. } => id,
            CircuitElement::Ccvs { id, .. } => id,
            CircuitElement::Cccs { id, .. } => id,
            CircuitElement::Pushbutton { id, .. } => id,
            CircuitElement::Relay { id, .. } => id,
            CircuitElement::Photoresistor { id, .. } => id,
            CircuitElement::Potentiometer { id, .. } => id,
            CircuitElement::ZenerDiode { id, .. } => id,
            CircuitElement::OpAmp { id, .. } => id,
            CircuitElement::Comparator { id, .. } => id,
            CircuitElement::SchottkyDiode { id, .. } => id,
            CircuitElement::Thermistor { id, .. } => id,
            CircuitElement::Battery { id, .. } => id,
            CircuitElement::Photodiode { id, .. } => id,
            CircuitElement::Varactor { id, .. } => id,
            CircuitElement::Igbt { id, .. } => id,
            CircuitElement::Transformer { id, .. } => id,
            CircuitElement::Fuse { id, .. } => id,
            CircuitElement::VoltageRegulator { id, .. } => id,
            CircuitElement::SwitchSpdt { id, .. } => id,
            CircuitElement::Crystal { id, .. } => id,
            CircuitElement::FerriteBead { id, .. } => id,
        }
    }

    /// Extra MNA unknowns the element adds after the node voltages: one
    /// branch current for a voltage-type source, two for a transformer
    /// (one per winding), and two for an op-amp with an [`OpAmpModel`] or a
    /// comparator (its output current, then its internal gain-stage voltage).
    pub fn branch_slots(&self) -> usize {
        match self {
            CircuitElement::OpAmp { model: Some(_), .. } | CircuitElement::Comparator { .. } => 2,
            CircuitElement::VoltageSource { .. }
            | CircuitElement::Vcvs { .. }
            | CircuitElement::Ccvs { .. }
            | CircuitElement::OpAmp { .. }
            | CircuitElement::VoltageRegulator { .. } => 1,
            CircuitElement::Transformer { .. } => 2,
            _ => 0,
        }
    }

    /// Mutable [`nodes`](Self::nodes), for renaming nets.
    pub fn nodes_mut(&mut self) -> &mut [String] {
        match self {
            CircuitElement::Resistor { nodes, .. } => nodes,
            CircuitElement::VoltageSource { nodes, .. } => nodes,
            CircuitElement::CurrentSource { nodes, .. } => nodes,
            CircuitElement::Switch { nodes, .. } => nodes,
            CircuitElement::Capacitor { nodes, .. } => nodes,
            CircuitElement::Inductor { nodes, .. } => nodes,
            CircuitElement::Diode { nodes, .. } => nodes,
            CircuitElement::Led { nodes, .. } => nodes,
            CircuitElement::Bjt { nodes, .. } => nodes,
            CircuitElement::Mosfet { nodes, .. } => nodes,
            CircuitElement::Jfet { nodes, .. } => nodes,
            CircuitElement::Vcvs { nodes, .. } => nodes,
            CircuitElement::Vccs { nodes, .. } => nodes,
            CircuitElement::Ccvs { nodes, .. } => nodes,
            CircuitElement::Cccs { nodes, .. } => nodes,
            CircuitElement::Pushbutton { nodes, .. } => nodes,
            CircuitElement::Relay { nodes, .. } => nodes,
            CircuitElement::Photoresistor { nodes, .. } => nodes,
            CircuitElement::Potentiometer { nodes, .. } => nodes,
            CircuitElement::ZenerDiode { nodes, .. } => nodes,
            CircuitElement::OpAmp { nodes, .. } => nodes,
            CircuitElement::Comparator { nodes, .. } => nodes,
            CircuitElement::SchottkyDiode { nodes, .. } => nodes,
            CircuitElement::Thermistor { nodes, .. } => nodes,
            CircuitElement::Battery { nodes, .. } => nodes,
            CircuitElement::Photodiode { nodes, .. } => nodes,
            CircuitElement::Varactor { nodes, .. } => nodes,
            CircuitElement::Igbt { nodes, .. } => nodes,
            CircuitElement::Transformer { nodes, .. } => nodes,
            CircuitElement::Fuse { nodes, .. } => nodes,
            CircuitElement::VoltageRegulator { nodes, .. } => nodes,
            CircuitElement::SwitchSpdt { nodes, .. } => nodes,
            CircuitElement::Crystal { nodes, .. } => nodes,
            CircuitElement::FerriteBead { nodes, .. } => nodes,
        }
    }

    /// Mutable [`id`](Self::id), for renaming.
    pub fn id_mut(&mut self) -> &mut String {
        match self {
            CircuitElement::Resistor { id, .. } => id,
            CircuitElement::VoltageSource { id, .. } => id,
            CircuitElement::CurrentSource { id, .. } => id,
            CircuitElement::Switch { id, .. } => id,
            CircuitElement::Capacitor { id, .. } => id,
            CircuitElement::Inductor { id, .. } => id,
            CircuitElement::Diode { id, .. } => id,
            CircuitElement::Led { id, .. } => id,
            CircuitElement::Bjt { id, .. } => id,
            CircuitElement::Mosfet { id, .. } => id,
            CircuitElement::Jfet { id, .. } => id,
            CircuitElement::Vcvs { id, .. } => id,
            CircuitElement::Vccs { id, .. } => id,
            CircuitElement::Ccvs { id, .. } => id,
            CircuitElement::Cccs { id, .. } => id,
            CircuitElement::Pushbutton { id, .. } => id,
            CircuitElement::Relay { id, .. } => id,
            CircuitElement::Photoresistor { id, .. } => id,
            CircuitElement::Potentiometer { id, .. } => id,
            CircuitElement::ZenerDiode { id, .. } => id,
            CircuitElement::OpAmp { id, .. } => id,
            CircuitElement::Comparator { id, .. } => id,
            CircuitElement::SchottkyDiode { id, .. } => id,
            CircuitElement::Thermistor { id, .. } => id,
            CircuitElement::Battery { id, .. } => id,
            CircuitElement::Photodiode { id, .. } => id,
            CircuitElement::Varactor { id, .. } => id,
            CircuitElement::Igbt { id, .. } => id,
            CircuitElement::Transformer { id, .. } => id,
            CircuitElement::Fuse { id, .. } => id,
            CircuitElement::VoltageRegulator { id, .. } => id,
            CircuitElement::SwitchSpdt { id, .. } => id,
            CircuitElement::Crystal { id, .. } => id,
            CircuitElement::FerriteBead { id, .. } => id,
        }
    }

    /// The control-net pair of a VCVS or VCCS.
    pub fn control_nodes_mut(&mut self) -> Option<&mut [String; 2]> {
        match self {
            CircuitElement::Vcvs { control_nodes, .. }
            | CircuitElement::Vccs { control_nodes, .. } => Some(control_nodes),
            _ => None,
        }
    }

    /// The controlling source id of a CCVS or CCCS.
    pub fn control_source_mut(&mut self) -> Option<&mut String> {
        match self {
            CircuitElement::Ccvs { control_source, .. }
            | CircuitElement::Cccs { control_source, .. } => Some(control_source),
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
///
/// let circuit = Circuit::new(
///     "0",
///     vec![
///         CircuitElement::VoltageSource {
///             id: "V1".into(),
///             nodes: ["n1".into(), "0".into()],
///             voltage: 9.0,
///             waveform: None,
///         },
///         CircuitElement::Resistor {
///             id: "R1".into(),
///             nodes: ["n1".into(), "0".into()],
///             resistance: 1_000.0,
///         },
///     ],
/// );
/// assert!(circuit.is_flat());
/// ```
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
        self.components.iter().any(|c| {
            matches!(
                c,
                CircuitElement::OpAmp { model: Some(_), .. }
                    | CircuitElement::Comparator { .. }
                    | CircuitElement::Inductor {
                        saturation: Some(_),
                        ..
                    }
                    | CircuitElement::Diode { .. }
                    | CircuitElement::Led { .. }
                    | CircuitElement::Bjt { .. }
                    | CircuitElement::Mosfet { .. }
                    | CircuitElement::Relay { .. }
                    | CircuitElement::ZenerDiode { .. }
                    | CircuitElement::SchottkyDiode { .. }
                    | CircuitElement::Photodiode { .. }
                    | CircuitElement::Igbt { .. }
                    | CircuitElement::Jfet { .. }
                    | CircuitElement::VoltageRegulator { .. } // Thermistor is passive (temperature-dependent resistance, no NR)
                                                              // Varactor is passive in DC (open circuit), no NR needed
            )
        })
    }

    /// Returns true if the circuit contains any reactive elements (Capacitor, Inductor, or
    /// Relay with inductance > 0, or Transformer, or BJT/MOSFET with non-zero parasitic caps,
    /// or a diode whose model stores charge).
    pub fn has_reactive_elements(&self) -> bool {
        self.components.iter().any(|c| match c {
            CircuitElement::Capacitor { .. }
            | CircuitElement::Inductor { .. }
            | CircuitElement::Varactor { .. }
            | CircuitElement::Transformer { .. } => true,
            CircuitElement::Relay { inductance, .. } if *inductance > 0.0 => true,
            CircuitElement::Battery { params, .. } => params.rc.is_some(),
            CircuitElement::Bjt {
                parasitic_caps: Some(caps),
                ..
            } if caps.cbe > 0.0 || caps.cbc > 0.0 => true,
            CircuitElement::Mosfet {
                parasitic_caps: Some(caps),
                ..
            } if caps.cgs > 0.0 || caps.cgd > 0.0 => true,
            CircuitElement::Diode {
                model: Some(model), ..
            } => model.stores_charge(),
            _ => false,
        })
    }

    /// Returns true if any source has a time-varying waveform.
    pub fn has_waveform_sources(&self) -> bool {
        self.components.iter().any(|c| match c {
            CircuitElement::VoltageSource { waveform, .. }
            | CircuitElement::CurrentSource { waveform, .. } => waveform.is_some(),
            _ => false,
        })
    }

    /// Find the branch index for a voltage source by its ID.
    /// Returns None if not found.
    pub fn vsource_branch_index(&self, source_id: &str) -> Option<usize> {
        let mut idx = 0;
        for component in &self.components {
            let slots = component.branch_slots();
            let has_current = matches!(
                component,
                CircuitElement::VoltageSource { .. }
                    | CircuitElement::Vcvs { .. }
                    | CircuitElement::Ccvs { .. }
                    | CircuitElement::OpAmp { .. }
                    | CircuitElement::Comparator { .. }
                    | CircuitElement::VoltageRegulator { .. }
            );
            if has_current && component.id() == source_id {
                return Some(idx);
            }
            idx += slots;
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn has_reactive_elements_true_with_capacitor() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["n1".into(), "0".into()],
                capacitance: 100e-6,
            }],
            ..Default::default()
        };
        assert!(circuit.has_reactive_elements());
    }

    #[test]
    fn has_reactive_elements_true_with_inductor() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Inductor {
                id: "L1".into(),
                nodes: ["n1".into(), "0".into()],
                inductance: 10e-3,
                saturation: None,
            }],
            ..Default::default()
        };
        assert!(circuit.has_reactive_elements());
    }

    #[test]
    fn has_reactive_elements_false_resistor_only() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1000.0,
            }],
            ..Default::default()
        };
        assert!(!circuit.has_reactive_elements());
    }

    #[test]
    fn diode_storing_charge_is_reactive() {
        let diode = |model: Option<DiodeModel>| Circuit {
            components: vec![CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model,
            }],
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
            components: vec![CircuitElement::Bjt {
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
            }],
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
            components: vec![CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "e".into()],
                kind: sindr_devices::bjt::BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }],
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
            components: vec![CircuitElement::Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "s".into()],
                kind: sindr_devices::mosfet::MosfetKind::Nmos,
                params: sindr_devices::mosfet::MosfetParams::default(),
                parasitic_caps: Some(MosfetParasiticCaps {
                    cgs: 100e-12,
                    cgd: 50e-12,
                }),
                body_diode: None,
            }],
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
            CircuitElement::Bjt { bf, .. } => assert_eq!(*bf, 100.0),
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
            CircuitElement::Diode { model: None, .. }
        ));
        match &circuit.components[1] {
            CircuitElement::Diode { model, .. } => assert_eq!(
                *model,
                Some(DiodeModel {
                    rs: 0.5,
                    ..DiodeModel::default()
                })
            ),
            other => panic!("expected Diode, got {other:?}"),
        }
        match &circuit.components[2] {
            CircuitElement::Bjt { model, .. } => assert_eq!(
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
            components: vec![CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "e".into()],
                kind: sindr_devices::bjt::BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }],
            ..Default::default()
        };
        assert!(circuit.has_nonlinear_elements());
    }

    #[test]
    fn relay_has_nonlinear_returns_true() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Relay {
                id: "K1".into(),
                nodes: ["coil_p".into(), "0".into(), "c1".into(), "c2".into()],
                coil_resistance: 500.0,
                pickup_voltage: 5.0,
                inductance: 0.0,
            }],
            ..Default::default()
        };
        assert!(circuit.has_nonlinear_elements());
    }

    #[test]
    fn relay_with_inductance_has_reactive_elements() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Relay {
                id: "K1".into(),
                nodes: ["coil_p".into(), "0".into(), "c1".into(), "c2".into()],
                coil_resistance: 500.0,
                pickup_voltage: 5.0,
                inductance: 0.1,
            }],
            ..Default::default()
        };
        assert!(circuit.has_reactive_elements());
    }

    #[test]
    fn relay_without_inductance_not_reactive() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Relay {
                id: "K1".into(),
                nodes: ["coil_p".into(), "0".into(), "c1".into(), "c2".into()],
                coil_resistance: 500.0,
                pickup_voltage: 5.0,
                inductance: 0.0,
            }],
            ..Default::default()
        };
        assert!(!circuit.has_reactive_elements());
    }

    #[test]
    fn pushbutton_has_nonlinear_returns_false() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Pushbutton {
                id: "PB1".into(),
                nodes: ["n1".into(), "0".into()],
                closed: true,
            }],
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
            CircuitElement::Relay {
                coil_resistance, ..
            } => {
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
            CircuitElement::ZenerDiode { id, vz, .. } => {
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
                CircuitElement::OpAmp {
                    id: "U1".into(),
                    nodes: ["in_p".into(), "in_m".into(), "out".into()],
                    v_pos: 15.0,
                    v_neg: -15.0,
                    model: None,
                },
                CircuitElement::Comparator {
                    id: "U2".into(),
                    nodes: ["in_p".into(), "in_m".into(), "out2".into()],
                    v_pos: 5.0,
                    v_neg: 0.0,
                },
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
            CircuitElement::OpAmp { v_pos, v_neg, .. } => {
                assert_eq!(*v_pos, 15.0);
                assert_eq!(*v_neg, -15.0);
            }
            _ => panic!("Expected OpAmp variant"),
        }
    }
}
