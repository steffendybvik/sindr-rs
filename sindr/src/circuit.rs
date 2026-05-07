use sindr_devices::bjt::BjtKind;
use sindr_devices::jfet::JfetKind;
use sindr_devices::mosfet::{MosfetKind, MosfetParams};

use crate::waveform::Waveform;

/// BJT parasitic capacitances. All values in Farads. Default 0.0 disables each cap.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default)]
pub struct BjtParasiticCaps {
    /// Base-Emitter junction capacitance (F). Typical: 5–50 pF.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cbe: f64,
    /// Base-Collector junction capacitance (F). Typical: 1–10 pF.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cbc: f64,
}

/// MOSFET gate parasitic capacitances. All values in Farads.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default)]
pub struct MosfetParasiticCaps {
    /// Gate-Source capacitance (F). Typical: 100–500 pF for power MOSFETs.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cgs: f64,
    /// Gate-Drain capacitance (Miller capacitance) (F). Typical: 10–100 pF.
    #[cfg_attr(feature = "serde", serde(default))]
    pub cgd: f64,
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
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", serde(tag = "type"))]
pub enum CircuitElement {
    /// Resistor between two nodes.
    #[cfg_attr(feature = "serde", serde(rename = "resistor"))]
    Resistor {
        id: String,
        nodes: [String; 2],
        resistance: f64,
    },

    /// Independent voltage source. `nodes[0]` is the positive terminal,
    /// `nodes[1]` is the negative terminal.
    #[cfg_attr(feature = "serde", serde(rename = "voltage_source"))]
    VoltageSource {
        id: String,
        nodes: [String; 2],
        voltage: f64,
        #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
        waveform: Option<Waveform>,
    },

    /// Independent current source. Current flows from `nodes[0]` toward
    /// `nodes[1]`.
    #[cfg_attr(feature = "serde", serde(rename = "current_source"))]
    CurrentSource {
        id: String,
        nodes: [String; 2],
        current: f64,
        #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
        waveform: Option<Waveform>,
    },

    /// Switch between two nodes. Modeled as a resistor with very low (closed)
    /// or very high (open) resistance for DC analysis.
    #[cfg_attr(feature = "serde", serde(rename = "switch"))]
    Switch {
        id: String,
        nodes: [String; 2],
        closed: bool,
    },

    /// Capacitor between two nodes. Stub for DC analysis (open circuit).
    #[cfg_attr(feature = "serde", serde(rename = "capacitor"))]
    Capacitor {
        id: String,
        nodes: [String; 2],
        capacitance: f64,
    },

    /// Inductor between two nodes. Stub for DC analysis (open circuit).
    #[cfg_attr(feature = "serde", serde(rename = "inductor"))]
    Inductor {
        id: String,
        nodes: [String; 2],
        inductance: f64,
    },

    /// Diode between two nodes. Stub for DC analysis (open circuit).
    #[cfg_attr(feature = "serde", serde(rename = "diode"))]
    Diode {
        id: String,
        nodes: [String; 2],
        /// Junction temperature (K). Default 300.15 K. Used for IS temperature scaling.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// LED between two nodes. Stub for DC analysis (open circuit).
    #[cfg_attr(feature = "serde", serde(rename = "led"))]
    Led {
        id: String,
        nodes: [String; 2],
        color: String,
        /// Junction temperature (K). Default 300.15 K. Used for IS temperature scaling.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// BJT transistor (3 terminals). Stub for DC analysis until Phase 18.
    #[cfg_attr(feature = "serde", serde(rename = "bjt"))]
    Bjt {
        id: String,
        nodes: [String; 3], // [base, collector, emitter]
        kind: BjtKind,
        #[cfg_attr(feature = "serde", serde(default = "default_bf"))]
        bf: f64,
        /// Junction temperature (K). Default 300.15 K. Used for IS temperature scaling.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
        /// Optional parasitic capacitances (Cbe, Cbc). None = no parasitic caps (default).
        #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
        parasitic_caps: Option<BjtParasiticCaps>,
    },

    /// MOSFET transistor (3 terminals: gate, drain, source).
    #[cfg_attr(feature = "serde", serde(rename = "mosfet"))]
    Mosfet {
        id: String,
        nodes: [String; 3], // [gate, drain, source]
        kind: MosfetKind,
        #[cfg_attr(feature = "serde", serde(default))]
        params: MosfetParams,
        /// Optional parasitic capacitances (Cgs, Cgd). None = no parasitic caps (default).
        #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
        parasitic_caps: Option<MosfetParasiticCaps>,
    },

    /// N-channel or P-channel JFET (Shockley square-law model).
    /// nodes: [gate, drain, source]
    #[cfg_attr(feature = "serde", serde(rename = "jfet"))]
    Jfet {
        id: String,
        nodes: [String; 3], // [gate, drain, source]
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
        id: String,
        nodes: [String; 2],
        control_nodes: [String; 2],
        gain: f64, // mu
    },

    /// Voltage-Controlled Current Source: I_out = gm * V_control.
    /// nodes: [out_from, out_to], control_nodes: [ctrl+, ctrl-]
    #[cfg_attr(feature = "serde", serde(rename = "vccs"))]
    Vccs {
        id: String,
        nodes: [String; 2],
        control_nodes: [String; 2],
        gm: f64,
    },

    /// Current-Controlled Voltage Source: V_out = rm * I_control.
    /// nodes: [out+, out-], control_source: id of controlling voltage source
    #[cfg_attr(feature = "serde", serde(rename = "ccvs"))]
    Ccvs {
        id: String,
        nodes: [String; 2],
        control_source: String,
        rm: f64,
    },

    /// Current-Controlled Current Source: I_out = alpha * I_control.
    /// nodes: [out_from, out_to], control_source: id of controlling voltage source
    #[cfg_attr(feature = "serde", serde(rename = "cccs"))]
    Cccs {
        id: String,
        nodes: [String; 2],
        control_source: String,
        alpha: f64,
    },

    /// Pushbutton (momentary switch): identical behaviour to Switch.
    /// When closed = true, current flows; when false, open circuit.
    #[cfg_attr(feature = "serde", serde(rename = "pushbutton"))]
    Pushbutton {
        id: String,
        nodes: [String; 2],
        closed: bool,
    },

    /// Relay: 4-terminal component with a coil and a contact pair.
    /// nodes: [coil+, coil-, contact1, contact2]
    /// Contact closes when |V_coil| >= pickup_voltage.
    #[cfg_attr(feature = "serde", serde(rename = "relay"))]
    Relay {
        id: String,
        nodes: [String; 4],
        #[cfg_attr(feature = "serde", serde(default = "default_relay_coil_resistance"))]
        coil_resistance: f64,
        pickup_voltage: f64,
        /// Coil inductance in Henry. 0.0 = purely resistive coil (backward compatible).
        #[cfg_attr(feature = "serde", serde(default))]
        inductance: f64,
    },

    /// Photoresistor (LDR): resistance varies logarithmically with light.
    /// light_level = 0.0 → ~1 MΩ (dark), 1.0 → ~1 kΩ (bright).
    #[cfg_attr(feature = "serde", serde(rename = "photoresistor"))]
    Photoresistor {
        id: String,
        nodes: [String; 2],
        light_level: f64,
    },

    /// Potentiometer: 3-terminal voltage divider.
    /// nodes: [top, wiper, bottom]
    /// position = 0.0 → wiper at top, 1.0 → wiper at bottom.
    #[cfg_attr(feature = "serde", serde(rename = "potentiometer"))]
    Potentiometer {
        id: String,
        nodes: [String; 3],
        resistance: f64,
        position: f64,
    },

    /// Zener diode between two nodes.
    /// Behaves as a normal diode in forward bias; clamps voltage at `vz` in reverse breakdown.
    /// nodes: [anode, cathode]
    #[cfg_attr(feature = "serde", serde(rename = "zener_diode"))]
    ZenerDiode {
        id: String,
        nodes: [String; 2],
        vz: f64,
        /// Junction temperature (K). Default 300.15 K. Used for IS temperature scaling.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// Ideal op-amp (modelled as high-gain VCVS, gain=1e5).
    /// nodes: [in_plus, in_minus, out]
    #[cfg_attr(feature = "serde", serde(rename = "op_amp"))]
    OpAmp {
        id: String,
        nodes: [String; 3], // [in_plus, in_minus, out]
        #[cfg_attr(feature = "serde", serde(default = "default_v_pos"))]
        v_pos: f64,
        #[cfg_attr(feature = "serde", serde(default = "default_v_neg"))]
        v_neg: f64,
    },

    /// Ideal comparator (same VCVS stamp as OpAmp; output saturates to supply rails).
    /// nodes: [in_plus, in_minus, out]
    #[cfg_attr(feature = "serde", serde(rename = "comparator"))]
    Comparator {
        id: String,
        nodes: [String; 3], // [in_plus, in_minus, out]
        #[cfg_attr(feature = "serde", serde(default = "default_v_pos"))]
        v_pos: f64,
        #[cfg_attr(feature = "serde", serde(default = "default_v_neg"))]
        v_neg: f64,
    },

    /// Schottky diode: lower forward voltage (~0.3V) than silicon diode.
    /// nodes: [anode, cathode]
    #[cfg_attr(feature = "serde", serde(rename = "schottky_diode"))]
    SchottkyDiode {
        id: String,
        nodes: [String; 2],
        /// Junction temperature (K). Default 300.15 K. Used for IS temperature scaling.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// NTC thermistor: resistance varies with temperature (passive, no NR).
    /// nodes: [n1, n2]
    #[cfg_attr(feature = "serde", serde(rename = "thermistor"))]
    Thermistor {
        id: String,
        nodes: [String; 2],
        /// Temperature in Kelvin. Default: 298.15 K (25°C).
        #[cfg_attr(feature = "serde", serde(default = "default_temperature"))]
        temperature: f64,
    },

    /// Photodiode: diode + photocurrent source driven by irradiance.
    /// nodes: [anode, cathode]
    #[cfg_attr(feature = "serde", serde(rename = "photodiode"))]
    Photodiode {
        id: String,
        nodes: [String; 2],
        /// Incident irradiance in W (optical power, not W/m²).
        /// 0.0 = dark, 0.1 = ~50mA photocurrent with default responsivity.
        #[cfg_attr(feature = "serde", serde(default))]
        irradiance: f64,
        /// Junction temperature (K). Default 300.15 K. Used for IS temperature scaling.
        #[cfg_attr(feature = "serde", serde(default = "default_junction_temperature"))]
        temperature: f64,
    },

    /// Varactor diode: voltage-dependent junction capacitance.
    /// Open circuit in DC; voltage-dependent capacitor in transient.
    /// nodes: [anode, cathode]
    #[cfg_attr(feature = "serde", serde(rename = "varactor"))]
    Varactor {
        id: String,
        nodes: [String; 2],
        #[cfg_attr(feature = "serde", serde(default))]
        params: sindr_devices::varactor::VaractorParams,
    },

    /// IGBT: MOSFET gate control + BJT output characteristics.
    /// Nonlinear element — requires Newton-Raphson iteration.
    /// nodes: [gate, collector, emitter]
    #[cfg_attr(feature = "serde", serde(rename = "igbt"))]
    Igbt {
        id: String,
        nodes: [String; 3],
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
    /// In DC analysis: both windings stamped as short circuits (zero resistance).
    /// In transient: coupled inductor Backward Euler stamp with 2 branch current unknowns.
    #[cfg_attr(feature = "serde", serde(rename = "transformer"))]
    Transformer {
        id: String,
        nodes: [String; 4], // [p1, q1, p2, q2]
        l1: f64,            // Primary inductance (H)
        l2: f64,            // Secondary inductance (H)
        #[cfg_attr(feature = "serde", serde(default = "default_coupling"))]
        k: f64,             // Coupling coefficient [0, 0.999]. Default 0.999 (near-ideal).
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
        id: String,
        nodes: [String; 2],
        /// Current rating in Amperes (display only — no auto-blow logic in v1).
        #[cfg_attr(feature = "serde", serde(default = "default_fuse_rating"))]
        rating: f64,
        /// When true, fuse is blown (open circuit). User-settable, like switch.closed.
        #[cfg_attr(feature = "serde", serde(default))]
        blown: bool,
    },

    /// Ideal linear voltage regulator — holds output node at a fixed regulated voltage.
    ///
    /// Modelled as an ideal voltage source between output (`nodes[1]`) and gnd (`nodes[2]`).
    /// The input node (`nodes[0]`) is wiring-only; it is not stamped into MNA.
    ///
    /// `nodes: [input, output, gnd]`
    #[cfg_attr(feature = "serde", serde(rename = "voltage_regulator"))]
    VoltageRegulator {
        id: String,
        nodes: [String; 3], // [input, output, gnd]
        /// Regulated output voltage (V). E.g. 5.0 for a 7805.
        voltage: f64,
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
            CircuitElement::Photodiode { nodes, .. } => nodes,
            CircuitElement::Varactor { nodes, .. } => nodes,
            CircuitElement::Igbt { nodes, .. } => nodes,
            CircuitElement::Transformer { nodes, .. } => nodes,
            CircuitElement::Fuse { nodes, .. } => nodes,
            CircuitElement::VoltageRegulator { nodes, .. } => nodes,
        }
    }

    /// Returns all node names referenced by this component, including control nodes.
    pub fn all_nodes(&self) -> Vec<&String> {
        match self {
            CircuitElement::Vcvs { nodes, control_nodes, .. } => {
                vec![&nodes[0], &nodes[1], &control_nodes[0], &control_nodes[1]]
            }
            CircuitElement::Vccs { nodes, control_nodes, .. } => {
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
            CircuitElement::Photodiode { id, .. } => id,
            CircuitElement::Varactor { id, .. } => id,
            CircuitElement::Igbt { id, .. } => id,
            CircuitElement::Transformer { id, .. } => id,
            CircuitElement::Fuse { id, .. } => id,
            CircuitElement::VoltageRegulator { id, .. } => id,
        }
    }
}

/// A complete circuit description ready for simulation.
///
/// A circuit is a flat list of [`CircuitElement`]s plus the name of the
/// ground node. Components share a node simply by referencing the same
/// node-name string. Pass to [`solve_circuit`](crate::solve_circuit) to run.
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement};
///
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
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
/// };
/// ```
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct Circuit {
    /// All components in the circuit. Order has no semantic meaning.
    pub components: Vec<CircuitElement>,
    /// Name of the reference (0 V) node. Must match the node string used by
    /// at least one component (typically `"0"` or `"gnd"`).
    pub ground_node: String,
}

impl Circuit {
    /// Count the number of independent voltage sources in the circuit.
    /// Also counts VCVS, CCVS, OpAmp, and Comparator which need branch currents.
    /// Transformer adds 2 branch current unknowns (one per winding).
    pub fn count_voltage_sources(&self) -> usize {
        self.components
            .iter()
            .map(|c| match c {
                CircuitElement::VoltageSource { .. }
                | CircuitElement::Vcvs { .. }
                | CircuitElement::Ccvs { .. }
                | CircuitElement::OpAmp { .. }
                | CircuitElement::Comparator { .. }
                | CircuitElement::VoltageRegulator { .. } => 1,
                CircuitElement::Transformer { .. } => 2, // Two branch current unknowns (I_L1, I_L2)
                _ => 0,
            })
            .sum()
    }

    /// Returns true if the circuit contains any nonlinear elements.
    pub fn has_nonlinear_elements(&self) -> bool {
        self.components.iter().any(|c| {
            matches!(
                c,
                CircuitElement::Diode { .. }
                    | CircuitElement::Led { .. }
                    | CircuitElement::Bjt { .. }
                    | CircuitElement::Mosfet { .. }
                    | CircuitElement::Relay { .. }
                    | CircuitElement::ZenerDiode { .. }
                    | CircuitElement::SchottkyDiode { .. }
                    | CircuitElement::Photodiode { .. }
                    | CircuitElement::Igbt { .. }
                    | CircuitElement::Jfet { .. }
                // Thermistor is passive (temperature-dependent resistance, no NR)
                // Varactor is passive in DC (open circuit), no NR needed
            )
        })
    }

    /// Returns true if the circuit contains any reactive elements (Capacitor, Inductor, or
    /// Relay with inductance > 0, or Transformer, or BJT/MOSFET with non-zero parasitic caps).
    pub fn has_reactive_elements(&self) -> bool {
        self.components.iter().any(|c| match c {
            CircuitElement::Capacitor { .. }
            | CircuitElement::Inductor { .. }
            | CircuitElement::Varactor { .. }
            | CircuitElement::Transformer { .. } => true,
            CircuitElement::Relay { inductance, .. } if *inductance > 0.0 => true,
            CircuitElement::Bjt { parasitic_caps: Some(caps), .. }
                if caps.cbe > 0.0 || caps.cbc > 0.0 => true,
            CircuitElement::Mosfet { parasitic_caps: Some(caps), .. }
                if caps.cgs > 0.0 || caps.cgd > 0.0 => true,
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
            match component {
                CircuitElement::VoltageSource { id, .. }
                | CircuitElement::Vcvs { id, .. }
                | CircuitElement::Ccvs { id, .. }
                | CircuitElement::OpAmp { id, .. }
                | CircuitElement::Comparator { id, .. }
                | CircuitElement::VoltageRegulator { id, .. } => {
                    if id == source_id {
                        return Some(idx);
                    }
                    idx += 1;
                }
                CircuitElement::Transformer { .. } => {
                    idx += 2; // Transformer uses 2 branch current slots
                }
                _ => {}
            }
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
            }],
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
        };
        assert!(!circuit.has_reactive_elements());
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
                parasitic_caps: Some(BjtParasiticCaps { cbe: 10e-12, cbc: 0.0 }),
            }],
        };
        assert!(circuit.has_reactive_elements(), "BJT with cbe>0 should be reactive");
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
            }],
        };
        // BJT without parasitic caps is not reactive (nonlinear, but not reactive)
        assert!(!circuit.has_reactive_elements(), "BJT without parasitic caps should not be reactive");
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
                parasitic_caps: Some(MosfetParasiticCaps { cgs: 100e-12, cgd: 50e-12 }),
            }],
        };
        assert!(circuit.has_reactive_elements(), "MOSFET with cgs>0 should be reactive");
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
            }],
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
            CircuitElement::Relay { coil_resistance, .. } => {
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
                },
                CircuitElement::Comparator {
                    id: "U2".into(),
                    nodes: ["in_p".into(), "in_m".into(), "out2".into()],
                    v_pos: 5.0,
                    v_neg: 0.0,
                },
            ],
        };
        // Both OpAmp and Comparator count as voltage sources (VCVS branch)
        assert_eq!(circuit.count_voltage_sources(), 2);
        // Neither is nonlinear
        assert!(!circuit.has_nonlinear_elements());
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
