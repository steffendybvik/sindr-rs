//! Solver output types.
//!
//! [`SimulationResult`] is what every analysis path returns: node voltages,
//! branch currents, per-component V/I/P, and (when relevant) per-device
//! operating-point detail and a transient time-series. The optional
//! per-device fields are populated only when the matching component types
//! appear in the circuit.

use std::collections::HashMap;

use nalgebra::DVector;

use sindr_devices::bjt::{self, BjtKind, BjtParams};
use sindr_devices::diode::{self, DiodeParams};
use sindr_devices::jfet;
use sindr_devices::led::led_params_from_str;
use sindr_devices::mosfet::{self, MosfetKind};

use crate::circuit::{Circuit, CircuitElement};
use crate::node_map::NodeMap;
use crate::stamp::{ldr_resistance, SWITCH_R_CLOSED, SWITCH_R_OPEN};

/// Per-component voltage, current, and power.
///
/// Reported for every two-terminal component. Sign convention: current
/// flows from `nodes[0]` to `nodes[1]`; positive `power` means the
/// component is dissipating energy.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct ComponentResult {
    /// Component id (matches the id on the source [`CircuitElement`]).
    pub id: String,
    /// Voltage from `nodes[0]` to `nodes[1]` (V).
    pub voltage_across: f64,
    /// Current from `nodes[0]` to `nodes[1]` (A).
    pub current_through: f64,
    /// Instantaneous power dissipation (W). Positive = dissipating.
    pub power: f64,
}

/// Per-BJT operating-point results: terminal voltages, terminal currents,
/// power, and which region of operation the device is in.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct BjtResult {
    /// Component id.
    pub id: String,
    /// Base–emitter voltage (V).
    pub vbe: f64,
    /// Collector–emitter voltage (V).
    pub vce: f64,
    /// Base current (A).
    pub ib: f64,
    /// Collector current (A).
    pub ic: f64,
    /// Emitter current (A).
    pub ie: f64,
    /// Total dissipation (W).
    pub power: f64,
    /// Operating region: `"active"`, `"saturation"`, or `"cutoff"`.
    pub region: String,
}

/// Per-MOSFET operating-point results.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct MosfetResult {
    /// Component id.
    pub id: String,
    /// Gate–source voltage (V).
    pub vgs: f64,
    /// Drain–source voltage (V).
    pub vds: f64,
    /// Drain current (A).
    pub id_current: f64,
    /// Total dissipation (W).
    pub power: f64,
    /// Operating region: `"cutoff"`, `"triode"`, or `"saturation"`.
    pub region: String,
}

/// Per-op-amp / comparator results: input/output voltages.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct OpAmpResult {
    /// Component id.
    pub id: String,
    /// Voltage at the non-inverting input (V).
    pub v_in_plus: f64,
    /// Voltage at the inverting input (V).
    pub v_in_minus: f64,
    /// `v_in_plus - v_in_minus` (V).
    pub v_differential: f64,
    /// Output voltage (V), clipped to the rail range.
    pub v_out: f64,
}

/// Per-relay results: coil voltage and contact state.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct RelayResult {
    /// Component id.
    pub id: String,
    /// Voltage across the coil terminals (V).
    pub coil_voltage: f64,
    /// `true` when coil voltage exceeds the pickup threshold.
    pub contact_closed: bool,
}

/// Per-microcontroller results: per-pin GPIO output currents.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct McuResult {
    /// Component id.
    pub id: String,
    /// Current sourced/sunk by each GPIO pin (A), in pin index order.
    pub pin_currents: Vec<f64>,
    /// GPIO logic-high voltage used for this MCU (V).
    pub gpio_voltage: f64,
}

/// One timestep of a transient simulation: time, node voltages, per-component
/// state at that instant.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct TimestepSnapshot {
    /// Simulation time at this snapshot (s).
    pub time: f64,
    /// Node voltages at this instant (V).
    pub node_voltages: HashMap<String, f64>,
    /// Per-component voltage/current/power at this instant.
    pub component_results: Vec<ComponentResult>,
}

/// Time-series data from a transient simulation.
///
/// Present on [`SimulationResult::transient`] when the circuit contains
/// reactive elements (capacitors, inductors) or time-varying sources.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct TransientData {
    /// Snapshots at each integration step, in time order.
    pub timesteps: Vec<TimestepSnapshot>,
    /// Integration time step (s).
    pub time_step: f64,
    /// Total simulated duration (s).
    pub duration: f64,
}

/// Complete simulation output.
///
/// At minimum, contains node voltages, branch currents (for voltage sources
/// and other branch-current-bearing elements), and per-component results.
/// The optional fields are populated only when the corresponding
/// component types appear in the circuit:
///
/// - `bjt_results` — for [`Bjt`](crate::CircuitElement::Bjt) components
/// - `mosfet_results` — for [`Mosfet`](crate::CircuitElement::Mosfet)
/// - `op_amp_results` — for op-amps and comparators
/// - `relay_results` — for relays
/// - `mcu_results` — for microcontrollers
/// - `transient` — for circuits with capacitors, inductors, or waveform
///   sources (DC analysis returns `None`)
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct SimulationResult {
    /// Voltage at every node (V), keyed by node name. Includes the ground
    /// node (always 0.0).
    ///
    /// Indexing with `[]` panics on a missing key, the same as any
    /// [`HashMap`]. If the node name might be absent (e.g. typos, or a
    /// node only referenced via a control terminal), use
    /// `node_voltages.get("n2")` to get an `Option<&f64>` instead.
    pub node_voltages: HashMap<String, f64>,
    /// Branch currents (A) for components that introduce a current unknown
    /// — voltage sources, op-amps, transformers, etc. Keyed by component id.
    pub branch_currents: HashMap<String, f64>,
    /// Per-component voltage, current, and power for every two-terminal
    /// component.
    pub component_results: Vec<ComponentResult>,
    /// Detailed per-BJT results. Empty when the circuit has no BJTs.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub bjt_results: Vec<BjtResult>,
    /// Detailed per-MOSFET results. Empty when the circuit has no MOSFETs.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub mosfet_results: Vec<MosfetResult>,
    /// Detailed per-op-amp / comparator results.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub op_amp_results: Vec<OpAmpResult>,
    /// Detailed per-relay results.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub relay_results: Vec<RelayResult>,
    /// Detailed per-microcontroller results.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub mcu_results: Vec<McuResult>,
    /// Transient time-series, present only when the circuit has reactive
    /// elements or waveform sources.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub transient: Option<TransientData>,
}

/// Look up a node voltage from the solution vector.
///
/// Ground nodes (index `None`) return 0.0.
fn node_voltage(node: &str, node_map: &NodeMap, solution: &DVector<f64>) -> f64 {
    match node_map.index(node) {
        Some(idx) => solution[idx],
        None => 0.0, // ground
    }
}

/// Extract simulation results from the MNA solution vector.
///
/// Builds node voltages, branch currents (for voltage sources), and
/// per-component V/I/P using passive sign convention.
pub fn extract_results(
    circuit: &Circuit,
    node_map: &NodeMap,
    solution: &DVector<f64>,
    num_nodes: usize,
) -> SimulationResult {
    // Node voltages: every mapped node + ground = 0.0
    let mut node_voltages = HashMap::new();
    for i in 0..num_nodes {
        if let Some(name) = node_map.node_name(i) {
            node_voltages.insert(name.to_string(), solution[i]);
        }
    }
    node_voltages.insert(circuit.ground_node.clone(), 0.0);

    // Per-component results
    let mut component_results = Vec::new();
    let mut bjt_results_vec: Vec<BjtResult> = Vec::new();
    let mut mosfet_results_vec: Vec<MosfetResult> = Vec::new();
    let mut op_amp_results_vec: Vec<OpAmpResult> = Vec::new();
    let mut relay_results_vec: Vec<RelayResult> = Vec::new();
    let mut branch_currents = HashMap::new();
    let mut vsource_index: usize = 0;

    for component in &circuit.components {
        match component {
            CircuitElement::Resistor {
                id,
                nodes,
                resistance,
            } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_through = v_across / resistance;
                let power = v_across * i_through;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power,
                });
            }
            CircuitElement::VoltageSource {
                id,
                nodes: _,
                voltage,
                ..
            } => {
                let i_through = solution[num_nodes + vsource_index];
                let power = voltage * i_through;
                branch_currents.insert(id.clone(), i_through);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: *voltage,
                    current_through: i_through,
                    power,
                });
                vsource_index += 1;
            }
            CircuitElement::CurrentSource {
                id, nodes, current, ..
            } => {
                // Passive sign convention: V_across = v(from) - v(to)
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let power = v_across * current;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: *current,
                    power,
                });
            }
            CircuitElement::Switch { id, nodes, closed } => {
                // Switch modeled as resistor: R_closed = 0.01, R_open = 1e9
                let resistance = if *closed { 0.01 } else { 1e9 };
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_through = v_across / resistance;
                let power = v_across * i_through;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power,
                });
            }
            CircuitElement::Diode { id, nodes, .. } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let params = DiodeParams::silicon();
                let i_through = diode::diode_current(v_across, &params);
                let power = v_across * i_through;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power,
                });
            }
            CircuitElement::Led {
                id, nodes, color, ..
            } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let params = led_params_from_str(color);
                let i_through = diode::diode_current(v_across, &params);
                let power = v_across * i_through;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power,
                });
            }
            // Stubs: open-circuit in DC, return 0V/0A/0W
            CircuitElement::Capacitor { id, .. } | CircuitElement::Inductor { id, .. } => {
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: 0.0,
                    current_through: 0.0,
                    power: 0.0,
                });
            }
            CircuitElement::Bjt {
                id,
                nodes,
                kind,
                bf,
                temperature,
                ..
            } => {
                let vb = node_voltage(&nodes[0], node_map, solution);
                let vc = node_voltage(&nodes[1], node_map, solution);
                let ve = node_voltage(&nodes[2], node_map, solution);

                let sign = match kind {
                    BjtKind::Npn => 1.0,
                    BjtKind::Pnp => -1.0,
                };
                let vbe_eff = sign * (vb - ve);
                let vbc_eff = sign * (vb - vc);

                let mut params = BjtParams::new(*bf);
                if (*temperature - 300.15).abs() > 1e-6 {
                    params.is =
                        diode::temperature_scale_is(params.is, *temperature, 300.15, 1.11, 3.0);
                }
                let comp = bjt::bjt_companion(vbe_eff, vbc_eff, &params);
                let region = bjt::detect_region(vbe_eff, vbc_eff);

                // Physical terminal currents (sign-adjusted for PNP)
                let ic = sign * comp.ic;
                let ib = sign * comp.ib;
                let ie = -(ic + ib); // KCL

                let vbe = vb - ve; // physical voltage (not effective)
                let vce = vc - ve; // physical voltage

                // Power: collector dissipation
                let power = (ic * vce).abs();

                // ComponentResult for backward compat (Vce as voltage_across, Ic as current)
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vce,
                    current_through: ic,
                    power,
                });

                // Rich BJT result
                bjt_results_vec.push(BjtResult {
                    id: id.clone(),
                    vbe,
                    vce,
                    ib,
                    ic,
                    ie,
                    power,
                    region: region.as_str().to_string(),
                });
            }
            CircuitElement::Mosfet {
                id,
                nodes,
                kind,
                params,
                ..
            } => {
                let vg = node_voltage(&nodes[0], node_map, solution);
                let vd = node_voltage(&nodes[1], node_map, solution);
                let vs = node_voltage(&nodes[2], node_map, solution);

                let (vgs, vds, vbs) = match kind {
                    MosfetKind::Nmos => (vg - vs, vd - vs, -vs),
                    MosfetKind::Pmos => (vs - vg, vs - vd, vs),
                };

                let comp = mosfet::mosfet_companion(vgs, vds, vbs, params);
                let id_current = match kind {
                    MosfetKind::Nmos => comp.id,
                    MosfetKind::Pmos => -comp.id,
                };

                let vds_phys = vd - vs;
                let vgs_phys = vg - vs;
                let power = (id_current * vds_phys).abs();

                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vds_phys,
                    current_through: id_current,
                    power,
                });

                mosfet_results_vec.push(MosfetResult {
                    id: id.clone(),
                    vgs: vgs_phys,
                    vds: vds_phys,
                    id_current,
                    power,
                    region: comp.region.as_str().to_string(),
                });
            }
            CircuitElement::Vcvs { id, nodes, .. } | CircuitElement::Ccvs { id, nodes, .. } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_through = solution[num_nodes + vsource_index];
                let power = v_across * i_through;
                branch_currents.insert(id.clone(), i_through);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power,
                });
                vsource_index += 1;
            }
            CircuitElement::Vccs {
                id,
                nodes,
                control_nodes,
                gm,
            } => {
                let v_ctrl_p = node_voltage(&control_nodes[0], node_map, solution);
                let v_ctrl_n = node_voltage(&control_nodes[1], node_map, solution);
                let v_ctrl = v_ctrl_p - v_ctrl_n;
                let i_out = gm * v_ctrl;
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_out,
                    power: v_across * i_out,
                });
            }
            CircuitElement::Cccs {
                id,
                nodes,
                control_source,
                alpha,
            } => {
                // Find controlling current from its branch
                let ctrl_branch = circuit.vsource_branch_index(control_source);
                let i_ctrl = ctrl_branch.map_or(0.0, |b| solution[num_nodes + b]);
                let i_out = alpha * i_ctrl;
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_out,
                    power: v_across * i_out,
                });
            }
            CircuitElement::Pushbutton { id, nodes, closed } => {
                let resistance = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_through = v_across / resistance;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Photoresistor {
                id,
                nodes,
                light_level,
            } => {
                let resistance = ldr_resistance(*light_level);
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_through = v_across / resistance;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Potentiometer { id, nodes, .. } => {
                let v_top = node_voltage(&nodes[0], node_map, solution);
                let v_bot = node_voltage(&nodes[2], node_map, solution);
                let v_across = v_top - v_bot;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: 0.0,
                    power: 0.0,
                });
            }
            CircuitElement::Relay {
                id,
                nodes,
                coil_resistance,
                pickup_voltage,
                ..
            } => {
                let vc_pos = node_voltage(&nodes[0], node_map, solution);
                let vc_neg = node_voltage(&nodes[1], node_map, solution);
                let coil_voltage = vc_pos - vc_neg;
                let coil_current = coil_voltage / coil_resistance;
                let contact_closed = coil_voltage.abs() >= *pickup_voltage;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: coil_voltage,
                    current_through: coil_current,
                    power: coil_voltage * coil_current,
                });
                relay_results_vec.push(RelayResult {
                    id: id.clone(),
                    coil_voltage,
                    contact_closed,
                });
            }
            CircuitElement::ZenerDiode { id, nodes, vz, .. } => {
                let va = node_voltage(&nodes[0], node_map, solution);
                let vk = node_voltage(&nodes[1], node_map, solution);
                let v_across = va - vk;
                let params = sindr_devices::zener::ZenerParams::new(*vz);
                let (g_eq, i_eq) = sindr_devices::zener::zener_companion(v_across, &params);
                let i_through = g_eq * v_across + i_eq;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::SchottkyDiode { id, nodes, .. } => {
                let va = node_voltage(&nodes[0], node_map, solution);
                let vk = node_voltage(&nodes[1], node_map, solution);
                let v_across = va - vk;
                let schottky_params = sindr_devices::schottky::SchottkyParams::default();
                let i_through = diode::diode_current(
                    v_across,
                    &DiodeParams {
                        is: schottky_params.is,
                        n: schottky_params.n,
                        rs: 0.0,
                        temperature: 300.15,
                    },
                );
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Thermistor {
                id,
                nodes,
                temperature,
            } => {
                let therm_params = sindr_devices::thermistor::ThermistorParams::default();
                let r =
                    sindr_devices::thermistor::thermistor_resistance(*temperature, &therm_params);
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_through = v_across / r;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Photodiode {
                id,
                nodes,
                irradiance,
                ..
            } => {
                let va = node_voltage(&nodes[0], node_map, solution);
                let vk = node_voltage(&nodes[1], node_map, solution);
                let v_across = va - vk;
                let photo_params = sindr_devices::photodiode::PhotodiodeParams::default();
                // Total current = dark diode current - photocurrent
                let i_dark = diode::diode_current(
                    v_across,
                    &DiodeParams {
                        is: photo_params.is,
                        n: photo_params.n,
                        rs: 0.0,
                        temperature: 300.15,
                    },
                );
                let i_photo = photo_params.responsivity * irradiance.max(0.0);
                let i_through = i_dark - i_photo;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Varactor { id, nodes, .. } => {
                // Varactor in DC: open circuit — report terminal voltage, zero current
                let va = node_voltage(&nodes[0], node_map, solution);
                let vk = node_voltage(&nodes[1], node_map, solution);
                let v_across = va - vk;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: 0.0,
                    power: 0.0,
                });
            }
            CircuitElement::Igbt {
                id, nodes, params, ..
            } => {
                // IGBT: report gate-emitter voltage, collector-emitter voltage, and drain current
                let vg = node_voltage(&nodes[0], node_map, solution);
                let vc = node_voltage(&nodes[1], node_map, solution);
                let ve = node_voltage(&nodes[2], node_map, solution);
                let vge = vg - ve;
                let vce = vc - ve;
                let comp = sindr_devices::igbt::igbt_companion(vge, vce, params);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vce,
                    current_through: comp.ids,
                    power: (comp.ids * vce).abs(),
                });
            }
            CircuitElement::Transformer { id, nodes, .. } => {
                // In DC analysis, transformer is stamped as near-short-circuits (1e-9 Ohm).
                // Report primary and secondary voltages; DC current is indeterminate (use 0.0).
                let v_p1 = node_voltage(&nodes[0], node_map, solution);
                let v_q1 = node_voltage(&nodes[1], node_map, solution);
                let v_primary = v_p1 - v_q1;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_primary,
                    current_through: 0.0,
                    power: 0.0,
                });
                vsource_index += 2; // Transformer uses 2 branch current slots in MNA
            }
            CircuitElement::Jfet {
                id,
                nodes,
                kind,
                idss,
                vp,
                ..
            } => {
                let vg = node_voltage(&nodes[0], node_map, solution);
                let vd = node_voltage(&nodes[1], node_map, solution);
                let vs = node_voltage(&nodes[2], node_map, solution);
                let vgs = vg - vs;
                let vds = vd - vs;
                // Compute drain current from companion model at final solution point
                let c = jfet::jfet_companion(vgs, vds, *kind, *idss, *vp);
                let id_current = c.gm * vgs + c.gds * vds + c.i_eq;
                let power = (id_current * vds).abs();
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vds,
                    current_through: id_current,
                    power,
                });
            }
            CircuitElement::Fuse {
                id, nodes, blown, ..
            } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let resistance = if *blown { 1e9_f64 } else { 0.001_f64 };
                let i_through = v_across / resistance;
                let power = v_across * i_through;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power,
                });
            }
            CircuitElement::VoltageRegulator { id, nodes, voltage } => {
                // Stamped as ideal voltage source between nodes[1] (output) and nodes[2] (gnd).
                // Report voltage at output relative to the regulator's gnd node.
                let vout = node_voltage(&nodes[1], node_map, solution);
                let vgnd = node_voltage(&nodes[2], node_map, solution);
                let v_across = vout - vgnd;
                let i_through = solution[num_nodes + vsource_index];
                let power = voltage * i_through;
                branch_currents.insert(id.clone(), i_through);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power,
                });
                vsource_index += 1;
            }
            CircuitElement::OpAmp { id, nodes, .. }
            | CircuitElement::Comparator { id, nodes, .. } => {
                // VCVS: read output voltage from solution; read branch current
                let v_in_plus = node_voltage(&nodes[0], node_map, solution);
                let v_in_minus = node_voltage(&nodes[1], node_map, solution);
                let v_out = node_voltage(&nodes[2], node_map, solution);
                let v_differential = v_in_plus - v_in_minus;
                let i_through = solution[num_nodes + vsource_index];
                let power = v_out * i_through;
                branch_currents.insert(id.clone(), i_through);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_out,
                    current_through: i_through,
                    power,
                });
                op_amp_results_vec.push(OpAmpResult {
                    id: id.clone(),
                    v_in_plus,
                    v_in_minus,
                    v_differential,
                    v_out,
                });
                vsource_index += 1;
            }
        }
    }

    SimulationResult {
        node_voltages,
        branch_currents,
        component_results,
        bjt_results: bjt_results_vec,
        mosfet_results: mosfet_results_vec,
        op_amp_results: op_amp_results_vec,
        relay_results: relay_results_vec,
        mcu_results: vec![],
        transient: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DC-only SimulationResult (transient: None) serializes WITHOUT a "transient" key.
    #[cfg(feature = "serde")]
    #[test]
    fn dc_result_omits_transient_key() {
        let result = SimulationResult {
            node_voltages: HashMap::from([("n1".into(), 5.0)]),
            branch_currents: HashMap::new(),
            component_results: vec![],
            bjt_results: vec![],
            mosfet_results: vec![],
            op_amp_results: vec![],
            relay_results: vec![],
            mcu_results: vec![],
            transient: None,
        };
        let json = serde_json::to_string(&result).unwrap();
        assert!(!json.contains("\"transient\""));
        assert!(!json.contains("\"bjt_results\""));
    }

    /// SimulationResult with transient data serializes the timesteps array.
    #[cfg(feature = "serde")]
    #[test]
    fn transient_result_includes_timesteps() {
        let result = SimulationResult {
            node_voltages: HashMap::from([("n1".into(), 5.0)]),
            branch_currents: HashMap::new(),
            component_results: vec![],
            bjt_results: vec![],
            mosfet_results: vec![],
            op_amp_results: vec![],
            relay_results: vec![],
            mcu_results: vec![],
            transient: Some(TransientData {
                timesteps: vec![
                    TimestepSnapshot {
                        time: 0.0,
                        node_voltages: HashMap::from([("n1".into(), 0.0)]),
                        component_results: vec![],
                    },
                    TimestepSnapshot {
                        time: 0.001,
                        node_voltages: HashMap::from([("n1".into(), 3.2)]),
                        component_results: vec![],
                    },
                ],
                time_step: 0.001,
                duration: 0.002,
            }),
        };
        let json = serde_json::to_string(&result).unwrap();
        assert!(json.contains("\"transient\""));
        assert!(json.contains("\"timesteps\""));
        assert!(json.contains("\"time_step\""));
        assert!(json.contains("\"duration\""));

        // Roundtrip: deserialize back
        let parsed: SimulationResult = serde_json::from_str(&json).unwrap();
        let td = parsed.transient.unwrap();
        assert_eq!(td.timesteps.len(), 2);
        assert_eq!(td.time_step, 0.001);
        assert_eq!(td.duration, 0.002);
        assert_eq!(td.timesteps[1].node_voltages["n1"], 3.2);
    }
}
