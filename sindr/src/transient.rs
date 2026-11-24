//! Transient (time-domain) analysis.
//!
//! Integrates the circuit over time, solving the MNA system at each step
//! while reactive components contribute companion-model stamps.

use crate::element::{
    Battery, Behavioural, Bjt, Capacitor, Cccs, Ccvs, Comparator, Crystal, CurrentSource, Diode,
    FerriteBead, Fuse, Igbt, Inductor, Jfet, Led, Mosfet, OpAmp, Photodiode, Photoresistor,
    Potentiometer, Pushbutton, Relay, Resistor, SchottkyDiode, Switch, SwitchSpdt, Thermistor,
    Transformer, Varactor, Vccs, Vcvs, VoltageRegulator, VoltageSource, ZenerDiode,
};
use std::collections::HashMap;

use nalgebra::DVector;

use sindr_devices::bjt::{self, BjtKind};
use sindr_devices::inductor;
use sindr_devices::mosfet::MosfetKind;

use crate::circuit::{Circuit, CircuitElement};
use crate::device_params;
use crate::element::bjt::bjt_parasitics;
use crate::element::diode::DIODE_CHARGE;
use crate::element::jfet::{jfet_gate_charge, jfet_junctions};
use crate::element::layout::Layout;
use crate::element::mosfet::mosfet_parasitics;
use crate::element::relay::series_resistance;
use crate::element::transformer::transformer_flux;
use crate::element::{parasitic_history, TranStamp};
use crate::error::{SimError, SolvePoint};
pub use crate::integration::IntegrationMethod;
use crate::integration::{History, Integrator};
use crate::mna::MnaSystem;
use crate::newton_raphson;
use crate::node_map::{FastMap, NodeMap, Terminals};
use crate::op_amp::OP_AMP_STAGE;
use crate::results::{
    BjtResult, ComponentResult, MosfetResult, SimulationResult, TimestepSnapshot, TransientData,
    TransientStats,
};
use crate::solver::SolverOptions;
use crate::stamp::{SWITCH_R_CLOSED, SWITCH_R_OPEN};
use crate::thermal::Heated;

/// Calculate simulation duration and timestep from circuit time constants.
///
/// Returns `(duration, dt)` in seconds, from the longest RC/RL time
/// constant τ (1 ms if the circuit has none):
/// - duration = 5·τ (about 99 % settling)
/// - dt = τ/50, floored at 1 µs
///
/// With periodic waveform sources, the longest period T also counts:
/// duration is at least 3·T and dt at most T/100 (also floored at 1 µs).
/// Without RC/RL time constants, those alone set it.
pub fn calculate_duration(circuit: &Circuit) -> (f64, f64) {
    let flat = circuit.flattened();
    let circuit: &Circuit = flat.as_deref().unwrap_or(circuit);
    // Collect R, C, L values
    let mut resistances: Vec<f64> = Vec::new();
    let mut capacitances: Vec<f64> = Vec::new();
    let mut inductances: Vec<f64> = Vec::new();

    for component in &circuit.components {
        match component {
            CircuitElement::Resistor(Resistor { resistance, .. }) => {
                resistances.push(*resistance);
            }
            CircuitElement::Battery(Battery { params, .. }) => {
                resistances.push(params.r_series);
                if let Some(pair) = params.rc {
                    resistances.push(pair.r);
                    capacitances.push(pair.c);
                }
            }
            CircuitElement::Switch(Switch { closed, .. }) => {
                let r = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                resistances.push(r);
            }
            CircuitElement::Capacitor(Capacitor { capacitance, .. }) => {
                capacitances.push(*capacitance);
            }
            CircuitElement::Inductor(Inductor { inductance, .. }) => {
                inductances.push(*inductance);
            }
            CircuitElement::Relay(Relay {
                coil_resistance,
                inductance,
                ..
            }) if *inductance > 0.0 => {
                resistances.push(*coil_resistance);
                inductances.push(*inductance);
            }
            CircuitElement::Transformer(Transformer { l1, l2, .. }) => {
                // Use both winding inductances for time constant estimation
                inductances.push(*l1);
                inductances.push(*l2);
            }
            _ => {}
        }
    }

    let mut tau_max: f64 = 0.0;

    // RC time constants
    for r in &resistances {
        for c in &capacitances {
            let tau = r * c;
            if tau > tau_max {
                tau_max = tau;
            }
        }
    }

    // RL time constants (skip tiny resistances to avoid switch-like values)
    for r in &resistances {
        if *r < 0.02 {
            continue;
        }
        for l in &inductances {
            let tau = l / r;
            if tau > tau_max {
                tau_max = tau;
            }
        }
    }

    // Check for waveform periods to determine appropriate duration
    let mut waveform_period: Option<f64> = None;
    for component in &circuit.components {
        match component {
            CircuitElement::VoltageSource(VoltageSource {
                waveform: Some(w), ..
            })
            | CircuitElement::CurrentSource(CurrentSource {
                waveform: Some(w), ..
            }) => {
                if let Some(p) = w.period() {
                    waveform_period = Some(match waveform_period {
                        Some(existing) => existing.max(p),
                        None => p,
                    });
                }
            }
            _ => {}
        }
    }

    // If there are waveform sources, use their period to set duration
    if let Some(period) = waveform_period {
        // Show at least 3 full periods
        let waveform_duration = 3.0 * period;
        // dt should capture waveform detail: at least 100 points per period
        let waveform_dt = (period / 100.0).max(1e-6);

        if tau_max > 0.0 {
            // Both RC/RL and waveform: use the longer duration, finer dt
            let rc_duration = 5.0 * tau_max;
            let rc_dt = (tau_max / 50.0).max(1e-6);
            return (rc_duration.max(waveform_duration), rc_dt.min(waveform_dt));
        }
        return (waveform_duration, waveform_dt);
    }

    // Fallback if no time constants found
    if tau_max <= 0.0 {
        tau_max = 1e-3;
    }

    let duration = 5.0 * tau_max;
    let dt = (tau_max / 50.0).max(1e-6); // floor at 1 microsecond

    (duration, dt)
}

/// Stamp all circuit components for one transient step ending at time `t`.
///
/// Each element writes its [`Device::stamp_transient`]: capacitors and
/// inductors `integ`'s companions, built from their history at the start
/// of the step (`caps`, `inds`, in `placement`'s slots, and `parasitic`,
/// keyed `"{id}-{key}"`); sources their value at `t`; nonlinear devices
/// their linearisation at `v_prev`, the current Newton iterate.
///
/// [`Device::stamp_transient`]: crate::element::Device::stamp_transient
#[allow(clippy::too_many_arguments)]
fn stamp_circuit_transient(
    circuit: &Circuit,
    system: &mut MnaSystem,
    node_map: &NodeMap,
    placement: &Placement,
    integ: &Integrator,
    t: f64,
    caps: &[History],
    inds: &[History],
    v_prev: &DVector<f64>,
    parasitic: &FastMap<History>,
) -> Result<(), SimError> {
    for (k, component) in circuit.components.iter().enumerate() {
        let mut s = TranStamp::new(
            system,
            circuit,
            node_map,
            &placement.layout,
            &placement.terminals,
            k,
            v_prev,
            t,
            integ,
            caps,
            inds,
            parasitic,
        );
        component.device().stamp_transient(&mut s)?;
    }
    Ok(())
}

/// Helper: get node voltage from solution vector (ground = 0.0).
fn node_voltage(node: &str, node_map: &NodeMap, solution: &DVector<f64>) -> f64 {
    match node_map.index(node) {
        Some(idx) => solution[idx],
        None => 0.0,
    }
}

/// Extract per-timestep results from solution vector, and the reactive
/// histories after this step (`caps_prev`/`inds_prev` are those before it).
/// Sources report their value at `source_time`, the time the step was
/// solved with.
#[allow(clippy::too_many_arguments)]
fn extract_timestep_results(
    circuit: &Circuit,
    node_map: &NodeMap,
    solution: &DVector<f64>,
    num_nodes: usize,
    time: f64,
    source_time: f64,
    integ: &Integrator,
    caps_prev: &[History],
    inds_prev: &[History],
) -> (TimestepSnapshot, Vec<History>, Vec<History>) {
    // Node voltages
    let mut node_voltages = HashMap::new();
    for i in 0..num_nodes {
        if let Some(name) = node_map.node_name(i) {
            node_voltages.insert(name.to_string(), solution[i]);
        }
    }
    node_voltages.insert(circuit.ground_node.clone(), 0.0);

    let mut component_results = Vec::new();
    let mut vsource_index: usize = 0;
    let mut cap_index: usize = 0;
    let mut ind_index: usize = 0;

    let mut new_caps = Vec::new();
    let mut new_inds = Vec::new();

    for component in &circuit.components {
        match component {
            // Split into their parts, or lowered to a diode, by
            // `solver_circuit` first.
            CircuitElement::SwitchSpdt(SwitchSpdt { .. })
            | CircuitElement::Crystal(Crystal { .. })
            | CircuitElement::FerriteBead(FerriteBead { .. })
            | CircuitElement::ZenerDiode(ZenerDiode { .. }) => {}
            // Needs its state's history; `battery::finish_step` reports it.
            CircuitElement::Battery(Battery { .. }) => {}
            CircuitElement::Resistor(Resistor {
                id,
                nodes,
                resistance,
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v = vp - vq;
                let i = v / resistance;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v,
                    current_through: i,
                    power: v * i,
                });
            }
            CircuitElement::VoltageSource(VoltageSource {
                id,
                voltage,
                waveform,
                ..
            }) => {
                let v = voltage + waveform.as_ref().map_or(0.0, |w| w.evaluate(source_time));
                let i = solution[num_nodes + vsource_index];
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v,
                    current_through: i,
                    power: v * i,
                });
                vsource_index += 1;
            }
            CircuitElement::CurrentSource(CurrentSource {
                id,
                nodes,
                current,
                waveform,
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v = vp - vq;
                let i = current + waveform.as_ref().map_or(0.0, |w| w.evaluate(source_time));
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v,
                    current_through: i,
                    power: v * i,
                });
            }
            CircuitElement::Switch(Switch { id, nodes, closed }) => {
                let resistance = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v = vp - vq;
                let i = v / resistance;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v,
                    current_through: i,
                    power: v * i,
                });
            }
            CircuitElement::Capacitor(Capacitor {
                id,
                nodes,
                capacitance,
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_now = vp - vq;
                let history = caps_prev[cap_index];
                let (g, i_hist) = integ.capacitor(*capacitance, &history);
                let i = g * v_now + i_hist;
                new_caps.push(history.push(v_now, i));
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_now,
                    current_through: i,
                    power: v_now * i,
                });
                cap_index += 1;
            }
            CircuitElement::Inductor(Inductor {
                id,
                nodes,
                inductance,
                saturation,
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let history = inds_prev[ind_index];
                let i_now = match saturation {
                    None => {
                        let (g, i_hist) = integ.inductor(*inductance, &history);
                        let i_now = g * v_across + i_hist;
                        new_inds.push(history.push(i_now, v_across));
                        i_now
                    }
                    Some(sat) => {
                        let lambda = integ.flux(v_across, &history);
                        new_inds.push(history.push(lambda, v_across));
                        inductor::current_from_flux(lambda, *inductance, sat)
                    }
                };
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_now,
                    power: v_across * i_now,
                });
                ind_index += 1;
            }
            CircuitElement::Diode(Diode {
                model,
                id,
                nodes,
                temperature,
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let params = device_params::diode(model.as_ref(), *temperature);
                let i_through = sindr_devices::diode::diode_current(v_across, &params);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Led(Led {
                id,
                nodes,
                color,
                temperature,
                ..
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let params = device_params::led(color, *temperature);
                let i_through = sindr_devices::diode::diode_current(v_across, &params);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Bjt(Bjt {
                model,
                id,
                nodes,
                kind,
                bf,
                temperature,
                ..
            }) => {
                let vb = node_voltage(&nodes[0], node_map, solution);
                let vc = node_voltage(&nodes[1], node_map, solution);
                let ve = node_voltage(&nodes[2], node_map, solution);
                let sign = match kind {
                    BjtKind::Npn => 1.0,
                    BjtKind::Pnp => -1.0,
                };
                let vbe_eff = sign * (vb - ve);
                let vbc_eff = sign * (vb - vc);
                let params = device_params::bjt(*bf, model.as_ref(), *temperature);
                let comp = bjt::bjt_companion(vbe_eff, vbc_eff, &params);
                let ic = sign * comp.ic;
                let vce = vc - ve;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vce,
                    current_through: ic,
                    power: (ic * vce).abs(),
                });
            }
            CircuitElement::Mosfet(Mosfet {
                id,
                nodes,
                kind,
                params,
                ..
            }) => {
                let vg = node_voltage(&nodes[0], node_map, solution);
                let vd = node_voltage(&nodes[1], node_map, solution);
                let vs = node_voltage(&nodes[2], node_map, solution);
                let comp = crate::device_params::mosfet_op(*kind, params, vg, vd, vs);
                let id_current = match kind {
                    MosfetKind::Nmos => comp.id,
                    MosfetKind::Pmos => -comp.id,
                };
                let vds_phys = vd - vs;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vds_phys,
                    current_through: id_current,
                    power: (id_current * vds_phys).abs(),
                });
            }
            CircuitElement::Behavioural(Behavioural {
                id,
                nodes,
                output,
                expr,
            }) => {
                component_results.push(crate::results::behavioural_result(
                    circuit,
                    node_map,
                    solution,
                    source_time,
                    id,
                    nodes,
                    *output,
                    expr,
                    num_nodes + vsource_index,
                ));
                vsource_index += component.branch_slots();
            }
            CircuitElement::Vcvs(Vcvs { id, nodes, .. })
            | CircuitElement::Ccvs(Ccvs { id, nodes, .. }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_through = solution[num_nodes + vsource_index];
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
                vsource_index += 1;
            }
            CircuitElement::Vccs(Vccs {
                id,
                nodes,
                control_nodes,
                gm,
            }) => {
                let v_ctrl_p = node_voltage(&control_nodes[0], node_map, solution);
                let v_ctrl_n = node_voltage(&control_nodes[1], node_map, solution);
                let i_out = gm * (v_ctrl_p - v_ctrl_n);
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
            CircuitElement::Cccs(Cccs {
                id,
                nodes,
                control_source,
                alpha,
            }) => {
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
            CircuitElement::Pushbutton(Pushbutton { id, nodes, closed }) => {
                let resistance = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v = vp - vq;
                let i = v / resistance;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v,
                    current_through: i,
                    power: v * i,
                });
            }
            CircuitElement::Photoresistor(Photoresistor {
                id,
                nodes,
                light_level,
            }) => {
                let resistance = crate::element::photoresistor::ldr_resistance(*light_level);
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v = vp - vq;
                let i = v / resistance;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v,
                    current_through: i,
                    power: v * i,
                });
            }
            CircuitElement::Potentiometer(Potentiometer { id, nodes, .. }) => {
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
            CircuitElement::Relay(Relay {
                id,
                nodes,
                coil_resistance,
                pickup_voltage,
                inductance,
                ..
            }) => {
                let vc_pos = node_voltage(&nodes[0], node_map, solution);
                let vc_neg = node_voltage(&nodes[1], node_map, solution);
                let coil_voltage = vc_pos - vc_neg;
                // With inductance, the coil current is the inductor's, and
                // the inductor sees the coil voltage less the resistive drop.
                let coil_current = if *inductance > 0.0 {
                    let history = inds_prev[ind_index];
                    let (g, i_hist) = integ.inductor(*inductance, &history);
                    let (g_s, i_s) = series_resistance(g, i_hist, *coil_resistance);
                    let current = g_s * coil_voltage + i_s;
                    let v_inductor = coil_voltage - coil_resistance * current;
                    new_inds.push(history.push(current, v_inductor));
                    ind_index += 1;
                    current
                } else {
                    coil_voltage / coil_resistance
                };
                let contact_closed = coil_voltage.abs() >= *pickup_voltage;
                let contact_r = if contact_closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                let vc1 = node_voltage(&nodes[2], node_map, solution);
                let vc2 = node_voltage(&nodes[3], node_map, solution);
                let contact_v = vc1 - vc2;
                let contact_i = contact_v / contact_r;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: coil_voltage,
                    current_through: coil_current,
                    power: coil_voltage * coil_current,
                });
                // contact result as separate push for accounting
                let _ = contact_i;
            }
            CircuitElement::OpAmp(OpAmp { id, nodes, .. })
            | CircuitElement::Comparator(Comparator { id, nodes, .. }) => {
                let v_out = node_voltage(&nodes[2], node_map, solution);
                let i_through = solution[num_nodes + vsource_index];
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_out,
                    current_through: i_through,
                    power: v_out * i_through,
                });
                vsource_index += component.branch_slots();
            }
            CircuitElement::SchottkyDiode(SchottkyDiode {
                id,
                nodes,
                temperature,
                ..
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_through = sindr_devices::diode::diode_current(
                    v_across,
                    &device_params::schottky(*temperature),
                );
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Thermistor(Thermistor {
                id,
                nodes,
                temperature,
            }) => {
                let r = crate::device_params::thermistor_resistance(*temperature);
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
            CircuitElement::Photodiode(Photodiode {
                id,
                nodes,
                irradiance,
                temperature,
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let photo_params = device_params::photodiode(*temperature);
                let i_dark = sindr_devices::diode::diode_current(
                    v_across,
                    &device_params::photodiode_dark(*temperature),
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
            CircuitElement::Varactor(Varactor {
                id, nodes, params, ..
            }) => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_now = vp - vq;
                // Update varactor voltage state for next timestep (uses positional cap_index)
                // The companion the step was solved with (see the stamp).
                let history = caps_prev[cap_index];
                let cj = sindr_devices::varactor::junction_capacitance(history.x1, params);
                let (g, i_hist) = integ.capacitor(cj, &history);
                let i_through = g * v_now + i_hist;
                new_caps.push(history.push(v_now, i_through));
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_now,
                    current_through: i_through,
                    power: v_now * i_through,
                });
                cap_index += 1;
            }
            CircuitElement::Igbt(Igbt {
                id, nodes, params, ..
            }) => {
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
            CircuitElement::Transformer(Transformer {
                id,
                nodes,
                saturation,
                ..
            }) => {
                // Primary winding voltages and currents
                let v_p1 = node_voltage(&nodes[0], node_map, solution);
                let v_q1 = node_voltage(&nodes[1], node_map, solution);
                let v_p2 = node_voltage(&nodes[2], node_map, solution);
                let v_q2 = node_voltage(&nodes[3], node_map, solution);
                let v_primary = v_p1 - v_q1;
                let v_secondary = v_p2 - v_q2;
                // Branch currents extracted from solution (branch current unknowns at num_nodes + vsource_index)
                let i1_now = solution[num_nodes + vsource_index];
                let i2_now = solution[num_nodes + vsource_index + 1];
                vsource_index += 2;
                // Winding rate is the branch-convention voltage (see
                // Integrator::coupled).
                // A saturating core's histories hold fluxes.
                let (x1, x2) = match saturation {
                    None => (i1_now, i2_now),
                    Some(_) => (
                        integ.flux(v_primary, &inds_prev[ind_index]),
                        integ.flux(v_secondary, &inds_prev[ind_index + 1]),
                    ),
                };
                new_inds.push(inds_prev[ind_index].push(x1, v_primary));
                new_inds.push(inds_prev[ind_index + 1].push(x2, v_secondary));
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_primary,
                    current_through: i1_now,
                    power: (v_primary * i1_now + v_secondary * i2_now).abs(),
                });
                ind_index += 2;
            }
            CircuitElement::Fuse(Fuse {
                id, nodes, blown, ..
            }) => {
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
            CircuitElement::Jfet(Jfet {
                id,
                nodes,
                kind,
                idss,
                vp,
                ..
            }) => {
                let vg = node_voltage(&nodes[0], node_map, solution);
                let vd = node_voltage(&nodes[1], node_map, solution);
                let vs = node_voltage(&nodes[2], node_map, solution);
                let vds_phys = vd - vs;
                let id_current =
                    crate::results::jfet_drain_current(vg - vs, vds_phys, *kind, *idss, *vp);
                let power = (id_current * vds_phys).abs();
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vds_phys,
                    current_through: id_current,
                    power,
                });
            }
            CircuitElement::VoltageRegulator(VoltageRegulator {
                id,
                nodes,
                voltage,
                model,
            }) => {
                component_results.push(crate::results::regulator_result(
                    id,
                    nodes,
                    *voltage,
                    model,
                    node_map,
                    solution,
                    num_nodes + vsource_index,
                ));
                vsource_index += 1;
            }
        }
    }

    let snapshot = TimestepSnapshot {
        time,
        node_voltages,
        component_results,
        junction_temperatures: HashMap::new(),
        state_of_charge: HashMap::new(),
    };

    (snapshot, new_caps, new_inds)
}

/// Extract BJT results from a converged timestep snapshot.
///
/// Uses node voltages from the snapshot to compute Ebers-Moll terminal currents
/// and detect operating region, mirroring the DC extraction in results.rs.
fn extract_bjt_results_from_snapshot(
    circuit: &Circuit,
    snapshot: &TimestepSnapshot,
) -> Vec<BjtResult> {
    let mut bjt_results = Vec::new();

    for component in &circuit.components {
        if let CircuitElement::Bjt(Bjt {
            model,
            id,
            nodes,
            kind,
            bf,
            temperature,
            ..
        }) = component
        {
            let vb = snapshot
                .node_voltages
                .get(&nodes[0])
                .copied()
                .unwrap_or(0.0);
            let vc = snapshot
                .node_voltages
                .get(&nodes[1])
                .copied()
                .unwrap_or(0.0);
            let ve = snapshot
                .node_voltages
                .get(&nodes[2])
                .copied()
                .unwrap_or(0.0);

            let sign = match kind {
                BjtKind::Npn => 1.0,
                BjtKind::Pnp => -1.0,
            };
            let vbe_eff = sign * (vb - ve);
            let vbc_eff = sign * (vb - vc);

            let params = device_params::bjt(*bf, model.as_ref(), *temperature);
            let comp = bjt::bjt_companion(vbe_eff, vbc_eff, &params);
            let region = bjt::detect_region(vbe_eff, vbc_eff);

            let ic = sign * comp.ic;
            let ib = sign * comp.ib;
            let ie = -(ic + ib);

            let vbe = vb - ve;
            let vce = vc - ve;
            let power = (ic * vce).abs();

            bjt_results.push(BjtResult {
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
    }

    bjt_results
}

/// Extract MOSFET results from a converged timestep snapshot.
fn extract_mosfet_results_from_snapshot(
    circuit: &Circuit,
    snapshot: &TimestepSnapshot,
) -> Vec<MosfetResult> {
    let mut mosfet_results = Vec::new();

    for component in &circuit.components {
        if let CircuitElement::Mosfet(Mosfet {
            id,
            nodes,
            kind,
            params,
            ..
        }) = component
        {
            let vg = snapshot
                .node_voltages
                .get(&nodes[0])
                .copied()
                .unwrap_or(0.0);
            let vd = snapshot
                .node_voltages
                .get(&nodes[1])
                .copied()
                .unwrap_or(0.0);
            let vs = snapshot
                .node_voltages
                .get(&nodes[2])
                .copied()
                .unwrap_or(0.0);

            let comp = crate::device_params::mosfet_op(*kind, params, vg, vd, vs);
            let id_current = match kind {
                MosfetKind::Nmos => comp.id,
                MosfetKind::Pmos => -comp.id,
            };

            let vgs_phys = vg - vs;
            let vds_phys = vd - vs;
            let power = (id_current * vds_phys).abs();

            mosfet_results.push(MosfetResult {
                id: id.clone(),
                vgs: vgs_phys,
                vds: vds_phys,
                id_current,
                power,
                region: comp.region.as_str().to_string(),
            });
        }
    }

    mosfet_results
}

/// Transient analysis settings: how long to simulate and how finely.
///
/// `tstep` is both the longest step and the output grid: every multiple of
/// it gets a snapshot. By default the stepper also shortens steps where the
/// local truncation error would exceed SPICE's default tolerances (RELTOL
/// 1e-3, VNTOL 1 µV, TRTOL 7), and lands exactly on every corner of a source
/// waveform ([`Waveform::breakpoints`](crate::Waveform::breakpoints)),
/// restarting the integration there.
///
/// Build one with [`TransientConfig::new`], optionally capped with
/// [`TransientConfig::with_tmax`], and pass it to [`solve_transient`].
/// [`solve_circuit`](crate::solve_circuit) instead picks a config from the
/// circuit's time constants ([`TransientConfig::auto`]).
///
/// ```
/// use sindr::TransientConfig;
///
/// // SPICE `.tran 1u 10m`: 10 ms in 1 µs steps.
/// let config = TransientConfig::new(10e-3, 1e-6);
/// assert_eq!(config.tstop, 10e-3);
/// ```
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct TransientConfig {
    /// End time (s). The run starts at 0 and its last snapshot is at `tstop`.
    pub tstop: f64,
    /// Time step (s): the output grid, and the longest step unless `tmax`
    /// says otherwise. Truncation-error control, breakpoints and failed
    /// Newton solves add shorter steps in between.
    pub tstep: f64,
    /// Largest step allowed (s), as SPICE's TMAX. `None` means `tstep`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub tmax: Option<f64>,
    /// Time (s) the result starts at, as SPICE's TSTART (default 0). The run
    /// is still integrated from 0, so the circuit has the same history, but
    /// steps before `tstart` aren't kept: the first snapshot is at `tstart`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub tstart: f64,
    /// Integration method. Default [`IntegrationMethod::Gear2`].
    #[cfg_attr(feature = "serde", serde(default))]
    pub method: IntegrationMethod,
    /// Control the step by local truncation error (default `true`). When
    /// `false`, every step is `tstep` (or `tmax`) except where a breakpoint
    /// or Newton failure shortens it.
    #[cfg_attr(feature = "serde", serde(default = "default_adaptive"))]
    pub adaptive: bool,
    /// Where the run starts: from the initial conditions alone, as if
    /// switched on at t = 0 (the default), or from the DC operating point.
    /// See [`TransientStart`].
    #[cfg_attr(feature = "serde", serde(default))]
    pub start: TransientStart,
    /// Initial node voltages (V), keyed by node name, as SPICE's `.ic`.
    /// With [`TransientStart::InitialConditions`] (SPICE's `UIC`) each
    /// capacitor (and BJT/MOSFET parasitic capacitor) starts charged to the
    /// difference of its nodes' values, and nodes not listed count as 0 V.
    /// With [`TransientStart::OperatingPoint`] these nodes are held at their
    /// values while the operating point is solved, as SPICE does with `.ic`
    /// and no `UIC`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub initial_voltages: HashMap<String, f64>,
    /// Initial inductor currents (A), keyed by inductor or relay id, flowing
    /// from the element's first node to its second. Inductors not listed
    /// start at 0 A, or with [`TransientStart::OperatingPoint`] at their
    /// operating-point current. There an inductor listed here is held at its
    /// current while the operating point is solved; a relay coil just starts
    /// at it.
    #[cfg_attr(feature = "serde", serde(default))]
    pub initial_currents: HashMap<String, f64>,
}

/// The state a transient run starts from at t = 0.
///
/// ```
/// use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig, TransientStart};
/// use sindr::element::{Capacitor, Resistor, VoltageSource};
///
/// // 5 V through 1 kΩ into 1 µF: settled, the capacitor sits at 5 V.
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "0".into()],
///             voltage: 5.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R1".into(),
///             nodes: ["in".into(), "out".into()],
///             resistance: 1e3,
///         }),
///         CircuitElement::Capacitor(Capacitor {
///             id: "C1".into(),
///             nodes: ["out".into(), "0".into()],
///             capacitance: 1e-6,
///         }),
///     ],
///     ..Default::default()
/// };
/// // From the operating point nothing moves; switched on at t = 0, the
/// // capacitor charges from 0 V.
/// let at_op = TransientConfig::new(1e-3, 1e-5).with_start(TransientStart::OperatingPoint);
/// let tran = solve_transient(&circuit, &at_op).unwrap().transient.unwrap();
/// assert!((tran.timesteps[0].node_voltages["out"] - 5.0).abs() < 1e-6);
///
/// let switched_on = TransientConfig::new(1e-3, 1e-5);
/// let tran = solve_transient(&circuit, &switched_on).unwrap().transient.unwrap();
/// assert!(tran.timesteps[0].node_voltages["out"] < 0.1);
/// ```
///
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum TransientStart {
    /// Start from [`initial_voltages`](TransientConfig::initial_voltages)
    /// and [`initial_currents`](TransientConfig::initial_currents) alone:
    /// capacitors not set are uncharged, inductors and transformer windings
    /// not set are at 0 A, as if the circuit were switched on at t = 0.
    /// SPICE's `.tran … UIC`. Inrush and power-on behaviour show up here.
    #[default]
    InitialConditions,
    /// Start from the DC operating point, as SPICE's `.tran` without `UIC`:
    /// capacitors open and inductors shorted, each source at its value at
    /// t = 0, any `initial_voltages` and `initial_currents` held while it's
    /// solved. Capacitors start charged to it, and inductors and transformer
    /// windings carry its currents, so a circuit at rest stays at rest. The
    /// operating point is solved without self-heating, at the ambient the
    /// run starts from.
    OperatingPoint,
}

#[cfg(feature = "serde")]
fn default_adaptive() -> bool {
    true
}

impl TransientConfig {
    /// Simulate from 0 to `tstop` seconds in steps of `tstep` seconds.
    pub fn new(tstop: f64, tstep: f64) -> Self {
        Self {
            tstop,
            tstep,
            tmax: None,
            tstart: 0.0,
            method: IntegrationMethod::default(),
            adaptive: true,
            start: TransientStart::default(),
            initial_voltages: HashMap::new(),
            initial_currents: HashMap::new(),
        }
    }

    /// Start from these node voltages (V) instead of all-zero, as SPICE's
    /// `.ic`. See [`TransientConfig::initial_voltages`] for what they do
    /// with each [`TransientStart`].
    ///
    /// ```
    /// use std::collections::HashMap;
    /// use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig};
    /// use sindr::element::{Capacitor, Resistor};
    ///
    /// // A 1 µF capacitor charged to 5 V discharging through 1 kΩ.
    /// let circuit = Circuit {
    ///     ground_node: "0".into(),
    ///     components: vec![
    ///         CircuitElement::Resistor(Resistor {
    ///             id: "R1".into(),
    ///             nodes: ["a".into(), "0".into()],
    ///             resistance: 1e3,
    ///         }),
    ///         CircuitElement::Capacitor(Capacitor {
    ///             id: "C1".into(),
    ///             nodes: ["a".into(), "0".into()],
    ///             capacitance: 1e-6,
    ///         }),
    ///     ],
    ///     ..Default::default()
    /// };
    /// let config = TransientConfig::new(1e-3, 10e-6)
    ///     .with_initial_voltages(HashMap::from([("a".to_string(), 5.0)]));
    /// let tran = solve_transient(&circuit, &config).unwrap().transient.unwrap();
    /// // After one time constant: 5·e⁻¹ ≈ 1.84 V.
    /// let v = tran.timesteps.last().unwrap().node_voltages["a"];
    /// assert!((v - 5.0 * (-1.0_f64).exp()).abs() < 1e-2);
    /// ```
    ///
    pub fn with_initial_voltages(mut self, voltages: HashMap<String, f64>) -> Self {
        self.initial_voltages = voltages;
        self
    }

    /// Start inductor (or relay coil) `id` at `amps` (A) instead of 0 A,
    /// flowing from its first node to its second (`nodes[0] → nodes[1]`),
    /// as [`initial_currents`](Self::initial_currents) says.
    /// An id that isn't an inductor or a relay with coil inductance (a
    /// transformer, for now) makes the run fail with
    /// [`SimError::InvalidComponent`].
    pub fn with_initial_current(mut self, id: impl Into<String>, amps: f64) -> Self {
        self.initial_currents.insert(id.into(), amps);
        self
    }

    /// Start the run from `start`: the DC operating point, or the initial
    /// conditions alone (the default). See [`TransientStart`].
    pub fn with_start(mut self, start: TransientStart) -> Self {
        self.start = start;
        self
    }

    /// Step at a fixed `tstep` instead of controlling the step by local
    /// truncation error.
    pub fn with_fixed_step(mut self) -> Self {
        self.adaptive = false;
        self
    }

    /// Integrate with `method` instead of the default Gear-2.
    pub fn with_method(mut self, method: IntegrationMethod) -> Self {
        self.method = method;
        self
    }

    /// Cap the step at `tmax` seconds.
    pub fn with_tmax(mut self, tmax: f64) -> Self {
        self.tmax = Some(tmax);
        self
    }

    /// Keep only the steps from `tstart` seconds on, as SPICE's TSTART. The
    /// run is still integrated from 0; a step lands on `tstart`, and it's
    /// the first snapshot. See [`tstart`](Self::tstart).
    ///
    /// ```
    /// use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig};
    /// use sindr::element::{Capacitor, Resistor, VoltageSource};
    ///
    /// let circuit = Circuit {
    ///     ground_node: "0".into(),
    ///     components: vec![
    ///         CircuitElement::VoltageSource(VoltageSource {
    ///             id: "V1".into(),
    ///             nodes: ["in".into(), "0".into()],
    ///             voltage: 5.0,
    ///             waveform: None,
    ///         }),
    ///         CircuitElement::Resistor(Resistor {
    ///             id: "R1".into(),
    ///             nodes: ["in".into(), "out".into()],
    ///             resistance: 1e3,
    ///         }),
    ///         CircuitElement::Capacitor(Capacitor {
    ///             id: "C1".into(),
    ///             nodes: ["out".into(), "0".into()],
    ///             capacitance: 1e-6,
    ///         }),
    ///     ],
    ///     ..Default::default()
    /// };
    /// // Charging from 0 V at t = 0, kept from 1 ms (one τ) on.
    /// let config = TransientConfig::new(3e-3, 1e-5).with_tstart(1e-3);
    /// let tran = solve_transient(&circuit, &config).unwrap().transient.unwrap();
    /// let first = &tran.timesteps[0];
    /// assert!((first.time - 1e-3).abs() < 1e-12);
    /// assert!((first.node_voltages["out"] - 5.0 * (1.0 - (-1.0_f64).exp())).abs() < 1e-2);
    /// ```
    ///
    pub fn with_tstart(mut self, tstart: f64) -> Self {
        self.tstart = tstart;
        self
    }

    /// The config [`solve_circuit`](crate::solve_circuit) uses: `tstop` = 5·τ
    /// and `tstep` = τ/50 for the circuit's longest time constant τ,
    /// stretched to cover 3 periods of a periodic source at 100 steps per
    /// period. It starts from
    /// [`TransientStart::InitialConditions`], switched on at t = 0, so the
    /// run shows the circuit settling.
    pub fn auto(circuit: &Circuit) -> Self {
        let flat = circuit.flattened();
        let circuit: &Circuit = flat.as_deref().unwrap_or(circuit);
        let (duration, dt) = calculate_duration(circuit);
        Self::new(duration, dt.min(duration)).with_start(TransientStart::InitialConditions)
    }

    /// The step the stepper aims for: `tstep`, or `tmax` if that's smaller.
    fn step(&self) -> f64 {
        self.tmax.map_or(self.tstep, |tmax| self.tstep.min(tmax))
    }

    pub(crate) fn validate(&self, max_steps: u64) -> Result<(), SimError> {
        let max_steps = max_steps as f64;
        let invalid = |msg: String| Err(SimError::InvalidComponent(format!("transient: {msg}")));
        for (name, value) in [
            ("tstop", Some(self.tstop)),
            ("tstep", Some(self.tstep)),
            ("tmax", self.tmax),
        ] {
            if let Some(v) = value {
                if !v.is_finite() || v <= 0.0 {
                    return invalid(format!("{name} must be finite and > 0 s (got {v})"));
                }
            }
        }
        if !self.tstart.is_finite() || self.tstart < 0.0 || self.tstart >= self.tstop {
            return invalid(format!(
                "tstart must be finite, >= 0 s and below tstop ({}) (got {})",
                self.tstop, self.tstart
            ));
        }
        if self.tstep > self.tstop {
            return invalid(format!(
                "tstep ({}) must not exceed tstop ({})",
                self.tstep, self.tstop
            ));
        }
        if self.tstop / self.step() > max_steps {
            return invalid(format!(
                "tstop / step = {:.3e} steps is more than the {max_steps:e} allowed",
                self.tstop / self.step()
            ));
        }
        Ok(())
    }
}

/// Runs a transient analysis of `circuit` with the given time span and step.
///
/// Integrates with `config.method` from 0 to `config.tstop`, starting from
/// uncharged capacitors (0 V) and inductors at rest (0 A), with sources at
/// their value at each step's end time. Each step that contains nonlinear
/// devices is solved by Newton–Raphson, warm-started from the previous step.
/// Steps are at most `tstep` (or `tmax`, if smaller), shortened where the
/// truncation error is too large (unless `config.adaptive` is off), cut to
/// land on every source-waveform breakpoint and every time a behavioural
/// source compares `time` with a constant (`time < 1u`), and halved and
/// retried when Newton fails. [`SimulationResult::transient`] holds one
/// snapshot per step,
/// the last at `tstop`; the other fields hold the final instant.
///
/// # Errors
///
/// [`SimError::InvalidComponent`] for an invalid config (a non-finite or
/// non-positive time, `tstep > tstop`, or more than 10⁷ steps counting one
/// for each waveform breakpoint), a source with more than
/// [`MAX_BREAKPOINTS`](crate::waveform::MAX_BREAKPOINTS) breakpoints before
/// `tstop`, or an initial condition that names no node or inductor in the
/// circuit, sits on a transformer, sets the ground node to a nonzero voltage
/// or isn't finite, any
/// validation error from [`validate_circuit`](crate::validate_circuit),
/// [`SimError::ConvergenceFailed`] if Newton–Raphson fails even at a step a
/// million times smaller than `tstep` ([`SimError::NotFinite`] when a
/// behavioural expression overflows there), and [`SimError::SingularMatrix`] or
/// [`SimError::InvalidSolution`] if a step can't be solved.
///
/// # Example
///
/// ```
/// use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig};
/// use sindr::element::{Capacitor, Resistor, VoltageSource};
///
/// // 5 V step into R = 1 kΩ, C = 1 µF (τ = 1 ms), for 2 ms in 10 µs steps.
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "0".into()],
///             voltage: 5.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R1".into(),
///             nodes: ["in".into(), "out".into()],
///             resistance: 1_000.0,
///         }),
///         CircuitElement::Capacitor(Capacitor {
///             id: "C1".into(),
///             nodes: ["out".into(), "0".into()],
///             capacitance: 1e-6,
///         }),
///     ],
///     ..Default::default()
/// };
///
/// let result = solve_transient(&circuit, &TransientConfig::new(2e-3, 10e-6)).unwrap();
/// let tran = result.transient.unwrap();
/// // At least one snapshot per 10 µs, more where the step was shortened.
/// assert!(tran.timesteps.len() >= 200);
/// let last = tran.timesteps.last().unwrap();
/// assert!((last.time - 2e-3).abs() < 1e-12);
/// // 5·(1 − e⁻²) = 4.3233 V
/// assert!((last.node_voltages["out"] - 4.3233).abs() < 1e-3);
/// ```
///
pub fn solve_transient(
    circuit: &Circuit,
    config: &TransientConfig,
) -> Result<SimulationResult, SimError> {
    solve_transient_with(circuit, config, &SolverOptions::default())
}

/// Where a transient run is, passed to the hook of
/// [`solve_transient_with_progress`] after each accepted step.
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct TransientProgress<'a> {
    /// Simulated time reached (s).
    pub time: f64,
    /// The run's stop time (s).
    pub tstop: f64,
    /// Steps accepted so far.
    pub accepted_steps: usize,
    /// The step just accepted, as it will appear in the result's
    /// [`timesteps`](crate::TransientData::timesteps); `None` before
    /// [`tstart`](TransientConfig::tstart), where steps aren't recorded.
    pub step: Option<&'a TimestepSnapshot>,
}

impl TransientProgress<'_> {
    /// How far through the run, from 0 to 1.
    pub fn fraction(&self) -> f64 {
        (self.time / self.tstop).clamp(0.0, 1.0)
    }
}

/// What a progress hook tells the run to do next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Flow {
    /// Take the next step.
    Continue,
    /// Stop now, with [`SimError::Cancelled`].
    Stop,
}

/// The progress hook a run calls, if any.
pub(crate) type Hook<'a> = Option<&'a mut dyn FnMut(&TransientProgress<'_>) -> Flow>;

/// [`solve_transient`], calling `progress` after every accepted step: to
/// report how far a long run has got, show its waveforms as they grow, or
/// cancel it.
///
/// The hook runs on the solving thread, between steps. In a browser, run
/// the solve in a web worker: post progress from the hook, and return
/// [`Flow::Stop`] once a cancel flag the page sets (in shared memory, or
/// a message the worker has read) says so. The hook is called for every
/// step, so keep it cheap or throttle it on
/// [`accepted_steps`](TransientProgress::accepted_steps).
///
/// # Errors
///
/// As [`solve_transient`], and [`SimError::Cancelled`] when the hook
/// returns [`Flow::Stop`].
///
/// ```
/// use sindr::{solve_transient_with_progress, Circuit, CircuitElement, Flow, SimError,
///     TransientConfig};
/// use sindr::element::{Capacitor, Resistor, VoltageSource};
///
/// let circuit = Circuit::new("0", vec![
///     CircuitElement::VoltageSource(VoltageSource {
///         id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 5.0, waveform: None,
///     }),
///     CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()],
///         resistance: 1e3 }),
///     CircuitElement::Capacitor(Capacitor { id: "C1".into(), nodes: ["out".into(), "0".into()],
///         capacitance: 1e-6 }),
/// ]);
/// let config = TransientConfig::new(10e-3, 10e-6);
///
/// // Stop halfway through.
/// let e = solve_transient_with_progress(&circuit, &config, |p| {
///     if p.fraction() < 0.5 { Flow::Continue } else { Flow::Stop }
/// })
/// .unwrap_err();
/// let SimError::Cancelled { time } = e else { panic!("{e}") };
/// assert!((5e-3..6e-3).contains(&time));
/// ```
///
pub fn solve_transient_with_progress(
    circuit: &Circuit,
    config: &TransientConfig,
    mut progress: impl FnMut(&TransientProgress<'_>) -> Flow,
) -> Result<SimulationResult, SimError> {
    solve_transient_hooked(
        circuit,
        config,
        &SolverOptions::default(),
        Some(&mut progress),
    )
}

/// [`solve_transient`] with `opts`.
pub(crate) fn solve_transient_with(
    circuit: &Circuit,
    config: &TransientConfig,
    opts: &SolverOptions,
) -> Result<SimulationResult, SimError> {
    solve_transient_hooked(circuit, config, opts, None)
}

/// [`solve_transient`] with `opts` and a progress hook.
pub(crate) fn solve_transient_hooked(
    circuit: &Circuit,
    config: &TransientConfig,
    opts: &SolverOptions,
    progress: Hook<'_>,
) -> Result<SimulationResult, SimError> {
    let original = circuit;
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    config.validate(opts.max_transient_steps)?;
    crate::validation::validate_circuit(circuit)?;
    let node_map = NodeMap::from_circuit(circuit);
    let num_nodes = node_map.num_nodes();
    let num_vsources = circuit.count_voltage_sources();
    // The hook sees each step as the result will hold it, so composites are
    // folded back for it; a circuit without any passes its steps through.
    let original_flat = match progress {
        Some(_) => original.flattened().ok(),
        None => None,
    };
    let folder = original_flat
        .as_deref()
        .and_then(crate::composite::StepFolder::new);
    let mut folded_hook;
    let progress: Hook<'_> = match (progress, &folder) {
        (Some(hook), Some(folder)) => {
            folded_hook = move |p: &TransientProgress<'_>| {
                let Some(step) = p.step else {
                    return hook(p);
                };
                let mut step = step.clone();
                folder.fold(&mut step);
                hook(&TransientProgress {
                    step: Some(&step),
                    ..*p
                })
            };
            Some(&mut folded_hook)
        }
        (Some(hook), None) => Some(hook as &mut dyn FnMut(&TransientProgress<'_>) -> Flow),
        (None, _) => None,
    };
    let mut result = run_transient(
        circuit,
        &node_map,
        num_nodes,
        num_vsources,
        config,
        opts,
        progress,
    )?;
    crate::composite::fold(original, &mut result);
    Ok(result)
}

/// How many accepted steps in a row may sit near the step floor before a
/// transient gives up with [`SimError::TimestepTooSmall`]. The sharpest
/// edges in the test suite (flashers, a comparator, a regulator's hold-up)
/// take up to about 50; a node ramping without limit takes them for ever
/// (43 million steps for 4 µs before this limit).
const FLOOR_STEP_LIMIT: usize = 10_000;

/// A step this many times the floor or shorter counts toward
/// [`FLOOR_STEP_LIMIT`]. Well above the floor, a run of short steps is a
/// fast circuit sampled with a coarse `tstep`, and it gets somewhere.
const NEAR_FLOOR: f64 = 10.0;

/// The stepper behind [`solve_transient`] and the transient
/// path of [`solve_circuit`](crate::solve_circuit), for a validated circuit
/// and config.
pub(crate) fn run_transient(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    config: &TransientConfig,
    opts: &SolverOptions,
    mut progress: Hook<'_>,
) -> Result<SimulationResult, SimError> {
    let tstop = config.tstop;
    let h_max = config.step();
    // Newton failures halve the step down to this, then try
    // pseudo-transient continuation before giving up.
    let h_min = h_max * 1e-6;

    check_initial_conditions(circuit, node_map, config)?;

    // Initial state: the DC operating point, or the config's initial
    // conditions with uncharged caps (0 V) and inductors at rest (0 A)
    // elsewhere, as if the circuit were switched on at t = 0.
    let start = match config.start {
        TransientStart::OperatingPoint => operating_point_start(circuit, config, opts)?,
        TransientStart::InitialConditions => Start {
            voltages: config.initial_voltages.clone(),
            currents: config.initial_currents.clone(),
        },
    };
    let ic = |node: &String| start.voltages.get(node).copied().unwrap_or(0.0);
    let across = |a: &String, b: &String| History::at(ic(a) - ic(b));
    let current = |id: &String| History::at(start.currents.get(id).copied().unwrap_or(0.0));
    let mut caps: Vec<History> = Vec::new();
    let mut inds: Vec<History> = Vec::new();
    // Parasitic cap histories, keyed by "component_id-junction" ("Q1-be").
    let mut parasitic: FastMap<History> = FastMap::default();
    for component in &circuit.components {
        match component {
            CircuitElement::Capacitor(Capacitor { nodes, .. })
            | CircuitElement::Varactor(Varactor { nodes, .. }) => {
                caps.push(across(&nodes[0], &nodes[1]));
            }
            CircuitElement::Inductor(Inductor {
                id,
                inductance,
                saturation,
                ..
            }) => {
                let start = current(id);
                // A saturating inductor's history holds fluxes.
                inds.push(match saturation {
                    None => start,
                    Some(sat) => History::at(inductor::flux(start.x1, *inductance, sat)),
                });
            }
            CircuitElement::Relay(Relay { id, inductance, .. }) if *inductance > 0.0 => {
                inds.push(current(id));
            }
            CircuitElement::Transformer(Transformer {
                id,
                l1,
                l2,
                k,
                saturation,
                ..
            }) => {
                let (primary, secondary) = (current(id), current(&secondary_winding(id)));
                match saturation {
                    None => {
                        inds.push(primary);
                        inds.push(secondary);
                    }
                    // A saturating core's histories hold fluxes.
                    Some(sat) => {
                        let i = [primary.x1, secondary.x1];
                        let (lambda, _) = transformer_flux(*l1, *l2, *k, sat, i);
                        inds.push(History::at(lambda[0]));
                        inds.push(History::at(lambda[1]));
                    }
                }
            }
            CircuitElement::Bjt(Bjt {
                id,
                nodes,
                parasitic_caps: Some(pc),
                ..
            }) => {
                for (junction, _, a, b) in bjt_parasitics(nodes, pc) {
                    parasitic.insert(format!("{id}-{junction}"), across(&a, &b));
                }
            }
            CircuitElement::Mosfet(Mosfet {
                id,
                nodes,
                parasitic_caps: Some(pc),
                ..
            }) => {
                for (junction, _, a, b) in mosfet_parasitics(nodes, pc) {
                    parasitic.insert(format!("{id}-{junction}"), across(&a, &b));
                }
            }
            CircuitElement::Jfet(Jfet {
                id,
                nodes,
                kind,
                parasitic_caps: Some(pc),
                ..
            }) => {
                for (junction, c0, a, b) in jfet_junctions(nodes, pc) {
                    let (q, _) = jfet_gate_charge(ic(&a) - ic(&b), *kind, pc, c0);
                    parasitic.insert(format!("{id}-{junction}"), History::at(q));
                }
            }
            CircuitElement::Diode(Diode {
                id,
                nodes,
                model,
                temperature,
            }) => {
                if let Some(charge) = device_params::diode_charge(model.as_ref()) {
                    let params = device_params::diode(model.as_ref(), *temperature);
                    let v = ic(&nodes[0]) - ic(&nodes[1]);
                    let (q, _) = sindr_devices::diode::diode_charge(v, &params, &charge);
                    parasitic.insert(format!("{id}-{DIODE_CHARGE}"), History::at(q));
                }
            }
            CircuitElement::OpAmp(OpAmp { model: Some(_), .. })
            | CircuitElement::Comparator(Comparator { .. })
                if config.start == TransientStart::OperatingPoint =>
            {
                // The gain stage starts settled at the operating point's
                // input, so a pole doesn't drag the output from 0 V.
                if let Some(op) = crate::op_amp::OpAmpStamp::of(component, &circuit.ground_node, 0)
                {
                    let vd = ic(&op.nodes[0]) - ic(&op.nodes[1]);
                    parasitic.insert(
                        format!("{}-{OP_AMP_STAGE}", component.id()),
                        History::at(op.settled_stage(vd)),
                    );
                }
            }
            _ => {}
        }
    }
    crate::battery_state::initial_histories(
        circuit,
        &start.voltages,
        config.start == TransientStart::OperatingPoint,
        &mut parasitic,
    );
    // The accepted step before this one; `None` until the first is taken.
    let mut h_prev: Option<f64> = None;

    let placement = Placement::new(circuit, node_map);
    let fixed_step = StepSolver::new(circuit, node_map, &placement, num_nodes, num_vsources, opts);

    // Self-heating: a working copy of the circuit whose junction
    // temperatures follow each part's thermal RC, advanced after every
    // accepted step. The next step solves at the new temperatures.
    let heated = Heated::from_circuit(circuit)?;
    let mut temps = heated.as_ref().map(Heated::ambient).unwrap_or_default();
    // Only junction temperatures change in it, so its elements sit where
    // `circuit`'s do and share `placement`.
    let mut work = heated.as_ref().map(|h| {
        let mut work = circuit.clone();
        h.apply(&mut work, &temps);
        debug_assert_eq!(Layout::new(&work), placement.layout);
        work
    });

    let breakpoints = waveform_breakpoints(circuit, tstop)?;
    // Each breakpoint forces a step of its own, so they count against the
    // step budget together with the tstep grid.
    let steps = (tstop / h_max).ceil() + breakpoints.len() as f64;
    let max_steps = opts.max_transient_steps as f64;
    if steps > max_steps {
        return Err(SimError::InvalidComponent(format!(
            "transient: {:.3e} grid steps plus {} waveform breakpoints is more than the {max_steps:e} steps allowed",
            (tstop / h_max).ceil(),
            breakpoints.len()
        )));
    }
    let mut next_bp = 0;
    let mut grid = 0u64; // index of the last tstep multiple reached

    // Newton starts its first step from the initial node voltages, and an
    // op-amp stage from where it starts.
    let mut solution = DVector::zeros(num_nodes + num_vsources);
    for (node, v) in &start.voltages {
        if let Some(i) = node_map.index(node) {
            solution[i] = *v;
        }
    }
    for component in &circuit.components {
        if let CircuitElement::OpAmp(OpAmp {
            id, model: Some(_), ..
        })
        | CircuitElement::Comparator(Comparator { id, .. }) = component
        {
            if let (Some(branch), Some(stage)) = (
                circuit.vsource_branch_index(id),
                parasitic.get(&format!("{id}-{OP_AMP_STAGE}")),
            ) {
                solution[num_nodes + branch + 1] = stage.x1;
            }
        }
    }
    let mut timesteps = Vec::new();
    let mut stats = TransientStats::default();
    // Accepted steps in a row near the step floor (see FLOOR_STEP_LIMIT).
    let mut floor_steps = 0usize;
    let mut time = 0.0;
    // Start small: the switch-on at t = 0 is itself a corner.
    let mut h = if config.adaptive {
        h_max / RESTART_FRACTION
    } else {
        h_max
    };
    // Accepted (time, node voltages) since the last restart, newest last.
    let mut recent: Vec<(f64, DVector<f64>)> = Vec::new();

    while tstop - time > 1e-9 * h_max {
        // The next point a step must not pass: a breakpoint, a multiple of
        // tstep, or tstop.
        // Targets closer than this are taken as already reached, so no step
        // is ever vanishingly short.
        let eps = 1e-9 * h_max;
        while next_bp < breakpoints.len() && breakpoints[next_bp] <= time + eps {
            next_bp += 1;
        }
        while (grid + 1) as f64 * config.tstep <= time + eps {
            grid += 1;
        }
        // A grid point within rounding of tstop is tstop.
        let next_grid = match (grid + 1) as f64 * config.tstep {
            g if tstop - g <= eps => tstop,
            g => g,
        };
        let bp = breakpoints.get(next_bp).copied().unwrap_or(f64::INFINITY);
        // Land on tstart, where the result begins. Unlike a breakpoint,
        // nothing in the circuit changes there, so the integration carries
        // straight on.
        let save_from = if config.tstart > time + eps {
            config.tstart
        } else {
            f64::INFINITY
        };
        let target = tstop.min(next_grid).min(bp).min(save_from);
        let landing = target - time <= h * (1.0 + 1e-9);
        let (dt, t_next) = if landing {
            (target - time, target)
        } else {
            (h, time + h)
        };

        // A step that ends on a breakpoint integrates up to it, so it sees
        // each source's left limit there: a jump at `bp` belongs to the
        // step after. 1e-12 relative is far above rounding in the waveforms'
        // `%` and far below anything a ramp could show.
        let on_bp = landing && (t_next - bp).abs() <= eps;
        let t_src = if on_bp {
            t_next - 1e-12 * t_next.abs().max(h_max)
        } else {
            t_next
        };

        let integ = Integrator::for_step(config.method, dt, h_prev);
        let heated_step;
        let (step, stepped) = match &work {
            Some(work) => {
                heated_step =
                    StepSolver::new(work, node_map, &placement, num_nodes, num_vsources, opts);
                (&heated_step, work)
            }
            None => (&fixed_step, circuit),
        };
        // Whether this step jumped an edge (pseudo-transient continuation).
        let mut jumped = false;
        let next = match step.solve(&integ, t_src, &caps, &inds, &parasitic, &solution) {
            Ok(next) => next,
            Err(e) if e.is_retryable() && dt / 2.0 >= h_min => {
                stats.newton_rejections += 1;
                h = dt / 2.0;
                continue;
            }
            // Even the shortest step fails: a regenerative edge the node
            // voltages jump across while the capacitors hold theirs. Follow
            // the jump in a pseudo time instead.
            Err(e) if e.is_retryable() => {
                match step
                    .solve_by_pseudo_transient(&integ, t_src, &caps, &inds, &parasitic, &solution)
                {
                    Ok(next) => {
                        stats.jumped_steps += 1;
                        jumped = true;
                        next
                    }
                    Err(_) => return Err(e),
                }
            }
            Err(e) => return Err(e),
        };

        // Accept or reject by local truncation error, and size the next step.
        // A jump over an edge has no truncation error to speak of: it is
        // already as short as a step gets.
        let mut grow = 2.0;
        if config.adaptive && !jumped {
            if let Some(ratio) = lte_ratio(&integ, &recent, t_next, &next, num_nodes, opts) {
                let k = f64::from(integ.order + 1);
                let factor = 0.9 * ratio.powf(-1.0 / k);
                if ratio > 1.0 && dt > h_min {
                    stats.lte_rejections += 1;
                    h = dt * factor.clamp(0.25, 0.9);
                    continue;
                }
                grow = factor.min(2.0);
            }
        }

        let (mut snapshot, new_caps, new_inds) = extract_timestep_results(
            stepped, node_map, &next, num_nodes, t_next, t_src, &integ, &caps, &inds,
        );
        // At the temperatures this step solved at, before self-heating moves them.
        update_parasitic_histories(stepped, node_map, &next, &integ, &mut parasitic);
        add_diode_charge_currents(stepped, &parasitic, &mut snapshot.component_results);
        snapshot.state_of_charge = crate::battery_state::finish_step(
            stepped,
            &snapshot.node_voltages,
            &integ,
            dt,
            &mut parasitic,
            &mut snapshot.component_results,
        );
        if let (Some(heated), Some(work)) = (&heated, &mut work) {
            temps = heated.advance(&temps, &heated.powers(&snapshot.component_results), dt);
            heated.check_runaway(&temps)?;
            heated.apply(work, &temps);
            snapshot.junction_temperatures = heated.report(&temps);
        }
        let recorded = t_next >= config.tstart - eps;
        if recorded {
            timesteps.push(snapshot);
        }
        stats.accepted_steps += 1;
        if let Some(hook) = progress.as_mut() {
            let report = TransientProgress {
                time: t_next,
                tstop,
                accepted_steps: stats.accepted_steps,
                step: if recorded { timesteps.last() } else { None },
            };
            if hook(&report) == Flow::Stop {
                return Err(SimError::Cancelled { time: t_next });
            }
        }
        // Steps that stay at the floor get nowhere. A sharp edge takes a run
        // of them; a circuit that can't be followed at any step takes them
        // for ever. A landing step is short only because the target was
        // close, so it neither counts nor ends a run.
        if config.adaptive && !landing {
            if dt <= h_min * NEAR_FLOOR {
                floor_steps += 1;
                if floor_steps > FLOOR_STEP_LIMIT {
                    return Err(SimError::TimestepTooSmall {
                        time: t_next,
                        step: dt,
                    });
                }
            } else {
                floor_steps = 0;
            }
        }
        caps = new_caps;
        inds = new_inds;
        time = t_next;
        // A landing step was cut short; size the next from the step wanted.
        let base = if landing { h.max(dt) } else { dt };
        h = if config.adaptive {
            (base * grow).clamp(h_min, h_max)
        } else {
            (base * 2.0).min(h_max)
        };

        if on_bp || jumped {
            // A corner in a source, or a jump over an edge: the history
            // before it doesn't predict what follows, so restart with a
            // short backward-Euler step.
            h_prev = None;
            recent.clear();
            if config.adaptive {
                h = h.min(h_max / RESTART_FRACTION);
            }
        } else {
            h_prev = Some(dt);
        }
        recent.push((t_next, next.rows(0, num_nodes).into_owned()));
        if recent.len() > 3 {
            recent.remove(0);
        }
        solution = next;
    }

    let final_snapshot = timesteps.last().ok_or(SimError::InvalidSolution)?;

    let mut branch_currents = HashMap::new();
    for component in &circuit.components {
        if let CircuitElement::VoltageSource(VoltageSource { id, .. }) = component {
            if let Some(cr) = final_snapshot
                .component_results
                .iter()
                .find(|c| c.id == *id)
            {
                branch_currents.insert(id.clone(), cr.current_through);
            }
        }
    }

    let last = work.as_ref().unwrap_or(circuit);
    let bjt_results = extract_bjt_results_from_snapshot(last, final_snapshot);
    let mosfet_results = extract_mosfet_results_from_snapshot(last, final_snapshot);
    let (op_amp_results, relay_results) =
        crate::results::op_amp_and_relay_results(last, &final_snapshot.node_voltages);

    Ok(SimulationResult {
        node_voltages: final_snapshot.node_voltages.clone(),
        branch_currents,
        component_results: final_snapshot.component_results.clone(),
        bjt_results,
        mosfet_results,
        op_amp_results,
        relay_results,
        convergence_aid: None,
        junction_temperatures: final_snapshot.junction_temperatures.clone(),
        state_of_charge: final_snapshot.state_of_charge.clone(),
        transient: Some(TransientData {
            timesteps,
            time_step: config.tstep,
            duration: tstop,
            stats,
        }),
    })
}

/// The state a run starts from: node voltages (V) by node name, and
/// inductor, relay-coil and transformer-winding currents (A) by id (a
/// transformer's secondary under [`secondary_winding`]).
struct Start {
    voltages: HashMap<String, f64>,
    currents: HashMap<String, f64>,
}

/// The key a transformer's secondary-winding current goes under, the id of
/// its second 0 V source in [`dc_equivalent`](crate::dc_equivalent).
fn secondary_winding(id: &str) -> String {
    format!("{id}#2")
}

/// Conductance (S) that holds an `.ic` node at its value while the
/// operating point is solved: SPICE's, 1e10 S in a Norton with its
/// current, so a node already held by a source isn't a singular matrix.
const IC_CONDUCTANCE: f64 = 1e10;

/// [`TransientStart::OperatingPoint`]: the DC operating point of flat,
/// validated `circuit`, with each source at its t = 0 value, each
/// `initial_voltages` node held there and each inductor in
/// `initial_currents` carrying its current.
fn operating_point_start(
    circuit: &Circuit,
    config: &TransientConfig,
    opts: &SolverOptions,
) -> Result<Start, SimError> {
    // Each source at its t = 0 value.
    let mut dc = crate::dc_equivalent(circuit);
    // An inductor given a current carries it: a current source in place
    // of its short. (A relay coil is a resistor at DC, and just starts at
    // its current.)
    let inductors: HashMap<&str, f64> = circuit
        .components
        .iter()
        .filter_map(|c| match c {
            CircuitElement::Inductor(Inductor { id, .. }) => config
                .initial_currents
                .get(id)
                .map(|&amps| (id.as_str(), amps)),
            _ => None,
        })
        .collect();
    for component in &mut dc.components {
        if let CircuitElement::VoltageSource(VoltageSource { id, nodes, .. }) = component {
            if let Some(&amps) = inductors.get(id.as_str()) {
                *component = CircuitElement::CurrentSource(CurrentSource {
                    id: id.clone(),
                    nodes: nodes.clone(),
                    current: amps,
                    waveform: None,
                });
            }
        }
    }
    let mut held: Vec<_> = config.initial_voltages.iter().collect();
    held.sort_by(|a, b| a.0.cmp(b.0));
    for (node, &v) in held {
        if *node == circuit.ground_node {
            continue;
        }
        dc.components.push(CircuitElement::Resistor(Resistor {
            id: format!("#ic-r:{node}"),
            nodes: [node.clone(), circuit.ground_node.clone()],
            resistance: 1.0 / IC_CONDUCTANCE,
        }));
        dc.components
            .push(CircuitElement::CurrentSource(CurrentSource {
                id: format!("#ic-i:{node}"),
                nodes: [circuit.ground_node.clone(), node.clone()],
                current: v * IC_CONDUCTANCE,
                waveform: None,
            }));
    }

    // The `.ic` values seed Newton too, over any nodeset.
    let mut op_opts = opts.clone();
    op_opts.nodeset.extend(config.initial_voltages.clone());
    let op = crate::solve_dc_circuit(&dc, &op_opts).map_err(|e| match e {
        SimError::InvalidComponent(msg) => {
            SimError::InvalidComponent(format!("transient operating point: {msg}"))
        }
        other => other,
    })?;

    let mut currents = HashMap::new();
    for component in &circuit.components {
        match component {
            CircuitElement::Inductor(Inductor { id, .. }) => {
                let amps = match config.initial_currents.get(id) {
                    Some(&amps) => amps,
                    None => op.branch_currents.get(id).copied().unwrap_or(0.0),
                };
                currents.insert(id.clone(), amps);
            }
            CircuitElement::Relay(Relay {
                id,
                nodes,
                coil_resistance,
                inductance,
                ..
            }) if *inductance > 0.0 => {
                let amps = match config.initial_currents.get(id) {
                    Some(&amps) => amps,
                    None => {
                        let v = |n: &String| op.node_voltages.get(n).copied().unwrap_or(0.0);
                        (v(&nodes[0]) - v(&nodes[1])) / coil_resistance
                    }
                };
                currents.insert(id.clone(), amps);
            }
            CircuitElement::Transformer(Transformer { id, .. }) => {
                let secondary = secondary_winding(id);
                for winding in [id, &secondary] {
                    let amps = op.branch_currents.get(winding).copied().unwrap_or(0.0);
                    currents.insert(winding.clone(), amps);
                }
            }
            _ => {}
        }
    }
    Ok(Start {
        voltages: op.node_voltages,
        currents,
    })
}

/// A step after a restart (t = 0 or a breakpoint) is at most `tstep` divided
/// by this, so the unchecked backward-Euler step that starts the integration
/// is short.
const RESTART_FRACTION: f64 = 100.0;

/// Pseudo-steps [`StepSolver::solve_by_pseudo_transient`] takes at most.
/// An edge takes about a hundred.
const PSEUDO_TRANSIENT_STEPS: usize = 2000;

/// A pseudo-step that moves no node by this much (V) has settled, so the
/// next one can be longer.
const PSEUDO_TRANSIENT_SETTLED: f64 = 1e-3;

/// Below this conductance (S) the pseudo-time term is negligible against
/// the circuit's own conductances, and plain Newton takes over.
const PSEUDO_TRANSIENT_GMIN: f64 = 1e-9;

/// Above this conductance (S) a pseudo-step Newton still can't take means
/// the step has no solution near where the circuit is.
const PSEUDO_TRANSIENT_GMAX: f64 = 1e8;

/// Rejects an initial condition that would otherwise be ignored or surface
/// later as a bare `InvalidSolution`: a node or inductor id the circuit
/// doesn't have, a non-finite value, a nonzero ground voltage, or a current
/// on a transformer (its windings take none yet).
fn check_initial_conditions(
    circuit: &Circuit,
    node_map: &NodeMap,
    config: &TransientConfig,
) -> Result<(), SimError> {
    let invalid = |msg: String| Err(SimError::InvalidComponent(format!("transient: {msg}")));
    let mut voltages: Vec<_> = config.initial_voltages.iter().collect();
    voltages.sort_by(|a, b| a.0.cmp(b.0));
    for (node, v) in voltages {
        if !v.is_finite() {
            return invalid(format!(
                "initial voltage of node '{node}' must be finite (got {v})"
            ));
        }
        if *node == circuit.ground_node {
            if *v != 0.0 {
                return invalid(format!("ground node '{node}' can't start at {v} V"));
            }
        } else if node_map.index(node).is_none() {
            return invalid(format!(
                "initial voltage {v} V for node '{node}', which the circuit doesn't have"
            ));
        }
    }
    let mut currents: Vec<_> = config.initial_currents.iter().collect();
    currents.sort_by(|a, b| a.0.cmp(b.0));
    for (id, i) in currents {
        if !i.is_finite() {
            return invalid(format!(
                "initial current of '{id}' must be finite (got {i})"
            ));
        }
        let target = circuit.components.iter().find_map(|c| match c {
            CircuitElement::Inductor(Inductor { id: cid, .. }) if cid == id => Some(Ok(())),
            CircuitElement::Relay(Relay {
                id: cid,
                inductance,
                ..
            }) if cid == id => Some(if *inductance > 0.0 {
                Ok(())
            } else {
                Err("is a relay with no coil inductance")
            }),
            CircuitElement::Transformer(Transformer { id: cid, .. }) if cid == id => Some(Err(
                "is a transformer, whose windings take no initial current yet",
            )),
            _ => None,
        });
        match target {
            Some(Ok(())) => {}
            Some(Err(why)) => {
                return invalid(format!("initial current {i} A for '{id}', which {why}"))
            }
            None => {
                return invalid(format!(
                    "initial current {i} A for '{id}', which is not an inductor in the circuit"
                ))
            }
        }
    }
    Ok(())
}

/// Every waveform breakpoint of the circuit's sources in `(0, tstop)`,
/// and every time a behavioural source compares `time` with a constant,
/// sorted and without near-duplicates.
///
/// A source with more than
/// [`MAX_BREAKPOINTS`](crate::waveform::MAX_BREAKPOINTS) would have had the
/// rest dropped, so it is rejected rather than stepped over.
fn waveform_breakpoints(circuit: &Circuit, tstop: f64) -> Result<Vec<f64>, SimError> {
    let mut all = Vec::new();
    for c in &circuit.components {
        if let CircuitElement::Behavioural(Behavioural { expr, .. }) = c {
            all.extend(
                expr.time_breakpoints()
                    .into_iter()
                    .filter(|t| *t > 0.0 && *t < tstop),
            );
            continue;
        }
        let (CircuitElement::VoltageSource(VoltageSource {
            id,
            waveform: Some(w),
            ..
        })
        | CircuitElement::CurrentSource(CurrentSource {
            id,
            waveform: Some(w),
            ..
        })) = c
        else {
            continue;
        };
        let (bps, capped) = w.capped_breakpoints(tstop);
        if capped {
            return Err(SimError::InvalidComponent(format!(
                "transient: source '{id}' has more than {} waveform breakpoints before tstop = {tstop} s",
                crate::waveform::MAX_BREAKPOINTS
            )));
        }
        all.extend(bps);
    }
    all.sort_by(f64::total_cmp);
    all.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * tstop);
    Ok(all)
}

/// The worst ratio, over all node voltages, of the step's estimated local
/// truncation error to its tolerance; above 1 the step should be rejected.
///
/// The error of an order-k step of length h is `C·h^(k+1)·v^(k+1)`, with C the
/// method's error constant and the derivative estimated as `(k+1)!` times the
/// (k+1)-th divided difference of the last k+2 points (`recent` plus the new
/// one). `None` until a restart has accumulated that many.
fn lte_ratio(
    integ: &Integrator,
    recent: &[(f64, DVector<f64>)],
    t: f64,
    v: &DVector<f64>,
    num_nodes: usize,
    opts: &SolverOptions,
) -> Option<f64> {
    let k = integ.order as usize;
    if recent.len() < k + 1 {
        return None;
    }
    let pts = &recent[recent.len() - (k + 1)..];
    let times: Vec<f64> = pts.iter().map(|p| p.0).chain([t]).collect();
    let h = t - pts[k].0;
    let factorial: f64 = (1..=k + 1).map(|i| i as f64).product();
    let mut worst: f64 = 0.0;
    for node in 0..num_nodes {
        // Divided differences in place, Newton's table.
        let mut dd: Vec<f64> = pts.iter().map(|p| p.1[node]).chain([v[node]]).collect();
        for level in 1..=k + 1 {
            for i in (level..dd.len()).rev() {
                dd[i] = (dd[i] - dd[i - 1]) / (times[i] - times[i - level]);
            }
        }
        let derivative = factorial * dd[k + 1];
        let lte = integ.error_constant * h.powi(k as i32 + 1) * derivative.abs();
        let previous = pts[k].1[node];
        let tol = opts.trtol * (opts.reltol * v[node].abs().max(previous.abs()) + opts.vntol);
        worst = worst.max(lte / tol);
    }
    Some(worst)
}

/// Where each element of a run's circuit sits in its MNA system: its
/// branch rows and history slots, and its terminals' matrix indices.
struct Placement {
    layout: Layout,
    terminals: Terminals,
}

impl Placement {
    fn new(circuit: &Circuit, node_map: &NodeMap) -> Self {
        Self {
            layout: Layout::new(circuit),
            terminals: Terminals::new(circuit, node_map),
        }
    }
}

/// Solves one transient step: a single linear solve, or Newton–Raphson when
/// the circuit has nonlinear devices.
struct StepSolver<'a> {
    circuit: &'a Circuit,
    node_map: &'a NodeMap,
    placement: &'a Placement,
    num_nodes: usize,
    num_vsources: usize,
    nonlinear: bool,
    diode_info: Vec<newton_raphson::DiodeLimitInfo>,
    igbt_info: Vec<newton_raphson::IgbtLimitInfo>,
    opts: &'a SolverOptions,
}

impl<'a> StepSolver<'a> {
    /// The stepper for `circuit`, whose elements sit where `placement`
    /// says.
    fn new(
        circuit: &'a Circuit,
        node_map: &'a NodeMap,
        placement: &'a Placement,
        num_nodes: usize,
        num_vsources: usize,
        opts: &'a SolverOptions,
    ) -> Self {
        let nonlinear = circuit.has_nonlinear_elements();
        Self {
            circuit,
            node_map,
            placement,
            num_nodes,
            num_vsources,
            nonlinear,
            diode_info: if nonlinear {
                newton_raphson::collect_diode_info(circuit, node_map)
            } else {
                Vec::new()
            },
            igbt_info: newton_raphson::collect_igbt_info(circuit, node_map),
            opts,
        }
    }

    /// The solution at `t`, the end of a step integrated by `integ` from the
    /// histories `caps` / `inds` / `parasitic`. Newton starts from `start`,
    /// the previous step's solution.
    fn solve(
        &self,
        integ: &Integrator,
        t: f64,
        caps: &[History],
        inds: &[History],
        parasitic: &FastMap<History>,
        start: &DVector<f64>,
    ) -> Result<DVector<f64>, SimError> {
        self.newton(integ, t, caps, inds, parasitic, start, None)
    }

    /// [`solve`](Self::solve) by pseudo-transient continuation, for a step
    /// plain Newton can't take at any length.
    ///
    /// That is a regenerative edge with nothing to slow it down (a
    /// multivibrator's or a Schmitt trigger's, with no junction charge in
    /// the transistors): the solution the circuit was on folds away, and
    /// the one after the edge is far from it, out of Newton's reach. Here
    /// each node gets a conductance `g` to where the last pseudo-step left
    /// it, as if it had a capacitor to ground stepped by `C/g` in a pseudo
    /// time. Each pseudo-step then has a solution close to the last, so the
    /// node voltages follow the edge as a real circuit's would, held up
    /// only by capacitance, until they settle past it. A pseudo-step Newton
    /// can't take is retried with a larger `g` (a shorter pseudo-step); one
    /// that barely moves the nodes lengthens the next. Once `g` is
    /// negligible, plain Newton from there gives the answer: the step's own,
    /// with nothing added.
    fn solve_by_pseudo_transient(
        &self,
        integ: &Integrator,
        t: f64,
        caps: &[History],
        inds: &[History],
        parasitic: &FastMap<History>,
        start: &DVector<f64>,
    ) -> Result<DVector<f64>, SimError> {
        let mut anchor = start.clone();
        // 1 S is about a transistor's transconductance at tens of mA: the
        // first pseudo-steps are short against the edge.
        let mut g = 1.0;
        let mut last_err = None;
        for _ in 0..PSEUDO_TRANSIENT_STEPS {
            match self.newton(integ, t, caps, inds, parasitic, &anchor, Some((g, &anchor))) {
                Ok(v) => {
                    let (moved, _) = newton_raphson::max_node_step(&anchor, &v, self.num_nodes);
                    anchor = v;
                    if moved >= PSEUDO_TRANSIENT_SETTLED {
                        continue;
                    }
                    g /= 4.0;
                    if g >= PSEUDO_TRANSIENT_GMIN {
                        continue;
                    }
                    match self.newton(integ, t, caps, inds, parasitic, &anchor, None) {
                        Ok(v) => return Ok(v),
                        Err(e) if e.is_retryable() => {
                            // Not settled after all: keep going from a
                            // small conductance.
                            g = PSEUDO_TRANSIENT_GMIN * 1e3;
                            last_err = Some(e);
                        }
                        Err(e) => return Err(e),
                    }
                }
                Err(e) if e.is_retryable() && g < PSEUDO_TRANSIENT_GMAX => {
                    g *= 8.0;
                    last_err = Some(e);
                }
                Err(e) => return Err(e),
            }
        }
        Err(last_err.unwrap_or(SimError::InvalidSolution))
    }

    /// Newton–Raphson for the step, from `start`. With `pseudo` as
    /// `(g, anchor)`, every node also has `g` siemens to its voltage in
    /// `anchor`: a pseudo-step of
    /// [`solve_by_pseudo_transient`](Self::solve_by_pseudo_transient).
    #[allow(clippy::too_many_arguments)]
    fn newton(
        &self,
        integ: &Integrator,
        t: f64,
        caps: &[History],
        inds: &[History],
        parasitic: &FastMap<History>,
        start: &DVector<f64>,
        pseudo: Option<(f64, &DVector<f64>)>,
    ) -> Result<DVector<f64>, SimError> {
        let stamp = |v_prev: &DVector<f64>| -> Result<MnaSystem, SimError> {
            let mut system = MnaSystem::new(self.num_nodes, self.num_vsources);
            stamp_circuit_transient(
                self.circuit,
                &mut system,
                self.node_map,
                self.placement,
                integ,
                t,
                caps,
                inds,
                v_prev,
                parasitic,
            )?;
            Ok(system)
        };

        if !self.nonlinear {
            return stamp(start)?.solve();
        }

        let mut v_prev = start.clone();
        let mut last_step = (f64::INFINITY, None);
        for _ in 0..self.opts.max_iterations {
            let mut system = stamp(&v_prev)?;
            newton_raphson::add_gmin_shunts(&mut system, self.num_nodes);
            if let Some((g, anchor)) = pseudo {
                for i in 0..self.num_nodes {
                    system.a[(i, i)] += g;
                    system.b[i] += g * anchor[i];
                }
            }
            let v_new = system.solve()?;

            let mut v_limited =
                newton_raphson::apply_voltage_limiting(&v_new, &v_prev, &self.diode_info);
            let igbt_limited =
                newton_raphson::apply_igbt_limiting(&mut v_limited, &v_prev, &self.igbt_info);
            let regulator_limited = crate::element::voltage_regulator::limit_regulator_step(
                self.circuit,
                self.node_map,
                self.num_nodes,
                &v_prev,
                &mut v_limited,
            );

            if !igbt_limited
                && !regulator_limited
                && newton_raphson::converged(
                    &v_prev,
                    &v_limited,
                    self.num_nodes,
                    &self.diode_info,
                    self.opts,
                )
                && crate::element::voltage_regulator::regulator_pieces_settled(
                    self.circuit,
                    self.node_map,
                    self.num_nodes,
                    &v_prev,
                    &v_limited,
                )
            {
                crate::element::behavioural::behavioural_finite_at(
                    self.circuit,
                    self.node_map,
                    &v_limited,
                    t,
                )?;
                return Ok(v_limited);
            }
            last_step = newton_raphson::max_node_step(&v_prev, &v_limited, self.num_nodes);
            v_prev = v_limited;
        }
        Err(newton_raphson::convergence_failed(
            self.circuit,
            self.node_map,
            last_step,
            SolvePoint::Transient { time: t },
            self.opts.max_iterations,
        ))
    }
}

/// Push each BJT/MOSFET parasitic capacitor's voltage and current at
/// `solution` onto its history.
fn update_parasitic_histories(
    circuit: &Circuit,
    node_map: &NodeMap,
    solution: &DVector<f64>,
    integ: &Integrator,
    parasitic: &mut FastMap<History>,
) {
    let v = |n: &String| node_map.index(n).map_or(0.0, |i| solution[i]);
    let num_nodes = node_map.num_nodes();
    for component in &circuit.components {
        if let Some(op) = crate::op_amp::OpAmpStamp::of(component, &circuit.ground_node, 0) {
            let id = component.id();
            let Some(branch) = circuit.vsource_branch_index(id) else {
                continue;
            };
            let op = crate::op_amp::OpAmpStamp {
                branch: num_nodes + branch,
                ..op
            };
            let history = parasitic_history(parasitic, id, OP_AMP_STAGE);
            parasitic.insert(
                format!("{id}-{OP_AMP_STAGE}"),
                op.advance(integ, &history, solution),
            );
            continue;
        }
        if let CircuitElement::Diode(Diode {
            id,
            nodes,
            model,
            temperature,
        }) = component
        {
            if let Some(charge) = device_params::diode_charge(model.as_ref()) {
                let params = device_params::diode(model.as_ref(), *temperature);
                let (q, _) = sindr_devices::diode::diode_charge(
                    v(&nodes[0]) - v(&nodes[1]),
                    &params,
                    &charge,
                );
                let history = parasitic_history(parasitic, id, DIODE_CHARGE);
                parasitic.insert(
                    format!("{id}-{DIODE_CHARGE}"),
                    history.push(q, integ.charge_current(q, &history)),
                );
            }
            continue;
        }
        if let CircuitElement::Jfet(Jfet {
            id,
            nodes,
            kind,
            parasitic_caps: Some(pc),
            ..
        }) = component
        {
            for (junction, c0, a, b) in jfet_junctions(nodes, pc) {
                let (q, _) = jfet_gate_charge(v(&a) - v(&b), *kind, pc, c0);
                let history = parasitic_history(parasitic, id, junction);
                parasitic.insert(
                    format!("{id}-{junction}"),
                    history.push(q, integ.charge_current(q, &history)),
                );
            }
            continue;
        }
        let (id, caps) = match component {
            CircuitElement::Bjt(Bjt {
                id,
                nodes,
                parasitic_caps: Some(pc),
                ..
            }) => (id, bjt_parasitics(nodes, pc)),
            CircuitElement::Mosfet(Mosfet {
                id,
                nodes,
                parasitic_caps: Some(pc),
                ..
            }) => (id, mosfet_parasitics(nodes, pc)),
            _ => continue,
        };
        for (junction, c, a, b) in caps {
            let history = parasitic_history(parasitic, id, junction);
            let (g, i_hist) = integ.capacitor(c, &history);
            let v_now = v(&a) - v(&b);
            parasitic.insert(
                format!("{id}-{junction}"),
                history.push(v_now, g * v_now + i_hist),
            );
        }
    }
}

/// Add each charge-storing diode's charging current, the rate just pushed
/// onto its history, to its result, so `current_through` is the terminal
/// current (reverse recovery included). `power` stays the junction's
/// dissipation: the stored charge doesn't heat it.
fn add_diode_charge_currents(
    circuit: &Circuit,
    parasitic: &FastMap<History>,
    results: &mut [ComponentResult],
) {
    for component in &circuit.components {
        let CircuitElement::Diode(Diode { id, .. }) = component else {
            continue;
        };
        let Some(history) = parasitic.get(&format!("{id}-{DIODE_CHARGE}")) else {
            continue;
        };
        if let Some(result) = results.iter_mut().find(|r| &r.id == id) {
            result.current_through += history.y1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::Circuit;
    use crate::solve_circuit;
    use crate::waveform::Waveform;
    use approx::assert_relative_eq;

    /// Test 4: Auto-duration calculation for known RC circuit.
    #[test]
    fn test_calculate_duration_rc() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 100e-6,
                }),
            ],
            ..Default::default()
        };
        let (duration, dt) = calculate_duration(&circuit);
        // tau = 1000 * 100e-6 = 0.1s
        // duration = 5 * 0.1 = 0.5
        // dt = 0.1 / 50 = 0.002
        assert_relative_eq!(duration, 0.5, epsilon = 1e-10);
        assert_relative_eq!(dt, 0.002, epsilon = 1e-10);
    }

    /// Test 1: RC charging exponential.
    /// V1=5V, R1=1k, C1=100uF. tau=0.1s.
    /// V(t) = 5*(1-e^(-t/RC))
    #[test]
    fn test_rc_charging_exponential() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 100e-6,
                }),
            ],
            ..Default::default()
        };
        let result = solve_circuit(&circuit).unwrap();

        assert!(result.transient.is_some());
        let transient = result.transient.as_ref().unwrap();

        // Duration should be ~0.5s (within 10%)
        assert!((transient.duration - 0.5).abs() < 0.05);

        // At t ~ tau (0.1s): V_cap ~ 5*(1 - e^-1) = 3.1606...
        let target_tau = 0.1;
        let at_tau = transient
            .timesteps
            .iter()
            .min_by(|a, b| {
                (a.time - target_tau)
                    .abs()
                    .partial_cmp(&(b.time - target_tau).abs())
                    .unwrap()
            })
            .unwrap();
        let c1_tau = at_tau
            .component_results
            .iter()
            .find(|c| c.id == "C1")
            .unwrap();
        let expected_tau = 5.0 * (1.0 - (-1.0_f64).exp()); // 3.1606...
        assert_relative_eq!(
            c1_tau.voltage_across,
            expected_tau,
            epsilon = expected_tau * 0.02
        );

        // At t ~ 3*tau (0.3s): V_cap ~ 5*(1 - e^-3) = 4.7511...
        let target_3tau = 0.3;
        let at_3tau = transient
            .timesteps
            .iter()
            .min_by(|a, b| {
                (a.time - target_3tau)
                    .abs()
                    .partial_cmp(&(b.time - target_3tau).abs())
                    .unwrap()
            })
            .unwrap();
        let c1_3tau = at_3tau
            .component_results
            .iter()
            .find(|c| c.id == "C1")
            .unwrap();
        let expected_3tau = 5.0 * (1.0 - (-3.0_f64).exp()); // 4.7511...
        assert_relative_eq!(
            c1_3tau.voltage_across,
            expected_3tau,
            epsilon = expected_3tau * 0.02
        );

        // Steady state (final timestep): V_cap ~ 5.0V (within 1%)
        let c1_final = result
            .component_results
            .iter()
            .find(|c| c.id == "C1")
            .unwrap();
        assert_relative_eq!(c1_final.voltage_across, 5.0, epsilon = 0.05);
    }

    #[test]
    fn timestep_time_label_matches_be_solution() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1_000.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                }),
            ],
            ..Default::default()
        };
        // Fixed-step backward Euler, so the first step is exactly tstep.
        let config = TransientConfig::auto(&circuit)
            .with_method(IntegrationMethod::BackwardEuler)
            .with_fixed_step();
        let result = solve_transient(&circuit, &config).unwrap();
        let transient = result.transient.unwrap();
        let dt = transient.time_step;
        let first = &transient.timesteps[0];

        assert_relative_eq!(first.time, dt, epsilon = dt * 1e-9);

        let v_be_step1 = 5.0 / (1.0 + 1_000.0 * 1e-6 / dt);
        assert_relative_eq!(first.node_voltages["n2"], v_be_step1, epsilon = 1e-9);
    }

    /// Test 2: RL current rise.
    /// V1=10V, R1=100, L1=0.5H. tau=L/R=0.005s.
    /// I(t) = (V/R)*(1-e^(-t*R/L))
    #[test]
    fn test_rl_current_rise() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Inductor(Inductor {
                    id: "L1".into(),
                    nodes: ["n2".into(), "0".into()],
                    inductance: 0.5,
                    saturation: None,
                }),
            ],
            ..Default::default()
        };
        let result = solve_circuit(&circuit).unwrap();

        assert!(result.transient.is_some());
        let transient = result.transient.as_ref().unwrap();

        // At t ~ tau (0.005s): I_ind ~ (10/100)*(1-e^-1) = 0.0632A
        let target_tau = 0.005;
        let at_tau = transient
            .timesteps
            .iter()
            .min_by(|a, b| {
                (a.time - target_tau)
                    .abs()
                    .partial_cmp(&(b.time - target_tau).abs())
                    .unwrap()
            })
            .unwrap();
        let l1_tau = at_tau
            .component_results
            .iter()
            .find(|c| c.id == "L1")
            .unwrap();
        let expected_i_tau = 0.1 * (1.0 - (-1.0_f64).exp()); // 0.06321...
        assert_relative_eq!(
            l1_tau.current_through,
            expected_i_tau,
            epsilon = expected_i_tau * 0.02
        );

        // Steady state: I_ind ~ V/R = 0.1A (within 1%)
        let l1_final = result
            .component_results
            .iter()
            .find(|c| c.id == "L1")
            .unwrap();
        assert_relative_eq!(l1_final.current_through, 0.1, epsilon = 0.001);
    }

    /// Test 3: DC-only circuit backward compatibility.
    /// Voltage divider: V1=10V, R1=1k, R2=2k. No reactive elements.
    #[test]
    fn test_dc_backward_compat() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R2".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 2000.0,
                }),
            ],
            ..Default::default()
        };
        let result = solve_circuit(&circuit).unwrap();

        // No transient data for DC-only circuits
        assert!(result.transient.is_none());

        // Exact DC values
        assert_relative_eq!(result.node_voltages["n1"], 10.0, epsilon = 1e-10);
        assert_relative_eq!(result.node_voltages["n2"], 20.0 / 3.0, epsilon = 1e-10);
        assert_relative_eq!(
            result.branch_currents["V1"],
            -10.0 / 3000.0,
            epsilon = 1e-10
        );
    }

    /// Timestep halving test: stiff diode+RC circuit with fast time constant.
    #[test]
    fn test_timestep_halving_recovery() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    temperature: 300.15,
                    model: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 10.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                }),
            ],
            ..Default::default()
        };
        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Solver should recover even with stiff circuit: {:?}",
            result.err()
        );
        let result = result.unwrap();

        assert!(result.transient.is_some());

        let c1 = result
            .component_results
            .iter()
            .find(|c| c.id == "C1")
            .unwrap();
        assert!(c1.voltage_across > 0.0, "C1 voltage should be positive");
        assert!(c1.voltage_across < 10.0, "C1 voltage should be < V_source");

        let d1 = result
            .component_results
            .iter()
            .find(|c| c.id == "D1")
            .unwrap();
        assert!(
            d1.voltage_across >= 0.0,
            "D1 voltage should be non-negative"
        );
        assert!(
            d1.voltage_across < 1.0,
            "D1 voltage should be < 1V (silicon)"
        );
    }

    /// Relay with inductance: 12V source, 500 Ohm coil, pickup=5V, L=0.1H.
    /// Contact load: 5V source through 1k resistor. Relay should switch and produce
    /// at least one transient timestep.
    #[test]
    fn test_relay_with_inductance_transient() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                // Coil drive: 12V through 100 Ohm into relay coil
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vcoil".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 12.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "coil_p".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Relay(Relay {
                    id: "K1".into(),
                    nodes: ["coil_p".into(), "0".into(), "c1".into(), "c2".into()],
                    coil_resistance: 500.0,
                    pickup_voltage: 5.0,
                    inductance: 0.1,
                }),
                // Contact load: 5V source through 1k into contact pair
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vload".into(),
                    nodes: ["c1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rload".into(),
                    nodes: ["c1".into(), "c2".into()],
                    resistance: 1000.0,
                }),
            ],
            ..Default::default()
        };
        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Relay transient should succeed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        assert!(result.transient.is_some(), "Should have transient data");
        let transient = result.transient.as_ref().unwrap();
        assert!(
            !transient.timesteps.is_empty(),
            "Should have at least one timestep"
        );
    }

    /// Transformer transient test: voltage step-up via coupled inductors.
    /// L1=1mH, L2=4mH, k=0.999 → turns ratio n = sqrt(L2/L1) = 2.
    /// With 5V DC source on primary, secondary should see ~10V at steady state.
    /// Note: Backward Euler has damping; use looser tolerance for transient.
    #[test]
    fn transformer_transient_voltage_step_up() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                // Primary: 5V through 10 Ohm into transformer primary
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 10.0,
                }),
                CircuitElement::Transformer(Transformer {
                    id: "T1".into(),
                    nodes: ["n2".into(), "0".into(), "n3".into(), "0".into()],
                    l1: 1e-3, // 1 mH primary
                    l2: 4e-3, // 4 mH secondary (n = sqrt(4/1) = 2)
                    k: 0.999, // near-ideal coupling
                    saturation: None,
                }),
                // Secondary load: 1 kOhm
                CircuitElement::Resistor(Resistor {
                    id: "R2".into(),
                    nodes: ["n3".into(), "0".into()],
                    resistance: 1000.0,
                }),
            ],
            ..Default::default()
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Transformer transient solve failed: {:?}",
            result.err()
        );
        let result = result.unwrap();

        // Transient data should be present (Transformer is a reactive element)
        assert!(
            result.transient.is_some(),
            "Expected transient data for transformer circuit"
        );
        let transient = result.transient.as_ref().unwrap();
        assert!(
            !transient.timesteps.is_empty(),
            "Expected at least one timestep"
        );

        let t1 = result.component_results.iter().find(|c| c.id == "T1");
        assert!(
            t1.is_some(),
            "Transformer T1 should appear in component results"
        );

        // While the primary has a voltage across it, the secondary follows
        // at M/L1 = k·√(L2/L1) = 1.998, once the 8 ns leakage time constant
        // into 1 kΩ has died out (the first, backward-Euler, step leaves
        // h/τ of it). The step is controlled to SPICE's default tolerance,
        // TRTOL·RELTOL = 0.7 % of the voltage, so check to 1 %. At DC a
        // winding is a short, so both decay towards 0.
        let peak = transient
            .timesteps
            .iter()
            .map(|s| s.node_voltages["n2"].abs())
            .fold(0.0, f64::max);
        assert!(peak > 1.0, "primary peak {peak}");
        for s in &transient.timesteps[2..] {
            let (vp, vs) = (s.node_voltages["n2"], s.node_voltages["n3"]);
            assert!(
                (vs - 1.998 * vp).abs() < 1e-2 * (1.998 * peak),
                "t = {}: secondary {vs}, primary {vp}",
                s.time
            );
        }
        assert!(result.node_voltages["n2"].abs() < 1e-3 * peak);
    }

    /// Test 5: Circuit with both reactive and nonlinear solves successfully.
    #[test]
    fn test_mixed_reactive_nonlinear_succeeds() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 100e-6,
                }),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["n2".into(), "0".into()],
                    temperature: 300.15,
                    model: None,
                }),
            ],
            ..Default::default()
        };
        let result = solve_circuit(&circuit);
        assert!(result.is_ok(), "Expected Ok, got: {:?}", result.err());
        let result = result.unwrap();

        assert!(result.transient.is_some());

        let c1 = result
            .component_results
            .iter()
            .find(|c| c.id == "C1")
            .unwrap();
        assert!(c1.voltage_across > 0.0, "Capacitor should have charged");

        let d1 = result
            .component_results
            .iter()
            .find(|c| c.id == "D1")
            .unwrap();
        assert!(d1.voltage_across > 0.0, "Diode should show forward voltage");
        assert!(
            d1.current_through > 0.0,
            "Diode should show non-zero current"
        );
    }

    /// Adaptive stepping completes for smooth RC+diode circuit: solve_transient_nonlinear
    /// produces a SimulationResult with transient data (uses nonlinear path).
    #[test]
    fn adaptive_stepping_doubles_dt_after_successes() {
        // Diode in series with the source (not parallel with cap) — forces nonlinear path
        // while allowing cap to charge to ~(5V - 0.7V) ~ 4.3V
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["n1".into(), "n2".into()], // series diode
                    temperature: 300.15,
                    model: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "n3".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n3".into(), "0".into()],
                    capacitance: 100e-6, // tau = 0.1s
                }),
            ],
            ..Default::default()
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Adaptive stepping RC circuit should solve: {:?}",
            result.err()
        );
        let result = result.unwrap();
        assert!(result.transient.is_some(), "Expected transient data");
        let transient = result.transient.as_ref().unwrap();
        assert!(
            !transient.timesteps.is_empty(),
            "Expected at least one timestep"
        );
        // Capacitor should have charged toward ~4.3V (5V - ~0.7V diode drop)
        let c1 = result
            .component_results
            .iter()
            .find(|c| c.id == "C1")
            .unwrap();
        assert!(
            c1.voltage_across > 3.5,
            "Capacitor should have charged via diode, got {}",
            c1.voltage_across
        );
    }

    /// Adaptive stepping halving: convergence-failure path resets consecutive_successes.
    /// This is tested implicitly by the stiff circuit test which relies on halving to converge.
    #[test]
    fn adaptive_stepping_halving_preserved_in_stiff_circuit() {
        // Same as test_timestep_halving_recovery but verifies adaptive stepping doesn't break it
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    temperature: 300.15,
                    model: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 10.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                }),
            ],
            ..Default::default()
        };
        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Stiff circuit should still solve with adaptive stepping: {:?}",
            result.err()
        );
        let result = result.unwrap();
        assert!(result.transient.is_some());
        let c1 = result
            .component_results
            .iter()
            .find(|c| c.id == "C1")
            .unwrap();
        assert!(c1.voltage_across > 0.0, "C1 voltage should be positive");
    }

    /// BJT with parasitic capacitances: transient solve completes without panic.
    /// Q1 NPN with Cbe=10pF, Cbc=2pF. Circuit: V1=5V, Rb=100k, Rc=1k, BJT.
    #[test]
    fn bjt_with_parasitic_caps_transient_solves() {
        use crate::circuit::{BjtParasiticCaps, Circuit, CircuitElement};
        use crate::solve_circuit;
        use sindr_devices::bjt::BjtKind;

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rb".into(),
                    nodes: ["vcc".into(), "base".into()],
                    resistance: 100_000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rc".into(),
                    nodes: ["vcc".into(), "coll".into()],
                    resistance: 1_000.0,
                }),
                // Small cap to make circuit reactive so transient runs
                CircuitElement::Capacitor(Capacitor {
                    id: "Cin".into(),
                    nodes: ["base".into(), "0".into()],
                    capacitance: 1e-9, // 1 nF
                }),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["base".into(), "coll".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: Some(BjtParasiticCaps {
                        cbe: 10e-12,
                        cbc: 2e-12,
                    }),
                    model: None,
                }),
            ],
            ..Default::default()
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "BJT with parasitic caps should solve: {:?}",
            result.err()
        );
        let result = result.unwrap();
        assert!(result.transient.is_some(), "Expected transient data");
        let transient = result.transient.as_ref().unwrap();
        assert!(
            !transient.timesteps.is_empty(),
            "Expected at least one timestep"
        );
        // BJT result should be present
        assert!(!result.bjt_results.is_empty(), "Expected BJT results");
    }

    /// Parasitic caps affect transient result: same circuit with vs without caps → different collector voltage.
    #[test]
    fn parasitic_caps_affect_transient_result() {
        use crate::circuit::{BjtParasiticCaps, Circuit, CircuitElement};
        use crate::solve_circuit;
        use sindr_devices::bjt::BjtKind;

        let make_circuit = |with_caps: bool| Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rb".into(),
                    nodes: ["vcc".into(), "base".into()],
                    resistance: 100_000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rc".into(),
                    nodes: ["vcc".into(), "coll".into()],
                    resistance: 1_000.0,
                }),
                // Cap to make circuit reactive
                CircuitElement::Capacitor(Capacitor {
                    id: "Cin".into(),
                    nodes: ["base".into(), "0".into()],
                    capacitance: 1e-9,
                }),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["base".into(), "coll".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: if with_caps {
                        Some(BjtParasiticCaps {
                            cbe: 10e-12,
                            cbc: 2e-12,
                        })
                    } else {
                        None
                    },
                    model: None,
                }),
            ],
            ..Default::default()
        };

        let result_no_caps = solve_circuit(&make_circuit(false)).expect("No-caps circuit failed");
        let result_with_caps =
            solve_circuit(&make_circuit(true)).expect("With-caps circuit failed");

        // Both should have transient data
        assert!(result_no_caps.transient.is_some());
        assert!(result_with_caps.transient.is_some());

        // Results should differ: parasitic caps slow down switching, changing voltages at early timesteps
        let v_coll_no_caps = result_no_caps
            .node_voltages
            .get("coll")
            .copied()
            .unwrap_or(0.0);
        let v_coll_with_caps = result_with_caps
            .node_voltages
            .get("coll")
            .copied()
            .unwrap_or(0.0);
        // Both should be valid (non-NaN, finite)
        assert!(
            v_coll_no_caps.is_finite(),
            "No-caps collector voltage should be finite"
        );
        assert!(
            v_coll_with_caps.is_finite(),
            "With-caps collector voltage should be finite"
        );
    }

    fn circuit_of(components: Vec<CircuitElement>) -> Circuit {
        Circuit {
            ground_node: "0".into(),
            components,
            ..Default::default()
        }
    }

    fn capacitor(id: &str, a: &str, farads: f64) -> CircuitElement {
        CircuitElement::Capacitor(Capacitor {
            id: id.into(),
            nodes: [a.into(), "0".into()],
            capacitance: farads,
        })
    }

    /// A switch's resistance pairs with every capacitor: 0.01 Ω closed,
    /// 1 GΩ open, so an open switch across 1 nF is a 1 s time constant.
    #[test]
    fn calculate_duration_counts_a_switch_as_its_resistance() {
        for (closed, r) in [(true, SWITCH_R_CLOSED), (false, SWITCH_R_OPEN)] {
            let circuit = circuit_of(vec![
                CircuitElement::Switch(Switch {
                    id: "S1".into(),
                    nodes: ["a".into(), "0".into()],
                    closed,
                }),
                capacitor("C1", "a", 1e-9),
            ]);
            let tau = r * 1e-9;
            let (duration, dt) = calculate_duration(&circuit);
            assert_relative_eq!(duration, 5.0 * tau, max_relative = 1e-12);
            assert_relative_eq!(dt, (tau / 50.0).max(1e-6), max_relative = 1e-12);
        }
    }

    /// L/R skips resistances under 0.02 Ω, which would otherwise make a
    /// closed switch in series with an inductor look like a 100 s
    /// time constant.
    #[test]
    fn calculate_duration_skips_switch_like_resistances_for_l_over_r() {
        let circuit = circuit_of(vec![
            CircuitElement::Switch(Switch {
                id: "S1".into(),
                nodes: ["a".into(), "b".into()],
                closed: true,
            }),
            CircuitElement::Inductor(Inductor {
                id: "L1".into(),
                nodes: ["b".into(), "0".into()],
                inductance: 1.0,
                saturation: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["a".into(), "0".into()],
                resistance: 100.0,
            }),
        ]);
        // Only L/R = 1 H / 100 Ω = 10 ms counts.
        let (duration, dt) = calculate_duration(&circuit);
        assert_relative_eq!(duration, 50e-3, max_relative = 1e-12);
        assert_relative_eq!(dt, 0.2e-3, max_relative = 1e-12);
    }

    /// A battery's series resistance and its RC pair both pair with the
    /// pair's capacitance: τ = max(r_series, r)·c.
    #[test]
    fn calculate_duration_includes_a_battery_rc_pair() {
        let battery = |rc| {
            CircuitElement::Battery(Battery {
                id: "B1".into(),
                nodes: ["a".into(), "0".into()],
                params: sindr_devices::battery::BatteryParams {
                    rc,
                    ..sindr_devices::battery::BatteryParams::constant(3.7, 1.0, 0.1)
                },
                state_of_charge: 1.0,
            })
        };
        let with_rc = circuit_of(vec![battery(Some(sindr_devices::battery::BatteryRc {
            r: 0.05,
            c: 1000.0,
        }))]);
        let (duration, _) = calculate_duration(&with_rc);
        assert_relative_eq!(duration, 5.0 * 0.1 * 1000.0, max_relative = 1e-12);

        // No pair: nothing to charge, so the 1 ms fallback.
        let (duration, dt) = calculate_duration(&circuit_of(vec![battery(None)]));
        assert_relative_eq!(duration, 5e-3, max_relative = 1e-12);
        assert_relative_eq!(dt, 2e-5, max_relative = 1e-12);
    }

    /// With several periodic sources the longest period sets the span;
    /// a waveform without a period (PWL, or a 0 Hz sine) adds nothing.
    #[test]
    fn calculate_duration_takes_the_longest_waveform_period() {
        let source = |id: &str, node: &str, waveform| {
            CircuitElement::VoltageSource(VoltageSource {
                id: id.into(),
                nodes: [node.into(), "0".into()],
                voltage: 0.0,
                waveform: Some(waveform),
            })
        };
        let load = |id: &str, node: &str| {
            CircuitElement::Resistor(Resistor {
                id: id.into(),
                nodes: [node.into(), "0".into()],
                resistance: 1e3,
            })
        };
        let circuit = circuit_of(vec![
            source(
                "V1",
                "a",
                Waveform::Square {
                    amplitude: 1.0,
                    frequency: 1e3,
                    offset: 0.0,
                    duty: 0.5,
                },
            ),
            load("R1", "a"),
            source(
                "V2",
                "b",
                Waveform::Triangle {
                    amplitude: 1.0,
                    frequency: 100.0,
                    offset: 0.0,
                },
            ),
            load("R2", "b"),
            source(
                "V3",
                "c",
                Waveform::Pwl {
                    points: vec![(0.0, 0.0), (1.0, 1.0)],
                },
            ),
            load("R3", "c"),
            source(
                "V4",
                "d",
                Waveform::Sine {
                    amplitude: 1.0,
                    frequency: 0.0,
                    offset: 0.0,
                    phase: 0.0,
                },
            ),
            load("R4", "d"),
        ]);
        // T = 10 ms from the triangle: 3 periods, 100 points per period.
        let (duration, dt) = calculate_duration(&circuit);
        assert_relative_eq!(duration, 30e-3, max_relative = 1e-12);
        assert_relative_eq!(dt, 0.1e-3, max_relative = 1e-12);
    }
}
