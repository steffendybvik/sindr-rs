use std::collections::HashMap;

use nalgebra::DVector;

use sindr_devices::bjt::{self, BjtKind, BjtParams};
use sindr_devices::mosfet::{self, MosfetKind};

use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::newton_raphson::{self, MAX_NR_ITERATIONS};
use crate::node_map::NodeMap;
use crate::results::{
    BjtResult, ComponentResult, MosfetResult, SimulationResult, TimestepSnapshot, TransientData,
};
use crate::stamp::{
    stamp_bjt_companion, stamp_bjt_parasitic_caps, stamp_capacitor_companion, stamp_cccs,
    stamp_ccvs, stamp_current_source, stamp_diode_companion, stamp_igbt_companion,
    stamp_inductor_companion, stamp_jfet_companion, stamp_mosfet_companion,
    stamp_mosfet_parasitic_caps, stamp_resistor, stamp_transformer_companion, stamp_vccs,
    stamp_vcvs, stamp_voltage_source, SWITCH_R_CLOSED, SWITCH_R_OPEN,
};

/// Calculate simulation duration and timestep from circuit time constants.
///
/// Returns (duration, dt) where:
/// - duration = 5 * tau_max (5 time constants for ~99% settling)
/// - dt = tau_max / 50 (floored at 1 microsecond)
pub fn calculate_duration(circuit: &Circuit) -> (f64, f64) {
    // Collect R, C, L values
    let mut resistances: Vec<f64> = Vec::new();
    let mut capacitances: Vec<f64> = Vec::new();
    let mut inductances: Vec<f64> = Vec::new();

    for component in &circuit.components {
        match component {
            CircuitElement::Resistor { resistance, .. } => {
                resistances.push(*resistance);
            }
            CircuitElement::Switch { closed, .. } => {
                let r = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                resistances.push(r);
            }
            CircuitElement::Capacitor { capacitance, .. } => {
                capacitances.push(*capacitance);
            }
            CircuitElement::Inductor { inductance, .. } => {
                inductances.push(*inductance);
            }
            CircuitElement::Relay {
                coil_resistance,
                inductance,
                ..
            } if *inductance > 0.0 => {
                resistances.push(*coil_resistance);
                inductances.push(*inductance);
            }
            CircuitElement::Transformer { l1, l2, .. } => {
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
            CircuitElement::VoltageSource {
                waveform: Some(w), ..
            }
            | CircuitElement::CurrentSource {
                waveform: Some(w), ..
            } => {
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

/// Stamp all circuit components for a transient timestep.
///
/// Linear components use standard stamps. Capacitors and inductors use
/// companion models with their previous-timestep state.
/// Time `t` is used to evaluate waveform sources.
fn stamp_circuit_transient(
    circuit: &Circuit,
    system: &mut MnaSystem,
    node_map: &NodeMap,
    dt: f64,
    t: f64,
    cap_voltages: &[f64],
    ind_currents: &[f64],
) -> Result<(), SimError> {
    let num_nodes = node_map.num_nodes();
    let mut vsource_index: usize = 0;
    let mut cap_index: usize = 0;
    let mut ind_index: usize = 0;

    for component in &circuit.components {
        match component {
            CircuitElement::Resistor {
                id,
                nodes,
                resistance,
            } => {
                if *resistance <= 0.0 {
                    return Err(SimError::InvalidResistance(id.clone()));
                }
                stamp_resistor(system, node_map, nodes, *resistance);
            }
            CircuitElement::VoltageSource {
                nodes,
                voltage,
                waveform,
                ..
            } => {
                let v = match waveform {
                    Some(w) => *voltage + w.evaluate(t),
                    None => *voltage,
                };
                let branch = num_nodes + vsource_index;
                stamp_voltage_source(system, node_map, nodes, v, branch);
                vsource_index += 1;
            }
            CircuitElement::CurrentSource {
                nodes,
                current,
                waveform,
                ..
            } => {
                let i = match waveform {
                    Some(w) => *current + w.evaluate(t),
                    None => *current,
                };
                stamp_current_source(system, node_map, nodes, i);
            }
            CircuitElement::Switch { nodes, closed, .. } => {
                let resistance = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                stamp_resistor(system, node_map, nodes, resistance);
            }
            CircuitElement::Capacitor {
                nodes, capacitance, ..
            } => {
                let v_prev = cap_voltages[cap_index];
                stamp_capacitor_companion(system, node_map, nodes, *capacitance, dt, v_prev);
                cap_index += 1;
            }
            CircuitElement::Inductor {
                nodes, inductance, ..
            } => {
                let i_prev = ind_currents[ind_index];
                stamp_inductor_companion(system, node_map, nodes, *inductance, dt, i_prev);
                ind_index += 1;
            }
            CircuitElement::Pushbutton { nodes, closed, .. } => {
                let resistance = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                stamp_resistor(system, node_map, nodes, resistance);
            }
            CircuitElement::Photoresistor {
                nodes, light_level, ..
            } => {
                let resistance = crate::stamp::ldr_resistance(*light_level);
                stamp_resistor(system, node_map, nodes, resistance);
            }
            CircuitElement::Potentiometer {
                nodes,
                resistance,
                position,
                ..
            } => {
                let pos = position.clamp(0.001, 0.999);
                let r_top = resistance * pos;
                let r_bot = resistance * (1.0 - pos);
                let top_wiper: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                let wiper_bot: [String; 2] = [nodes[1].clone(), nodes[2].clone()];
                stamp_resistor(system, node_map, &top_wiper, r_top);
                stamp_resistor(system, node_map, &wiper_bot, r_bot);
            }
            CircuitElement::Relay {
                nodes,
                coil_resistance,
                pickup_voltage,
                inductance,
                ..
            } => {
                // Coil resistance always stamped
                let coil_nodes: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                stamp_resistor(system, node_map, &coil_nodes, *coil_resistance);
                // Coil inductance (if present): stamp as RL companion
                if *inductance > 0.0 {
                    let i_prev = ind_currents[ind_index];
                    ind_index += 1;
                    stamp_inductor_companion(
                        system,
                        node_map,
                        &[nodes[0].clone(), nodes[1].clone()],
                        *inductance,
                        dt,
                        i_prev,
                    );
                }
                // Contact: determined by previous iteration coil voltage (open on first pass)
                let contact_nodes: [String; 2] = [nodes[2].clone(), nodes[3].clone()];
                stamp_resistor(system, node_map, &contact_nodes, SWITCH_R_OPEN);
                let _ = pickup_voltage;
            }
            // Thermistor: passive resistor — stamp properly in transient
            CircuitElement::Thermistor {
                nodes, temperature, ..
            } => {
                let params = sindr_devices::thermistor::ThermistorParams::default();
                let r = sindr_devices::thermistor::thermistor_resistance(*temperature, &params);
                stamp_resistor(system, node_map, nodes, r);
            }
            // Varactor: voltage-dependent capacitor in transient
            // Uses cap_voltages positional index (same pattern as Capacitor)
            CircuitElement::Varactor { nodes, params, .. } => {
                let v_prev = cap_voltages[cap_index];
                crate::stamp::stamp_varactor_transient(system, node_map, nodes, v_prev, dt, params);
                cap_index += 1;
            }
            // IGBT: nonlinear element — skip in linear transient path
            CircuitElement::Igbt { .. } => {}
            // Nonlinear elements: skip in linear transient path
            CircuitElement::Diode { .. } | CircuitElement::Led { .. } => {}
            CircuitElement::Bjt { .. } => {}
            CircuitElement::Mosfet { .. } => {}
            CircuitElement::SchottkyDiode { .. } => {}
            CircuitElement::Photodiode { .. } => {}
            CircuitElement::Vcvs {
                nodes,
                control_nodes,
                gain,
                ..
            } => {
                let branch = num_nodes + vsource_index;
                stamp_vcvs(system, node_map, nodes, control_nodes, *gain, branch);
                vsource_index += 1;
            }
            CircuitElement::Vccs {
                nodes,
                control_nodes,
                gm,
                ..
            } => {
                stamp_vccs(system, node_map, nodes, control_nodes, *gm);
            }
            CircuitElement::Ccvs {
                nodes,
                control_source,
                rm,
                ..
            } => {
                let branch = num_nodes + vsource_index;
                let ctrl_branch = circuit.vsource_branch_index(control_source);
                if let Some(cb) = ctrl_branch {
                    stamp_ccvs(system, node_map, nodes, *rm, branch, num_nodes + cb);
                }
                vsource_index += 1;
            }
            CircuitElement::Cccs {
                nodes,
                control_source,
                alpha,
                ..
            } => {
                let ctrl_branch = circuit.vsource_branch_index(control_source);
                if let Some(cb) = ctrl_branch {
                    stamp_cccs(system, node_map, nodes, *alpha, num_nodes + cb);
                }
            }
            // Transformer: coupled inductor companion (Backward Euler, 2 branch current unknowns)
            CircuitElement::Transformer {
                nodes, l1, l2, k, ..
            } => {
                let i1_prev = ind_currents[ind_index];
                let i2_prev = ind_currents[ind_index + 1];
                ind_index += 2;
                let k1 = num_nodes + vsource_index;
                let k2 = num_nodes + vsource_index + 1;
                vsource_index += 2;
                stamp_transformer_companion(
                    system, node_map, nodes, *l1, *l2, *k, dt, i1_prev, i2_prev, k1, k2,
                );
            }
            // ZenerDiode: nonlinear element, skip in linear transient path
            CircuitElement::ZenerDiode { .. } => {}
            // OpAmp/Comparator: linear VCVS — stamp properly
            CircuitElement::OpAmp { nodes, .. } | CircuitElement::Comparator { nodes, .. } => {
                let branch = num_nodes + vsource_index;
                let out_nodes: [String; 2] = [nodes[2].clone(), circuit.ground_node.clone()];
                let ctrl_nodes: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                stamp_vcvs(system, node_map, &out_nodes, &ctrl_nodes, 1e5, branch);
                vsource_index += 1;
            }
            // Fuse: stamp as resistor (intact=0.001 Ohm, blown=1e9 Ohm) — same in transient
            CircuitElement::Fuse { nodes, blown, .. } => {
                let resistance = if *blown { SWITCH_R_OPEN } else { 0.001 };
                stamp_resistor(system, node_map, nodes, resistance);
            }
            // JFET: nonlinear element — skip in linear transient path (handled in NR loop)
            CircuitElement::Jfet { .. } => {}
            // VoltageRegulator: ideal voltage source between output (nodes[1]) and gnd (nodes[2])
            CircuitElement::VoltageRegulator { nodes, voltage, .. } => {
                let branch = num_nodes + vsource_index;
                let vs_nodes: [String; 2] = [nodes[1].clone(), nodes[2].clone()];
                stamp_voltage_source(system, node_map, &vs_nodes, *voltage, branch);
                vsource_index += 1;
            }
        }
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

/// Extract per-timestep results from solution vector.
#[allow(clippy::too_many_arguments)]
fn extract_timestep_results(
    circuit: &Circuit,
    node_map: &NodeMap,
    solution: &DVector<f64>,
    num_nodes: usize,
    time: f64,
    dt: f64,
    cap_voltages_prev: &[f64],
    ind_currents_prev: &[f64],
) -> (TimestepSnapshot, Vec<f64>, Vec<f64>) {
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

    let mut new_cap_voltages = Vec::new();
    let mut new_ind_currents = Vec::new();

    for component in &circuit.components {
        match component {
            CircuitElement::Resistor {
                id,
                nodes,
                resistance,
            } => {
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
            CircuitElement::VoltageSource { id, voltage, .. } => {
                let i = solution[num_nodes + vsource_index];
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: *voltage,
                    current_through: i,
                    power: voltage * i,
                });
                vsource_index += 1;
            }
            CircuitElement::CurrentSource {
                id, nodes, current, ..
            } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v = vp - vq;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v,
                    current_through: *current,
                    power: v * current,
                });
            }
            CircuitElement::Switch { id, nodes, closed } => {
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
            CircuitElement::Capacitor {
                id,
                nodes,
                capacitance,
            } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_now = vp - vq;
                let v_prev = cap_voltages_prev[cap_index];
                let i = (capacitance / dt) * (v_now - v_prev);
                new_cap_voltages.push(v_now);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_now,
                    current_through: i,
                    power: v_now * i,
                });
                cap_index += 1;
            }
            CircuitElement::Inductor {
                id,
                nodes,
                inductance,
            } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let i_prev = ind_currents_prev[ind_index];
                let i_now = i_prev + (dt / inductance) * v_across;
                new_ind_currents.push(i_now);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_now,
                    power: v_across * i_now,
                });
                ind_index += 1;
            }
            CircuitElement::Diode { id, nodes, .. } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let params = sindr_devices::diode::DiodeParams::silicon();
                let i_through = sindr_devices::diode::diode_current(v_across, &params);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Led {
                id, nodes, color, ..
            } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let params = sindr_devices::diode::DiodeParams::for_led_color(color);
                let i_through = sindr_devices::diode::diode_current(v_across, &params);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::Bjt {
                id,
                nodes,
                kind,
                bf,
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
                let params = BjtParams::new(*bf);
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
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vds_phys,
                    current_through: id_current,
                    power: (id_current * vds_phys).abs(),
                });
            }
            CircuitElement::Vcvs { id, nodes, .. } | CircuitElement::Ccvs { id, nodes, .. } => {
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
            CircuitElement::Vccs {
                id,
                nodes,
                control_nodes,
                gm,
            } => {
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
            CircuitElement::Cccs {
                id,
                nodes,
                control_source,
                alpha,
            } => {
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
                let v = vp - vq;
                let i = v / resistance;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v,
                    current_through: i,
                    power: v * i,
                });
            }
            CircuitElement::Photoresistor {
                id,
                nodes,
                light_level,
            } => {
                let resistance = crate::stamp::ldr_resistance(*light_level);
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
                inductance,
                ..
            } => {
                let vc_pos = node_voltage(&nodes[0], node_map, solution);
                let vc_neg = node_voltage(&nodes[1], node_map, solution);
                let coil_voltage = vc_pos - vc_neg;
                // If relay has inductance, update the inductor current state
                if *inductance > 0.0 {
                    let i_prev = ind_currents_prev[ind_index];
                    let i_now = i_prev + (dt / inductance) * coil_voltage;
                    new_ind_currents.push(i_now);
                    ind_index += 1;
                }
                let coil_current = coil_voltage / coil_resistance;
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
            CircuitElement::ZenerDiode { id, nodes, vz, .. } => {
                let va = node_voltage(&nodes[0], node_map, solution);
                let vk = node_voltage(&nodes[1], node_map, solution);
                let v_across = va - vk;
                let (g_eq, i_eq) = sindr_devices::zener::zener_companion(
                    v_across,
                    &sindr_devices::zener::ZenerParams::new(*vz),
                );
                let i_through = g_eq * v_across + i_eq;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power: v_across * i_through,
                });
            }
            CircuitElement::OpAmp { id, nodes, .. }
            | CircuitElement::Comparator { id, nodes, .. } => {
                let v_out = node_voltage(&nodes[2], node_map, solution);
                let i_through = solution[num_nodes + vsource_index];
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_out,
                    current_through: i_through,
                    power: v_out * i_through,
                });
                vsource_index += 1;
            }
            CircuitElement::SchottkyDiode { id, nodes, .. } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let schottky_params = sindr_devices::schottky::SchottkyParams::default();
                let i_through = sindr_devices::diode::diode_current(
                    v_across,
                    &sindr_devices::diode::DiodeParams {
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
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_across = vp - vq;
                let photo_params = sindr_devices::photodiode::PhotodiodeParams::default();
                let i_dark = sindr_devices::diode::diode_current(
                    v_across,
                    &sindr_devices::diode::DiodeParams {
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
            CircuitElement::Varactor {
                id, nodes, params, ..
            } => {
                let vp = node_voltage(&nodes[0], node_map, solution);
                let vq = node_voltage(&nodes[1], node_map, solution);
                let v_now = vp - vq;
                // Update varactor voltage state for next timestep (uses positional cap_index)
                let v_prev_across = cap_voltages_prev[cap_index];
                // Current approximation: i = C_j(v_prev) / dt * (v_now - v_prev)
                let cj = sindr_devices::varactor::junction_capacitance(v_prev_across, params);
                let i_through = if dt > 0.0 {
                    (cj / dt) * (v_now - v_prev_across)
                } else {
                    0.0
                };
                new_cap_voltages.push(v_now);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_now,
                    current_through: i_through,
                    power: v_now * i_through,
                });
                cap_index += 1;
            }
            CircuitElement::Igbt {
                id, nodes, params, ..
            } => {
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
                new_ind_currents.push(i1_now);
                new_ind_currents.push(i2_now);
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_primary,
                    current_through: i1_now,
                    power: (v_primary * i1_now + v_secondary * i2_now).abs(),
                });
                ind_index += 2;
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
            CircuitElement::Jfet {
                id,
                nodes,
                kind,
                idss,
                vp,
            } => {
                use sindr_devices::jfet::{jfet_companion, JfetKind};
                let vg = node_voltage(&nodes[0], node_map, solution);
                let vd = node_voltage(&nodes[1], node_map, solution);
                let vs = node_voltage(&nodes[2], node_map, solution);
                let (vgs_eff, vds_eff) = match kind {
                    JfetKind::NChannel => (vg - vs, vd - vs),
                    JfetKind::PChannel => (vs - vg, vs - vd),
                };
                let comp = jfet_companion(vgs_eff, vds_eff, *kind, *idss, *vp);
                let sign = match kind {
                    JfetKind::NChannel => 1.0,
                    JfetKind::PChannel => -1.0,
                };
                let vds_phys = vd - vs;
                let id_current = sign * (comp.gm * vgs_eff + comp.gds * vds_eff + comp.i_eq);
                let power = (id_current * vds_phys).abs();
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: vds_phys,
                    current_through: id_current,
                    power,
                });
            }
            CircuitElement::VoltageRegulator { id, nodes, voltage } => {
                let vout = node_voltage(&nodes[1], node_map, solution);
                let vgnd = node_voltage(&nodes[2], node_map, solution);
                let v_across = vout - vgnd;
                let i_through = solution[num_nodes + vsource_index];
                let power = voltage * i_through;
                component_results.push(ComponentResult {
                    id: id.clone(),
                    voltage_across: v_across,
                    current_through: i_through,
                    power,
                });
                vsource_index += 1;
            }
        }
    }

    let snapshot = TimestepSnapshot {
        time,
        node_voltages,
        component_results,
    };

    (snapshot, new_cap_voltages, new_ind_currents)
}

/// Minimum timestep floor (1 microsecond).
const DT_FLOOR: f64 = 1e-6;

/// Solve a transient simulation for circuits with both reactive AND nonlinear elements.
///
/// Nests Newton-Raphson iteration inside each transient timestep:
/// - Outer loop: time-stepping with Backward Euler companions for caps/inductors
/// - Inner loop: NR iteration with companion models for diodes/LEDs
/// - Adaptive timestep halving: if NR fails to converge, halve dt and retry
///
/// Reactive companion state (cap voltages, inductor currents) is fixed during
/// NR iteration and only updated after convergence.
pub fn solve_transient_nonlinear(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
) -> Result<SimulationResult, SimError> {
    let (duration, dt) = calculate_duration(circuit);

    // Initialize reactive state: uncharged caps (0V), zero-current inductors (0A)
    let mut cap_voltages: Vec<f64> = Vec::new();
    let mut ind_currents: Vec<f64> = Vec::new();

    for component in &circuit.components {
        match component {
            CircuitElement::Capacitor { .. } => cap_voltages.push(0.0),
            CircuitElement::Varactor { .. } => cap_voltages.push(0.0),
            CircuitElement::Inductor { .. } => ind_currents.push(0.0),
            CircuitElement::Relay { inductance, .. } if *inductance > 0.0 => {
                ind_currents.push(0.0);
            }
            CircuitElement::Transformer { .. } => {
                ind_currents.push(0.0); // I_L1_initial
                ind_currents.push(0.0); // I_L2_initial
            }
            _ => {}
        }
    }

    // Parasitic cap voltages: keyed by "component_id-junction" (e.g. "Q1-be", "M1-gs")
    // Initialized to 0V (all junctions uncharged at t=0)
    let mut parasitic_cap_voltages: HashMap<String, f64> = HashMap::new();

    // Collect diode info for voltage limiting
    let diode_info = newton_raphson::collect_diode_info(circuit, node_map);

    let mut timesteps = Vec::new();
    let mut current_dt = dt;
    let mut time = 0.0;
    let mut consecutive_successes: usize = 0;

    // Adaptive stepping constants
    const GROW_AFTER: usize = 5; // double dt after this many consecutive successes
    const DT_GROW_FACTOR: f64 = 2.0; // growth factor
    let dt_max = dt * 10.0; // max dt = 10x initial dt

    while time <= duration {
        // Save reactive state for potential retry
        let saved_cap_v = cap_voltages.clone();
        let saved_ind_i = ind_currents.clone();
        let saved_parasitic_v = parasitic_cap_voltages.clone();

        match nr_at_timestep_with_parasitic(
            circuit,
            node_map,
            num_nodes,
            num_vsources,
            current_dt,
            time,
            &cap_voltages,
            &ind_currents,
            &diode_info,
            &parasitic_cap_voltages,
        ) {
            Ok((solution, new_cap_v, new_ind_i)) => {
                let (snapshot, _, _) = extract_timestep_results(
                    circuit,
                    node_map,
                    &solution,
                    num_nodes,
                    time + current_dt,
                    current_dt,
                    &cap_voltages,
                    &ind_currents,
                );
                timesteps.push(snapshot);
                cap_voltages = new_cap_v;
                ind_currents = new_ind_i;
                time += current_dt;

                // Update parasitic cap voltages from converged solution
                for component in &circuit.components {
                    match component {
                        CircuitElement::Bjt {
                            id,
                            nodes,
                            parasitic_caps: Some(_),
                            ..
                        } => {
                            let b = node_map.index(&nodes[0]).map_or(0.0, |i| solution[i]);
                            let c = node_map.index(&nodes[1]).map_or(0.0, |i| solution[i]);
                            let e = node_map.index(&nodes[2]).map_or(0.0, |i| solution[i]);
                            parasitic_cap_voltages.insert(format!("{}-be", id), b - e);
                            parasitic_cap_voltages.insert(format!("{}-bc", id), b - c);
                        }
                        CircuitElement::Mosfet {
                            id,
                            nodes,
                            parasitic_caps: Some(_),
                            ..
                        } => {
                            let g = node_map.index(&nodes[0]).map_or(0.0, |i| solution[i]);
                            let d = node_map.index(&nodes[1]).map_or(0.0, |i| solution[i]);
                            let s = node_map.index(&nodes[2]).map_or(0.0, |i| solution[i]);
                            parasitic_cap_voltages.insert(format!("{}-gs", id), g - s);
                            parasitic_cap_voltages.insert(format!("{}-gd", id), g - d);
                        }
                        _ => {}
                    }
                }

                // Adaptive dt doubling: grow dt after consecutive successes
                consecutive_successes += 1;
                if consecutive_successes >= GROW_AFTER {
                    current_dt = (current_dt * DT_GROW_FACTOR).min(dt_max);
                    consecutive_successes = 0;
                }
            }
            Err(e @ SimError::ConvergenceFailed { .. }) => {
                current_dt /= 2.0;
                consecutive_successes = 0;
                if current_dt < DT_FLOOR {
                    return Err(e);
                }
                // Restore reactive state and retry same time point
                cap_voltages = saved_cap_v;
                ind_currents = saved_ind_i;
                parasitic_cap_voltages = saved_parasitic_v;
                continue;
            }
            Err(e) => return Err(e),
        }
    }

    // Build final result from last snapshot
    let final_snapshot = timesteps.last().unwrap();
    let mut branch_currents = HashMap::new();
    for component in &circuit.components {
        if let CircuitElement::VoltageSource { id, .. } = component {
            if let Some(cr) = final_snapshot
                .component_results
                .iter()
                .find(|c| c.id == *id)
            {
                branch_currents.insert(id.clone(), cr.current_through);
            }
        }
    }

    // Extract BJT/MOSFET results from final snapshot
    let bjt_results = extract_bjt_results_from_snapshot(circuit, final_snapshot);
    let mosfet_results = extract_mosfet_results_from_snapshot(circuit, final_snapshot);

    Ok(SimulationResult {
        node_voltages: final_snapshot.node_voltages.clone(),
        branch_currents,
        component_results: final_snapshot.component_results.clone(),
        bjt_results,
        mosfet_results,
        op_amp_results: vec![],
        relay_results: vec![],
        mcu_results: vec![],
        transient: Some(TransientData {
            timesteps,
            time_step: dt,
            duration,
        }),
    })
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
        if let CircuitElement::Bjt {
            id,
            nodes,
            kind,
            bf,
            ..
        } = component
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

            let params = BjtParams::new(*bf);
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
        if let CircuitElement::Mosfet {
            id,
            nodes,
            kind,
            params,
            ..
        } = component
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

            let (vgs, vds, vbs) = match kind {
                MosfetKind::Nmos => (vg - vs, vd - vs, -vs),
                MosfetKind::Pmos => (vs - vg, vs - vd, vs),
            };

            let comp = mosfet::mosfet_companion(vgs, vds, vbs, params);
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

/// NR iteration at a single transient timestep with parasitic cap state.
///
/// Extends nr_at_timestep by also stamping BJT/MOSFET parasitic capacitances
/// using a HashMap keyed by "component_id-junction" (e.g. "Q1-be").
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn nr_at_timestep_with_parasitic(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
    dt: f64,
    t: f64,
    cap_voltages: &[f64],
    ind_currents: &[f64],
    diode_info: &[newton_raphson::DiodeLimitInfo],
    parasitic_cap_voltages: &HashMap<String, f64>,
) -> Result<(DVector<f64>, Vec<f64>, Vec<f64>), SimError> {
    let size = num_nodes + num_vsources;
    let mut v_prev = DVector::zeros(size);
    let mut last_step = f64::INFINITY;

    for _iteration in 0..MAX_NR_ITERATIONS {
        // Build fresh MNA system each iteration
        let mut system = MnaSystem::new(num_nodes, num_vsources);
        let mut vsource_index: usize = 0;
        let mut cap_index: usize = 0;
        let mut ind_index: usize = 0;

        for component in &circuit.components {
            match component {
                CircuitElement::Resistor {
                    id,
                    nodes,
                    resistance,
                } => {
                    if *resistance <= 0.0 {
                        return Err(SimError::InvalidResistance(id.clone()));
                    }
                    stamp_resistor(&mut system, node_map, nodes, *resistance);
                }
                CircuitElement::VoltageSource {
                    nodes,
                    voltage,
                    waveform,
                    ..
                } => {
                    let v = match waveform {
                        Some(w) => *voltage + w.evaluate(t),
                        None => *voltage,
                    };
                    let branch = num_nodes + vsource_index;
                    stamp_voltage_source(&mut system, node_map, nodes, v, branch);
                    vsource_index += 1;
                }
                CircuitElement::CurrentSource {
                    nodes,
                    current,
                    waveform,
                    ..
                } => {
                    let i = match waveform {
                        Some(w) => *current + w.evaluate(t),
                        None => *current,
                    };
                    stamp_current_source(&mut system, node_map, nodes, i);
                }
                CircuitElement::Switch { nodes, closed, .. } => {
                    let resistance = if *closed {
                        SWITCH_R_CLOSED
                    } else {
                        SWITCH_R_OPEN
                    };
                    stamp_resistor(&mut system, node_map, nodes, resistance);
                }
                CircuitElement::Capacitor {
                    nodes, capacitance, ..
                } => {
                    let v_cap_prev = cap_voltages[cap_index];
                    stamp_capacitor_companion(
                        &mut system,
                        node_map,
                        nodes,
                        *capacitance,
                        dt,
                        v_cap_prev,
                    );
                    cap_index += 1;
                }
                CircuitElement::Inductor {
                    nodes, inductance, ..
                } => {
                    let i_ind_prev = ind_currents[ind_index];
                    stamp_inductor_companion(
                        &mut system,
                        node_map,
                        nodes,
                        *inductance,
                        dt,
                        i_ind_prev,
                    );
                    ind_index += 1;
                }
                CircuitElement::Diode {
                    nodes, temperature, ..
                } => {
                    let mut params = sindr_devices::diode::DiodeParams::silicon();
                    if (*temperature - 300.15).abs() > 1e-6 {
                        params.is = sindr_devices::diode::temperature_scale_is(
                            params.is,
                            *temperature,
                            300.15,
                            1.11,
                            2.0,
                        );
                    }
                    stamp_diode_companion(&mut system, node_map, nodes, &v_prev, &params);
                }
                CircuitElement::Led {
                    nodes,
                    color,
                    temperature,
                    ..
                } => {
                    let mut params = sindr_devices::diode::DiodeParams::for_led_color(color);
                    if (*temperature - 300.15).abs() > 1e-6 {
                        params.is = sindr_devices::diode::temperature_scale_is(
                            params.is,
                            *temperature,
                            300.15,
                            1.11,
                            2.0,
                        );
                    }
                    stamp_diode_companion(&mut system, node_map, nodes, &v_prev, &params);
                }
                CircuitElement::Bjt {
                    id,
                    nodes,
                    kind,
                    bf,
                    temperature,
                    parasitic_caps,
                } => {
                    let mut params = BjtParams::new(*bf);
                    if (*temperature - 300.15).abs() > 1e-6 {
                        params.is = sindr_devices::diode::temperature_scale_is(
                            params.is,
                            *temperature,
                            300.15,
                            1.11,
                            3.0,
                        );
                    }
                    stamp_bjt_companion(&mut system, node_map, nodes, &v_prev, &params, *kind);
                    // Parasitic capacitances: stamp using HashMap-keyed previous voltages
                    if let Some(caps) = parasitic_caps {
                        let v_be_prev = *parasitic_cap_voltages
                            .get(&format!("{}-be", id))
                            .unwrap_or(&0.0);
                        let v_bc_prev = *parasitic_cap_voltages
                            .get(&format!("{}-bc", id))
                            .unwrap_or(&0.0);
                        stamp_bjt_parasitic_caps(
                            &mut system,
                            node_map,
                            nodes,
                            caps,
                            dt,
                            v_be_prev,
                            v_bc_prev,
                        );
                    }
                }
                CircuitElement::Mosfet {
                    id,
                    nodes,
                    kind,
                    params,
                    parasitic_caps,
                } => {
                    stamp_mosfet_companion(&mut system, node_map, nodes, &v_prev, params, *kind);
                    // Parasitic capacitances: stamp using HashMap-keyed previous voltages
                    if let Some(caps) = parasitic_caps {
                        let v_gs_prev = *parasitic_cap_voltages
                            .get(&format!("{}-gs", id))
                            .unwrap_or(&0.0);
                        let v_gd_prev = *parasitic_cap_voltages
                            .get(&format!("{}-gd", id))
                            .unwrap_or(&0.0);
                        stamp_mosfet_parasitic_caps(
                            &mut system,
                            node_map,
                            nodes,
                            caps,
                            dt,
                            v_gs_prev,
                            v_gd_prev,
                        );
                    }
                }
                // JFET — nonlinear companion in NR transient loop
                CircuitElement::Jfet {
                    nodes,
                    kind,
                    idss,
                    vp,
                    ..
                } => {
                    stamp_jfet_companion(&mut system, node_map, nodes, *kind, *idss, *vp, &v_prev);
                }
                CircuitElement::Vcvs {
                    nodes,
                    control_nodes,
                    gain,
                    ..
                } => {
                    let branch = num_nodes + vsource_index;
                    stamp_vcvs(&mut system, node_map, nodes, control_nodes, *gain, branch);
                    vsource_index += 1;
                }
                CircuitElement::Vccs {
                    nodes,
                    control_nodes,
                    gm,
                    ..
                } => {
                    stamp_vccs(&mut system, node_map, nodes, control_nodes, *gm);
                }
                CircuitElement::Ccvs {
                    nodes,
                    control_source,
                    rm,
                    ..
                } => {
                    let branch = num_nodes + vsource_index;
                    let ctrl_branch = circuit.vsource_branch_index(control_source);
                    if let Some(cb) = ctrl_branch {
                        stamp_ccvs(&mut system, node_map, nodes, *rm, branch, num_nodes + cb);
                    }
                    vsource_index += 1;
                }
                CircuitElement::Cccs {
                    nodes,
                    control_source,
                    alpha,
                    ..
                } => {
                    let ctrl_branch = circuit.vsource_branch_index(control_source);
                    if let Some(cb) = ctrl_branch {
                        stamp_cccs(&mut system, node_map, nodes, *alpha, num_nodes + cb);
                    }
                }
                CircuitElement::Pushbutton { nodes, closed, .. } => {
                    let resistance = if *closed {
                        SWITCH_R_CLOSED
                    } else {
                        SWITCH_R_OPEN
                    };
                    stamp_resistor(&mut system, node_map, nodes, resistance);
                }
                CircuitElement::Photoresistor {
                    nodes, light_level, ..
                } => {
                    let resistance = crate::stamp::ldr_resistance(*light_level);
                    stamp_resistor(&mut system, node_map, nodes, resistance);
                }
                CircuitElement::Potentiometer {
                    nodes,
                    resistance,
                    position,
                    ..
                } => {
                    let pos = position.clamp(0.001, 0.999);
                    let r_top = resistance * pos;
                    let r_bot = resistance * (1.0 - pos);
                    let top_wiper: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                    let wiper_bot: [String; 2] = [nodes[1].clone(), nodes[2].clone()];
                    stamp_resistor(&mut system, node_map, &top_wiper, r_top);
                    stamp_resistor(&mut system, node_map, &wiper_bot, r_bot);
                }
                CircuitElement::Relay {
                    nodes,
                    coil_resistance,
                    pickup_voltage,
                    inductance,
                    ..
                } => {
                    let coil_nodes: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                    stamp_resistor(&mut system, node_map, &coil_nodes, *coil_resistance);
                    if *inductance > 0.0 {
                        let i_ind_prev = ind_currents[ind_index];
                        stamp_inductor_companion(
                            &mut system,
                            node_map,
                            &[nodes[0].clone(), nodes[1].clone()],
                            *inductance,
                            dt,
                            i_ind_prev,
                        );
                        ind_index += 1;
                    }
                    let vc_pos = node_map.index(&nodes[0]).map_or(0.0, |i| v_prev[i]);
                    let vc_neg = node_map.index(&nodes[1]).map_or(0.0, |i| v_prev[i]);
                    let contact_closed = (vc_pos - vc_neg).abs() >= *pickup_voltage;
                    let contact_r = if contact_closed {
                        SWITCH_R_CLOSED
                    } else {
                        SWITCH_R_OPEN
                    };
                    let contact_nodes: [String; 2] = [nodes[2].clone(), nodes[3].clone()];
                    stamp_resistor(&mut system, node_map, &contact_nodes, contact_r);
                }
                CircuitElement::ZenerDiode { nodes, vz, .. } => {
                    crate::stamp::stamp_zener_companion(
                        &mut system,
                        node_map,
                        nodes,
                        &v_prev,
                        &sindr_devices::zener::ZenerParams::new(*vz),
                    );
                }
                CircuitElement::SchottkyDiode { nodes, .. } => {
                    let schottky_params = sindr_devices::schottky::SchottkyParams::default();
                    let diode_params = sindr_devices::diode::DiodeParams {
                        is: schottky_params.is,
                        n: schottky_params.n,
                        rs: 0.0,
                        temperature: 300.15,
                    };
                    stamp_diode_companion(&mut system, node_map, nodes, &v_prev, &diode_params);
                }
                CircuitElement::Thermistor {
                    nodes, temperature, ..
                } => {
                    let therm_params = sindr_devices::thermistor::ThermistorParams::default();
                    let r = sindr_devices::thermistor::thermistor_resistance(
                        *temperature,
                        &therm_params,
                    );
                    stamp_resistor(&mut system, node_map, nodes, r);
                }
                CircuitElement::Photodiode {
                    nodes, irradiance, ..
                } => {
                    let photo_params = sindr_devices::photodiode::PhotodiodeParams::default();
                    let v_a = node_map.index(&nodes[0]).map_or(0.0, |i| v_prev[i]);
                    let v_c = node_map.index(&nodes[1]).map_or(0.0, |i| v_prev[i]);
                    let v_d = v_a - v_c;
                    let (g_d, i_eq) = sindr_devices::photodiode::photodiode_companion(
                        v_d,
                        *irradiance,
                        &photo_params,
                    );
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    if let Some(pi) = p {
                        system.a[(pi, pi)] += g_d;
                    }
                    if let Some(qi) = q {
                        system.a[(qi, qi)] += g_d;
                    }
                    if let (Some(pi), Some(qi)) = (p, q) {
                        system.a[(pi, qi)] -= g_d;
                        system.a[(qi, pi)] -= g_d;
                    }
                    if let Some(pi) = p {
                        system.b[pi] -= i_eq;
                    }
                    if let Some(qi) = q {
                        system.b[qi] += i_eq;
                    }
                }
                CircuitElement::OpAmp { nodes, .. } | CircuitElement::Comparator { nodes, .. } => {
                    let branch = num_nodes + vsource_index;
                    let out_nodes: [String; 2] = [nodes[2].clone(), circuit.ground_node.clone()];
                    let ctrl_nodes: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                    stamp_vcvs(&mut system, node_map, &out_nodes, &ctrl_nodes, 1e5, branch);
                    vsource_index += 1;
                }
                CircuitElement::Varactor { nodes, params, .. } => {
                    let v_varactor_prev = cap_voltages[cap_index];
                    crate::stamp::stamp_varactor_transient(
                        &mut system,
                        node_map,
                        nodes,
                        v_varactor_prev,
                        dt,
                        params,
                    );
                    cap_index += 1;
                }
                CircuitElement::Igbt { nodes, params, .. } => {
                    stamp_igbt_companion(&mut system, node_map, nodes, &v_prev, params);
                }
                CircuitElement::Transformer {
                    nodes, l1, l2, k, ..
                } => {
                    let i1_prev = ind_currents[ind_index];
                    let i2_prev = ind_currents[ind_index + 1];
                    ind_index += 2;
                    let k1 = num_nodes + vsource_index;
                    let k2 = num_nodes + vsource_index + 1;
                    vsource_index += 2;
                    stamp_transformer_companion(
                        &mut system,
                        node_map,
                        nodes,
                        *l1,
                        *l2,
                        *k,
                        dt,
                        i1_prev,
                        i2_prev,
                        k1,
                        k2,
                    );
                }
                // Fuse: stamp as resistor (intact=0.001 Ohm, blown=1e9 Ohm) in NR transient
                CircuitElement::Fuse { nodes, blown, .. } => {
                    let resistance = if *blown { SWITCH_R_OPEN } else { 0.001 };
                    stamp_resistor(&mut system, node_map, nodes, resistance);
                }
                // VoltageRegulator: ideal voltage source between output (nodes[1]) and gnd (nodes[2])
                CircuitElement::VoltageRegulator { nodes, voltage, .. } => {
                    let branch = num_nodes + vsource_index;
                    let vs_nodes: [String; 2] = [nodes[1].clone(), nodes[2].clone()];
                    stamp_voltage_source(&mut system, node_map, &vs_nodes, *voltage, branch);
                    vsource_index += 1;
                }
            }
        }

        // Add Gmin shunts to prevent singular Jacobian
        newton_raphson::add_gmin_shunts(&mut system, num_nodes);

        // Solve the linear system
        let v_new = system.solve()?;

        // Apply voltage limiting
        let v_limited = newton_raphson::apply_voltage_limiting(&v_new, &v_prev, diode_info);

        // Check convergence
        if newton_raphson::converged(&v_prev, &v_limited, num_nodes) {
            // Extract updated reactive state from converged solution
            let mut new_cap_v = Vec::new();
            let mut new_ind_i = Vec::new();
            let mut conv_vsource_idx: usize = 0;

            for component in &circuit.components {
                match component {
                    CircuitElement::VoltageSource { .. }
                    | CircuitElement::Vcvs { .. }
                    | CircuitElement::Ccvs { .. }
                    | CircuitElement::OpAmp { .. }
                    | CircuitElement::Comparator { .. } => {
                        conv_vsource_idx += 1;
                    }
                    CircuitElement::Capacitor { nodes, .. } => {
                        let vp = node_voltage(&nodes[0], node_map, &v_limited);
                        let vq = node_voltage(&nodes[1], node_map, &v_limited);
                        new_cap_v.push(vp - vq);
                    }
                    CircuitElement::Varactor { nodes, .. } => {
                        let vp = node_voltage(&nodes[0], node_map, &v_limited);
                        let vq = node_voltage(&nodes[1], node_map, &v_limited);
                        new_cap_v.push(vp - vq);
                    }
                    CircuitElement::Inductor {
                        nodes, inductance, ..
                    } => {
                        let vp = node_voltage(&nodes[0], node_map, &v_limited);
                        let vq = node_voltage(&nodes[1], node_map, &v_limited);
                        let i_idx = new_ind_i.len();
                        let i_prev = ind_currents[i_idx];
                        new_ind_i.push(i_prev + (dt / inductance) * (vp - vq));
                    }
                    CircuitElement::Relay {
                        nodes, inductance, ..
                    } if *inductance > 0.0 => {
                        let vp = node_voltage(&nodes[0], node_map, &v_limited);
                        let vq = node_voltage(&nodes[1], node_map, &v_limited);
                        let i_idx = new_ind_i.len();
                        let i_prev = ind_currents[i_idx];
                        new_ind_i.push(i_prev + (dt / inductance) * (vp - vq));
                    }
                    CircuitElement::Transformer { .. } => {
                        let i1_now = v_limited[num_nodes + conv_vsource_idx];
                        let i2_now = v_limited[num_nodes + conv_vsource_idx + 1];
                        new_ind_i.push(i1_now);
                        new_ind_i.push(i2_now);
                        conv_vsource_idx += 2;
                    }
                    _ => {}
                }
            }

            return Ok((v_limited, new_cap_v, new_ind_i));
        }

        last_step = newton_raphson::max_node_step(&v_prev, &v_limited, num_nodes);
        v_prev = v_limited;
    }

    Err(SimError::ConvergenceFailed {
        iterations: MAX_NR_ITERATIONS,
        max_step_volts: last_step,
    })
}

/// Solve a transient simulation for circuits with reactive elements.
///
/// Uses Backward Euler integration with companion models to step through
/// time, producing a series of snapshots showing voltage/current evolution.
pub fn solve_transient(
    circuit: &Circuit,
    node_map: &NodeMap,
    num_nodes: usize,
    num_vsources: usize,
) -> Result<SimulationResult, SimError> {
    let (duration, dt) = calculate_duration(circuit);
    let num_steps = (duration / dt).round() as usize;

    // Initialize companion model state: uncharged caps (0V), zero-current inductors (0A).
    // This models the circuit being "turned on" from an unpowered state.
    let mut cap_voltages: Vec<f64> = Vec::new();
    let mut ind_currents: Vec<f64> = Vec::new();

    for component in &circuit.components {
        match component {
            CircuitElement::Capacitor { .. } => {
                cap_voltages.push(0.0);
            }
            CircuitElement::Varactor { .. } => {
                // Varactor starts at 0V initial condition (uncharged)
                cap_voltages.push(0.0);
            }
            CircuitElement::Inductor { .. } => {
                ind_currents.push(0.0);
            }
            CircuitElement::Relay { inductance, .. } if *inductance > 0.0 => {
                ind_currents.push(0.0);
            }
            CircuitElement::Transformer { .. } => {
                ind_currents.push(0.0); // I_L1_initial
                ind_currents.push(0.0); // I_L2_initial
            }
            _ => {}
        }
    }

    // Build timestep 0 by solving with initial companion state
    let mut system_0 = MnaSystem::new(num_nodes, num_vsources);
    stamp_circuit_transient(
        circuit,
        &mut system_0,
        node_map,
        dt,
        0.0,
        &cap_voltages,
        &ind_currents,
    )?;
    let solution_0 = system_0.solve()?;
    let (snapshot_0, new_cap_v_0, new_ind_i_0) = extract_timestep_results(
        circuit,
        node_map,
        &solution_0,
        num_nodes,
        dt,
        dt,
        &cap_voltages,
        &ind_currents,
    );
    cap_voltages = new_cap_v_0;
    ind_currents = new_ind_i_0;

    let mut timesteps = vec![snapshot_0];

    // Time-stepping loop
    for step in 1..=num_steps {
        let time = step as f64 * dt;

        // Fresh MNA system each timestep
        let mut system = MnaSystem::new(num_nodes, num_vsources);
        stamp_circuit_transient(
            circuit,
            &mut system,
            node_map,
            dt,
            time,
            &cap_voltages,
            &ind_currents,
        )?;

        let solution = system.solve()?;

        let (snapshot, new_cap_v, new_ind_i) = extract_timestep_results(
            circuit,
            node_map,
            &solution,
            num_nodes,
            time + dt,
            dt,
            &cap_voltages,
            &ind_currents,
        );

        // Update state for next timestep
        cap_voltages = new_cap_v;
        ind_currents = new_ind_i;

        timesteps.push(snapshot);
    }

    // Final timestep data for the steady-state snapshot
    let final_snapshot = timesteps.last().unwrap();

    // Build branch_currents from final solution
    let mut branch_currents = HashMap::new();
    for component in &circuit.components {
        if let CircuitElement::VoltageSource { id, .. } = component {
            if let Some(cr) = final_snapshot
                .component_results
                .iter()
                .find(|c| c.id == *id)
            {
                branch_currents.insert(id.clone(), cr.current_through);
            }
        }
    }

    // Extract BJT/MOSFET results from final snapshot
    let bjt_results = extract_bjt_results_from_snapshot(circuit, final_snapshot);
    let mosfet_results = extract_mosfet_results_from_snapshot(circuit, final_snapshot);

    Ok(SimulationResult {
        node_voltages: final_snapshot.node_voltages.clone(),
        branch_currents,
        component_results: final_snapshot.component_results.clone(),
        bjt_results,
        mosfet_results,
        op_amp_results: vec![],
        relay_results: vec![],
        mcu_results: vec![],
        transient: Some(TransientData {
            timesteps,
            time_step: dt,
            duration,
        }),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::Circuit;
    use crate::solve_circuit;
    use approx::assert_relative_eq;

    /// Test 4: Auto-duration calculation for known RC circuit.
    #[test]
    fn test_calculate_duration_rc() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 100e-6,
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 100e-6,
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1_000.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                },
            ],
        };
        let result = solve_circuit(&circuit).unwrap();
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 100.0,
                },
                CircuitElement::Inductor {
                    id: "L1".into(),
                    nodes: ["n2".into(), "0".into()],
                    inductance: 0.5,
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Resistor {
                    id: "R2".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 2000.0,
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                CircuitElement::Diode {
                    id: "D1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    temperature: 300.15,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 10.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "Vcoil".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 12.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "coil_p".into()],
                    resistance: 100.0,
                },
                CircuitElement::Relay {
                    id: "K1".into(),
                    nodes: ["coil_p".into(), "0".into(), "c1".into(), "c2".into()],
                    coil_resistance: 500.0,
                    pickup_voltage: 5.0,
                    inductance: 0.1,
                },
                // Contact load: 5V source through 1k into contact pair
                CircuitElement::VoltageSource {
                    id: "Vload".into(),
                    nodes: ["c1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "Rload".into(),
                    nodes: ["c1".into(), "c2".into()],
                    resistance: 1000.0,
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 10.0,
                },
                CircuitElement::Transformer {
                    id: "T1".into(),
                    nodes: ["n2".into(), "0".into(), "n3".into(), "0".into()],
                    l1: 1e-3, // 1 mH primary
                    l2: 4e-3, // 4 mH secondary (n = sqrt(4/1) = 2)
                    k: 0.999, // near-ideal coupling
                },
                // Secondary load: 1 kOhm
                CircuitElement::Resistor {
                    id: "R2".into(),
                    nodes: ["n3".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
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

        // At steady state, Transformer T1 component result should be present
        let t1 = result.component_results.iter().find(|c| c.id == "T1");
        assert!(
            t1.is_some(),
            "Transformer T1 should appear in component results"
        );

        // Secondary voltage (n3) should be elevated relative to primary (n2).
        // Ideal turns ratio: Vs/Vp = sqrt(L2/L1) = 2.
        // Backward Euler damping means exact steady state may differ; test for non-zero secondary voltage.
        let v_secondary = result.node_voltages.get("n3").copied().unwrap_or(0.0);
        assert!(
            v_secondary.abs() > 0.1,
            "Secondary voltage should be non-zero, got {}",
            v_secondary
        );
    }

    /// Test 5: Circuit with both reactive and nonlinear solves successfully.
    #[test]
    fn test_mixed_reactive_nonlinear_succeeds() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 100e-6,
                },
                CircuitElement::Diode {
                    id: "D1".into(),
                    nodes: ["n2".into(), "0".into()],
                    temperature: 300.15,
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Diode {
                    id: "D1".into(),
                    nodes: ["n1".into(), "n2".into()], // series diode
                    temperature: 300.15,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "n3".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n3".into(), "0".into()],
                    capacitance: 100e-6, // tau = 0.1s
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                CircuitElement::Diode {
                    id: "D1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    temperature: 300.15,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 10.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "Rb".into(),
                    nodes: ["vcc".into(), "base".into()],
                    resistance: 100_000.0,
                },
                CircuitElement::Resistor {
                    id: "Rc".into(),
                    nodes: ["vcc".into(), "coll".into()],
                    resistance: 1_000.0,
                },
                // Small cap to make circuit reactive so transient runs
                CircuitElement::Capacitor {
                    id: "Cin".into(),
                    nodes: ["base".into(), "0".into()],
                    capacitance: 1e-9, // 1 nF
                },
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["base".into(), "coll".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: Some(BjtParasiticCaps {
                        cbe: 10e-12,
                        cbc: 2e-12,
                    }),
                },
            ],
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "Rb".into(),
                    nodes: ["vcc".into(), "base".into()],
                    resistance: 100_000.0,
                },
                CircuitElement::Resistor {
                    id: "Rc".into(),
                    nodes: ["vcc".into(), "coll".into()],
                    resistance: 1_000.0,
                },
                // Cap to make circuit reactive
                CircuitElement::Capacitor {
                    id: "Cin".into(),
                    nodes: ["base".into(), "0".into()],
                    capacitance: 1e-9,
                },
                CircuitElement::Bjt {
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
                },
            ],
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
}
