use nalgebra::DVector;

use sindr_devices::bjt::{self, BjtKind, BjtParams};
use sindr_devices::diode::{self, temperature_scale_is, DiodeParams};
use sindr_devices::jfet::JfetKind;
use sindr_devices::led::led_params_from_str;
use sindr_devices::mosfet::{self, MosfetKind, MosfetParams};
use sindr_devices::photoresistor::{ldr_resistance as rs_ldr_resistance, PhotoresistorParams};
use sindr_devices::zener::{zener_companion as rs_zener_companion, ZenerParams};

use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;

/// Switch closed resistance (very low, near short circuit).
pub const SWITCH_R_CLOSED: f64 = 0.01;
/// Switch open resistance (very high, near open circuit).
pub const SWITCH_R_OPEN: f64 = 1e9;

/// Compute LDR (photoresistor) resistance from light level.
pub fn ldr_resistance(light_level: f64) -> f64 {
    rs_ldr_resistance(light_level, &PhotoresistorParams::default())
}

/// Stamp all components in a circuit into the MNA system.
///
/// Iterates through every component and dispatches to the appropriate
/// stamping function. Voltage sources are numbered sequentially
/// (0, 1, 2, ...) in the order they appear in the component list.
///
/// When `v_prev` is `Some`, nonlinear components (Diode, Led, BJT, MOSFET) are stamped
/// using their companion model evaluated at the given operating point.
/// When `v_prev` is `None`, nonlinear components are skipped (linear solve).
pub fn stamp_circuit(
    circuit: &Circuit,
    system: &mut MnaSystem,
    node_map: &NodeMap,
    v_prev: Option<&DVector<f64>>,
) -> Result<(), SimError> {
    let num_nodes = node_map.num_nodes();
    let mut vsource_index: usize = 0;

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
            CircuitElement::VoltageSource { nodes, voltage, .. } => {
                let branch = num_nodes + vsource_index;
                stamp_voltage_source(system, node_map, nodes, *voltage, branch);
                vsource_index += 1;
            }
            CircuitElement::CurrentSource { nodes, current, .. } => {
                stamp_current_source(system, node_map, nodes, *current);
            }
            CircuitElement::Switch { nodes, closed, .. } => {
                let resistance = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                stamp_resistor(system, node_map, nodes, resistance);
            }
            // Open-circuit stubs -- simulation added in future phases
            CircuitElement::Capacitor { .. } => {}
            CircuitElement::Inductor { .. } => {}
            CircuitElement::Diode {
                nodes, temperature, ..
            } => {
                if let Some(vp) = v_prev {
                    let mut params = DiodeParams::silicon();
                    if (*temperature - 300.15).abs() > 1e-6 {
                        params.is =
                            temperature_scale_is(params.is, *temperature, 300.15, 1.11, 2.0);
                    }
                    stamp_diode_companion(system, node_map, nodes, vp, &params);
                }
            }
            CircuitElement::Led {
                nodes,
                color,
                temperature,
                ..
            } => {
                if let Some(vp) = v_prev {
                    let mut params = led_params_from_str(color);
                    if (*temperature - 300.15).abs() > 1e-6 {
                        params.is =
                            temperature_scale_is(params.is, *temperature, 300.15, 1.11, 2.0);
                    }
                    stamp_diode_companion(system, node_map, nodes, vp, &params);
                }
            }
            CircuitElement::Bjt {
                nodes,
                kind,
                bf,
                temperature,
                ..
            } => {
                if let Some(vp) = v_prev {
                    let mut params = BjtParams::new(*bf);
                    if (*temperature - 300.15).abs() > 1e-6 {
                        // Scale IS for temperature: xti=3 for BJT (SPICE standard)
                        params.is =
                            temperature_scale_is(params.is, *temperature, 300.15, 1.11, 3.0);
                    }
                    stamp_bjt_companion(system, node_map, nodes, vp, &params, *kind);
                }
            }
            CircuitElement::Mosfet {
                nodes,
                kind,
                params,
                ..
            } => {
                if let Some(vp) = v_prev {
                    stamp_mosfet_companion(system, node_map, nodes, vp, params, *kind);
                }
            }
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
                    let ctrl_branch_idx = num_nodes + cb;
                    stamp_ccvs(system, node_map, nodes, *rm, branch, ctrl_branch_idx);
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
                    let ctrl_branch_idx = num_nodes + cb;
                    stamp_cccs(system, node_map, nodes, *alpha, ctrl_branch_idx);
                }
            }
            CircuitElement::Pushbutton { nodes, closed, .. } => {
                let resistance = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                stamp_resistor(system, node_map, nodes, resistance);
            }
            CircuitElement::Fuse { nodes, blown, .. } => {
                // Intact (blown=false): 0.001 Ω (1 mΩ) — negligible drop.
                // Blown (blown=true):  1e9 Ω — effectively open circuit.
                let resistance = if *blown { SWITCH_R_OPEN } else { 0.001 };
                stamp_resistor(system, node_map, nodes, resistance);
            }
            CircuitElement::Photoresistor {
                nodes, light_level, ..
            } => {
                let resistance = ldr_resistance(*light_level);
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
                // Coil: passive resistive load
                let coil_nodes: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                stamp_resistor(system, node_map, &coil_nodes, *coil_resistance);
                let _ = inductance; // inductance is handled in transient solver
                                    // Contact: determined by previous iteration coil voltage
                let contact_closed = if let Some(vp) = v_prev {
                    let vc_pos = node_map.index(&nodes[0]).map_or(0.0, |i| vp[i]);
                    let vc_neg = node_map.index(&nodes[1]).map_or(0.0, |i| vp[i]);
                    (vc_pos - vc_neg).abs() >= *pickup_voltage
                } else {
                    false
                };
                let contact_r = if contact_closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                let contact_nodes: [String; 2] = [nodes[2].clone(), nodes[3].clone()];
                stamp_resistor(system, node_map, &contact_nodes, contact_r);
            }
            // ZenerDiode: piecewise nonlinear companion model with breakdown clamping
            CircuitElement::ZenerDiode {
                nodes,
                vz,
                temperature,
                ..
            } => {
                if let Some(vp) = v_prev {
                    let mut zparams = ZenerParams::new(*vz);
                    if (*temperature - 300.15).abs() > 1e-6 {
                        zparams.is =
                            temperature_scale_is(zparams.is, *temperature, 300.15, 1.11, 2.0);
                    }
                    stamp_zener_companion(system, node_map, nodes, vp, &zparams);
                }
            }
            // OpAmp: ideal high-gain VCVS (gain=1e5). OUT is relative to circuit ground.
            CircuitElement::OpAmp { nodes, .. } => {
                let branch = num_nodes + vsource_index;
                let out_nodes: [String; 2] = [nodes[2].clone(), circuit.ground_node.clone()];
                let ctrl_nodes: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                stamp_vcvs(system, node_map, &out_nodes, &ctrl_nodes, 1e5, branch);
                vsource_index += 1;
            }
            // Comparator: identical VCVS stamp to OpAmp; output saturates naturally at rails.
            CircuitElement::Comparator { nodes, .. } => {
                let branch = num_nodes + vsource_index;
                let out_nodes: [String; 2] = [nodes[2].clone(), circuit.ground_node.clone()];
                let ctrl_nodes: [String; 2] = [nodes[0].clone(), nodes[1].clone()];
                stamp_vcvs(system, node_map, &out_nodes, &ctrl_nodes, 1e5, branch);
                vsource_index += 1;
            }
            // Schottky diode — same nonlinear pattern as silicon Diode but with Schottky IS/N
            CircuitElement::SchottkyDiode {
                nodes, temperature, ..
            } => {
                if let Some(vp) = v_prev {
                    let schottky = sindr_devices::schottky::SchottkyParams::default();
                    let is_scaled = if (*temperature - 300.15).abs() > 1e-6 {
                        temperature_scale_is(schottky.is, *temperature, 300.15, 1.11, 2.0)
                    } else {
                        schottky.is
                    };
                    stamp_diode_companion(
                        system,
                        node_map,
                        nodes,
                        vp,
                        &DiodeParams {
                            is: is_scaled,
                            n: schottky.n,
                            rs: 0.0,
                            temperature: *temperature,
                        },
                    );
                }
            }
            // Thermistor — passive resistor, resistance computed from temperature param
            CircuitElement::Thermistor {
                nodes, temperature, ..
            } => {
                let params = sindr_devices::thermistor::ThermistorParams::default();
                let r = sindr_devices::thermistor::thermistor_resistance(*temperature, &params);
                stamp_resistor(system, node_map, nodes, r);
            }
            // Varactor — open circuit in DC; transient handled in stamp_varactor_transient
            CircuitElement::Varactor { nodes, .. } => {
                stamp_varactor_dc(system, node_map, nodes);
            }
            // IGBT — nonlinear companion model (gate-controlled); NR iteration
            CircuitElement::Igbt { nodes, params, .. } => {
                if let Some(vp) = v_prev {
                    stamp_igbt_companion(system, node_map, nodes, vp, params);
                }
            }
            // JFET — Shockley square-law, NR companion (no branch variable)
            CircuitElement::Jfet {
                nodes,
                kind,
                idss,
                vp,
                ..
            } => {
                if let Some(vp_prev) = v_prev {
                    stamp_jfet_companion(system, node_map, nodes, *kind, *idss, *vp, vp_prev);
                }
            }
            // Transformer — DC: both windings are near-short-circuit resistors
            CircuitElement::Transformer { nodes, .. } => {
                // DC: pure inductors are short circuits. Use very small resistance to avoid singular matrix.
                stamp_resistor(
                    system,
                    node_map,
                    &[nodes[0].clone(), nodes[1].clone()],
                    1e-9,
                );
                stamp_resistor(
                    system,
                    node_map,
                    &[nodes[2].clone(), nodes[3].clone()],
                    1e-9,
                );
            }
            // VoltageRegulator — ideal voltage source between output (nodes[1]) and gnd (nodes[2]).
            // Input node (nodes[0]) is wiring-only and is ignored.
            CircuitElement::VoltageRegulator { nodes, voltage, .. } => {
                let branch = num_nodes + vsource_index;
                let vs_nodes: [String; 2] = [nodes[1].clone(), nodes[2].clone()];
                stamp_voltage_source(system, node_map, &vs_nodes, *voltage, branch);
                vsource_index += 1;
            }
            // Photodiode — diode + photocurrent offset
            CircuitElement::Photodiode {
                nodes,
                irradiance,
                temperature,
                ..
            } => {
                if let Some(vp) = v_prev {
                    let _ = temperature; // temperature scaling for photodiode IS reserved for temp_sweep
                    let params = sindr_devices::photodiode::PhotodiodeParams::default();
                    let v_a = node_map.index(&nodes[0]).map_or(0.0, |i| vp[i]);
                    let v_c = node_map.index(&nodes[1]).map_or(0.0, |i| vp[i]);
                    let v_d = v_a - v_c;
                    let (g_d, i_eq) =
                        sindr_devices::photodiode::photodiode_companion(v_d, *irradiance, &params);
                    // Stamp Norton equivalent: conductance between anode/cathode + current source
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
            }
        }
    }

    Ok(())
}

/// Stamp a resistor (conductance `g = 1/R`) into the G submatrix.
///
/// For nodes `p` and `q` with conductance `g`:
/// ```text
///   A[p,p] += g,  A[q,q] += g,  A[p,q] -= g,  A[q,p] -= g
/// ```
/// Ground entries (where index is `None`) are skipped.
pub fn stamp_resistor(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    resistance: f64,
) {
    let g = 1.0 / resistance;
    let p = node_map.index(&nodes[0]);
    let q = node_map.index(&nodes[1]);

    if let Some(pi) = p {
        system.a[(pi, pi)] += g;
    }
    if let Some(qi) = q {
        system.a[(qi, qi)] += g;
    }
    if let (Some(pi), Some(qi)) = (p, q) {
        system.a[(pi, qi)] -= g;
        system.a[(qi, pi)] -= g;
    }
}

/// Stamp an independent voltage source into B, C submatrices and RHS.
///
/// `nodes[0]` is the positive terminal (`p`), `nodes[1]` the negative (`q`).
/// Branch index `k` is the row/column for this source in the MNA system.
///
/// Stamps:
/// ```text
///   A[p,k] += 1, A[q,k] -= 1, A[k,p] += 1, A[k,q] -= 1, b[k] = voltage
/// ```
pub fn stamp_voltage_source(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    voltage: f64,
    branch: usize,
) {
    let p = node_map.index(&nodes[0]);
    let q = node_map.index(&nodes[1]);

    // B submatrix (node rows, branch column)
    if let Some(pi) = p {
        system.a[(pi, branch)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(qi, branch)] -= 1.0;
    }

    // C submatrix (branch row, node columns)
    if let Some(pi) = p {
        system.a[(branch, pi)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(branch, qi)] -= 1.0;
    }

    // RHS: voltage value
    system.b[branch] = voltage;
}

/// Stamp an independent current source into the RHS vector.
///
/// Current flows from `nodes[0]` toward `nodes[1]`.
/// KCL convention: current *entering* a node is positive.
/// ```text
///   b[to]   += current   (current enters the to-node)
///   b[from] -= current   (current leaves the from-node)
/// ```
pub fn stamp_current_source(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    current: f64,
) {
    let from = node_map.index(&nodes[0]);
    let to = node_map.index(&nodes[1]);

    if let Some(ti) = to {
        system.b[ti] += current;
    }
    if let Some(fi) = from {
        system.b[fi] -= current;
    }
}

/// Stamp a diode/LED companion model into the MNA system.
///
/// Evaluates the companion model at the current operating point (from v_prev)
/// and stamps the resulting conductance and current source.
///
/// nodes[0] = anode, nodes[1] = cathode.
pub(crate) fn stamp_diode_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    v_prev: &DVector<f64>,
    params: &DiodeParams,
) {
    let p = node_map.index(&nodes[0]); // anode
    let q = node_map.index(&nodes[1]); // cathode

    // Compute diode voltage from operating point
    let v_anode = p.map_or(0.0, |i| v_prev[i]);
    let v_cathode = q.map_or(0.0, |i| v_prev[i]);
    let v_d = v_anode - v_cathode;

    // Get companion model parameters
    let (g_d, i_eq) = diode::diode_companion(v_d, params);

    // Stamp conductance (same pattern as resistor with g = g_d)
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

    // Stamp current source (current flows anode -> cathode)
    if let Some(pi) = p {
        system.b[pi] -= i_eq;
    }
    if let Some(qi) = q {
        system.b[qi] += i_eq;
    }
}

/// Stamp a zener diode companion model into the MNA system.
///
/// nodes[0] = anode, nodes[1] = cathode.
pub(crate) fn stamp_zener_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    v_prev: &DVector<f64>,
    params: &ZenerParams,
) {
    let p = node_map.index(&nodes[0]); // anode
    let q = node_map.index(&nodes[1]); // cathode

    let v_anode = p.map_or(0.0, |i| v_prev[i]);
    let v_cathode = q.map_or(0.0, |i| v_prev[i]);
    let v_d = v_anode - v_cathode;

    let (g_eq, i_eq) = rs_zener_companion(v_d, params);

    if let Some(pi) = p {
        system.a[(pi, pi)] += g_eq;
    }
    if let Some(qi) = q {
        system.a[(qi, qi)] += g_eq;
    }
    if let (Some(pi), Some(qi)) = (p, q) {
        system.a[(pi, qi)] -= g_eq;
        system.a[(qi, pi)] -= g_eq;
    }

    if let Some(pi) = p {
        system.b[pi] -= i_eq;
    }
    if let Some(qi) = q {
        system.b[qi] += i_eq;
    }
}

/// Stamp a capacitor companion model (Backward Euler) into the MNA system.
///
/// Converts a capacitor into a conductance + current source pair:
/// ```text
///   G_eq = C / dt
///   I_eq = G_eq * V_prev_across = (C / dt) * V(n-1)
/// ```
///
/// The conductance is stamped like a resistor. The current source
/// flows from `nodes[1]` to `nodes[0]` (into positive terminal):
/// ```text
///   b[p] += I_eq,  b[q] -= I_eq
/// ```
pub fn stamp_capacitor_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    capacitance: f64,
    dt: f64,
    v_prev_across: f64,
) {
    let g_eq = capacitance / dt;
    let i_eq = g_eq * v_prev_across;

    let p = node_map.index(&nodes[0]);
    let q = node_map.index(&nodes[1]);

    // Stamp conductance (same pattern as resistor)
    if let Some(pi) = p {
        system.a[(pi, pi)] += g_eq;
    }
    if let Some(qi) = q {
        system.a[(qi, qi)] += g_eq;
    }
    if let (Some(pi), Some(qi)) = (p, q) {
        system.a[(pi, qi)] -= g_eq;
        system.a[(qi, pi)] -= g_eq;
    }

    // Stamp current source: current enters nodes[0] (positive terminal)
    if let Some(pi) = p {
        system.b[pi] += i_eq;
    }
    if let Some(qi) = q {
        system.b[qi] -= i_eq;
    }
}

/// Stamp an inductor companion model (Backward Euler) into the MNA system.
///
/// Converts an inductor into a conductance + current source pair:
/// ```text
///   G_eq = dt / L
///   I_eq = I_prev (previous inductor current)
/// ```
///
/// The conductance is stamped like a resistor. The current source
/// flows from `nodes[1]` to `nodes[0]` (into positive terminal):
/// ```text
///   b[p] += I_eq,  b[q] -= I_eq
/// ```
pub fn stamp_inductor_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    inductance: f64,
    dt: f64,
    i_prev: f64,
) {
    let g_eq = dt / inductance;
    let i_eq = i_prev;

    let p = node_map.index(&nodes[0]);
    let q = node_map.index(&nodes[1]);

    // Stamp conductance (same pattern as resistor)
    if let Some(pi) = p {
        system.a[(pi, pi)] += g_eq;
    }
    if let Some(qi) = q {
        system.a[(qi, qi)] += g_eq;
    }
    if let (Some(pi), Some(qi)) = (p, q) {
        system.a[(pi, qi)] -= g_eq;
        system.a[(qi, pi)] -= g_eq;
    }

    // Stamp current source: inductor current flows from p to q,
    // so the equivalent source REMOVES current from p and INJECTS into q.
    if let Some(pi) = p {
        system.b[pi] -= i_eq;
    }
    if let Some(qi) = q {
        system.b[qi] += i_eq;
    }
}

/// Stamp a BJT Ebers-Moll companion model into the MNA system.
///
/// Evaluates the companion model at the current operating point (from v_prev)
/// and stamps the resulting 9 G-matrix entries + 3 RHS entries.
///
/// nodes: [base, collector, emitter].
/// PNP is handled via polarity multiplier on junction voltages and output currents.
pub(crate) fn stamp_bjt_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3],
    v_prev: &DVector<f64>,
    params: &BjtParams,
    kind: BjtKind,
) {
    let b = node_map.index(&nodes[0]); // base
    let c = node_map.index(&nodes[1]); // collector
    let e = node_map.index(&nodes[2]); // emitter

    // Get node voltages from previous iteration
    let vb = b.map_or(0.0, |i| v_prev[i]);
    let vc = c.map_or(0.0, |i| v_prev[i]);
    let ve = e.map_or(0.0, |i| v_prev[i]);

    // Apply polarity for PNP
    let sign = match kind {
        BjtKind::Npn => 1.0,
        BjtKind::Pnp => -1.0,
    };
    let vbe_eff = sign * (vb - ve);
    let vbc_eff = sign * (vb - vc);

    // Evaluate companion model
    let comp = bjt::bjt_companion(vbe_eff, vbc_eff, params);

    let g_be = comp.g_be;
    let g_bc = comp.g_bc;
    let alpha_f = params.bf / (params.bf + 1.0);
    let alpha_r = params.br / (params.br + 1.0);

    // Helper closure: stamp a value into G matrix (skip ground nodes)
    let stamp_g = |system: &mut MnaSystem, row: Option<usize>, col: Option<usize>, val: f64| {
        if let (Some(r), Some(c)) = (row, col) {
            system.a[(r, c)] += val;
        }
    };

    // --- G matrix stamp (9 entries) ---
    // These entries are IDENTICAL for NPN and PNP because the sign cancels
    // in the chain rule: dIc_actual/dVb = sign * dIc_internal/dVbe_eff * sign = dIc_internal/dVbe_eff.

    // Row B (base KCL -- dIb/dV):
    stamp_g(system, b, b, g_be / params.bf + g_bc / params.br);
    stamp_g(system, b, c, -g_bc / params.br);
    stamp_g(system, b, e, -g_be / params.bf);

    // Row C (collector KCL -- dIc/dV):
    // Ic = IF - IR/alpha_R, so dIc/dVbc = -g_bc/alpha_R (NEGATIVE)
    stamp_g(system, c, b, g_be - g_bc / alpha_r); // dIc/dVbe * 1 + dIc/dVbc * 1
    stamp_g(system, c, c, g_bc / alpha_r); // dIc/dVbc * (-1)
    stamp_g(system, c, e, -g_be); // dIc/dVbe * (-1) -- unchanged

    // Row E (emitter KCL -- Ie = -IF/alpha_F + IR):
    // dIe/dVbe = -g_be/alpha_F, dIe/dVbc = g_bc
    stamp_g(system, e, b, -g_be / alpha_f + g_bc); // dIe/dVbe * 1 + dIe/dVbc * 1
    stamp_g(system, e, c, -g_bc); // dIe/dVbc * (-1)
    stamp_g(system, e, e, g_be / alpha_f); // dIe/dVbe * (-1) -- unchanged

    // --- RHS stamp (3 entries) ---
    // Companion current sources (linearization residuals):
    let ib_eq = comp.ib - (g_be / params.bf) * vbe_eff - (g_bc / params.br) * vbc_eff;
    let ic_eq = comp.ic - g_be * vbe_eff + (g_bc / alpha_r) * vbc_eff;
    let ie_eq = -(ib_eq + ic_eq);

    // For PNP, multiply equivalent currents by sign
    let ib_rhs = sign * ib_eq;
    let ic_rhs = sign * ic_eq;
    let ie_rhs = sign * ie_eq;

    // Stamp into RHS (MNA convention: current entering node is positive)
    if let Some(bi) = b {
        system.b[bi] -= ib_rhs;
    }
    if let Some(ci) = c {
        system.b[ci] -= ic_rhs;
    }
    if let Some(ei) = e {
        system.b[ei] -= ie_rhs;
    }

    // --- Early voltage output conductance: g_ce between collector and emitter ---
    // comp.g_ce = 0.0 when params.vaf = 0.0 (no Early effect); non-zero otherwise.
    let g_ce = comp.g_ce;
    if g_ce > 0.0 {
        // Re-resolve c and e since they were consumed by stamp_g closure above
        let c_idx = node_map.index(&nodes[1]); // collector
        let e_idx = node_map.index(&nodes[2]); // emitter
        if let Some(ci) = c_idx {
            system.a[(ci, ci)] += g_ce;
        }
        if let Some(ei) = e_idx {
            system.a[(ei, ei)] += g_ce;
        }
        if let (Some(ci), Some(ei)) = (c_idx, e_idx) {
            system.a[(ci, ei)] -= g_ce;
            system.a[(ei, ci)] -= g_ce;
        }
    }
}

/// Stamp a MOSFET companion model into the MNA system.
///
/// Linearizes the Level 1 MOSFET model at the current operating point
/// and stamps the resulting conductances + current sources.
///
/// nodes: [gate, drain, source].
/// PMOS is handled via polarity flip on terminal voltages.
pub(crate) fn stamp_mosfet_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3],
    v_prev: &DVector<f64>,
    params: &MosfetParams,
    kind: MosfetKind,
) {
    let g = node_map.index(&nodes[0]); // gate
    let d = node_map.index(&nodes[1]); // drain
    let s = node_map.index(&nodes[2]); // source

    let vg = g.map_or(0.0, |i| v_prev[i]);
    let vd = d.map_or(0.0, |i| v_prev[i]);
    let vs = s.map_or(0.0, |i| v_prev[i]);

    // Sign adjustment for PMOS
    let (vgs, vds, vbs) = match kind {
        MosfetKind::Nmos => (vg - vs, vd - vs, -vs), // body at ground
        MosfetKind::Pmos => (vs - vg, vs - vd, vs),
    };

    let comp = mosfet::mosfet_companion(vgs, vds, vbs, params);

    // Helper closure
    let stamp_g = |system: &mut MnaSystem, row: Option<usize>, col: Option<usize>, val: f64| {
        if let (Some(r), Some(c)) = (row, col) {
            system.a[(r, c)] += val;
        }
    };

    // For NMOS: Id flows drain->source, controlled by Vgs and Vds
    // Linearized: Id = Id0 + gm*(vgs - Vgs0) + gds*(vds - Vds0) + gmb*(vbs - Vbs0)
    //
    // G-matrix stamps (3x3 Jacobian):
    // dId/dVg = gm, dId/dVd = gds, dId/dVs = -(gm + gds + gmb)
    // For drain row (current entering drain):
    stamp_g(system, d, g, comp.gm);
    stamp_g(system, d, d, comp.gds);
    stamp_g(system, d, s, -(comp.gm + comp.gds + comp.gmb));

    // Source row (KCL: Is = -Id, so dIs/dX = -dId/dX):
    stamp_g(system, s, g, -comp.gm);
    stamp_g(system, s, d, -comp.gds);
    stamp_g(system, s, s, comp.gm + comp.gds + comp.gmb);

    // Gate row: Ig = 0 (no gate current in MOSFET), no stamps needed

    // RHS: companion current sources
    // I_eq = Id0 - gm*Vgs0 - gds*Vds0 - gmb*Vbs0
    let i_eq = comp.id - comp.gm * vgs - comp.gds * vds - comp.gmb * vbs;

    // Apply PMOS sign correction: physical current flows source->drain
    let sign = match kind {
        MosfetKind::Nmos => 1.0,
        MosfetKind::Pmos => -1.0,
    };

    let i_rhs = sign * i_eq;

    // Stamp RHS: current enters drain, leaves source
    if let Some(di) = d {
        system.b[di] -= i_rhs;
    }
    if let Some(si) = s {
        system.b[si] += i_rhs;
    }
}

/// Stamp a varactor in DC analysis: open circuit via very large shunt resistor (1e12 Ω).
///
/// In transient analysis, use stamp_varactor_transient instead.
pub(crate) fn stamp_varactor_dc(system: &mut MnaSystem, node_map: &NodeMap, nodes: &[String; 2]) {
    // 1e12 Ω shunt = open circuit for practical purposes
    stamp_resistor(system, node_map, nodes, 1e12);
}

/// Stamp a varactor in transient analysis: voltage-dependent capacitor companion model.
///
/// Uses backward Euler: g_eq = C_j(v_prev) / dt, i_eq = -g_eq * v_prev.
/// v_prev is the voltage across the varactor at the previous timestep (solution_prev[anode] - solution_prev[cathode]).
pub(crate) fn stamp_varactor_transient(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    v_prev_across: f64,
    dt: f64,
    params: &sindr_devices::varactor::VaractorParams,
) {
    let (g_eq, i_eq) = sindr_devices::varactor::varactor_companion(v_prev_across, dt, params);
    let p = node_map.index(&nodes[0]);
    let q = node_map.index(&nodes[1]);

    // Stamp conductance (same pattern as capacitor companion)
    if let Some(pi) = p {
        system.a[(pi, pi)] += g_eq;
    }
    if let Some(qi) = q {
        system.a[(qi, qi)] += g_eq;
    }
    if let (Some(pi), Some(qi)) = (p, q) {
        system.a[(pi, qi)] -= g_eq;
        system.a[(qi, pi)] -= g_eq;
    }

    // Stamp history current source: enters positive terminal (nodes[0])
    // i_eq = -g_eq * v_prev (from varactor_companion sign convention)
    if let Some(pi) = p {
        system.b[pi] += i_eq;
    }
    if let Some(qi) = q {
        system.b[qi] -= i_eq;
    }
}

/// Stamp an IGBT companion model (nonlinear, called each NR iteration).
///
/// Linearizes IGBT (MOSFET gate + BJT output) at operating point v_prev.
/// nodes: [gate, collector, emitter]
pub(crate) fn stamp_igbt_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3],
    v_prev: &DVector<f64>,
    params: &sindr_devices::igbt::IgbtParams,
) {
    let g = node_map.index(&nodes[0]); // gate
    let c = node_map.index(&nodes[1]); // collector
    let e = node_map.index(&nodes[2]); // emitter

    let vg = g.map_or(0.0, |i| v_prev[i]);
    let vc = c.map_or(0.0, |i| v_prev[i]);
    let ve = e.map_or(0.0, |i| v_prev[i]);
    let vge = vg - ve;
    let vce = vc - ve;

    let comp = sindr_devices::igbt::igbt_companion(vge, vce, params);

    let g_total = comp.gm + comp.g_ce;

    // Stamp gm + g_ce between collector and emitter (output conductance)
    if let Some(ci) = c {
        system.a[(ci, ci)] += g_total;
    }
    if let Some(ei) = e {
        system.a[(ei, ei)] += g_total;
    }
    if let (Some(ci), Some(ei)) = (c, e) {
        system.a[(ci, ei)] -= g_total;
        system.a[(ei, ci)] -= g_total;
    }

    // Stamp companion current source: I_eq from emitter to collector
    // i_eq = ids - gm*vge - g_ce*vce (companion source value)
    if let Some(ci) = c {
        system.b[ci] -= comp.i_eq;
    }
    if let Some(ei) = e {
        system.b[ei] += comp.i_eq;
    }
}

/// Stamp a JFET companion model into the MNA system.
///
/// Linearizes the Shockley square-law JFET model at the current operating point.
/// nodes: [gate, drain, source].
/// P-channel is handled via sign flip inside jfet_companion().
pub(crate) fn stamp_jfet_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3], // [gate, drain, source]
    kind: JfetKind,
    idss: f64,
    vp: f64,
    v_prev: &nalgebra::DVector<f64>,
) {
    let g = node_map.index(&nodes[0]); // gate
    let d = node_map.index(&nodes[1]); // drain
    let s = node_map.index(&nodes[2]); // source

    let vg = g.map_or(0.0, |i| v_prev[i]);
    let vd = d.map_or(0.0, |i| v_prev[i]);
    let vs = s.map_or(0.0, |i| v_prev[i]);

    let vgs = vg - vs;
    let vds = vd - vs;

    let c = sindr_devices::jfet::jfet_companion(vgs, vds, kind, idss, vp);

    // Stamp drain-source conductance gds
    if let Some(di) = d {
        system.a[(di, di)] += c.gds;
    }
    if let Some(si) = s {
        system.a[(si, si)] += c.gds;
    }
    if let (Some(di), Some(si)) = (d, s) {
        system.a[(di, si)] -= c.gds;
        system.a[(si, di)] -= c.gds;
    }

    // Stamp VCCS: gm * Vgs = gm * (Vg - Vs)
    // Current flows drain->source (exits drain, enters source)
    if let (Some(di), Some(gi)) = (d, g) {
        system.a[(di, gi)] += c.gm;
    }
    if let (Some(di), Some(si)) = (d, s) {
        system.a[(di, si)] -= c.gm;
    }
    if let (Some(si), Some(gi)) = (s, g) {
        system.a[(si, gi)] -= c.gm;
    }
    if let (Some(si2), Some(si)) = (s, s) {
        system.a[(si2, si)] += c.gm;
    }

    // Stamp equivalent current source i_eq (flows from source to drain in MNA)
    // Id = gm*Vgs + gds*Vds + i_eq
    if let Some(di) = d {
        system.b[di] -= c.i_eq;
    }
    if let Some(si) = s {
        system.b[si] += c.i_eq;
    }
}

/// Stamp BJT parasitic capacitances as companion capacitors.
///
/// Uses HashMap-keyed v_prev values (not positional Vec).
/// nodes: [base, collector, emitter]
pub(crate) fn stamp_bjt_parasitic_caps(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3],
    caps: &crate::circuit::BjtParasiticCaps,
    dt: f64,
    v_be_prev: f64,
    v_bc_prev: f64,
) {
    if caps.cbe > 0.0 {
        stamp_capacitor_companion(
            system,
            node_map,
            &[nodes[0].clone(), nodes[2].clone()], // base-emitter
            caps.cbe,
            dt,
            v_be_prev,
        );
    }
    if caps.cbc > 0.0 {
        stamp_capacitor_companion(
            system,
            node_map,
            &[nodes[0].clone(), nodes[1].clone()], // base-collector
            caps.cbc,
            dt,
            v_bc_prev,
        );
    }
}

/// Stamp MOSFET parasitic capacitances (Cgs, Cgd) as companion capacitors.
///
/// Uses HashMap-keyed v_prev values (not positional Vec).
/// nodes: [gate, drain, source]
pub(crate) fn stamp_mosfet_parasitic_caps(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 3],
    caps: &crate::circuit::MosfetParasiticCaps,
    dt: f64,
    v_gs_prev: f64,
    v_gd_prev: f64,
) {
    if caps.cgs > 0.0 {
        stamp_capacitor_companion(
            system,
            node_map,
            &[nodes[0].clone(), nodes[2].clone()], // gate-source
            caps.cgs,
            dt,
            v_gs_prev,
        );
    }
    if caps.cgd > 0.0 {
        stamp_capacitor_companion(
            system,
            node_map,
            &[nodes[0].clone(), nodes[1].clone()], // gate-drain
            caps.cgd,
            dt,
            v_gd_prev,
        );
    }
}

/// Stamp coupled inductor transformer (Backward Euler, 2 branch current unknowns).
///
/// branch_row_k1: index into extended MNA rows for primary branch current (= num_nodes + vsource_offset)
/// branch_row_k2: branch_row_k1 + 1 (secondary)
/// i1_prev, i2_prev: primary and secondary currents from previous timestep
///
/// MNA equations (backward Euler):
///   -V(p1) + V(q1) + (L1/dt)*I1 + (M/dt)*I2 = L1/dt*I1_prev + M/dt*I2_prev
///   -V(p2) + V(q2) + (L2/dt)*I2 + (M/dt)*I1 = L2/dt*I2_prev + M/dt*I1_prev
#[allow(clippy::too_many_arguments)]
pub(crate) fn stamp_transformer_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 4], // [p1, q1, p2, q2]
    l1: f64,
    l2: f64,
    k: f64,
    dt: f64,
    i1_prev: f64,
    i2_prev: f64,
    k1: usize, // branch row/col index for primary current
    k2: usize, // branch row/col index for secondary current
) {
    let m = k * (l1 * l2).sqrt(); // mutual inductance

    let p1 = node_map.index(&nodes[0]);
    let q1 = node_map.index(&nodes[1]);
    let p2 = node_map.index(&nodes[2]);
    let q2 = node_map.index(&nodes[3]);

    // Primary winding: branch k1 between p1 and q1
    if let Some(p1i) = p1 {
        system.a[(p1i, k1)] += 1.0;
        system.a[(k1, p1i)] += 1.0;
    }
    if let Some(q1i) = q1 {
        system.a[(q1i, k1)] -= 1.0;
        system.a[(k1, q1i)] -= 1.0;
    }
    system.a[(k1, k1)] += l1 / dt;
    system.a[(k1, k2)] += m / dt;

    // Secondary winding: branch k2 between p2 and q2
    if let Some(p2i) = p2 {
        system.a[(p2i, k2)] += 1.0;
        system.a[(k2, p2i)] += 1.0;
    }
    if let Some(q2i) = q2 {
        system.a[(q2i, k2)] -= 1.0;
        system.a[(k2, q2i)] -= 1.0;
    }
    system.a[(k2, k2)] += l2 / dt;
    system.a[(k2, k1)] += m / dt; // symmetric

    // History currents (RHS b vector, branch rows):
    system.b[k1] += (l1 / dt) * i1_prev + (m / dt) * i2_prev;
    system.b[k2] += (l2 / dt) * i2_prev + (m / dt) * i1_prev;
}

/// Stamp a Voltage-Controlled Voltage Source (VCVS).
///
/// V_out = mu * V_control
/// nodes: [out+, out-], control_nodes: [ctrl+, ctrl-]
///
/// MNA stamp: adds a branch equation V(out+) - V(out-) = mu * (V(ctrl+) - V(ctrl-))
pub fn stamp_vcvs(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    control_nodes: &[String; 2],
    gain: f64,
    branch: usize,
) {
    let p = node_map.index(&nodes[0]);
    let q = node_map.index(&nodes[1]);
    let cp = node_map.index(&control_nodes[0]);
    let cq = node_map.index(&control_nodes[1]);

    // B submatrix: node rows, branch column
    if let Some(pi) = p {
        system.a[(pi, branch)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(qi, branch)] -= 1.0;
    }

    // C submatrix: branch row - V(p) - V(q) - mu*V(cp) + mu*V(cq) = 0
    if let Some(pi) = p {
        system.a[(branch, pi)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(branch, qi)] -= 1.0;
    }
    if let Some(cpi) = cp {
        system.a[(branch, cpi)] -= gain;
    }
    if let Some(cqi) = cq {
        system.a[(branch, cqi)] += gain;
    }
    // RHS = 0 (no independent voltage)
}

/// Stamp a Voltage-Controlled Current Source (VCCS).
///
/// I_out = gm * V_control
/// nodes: [from, to] (current flows from->to), control_nodes: [ctrl+, ctrl-]
pub fn stamp_vccs(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    control_nodes: &[String; 2],
    gm: f64,
) {
    let from = node_map.index(&nodes[0]);
    let to = node_map.index(&nodes[1]);
    let cp = node_map.index(&control_nodes[0]);
    let cq = node_map.index(&control_nodes[1]);

    // I = gm * (V(cp) - V(cq))
    // Stamps into G matrix:
    // Row to: += gm*V(cp) - gm*V(cq)
    // Row from: -= gm*V(cp) - gm*V(cq)
    if let (Some(ti), Some(cpi)) = (to, cp) {
        system.a[(ti, cpi)] += gm;
    }
    if let (Some(ti), Some(cqi)) = (to, cq) {
        system.a[(ti, cqi)] -= gm;
    }
    if let (Some(fi), Some(cpi)) = (from, cp) {
        system.a[(fi, cpi)] -= gm;
    }
    if let (Some(fi), Some(cqi)) = (from, cq) {
        system.a[(fi, cqi)] += gm;
    }
}

/// Stamp a Current-Controlled Voltage Source (CCVS).
///
/// V_out = rm * I_control
/// nodes: [out+, out-], rm = transresistance
///
/// The controlling current is the branch current of another voltage source.
pub fn stamp_ccvs(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    rm: f64,
    branch: usize,
    ctrl_branch: usize,
) {
    let p = node_map.index(&nodes[0]);
    let q = node_map.index(&nodes[1]);

    // B submatrix
    if let Some(pi) = p {
        system.a[(pi, branch)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(qi, branch)] -= 1.0;
    }

    // C submatrix: V(p) - V(q) = rm * I_ctrl
    // branch row: V(p) - V(q) - rm*I_ctrl = 0
    if let Some(pi) = p {
        system.a[(branch, pi)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(branch, qi)] -= 1.0;
    }
    system.a[(branch, ctrl_branch)] -= rm;
}

/// Stamp a Current-Controlled Current Source (CCCS).
///
/// I_out = alpha * I_control
/// nodes: [from, to] (current flows from->to)
///
/// The controlling current is the branch current of another voltage source.
pub fn stamp_cccs(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[String; 2],
    alpha: f64,
    ctrl_branch: usize,
) {
    let from = node_map.index(&nodes[0]);
    let to = node_map.index(&nodes[1]);

    // I_out = alpha * I_ctrl (branch current variable at ctrl_branch)
    // Stamps: row(to) += alpha * I_ctrl, row(from) -= alpha * I_ctrl
    if let Some(ti) = to {
        system.a[(ti, ctrl_branch)] += alpha;
    }
    if let Some(fi) = from {
        system.a[(fi, ctrl_branch)] -= alpha;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::Circuit;
    use approx::assert_relative_eq;

    /// Resistor stamp isolation: R=1k between n1 and ground.
    /// A matrix is 1x1 (one non-ground node, zero vsources).
    /// A[0,0] should equal 0.001.
    #[test]
    fn stamp_resistor_to_ground() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1000.0,
            }],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();

        assert_eq!(system.size(), 1);
        assert_relative_eq!(system.a[(0, 0)], 0.001, epsilon = 1e-15);
    }

    /// Resistor stamp between two non-ground nodes: R=1k between n1 and n2.
    /// A[0,0]=0.001, A[1,1]=0.001, A[0,1]=-0.001, A[1,0]=-0.001.
    #[test]
    fn stamp_resistor_between_nodes() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            }],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();

        let i1 = node_map.index("n1").unwrap();
        let i2 = node_map.index("n2").unwrap();
        assert_relative_eq!(system.a[(i1, i1)], 0.001, epsilon = 1e-15);
        assert_relative_eq!(system.a[(i2, i2)], 0.001, epsilon = 1e-15);
        assert_relative_eq!(system.a[(i1, i2)], -0.001, epsilon = 1e-15);
        assert_relative_eq!(system.a[(i2, i1)], -0.001, epsilon = 1e-15);
    }

    /// VoltageSource stamp: V=10V, n1(+), ground(-). 1 vsource.
    /// A[0,1]=1.0, A[1,0]=1.0, b[1]=10.0.
    #[test]
    fn stamp_voltage_source_to_ground() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            }],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), circuit.count_voltage_sources());

        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();

        // n1 is index 0, vsource branch is index 1
        assert_relative_eq!(system.a[(0, 1)], 1.0, epsilon = 1e-15);
        assert_relative_eq!(system.a[(1, 0)], 1.0, epsilon = 1e-15);
        assert_relative_eq!(system.b[1], 10.0, epsilon = 1e-15);
    }

    /// CurrentSource stamp: I=0.002A from ground to n1.
    /// b[0] = 0.002 (current injected into n1).
    #[test]
    fn stamp_current_source_from_ground() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "n1".into()],
                current: 0.002,
                waveform: None,
            }],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();

        assert_relative_eq!(system.b[0], 0.002, epsilon = 1e-15);
    }

    /// Capacitor companion model: C=100uF, dt=1ms, V_prev=5V.
    /// G_eq = 100e-6 / 1e-3 = 0.1, I_eq = 0.1 * 5.0 = 0.5
    /// Between n1 and ground: A[0,0] = 0.1, b[0] = 0.5.
    #[test]
    fn stamp_capacitor_companion_to_ground() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["n1".into(), "0".into()],
                capacitance: 100e-6,
            }],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        let nodes = ["n1".into(), "0".into()];
        stamp_capacitor_companion(&mut system, &node_map, &nodes, 100e-6, 1e-3, 5.0);

        // G_eq = C/dt = 100e-6/1e-3 = 0.1
        assert_relative_eq!(system.a[(0, 0)], 0.1, epsilon = 1e-12);
        // I_eq = G_eq * V_prev = 0.1 * 5.0 = 0.5
        assert_relative_eq!(system.b[0], 0.5, epsilon = 1e-12);
    }

    /// Capacitor companion between two non-ground nodes: C=1uF, dt=0.1ms, V_prev=3V.
    /// G_eq = 1e-6 / 1e-4 = 0.01, I_eq = 0.01 * 3.0 = 0.03
    #[test]
    fn stamp_capacitor_companion_between_nodes() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    capacitance: 1e-6,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        let nodes = ["n1".into(), "n2".into()];
        stamp_capacitor_companion(&mut system, &node_map, &nodes, 1e-6, 1e-4, 3.0);

        let p = node_map.index("n1").unwrap();
        let q = node_map.index("n2").unwrap();
        let g_eq = 1e-6 / 1e-4; // 0.01
        let i_eq = g_eq * 3.0; // 0.03

        assert_relative_eq!(system.a[(p, p)], g_eq, epsilon = 1e-12);
        assert_relative_eq!(system.a[(q, q)], g_eq, epsilon = 1e-12);
        assert_relative_eq!(system.a[(p, q)], -g_eq, epsilon = 1e-12);
        assert_relative_eq!(system.a[(q, p)], -g_eq, epsilon = 1e-12);
        // I_eq enters p, leaves q
        assert_relative_eq!(system.b[p], i_eq, epsilon = 1e-12);
        assert_relative_eq!(system.b[q], -i_eq, epsilon = 1e-12);
    }

    /// Inductor companion model: L=10mH, dt=1ms, I_prev=0.5A.
    /// G_eq = dt/L = 1e-3/10e-3 = 0.1, I_eq = 0.5
    /// Between n1 and ground: A[0,0] = 0.1, b[0] = -0.5 (current leaves p).
    #[test]
    fn stamp_inductor_companion_to_ground() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Inductor {
                id: "L1".into(),
                nodes: ["n1".into(), "0".into()],
                inductance: 10e-3,
            }],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        let nodes = ["n1".into(), "0".into()];
        stamp_inductor_companion(&mut system, &node_map, &nodes, 10e-3, 1e-3, 0.5);

        // G_eq = dt/L = 1e-3/10e-3 = 0.1
        assert_relative_eq!(system.a[(0, 0)], 0.1, epsilon = 1e-12);
        // I_eq = -I_prev = -0.5 (current source removes current from p)
        assert_relative_eq!(system.b[0], -0.5, epsilon = 1e-12);
    }

    /// Inductor companion between two non-ground nodes: L=1mH, dt=0.1ms, I_prev=0.1A.
    /// G_eq = dt/L = 1e-4/1e-3 = 0.1, I_eq = 0.1
    #[test]
    fn stamp_inductor_companion_between_nodes() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::Inductor {
                    id: "L1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    inductance: 1e-3,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        let nodes = ["n1".into(), "n2".into()];
        stamp_inductor_companion(&mut system, &node_map, &nodes, 1e-3, 1e-4, 0.1);

        let p = node_map.index("n1").unwrap();
        let q = node_map.index("n2").unwrap();
        let g_eq = 1e-4 / 1e-3; // 0.1
        let i_eq = 0.1;

        assert_relative_eq!(system.a[(p, p)], g_eq, epsilon = 1e-12);
        assert_relative_eq!(system.a[(q, q)], g_eq, epsilon = 1e-12);
        assert_relative_eq!(system.a[(p, q)], -g_eq, epsilon = 1e-12);
        assert_relative_eq!(system.a[(q, p)], -g_eq, epsilon = 1e-12);
        assert_relative_eq!(system.b[p], -i_eq, epsilon = 1e-12);
        assert_relative_eq!(system.b[q], i_eq, epsilon = 1e-12);
    }

    /// Invalid resistance (zero) returns SimError::InvalidResistance.
    #[test]
    fn stamp_zero_resistance_returns_error() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 0.0,
            }],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        let result = stamp_circuit(&circuit, &mut system, &node_map, None);
        assert!(result.is_err());
        match result.unwrap_err() {
            SimError::InvalidResistance(id) => assert_eq!(id, "R1"),
            other => panic!("expected InvalidResistance, got: {other}"),
        }
    }

    /// BJT stamp test: NPN in active region (Vb=0.7, Vc=5.0, Ve=0).
    /// Verifies all 9 G-matrix entries and 3 RHS entries against hand-calculated values.
    #[test]
    fn stamp_bjt_companion_matrix_entries() {
        use sindr_devices::bjt::{BjtKind, BjtParams};
        use sindr_devices::diode::V_T;

        // Circuit: BJT (base=n1, collector=n2, emitter=GND) + Vcc voltage source
        // We need nodes n1 (base) and n2 (collector); emitter is ground.
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["n2".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "Rb".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 100000.0,
                },
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["n1".into(), "n2".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                },
            ],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n_nodes = node_map.num_nodes();
        let n_vsources = circuit.count_voltage_sources();
        let size = n_nodes + n_vsources;

        let mut system = MnaSystem::new(n_nodes, n_vsources);

        // Set up v_prev: Vb=0.7, Vc=5.0, Ve=0 (ground)
        let mut v_prev = nalgebra::DVector::zeros(size);
        let bi = node_map.index("n1").unwrap();
        let ci = node_map.index("n2").unwrap();
        v_prev[bi] = 0.7;
        v_prev[ci] = 5.0;

        // Stamp only the BJT
        let params = BjtParams::new(100.0);
        let nodes: [String; 3] = ["n1".into(), "n2".into(), "0".into()];
        stamp_bjt_companion(
            &mut system,
            &node_map,
            &nodes,
            &v_prev,
            &params,
            BjtKind::Npn,
        );

        // Hand-calculate expected companion values at Vbe=0.7, Vbc=0.7-5.0=-4.3
        let vbe = 0.7;
        let vbc = -4.3;
        let nf_vt = 1.0 * V_T;
        let nr_vt = 1.0 * V_T;
        let exp_be = (vbe / nf_vt).exp();
        let exp_bc = (vbc / nr_vt).exp();
        let g_be = (1e-14 / nf_vt) * exp_be;
        let g_bc = (1e-14 / nr_vt) * exp_bc;
        let bf = 100.0;
        let br = 1.0;
        let alpha_r = br / (br + 1.0);

        // G matrix: Row B (base)
        assert_relative_eq!(system.a[(bi, bi)], g_be / bf + g_bc / br, epsilon = 1e-15);
        assert_relative_eq!(system.a[(bi, ci)], -g_bc / br, epsilon = 1e-15);
        // G[B,E] is ground, skipped

        // G matrix: Row C (collector) -- CORRECTED signs per docs/bjt-ebers-moll.md
        assert_relative_eq!(system.a[(ci, bi)], g_be - g_bc / alpha_r, epsilon = 1e-15);
        assert_relative_eq!(system.a[(ci, ci)], g_bc / alpha_r, epsilon = 1e-15);
        // G[C,E] is ground, skipped

        // Diagonal entries should be positive (self-conductance)
        assert!(system.a[(bi, bi)] > 0.0, "G[B,B] should be positive");
        // G[C,C] is positive (g_bc/alpha_r term, corrected sign)
        assert!(system.a[(ci, ci)] >= 0.0, "G[C,C] should be non-negative");

        // Cross-coupling: G[C,B] should contain the g_be term (large in active region)
        assert!(
            system.a[(ci, bi)] > 0.1,
            "G[C,B] should be substantial in active region, got {}",
            system.a[(ci, bi)]
        );

        // RHS should be non-zero (companion current sources present)
        assert!(system.b[bi].abs() > 1e-10, "RHS[B] should be non-zero");
        assert!(system.b[ci].abs() > 1e-10, "RHS[C] should be non-zero");

        // Verify RHS values against hand-calculated companion currents
        let i_f = 1e-14 * (exp_be - 1.0);
        let i_r = 1e-14 * (exp_bc - 1.0);
        let ic = i_f - i_r / alpha_r;
        let ib = i_f / bf + i_r / br;

        let ib_eq = ib - (g_be / bf) * vbe - (g_bc / br) * vbc;
        let ic_eq = ic - g_be * vbe + (g_bc / alpha_r) * vbc;

        assert_relative_eq!(system.b[bi], -ib_eq, epsilon = 1e-15);
        assert_relative_eq!(system.b[ci], -ic_eq, epsilon = 1e-15);
        // Emitter is ground, so no RHS entry for it (skipped)
    }

    /// KCL column-sum test: each column of the 3x3 BJT Jacobian submatrix
    /// must sum to zero (current into each node sums to zero).
    #[test]
    fn test_bjt_stamp_kcl_column_sum() {
        use sindr_devices::bjt::{BjtKind, BjtParams};

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "Vbb".into(),
                    nodes: ["n_b".into(), "0".into()],
                    voltage: 0.7,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "Rc".into(),
                    nodes: ["n_c".into(), "0".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["n_b".into(), "n_c".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                },
            ],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n_nodes = node_map.num_nodes();
        let n_vsources = circuit.count_voltage_sources();
        let size = n_nodes + n_vsources;

        let mut system = MnaSystem::new(n_nodes, n_vsources);

        let mut v_prev = nalgebra::DVector::zeros(size);
        let bi = node_map.index("n_b").unwrap();
        let ci = node_map.index("n_c").unwrap();
        v_prev[bi] = 0.7;
        v_prev[ci] = 0.0;

        let params = BjtParams::new(100.0);
        let nodes: [String; 3] = ["n_b".into(), "n_c".into(), "0".into()];
        stamp_bjt_companion(
            &mut system,
            &node_map,
            &nodes,
            &v_prev,
            &params,
            BjtKind::Npn,
        );

        let params2 = BjtParams::new(100.0);
        let bf = params2.bf;
        let br = params2.br;
        let alpha_f = bf / (bf + 1.0);
        let alpha_r = br / (br + 1.0);

        let vbe = 0.7;
        let vbc = 0.7 - 0.0;
        let comp = sindr_devices::bjt::bjt_companion(vbe, vbc, &params2);
        let g_be = comp.g_be;
        let g_bc = comp.g_bc;

        assert!(
            g_bc > 1e-6,
            "g_bc={} should be significant in saturation",
            g_bc
        );

        let col_b = (g_be / bf + g_bc / br) + (g_be - g_bc / alpha_r) + (-g_be / alpha_f + g_bc);
        assert!(
            col_b.abs() < 1e-12,
            "Column B should sum to 0 (KCL), got {}",
            col_b
        );

        let col_c = (-g_bc / br) + (g_bc / alpha_r) + (-g_bc);
        assert!(
            col_c.abs() < 1e-12,
            "Column C should sum to 0 (KCL), got {}",
            col_c
        );

        let col_e = (-g_be / bf) + (-g_be) + (g_be / alpha_f);
        assert!(
            col_e.abs() < 1e-12,
            "Column E should sum to 0 (KCL), got {}",
            col_e
        );

        assert_relative_eq!(system.a[(bi, bi)], g_be / bf + g_bc / br, epsilon = 1e-12);
        assert_relative_eq!(system.a[(bi, ci)], -g_bc / br, epsilon = 1e-12);
        assert_relative_eq!(system.a[(ci, bi)], g_be - g_bc / alpha_r, epsilon = 1e-12);
        assert_relative_eq!(system.a[(ci, ci)], g_bc / alpha_r, epsilon = 1e-12);
    }

    /// Stamp entries for NPN in active mode (Vbe=0.7V, Vbc=-5.0V, BF=100, BR=1).
    #[test]
    fn test_bjt_stamp_entries_npn_active() {
        use sindr_devices::bjt::{BjtKind, BjtParams};

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["n_vcc".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "Rb".into(),
                    nodes: ["n_b".into(), "0".into()],
                    resistance: 100_000.0,
                },
                CircuitElement::Resistor {
                    id: "Re".into(),
                    nodes: ["n_e".into(), "0".into()],
                    resistance: 100.0,
                },
                CircuitElement::Resistor {
                    id: "Rc".into(),
                    nodes: ["n_c".into(), "0".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["n_b".into(), "n_c".into(), "n_e".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                },
            ],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n_nodes = node_map.num_nodes();
        let n_vsources = circuit.count_voltage_sources();
        let size = n_nodes + n_vsources;

        let mut system = MnaSystem::new(n_nodes, n_vsources);

        let bi = node_map.index("n_b").unwrap();
        let ci = node_map.index("n_c").unwrap();
        let ei = node_map.index("n_e").unwrap();

        let mut v_prev = nalgebra::DVector::zeros(size);
        v_prev[bi] = 0.7;
        v_prev[ci] = 5.0;
        v_prev[ei] = 0.0;

        let params = BjtParams::new(100.0);
        let nodes: [String; 3] = ["n_b".into(), "n_c".into(), "n_e".into()];
        stamp_bjt_companion(
            &mut system,
            &node_map,
            &nodes,
            &v_prev,
            &params,
            BjtKind::Npn,
        );

        let vbe = 0.7;
        let vbc = 0.7 - 5.0;
        let comp = sindr_devices::bjt::bjt_companion(vbe, vbc, &params);
        let g_be = comp.g_be;
        let g_bc = comp.g_bc;
        let bf = 100.0;
        let br = 1.0;
        let alpha_f = bf / (bf + 1.0);
        let alpha_r = br / (br + 1.0);

        assert_relative_eq!(system.a[(bi, bi)], g_be / bf + g_bc / br, epsilon = 1e-12);
        assert_relative_eq!(system.a[(bi, ci)], -g_bc / br, epsilon = 1e-12);
        assert_relative_eq!(system.a[(bi, ei)], -g_be / bf, epsilon = 1e-12);

        assert_relative_eq!(system.a[(ci, bi)], g_be + g_bc / alpha_r, epsilon = 1e-6);

        assert_relative_eq!(system.a[(ci, ei)], -g_be, epsilon = 1e-12);

        assert_relative_eq!(system.a[(ei, ei)], g_be / alpha_f, epsilon = 1e-12);
    }

    /// Stamp entries for NPN in saturation (Vbe=0.7V, Vbc=0.3V).
    #[test]
    fn test_bjt_stamp_entries_npn_saturation() {
        use sindr_devices::bjt::{BjtKind, BjtParams};

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["n_vcc".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "Rb".into(),
                    nodes: ["n_b".into(), "0".into()],
                    resistance: 100_000.0,
                },
                CircuitElement::Resistor {
                    id: "Re".into(),
                    nodes: ["n_e".into(), "0".into()],
                    resistance: 100.0,
                },
                CircuitElement::Resistor {
                    id: "Rc".into(),
                    nodes: ["n_c".into(), "0".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["n_b".into(), "n_c".into(), "n_e".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                },
            ],
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n_nodes = node_map.num_nodes();
        let n_vsources = circuit.count_voltage_sources();
        let size = n_nodes + n_vsources;

        let mut system = MnaSystem::new(n_nodes, n_vsources);

        let bi = node_map.index("n_b").unwrap();
        let ci = node_map.index("n_c").unwrap();
        let ei = node_map.index("n_e").unwrap();

        let mut v_prev = nalgebra::DVector::zeros(size);
        v_prev[bi] = 0.7;
        v_prev[ci] = 0.0;
        v_prev[ei] = 0.0;

        let params = BjtParams::new(100.0);
        let nodes: [String; 3] = ["n_b".into(), "n_c".into(), "n_e".into()];
        stamp_bjt_companion(
            &mut system,
            &node_map,
            &nodes,
            &v_prev,
            &params,
            BjtKind::Npn,
        );

        let vbe = 0.7;
        let vbc = 0.7;
        let comp = sindr_devices::bjt::bjt_companion(vbe, vbc, &params);
        let g_be = comp.g_be;
        let g_bc = comp.g_bc;
        let bf = 100.0;
        let br = 1.0;
        let alpha_f = bf / (bf + 1.0);
        let alpha_r = br / (br + 1.0);

        assert!(
            g_bc > 1e-3,
            "g_bc={} should be significant in saturation",
            g_bc
        );

        assert_relative_eq!(system.a[(bi, bi)], g_be / bf + g_bc / br, epsilon = 0.01);
        assert_relative_eq!(system.a[(bi, ci)], -g_bc / br, epsilon = 0.01);
        assert_relative_eq!(system.a[(bi, ei)], -g_be / bf, epsilon = 0.01);

        assert_relative_eq!(system.a[(ci, bi)], g_be - g_bc / alpha_r, epsilon = 0.01);
        assert_relative_eq!(system.a[(ci, ci)], g_bc / alpha_r, epsilon = 0.01);
        assert_relative_eq!(system.a[(ci, ei)], -g_be, epsilon = 0.01);

        assert_relative_eq!(system.a[(ei, bi)], -g_be / alpha_f + g_bc, epsilon = 0.01);
        assert_relative_eq!(system.a[(ei, ci)], -g_bc, epsilon = 0.01);
        assert_relative_eq!(system.a[(ei, ei)], g_be / alpha_f, epsilon = 0.01);
    }

    /// Transformer DC solve: 10V source on primary → DC solve completes without panic.
    /// Primary and secondary are both near-short-circuits (1e-9 Ohm) in DC.
    #[test]
    fn transformer_dc_solve() {
        use crate::solve_circuit;

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["p1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["p1".into(), "n1".into()],
                    resistance: 100.0,
                },
                CircuitElement::Transformer {
                    id: "T1".into(),
                    nodes: ["n1".into(), "0".into(), "n2".into(), "0".into()],
                    l1: 1e-3,
                    l2: 4e-3,
                    k: 0.999,
                },
                CircuitElement::Resistor {
                    id: "R2".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Transformer DC circuit solve failed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        // n1 should be very close to 10V (primary is near-short-circuit)
        let v_n1 = result.node_voltages["n1"];
        assert!(v_n1 > 9.0, "Primary voltage should be ~10V, got {}", v_n1);
    }

    /// Varactor DC stamp: treated as 1e12 Ω shunt (open circuit).
    /// Simple circuit: V1=5V, varactor between n1 and ground.
    /// DC solve should return result without panic.
    #[test]
    fn varactor_dc_circuit_solves() {
        use crate::solve_circuit;

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
                CircuitElement::Varactor {
                    id: "CV1".into(),
                    nodes: ["n2".into(), "0".into()],
                    params: sindr_devices::varactor::VaractorParams::default(),
                },
            ],
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Varactor DC circuit solve failed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        // n1 = 5V (voltage source), n2 should be very close to 5V (varactor = open circuit = 1e12 Ohm)
        let v_n1 = result.node_voltages["n1"];
        assert_relative_eq!(v_n1, 5.0, epsilon = 1e-6);
        // Varactor result should be present
        let cv1 = result
            .component_results
            .iter()
            .find(|c| c.id == "CV1")
            .unwrap();
        // Voltage across varactor is approximately 5V (n2 ≈ 5V due to 1e12 shunt)
        assert!(
            cv1.voltage_across > 4.9,
            "Varactor voltage should be ~5V, got {}",
            cv1.voltage_across
        );
    }

    /// IGBT cutoff test: Vge=0 (gate at 0V, below threshold) → ids=0 in result.
    #[test]
    fn igbt_cutoff_circuit() {
        use crate::solve_circuit;

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                // Gate at 0V (tied to ground via large resistor effectively at 0V)
                CircuitElement::VoltageSource {
                    id: "Vg".into(),
                    nodes: ["gate".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                },
                // Collector supply: 12V
                CircuitElement::VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["coll".into(), "0".into()],
                    voltage: 12.0,
                    waveform: None,
                },
                // Load resistor
                CircuitElement::Resistor {
                    id: "Rload".into(),
                    nodes: ["coll".into(), "0".into()],
                    resistance: 100.0,
                },
                CircuitElement::Igbt {
                    id: "T1".into(),
                    nodes: ["gate".into(), "coll".into(), "0".into()],
                    params: sindr_devices::igbt::IgbtParams::default(), // vth=5.0
                },
            ],
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "IGBT cutoff circuit failed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        let t1 = result
            .component_results
            .iter()
            .find(|c| c.id == "T1")
            .unwrap();
        // With Vge=0 < Vth=5V, IGBT is in cutoff: ids = 0
        assert_relative_eq!(t1.current_through, 0.0, epsilon = 1e-9);
    }

    /// IGBT active test: Vge=10V, Vce=12V → non-zero collector current.
    #[test]
    fn igbt_active_circuit() {
        use crate::solve_circuit;

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                // Gate at 10V (above threshold of 5V)
                CircuitElement::VoltageSource {
                    id: "Vg".into(),
                    nodes: ["gate".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                // Collector supply
                CircuitElement::VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["n_vcc".into(), "0".into()],
                    voltage: 12.0,
                    waveform: None,
                },
                // Load resistor between supply and collector
                CircuitElement::Resistor {
                    id: "Rload".into(),
                    nodes: ["n_vcc".into(), "coll".into()],
                    resistance: 100.0,
                },
                CircuitElement::Igbt {
                    id: "T1".into(),
                    nodes: ["gate".into(), "coll".into(), "0".into()],
                    params: sindr_devices::igbt::IgbtParams::default(), // vth=5, k=5
                },
            ],
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "IGBT active circuit failed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        let t1 = result
            .component_results
            .iter()
            .find(|c| c.id == "T1")
            .unwrap();
        // With Vge=10 > Vth=5, IGBT should conduct
        assert!(
            t1.current_through > 0.0,
            "IGBT should have non-zero collector current, got {}",
            t1.current_through
        );
    }
}
