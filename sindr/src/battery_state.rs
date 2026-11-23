//! The solver side of [`CircuitElement::Battery`]: its Norton equivalent at
//! DC and per transient step, and the state it carries between steps.
//!
//! With `i` the discharge current (out of the positive terminal) and `v`
//! the terminal voltage, the cell is `v = OCV(SoC) − R0·i − v1`. Every
//! form here is written as the current from `nodes[0]` to `nodes[1]`
//! inside the cell, `g·v + i_hist`, which is `−i`.

use crate::node_map::FastMap;
use std::collections::HashMap;

use sindr_devices::battery::{dc_resistance, open_circuit_voltage, BatteryParams};

use crate::circuit::{Circuit, CircuitElement};
use crate::integration::{History, Integrator};
use crate::results::ComponentResult;

/// Key suffix, in the transient's per-element history map, of a battery's
/// RC-pair voltage (V) and its capacitor current (A).
pub(crate) const RC: &str = "rc";
/// Key suffix of a battery's state of charge: `x1` the state of charge,
/// `y1` the discharge current (A) at the end of the last step, NaN before
/// the first.
pub(crate) const SOC: &str = "soc";

/// Norton form at DC, the RC pair settled: OCV behind `R0 + R1`.
pub(crate) fn dc_norton(params: &BatteryParams, soc: f64) -> (f64, f64) {
    let r = dc_resistance(params);
    (1.0 / r, -open_circuit_voltage(soc, params) / r)
}

/// Norton form over one step, from the state of charge at its start and
/// the RC pair's history.
///
/// The pair carries `i = v1/R1 + i_C` with the capacitor's companion
/// `i_C = g_C·v1 + i_C,hist`, so `v1 = α·i + β` with `α = 1/(1/R1 + g_C)`
/// and `β = −α·i_C,hist`. Then `v = OCV − β − (R0 + α)·i`.
pub(crate) fn step_norton(
    params: &BatteryParams,
    soc: f64,
    rc: &History,
    integ: &Integrator,
) -> (f64, f64) {
    let (alpha, beta) = rc_affine(params, rc, integ);
    let r = params.r_series + alpha;
    (1.0 / r, -(open_circuit_voltage(soc, params) - beta) / r)
}

/// `(α, β)` with the RC pair's voltage at the end of the step `α·i + β`.
fn rc_affine(params: &BatteryParams, rc: &History, integ: &Integrator) -> (f64, f64) {
    match params.rc {
        None => (0.0, 0.0),
        Some(pair) => {
            let (g_c, i_hist) = integ.capacitor(pair.c, rc);
            let alpha = 1.0 / (1.0 / pair.r + g_c);
            (alpha, -alpha * i_hist)
        }
    }
}

fn voltage(node_voltages: &HashMap<String, f64>, node: &str) -> f64 {
    node_voltages.get(node).copied().unwrap_or(0.0)
}

/// The starting histories for each battery: its state of charge, and its
/// RC pair at rest, or settled at the operating point's current when
/// `settled` (an operating-point start, with terminal voltages `start`).
pub(crate) fn initial_histories(
    circuit: &Circuit,
    start: &HashMap<String, f64>,
    settled: bool,
    histories: &mut FastMap<History>,
) {
    for component in &circuit.components {
        let CircuitElement::Battery {
            id,
            nodes,
            params,
            state_of_charge,
        } = component
        else {
            continue;
        };
        histories.insert(
            format!("{id}-{SOC}"),
            History {
                x1: *state_of_charge,
                x2: *state_of_charge,
                y1: f64::NAN,
            },
        );
        let v1 = match (params.rc, settled) {
            (Some(pair), true) => {
                let v = voltage(start, &nodes[0]) - voltage(start, &nodes[1]);
                let (g, i_hist) = dc_norton(params, *state_of_charge);
                let discharge = -(g * v + i_hist);
                pair.r * discharge
            }
            _ => 0.0,
        };
        histories.insert(format!("{id}-{RC}"), History::at(v1));
    }
}

/// After an accepted step of length `dt` ending at `node_voltages`: report
/// each battery's terminal result, and move its RC pair and state of
/// charge on. Returns each battery's new state of charge.
pub(crate) fn finish_step(
    circuit: &Circuit,
    node_voltages: &HashMap<String, f64>,
    integ: &Integrator,
    dt: f64,
    histories: &mut FastMap<History>,
    results: &mut Vec<ComponentResult>,
) -> HashMap<String, f64> {
    let mut states = HashMap::new();
    for component in &circuit.components {
        let CircuitElement::Battery {
            id, nodes, params, ..
        } = component
        else {
            continue;
        };
        let soc_key = format!("{id}-{SOC}");
        let rc_key = format!("{id}-{RC}");
        let soc = histories.get(&soc_key).copied().unwrap_or_default();
        let rc = histories.get(&rc_key).copied().unwrap_or_default();

        let v = voltage(node_voltages, &nodes[0]) - voltage(node_voltages, &nodes[1]);
        let (g, i_hist) = step_norton(params, soc.x1, &rc, integ);
        let through = g * v + i_hist;
        let discharge = -through;
        results.push(ComponentResult {
            id: id.clone(),
            voltage_across: v,
            current_through: through,
            power: v * through,
        });

        if let Some(pair) = params.rc {
            let (alpha, beta) = rc_affine(params, &rc, integ);
            let v1 = alpha * discharge + beta;
            histories.insert(rc_key, rc.push(v1, discharge - v1 / pair.r));
        }
        // Trapezoidal charge count; the first step has no earlier current.
        let previous = if soc.y1.is_nan() { discharge } else { soc.y1 };
        let amp_hours = 0.5 * (previous + discharge) * dt / 3600.0;
        let next = sindr_devices::battery::discharged(soc.x1, amp_hours, params);
        histories.insert(soc_key, soc.push(next, discharge));
        states.insert(id.clone(), next);
    }
    states
}
