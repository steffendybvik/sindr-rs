//! Pre-solve circuit validation.
//!
//! Catches obvious topology mistakes (no ground, disconnected subgraphs,
//! floating nodes) and device values the models can't use (a junction
//! temperature at or below 0 K, a BJT `bf` at or below 0) before they reach
//! the solver, where the failure mode would be a less helpful "singular
//! matrix" or "invalid solution" error. Run automatically
//! by [`solve_circuit`](crate::solve_circuit); also exposed standalone via
//! [`validate_circuit`].

use std::collections::{HashMap, HashSet, VecDeque};

use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;

/// Most points one AC, DC or temperature sweep may have. Each point is a
/// full solve, so this is far past anything useful; it exists so a huge
/// count returns an error instead of failing to allocate.
pub(crate) const MAX_SWEEP_POINTS: usize = 10_000_000;

/// Rejects a sweep of more than [`MAX_SWEEP_POINTS`] points.
pub(crate) fn check_sweep_points(what: &str, name: &str, n: usize) -> Result<(), SimError> {
    if n > MAX_SWEEP_POINTS {
        return Err(SimError::InvalidComponent(format!(
            "{what}: {name} = {n} is more than the {MAX_SWEEP_POINTS} allowed"
        )));
    }
    Ok(())
}

/// Validate a circuit before solving.
///
/// Checks are run in order of most-helpful-first:
/// 1. Ground node exists (at least one component touches it)
/// 2. All nodes reachable from ground (BFS connectivity)
/// 3. No floating nodes (every non-ground node has degree >= 2)
/// 4. Unique component ids
/// 5. Values the models can use: resistances, inductances and every
///    `temperature` (K) positive and finite, capacitances zero or more,
///    source values and controlled-source gains finite, a BJT's `bf`
///    positive and finite, and the device-model fields each check names
///
/// # Errors
///
/// Returns [`SimError::NoGround`] if no component terminal matches
/// `circuit.ground_node`.
///
/// Returns [`SimError::DisconnectedNodes`] listing nodes unreachable from
/// ground.
///
/// Returns [`SimError::FloatingNode`] naming the first node with fewer than
/// 2 connections.
///
/// Returns [`SimError::InvalidResistance`] for a resistor whose resistance
/// is zero, negative or not finite.
///
/// Returns [`SimError::InvalidComponent`] for a repeated id, or naming the
/// component, field and value for any other value out of range.
pub fn validate_circuit(circuit: &Circuit) -> Result<(), SimError> {
    let flat = circuit.flattened()?;
    let circuit: &Circuit = &flat;
    // ------------------------------------------------------------------
    // Check 1: Ground node exists
    // ------------------------------------------------------------------
    let has_ground = circuit
        .components
        .iter()
        .any(|c| c.nodes().iter().any(|n| n == &circuit.ground_node));

    if !has_ground {
        return Err(SimError::NoGround);
    }

    // ------------------------------------------------------------------
    // Check 2: All nodes reachable from ground — BFS connectivity
    // ------------------------------------------------------------------
    let mut adj: HashMap<&str, Vec<&str>> = HashMap::new();
    for comp in &circuit.components {
        let nodes: Vec<&str> = comp.all_nodes().into_iter().map(|s| s.as_str()).collect();
        for i in 0..nodes.len() {
            for j in (i + 1)..nodes.len() {
                adj.entry(nodes[i]).or_default().push(nodes[j]);
                adj.entry(nodes[j]).or_default().push(nodes[i]);
            }
        }
    }

    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    visited.insert(circuit.ground_node.as_str());
    queue.push_back(circuit.ground_node.as_str());
    while let Some(node) = queue.pop_front() {
        if let Some(neighbors) = adj.get(node) {
            for &neighbor in neighbors {
                if visited.insert(neighbor) {
                    queue.push_back(neighbor);
                }
            }
        }
    }

    let all_nodes: HashSet<&str> = adj.keys().copied().collect();
    let mut disconnected: Vec<String> = all_nodes
        .difference(&visited)
        .map(|s| s.to_string())
        .collect();
    disconnected.sort(); // deterministic ordering for tests
    if !disconnected.is_empty() {
        return Err(SimError::DisconnectedNodes(disconnected));
    }

    // ------------------------------------------------------------------
    // Check 3: Floating nodes — degree < 2
    // ------------------------------------------------------------------
    // Count how many component terminals touch each non-ground node.
    let mut degree: HashMap<&str, usize> = HashMap::new();
    for comp in &circuit.components {
        for node in comp.all_nodes() {
            if *node != circuit.ground_node {
                *degree.entry(node.as_str()).or_insert(0) += 1;
            }
        }
    }
    for (node, count) in &degree {
        if *count < 2 {
            return Err(SimError::FloatingNode(node.to_string()));
        }
    }

    // ------------------------------------------------------------------
    // Check 4: Unique ids. Results, controlled sources and sweeps find a
    // component by id, so a repeated one would silently pick the first.
    // ------------------------------------------------------------------
    let mut ids = HashSet::new();
    for comp in &circuit.components {
        if !ids.insert(comp.id()) {
            return Err(SimError::InvalidComponent(format!(
                "`{}`: more than one component has this id",
                comp.id()
            )));
        }
    }

    // ------------------------------------------------------------------
    // Check 5: Device values the models can't take
    // ------------------------------------------------------------------
    for comp in &circuit.components {
        check_values(comp)?;
    }

    Ok(())
}

/// Rejects a device temperature, BJT gain or model parameter the models can't use, naming the
/// component, field and value. Left alone, a 0 K junction divides by zero and
/// fails much later as a bare [`SimError::InvalidSolution`], and β ≤ 0 as
/// that or a convergence failure.
fn check_values(comp: &CircuitElement) -> Result<(), SimError> {
    let positive = |field: &str, value: f64, unit: &str| {
        if value.is_finite() && value > 0.0 {
            Ok(())
        } else {
            Err(SimError::InvalidComponent(format!(
                "`{}`: {field} {value}{unit} must be a positive, finite number",
                comp.id()
            )))
        }
    };
    let non_negative = |field: &str, value: f64, unit: &str| {
        if value.is_finite() && value >= 0.0 {
            Ok(())
        } else {
            Err(SimError::InvalidComponent(format!(
                "`{}`: {field} {value}{unit} must be zero or a positive, finite number",
                comp.id()
            )))
        }
    };
    let finite = |field: &str, value: f64, unit: &str| {
        if value.is_finite() {
            Ok(())
        } else {
            Err(SimError::InvalidComponent(format!(
                "`{}`: {field} {value}{unit} must be a finite number",
                comp.id()
            )))
        }
    };
    // A dimensionless coefficient in `[0, max]`, as SPICE bounds M and FC:
    // at 1 the junction charge divides by zero.
    let within = |field: &str, value: f64, max: f64| {
        if (0.0..=max).contains(&value) {
            Ok(())
        } else {
            Err(SimError::InvalidComponent(format!(
                "`{}`: {field} {value} must be between 0 and {max}",
                comp.id()
            )))
        }
    };
    match comp {
        CircuitElement::Diode {
            temperature,
            model: Some(model),
            ..
        } => {
            positive("temperature", *temperature, " K")?;
            positive("model.is", model.is, " A")?;
            positive("model.n", model.n, "")?;
            non_negative("model.rs", model.rs, " Ω")?;
            non_negative("model.cjo", model.cjo, " F")?;
            positive("model.vj", model.vj, " V")?;
            within("model.m", model.m, 0.9)?;
            within("model.fc", model.fc, 0.95)?;
            non_negative("model.tt", model.tt, " s")?;
            if let Some(bv) = model.bv {
                positive("model.bv", bv, " V")?;
                positive("model.ibv", model.ibv, " A")?;
            }
            non_negative("model.kf", model.kf, "")?;
            positive("model.af", model.af, "")?;
            Ok(())
        }
        CircuitElement::Diode { temperature, .. }
        | CircuitElement::Led { temperature, .. }
        | CircuitElement::ZenerDiode { temperature, .. }
        | CircuitElement::SchottkyDiode { temperature, .. }
        | CircuitElement::Thermistor { temperature, .. }
        | CircuitElement::Photodiode { temperature, .. } => {
            positive("temperature", *temperature, " K")
        }
        CircuitElement::Bjt {
            bf,
            temperature,
            model,
            ..
        } => {
            positive("temperature", *temperature, " K")?;
            positive("bf", *bf, "")?;
            if let Some(model) = model {
                positive("model.is", model.is, " A")?;
                positive("model.br", model.br, "")?;
                positive("model.nf", model.nf, "")?;
                positive("model.nr", model.nr, "")?;
                non_negative("model.vaf", model.vaf, " V")?;
                non_negative("model.kf", model.kf, "")?;
                positive("model.af", model.af, "")?;
            }
            Ok(())
        }
        CircuitElement::OpAmp {
            v_pos,
            v_neg,
            model: Some(model),
            ..
        } => {
            finite("v_pos", *v_pos, " V")?;
            finite("v_neg", *v_neg, " V")?;
            if v_pos <= v_neg {
                return Err(SimError::InvalidComponent(format!(
                    "`{}`: v_pos {v_pos} V must be above v_neg {v_neg} V",
                    comp.id()
                )));
            }
            positive("model.gain", model.gain, "")?;
            if let Some(gbw) = model.gbw {
                positive("model.gbw", gbw, " Hz")?;
            }
            if let Some(sr) = model.slew_rate {
                positive("model.slew_rate", sr, " V/s")?;
                if model.gbw.is_none() {
                    return Err(SimError::InvalidComponent(format!(
                        "`{}`: model.slew_rate needs model.gbw: the slew limit acts on the gain stage's pole",
                        comp.id()
                    )));
                }
            }
            non_negative("model.r_out", model.r_out, " Ω")?;
            finite("model.offset_voltage", model.offset_voltage, " V")?;
            finite("model.bias_current", model.bias_current, " A")
        }
        CircuitElement::VoltageRegulator { voltage, model, .. } => {
            positive("voltage", *voltage, " V")?;
            if let Some(m) = model {
                non_negative("model.dropout", m.dropout, " V")?;
                non_negative("model.dropout_resistance", m.dropout_resistance, " Ω")?;
                non_negative("model.quiescent_current", m.quiescent_current, " A")?;
                non_negative("model.line_regulation", m.line_regulation, " V/V")?;
                non_negative("model.output_resistance", m.output_resistance, " Ω")?;
                if let Some(limit) = m.current_limit {
                    positive("model.current_limit", limit, " A")?;
                }
            }
            Ok(())
        }
        // Only the noise fields: the rest of a MOSFET's parameters have
        // never been checked here.
        CircuitElement::Mosfet { params, .. } => {
            non_negative("params.kf", params.kf, "")?;
            positive("params.af", params.af, "")
        }
        CircuitElement::Resistor { id, resistance, .. } => {
            if resistance.is_finite() && *resistance > 0.0 {
                Ok(())
            } else {
                Err(SimError::InvalidResistance(id.clone()))
            }
        }
        CircuitElement::VoltageSource { voltage, .. } => finite("voltage", *voltage, " V"),
        CircuitElement::CurrentSource { current, .. } => finite("current", *current, " A"),
        // A zero capacitor is an open circuit, as in SPICE; a zero inductor
        // would divide by zero in its companion model.
        CircuitElement::Capacitor { capacitance, .. } => {
            non_negative("capacitance", *capacitance, " F")
        }
        CircuitElement::Inductor { inductance, .. } => positive("inductance", *inductance, " H"),
        CircuitElement::Vcvs { gain, .. } => finite("gain", *gain, ""),
        CircuitElement::Vccs { gm, .. } => finite("gm", *gm, " S"),
        CircuitElement::Ccvs { rm, .. } => finite("rm", *rm, " Ω"),
        CircuitElement::Cccs { alpha, .. } => finite("alpha", *alpha, ""),
        CircuitElement::Relay {
            coil_resistance,
            pickup_voltage,
            inductance,
            ..
        } => {
            positive("coil_resistance", *coil_resistance, " Ω")?;
            positive("pickup_voltage", *pickup_voltage, " V")?;
            non_negative("inductance", *inductance, " H")
        }
        // Light level, wiper position and coupling are clamped into range
        // by the models, so only a non-finite value is refused.
        CircuitElement::Photoresistor { light_level, .. } => {
            finite("light_level", *light_level, "")
        }
        CircuitElement::Potentiometer {
            resistance,
            position,
            ..
        } => {
            positive("resistance", *resistance, " Ω")?;
            finite("position", *position, "")
        }
        CircuitElement::Transformer { l1, l2, k, .. } => {
            positive("l1", *l1, " H")?;
            positive("l2", *l2, " H")?;
            finite("k", *k, "")
        }
        CircuitElement::Fuse { rating, .. } => positive("rating", *rating, " A"),
        CircuitElement::Comparator { v_pos, v_neg, .. }
        | CircuitElement::OpAmp { v_pos, v_neg, .. } => {
            finite("v_pos", *v_pos, " V")?;
            finite("v_neg", *v_neg, " V")
        }
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::{Circuit, CircuitElement};
    use crate::error::SimError;
    use crate::solve_circuit;

    #[test]
    fn bad_model_parameters_name_the_field() {
        let with = |component: CircuitElement| Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["a".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["a".into(), "b".into()],
                    resistance: 1e3,
                },
                component,
            ],
            ..Default::default()
        };
        let diode = |model: crate::DiodeModel| {
            with(CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["b".into(), "0".into()],
                temperature: 300.15,
                model: Some(model),
            })
        };
        let bjt = |model: crate::BjtModel| {
            with(CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "a".into(), "0".into()],
                kind: crate::BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: Some(model),
            })
        };
        let d = crate::DiodeModel::default();
        let q = crate::BjtModel::default();
        let bad = [
            (diode(crate::DiodeModel { n: 0.0, ..d }), "model.n"),
            (diode(crate::DiodeModel { is: -1.0, ..d }), "model.is"),
            (diode(crate::DiodeModel { rs: -10.0, ..d }), "model.rs"),
            (diode(crate::DiodeModel { rs: f64::NAN, ..d }), "model.rs"),
            (diode(crate::DiodeModel { cjo: -1e-12, ..d }), "model.cjo"),
            (diode(crate::DiodeModel { vj: 0.0, ..d }), "model.vj"),
            (diode(crate::DiodeModel { m: 1.0, ..d }), "model.m"),
            (diode(crate::DiodeModel { fc: 0.99, ..d }), "model.fc"),
            (diode(crate::DiodeModel { tt: -1e-9, ..d }), "model.tt"),
            (diode(crate::DiodeModel { bv: Some(0.0), ..d }), "model.bv"),
            (
                diode(crate::DiodeModel {
                    bv: Some(100.0),
                    ibv: 0.0,
                    ..d
                }),
                "model.ibv",
            ),
            (bjt(crate::BjtModel { br: 0.0, ..q }), "model.br"),
            (bjt(crate::BjtModel { nf: -1.0, ..q }), "model.nf"),
            (bjt(crate::BjtModel { nr: 0.0, ..q }), "model.nr"),
            (bjt(crate::BjtModel { vaf: -5.0, ..q }), "model.vaf"),
        ];
        for (circuit, field) in bad {
            match validate_circuit(&circuit) {
                Err(SimError::InvalidComponent(msg)) => assert!(msg.contains(field), "{msg}"),
                other => panic!("{field}: {other:?}"),
            }
        }
        assert!(validate_circuit(&diode(crate::DiodeModel { rs: 1e9, ..d })).is_ok());
        assert!(validate_circuit(&bjt(crate::BjtModel { vaf: 0.0, ..q })).is_ok());
    }

    #[test]
    fn bad_device_temperature_or_gain_names_the_field() {
        let diode = |temperature: f64| Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["a".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["a".into(), "b".into()],
                    resistance: 1e3,
                },
                CircuitElement::Diode {
                    id: "D1".into(),
                    nodes: ["b".into(), "0".into()],
                    temperature,
                    model: None,
                },
            ],
            ..Default::default()
        };
        for t in [0.0, -10.0, f64::NAN, f64::INFINITY] {
            match validate_circuit(&diode(t)) {
                Err(SimError::InvalidComponent(msg)) => {
                    assert!(msg.contains("D1") && msg.contains("temperature"), "{msg}");
                }
                other => panic!("T = {t}: {other:?}"),
            }
        }
        assert!(validate_circuit(&diode(300.15)).is_ok());

        let mut bjt = diode(300.15);
        bjt.components[2] = CircuitElement::Bjt {
            id: "Q1".into(),
            nodes: ["b".into(), "a".into(), "0".into()],
            kind: crate::BjtKind::Npn,
            bf: 0.0,
            temperature: 300.15,
            parasitic_caps: None,
            model: None,
        };
        match validate_circuit(&bjt) {
            Err(SimError::InvalidComponent(msg)) => {
                assert!(msg.contains("Q1") && msg.contains("bf"), "{msg}");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn no_ground_node_returns_error() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "n3".into()],
                    resistance: 1000.0,
                },
            ],
            ..Default::default()
        };
        match validate_circuit(&circuit) {
            Err(SimError::NoGround) => {} // expected
            other => panic!("expected NoGround, got: {other:?}"),
        }
    }

    #[test]
    fn disconnected_subcircuit_returns_error() {
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
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Resistor {
                    id: "R2".into(),
                    nodes: ["n3".into(), "n4".into()],
                    resistance: 1000.0,
                },
            ],
            ..Default::default()
        };
        match validate_circuit(&circuit) {
            Err(SimError::DisconnectedNodes(nodes)) => {
                assert!(nodes.contains(&"n3".to_string()));
                assert!(nodes.contains(&"n4".to_string()));
            }
            other => panic!("expected DisconnectedNodes, got: {other:?}"),
        }
    }

    #[test]
    fn floating_node_returns_error() {
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
            ],
            ..Default::default()
        };
        match validate_circuit(&circuit) {
            Err(SimError::FloatingNode(node)) => assert_eq!(node, "n2"),
            other => panic!("expected FloatingNode, got: {other:?}"),
        }
    }

    #[test]
    fn valid_circuit_passes_validation() {
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
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
            ..Default::default()
        };
        assert!(validate_circuit(&circuit).is_ok());
    }

    #[test]
    fn parallel_voltage_sources_different_values_returns_singular() {
        // Validation passes (all nodes connected, no floating), but solve
        // fails because parallel voltage sources with different values
        // produce a singular matrix.
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::VoltageSource {
                    id: "V2".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
            ],
            ..Default::default()
        };
        // Validation itself should pass
        assert!(validate_circuit(&circuit).is_ok());
        // But solving should fail with SingularMatrix
        match solve_circuit(&circuit) {
            Err(SimError::SingularMatrix) => {} // expected
            other => panic!("expected SingularMatrix, got: {other:?}"),
        }
    }
}
