//! Pre-solve circuit validation.
//!
//! Catches obvious topology mistakes (no ground, disconnected subgraphs,
//! floating nodes) and device values the models can't use (a junction
//! temperature at or below 0 K, a BJT `bf` at or below 0) before they reach
//! the solver, where the failure mode would be a less helpful "singular
//! matrix" or "invalid solution" error. Run automatically
//! by [`solve_circuit`](crate::solve_circuit); also exposed standalone via
//! [`validate_circuit`].

use crate::element::Behavioural;
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
/// 6. A behavioural source's `i(src)` names a voltage-defined element
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
        comp.device().validate()?;
    }

    // ------------------------------------------------------------------
    // Check 6: what a behavioural source's expression reads exists
    // ------------------------------------------------------------------
    for comp in &circuit.components {
        if let CircuitElement::Behavioural(Behavioural { id, expr, .. }) = comp {
            for variable in expr.variables() {
                if let crate::expression::Variable::Current { source } = variable {
                    if circuit.vsource_branch_index(source).is_none() {
                        return Err(SimError::InvalidComponent(format!(
                            "`{id}` reads i({source}), but `{source}` isn't a voltage-defined \
                             element (voltage source, VCVS, CCVS, op-amp, regulator or \
                             behavioural voltage source)"
                        )));
                    }
                }
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::{Circuit, CircuitElement};
    use crate::element::{Bjt, Diode, Jfet, Resistor, VoltageSource};
    use crate::error::SimError;
    use crate::solve_circuit;

    #[test]
    fn bad_model_parameters_name_the_field() {
        let with = |component: CircuitElement| Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["a".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["a".into(), "b".into()],
                    resistance: 1e3,
                }),
                component,
            ],
            ..Default::default()
        };
        let diode = |model: crate::DiodeModel| {
            with(CircuitElement::Diode(Diode {
                id: "D1".into(),
                nodes: ["b".into(), "0".into()],
                temperature: 300.15,
                model: Some(model),
            }))
        };
        let bjt = |model: crate::BjtModel| {
            with(CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "a".into(), "0".into()],
                kind: crate::BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: Some(model),
            }))
        };
        let jfet = |caps: crate::circuit::JfetParasiticCaps| {
            with(CircuitElement::Jfet(Jfet {
                id: "J1".into(),
                nodes: ["b".into(), "a".into(), "0".into()],
                kind: crate::JfetKind::NChannel,
                idss: 10e-3,
                vp: -2.0,
                parasitic_caps: Some(caps),
            }))
        };
        let d = crate::DiodeModel::default();
        let q = crate::BjtModel::default();
        let j = crate::circuit::JfetParasiticCaps {
            cgs: 1e-12,
            ..Default::default()
        };
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
            (diode(crate::DiodeModel { eg: 0.0, ..d }), "model.eg"),
            (diode(crate::DiodeModel { xti: -1.0, ..d }), "model.xti"),
            (bjt(crate::BjtModel { eg: f64::NAN, ..q }), "model.eg"),
            (bjt(crate::BjtModel { xti: -0.5, ..q }), "model.xti"),
            (
                jfet(crate::circuit::JfetParasiticCaps { cgd: -1e-12, ..j }),
                "parasitic_caps.cgd",
            ),
            (
                jfet(crate::circuit::JfetParasiticCaps { pb: 0.0, ..j }),
                "parasitic_caps.pb",
            ),
            (
                jfet(crate::circuit::JfetParasiticCaps { m: 1.0, ..j }),
                "parasitic_caps.m",
            ),
            (
                jfet(crate::circuit::JfetParasiticCaps { fc: 0.99, ..j }),
                "parasitic_caps.fc",
            ),
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["a".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["a".into(), "b".into()],
                    resistance: 1e3,
                }),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["b".into(), "0".into()],
                    temperature,
                    model: None,
                }),
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
        bjt.components[2] = CircuitElement::Bjt(Bjt {
            id: "Q1".into(),
            nodes: ["b".into(), "a".into(), "0".into()],
            kind: crate::BjtKind::Npn,
            bf: 0.0,
            temperature: 300.15,
            parasitic_caps: None,
            model: None,
        });
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n2".into(), "n3".into()],
                    resistance: 1000.0,
                }),
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R2".into(),
                    nodes: ["n3".into(), "n4".into()],
                    resistance: 1000.0,
                }),
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                }),
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V2".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
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
