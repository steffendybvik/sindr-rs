use std::collections::{HashMap, HashSet, VecDeque};

use crate::circuit::Circuit;
use crate::error::SimError;

/// Validate a circuit before solving.
///
/// Checks are run in order of most-helpful-first:
/// 1. Ground node exists (at least one component touches it)
/// 2. All nodes reachable from ground (BFS connectivity)
/// 3. No floating nodes (every non-ground node has degree >= 2)
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
pub fn validate_circuit(circuit: &Circuit) -> Result<(), SimError> {
    // ------------------------------------------------------------------
    // Check 1: Ground node exists (SIM-9)
    // ------------------------------------------------------------------
    let has_ground = circuit
        .components
        .iter()
        .any(|c| c.nodes().iter().any(|n| n == &circuit.ground_node));

    if !has_ground {
        return Err(SimError::NoGround);
    }

    // ------------------------------------------------------------------
    // Check 2: All nodes reachable from ground — BFS connectivity (SIM-7)
    // ------------------------------------------------------------------
    let mut adj: HashMap<&str, Vec<&str>> = HashMap::new();
    for comp in &circuit.components {
        let nodes: Vec<&str> = comp.all_nodes().into_iter().map(|s| s.as_str()).collect();
        for i in 0..nodes.len() {
            for j in (i + 1)..nodes.len() {
                adj.entry(nodes[i])
                    .or_default()
                    .push(nodes[j]);
                adj.entry(nodes[j])
                    .or_default()
                    .push(nodes[i]);
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
    // Check 3: Floating nodes — degree < 2 (SIM-8)
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

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::{Circuit, CircuitElement};
    use crate::error::SimError;
    use crate::solve_circuit;

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
