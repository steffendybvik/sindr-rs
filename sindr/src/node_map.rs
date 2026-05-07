use std::collections::{BTreeSet, HashMap};

use crate::circuit::Circuit;

/// Maps circuit node names to MNA matrix indices.
///
/// The ground node is excluded from the matrix (maps to `None`).
/// All other nodes receive sequential zero-based indices.
#[derive(Debug, Clone)]
pub struct NodeMap {
    map: HashMap<String, usize>,
    reverse: Vec<String>,
    ground: String,
}

impl NodeMap {
    /// Build a `NodeMap` by extracting all unique nodes from a [`Circuit`].
    ///
    /// Nodes are sorted alphabetically (via `BTreeSet`) before index
    /// assignment to ensure deterministic, reproducible ordering.
    pub fn from_circuit(circuit: &Circuit) -> Self {
        let mut unique: BTreeSet<String> = BTreeSet::new();
        for component in &circuit.components {
            for node in component.all_nodes() {
                unique.insert(node.clone());
            }
        }
        // Remove ground
        unique.remove(&circuit.ground_node);

        let mut map = HashMap::new();
        let mut reverse = Vec::new();
        for node in unique {
            let idx = map.len();
            map.insert(node.clone(), idx);
            reverse.push(node);
        }

        Self {
            map,
            reverse,
            ground: circuit.ground_node.clone(),
        }
    }

    /// Build a `NodeMap` from a list of node names and a ground node name.
    ///
    /// The ground node is filtered out; remaining nodes are assigned
    /// sequential indices starting from 0.
    pub fn from_nodes(nodes: &[String], ground: &str) -> Self {
        let mut map = HashMap::new();
        let mut reverse = Vec::new();

        for node in nodes {
            if node == ground {
                continue;
            }
            if map.contains_key(node.as_str()) {
                continue;
            }
            let idx = map.len();
            map.insert(node.clone(), idx);
            reverse.push(node.clone());
        }

        Self {
            map,
            reverse,
            ground: ground.to_string(),
        }
    }

    /// Look up the matrix index for a node.
    ///
    /// Returns `None` for the ground node, `Some(index)` for all others.
    pub fn index(&self, node: &str) -> Option<usize> {
        if node == self.ground {
            None
        } else {
            self.map.get(node).copied()
        }
    }

    /// Number of non-ground nodes (i.e. the number of node-voltage unknowns).
    pub fn num_nodes(&self) -> usize {
        self.map.len()
    }

    /// Reverse lookup: get the node name for a given matrix index.
    pub fn node_name(&self, index: usize) -> Option<&str> {
        self.reverse.get(index).map(|s| s.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn three_nodes_with_ground() {
        let nodes: Vec<String> = vec!["0".into(), "n1".into(), "n2".into()];
        let nm = NodeMap::from_nodes(&nodes, "0");

        assert_eq!(nm.num_nodes(), 2);
        assert_eq!(nm.index("0"), None, "ground must map to None");
        assert!(nm.index("n1").is_some());
        assert!(nm.index("n2").is_some());

        // Indices must be distinct
        assert_ne!(nm.index("n1"), nm.index("n2"));
    }

    #[test]
    fn empty_node_list_only_ground() {
        let nodes: Vec<String> = vec!["0".into()];
        let nm = NodeMap::from_nodes(&nodes, "0");

        assert_eq!(nm.num_nodes(), 0);
        assert_eq!(nm.index("0"), None);
    }

    #[test]
    fn reverse_lookup_matches_forward() {
        let nodes: Vec<String> = vec!["0".into(), "a".into(), "b".into(), "c".into()];
        let nm = NodeMap::from_nodes(&nodes, "0");

        for name in &["a", "b", "c"] {
            let idx = nm.index(name).expect("node should have an index");
            let reverse_name = nm.node_name(idx).expect("index should have a name");
            assert_eq!(reverse_name, *name);
        }
    }

    #[test]
    fn duplicate_nodes_are_deduplicated() {
        let nodes: Vec<String> = vec!["0".into(), "n1".into(), "n1".into(), "n2".into()];
        let nm = NodeMap::from_nodes(&nodes, "0");

        assert_eq!(nm.num_nodes(), 2);
    }

    #[test]
    fn unknown_node_returns_none() {
        let nodes: Vec<String> = vec!["0".into(), "n1".into()];
        let nm = NodeMap::from_nodes(&nodes, "0");

        assert_eq!(nm.index("nonexistent"), None);
    }
}
