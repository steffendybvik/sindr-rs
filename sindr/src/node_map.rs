//! Node-name ↔ matrix-index mapping.
//!
//! MNA matrices are indexed by integer; circuits are described in terms of
//! arbitrary node-name strings. [`NodeMap`] is the bidirectional translation
//! table between the two. The ground node is excluded from the matrix and
//! maps to `None` — every other unique node string gets a sequential index.

use std::collections::BTreeSet;
use std::hash::{BuildHasherDefault, Hasher};

/// FNV-1a: node names are short, and SipHash's DoS resistance buys nothing
/// for a table built from the caller's own circuit.
#[derive(Default)]
pub(crate) struct Fnv(u64);

impl Hasher for Fnv {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        let mut h = if self.0 == 0 {
            0xcbf2_9ce4_8422_2325
        } else {
            self.0
        };
        for b in bytes {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
        self.0 = h;
    }
}

type HashMap<K, V> = std::collections::HashMap<K, V, BuildHasherDefault<Fnv>>;

/// A string-keyed map with the [`Fnv`] hasher, for tables looked up on every
/// iteration of a solve.
pub(crate) type FastMap<V> = HashMap<String, V>;

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
                unique.insert(node.to_string());
            }
        }
        // Remove ground
        unique.remove(&circuit.ground_node);

        let mut map = HashMap::default();
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
    #[cfg(test)]
    pub fn from_nodes(nodes: &[String], ground: &str) -> Self {
        let mut map = HashMap::default();
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

/// Each element's terminals as matrix indices, resolved once per solve so
/// the per-iteration stamps don't look names up. Element `k` of the circuit
/// it was built from has [`of(k)`](Self::of), in
/// [`nodes`](crate::circuit::CircuitElement::nodes) order, and
/// [`sensed(k)`](Self::sensed), what it reads without connecting to.
#[derive(Debug, Clone, Default)]
pub(crate) struct Terminals {
    start: Vec<usize>,
    index: Vec<Option<usize>>,
    sensed_start: Vec<usize>,
    sensed: Vec<Sensed>,
}

/// A quantity an element reads from the solution without connecting to
/// it: a controlled source's control pair, a variable of a behavioural
/// expression.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Sensed {
    /// The voltage from the first index to the second (`None` is ground).
    Voltage([Option<usize>; 2]),
    /// The unknown at this row: an element's branch current. `None` when
    /// the circuit has no element of that id whose current can be read.
    Current(Option<usize>),
    /// The time.
    Time,
}

impl Terminals {
    pub(crate) fn new(circuit: &Circuit, node_map: &NodeMap) -> Self {
        let n = circuit.components.len() + 1;
        let (mut start, mut index) = (Vec::with_capacity(n), Vec::new());
        let (mut sensed_start, mut sensed) = (Vec::with_capacity(n), Vec::new());
        for component in &circuit.components {
            start.push(index.len());
            index.extend(component.nodes().iter().map(|n| node_map.index(n)));
            sensed_start.push(sensed.len());
            let device = component.device();
            sensed.extend(device.sensed().iter().map(|r| r.resolve(circuit, node_map)));
        }
        start.push(index.len());
        sensed_start.push(sensed.len());
        Self {
            start,
            index,
            sensed_start,
            sensed,
        }
    }

    /// Element `k`'s terminals.
    pub(crate) fn of(&self, k: usize) -> &[Option<usize>] {
        &self.index[self.start[k]..self.start[k + 1]]
    }

    /// What element `k` reads, in [`Device::sensed`] order.
    ///
    /// [`Device::sensed`]: crate::element::Device::sensed
    pub(crate) fn sensed(&self, k: usize) -> &[Sensed] {
        &self.sensed[self.sensed_start[k]..self.sensed_start[k + 1]]
    }
}

impl Sensed {
    /// The control pair of a VCVS or VCCS, its only read: the first
    /// voltage in `sensed`, both ground when there is none.
    pub(crate) fn control_pair(sensed: &[Sensed]) -> [Option<usize>; 2] {
        match sensed.first() {
            Some(Sensed::Voltage(pair)) => *pair,
            _ => [None, None],
        }
    }
}

/// A quantity an element reads, by name, as [`Device::sensed`] lists it.
///
/// [`Device::sensed`]: crate::element::Device::sensed
pub(crate) enum Read<'a> {
    /// `v(node, reference)`; ground when `reference` is `None`.
    Voltage(&'a str, Option<&'a str>),
    /// The branch current of the element with this id.
    Current(&'a str),
    /// The time.
    Time,
}

impl Read<'_> {
    /// As matrix indices.
    pub(crate) fn resolve(&self, circuit: &Circuit, node_map: &NodeMap) -> Sensed {
        match *self {
            Read::Voltage(p, q) => {
                Sensed::Voltage([node_map.index(p), q.and_then(|q| node_map.index(q))])
            }
            Read::Current(id) => Sensed::Current(
                circuit
                    .vsource_branch_index(id)
                    .map(|b| node_map.num_nodes() + b),
            ),
            Read::Time => Sensed::Time,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::Resistor;

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

    #[test]
    fn terminals_resolve_each_element_in_node_order() {
        use crate::circuit::CircuitElement;
        let circuit = Circuit::new(
            "0",
            vec![
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["b".into(), "a".into()],
                    resistance: 1.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R2".into(),
                    nodes: ["0".into(), "b".into()],
                    resistance: 1.0,
                }),
            ],
        );
        let nm = NodeMap::from_circuit(&circuit);
        let t = Terminals::new(&circuit, &nm);
        assert_eq!(t.of(0), [nm.index("b"), nm.index("a")]);
        assert_eq!(t.of(1), [None, nm.index("b")]);
    }

    #[test]
    fn terminals_resolve_what_each_element_senses() {
        use crate::circuit::{BehaviouralOutput, CircuitElement};
        use crate::element::{Behavioural, Vcvs, VoltageSource};
        use crate::expression::Expression;
        let circuit = Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["a".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                }),
                CircuitElement::Vcvs(Vcvs {
                    id: "E1".into(),
                    nodes: ["b".into(), "0".into()],
                    control_nodes: ["a".into(), "0".into()],
                    gain: 2.0,
                }),
                CircuitElement::Behavioural(Behavioural {
                    id: "B1".into(),
                    nodes: ["c".into(), "0".into()],
                    output: BehaviouralOutput::Current,
                    expr: Expression::parse("v(a, b) * i(V1) + time + i(nope)").unwrap(),
                }),
            ],
        );
        let nm = NodeMap::from_circuit(&circuit);
        let t = Terminals::new(&circuit, &nm);
        let (a, b) = (nm.index("a"), nm.index("b"));
        assert_eq!(t.sensed(0), []);
        assert_eq!(t.sensed(1), [Sensed::Voltage([a, None])]);
        assert_eq!(Sensed::control_pair(t.sensed(1)), [a, None]);
        assert_eq!(
            t.sensed(2),
            [
                Sensed::Voltage([a, b]),
                Sensed::Current(Some(nm.num_nodes())),
                Sensed::Time,
                Sensed::Current(None),
            ]
        );
    }

    #[test]
    fn the_fast_hasher_finds_every_node() {
        let nodes: Vec<String> = (0..1000).map(|i| format!("n{i}")).collect();
        let nm = NodeMap::from_nodes(&nodes, "n0");
        for (i, n) in nodes.iter().enumerate().skip(1) {
            assert_eq!(nm.index(n), Some(i - 1));
        }
    }
}
