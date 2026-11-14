//! Hierarchy recovery for flattened subcircuit instances.
//!
//! After lowering, every subcircuit-internal node is renamed to a flat,
//! `.`-separated path (`X1.X2.in`). The [`SourceMap`] lets callers walk
//! back from a flattened name to the original instance path + node, which
//! is what most schematic-style probe UIs want.

use std::collections::HashMap;

/// Map from a flattened node name to its original hierarchical location.
pub type SourceMap = HashMap<String, HierarchyPath>;

/// Original hierarchical position of a flattened node.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HierarchyPath {
    /// Stack of `X<inst>` instance names from outermost to innermost.
    /// Empty for nodes at the top level.
    pub instance_path: Vec<String>,
    /// Node name as written inside its declaring scope (the subckt body or
    /// the top-level netlist).
    pub original_node: String,
}

impl HierarchyPath {
    /// Separator used when flattening hierarchical node names.
    ///
    /// `.` matches the convention used by ngspice / xyce.
    pub fn flatten_separator() -> char {
        '.'
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spice::parse_str;

    /// `stage` wraps `inner`, so `inner`'s internal node sits two levels deep.
    const NESTED: &str = "* nested
V1 in 0 1
X1 in out stage
R9 out 0 1k
.subckt inner a b
R1 a mid 1k
R2 mid b 1k
.ends
.subckt stage p q
X2 p n inner
R3 n q 1k
.ends
.end
";

    fn path(instances: &[&str], node: &str) -> HierarchyPath {
        HierarchyPath {
            instance_path: instances.iter().map(|s| s.to_string()).collect(),
            original_node: node.into(),
        }
    }

    #[test]
    fn internal_nodes_map_back_to_their_instance_path() {
        let map = parse_str(NESTED).unwrap().source_map;
        assert_eq!(map.get("x1.n"), Some(&path(&["x1"], "n")));
        assert_eq!(map.get("x1.x2.mid"), Some(&path(&["x1", "x2"], "mid")));
    }

    #[test]
    fn ports_and_top_level_nodes_are_not_mapped() {
        let map = parse_str(NESTED).unwrap().source_map;
        // Ports resolve to the caller's node, so they never get a flat name.
        assert_eq!(map.len(), 2, "unexpected entries: {:?}", map.keys());
        for node in ["in", "out", "0", "x1.p", "x1.q", "x1.x2.a", "x1.x2.b"] {
            assert!(!map.contains_key(node), "{node} should not be mapped");
        }
    }

    #[test]
    fn every_flattened_node_in_the_circuit_has_an_entry() {
        let netlist = parse_str(NESTED).unwrap();
        let sep = HierarchyPath::flatten_separator();
        for element in &netlist.circuit.components {
            for node in element.nodes() {
                if !node.contains(sep) {
                    continue;
                }
                let entry = netlist
                    .source_map
                    .get(node.as_str())
                    .unwrap_or_else(|| panic!("{node} missing from source map"));
                let mut rebuilt = entry.instance_path.join(&sep.to_string());
                rebuilt.push(sep);
                rebuilt.push_str(&entry.original_node);
                assert_eq!(&rebuilt, node);
            }
        }
    }

    #[test]
    fn flat_netlist_has_an_empty_map() {
        let map = parse_str("* flat\nV1 a 0 1\nR1 a 0 1k\n.end\n")
            .unwrap()
            .source_map;
        assert!(map.is_empty());
    }

    #[test]
    fn separator_is_a_dot() {
        assert_eq!(HierarchyPath::flatten_separator(), '.');
    }
}
