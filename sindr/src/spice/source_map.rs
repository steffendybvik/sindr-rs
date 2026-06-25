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
