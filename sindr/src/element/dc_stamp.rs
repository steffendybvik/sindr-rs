//! What an element's DC stamp writes into and reads from.

use nalgebra::DVector;

use crate::circuit::Circuit;
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::{NodeMap, Terminals};

use super::layout::Layout;

/// One element's view of a DC stamp, for [`Device::stamp_dc`]: the system
/// it writes into, where its terminals and branch rows are, and the
/// operating point it is linearised at.
///
/// [`Device::stamp_dc`]: super::Device::stamp_dc
pub(crate) struct DcStamp<'a> {
    /// The MNA system being assembled.
    pub(crate) system: &'a mut MnaSystem,
    /// The matrix row of the element's first branch unknown, past the node
    /// voltages. An element without branches doesn't use it.
    pub(crate) branch: usize,
    /// The iterate to linearise at, or `None` for the linear first pass,
    /// where devices that need one stamp nothing.
    pub(crate) v_prev: Option<&'a DVector<f64>>,
    /// For the nets an element reads without connecting to them: a
    /// controlled source's control pair, the nodes an expression reads, the
    /// circuit's ground.
    pub(crate) node_map: &'a NodeMap,
    /// The circuit being stamped.
    pub(crate) circuit: &'a Circuit,
    terminals: &'a [Option<usize>],
    layout: &'a Layout,
    k: usize,
}

impl<'a> DcStamp<'a> {
    /// The stamp of `circuit`'s element `k`.
    pub(crate) fn new(
        system: &'a mut MnaSystem,
        circuit: &'a Circuit,
        node_map: &'a NodeMap,
        layout: &'a Layout,
        terminals: &'a Terminals,
        k: usize,
        v_prev: Option<&'a DVector<f64>>,
    ) -> Self {
        Self {
            system,
            branch: node_map.num_nodes() + layout.branch(k),
            v_prev,
            node_map,
            circuit,
            terminals: terminals.of(k),
            layout,
            k,
        }
    }

    /// The element's first `N` terminals as matrix indices, in `nodes()`
    /// order (`None` is ground).
    pub(crate) fn nodes<const N: usize>(&self) -> [Option<usize>; N] {
        std::array::from_fn(|i| self.terminals[i])
    }

    /// The matrix row of the branch current a CCVS or CCCS reads, from
    /// its [`Layout::control`]. `None` when its control source isn't there.
    pub(crate) fn control_branch(&self) -> Option<usize> {
        self.layout
            .control(self.k)
            .map(|b| self.node_map.num_nodes() + b)
    }
}

/// The error from stamping an element the analyses lower to simpler
/// ones first (an SPDT switch, a crystal, a ferrite bead).
pub(super) fn stamped_as_parts(id: &str) -> SimError {
    SimError::component(
        id,
        "is stamped as its parts: pass the circuit through an analysis \
         entry point (solve_circuit, solve_transient, …), not stamp_circuit",
    )
}
