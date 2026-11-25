//! What an element's small-signal (AC) stamp writes into and reads from.

use nalgebra::DVector;
use num_complex::Complex64;

use crate::circuit::Circuit;
use crate::mna::MnaMatrix;
use crate::node_map::{NodeMap, Sensed, Terminals};

use super::layout::Layout;

/// One element's view of its linearisation at the DC bias point, for
/// [`Device::linearise`]: where its terminals are and the bias it is
/// linearised at.
///
/// [`Device::linearise`]: super::Device::linearise
pub(crate) struct AcBias<'a> {
    /// The DC bias point: node voltages, then branch currents.
    pub(crate) dc_solution: &'a DVector<f64>,
    terminals: &'a [Option<usize>],
}

impl<'a> AcBias<'a> {
    /// The bias of `circuit`'s element `k`.
    pub(crate) fn new(terminals: &'a Terminals, k: usize, dc_solution: &'a DVector<f64>) -> Self {
        Self {
            dc_solution,
            terminals: terminals.of(k),
        }
    }

    /// The element's first `N` terminals as matrix indices, in `nodes()`
    /// order (`None` is ground).
    pub(crate) fn nodes<const N: usize>(&self) -> [Option<usize>; N] {
        std::array::from_fn(|i| self.terminals[i])
    }

    /// The bias voltage at matrix index `node` (0 V for ground).
    pub(crate) fn voltage(&self, node: Option<usize>) -> f64 {
        node.map_or(0.0, |i| self.dc_solution[i])
    }

    /// The bias voltage from the element's first terminal to its second.
    pub(crate) fn across(&self) -> f64 {
        let [p, q] = self.nodes();
        self.voltage(p) - self.voltage(q)
    }

    /// The conductance `g` between the element's first two terminals.
    pub(crate) fn two_terminal(&self, g: f64) -> Vec<Conductance> {
        let [p, q] = self.nodes();
        vec![(p, p, g), (q, q, g), (p, q, -g), (q, p, -g)]
    }
}

/// One entry of a linearised element's conductance matrix: row, column
/// (`None` is ground) and value (S).
pub(crate) type Conductance = (Option<usize>, Option<usize>, f64);

/// One element's view of the small-signal system at a complex frequency,
/// for [`Device::stamp_ac`]: the matrix it writes into, where its
/// terminals and branch rows are, the frequency, and the bias point it is
/// linearised at.
///
/// [`Device::stamp_ac`]: super::Device::stamp_ac
pub(crate) struct AcStamp<'a> {
    /// The complex MNA matrix being assembled.
    pub(crate) a: &'a mut MnaMatrix<Complex64>,
    /// The complex frequency `s` (`jω` for an AC sweep).
    pub(crate) s: Complex64,
    /// Whether the system is built as the pencil `G + s·C`: a stamp that
    /// isn't affine in `s` takes extra unknowns instead, from
    /// [`aux`](Self::aux).
    pub(crate) pencil: bool,
    /// The matrix row of the element's first branch unknown, past the node
    /// voltages. An element without branches doesn't use it.
    pub(crate) branch: usize,
    /// The DC bias point: node voltages, then branch currents.
    pub(crate) dc_solution: &'a DVector<f64>,
    /// For the nets an element reads without connecting to them: a
    /// controlled source's control pair, the nodes an expression reads.
    pub(crate) node_map: &'a NodeMap,
    /// The circuit being stamped.
    pub(crate) circuit: &'a Circuit,
    terminals: &'a [Option<usize>],
    sensed: &'a [Sensed],
    layout: &'a Layout,
    next_aux: &'a mut usize,
}

impl<'a> AcStamp<'a> {
    /// The stamp of `circuit`'s element `k` at `s`. `next_aux` is the
    /// pencil form's next free extra unknown.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        a: &'a mut MnaMatrix<Complex64>,
        circuit: &'a Circuit,
        node_map: &'a NodeMap,
        layout: &'a Layout,
        terminals: &'a Terminals,
        k: usize,
        dc_solution: &'a DVector<f64>,
        s: Complex64,
        pencil: bool,
        next_aux: &'a mut usize,
    ) -> Self {
        Self {
            a,
            s,
            pencil,
            branch: node_map.num_nodes() + layout.branch(k),
            dc_solution,
            node_map,
            circuit,
            terminals: terminals.of(k),
            sensed: terminals.sensed(k),
            layout,
            next_aux,
        }
    }

    /// The element's first `N` terminals as matrix indices, in `nodes()`
    /// order (`None` is ground).
    pub(crate) fn nodes<const N: usize>(&self) -> [Option<usize>; N] {
        std::array::from_fn(|i| self.terminals[i])
    }

    /// What the element reads without connecting to, in
    /// [`Device::sensed`](super::Device::sensed) order.
    pub(crate) fn sensed(&self) -> &'a [Sensed] {
        self.sensed
    }

    /// The bias voltage at matrix index `node` (0 V for ground).
    pub(crate) fn voltage(&self, node: Option<usize>) -> f64 {
        node.map_or(0.0, |i| self.dc_solution[i])
    }

    /// The bias voltage from the element's first terminal to its second.
    pub(crate) fn across(&self) -> f64 {
        let [p, q] = self.nodes();
        self.voltage(p) - self.voltage(q)
    }

    /// Adds `value` at (`row`, `col`), unless either is ground.
    pub(crate) fn add(&mut self, row: Option<usize>, col: Option<usize>, value: Complex64) {
        if let (Some(r), Some(c)) = (row, col) {
            self.a[(r, c)] += value;
        }
    }

    /// An admittance `y` between `p` and `q`.
    pub(crate) fn admittance(&mut self, [p, q]: [Option<usize>; 2], y: Complex64) {
        self.add(p, p, y);
        self.add(q, q, y);
        self.add(p, q, -y);
        self.add(q, p, -y);
    }

    /// A conductance `g` (S) between `p` and `q`.
    pub(crate) fn conductance(&mut self, nodes: [Option<usize>; 2], g: f64) {
        self.admittance(nodes, Complex64::new(g, 0.0));
    }

    /// Couples branch row `branch` to the pair `p`, `q`: the branch current
    /// leaves `p` and enters `q`, and the row reads `V_p − V_q`.
    pub(crate) fn branch_pair(&mut self, branch: usize, [p, q]: [Option<usize>; 2]) {
        let one = Complex64::new(1.0, 0.0);
        if let Some(p) = p {
            self.a[(p, branch)] += one;
            self.a[(branch, p)] += one;
        }
        if let Some(q) = q {
            self.a[(q, branch)] -= one;
            self.a[(branch, q)] -= one;
        }
    }

    /// The pencil form's next free extra unknown, which the element then
    /// owns. Only for the pencil form, and only as many as
    /// [`Device::pencil_rows`](super::Device::pencil_rows) says.
    pub(crate) fn aux(&mut self) -> usize {
        let row = *self.next_aux;
        *self.next_aux += 1;
        row
    }

    /// The matrix row of the branch current of the element `source` names,
    /// as a CCVS or CCCS reads it. `None` when there is none.
    pub(crate) fn source_branch(&self, source: &str) -> Option<usize> {
        self.circuit
            .components
            .iter()
            .position(|c| {
                let device = c.device();
                device.exposes_branch_current() && device.id() == source
            })
            .map(|k| self.node_map.num_nodes() + self.layout.branch(k))
    }
}
