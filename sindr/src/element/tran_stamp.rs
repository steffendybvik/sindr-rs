//! What an element's transient stamp writes into and reads from.

use nalgebra::DVector;

use crate::integration::{History, Integrator};
use crate::mna::MnaSystem;
use crate::node_map::{FastMap, NodeMap, Sensed, Terminals};

use super::layout::Layout;
use super::DcStamp;

/// One element's view of a transient step's stamp, for
/// [`Device::stamp_transient`]: the system it writes into, where its
/// terminals, branch rows and histories are, the step's time and
/// integrator, and the Newton iterate it is linearised at.
///
/// [`Device::stamp_transient`]: super::Device::stamp_transient
pub(crate) struct TranStamp<'a> {
    /// The MNA system being assembled.
    pub(crate) system: &'a mut MnaSystem,
    /// The matrix row of the element's first branch unknown, past the node
    /// voltages. An element without branches doesn't use it.
    pub(crate) branch: usize,
    /// The Newton iterate to linearise at: the previous step's solution on
    /// the first iteration.
    pub(crate) v_prev: &'a DVector<f64>,
    /// The time the step ends at, where sources are evaluated.
    pub(crate) t: f64,
    /// The step's integration formula.
    pub(crate) integ: &'a Integrator,
    node_map: &'a NodeMap,
    k: usize,
    terminals: &'a Terminals,
    layout: &'a Layout,
    caps: &'a [History],
    inds: &'a [History],
    parasitic: &'a FastMap<History>,
}

impl<'a> TranStamp<'a> {
    /// The stamp of `circuit`'s element `k` for a step integrated by
    /// `integ` to time `t`, from the histories at the step's start: `caps`
    /// and `inds` in [`Layout`] slot order, `parasitic` keyed
    /// `"{id}-{key}"`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        system: &'a mut MnaSystem,
        node_map: &'a NodeMap,
        layout: &'a Layout,
        terminals: &'a Terminals,
        k: usize,
        v_prev: &'a DVector<f64>,
        t: f64,
        integ: &'a Integrator,
        caps: &'a [History],
        inds: &'a [History],
        parasitic: &'a FastMap<History>,
    ) -> Self {
        Self {
            system,
            branch: node_map.num_nodes() + layout.branch(k),
            v_prev,
            t,
            integ,
            node_map,
            k,
            terminals,
            layout,
            caps,
            inds,
            parasitic,
        }
    }

    /// The element's first `N` terminals as matrix indices, in `nodes()`
    /// order (`None` is ground).
    pub(crate) fn nodes<const N: usize>(&self) -> [Option<usize>; N] {
        let terminals = self.terminals.of(self.k);
        std::array::from_fn(|i| terminals[i])
    }

    /// What the element reads without connecting to, in
    /// [`Device::sensed`](super::Device::sensed) order.
    pub(crate) fn sensed(&self) -> &'a [Sensed] {
        self.terminals.sensed(self.k)
    }

    /// The voltage at matrix index `node` in the iterate (0 V for ground).
    pub(crate) fn voltage(&self, node: Option<usize>) -> f64 {
        node.map_or(0.0, |i| self.v_prev[i])
    }

    /// The element's `i`-th capacitor history, of the
    /// [`Device::capacitor_histories`](super::Device::capacitor_histories)
    /// it keeps.
    pub(crate) fn capacitor(&self, i: usize) -> &'a History {
        &self.caps[self.layout.capacitor(self.k) + i]
    }

    /// The element's `i`-th inductor history, of the
    /// [`Device::inductor_histories`](super::Device::inductor_histories)
    /// it keeps.
    pub(crate) fn inductor(&self, i: usize) -> &'a History {
        &self.inds[self.layout.inductor(self.k) + i]
    }

    /// The history kept under component `id`'s `key` (a junction, a stored
    /// charge, a battery's state), uncharged before its first step.
    pub(crate) fn history(&self, id: &str, key: &str) -> History {
        parasitic_history(self.parasitic, id, key)
    }

    /// The element's DC stamp at the iterate, for a device whose
    /// transient model is its DC one, or starts from it.
    pub(crate) fn dc(&mut self) -> DcStamp<'_> {
        DcStamp::new(
            self.system,
            self.node_map,
            self.layout,
            self.terminals,
            self.k,
            Some(self.v_prev),
        )
    }
}

/// The history in `parasitic` of component `id`'s `key` (uncharged before
/// its first step).
pub(crate) fn parasitic_history(parasitic: &FastMap<History>, id: &str, key: &str) -> History {
    parasitic
        .get(&format!("{id}-{key}"))
        .copied()
        .unwrap_or_default()
}
