//! What an element reports at a solution: its results after a DC solve or
//! a transient step.

use std::collections::HashMap;

use nalgebra::DVector;

use crate::circuit::Circuit;
use crate::handle::NodeId;
use crate::integration::{History, Integrator};
use crate::node_map::NodeMap;
use crate::results::{BjtResult, ComponentResult, MosfetResult, OpAmpResult, RelayResult};

use super::layout::Layout;

/// One element's view of a solution, for [`Device::report`]: the voltages
/// and branch currents it reads, and the results it writes.
///
/// After a DC solve, every result is kept. After a transient step only the
/// [`ComponentResult`]s are, and the element moves its histories on to the
/// step's end with [`set_capacitor`](Self::set_capacitor) and
/// [`set_inductor`](Self::set_inductor).
///
/// [`Device::report`]: super::Device::report
pub(crate) struct Report<'a> {
    /// The circuit being reported.
    pub(crate) circuit: &'a Circuit,
    /// For the nets an element reads.
    pub(crate) node_map: &'a NodeMap,
    /// The solution: node voltages, then branch currents.
    pub(crate) solution: &'a DVector<f64>,
    /// The index in `solution` of the element's first branch current. An
    /// element without branches doesn't use it.
    pub(crate) branch: usize,
    step: Option<Step<'a>>,
    layout: &'a Layout,
    k: usize,
    out: &'a mut Reported,
}

/// What a transient step adds to a [`Report`]: the time its sources were
/// solved at, its integrator, and the histories at its start.
#[derive(Clone, Copy)]
pub(crate) struct Step<'a> {
    /// The time the step's sources were evaluated at.
    pub(crate) source_time: f64,
    /// The step's integration formula.
    pub(crate) integ: &'a Integrator,
    caps: &'a [History],
    inds: &'a [History],
}

impl<'a> Step<'a> {
    /// A step solved with sources at `source_time` and integrated by
    /// `integ` from `caps` and `inds`, in [`Layout`] slot order.
    pub(crate) fn new(
        source_time: f64,
        integ: &'a Integrator,
        caps: &'a [History],
        inds: &'a [History],
    ) -> Self {
        Self {
            source_time,
            integ,
            caps,
            inds,
        }
    }
}

/// The results the elements of a circuit report, in circuit order.
#[derive(Default)]
pub(crate) struct Reported {
    pub(crate) components: Vec<ComponentResult>,
    pub(crate) branch_currents: HashMap<String, f64>,
    pub(crate) bjts: Vec<BjtResult>,
    pub(crate) mosfets: Vec<MosfetResult>,
    pub(crate) op_amps: Vec<OpAmpResult>,
    pub(crate) relays: Vec<RelayResult>,
    pub(crate) state_of_charge: HashMap<String, f64>,
    /// After a transient step: the capacitor and inductor histories at its
    /// end, in [`Layout`] slot order.
    pub(crate) caps: Vec<History>,
    pub(crate) inds: Vec<History>,
}

impl Reported {
    /// Every element of `circuit` reported at `solution`: after a DC solve
    /// when `step` is `None`, otherwise after that transient step.
    pub(crate) fn collect(
        circuit: &Circuit,
        node_map: &NodeMap,
        layout: &Layout,
        solution: &DVector<f64>,
        step: Option<Step<'_>>,
    ) -> Self {
        let mut out = Self::default();
        if let Some(step) = step {
            out.caps = step.caps.to_vec();
            out.inds = step.inds.to_vec();
        }
        for (k, component) in circuit.components.iter().enumerate() {
            let mut r = Report {
                circuit,
                node_map,
                solution,
                branch: node_map.num_nodes() + layout.branch(k),
                step,
                layout,
                k,
                out: &mut out,
            };
            component.device().report(&mut r);
        }
        out
    }
}

impl Report<'_> {
    /// The voltage at `node` (0 V for ground).
    pub(crate) fn v(&self, node: &NodeId) -> f64 {
        self.node_map.index(node).map_or(0.0, |i| self.solution[i])
    }

    /// The voltage from `a` to `b`.
    pub(crate) fn across(&self, a: &NodeId, b: &NodeId) -> f64 {
        self.v(a) - self.v(b)
    }

    /// The element's `i`-th branch current.
    pub(crate) fn branch_current(&self, i: usize) -> f64 {
        self.solution[self.branch + i]
    }

    /// The current a CCVS or CCCS reads, from its [`Layout::control`]:
    /// `None` when its control source isn't there.
    pub(crate) fn control_current(&self) -> Option<f64> {
        self.layout
            .control(self.k)
            .map(|b| self.solution[self.node_map.num_nodes() + b])
    }

    /// The transient step being reported, or `None` after a DC solve.
    pub(crate) fn step(&self) -> Option<Step<'_>> {
        self.step
    }

    /// The time sources are evaluated at: the step's, or 0 after a DC
    /// solve.
    pub(crate) fn time(&self) -> f64 {
        self.step.map_or(0.0, |s| s.source_time)
    }

    /// Reports the element's port: `v` across it, `i` through it, and the
    /// power `v · i` they give.
    pub(crate) fn port(&mut self, id: &str, v: f64, i: f64) {
        self.component(id, v, i, v * i);
    }

    /// Reports the element's port with a `power` other than `v · i`.
    pub(crate) fn component(&mut self, id: &str, v: f64, i: f64, power: f64) {
        self.out.components.push(ComponentResult {
            id: id.to_string(),
            voltage_across: v,
            current_through: i,
            power,
        });
    }

    /// Reports `i` as the branch current others read it by, under `id`.
    /// Kept after a DC solve only.
    pub(crate) fn branch_current_of(&mut self, id: &str, i: f64) {
        if self.step.is_none() {
            self.out.branch_currents.insert(id.to_string(), i);
        }
    }

    /// Reports a BJT's terminal values. Kept after a DC solve only.
    pub(crate) fn bjt(&mut self, result: impl FnOnce() -> BjtResult) {
        if self.step.is_none() {
            self.out.bjts.push(result());
        }
    }

    /// Reports a MOSFET's terminal values. Kept after a DC solve only.
    pub(crate) fn mosfet(&mut self, result: impl FnOnce() -> MosfetResult) {
        if self.step.is_none() {
            self.out.mosfets.push(result());
        }
    }

    /// Reports an op-amp's or comparator's pins. Kept after a DC solve
    /// only.
    pub(crate) fn op_amp(&mut self, result: impl FnOnce() -> OpAmpResult) {
        if self.step.is_none() {
            self.out.op_amps.push(result());
        }
    }

    /// Reports a relay's coil and contact. Kept after a DC solve only.
    pub(crate) fn relay(&mut self, result: impl FnOnce() -> RelayResult) {
        if self.step.is_none() {
            self.out.relays.push(result());
        }
    }

    /// Reports a battery's state of charge. Kept after a DC solve only.
    pub(crate) fn state_of_charge(&mut self, id: &str, soc: f64) {
        if self.step.is_none() {
            self.out.state_of_charge.insert(id.to_string(), soc);
        }
    }

    /// The element's `i`-th capacitor history at the step's start.
    ///
    /// # Panics
    ///
    /// After a DC solve, which keeps no histories.
    pub(crate) fn capacitor(&self, i: usize) -> History {
        let step = self.step.expect("a DC solve keeps no histories");
        step.caps[self.layout.capacitor(self.k) + i]
    }

    /// The element's `i`-th inductor history at the step's start.
    ///
    /// # Panics
    ///
    /// After a DC solve, which keeps no histories.
    pub(crate) fn inductor(&self, i: usize) -> History {
        let step = self.step.expect("a DC solve keeps no histories");
        step.inds[self.layout.inductor(self.k) + i]
    }

    /// Sets the element's `i`-th capacitor history at the step's end.
    pub(crate) fn set_capacitor(&mut self, i: usize, h: History) {
        self.out.caps[self.layout.capacitor(self.k) + i] = h;
    }

    /// Sets the element's `i`-th inductor history at the step's end.
    pub(crate) fn set_inductor(&mut self, i: usize, h: History) {
        self.out.inds[self.layout.inductor(self.k) + i] = h;
    }
}
