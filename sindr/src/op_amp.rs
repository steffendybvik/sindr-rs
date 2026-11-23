//! Op-amp macromodel ([`OpAmpModel`]) stamps.
//!
//! An op-amp with a model has two MNA unknowns after the node voltages:
//! its output current `i` (slot `k`, the one `vsource_branch_index` names)
//! and its gain-stage voltage `x` (slot `k + 1`). With
//! `u = gain · (v₊ − v₋ + offset)` and `sat` a smooth clamp to the rails:
//!
//! ```text
//!   output:  v_out − sat(x) − r_out · i = 0
//!   stage:   τ · dx/dt = D(u − x) + K · (sat(x) − x)      (DC: = 0)
//! ```
//!
//! τ = gain / (2π · gbw) puts the open-loop pole at gbw / gain, and
//! `D(z) = SR·τ · tanh(z / (SR·τ))` caps dx/dt at the slew rate (`D(z) = z`
//! without one). The `K` term is back-calculation anti-windup: past a rail
//! it pulls `x` back, so a saturated stage sits within about `|v₊ − v₋|` of
//! the rail instead of winding up towards `u`, and recovers quickly.
//! `K = gain`. Within the rails `sat(x) = x` and the term vanishes.
//!
//! A comparator is the same stamp with [`COMPARATOR`]: gain 1e5 and no pole,
//! so its output follows the clamped gain stage instantly.

use crate::element::{Comparator, OpAmp};
use nalgebra::DVector;

use crate::circuit::{CircuitElement, OpAmpModel};
use crate::integration::{History, Integrator};
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;
use crate::stamp::stamp_current_source_at;

/// Width of the clamp's knee (V): within a few of these of a rail the
/// output bends over smoothly instead of with a kink Newton can't follow.
const KNEE: f64 = 0.01;

/// A comparator's model: the unmodelled op-amp's gain of 1e5 with no pole,
/// slew limit, output resistance, offset or bias, so it differs from that
/// op-amp only in holding its output between the rails.
pub(crate) const COMPARATOR: OpAmpModel = OpAmpModel {
    gain: 1e5,
    gbw: None,
    slew_rate: None,
    r_out: 0.0,
    offset_voltage: 0.0,
    bias_current: 0.0,
};

/// The terminals and unknowns of one modelled op-amp or comparator.
pub(crate) struct OpAmpStamp<'a> {
    pub model: &'a OpAmpModel,
    /// `[in_plus, in_minus, out]`.
    pub nodes: &'a [String; 3],
    pub ground: &'a str,
    pub v_pos: f64,
    pub v_neg: f64,
    /// Row/column of the output current; the stage voltage is the next one.
    pub branch: usize,
}

/// An op-amp's or comparator's terminals as matrix indices (`None` is
/// ground).
pub(crate) struct Pins {
    /// `[in_plus, in_minus, out]`.
    pub(crate) nodes: [Option<usize>; 3],
    /// The circuit's ground, which the output is relative to.
    pub(crate) ground: Option<usize>,
}

/// `w · ln(1 + e^(z/w))` without overflow, and its derivative.
fn softplus(z: f64) -> (f64, f64) {
    let t = z / KNEE;
    let sigma = 1.0 / (1.0 + (-t).exp());
    let value = if t > 30.0 { z } else { KNEE * t.exp().ln_1p() };
    (value, sigma)
}

/// `u` clamped smoothly to `[lo, hi]`, and d/du.
pub(crate) fn clamp(u: f64, lo: f64, hi: f64) -> (f64, f64) {
    let (over, d_over) = softplus(u - hi);
    let (under, d_under) = softplus(lo - u);
    (u - over + under, 1.0 - d_over - d_under)
}

impl<'a> OpAmpStamp<'a> {
    /// The stamp of an element whose output is clamped to its rails: an
    /// op-amp with a model, or a comparator. `None` for anything else.
    /// `branch` is its first MNA branch slot.
    pub(crate) fn of(element: &'a CircuitElement, ground: &'a str, branch: usize) -> Option<Self> {
        match element {
            CircuitElement::OpAmp(op_amp) => Self::of_op_amp(op_amp, ground, branch),
            CircuitElement::Comparator(comparator) => {
                Some(Self::of_comparator(comparator, ground, branch))
            }
            _ => None,
        }
    }

    /// [`of`](Self::of) an op-amp: `None` without a model.
    pub(crate) fn of_op_amp(op_amp: &'a OpAmp, ground: &'a str, branch: usize) -> Option<Self> {
        let model = op_amp.model.as_ref()?;
        Some(Self {
            model,
            nodes: &op_amp.nodes,
            ground,
            v_pos: op_amp.v_pos,
            v_neg: op_amp.v_neg,
            branch,
        })
    }

    /// [`of`](Self::of) a comparator.
    pub(crate) fn of_comparator(
        comparator: &'a Comparator,
        ground: &'a str,
        branch: usize,
    ) -> Self {
        Self {
            model: &COMPARATOR,
            nodes: &comparator.nodes,
            ground,
            v_pos: comparator.v_pos,
            v_neg: comparator.v_neg,
            branch,
        }
    }

    /// `[in_plus, in_minus, out]` and the ground as matrix indices.
    fn pins(&self, node_map: &NodeMap) -> Pins {
        let [p, m, o] = &self.nodes;
        Pins {
            nodes: [node_map.index(p), node_map.index(m), node_map.index(o)],
            ground: node_map.index(self.ground),
        }
    }

    fn stage(&self) -> usize {
        self.branch + 1
    }

    /// The stage's time constant τ (s), or `None` without a pole.
    pub(crate) fn tau(&self) -> Option<f64> {
        self.model
            .gbw
            .map(|gbw| self.model.gain / (2.0 * std::f64::consts::PI * gbw))
    }

    fn sat(&self, x: f64) -> (f64, f64) {
        clamp(x, self.v_neg, self.v_pos)
    }

    /// `D(z)` and `D'(z)`: the slew-limited drive.
    fn drive(&self, z: f64) -> (f64, f64) {
        match (self.model.slew_rate, self.tau()) {
            (Some(sr), Some(tau)) => {
                let k = sr * tau;
                let t = (z / k).tanh();
                (k * t, 1.0 - t * t)
            }
            _ => (z, 1.0),
        }
    }

    /// `v₊ − v₋` at the node voltages in `v`.
    fn vd(&self, node_map: &NodeMap, v: &DVector<f64>) -> f64 {
        Self::vd_at(&self.pins(node_map), v)
    }

    /// [`vd`](Self::vd) with the pins resolved.
    fn vd_at(pins: &Pins, v: &DVector<f64>) -> f64 {
        let at = |i: Option<usize>| i.map_or(0.0, |i| v[i]);
        at(pins.nodes[0]) - at(pins.nodes[1])
    }

    /// The stage's right-hand side `F = D(u − x) + K·(sat(x) − x)` at
    /// `(x, vd)`, with `−∂F/∂x` and `∂F/∂vd`.
    fn stage_rate(&self, x: f64, vd: f64) -> (f64, f64, f64) {
        let gain = self.model.gain;
        let u = gain * (vd + self.model.offset_voltage);
        let (d, dd) = self.drive(u - x);
        let (s, ds) = self.sat(x);
        (d + gain * (s - x), dd + gain * (1.0 - ds), dd * gain)
    }

    /// The DC stage voltage for a given `v₊ − v₋`: the root of `F`, which
    /// falls monotonically in `x`.
    pub(crate) fn settled_stage(&self, vd: f64) -> f64 {
        let u = self.model.gain * (vd + self.model.offset_voltage);
        let mut x = self.sat(u).0;
        for _ in 0..50 {
            let (f, c_x, _) = self.stage_rate(x, vd);
            let step = f / c_x;
            x += step;
            if step.abs() <= 1e-12 * x.abs().max(1.0) {
                break;
            }
        }
        x
    }

    /// The output row, the bias currents, and the KCL entries of the output
    /// current, linearised at stage voltage `x0`.
    fn stamp_output(&self, system: &mut MnaSystem, pins: &Pins, x0: f64) {
        let br = self.branch;
        if let Some(o) = pins.nodes[2] {
            system.a[(o, br)] += 1.0;
            system.a[(br, o)] += 1.0;
        }
        if let Some(g) = pins.ground {
            system.a[(g, br)] -= 1.0;
            system.a[(br, g)] -= 1.0;
        }
        // v_out − [s0 + s'·(x − x0)] − r_out·i = 0
        let (s0, ds) = self.sat(x0);
        system.a[(br, self.stage())] -= ds;
        system.a[(br, br)] -= self.model.r_out;
        system.b[br] += s0 - ds * x0;
        let ib = self.model.bias_current;
        if ib != 0.0 {
            for &input in &pins.nodes[..2] {
                stamp_current_source_at(system, [input, pins.ground], ib);
            }
        }
    }

    /// Stage row `c_x · x − c_u · (v₊ − v₋) = rhs`.
    fn stamp_stage(&self, system: &mut MnaSystem, pins: &Pins, c_x: f64, c_u: f64, rhs: f64) {
        let x = self.stage();
        system.a[(x, x)] += c_x;
        if let Some(p) = pins.nodes[0] {
            system.a[(x, p)] -= c_u;
        }
        if let Some(m) = pins.nodes[1] {
            system.a[(x, m)] += c_u;
        }
        system.b[x] += rhs;
    }

    /// DC: `F(x, vd) = 0` and the output row, linearised at `v_prev` (at
    /// rest when there is no iterate yet).
    pub(crate) fn stamp_dc(
        &self,
        system: &mut MnaSystem,
        node_map: &NodeMap,
        v_prev: Option<&DVector<f64>>,
    ) {
        self.stamp_dc_at(system, &self.pins(node_map), v_prev);
    }

    /// [`stamp_dc`](Self::stamp_dc) with the pins resolved.
    pub(crate) fn stamp_dc_at(
        &self,
        system: &mut MnaSystem,
        pins: &Pins,
        v_prev: Option<&DVector<f64>>,
    ) {
        let (x0, vd0) = v_prev.map_or((0.0, 0.0), |v| (v[self.stage()], Self::vd_at(pins, v)));
        self.stamp_output(system, pins, x0);
        let (f0, c_x, c_u) = self.stage_rate(x0, vd0);
        // F0 − c_x·(x − x0) + c_u·(vd − vd0) = 0
        self.stamp_stage(system, pins, c_x, c_u, f0 + c_x * x0 - c_u * vd0);
    }

    /// One transient step: `τ·(g·x + h) = F(x, vd)`, with `(g, h)` the
    /// integrator's rate companion, linearised at `v_prev`. Without a pole
    /// it is the DC row.
    pub(crate) fn stamp_transient(
        &self,
        system: &mut MnaSystem,
        node_map: &NodeMap,
        integ: &Integrator,
        history: &History,
        v_prev: &DVector<f64>,
    ) {
        let Some(tau) = self.tau() else {
            return self.stamp_dc(system, node_map, Some(v_prev));
        };
        let pins = self.pins(node_map);
        let x0 = v_prev[self.stage()];
        let vd0 = Self::vd_at(&pins, v_prev);
        self.stamp_output(system, &pins, x0);
        let (f0, c_x, c_u) = self.stage_rate(x0, vd0);
        let (g, h) = integ.capacitor(1.0, history);
        self.stamp_stage(
            system,
            &pins,
            tau * g + c_x,
            c_u,
            -tau * h + f0 + c_x * x0 - c_u * vd0,
        );
    }

    /// The small-signal stage row `(jωτ + c_x)·x − c_u·vd = 0` and output
    /// row slope `sat'(x)` at the bias point whose node voltages are in `v`:
    /// `(τ, c_x, c_u, sat')`.
    pub(crate) fn small_signal(
        &self,
        node_map: &NodeMap,
        v: &DVector<f64>,
    ) -> (f64, f64, f64, f64) {
        let vd = self.vd(node_map, v);
        let x = self.settled_stage(vd);
        let (_, c_x, c_u) = self.stage_rate(x, vd);
        (self.tau().unwrap_or(0.0), c_x, c_u, self.sat(x).1)
    }

    /// The stage's history after a step that ended at `solution`.
    pub(crate) fn advance(
        &self,
        integ: &Integrator,
        history: &History,
        solution: &DVector<f64>,
    ) -> History {
        let x = solution[self.stage()];
        let (g, h) = integ.capacitor(1.0, history);
        history.push(x, g * x + h)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamp_is_the_identity_away_from_the_rails() {
        let (s, ds) = clamp(3.0, -15.0, 15.0);
        assert!((s - 3.0).abs() < 1e-12 && (ds - 1.0).abs() < 1e-12);
    }

    #[test]
    fn clamp_holds_the_rails_far_past_them() {
        let (s, ds) = clamp(1e5, -15.0, 15.0);
        assert!((s - 15.0).abs() < 1e-9 && ds.abs() < 1e-12, "{s} {ds}");
        let (s, ds) = clamp(-1e5, -15.0, 15.0);
        assert!((s + 15.0).abs() < 1e-9 && ds.abs() < 1e-12, "{s} {ds}");
    }

    #[test]
    fn clamp_derivative_matches_a_difference() {
        for u in [14.98, 15.0, 15.02, -14.99, 0.0] {
            let h = 1e-7;
            let numeric = (clamp(u + h, -15.0, 15.0).0 - clamp(u - h, -15.0, 15.0).0) / (2.0 * h);
            let (_, ds) = clamp(u, -15.0, 15.0);
            assert!((numeric - ds).abs() < 1e-5, "u={u}: {numeric} vs {ds}");
        }
    }
}
