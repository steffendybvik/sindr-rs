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

use nalgebra::DVector;

use crate::circuit::{CircuitElement, OpAmpModel};
use crate::integration::{History, Integrator};
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;
use crate::stamp::stamp_current_source;

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
        let (model, nodes, v_pos, v_neg) = match element {
            CircuitElement::OpAmp {
                nodes,
                v_pos,
                v_neg,
                model: Some(model),
                ..
            } => (model, nodes, *v_pos, *v_neg),
            CircuitElement::Comparator {
                nodes,
                v_pos,
                v_neg,
                ..
            } => (&COMPARATOR, nodes, *v_pos, *v_neg),
            _ => return None,
        };
        Some(Self {
            model,
            nodes,
            ground,
            v_pos,
            v_neg,
            branch,
        })
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
        let at = |n: &String| node_map.index(n).map_or(0.0, |i| v[i]);
        at(&self.nodes[0]) - at(&self.nodes[1])
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
    fn stamp_output(&self, system: &mut MnaSystem, node_map: &NodeMap, x0: f64) {
        let br = self.branch;
        if let Some(o) = node_map.index(&self.nodes[2]) {
            system.a[(o, br)] += 1.0;
            system.a[(br, o)] += 1.0;
        }
        if let Some(g) = node_map.index(self.ground) {
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
            for input in &self.nodes[..2] {
                stamp_current_source(
                    system,
                    node_map,
                    &[input.clone(), self.ground.to_string()],
                    ib,
                );
            }
        }
    }

    /// Stage row `c_x · x − c_u · (v₊ − v₋) = rhs`.
    fn stamp_stage(
        &self,
        system: &mut MnaSystem,
        node_map: &NodeMap,
        c_x: f64,
        c_u: f64,
        rhs: f64,
    ) {
        let x = self.stage();
        system.a[(x, x)] += c_x;
        if let Some(p) = node_map.index(&self.nodes[0]) {
            system.a[(x, p)] -= c_u;
        }
        if let Some(m) = node_map.index(&self.nodes[1]) {
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
        let (x0, vd0) = v_prev.map_or((0.0, 0.0), |v| (v[self.stage()], self.vd(node_map, v)));
        self.stamp_output(system, node_map, x0);
        let (f0, c_x, c_u) = self.stage_rate(x0, vd0);
        // F0 − c_x·(x − x0) + c_u·(vd − vd0) = 0
        self.stamp_stage(system, node_map, c_x, c_u, f0 + c_x * x0 - c_u * vd0);
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
        let x0 = v_prev[self.stage()];
        let vd0 = self.vd(node_map, v_prev);
        self.stamp_output(system, node_map, x0);
        let (f0, c_x, c_u) = self.stage_rate(x0, vd0);
        let (g, h) = integ.capacitor(1.0, history);
        self.stamp_stage(
            system,
            node_map,
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
