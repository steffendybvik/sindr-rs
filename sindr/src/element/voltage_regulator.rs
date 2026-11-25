//! A linear voltage regulator with dropout, quiescent current and a
//! current limit.

use crate::handle::NodeId;
use nalgebra::DVector;
use sindr_devices::regulator::{regulator_piece, RegulatorLinear, RegulatorParams};

use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::node_map::{NodeMap, Terminals};

use super::{AcStamp, Check, DcStamp, Device, Report};
use crate::ratings::{RatingKind, RatingProbe};
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::spice::export::{unsupported, Deck, ExportError};

/// Linear voltage regulator: holds the output at `voltage` above its
/// ground pin while it has headroom, and draws its output current plus
/// its quiescent current from the input.
///
/// Piecewise linear (see `sindr_devices::regulator`): out of headroom
/// the output follows the input down by the dropout; past the current
/// limit it holds the current. `model` sets dropout, quiescent current,
/// current limit and line and load regulation;
/// `RegulatorParams::lm7805()` and `::ams1117()` are datasheet presets.
/// Its result is the pass element: `Vin − Vout`, the output current,
/// and the power it dissipates.
///
/// `nodes: [input, output, gnd]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct VoltageRegulator {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[input, output, gnd]`.
    pub nodes: [NodeId; 3], // [input, output, gnd]
    /// Regulated output voltage (V) from the ground pin. E.g. 5.0 for a
    /// 7805, or −5.0 for a 7905, whose input sits below ground and whose
    /// output current flows in at the output pin.
    pub voltage: f64,
    /// Dropout, quiescent current, limit and regulation. `None` is an
    /// ideal regulator that still can't put out more than it gets in.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<sindr_devices::regulator::RegulatorParams>,
}

impl Device for VoltageRegulator {
    device_identity!();

    /// Its pass element, input to output: `Vin − Vout`, the output
    /// current, and the power it dissipates, quiescent included.
    fn report(&self, r: &mut Report<'_>) {
        let (vin, vout, iout) =
            regulator_state(r.node_map, &self.nodes, r.branch, Some(r.solution))
                .unwrap_or_default();
        let params = self.model.unwrap_or_default();
        let power =
            sindr_devices::regulator::regulator_dissipation(&params, self.voltage, vin, vout, iout);
        r.branch_current_of(&self.id, r.branch_current(0));
        r.component(&self.id, vin - vout, iout, power);
    }

    /// The piece of its characteristic the bias point is on: input ripple
    /// reaches the output through line regulation (or all of it in
    /// dropout), and load current through the output resistance. The
    /// quiescent current only matters below dropout, where it ramps with
    /// the input.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let branch = s.branch;
        let pins = s.nodes();
        let piece = piece(pins, self.voltage, &self.model, branch, Some(s.dc_solution));
        let c = |x: f64| Complex64::new(x, 0.0);
        let [input, out, gnd] = pins;
        if let Some(o) = out {
            s.a[(o, branch)] += c(1.0);
            s.a[(branch, o)] += c(1.0);
        }
        if let Some(i) = input {
            s.a[(i, branch)] -= c(1.0);
            s.a[(branch, i)] -= c(piece.d_vin);
        }
        if let Some(g) = gnd {
            s.a[(branch, g)] += c(piece.d_vin - 1.0);
        }
        s.a[(branch, branch)] += c(piece.d_iout);
        // The quiescent current is fixed above dropout; below it, it ramps
        // with the input, a conductance from input to ground.
        if let Some(params) = &self.model {
            let vin = state(pins, branch, Some(s.dc_solution)).map_or(0.0, |(vin, _, _)| vin);
            let (_, g_q) = sindr_devices::regulator::regulator_quiescent(params, self.voltage, vin);
            for (r, col, sign) in [
                (input, input, 1.0),
                (input, gnd, -1.0),
                (gnd, input, -1.0),
                (gnd, gnd, 1.0),
            ] {
                s.add(r, col, c(sign * g_q));
            }
        }
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        use RatingKind::*;
        &[Voltage, Current, Power]
    }

    /// Its voltage rating is its maximum input, from input to ground.
    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        match kind {
            RatingKind::Voltage => m.between(0, 2),
            _ => m.port_rating(kind),
        }
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let nodes = s.nodes();
        stamp_regulator_at(
            s.system,
            nodes,
            self.voltage,
            &self.model,
            s.branch,
            s.v_prev,
        );
        Ok(())
    }

    /// Its output current.
    fn branches(&self) -> usize {
        1
    }

    /// Dropout and the current limit make it piecewise linear.
    fn is_nonlinear(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        let voltage = self.voltage;
        // Negative for a 79xx-style regulator; zero regulates nothing.
        if !voltage.is_finite() || voltage == 0.0 {
            return Err(check.fail(format_args!(
                "voltage {voltage} V must be a non-zero, finite number"
            )));
        }
        if let Some(m) = &self.model {
            check.non_negative("model.dropout", m.dropout, " V")?;
            check.non_negative("model.dropout_resistance", m.dropout_resistance, " Ω")?;
            check.non_negative("model.quiescent_current", m.quiescent_current, " A")?;
            check.non_negative("model.line_regulation", m.line_regulation, " V/V")?;
            check.non_negative("model.output_resistance", m.output_resistance, " Ω")?;
            if let Some(limit) = m.current_limit {
                check.positive("model.current_limit", limit, " A")?;
            }
        }
        Ok(())
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, _: &mut Deck<'_>, _: &str) -> Result<(), ExportError> {
        Err(unsupported(
            &self.id,
            "a regulator macromodel has no SPICE primitive",
        ))
    }
}

/// The input voltage and output current a regulator is at in `solution`
/// (both from its ground pin), or `None` without one.
pub(crate) fn regulator_state(
    node_map: &NodeMap,
    nodes: &[NodeId; 3],
    branch: usize,
    solution: Option<&DVector<f64>>,
) -> Option<(f64, f64, f64)> {
    state(pins(node_map, nodes), branch, solution)
}

/// `[input, output, gnd]` as matrix indices (`None` is ground).
fn pins(node_map: &NodeMap, nodes: &[NodeId; 3]) -> [Option<usize>; 3] {
    nodes.each_ref().map(|n| node_map.index(n))
}

/// Regulator `k`'s `[input, output, gnd]` from the solve's terminals.
fn regulator_pins(terminals: &Terminals, k: usize) -> [Option<usize>; 3] {
    let pins = terminals.of(k);
    std::array::from_fn(|i| pins.get(i).copied().flatten())
}

/// [`regulator_state`] with the pins resolved.
fn state(
    [input, out, gnd]: [Option<usize>; 3],
    branch: usize,
    solution: Option<&DVector<f64>>,
) -> Option<(f64, f64, f64)> {
    let x = solution?;
    let v = |i: Option<usize>| i.map_or(0.0, |i| x[i]);
    let vg = v(gnd);
    // The branch unknown is the current into the output pin; the regulator
    // sources the opposite.
    Some((v(input) - vg, v(out) - vg, -x[branch]))
}

/// The linear piece of a regulator's characteristic at `solution`, given
/// its `[input, output, gnd]` as matrix indices, or the regulating piece
/// at no load without one (unbounded headroom on the side of the set
/// voltage, so a negative regulator starts regulating too).
fn piece(
    pins: [Option<usize>; 3],
    voltage: f64,
    model: &Option<RegulatorParams>,
    branch: usize,
    solution: Option<&DVector<f64>>,
) -> RegulatorLinear {
    let params = model.unwrap_or_default();
    let (vin, iout) = state(pins, branch, solution)
        .map_or((f64::INFINITY.copysign(voltage), 0.0), |(vin, _, iout)| {
            (vin, iout)
        });
    regulator_piece(&params, voltage, vin, iout)
}

/// `true` when every regulator in `circuit` is on the same piece of its
/// characteristic, and of its quiescent-current ramp, at `v_new` as at
/// `v_prev`, the iterate it was stamped from.
///
/// The characteristic is piecewise linear, so a Newton step that lands on
/// another piece can be within the node tolerance (about 1 mV on a 1 V
/// node) and still be the solution of the wrong piece. Leaving the 0 V
/// floor, that stale millivolt across a large output capacitor becomes a
/// current spike on the next short step.
pub(crate) fn regulator_pieces_settled(
    circuit: &Circuit,
    terminals: &Terminals,
    num_nodes: usize,
    v_prev: &DVector<f64>,
    v_new: &DVector<f64>,
) -> bool {
    let mut branch = num_nodes;
    for (k, component) in circuit.components.iter().enumerate() {
        if let CircuitElement::VoltageRegulator(VoltageRegulator { voltage, model, .. }) = component
        {
            let pins = regulator_pins(terminals, k);
            let at = |x: &DVector<f64>| regulator_pieces_at(pins, *voltage, model, branch, x);
            if at(v_prev) != at(v_new) {
                return false;
            }
        }
        if let CircuitElement::SwitchingRegulator(sw) = component {
            let pins = regulator_pins(terminals, k);
            if sw.region_at(pins, branch, v_prev) != sw.region_at(pins, branch, v_new) {
                return false;
            }
        }
        branch += component.branch_slots();
    }
    true
}

/// The piece of its characteristic, and of its quiescent-current ramp, a
/// regulator is on at `x`.
fn regulator_pieces_at(
    pins: [Option<usize>; 3],
    voltage: f64,
    model: &Option<RegulatorParams>,
    branch: usize,
    x: &DVector<f64>,
) -> (RegulatorLinear, Option<(f64, f64)>) {
    let piece = piece(pins, voltage, model, branch, Some(x));
    let ramp = model
        .as_ref()
        .zip(state(pins, branch, Some(x)))
        .map(|(params, (vin, _, _))| {
            sindr_devices::regulator::regulator_quiescent(params, voltage, vin)
        });
    (piece, ramp)
}

/// Shorten a Newton step from `v_prev` to `v_new` so that no regulator,
/// linear or switching, passes more than one edge of its piecewise characteristic. Returns
/// whether the step was shortened: a shortened iterate is not a Newton
/// solution, so it must not count as converged.
///
/// Each iterate solves the regulators' pieces at `v_prev`, which hold only
/// up to the first edge along the step. Where a node is held by little else
/// (the input behind an inductor whose companion conductance is `h/L`,
/// about 1 µS on a 1 ps step), that solution can lie many volts and every
/// piece away, and the next iterate's piece throws it back: Newton cycles
/// between the floor and regulation for ever. Stopping inside the next
/// piece along the step, as Katzenelson's method for piecewise-linear
/// networks does, walks the regulator one piece at a time. Halfway to the
/// next edge keeps it clear of both.
pub(crate) fn limit_regulator_step(
    circuit: &Circuit,
    terminals: &Terminals,
    num_nodes: usize,
    v_prev: &DVector<f64>,
    v_new: &mut DVector<f64>,
) -> bool {
    // Fractions of the step at which some regulator that changes piece
    // crosses an edge.
    let mut crossings: Vec<f64> = Vec::new();
    let mut branch = num_nodes;
    for (k, component) in circuit.components.iter().enumerate() {
        if let CircuitElement::VoltageRegulator(VoltageRegulator { voltage, model, .. }) = component
        {
            let pins = regulator_pins(terminals, k);
            let at = |x: &DVector<f64>| regulator_pieces_at(pins, *voltage, model, branch, x);
            if at(v_prev) != at(v_new) {
                let params = model.unwrap_or_default();
                let edges = |x: &DVector<f64>| {
                    let (vin, _, iout) = state(pins, branch, Some(x)).unwrap_or_default();
                    let mut e =
                        sindr_devices::regulator::regulator_edges(&params, *voltage, vin, iout);
                    if model.is_none() {
                        // No quiescent current, so no ramp to cross.
                        e[3] = f64::INFINITY;
                        e[4] = f64::INFINITY;
                    }
                    e
                };
                push_crossings(&mut crossings, edges(v_prev), edges(v_new));
            }
        }
        if let CircuitElement::SwitchingRegulator(sw) = component {
            let pins = regulator_pins(terminals, k);
            if sw.region_at(pins, branch, v_prev) != sw.region_at(pins, branch, v_new) {
                push_crossings(
                    &mut crossings,
                    sw.edges_at(pins, branch, v_prev),
                    sw.edges_at(pins, branch, v_new),
                );
            }
        }
        branch += component.branch_slots();
    }
    crossings.sort_by(f64::total_cmp);
    // Edges that meet (the floor and the top of the ramp, both at the
    // dropout voltage) are crossed together.
    let first = match crossings.first() {
        Some(&first) => first,
        None => return false,
    };
    let Some(&next) = crossings.iter().find(|&&c| c > first + 1e-9) else {
        return false;
    };
    let alpha = 0.5 * (first + next);
    *v_new = v_prev + (&*v_new - v_prev) * alpha;
    true
}

/// The fractions of a step at which edges `a` (at its start) and `b` (at
/// its end) change sign.
fn push_crossings<const N: usize>(crossings: &mut Vec<f64>, a: [f64; N], b: [f64; N]) {
    for (a, b) in a.into_iter().zip(b) {
        let side = |e: f64| (e > 0.0, e < 0.0);
        if a.is_finite() && b.is_finite() && side(a) != side(b) {
            crossings.push(a / (a - b));
        }
    }
}

/// Stamp a linear regulator `[in, out, gnd]` on the piece of its
/// characteristic the previous iterate `v_prev` is on.
///
/// Branch row: `Vout − Vg = offset + d_vin·(Vin − Vg) + d_iout·Iout`, with
/// `Iout = −i_branch`. KCL: the output pin takes `i_branch`, the input
/// gives `Iout + Iq`, and the quiescent current returns through ground
/// (all mirrored for a negative `voltage`).
///
/// Terminals as matrix indices (`None` is ground).
pub(crate) fn stamp_regulator_at(
    system: &mut MnaSystem,
    [input, out, gnd]: [Option<usize>; 3],
    voltage: f64,
    model: &Option<RegulatorParams>,
    branch: usize,
    v_prev: Option<&DVector<f64>>,
) {
    let piece = piece([input, out, gnd], voltage, model, branch, v_prev);
    // Quiescent current `iq + g·(Vin − Vg)` from the input to the ground pin,
    // on the piece of its ramp the previous iterate is on. A negative
    // regulator's runs the other way: in at the ground pin, out at the
    // (negative) input.
    let (iq, g_q) = match model {
        Some(params) => {
            let vin = state([input, out, gnd], branch, v_prev)
                .map_or(f64::INFINITY.copysign(voltage), |(vin, _, _)| vin);
            sindr_devices::regulator::regulator_quiescent(params, voltage, vin)
        }
        None => (0.0, 0.0),
    };
    let mut conductance = |r: Option<usize>, c: Option<usize>, g: f64| {
        if let (Some(r), Some(c)) = (r, c) {
            system.a[(r, c)] += g;
        }
    };
    conductance(input, input, g_q);
    conductance(input, gnd, -g_q);
    conductance(gnd, input, -g_q);
    conductance(gnd, gnd, g_q);
    if let Some(o) = out {
        system.a[(o, branch)] += 1.0;
        system.a[(branch, o)] += 1.0;
    }
    if let Some(i) = input {
        system.a[(i, branch)] -= 1.0;
        system.a[(branch, i)] -= piece.d_vin;
        system.b[i] -= iq;
    }
    if let Some(g) = gnd {
        system.a[(branch, g)] += piece.d_vin - 1.0;
        system.b[g] += iq;
    }
    system.a[(branch, branch)] += piece.d_iout;
    system.b[branch] = piece.offset;
}
