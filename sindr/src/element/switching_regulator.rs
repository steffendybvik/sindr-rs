//! An averaged switching regulator (buck, boost): its behaviour over a
//! switching period, without the switching edges.

use crate::handle::NodeId;
use nalgebra::DVector;
use num_complex::Complex64;
use sindr_devices::converter::{
    converter_dissipation, converter_edges, converter_input, converter_piece, ConverterParams,
    ConverterPiece, ConverterRegion, ConverterTopology, InputCurrent,
};

use crate::error::SimError;
use crate::mna::MnaSystem;

use super::{AcStamp, Check, DcStamp, Device, Report};
use crate::ratings::{RatingKind, RatingProbe};

#[cfg(feature = "spice")]
use crate::spice::export::{unsupported, Deck, ExportError};

/// Averaged switching regulator: a buck or boost converter modelled by its
/// average over a switching period, so a 5 V module simulates in
/// milliseconds instead of resolving every edge.
///
/// The output holds `voltage` above the ground pin, behind a current limit
/// and a dropout (a buck out of headroom follows its input down; a boost
/// whose input is above its set point passes it through). The input draws
/// what power balance asks, `Vout·Iout / (η·Vin)`, plus the quiescent
/// current. Below the under-voltage lockout it's off: output open, nothing
/// drawn. `model` holds efficiency (constant or over load), limit, dropout,
/// lockout, and the switching frequency and inductor for a ripple estimate;
/// see `sindr_devices::converter`. Its result is its output voltage, its
/// output current and the power it loses.
///
/// There is no switching ripple, loop response or soft start: in a
/// transient it reaches each new operating point at once.
///
/// `nodes: [input, output, gnd]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct SwitchingRegulator {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[input, output, gnd]`.
    pub nodes: [NodeId; 3], // [input, output, gnd]
    /// Buck or boost.
    #[cfg_attr(feature = "serde", serde(default))]
    pub topology: ConverterTopology,
    /// Regulated output voltage (V) from the ground pin, positive. From a
    /// feedback divider: `sindr_devices::converter::feedback_voltage`.
    pub voltage: f64,
    /// Efficiency, limit, dropout and lockout. The default is ideal.
    #[cfg_attr(feature = "serde", serde(default))]
    pub model: ConverterParams,
}

impl SwitchingRegulator {
    /// The piece of its characteristic it is on at `x`, given its pins.
    fn piece_at(&self, pins: Pins, branch: usize, x: Option<&DVector<f64>>) -> ConverterPiece {
        // No iterate yet: start regulating at no load.
        let (vin, _, iout) = state(pins, branch, x).unwrap_or((f64::INFINITY, 0.0, 0.0));
        converter_piece(&self.model, self.topology, self.voltage, vin, iout)
    }

    /// Its input current at `x`, linearised, on `piece`. Nothing without an
    /// iterate but the quiescent current.
    fn input_at(
        &self,
        piece: &ConverterPiece,
        pins: Pins,
        branch: usize,
        x: Option<&DVector<f64>>,
    ) -> InputCurrent {
        match state(pins, branch, x) {
            Some((vin, vout, iout)) => converter_input(&self.model, piece.region, vin, vout, iout),
            None if piece.region == ConverterRegion::Off => InputCurrent::default(),
            None => InputCurrent {
                i: self.model.quiescent_current,
                ..Default::default()
            },
        }
    }

    /// Where its characteristic changes piece at `x`.
    pub(crate) fn edges_at(&self, pins: Pins, branch: usize, x: &DVector<f64>) -> [f64; 5] {
        let (vin, _, iout) = state(pins, branch, Some(x)).unwrap_or_default();
        converter_edges(&self.model, self.topology, self.voltage, vin, iout)
    }

    /// The piece of its characteristic it is on at `x`.
    pub(crate) fn region_at(&self, pins: Pins, branch: usize, x: &DVector<f64>) -> ConverterPiece {
        self.piece_at(pins, branch, Some(x))
    }
}

/// `[input, output, gnd]` as matrix indices (`None` is ground).
pub(crate) type Pins = [Option<usize>; 3];

impl Device for SwitchingRegulator {
    device_identity!();

    /// Its output current.
    fn branches(&self) -> usize {
        1
    }

    /// Power balance divides by the input; dropout, the limit and the
    /// lockout are edges.
    fn is_nonlinear(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.positive("voltage", self.voltage, " V")?;
        let m = &self.model;
        if !(m.efficiency > 0.0 && m.efficiency <= 1.0) {
            return Err(check.fail(format_args!(
                "model.efficiency {} must be above 0 and at most 1",
                m.efficiency
            )));
        }
        let mut last = f64::NEG_INFINITY;
        for &[i, eta] in &m.efficiency_curve {
            if !(i.is_finite() && i >= 0.0 && i > last) {
                return Err(check.fail(format_args!(
                    "model.efficiency_curve currents must be finite, non-negative and \
                     rising, not {i} A after {last} A"
                )));
            }
            if !(eta > 0.0 && eta <= 1.0) {
                return Err(check.fail(format_args!(
                    "model.efficiency_curve efficiency {eta} at {i} A must be above 0 and \
                     at most 1"
                )));
            }
            last = i;
        }
        check.non_negative("model.dropout", m.dropout, " V")?;
        check.non_negative("model.dropout_resistance", m.dropout_resistance, " Ω")?;
        check.non_negative("model.uvlo", m.uvlo, " V")?;
        check.non_negative("model.quiescent_current", m.quiescent_current, " A")?;
        if let Some(limit) = m.current_limit {
            check.positive("model.current_limit", limit, " A")?;
        }
        if let Some(f) = m.switching_frequency {
            check.positive("model.switching_frequency", f, " Hz")?;
        }
        if let Some(l) = m.inductance {
            check.positive("model.inductance", l, " H")?;
        }
        Ok(())
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let pins = s.nodes();
        let piece = self.piece_at(pins, s.branch, s.v_prev);
        let input = self.input_at(&piece, pins, s.branch, s.v_prev);
        stamp_at(
            s.system,
            pins,
            s.branch,
            &piece,
            &input,
            state(pins, s.branch, s.v_prev),
        );
        Ok(())
    }

    /// The piece and the input current's partials at the DC bias.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let pins = s.nodes();
        let branch = s.branch;
        let x = Some(s.dc_solution);
        let piece = self.piece_at(pins, branch, x);
        let input = self.input_at(&piece, pins, branch, x);
        let c = |x: f64| Complex64::new(x, 0.0);
        for (row, col, value) in matrix(pins, branch, &piece, &input) {
            s.add(Some(row), Some(col), c(value));
        }
    }

    /// Its output voltage and current, and the power it loses.
    fn report(&self, r: &mut Report<'_>) {
        let (vin, vout, iout) = switching_state(r.node_map, &self.nodes, r.branch, r.solution);
        let region = converter_piece(&self.model, self.topology, self.voltage, vin, iout).region;
        let power = converter_dissipation(&self.model, region, vin, vout, iout);
        r.branch_current_of(&self.id, r.branch_current(0));
        r.component(&self.id, vout, iout, power);
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        use RatingKind::*;
        &[Voltage, Current, Power]
    }

    /// Its voltage rating is its maximum input, from input to ground; its
    /// current rating, its output current.
    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        match kind {
            RatingKind::Voltage => m.between(0, 2),
            _ => m.port_rating(kind),
        }
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, _: &mut Deck<'_>, _: &str) -> Result<(), ExportError> {
        Err(unsupported(
            &self.id,
            "an averaged switching regulator has no SPICE primitive",
        ))
    }
}

/// The input voltage, output voltage and output current a converter is at
/// in `x` (all from its ground pin), or `None` without one.
fn state(
    [input, out, gnd]: Pins,
    branch: usize,
    x: Option<&DVector<f64>>,
) -> Option<(f64, f64, f64)> {
    let x = x?;
    let v = |i: Option<usize>| i.map_or(0.0, |i| x[i]);
    let vg = v(gnd);
    // The branch unknown is the current into the output pin; the converter
    // sources the opposite.
    Some((v(input) - vg, v(out) - vg, -x[branch]))
}

/// The converter's matrix entries on `piece`, with `input` its input
/// current's partials: `(row, column, value)`, ground rows left out.
///
/// Branch row: `Vout − Vg = offset + d_vin·(Vin − Vg) + d_iout·Iout` with
/// `Iout = −i_branch`, or `Iout = 0` when off. KCL: the output pin takes
/// `i_branch`, which returns at the ground pin; the input draws `Iin` to
/// the ground pin.
fn matrix(
    [input, out, gnd]: Pins,
    branch: usize,
    piece: &ConverterPiece,
    i: &InputCurrent,
) -> Vec<(usize, usize, f64)> {
    let mut m = Vec::with_capacity(16);
    let mut put = |r: Option<usize>, c: Option<usize>, v: f64| {
        if let (Some(r), Some(c)) = (r, c) {
            if v != 0.0 {
                m.push((r, c, v));
            }
        }
    };
    let b = Some(branch);
    put(out, b, 1.0);
    put(gnd, b, -1.0);
    if piece.region == ConverterRegion::Off {
        put(b, b, 1.0);
    } else {
        put(b, out, 1.0);
        put(b, input, -piece.d_vin);
        put(b, gnd, piece.d_vin - 1.0);
        put(b, b, piece.d_iout);
    }
    // Iin = … + d_vin·(Vin − Vg) + d_vout·(Vout − Vg) − d_iout·i_branch,
    // leaving the input and arriving at the ground pin.
    for (row, sign) in [(input, 1.0), (gnd, -1.0)] {
        put(row, input, sign * i.d_vin);
        put(row, out, sign * i.d_vout);
        put(row, gnd, -sign * (i.d_vin + i.d_vout));
        put(row, b, -sign * i.d_iout);
    }
    m
}

/// Stamp a converter on `piece`, its input current linearised at `at`
/// (`(vin, vout, iout)` from the ground pin; `None` takes `input.i` as it
/// is).
fn stamp_at(
    system: &mut MnaSystem,
    pins: Pins,
    branch: usize,
    piece: &ConverterPiece,
    input: &InputCurrent,
    at: Option<(f64, f64, f64)>,
) {
    for (r, c, v) in matrix(pins, branch, piece, input) {
        system.a[(r, c)] += v;
    }
    if piece.region != ConverterRegion::Off {
        system.b[branch] = piece.offset;
    }
    // The constant part of the linearised input current.
    let (vin, vout, iout) = at.unwrap_or_default();
    let i_eq = input.i - input.d_vin * vin - input.d_vout * vout - input.d_iout * iout;
    let [inp, _, gnd] = pins;
    if let Some(i) = inp {
        system.b[i] -= i_eq;
    }
    if let Some(g) = gnd {
        system.b[g] += i_eq;
    }
}

/// The input voltage, output voltage and output current of converter
/// `nodes` in `solution`, from its ground pin.
fn switching_state(
    node_map: &crate::node_map::NodeMap,
    nodes: &[NodeId; 3],
    branch: usize,
    solution: &DVector<f64>,
) -> (f64, f64, f64) {
    let pins = nodes.each_ref().map(|n| node_map.index(n));
    state(pins, branch, Some(solution)).unwrap_or_default()
}
