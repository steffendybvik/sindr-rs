//! Averaged switching regulator (buck, boost): the converter's behaviour
//! over a switching period, without its switching edges.
//!
//! The output is a piecewise-linear function of the input voltage and the
//! output current, both measured from the converter's ground pin. A buck:
//!
//! ```text
//! regulating: Vout = V
//! dropout:    Vout = Vin − Vdo − Rdo·Iout     (full duty, out of headroom)
//! ```
//!
//! whichever is lower. A boost can't hold its output below its input, which
//! reaches the output through its inductor and diode:
//!
//! ```text
//! regulating:   Vout = V
//! pass-through: Vout = Vin − Vdo − Rdo·Iout   (input above the set point)
//! ```
//!
//! whichever is higher. Past the current limit the regulated output falls
//! steeply, so the current stays within 1 % of the limit (a boost's
//! pass-through path isn't limited, as a real one's isn't). Below the
//! under-voltage lockout the converter is off and its output open.
//!
//! The input draws what power balance asks: `Iin = Vout·Iout / (η·Vin)`, plus
//! the quiescent current while it runs. Power sent back into the output
//! (`Vout·Iout < 0`) returns to the input less the same losses.

use crate::regulator::{regulator_edges, regulator_piece, RegulatorParams};

/// How steeply the output falls past the current limit: the full set
/// voltage over this fraction of the limit current. The same as the linear
/// regulator's.
const LIMIT_SHARPNESS: f64 = 0.01;

/// The least input a converter runs from (V), whatever its UVLO: below it
/// power balance would ask an unbounded input current.
pub const MIN_INPUT: f64 = 0.1;

/// Which converter it is.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConverterTopology {
    /// Step-down: the output is at most the input, less the dropout.
    #[default]
    Buck,
    /// Step-up: the output is at least the input, less the diode drop.
    Boost,
}

/// Averaged converter parameters. [`Default`] is ideal: 100 % efficient,
/// no dropout, no limit, no lockout above [`MIN_INPUT`], no quiescent
/// current.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, PartialEq)]
pub struct ConverterParams {
    /// Efficiency, `Pout / Pin` (0 to 1], used when `efficiency_curve` is
    /// empty.
    pub efficiency: f64,
    /// Efficiency over load: `[output current (A), efficiency]` points in
    /// rising current, interpolated linearly and held flat past either
    /// end. Empty uses the constant `efficiency`.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Vec::is_empty"))]
    pub efficiency_curve: Vec<[f64; 2]>,
    /// Output current limit (A). `None` is unlimited.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub current_limit: Option<f64>,
    /// A buck's dropout: the least `Vin − Vout` it regulates with (V). A
    /// boost's drop from input to output when the input is above its set
    /// point (its diode).
    pub dropout: f64,
    /// Rise in that drop per ampere of output current (Ω).
    pub dropout_resistance: f64,
    /// Under-voltage lockout (V): below this input the converter is off and
    /// its output open. [`MIN_INPUT`] at least.
    pub uvlo: f64,
    /// Current drawn from the input while it runs (A), on top of what power
    /// balance asks.
    pub quiescent_current: f64,
    /// Switching frequency (Hz). Not simulated: it sets the ripple estimate
    /// ([`ConverterParams::ripple_current`]) only.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub switching_frequency: Option<f64>,
    /// The converter's inductor (H), for the ripple estimate only.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub inductance: Option<f64>,
}

impl Default for ConverterParams {
    fn default() -> Self {
        Self {
            efficiency: 1.0,
            efficiency_curve: Vec::new(),
            current_limit: None,
            dropout: 0.0,
            dropout_resistance: 0.0,
            uvlo: 0.0,
            quiescent_current: 0.0,
            switching_frequency: None,
            inductance: None,
        }
    }
}

impl ConverterParams {
    /// The efficiency at output current `iout` (A, either direction), and
    /// its slope `dη/d|Iout|`.
    pub fn efficiency_at(&self, iout: f64) -> (f64, f64) {
        let curve = &self.efficiency_curve;
        let i = iout.abs();
        match curve.as_slice() {
            [] => (self.efficiency, 0.0),
            [[_, eta]] => (*eta, 0.0),
            [[i0, eta0], ..] if i <= *i0 => (*eta0, 0.0),
            [.., [in_, etan]] if i >= *in_ => (*etan, 0.0),
            _ => {
                let k = curve
                    .partition_point(|p| p[0] <= i)
                    .clamp(1, curve.len() - 1);
                let ([i0, e0], [i1, e1]) = (curve[k - 1], curve[k]);
                let slope = (e1 - e0) / (i1 - i0);
                (e0 + slope * (i - i0), slope)
            }
        }
    }

    /// The input below which the converter is off (V).
    pub fn lockout(&self) -> f64 {
        self.uvlo.max(MIN_INPUT)
    }

    /// Peak-to-peak inductor ripple current (A) at `vin` and `vout` (from
    /// the ground pin), in continuous conduction: `Vout·(1 − D)/(L·f)` for
    /// a buck, `Vin·D/(L·f)` for a boost. `None` without a switching
    /// frequency and an inductance, or where the topology can't run (a buck
    /// asked for more than its input).
    pub fn ripple_current(&self, topology: ConverterTopology, vin: f64, vout: f64) -> Option<f64> {
        let lf = self.inductance? * self.switching_frequency?;
        match topology {
            ConverterTopology::Buck if vin > 0.0 && (0.0..=vin).contains(&vout) => {
                Some(vout * (1.0 - vout / vin) / lf)
            }
            ConverterTopology::Boost if vin > 0.0 && vout >= vin => {
                Some(vin * (1.0 - vin / vout) / lf)
            }
            _ => None,
        }
    }

    /// The linear regulator whose output characteristic a buck shares.
    fn buck_output(&self) -> RegulatorParams {
        RegulatorParams {
            dropout: self.dropout,
            dropout_resistance: self.dropout_resistance,
            current_limit: self.current_limit,
            ..Default::default()
        }
    }
}

/// The output voltage a feedback divider sets: `vref·(1 + r_top/r_bottom)`,
/// with `r_top` from the output to the feedback pin and `r_bottom` from it
/// to ground.
pub fn feedback_voltage(vref: f64, r_top: f64, r_bottom: f64) -> f64 {
    vref * (1.0 + r_top / r_bottom)
}

/// Which part of its characteristic a converter is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConverterRegion {
    /// Below its lockout: off, output open, nothing drawn.
    Off,
    /// Holding its set voltage.
    Regulating,
    /// A buck out of headroom, or a boost whose input is above its set
    /// point: the output follows the input.
    Dropout,
}

/// The converter's output on one linear piece: open when `region` is
/// [`ConverterRegion::Off`], else `Vout = offset + d_vin·Vin + d_iout·Iout`,
/// all from the ground pin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConverterPiece {
    /// Constant term (V).
    pub offset: f64,
    /// ∂Vout/∂Vin (V/V).
    pub d_vin: f64,
    /// ∂Vout/∂Iout (Ω), zero or negative.
    pub d_iout: f64,
    /// Off, regulating or in dropout.
    pub region: ConverterRegion,
    /// `true` when past the current limit.
    pub limiting: bool,
}

impl ConverterPiece {
    /// The output voltage this piece gives at `vin`, `iout`.
    pub fn vout(&self, vin: f64, iout: f64) -> f64 {
        self.offset + self.d_vin * vin + self.d_iout * iout
    }

    const OFF: Self = Self {
        offset: 0.0,
        d_vin: 0.0,
        d_iout: 0.0,
        region: ConverterRegion::Off,
        limiting: false,
    };
}

/// The piece of a converter set to `voltage` (positive) that applies at
/// input `vin` (V, from the ground pin) and output current `iout` (A, out
/// of the output pin).
pub fn converter_piece(
    params: &ConverterParams,
    topology: ConverterTopology,
    voltage: f64,
    vin: f64,
    iout: f64,
) -> ConverterPiece {
    if vin < params.lockout() {
        return ConverterPiece::OFF;
    }
    match topology {
        ConverterTopology::Buck => {
            let p = regulator_piece(&params.buck_output(), voltage, vin, iout);
            ConverterPiece {
                offset: p.offset,
                d_vin: p.d_vin,
                d_iout: p.d_iout,
                region: match p.region {
                    crate::regulator::RegulatorRegion::Regulating => ConverterRegion::Regulating,
                    _ => ConverterRegion::Dropout,
                },
                limiting: p.limiting,
            }
        }
        ConverterTopology::Boost => {
            let mut regulating = ConverterPiece {
                offset: voltage,
                d_vin: 0.0,
                d_iout: 0.0,
                region: ConverterRegion::Regulating,
                limiting: false,
            };
            if let Some(limit) = params.current_limit.filter(|l| *l > 0.0) {
                if iout > limit {
                    let k = voltage.max(1.0) / (LIMIT_SHARPNESS * limit);
                    regulating.offset += k * limit;
                    regulating.d_iout = -k;
                    regulating.limiting = true;
                }
            }
            let pass = ConverterPiece {
                offset: -params.dropout,
                d_vin: 1.0,
                d_iout: -params.dropout_resistance,
                region: ConverterRegion::Dropout,
                limiting: false,
            };
            if pass.vout(vin, iout) > regulating.vout(vin, iout) {
                pass
            } else {
                regulating
            }
        }
    }
}

/// Where a converter changes piece, as signed distances at input `vin` and
/// output current `iout` (from the ground pin). [`converter_piece`] switches
/// only where one of these changes sign. Each is linear in `vin` and
/// `iout`, so a straight step between two operating points crosses each at
/// most once. An edge the parameters don't have is `+∞`.
pub fn converter_edges(
    params: &ConverterParams,
    topology: ConverterTopology,
    voltage: f64,
    vin: f64,
    iout: f64,
) -> [f64; 5] {
    let lockout = vin - params.lockout();
    let limit = params
        .current_limit
        .filter(|l| *l > 0.0)
        .map_or(f64::INFINITY, |l| iout - l);
    match topology {
        ConverterTopology::Buck => {
            let [headroom, floor, limit, ..] =
                regulator_edges(&params.buck_output(), voltage, vin, iout);
            [lockout, headroom, floor, limit, f64::INFINITY]
        }
        ConverterTopology::Boost => {
            let pass = vin - params.dropout - params.dropout_resistance * iout;
            let limited = params
                .current_limit
                .filter(|l| *l > 0.0)
                .map_or(f64::INFINITY, |l| {
                    let k = voltage.max(1.0) / (LIMIT_SHARPNESS * l);
                    pass - (voltage - k * (iout - l))
                });
            [lockout, pass - voltage, limited, limit, f64::INFINITY]
        }
    }
}

/// The current a converter draws into its input pin (out at its ground
/// pin), linearised: `i` at the operating point and its partials.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct InputCurrent {
    /// Input current (A).
    pub i: f64,
    /// ∂Iin/∂Vin (S).
    pub d_vin: f64,
    /// ∂Iin/∂Vout (S).
    pub d_vout: f64,
    /// ∂Iin/∂Iout (A/A).
    pub d_iout: f64,
}

/// The input current a converter draws at `vin`, `vout` and `iout` (from
/// the ground pin) on `region`: power balance, `Vout·Iout / (η·Vin)`, plus
/// the quiescent current, or nothing when it's off. Power flowing back
/// (`Vout·Iout < 0`) reaches the input as `η·Vout·Iout / Vin`.
pub fn converter_input(
    params: &ConverterParams,
    region: ConverterRegion,
    vin: f64,
    vout: f64,
    iout: f64,
) -> InputCurrent {
    if region == ConverterRegion::Off || vin <= 0.0 {
        return InputCurrent::default();
    }
    let (eta, d_eta) = params.efficiency_at(iout);
    let sign = iout.signum();
    // Pin = Pout·f, f = 1/η forward and η back; f' is its slope in Iout.
    let (f, df) = if vout * iout >= 0.0 {
        (1.0 / eta, -d_eta * sign / (eta * eta))
    } else {
        (eta, d_eta * sign)
    };
    let p_in = vout * iout * f;
    InputCurrent {
        i: p_in / vin + params.quiescent_current,
        d_vin: -p_in / (vin * vin),
        d_vout: iout * f / vin,
        d_iout: vout * (f + iout * df) / vin,
    }
}

/// Power a converter dissipates (W): what it draws, `Vin·Iin`, less what
/// it delivers, `Vout·Iout`.
pub fn converter_dissipation(
    params: &ConverterParams,
    region: ConverterRegion,
    vin: f64,
    vout: f64,
    iout: f64,
) -> f64 {
    vin * converter_input(params, region, vin, vout, iout).i - vout * iout
}

#[cfg(test)]
mod tests {
    use super::*;
    use ConverterTopology::*;

    #[test]
    fn buck_regulates_drops_out_and_locks_out() {
        let p = ConverterParams {
            dropout: 0.3,
            uvlo: 4.0,
            ..Default::default()
        };
        let piece = converter_piece(&p, Buck, 5.0, 12.0, 1.0);
        assert_eq!(piece.region, ConverterRegion::Regulating);
        assert_eq!(piece.vout(12.0, 1.0), 5.0);
        let piece = converter_piece(&p, Buck, 5.0, 5.0, 1.0);
        assert_eq!(piece.region, ConverterRegion::Dropout);
        assert!((piece.vout(5.0, 1.0) - 4.7).abs() < 1e-12);
        let piece = converter_piece(&p, Buck, 5.0, 3.9, 1.0);
        assert_eq!(piece.region, ConverterRegion::Off);
    }

    #[test]
    fn boost_passes_its_input_through_above_its_set_point() {
        let p = ConverterParams {
            dropout: 0.4,
            ..Default::default()
        };
        let piece = converter_piece(&p, Boost, 5.0, 3.0, 0.5);
        assert_eq!(piece.region, ConverterRegion::Regulating);
        assert_eq!(piece.vout(3.0, 0.5), 5.0);
        let piece = converter_piece(&p, Boost, 5.0, 6.0, 0.5);
        assert_eq!(piece.region, ConverterRegion::Dropout);
        assert!((piece.vout(6.0, 0.5) - 5.6).abs() < 1e-12);
    }

    #[test]
    fn current_limit_holds_a_short_to_one_percent() {
        // A boost's output can't fall below its input, so its load holds it
        // above that.
        for (topology, vin, held) in [(Buck, 12.0, 1.0), (Boost, 3.5, 4.0)] {
            let p = ConverterParams {
                current_limit: Some(2.0),
                ..Default::default()
            };
            let piece = converter_piece(&p, topology, 5.0, vin, 2.001);
            assert!(piece.limiting, "{topology:?}");
            let i = (held - piece.offset - piece.d_vin * vin) / piece.d_iout;
            assert!((i - 2.0).abs() < 0.01 * 2.0, "{topology:?}: {i}");
        }
    }

    #[test]
    fn input_current_balances_power() {
        let p = ConverterParams {
            efficiency: 0.8,
            ..Default::default()
        };
        let i = converter_input(&p, ConverterRegion::Regulating, 12.0, 5.0, 2.0);
        assert!((12.0 * i.i * 0.8 - 10.0).abs() < 1e-12);
        // Partials against finite differences.
        let h = 1e-6;
        let at =
            |vin, vout, iout| converter_input(&p, ConverterRegion::Regulating, vin, vout, iout).i;
        assert!(
            (i.d_vin - (at(12.0 + h, 5.0, 2.0) - at(12.0 - h, 5.0, 2.0)) / (2.0 * h)).abs() < 1e-6
        );
        assert!(
            (i.d_vout - (at(12.0, 5.0 + h, 2.0) - at(12.0, 5.0 - h, 2.0)) / (2.0 * h)).abs() < 1e-6
        );
        assert!(
            (i.d_iout - (at(12.0, 5.0, 2.0 + h) - at(12.0, 5.0, 2.0 - h)) / (2.0 * h)).abs() < 1e-6
        );
        let pd = converter_dissipation(&p, ConverterRegion::Regulating, 12.0, 5.0, 2.0);
        assert!((pd - 2.5).abs() < 1e-12, "{pd}");
    }

    #[test]
    fn efficiency_curve_interpolates_and_its_slope_is_right() {
        let p = ConverterParams {
            efficiency_curve: vec![[0.01, 0.6], [0.1, 0.85], [1.0, 0.92], [3.0, 0.86]],
            ..Default::default()
        };
        assert_eq!(p.efficiency_at(0.0), (0.6, 0.0));
        assert_eq!(p.efficiency_at(5.0), (0.86, 0.0));
        let (eta, slope) = p.efficiency_at(2.0);
        assert!((eta - 0.89).abs() < 1e-12 && (slope + 0.03).abs() < 1e-12);
        let at = |iout| converter_input(&p, ConverterRegion::Regulating, 12.0, 5.0, iout).i;
        let i = converter_input(&p, ConverterRegion::Regulating, 12.0, 5.0, 0.5);
        let h = 1e-6;
        assert!((i.d_iout - (at(0.5 + h) - at(0.5 - h)) / (2.0 * h)).abs() < 1e-6);
    }

    #[test]
    fn ripple_estimates() {
        let p = ConverterParams {
            switching_frequency: Some(500e3),
            inductance: Some(10e-6),
            ..Default::default()
        };
        // Buck 12 → 5: 5·(7/12)/(10 µ·500 k) = 0.583 A.
        let r = p.ripple_current(Buck, 12.0, 5.0).unwrap();
        assert!((r - 5.0 * 7.0 / 12.0 / 5.0).abs() < 1e-12);
        // Boost 3.3 → 5: 3.3·(1.7/5)/5 = 0.2244 A.
        let r = p.ripple_current(Boost, 3.3, 5.0).unwrap();
        assert!((r - 3.3 * 1.7 / 5.0 / 5.0).abs() < 1e-12);
        assert_eq!(p.ripple_current(Buck, 3.0, 5.0), None);
    }

    #[test]
    fn edges_change_sign_where_the_piece_does() {
        let p = ConverterParams {
            dropout: 0.3,
            uvlo: 3.0,
            current_limit: Some(1.0),
            ..Default::default()
        };
        for topology in [Buck, Boost] {
            for (a, b) in [
                ((12.0, 0.5), (4.0, 0.5)),
                ((6.0, 0.5), (2.0, 0.5)),
                ((12.0, 0.5), (12.0, 2.0)),
            ] {
                let pa = converter_piece(&p, topology, 5.0, a.0, a.1);
                let pb = converter_piece(&p, topology, 5.0, b.0, b.1);
                if pa != pb {
                    let ea = converter_edges(&p, topology, 5.0, a.0, a.1);
                    let eb = converter_edges(&p, topology, 5.0, b.0, b.1);
                    assert!(
                        ea.iter()
                            .zip(eb)
                            .any(|(x, y)| x.is_finite() && (*x > 0.0) != (y > 0.0)),
                        "{topology:?} {a:?} → {b:?}"
                    );
                }
            }
        }
    }
}
