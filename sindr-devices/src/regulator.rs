//! Linear voltage regulator: dropout, quiescent current, current limit,
//! line and load regulation.
//!
//! The output is a piecewise-linear function of the input voltage and the
//! output current, both measured from the regulator's ground pin:
//!
//! ```text
//! regulating: Vout = V + line·(Vin − (V + Vdo)) − Rout·Iout
//! dropout:    Vout = Vin − Vdo − Rdo·Iout
//! ```
//!
//! whichever is lower. Past the current limit the output falls steeply, so
//! the current stays within 1 % of the limit into any load, short circuits
//! included. The input draws `Iout + Iq`.
//!
//! A negative set voltage is a negative regulator (7905, 7912): the same
//! characteristic mirrored through the ground pin, so `Vin`, `Vout` and
//! `Iout` all change sign, and "lower" above reads "nearer zero".

/// How steeply the output falls past the current limit: the full set
/// voltage over this fraction of the limit current.
const LIMIT_SHARPNESS: f64 = 0.01;

/// Regulator parameters. [`Default`] is ideal: no dropout, no quiescent
/// current, no limit, perfect regulation, but never more out than in.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RegulatorParams {
    /// Dropout voltage at no load (V): the least `Vin − Vout` it regulates
    /// with.
    pub dropout: f64,
    /// Rise in dropout per ampere of output current (Ω), for a dropout that
    /// grows with load.
    pub dropout_resistance: f64,
    /// Quiescent (ground-pin) current (A), drawn from the input on top of
    /// the output current.
    pub quiescent_current: f64,
    /// Output current limit (A). `None` is unlimited.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub current_limit: Option<f64>,
    /// Line regulation: change in output per volt of input (V/V), around
    /// the input where it just leaves dropout.
    pub line_regulation: f64,
    /// Load regulation as an output resistance (Ω): the output falls this
    /// much per ampere drawn.
    pub output_resistance: f64,
}

impl RegulatorParams {
    /// An LM7805-class 5 V regulator, from typical 78xx datasheets at
    /// 25 °C: 2 V dropout (typical, 1 A), 4.3 mA quiescent
    /// (typical), 2.2 A peak current, line regulation 3 mV over 7–25 V
    /// (typical), load regulation 100 mV over 5 mA–1.5 A (maximum; no
    /// typical is given). Short-circuit foldback (0.23 A at 35 V in) is not
    /// modelled.
    pub fn lm7805() -> Self {
        Self {
            dropout: 2.0,
            dropout_resistance: 0.0,
            quiescent_current: 4.3e-3,
            current_limit: Some(2.2),
            line_regulation: 3e-3 / 18.0,
            output_resistance: 0.1 / 1.495,
        }
    }

    /// An AMS1117-class low-dropout regulator, from its datasheet: 1.1 V
    /// dropout at 0.8 A (typical), 5 mA quiescent (typical), 1.1 A current
    /// limit (typical), line regulation 0.2 % and load regulation 0.4 %
    /// (maxima) of a 3.3 V output, over 4.75–12 V in and 0–0.8 A out.
    pub fn ams1117() -> Self {
        Self {
            dropout: 1.1,
            dropout_resistance: 0.0,
            quiescent_current: 5e-3,
            current_limit: Some(1.1),
            line_regulation: 0.002 * 3.3 / 7.25,
            output_resistance: 0.004 * 3.3 / 0.8,
        }
    }
}

/// Which part of its characteristic a regulator is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegulatorRegion {
    /// Holding its set voltage (less line and load regulation).
    Regulating,
    /// Out of headroom: the output follows the input down.
    Dropout,
}

/// The regulator's output on one linear piece:
/// `Vout = offset + d_vin·Vin + d_iout·Iout`, all from the ground pin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RegulatorLinear {
    /// Constant term (V).
    pub offset: f64,
    /// ∂Vout/∂Vin (V/V).
    pub d_vin: f64,
    /// ∂Vout/∂Iout (Ω), negative.
    pub d_iout: f64,
    /// Regulating or in dropout.
    pub region: RegulatorRegion,
    /// `true` when past the current limit.
    pub limiting: bool,
}

impl RegulatorLinear {
    /// The output voltage this piece gives at `vin`, `iout`.
    pub fn vout(&self, vin: f64, iout: f64) -> f64 {
        self.offset + self.d_vin * vin + self.d_iout * iout
    }
}

/// The piece of a regulator set to `voltage` that applies at input `vin`
/// (V, from the ground pin) and output current `iout` (A, out of the output
/// pin).
pub fn regulator_piece(
    params: &RegulatorParams,
    voltage: f64,
    vin: f64,
    iout: f64,
) -> RegulatorLinear {
    if voltage < 0.0 {
        // Mirror through the ground pin: −Vout = f(−Vin, −Iout).
        let p = regulator_piece(params, -voltage, -vin, -iout);
        return RegulatorLinear {
            offset: -p.offset,
            ..p
        };
    }
    let regulating = RegulatorLinear {
        offset: voltage - params.line_regulation * (voltage + params.dropout),
        d_vin: params.line_regulation,
        d_iout: -params.output_resistance,
        region: RegulatorRegion::Regulating,
        limiting: false,
    };
    let dropout = RegulatorLinear {
        offset: -params.dropout,
        d_vin: 1.0,
        d_iout: -params.dropout_resistance,
        region: RegulatorRegion::Dropout,
        limiting: false,
    };
    // With no input left the pass element is off: the output can't be
    // driven below the ground pin, only pulled there by its load. The floor
    // meets the dropout piece at `Vin = dropout`, so the characteristic
    // stays continuous.
    let off = RegulatorLinear {
        offset: 0.0,
        d_vin: 0.0,
        ..dropout
    };
    let mut piece = if dropout.vout(vin, iout) >= regulating.vout(vin, iout) {
        regulating
    } else if dropout.vout(vin, iout) < off.vout(vin, iout) {
        off
    } else {
        dropout
    };
    if let Some(limit) = params.current_limit.filter(|l| *l > 0.0) {
        if iout > limit {
            let k = voltage.abs().max(1.0) / (LIMIT_SHARPNESS * limit);
            piece.offset += k * limit;
            piece.d_iout -= k;
            piece.limiting = true;
        }
    }
    piece
}

/// Where a regulator set to `voltage` changes piece, as signed distances
/// at input `vin` and output current `iout` (from the ground pin): leaving
/// regulation, reaching the 0 V floor, the current limit, and the two ends
/// of the quiescent-current ramp. [`regulator_piece`] and
/// [`regulator_quiescent`] switch only where one of these changes sign.
/// Each is linear in `vin` and `iout`, so a straight step between two
/// operating points crosses each at most once. An edge the parameters
/// don't have (no current limit) is `+∞`.
pub fn regulator_edges(params: &RegulatorParams, voltage: f64, vin: f64, iout: f64) -> [f64; 5] {
    if voltage < 0.0 {
        return regulator_edges(params, -voltage, -vin, -iout);
    }
    let regulating = voltage + params.line_regulation * (vin - (voltage + params.dropout))
        - params.output_resistance * iout;
    let dropout = vin - params.dropout - params.dropout_resistance * iout;
    let limit = params
        .current_limit
        .filter(|l| *l > 0.0)
        .map_or(f64::INFINITY, |l| iout - l);
    [
        dropout - regulating,
        vin - params.dropout,
        limit,
        vin,
        vin - params.dropout.max(0.1),
    ]
}

/// The quiescent current a regulator set to `voltage` draws from its input
/// at `vin` (V, from the ground pin), as `i0 + g·Vin` (A, into the input and
/// out at the ground pin). It is the full `quiescent_current` once the input
/// reaches the dropout voltage (0.1 V at least), ramps linearly to 0 below
/// that, and is 0 with no input. Mirrored for a negative `voltage`.
pub fn regulator_quiescent(params: &RegulatorParams, voltage: f64, vin: f64) -> (f64, f64) {
    if voltage < 0.0 {
        let (i0, g) = regulator_quiescent(params, -voltage, -vin);
        return (-i0, g);
    }
    let iq = params.quiescent_current;
    let knee = params.dropout.max(0.1);
    if vin <= 0.0 {
        (0.0, 0.0)
    } else if vin >= knee {
        (iq, 0.0)
    } else {
        (0.0, iq / knee)
    }
}

/// Power a regulator set to `voltage` dissipates (W): `(Vin − Vout)·Iout +
/// Vin·Iq`, with voltages from the ground pin and `Iq` the quiescent current
/// drawn at `vin` ([`regulator_quiescent`]). Both terms are positive for a
/// negative regulator too, where `Vin`, `Vout`, `Iout` and `Iq` are all
/// negative.
pub fn regulator_dissipation(
    params: &RegulatorParams,
    voltage: f64,
    vin: f64,
    vout: f64,
    iout: f64,
) -> f64 {
    let (i0, g) = regulator_quiescent(params, voltage, vin);
    (vin - vout) * iout + vin * (i0 + g * vin)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn regulates_with_headroom_and_drops_out_without() {
        let p = RegulatorParams::lm7805();
        let piece = regulator_piece(&p, 5.0, 12.0, 0.5);
        assert_eq!(piece.region, RegulatorRegion::Regulating);
        assert!((piece.vout(12.0, 0.5) - 5.0).abs() < 0.05);

        let piece = regulator_piece(&p, 5.0, 6.0, 0.5);
        assert_eq!(piece.region, RegulatorRegion::Dropout);
        assert!((piece.vout(6.0, 0.5) - 4.0).abs() < 1e-12);
    }

    #[test]
    fn current_limit_holds_a_short_to_one_percent() {
        let p = RegulatorParams::lm7805();
        // Into a short, Vout = 0 at the current the limited piece gives.
        let piece = regulator_piece(&p, 5.0, 12.0, 3.0);
        assert!(piece.limiting);
        let i_short = -piece.offset_at_vin(12.0) / piece.d_iout;
        assert!((i_short - 2.2).abs() < 0.01 * 2.2, "{i_short}");
    }

    #[test]
    fn ideal_never_boosts() {
        let p = RegulatorParams::default();
        let piece = regulator_piece(&p, 5.0, 3.0, 0.1);
        assert_eq!(piece.region, RegulatorRegion::Dropout);
        assert_eq!(piece.vout(3.0, 0.1), 3.0);
    }

    #[test]
    fn negative_regulator_mirrors_the_positive_one() {
        let p = RegulatorParams::lm7805();
        for (vin, iout) in [(12.0, 0.5), (6.0, 0.5), (12.0, 3.0), (3.0, 0.0)] {
            let pos = regulator_piece(&p, 5.0, vin, iout);
            let neg = regulator_piece(&p, -5.0, -vin, -iout);
            assert_eq!(neg.region, pos.region);
            assert_eq!(neg.limiting, pos.limiting);
            assert!((neg.vout(-vin, -iout) + pos.vout(vin, iout)).abs() < 1e-12);
        }
        let piece = regulator_piece(&p, -5.0, -12.0, -0.5);
        assert_eq!(piece.region, RegulatorRegion::Regulating);
        assert!((piece.vout(-12.0, -0.5) + 5.0).abs() < 0.05);
        let pd = regulator_dissipation(&p, -5.0, -12.0, -5.0, -0.5);
        assert!((pd - (3.5 + 12.0 * 4.3e-3)).abs() < 1e-12, "{pd}");
    }

    impl RegulatorLinear {
        fn offset_at_vin(&self, vin: f64) -> f64 {
            self.offset + self.d_vin * vin
        }
    }
}
