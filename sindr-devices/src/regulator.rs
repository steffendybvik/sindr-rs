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

/// How steeply the output falls past the current limit: the full set
/// voltage over this fraction of the limit current.
const LIMIT_SHARPNESS: f64 = 0.01;

/// Regulator parameters. [`Default`] is ideal: no dropout, no quiescent
/// current, no limit, perfect regulation, but never more out than in.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
    /// An LM7805-class 5 V regulator, from the ST L78 and Fairchild LM78xx
    /// datasheets at 25 °C: 2 V dropout (typical, 1 A), 4.3 mA quiescent
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
    let mut piece = if dropout.vout(vin, iout) < regulating.vout(vin, iout) {
        dropout
    } else {
        regulating
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

/// Power the regulator dissipates (W): `(Vin − Vout)·Iout + Vin·Iq`, with
/// voltages from the ground pin.
pub fn regulator_dissipation(params: &RegulatorParams, vin: f64, vout: f64, iout: f64) -> f64 {
    (vin - vout) * iout + vin * params.quiescent_current
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

    impl RegulatorLinear {
        fn offset_at_vin(&self, vin: f64) -> f64 {
            self.offset + self.d_vin * vin
        }
    }
}
