//! Public, fully-evaluated analysis directives extracted from a netlist.
//!
//! These are returned in [`crate::spice::ParsedNetlist::analyses`]. They map 1-to-1
//! onto the analyses that `sindr` knows how to run.

/// One analysis request declared in a netlist (`.op`, `.tran`, `.dc`, `.ac`, `.tf`, `.sens`, `.noise`, `.pz`).
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum AnalysisRequest {
    /// `.op` — DC operating-point analysis.
    Op,
    /// `.tran tstep tstop [tstart [tmax]] [uic]` — transient analysis.
    Tran {
        /// Print/integration step in seconds.
        tstep: f64,
        /// Stop time in seconds.
        tstop: f64,
        /// Optional start time (s), SPICE's TSTART: the run is integrated
        /// from 0, and the result starts here
        /// ([`with_tstart`](crate::TransientConfig::with_tstart)).
        tstart: Option<f64>,
        /// Optional largest step (s), SPICE's TMAX.
        tmax: Option<f64>,
        /// `uic`: start from the `.ic` values alone rather than the DC
        /// operating point (see [`TransientStart`](crate::TransientStart)).
        uic: bool,
    },
    /// `.dc <source> start stop step [<source2> start2 stop2 step2]` — DC
    /// sweep over one source, or two with the first inside the second.
    ///
    /// The parser checks that each source is a voltage or current source in
    /// the deck, that the two differ, and that each step is non-zero and
    /// points from its start towards its stop (any step when they are
    /// equal). [`dc_axes`](Self::dc_axes) turns it into the axes
    /// [`dc_sweep`](crate::dc_sweep()) and
    /// [`dc_sweep_nested`](crate::dc_sweep_nested) take.
    Dc {
        /// Source id being swept (the inner, faster sweep when there are two).
        source: String,
        /// Sweep start value.
        start: f64,
        /// Sweep stop value.
        stop: f64,
        /// Sweep step.
        step: f64,
        /// The second, outer source, if the card names one.
        outer: Option<DcSweepSpec>,
    },
    /// `.ac <sweep> points fstart fstop` — small-signal AC sweep.
    ///
    /// The parser checks that `points` is in 1..=10⁷, `fstart <= fstop`, and
    /// `fstart` is above 0 for `dec`/`oct` (at least 0 for `lin`).
    Ac {
        /// Sweep style (decade / octave / linear).
        sweep: AcSweep,
        /// Number of points (per decade/octave for log sweeps; total for linear).
        points: usize,
        /// Start frequency in Hz.
        fstart: f64,
        /// Stop frequency in Hz.
        fstop: f64,
    },
    /// `.tf v(node[,ref]) source` or `.tf i(vsource) source` — DC
    /// small-signal transfer function.
    Tf {
        /// What the gain is measured at.
        output: crate::TfOutput,
        /// Input source id.
        input: String,
    },
    /// `.sens v(node[,ref])` or `.sens i(vsource)` — DC sensitivity.
    Sens {
        /// The output whose sensitivities are wanted.
        output: crate::TfOutput,
    },
    /// `.sens v(node[,ref]) ac <sweep> points fstart fstop` or `.sens
    /// i(vsource) ac ...` — AC sensitivity. The sweep is checked as `.ac`'s.
    SensAc {
        /// The output whose sensitivities are wanted.
        output: crate::TfOutput,
        /// Sweep style (decade / octave / linear).
        sweep: AcSweep,
        /// Points per decade or octave, or the total for linear.
        points: usize,
        /// Start frequency in Hz.
        fstart: f64,
        /// Stop frequency in Hz.
        fstop: f64,
    },
    /// `.noise v(node[,ref]) source <sweep> points fstart fstop` — noise
    /// analysis. The parser checks `points` as `.ac` does, and that
    /// `fstart` is above 0.
    Noise {
        /// Output node.
        node: String,
        /// Reference node; ground when `None`.
        reference: Option<String>,
        /// The source noise is referred back to.
        input: String,
        /// Sweep style (decade / octave / linear).
        sweep: AcSweep,
        /// Points per decade or octave, or the total for linear.
        points: usize,
        /// Start frequency in Hz.
        fstart: f64,
        /// Stop frequency in Hz.
        fstop: f64,
    },
    /// `.pz in in_ref out out_ref vol|cur pol|zer|pz` — pole-zero analysis
    /// of the transfer function from the input pair to the output pair.
    Pz {
        /// Positive input node.
        input: String,
        /// Negative input node.
        input_reference: String,
        /// Positive output node.
        output: String,
        /// Negative output node.
        output_reference: String,
        /// `vol` (voltage gain) or `cur` (transimpedance).
        transfer: crate::PzTransfer,
        /// `pol`, `zer` or `pz`.
        find: crate::PzFind,
    },
}

/// The outer source of a two-source `.dc`: `<source> start stop step`.
#[derive(Debug, Clone, PartialEq)]
pub struct DcSweepSpec {
    /// Source id being swept.
    pub source: String,
    /// Sweep start value.
    pub start: f64,
    /// Sweep stop value.
    pub stop: f64,
    /// Sweep step.
    pub step: f64,
}

/// ngspice's default RELTOL. A `dec` / `oct` sweep keeps every point up to
/// `fstop·(1 + ratio·RELTOL)`, so a point that rounding puts just past
/// `fstop` isn't lost.
const NGSPICE_RELTOL: f64 = 1e-3;

/// The frequency grid of a `.ac` (`ac = true`) or `.noise` sweep as ngspice
/// steps it: the last frequency, the point count and the spacing. The first
/// frequency is always `fstart`.
///
/// A `dec` or `oct` sweep multiplies `fstart` by a fixed ratio, 10^(1/points)
/// or 2^(1/points), and keeps every point up to `fstop` (plus the
/// tolerance), so it stops at or a hair past `fstop` only when the band is a
/// whole number of steps, and short of it otherwise. A `.ac dec` over a
/// decade or more is the exception: ngspice splits it into
/// ⌊points·decades⌋ equal steps that land on `fstop`.
fn sweep_grid(
    ac: bool,
    sweep: AcSweep,
    points: usize,
    fstart: f64,
    fstop: f64,
) -> (f64, usize, crate::ac_analysis::FrequencySpacing) {
    use crate::ac_analysis::FrequencySpacing;
    let base: f64 = match sweep {
        AcSweep::Lin => return (fstop, points, FrequencySpacing::Linear),
        AcSweep::Dec => 10.0,
        AcSweep::Oct => 2.0,
    };
    if fstart == fstop {
        return (fstart, 1, FrequencySpacing::Logarithmic);
    }
    let ratio = fstop / fstart;
    let steps_to_fstop = (ac && sweep == AcSweep::Dec && fstop / 10.0 >= fstart)
        .then(|| (points as f64 * ratio.log10()).floor());
    let ln_delta = match steps_to_fstop {
        Some(steps) => ratio.ln() / steps,
        None => base.ln() / points as f64,
    };
    let delta = ln_delta.exp();
    let n = ((ratio * (1.0 + delta * NGSPICE_RELTOL)).ln() / ln_delta).floor() as usize;
    let n = n.saturating_add(1);
    let last = if steps_to_fstop == Some((n - 1) as f64) {
        fstop
    } else {
        fstart * ((n - 1) as f64 * ln_delta).exp()
    };
    (last, n, FrequencySpacing::Logarithmic)
}

/// An `AC [mag [phase]]` spec on a `V` or `I` line: the small-signal
/// stimulus a deck's `.ac` drives. A bare `AC` is 1 at 0°, as in ngspice.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub struct AcStimulus {
    /// Id of the source as the flattened circuit names it (`v1`, `x1.v2`).
    pub source: String,
    /// `true` for an `I` line (a current source), `false` for a `V` line.
    pub current: bool,
    /// Magnitude: volts for a `V` line, amps for an `I` line.
    pub magnitude: f64,
    /// Phase in degrees.
    pub phase_deg: f64,
}

impl AnalysisRequest {
    /// The sweep axes of a `.dc` request: the inner source, and the outer
    /// one if the card names two. `None` for any other analysis.
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// let deck = "* family\nV1 a 0 0\nV2 b 0 0\nR1 a b 1k\n.dc V1 0 1 0.1 V2 0 2 1\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// let (inner, outer) = netlist.analyses[0].dc_axes().unwrap()?;
    /// assert_eq!(inner.num_points, 11);
    /// let sweep = sindr::dc_sweep_nested(&netlist.circuit, &inner, &outer.unwrap())?;
    /// assert_eq!(sweep.curves.len(), 3);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    ///
    /// # Errors
    ///
    /// The inner `Result` is [`SimError::InvalidConfig`](crate::SimError::InvalidConfig)
    /// for an axis of more than 10⁷ points.
    #[allow(clippy::type_complexity)]
    pub fn dc_axes(
        &self,
    ) -> Option<
        Result<
            (
                crate::dc_sweep::DcSweepAxis,
                Option<crate::dc_sweep::DcSweepAxis>,
            ),
            crate::SimError,
        >,
    > {
        use crate::dc_sweep::DcSweepAxis;
        let AnalysisRequest::Dc {
            source,
            start,
            stop,
            step,
            outer,
        } = self
        else {
            return None;
        };
        Some((|| {
            let inner = DcSweepAxis::from_step(source.clone(), *start, *stop, *step)?;
            let outer = match outer {
                Some(o) => Some(DcSweepAxis::from_step(
                    o.source.clone(),
                    o.start,
                    o.stop,
                    o.step,
                )?),
                None => None,
            };
            Ok((inner, outer))
        })())
    }

    /// The [`TransientConfig`](crate::TransientConfig) for a `.tran`
    /// request, to pass to [`solve_transient`](crate::solve_transient);
    /// `None` for any other analysis.
    ///
    /// As SPICE, the run starts from the DC operating point
    /// ([`TransientStart::OperatingPoint`](crate::TransientStart::OperatingPoint)),
    /// or with `uic` from the initial conditions alone
    /// ([`TransientStart::InitialConditions`](crate::TransientStart::InitialConditions)).
    /// `tmax` caps the step. The deck's `.ic` values aren't in the config:
    /// [`ParsedNetlist::transient_config`](crate::spice::ParsedNetlist::transient_config)
    /// adds them.
    /// `tstart` keeps the result from that time on
    /// ([`with_tstart`](crate::TransientConfig::with_tstart)).
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// let deck = "* rc\nV1 in 0 1\nR1 in out 1k\nC1 out 0 1u\n.tran 1u 10m\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// let config = netlist.analyses[0].transient_config().unwrap();
    /// let result = sindr::solve_transient(&netlist.circuit, &config)?;
    /// let last = result.transient.unwrap().timesteps.last().unwrap().time;
    /// assert!((last - 10e-3).abs() < 1e-12);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn transient_config(&self) -> Option<crate::TransientConfig> {
        match self {
            AnalysisRequest::Tran {
                tstep,
                tstop,
                tstart,
                tmax,
                uic,
            } => {
                let start = if *uic {
                    crate::TransientStart::InitialConditions
                } else {
                    crate::TransientStart::OperatingPoint
                };
                let config = crate::TransientConfig::new(*tstop, *tstep)
                    .with_start(start)
                    .with_tstart(tstart.unwrap_or(0.0));
                Some(match tmax {
                    Some(tmax) => config.with_tmax(*tmax),
                    None => config,
                })
            }
            _ => None,
        }
    }

    /// The [`TfConfig`](crate::TfConfig) for a `.tf` request, to pass to
    /// [`solve_tf`](crate::solve_tf); `None` for any other analysis.
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// let deck = "* divider\nV1 in 0 1\nR1 in out 1k\nR2 out 0 3k\n.tf v(out) v1\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// let config = netlist.analyses[0].tf_config().unwrap();
    /// let tf = sindr::solve_tf(&netlist.circuit, &config)?;
    /// assert!((tf.gain - 0.75).abs() < 1e-12);
    /// assert!((tf.output_resistance - 750.0).abs() < 1e-9);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn tf_config(&self) -> Option<crate::TfConfig> {
        match self {
            AnalysisRequest::Tf { output, input } => {
                Some(crate::TfConfig::new(output.clone(), input.clone()))
            }
            _ => None,
        }
    }

    /// The [`AcConfig`](crate::ac_analysis::AcConfig) for an `.ac` request,
    /// driving `source_id` at 1 V and 0°; `None` for any other analysis.
    /// [`ParsedNetlist::ac_config`](crate::spice::ParsedNetlist::ac_config)
    /// takes the source and its stimulus from the deck's `AC` spec instead.
    ///
    /// The frequencies are ngspice's. A `lin` sweep is `points` from
    /// `fstart` to `fstop`. A `dec` sweep over a decade or more is
    /// ⌊points·decades⌋ equal log steps from `fstart` to `fstop`, so
    /// `.ac dec 10 1 50` is 17 points. Any other `dec` or `oct` sweep steps
    /// from `fstart` by 10^(1/points) or 2^(1/points) and stops at the last
    /// point not past `fstop` (within ngspice's 0.1 % tolerance), so the
    /// config's `f_stop` can fall short of the card's: `.ac oct 3 1 10`
    /// ends at 8 Hz.
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// let deck = "* rc\nV1 in 0 0\nR1 in out 1k\nC1 out 0 1u\n.ac dec 10 1 100k\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// let config = netlist.analyses[0].ac_config("v1").unwrap();
    /// assert_eq!(config.num_points, 51);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn ac_config(&self, source_id: &str) -> Option<crate::ac_analysis::AcConfig> {
        let AnalysisRequest::Ac {
            sweep,
            points,
            fstart,
            fstop,
        } = self
        else {
            return None;
        };
        let (f_stop, num_points, spacing) = sweep_grid(true, *sweep, *points, *fstart, *fstop);
        Some(
            crate::ac_analysis::AcConfig::new(source_id, *fstart, f_stop, num_points)
                .with_spacing(spacing),
        )
    }

    /// The [`NoiseConfig`](crate::NoiseConfig) for a `.noise` request, to
    /// pass to [`solve_noise`](crate::solve_noise); `None` for any other
    /// analysis.
    ///
    /// The frequencies are ngspice's: a `lin` sweep is `points` from
    /// `fstart` to `fstop`; a `dec` or `oct` sweep steps from `fstart` by
    /// 10^(1/points) or 2^(1/points) and stops at the last point not past
    /// `fstop` (within ngspice's 0.1 % tolerance). The config's `f_stop` is
    /// that last point, so `.noise ... dec 10 1 50` ends at 10^1.6 ≈ 39.8 Hz.
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// let deck = "* rc\nV1 in 0 0\nR1 in out 1k\nC1 out 0 1u\n.noise v(out) v1 dec 10 1 100k\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// let config = netlist.analyses[0].noise_config().unwrap();
    /// assert_eq!(config.num_points, 51);
    /// let noise = sindr::solve_noise(&netlist.circuit, &config)?;
    /// // 1 kΩ's thermal noise, √(4kTR), well below the pole.
    /// assert!((noise.points[0].output_density() - 4.0713e-9).abs() < 1e-12);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn noise_config(&self) -> Option<crate::NoiseConfig> {
        let AnalysisRequest::Noise {
            node,
            reference,
            input,
            sweep,
            points,
            fstart,
            fstop,
        } = self
        else {
            return None;
        };
        let (f_stop, num_points, spacing) = sweep_grid(false, *sweep, *points, *fstart, *fstop);
        Some(crate::NoiseConfig {
            output: node.into(),
            reference: reference.clone().map(Into::into),
            input_source: input.into(),
            f_start: *fstart,
            f_stop,
            num_points,
            spacing,
        })
    }

    /// The [`PzConfig`](crate::PzConfig) for a `.pz` request, to pass to
    /// [`solve_pz`](crate::solve_pz); `None` for any other analysis.
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// let deck = "* rc\nV1 in 0 0\nR1 in out 1k\nC1 out 0 1u\n.pz in 0 out 0 vol pz\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// let config = netlist.analyses[0].pz_config().unwrap();
    /// let pz = sindr::solve_pz(&netlist.circuit, &config)?;
    /// // One pole at −1/RC.
    /// assert!((pz.poles[0].re + 1_000.0).abs() < 1e-9);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn pz_config(&self) -> Option<crate::PzConfig> {
        let AnalysisRequest::Pz {
            input,
            input_reference,
            output,
            output_reference,
            transfer,
            find,
        } = self
        else {
            return None;
        };
        let config = match transfer {
            crate::PzTransfer::Transimpedance => crate::PzConfig::transimpedance(
                input.as_str(),
                input_reference.as_str(),
                output.as_str(),
                output_reference.as_str(),
            ),
            _ => crate::PzConfig::voltage_gain(
                input.as_str(),
                input_reference.as_str(),
                output.as_str(),
                output_reference.as_str(),
            ),
        };
        Some(config.with_find(*find))
    }

    /// The [`SensConfig`](crate::SensConfig) for a `.sens` request, to pass
    /// to [`solve_sensitivity`](crate::solve_sensitivity); `None` for any
    /// other analysis.
    ///
    /// ```
    /// use sindr::spice::parse_str;
    ///
    /// let deck = "* divider\nV1 in 0 4\nR1 in out 1k\nR2 out 0 3k\n.sens v(out)\n.end\n";
    /// let netlist = parse_str(deck)?;
    /// let config = netlist.analyses[0].sens_config().unwrap();
    /// let sens = sindr::solve_sensitivity(&netlist.circuit, &config)?;
    /// assert!((sens.get("v1").unwrap().sensitivity - 0.75).abs() < 1e-12);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn sens_config(&self) -> Option<crate::SensConfig> {
        match self {
            AnalysisRequest::Sens { output } => Some(crate::SensConfig::new(output.clone())),
            _ => None,
        }
    }

    /// The [`AcSensConfig`](crate::AcSensConfig) for a `.sens ... ac`
    /// request, with the AC stimulus on `source_id` at 1 V and 0°, to pass to
    /// [`solve_ac_sensitivity`](crate::solve_ac_sensitivity); `None` for any
    /// other analysis. The frequencies are those of the same `.ac` card
    /// ([`ac_config`](Self::ac_config)).
    /// [`ParsedNetlist::ac_sens_config`](crate::spice::ParsedNetlist::ac_sens_config)
    /// takes the stimulus from the deck's `AC` spec instead.
    pub fn ac_sens_config(&self, source_id: &str) -> Option<crate::AcSensConfig> {
        let AnalysisRequest::SensAc {
            output,
            sweep,
            points,
            fstart,
            fstop,
        } = self
        else {
            return None;
        };
        let sweep = AnalysisRequest::Ac {
            sweep: *sweep,
            points: *points,
            fstart: *fstart,
            fstop: *fstop,
        };
        let ac = sweep.ac_config(source_id)?;
        Some(crate::AcSensConfig::new(output.clone(), ac))
    }
}

/// Frequency-axis spacing for an AC sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum AcSweep {
    /// `dec` — points per decade, log spacing.
    Dec,
    /// `oct` — points per octave, log spacing.
    Oct,
    /// `lin` — total points, linear spacing.
    Lin,
}

/// Which analysis a `.meas` card measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MeasureAnalysis {
    /// `.meas tran`: on the `.tran` result, x is time (s).
    Tran,
    /// `.meas ac`: on the `.ac` result, x is frequency (Hz).
    Ac,
    /// `.meas dc`: on the `.dc` sweep, x is the swept source's value.
    Dc,
}

/// A `.meas` card: the analysis it applies to and the measurement.
///
/// Take it with [`Measurement::on`](crate::measure::Measurement::on) on that
/// analysis's result, or wrap it in a [`Spec`](crate::measure::Spec) to give
/// it limits.
///
/// ```
/// use sindr::spice::{parse_str, MeasureAnalysis};
///
/// let deck = "* rc\nV1 in 0 1\nR1 in out 1k\nC1 out 0 1u\n.tran 10u 5m uic\n\
///             .meas tran t_half when v(out)=0.5 rise=1\n.end\n";
/// let netlist = parse_str(deck)?;
/// let meas = &netlist.measurements[0];
/// assert_eq!(meas.analysis, MeasureAnalysis::Tran);
/// let result = sindr::solve_transient(&netlist.circuit, &netlist.analyses[0].transient_config().unwrap())?;
/// // An RC step crosses half way at τ·ln 2.
/// let t = meas.measurement.on(&result)?;
/// assert!((t - 1e-3 * std::f64::consts::LN_2).abs() < 2e-5);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct MeasureRequest {
    /// The analysis whose result it measures.
    pub analysis: MeasureAnalysis,
    /// The measurement, named as the card names it.
    pub measurement: crate::measure::Measurement,
}

/// A signal on a `.four` card and the Fourier analysis to run on it.
///
/// `.four 1k v(out) i(r1)` gives two, one per signal, each with ngspice's
/// defaults at the card's fundamental: 9 harmonics above DC on a
/// 200-point grid. Run it with
/// [`Fourier::on`](crate::fourier::Fourier::on) on the deck's `.tran`
/// result.
///
/// ```
/// use sindr::spice::parse_str;
///
/// let deck = "* divider\nV1 in 0 SIN(0 1 1k)\nR1 in out 1k\nR2 out 0 1k\n\
///             .tran 5u 2m\n.four 1k v(out)\n.end\n";
/// let netlist = parse_str(deck)?;
/// let four = &netlist.fourier[0];
/// let result = sindr::solve_transient(&netlist.circuit, &netlist.transient_config().unwrap())?;
/// let harmonics = four.fourier.on(&result, &four.signal)?;
/// assert!((harmonics.harmonic(1).unwrap().magnitude - 0.5).abs() < 1e-3);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct FourierRequest {
    /// The voltage or current to analyse.
    pub signal: crate::measure::Signal,
    /// The analysis at the card's fundamental.
    pub fourier: crate::fourier::Fourier,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spice::parse_str;

    fn analyses(directives: &str) -> Vec<AnalysisRequest> {
        let deck = format!("* analyses\nV1 in 0 1\nR1 in 0 1k\n{directives}\n.end\n");
        parse_str(&deck).unwrap().analyses
    }

    fn measures(cards: &str) -> Vec<MeasureRequest> {
        let deck = format!(
            "* meas\nV1 in 0 1\nR1 in out 1k\nC1 out 0 1u\n.param tend=2m\n{cards}\n.end\n"
        );
        parse_str(&deck).unwrap().measurements
    }

    fn meas_error(card: &str) -> String {
        let deck = format!("* meas\nV1 in 0 1\nR1 in 0 1k\n{card}\n.end\n");
        parse_str(&deck).unwrap_err().to_string()
    }

    #[test]
    fn meas_statistics_with_a_window() {
        use crate::measure::{Measure, Measurement, Signal, Stat};
        let m = measures(
            ".MEAS TRAN Vavg AVG V(out) FROM=1m TO={tend}\n.measure tran ripple pp v(out,in)",
        );
        assert_eq!(m.len(), 2);
        assert_eq!(m[0].analysis, MeasureAnalysis::Tran);
        assert_eq!(
            m[0].measurement,
            Measurement::new("vavg", Measure::stat(Signal::v("out"), Stat::Avg)).window(1e-3, 2e-3)
        );
        assert_eq!(
            m[1].measurement,
            Measurement::new(
                "ripple",
                Measure::stat(Signal::v_diff("out", "in"), Stat::PeakToPeak)
            )
        );
    }

    #[test]
    fn meas_find_when_and_trig_targ() {
        use crate::measure::{Crossing, Measure, Signal};
        let m = measures(
            ".meas tran v1m find v(out) at=1m\n\
             .meas ac bw when vdb(out)=-3 fall=1\n\
             .meas tran last_cross when i(r1) = 0.5m cross=last\n\
             .meas tran tpd trig v(in) val=0.5 rise=1 targ v(out) val=0.5 rise=2",
        );
        assert_eq!(
            m[0].measurement.measure,
            Measure::find(Signal::v("out"), 1e-3)
        );
        assert_eq!(m[1].analysis, MeasureAnalysis::Ac);
        assert_eq!(
            m[1].measurement.measure,
            Measure::When(Crossing::new(Signal::v("out").db(), -3.0).falling())
        );
        assert_eq!(
            m[2].measurement.measure,
            Measure::When(Crossing::new(Signal::i("r1"), 0.5e-3).last())
        );
        assert_eq!(
            m[3].measurement.measure,
            Measure::Delay {
                trig: Crossing::new(Signal::v("in"), 0.5).rising(),
                targ: Crossing::new(Signal::v("out"), 0.5).rising().nth(2),
            }
        );
    }

    #[test]
    fn meas_rejects_what_it_cant_measure() {
        for (card, expect) in [
            (".meas op x max v(in)", "not `op`"),
            (".meas tran x max", "expected a signal"),
            (".meas tran x max out", "expected a signal"),
            (".meas tran x find v(in) when v(in)=1", "isn't supported"),
            (".meas tran x find v(in)", "needs `at=<x>`"),
            (".meas tran x when v(in)=v(out)", "crossing another signal"),
            (".meas tran x when v(in)=1 rise=0", "count from 1"),
            (".meas tran x when v(in)=1 rise=1 fall=1", "only one of"),
            (".meas tran x trig v(in) val=1", "needs a `targ`"),
            (".meas tran x max v(in) td=1", "`td=` isn't supported"),
            (".meas tran x deriv v(in) at=1", "aren't supported"),
            (".meas ac x max vr(in)", "real and imaginary"),
        ] {
            let err = meas_error(card);
            assert!(err.contains(expect), "{card}: {err}");
        }
    }

    #[test]
    fn four_gives_one_request_per_signal() {
        use crate::fourier::Fourier;
        use crate::measure::Signal;
        let deck = "* four\nV1 in 0 1\nR1 in out 1k\nR2 out 0 1k\n.param f=2k\n\
                    .four 1k v(out) i(r1)\n.FOUR {f} vm(in,out)\n.end\n";
        let four = parse_str(deck).unwrap().fourier;
        assert_eq!(
            four,
            vec![
                FourierRequest {
                    signal: Signal::v("out"),
                    fourier: Fourier::new(1e3)
                },
                FourierRequest {
                    signal: Signal::i("r1"),
                    fourier: Fourier::new(1e3)
                },
                FourierRequest {
                    signal: Signal::v_diff("in", "out"),
                    fourier: Fourier::new(2e3)
                },
            ]
        );
    }

    #[test]
    fn four_rejects_what_it_cant_analyse() {
        for (card, expect) in [
            (".four", "expected a fundamental frequency"),
            (".four 1k", "expected a signal"),
            (".four 1k out", "expected a signal such as"),
            (".four 1k vdb(in)", "reads a voltage or a current"),
            (".four 0 v(in)", "positive frequency"),
            (".four {-1k} v(in)", "positive frequency"),
        ] {
            let err = meas_error(card);
            assert!(err.contains(expect), "{card}: {err}");
        }
    }

    #[test]
    fn op() {
        assert_eq!(analyses(".op"), vec![AnalysisRequest::Op]);
    }

    #[test]
    fn tran_with_and_without_tstart() {
        assert_eq!(
            analyses(".tran 1u 1m\n.tran 10n 2m 0.5m"),
            vec![
                AnalysisRequest::Tran {
                    tstep: 1e-6,
                    tstop: 1e-3,
                    tstart: None,
                    tmax: None,
                    uic: false,
                },
                AnalysisRequest::Tran {
                    tstep: 10e-9,
                    tstop: 2e-3,
                    tstart: Some(0.5e-3),
                    tmax: None,
                    uic: false,
                },
            ]
        );
    }

    #[test]
    fn dc_sweep_lowercases_the_source() {
        assert_eq!(
            analyses(".dc V1 0 5 0.5"),
            vec![AnalysisRequest::Dc {
                source: "v1".into(),
                start: 0.0,
                stop: 5.0,
                step: 0.5,
                outer: None,
            }]
        );
    }

    #[test]
    fn dc_sweep_with_two_sources() {
        let deck = "* two\nV1 a 0 1\nV2 b 0 1\nR1 a b 1k\n.dc V1 0 5 0.5 V2 5 0 -1\n.end\n";
        assert_eq!(
            parse_str(deck).unwrap().analyses,
            vec![AnalysisRequest::Dc {
                source: "v1".into(),
                start: 0.0,
                stop: 5.0,
                step: 0.5,
                outer: Some(DcSweepSpec {
                    source: "v2".into(),
                    start: 5.0,
                    stop: 0.0,
                    step: -1.0,
                }),
            }]
        );
    }

    #[test]
    fn ac_sweep_kinds() {
        let ac = |sweep, points| AnalysisRequest::Ac {
            sweep,
            points,
            fstart: 1.0,
            fstop: 1e6,
        };
        assert_eq!(
            analyses(".ac dec 10 1 1meg\n.ac OCT 4 1 1meg\n.ac lin 101 1 1meg"),
            vec![
                ac(AcSweep::Dec, 10),
                ac(AcSweep::Oct, 4),
                ac(AcSweep::Lin, 101)
            ]
        );
    }

    #[test]
    fn tf_outputs() {
        use crate::TfOutput;
        let tf = |output| AnalysisRequest::Tf {
            output,
            input: "v1".into(),
        };
        assert_eq!(
            analyses(".tf v(in) V1\n.tf V( in , 0 ) v1\n.tf I(V1) v1"),
            vec![
                tf(TfOutput::Voltage {
                    node: "in".into(),
                    reference: None
                }),
                tf(TfOutput::Voltage {
                    node: "in".into(),
                    reference: Some("0".into())
                }),
                tf(TfOutput::Current {
                    source_id: "v1".into()
                }),
            ]
        );
    }

    #[test]
    fn sens_outputs() {
        use crate::TfOutput;
        assert_eq!(
            analyses(".sens V(in)\n.sens v(in,0)\n.sens I(V1)"),
            vec![
                AnalysisRequest::Sens {
                    output: TfOutput::Voltage {
                        node: "in".into(),
                        reference: None
                    }
                },
                AnalysisRequest::Sens {
                    output: TfOutput::Voltage {
                        node: "in".into(),
                        reference: Some("0".into())
                    }
                },
                AnalysisRequest::Sens {
                    output: TfOutput::Current {
                        source_id: "v1".into()
                    }
                },
            ]
        );
    }

    #[test]
    fn ac_sens_card() {
        use crate::TfOutput;
        let got = analyses(".sens v(out,in) ac dec 10 1 1meg\n.sens i(v1) ac lin 3 10 30");
        assert_eq!(
            got[0],
            AnalysisRequest::SensAc {
                output: TfOutput::Voltage {
                    node: "out".into(),
                    reference: Some("in".into())
                },
                sweep: AcSweep::Dec,
                points: 10,
                fstart: 1.0,
                fstop: 1e6,
            }
        );
        assert!(got[0].sens_config().is_none());
        let config = got[0].ac_sens_config("v1").unwrap();
        assert_eq!(
            (config.ac.num_points, config.ac.source_id.as_str()),
            (61, "v1")
        );
        let config = got[1].ac_sens_config("v1").unwrap();
        assert_eq!(config.output, TfOutput::current("v1"));
        assert_eq!(
            (config.ac.f_start, config.ac.f_stop, config.ac.num_points),
            (10.0, 30.0, 3)
        );
    }

    #[test]
    fn malformed_sens_is_an_error() {
        for card in [
            ".sens",
            ".sens in",
            ".sens v(in) dc",
            ".sens v(in) ac",
            ".sens v(in) ac log 10 1 1k",
            ".sens v(in) ac dec 10 1",
            ".sens v(in) ac dec 0 1 1k",
            ".sens v(in) ac dec 10 1k 1",
            ".sens v(in) ac dec 10 1 1k extra",
        ] {
            let deck = format!("* sens\nV1 in 0 1\nR1 in 0 1k\n{card}\n.end\n");
            assert!(parse_str(&deck).is_err(), "{card}");
        }
    }

    #[test]
    fn malformed_tf_is_an_error() {
        for card in [
            ".tf v(in)",
            ".tf in v1",
            ".tf v(in v1",
            ".tf v(in) v1 extra",
        ] {
            let deck = format!("* tf\nV1 in 0 1\nR1 in 0 1k\n{card}\n.end\n");
            assert!(parse_str(&deck).is_err(), "{card}");
        }
    }

    #[test]
    fn noise_card() {
        assert_eq!(
            analyses(".noise V(out, 0) V1 oct 4 10 1k 1"),
            vec![AnalysisRequest::Noise {
                node: "out".into(),
                reference: Some("0".into()),
                input: "v1".into(),
                sweep: AcSweep::Oct,
                points: 4,
                fstart: 10.0,
                fstop: 1e3,
            }]
        );
        let config = analyses(".noise v(in) v1 oct 4 10 1280")[0]
            .noise_config()
            .unwrap();
        // 7 octaves at 4 per octave, both ends.
        assert_eq!(config.num_points, 29);
        for card in [
            ".noise i(v1) v1 dec 10 1 1k",
            ".noise v(in) v1 dec 10 0 1k",
            ".noise v(in) v1 dec 10 1k 1",
            ".noise v(in) v1 log 10 1 1k",
        ] {
            let deck = format!("* noise\nV1 in 0 1\nR1 in 0 1k\n{card}\n.end\n");
            assert!(parse_str(&deck).is_err(), "{card}");
        }
    }

    /// `(first, last, count)` of a deck's `.ac` and `.noise` grids.
    fn grids(sweep: &str) -> ((f64, f64, usize), (f64, f64, usize)) {
        let got = analyses(&format!(".ac {sweep}\n.noise v(in) v1 {sweep}"));
        let ac = got[0].ac_config("v1").unwrap();
        let noise = got[1].noise_config().unwrap();
        (
            (ac.f_start, ac.f_stop, ac.num_points),
            (noise.f_start, noise.f_stop, noise.num_points),
        )
    }

    fn assert_grid(got: (f64, f64, usize), want: (f64, f64, usize), what: &str) {
        assert_eq!(got.2, want.2, "{what}: point count");
        assert_eq!(got.0, want.0, "{what}: first frequency");
        // ngspice prints eight digits.
        assert!(
            (got.1 / want.1 - 1.0).abs() < 1e-8,
            "{what}: last frequency {} vs {}",
            got.1,
            want.1
        );
    }

    /// Each grid is ngspice 47's for the same card (`print frequency`).
    #[test]
    fn sweep_grids_match_ngspice() {
        for (sweep, ac, noise) in [
            // Whole decades: both land on fstop.
            ("dec 10 1 100k", (1.0, 1e5, 51), (1.0, 1e5, 51)),
            // `.ac dec` spreads ⌊10·1.699⌋ = 16 steps over the band;
            // `.noise` steps by 10^0.1 and stops at 10^1.6.
            ("dec 10 1 50", (1.0, 50.0, 17), (1.0, 39.810_717, 17)),
            ("dec 4 1 50", (1.0, 50.0, 7), (1.0, 31.622_776_6, 7)),
            // Under a decade `.ac` steps by 10^(1/points) too.
            ("dec 10 2 3", (2.0, 2.517_850_8, 2), (2.0, 2.517_850_8, 2)),
            ("dec 1 1 5", (1.0, 1.0, 1), (1.0, 1.0, 1)),
            // 10 is within 0.1 %·10 of 9.99, so ngspice keeps it.
            ("dec 1 1 9.99", (1.0, 10.0, 2), (1.0, 10.0, 2)),
            ("dec 1 1 9.9", (1.0, 1.0, 1), (1.0, 1.0, 1)),
            // An octave sweep never stretches its step to reach fstop.
            ("oct 3 1 10", (1.0, 8.0, 10), (1.0, 8.0, 10)),
            ("oct 4 10 1280", (10.0, 1280.0, 29), (10.0, 1280.0, 29)),
            ("lin 5 1 2", (1.0, 2.0, 5), (1.0, 2.0, 5)),
            ("dec 10 5 5", (5.0, 5.0, 1), (5.0, 5.0, 1)),
        ] {
            let (got_ac, got_noise) = grids(sweep);
            assert_grid(got_ac, ac, &format!(".ac {sweep}"));
            assert_grid(got_noise, noise, &format!(".noise {sweep}"));
        }
    }

    /// The frequencies `solve_ac` sweeps for `.ac dec 10 1 50`, as ngspice
    /// 47 prints them: 50^(k/16).
    #[test]
    fn partial_decade_ac_frequencies() {
        let config = analyses(".ac dec 10 1 50")[0].ac_config("v1").unwrap();
        let freqs = crate::ac_analysis::generate_frequencies(&config);
        let ngspice = [
            1.0,
            1.276_984_498_3,
            1.630_689_409_0,
            2.082_365_096_8,
            2.659_147_948_5,
            3.395_690_708_9,
            4.336_244_396_4,
            5.537_316_875_2,
            7.071_067_811_9,
            9.029_643_982_3,
            11.530_715_391,
            14.724_544_809,
            18.803_015_465,
            24.011_159_271,
            30.661_878_176,
            39.154_743_120,
            50.0,
        ];
        assert_eq!(freqs.len(), ngspice.len());
        for (got, want) in freqs.iter().zip(ngspice) {
            assert!((got / want - 1.0).abs() < 1e-10, "{got} vs {want}");
        }
    }

    /// A request built by hand needn't be valid, but its grid mustn't
    /// panic: an inverted band is a single point, and `solve_ac` rejects a
    /// zero `fstart` whatever the count.
    #[test]
    fn sweep_grid_survives_a_hand_built_request() {
        let ac = |fstart, fstop| {
            AnalysisRequest::Ac {
                sweep: AcSweep::Dec,
                points: 10,
                fstart,
                fstop,
            }
            .ac_config("v1")
            .unwrap()
        };
        assert!(ac(0.0, 1e3).num_points >= 1);
        assert_eq!(ac(1e3, 1.0).num_points, 1);
    }

    /// Each `*_config` builds its own analysis's config and `None` for
    /// every other, so a caller can `find_map` over a deck's analyses.
    #[test]
    fn configs_come_only_from_their_own_card() {
        use crate::TfOutput;
        let got = analyses(
            ".op\n.tf v(in) v1\n.sens i(v1)\n.dc v1 0 1 0.5\n.tran 1u 1m\n\
             .ac dec 10 1 1k\n.noise v(in) v1 dec 10 1 1k",
        );
        let found = |f: &dyn Fn(&AnalysisRequest) -> bool| -> Vec<usize> {
            (0..got.len()).filter(|&i| f(&got[i])).collect()
        };
        assert_eq!(found(&|a| a.tf_config().is_some()), [1]);
        assert_eq!(found(&|a| a.sens_config().is_some()), [2]);
        assert_eq!(found(&|a| a.dc_axes().is_some()), [3]);
        assert_eq!(found(&|a| a.transient_config().is_some()), [4]);
        assert_eq!(found(&|a| a.ac_config("v1").is_some()), [5]);
        assert_eq!(found(&|a| a.noise_config().is_some()), [6]);

        let tf = got[1].tf_config().unwrap();
        assert_eq!(tf.input_source, "v1");
        assert_eq!(
            tf.output,
            TfOutput::Voltage {
                node: "in".into(),
                reference: None
            }
        );
        assert_eq!(
            got[2].sens_config().unwrap().output,
            TfOutput::Current {
                source_id: "v1".into()
            }
        );
        let (inner, outer) = got[3].dc_axes().unwrap().unwrap();
        assert_eq!((inner.source_id.as_str(), inner.num_points), ("v1", 3));
        assert!(outer.is_none());
    }

    /// A linear `.ac` sweep's count is the total, as written.
    #[test]
    fn linear_ac_config() {
        let config = analyses(".ac lin 11 0 1k")[0].ac_config("v1").unwrap();
        assert_eq!(
            (config.f_start, config.f_stop, config.num_points),
            (0.0, 1e3, 11)
        );
        assert_eq!(config.spacing, crate::ac_analysis::FrequencySpacing::Linear);
        assert_eq!((config.ac_magnitude, config.ac_phase_deg), (1.0, 0.0));
    }

    #[test]
    fn directives_keep_source_order() {
        let kinds: Vec<&str> = analyses(
            ".ac lin 2 1 2\n.sens v(in)\n.op\n.noise v(in) v1 lin 2 1 2\n.tran 1u 1m\n\
             .tf v(in) v1\n.pz in 0 in 0 vol pz\n.dc V1 0 1 1",
        )
        .iter()
        .map(|a| match a {
            AnalysisRequest::Op => "op",
            AnalysisRequest::Tran { .. } => "tran",
            AnalysisRequest::Dc { .. } => "dc",
            AnalysisRequest::Ac { .. } => "ac",
            AnalysisRequest::Tf { .. } => "tf",
            AnalysisRequest::Sens { .. } => "sens",
            AnalysisRequest::SensAc { .. } => "sens ac",
            AnalysisRequest::Noise { .. } => "noise",
            AnalysisRequest::Pz { .. } => "pz",
        })
        .collect();
        assert_eq!(
            kinds,
            ["ac", "sens", "op", "noise", "tran", "tf", "pz", "dc"]
        );
    }

    #[test]
    fn directive_values_use_params() {
        let got = analyses(".param tstop=2m\n.tran 1u {tstop}");
        assert_eq!(
            got,
            vec![AnalysisRequest::Tran {
                tstep: 1e-6,
                tstop: 2e-3,
                tstart: None,
                tmax: None,
                uic: false,
            }]
        );
    }

    #[test]
    fn deck_without_directives_has_none() {
        assert!(analyses("").is_empty());
    }
}
