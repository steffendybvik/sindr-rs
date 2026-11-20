//! Public, fully-evaluated analysis directives extracted from a netlist.
//!
//! These are returned in [`crate::spice::ParsedNetlist::analyses`]. They map 1-to-1
//! onto the analyses that `sindr` knows how to run.

/// One analysis request declared in a netlist (`.op`, `.tran`, `.dc`, `.ac`, `.tf`, `.sens`, `.noise`).
#[derive(Debug, Clone, PartialEq)]
pub enum AnalysisRequest {
    /// `.op` — DC operating-point analysis.
    Op,
    /// `.tran tstep tstop [tstart [tmax]] [uic]` — transient analysis.
    Tran {
        /// Print/integration step in seconds.
        tstep: f64,
        /// Stop time in seconds.
        tstop: f64,
        /// Optional start time (s). Read but not used: sindr's results
        /// start at 0.
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
    /// The inner `Result` is [`SimError::InvalidComponent`](crate::SimError)
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
    /// add [`ParsedNetlist::initial_conditions`](crate::spice::ParsedNetlist::initial_conditions)
    /// with [`with_initial_voltages`](crate::TransientConfig::with_initial_voltages).
    /// `tstart` is not applied: the result holds every step from the first
    /// one on.
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
                tmax,
                uic,
                ..
            } => {
                let start = if *uic {
                    crate::TransientStart::InitialConditions
                } else {
                    crate::TransientStart::OperatingPoint
                };
                let config = crate::TransientConfig::new(*tstop, *tstep).with_start(start);
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
            AnalysisRequest::Tf { output, input } => Some(crate::TfConfig {
                output: output.clone(),
                input_source: input.clone(),
            }),
            _ => None,
        }
    }

    /// The [`NoiseConfig`](crate::NoiseConfig) for a `.noise` request, to
    /// pass to [`solve_noise`](crate::solve_noise); `None` for any other
    /// analysis. A `dec` or `oct` sweep's per-decade or per-octave count
    /// becomes the total over the band, both ends included.
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
        let span = |base: f64| (fstop / fstart).log(base);
        let (num_points, spacing) = match sweep {
            AcSweep::Dec => (
                (*points as f64 * span(10.0)).round() as usize + 1,
                crate::ac_analysis::FrequencySpacing::Logarithmic,
            ),
            AcSweep::Oct => (
                (*points as f64 * span(2.0)).round() as usize + 1,
                crate::ac_analysis::FrequencySpacing::Logarithmic,
            ),
            AcSweep::Lin => (*points, crate::ac_analysis::FrequencySpacing::Linear),
        };
        Some(crate::NoiseConfig {
            output: node.clone(),
            reference: reference.clone(),
            input_source: input.clone(),
            f_start: *fstart,
            f_stop: *fstop,
            num_points,
            spacing,
        })
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
            AnalysisRequest::Sens { output } => Some(crate::SensConfig {
                output: output.clone(),
            }),
            _ => None,
        }
    }
}

/// Frequency-axis spacing for an AC sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    fn malformed_sens_is_an_error() {
        // AC sensitivity's form isn't supported, so it must not parse as DC.
        for card in [".sens", ".sens in", ".sens v(in) ac dec 10 1 1k"] {
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

    #[test]
    fn directives_keep_source_order() {
        let kinds: Vec<&str> = analyses(".ac lin 2 1 2\n.op\n.tran 1u 1m\n.dc V1 0 1 1")
            .iter()
            .map(|a| match a {
                AnalysisRequest::Op => "op",
                AnalysisRequest::Tran { .. } => "tran",
                AnalysisRequest::Dc { .. } => "dc",
                AnalysisRequest::Ac { .. } => "ac",
                AnalysisRequest::Tf { .. } => "tf",
                AnalysisRequest::Sens { .. } => "sens",
                AnalysisRequest::Noise { .. } => "noise",
            })
            .collect();
        assert_eq!(kinds, ["ac", "op", "tran", "dc"]);
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
