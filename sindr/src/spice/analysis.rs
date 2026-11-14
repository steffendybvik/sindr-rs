//! Public, fully-evaluated analysis directives extracted from a netlist.
//!
//! These are returned in [`crate::spice::ParsedNetlist::analyses`]. They map 1-to-1
//! onto the analyses that `sindr` knows how to run.

/// One analysis request declared in a netlist (`.op`, `.tran`, `.dc`, `.ac`, `.tf`).
#[derive(Debug, Clone, PartialEq)]
pub enum AnalysisRequest {
    /// `.op` — DC operating-point analysis.
    Op,
    /// `.tran tstep tstop [tstart]` — transient analysis.
    Tran {
        /// Print/integration step in seconds.
        tstep: f64,
        /// Stop time in seconds.
        tstop: f64,
        /// Optional start time (`uic`-style not modelled here).
        tstart: Option<f64>,
    },
    /// `.dc <source> start stop step` — DC sweep over a single source.
    Dc {
        /// Source id being swept.
        source: String,
        /// Sweep start value.
        start: f64,
        /// Sweep stop value.
        stop: f64,
        /// Sweep step.
        step: f64,
    },
    /// `.ac <sweep> points fstart fstop` — small-signal AC sweep.
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
}

impl AnalysisRequest {
    /// The [`TransientConfig`](crate::TransientConfig) for a `.tran`
    /// request, to pass to [`solve_transient`](crate::solve_transient);
    /// `None` for any other analysis.
    ///
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
            AnalysisRequest::Tran { tstep, tstop, .. } => {
                Some(crate::TransientConfig::new(*tstop, *tstep))
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spice::parse_str;

    fn analyses(directives: &str) -> Vec<AnalysisRequest> {
        let deck = format!("* analyses\nV1 in 0 1\nR1 in 0 1k\n{directives}\n.end\n");
        parse_str(&deck).unwrap().analyses
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
                    tstart: None
                },
                AnalysisRequest::Tran {
                    tstep: 10e-9,
                    tstop: 2e-3,
                    tstart: Some(0.5e-3)
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
                step: 0.5
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
    fn directives_keep_source_order() {
        let kinds: Vec<&str> = analyses(".ac lin 2 1 2\n.op\n.tran 1u 1m\n.dc V1 0 1 1")
            .iter()
            .map(|a| match a {
                AnalysisRequest::Op => "op",
                AnalysisRequest::Tran { .. } => "tran",
                AnalysisRequest::Dc { .. } => "dc",
                AnalysisRequest::Ac { .. } => "ac",
                AnalysisRequest::Tf { .. } => "tf",
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
                tstart: None
            }]
        );
    }

    #[test]
    fn deck_without_directives_has_none() {
        assert!(analyses("").is_empty());
    }
}
