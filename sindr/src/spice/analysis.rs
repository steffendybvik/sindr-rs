//! Public, fully-evaluated analysis directives extracted from a netlist.
//!
//! These are returned in [`crate::spice::ParsedNetlist::analyses`]. They map 1-to-1
//! onto the analyses that `sindr` knows how to run.

/// One analysis request declared in a netlist (`.tran`, `.dc`, `.ac`, `.op`).
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
