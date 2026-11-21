//! Measurements and specs: a verdict on a result, and the reason when it
//! fails.
//!
//! A [`Measurement`] turns a result into one number: the value at a time or
//! frequency, a min, max, average or RMS, a crossing time, a delay, a rise
//! or fall time, a settling time or a −3 dB point. A [`Spec`] adds limits to
//! a measurement, and [`check_specs`] reports, for each spec, the measured
//! value, its limits, pass or fail and the margin left. The ratings checks
//! ([`check_ratings`](crate::check_ratings)) can ride along in the same
//! [`SpecReport`].
//!
//! Measurements work on anything [`Measurable`]:
//!
//! | Result | x axis |
//! |---|---|
//! | [`SimulationResult`] with a transient | time (s) |
//! | [`SimulationResult`] without one (DC operating point) | a single point at x = 0 |
//! | [`AcResult`] | frequency (Hz) |
//! | [`DcSweepResult`] | the swept source's value (V or A) |
//! | [`TempSweepResult`] | temperature (K) |
//!
//! A [`Signal`] is a node voltage (`v(out)`), a voltage between two nodes
//! (`v(a, b)`) or a component's current (`i(R1)`, its
//! [`ComponentResult::current_through`](crate::ComponentResult::current_through)
//! direction). Its [`View`] picks the real value (the magnitude on an AC
//! result), dB (`20·log10|x|`) or phase in degrees (AC only, unwrapped
//! along the sweep).
//!
//! SPICE `.meas` cards are parsed onto measurements by the `spice` feature
//! (`ParsedNetlist::measurements`).
//!
//! ```
//! use sindr::measure::{check_specs, Measure, Measurement, Signal, Spec, Stat};
//! use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig};
//!
//! // RC low-pass, τ = 1 ms, charging towards 5 V.
//! let circuit = Circuit::new(
//!     "0",
//!     vec![
//!         CircuitElement::VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 5.0, waveform: None },
//!         CircuitElement::Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()], resistance: 1e3 },
//!         CircuitElement::Capacitor { id: "C1".into(), nodes: ["out".into(), "0".into()], capacitance: 1e-6 },
//!     ],
//! );
//! let result = solve_transient(&circuit, &TransientConfig::new(10e-3, 10e-6))?;
//!
//! let specs = [
//!     Spec::new(Measurement::new("vout_final", Measure::stat(Signal::v("out"), Stat::Final)))
//!         .between(4.9, 5.1),
//!     // 10–90 % rise of an RC is 2.2·τ.
//!     Spec::new(Measurement::new("rise", Measure::rise_time(Signal::v("out")))).at_most(2.5e-3),
//! ];
//! let report = check_specs(&result, &specs);
//! assert!(report.passed(), "{report:?}");
//! let rise = report.checks[1].value.unwrap();
//! assert!((rise - 2.197e-3).abs() < 0.05e-3);
//! # Ok::<(), sindr::SimError>(())
//! ```

use num_complex::Complex64;

use crate::ac_analysis::AcResult;
use crate::dc_sweep::DcSweepResult;
use crate::error::SimError;
use crate::ratings::RatingsReport;
use crate::results::{ComponentResult, SimulationResult};
use crate::temp_sweep::TempSweepResult;

// ---------------------------------------------------------------------------
// Signals
// ---------------------------------------------------------------------------

/// What a signal reads: a voltage or a current.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Quantity {
    /// Voltage at `node` (V), against `reference` or ground.
    Voltage {
        /// Node name.
        node: String,
        /// Reference node; `None` is ground.
        reference: Option<String>,
    },
    /// Current through a component (A), in the direction of its
    /// [`ComponentResult::current_through`](crate::ComponentResult::current_through).
    /// Falls back to [`SimulationResult::branch_currents`] for an element
    /// without a component result. Not available on AC results.
    Current {
        /// Component id, flattened (`X1.R3`).
        component: String,
    },
}

/// How a signal's value is read.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum View {
    /// The value itself; the magnitude on an AC result.
    #[default]
    Value,
    /// `20·log10|x|`, in dB. On AC this is absolute (SPICE's `vdb`), not
    /// relative to the source amplitude.
    Db,
    /// Phase in degrees, AC only, unwrapped along the sweep so a phase that
    /// keeps falling past −180° reads −190°, not +170°.
    Phase,
}

/// A signal to measure: a [`Quantity`] read through a [`View`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Signal {
    /// Voltage or current.
    pub quantity: Quantity,
    /// Value, dB or phase.
    #[cfg_attr(feature = "serde", serde(default))]
    pub view: View,
}

impl Signal {
    /// Voltage at `node` against ground: `v(node)`.
    pub fn v(node: impl Into<String>) -> Self {
        Signal {
            quantity: Quantity::Voltage {
                node: node.into(),
                reference: None,
            },
            view: View::Value,
        }
    }

    /// Voltage between two nodes, `node − reference`: `v(node, reference)`.
    pub fn v_diff(node: impl Into<String>, reference: impl Into<String>) -> Self {
        Signal {
            quantity: Quantity::Voltage {
                node: node.into(),
                reference: Some(reference.into()),
            },
            view: View::Value,
        }
    }

    /// Current through a component: `i(component)`.
    pub fn i(component: impl Into<String>) -> Self {
        Signal {
            quantity: Quantity::Current {
                component: component.into(),
            },
            view: View::Value,
        }
    }

    /// The same signal in dB.
    pub fn db(mut self) -> Self {
        self.view = View::Db;
        self
    }

    /// The same signal's phase, in degrees (AC only).
    pub fn phase(mut self) -> Self {
        self.view = View::Phase;
        self
    }
}

impl std::fmt::Display for Signal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let prefix = match (&self.quantity, self.view) {
            (Quantity::Voltage { .. }, View::Value) => "v",
            (Quantity::Voltage { .. }, View::Db) => "vdb",
            (Quantity::Voltage { .. }, View::Phase) => "vp",
            (Quantity::Current { .. }, View::Value) => "i",
            (Quantity::Current { .. }, View::Db) => "idb",
            (Quantity::Current { .. }, View::Phase) => "ip",
        };
        match &self.quantity {
            Quantity::Voltage {
                node,
                reference: None,
            } => write!(f, "{prefix}({node})"),
            Quantity::Voltage {
                node,
                reference: Some(r),
            } => write!(f, "{prefix}({node},{r})"),
            Quantity::Current { component } => write!(f, "{prefix}({component})"),
        }
    }
}

// ---------------------------------------------------------------------------
// Measurable results
// ---------------------------------------------------------------------------

/// A result a [`Measurement`] can be taken on.
pub trait Measurable {
    /// The signal's samples as `(x, y)` pairs, in ascending x.
    ///
    /// # Errors
    ///
    /// The reason it can't be read: an unknown node, a phase on a
    /// transient.
    fn trace(&self, signal: &Signal) -> Result<Vec<(f64, f64)>, String>;
}

fn voltage_of(
    voltages: &std::collections::HashMap<String, f64>,
    node: &str,
    reference: Option<&str>,
) -> Result<f64, String> {
    let get = |n: &str| {
        voltages
            .get(n)
            .copied()
            .ok_or_else(|| format!("no node `{n}` in the result"))
    };
    let v = get(node)?;
    match reference {
        Some(r) => Ok(v - get(r)?),
        None => Ok(v),
    }
}

fn current_of(
    components: &[ComponentResult],
    branches: Option<&std::collections::HashMap<String, f64>>,
    id: &str,
) -> Result<f64, String> {
    if let Some(c) = components.iter().find(|c| c.id == id) {
        return Ok(c.current_through);
    }
    branches
        .and_then(|b| b.get(id).copied())
        .ok_or_else(|| format!("no component `{id}` with a current in the result"))
}

/// Apply a real-valued view; phase needs AC.
fn real_view(signal: &Signal, y: f64) -> Result<f64, String> {
    match signal.view {
        View::Value => Ok(y),
        View::Db => Ok(20.0 * y.abs().log10()),
        View::Phase => Err(format!(
            "`{signal}` reads a phase, which only an AC result has"
        )),
    }
}

/// Reverse a trace whose x runs downwards (a DC sweep from high to low).
fn ascending(mut trace: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    if trace.len() > 1 && trace[0].0 > trace[trace.len() - 1].0 {
        trace.reverse();
    }
    trace
}

fn op_value(result: &SimulationResult, signal: &Signal) -> Result<f64, String> {
    let y = match &signal.quantity {
        Quantity::Voltage { node, reference } => {
            voltage_of(&result.node_voltages, node, reference.as_deref())?
        }
        Quantity::Current { component } => current_of(
            &result.component_results,
            Some(&result.branch_currents),
            component,
        )?,
    };
    real_view(signal, y)
}

impl Measurable for SimulationResult {
    fn trace(&self, signal: &Signal) -> Result<Vec<(f64, f64)>, String> {
        let Some(transient) = &self.transient else {
            return Ok(vec![(0.0, op_value(self, signal)?)]);
        };
        transient
            .timesteps
            .iter()
            .map(|step| {
                let y = match &signal.quantity {
                    Quantity::Voltage { node, reference } => {
                        voltage_of(&step.node_voltages, node, reference.as_deref())?
                    }
                    Quantity::Current { component } => {
                        current_of(&step.component_results, None, component)?
                    }
                };
                Ok((step.time, real_view(signal, y)?))
            })
            .collect()
    }
}

impl Measurable for DcSweepResult {
    fn trace(&self, signal: &Signal) -> Result<Vec<(f64, f64)>, String> {
        let trace = self
            .points
            .iter()
            .map(|p| Ok((p.sweep_value, op_value(&p.result, signal)?)))
            .collect::<Result<Vec<_>, String>>()?;
        Ok(ascending(trace))
    }
}

impl Measurable for TempSweepResult {
    fn trace(&self, signal: &Signal) -> Result<Vec<(f64, f64)>, String> {
        let trace = self
            .points
            .iter()
            .map(|p| Ok((p.temperature_kelvin, op_value(&p.result, signal)?)))
            .collect::<Result<Vec<_>, String>>()?;
        Ok(ascending(trace))
    }
}

impl Measurable for AcResult {
    fn trace(&self, signal: &Signal) -> Result<Vec<(f64, f64)>, String> {
        let (node, reference) = match &signal.quantity {
            Quantity::Voltage { node, reference } => (node, reference.as_deref()),
            Quantity::Current { .. } => {
                return Err(format!(
                    "`{signal}`: AC results hold node voltages only, not currents"
                ))
            }
        };
        let phasor = |p: &crate::ac_analysis::AcPoint, n: &str| -> Result<Complex64, String> {
            p.node_voltages
                .get(n)
                .copied()
                .ok_or_else(|| format!("no node `{n}` in the result"))
        };
        let mut trace = Vec::with_capacity(self.points.len());
        let mut previous_phase: Option<f64> = None;
        for p in &self.points {
            let mut v = phasor(p, node)?;
            if let Some(r) = reference {
                v -= phasor(p, r)?;
            }
            let y = match signal.view {
                View::Value => v.norm(),
                View::Db => 20.0 * v.norm().log10(),
                View::Phase => {
                    let mut ph = v.arg().to_degrees();
                    if let Some(prev) = previous_phase {
                        ph += 360.0 * ((prev - ph) / 360.0).round();
                    }
                    previous_phase = Some(ph);
                    ph
                }
            };
            trace.push((p.frequency, y));
        }
        Ok(trace)
    }
}

// ---------------------------------------------------------------------------
// Measurements
// ---------------------------------------------------------------------------

/// A statistic over a signal, within the measurement's window.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Stat {
    /// Smallest sample.
    Min,
    /// Largest sample.
    Max,
    /// Mean over x, by trapezoids, so uneven time steps don't skew it.
    Avg,
    /// Root mean square over x, by trapezoids of y².
    Rms,
    /// Largest minus smallest sample.
    PeakToPeak,
    /// Integral over x, by trapezoids (V·s for a voltage on a transient).
    Integral,
    /// The last sample: the end of a transient, the only point of a DC
    /// operating point.
    Final,
}

/// Which way a signal crosses a level.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edge {
    /// Going up through the level.
    Rise,
    /// Going down through the level.
    Fall,
    /// Either way.
    Cross,
}

/// Which crossing counts.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Occurrence {
    /// The nth one, from 1, in ascending x.
    Nth(usize),
    /// The last one.
    Last,
}

/// A signal crossing a level: SPICE's `WHEN v(out)=2.5 RISE=2`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Crossing {
    /// The signal.
    pub signal: Signal,
    /// The level it crosses, in the signal's unit.
    pub level: f64,
    /// Rising, falling or either.
    pub edge: Edge,
    /// Which crossing.
    pub occurrence: Occurrence,
}

impl Crossing {
    /// The first crossing of `level`, either way.
    pub fn new(signal: Signal, level: f64) -> Self {
        Crossing {
            signal,
            level,
            edge: Edge::Cross,
            occurrence: Occurrence::Nth(1),
        }
    }

    /// Only rising crossings count.
    pub fn rising(mut self) -> Self {
        self.edge = Edge::Rise;
        self
    }

    /// Only falling crossings count.
    pub fn falling(mut self) -> Self {
        self.edge = Edge::Fall;
        self
    }

    /// The nth crossing, from 1.
    pub fn nth(mut self, n: usize) -> Self {
        self.occurrence = Occurrence::Nth(n);
        self
    }

    /// The last crossing.
    pub fn last(mut self) -> Self {
        self.occurrence = Occurrence::Last;
        self
    }
}

/// What a [`Measurement`] measures. Each variant carries its own signal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Measure {
    /// The signal's value at `at`, interpolated linearly: SPICE's
    /// `FIND v(out) AT=1m`.
    Find {
        /// The signal.
        signal: Signal,
        /// Where, on the result's x axis.
        at: f64,
    },
    /// A statistic over the window: SPICE's `AVG`, `RMS`, `MIN`, `MAX`,
    /// `PP` and `INTEG`.
    Stat {
        /// The signal.
        signal: Signal,
        /// Which statistic.
        stat: Stat,
    },
    /// The x at which a crossing happens: SPICE's `WHEN`.
    When(Crossing),
    /// `x(targ) − x(trig)`: SPICE's `TRIG … TARG …`, a propagation delay.
    Delay {
        /// Where the interval starts.
        trig: Crossing,
        /// Where it ends.
        targ: Crossing,
    },
    /// Rise or fall time between two fractions of the signal's swing over
    /// the window (0.1 and 0.9 for the usual 10–90 %). For a rise, from the
    /// last time it rose through `low` before it first rose through `high`;
    /// a fall the other way round.
    Transition {
        /// The signal.
        signal: Signal,
        /// [`Edge::Rise`] or [`Edge::Fall`].
        edge: Edge,
        /// Lower fraction of the swing, 0 to 1.
        low: f64,
        /// Upper fraction of the swing, 0 to 1.
        high: f64,
    },
    /// How long, from the start of the window, until the signal stays
    /// within `band` of its final value (or of `target`) to the end.
    Settling {
        /// The signal.
        signal: Signal,
        /// Allowed error either side, in the signal's unit.
        band: f64,
        /// The value it settles to; `None` takes the last sample.
        target: Option<f64>,
    },
    /// The first x after the peak where the signal, in dB, has fallen
    /// `drop_db` below that peak: the −3 dB point of a low-pass or the
    /// upper one of a band-pass, with `drop_db` 3.
    Bandwidth {
        /// The signal; read in dB whatever its view.
        signal: Signal,
        /// How far below the peak, in dB.
        drop_db: f64,
    },
}

impl Measure {
    /// A statistic of a signal.
    pub fn stat(signal: Signal, stat: Stat) -> Self {
        Measure::Stat { signal, stat }
    }

    /// The signal's value at `at`.
    pub fn find(signal: Signal, at: f64) -> Self {
        Measure::Find { signal, at }
    }

    /// 10–90 % rise time.
    pub fn rise_time(signal: Signal) -> Self {
        Measure::Transition {
            signal,
            edge: Edge::Rise,
            low: 0.1,
            high: 0.9,
        }
    }

    /// 90–10 % fall time.
    pub fn fall_time(signal: Signal) -> Self {
        Measure::Transition {
            signal,
            edge: Edge::Fall,
            low: 0.1,
            high: 0.9,
        }
    }

    /// Settling time to within `band` of the final value.
    pub fn settling(signal: Signal, band: f64) -> Self {
        Measure::Settling {
            signal,
            band,
            target: None,
        }
    }

    /// The −3 dB point.
    pub fn bandwidth(signal: Signal) -> Self {
        Measure::Bandwidth {
            signal,
            drop_db: 3.0,
        }
    }
}

/// A named measurement, optionally limited to a window of x.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Measurement {
    /// Name, reported back with the value.
    pub name: String,
    /// What is measured.
    pub measure: Measure,
    /// Start of the window on the x axis; `None` is the start of the result.
    #[cfg_attr(feature = "serde", serde(default))]
    pub from: Option<f64>,
    /// End of the window; `None` is the end of the result.
    #[cfg_attr(feature = "serde", serde(default))]
    pub to: Option<f64>,
}

impl Measurement {
    /// A measurement over the whole result.
    pub fn new(name: impl Into<String>, measure: Measure) -> Self {
        Measurement {
            name: name.into(),
            measure,
            from: None,
            to: None,
        }
    }

    /// Limit it to `from ≤ x ≤ to`.
    pub fn window(mut self, from: f64, to: f64) -> Self {
        self.from = Some(from);
        self.to = Some(to);
        self
    }

    /// Take the measurement on a result.
    ///
    /// # Errors
    ///
    /// [`SimError::Measurement`] when it can't be taken: an unknown node or
    /// component, a window outside the result, a level never crossed.
    pub fn on(&self, result: &impl Measurable) -> Result<f64, SimError> {
        self.evaluate(result)
            .map_err(|reason| SimError::Measurement {
                name: self.name.clone(),
                reason,
            })
    }

    fn evaluate(&self, result: &impl Measurable) -> Result<f64, String> {
        let windowed = |signal: &Signal| -> Result<Vec<(f64, f64)>, String> {
            window(&result.trace(signal)?, self.from, self.to)
        };
        let value = match &self.measure {
            Measure::Find { signal, at } => {
                let trace = windowed(signal)?;
                interpolate(&trace, *at).ok_or_else(|| {
                    format!("x = {at} is outside the result ({})", span_text(&trace))
                })?
            }
            Measure::Stat { signal, stat } => statistic(&windowed(signal)?, *stat),
            Measure::When(crossing) => {
                let trace = windowed(&crossing.signal)?;
                find_crossing(&trace, crossing)?
            }
            Measure::Delay { trig, targ } => {
                let t0 = find_crossing(&windowed(&trig.signal)?, trig)?;
                let t1 = find_crossing(&windowed(&targ.signal)?, targ)?;
                t1 - t0
            }
            Measure::Transition {
                signal,
                edge,
                low,
                high,
            } => transition(&windowed(signal)?, *edge, *low, *high)?,
            Measure::Settling {
                signal,
                band,
                target,
            } => settling(&windowed(signal)?, *band, *target)?,
            Measure::Bandwidth { signal, drop_db } => {
                let mut db = signal.clone();
                db.view = View::Db;
                bandwidth(&windowed(&db)?, *drop_db)?
            }
        };
        if value.is_finite() {
            Ok(value)
        } else {
            Err(format!("the value isn't finite ({value})"))
        }
    }
}

fn span_text(trace: &[(f64, f64)]) -> String {
    match (trace.first(), trace.last()) {
        (Some(a), Some(b)) => format!("{} to {}", a.0, b.0),
        _ => "empty".to_string(),
    }
}

/// Linear interpolation; `None` outside the trace. A single-point trace
/// answers for any x, since a DC operating point has no x axis.
fn interpolate(trace: &[(f64, f64)], x: f64) -> Option<f64> {
    match trace {
        [] => None,
        [(_, y)] => Some(*y),
        _ => {
            let (first, last) = (trace[0].0, trace[trace.len() - 1].0);
            if x < first || x > last {
                return None;
            }
            let i = trace.partition_point(|p| p.0 < x);
            if i == 0 {
                return Some(trace[0].1);
            }
            let (x0, y0) = trace[i - 1];
            let (x1, y1) = trace[i];
            if x1 == x0 {
                return Some(y1);
            }
            Some(y0 + (y1 - y0) * (x - x0) / (x1 - x0))
        }
    }
}

/// Clip a trace to `[from, to]`, with interpolated end points.
fn window(
    trace: &[(f64, f64)],
    from: Option<f64>,
    to: Option<f64>,
) -> Result<Vec<(f64, f64)>, String> {
    if trace.is_empty() {
        return Err("the result has no points".to_string());
    }
    if trace.len() == 1 || (from.is_none() && to.is_none()) {
        return Ok(trace.to_vec());
    }
    let lo = from.unwrap_or(trace[0].0);
    let hi = to.unwrap_or(trace[trace.len() - 1].0);
    if lo > hi {
        return Err(format!("the window runs backwards ({lo} to {hi})"));
    }
    let (Some(y_lo), Some(y_hi)) = (interpolate(trace, lo), interpolate(trace, hi)) else {
        return Err(format!(
            "the window {lo} to {hi} isn't inside the result ({})",
            span_text(trace)
        ));
    };
    let mut out = vec![(lo, y_lo)];
    out.extend(trace.iter().copied().filter(|p| p.0 > lo && p.0 < hi));
    if hi > lo {
        out.push((hi, y_hi));
    }
    Ok(out)
}

fn statistic(trace: &[(f64, f64)], stat: Stat) -> f64 {
    let ys = trace.iter().map(|p| p.1);
    let min = ys.clone().fold(f64::INFINITY, f64::min);
    let max = ys.fold(f64::NEG_INFINITY, f64::max);
    let integral = |f: &dyn Fn(f64) -> f64| -> f64 {
        trace
            .windows(2)
            .map(|w| 0.5 * (f(w[0].1) + f(w[1].1)) * (w[1].0 - w[0].0))
            .sum()
    };
    let span = trace[trace.len() - 1].0 - trace[0].0;
    match stat {
        Stat::Min => min,
        Stat::Max => max,
        Stat::PeakToPeak => max - min,
        Stat::Final => trace[trace.len() - 1].1,
        Stat::Integral => integral(&|y| y),
        // A single point (or a zero-width window) has no span to average
        // over: its value is the mean.
        Stat::Avg if span == 0.0 => trace[0].1,
        Stat::Avg => integral(&|y| y) / span,
        Stat::Rms if span == 0.0 => trace[0].1.abs(),
        Stat::Rms => (integral(&|y| y * y) / span).sqrt(),
    }
}

/// Every crossing of `level`, as `(x, rising)`. A sample that lands exactly
/// on the level counts once, on the segment that reaches it.
fn crossings(trace: &[(f64, f64)], level: f64) -> Vec<(f64, bool)> {
    trace
        .windows(2)
        .filter_map(|w| {
            let ((x0, y0), (x1, y1)) = (w[0], w[1]);
            let rising = y0 < level && y1 >= level;
            let falling = y0 > level && y1 <= level;
            if !(rising || falling) {
                return None;
            }
            let x = x0 + (level - y0) * (x1 - x0) / (y1 - y0);
            Some((x, rising))
        })
        .collect()
}

fn edge_matches(edge: Edge, rising: bool) -> bool {
    match edge {
        Edge::Rise => rising,
        Edge::Fall => !rising,
        Edge::Cross => true,
    }
}

fn edge_text(edge: Edge) -> &'static str {
    match edge {
        Edge::Rise => "rises through",
        Edge::Fall => "falls through",
        Edge::Cross => "crosses",
    }
}

fn find_crossing(trace: &[(f64, f64)], crossing: &Crossing) -> Result<f64, String> {
    let all: Vec<f64> = crossings(trace, crossing.level)
        .into_iter()
        .filter(|&(_, rising)| edge_matches(crossing.edge, rising))
        .map(|(x, _)| x)
        .collect();
    let found = match crossing.occurrence {
        Occurrence::Nth(0) => return Err("crossings count from 1".to_string()),
        Occurrence::Nth(n) => all.get(n - 1).copied(),
        Occurrence::Last => all.last().copied(),
    };
    found.ok_or_else(|| {
        let which = match crossing.occurrence {
            Occurrence::Nth(n) => format!("{n} time(s)"),
            Occurrence::Last => "at all".to_string(),
        };
        format!(
            "`{}` never {} {} {which} (found {} such crossing(s); it spans {} to {})",
            crossing.signal,
            edge_text(crossing.edge),
            crossing.level,
            all.len(),
            statistic(trace, Stat::Min),
            statistic(trace, Stat::Max),
        )
    })
}

fn transition(trace: &[(f64, f64)], edge: Edge, low: f64, high: f64) -> Result<f64, String> {
    if !(0.0..=1.0).contains(&low) || !(0.0..=1.0).contains(&high) || low >= high {
        return Err(format!(
            "transition fractions must satisfy 0 ≤ low < high ≤ 1 (got {low}, {high})"
        ));
    }
    let min = statistic(trace, Stat::Min);
    let swing = statistic(trace, Stat::PeakToPeak);
    if swing == 0.0 {
        return Err("the signal doesn't move, so it has no transition".to_string());
    }
    let lo = min + low * swing;
    let hi = min + high * swing;
    // A rise leaves `lo` and reaches `hi`; a fall leaves `hi` and reaches `lo`.
    let (start_level, end_level, rising) = match edge {
        Edge::Rise => (lo, hi, true),
        Edge::Fall => (hi, lo, false),
        Edge::Cross => return Err("a transition is a rise or a fall".to_string()),
    };
    let end = crossings(trace, end_level)
        .into_iter()
        .find(|&(_, r)| r == rising)
        .map(|(x, _)| x)
        .ok_or_else(|| {
            format!(
                "the signal never {} {end_level}",
                edge_text(if rising { Edge::Rise } else { Edge::Fall })
            )
        })?;
    let start = crossings(trace, start_level)
        .into_iter()
        .rev()
        .find(|&(x, r)| r == rising && x <= end)
        .map(|(x, _)| x)
        .unwrap_or(trace[0].0);
    Ok(end - start)
}

fn settling(trace: &[(f64, f64)], band: f64, target: Option<f64>) -> Result<f64, String> {
    if band.is_nan() || band <= 0.0 {
        return Err(format!("the settling band must be above 0 (got {band})"));
    }
    let x0 = trace[0].0;
    let final_value = target.unwrap_or(trace[trace.len() - 1].1);
    let outside = |y: f64| (y - final_value).abs() > band;
    let Some(i) = trace.iter().rposition(|p| outside(p.1)) else {
        return Ok(0.0);
    };
    if i == trace.len() - 1 {
        return Err(format!(
            "the signal ends {} away from {final_value}, outside the ±{band} band",
            (trace[i].1 - final_value).abs()
        ));
    }
    // It enters the band between sample i and i + 1: interpolate onto the
    // edge of the band it came from.
    let (xa, ya) = trace[i];
    let (xb, yb) = trace[i + 1];
    let edge = if ya > final_value {
        final_value + band
    } else {
        final_value - band
    };
    let x = if yb == ya {
        xb
    } else {
        xa + (edge - ya) * (xb - xa) / (yb - ya)
    };
    Ok(x - x0)
}

fn bandwidth(trace: &[(f64, f64)], drop_db: f64) -> Result<f64, String> {
    let (peak_i, &(_, peak)) = trace
        .iter()
        .enumerate()
        .max_by(|a, b| a.1 .1.total_cmp(&b.1 .1))
        .ok_or_else(|| "the result has no points".to_string())?;
    let level = peak - drop_db;
    crossings(&trace[peak_i..], level)
        .into_iter()
        .find(|&(_, rising)| !rising)
        .map(|(x, _)| x)
        .ok_or_else(|| {
            format!(
                "the signal never falls {drop_db} dB below its {peak:.3} dB peak within the sweep"
            )
        })
}

// ---------------------------------------------------------------------------
// Specs
// ---------------------------------------------------------------------------

/// A measurement with limits: `V(vout)` 4.9–5.1 V, ripple under 50 mV.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Spec {
    /// What is measured; its name names the spec.
    pub measurement: Measurement,
    /// Lowest value that passes, inclusive. `None` for no lower limit.
    #[cfg_attr(feature = "serde", serde(default))]
    pub min: Option<f64>,
    /// Highest value that passes, inclusive. `None` for no upper limit.
    #[cfg_attr(feature = "serde", serde(default))]
    pub max: Option<f64>,
}

impl Spec {
    /// A spec with no limits yet. With none it passes whenever the
    /// measurement can be taken, which records a value in the report.
    pub fn new(measurement: Measurement) -> Self {
        Spec {
            measurement,
            min: None,
            max: None,
        }
    }

    /// Pass when `min ≤ value ≤ max`.
    pub fn between(mut self, min: f64, max: f64) -> Self {
        self.min = Some(min);
        self.max = Some(max);
        self
    }

    /// Pass when `value ≥ min`.
    pub fn at_least(mut self, min: f64) -> Self {
        self.min = Some(min);
        self
    }

    /// Pass when `value ≤ max`.
    pub fn at_most(mut self, max: f64) -> Self {
        self.max = Some(max);
        self
    }

    /// Measure on a result and judge it.
    pub fn check(&self, result: &impl Measurable) -> SpecCheck {
        let name = self.measurement.name.clone();
        match self.measurement.evaluate(result) {
            Ok(value) => {
                let below = self.min.map(|m| value - m);
                let above = self.max.map(|m| m - value);
                let margin = match (below, above) {
                    (Some(a), Some(b)) => Some(a.min(b)),
                    (a, b) => a.or(b),
                };
                let status = if margin.is_none_or(|m| m >= 0.0) {
                    SpecStatus::Pass
                } else {
                    SpecStatus::Fail
                };
                SpecCheck {
                    name,
                    value: Some(value),
                    min: self.min,
                    max: self.max,
                    margin,
                    status,
                    reason: None,
                }
            }
            Err(reason) => SpecCheck {
                name,
                value: None,
                min: self.min,
                max: self.max,
                margin: None,
                status: SpecStatus::Unmeasured,
                reason: Some(reason),
            },
        }
    }
}

/// How a spec did.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SpecStatus {
    /// Within its limits.
    Pass,
    /// Outside them.
    Fail,
    /// The measurement couldn't be taken; [`SpecCheck::reason`] says why.
    /// Never counted as a pass.
    Unmeasured,
}

/// One spec checked against one result.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct SpecCheck {
    /// The measurement's name.
    pub name: String,
    /// The measured value, in the signal's unit; `None` when unmeasured.
    pub value: Option<f64>,
    /// Lower limit.
    pub min: Option<f64>,
    /// Upper limit.
    pub max: Option<f64>,
    /// Distance to the nearer limit, in the signal's unit: positive inside
    /// the limits, negative outside. `None` with no limits or no value.
    pub margin: Option<f64>,
    /// Pass, fail or unmeasured.
    pub status: SpecStatus,
    /// Why the measurement couldn't be taken.
    pub reason: Option<String>,
}

/// Every spec's verdict, with the ratings checks alongside when attached.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SpecReport {
    /// One entry per spec, in the order given.
    pub checks: Vec<SpecCheck>,
    /// The ratings checks, when attached with [`with_ratings`](Self::with_ratings).
    pub ratings: Option<RatingsReport>,
}

impl SpecReport {
    /// Attach a ratings report, so [`passed`](Self::passed) also needs every
    /// part within its ratings.
    pub fn with_ratings(mut self, ratings: RatingsReport) -> Self {
        self.ratings = Some(ratings);
        self
    }

    /// Join another report, such as the specs checked on a second analysis.
    /// Its ratings checks are appended to this one's.
    pub fn merge(mut self, other: SpecReport) -> Self {
        self.checks.extend(other.checks);
        self.ratings = match (self.ratings, other.ratings) {
            (Some(mut a), Some(b)) => {
                a.checks.extend(b.checks);
                Some(a)
            }
            (a, b) => a.or(b),
        };
        self
    }

    /// `true` only when every spec passed and, with ratings attached, every
    /// part is within them ([`RatingsReport::is_safe`]).
    pub fn passed(&self) -> bool {
        self.checks.iter().all(|c| c.status == SpecStatus::Pass)
            && self.ratings.as_ref().is_none_or(RatingsReport::is_safe)
    }

    /// Specs outside their limits.
    pub fn failed(&self) -> Vec<&SpecCheck> {
        self.with_status(SpecStatus::Fail)
    }

    /// Specs whose measurement couldn't be taken.
    pub fn unmeasured(&self) -> Vec<&SpecCheck> {
        self.with_status(SpecStatus::Unmeasured)
    }

    /// The check for one spec, by name.
    pub fn get(&self, name: &str) -> Option<&SpecCheck> {
        self.checks.iter().find(|c| c.name == name)
    }

    fn with_status(&self, status: SpecStatus) -> Vec<&SpecCheck> {
        self.checks.iter().filter(|c| c.status == status).collect()
    }
}

/// Check every spec against a result.
///
/// A measurement that can't be taken is [`SpecStatus::Unmeasured`] with the
/// reason, not an error, so one bad spec doesn't hide the others' verdicts.
pub fn check_specs(result: &impl Measurable, specs: &[Spec]) -> SpecReport {
    SpecReport {
        checks: specs.iter().map(|s| s.check(result)).collect(),
        ratings: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
    use crate::circuit::{Circuit, CircuitElement};
    use crate::waveform::Waveform;
    use crate::{dc_sweep, solve_circuit, solve_transient, TransientConfig};

    struct Points(Vec<(f64, f64)>);

    impl Measurable for Points {
        fn trace(&self, _: &Signal) -> Result<Vec<(f64, f64)>, String> {
            Ok(self.0.clone())
        }
    }

    fn measure(points: &[(f64, f64)], m: Measure) -> Result<f64, SimError> {
        Measurement::new("m", m).on(&Points(points.to_vec()))
    }

    fn sine(n: usize) -> Vec<(f64, f64)> {
        (0..=n)
            .map(|k| {
                let t = k as f64 / n as f64;
                (t, (2.0 * std::f64::consts::PI * t).sin())
            })
            .collect()
    }

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "{a} vs {b}");
    }

    #[test]
    fn statistics_of_one_sine_period() {
        let s = sine(1000);
        let v = || Signal::v("x");
        close(
            measure(&s, Measure::stat(v(), Stat::Max)).unwrap(),
            1.0,
            1e-4,
        );
        close(
            measure(&s, Measure::stat(v(), Stat::Min)).unwrap(),
            -1.0,
            1e-4,
        );
        close(
            measure(&s, Measure::stat(v(), Stat::PeakToPeak)).unwrap(),
            2.0,
            1e-4,
        );
        close(
            measure(&s, Measure::stat(v(), Stat::Avg)).unwrap(),
            0.0,
            1e-9,
        );
        close(
            measure(&s, Measure::stat(v(), Stat::Rms)).unwrap(),
            std::f64::consts::FRAC_1_SQRT_2,
            1e-4,
        );
        close(
            measure(&s, Measure::stat(v(), Stat::Integral)).unwrap(),
            0.0,
            1e-9,
        );
    }

    #[test]
    fn average_is_weighted_by_step_width() {
        // 0 for 9 s, then 10 for 1 s: mean 1, not the sample mean 3.33.
        let p = [(0.0, 0.0), (9.0, 0.0), (9.0, 10.0), (10.0, 10.0)];
        close(
            measure(&p, Measure::stat(Signal::v("x"), Stat::Avg)).unwrap(),
            1.0,
            1e-12,
        );
    }

    #[test]
    fn window_interpolates_its_ends() {
        let p = [(0.0, 0.0), (10.0, 10.0)];
        let m = Measurement::new("m", Measure::stat(Signal::v("x"), Stat::Avg)).window(2.0, 4.0);
        close(m.on(&Points(p.to_vec())).unwrap(), 3.0, 1e-12);
        let outside =
            Measurement::new("m", Measure::stat(Signal::v("x"), Stat::Avg)).window(5.0, 20.0);
        assert!(matches!(
            outside.on(&Points(p.to_vec())),
            Err(SimError::Measurement { .. })
        ));
    }

    #[test]
    fn find_interpolates() {
        let p = [(0.0, 0.0), (1.0, 2.0)];
        close(
            measure(&p, Measure::find(Signal::v("x"), 0.25)).unwrap(),
            0.5,
            1e-12,
        );
        assert!(measure(&p, Measure::find(Signal::v("x"), 2.0)).is_err());
    }

    #[test]
    fn crossings_count_by_edge() {
        let s = sine(1000);
        let at = |c: Crossing| measure(&s, Measure::When(c)).unwrap();
        // Falls through 0 at t = 0.5; the first sample sits on 0 and isn't a crossing.
        close(at(Crossing::new(Signal::v("x"), 0.0)), 0.5, 1e-9);
        close(
            at(Crossing::new(Signal::v("x"), -0.5).rising().last()),
            11.0 / 12.0,
            1e-4,
        );
        close(
            at(Crossing::new(Signal::v("x"), 0.5).rising()),
            1.0 / 12.0,
            1e-4,
        );
        close(
            at(Crossing::new(Signal::v("x"), 0.5).falling()),
            5.0 / 12.0,
            1e-4,
        );
        let err = measure(
            &s,
            Measure::When(Crossing::new(Signal::v("x"), 0.5).rising().nth(2)),
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("never rises through 0.5 2 time(s)"), "{err}");
    }

    #[test]
    fn delay_between_two_crossings() {
        let s = sine(1000);
        let m = Measure::Delay {
            trig: Crossing::new(Signal::v("x"), 0.5).rising(),
            targ: Crossing::new(Signal::v("x"), 0.5).falling(),
        };
        close(measure(&s, m).unwrap(), 1.0 / 3.0, 1e-4);
    }

    #[test]
    fn ramp_rise_and_fall_times() {
        // 0 → 1 over 1 s, hold, 1 → 0 over 2 s.
        let p = [(0.0, 0.0), (1.0, 1.0), (5.0, 1.0), (7.0, 0.0)];
        close(
            measure(&p, Measure::rise_time(Signal::v("x"))).unwrap(),
            0.8,
            1e-12,
        );
        close(
            measure(&p, Measure::fall_time(Signal::v("x"))).unwrap(),
            1.6,
            1e-12,
        );
        let flat = [(0.0, 1.0), (1.0, 1.0)];
        assert!(measure(&flat, Measure::rise_time(Signal::v("x"))).is_err());
    }

    #[test]
    fn settling_into_a_band() {
        // Decaying overshoot: 2, 0.5, 1.2, 0.95, 1.0 around a final value of 1.
        let p = [(0.0, 2.0), (1.0, 0.5), (2.0, 1.2), (3.0, 0.95), (4.0, 1.0)];
        // ±0.1: last outside at t = 2 (1.2), enters at 1.1 → t = 2 + 0.1/0.25 = 2.4.
        close(
            measure(&p, Measure::settling(Signal::v("x"), 0.1)).unwrap(),
            2.4,
            1e-12,
        );
        let never = [(0.0, 0.0), (1.0, 1.0)];
        let m = Measure::Settling {
            signal: Signal::v("x"),
            band: 0.1,
            target: Some(5.0),
        };
        assert!(measure(&never, m).is_err());
    }

    fn rc(waveform: Option<Waveform>) -> Circuit {
        Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["in".into(), "0".into()],
                    // A waveform adds to the DC value.
                    voltage: if waveform.is_some() { 0.0 } else { 1.0 },
                    waveform,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["in".into(), "out".into()],
                    resistance: 1e3,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["out".into(), "0".into()],
                    capacitance: 1e-6,
                },
            ],
        )
    }

    #[test]
    fn rc_bandwidth_and_phase() {
        let config = AcConfig {
            f_start: 1.0,
            f_stop: 1e6,
            num_points: 601,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
            ac_phase_deg: 0.0,
        };
        let ac = solve_ac(&rc(None), &config).unwrap();
        let fc = 1.0 / (2.0 * std::f64::consts::PI * 1e-3);
        let bw = Measurement::new("bw", Measure::bandwidth(Signal::v("out")))
            .on(&ac)
            .unwrap();
        assert!((bw - fc).abs() / fc < 0.01, "{bw} vs {fc}");
        // The same point the ngspice way: WHEN vdb(out)=-3 FALL=1.
        let when = Measurement::new(
            "w",
            Measure::When(Crossing::new(Signal::v("out").db(), -3.0103).falling()),
        )
        .on(&ac)
        .unwrap();
        assert!((when - fc).abs() / fc < 0.01);
        let phase = Measurement::new("p", Measure::find(Signal::v("out").phase(), fc))
            .on(&ac)
            .unwrap();
        close(phase, -45.0, 0.5);
        // Ground is in the AC result too.
        let v_in = Measurement::new("vin", Measure::find(Signal::v_diff("in", "0"), fc))
            .on(&ac)
            .unwrap();
        close(v_in, 1.0, 1e-12);
        let current = Measurement::new("i", Measure::stat(Signal::i("R1"), Stat::Max)).on(&ac);
        assert!(current
            .unwrap_err()
            .to_string()
            .contains("node voltages only"));
    }

    #[test]
    fn op_point_and_dc_sweep() {
        let divider = Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["in".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["in".into(), "out".into()],
                    resistance: 1e3,
                },
                CircuitElement::Resistor {
                    id: "R2".into(),
                    nodes: ["out".into(), "0".into()],
                    resistance: 1e3,
                },
            ],
        );
        let op = solve_circuit(&divider).unwrap();
        let specs = [
            Spec::new(Measurement::new(
                "vout",
                Measure::stat(Signal::v("out"), Stat::Final),
            ))
            .between(4.9, 5.1),
            Spec::new(Measurement::new(
                "vr1",
                Measure::stat(Signal::v_diff("in", "out"), Stat::Final),
            ))
            .at_most(4.0),
            Spec::new(Measurement::new(
                "ir1",
                Measure::stat(Signal::i("R1"), Stat::Final),
            ))
            .between(4e-3, 6e-3),
            Spec::new(Measurement::new(
                "typo",
                Measure::stat(Signal::v("outt"), Stat::Final),
            )),
        ];
        let report = check_specs(&op, &specs);
        let vout = report.get("vout").unwrap();
        assert_eq!(vout.status, SpecStatus::Pass);
        close(vout.margin.unwrap(), 0.1, 1e-9);
        let vr1 = report.get("vr1").unwrap();
        assert_eq!(vr1.status, SpecStatus::Fail);
        close(vr1.margin.unwrap(), -1.0, 1e-9);
        assert_eq!(report.get("ir1").unwrap().status, SpecStatus::Pass);
        let typo = report.get("typo").unwrap();
        assert_eq!(typo.status, SpecStatus::Unmeasured);
        assert!(typo.reason.as_deref().unwrap().contains("outt"));
        assert!(!report.passed());
        assert_eq!(report.failed().len(), 1);
        assert_eq!(report.unmeasured().len(), 1);

        // Swept from high to low: the trace still runs in ascending x.
        let sweep = dc_sweep(&divider, "V1", 10.0, 0.0, 11).unwrap();
        let half = Measurement::new(
            "half",
            Measure::When(Crossing::new(Signal::v("out"), 2.5).rising()),
        )
        .on(&sweep)
        .unwrap();
        close(half, 5.0, 1e-9);
    }

    #[test]
    fn ratings_ride_along() {
        let op = solve_circuit(&rc(None)).unwrap();
        let ok = check_specs(&op, &[]);
        assert!(ok.passed());
        let unsafe_ratings = RatingsReport {
            checks: vec![crate::RatingCheck {
                part: "R1".into(),
                rating: crate::RatingKind::Power,
                limit: None,
                worst: None,
                time: None,
                margin_percent: None,
                status: crate::RatingStatus::Unverified,
            }],
        };
        let merged = ok.merge(check_specs(&op, &[]).with_ratings(unsafe_ratings));
        assert!(merged.ratings.is_some());
        assert!(!merged.passed());
    }

    #[test]
    fn transient_pulse_delay() {
        let wave = Waveform::Pulse {
            v1: 0.0,
            v2: 1.0,
            delay: 1e-3,
            rise_time: 1e-6,
            fall_time: 1e-6,
            pulse_width: 5e-3,
            period: 20e-3,
        };
        let r = solve_transient(&rc(Some(wave)), &TransientConfig::new(6e-3, 5e-6)).unwrap();
        // The output reaches half the step τ·ln 2 after the input does.
        let d = Measurement::new(
            "tpd",
            Measure::Delay {
                trig: Crossing::new(Signal::v("in"), 0.5).rising(),
                targ: Crossing::new(Signal::v("out"), 0.5).rising(),
            },
        )
        .on(&r)
        .unwrap();
        close(d, 1e-3 * std::f64::consts::LN_2, 2e-5);
        let ir = Measurement::new("ir", Measure::stat(Signal::i("R1"), Stat::Max))
            .on(&r)
            .unwrap();
        assert!(ir > 0.9e-3 && ir <= 1.0e-3 + 1e-9, "{ir}");
    }
}
