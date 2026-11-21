//! Monte Carlo and corner analysis: how a circuit's measurements spread
//! when its parts vary within their tolerances.
//!
//! A [`Tolerance`] says how one parameter of one component varies: by how
//! much, and with which [`Distribution`]. [`monte_carlo`] draws every
//! tolerance at random for each of its runs; [`corners`] sets each at its
//! low or high end, every combination or a chosen list. Both wrap any
//! analysis, as a closure from a [`Circuit`] to a result, and take each
//! [`Measurement`] on every run. A run that fails (a convergence failure,
//! say) is recorded, not fatal.
//!
//! The draws come from a seeded generator, and each run's from its own
//! index, so a run repeats exactly from its seed whether runs go one after
//! another or, with the `parallel` feature, across threads.
//!
//! ```
//! use sindr::measure::{Measure, Measurement, Signal, Stat};
//! use sindr::variation::{monte_carlo, MonteCarloConfig, Parameter, Tolerance};
//! use sindr::{solve_op, Circuit, CircuitElement};
//!
//! let r = |id: &str, a: &str, b: &str| CircuitElement::Resistor {
//!     id: id.into(),
//!     nodes: [a.into(), b.into()],
//!     resistance: 10e3,
//! };
//! let divider = Circuit::new(
//!     "0",
//!     vec![
//!         CircuitElement::VoltageSource {
//!             id: "v1".into(),
//!             nodes: ["in".into(), "0".into()],
//!             voltage: 10.0,
//!             waveform: None,
//!         },
//!         r("r1", "in", "out"),
//!         r("r2", "out", "0"),
//!     ],
//! );
//! // Every resistor ±1 %, 3σ Gaussian.
//! let tolerances = Tolerance::on_every(&divider, Parameter::Resistance, 0.01);
//! let config = MonteCarloConfig::new(tolerances, 200).with_seed(7);
//! let vout = Measurement::new("vout", Measure::stat(Signal::v("out"), Stat::Avg));
//! let result = monte_carlo(&divider, &config, &[vout], solve_op)?;
//! let stats = result.stats("vout").unwrap();
//! assert!((stats.mean - 5.0).abs() < 0.01);
//! assert!(stats.min > 4.9 && stats.max < 5.1);
//! # Ok::<(), sindr::SimError>(())
//! ```

use crate::measure::{Measurable, Measurement};
use crate::{Circuit, CircuitElement, SimError};

/// Which parameter of a component a [`Tolerance`] varies.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Parameter {
    /// A resistor's resistance (Ω).
    Resistance,
    /// A capacitor's capacitance (F).
    Capacitance,
    /// An inductor's inductance (H).
    Inductance,
    /// A voltage source's DC voltage (V).
    Voltage,
    /// A current source's DC current (A).
    Current,
}

impl Parameter {
    /// The parameter on `element`, if it has it.
    fn get(self, element: &CircuitElement) -> Option<f64> {
        match (self, element) {
            (Parameter::Resistance, CircuitElement::Resistor { resistance, .. }) => {
                Some(*resistance)
            }
            (Parameter::Capacitance, CircuitElement::Capacitor { capacitance, .. }) => {
                Some(*capacitance)
            }
            (Parameter::Inductance, CircuitElement::Inductor { inductance, .. }) => {
                Some(*inductance)
            }
            (Parameter::Voltage, CircuitElement::VoltageSource { voltage, .. }) => Some(*voltage),
            (Parameter::Current, CircuitElement::CurrentSource { current, .. }) => Some(*current),
            _ => None,
        }
    }

    fn set(self, element: &mut CircuitElement, value: f64) {
        match (self, element) {
            (Parameter::Resistance, CircuitElement::Resistor { resistance, .. }) => {
                *resistance = value;
            }
            (Parameter::Capacitance, CircuitElement::Capacitor { capacitance, .. }) => {
                *capacitance = value;
            }
            (Parameter::Inductance, CircuitElement::Inductor { inductance, .. }) => {
                *inductance = value;
            }
            (Parameter::Voltage, CircuitElement::VoltageSource { voltage, .. }) => {
                *voltage = value;
            }
            (Parameter::Current, CircuitElement::CurrentSource { current, .. }) => {
                *current = value;
            }
            _ => {}
        }
    }
}

/// How a [`Tolerance`]'s parameter is drawn in [`monte_carlo`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum Distribution {
    /// Evenly across `nominal·(1 ± relative)`.
    Uniform,
    /// Normal about the nominal, with the band `relative` at `sigmas`
    /// standard deviations: a 1 % part at 3σ has σ = 0.33 %. Unclipped, so
    /// a draw falls outside the band about 0.27 % of the time at 3σ.
    Gaussian {
        /// Standard deviations the band spans, above 0.
        sigmas: f64,
    },
}

/// How one parameter of one component varies.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct Tolerance {
    /// The component's id, flattened (`x1.r3` inside an instance).
    pub component: String,
    /// Which of its parameters.
    pub parameter: Parameter,
    /// The band as a fraction of the nominal value, in `[0, 1)`: 0.05
    /// for 5 %.
    pub relative: f64,
    /// How [`monte_carlo`] draws within it.
    pub distribution: Distribution,
}

impl Tolerance {
    /// `parameter` of `component` within `relative`, Gaussian at 3σ, as
    /// a part's tolerance is usually specified.
    pub fn new(component: impl Into<String>, parameter: Parameter, relative: f64) -> Self {
        Self {
            component: component.into(),
            parameter,
            relative,
            distribution: Distribution::Gaussian { sigmas: 3.0 },
        }
    }

    /// The same, drawn uniformly across the band.
    pub fn uniform(component: impl Into<String>, parameter: Parameter, relative: f64) -> Self {
        Self::new(component, parameter, relative).with_distribution(Distribution::Uniform)
    }

    /// With `distribution` in place of the default.
    pub fn with_distribution(mut self, distribution: Distribution) -> Self {
        self.distribution = distribution;
        self
    }

    /// A tolerance of `relative` on `parameter` of every component of
    /// `circuit` that has it, hierarchy included, each independent.
    pub fn on_every(circuit: &Circuit, parameter: Parameter, relative: f64) -> Vec<Tolerance> {
        let flat = circuit.flattened();
        let components = match &flat {
            Ok(flat) => &flat.components,
            Err(_) => &circuit.components,
        };
        components
            .iter()
            .filter(|c| parameter.get(c).is_some())
            .map(|c| Tolerance::new(c.id(), parameter, relative))
            .collect()
    }
}

/// Configuration of a [`monte_carlo`] analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct MonteCarloConfig {
    /// What varies.
    pub tolerances: Vec<Tolerance>,
    /// Number of runs, at least 1.
    pub runs: usize,
    /// Seed of the draws: the same seed gives the same runs.
    pub seed: u64,
}

impl MonteCarloConfig {
    /// `runs` runs over `tolerances`, with seed 0.
    pub fn new(tolerances: Vec<Tolerance>, runs: usize) -> Self {
        Self {
            tolerances,
            runs,
            seed: 0,
        }
    }

    /// With `seed` in place of 0.
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }
}

/// Where a [`Tolerance`] sits in one corner of a [`corners`] analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Corner {
    /// `nominal·(1 − relative)`.
    Low,
    /// The nominal value.
    Nominal,
    /// `nominal·(1 + relative)`.
    High,
}

/// Which corners a [`corners`] analysis runs.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Corners {
    /// Every low/high combination: 2ⁿ runs for n tolerances, at most
    /// [`MAX_CORNER_TOLERANCES`] of them.
    All,
    /// These corners, each a [`Corner`] per tolerance in order.
    List(Vec<Vec<Corner>>),
}

/// Most tolerances [`Corners::All`] takes: 2¹⁶ = 65 536 runs.
pub const MAX_CORNER_TOLERANCES: usize = 16;

/// One run of a [`monte_carlo`] or [`corners`] analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct VariationRun {
    /// Each tolerance's parameter value in this run, in order.
    pub values: Vec<f64>,
    /// Each measurement, in order, or why it couldn't be taken: the
    /// analysis failing gives every measurement its error.
    pub measurements: Vec<Result<f64, String>>,
}

/// One measurement's spread over the runs it could be taken on.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct MeasurementStats {
    /// The measurement's name.
    pub name: String,
    /// Mean over the measured runs.
    pub mean: f64,
    /// Sample standard deviation (n − 1); 0 for one run.
    pub std_dev: f64,
    /// Smallest value.
    pub min: f64,
    /// Largest value.
    pub max: f64,
    /// Runs it was taken on.
    pub measured: usize,
    /// Runs it couldn't be taken on.
    pub failed: usize,
}

/// Result of a [`monte_carlo`] or [`corners`] analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct VariationResult {
    /// The tolerances, in the order each run's `values` follow.
    pub tolerances: Vec<Tolerance>,
    /// Every run, in order.
    pub runs: Vec<VariationRun>,
    /// One entry per measurement, in order. NaN statistics when no run
    /// could take it.
    pub stats: Vec<MeasurementStats>,
}

impl VariationResult {
    /// The statistics of the measurement called `name`.
    pub fn stats(&self, name: &str) -> Option<&MeasurementStats> {
        self.stats.iter().find(|s| s.name == name)
    }

    /// The fraction of runs whose measurement `name` lies in
    /// `[min, max]`; a run that couldn't take it fails. `None` for an
    /// unknown name.
    pub fn yield_within(&self, name: &str, min: f64, max: f64) -> Option<f64> {
        let k = self.stats.iter().position(|s| s.name == name)?;
        let pass = self
            .runs
            .iter()
            .filter(|run| matches!(run.measurements[k], Ok(v) if (min..=max).contains(&v)))
            .count();
        Some(pass as f64 / self.runs.len() as f64)
    }
}

/// Runs `analysis` `config.runs` times, each with every tolerance drawn
/// afresh, and takes `measurements` on each result.
///
/// # Errors
///
/// [`SimError::InvalidComponent`] for a tolerance on a component that
/// isn't there or doesn't have its parameter, a relative band outside
/// `[0, 1)`, a Gaussian whose `sigmas` isn't above 0, or no runs. A run
/// that fails isn't an error: see [`VariationRun::measurements`].
pub fn monte_carlo<R, F>(
    circuit: &Circuit,
    config: &MonteCarloConfig,
    measurements: &[Measurement],
    analysis: F,
) -> Result<VariationResult, SimError>
where
    R: Measurable,
    F: Fn(&Circuit) -> Result<R, SimError> + Sync,
{
    if config.runs == 0 {
        return Err(SimError::InvalidComponent(
            "Monte Carlo needs at least one run".to_string(),
        ));
    }
    let base = Base::new(circuit, &config.tolerances)?;
    let seed = config.seed;
    let runs = run_all(config.runs, |k| {
        let mut rng = SplitMix64::for_run(seed, k as u64);
        let deviations: Vec<f64> = config
            .tolerances
            .iter()
            .map(|t| match t.distribution {
                Distribution::Uniform => t.relative * (2.0 * rng.unit() - 1.0),
                Distribution::Gaussian { sigmas } => t.relative / sigmas * rng.normal(),
            })
            .collect();
        base.run(&deviations, measurements, &analysis)
    });
    Ok(finish(config.tolerances.clone(), runs, measurements))
}

/// Runs `analysis` with each tolerance at the low or high end of its band
/// (or nominal), for every combination or the listed ones, and takes
/// `measurements` on each result. A tolerance's distribution doesn't
/// matter here.
///
/// # Errors
///
/// As [`monte_carlo`], and for more than [`MAX_CORNER_TOLERANCES`]
/// tolerances with [`Corners::All`], or a listed corner whose length isn't
/// the number of tolerances.
pub fn corners<R, F>(
    circuit: &Circuit,
    tolerances: &[Tolerance],
    which: &Corners,
    measurements: &[Measurement],
    analysis: F,
) -> Result<VariationResult, SimError>
where
    R: Measurable,
    F: Fn(&Circuit) -> Result<R, SimError> + Sync,
{
    let n = tolerances.len();
    let list: Vec<Vec<Corner>> = match which {
        Corners::All => {
            if n > MAX_CORNER_TOLERANCES {
                return Err(SimError::InvalidComponent(format!(
                    "every corner of {n} tolerances is 2^{n} runs; list the corners, or take at \
                     most {MAX_CORNER_TOLERANCES}"
                )));
            }
            (0..1usize << n)
                .map(|bits| {
                    (0..n)
                        .map(|k| {
                            if bits >> k & 1 == 1 {
                                Corner::High
                            } else {
                                Corner::Low
                            }
                        })
                        .collect()
                })
                .collect()
        }
        Corners::List(list) => {
            if let Some(bad) = list.iter().find(|c| c.len() != n) {
                return Err(SimError::InvalidComponent(format!(
                    "a corner names {} tolerances, not {n}",
                    bad.len()
                )));
            }
            list.clone()
        }
    };
    let base = Base::new(circuit, tolerances)?;
    let runs = run_all(list.len(), |k| {
        let deviations: Vec<f64> = list[k]
            .iter()
            .zip(tolerances)
            .map(|(corner, t)| match corner {
                Corner::Low => -t.relative,
                Corner::Nominal => 0.0,
                Corner::High => t.relative,
            })
            .collect();
        base.run(&deviations, measurements, &analysis)
    });
    Ok(finish(tolerances.to_vec(), runs, measurements))
}

/// The flattened circuit and where each tolerance's component sits in it.
struct Base {
    circuit: Circuit,
    targets: Vec<(usize, Parameter, f64)>,
}

impl Base {
    fn new(circuit: &Circuit, tolerances: &[Tolerance]) -> Result<Base, SimError> {
        let circuit = circuit.flattened()?.into_owned();
        let mut targets = Vec::with_capacity(tolerances.len());
        for t in tolerances {
            if !(t.relative.is_finite() && (0.0..1.0).contains(&t.relative)) {
                return Err(SimError::InvalidComponent(format!(
                    "`{}`: tolerance {} must be in [0, 1)",
                    t.component, t.relative
                )));
            }
            if let Distribution::Gaussian { sigmas } = t.distribution {
                if !(sigmas.is_finite() && sigmas > 0.0) {
                    return Err(SimError::InvalidComponent(format!(
                        "`{}`: a Gaussian tolerance needs sigmas above 0, not {sigmas}",
                        t.component
                    )));
                }
            }
            let index = circuit
                .components
                .iter()
                .position(|c| c.id() == t.component)
                .ok_or_else(|| {
                    SimError::InvalidComponent(format!(
                        "tolerance on `{}`, which isn't in the circuit",
                        t.component
                    ))
                })?;
            let nominal = t.parameter.get(&circuit.components[index]).ok_or_else(|| {
                SimError::InvalidComponent(format!(
                    "`{}` has no {:?} to vary",
                    t.component, t.parameter
                ))
            })?;
            targets.push((index, t.parameter, nominal));
        }
        Ok(Base { circuit, targets })
    }

    /// One run with each tolerance's parameter at `nominal·(1 + deviation)`.
    fn run<R: Measurable>(
        &self,
        deviations: &[f64],
        measurements: &[Measurement],
        analysis: &(impl Fn(&Circuit) -> Result<R, SimError> + Sync),
    ) -> VariationRun {
        let mut circuit = self.circuit.clone();
        let values: Vec<f64> = self
            .targets
            .iter()
            .zip(deviations)
            .map(|(&(index, parameter, nominal), d)| {
                let value = nominal * (1.0 + d);
                parameter.set(&mut circuit.components[index], value);
                value
            })
            .collect();
        let measurements = match analysis(&circuit) {
            Ok(result) => measurements
                .iter()
                .map(|m| m.on(&result).map_err(|e| e.to_string()))
                .collect(),
            Err(e) => vec![Err(e.to_string()); measurements.len()],
        };
        VariationRun {
            values,
            measurements,
        }
    }
}

/// `run(k)` for each `k` in `0..n`, in order: one thread per core with
/// the `parallel` feature, otherwise one after another.
fn run_all(n: usize, run: impl Fn(usize) -> VariationRun + Sync) -> Vec<VariationRun> {
    #[cfg(feature = "parallel")]
    {
        let threads = std::thread::available_parallelism()
            .map_or(1, std::num::NonZeroUsize::get)
            .min(n);
        if threads > 1 {
            let chunk = n.div_ceil(threads);
            let run = &run;
            return std::thread::scope(|scope| {
                let handles: Vec<_> = (0..threads)
                    .map(|t| {
                        scope.spawn(move || {
                            (t * chunk..((t + 1) * chunk).min(n))
                                .map(run)
                                .collect::<Vec<_>>()
                        })
                    })
                    .collect();
                handles
                    .into_iter()
                    .flat_map(|h| h.join().unwrap_or_else(|p| std::panic::resume_unwind(p)))
                    .collect()
            });
        }
    }
    (0..n).map(run).collect()
}

fn finish(
    tolerances: Vec<Tolerance>,
    runs: Vec<VariationRun>,
    measurements: &[Measurement],
) -> VariationResult {
    let stats = measurements
        .iter()
        .enumerate()
        .map(|(k, m)| {
            let values: Vec<f64> = runs
                .iter()
                .filter_map(|r| r.measurements[k].as_ref().ok().copied())
                .collect();
            let n = values.len();
            let mean = values.iter().sum::<f64>() / n as f64;
            let variance = if n > 1 {
                values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1) as f64
            } else if n == 1 {
                0.0
            } else {
                f64::NAN
            };
            let fold =
                |f: fn(f64, f64) -> f64| values.iter().copied().reduce(f).unwrap_or(f64::NAN);
            MeasurementStats {
                name: m.name.clone(),
                mean,
                std_dev: variance.sqrt(),
                min: fold(f64::min),
                max: fold(f64::max),
                measured: n,
                failed: runs.len() - n,
            }
        })
        .collect();
    VariationResult {
        tolerances,
        runs,
        stats,
    }
}

/// SplitMix64: small, fast, and good enough for tolerance draws. Each run
/// gets its own stream from `(seed, run)`, so the draws don't depend on
/// which thread runs it or in what order.
struct SplitMix64(u64);

impl SplitMix64 {
    fn for_run(seed: u64, run: u64) -> Self {
        let mut mix = SplitMix64(seed ^ run.wrapping_mul(0xD1B5_4A32_D192_ED03));
        // Decorrelate neighbouring seeds before the first draw.
        mix.next();
        mix
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`, from the top 53 bits.
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal, by Box–Muller.
    fn normal(&mut self) -> f64 {
        // 1 − unit() is in (0, 1], so its log is finite.
        let r = (-2.0 * (1.0 - self.unit()).ln()).sqrt();
        r * (std::f64::consts::TAU * self.unit()).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn draws_are_uniform_and_normal() {
        let mut rng = SplitMix64::for_run(42, 0);
        let n = 200_000;
        let u: Vec<f64> = (0..n).map(|_| rng.unit()).collect();
        assert!(u.iter().all(|x| (0.0..1.0).contains(x)));
        let mean = u.iter().sum::<f64>() / n as f64;
        assert!((mean - 0.5).abs() < 0.005, "{mean}");
        let z: Vec<f64> = (0..n).map(|_| rng.normal()).collect();
        let mean = z.iter().sum::<f64>() / n as f64;
        let var = z.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        assert!(
            mean.abs() < 0.01 && (var - 1.0).abs() < 0.01,
            "{mean} {var}"
        );
        // About 0.27 % beyond 3σ.
        let tail = z.iter().filter(|x| x.abs() > 3.0).count() as f64 / n as f64;
        assert!((tail - 0.0027).abs() < 0.001, "{tail}");
    }

    /// Threads change who runs each index, not what it draws or where it
    /// lands.
    #[test]
    fn run_all_keeps_order_and_draws() {
        let run = |k: usize| VariationRun {
            values: vec![SplitMix64::for_run(7, k as u64).unit()],
            measurements: vec![Ok(k as f64)],
        };
        let all = run_all(1001, run);
        let serial: Vec<_> = (0..1001).map(run).collect();
        assert_eq!(all, serial);
    }

    #[test]
    fn streams_repeat_by_seed_and_run_and_differ_otherwise() {
        let first = |seed, run| SplitMix64::for_run(seed, run).next();
        assert_eq!(first(1, 5), first(1, 5));
        assert_ne!(first(1, 5), first(1, 6));
        assert_ne!(first(1, 5), first(2, 5));
    }
}
