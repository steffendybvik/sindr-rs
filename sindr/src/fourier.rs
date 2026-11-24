//! Spectral analysis of transient results: SPICE `.four` harmonics and THD,
//! and a windowed FFT.
//!
//! Both read a [`Signal`] (a node voltage, a voltage between two nodes or a
//! component's current) from a [`SimulationResult`] that holds a transient,
//! or work on a `(time, value)` trace you already have.
//!
//! # Fourier analysis (`.four`)
//!
//! [`Fourier`] follows ngspice's `.four` and `fourier` command, so its
//! numbers can be checked against ngspice's:
//!
//! 1. Take the last period of the run, `[tstop − 1/f0, tstop]`.
//! 2. Resample it linearly onto [`grid_points`](Fourier::grid_points)
//!    points (200 by default) at `t_i = tstop − 1/f0 + i/(f0·N)`,
//!    `i = 0 … N−1`.
//! 3. For harmonic `k`, sum `S = Σ x_i·sin(2πki/N)` and
//!    `C = Σ x_i·cos(2πki/N)`. The magnitude is `2/N·√(S² + C²)` and the
//!    phase is `atan2(C, S)` in degrees, measured against a sine that
//!    starts with the period. The DC term is the mean.
//! 4. THD is `100·√(Σ_{k≥2} mag_k²) / mag_1`, over the harmonics asked for
//!    (9 above DC by default, ngspice's 10 frequencies).
//!
//! The run must reach at least one period, and should have settled by its
//! last period: start-up transients leak into every harmonic. Steps of at
//! most `1/(f0·N)` keep the linear resample faithful; ngspice's
//! `.tran tstep` is usually set to that.
//!
//! # FFT
//!
//! [`Fft`] resamples a span of the trace onto a uniform grid and takes a
//! radix-2 FFT, after multiplying by a [`Window`]:
//!
//! - **Resampling:** [`points`](Fft::points) samples (a power of two; by
//!   default the smallest power of two at least the number of steps in the
//!   span) at `t_i = from + i·(to − from)/N`, `i = 0 … N−1`, linearly
//!   interpolated between steps. The sample at `to` is left out, so a span
//!   of whole periods repeats seamlessly. The sample rate is
//!   `N/(to − from)` and the bins are `1/(to − from)` apart.
//! - **Window:** periodic (DFT-even) forms, `w_i` for `i = 0 … N−1` with
//!   period `N`; Hann by default. See [`Window`] for each one's
//!   coefficients and leakage.
//! - **Scaling:** each bin's [`magnitude`](SpectrumBin::magnitude) is the
//!   peak amplitude of the sine it stands for: `2·|X_k| / Σw` (`|X_k| / Σw`
//!   for DC and the Nyquist bin), so a tone centred on a bin reads its own
//!   amplitude whatever the window. Phases are in degrees against a sine
//!   that starts at `from`, as `.four`'s are.
//!
//! A tone centred on bin `k` spreads over bins `k − h … k + h` and leaves
//! the rest at rounding level, with `h` the window's
//! [`half_width`](Window::half_width): 0 for rectangular, 1 for Hann and
//! Hamming, 2 for Blackman, 4 for flat-top. A tone between bins leaks
//! further, by the window's side lobes.
//!
//! ```
//! use sindr::fourier::{Fft, Fourier, Window};
//! use sindr::measure::Signal;
//! use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig, Waveform};
//! use sindr::element::{Resistor, VoltageSource};
//!
//! // A 1 kHz, 1 V sine across a divider: half of it at `out`.
//! let circuit = Circuit::new("0", vec![
//!     CircuitElement::VoltageSource(VoltageSource {
//!         id: "V1".into(),
//!         nodes: ["in".into(), "0".into()],
//!         voltage: 0.0,
//!         waveform: Some(Waveform::Sine { amplitude: 1.0, frequency: 1e3, offset: 0.0, phase: 0.0 }),
//!     }),
//!     CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()], resistance: 1e3 }),
//!     CircuitElement::Resistor(Resistor { id: "R2".into(), nodes: ["out".into(), "0".into()], resistance: 1e3 }),
//! ]);
//! let result = solve_transient(&circuit, &TransientConfig::new(2e-3, 5e-6))?;
//!
//! let four = Fourier::new(1e3).on(&result, &Signal::v("out"))?;
//! assert!((four.harmonics[0].magnitude - 0.5).abs() < 1e-3);
//! assert!(four.thd_percent < 0.1);
//!
//! let spectrum = Fft::new().window(Window::Hann).on(&result, &Signal::v("out"))?;
//! let peak = spectrum.peak().unwrap();
//! assert!((peak.frequency - 1e3).abs() < 1.0);
//! # Ok::<(), sindr::SimError>(())
//! ```

use std::f64::consts::PI;

use crate::error::SimError;
use crate::measure::{interpolate, Measurable, Signal};
use crate::results::SimulationResult;

fn spectrum_error(signal: Option<&Signal>, reason: impl std::fmt::Display) -> SimError {
    SimError::Spectrum {
        reason: match signal {
            Some(s) => format!("`{s}`: {reason}"),
            None => reason.to_string(),
        },
    }
}

/// The signal's transient trace, or why there isn't one.
fn transient_trace(
    result: &SimulationResult,
    signal: &Signal,
) -> Result<Vec<(f64, f64)>, SimError> {
    if result.transient.is_none() {
        return Err(spectrum_error(
            Some(signal),
            "the result has no transient to take a spectrum of",
        ));
    }
    result
        .trace(signal)
        .map_err(|reason| spectrum_error(Some(signal), reason))
}

fn check_trace(trace: &[(f64, f64)]) -> Result<(), String> {
    if trace.len() < 2 {
        return Err(format!(
            "a trace of {} point(s) has no time axis to analyse",
            trace.len()
        ));
    }
    if trace.iter().any(|(t, v)| !t.is_finite() || !v.is_finite()) {
        return Err("the trace holds a value that isn't finite".into());
    }
    if trace.windows(2).any(|w| w[1].0 < w[0].0) {
        return Err("the trace's times aren't in ascending order".into());
    }
    Ok(())
}

/// `n` samples of `trace` at `start + i·span/n`, `i = 0 … n−1`.
fn resample(trace: &[(f64, f64)], start: f64, span: f64, n: usize) -> Vec<f64> {
    let last = trace[trace.len() - 1].0;
    (0..n)
        .map(|i| {
            // Rounding can put the last instant a hair past the end.
            let t = (start + span * i as f64 / n as f64).min(last);
            interpolate(trace, t).unwrap_or(trace[trace.len() - 1].1)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Fourier (.four)
// ---------------------------------------------------------------------------

/// A Fourier analysis at a fundamental frequency: SPICE's `.four`. See the
/// [module docs](self) for the algorithm.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Fourier {
    /// The fundamental frequency (Hz).
    pub fundamental: f64,
    /// How many harmonics above DC to report, the fundamental included.
    /// 9 by default: ngspice's 10 frequencies, DC counted.
    pub harmonics: usize,
    /// Points the last period is resampled onto. 200 by default, as
    /// ngspice's.
    pub grid_points: usize,
}

impl Fourier {
    /// A Fourier analysis at `fundamental` Hz, with ngspice's defaults: 9
    /// harmonics above DC on a 200-point grid.
    pub fn new(fundamental: f64) -> Self {
        Fourier {
            fundamental,
            harmonics: 9,
            grid_points: 200,
        }
    }

    /// Report `n` harmonics above DC, the fundamental included.
    pub fn harmonics(mut self, n: usize) -> Self {
        self.harmonics = n;
        self
    }

    /// Resample the last period onto `n` points.
    pub fn grid_points(mut self, n: usize) -> Self {
        self.grid_points = n;
        self
    }

    /// The harmonics of `signal` in a transient `result`.
    ///
    /// # Errors
    ///
    /// [`SimError::Spectrum`] when the result has no transient, the signal
    /// isn't in it, the run is shorter than one period, or the settings
    /// can't be used (see [`Fourier::of`]).
    pub fn on(
        &self,
        result: &SimulationResult,
        signal: &Signal,
    ) -> Result<FourierResult, SimError> {
        let trace = transient_trace(result, signal)?;
        self.analyse(&trace)
            .map_err(|reason| spectrum_error(Some(signal), reason))
    }

    /// The harmonics of a `(time in s, value)` trace in ascending time,
    /// such as [`TransientData::node_voltage_curve`](crate::TransientData::node_voltage_curve).
    ///
    /// # Errors
    ///
    /// [`SimError::Spectrum`] when the fundamental isn't a positive finite
    /// frequency, there are no harmonics or fewer grid points than twice
    /// the highest harmonic, the trace is shorter than one period, or it
    /// isn't finite and in ascending time.
    pub fn of(&self, trace: &[(f64, f64)]) -> Result<FourierResult, SimError> {
        self.analyse(trace).map_err(|r| spectrum_error(None, r))
    }

    fn analyse(&self, trace: &[(f64, f64)]) -> Result<FourierResult, String> {
        let f0 = self.fundamental;
        if !(f0.is_finite() && f0 > 0.0) {
            return Err(format!("fundamental {f0} Hz isn't a positive frequency"));
        }
        if self.harmonics == 0 {
            return Err("no harmonics asked for".into());
        }
        let n = self.grid_points;
        if n < 2 * self.harmonics + 1 {
            return Err(format!(
                "{n} grid points can't resolve {} harmonics: at least {} are needed",
                self.harmonics,
                2 * self.harmonics + 1
            ));
        }
        check_trace(trace)?;
        let (start, stop) = (trace[0].0, trace[trace.len() - 1].0);
        let period = 1.0 / f0;
        // Allow for rounding in a run meant to be exactly one period long.
        if stop - start < period * (1.0 - 1e-9) {
            return Err(format!(
                "the run, {start} to {stop} s, is shorter than one period of {f0} Hz ({period} s)"
            ));
        }
        let from = (stop - period).max(start);
        let x = resample(trace, from, stop - from, n);

        let dc = x.iter().sum::<f64>() / n as f64;
        let scale = 2.0 / n as f64;
        let harmonics: Vec<(f64, f64)> = (1..=self.harmonics)
            .map(|k| {
                let (mut s, mut c) = (0.0, 0.0);
                for (i, xi) in x.iter().enumerate() {
                    let arg = 2.0 * PI * ((k * i) % n) as f64 / n as f64;
                    s += xi * arg.sin();
                    c += xi * arg.cos();
                }
                let (s, c) = (s * scale, c * scale);
                (s.hypot(c), c.atan2(s).to_degrees())
            })
            .collect();

        let (mag1, phase1) = harmonics[0];
        let normalised = |m: f64| if mag1 > 0.0 { m / mag1 } else { f64::NAN };
        let thd_percent = 100.0
            * harmonics[1..]
                .iter()
                .map(|(m, _)| normalised(*m).powi(2))
                .sum::<f64>()
                .sqrt();
        Ok(FourierResult {
            fundamental: f0,
            dc,
            harmonics: harmonics
                .iter()
                .enumerate()
                .map(|(i, &(magnitude, phase))| Harmonic {
                    number: i + 1,
                    frequency: (i + 1) as f64 * f0,
                    magnitude,
                    phase,
                    normalized_magnitude: normalised(magnitude),
                    normalized_phase: phase - phase1,
                })
                .collect(),
            thd_percent,
            grid_points: n,
        })
    }
}

/// A Fourier analysis's answer: the DC term, each harmonic and the THD.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct FourierResult {
    /// The fundamental frequency (Hz).
    pub fundamental: f64,
    /// The mean over the last period: harmonic 0 in ngspice's table.
    pub dc: f64,
    /// Harmonics 1 (the fundamental) up, in order.
    pub harmonics: Vec<Harmonic>,
    /// Total harmonic distortion (%): the harmonics above the fundamental,
    /// root-sum-squared, over the fundamental. NaN when the fundamental's
    /// magnitude is zero.
    pub thd_percent: f64,
    /// Points the last period was resampled onto.
    pub grid_points: usize,
}

impl FourierResult {
    /// Harmonic `k` (1 is the fundamental), if it was asked for.
    pub fn harmonic(&self, k: usize) -> Option<&Harmonic> {
        k.checked_sub(1).and_then(|i| self.harmonics.get(i))
    }
}

/// One harmonic of a [`FourierResult`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Harmonic {
    /// Which harmonic: 1 is the fundamental.
    pub number: usize,
    /// Its frequency (Hz).
    pub frequency: f64,
    /// Its peak amplitude, in the signal's unit.
    pub magnitude: f64,
    /// Its phase (degrees) against a sine starting with the period.
    pub phase: f64,
    /// Its magnitude over the fundamental's.
    pub normalized_magnitude: f64,
    /// Its phase less the fundamental's (degrees).
    pub normalized_phase: f64,
}

// ---------------------------------------------------------------------------
// FFT
// ---------------------------------------------------------------------------

/// The window an [`Fft`] multiplies its samples by, in periodic (DFT-even)
/// form: `w_i = Σ_m (−1)^m·a_m·cos(2πmi/N)` for `i = 0 … N−1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[non_exhaustive]
pub enum Window {
    /// No window (`a = 1`): the narrowest peak and the most leakage, −13 dB
    /// side lobes. Exact for a span of whole periods.
    Rectangular,
    /// Hann (`a = 0.5, 0.5`): side lobes from −31 dB, falling fast. The
    /// default, as ngspice's `fft`.
    #[default]
    Hann,
    /// Hamming (`a = 0.54, 0.46`): a lower first side lobe (−43 dB) that
    /// falls slowly.
    Hamming,
    /// Blackman (`a = 0.42, 0.5, 0.08`): side lobes from −58 dB.
    Blackman,
    /// Flat-top (`a = 0.21557895, 0.41663158, 0.277263158, 0.083578947,
    /// 0.006947368`): reads a tone's amplitude to within 0.01 dB wherever
    /// it falls between bins, at the cost of a wide peak.
    FlatTop,
}

impl Window {
    fn terms(self) -> &'static [f64] {
        match self {
            Window::Rectangular => &[1.0],
            Window::Hann => &[0.5, 0.5],
            Window::Hamming => &[0.54, 0.46],
            Window::Blackman => &[0.42, 0.5, 0.08],
            Window::FlatTop => &[
                0.215_578_95,
                0.416_631_58,
                0.277_263_158,
                0.083_578_947,
                0.006_947_368,
            ],
        }
    }

    /// The window's value at sample `i` of `n`.
    pub fn coefficient(self, i: usize, n: usize) -> f64 {
        let x = 2.0 * PI * i as f64 / n as f64;
        self.terms()
            .iter()
            .enumerate()
            .map(|(m, &a)| if m % 2 == 0 { a } else { -a } * (m as f64 * x).cos())
            .sum()
    }

    /// How many bins either side of a bin-centred tone it spreads into:
    /// the bins past these hold only rounding.
    pub fn half_width(self) -> usize {
        self.terms().len() - 1
    }
}

/// A windowed FFT of a transient trace. See the [module docs](self) for the
/// resampling, window and scaling.
#[derive(Debug, Clone, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Fft {
    /// The window; Hann by default.
    #[cfg_attr(feature = "serde", serde(default))]
    pub window: Window,
    /// Samples, a power of two. `None` (the default) takes the smallest
    /// power of two at least the number of steps in the span.
    #[cfg_attr(feature = "serde", serde(default))]
    pub points: Option<usize>,
    /// Start of the span (s); the trace's start when `None`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub from: Option<f64>,
    /// End of the span (s); the trace's end when `None`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub to: Option<f64>,
}

impl Fft {
    /// An FFT of the whole trace with a Hann window and as many samples as
    /// it has steps, rounded up to a power of two.
    pub fn new() -> Self {
        Self::default()
    }

    /// Use `window`.
    pub fn window(mut self, window: Window) -> Self {
        self.window = window;
        self
    }

    /// Resample onto `n` points, a power of two.
    pub fn points(mut self, n: usize) -> Self {
        self.points = Some(n);
        self
    }

    /// Take the span `[from, to)` of the trace only: skip a start-up, or
    /// pick whole periods.
    pub fn span(mut self, from: f64, to: f64) -> Self {
        self.from = Some(from);
        self.to = Some(to);
        self
    }

    /// The spectrum of `signal` in a transient `result`.
    ///
    /// # Errors
    ///
    /// [`SimError::Spectrum`] when the result has no transient, the signal
    /// isn't in it, or the settings can't be used (see [`Fft::of`]).
    pub fn on(&self, result: &SimulationResult, signal: &Signal) -> Result<Spectrum, SimError> {
        let trace = transient_trace(result, signal)?;
        self.analyse(&trace)
            .map_err(|reason| spectrum_error(Some(signal), reason))
    }

    /// The spectrum of a `(time in s, value)` trace in ascending time.
    ///
    /// # Errors
    ///
    /// [`SimError::Spectrum`] when the span is empty or outside the trace,
    /// the point count isn't a power of two of at least 2, or the trace
    /// isn't finite and in ascending time.
    pub fn of(&self, trace: &[(f64, f64)]) -> Result<Spectrum, SimError> {
        self.analyse(trace).map_err(|r| spectrum_error(None, r))
    }

    fn analyse(&self, trace: &[(f64, f64)]) -> Result<Spectrum, String> {
        check_trace(trace)?;
        let (first, last) = (trace[0].0, trace[trace.len() - 1].0);
        let from = self.from.unwrap_or(first);
        let to = self.to.unwrap_or(last);
        if !(from.is_finite() && to.is_finite() && to > from) {
            return Err(format!("the span {from} to {to} s is empty"));
        }
        if from < first || to > last {
            return Err(format!(
                "the span {from} to {to} s isn't inside the run, {first} to {last} s"
            ));
        }
        let n = match self.points {
            Some(n) => n,
            None => {
                let steps = trace.iter().filter(|(t, _)| *t >= from && *t < to).count();
                steps.max(2).next_power_of_two()
            }
        };
        if n < 2 || !n.is_power_of_two() {
            return Err(format!("{n} points isn't a power of two of at least 2"));
        }

        let span = to - from;
        let samples = resample(trace, from, span, n);
        let weights: Vec<f64> = (0..n).map(|i| self.window.coefficient(i, n)).collect();
        let gain: f64 = weights.iter().sum();
        let mut re: Vec<f64> = samples.iter().zip(&weights).map(|(x, w)| x * w).collect();
        let mut im = vec![0.0; n];
        fft_in_place(&mut re, &mut im);

        let bins = (0..=n / 2)
            .map(|k| {
                let one_sided = if k == 0 || k == n / 2 { 1.0 } else { 2.0 };
                // X_k = C − jS, so the sine-referenced phase is atan2(C, S).
                let (c, s) = (re[k], -im[k]);
                SpectrumBin {
                    frequency: k as f64 / span,
                    magnitude: one_sided * c.hypot(s) / gain,
                    phase: c.atan2(s).to_degrees(),
                }
            })
            .collect();
        Ok(Spectrum {
            window: self.window,
            points: n,
            from,
            to,
            sample_rate: n as f64 / span,
            resolution: 1.0 / span,
            bins,
        })
    }
}

/// Iterative radix-2 Cooley–Tukey, forward, unscaled. `re.len()` is a power
/// of two.
fn fft_in_place(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        for m in 0..half {
            let angle = -2.0 * PI * m as f64 / len as f64;
            let (w_im, w_re) = angle.sin_cos();
            for start in (0..n).step_by(len) {
                let (a, b) = (start + m, start + m + half);
                let t_re = re[b] * w_re - im[b] * w_im;
                let t_im = re[b] * w_im + im[b] * w_re;
                re[b] = re[a] - t_re;
                im[b] = im[a] - t_im;
                re[a] += t_re;
                im[a] += t_im;
            }
        }
        len *= 2;
    }
}

/// An [`Fft`]'s answer: one-sided bins from DC to Nyquist.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Spectrum {
    /// The window used.
    pub window: Window,
    /// Samples the span was resampled onto.
    pub points: usize,
    /// Start of the span (s).
    pub from: f64,
    /// End of the span (s).
    pub to: f64,
    /// Samples per second of the resample (Hz).
    pub sample_rate: f64,
    /// Spacing of the bins (Hz): one over the span.
    pub resolution: f64,
    /// Bins `0 … points/2`, at `k·resolution`.
    pub bins: Vec<SpectrumBin>,
}

impl Spectrum {
    /// The largest bin above DC, or `None` when there's only DC.
    pub fn peak(&self) -> Option<&SpectrumBin> {
        self.bins
            .iter()
            .skip(1)
            .max_by(|a, b| a.magnitude.total_cmp(&b.magnitude))
    }
}

/// One bin of a [`Spectrum`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct SpectrumBin {
    /// The bin's frequency (Hz).
    pub frequency: f64,
    /// The peak amplitude of a sine centred on this bin, in the signal's
    /// unit (the window's gain divided out).
    pub magnitude: f64,
    /// Phase (degrees) against a sine starting at the span's start.
    pub phase: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sine(amplitude: f64, f: f64, phase_deg: f64, t_end: f64, steps: usize) -> Vec<(f64, f64)> {
        (0..=steps)
            .map(|i| {
                let t = t_end * i as f64 / steps as f64;
                (
                    t,
                    amplitude * (2.0 * PI * f * t + phase_deg.to_radians()).sin(),
                )
            })
            .collect()
    }

    #[test]
    fn fourier_of_a_pure_sine_is_its_fundamental() {
        let trace = sine(2.0, 1e3, 30.0, 3e-3, 3000);
        let r = Fourier::new(1e3).of(&trace).unwrap();
        let h1 = r.harmonic(1).unwrap();
        assert!((h1.magnitude - 2.0).abs() < 1e-4, "{h1:?}");
        assert!((h1.phase - 30.0).abs() < 0.01, "{h1:?}");
        assert!(r.dc.abs() < 1e-6);
        assert!(r.thd_percent < 1e-3, "{}", r.thd_percent);
        assert_eq!(r.harmonics.len(), 9);
        assert_eq!(r.harmonic(3).unwrap().frequency, 3e3);
    }

    #[test]
    fn fourier_reads_a_square_waves_odd_harmonics() {
        // ±1 square: odd harmonics at 4/(πk), THD √(π²/8 − 1) ≈ 48.3 %.
        let trace: Vec<_> = (0..=4000)
            .map(|i| {
                let t = 2e-3 * i as f64 / 4000.0;
                (t, if (t * 1e3).fract() < 0.5 { 1.0 } else { -1.0 })
            })
            .collect();
        let r = Fourier::new(1e3)
            .harmonics(99)
            .grid_points(4000)
            .of(&trace)
            .unwrap();
        for k in [1, 3, 5] {
            let m = r.harmonic(k).unwrap().magnitude;
            assert!(
                (m - 4.0 / (PI * k as f64)).abs() < 2e-3,
                "harmonic {k}: {m}"
            );
        }
        assert!(r.harmonic(2).unwrap().magnitude < 2e-3);
        assert!((r.thd_percent - 48.3).abs() < 1.0, "{}", r.thd_percent);
    }

    #[test]
    fn fourier_refuses_a_run_shorter_than_a_period() {
        let trace = sine(1.0, 1e3, 0.0, 0.5e-3, 100);
        let err = Fourier::new(1e3).of(&trace).unwrap_err();
        assert!(err.to_string().contains("shorter than one period"), "{err}");
        assert!(Fourier::new(0.0).of(&trace).is_err());
        assert!(Fourier::new(1e3).grid_points(10).of(&trace).is_err());
    }

    #[test]
    fn fft_matches_a_direct_dft() {
        let x: Vec<f64> = (0..16).map(|i| ((i * 7 % 5) as f64) - 1.3).collect();
        let (mut re, mut im) = (x.clone(), vec![0.0; 16]);
        fft_in_place(&mut re, &mut im);
        for k in 0..16 {
            let (mut dr, mut di) = (0.0, 0.0);
            for (i, xi) in x.iter().enumerate() {
                let a = -2.0 * PI * (k * i) as f64 / 16.0;
                dr += xi * a.cos();
                di += xi * a.sin();
            }
            assert!(
                (re[k] - dr).abs() < 1e-9 && (im[k] - di).abs() < 1e-9,
                "bin {k}"
            );
        }
    }

    #[test]
    fn a_bin_centred_sine_stays_within_each_windows_half_width() {
        // 8 periods of 1 kHz over 8 ms: the tone sits on bin 8.
        let trace = sine(1.5, 1e3, 0.0, 8e-3, 8192);
        for window in [
            Window::Rectangular,
            Window::Hann,
            Window::Hamming,
            Window::Blackman,
            Window::FlatTop,
        ] {
            let s = Fft::new().window(window).points(1024).of(&trace).unwrap();
            assert_eq!(s.resolution, 125.0);
            let peak = s.peak().unwrap();
            assert_eq!(peak.frequency, 1e3, "{window:?}");
            assert!((peak.magnitude - 1.5).abs() < 1e-9, "{window:?}: {peak:?}");
            assert!(peak.phase.abs() < 1e-6, "{window:?}: {peak:?}");
            let h = window.half_width();
            for (k, bin) in s.bins.iter().enumerate() {
                if k.abs_diff(8) > h {
                    assert!(
                        bin.magnitude < 1e-12,
                        "{window:?} bin {k}: {}",
                        bin.magnitude
                    );
                }
            }
        }
    }

    #[test]
    fn fft_default_points_round_up_the_step_count() {
        let trace = sine(1.0, 1e3, 0.0, 1e-3, 300);
        let s = Fft::new().of(&trace).unwrap();
        assert_eq!(s.points, 512);
        assert_eq!(s.bins.len(), 257);
        assert!(Fft::new().points(300).of(&trace).is_err());
        assert!(Fft::new().span(0.0, 2e-3).of(&trace).is_err());
    }
}
