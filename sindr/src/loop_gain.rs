//! Loop gain and stability margins at a break point, by Tian's method.
//!
//! The loop is broken by a probe: an independent voltage source (normally
//! 0 V) in series with the loop, its `nodes[1]` on the side that drives
//! the loop (an amplifier output, say) and its `nodes[0]` on the side that
//! is driven (the feedback network). Nothing else is cut, so the loading
//! of both sides stays as it is.
//!
//! At each frequency two small-signal solves inject a test signal at the
//! probe: a unit voltage in series (the probe's own value), then a unit
//! current into `nodes[0]` with the probe a short. With `v` the voltage at
//! `nodes[0]` in the first and `i` the probe's current (from `nodes[0]`
//! through it to `nodes[1]`) in the second, the loop gain is
//!
//! ```text
//!   T = (i − v) / (v + 1 − i)
//! ```
//!
//! This is Middlebrook's double-injection result,
//! `T = (Tv·Ti − 1) / (Tv + Ti + 2)` with `Tv` and `Ti` the voltage and
//! current loop gains measured at the same point, in the single-probe form
//! Tian, Kolen et al. gave. It is exact for loops that carry signal both
//! ways through the break point (finite source and load impedances,
//! bidirectional feedback), where a voltage or current injection alone is
//! only an approximation.
//!
//! `T` is the return ratio: positive at low frequency for negative feedback,
//! and the closed-loop response goes as `1/(1 + T)`.

use crate::element::VoltageSource;
use crate::handle::ComponentId;
use num_complex::Complex64;

use crate::ac_analysis::{
    at_junction_temperatures, bias_point_with_result, generate_frequencies, incremental_inductors,
    linearize_at_dc, small_signal_system, validate_config, AcConfig, FrequencySpacing,
    SmallSignalModel,
};
use crate::circuit::{Circuit, CircuitElement};
use crate::error::{ConfigKind, SimError};
use crate::node_map::NodeMap;

/// Loop-gain analysis configuration. Build one with [`LoopGainConfig::new`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct LoopGainConfig {
    /// Id of the probe: an independent voltage source in series with the
    /// loop, `nodes[1]` on the driving side and `nodes[0]` on the driven
    /// side. Its DC value biases the circuit as usual; 0 V leaves it out.
    pub probe: ComponentId,
    /// Start frequency (Hz).
    pub f_start: f64,
    /// Stop frequency (Hz).
    pub f_stop: f64,
    /// Number of frequency points, both ends included.
    pub num_points: usize,
    /// Frequency spacing. Default logarithmic.
    pub spacing: FrequencySpacing,
}

impl LoopGainConfig {
    /// The loop gain at the probe `probe`, at `num_points` frequencies from
    /// `f_start` to `f_stop` (Hz), log-spaced.
    ///
    /// ```
    /// use sindr::loop_gain::LoopGainConfig;
    ///
    /// let config = LoopGainConfig::new("Vprobe", 1.0, 100e6, 801);
    /// assert_eq!(config.probe, "Vprobe");
    /// ```
    pub fn new(
        probe: impl Into<ComponentId>,
        f_start: f64,
        f_stop: f64,
        num_points: usize,
    ) -> Self {
        Self {
            probe: probe.into(),
            f_start,
            f_stop,
            num_points,
            spacing: FrequencySpacing::Logarithmic,
        }
    }

    /// Sets the frequency spacing.
    pub fn with_spacing(mut self, spacing: FrequencySpacing) -> Self {
        self.spacing = spacing;
        self
    }

    fn sweep(&self) -> AcConfig {
        AcConfig::new(
            self.probe.clone(),
            self.f_start,
            self.f_stop,
            self.num_points,
        )
        .with_spacing(self.spacing)
    }
}

/// The loop gain at one frequency.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct LoopGainPoint {
    /// Frequency (Hz).
    pub frequency: f64,
    /// The loop gain `T(j·2πf)`.
    #[cfg_attr(feature = "schemars", schemars(with = "[f64; 2]"))]
    pub gain: Complex64,
    /// Phase of `T` (degrees), unwrapped along the sweep. The first point's
    /// turn is picked from the slope of `|T|` there (`|T| ∝ f^k` is near
    /// `90°·k`), so a sweep starting past a double pole starts below −180°.
    pub phase_deg: f64,
}

impl LoopGainPoint {
    /// `|T|` in dB.
    pub fn gain_db(&self) -> f64 {
        20.0 * self.gain.norm().log10()
    }
}

/// Result of a loop-gain analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct LoopGainResult {
    /// The loop gain at every frequency of the sweep.
    pub points: Vec<LoopGainPoint>,
    /// Frequencies (Hz) where `|T|` crosses 1 (0 dB), lowest first.
    pub gain_crossovers: Vec<f64>,
    /// Frequencies (Hz) where the unwrapped phase of `T` crosses −180°
    /// (or −540°, …), lowest first.
    pub phase_crossovers: Vec<f64>,
    /// Phase margin (degrees): at each gain crossover, how far `∠T` is from
    /// −180° (or another odd multiple of 180°), positive on the stable side,
    /// in (−180°, 180°]; the smallest over all of them. A crossover past
    /// −540° of unwrapped phase is a margin of a few degrees either side of
    /// zero, not one below −360°, and a crossover with a phase lead (the
    /// low one of an AC-coupled loop) is measured to +180°. `None` when
    /// `|T|` never crosses 1 in the sweep.
    pub phase_margin_deg: Option<f64>,
    /// Gain margin (dB): `−20·log₁₀|T|` at the phase crossover, the
    /// smallest over all of them. `None` when the phase never reaches
    /// −180° in the sweep.
    pub gain_margin_db: Option<f64>,
    /// Configuration used to produce this result.
    pub config: LoopGainConfig,
}

/// Runs a loop-gain analysis at a probe, with phase and gain margins.
///
/// # Algorithm
///
/// 1. Solve the DC operating point and linearise every nonlinear element
///    at it, as [`solve_ac`](crate::ac_analysis::solve_ac) does.
/// 2. At each frequency, inject a unit voltage at the probe and then a
///    unit current beside it, and combine the two responses into the loop
///    gain `T` by Tian's formula (see the [module documentation](self)).
/// 3. Find where `|T|` crosses 1 and where its unwrapped phase crosses
///    −180° between sweep points, refine each crossing by bisection on `T`
///    itself (so a margin is exact, not interpolated), and read the
///    margins there.
///
/// The phase is unwrapped along the sweep, so the sweep must be dense
/// enough that the phase moves less than 180° between points.
///
/// # Errors
///
/// - [`SimError::InvalidConfig`] if the sweep is invalid (as for
///   [`AcConfig`]), or the probe is not an independent voltage source with
///   neither terminal on ground. Checked before the DC solve
/// - Any [`SimError`] from the DC operating-point solve
/// - [`SimError::SingularMatrix`] if the linearised system is singular at
///   some frequency
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement, OpAmpModel};
/// use sindr::loop_gain::{solve_loop_gain, LoopGainConfig};
/// use sindr::element::{OpAmp, Resistor, VoltageSource};
///
/// // A follower on a single-pole op-amp (100 dB, 1 MHz GBW), broken at
/// // its output: T = A(s), phase margin just over 90°.
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::OpAmp(OpAmp {
///             id: "U1".into(),
///             nodes: ["0".into(), "fb".into(), "out".into()],
///             v_pos: 15.0,
///             v_neg: -15.0,
///             model: Some(OpAmpModel::default()),
///         }),
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "Vprobe".into(),
///             nodes: ["fb".into(), "out".into()],
///             voltage: 0.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "RL".into(),
///             nodes: ["out".into(), "0".into()],
///             resistance: 10e3,
///         }),
///     ],
///     ..Default::default()
/// };
///
/// let lg = solve_loop_gain(&circuit, &LoopGainConfig::new("Vprobe", 1.0, 100e6, 81))?;
/// let pm = lg.phase_margin_deg.unwrap();
/// assert!((pm - 90.0).abs() < 1e-3);
/// assert!((lg.gain_crossovers[0] - 1e6).abs() < 1.0);
/// assert!(lg.gain_margin_db.is_none());
/// # Ok::<(), sindr::SimError>(())
/// ```
///
pub fn solve_loop_gain(
    circuit: &Circuit,
    config: &LoopGainConfig,
) -> Result<LoopGainResult, SimError> {
    solve_loop_gain_with(circuit, config, &crate::SolverOptions::default())
}

/// [`solve_loop_gain`] with `opts`.
pub(crate) fn solve_loop_gain_with(
    circuit: &Circuit,
    config: &LoopGainConfig,
    opts: &crate::SolverOptions,
) -> Result<LoopGainResult, SimError> {
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    let sweep = config.sweep();
    validate_config(&sweep)?;
    let probe_nodes = circuit
        .components
        .iter()
        .find_map(|c| match c {
            CircuitElement::VoltageSource(VoltageSource { id, nodes, .. })
                if *id == config.probe =>
            {
                Some(nodes.clone())
            }
            _ => None,
        })
        .ok_or_else(|| {
            SimError::config(
                ConfigKind::LoopGain,
                format!(
                    "probe `{}` is not an independent voltage source",
                    config.probe
                ),
            )
        })?;
    if probe_nodes.iter().any(|n| *n == circuit.ground_node) {
        return Err(SimError::config(
            ConfigKind::LoopGain,
            format!(
                "probe `{}` has a terminal on ground; it must sit in series with the loop",
                config.probe
            ),
        ));
    }

    let (node_map, dc_solution, dc) = bias_point_with_result(circuit, opts)?;
    let circuit = &incremental_inductors(&at_junction_temperatures(circuit, &dc), &dc);
    let ss_models = linearize_at_dc(circuit, &node_map, &dc_solution);
    let probe = Probe {
        circuit,
        node_map: &node_map,
        dc_solution: &dc_solution,
        ss_models: &ss_models,
        id: &config.probe,
        driven: node_map.index(&probe_nodes[0]),
    };

    let mut points: Vec<LoopGainPoint> = Vec::new();
    for f in generate_frequencies(&sweep) {
        let gain = probe.at(f)?;
        let phase_deg = match points.last() {
            Some(prev) => unwrap_near(gain, prev.phase_deg),
            None => first_phase(gain, probe.at(f * SLOPE_STEP)?),
        };
        points.push(LoopGainPoint {
            frequency: f,
            gain,
            phase_deg,
        });
    }

    let mut gain_crossovers = Vec::new();
    let mut phase_crossovers = Vec::new();
    let mut phase_margin_deg: Option<f64> = None;
    let mut gain_margin_db: Option<f64> = None;
    for pair in points.windows(2) {
        let (lo, hi) = (&pair[0], &pair[1]);
        let (m_lo, m_hi) = (lo.gain.norm().ln(), hi.gain.norm().ln());
        if (m_lo > 0.0) != (m_hi > 0.0) {
            let f = bisect(lo.frequency, hi.frequency, m_lo, |f| {
                Ok(probe.at(f)?.norm().ln())
            })?;
            let phase = unwrap_near(probe.at(f)?, lo.phase_deg);
            let pm = phase_margin(phase);
            phase_margin_deg = Some(phase_margin_deg.map_or(pm, |m| m.min(pm)));
            gain_crossovers.push(f);
        }
        // The −180° − k·360° level between the two phases, if any.
        let level = ((lo.phase_deg.max(hi.phase_deg) + 180.0) / 360.0).floor() * 360.0 - 180.0;
        let (a, b) = (lo.phase_deg - level, hi.phase_deg - level);
        if (a > 0.0) != (b > 0.0) {
            let f = bisect(lo.frequency, hi.frequency, a, |f| {
                Ok(unwrap_near(probe.at(f)?, lo.phase_deg) - level)
            })?;
            let gm = -20.0 * probe.at(f)?.norm().log10();
            gain_margin_db = Some(gain_margin_db.map_or(gm, |m| m.min(gm)));
            phase_crossovers.push(f);
        }
    }

    Ok(LoopGainResult {
        points,
        gain_crossovers,
        phase_crossovers,
        phase_margin_deg,
        gain_margin_db,
        config: config.clone(),
    })
}

/// The linearised circuit around a probe, to evaluate `T` at any frequency.
struct Probe<'a> {
    circuit: &'a Circuit,
    node_map: &'a NodeMap,
    dc_solution: &'a nalgebra::DVector<f64>,
    ss_models: &'a SmallSignalModel,
    id: &'a str,
    /// Row of the probe's driven terminal, `nodes[0]`.
    driven: Option<usize>,
}

impl Probe<'_> {
    /// `T(j·2πf)`.
    fn at(&self, f: f64) -> Result<Complex64, SimError> {
        let omega = 2.0 * std::f64::consts::PI * f;
        let system = small_signal_system(
            self.circuit,
            self.node_map,
            self.dc_solution,
            self.ss_models,
            omega,
        );
        let (Some(k), Some(p)) = (system.vsource_branch(self.id), self.driven) else {
            return Err(SimError::config(
                ConfigKind::LoopGain,
                format!("probe `{}` is not an independent voltage source", self.id),
            ));
        };
        let zero = nalgebra::DVector::from_element(system.size(), Complex64::new(0.0, 0.0));
        // Voltage injection: the probe at 1, everything else zero.
        let mut b = zero.clone();
        b[k] = Complex64::new(1.0, 0.0);
        let v = system.solve(&b)?[p];
        // Current injection: 1 A into the driven side, the probe a short.
        let mut b = zero;
        b[p] = Complex64::new(1.0, 0.0);
        let i = system.solve(&b)?[k];
        let one = Complex64::new(1.0, 0.0);
        Ok((i - v) / (v + one - i))
    }
}

/// `deg` moved by whole turns into (−180°, 180°].
fn wrap_degrees(deg: f64) -> f64 {
    deg - ((deg - 180.0) / 360.0).ceil() * 360.0
}

/// The phase margin at a gain crossover whose unwrapped phase is `phase`:
/// how far the phase is from the nearest −180° + k·360°, positive on the
/// stable side. Below −180° that's `180° + phase` moved by whole turns; a
/// phase above 0° (the low crossover of an AC-coupled loop) is measured to
/// +180°, so a lead of 90° is a margin of 90°, not −90°.
fn phase_margin(phase: f64) -> f64 {
    let pm = 180.0 + phase;
    wrap_degrees(if pm > 180.0 { 360.0 - pm } else { pm })
}

/// Frequency ratio over which [`first_phase`] reads the slope of `|T|`.
const SLOPE_STEP: f64 = 1.01;

/// The unwrapped phase of the sweep's first point `t`, with `t_next` the
/// gain at [`SLOPE_STEP`] times its frequency. The principal value can be a
/// turn off: past a double pole, −190° reads as +170°. Bode's gain–phase
/// relation picks the turn: `|T| ∝ f^k` has a phase near `90°·k`. The phase
/// is put within (−270°, +90°] of that, so a sign inversion counts as a lag
/// (−180°) rather than a lead.
fn first_phase(t: Complex64, t_next: Complex64) -> f64 {
    let k = (t_next.norm() / t.norm()).ln() / SLOPE_STEP.ln();
    // A resonance's slope is steep but says nothing past a half turn.
    let estimate = (90.0 * k).clamp(-270.0, 180.0);
    let phase = t.arg().to_degrees();
    phase - ((phase - estimate - 90.0) / 360.0).ceil() * 360.0
}

/// The phase of `z` (degrees), the one of its branches nearest `near`.
fn unwrap_near(z: Complex64, near: f64) -> f64 {
    let phase = z.arg().to_degrees();
    phase + ((near - phase) / 360.0).round() * 360.0
}

/// A root of `h` between `lo` and `hi`, with `h(lo) = h_lo` and `h(hi)` of
/// the other sign: bisection, on a log scale when both ends are positive.
fn bisect(
    mut lo: f64,
    mut hi: f64,
    mut h_lo: f64,
    h: impl Fn(f64) -> Result<f64, SimError>,
) -> Result<f64, SimError> {
    let log = lo > 0.0;
    for _ in 0..80 {
        let mid = if log {
            (lo * hi).sqrt()
        } else {
            0.5 * (lo + hi)
        };
        if mid <= lo || mid >= hi {
            break;
        }
        let h_mid = h(mid)?;
        if h_mid == 0.0 {
            return Ok(mid);
        }
        if h_mid.signum() == h_lo.signum() {
            lo = mid;
            h_lo = h_mid;
        } else {
            hi = mid;
        }
    }
    Ok(if log {
        (lo * hi).sqrt()
    } else {
        0.5 * (lo + hi)
    })
}

#[cfg(test)]
mod tests {
    use super::{first_phase, phase_margin, wrap_degrees, SLOPE_STEP};
    use num_complex::Complex64;

    #[test]
    fn first_phase_takes_the_turn_the_slope_points_to() {
        // `T = k0·(j·f)^n / (1 + j·f)^m`, its true phase and slope known.
        let t = |f: f64, k0: f64, n: i32, m: i32| {
            let jf = Complex64::new(0.0, f);
            k0 * jf.powi(n) / (Complex64::new(1.0, 0.0) + jf).powi(m)
        };
        for (f, k0, n, m, want) in [
            // Past a triple pole's −180°: reads +170° as a principal value.
            (2.0, 1.0, 0, 3, -3.0 * 2.0_f64.atan().to_degrees()),
            (0.01, 1.0, 0, 1, -0.01_f64.atan().to_degrees()),
            // A coupling capacitor's lead, and two of them.
            (0.01, 1.0, 1, 0, 90.0),
            (0.01, 1.0, 2, 0, 180.0),
            // Below a double integrator's −180°.
            (0.01, 1.0, -2, 1, -180.0 - 0.01_f64.atan().to_degrees()),
            // Negative real at low frequency: a lag, whichever side of the
            // real axis it starts.
            (0.01, -1.0, 0, 1, -180.0 - 0.01_f64.atan().to_degrees()),
            (0.01, -1.0, 0, -1, -180.0 + 0.01_f64.atan().to_degrees()),
        ] {
            let got = first_phase(t(f, k0, n, m), t(f * SLOPE_STEP, k0, n, m));
            assert!(
                (got - want).abs() < 1e-9,
                "{f} {k0} {n} {m}: {got} vs {want}"
            );
        }
    }

    #[test]
    fn phase_margin_measures_to_the_nearest_odd_half_turn() {
        for (phase, want) in [
            (-90.0, 90.0),
            (-135.0, 45.0),
            (-200.0, -20.0),
            // A phase lead at the low crossover of an AC-coupled loop.
            (90.0, 90.0),
            (170.0, 10.0),
            (200.0, -20.0),
            // Past −540° of unwrapped phase.
            (-591.6, -51.6),
            (-360.0, 180.0),
        ] {
            let got = phase_margin(phase);
            assert!((got - want).abs() < 1e-9, "{phase}: {got} vs {want}");
        }
    }

    #[test]
    fn wrap_degrees_lands_in_the_half_open_turn() {
        for (deg, want) in [
            (0.0, 0.0),
            (180.0, 180.0),
            (-180.0, 180.0),
            (-20.0, -20.0),
            // 180° + a crossover phase of −560°.
            (-380.0, -20.0),
            (-560.0 + 180.0 - 360.0, -20.0),
            (540.0, 180.0),
            (190.0, -170.0),
        ] {
            let got = wrap_degrees(deg);
            assert!((got - want).abs() < 1e-12, "{deg}: {got} vs {want}");
        }
    }
}
