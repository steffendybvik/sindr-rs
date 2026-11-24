//! Noise analysis, SPICE's `.noise`: thermal, shot and flicker noise at an
//! output voltage, and referred back to an input source.
//!
//! Every noise source is a current between two nodes with a power spectral
//! density (A²/Hz), set by the DC operating point:
//!
//! | Component | Source | Between | PSD |
//! |---|---|---|---|
//! | resistor, potentiometer (each half), thermistor, photoresistor | thermal | its terminals | `4kT/R` |
//! | diode, LED, Schottky, Zener, photodiode | shot | anode–cathode | `2q·\|I\|` |
//! | diode with a `model` | flicker | anode–cathode | `KF·\|I\|^AF / f` |
//! | BJT | shot | collector–emitter, base–emitter | `2q·\|Ic\|`, `2q·\|Ib\|` |
//! | BJT with a `model` | flicker | base–emitter | `KF·\|Ib\|^AF / f` |
//! | MOSFET | channel thermal | drain–source | `(8/3)·kT·gm` |
//! | MOSFET | flicker | drain–source | `KF·\|Id\|^AF / f` |
//! | JFET | channel thermal | drain–source | `(8/3)·kT·gm` |
//!
//! `T` is 300.15 K (27 °C), as in SPICE, except that a thermistor's thermal
//! noise uses its own temperature. Sources, controlled sources, op-amps,
//! regulators, switches, fuses, IGBTs and a diode's series resistance are
//! noiseless.
//!
//! At each frequency one adjoint solve, `Aᵀ·y = e_out`, gives the transfer
//! from a unit current at every node to the output. Each source then adds
//! `|y_p − y_q|²·PSD` to the output's PSD. One forward solve from the input
//! source gives the gain that refers the output noise back to the input.
//! Integrated noise assumes each PSD follows a power law between points, as
//! ngspice does.

use crate::element::{CurrentSource, VoltageSource};
use nalgebra::DVector;
use num_complex::Complex64;

use crate::ac_analysis::{
    bias_point_with_result, generate_frequencies, linearize_at_dc, small_signal_system,
    validate_config, AcConfig, FrequencySpacing,
};
use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::node_map::NodeMap;
use crate::results::SimulationResult;
use crate::transfer_function::Port;

/// Boltzmann constant (J/K).
const BOLTZMANN: f64 = 1.380_649e-23;
/// Elementary charge (C).
const CHARGE: f64 = 1.602_176_634e-19;
/// SPICE's default circuit temperature (K).
const T_NOISE: f64 = 300.15;

/// Noise analysis configuration: SPICE's
/// `.noise V(output[,reference]) input_source {dec|lin} points f_start f_stop`.
/// Build one with [`NoiseConfig::new`] and its `with_*` setters.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct NoiseConfig {
    /// The output node.
    pub output: String,
    /// The node the output is measured against; ground when `None`.
    pub reference: Option<String>,
    /// The independent voltage or current source noise is referred back to.
    pub input_source: String,
    /// Start frequency (Hz), above 0: flicker noise is infinite at DC.
    pub f_start: f64,
    /// Stop frequency (Hz).
    pub f_stop: f64,
    /// Number of frequency points, both ends included.
    pub num_points: usize,
    /// Frequency spacing; logarithmic is `num_points` in total.
    pub spacing: FrequencySpacing,
}

impl NoiseConfig {
    /// Noise at node `output` against ground, referred back to the
    /// independent source `input_source`, at `num_points` frequencies from
    /// `f_start` to `f_stop` (Hz), both included. Spacing defaults to
    /// [`FrequencySpacing::Logarithmic`].
    ///
    /// ```
    /// use sindr::ac_analysis::FrequencySpacing;
    /// use sindr::NoiseConfig;
    ///
    /// // SPICE `.noise v(out,ref) V1 lin 100 1k 100k`.
    /// let config = NoiseConfig::new("out", "V1", 1e3, 1e5, 100)
    ///     .with_reference("ref")
    ///     .with_spacing(FrequencySpacing::Linear);
    /// assert_eq!(config.reference.as_deref(), Some("ref"));
    /// ```
    pub fn new(
        output: impl Into<String>,
        input_source: impl Into<String>,
        f_start: f64,
        f_stop: f64,
        num_points: usize,
    ) -> Self {
        Self {
            output: output.into(),
            reference: None,
            input_source: input_source.into(),
            f_start,
            f_stop,
            num_points,
            spacing: FrequencySpacing::Logarithmic,
        }
    }

    /// Measure the output against node `reference` instead of ground.
    pub fn with_reference(mut self, reference: impl Into<String>) -> Self {
        self.reference = Some(reference.into());
        self
    }

    /// Space the frequencies by `spacing` instead of logarithmically.
    pub fn with_spacing(mut self, spacing: FrequencySpacing) -> Self {
        self.spacing = spacing;
        self
    }
}

/// The physical mechanism behind a noise source.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum NoiseKind {
    /// Johnson–Nyquist noise of a resistance, or a FET channel's.
    Thermal,
    /// Shot noise of a current crossing a junction.
    Shot,
    /// Flicker (1/f) noise.
    Flicker,
}

/// One noise source in the circuit.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct NoiseSource {
    /// Component id (flattened).
    pub component: String,
    /// Which mechanism.
    pub kind: NoiseKind,
    /// Which current it sits on, for parts with more than one: `"ic"`,
    /// `"ib"`, `"top"`/`"bottom"` for a potentiometer; empty otherwise.
    pub branch: String,
}

/// Noise at one frequency.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct NoisePoint {
    /// Frequency (Hz).
    pub frequency: f64,
    /// Output noise power spectral density (V²/Hz).
    pub output_psd: f64,
    /// Noise referred to the input source (V²/Hz for a voltage source,
    /// A²/Hz for a current source): `output_psd / |gain|²`. Infinite where
    /// the gain is zero.
    pub input_psd: f64,
    /// The gain from the input source to the output, `|V(out)/input|`.
    pub gain: f64,
    /// Each source's share of `output_psd` (V²/Hz), in the order of
    /// [`NoiseResult::sources`].
    pub contributions: Vec<f64>,
}

impl NoisePoint {
    /// Output noise density (V/√Hz), what ngspice calls `onoise_spectrum`.
    pub fn output_density(&self) -> f64 {
        self.output_psd.sqrt()
    }

    /// Input-referred noise density (V/√Hz or A/√Hz): `inoise_spectrum`.
    pub fn input_density(&self) -> f64 {
        self.input_psd.sqrt()
    }
}

/// Result of a noise analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct NoiseResult {
    /// Every noise source, in the order of [`NoisePoint::contributions`].
    pub sources: Vec<NoiseSource>,
    /// One point per frequency.
    pub points: Vec<NoisePoint>,
    /// Output noise integrated over the band (V rms): `onoise_total`.
    pub total_output_noise: f64,
    /// Input-referred noise integrated over the band (V or A rms):
    /// `inoise_total`.
    pub total_input_noise: f64,
    /// Each source's output noise integrated over the band (V rms), in the
    /// order of [`sources`](Self::sources).
    pub source_totals: Vec<f64>,
    /// Configuration used to produce this result.
    pub config: NoiseConfig,
}

/// A source: its current runs between `p` and `q`; `psd(f)` in A²/Hz.
struct Generator {
    source: NoiseSource,
    p: Option<usize>,
    q: Option<usize>,
    white: f64,
    /// `KF·|I|^AF`: divided by f.
    flicker: f64,
}

/// Runs a noise analysis, as SPICE's `.noise`. See the [module docs](self)
/// for the sources and the method.
///
/// # Errors
///
/// - [`SimError::InvalidComponent`] if the input isn't an independent
///   voltage or current source, the output or reference node isn't in the
///   circuit, or the frequencies are invalid (`f_start` must be above 0)
/// - Any [`SimError`] from the DC operating point
/// - [`SimError::SingularMatrix`] if the linearised system is singular
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement};
/// use sindr::noise::{solve_noise, NoiseConfig};
/// use sindr::element::{Capacitor, Resistor, VoltageSource};
///
/// // A 1 kΩ / 1 µF low-pass: the output noise integrates to √(kT/C).
/// let circuit = Circuit::new("0", vec![
///     CircuitElement::VoltageSource(VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 0.0, waveform: None }),
///     CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()], resistance: 1e3 }),
///     CircuitElement::Capacitor(Capacitor { id: "C1".into(), nodes: ["out".into(), "0".into()], capacitance: 1e-6 }),
/// ]);
/// let noise = solve_noise(&circuit, &NoiseConfig::new("out", "V1", 0.01, 1e7, 91))?;
/// let kt_over_c = (1.380649e-23 * 300.15 / 1e-6_f64).sqrt();
/// assert!((noise.total_output_noise / kt_over_c - 1.0).abs() < 0.01);
/// # Ok::<(), sindr::SimError>(())
/// ```
///
pub fn solve_noise(circuit: &Circuit, config: &NoiseConfig) -> Result<NoiseResult, SimError> {
    solve_noise_with(circuit, config, &crate::SolverOptions::default())
}

/// [`solve_noise`] with `opts`.
pub(crate) fn solve_noise_with(
    circuit: &Circuit,
    config: &NoiseConfig,
    opts: &crate::SolverOptions,
) -> Result<NoiseResult, SimError> {
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    let invalid = |msg: String| SimError::InvalidComponent(format!("noise: {msg}"));

    let ac = AcConfig::new(
        config.input_source.clone(),
        config.f_start,
        config.f_stop,
        config.num_points,
    )
    .with_spacing(config.spacing);
    validate_config(&ac)?;
    if config.f_start <= 0.0 {
        return Err(invalid(format!(
            "f_start must be above 0 Hz, where flicker noise is finite (got {})",
            config.f_start
        )));
    }
    let node_map = NodeMap::from_circuit(circuit);
    let node = |role: &str, name: &str| match node_map.index(name) {
        None if name != circuit.ground_node => Err(invalid(format!(
            "{role} node `{name}` is not in the circuit"
        ))),
        index => Ok(index),
    };
    let out = node("output", &config.output)?;
    let reference = config
        .reference
        .as_deref()
        .map(|name| node("reference", name))
        .transpose()?
        .flatten();
    let is_input = circuit.components.iter().any(|c| {
        matches!(c, CircuitElement::VoltageSource(VoltageSource { id, .. }) | CircuitElement::CurrentSource(CurrentSource { id, .. }) if *id == config.input_source)
    });
    if !is_input {
        return Err(invalid(format!(
            "input `{}` is not an independent voltage or current source",
            config.input_source
        )));
    }

    let (node_map, dc_solution, dc) = bias_point_with_result(circuit, opts)?;
    let circuit = &crate::ac_analysis::incremental_inductors(
        &crate::ac_analysis::at_junction_temperatures(circuit, &dc),
        &dc,
    );
    let ss_models = linearize_at_dc(circuit, &node_map, &dc_solution);
    let generators = generators(circuit, &node_map, &dc);

    let mut points = Vec::new();
    for frequency in generate_frequencies(&ac) {
        let omega = 2.0 * std::f64::consts::PI * frequency;
        let system = small_signal_system(circuit, &node_map, &dc_solution, &ss_models, omega);
        let size = system.size();

        let mut e_out = DVector::from_element(size, Complex64::new(0.0, 0.0));
        if let Some(i) = out {
            e_out[i] += 1.0;
        }
        if let Some(r) = reference {
            e_out[r] -= 1.0;
        }
        let y = system.solve_transpose(&e_out)?;
        let at = |n: Option<usize>| n.map_or(Complex64::new(0.0, 0.0), |i| y[i]);

        let contributions: Vec<f64> = generators
            .iter()
            .map(|g| (at(g.p) - at(g.q)).norm_sqr() * (g.white + g.flicker / frequency))
            .collect();
        let output_psd: f64 = contributions.iter().sum();

        let input = Port::input(circuit, &node_map, &system, &config.input_source)?;
        let x = system.solve(&input.excitation(size))?;
        let v = |n: Option<usize>| n.map_or(Complex64::new(0.0, 0.0), |i| x[i]);
        let gain = (v(out) - v(reference)).norm();
        let input_psd = if gain > 0.0 {
            output_psd / (gain * gain)
        } else {
            f64::INFINITY
        };

        points.push(NoisePoint {
            frequency,
            output_psd,
            input_psd,
            gain,
            contributions,
        });
    }

    let band = |psd: &dyn Fn(&NoisePoint) -> f64| integrate(&points, psd).sqrt();
    let source_totals = (0..generators.len())
        .map(|k| band(&|p: &NoisePoint| p.contributions[k]))
        .collect();
    Ok(NoiseResult {
        total_output_noise: band(&|p: &NoisePoint| p.output_psd),
        total_input_noise: band(&|p: &NoisePoint| p.input_psd),
        source_totals,
        sources: generators.into_iter().map(|g| g.source).collect(),
        points,
        config: config.clone(),
    })
}

/// `∫ psd df` over the points, taking the PSD as a power law between each
/// pair (exact for 1/f and for the f⁻² roll-off of a pole), as ngspice does.
fn integrate(points: &[NoisePoint], psd: &dyn Fn(&NoisePoint) -> f64) -> f64 {
    points
        .windows(2)
        .map(|w| {
            let (f1, f2) = (w[0].frequency, w[1].frequency);
            let (n1, n2) = (psd(&w[0]), psd(&w[1]));
            if !(n1 > 0.0 && n2 > 0.0 && n1.is_finite() && n2.is_finite()) || f1 <= 0.0 {
                return 0.5 * (n1 + n2) * (f2 - f1);
            }
            let ratio = f2 / f1;
            let exponent = (n2 / n1).ln() / ratio.ln();
            if (exponent + 1.0).abs() < 1e-9 {
                n1 * f1 * ratio.ln()
            } else {
                n1 * f1 / (exponent + 1.0) * (ratio.powf(exponent + 1.0) - 1.0)
            }
        })
        .sum()
}

/// Every noise source in flat `circuit`, at the operating point `dc`.
fn generators(circuit: &Circuit, node_map: &NodeMap, dc: &SimulationResult) -> Vec<Generator> {
    let mut sources = NoiseSources {
        node_map,
        dc,
        out: Vec::new(),
    };
    for c in &circuit.components {
        c.device().noise(&mut sources);
    }
    sources.out
}

/// Where an element's [`Device::noise`] puts its generators, with the
/// operating point they are taken at.
///
/// [`Device::noise`]: crate::element::Device::noise
pub(crate) struct NoiseSources<'a> {
    node_map: &'a NodeMap,
    /// The DC operating point.
    pub(crate) dc: &'a SimulationResult,
    out: Vec<Generator>,
}

impl NoiseSources<'_> {
    /// The DC current through component `id` (0 A if it has no result).
    pub(crate) fn current(&self, id: &str) -> f64 {
        self.dc
            .component_results
            .iter()
            .find(|r| r.id == id)
            .map_or(0.0, |r| r.current_through)
    }

    /// The DC voltage at node `n` (0 V if it has none).
    pub(crate) fn voltage(&self, n: &str) -> f64 {
        self.dc.node_voltages.get(n).copied().unwrap_or(0.0)
    }

    fn push(
        &mut self,
        id: &str,
        kind: NoiseKind,
        branch: &str,
        [a, b]: [&String; 2],
        white: f64,
        flicker: f64,
    ) {
        self.out.push(Generator {
            source: NoiseSource {
                component: id.to_string(),
                kind,
                branch: branch.to_string(),
            },
            p: self.node_map.index(a),
            q: self.node_map.index(b),
            white,
            flicker,
        });
    }

    /// The thermal noise `4kT/R` of a resistance `r` (Ω) at `t` (K) from
    /// `a` to `b`; none for a resistance that isn't positive.
    pub(crate) fn thermal(&mut self, id: &str, branch: &str, ab: [&String; 2], r: f64, t: f64) {
        let white = if r > 0.0 {
            4.0 * BOLTZMANN * t / r
        } else {
            0.0
        };
        self.push(id, NoiseKind::Thermal, branch, ab, white, 0.0);
    }

    /// [`thermal`](Self::thermal) at the 27 °C noise temperature.
    pub(crate) fn thermal_27c(&mut self, id: &str, branch: &str, ab: [&String; 2], r: f64) {
        self.thermal(id, branch, ab, r, T_NOISE);
    }

    /// The shot noise `2q|I|` of a junction current `i` (A) from `a` to `b`.
    pub(crate) fn shot(&mut self, id: &str, branch: &str, ab: [&String; 2], i: f64) {
        self.push(id, NoiseKind::Shot, branch, ab, 2.0 * CHARGE * i.abs(), 0.0);
    }

    /// The flicker noise `KF·|I|^AF / f` of a current `i` (A) from `a` to `b`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn flicker(
        &mut self,
        id: &str,
        branch: &str,
        ab: [&String; 2],
        i: f64,
        kf: f64,
        af: f64,
    ) {
        self.push(
            id,
            NoiseKind::Flicker,
            branch,
            ab,
            0.0,
            kf * i.abs().powf(af),
        );
    }

    /// A FET channel's thermal noise, `(8/3)·kT·|gm|`, from drain `d` to
    /// source `s`.
    pub(crate) fn channel(&mut self, id: &str, ds: [&String; 2], gm: f64) {
        let white = 8.0 / 3.0 * BOLTZMANN * T_NOISE * gm.abs();
        self.push(id, NoiseKind::Thermal, "", ds, white, 0.0);
    }
}
