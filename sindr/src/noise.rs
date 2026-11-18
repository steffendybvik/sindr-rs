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

use std::collections::HashMap;

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
#[derive(Debug, Clone, PartialEq)]
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

/// The physical mechanism behind a noise source.
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
#[derive(Debug, Clone, PartialEq)]
pub struct NoiseSource {
    /// Component id (flattened).
    pub component: String,
    /// Which mechanism.
    pub kind: NoiseKind,
    /// Which current it sits on, for parts with more than one: `"ic"`,
    /// `"ib"`, `"top"`/`"bottom"` for a potentiometer; empty otherwise.
    pub branch: &'static str,
}

/// Noise at one frequency.
#[derive(Debug, Clone, PartialEq)]
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
#[derive(Debug, Clone, PartialEq)]
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
///   voltage or current source, an output node isn't in the circuit, or
///   the frequencies are invalid (`f_start` must be above 0)
/// - Any [`SimError`] from the DC operating point
/// - [`SimError::SingularMatrix`] if the linearised system is singular
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement};
/// use sindr::ac_analysis::FrequencySpacing;
/// use sindr::noise::{solve_noise, NoiseConfig};
///
/// // A 1 kΩ / 1 µF low-pass: the output noise integrates to √(kT/C).
/// let circuit = Circuit::new("0", vec![
///     CircuitElement::VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 0.0, waveform: None },
///     CircuitElement::Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()], resistance: 1e3 },
///     CircuitElement::Capacitor { id: "C1".into(), nodes: ["out".into(), "0".into()], capacitance: 1e-6 },
/// ]);
/// let noise = solve_noise(&circuit, &NoiseConfig {
///     output: "out".into(),
///     reference: None,
///     input_source: "V1".into(),
///     f_start: 0.01,
///     f_stop: 1e7,
///     num_points: 91,
///     spacing: FrequencySpacing::Logarithmic,
/// })?;
/// let kt_over_c = (1.380649e-23 * 300.15 / 1e-6_f64).sqrt();
/// assert!((noise.total_output_noise / kt_over_c - 1.0).abs() < 0.01);
/// # Ok::<(), sindr::SimError>(())
/// ```
pub fn solve_noise(circuit: &Circuit, config: &NoiseConfig) -> Result<NoiseResult, SimError> {
    solve_noise_with_initial_voltages(circuit, config, &HashMap::new())
}

/// [`solve_noise`], seeding the bias point's Newton–Raphson solve as
/// `.nodeset` does.
///
/// # Errors
///
/// As [`solve_noise`].
pub fn solve_noise_with_initial_voltages(
    circuit: &Circuit,
    config: &NoiseConfig,
    initial_voltages: &HashMap<String, f64>,
) -> Result<NoiseResult, SimError> {
    let flat = circuit.flattened()?;
    let circuit: &Circuit = &flat;
    let invalid = |msg: String| SimError::InvalidComponent(format!("noise: {msg}"));

    let ac = AcConfig {
        f_start: config.f_start,
        f_stop: config.f_stop,
        num_points: config.num_points,
        spacing: config.spacing,
        source_id: config.input_source.clone(),
        ac_magnitude: 1.0,
    };
    validate_config(&ac)?;
    if config.f_start <= 0.0 {
        return Err(invalid(format!(
            "f_start must be above 0 Hz, where flicker noise is finite (got {})",
            config.f_start
        )));
    }
    let node_map = NodeMap::from_circuit(circuit);
    let node = |name: &str| match node_map.index(name) {
        None if name != circuit.ground_node => Err(invalid(format!(
            "output node `{name}` is not in the circuit"
        ))),
        index => Ok(index),
    };
    let out = node(&config.output)?;
    let reference = config.reference.as_deref().map(node).transpose()?.flatten();
    let is_input = circuit.components.iter().any(|c| {
        matches!(c, CircuitElement::VoltageSource { id, .. } | CircuitElement::CurrentSource { id, .. } if *id == config.input_source)
    });
    if !is_input {
        return Err(invalid(format!(
            "input `{}` is not an independent voltage or current source",
            config.input_source
        )));
    }

    let (node_map, dc_solution, dc) = bias_point_with_result(circuit, initial_voltages)?;
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
    let idx = |n: &String| node_map.index(n);
    let current = |id: &str| {
        dc.component_results
            .iter()
            .find(|r| r.id == id)
            .map_or(0.0, |r| r.current_through)
    };
    let source = |id: &str, kind, branch| NoiseSource {
        component: id.to_string(),
        kind,
        branch,
    };
    let thermal = |id: &str, branch, a: &String, b: &String, r: f64, t: f64| Generator {
        source: source(id, NoiseKind::Thermal, branch),
        p: idx(a),
        q: idx(b),
        white: if r > 0.0 {
            4.0 * BOLTZMANN * t / r
        } else {
            0.0
        },
        flicker: 0.0,
    };
    let shot = |id: &str, branch, a: &String, b: &String, i: f64| Generator {
        source: source(id, NoiseKind::Shot, branch),
        p: idx(a),
        q: idx(b),
        white: 2.0 * CHARGE * i.abs(),
        flicker: 0.0,
    };
    let flicker = |id: &str, branch, a: &String, b: &String, i: f64, kf: f64, af: f64| Generator {
        source: source(id, NoiseKind::Flicker, branch),
        p: idx(a),
        q: idx(b),
        white: 0.0,
        flicker: kf * i.abs().powf(af),
    };
    let voltage = |n: &String| dc.node_voltages.get(n).copied().unwrap_or(0.0);

    let mut out = Vec::new();
    for c in &circuit.components {
        match c {
            CircuitElement::Resistor {
                id,
                nodes,
                resistance,
            } => out.push(thermal(id, "", &nodes[0], &nodes[1], *resistance, T_NOISE)),
            CircuitElement::Potentiometer {
                id,
                nodes,
                resistance,
                position,
            } => {
                let pos = position.clamp(0.001, 0.999);
                out.push(thermal(
                    id,
                    "top",
                    &nodes[0],
                    &nodes[1],
                    resistance * pos,
                    T_NOISE,
                ));
                out.push(thermal(
                    id,
                    "bottom",
                    &nodes[1],
                    &nodes[2],
                    resistance * (1.0 - pos),
                    T_NOISE,
                ));
            }
            CircuitElement::Thermistor {
                id,
                nodes,
                temperature,
            } => {
                let params = sindr_devices::thermistor::ThermistorParams::default();
                let r = sindr_devices::thermistor::thermistor_resistance(*temperature, &params);
                out.push(thermal(id, "", &nodes[0], &nodes[1], r, *temperature));
            }
            CircuitElement::Photoresistor {
                id,
                nodes,
                light_level,
            } => {
                let r = crate::stamp::ldr_resistance(*light_level);
                out.push(thermal(id, "", &nodes[0], &nodes[1], r, T_NOISE));
            }
            CircuitElement::Diode {
                id, nodes, model, ..
            } => {
                let i = current(id);
                out.push(shot(id, "", &nodes[0], &nodes[1], i));
                if let Some(m) = model.filter(|m| m.kf > 0.0) {
                    out.push(flicker(id, "", &nodes[0], &nodes[1], i, m.kf, m.af));
                }
            }
            CircuitElement::Led { id, nodes, .. }
            | CircuitElement::SchottkyDiode { id, nodes, .. }
            | CircuitElement::ZenerDiode { id, nodes, .. }
            | CircuitElement::Photodiode { id, nodes, .. } => {
                out.push(shot(id, "", &nodes[0], &nodes[1], current(id)));
            }
            CircuitElement::Bjt {
                id, nodes, model, ..
            } => {
                let Some(bjt) = dc.bjt_results.iter().find(|b| b.id == *id) else {
                    continue;
                };
                let [b, col, e] = nodes;
                out.push(shot(id, "ic", col, e, bjt.ic));
                out.push(shot(id, "ib", b, e, bjt.ib));
                if let Some(m) = model.filter(|m| m.kf > 0.0) {
                    out.push(flicker(id, "ib", b, e, bjt.ib, m.kf, m.af));
                }
            }
            CircuitElement::Mosfet {
                id,
                nodes,
                kind,
                params,
                ..
            } => {
                let [g, d, s] = nodes;
                let (vgs, vds) = kind.nmos_frame(voltage(g), voltage(d), voltage(s));
                let comp = sindr_devices::mosfet::mosfet_companion(
                    vgs,
                    vds,
                    0.0,
                    &params.in_nmos_frame(*kind),
                );
                out.push(channel(
                    source(id, NoiseKind::Thermal, ""),
                    idx(d),
                    idx(s),
                    comp.gm,
                ));
                if params.kf > 0.0 {
                    out.push(flicker(id, "", d, s, comp.id, params.kf, params.af));
                }
            }
            CircuitElement::Jfet {
                id,
                nodes,
                kind,
                idss,
                vp,
            } => {
                let [g, d, s] = nodes;
                let (vg, vd, vs) = (voltage(g), voltage(d), voltage(s));
                let comp = sindr_devices::jfet::jfet_companion(vg - vs, vd - vs, *kind, *idss, *vp);
                out.push(channel(
                    source(id, NoiseKind::Thermal, ""),
                    idx(d),
                    idx(s),
                    comp.gm,
                ));
            }
            _ => {}
        }
    }
    out
}

/// A FET channel's thermal noise, `(8/3)·kT·|gm|`, between drain and source.
fn channel(source: NoiseSource, d: Option<usize>, s: Option<usize>, gm: f64) -> Generator {
    Generator {
        source,
        p: d,
        q: s,
        white: 8.0 / 3.0 * BOLTZMANN * T_NOISE * gm.abs(),
        flicker: 0.0,
    }
}
