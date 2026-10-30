//! AC small-signal analysis module.
//!
//! Performs frequency-domain analysis by:
//! 1. Finding the DC operating point
//! 2. Linearizing all nonlinear elements at that point
//! 3. Building a complex-valued MNA system at each frequency
//! 4. Solving for node voltages as complex phasors

use std::collections::HashMap;

use nalgebra::{DMatrix, DVector};
use num_complex::Complex64;

use sindr_devices::bjt::{self, BjtKind};
use sindr_devices::diode;
use sindr_devices::jfet;
use sindr_devices::mosfet::{self, MosfetKind};

use crate::circuit::{Circuit, CircuitElement};
use crate::device_params;
use crate::error::SimError;
use crate::node_map::NodeMap;

/// Frequency sweep spacing.
#[derive(Debug, Clone, Copy)]
pub enum FrequencySpacing {
    /// Logarithmic spacing (decade-based).
    Logarithmic,
    /// Linear spacing.
    Linear,
}

/// AC analysis configuration.
#[derive(Debug, Clone)]
pub struct AcConfig {
    /// Start frequency (Hz).
    pub f_start: f64,
    /// Stop frequency (Hz).
    pub f_stop: f64,
    /// Number of frequency points, including both endpoints. With 1, only
    /// `f_start` is solved.
    pub num_points: usize,
    /// Frequency spacing.
    pub spacing: FrequencySpacing,
    /// ID of the AC source (voltage source with AC stimulus).
    pub source_id: String,
    /// AC magnitude of the source.
    pub ac_magnitude: f64,
}

/// Result at a single frequency point.
#[derive(Debug, Clone)]
pub struct AcPoint {
    /// Frequency of this point (Hz).
    pub frequency: f64,
    /// Complex node voltages (V) keyed by node name.
    pub node_voltages: HashMap<String, Complex64>,
}

impl AcPoint {
    /// Get gain in dB for a node relative to AC source magnitude.
    pub fn gain_db(&self, node: &str, source_magnitude: f64) -> Option<f64> {
        self.node_voltages.get(node).map(|v| {
            let gain = v.norm() / source_magnitude;
            20.0 * gain.log10()
        })
    }

    /// Get phase in degrees for a node.
    pub fn phase_deg(&self, node: &str) -> Option<f64> {
        self.node_voltages.get(node).map(|v| v.arg().to_degrees())
    }
}

/// Complete AC analysis result.
#[derive(Debug, Clone)]
pub struct AcResult {
    /// Result points, one per swept frequency.
    pub points: Vec<AcPoint>,
    /// Configuration used to produce this result.
    pub config: AcConfig,
}

impl AcResult {
    /// Extract gain (dB) curve for a node.
    pub fn gain_curve(&self, node: &str) -> Vec<(f64, f64)> {
        self.points
            .iter()
            .filter_map(|p| {
                p.gain_db(node, self.config.ac_magnitude)
                    .map(|g| (p.frequency, g))
            })
            .collect()
    }

    /// Extract phase (degrees) curve for a node.
    pub fn phase_curve(&self, node: &str) -> Vec<(f64, f64)> {
        self.points
            .iter()
            .filter_map(|p| p.phase_deg(node).map(|ph| (p.frequency, ph)))
            .collect()
    }
}

/// Complex MNA system for AC analysis.
#[allow(dead_code)]
struct ComplexMnaSystem {
    a: DMatrix<Complex64>,
    b: DVector<Complex64>,
    num_nodes: usize,
    num_vsources: usize,
}

impl ComplexMnaSystem {
    fn new(num_nodes: usize, num_vsources: usize) -> Self {
        let size = num_nodes + num_vsources;
        Self {
            a: DMatrix::from_element(size, size, Complex64::new(0.0, 0.0)),
            b: DVector::from_element(size, Complex64::new(0.0, 0.0)),
            num_nodes,
            num_vsources,
        }
    }

    fn solve(&self) -> Result<DVector<Complex64>, SimError> {
        let lu = self.a.clone().lu();
        let solution = lu.solve(&self.b).ok_or(SimError::SingularMatrix)?;

        for i in 0..solution.len() {
            if solution[i].re.is_nan()
                || solution[i].re.is_infinite()
                || solution[i].im.is_nan()
                || solution[i].im.is_infinite()
            {
                return Err(SimError::InvalidSolution);
            }
        }

        Ok(solution)
    }
}

/// Linearized small-signal model for a nonlinear element.
struct SmallSignalModel {
    /// Conductance stamps: (row_node, col_node, value)
    /// None means ground.
    conductances: Vec<(Option<usize>, Option<usize>, f64)>,
}

/// Extract small-signal models from nonlinear elements at DC operating point.
fn linearize_at_dc(
    circuit: &Circuit,
    node_map: &NodeMap,
    dc_solution: &nalgebra::DVector<f64>,
) -> Vec<SmallSignalModel> {
    let mut models = Vec::new();

    for component in &circuit.components {
        match component {
            CircuitElement::Diode {
                nodes, temperature, ..
            } => {
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                let vp = p.map_or(0.0, |i| dc_solution[i]);
                let vq = q.map_or(0.0, |i| dc_solution[i]);
                let v_d = vp - vq;

                let params = device_params::diode(*temperature);
                let (g_d, _) = diode::diode_companion(v_d, &params);

                models.push(SmallSignalModel {
                    conductances: vec![(p, p, g_d), (q, q, g_d), (p, q, -g_d), (q, p, -g_d)],
                });
            }
            CircuitElement::Led {
                nodes,
                color,
                temperature,
                ..
            } => {
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                let vp = p.map_or(0.0, |i| dc_solution[i]);
                let vq = q.map_or(0.0, |i| dc_solution[i]);
                let v_d = vp - vq;

                let params = device_params::led(color, *temperature);
                let (g_d, _) = diode::diode_companion(v_d, &params);

                models.push(SmallSignalModel {
                    conductances: vec![(p, p, g_d), (q, q, g_d), (p, q, -g_d), (q, p, -g_d)],
                });
            }
            CircuitElement::Bjt {
                nodes,
                kind,
                bf,
                temperature,
                ..
            } => {
                let b = node_map.index(&nodes[0]);
                let c = node_map.index(&nodes[1]);
                let e = node_map.index(&nodes[2]);

                let vb = b.map_or(0.0, |i| dc_solution[i]);
                let vc = c.map_or(0.0, |i| dc_solution[i]);
                let ve = e.map_or(0.0, |i| dc_solution[i]);

                let sign = match kind {
                    BjtKind::Npn => 1.0,
                    BjtKind::Pnp => -1.0,
                };
                let vbe_eff = sign * (vb - ve);
                let vbc_eff = sign * (vb - vc);

                // Same params as the DC solve (stamp.rs), so the linearisation
                // is taken of the model the operating point was solved with.
                let params = device_params::bjt(*bf, *temperature);

                // Full terminal Jacobian ∂(Ib, Ic, Ie)/∂(Vb, Vc, Ve): carries
                // gπ, gm, gμ and g_o in every region. The entries are the
                // same for NPN and PNP (the polarity sign cancels).
                let jac = bjt::bjt_terminal_jacobian(vbe_eff, vbc_eff, &params);
                let m = jac.terminal_matrix();
                let terminals = [b, c, e];
                let mut conds = Vec::with_capacity(9);
                for (r, &row) in terminals.iter().enumerate() {
                    for (k, &col) in terminals.iter().enumerate() {
                        conds.push((row, col, m[r][k]));
                    }
                }

                models.push(SmallSignalModel {
                    conductances: conds,
                });
            }
            CircuitElement::ZenerDiode {
                nodes,
                vz,
                temperature,
                ..
            } => {
                // Same parameters as the DC stamp, including the IS scaling.
                let params = device_params::zener(*vz, *temperature);
                let v_d = junction_voltage(nodes, node_map, dc_solution);
                let (g_d, _) = sindr_devices::zener::zener_companion(v_d, &params);
                models.push(two_terminal(nodes, node_map, g_d));
            }
            CircuitElement::SchottkyDiode {
                nodes, temperature, ..
            } => {
                let params = device_params::schottky(*temperature);
                let v_d = junction_voltage(nodes, node_map, dc_solution);
                let (g_d, _) = diode::diode_companion(v_d, &params);
                models.push(two_terminal(nodes, node_map, g_d));
            }
            CircuitElement::Photodiode {
                nodes,
                irradiance,
                temperature,
                ..
            } => {
                // The photocurrent is a constant source: no AC part.
                let params = device_params::photodiode(*temperature);
                let v_d = junction_voltage(nodes, node_map, dc_solution);
                let (g_d, _) =
                    sindr_devices::photodiode::photodiode_companion(v_d, *irradiance, &params);
                models.push(two_terminal(nodes, node_map, g_d));
            }
            CircuitElement::Jfet {
                nodes,
                kind,
                idss,
                vp,
                ..
            } => {
                let g = node_map.index(&nodes[0]);
                let d = node_map.index(&nodes[1]);
                let s = node_map.index(&nodes[2]);
                let v = |n: Option<usize>| n.map_or(0.0, |i| dc_solution[i]);
                // gm and gds are the same in the physical frame for both
                // channel types (see stamp_jfet_companion).
                let c = jfet::jfet_companion(v(g) - v(s), v(d) - v(s), *kind, *idss, *vp);
                models.push(SmallSignalModel {
                    conductances: vec![
                        (d, g, c.gm),
                        (d, s, -c.gm),
                        (s, g, -c.gm),
                        (s, s, c.gm),
                        (d, d, c.gds),
                        (s, s, c.gds),
                        (d, s, -c.gds),
                        (s, d, -c.gds),
                    ],
                });
            }
            CircuitElement::Igbt { nodes, params, .. } => {
                let g = node_map.index(&nodes[0]);
                let c = node_map.index(&nodes[1]);
                let e = node_map.index(&nodes[2]);
                let v = |n: Option<usize>| n.map_or(0.0, |i| dc_solution[i]);
                // The same companion as the DC stamp (stamp_igbt_companion).
                let comp = sindr_devices::igbt::igbt_companion(v(g) - v(e), v(c) - v(e), params);
                models.push(SmallSignalModel {
                    conductances: vec![
                        (c, g, comp.gm),
                        (c, e, -comp.gm),
                        (e, g, -comp.gm),
                        (e, e, comp.gm),
                        (c, c, comp.g_ce),
                        (e, e, comp.g_ce),
                        (c, e, -comp.g_ce),
                        (e, c, -comp.g_ce),
                    ],
                });
            }
            CircuitElement::Mosfet {
                nodes,
                kind,
                params,
                ..
            } => {
                let g = node_map.index(&nodes[0]);
                let d = node_map.index(&nodes[1]);
                let s = node_map.index(&nodes[2]);

                let vg = g.map_or(0.0, |i| dc_solution[i]);
                let vd = d.map_or(0.0, |i| dc_solution[i]);
                let vs = s.map_or(0.0, |i| dc_solution[i]);

                let (vgs, vds, vbs) = match kind {
                    MosfetKind::Nmos => (vg - vs, vd - vs, -vs), // body tied to ground for now
                    MosfetKind::Pmos => (vs - vg, vs - vd, vs),
                };

                let comp = mosfet::mosfet_companion(vgs, vds, vbs, &params.in_nmos_frame(*kind));

                let mut conds = vec![
                    // gm: Id depends on Vgs => dId/dVg = gm, dId/dVs += -gm
                    (d, g, comp.gm),
                    (d, s, -comp.gm),
                    (s, g, -comp.gm),
                    (s, s, comp.gm),
                    // gds: output conductance between drain and source
                    (d, d, comp.gds),
                    (s, s, comp.gds),
                    (d, s, -comp.gds),
                    (s, d, -comp.gds),
                ];

                // gmb: body transconductance (similar to gm but from Vbs)
                if comp.gmb.abs() > 1e-15 {
                    // dId/dVb = gmb, but body is tied to source/ground for simplicity
                    // In a 3-terminal model (no explicit body), gmb adds to gds-like term
                    conds.push((d, s, -comp.gmb));
                    conds.push((d, d, 0.0)); // body at ground
                    conds.push((s, s, comp.gmb));
                }

                models.push(SmallSignalModel {
                    conductances: conds,
                });
            }
            _ => {}
        }
    }

    models
}

/// Voltage across a two-terminal element's `nodes[0]` → `nodes[1]` at the
/// DC operating point.
fn junction_voltage(nodes: &[String; 2], node_map: &NodeMap, dc_solution: &DVector<f64>) -> f64 {
    let v = |n: &String| node_map.index(n).map_or(0.0, |i| dc_solution[i]);
    v(&nodes[0]) - v(&nodes[1])
}

/// A conductance `g` between a two-terminal element's nodes.
fn two_terminal(nodes: &[String; 2], node_map: &NodeMap, g: f64) -> SmallSignalModel {
    let p = node_map.index(&nodes[0]);
    let q = node_map.index(&nodes[1]);
    SmallSignalModel {
        conductances: vec![(p, p, g), (q, q, g), (p, q, -g), (q, p, -g)],
    }
}

/// Reject sweep configs that would underflow, divide by zero, or feed
/// `log10` a non-positive value.
fn validate_config(config: &AcConfig) -> Result<(), SimError> {
    let invalid = |msg: String| Err(SimError::InvalidComponent(format!("AC sweep: {msg}")));

    if config.num_points == 0 {
        return invalid("num_points must be at least 1".into());
    }
    for (name, value) in [
        ("f_start", config.f_start),
        ("f_stop", config.f_stop),
        ("ac_magnitude", config.ac_magnitude),
    ] {
        if !value.is_finite() {
            return invalid(format!("{name} must be finite (got {value})"));
        }
    }
    match config.spacing {
        FrequencySpacing::Logarithmic if config.f_start <= 0.0 => {
            return invalid(format!(
                "f_start must be > 0 Hz for logarithmic spacing (got {})",
                config.f_start
            ));
        }
        FrequencySpacing::Linear if config.f_start < 0.0 => {
            return invalid(format!("f_start must be >= 0 Hz (got {})", config.f_start));
        }
        _ => {}
    }
    if config.f_start > config.f_stop {
        return invalid(format!(
            "f_start ({}) must not exceed f_stop ({})",
            config.f_start, config.f_stop
        ));
    }
    Ok(())
}

/// Generate frequency points with the specified spacing.
///
/// Expects a config that passed [`validate_config`]. A single point sweeps
/// only `f_start`, as SPICE does for `.ac lin 1`.
fn generate_frequencies(config: &AcConfig) -> Vec<f64> {
    if config.num_points == 1 {
        return vec![config.f_start];
    }
    let mut freqs = Vec::with_capacity(config.num_points);
    match config.spacing {
        FrequencySpacing::Logarithmic => {
            let log_start = config.f_start.log10();
            let log_stop = config.f_stop.log10();
            let step = (log_stop - log_start) / (config.num_points - 1) as f64;
            for i in 0..config.num_points {
                freqs.push(10.0_f64.powf(log_start + step * i as f64));
            }
        }
        FrequencySpacing::Linear => {
            let step = (config.f_stop - config.f_start) / (config.num_points - 1) as f64;
            for i in 0..config.num_points {
                freqs.push(config.f_start + step * i as f64);
            }
        }
    }
    freqs
}

/// Stamp a complex conductance into the complex MNA matrix.
fn stamp_complex_conductance(
    system: &mut ComplexMnaSystem,
    row: Option<usize>,
    col: Option<usize>,
    val: Complex64,
) {
    if let (Some(r), Some(c)) = (row, col) {
        system.a[(r, c)] += val;
    }
}

/// Performs AC small-signal frequency-domain analysis.
///
/// Returns the complex node voltages at every frequency in the configured
/// sweep, plus helpers on [`AcResult`] for extracting gain (dB) and phase
/// (degrees) curves — i.e. Bode plot data.
///
/// # Algorithm
///
/// 1. Solve the DC operating point of `circuit`.
/// 2. Linearise every nonlinear element at that operating point.
/// 3. At each frequency, build a complex-valued MNA system (capacitors and
///    inductors become `jωC` / `jωL` admittances), solve it, and store the
///    complex node voltage phasors.
///
/// # Errors
///
/// - [`SimError::InvalidComponent`] if `config` is invalid: zero points,
///   non-finite values, `f_start > f_stop`, `f_start <= 0` with logarithmic
///   spacing, or `f_start < 0` with linear spacing
/// - Any [`SimError`] from the DC operating-point solve
/// - [`SimError::SingularMatrix`] if the linearised AC system is singular
///   at any frequency
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement};
/// use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
///
/// // Simple RC low-pass: V1 -> R1 -> n_out -> C1 -> gnd
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource {
///             id: "V1".into(),
///             nodes: ["n_in".into(), "0".into()],
///             voltage: 0.0,
///             waveform: None,
///         },
///         CircuitElement::Resistor {
///             id: "R1".into(),
///             nodes: ["n_in".into(), "n_out".into()],
///             resistance: 1_000.0,
///         },
///         CircuitElement::Capacitor {
///             id: "C1".into(),
///             nodes: ["n_out".into(), "0".into()],
///             capacitance: 1e-6,
///         },
///     ],
/// };
///
/// let config = AcConfig {
///     f_start: 1.0,
///     f_stop: 100_000.0,
///     num_points: 11,
///     spacing: FrequencySpacing::Logarithmic,
///     source_id: "V1".into(),
///     ac_magnitude: 1.0,
/// };
///
/// let result = solve_ac(&circuit, &config).unwrap();
/// assert_eq!(result.points.len(), 11);
///
/// // Bode plot data for the output node:
/// let gain = result.gain_curve("n_out");
/// let phase = result.phase_curve("n_out");
/// ```
pub fn solve_ac(circuit: &Circuit, config: &AcConfig) -> Result<AcResult, SimError> {
    validate_config(config)?;

    // Step 1: DC operating point
    let dc_result = crate::solve_dc_operating_point(circuit)?;

    let node_map = NodeMap::from_circuit(circuit);
    let num_nodes = node_map.num_nodes();
    let num_vsources = circuit.count_voltage_sources();

    // Reconstruct DC solution vector from node voltages
    let mut dc_solution = nalgebra::DVector::zeros(num_nodes + num_vsources);
    for i in 0..num_nodes {
        if let Some(name) = node_map.node_name(i) {
            if let Some(&v) = dc_result.node_voltages.get(name) {
                dc_solution[i] = v;
            }
        }
    }

    // Step 2: Linearize nonlinear elements
    let ss_models = linearize_at_dc(circuit, &node_map, &dc_solution);

    // Step 3: Frequency sweep
    let frequencies = generate_frequencies(config);
    let mut points = Vec::with_capacity(frequencies.len());

    for &freq in &frequencies {
        let omega = 2.0 * std::f64::consts::PI * freq;
        let j_omega = Complex64::new(0.0, omega);

        let mut system = ComplexMnaSystem::new(num_nodes, num_vsources);
        let mut vsource_index: usize = 0;

        // Stamp linear components
        for component in &circuit.components {
            match component {
                CircuitElement::Resistor {
                    nodes, resistance, ..
                } => {
                    let g = Complex64::new(1.0 / resistance, 0.0);
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, g);
                    stamp_complex_conductance(&mut system, q, q, g);
                    stamp_complex_conductance(&mut system, p, q, -g);
                    stamp_complex_conductance(&mut system, q, p, -g);
                }
                CircuitElement::Capacitor {
                    nodes, capacitance, ..
                } => {
                    let y = j_omega * *capacitance; // Y = j*omega*C
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, y);
                    stamp_complex_conductance(&mut system, q, q, y);
                    stamp_complex_conductance(&mut system, p, q, -y);
                    stamp_complex_conductance(&mut system, q, p, -y);
                }
                CircuitElement::Varactor { nodes, params, .. } => {
                    // Small-signal capacitance at the DC bias.
                    let v_d = junction_voltage(nodes, &node_map, &dc_solution);
                    let c = sindr_devices::varactor::junction_capacitance(v_d, params);
                    let y = j_omega * c;
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, y);
                    stamp_complex_conductance(&mut system, q, q, y);
                    stamp_complex_conductance(&mut system, p, q, -y);
                    stamp_complex_conductance(&mut system, q, p, -y);
                }
                CircuitElement::Inductor {
                    nodes, inductance, ..
                } => {
                    // Y = 1/(j*omega*L)
                    let y = if omega.abs() > 1e-15 {
                        Complex64::new(1.0, 0.0) / (j_omega * *inductance)
                    } else {
                        // At DC, inductor is a short circuit (very high conductance)
                        Complex64::new(1e12, 0.0)
                    };
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, y);
                    stamp_complex_conductance(&mut system, q, q, y);
                    stamp_complex_conductance(&mut system, p, q, -y);
                    stamp_complex_conductance(&mut system, q, p, -y);
                }
                CircuitElement::VoltageSource { id, nodes, .. } => {
                    let branch = num_nodes + vsource_index;
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    let one = Complex64::new(1.0, 0.0);

                    if let Some(pi) = p {
                        system.a[(pi, branch)] += one;
                        system.a[(branch, pi)] += one;
                    }
                    if let Some(qi) = q {
                        system.a[(qi, branch)] -= one;
                        system.a[(branch, qi)] -= one;
                    }

                    // AC source excitation
                    if id == &config.source_id {
                        system.b[branch] = Complex64::new(config.ac_magnitude, 0.0);
                    }
                    // Other voltage sources: V_ac = 0 (short circuit for AC)

                    vsource_index += 1;
                }
                CircuitElement::CurrentSource { nodes, .. } => {
                    // AC current source: zero AC (unless specified)
                    // For now, independent current sources contribute 0 to AC
                    let _ = nodes;
                }
                CircuitElement::Switch { nodes, closed, .. } => {
                    let r = if *closed { 0.01 } else { 1e9 };
                    let g = Complex64::new(1.0 / r, 0.0);
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, g);
                    stamp_complex_conductance(&mut system, q, q, g);
                    stamp_complex_conductance(&mut system, p, q, -g);
                    stamp_complex_conductance(&mut system, q, p, -g);
                }
                CircuitElement::Pushbutton { nodes, closed, .. } => {
                    let r = if *closed { 0.01 } else { 1e9 };
                    let g = Complex64::new(1.0 / r, 0.0);
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, g);
                    stamp_complex_conductance(&mut system, q, q, g);
                    stamp_complex_conductance(&mut system, p, q, -g);
                    stamp_complex_conductance(&mut system, q, p, -g);
                }
                CircuitElement::Photoresistor {
                    nodes, light_level, ..
                } => {
                    use crate::stamp::ldr_resistance;
                    let r = ldr_resistance(*light_level);
                    let g = Complex64::new(1.0 / r, 0.0);
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, g);
                    stamp_complex_conductance(&mut system, q, q, g);
                    stamp_complex_conductance(&mut system, p, q, -g);
                    stamp_complex_conductance(&mut system, q, p, -g);
                }
                CircuitElement::Potentiometer {
                    nodes,
                    resistance,
                    position,
                    ..
                } => {
                    let pos = position.clamp(0.001, 0.999);
                    let r_top = resistance * pos;
                    let r_bot = resistance * (1.0 - pos);
                    let g_top = Complex64::new(1.0 / r_top, 0.0);
                    let g_bot = Complex64::new(1.0 / r_bot, 0.0);
                    let p = node_map.index(&nodes[0]);
                    let w = node_map.index(&nodes[1]);
                    let q = node_map.index(&nodes[2]);
                    stamp_complex_conductance(&mut system, p, p, g_top);
                    stamp_complex_conductance(&mut system, w, w, g_top);
                    stamp_complex_conductance(&mut system, p, w, -g_top);
                    stamp_complex_conductance(&mut system, w, p, -g_top);
                    stamp_complex_conductance(&mut system, w, w, g_bot);
                    stamp_complex_conductance(&mut system, q, q, g_bot);
                    stamp_complex_conductance(&mut system, w, q, -g_bot);
                    stamp_complex_conductance(&mut system, q, w, -g_bot);
                }
                CircuitElement::Relay {
                    nodes,
                    coil_resistance,
                    ..
                } => {
                    let g = Complex64::new(1.0 / coil_resistance, 0.0);
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, g);
                    stamp_complex_conductance(&mut system, q, q, g);
                    stamp_complex_conductance(&mut system, p, q, -g);
                    stamp_complex_conductance(&mut system, q, p, -g);
                    // Contact: treat as open for AC analysis
                    let g_open = Complex64::new(1.0 / 1e9, 0.0);
                    let c1 = node_map.index(&nodes[2]);
                    let c2 = node_map.index(&nodes[3]);
                    stamp_complex_conductance(&mut system, c1, c1, g_open);
                    stamp_complex_conductance(&mut system, c2, c2, g_open);
                    stamp_complex_conductance(&mut system, c1, c2, -g_open);
                    stamp_complex_conductance(&mut system, c2, c1, -g_open);
                }
                // Thermistor: passive resistor with temperature-dependent resistance
                CircuitElement::Thermistor {
                    nodes, temperature, ..
                } => {
                    let params = sindr_devices::thermistor::ThermistorParams::default();
                    let r = sindr_devices::thermistor::thermistor_resistance(*temperature, &params);
                    let g = Complex64::new(1.0 / r, 0.0);
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, g);
                    stamp_complex_conductance(&mut system, q, q, g);
                    stamp_complex_conductance(&mut system, p, q, -g);
                    stamp_complex_conductance(&mut system, q, p, -g);
                }
                // Nonlinear elements: handled via small-signal models below
                CircuitElement::Diode { .. }
                | CircuitElement::Led { .. }
                | CircuitElement::Bjt { .. }
                | CircuitElement::Mosfet { .. }
                | CircuitElement::ZenerDiode { .. }
                | CircuitElement::SchottkyDiode { .. }
                | CircuitElement::Photodiode { .. }
                | CircuitElement::Jfet { .. }
                | CircuitElement::Igbt { .. } => {}
                // Transformer: coupled inductors — stamp admittances in AC (Y = 1/(j*omega*L))
                // For simplicity, stamp both windings as independent inductors (ignores mutual coupling in AC)
                // A full coupled-inductor AC stamp would require expanding the MNA with 2 branch vars here too.
                CircuitElement::Transformer { nodes, l1, l2, .. } => {
                    let y1 = if omega.abs() > 1e-15 {
                        Complex64::new(1.0, 0.0) / (j_omega * *l1)
                    } else {
                        Complex64::new(1e12, 0.0)
                    };
                    let p1 = node_map.index(&nodes[0]);
                    let q1 = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p1, p1, y1);
                    stamp_complex_conductance(&mut system, q1, q1, y1);
                    stamp_complex_conductance(&mut system, p1, q1, -y1);
                    stamp_complex_conductance(&mut system, q1, p1, -y1);

                    let y2 = if omega.abs() > 1e-15 {
                        Complex64::new(1.0, 0.0) / (j_omega * *l2)
                    } else {
                        Complex64::new(1e12, 0.0)
                    };
                    let p2 = node_map.index(&nodes[2]);
                    let q2 = node_map.index(&nodes[3]);
                    stamp_complex_conductance(&mut system, p2, p2, y2);
                    stamp_complex_conductance(&mut system, q2, q2, y2);
                    stamp_complex_conductance(&mut system, p2, q2, -y2);
                    stamp_complex_conductance(&mut system, q2, p2, -y2);
                }
                // OpAmp/Comparator: high-gain VCVS (gain=1e5) — linear, stamp properly in AC
                CircuitElement::OpAmp { nodes, .. } | CircuitElement::Comparator { nodes, .. } => {
                    let branch = num_nodes + vsource_index;
                    let p = node_map.index(&nodes[2]); // out
                    let gnd = node_map.index(&circuit.ground_node);
                    let cp = node_map.index(&nodes[0]); // in+
                    let cq = node_map.index(&nodes[1]); // in-
                    let one = Complex64::new(1.0, 0.0);
                    let gain = Complex64::new(1e5, 0.0);

                    if let Some(pi) = p {
                        system.a[(pi, branch)] += one;
                    }
                    if let Some(qi) = gnd {
                        system.a[(qi, branch)] -= one;
                    }
                    if let Some(pi) = p {
                        system.a[(branch, pi)] += one;
                    }
                    if let Some(qi) = gnd {
                        system.a[(branch, qi)] -= one;
                    }
                    if let Some(cpi) = cp {
                        system.a[(branch, cpi)] -= gain;
                    }
                    if let Some(cqi) = cq {
                        system.a[(branch, cqi)] += gain;
                    }
                    vsource_index += 1;
                }

                // Dependent sources in AC
                CircuitElement::Vcvs {
                    nodes,
                    control_nodes,
                    gain,
                    ..
                } => {
                    let branch = num_nodes + vsource_index;
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    let cp = node_map.index(&control_nodes[0]);
                    let cq = node_map.index(&control_nodes[1]);
                    let one = Complex64::new(1.0, 0.0);
                    let g = Complex64::new(*gain, 0.0);

                    if let Some(pi) = p {
                        system.a[(pi, branch)] += one;
                    }
                    if let Some(qi) = q {
                        system.a[(qi, branch)] -= one;
                    }
                    if let Some(pi) = p {
                        system.a[(branch, pi)] += one;
                    }
                    if let Some(qi) = q {
                        system.a[(branch, qi)] -= one;
                    }
                    if let Some(cpi) = cp {
                        system.a[(branch, cpi)] -= g;
                    }
                    if let Some(cqi) = cq {
                        system.a[(branch, cqi)] += g;
                    }
                    vsource_index += 1;
                }
                CircuitElement::Vccs {
                    nodes,
                    control_nodes,
                    gm,
                    ..
                } => {
                    let from = node_map.index(&nodes[0]);
                    let to = node_map.index(&nodes[1]);
                    let cp = node_map.index(&control_nodes[0]);
                    let cq = node_map.index(&control_nodes[1]);
                    let g = Complex64::new(*gm, 0.0);

                    if let (Some(ti), Some(cpi)) = (to, cp) {
                        system.a[(ti, cpi)] += g;
                    }
                    if let (Some(ti), Some(cqi)) = (to, cq) {
                        system.a[(ti, cqi)] -= g;
                    }
                    if let (Some(fi), Some(cpi)) = (from, cp) {
                        system.a[(fi, cpi)] -= g;
                    }
                    if let (Some(fi), Some(cqi)) = (from, cq) {
                        system.a[(fi, cqi)] += g;
                    }
                }
                CircuitElement::Ccvs {
                    nodes,
                    control_source,
                    rm,
                    ..
                } => {
                    let branch = num_nodes + vsource_index;
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    let one = Complex64::new(1.0, 0.0);
                    let r = Complex64::new(*rm, 0.0);

                    if let Some(pi) = p {
                        system.a[(pi, branch)] += one;
                    }
                    if let Some(qi) = q {
                        system.a[(qi, branch)] -= one;
                    }
                    if let Some(pi) = p {
                        system.a[(branch, pi)] += one;
                    }
                    if let Some(qi) = q {
                        system.a[(branch, qi)] -= one;
                    }
                    if let Some(cb) = circuit.vsource_branch_index(control_source) {
                        system.a[(branch, num_nodes + cb)] -= r;
                    }
                    vsource_index += 1;
                }
                CircuitElement::Cccs {
                    nodes,
                    control_source,
                    alpha,
                    ..
                } => {
                    let from = node_map.index(&nodes[0]);
                    let to = node_map.index(&nodes[1]);
                    let a = Complex64::new(*alpha, 0.0);

                    if let Some(cb) = circuit.vsource_branch_index(control_source) {
                        if let Some(ti) = to {
                            system.a[(ti, num_nodes + cb)] += a;
                        }
                        if let Some(fi) = from {
                            system.a[(fi, num_nodes + cb)] -= a;
                        }
                    }
                }
                // Fuse: stamp as resistor (intact=0.001 Ohm, blown=1e9 Ohm) in AC
                CircuitElement::Fuse { nodes, blown, .. } => {
                    let r = if *blown { 1e9_f64 } else { 0.001_f64 };
                    let g = Complex64::new(1.0 / r, 0.0);
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut system, p, p, g);
                    stamp_complex_conductance(&mut system, q, q, g);
                    stamp_complex_conductance(&mut system, p, q, -g);
                    stamp_complex_conductance(&mut system, q, p, -g);
                }
                // VoltageRegulator: ideal DC reference — AC small-signal = voltage source (zero AC)
                // Stamped as ideal voltage source between nodes[1] (output) and nodes[2] (gnd).
                CircuitElement::VoltageRegulator { nodes, .. } => {
                    let branch = num_nodes + vsource_index;
                    let p = node_map.index(&nodes[1]); // output
                    let q = node_map.index(&nodes[2]); // gnd
                    let one = Complex64::new(1.0, 0.0);
                    if let Some(pi) = p {
                        system.a[(pi, branch)] += one;
                        system.a[(branch, pi)] += one;
                    }
                    if let Some(qi) = q {
                        system.a[(qi, branch)] -= one;
                        system.a[(branch, qi)] -= one;
                    }
                    // Zero AC voltage (DC reference, not a signal source)
                    vsource_index += 1;
                }
            }
        }

        // Stamp small-signal models
        for model in &ss_models {
            for &(row, col, val) in &model.conductances {
                stamp_complex_conductance(&mut system, row, col, Complex64::new(val, 0.0));
            }
        }

        // Solve
        let solution = system.solve()?;

        // Extract node voltages
        let mut node_voltages = HashMap::new();
        for i in 0..num_nodes {
            if let Some(name) = node_map.node_name(i) {
                node_voltages.insert(name.to_string(), solution[i]);
            }
        }

        points.push(AcPoint {
            frequency: freq,
            node_voltages,
        });
    }

    Ok(AcResult {
        points,
        config: config.clone(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    /// RC lowpass: R=1k, C=1uF. Corner freq = 1/(2*pi*R*C) ~ 159.15 Hz.
    /// At corner: gain = -3dB, phase = -45 degrees.
    #[test]
    fn rc_lowpass_corner_frequency() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                },
            ],
        };

        let config = AcConfig {
            f_start: 1.0,
            f_stop: 100_000.0,
            num_points: 100,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
        };

        let result = solve_ac(&circuit, &config).unwrap();
        let gain_curve = result.gain_curve("n2");
        let phase_curve = result.phase_curve("n2");

        assert!(!gain_curve.is_empty());
        assert!(!phase_curve.is_empty());

        // Find point closest to corner frequency (159.15 Hz)
        let f_corner = 1.0 / (2.0 * std::f64::consts::PI * 1000.0 * 1e-6);
        let corner_point = gain_curve
            .iter()
            .min_by(|a, b| {
                (a.0 - f_corner)
                    .abs()
                    .partial_cmp(&(b.0 - f_corner).abs())
                    .unwrap()
            })
            .unwrap();

        // At corner: gain should be ~-3dB (within 1dB tolerance due to discrete freq points)
        assert_abs_diff_eq!(corner_point.1, -3.0, epsilon = 1.0);

        // Phase at corner should be ~-45 degrees
        let corner_phase = phase_curve
            .iter()
            .min_by(|a, b| {
                (a.0 - f_corner)
                    .abs()
                    .partial_cmp(&(b.0 - f_corner).abs())
                    .unwrap()
            })
            .unwrap();
        assert_abs_diff_eq!(corner_phase.1, -45.0, epsilon = 5.0);
    }

    /// At very low frequency, RC lowpass should pass signal through (~0dB).
    #[test]
    fn rc_lowpass_passband() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                },
            ],
        };

        let config = AcConfig {
            f_start: 1.0,
            f_stop: 10.0,
            num_points: 5,
            spacing: FrequencySpacing::Linear,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
        };

        let result = solve_ac(&circuit, &config).unwrap();
        let gain_curve = result.gain_curve("n2");

        // At 1 Hz (well below 159 Hz corner), gain should be ~0 dB
        assert_abs_diff_eq!(gain_curve[0].1, 0.0, epsilon = 0.1);
    }

    /// At high frequency, RC lowpass should attenuate at -20dB/decade.
    #[test]
    fn rc_lowpass_stopband_rolloff() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                },
                CircuitElement::Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                },
            ],
        };

        let config = AcConfig {
            f_start: 10_000.0,
            f_stop: 100_000.0,
            num_points: 3,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
        };

        let result = solve_ac(&circuit, &config).unwrap();
        let gain_curve = result.gain_curve("n2");

        // Between 10kHz and 100kHz (one decade), gain should drop ~20dB
        let gain_10k = gain_curve.first().unwrap().1;
        let gain_100k = gain_curve.last().unwrap().1;
        let rolloff = gain_10k - gain_100k;
        assert_abs_diff_eq!(rolloff, 20.0, epsilon = 2.0);
    }

    #[test]
    fn frequency_generation_logarithmic() {
        let config = AcConfig {
            f_start: 1.0,
            f_stop: 1000.0,
            num_points: 4,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
        };
        let freqs = generate_frequencies(&config);
        assert_eq!(freqs.len(), 4);
        assert_abs_diff_eq!(freqs[0], 1.0, epsilon = 1e-10);
        assert_abs_diff_eq!(freqs[1], 10.0, epsilon = 1e-10);
        assert_abs_diff_eq!(freqs[2], 100.0, epsilon = 1e-8);
        assert_abs_diff_eq!(freqs[3], 1000.0, epsilon = 1e-6);
    }

    #[test]
    fn frequency_generation_linear() {
        let config = AcConfig {
            f_start: 100.0,
            f_stop: 400.0,
            num_points: 4,
            spacing: FrequencySpacing::Linear,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
        };
        let freqs = generate_frequencies(&config);
        assert_eq!(freqs.len(), 4);
        assert_abs_diff_eq!(freqs[0], 100.0, epsilon = 1e-10);
        assert_abs_diff_eq!(freqs[1], 200.0, epsilon = 1e-10);
        assert_abs_diff_eq!(freqs[2], 300.0, epsilon = 1e-10);
        assert_abs_diff_eq!(freqs[3], 400.0, epsilon = 1e-10);
    }

    /// One-transistor stage with base, collector and emitter resistors, so
    /// gπ, gμ, gm and g_o all show in the gain. NPN: emitter side at ground,
    /// collector to VCC. PNP: emitter side at VCC, collector to ground.
    fn bjt_stage(kind: BjtKind, vin: f64, temperature: f64) -> Circuit {
        let (vcc, rc_to, re_to) = match kind {
            BjtKind::Npn => (5.0, "vcc", "0"),
            BjtKind::Pnp => (5.0, "0", "vcc"),
        };
        Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "VIN".into(),
                    nodes: ["in".into(), "0".into()],
                    voltage: vin,
                    waveform: None,
                },
                CircuitElement::VoltageSource {
                    id: "VCC".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: vcc,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "RB".into(),
                    nodes: ["in".into(), "b".into()],
                    resistance: 10_000.0,
                },
                CircuitElement::Resistor {
                    id: "RC".into(),
                    nodes: ["c".into(), rc_to.into()],
                    resistance: 1_000.0,
                },
                CircuitElement::Resistor {
                    id: "RE".into(),
                    nodes: ["e".into(), re_to.into()],
                    resistance: 100.0,
                },
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "e".into()],
                    kind,
                    bf: 100.0,
                    temperature,
                    parasitic_caps: None,
                },
            ],
        }
    }

    /// The AC gain vin → vc at low frequency must equal the slope of the DC
    /// transfer curve at the operating point: AC linearises the same model
    /// the DC solve uses.
    fn assert_ac_gain_matches_dc_slope(kind: BjtKind, vin: f64, temperature: f64) {
        let vc_dc = |v: f64| {
            let r = crate::solve_circuit(&bjt_stage(kind, v, temperature)).unwrap();
            r.node_voltages["c"]
        };
        let h = 1e-3;
        let slope = (vc_dc(vin + h) - vc_dc(vin - h)) / (2.0 * h);

        let config = AcConfig {
            f_start: 1.0,
            f_stop: 1.0,
            num_points: 1,
            spacing: FrequencySpacing::Linear,
            source_id: "VIN".into(),
            ac_magnitude: 1.0,
        };
        let result = solve_ac(&bjt_stage(kind, vin, temperature), &config).unwrap();
        let gain = result.points[0].node_voltages["c"];

        assert!(
            gain.im.abs() < 1e-9,
            "no reactive parts, gain should be real: {gain}"
        );
        let tol = 1e-3 * slope.abs() + 1e-5;
        assert!(
            (gain.re - slope).abs() <= tol,
            "{kind:?} vin={vin} T={temperature}: AC gain {} vs DC slope {slope}",
            gain.re
        );
    }

    #[test]
    fn bjt_ac_gain_matches_dc_slope_npn_forward_active() {
        assert_ac_gain_matches_dc_slope(BjtKind::Npn, 1.0, 300.15);
    }

    #[test]
    fn bjt_ac_gain_matches_dc_slope_npn_saturation() {
        assert_ac_gain_matches_dc_slope(BjtKind::Npn, 5.0, 300.15);
    }

    #[test]
    fn bjt_ac_gain_matches_dc_slope_pnp_forward_active() {
        assert_ac_gain_matches_dc_slope(BjtKind::Pnp, 4.0, 300.15);
    }

    #[test]
    fn bjt_ac_gain_matches_dc_slope_pnp_saturation() {
        assert_ac_gain_matches_dc_slope(BjtKind::Pnp, 0.0, 300.15);
    }

    /// Off nominal temperature the DC solve scales IS; AC must linearise
    /// with the same IS or the gain drifts from the DC slope.
    #[test]
    fn bjt_ac_gain_matches_dc_slope_at_raised_temperature() {
        assert_ac_gain_matches_dc_slope(BjtKind::Npn, 1.0, 350.0);
    }

    /// Low-frequency AC gain VIN → `node` must equal the slope of the DC
    /// transfer curve at the bias `vin`, for any purely resistive circuit
    /// `build(vin)` with a source `VIN`.
    fn assert_ac_matches_dc_slope(
        label: &str,
        build: impl Fn(f64) -> Circuit,
        vin: f64,
        node: &str,
    ) {
        let v_dc = |v: f64| crate::solve_circuit(&build(v)).unwrap().node_voltages[node];
        let h = 1e-3;
        let slope = (v_dc(vin + h) - v_dc(vin - h)) / (2.0 * h);
        let config = AcConfig {
            f_start: 1.0,
            f_stop: 1.0,
            num_points: 1,
            spacing: FrequencySpacing::Linear,
            source_id: "VIN".into(),
            ac_magnitude: 1.0,
        };
        let gain = solve_ac(&build(vin), &config).unwrap().points[0].node_voltages[node];
        let tol = 1e-3 * slope.abs() + 1e-5;
        assert!(
            (gain.re - slope).abs() <= tol && gain.im.abs() < 1e-9,
            "{label}: AC gain {gain} vs DC slope {slope}"
        );
    }

    fn vsrc(id: &str, p: &str, v: f64) -> CircuitElement {
        CircuitElement::VoltageSource {
            id: id.into(),
            nodes: [p.into(), "0".into()],
            voltage: v,
            waveform: None,
        }
    }

    fn res(id: &str, a: &str, b: &str, r: f64) -> CircuitElement {
        CircuitElement::Resistor {
            id: id.into(),
            nodes: [a.into(), b.into()],
            resistance: r,
        }
    }

    fn with(components: Vec<CircuitElement>) -> Circuit {
        Circuit {
            ground_node: "0".into(),
            components,
        }
    }

    #[test]
    fn jfet_ac_gain_matches_dc_slope() {
        use sindr_devices::jfet::JfetKind;
        // Common-source stage driven at the gate; P-channel is the mirror.
        for (kind, sign, vp) in [
            (JfetKind::NChannel, 1.0, -2.0),
            (JfetKind::PChannel, -1.0, 2.0),
        ] {
            let stage = |vin: f64| {
                with(vec![
                    vsrc("VDD", "vdd", sign * 12.0),
                    vsrc("VIN", "g", vin),
                    res("RD", "vdd", "d", 1_000.0),
                    res("RS", "s", "0", 100.0),
                    CircuitElement::Jfet {
                        id: "J1".into(),
                        nodes: ["g".into(), "d".into(), "s".into()],
                        kind,
                        idss: 10e-3,
                        vp,
                    },
                ])
            };
            assert_ac_matches_dc_slope(&format!("{kind:?}"), stage, sign * -0.8, "d");
        }
    }

    #[test]
    fn diode_family_ac_matches_dc_slope() {
        let behind = |element: CircuitElement, r: f64| {
            move |vin: f64| {
                with(vec![
                    vsrc("VIN", "in", vin),
                    res("R1", "in", "out", r),
                    element.clone(),
                ])
            }
        };
        let zener = CircuitElement::ZenerDiode {
            id: "D1".into(),
            nodes: ["0".into(), "out".into()],
            vz: 5.1,
            temperature: 300.15,
        };
        assert_ac_matches_dc_slope("zener in breakdown", behind(zener, 1_000.0), 10.0, "out");

        let schottky = CircuitElement::SchottkyDiode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
        };
        assert_ac_matches_dc_slope("schottky", behind(schottky, 1_000.0), 2.0, "out");

        let lit = CircuitElement::Photodiode {
            id: "D1".into(),
            nodes: ["0".into(), "out".into()],
            irradiance: 1e-4,
            temperature: 300.15,
        };
        assert_ac_matches_dc_slope("photodiode, reverse", behind(lit, 10_000.0), 5.0, "out");

        let forward = CircuitElement::Photodiode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            irradiance: 0.0,
            temperature: 300.15,
        };
        assert_ac_matches_dc_slope("photodiode, forward", behind(forward, 1_000.0), 2.0, "out");
    }

    /// A reverse-biased varactor behind R is an RC low-pass with
    /// C = C_j(V_bias): at ω = 1/(R·C_j) the gain is 1/√2 at −45°. (The
    /// varactor's 1 TΩ DC leak moves the bias by ~30 nV, hence 1e-8.)
    #[test]
    fn varactor_is_its_bias_capacitance_in_ac() {
        let params = sindr_devices::varactor::VaractorParams::default();
        let r = 10_000.0;
        let c_j = sindr_devices::varactor::junction_capacitance(-3.0, &params);
        let f_c = 1.0 / (2.0 * std::f64::consts::PI * r * c_j);
        let circuit = with(vec![
            vsrc("VIN", "in", 3.0),
            res("R1", "in", "out", r),
            CircuitElement::Varactor {
                id: "D1".into(),
                nodes: ["0".into(), "out".into()],
                params,
            },
        ]);
        let config = AcConfig {
            f_start: f_c,
            f_stop: f_c,
            num_points: 1,
            spacing: FrequencySpacing::Linear,
            source_id: "VIN".into(),
            ac_magnitude: 1.0,
        };
        let v = solve_ac(&circuit, &config).unwrap().points[0].node_voltages["out"];
        assert!(
            (v.norm() - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-8,
            "|H| = {}",
            v.norm()
        );
        assert!(
            (v.arg().to_degrees() + 45.0).abs() < 1e-6,
            "phase = {}",
            v.arg().to_degrees()
        );
    }

    /// A BJT stage with an input coupling capacitor and a collector choke.
    /// Its DC operating point must be the same stage with the capacitor
    /// removed and the choke replaced by a wire.
    #[test]
    fn dc_operating_point_opens_capacitors_and_shorts_inductors() {
        let stage = |reactive: bool| {
            let mut c = vec![
                vsrc("VCC", "vcc", 10.0),
                vsrc("VIN", "in", 0.0),
                res("RB", "vcc", "b", 470_000.0),
                res("RC", if reactive { "x" } else { "vcc" }, "c", 1_000.0),
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                },
            ];
            if reactive {
                c.push(CircuitElement::Capacitor {
                    id: "CIN".into(),
                    nodes: ["in".into(), "b".into()],
                    capacitance: 10e-6,
                });
                c.push(CircuitElement::Inductor {
                    id: "L1".into(),
                    nodes: ["vcc".into(), "x".into()],
                    inductance: 1e-3,
                });
            } else {
                // Keep `in` connected so both circuits have the same nodes.
                c.push(res("RIN", "in", "0", 1_000.0));
            }
            with(c)
        };
        let op = crate::solve_dc_operating_point(&stage(true)).unwrap();
        let plain = crate::solve_circuit(&stage(false)).unwrap();
        // Both are Newton solves stopped at RELTOL = 1e-3 (see SIN-14), so
        // they agree to that precision, not to round-off.
        for node in ["b", "c"] {
            assert!(
                (op.node_voltages[node] - plain.node_voltages[node]).abs()
                    < 2e-4 * plain.node_voltages[node].abs(),
                "V({node}): {} vs {}",
                op.node_voltages[node],
                plain.node_voltages[node]
            );
        }
        // AC biases there too: the mid-band gain is −gm·RC at that Ic.
        let ic = (10.0 - plain.node_voltages["c"]) / 1_000.0;
        let gm = ic / sindr_devices::diode::V_T;
        let config = AcConfig {
            f_start: 10e3,
            f_stop: 10e3,
            num_points: 1,
            spacing: FrequencySpacing::Linear,
            source_id: "VIN".into(),
            ac_magnitude: 1.0,
        };
        let gain = solve_ac(&stage(true), &config).unwrap().points[0].node_voltages["c"];
        assert!(
            (gain.norm() - gm * 1_000.0).abs() < 2e-2 * gm * 1_000.0,
            "|gain| {} vs gm·RC {}",
            gain.norm(),
            gm * 1_000.0
        );
    }
}
