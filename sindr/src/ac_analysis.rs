//! AC small-signal analysis module.
//!
//! Performs frequency-domain analysis by:
//! 1. Finding the DC operating point (capacitors open, inductors shorted)
//! 2. Linearizing all nonlinear elements at that point
//! 3. Building a complex-valued MNA system at each frequency
//! 4. Solving for node voltages as complex phasors

use std::borrow::Cow;
use std::collections::HashMap;

use nalgebra::DVector;
use num_complex::Complex64;

use sindr_devices::bjt::{self, BjtKind};
use sindr_devices::diode;
use sindr_devices::jfet;

use crate::circuit::{Circuit, CircuitElement};
use crate::device_params;
use crate::error::SimError;
use crate::mna::MnaMatrix;
use crate::node_map::NodeMap;

/// Frequency sweep spacing.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum FrequencySpacing {
    /// Logarithmic spacing: `num_points` in total, evenly spaced in
    /// log10(f) from `f_start` to `f_stop`. Not points per decade.
    Logarithmic,
    /// Linear spacing.
    Linear,
}

/// AC analysis configuration. Build one with [`AcConfig::new`] and its
/// `with_*` setters.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
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
    /// AC amplitude of the source: volts, linear (not dB). Results scale
    /// with it; 1 V makes node voltages read as gain.
    pub ac_magnitude: f64,
    /// AC phase of the source in degrees, as SPICE's `AC mag phase`. Every
    /// node's phase shifts by it; 0 makes [`AcPoint::phase_deg`] read as the
    /// phase relative to the source.
    #[cfg_attr(feature = "serde", serde(default))]
    pub ac_phase_deg: f64,
}

impl AcConfig {
    /// Sweep `num_points` frequencies from `f_start` to `f_stop` (Hz), both
    /// included, driving the voltage source `source_id`. Spacing defaults
    /// to [`FrequencySpacing::Logarithmic`], the source's AC magnitude to
    /// 1 V and its phase to 0°, so node voltages read as gain relative to
    /// the source.
    ///
    /// ```
    /// use sindr::ac_analysis::{AcConfig, FrequencySpacing};
    ///
    /// // SPICE `.ac lin 101 1k 2k` on `V1 in 0 AC 2 90`.
    /// let config = AcConfig::new("V1", 1e3, 2e3, 101)
    ///     .with_spacing(FrequencySpacing::Linear)
    ///     .with_magnitude(2.0)
    ///     .with_phase_deg(90.0);
    /// assert_eq!(config.num_points, 101);
    /// ```
    pub fn new(source_id: impl Into<String>, f_start: f64, f_stop: f64, num_points: usize) -> Self {
        Self {
            f_start,
            f_stop,
            num_points,
            spacing: FrequencySpacing::Logarithmic,
            source_id: source_id.into(),
            ac_magnitude: 1.0,
            ac_phase_deg: 0.0,
        }
    }

    /// Space the frequencies by `spacing` instead of logarithmically.
    pub fn with_spacing(mut self, spacing: FrequencySpacing) -> Self {
        self.spacing = spacing;
        self
    }

    /// Drive the source with an AC amplitude of `magnitude` volts (linear,
    /// not dB) instead of 1 V.
    pub fn with_magnitude(mut self, magnitude: f64) -> Self {
        self.ac_magnitude = magnitude;
        self
    }

    /// Give the source an AC phase of `phase_deg` degrees instead of 0°, as
    /// SPICE's `AC mag phase`.
    pub fn with_phase_deg(mut self, phase_deg: f64) -> Self {
        self.ac_phase_deg = phase_deg;
        self
    }
}

/// Result at a single frequency point.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct AcPoint {
    /// Frequency of this point (Hz).
    pub frequency: f64,
    /// Complex node voltages (V) keyed by node name. Includes the ground
    /// node (always 0).
    pub node_voltages: HashMap<String, Complex64>,
}

impl AcPoint {
    /// Gain in dB for a node: `20·log10(|V(node)| / source_magnitude)`, a
    /// voltage ratio. `source_magnitude` is in volts, normally
    /// [`AcConfig::ac_magnitude`]. `None` when the node isn't in the
    /// circuit.
    pub fn gain_db(&self, node: &str, source_magnitude: f64) -> Option<f64> {
        self.node_voltages.get(node).map(|v| {
            let gain = v.norm() / source_magnitude;
            20.0 * gain.log10()
        })
    }

    /// Phase of a node's voltage in degrees, in (−180°, 180°]: relative to
    /// the source when [`AcConfig::ac_phase_deg`] is 0, absolute otherwise,
    /// as ngspice reports it. `None` when the node isn't in the
    /// circuit.
    pub fn phase_deg(&self, node: &str) -> Option<f64> {
        self.node_voltages.get(node).map(|v| v.arg().to_degrees())
    }
}

/// Complete AC analysis result.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct AcResult {
    /// Result points, one per swept frequency.
    pub points: Vec<AcPoint>,
    /// Configuration used to produce this result.
    pub config: AcConfig,
}

impl AcResult {
    /// The gain curve for a node as `(frequency in Hz, gain in dB)` pairs,
    /// one per point, using [`AcConfig::ac_magnitude`].
    pub fn gain_curve(&self, node: &str) -> Vec<(f64, f64)> {
        self.points
            .iter()
            .filter_map(|p| {
                p.gain_db(node, self.config.ac_magnitude)
                    .map(|g| (p.frequency, g))
            })
            .collect()
    }

    /// The phase curve for a node as `(frequency in Hz, phase in degrees)`
    /// pairs, one per point; see [`AcPoint::phase_deg`].
    pub fn phase_curve(&self, node: &str) -> Vec<(f64, f64)> {
        self.points
            .iter()
            .filter_map(|p| p.phase_deg(node).map(|ph| (p.frequency, ph)))
            .collect()
    }
}

/// Linearized small-signal model for a nonlinear element.
pub(crate) struct SmallSignalModel {
    /// Conductance stamps: (row_node, col_node, value)
    /// None means ground.
    conductances: Vec<(Option<usize>, Option<usize>, f64)>,
}

/// Extract small-signal models from nonlinear elements at DC operating point.
pub(crate) fn linearize_at_dc(
    circuit: &Circuit,
    node_map: &NodeMap,
    dc_solution: &nalgebra::DVector<f64>,
) -> Vec<SmallSignalModel> {
    let mut models = Vec::new();

    for component in &circuit.components {
        match component {
            CircuitElement::Diode {
                model,
                nodes,
                temperature,
                ..
            } => {
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                let vp = p.map_or(0.0, |i| dc_solution[i]);
                let vq = q.map_or(0.0, |i| dc_solution[i]);
                let v_d = vp - vq;

                let params = device_params::diode(model.as_ref(), *temperature);
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
                model,
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
                let params = device_params::bjt(*bf, model.as_ref(), *temperature);

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
                let params = device_params::zener_diode(*vz, *temperature);
                let v_d = junction_voltage(nodes, node_map, dc_solution);
                let (g_d, _) = diode::diode_companion(v_d, &params);
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

                let comp = crate::device_params::mosfet_op(*kind, params, vg, vd, vs);

                // The body is tied to the source, so there is no gmb term.
                let conds = vec![
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
pub(crate) fn validate_config(config: &AcConfig) -> Result<(), SimError> {
    let invalid = |msg: String| Err(SimError::InvalidComponent(format!("AC sweep: {msg}")));

    if config.num_points == 0 {
        return invalid("num_points must be at least 1".into());
    }
    crate::validation::check_sweep_points("AC sweep", "num_points", config.num_points)?;
    for (name, value) in [
        ("f_start", config.f_start),
        ("f_stop", config.f_stop),
        ("ac_magnitude", config.ac_magnitude),
        ("ac_phase_deg", config.ac_phase_deg),
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
pub(crate) fn generate_frequencies(config: &AcConfig) -> Vec<f64> {
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
    a: &mut MnaMatrix<Complex64>,
    row: Option<usize>,
    col: Option<usize>,
    val: Complex64,
) {
    if let (Some(r), Some(c)) = (row, col) {
        a[(r, c)] += val;
    }
}

/// `circuit` with each self-heated device at the junction temperature its
/// operating point `dc` balanced at, so it linearises where it is biased
/// rather than at ambient. Borrowed unchanged when nothing is heated.
pub(crate) fn at_junction_temperatures<'a>(
    circuit: &'a Circuit,
    dc: &crate::SimulationResult,
) -> Cow<'a, Circuit> {
    if dc.junction_temperatures.is_empty() {
        return Cow::Borrowed(circuit);
    }
    let mut heated = circuit.clone();
    for element in &mut heated.components {
        let Some(&t) = dc.junction_temperatures.get(element.id()) else {
            continue;
        };
        if let Some(temperature) = crate::thermal::temperature_mut(element) {
            *temperature = t;
        }
    }
    Cow::Owned(heated)
}

/// `circuit` with each saturating inductor replaced by a linear one at its
/// incremental inductance `dλ/di` at the DC current `dc` carries through it
/// (the DC equivalent shorts it with a 0 V source of the same id).
pub(crate) fn incremental_inductors(circuit: &Circuit, dc: &crate::SimulationResult) -> Circuit {
    let mut out = circuit.clone();
    for element in &mut out.components {
        if let CircuitElement::Inductor {
            id,
            inductance,
            saturation,
            ..
        } = element
        {
            if let Some(sat) = saturation.take() {
                let i = dc.branch_currents.get(id.as_str()).copied().unwrap_or(0.0);
                *inductance = sindr_devices::inductor::incremental_inductance(i, *inductance, &sat);
            }
        }
    }
    out
}

/// The DC bias point of flat `circuit` as a solution vector over its node
/// map (node rows, and a regulator's branch row), for linearising around,
/// with the operating-point result it came from (device currents for noise
/// analysis, junction temperatures for [`at_junction_temperatures`]).
pub(crate) fn bias_point_with_result(
    circuit: &Circuit,
    opts: &crate::SolverOptions,
) -> Result<(NodeMap, DVector<f64>, crate::SimulationResult), SimError> {
    let dc_result = crate::solve_dc_operating_point(circuit, opts)?;
    let node_map = NodeMap::from_circuit(circuit);
    let num_nodes = node_map.num_nodes();
    let mut dc_solution = DVector::zeros(num_nodes + circuit.count_voltage_sources());
    for i in 0..num_nodes {
        if let Some(&v) = node_map
            .node_name(i)
            .and_then(|name| dc_result.node_voltages.get(name))
        {
            dc_solution[i] = v;
        }
    }
    // A regulator's piece depends on its output current, its branch row.
    for element in &circuit.components {
        if let CircuitElement::VoltageRegulator { id, .. } = element {
            if let (Some(k), Some(&i)) = (
                circuit.vsource_branch_index(id),
                dc_result.branch_currents.get(id),
            ) {
                dc_solution[num_nodes + k] = i;
            }
        }
    }
    Ok((node_map, dc_solution, dc_result))
}

/// The linearised (small-signal) MNA system of `circuit` at angular
/// frequency `omega`, with every source zero: `b` is left for the caller
/// to excite.
pub(crate) struct SmallSignalSystem {
    a: MnaMatrix<Complex64>,
    /// Branch row of each independent voltage source, by id.
    vsource_branches: HashMap<String, usize>,
}

impl SmallSignalSystem {
    /// Size of the system: node rows, then branch rows.
    pub(crate) fn size(&self) -> usize {
        self.a.size()
    }

    /// Branch row of the independent voltage source `id`.
    pub(crate) fn vsource_branch(&self, id: &str) -> Option<usize> {
        self.vsource_branches.get(id).copied()
    }

    /// Solves `A x = b` for the given excitation.
    pub(crate) fn solve(&self, b: &DVector<Complex64>) -> Result<DVector<Complex64>, SimError> {
        self.a.solve(b)
    }

    /// Solves `Aᵀ x = b`: the adjoint system.
    pub(crate) fn solve_transpose(
        &self,
        b: &DVector<Complex64>,
    ) -> Result<DVector<Complex64>, SimError> {
        self.a.transpose().solve(b)
    }
}

/// Builds the small-signal system of `circuit` at `omega` (rad/s),
/// linearised around `dc_solution` through `ss_models`.
pub(crate) fn small_signal_system(
    circuit: &Circuit,
    node_map: &NodeMap,
    dc_solution: &DVector<f64>,
    ss_models: &[SmallSignalModel],
    omega: f64,
) -> SmallSignalSystem {
    let num_nodes = node_map.num_nodes();
    let num_vsources = circuit.count_voltage_sources();
    let mut a = MnaMatrix::zeros(num_nodes + num_vsources);
    let mut vsource_index: usize = 0;
    let mut vsource_branches = HashMap::new();
    let j_omega = Complex64::new(0.0, omega);

    // Stamp linear components
    for component in &circuit.components {
        match component {
            // Split into their parts by `solver_circuit` first.
            CircuitElement::SwitchSpdt { .. }
            | CircuitElement::Crystal { .. }
            | CircuitElement::FerriteBead { .. } => {}
            CircuitElement::Resistor {
                nodes, resistance, ..
            } => {
                let g = Complex64::new(1.0 / resistance, 0.0);
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, g);
                stamp_complex_conductance(&mut a, q, q, g);
                stamp_complex_conductance(&mut a, p, q, -g);
                stamp_complex_conductance(&mut a, q, p, -g);
            }
            CircuitElement::Capacitor {
                nodes, capacitance, ..
            } => {
                let y = j_omega * *capacitance; // Y = j*omega*C
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, y);
                stamp_complex_conductance(&mut a, q, q, y);
                stamp_complex_conductance(&mut a, p, q, -y);
                stamp_complex_conductance(&mut a, q, p, -y);
            }
            CircuitElement::Bjt {
                nodes,
                parasitic_caps: Some(pc),
                ..
            } => {
                for (_, c, p, q) in crate::transient::bjt_parasitics(nodes, pc) {
                    let y = j_omega * c;
                    let (p, q) = (node_map.index(&p), node_map.index(&q));
                    stamp_complex_conductance(&mut a, p, p, y);
                    stamp_complex_conductance(&mut a, q, q, y);
                    stamp_complex_conductance(&mut a, p, q, -y);
                    stamp_complex_conductance(&mut a, q, p, -y);
                }
            }
            CircuitElement::Mosfet {
                nodes,
                parasitic_caps: Some(pc),
                ..
            } => {
                for (_, c, p, q) in crate::transient::mosfet_parasitics(nodes, pc) {
                    let y = j_omega * c;
                    let (p, q) = (node_map.index(&p), node_map.index(&q));
                    stamp_complex_conductance(&mut a, p, p, y);
                    stamp_complex_conductance(&mut a, q, q, y);
                    stamp_complex_conductance(&mut a, p, q, -y);
                    stamp_complex_conductance(&mut a, q, p, -y);
                }
            }
            CircuitElement::Varactor { nodes, params, .. } => {
                // Small-signal capacitance at the DC bias.
                let v_d = junction_voltage(nodes, node_map, dc_solution);
                let c = sindr_devices::varactor::junction_capacitance(v_d, params);
                let y = j_omega * c;
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, y);
                stamp_complex_conductance(&mut a, q, q, y);
                stamp_complex_conductance(&mut a, p, q, -y);
                stamp_complex_conductance(&mut a, q, p, -y);
            }
            CircuitElement::Battery { nodes, params, .. } => {
                // Its open-circuit voltage is a DC source, zero in AC; what
                // is left is Z = R0 + R1‖C1.
                let mut z = Complex64::new(params.r_series, 0.0);
                if let Some(pair) = params.rc {
                    z += Complex64::new(pair.r, 0.0)
                        / (Complex64::new(1.0, 0.0) + j_omega * pair.r * pair.c);
                }
                let y = Complex64::new(1.0, 0.0) / z;
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, y);
                stamp_complex_conductance(&mut a, q, q, y);
                stamp_complex_conductance(&mut a, p, q, -y);
                stamp_complex_conductance(&mut a, q, p, -y);
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
                stamp_complex_conductance(&mut a, p, p, y);
                stamp_complex_conductance(&mut a, q, q, y);
                stamp_complex_conductance(&mut a, p, q, -y);
                stamp_complex_conductance(&mut a, q, p, -y);
            }
            CircuitElement::VoltageSource { id, nodes, .. } => {
                let branch = num_nodes + vsource_index;
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                let one = Complex64::new(1.0, 0.0);

                if let Some(pi) = p {
                    a[(pi, branch)] += one;
                    a[(branch, pi)] += one;
                }
                if let Some(qi) = q {
                    a[(qi, branch)] -= one;
                    a[(branch, qi)] -= one;
                }

                vsource_branches.insert(id.clone(), branch);
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
                stamp_complex_conductance(&mut a, p, p, g);
                stamp_complex_conductance(&mut a, q, q, g);
                stamp_complex_conductance(&mut a, p, q, -g);
                stamp_complex_conductance(&mut a, q, p, -g);
            }
            CircuitElement::Pushbutton { nodes, closed, .. } => {
                let r = if *closed { 0.01 } else { 1e9 };
                let g = Complex64::new(1.0 / r, 0.0);
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, g);
                stamp_complex_conductance(&mut a, q, q, g);
                stamp_complex_conductance(&mut a, p, q, -g);
                stamp_complex_conductance(&mut a, q, p, -g);
            }
            CircuitElement::Photoresistor {
                nodes, light_level, ..
            } => {
                use crate::stamp::ldr_resistance;
                let r = ldr_resistance(*light_level);
                let g = Complex64::new(1.0 / r, 0.0);
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, g);
                stamp_complex_conductance(&mut a, q, q, g);
                stamp_complex_conductance(&mut a, p, q, -g);
                stamp_complex_conductance(&mut a, q, p, -g);
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
                stamp_complex_conductance(&mut a, p, p, g_top);
                stamp_complex_conductance(&mut a, w, w, g_top);
                stamp_complex_conductance(&mut a, p, w, -g_top);
                stamp_complex_conductance(&mut a, w, p, -g_top);
                stamp_complex_conductance(&mut a, w, w, g_bot);
                stamp_complex_conductance(&mut a, q, q, g_bot);
                stamp_complex_conductance(&mut a, w, q, -g_bot);
                stamp_complex_conductance(&mut a, q, w, -g_bot);
            }
            CircuitElement::Relay {
                nodes,
                coil_resistance,
                pickup_voltage,
                ..
            } => {
                let g = Complex64::new(1.0 / coil_resistance, 0.0);
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, g);
                stamp_complex_conductance(&mut a, q, q, g);
                stamp_complex_conductance(&mut a, p, q, -g);
                stamp_complex_conductance(&mut a, q, p, -g);
                // Contact: open or closed as at the DC bias point.
                let r = crate::stamp::relay_contact_resistance(
                    node_map,
                    nodes,
                    *pickup_voltage,
                    Some(dc_solution),
                );
                let g = Complex64::new(1.0 / r, 0.0);
                let c1 = node_map.index(&nodes[2]);
                let c2 = node_map.index(&nodes[3]);
                stamp_complex_conductance(&mut a, c1, c1, g);
                stamp_complex_conductance(&mut a, c2, c2, g);
                stamp_complex_conductance(&mut a, c1, c2, -g);
                stamp_complex_conductance(&mut a, c2, c1, -g);
            }
            // Thermistor: passive resistor with temperature-dependent resistance
            CircuitElement::Thermistor {
                nodes, temperature, ..
            } => {
                let r = crate::device_params::thermistor_resistance(*temperature);
                let g = Complex64::new(1.0 / r, 0.0);
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, g);
                stamp_complex_conductance(&mut a, q, q, g);
                stamp_complex_conductance(&mut a, p, q, -g);
                stamp_complex_conductance(&mut a, q, p, -g);
            }
            // A diode's stored charge. The junction, g_j + jωC_j, sits
            // behind RS; the small-signal model below already stamps its
            // ω = 0 conductance g_d, so this adds the rest of
            // 1/(RS + 1/(g_j + jωC_j)).
            CircuitElement::Diode {
                model,
                nodes,
                temperature,
                ..
            } => {
                if let Some(charge) = device_params::diode_charge(model.as_ref()) {
                    let params = device_params::diode(model.as_ref(), *temperature);
                    let v_d = junction_voltage(nodes, node_map, dc_solution);
                    let (g_j, c_j) = diode::junction_small_signal(v_d, &params, &charge);
                    let (g_d, _) = diode::diode_companion(v_d, &params);
                    let junction = Complex64::new(g_j, 0.0) + j_omega * c_j;
                    let y = if params.rs > 0.0 {
                        Complex64::new(1.0, 0.0) / (params.rs + 1.0 / junction)
                    } else {
                        junction
                    } - g_d;
                    let y = if y.is_finite() {
                        y
                    } else {
                        Complex64::new(0.0, 0.0)
                    };
                    let p = node_map.index(&nodes[0]);
                    let q = node_map.index(&nodes[1]);
                    stamp_complex_conductance(&mut a, p, p, y);
                    stamp_complex_conductance(&mut a, q, q, y);
                    stamp_complex_conductance(&mut a, p, q, -y);
                    stamp_complex_conductance(&mut a, q, p, -y);
                }
            }
            // Nonlinear elements: handled via small-signal models below
            CircuitElement::Led { .. }
            | CircuitElement::Bjt { .. }
            | CircuitElement::Mosfet { .. }
            | CircuitElement::ZenerDiode { .. }
            | CircuitElement::SchottkyDiode { .. }
            | CircuitElement::Photodiode { .. }
            | CircuitElement::Jfet { .. }
            | CircuitElement::Igbt { .. } => {}
            // Transformer: coupled inductors with a branch current per
            // winding (I1 into p1, I2 into p2), as in transient:
            //   V1 = jω·L1·I1 + jω·M·I2,  V2 = jω·M·I1 + jω·L2·I2,
            // with M = k·√(L1·L2). At ω = 0 both windings are shorts.
            CircuitElement::Transformer {
                nodes, l1, l2, k, ..
            } => {
                let m = *k * (*l1 * *l2).sqrt();
                let z = [[j_omega * *l1, j_omega * m], [j_omega * m, j_omega * *l2]];
                let rows = [num_nodes + vsource_index, num_nodes + vsource_index + 1];
                let one = Complex64::new(1.0, 0.0);
                for (w, &row) in rows.iter().enumerate() {
                    if let Some(pi) = node_map.index(&nodes[2 * w]) {
                        a[(pi, row)] += one;
                        a[(row, pi)] += one;
                    }
                    if let Some(qi) = node_map.index(&nodes[2 * w + 1]) {
                        a[(qi, row)] -= one;
                        a[(row, qi)] -= one;
                    }
                    for (j, &col) in rows.iter().enumerate() {
                        a[(row, col)] -= z[w][j];
                    }
                }
                vsource_index += 2;
            }
            // Modelled op-amp or comparator, linearised at the bias point:
            // see `op_amp`.
            CircuitElement::OpAmp { model: Some(_), .. } | CircuitElement::Comparator { .. } => {
                let branch = num_nodes + vsource_index;
                vsource_index += 2;
                let Some(op) =
                    crate::op_amp::OpAmpStamp::of(component, &circuit.ground_node, branch)
                else {
                    continue;
                };
                let (model, nodes) = (op.model, op.nodes);
                let br = op.branch;
                let x = br + 1;
                let one = Complex64::new(1.0, 0.0);
                // The output is referred to ground, which has no row.
                if let Some(o) = node_map.index(&nodes[2]) {
                    a[(o, br)] += one;
                    a[(br, o)] += one;
                }
                let (tau, c_x, c_u, slope) = op.small_signal(node_map, dc_solution);
                a[(br, x)] -= Complex64::new(slope, 0.0);
                a[(br, br)] -= Complex64::new(model.r_out, 0.0);
                a[(x, x)] += Complex64::new(c_x, 0.0) + j_omega * tau;
                if let Some(p) = node_map.index(&nodes[0]) {
                    a[(x, p)] -= Complex64::new(c_u, 0.0);
                }
                if let Some(m) = node_map.index(&nodes[1]) {
                    a[(x, m)] += Complex64::new(c_u, 0.0);
                }
            }
            // Unmodelled op-amp: high-gain VCVS (gain=1e5), linear.
            CircuitElement::OpAmp { nodes, .. } => {
                let branch = num_nodes + vsource_index;
                let p = node_map.index(&nodes[2]); // out, referred to ground
                let cp = node_map.index(&nodes[0]); // in+
                let cq = node_map.index(&nodes[1]); // in-
                let one = Complex64::new(1.0, 0.0);
                let gain = Complex64::new(1e5, 0.0);

                if let Some(pi) = p {
                    a[(pi, branch)] += one;
                    a[(branch, pi)] += one;
                }
                if let Some(cpi) = cp {
                    a[(branch, cpi)] -= gain;
                }
                if let Some(cqi) = cq {
                    a[(branch, cqi)] += gain;
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
                    a[(pi, branch)] += one;
                }
                if let Some(qi) = q {
                    a[(qi, branch)] -= one;
                }
                if let Some(pi) = p {
                    a[(branch, pi)] += one;
                }
                if let Some(qi) = q {
                    a[(branch, qi)] -= one;
                }
                if let Some(cpi) = cp {
                    a[(branch, cpi)] -= g;
                }
                if let Some(cqi) = cq {
                    a[(branch, cqi)] += g;
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

                // Same direction as the DC stamp: out of `from`, into `to`.
                if let (Some(fi), Some(cpi)) = (from, cp) {
                    a[(fi, cpi)] += g;
                }
                if let (Some(fi), Some(cqi)) = (from, cq) {
                    a[(fi, cqi)] -= g;
                }
                if let (Some(ti), Some(cpi)) = (to, cp) {
                    a[(ti, cpi)] -= g;
                }
                if let (Some(ti), Some(cqi)) = (to, cq) {
                    a[(ti, cqi)] += g;
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
                    a[(pi, branch)] += one;
                }
                if let Some(qi) = q {
                    a[(qi, branch)] -= one;
                }
                if let Some(pi) = p {
                    a[(branch, pi)] += one;
                }
                if let Some(qi) = q {
                    a[(branch, qi)] -= one;
                }
                if let Some(cb) = circuit.vsource_branch_index(control_source) {
                    a[(branch, num_nodes + cb)] -= r;
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
                let gain = Complex64::new(*alpha, 0.0);

                if let Some(cb) = circuit.vsource_branch_index(control_source) {
                    if let Some(fi) = from {
                        a[(fi, num_nodes + cb)] += gain;
                    }
                    if let Some(ti) = to {
                        a[(ti, num_nodes + cb)] -= gain;
                    }
                }
            }
            // Fuse: stamp as resistor (intact=0.001 Ohm, blown=1e9 Ohm) in AC
            CircuitElement::Fuse { nodes, blown, .. } => {
                let r = if *blown { 1e9_f64 } else { 0.001_f64 };
                let g = Complex64::new(1.0 / r, 0.0);
                let p = node_map.index(&nodes[0]);
                let q = node_map.index(&nodes[1]);
                stamp_complex_conductance(&mut a, p, p, g);
                stamp_complex_conductance(&mut a, q, q, g);
                stamp_complex_conductance(&mut a, p, q, -g);
                stamp_complex_conductance(&mut a, q, p, -g);
            }
            // The piece of its characteristic the bias point is on: input
            // ripple reaches the output through line regulation (or all of
            // it in dropout), and load current through the output
            // resistance. The quiescent current only matters below dropout, where it
            // ramps with the input.
            CircuitElement::VoltageRegulator {
                nodes,
                voltage,
                model,
                ..
            } => {
                let branch = num_nodes + vsource_index;
                let piece = crate::stamp::regulator_piece_at(
                    node_map,
                    nodes,
                    *voltage,
                    model,
                    branch,
                    Some(dc_solution),
                );
                let c = |x: f64| Complex64::new(x, 0.0);
                let input = node_map.index(&nodes[0]);
                let out = node_map.index(&nodes[1]);
                let gnd = node_map.index(&nodes[2]);
                if let Some(o) = out {
                    a[(o, branch)] += c(1.0);
                    a[(branch, o)] += c(1.0);
                }
                if let Some(i) = input {
                    a[(i, branch)] -= c(1.0);
                    a[(branch, i)] -= c(piece.d_vin);
                }
                if let Some(g) = gnd {
                    a[(branch, g)] += c(piece.d_vin - 1.0);
                }
                a[(branch, branch)] += c(piece.d_iout);
                // The quiescent current is fixed above dropout; below it, it
                // ramps with the input, a conductance from input to ground.
                if let Some(params) = model {
                    let vin =
                        crate::stamp::regulator_state(node_map, nodes, branch, Some(dc_solution))
                            .map_or(0.0, |(vin, _, _)| vin);
                    let (_, g_q) =
                        sindr_devices::regulator::regulator_quiescent(params, *voltage, vin);
                    for (r, col, sign) in [
                        (input, input, 1.0),
                        (input, gnd, -1.0),
                        (gnd, input, -1.0),
                        (gnd, gnd, 1.0),
                    ] {
                        if let (Some(r), Some(col)) = (r, col) {
                            a[(r, col)] += c(sign * g_q);
                        }
                    }
                }
                vsource_index += 1;
            }
        }
    }

    for model in ss_models {
        for &(row, col, val) in &model.conductances {
            stamp_complex_conductance(&mut a, row, col, Complex64::new(val, 0.0));
        }
    }

    SmallSignalSystem {
        a,
        vsource_branches,
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
/// 1. Solve the DC operating point of `circuit`, with capacitors and
///    varactors open and inductors and transformer windings shorted.
/// 2. Linearise every nonlinear element at that operating point: diodes,
///    LEDs, Zener, Schottky and photodiodes, BJTs (full terminal Jacobian,
///    junction temperature included), MOSFETs, JFETs, IGBTs, and varactors
///    (their capacitance at the bias). A relay's contact is open or closed
///    as the bias point leaves it.
/// 3. At each frequency, build a complex-valued MNA system (capacitors and
///    inductors become `jωC` / `jωL` admittances, a transformer the coupled
///    pair `V = jω·[[L1, M], [M, L2]]·I`), solve it, and store the complex
///    node voltage phasors.
///
/// # Errors
///
/// - [`SimError::InvalidComponent`] if `config` is invalid: zero points,
///   non-finite values, `f_start > f_stop`, `f_start <= 0` with logarithmic
///   spacing, `f_start < 0` with linear spacing, or more than 10⁷ points;
///   or if `source_id` isn't the id of a voltage source in `circuit`
/// - Any [`SimError`] from the DC operating-point solve
/// - [`SimError::SingularMatrix`] if the linearised AC system is singular
///   at any frequency
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement};
/// use sindr::ac_analysis::{solve_ac, AcConfig};
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
///     ..Default::default()
/// };
///
/// // 11 points from 1 Hz to 100 kHz, log-spaced, 1 V on V1.
/// let config = AcConfig::new("V1", 1.0, 100_000.0, 11);
///
/// let result = solve_ac(&circuit, &config).unwrap();
/// assert_eq!(result.points.len(), 11);
///
/// // Bode plot data for the output node:
/// let gain = result.gain_curve("n_out");
/// let phase = result.phase_curve("n_out");
/// ```
pub fn solve_ac(circuit: &Circuit, config: &AcConfig) -> Result<AcResult, SimError> {
    solve_ac_with(circuit, config, &crate::SolverOptions::default())
}

/// [`solve_ac`] with `opts`.
pub(crate) fn solve_ac_with(
    circuit: &Circuit,
    config: &AcConfig,
    opts: &crate::SolverOptions,
) -> Result<AcResult, SimError> {
    let original = circuit;
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    validate_config(config)?;
    let drives_a_voltage_source = circuit
        .components
        .iter()
        .any(|c| matches!(c, CircuitElement::VoltageSource { id, .. } if *id == config.source_id));
    if !drives_a_voltage_source {
        return Err(SimError::InvalidComponent(format!(
            "AC sweep: source_id '{}' is not a voltage source in the circuit",
            config.source_id
        )));
    }

    let (node_map, dc_solution, dc) = bias_point_with_result(circuit, opts)?;
    let circuit = &incremental_inductors(&at_junction_temperatures(circuit, &dc), &dc);
    let num_nodes = node_map.num_nodes();
    let ss_models = linearize_at_dc(circuit, &node_map, &dc_solution);

    // Step 3: Frequency sweep
    let frequencies = generate_frequencies(config);
    let mut points = Vec::with_capacity(frequencies.len());

    for &freq in &frequencies {
        let omega = 2.0 * std::f64::consts::PI * freq;

        let system = small_signal_system(circuit, &node_map, &dc_solution, &ss_models, omega);
        // Every other source is zero in AC: voltage sources short, current
        // sources open.
        let mut b = DVector::from_element(system.size(), Complex64::new(0.0, 0.0));
        if let Some(branch) = system.vsource_branch(&config.source_id) {
            b[branch] =
                Complex64::from_polar(config.ac_magnitude, config.ac_phase_deg.to_radians());
        }
        let solution = system.solve(&b)?;

        // Extract node voltages
        let mut node_voltages = HashMap::new();
        node_voltages.insert(circuit.ground_node.clone(), Complex64::new(0.0, 0.0));
        for i in 0..num_nodes {
            if let Some(name) = node_map.node_name(i) {
                node_voltages.insert(name.to_string(), solution[i]);
            }
        }
        crate::composite::strip_nodes(original, &mut node_voltages);

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

    #[test]
    fn config_defaults_and_setters() {
        let config = AcConfig::new("V1", 1.0, 1e6, 61);
        assert_eq!(config.source_id, "V1");
        assert_eq!(
            (config.f_start, config.f_stop, config.num_points),
            (1.0, 1e6, 61)
        );
        assert_eq!(config.spacing, FrequencySpacing::Logarithmic);
        assert_eq!(config.ac_magnitude, 1.0);
        assert_eq!(config.ac_phase_deg, 0.0);

        let config = config
            .with_spacing(FrequencySpacing::Linear)
            .with_magnitude(2.5)
            .with_phase_deg(-90.0);
        assert_eq!(config.spacing, FrequencySpacing::Linear);
        assert_eq!(config.ac_magnitude, 2.5);
        assert_eq!(config.ac_phase_deg, -90.0);
    }

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
            ..Default::default()
        };

        let config = AcConfig::new("V1", 1.0, 100_000.0, 100);

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
            ..Default::default()
        };

        let config = AcConfig::new("V1", 1.0, 10.0, 5).with_spacing(FrequencySpacing::Linear);

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
            ..Default::default()
        };

        let config = AcConfig::new("V1", 10_000.0, 100_000.0, 3);

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
        let config = AcConfig::new("V1", 1.0, 1000.0, 4);
        let freqs = generate_frequencies(&config);
        assert_eq!(freqs.len(), 4);
        assert_abs_diff_eq!(freqs[0], 1.0, epsilon = 1e-10);
        assert_abs_diff_eq!(freqs[1], 10.0, epsilon = 1e-10);
        assert_abs_diff_eq!(freqs[2], 100.0, epsilon = 1e-8);
        assert_abs_diff_eq!(freqs[3], 1000.0, epsilon = 1e-6);
    }

    #[test]
    fn frequency_generation_linear() {
        let config = AcConfig::new("V1", 100.0, 400.0, 4).with_spacing(FrequencySpacing::Linear);
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
                    model: None,
                },
            ],
            ..Default::default()
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

        let config = AcConfig::new("VIN", 1.0, 1.0, 1).with_spacing(FrequencySpacing::Linear);
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
        let config = AcConfig::new("VIN", 1.0, 1.0, 1).with_spacing(FrequencySpacing::Linear);
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
            ..Default::default()
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

    /// The AC response at one frequency of `VIN` into `circuit`.
    fn ac_at(circuit: &Circuit, f: f64) -> HashMap<String, Complex64> {
        let config = AcConfig::new("VIN", f, f, 1).with_spacing(FrequencySpacing::Linear);
        solve_ac(circuit, &config)
            .unwrap()
            .points
            .remove(0)
            .node_voltages
    }

    /// A reverse-biased diode card with CJO and no RS is an RC low-pass
    /// with C = CJO/(1 + V_R/VJ)^M. At 3 V reverse with VJ = 1 and M = 0.5
    /// that is CJO/2, so at ω = 1/(R·CJO/2) the gain is 1/√2 at −45°. The
    /// leakage (IS·R ≈ 1e-10 V) is far below the tolerance.
    #[test]
    fn diode_card_without_rs_is_its_junction_capacitance_in_ac() {
        let r = 10_000.0;
        let cjo = 20e-12;
        let f_c = 1.0 / (2.0 * std::f64::consts::PI * r * cjo / 2.0);
        let circuit = with(vec![
            vsrc("VIN", "in", 3.0),
            res("R1", "in", "out", r),
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["0".into(), "out".into()],
                temperature: 300.15,
                model: Some(crate::circuit::DiodeModel {
                    cjo,
                    ..Default::default()
                }),
            },
        ]);
        let v = ac_at(&circuit, f_c)["out"];
        assert!(
            (v.norm() - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-8,
            "|H| = {}",
            v.norm()
        );
        assert!((v.arg().to_degrees() + 45.0).abs() < 1e-6, "{v}");
    }

    /// An inductor is a short at 0 Hz (linear spacing may start there) and
    /// `jωL` above it: behind R, `H = jωL/(R + jωL)`, `j/(1 + j)` at
    /// ω = R/L.
    #[test]
    fn inductor_shorts_at_dc_and_is_jwl_above_it() {
        let (r, l) = (1e3, 1e-3);
        let f_c = r / (2.0 * std::f64::consts::PI * l);
        let circuit = with(vec![
            vsrc("VIN", "in", 0.0),
            res("R1", "in", "out", r),
            CircuitElement::Inductor {
                id: "L1".into(),
                nodes: ["out".into(), "0".into()],
                inductance: l,
                saturation: None,
            },
        ]);
        let config = AcConfig::new("VIN", 0.0, f_c, 2).with_spacing(FrequencySpacing::Linear);
        let points = solve_ac(&circuit, &config).unwrap().points;
        assert_eq!(points[0].frequency, 0.0);
        // 1 kΩ against the 1e12 S short.
        assert!(points[0].node_voltages["out"].norm() < 1e-12);
        let h = points[1].node_voltages["out"];
        let expected = Complex64::new(0.0, 1.0) / Complex64::new(1.0, 1.0);
        assert!((h - expected).norm() < 1e-12, "{h}");
    }

    /// An inverting amplifier on the modelled op-amp (in+ grounded) is the
    /// single-pole closed loop `−(RF/R1) / (1 + (1 + RF/R1)/A(jω))` with
    /// `A = A0/(1 + jf/fp)`, `fp = GBW/A0`: 10 Hz for the default model.
    #[test]
    fn modelled_inverting_amplifier_follows_the_single_pole_loop() {
        let model = crate::circuit::OpAmpModel::default();
        let (r1, rf) = (1e3, 10e3);
        let circuit = with(vec![
            vsrc("VIN", "in", 0.0),
            res("R1", "in", "fb", r1),
            res("RF", "fb", "out", rf),
            res("RL", "out", "0", 10e3),
            CircuitElement::OpAmp {
                id: "U1".into(),
                nodes: ["0".into(), "fb".into(), "out".into()],
                v_pos: 15.0,
                v_neg: -15.0,
                model: Some(model),
            },
        ]);
        let fp = model.gbw.unwrap() / model.gain;
        for f in [1.0, 1e3] {
            let a = Complex64::new(model.gain, 0.0) / Complex64::new(1.0, f / fp);
            let expected = -(rf / r1) / (1.0 + (1.0 + rf / r1) / a);
            let h = ac_at(&circuit, f)["out"];
            assert!(
                (h - expected).norm() < 1e-6 * expected.norm(),
                "{f} Hz: {h} vs {expected}"
            );
        }
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
        let config = AcConfig::new("VIN", f_c, f_c, 1).with_spacing(FrequencySpacing::Linear);
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
                    model: None,
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
                    saturation: None,
                });
            } else {
                // Keep `in` connected so both circuits have the same nodes.
                c.push(res("RIN", "in", "0", 1_000.0));
            }
            with(c)
        };
        let op = crate::solve_dc_operating_point(&stage(true), &crate::SolverOptions::default())
            .unwrap();
        let plain = crate::solve_circuit(&stage(false)).unwrap();
        // Both are Newton solves stopped at RELTOL = 1e-3, so
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
        let config = AcConfig::new("VIN", 10e3, 10e3, 1).with_spacing(FrequencySpacing::Linear);
        let gain = solve_ac(&stage(true), &config).unwrap().points[0].node_voltages["c"];
        assert!(
            (gain.norm() - gm * 1_000.0).abs() < 2e-2 * gm * 1_000.0,
            "|gain| {} vs gm·RC {}",
            gain.norm(),
            gm * 1_000.0
        );
    }
}
