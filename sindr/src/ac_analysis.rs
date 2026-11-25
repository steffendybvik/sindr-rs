//! AC small-signal analysis module.
//!
//! Performs frequency-domain analysis by:
//! 1. Finding the DC operating point (capacitors open, inductors shorted)
//! 2. Linearizing all nonlinear elements at that point
//! 3. Building a complex-valued MNA system at each frequency
//! 4. Solving for node voltages as complex phasors

use crate::element::layout::Layout;
use crate::element::{
    AcBias, AcStamp, Conductance, Inductor, SwitchingRegulator, Transformer, VoltageRegulator,
    VoltageSource,
};
use crate::handle::ComponentId;
use std::borrow::Cow;
use std::cell::OnceCell;
use std::collections::HashMap;

use nalgebra::{DMatrix, DVector};
use num_complex::Complex64;

use crate::circuit::{Circuit, CircuitElement};
use crate::error::{ConfigKind, SimError};
use crate::mna::{Factor, MnaMatrix};
use crate::node_map::{NodeMap, Terminals};

/// Frequency sweep spacing.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
    pub source_id: ComponentId,
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
    pub fn new(
        source_id: impl Into<ComponentId>,
        f_start: f64,
        f_stop: f64,
        num_points: usize,
    ) -> Self {
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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct AcPoint {
    /// Frequency of this point (Hz).
    pub frequency: f64,
    /// Complex node voltages (V) keyed by node name. Includes the ground
    /// node (always 0).
    #[cfg_attr(feature = "schemars", schemars(with = "HashMap<String, [f64; 2]>"))]
    pub node_voltages: HashMap<String, Complex64>,
    /// Complex branch currents (A) of the components that introduce a
    /// current unknown (voltage sources, controlled voltage sources,
    /// op-amps, regulators), keyed by component id: the small-signal
    /// counterpart of [`SimulationResult::branch_currents`], with its sign
    /// convention. A source delivering power reads negative.
    ///
    /// [`SimulationResult::branch_currents`]: crate::SimulationResult::branch_currents
    #[cfg_attr(feature = "serde", serde(default))]
    #[cfg_attr(feature = "schemars", schemars(with = "HashMap<String, [f64; 2]>"))]
    pub branch_currents: HashMap<String, Complex64>,
}

impl AcPoint {
    /// The complex voltage at `node` (V), or `None` when there's no such
    /// node. Ground reads 0.
    pub fn voltage(&self, node: impl AsRef<str>) -> Option<Complex64> {
        let node = node.as_ref();
        self.node_voltages.get(node).copied()
    }

    /// The magnitude of the voltage at `node`, `|V(node)|` (V).
    pub fn magnitude(&self, node: impl AsRef<str>) -> Option<f64> {
        let node = node.as_ref();
        self.voltage(node).map(|v| v.norm())
    }

    /// Gain in dB for a node: `20·log10(|V(node)| / source_magnitude)`, a
    /// voltage ratio. `source_magnitude` is in volts, normally
    /// [`AcConfig::ac_magnitude`]. `None` when the node isn't in the
    /// circuit.
    pub fn gain_db(&self, node: impl AsRef<str>, source_magnitude: f64) -> Option<f64> {
        let node = node.as_ref();
        self.node_voltages.get(node).map(|v| {
            let gain = v.norm() / source_magnitude;
            20.0 * gain.log10()
        })
    }

    /// The complex branch current of component `id` (A), with the sign
    /// convention of [`AcPoint::branch_currents`]. `None` when `id` has no
    /// branch current (resistors, capacitors, inductors and other parts
    /// without a current unknown) or isn't in the circuit.
    pub fn branch_current(&self, id: impl AsRef<str>) -> Option<Complex64> {
        let id = id.as_ref();
        self.branch_currents.get(id).copied()
    }

    /// Phase of a node's voltage in degrees, in (−180°, 180°]: relative to
    /// the source when [`AcConfig::ac_phase_deg`] is 0, absolute otherwise,
    /// as ngspice reports it. `None` when the node isn't in the
    /// circuit.
    pub fn phase_deg(&self, node: impl AsRef<str>) -> Option<f64> {
        let node = node.as_ref();
        self.node_voltages.get(node).map(|v| v.arg().to_degrees())
    }
}

/// Complete AC analysis result.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct AcResult {
    /// Result points, one per swept frequency.
    pub points: Vec<AcPoint>,
    /// Configuration used to produce this result.
    pub config: AcConfig,
}

impl AcResult {
    /// A node's complex voltage as `(frequency in Hz, V)` pairs, one per
    /// point. Empty when there's no such node.
    ///
    /// ```
    /// use sindr::{solve_ac, AcConfig, Circuit, CircuitElement};
    /// use sindr::element::{Capacitor, Resistor, VoltageSource};
    ///
    /// // RC low-pass, corner at 1/(2π·1k·1µ) ≈ 159 Hz.
    /// let circuit = Circuit::new("0", vec![
    ///     CircuitElement::VoltageSource(VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 0.0, waveform: None }),
    ///     CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()], resistance: 1e3 }),
    ///     CircuitElement::Capacitor(Capacitor { id: "C1".into(), nodes: ["out".into(), "0".into()], capacitance: 1e-6 }),
    /// ]);
    /// let ac = solve_ac(&circuit, &AcConfig::new("V1", 1.0, 1e5, 51))?;
    ///
    /// let first = &ac.points[0];
    /// assert!((first.magnitude("out").unwrap() - 1.0).abs() < 1e-3);
    /// let (f, v) = *ac.node_voltage_curve("out").last().unwrap();
    /// assert_eq!(f, 1e5);
    /// assert!(v.norm() < 0.01);
    /// assert_eq!(ac.magnitude_curve("out").len(), 51);
    /// assert!(ac.node_voltage_curve("nope").is_empty());
    /// # Ok::<(), sindr::SimError>(())
    /// ```
    ///
    pub fn node_voltage_curve(&self, node: impl AsRef<str>) -> Vec<(f64, Complex64)> {
        let node = node.as_ref();
        self.points
            .iter()
            .filter_map(|p| p.voltage(node).map(|v| (p.frequency, v)))
            .collect()
    }

    /// A component's complex branch current as `(frequency in Hz, I)` pairs,
    /// one per point; see [`AcPoint::branch_current`]. Empty when `id` has
    /// no branch current.
    ///
    /// ```
    /// use sindr::{solve_ac, AcConfig, Circuit, CircuitElement};
    /// use sindr::element::{Capacitor, Resistor, VoltageSource};
    ///
    /// // The input current of an RC low-pass: −V/(R + 1/(jωC)), negative
    /// // because V1 delivers it.
    /// let circuit = Circuit::new("0", vec![
    ///     CircuitElement::VoltageSource(VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 0.0, waveform: None }),
    ///     CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()], resistance: 1e3 }),
    ///     CircuitElement::Capacitor(Capacitor { id: "C1".into(), nodes: ["out".into(), "0".into()], capacitance: 1e-6 }),
    /// ]);
    /// let ac = solve_ac(&circuit, &AcConfig::new("V1", 1.0, 1e5, 51))?;
    ///
    /// // Far above the corner, C is a short: −1 V / 1 kΩ.
    /// let (f, i) = *ac.branch_current_curve("V1").last().unwrap();
    /// assert_eq!(f, 1e5);
    /// assert!((i.re + 1e-3).abs() < 1e-6 && i.im.abs() < 2e-6);
    /// assert!(ac.branch_current_curve("R1").is_empty());
    /// # Ok::<(), sindr::SimError>(())
    /// ```
    ///
    pub fn branch_current_curve(&self, id: impl AsRef<str>) -> Vec<(f64, Complex64)> {
        let id = id.as_ref();
        self.points
            .iter()
            .filter_map(|p| p.branch_current(id).map(|i| (p.frequency, i)))
            .collect()
    }

    /// A node's voltage magnitude as `(frequency in Hz, |V| in V)` pairs,
    /// one per point; [`AcResult::gain_curve`] has it in dB.
    pub fn magnitude_curve(&self, node: impl AsRef<str>) -> Vec<(f64, f64)> {
        let node = node.as_ref();
        self.points
            .iter()
            .filter_map(|p| p.magnitude(node).map(|m| (p.frequency, m)))
            .collect()
    }

    /// The gain curve for a node as `(frequency in Hz, gain in dB)` pairs,
    /// one per point, using [`AcConfig::ac_magnitude`].
    pub fn gain_curve(&self, node: impl AsRef<str>) -> Vec<(f64, f64)> {
        let node = node.as_ref();
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
    pub fn phase_curve(&self, node: impl AsRef<str>) -> Vec<(f64, f64)> {
        let node = node.as_ref();
        self.points
            .iter()
            .filter_map(|p| p.phase_deg(node).map(|ph| (p.frequency, ph)))
            .collect()
    }
}

/// A circuit linearised at its DC bias point, ready to stamp at any
/// frequency: the nonlinear elements' conductances there, and where each
/// element's terminals and branch rows are, resolved once for every
/// frequency of a sweep.
pub(crate) struct SmallSignalModel {
    conductances: Vec<Conductance>,
    layout: Layout,
    terminals: Terminals,
    /// Branch row of each independent voltage source, by id.
    vsource_branches: HashMap<String, usize>,
}

/// Linearises `circuit`'s nonlinear elements at the DC operating point.
pub(crate) fn linearize_at_dc(
    circuit: &Circuit,
    node_map: &NodeMap,
    dc_solution: &nalgebra::DVector<f64>,
) -> SmallSignalModel {
    let terminals = Terminals::new(circuit, node_map);
    let conductances = (0..circuit.components.len())
        .flat_map(|k| {
            circuit.components[k]
                .device()
                .linearise(&AcBias::new(&terminals, k, dc_solution))
        })
        .collect();
    let layout = Layout::new(circuit);
    let vsource_branches = circuit
        .components
        .iter()
        .enumerate()
        .filter_map(|(k, c)| match c {
            CircuitElement::VoltageSource(VoltageSource { id, .. }) => {
                Some((id.clone(), node_map.num_nodes() + layout.branch(k)))
            }
            _ => None,
        })
        .collect();
    SmallSignalModel {
        conductances,
        layout,
        terminals,
        vsource_branches,
    }
}

/// [`validate_config`], and that `config` drives a voltage source of flat
/// `circuit`.
pub(crate) fn validate_stimulus(circuit: &Circuit, config: &AcConfig) -> Result<(), SimError> {
    validate_config(config)?;
    let drives_a_voltage_source = circuit
        .components
        .iter()
        .any(|c| matches!(c, CircuitElement::VoltageSource(VoltageSource { id, .. }) if *id == config.source_id));
    if !drives_a_voltage_source {
        return Err(SimError::config(
            ConfigKind::Ac,
            format!(
                "source_id '{}' is not a voltage source in the circuit",
                config.source_id
            ),
        ));
    }
    Ok(())
}

/// Reject sweep configs that would underflow, divide by zero, or feed
/// `log10` a non-positive value.
pub(crate) fn validate_config(config: &AcConfig) -> Result<(), SimError> {
    let invalid = |msg: String| Err(SimError::config(ConfigKind::Ac, msg));

    if config.num_points == 0 {
        return invalid("num_points must be at least 1".into());
    }
    crate::validation::check_sweep_points(ConfigKind::Ac, "num_points", config.num_points)?;
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
/// (the DC equivalent shorts it with a 0 V source of the same id), and each
/// saturating transformer by the linear pair whose inductance matrix is its
/// incremental one at the DC winding currents (`id` and `id#2`).
pub(crate) fn incremental_inductors(circuit: &Circuit, dc: &crate::SimulationResult) -> Circuit {
    let mut out = circuit.clone();
    for element in &mut out.components {
        if let CircuitElement::Inductor(Inductor {
            id,
            inductance,
            saturation,
            ..
        }) = element
        {
            if let Some(sat) = saturation.take() {
                let i = dc.branch_currents.get(id.as_str()).copied().unwrap_or(0.0);
                *inductance = sindr_devices::inductor::incremental_inductance(i, *inductance, &sat);
            }
        }
        if let CircuitElement::Transformer(Transformer {
            id,
            l1,
            l2,
            k,
            saturation,
            ..
        }) = element
        {
            if let Some(sat) = saturation.take() {
                let current = |w: &str| dc.branch_currents.get(w).copied().unwrap_or(0.0);
                let i = [current(id), current(&format!("{id}#2"))];
                let (_, l) = crate::element::transformer::transformer_flux(*l1, *l2, *k, &sat, i);
                (*l1, *l2) = (l[0][0], l[1][1]);
                *k = l[0][1] / (l[0][0] * l[1][1]).sqrt();
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
    // A regulator's piece (linear or switching) depends on its output current, its branch row.
    for element in &circuit.components {
        if let CircuitElement::VoltageRegulator(VoltageRegulator { id, .. })
        | CircuitElement::SwitchingRegulator(SwitchingRegulator { id, .. }) = element
        {
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
pub(crate) struct SmallSignalSystem<'m> {
    a: MnaMatrix<Complex64>,
    /// Branch row of each independent voltage source, by id.
    vsource_branches: &'m HashMap<String, usize>,
    /// `a` factorised on the first solve, for every later one (an adjoint
    /// included) to reuse.
    lu: OnceCell<Factor<'static, Complex64>>,
}

impl SmallSignalSystem<'_> {
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
        self.factor()?.solve(b)
    }

    /// Solves `Aᵀ x = b`: the adjoint system, from the same factor.
    pub(crate) fn solve_transpose(
        &self,
        b: &DVector<Complex64>,
    ) -> Result<DVector<Complex64>, SimError> {
        self.factor()?.solve_transpose(b)
    }

    fn factor(&self) -> Result<&Factor<'static, Complex64>, SimError> {
        if let Some(lu) = self.lu.get() {
            return Ok(lu);
        }
        let lu = self.a.factor()?.into_owned();
        Ok(self.lu.get_or_init(|| lu))
    }
}

/// Builds the small-signal system of `circuit` at `omega` (rad/s),
/// linearised around `dc_solution` through `ss_models`.
pub(crate) fn small_signal_system<'m>(
    circuit: &Circuit,
    node_map: &NodeMap,
    dc_solution: &DVector<f64>,
    ss_models: &'m SmallSignalModel,
    omega: f64,
) -> SmallSignalSystem<'m> {
    build_small_signal(
        circuit,
        node_map,
        dc_solution,
        ss_models,
        Complex64::new(0.0, omega),
        false,
    )
}

/// The linearised system as the matrix pencil `G + s·C`, for pole-zero
/// analysis. Elements whose admittance isn't affine in `s` (an inductor's
/// `1/(sL)`, a battery's `R1‖C1`, a diode's charge behind its `RS`) get
/// extra unknowns after the node and branch rows (an inductor's current,
/// an internal node), so every entry is `g + s·c`.
pub(crate) struct SmallSignalPencil {
    /// The `s⁰` part.
    pub g: DMatrix<f64>,
    /// The `s¹` part.
    pub c: DMatrix<f64>,
    /// Branch row of each independent voltage source, by id.
    pub vsource_branches: HashMap<String, usize>,
}

/// The pencil `G + s·C` of `circuit`, linearised around `dc_solution`
/// through `ss_models`.
///
/// # Errors
///
/// [`SimError::InvalidConfig`] if the system isn't affine in `s`, which
/// would mean an element's stamp has no pencil form.
pub(crate) fn small_signal_pencil(
    circuit: &Circuit,
    node_map: &NodeMap,
    dc_solution: &DVector<f64>,
    ss_models: &SmallSignalModel,
) -> Result<SmallSignalPencil, SimError> {
    let at = |s: Complex64| build_small_signal(circuit, node_map, dc_solution, ss_models, s, true);
    // At s = j every stamp g + s·c is (g, c) with no rounding, which
    // subtracting A(0) from A(1) would not give for a small c beside a
    // large g.
    let a1 = at(Complex64::new(0.0, 1.0)).a.to_dense();
    let g = a1.map(|z| z.re);
    let c = a1.map(|z| z.im);
    // Every stamp is affine in s, so A(0) = G and A(2j) = G + 2j·C.
    let a0 = at(Complex64::new(0.0, 0.0)).a.to_dense();
    let a2 = at(Complex64::new(0.0, 2.0)).a.to_dense();
    let affine = (0..g.len()).all(|k| {
        let (g, c) = (g[k], c[k]);
        let tol = 1e-12 * (g.abs() + 2.0 * c.abs());
        (a0[k] - g).norm() <= tol && (a2[k] - Complex64::new(g, 2.0 * c)).norm() <= tol
    });
    if !affine {
        return Err(SimError::config(
            ConfigKind::PoleZero,
            "the linearised circuit isn't of the form G + sC",
        ));
    }
    Ok(SmallSignalPencil {
        g,
        c,
        vsource_branches: ss_models.vsource_branches.clone(),
    })
}

/// The small-signal system at complex frequency `s`. With `pencil`, the
/// stamps that aren't affine in `s` take extra unknowns instead (see
/// [`SmallSignalPencil`]); without it the system is the plain node and
/// branch one AC solves.
fn build_small_signal<'m>(
    circuit: &Circuit,
    node_map: &NodeMap,
    dc_solution: &DVector<f64>,
    ss_models: &'m SmallSignalModel,
    s: Complex64,
    pencil: bool,
) -> SmallSignalSystem<'m> {
    let num_nodes = node_map.num_nodes();
    let num_vsources = circuit.count_voltage_sources();
    let extra: usize = if pencil {
        circuit
            .components
            .iter()
            .map(|c| c.device().pencil_rows())
            .sum()
    } else {
        0
    };
    let mut a = MnaMatrix::zeros(num_nodes + num_vsources + extra);
    // The next free extra unknown of the pencil form.
    let mut aux = num_nodes + num_vsources;
    debug_assert_eq!(ss_models.layout, Layout::new(circuit));
    for (k, component) in circuit.components.iter().enumerate() {
        let mut stamp = AcStamp::new(
            &mut a,
            circuit,
            node_map,
            &ss_models.layout,
            &ss_models.terminals,
            k,
            dc_solution,
            s,
            pencil,
            &mut aux,
        );
        component.device().stamp_ac(&mut stamp);
    }

    for &(row, col, val) in &ss_models.conductances {
        stamp_complex_conductance(&mut a, row, col, Complex64::new(val, 0.0));
    }

    SmallSignalSystem {
        a,
        vsource_branches: &ss_models.vsource_branches,
        lu: OnceCell::new(),
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
/// - [`SimError::InvalidConfig`] if `config` is invalid: zero points,
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
/// use sindr::element::{Capacitor, Resistor, VoltageSource};
///
/// // Simple RC low-pass: V1 -> R1 -> n_out -> C1 -> gnd
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["n_in".into(), "0".into()],
///             voltage: 0.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R1".into(),
///             nodes: ["n_in".into(), "n_out".into()],
///             resistance: 1_000.0,
///         }),
///         CircuitElement::Capacitor(Capacitor {
///             id: "C1".into(),
///             nodes: ["n_out".into(), "0".into()],
///             capacitance: 1e-6,
///         }),
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
///
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
    validate_stimulus(circuit, config)?;

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

        // The same branch currents as DC's `branch_currents`: each element's
        // first branch row, when it exposes one.
        let branch_currents = circuit
            .components
            .iter()
            .enumerate()
            .filter(|(_, c)| c.device().exposes_branch_current())
            .map(|(k, c)| {
                let row = num_nodes + ss_models.layout.branch(k);
                (c.device().id().to_string(), solution[row])
            })
            .collect();

        points.push(AcPoint {
            frequency: freq,
            node_voltages,
            branch_currents,
        });
    }

    crate::composite::strip_nodes(
        original,
        points
            .iter_mut()
            .flat_map(|p| [&mut p.node_voltages, &mut p.branch_currents]),
    );
    Ok(AcResult {
        points,
        config: config.clone(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::{
        Bjt, Capacitor, Diode, Jfet, OpAmp, Photodiode, Resistor, SchottkyDiode, Varactor,
        ZenerDiode,
    };
    use approx::assert_abs_diff_eq;
    use sindr_devices::bjt::BjtKind;

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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                }),
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                }),
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 1.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: ["n2".into(), "0".into()],
                    capacitance: 1e-6,
                }),
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
                CircuitElement::VoltageSource(VoltageSource {
                    id: "VIN".into(),
                    nodes: ["in".into(), "0".into()],
                    voltage: vin,
                    waveform: None,
                }),
                CircuitElement::VoltageSource(VoltageSource {
                    id: "VCC".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: vcc,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "RB".into(),
                    nodes: ["in".into(), "b".into()],
                    resistance: 10_000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "RC".into(),
                    nodes: ["c".into(), rc_to.into()],
                    resistance: 1_000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "RE".into(),
                    nodes: ["e".into(), re_to.into()],
                    resistance: 100.0,
                }),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "e".into()],
                    kind,
                    bf: 100.0,
                    temperature,
                    parasitic_caps: None,
                    model: None,
                }),
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
        CircuitElement::VoltageSource(VoltageSource {
            id: id.into(),
            nodes: [p.into(), "0".into()],
            voltage: v,
            waveform: None,
        })
    }

    fn res(id: &str, a: &str, b: &str, r: f64) -> CircuitElement {
        CircuitElement::Resistor(Resistor {
            id: id.into(),
            nodes: [a.into(), b.into()],
            resistance: r,
        })
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
                    CircuitElement::Jfet(Jfet {
                        id: "J1".into(),
                        nodes: ["g".into(), "d".into(), "s".into()],
                        kind,
                        idss: 10e-3,
                        vp,
                        parasitic_caps: None,
                    }),
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
        let zener = CircuitElement::ZenerDiode(ZenerDiode {
            id: "D1".into(),
            nodes: ["0".into(), "out".into()],
            vz: 5.1,
            temperature: 300.15,
        });
        assert_ac_matches_dc_slope("zener in breakdown", behind(zener, 1_000.0), 10.0, "out");

        let schottky = CircuitElement::SchottkyDiode(SchottkyDiode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
        });
        assert_ac_matches_dc_slope("schottky", behind(schottky, 1_000.0), 2.0, "out");

        let lit = CircuitElement::Photodiode(Photodiode {
            id: "D1".into(),
            nodes: ["0".into(), "out".into()],
            irradiance: 1e-4,
            temperature: 300.15,
        });
        assert_ac_matches_dc_slope("photodiode, reverse", behind(lit, 10_000.0), 5.0, "out");

        let forward = CircuitElement::Photodiode(Photodiode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            irradiance: 0.0,
            temperature: 300.15,
        });
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
            CircuitElement::Diode(Diode {
                id: "D1".into(),
                nodes: ["0".into(), "out".into()],
                temperature: 300.15,
                model: Some(crate::circuit::DiodeModel {
                    cjo,
                    ..Default::default()
                }),
            }),
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
            CircuitElement::Inductor(Inductor {
                id: "L1".into(),
                nodes: ["out".into(), "0".into()],
                inductance: l,
                saturation: None,
            }),
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
            CircuitElement::OpAmp(OpAmp {
                id: "U1".into(),
                nodes: ["0".into(), "fb".into(), "out".into()],
                v_pos: 15.0,
                v_neg: -15.0,
                model: Some(model),
            }),
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
            CircuitElement::Varactor(Varactor {
                id: "D1".into(),
                nodes: ["0".into(), "out".into()],
                params,
            }),
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
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: None,
                }),
            ];
            if reactive {
                c.push(CircuitElement::Capacitor(Capacitor {
                    id: "CIN".into(),
                    nodes: ["in".into(), "b".into()],
                    capacitance: 10e-6,
                }));
                c.push(CircuitElement::Inductor(Inductor {
                    id: "L1".into(),
                    nodes: ["vcc".into(), "x".into()],
                    inductance: 1e-3,
                    saturation: None,
                }));
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

    /// The source of an RC low-pass carries `−V/Z` with `Z = R + 1/(jωC)`:
    /// negative, as in DC, since it delivers the current.
    #[test]
    fn source_branch_current_is_minus_v_over_z() {
        let (r, c) = (1e3, 1e-6);
        let circuit = with(vec![
            vsrc("VIN", "in", 0.0),
            res("R1", "in", "out", r),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: c,
            }),
        ]);
        let ac = solve_ac(&circuit, &AcConfig::new("VIN", 10.0, 1e4, 5)).unwrap();
        let curve = ac.branch_current_curve("VIN");
        assert_eq!(curve.len(), 5);
        for (point, &(f, i)) in ac.points.iter().zip(&curve) {
            let omega = 2.0 * std::f64::consts::PI * f;
            let z = Complex64::new(r, -1.0 / (omega * c));
            let expected = -Complex64::new(1.0, 0.0) / z;
            assert!(
                (i - expected).norm() < 1e-12 * expected.norm(),
                "{f} Hz: {i} vs {expected}"
            );
            assert_eq!(point.branch_current("VIN"), Some(i));
        }
        // Like DC's: only elements with a current unknown have one.
        assert_eq!(ac.points[0].branch_current("R1"), None);
        assert_eq!(ac.points[0].branch_current("C1"), None);
        assert!(ac.branch_current_curve("nope").is_empty());
    }

    /// An op-amp's branch current is the current into its output pin: the
    /// opposite of what it drives into RL and RF.
    #[test]
    fn op_amp_branch_current_balances_its_output_node() {
        let (r1, rf, rl) = (1e3, 10e3, 2e3);
        let circuit = with(vec![
            vsrc("VIN", "in", 0.0),
            res("R1", "in", "fb", r1),
            res("RF", "fb", "out", rf),
            res("RL", "out", "0", rl),
            CircuitElement::OpAmp(OpAmp {
                id: "U1".into(),
                nodes: ["0".into(), "fb".into(), "out".into()],
                v_pos: 15.0,
                v_neg: -15.0,
                model: Some(crate::circuit::OpAmpModel::default()),
            }),
        ]);
        let config = AcConfig::new("VIN", 1e3, 1e3, 1).with_spacing(FrequencySpacing::Linear);
        let point = &solve_ac(&circuit, &config).unwrap().points[0];
        let (out, fb) = (point.node_voltages["out"], point.node_voltages["fb"]);
        let driven = out / rl + (out - fb) / rf;
        let i = point.branch_current("U1").unwrap();
        assert!(
            (i + driven).norm() < 1e-9 * driven.norm(),
            "{i} vs {}",
            -driven
        );
    }

    /// A composite's parts stay inside it: a DC motor's sense source and
    /// back-EMF have branch rows, but no `M1#…` key comes out.
    #[test]
    fn composite_parts_have_no_branch_current() {
        let circuit = with(vec![
            vsrc("VIN", "in", 0.0),
            CircuitElement::DcMotor(crate::element::DcMotor {
                id: "M1".into(),
                nodes: ["in".into(), "0".into()],
                params: crate::element::DcMotorParams::new(2.0, 0.02),
            }),
        ]);
        let config = AcConfig::new("VIN", 1e3, 1e3, 1).with_spacing(FrequencySpacing::Linear);
        let point = &solve_ac(&circuit, &config).unwrap().points[0];
        let mut ids: Vec<&str> = point.branch_currents.keys().map(String::as_str).collect();
        ids.sort_unstable();
        assert_eq!(ids, ["VIN"]);
    }
}
