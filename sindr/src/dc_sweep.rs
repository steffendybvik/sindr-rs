//! DC parameter sweep module.
//!
//! Sweeps a voltage or current source across a range and collects operating
//! point results at each step ([`dc_sweep`]), or two sources, one inside the
//! other ([`dc_sweep_nested`]). Used for generating I-V curves, families of
//! device curves and transfer characteristics.

use crate::element::{CurrentSource, VoltageSource};
use crate::handle::ComponentId;
use std::collections::HashMap;

use crate::circuit::{Circuit, CircuitElement};
use crate::error::{ConfigKind, SimError};
use crate::results::SimulationResult;
use crate::solver::SolverOptions;

/// One operating point of a DC sweep: the sweep value and the full result
/// at that value.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct DcSweepPoint {
    /// The value of the swept source at this point (V, or A for a current
    /// source).
    pub sweep_value: f64,
    /// Full simulation result at this sweep value.
    pub result: SimulationResult,
}

/// Result of a DC sweep — one [`DcSweepPoint`] per swept value.
///
/// Use [`DcSweepResult::node_voltage_curve`] or
/// [`DcSweepResult::component_current_curve`] to extract a plottable curve.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct DcSweepResult {
    /// Component id of the source that was swept.
    pub source_id: ComponentId,
    /// Operating points, in sweep order from `start` to `stop`.
    pub points: Vec<DcSweepPoint>,
}

impl DcSweepResult {
    /// `(sweep value in V, node voltage in V)` pairs for a node, one per
    /// point.
    pub fn node_voltage_curve(&self, node: impl AsRef<str>) -> Vec<(f64, f64)> {
        let node = node.as_ref();
        self.points
            .iter()
            .filter_map(|p| p.result.voltage(node).map(|v| (p.sweep_value, v)))
            .collect()
    }

    /// `(sweep value in V, current in A)` pairs for a component, one per
    /// point. The current is its [`ComponentResult::current_through`]: from
    /// `nodes[0]` to `nodes[1]` for a two-terminal part (the collector or
    /// drain current for a transistor).
    ///
    /// [`ComponentResult::current_through`]: crate::ComponentResult::current_through
    pub fn component_current_curve(&self, component_id: impl AsRef<str>) -> Vec<(f64, f64)> {
        let component_id = component_id.as_ref();
        self.points
            .iter()
            .filter_map(|p| {
                p.result
                    .component(component_id)
                    .map(|c| (p.sweep_value, c.current_through))
            })
            .collect()
    }
}

/// One axis of a DC sweep: a source and the values it takes, `num_points`
/// of them evenly spaced from `start` to `stop`, both included.
///
/// ```
/// use sindr::dc_sweep::DcSweepAxis;
///
/// // SPICE's `V1 0 1 0.1`: 0, 0.1, …, 1 V, eleven points.
/// let axis = DcSweepAxis::from_step("V1", 0.0, 1.0, 0.1).unwrap();
/// assert_eq!(axis.num_points, 11);
/// assert_eq!(axis.stop, 1.0);
/// ```
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct DcSweepAxis {
    /// Id of the [`VoltageSource`](CircuitElement::VoltageSource) or
    /// [`CurrentSource`](CircuitElement::CurrentSource) to sweep.
    pub source_id: ComponentId,
    /// First value (V or A).
    pub start: f64,
    /// Last value (V or A).
    pub stop: f64,
    /// Points including both ends. At least 2, or 1 when `start == stop`.
    pub num_points: usize,
}

impl DcSweepAxis {
    /// `num_points` values of `source_id` from `start` to `stop`.
    pub fn new(
        source_id: impl Into<ComponentId>,
        start: f64,
        stop: f64,
        num_points: usize,
    ) -> Self {
        Self {
            source_id: source_id.into(),
            start,
            stop,
            num_points,
        }
    }

    /// SPICE's `.dc` form: from `start` in steps of `step` for as long as
    /// it doesn't pass `stop`. When `step` doesn't divide the span, the last
    /// value is short of `stop`, as in SPICE. When `start == stop`, or the
    /// first step already passes `stop`, the axis is the one point `start`
    /// (with `stop` set to `start`).
    ///
    /// # Errors
    ///
    /// [`SimError::InvalidConfig`] for a value that isn't finite, a step
    /// that is zero (with `start != stop`) or points away from `stop`, or
    /// more than 10⁷ points.
    pub fn from_step(
        source_id: impl Into<String>,
        start: f64,
        stop: f64,
        step: f64,
    ) -> Result<Self, SimError> {
        let source_id = source_id.into();
        let invalid = |msg: String| Err(SimError::config(ConfigKind::DcSweep, msg));
        if !(start.is_finite() && stop.is_finite() && step.is_finite()) {
            return invalid(format!(
                "{source_id} from {start} to {stop} in steps of {step} must be finite"
            ));
        }
        if start == stop {
            return Ok(Self::new(source_id, start, stop, 1));
        }
        if step == 0.0 || (stop - start).signum() != step.signum() {
            return invalid(format!(
                "{source_id}'s step {step} doesn't move from {start} towards {stop}"
            ));
        }
        // A whisker of slack, so 0 to 1 in 0.1 steps lands on 1.
        let intervals = ((stop - start) / step * (1.0 + 1e-9)).floor();
        if intervals + 1.0 > crate::validation::MAX_SWEEP_POINTS as f64 {
            return invalid(format!(
                "{source_id} from {start} to {stop} in steps of {step} is more than the {} points allowed",
                crate::validation::MAX_SWEEP_POINTS
            ));
        }
        let n = intervals as usize;
        let last = if ((start + step * intervals) - stop).abs() <= 1e-9 * step.abs() {
            stop
        } else {
            start + step * intervals
        };
        Ok(Self::new(source_id, start, last, n + 1))
    }

    /// The `k`-th value.
    fn value(&self, k: usize) -> f64 {
        if self.num_points <= 1 {
            return self.start;
        }
        if k + 1 == self.num_points {
            return self.stop;
        }
        self.start + (self.stop - self.start) * k as f64 / (self.num_points - 1) as f64
    }

    /// Rejects an axis the sweep can't run: fewer than 2 points (1 is
    /// allowed when `start == stop`), more than 10⁷, values that aren't
    /// finite, or a source that isn't in `circuit`.
    fn check(&self, circuit: &Circuit) -> Result<(), SimError> {
        if self.num_points < 2 && !(self.num_points == 1 && self.start == self.stop) {
            return Err(SimError::config(
                ConfigKind::DcSweep,
                "requires at least 2 points, or 1 when start equals stop",
            ));
        }
        crate::validation::check_sweep_points(ConfigKind::DcSweep, "num_points", self.num_points)?;
        if !self.start.is_finite() || !self.stop.is_finite() {
            return Err(SimError::config(
                ConfigKind::DcSweep,
                format!(
                    "endpoints must be finite (got {} to {})",
                    self.start, self.stop
                ),
            ));
        }
        let found = circuit.components.iter().any(|c| match c {
            CircuitElement::VoltageSource(VoltageSource { id, .. })
            | CircuitElement::CurrentSource(CurrentSource { id, .. }) => *id == self.source_id,
            _ => false,
        });
        if !found {
            if !circuit
                .components
                .iter()
                .any(|c| c.id() == self.source_id.as_str())
            {
                return Err(SimError::UnknownComponent(self.source_id.to_string()));
            }
            return Err(SimError::config(
                ConfigKind::DcSweep,
                format!("'{}' is not a voltage or current source", self.source_id),
            ));
        }
        Ok(())
    }
}

/// `circuit` with source `id` set to `value` (V or A), its waveform
/// dropped: a swept source is its sweep value alone, as in SPICE.
fn set_source(circuit: &mut Circuit, id: &str, value: f64) {
    for component in &mut circuit.components {
        match component {
            CircuitElement::VoltageSource(VoltageSource {
                id: cid,
                voltage,
                waveform,
                ..
            }) if cid == id => {
                *voltage = value;
                *waveform = None;
            }
            CircuitElement::CurrentSource(CurrentSource {
                id: cid,
                current,
                waveform,
                ..
            }) if cid == id => {
                *current = value;
                *waveform = None;
            }
            _ => {}
        }
    }
}

/// One curve along `axis` of flat, validated `circuit`, each point an
/// operating point as SPICE's `.dc` solves it: on the
/// [`dc_equivalent`](crate::dc_equivalent), capacitors open and inductors
/// shorted. Each point's Newton–Raphson starts from the previous point's
/// node voltages, the first from `seed` when there is one, else from
/// `opts.nodeset`.
fn sweep_curve(
    circuit: &Circuit,
    axis: &DcSweepAxis,
    seed: Option<&HashMap<String, f64>>,
    opts: &SolverOptions,
) -> Result<DcSweepResult, SimError> {
    let mut work = crate::dc_equivalent(circuit);
    // Only a source value changes from point to point: the nodes and the
    // heated parts stay as they are.
    let at = |value| crate::SolvePoint::DcSweep {
        source: axis.source_id.to_string(),
        value,
    };
    let node_map = crate::node_map::NodeMap::from_circuit(&work);
    let heated =
        crate::thermal::Heated::from_circuit(&work).map_err(|e| e.at(at(axis.value(0))))?;
    let mut points: Vec<DcSweepPoint> = Vec::with_capacity(axis.num_points);
    let mut point_opts = opts.clone();
    if let Some(seed) = seed {
        point_opts.nodeset = seed.clone();
    }
    for k in 0..axis.num_points {
        let sweep_value = axis.value(k);
        set_source(&mut work, &axis.source_id, sweep_value);
        if let Some(previous) = points.last() {
            point_opts
                .nodeset
                .clone_from(&previous.result.node_voltages);
        }
        let solve = |dc: &Circuit| crate::solve_dc_circuit_on(dc, &node_map, &point_opts);
        let mut result = match &heated {
            Some(heated) => crate::thermal::solve_dc(&work, heated, solve),
            None => solve(&work),
        }
        .map_err(|e| e.at(at(sweep_value)))?;
        crate::strip_secondary_windings(&mut result);
        points.push(DcSweepPoint {
            sweep_value,
            result,
        });
    }
    Ok(DcSweepResult {
        source_id: axis.source_id.clone(),
        points,
    })
}

/// Sweeps a voltage or current source across a range and solves the
/// operating point at each step.
///
/// Used for generating I-V curves and transfer characteristics. The returned
/// [`DcSweepResult`] has helpers to pull out plottable curves for any node
/// voltage or component current. Each point's Newton–Raphson starts from
/// the previous point's solution, as in SPICE, so a curve follows one
/// branch of a circuit with several operating points.
///
/// # Arguments
///
/// - `circuit` — the base circuit (unchanged; each sweep point gets a
///   modified copy)
/// - `source_id` — id of the [`VoltageSource`](CircuitElement::VoltageSource)
///   or [`CurrentSource`](CircuitElement::CurrentSource) to sweep
/// - `start`, `stop` — sweep endpoints (V or A), inclusive
/// - `num_points` — total points including both endpoints: at least 2, or
///   1 when `start == stop` (one solve at `start`)
///
/// For two sources at once, as SPICE's `.dc V1 … V2 …`, see
/// [`dc_sweep_nested`].
///
/// # Errors
///
/// - [`SimError::UnknownComponent`] if no component has `source_id`
/// - [`SimError::InvalidConfig`] if `num_points < 2` (1 is allowed when
///   `start == stop`) or above 10⁷, `start` or `stop` is not finite, or
///   `source_id` isn't a voltage or current source
/// - Any [`SimError`] returned by [`solve_circuit`](crate::solve_circuit)
///   at one of the sweep points (propagated immediately)
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement, dc_sweep};
/// use sindr::dc_sweep::DcSweepAxis;
/// use sindr::element::{Resistor, VoltageSource};
///
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["n1".into(), "0".into()],
///             voltage: 0.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R1".into(),
///             nodes: ["n1".into(), "0".into()],
///             resistance: 1_000.0,
///         }),
///     ],
///     ..Default::default()
/// };
///
/// // Sweep V1 from 0 V to 10 V in 11 steps.
/// let sweep = dc_sweep(&circuit, &DcSweepAxis::new("V1", 0.0, 10.0, 11)).unwrap();
/// assert_eq!(sweep.points.len(), 11);
///
/// // I = V/R should rise linearly from 0 to 10 mA.
/// let curve = sweep.component_current_curve("R1");
/// assert!((curve.last().unwrap().1 - 0.010).abs() < 1e-9);
/// ```
///
pub fn dc_sweep(circuit: &Circuit, axis: &DcSweepAxis) -> Result<DcSweepResult, SimError> {
    dc_sweep_with(circuit, axis, &SolverOptions::default())
}

/// [`dc_sweep`] with `opts`.
pub(crate) fn dc_sweep_with(
    circuit: &Circuit,
    axis: &DcSweepAxis,
    opts: &SolverOptions,
) -> Result<DcSweepResult, SimError> {
    let original = circuit;
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    axis.check(circuit)?;
    crate::validation::validate_circuit(circuit)?;
    let mut sweep = sweep_curve(circuit, axis, None, opts)?;
    crate::composite::fold_all(original, sweep.points.iter_mut().map(|p| &mut p.result));
    Ok(sweep)
}

/// A two-source DC sweep: one [`DcSweepResult`] along the inner source per
/// value of the outer one.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct NestedDcSweepResult {
    /// Component id of the outer (slower) source.
    pub outer_source_id: ComponentId,
    /// One curve per outer value, in sweep order.
    pub curves: Vec<NestedDcSweepCurve>,
}

/// One curve of a [`NestedDcSweepResult`]: the inner sweep at one value of
/// the outer source.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct NestedDcSweepCurve {
    /// The outer source's value for this curve (V or A).
    pub outer_value: f64,
    /// The inner sweep at that value.
    pub sweep: DcSweepResult,
}

/// Sweeps two sources, `inner` fastest: for each value of `outer`, a full
/// sweep of `inner`. SPICE's `.dc SRC1 … SRC2 …`, with `SRC1` as `inner`.
/// This is how a family of device curves is drawn: a MOSFET's drain current
/// against V_DS for each V_GS, or a BJT's collector current against V_CE
/// for each base current.
///
/// Each point's Newton–Raphson starts from its neighbour along the inner
/// axis, and each curve's first point from the first point of the curve
/// before.
///
/// # Errors
///
/// [`SimError::UnknownComponent`] for a source id no component has;
/// [`SimError::InvalidConfig`] for an axis with fewer than 2 points (1 is
/// allowed when its `start` and `stop` are equal), values that aren't
/// finite, a source that isn't a voltage or current source in the circuit,
/// both axes on the same source, or more than 10⁷ points in all; and any
/// error from [`solve_circuit`](crate::solve_circuit) at a point.
///
/// # Example
///
/// ```
/// use sindr::{dc_sweep_nested, Circuit, CircuitElement};
/// use sindr::dc_sweep::DcSweepAxis;
/// use sindr::element::{Resistor, VoltageSource};
///
/// // Two sources into a 1 kΩ: the current is (V1 + V2) / 1 kΩ.
/// let circuit = Circuit::new(
///     "0",
///     vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["a".into(), "b".into()],
///             voltage: 0.0,
///             waveform: None,
///         }),
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V2".into(),
///             nodes: ["b".into(), "0".into()],
///             voltage: 0.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R1".into(),
///             nodes: ["a".into(), "0".into()],
///             resistance: 1e3,
///         }),
///     ],
/// );
/// let sweep = dc_sweep_nested(
///     &circuit,
///     &DcSweepAxis::new("V1", 0.0, 1.0, 3),
///     &DcSweepAxis::new("V2", 0.0, 2.0, 2),
/// )
/// .unwrap();
/// assert_eq!(sweep.curves.len(), 2);
/// let last = sweep.curves[1].sweep.component_current_curve("R1");
/// assert!((last[2].1 - 3e-3).abs() < 1e-12); // (1 V + 2 V) / 1 kΩ
/// ```
///
pub fn dc_sweep_nested(
    circuit: &Circuit,
    inner: &DcSweepAxis,
    outer: &DcSweepAxis,
) -> Result<NestedDcSweepResult, SimError> {
    dc_sweep_nested_with(circuit, inner, outer, &SolverOptions::default())
}

/// [`dc_sweep_nested`] with `opts`.
pub(crate) fn dc_sweep_nested_with(
    circuit: &Circuit,
    inner: &DcSweepAxis,
    outer: &DcSweepAxis,
    opts: &SolverOptions,
) -> Result<NestedDcSweepResult, SimError> {
    let original = circuit;
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    inner.check(circuit)?;
    outer.check(circuit)?;
    if inner.source_id == outer.source_id {
        return Err(SimError::config(
            ConfigKind::DcSweep,
            format!(
                "'{}' can't be both the inner and the outer source",
                inner.source_id
            ),
        ));
    }
    let total = inner.num_points.saturating_mul(outer.num_points);
    crate::validation::check_sweep_points(ConfigKind::DcSweep, "inner × outer points", total)?;
    crate::validation::validate_circuit(circuit)?;

    let mut work = circuit.clone();
    let mut curves: Vec<NestedDcSweepCurve> = Vec::with_capacity(outer.num_points);
    for k in 0..outer.num_points {
        let outer_value = outer.value(k);
        set_source(&mut work, &outer.source_id, outer_value);
        let seed = curves
            .last()
            .and_then(|c| c.sweep.points.first())
            .map(|p| p.result.node_voltages.clone());
        let sweep = sweep_curve(&work, inner, seed.as_ref(), opts)?;
        curves.push(NestedDcSweepCurve { outer_value, sweep });
    }
    let points = curves.iter_mut().flat_map(|c| &mut c.sweep.points);
    crate::composite::fold_all(original, points.map(|p| &mut p.result));
    Ok(NestedDcSweepResult {
        outer_source_id: outer.source_id.clone(),
        curves,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::CircuitElement;
    use crate::element::{Diode, Resistor};
    use crate::solver::Solver;
    use approx::assert_relative_eq;

    #[test]
    fn sweep_resistor_linear() {
        // V1 sweeps 0-10V, R1=1k to ground
        // At each point: V_n1 = V_sweep, I_R1 = V/R
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                }),
            ],
            ..Default::default()
        };

        let result = dc_sweep(&circuit, &DcSweepAxis::new("V1", 0.0, 10.0, 11)).unwrap();
        assert_eq!(result.points.len(), 11);

        // Check linearity
        for (i, point) in result.points.iter().enumerate() {
            let expected_v = i as f64;
            assert_relative_eq!(point.sweep_value, expected_v, epsilon = 1e-10);
            assert_relative_eq!(point.result.node_voltages["n1"], expected_v, epsilon = 1e-6);
        }
    }

    #[test]
    fn sweep_diode_iv() {
        // Sweep V1 from -1V to 1V through a diode
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["n2".into(), "0".into()],
                    temperature: 300.15,
                    model: None,
                }),
            ],
            ..Default::default()
        };

        let result = dc_sweep(&circuit, &DcSweepAxis::new("V1", -1.0, 1.0, 21)).unwrap();
        assert_eq!(result.points.len(), 21);

        // At negative sweep values, diode current should be ~0
        let neg_point = &result.points[0]; // -1V
        let d1_neg = neg_point
            .result
            .component_results
            .iter()
            .find(|c| c.id == "D1")
            .unwrap();
        assert!(d1_neg.current_through.abs() < 1e-6);

        // At positive sweep values, diode should conduct
        let pos_point = result.points.last().unwrap(); // +1V
        let d1_pos = pos_point
            .result
            .component_results
            .iter()
            .find(|c| c.id == "D1")
            .unwrap();
        assert!(d1_pos.current_through > 1e-3);
    }

    #[test]
    fn sweep_invalid_source() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1000.0,
            })],
            ..Default::default()
        };

        let result = dc_sweep(&circuit, &DcSweepAxis::new("V_nonexistent", 0.0, 10.0, 11));
        assert!(matches!(result, Err(SimError::UnknownComponent(id)) if id == "V_nonexistent"));

        let err = dc_sweep(&circuit, &DcSweepAxis::new("R1", 0.0, 10.0, 11)).unwrap_err();
        assert!(
            err.to_string()
                .contains("'R1' is not a voltage or current source"),
            "{err}"
        );
    }

    #[test]
    fn sweep_too_few_points() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 0.0,
                waveform: None,
            })],
            ..Default::default()
        };

        let result = dc_sweep(&circuit, &DcSweepAxis::new("V1", 0.0, 10.0, 1));
        assert!(result.is_err());
        let result = dc_sweep(&circuit, &DcSweepAxis::new("V1", 0.0, 0.0, 0));
        assert!(result.is_err());
    }

    fn divider() -> Circuit {
        Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                }),
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V2".into(),
                    nodes: ["n2".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
            ],
        )
    }

    #[test]
    fn sweep_one_point_when_start_equals_stop() {
        let axis = DcSweepAxis::from_step("V1", 5.0, 5.0, 1.0).unwrap();
        assert_eq!(axis.num_points, 1);
        let sweep = dc_sweep(&divider(), &axis).unwrap();
        assert_eq!(sweep.points.len(), 1);
        assert_eq!(sweep.points[0].sweep_value, 5.0);
        assert_relative_eq!(
            sweep.component_current_curve("R1")[0].1,
            5e-3,
            epsilon = 1e-12
        );

        let sweep = Solver::default().dc_sweep(&divider(), &axis).unwrap();
        assert_eq!(sweep.points.len(), 1);
    }

    #[test]
    fn sweep_one_point_when_step_exceeds_span() {
        // SPICE's `.dc V1 0 1 2`: the first step passes 1 V, so 0 V alone.
        let axis = DcSweepAxis::from_step("V1", 0.0, 1.0, 2.0).unwrap();
        assert_eq!((axis.num_points, axis.start, axis.stop), (1, 0.0, 0.0));
        let sweep = dc_sweep(&divider(), &axis).unwrap();
        assert_eq!(sweep.points.len(), 1);
        assert_eq!(sweep.points[0].sweep_value, 0.0);
    }

    #[test]
    fn nested_sweep_with_one_point_axes() {
        let one = DcSweepAxis::from_step("V2", 2.0, 2.0, 1.0).unwrap();
        let many = DcSweepAxis::new("V1", 0.0, 1.0, 3);
        let sweep = dc_sweep_nested(&divider(), &many, &one).unwrap();
        assert_eq!(sweep.curves.len(), 1);
        assert_eq!(sweep.curves[0].outer_value, 2.0);
        assert_eq!(sweep.curves[0].sweep.points.len(), 3);

        let inner = DcSweepAxis::from_step("V1", 1.0, 1.0, 0.1).unwrap();
        let outer = DcSweepAxis::new("V2", 0.0, 2.0, 3);
        let sweep = Solver::default()
            .dc_sweep_nested(&divider(), &inner, &outer)
            .unwrap();
        assert_eq!(sweep.curves.len(), 3);
        assert!(sweep.curves.iter().all(|c| c.sweep.points.len() == 1));
    }

    #[test]
    fn sweep_curve_extraction() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                }),
            ],
            ..Default::default()
        };

        let result = dc_sweep(&circuit, &DcSweepAxis::new("V1", 0.0, 5.0, 6)).unwrap();
        let curve = result.node_voltage_curve("n1");
        assert_eq!(curve.len(), 6);

        let i_curve = result.component_current_curve("R1");
        assert_eq!(i_curve.len(), 6);
        // At 5V, I = 5/1000 = 5mA
        assert_relative_eq!(i_curve.last().unwrap().1, 0.005, epsilon = 1e-6);
    }

    /// The sweep reuses one node map and one table of heated parts; each
    /// point must match a solve that builds its own, bit for bit.
    #[test]
    fn points_match_a_solve_built_per_point() {
        let mut circuit = Circuit::new(
            "0",
            vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 10.0,
                }),
                CircuitElement::Diode(Diode {
                    id: "D1".into(),
                    nodes: ["n2".into(), "0".into()],
                    temperature: 300.15,
                    model: None,
                }),
            ],
        );
        circuit.parts.insert(
            "D1".into(),
            crate::Part {
                thermal: Some(crate::Thermal::junction_to_ambient(200.0)),
                ..Default::default()
            },
        );
        let axis = DcSweepAxis::new("V1", 0.0, 2.0, 9);
        let sweep = dc_sweep(&circuit, &axis).unwrap();

        let mut work = crate::dc_equivalent(&circuit);
        let mut opts = SolverOptions::default();
        for (k, point) in sweep.points.iter().enumerate() {
            set_source(&mut work, "V1", axis.value(k));
            let want =
                crate::solve_dc_heated(&work, |dc| crate::solve_dc_circuit(dc, &opts)).unwrap();
            assert_eq!(point.result.node_voltages, want.node_voltages);
            assert_eq!(
                point.result.junction_temperatures,
                want.junction_temperatures
            );
            assert!(!want.junction_temperatures.is_empty());
            opts.nodeset.clone_from(&want.node_voltages);
        }
    }

    /// A heated part the sweep can't model fails the sweep before any
    /// point, with the error a per-point solve gives.
    #[test]
    fn a_bad_heated_part_fails_the_sweep() {
        let circuit = |thermal: crate::Thermal| {
            let mut c = Circuit::new(
                "0",
                vec![
                    CircuitElement::VoltageSource(VoltageSource {
                        id: "V1".into(),
                        nodes: ["n1".into(), "0".into()],
                        voltage: 0.0,
                        waveform: None,
                    }),
                    CircuitElement::Resistor(Resistor {
                        id: "R1".into(),
                        nodes: ["n1".into(), "0".into()],
                        resistance: 10.0,
                    }),
                ],
            );
            c.parts.insert(
                "R1".into(),
                crate::Part {
                    thermal: Some(thermal),
                    ..Default::default()
                },
            );
            c
        };
        let axis = DcSweepAxis::new("V1", 0.0, 1.0, 3);
        let no_rth = crate::Thermal {
            rth_ja: None,
            ..crate::Thermal::junction_to_ambient(1.0)
        };
        for thermal in [no_rth, crate::Thermal::junction_to_ambient(-1.0)] {
            let c = circuit(thermal);
            let err = dc_sweep(&c, &axis).unwrap_err();
            assert!(
                matches!(&err, SimError::InvalidComponent { id, .. } if id == "R1"),
                "{err:?}"
            );
            // The heated parts fail to build, so the solve is never reached.
            let solve = |dc: &Circuit| crate::solve_dc_circuit(dc, &SolverOptions::default());
            let want = crate::solve_dc_heated(&crate::dc_equivalent(&c), solve).unwrap_err();
            assert_eq!(err.to_string(), want.to_string());
        }
    }
}
