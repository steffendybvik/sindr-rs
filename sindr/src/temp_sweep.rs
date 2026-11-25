//! Temperature sweep analysis.
//!
//! Varies junction temperature across a range and solves the circuit at each
//! temperature point. IS is re-scaled using the SPICE formula at each step,
//! and the thermal voltage follows the temperature.
//!
//! Mirrors the dc_sweep.rs pattern exactly: clones the circuit, mutates the
//! temperature field on all IS-bearing components, and calls solve_circuit.

use crate::circuit::{Circuit, CircuitElement};
use crate::element::{Bjt, Diode, Led, Photodiode, SchottkyDiode, ZenerDiode};
use crate::error::{ConfigKind, SimError};
use crate::results::SimulationResult;

/// One temperature sweep data point.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct TempSweepPoint {
    /// Temperature for this point (Kelvin).
    pub temperature: f64,
    /// Simulation result at this temperature.
    pub result: SimulationResult,
}

/// Result of a temperature sweep analysis.
///
/// One operating-point solution per temperature step. Use the `points`
/// vector to iterate `(temperature, result)` pairs.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct TempSweepResult {
    /// Start of the swept temperature range (K).
    pub start: f64,
    /// End of the swept temperature range (K).
    pub stop: f64,
    /// Operating points, in order from `start` to `stop`.
    pub points: Vec<TempSweepPoint>,
}

impl TempSweepResult {
    /// `(temperature in K, node voltage in V)` pairs for a node, one per
    /// point. Empty when there's no such node.
    pub fn node_voltage_curve(&self, node: impl AsRef<str>) -> Vec<(f64, f64)> {
        let node = node.as_ref();
        self.points
            .iter()
            .filter_map(|p| p.result.voltage(node).map(|v| (p.temperature, v)))
            .collect()
    }

    /// `(temperature in K, current in A)` pairs for a component, one per
    /// point: its [`ComponentResult::current_through`], as
    /// [`DcSweepResult::component_current_curve`] reads it.
    ///
    /// [`ComponentResult::current_through`]: crate::ComponentResult::current_through
    /// [`DcSweepResult::component_current_curve`]: crate::DcSweepResult::component_current_curve
    pub fn component_current_curve(&self, component_id: impl AsRef<str>) -> Vec<(f64, f64)> {
        let component_id = component_id.as_ref();
        self.points
            .iter()
            .filter_map(|p| {
                p.result
                    .component(component_id)
                    .map(|c| (p.temperature, c.current_through))
            })
            .collect()
    }
}

/// A temperature sweep: `num_points` temperatures from `start` to `stop`
/// (K), both ends included.
///
/// ```
/// use sindr::temp_sweep::TempSweepConfig;
///
/// // −40 °C to 125 °C in 12 points.
/// let config = TempSweepConfig::new(233.15, 398.15, 12);
/// assert_eq!(config.num_points, 12);
/// ```
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct TempSweepConfig {
    /// First temperature (K).
    pub start: f64,
    /// Last temperature (K).
    pub stop: f64,
    /// Points including both ends. At least 2.
    pub num_points: usize,
}

impl TempSweepConfig {
    /// `num_points` temperatures from `start` to `stop` (K).
    pub fn new(start: f64, stop: f64, num_points: usize) -> Self {
        Self {
            start,
            stop,
            num_points,
        }
    }
}

/// Sweeps junction temperature across a range and solves the circuit at
/// each step.
///
/// At each temperature, the saturation current `IS` of every IS-bearing
/// component (BJT, diode, LED, Zener, Schottky, photodiode) is rescaled
/// using the SPICE temperature formula, and its thermal voltage set to kT/q,
/// before solving. The base circuit is not modified.
///
/// `temperature` here is junction temperature — the thermistor's own
/// temperature input is independent.
///
/// # Arguments
///
/// - `circuit` — circuit to sweep (cloned per point, not modified)
/// - `config` — the temperatures: endpoints in kelvin (e.g. 250.0 to
///   400.0) and the number of points including both
///
/// # Errors
///
/// - [`SimError::InvalidConfig`] if `num_points < 2` or above 10⁷, or either endpoint
///   is not finite or not above 0 K
/// - Any [`SimError`] from solving at one of the temperatures (propagated
///   immediately)
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement, temperature_sweep};
/// use sindr::temp_sweep::TempSweepConfig;
/// use sindr::element::{Diode, VoltageSource};
///
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["n1".into(), "0".into()],
///             voltage: 1.0,
///             waveform: None,
///         }),
///         CircuitElement::Diode(Diode {
///             id: "D1".into(),
///             nodes: ["n1".into(), "0".into()],
///             temperature: 300.15,
///             model: None,
///         }),
///     ],
///     ..Default::default()
/// };
///
/// // Sweep from 250 K (-23 °C) to 400 K (127 °C) in 4 steps.
/// let sweep = temperature_sweep(&circuit, &TempSweepConfig::new(250.0, 400.0, 4)).unwrap();
/// assert_eq!(sweep.points.len(), 4);
/// ```
///
pub fn temperature_sweep(
    circuit: &Circuit,
    config: &TempSweepConfig,
) -> Result<TempSweepResult, SimError> {
    temperature_sweep_with(circuit, config, &crate::SolverOptions::default())
}

/// [`temperature_sweep`] with `opts`.
pub(crate) fn temperature_sweep_with(
    circuit: &Circuit,
    config: &TempSweepConfig,
    opts: &crate::SolverOptions,
) -> Result<TempSweepResult, SimError> {
    let &TempSweepConfig {
        start: temp_start,
        stop: temp_end,
        num_points: num_steps,
    } = config;
    let original = circuit;
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    if num_steps < 2 {
        return Err(SimError::config(
            ConfigKind::TemperatureSweep,
            "num_points must be >= 2",
        ));
    }
    crate::validation::check_sweep_points(ConfigKind::TemperatureSweep, "num_points", num_steps)?;
    for t in [temp_start, temp_end] {
        if !t.is_finite() || t <= 0.0 {
            return Err(SimError::config(
                ConfigKind::TemperatureSweep,
                format!("temperatures must be finite and > 0 K (got {t})"),
            ));
        }
    }

    let step_size = (temp_end - temp_start) / (num_steps as f64 - 1.0);
    let mut points = Vec::with_capacity(num_steps);

    for i in 0..num_steps {
        let t = temp_start + i as f64 * step_size;
        let mut sweep_circuit = circuit.clone();

        // Scale IS for all temperature-sensitive components
        for component in &mut sweep_circuit.components {
            match component {
                CircuitElement::Bjt(Bjt { temperature, .. }) => {
                    *temperature = t;
                }
                CircuitElement::Diode(Diode { temperature, .. }) => {
                    *temperature = t;
                }
                CircuitElement::Led(Led { temperature, .. }) => {
                    *temperature = t;
                }
                CircuitElement::ZenerDiode(ZenerDiode { temperature, .. }) => {
                    *temperature = t;
                }
                CircuitElement::SchottkyDiode(SchottkyDiode { temperature, .. }) => {
                    *temperature = t;
                }
                CircuitElement::Photodiode(Photodiode { temperature, .. }) => {
                    *temperature = t;
                }
                _ => {}
            }
        }

        // A self-heated part sits in an ambient at the swept temperature.
        // The circuit is flat here: a block's parts, a variant's included,
        // are top-level parts under their instance path.
        let thermals = sweep_circuit
            .parts
            .values_mut()
            .filter_map(|p| p.thermal.as_mut());
        for thermal in thermals {
            thermal.ambient = t;
        }

        let result = crate::solve_circuit_with(&sweep_circuit, opts)
            .map_err(|e| e.at(crate::SolvePoint::Temperature { kelvin: t }))?;
        points.push(TempSweepPoint {
            temperature: t,
            result,
        });
    }

    for point in &mut points {
        crate::composite::fold(original, &mut point.result);
    }
    Ok(TempSweepResult {
        start: temp_start,
        stop: temp_end,
        points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::Circuit;
    use crate::circuit::CircuitElement;
    use crate::element::{Resistor, VoltageSource};
    use sindr_devices::bjt::BjtKind;

    /// A simple resistor circuit swept 250K → 350K with 11 steps returns 11 points.
    #[test]
    fn temp_sweep_returns_num_steps_points() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
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

        let result = temperature_sweep(&circuit, &TempSweepConfig::new(250.0, 350.0, 11));
        assert!(
            result.is_ok(),
            "temperature_sweep should succeed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        assert_eq!(result.points.len(), 11, "Expected 11 points");
        assert_eq!(result.points[0].temperature, 250.0);
        assert_eq!(result.points[10].temperature, 350.0);
    }

    /// Common-emitter NPN BJT: collector current at 350K should be greater than at 250K.
    ///
    /// Higher temperature → higher IS → larger Ic for fixed base drive.
    #[test]
    fn temp_sweep_bjt_ic_increases_with_temperature() {
        // Common-emitter: V_BB=0.7V base drive, V_CC=5V, Rc=1k, Rb=10k
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vbb".into(),
                    nodes: ["base_in".into(), "0".into()],
                    voltage: 0.7,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rb".into(),
                    nodes: ["base_in".into(), "base".into()],
                    resistance: 10_000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rc".into(),
                    nodes: ["vcc".into(), "collector".into()],
                    resistance: 1_000.0,
                }),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["base".into(), "collector".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15, // will be overridden in sweep
                    parasitic_caps: None,
                    model: None,
                }),
            ],
            ..Default::default()
        };

        let result = temperature_sweep(&circuit, &TempSweepConfig::new(250.0, 350.0, 3));
        assert!(
            result.is_ok(),
            "temp sweep BJT should succeed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        assert_eq!(result.points.len(), 3);

        // Extract Ic from BJT results at 250K and 350K
        let get_ic = |point: &TempSweepPoint| -> f64 {
            point
                .result
                .bjt_results
                .iter()
                .find(|b| b.id == "Q1")
                .map(|b| b.ic)
                .unwrap_or(0.0)
        };

        let ic_low = get_ic(&result.points[0]); // 250K
        let ic_high = get_ic(&result.points[2]); // 350K

        assert!(
            ic_high > ic_low,
            "BJT Ic should increase with temperature: ic_250K={:.6e}, ic_350K={:.6e}",
            ic_low,
            ic_high,
        );
    }

    /// temperature_sweep with num_steps=1 should return an error.
    #[test]
    fn temp_sweep_requires_at_least_2_steps() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
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
        let result = temperature_sweep(&circuit, &TempSweepConfig::new(300.0, 400.0, 1));
        assert!(result.is_err(), "Should error with num_steps < 2");
    }
}
