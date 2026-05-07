//! DC parameter sweep module.
//!
//! Sweeps a voltage source value across a range and collects operating point
//! results at each step. Used for generating I-V curves and transfer characteristics.

use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::results::SimulationResult;

/// One operating point of a DC sweep: the sweep value and the full result
/// at that value.
#[derive(Debug, Clone)]
pub struct DcSweepPoint {
    /// The voltage applied to the swept source at this point (V).
    pub sweep_value: f64,
    /// Full simulation result at this sweep value.
    pub result: SimulationResult,
}

/// Result of a DC sweep — one [`DcSweepPoint`] per swept value.
///
/// Use [`DcSweepResult::node_voltage_curve`] or
/// [`DcSweepResult::component_current_curve`] to extract a plottable curve.
#[derive(Debug, Clone)]
pub struct DcSweepResult {
    /// Component id of the source that was swept.
    pub source_id: String,
    /// Operating points, in sweep order from `start` to `stop`.
    pub points: Vec<DcSweepPoint>,
}

impl DcSweepResult {
    /// Extract a vector of (sweep_value, node_voltage) pairs for a given node.
    pub fn node_voltage_curve(&self, node: &str) -> Vec<(f64, f64)> {
        self.points
            .iter()
            .filter_map(|p| {
                p.result
                    .node_voltages
                    .get(node)
                    .map(|v| (p.sweep_value, *v))
            })
            .collect()
    }

    /// Extract a vector of (sweep_value, current) pairs for a given component.
    pub fn component_current_curve(&self, component_id: &str) -> Vec<(f64, f64)> {
        self.points
            .iter()
            .filter_map(|p| {
                p.result
                    .component_results
                    .iter()
                    .find(|c| c.id == component_id)
                    .map(|c| (p.sweep_value, c.current_through))
            })
            .collect()
    }
}

/// Sweeps a voltage source across a range and solves the operating point at
/// each step.
///
/// Used for generating I-V curves and transfer characteristics. The returned
/// [`DcSweepResult`] has helpers to pull out plottable curves for any node
/// voltage or component current.
///
/// # Arguments
///
/// - `circuit` — the base circuit (unchanged; each sweep point gets a
///   modified copy)
/// - `source_id` — id of the [`VoltageSource`](CircuitElement::VoltageSource)
///   to sweep
/// - `start`, `stop` — sweep endpoints (V), inclusive
/// - `num_points` — total points including both endpoints (minimum 2)
///
/// # Errors
///
/// - [`SimError::InvalidComponent`] if `num_points < 2` or `source_id`
///   doesn't match any voltage source in the circuit
/// - Any [`SimError`] returned by [`solve_circuit`](crate::solve_circuit)
///   at one of the sweep points (propagated immediately)
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement, dc_sweep};
///
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource {
///             id: "V1".into(),
///             nodes: ["n1".into(), "0".into()],
///             voltage: 0.0,
///             waveform: None,
///         },
///         CircuitElement::Resistor {
///             id: "R1".into(),
///             nodes: ["n1".into(), "0".into()],
///             resistance: 1_000.0,
///         },
///     ],
/// };
///
/// // Sweep V1 from 0 V to 10 V in 11 steps.
/// let sweep = dc_sweep(&circuit, "V1", 0.0, 10.0, 11).unwrap();
/// assert_eq!(sweep.points.len(), 11);
///
/// // I = V/R should rise linearly from 0 to 10 mA.
/// let curve = sweep.component_current_curve("R1");
/// assert!((curve.last().unwrap().1 - 0.010).abs() < 1e-9);
/// ```
pub fn dc_sweep(
    circuit: &Circuit,
    source_id: &str,
    start: f64,
    stop: f64,
    num_points: usize,
) -> Result<DcSweepResult, SimError> {
    if num_points < 2 {
        return Err(SimError::InvalidComponent(
            "DC sweep requires at least 2 points".to_string(),
        ));
    }

    // Verify source exists
    let source_exists = circuit
        .components
        .iter()
        .any(|c| matches!(c, CircuitElement::VoltageSource { id, .. } if id == source_id));
    if !source_exists {
        return Err(SimError::InvalidComponent(format!(
            "Voltage source '{}' not found in circuit",
            source_id
        )));
    }

    let step = (stop - start) / (num_points - 1) as f64;
    let mut points = Vec::with_capacity(num_points);

    for i in 0..num_points {
        let sweep_value = start + step * i as f64;

        // Create modified circuit with swept voltage
        let mut swept_circuit = circuit.clone();
        for component in &mut swept_circuit.components {
            if let CircuitElement::VoltageSource { id, voltage, .. } = component {
                if id == source_id {
                    *voltage = sweep_value;
                }
            }
        }

        let result = crate::solve_circuit(&swept_circuit)?;
        points.push(DcSweepPoint {
            sweep_value,
            result,
        });
    }

    Ok(DcSweepResult {
        source_id: source_id.to_string(),
        points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::CircuitElement;
    use approx::assert_relative_eq;

    #[test]
    fn sweep_resistor_linear() {
        // V1 sweeps 0-10V, R1=1k to ground
        // At each point: V_n1 = V_sweep, I_R1 = V/R
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
        };

        let result = dc_sweep(&circuit, "V1", 0.0, 10.0, 11).unwrap();
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 100.0,
                },
                CircuitElement::Diode {
                    id: "D1".into(),
                    nodes: ["n2".into(), "0".into()],
                    temperature: 300.15,
                },
            ],
        };

        let result = dc_sweep(&circuit, "V1", -1.0, 1.0, 21).unwrap();
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
            components: vec![CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1000.0,
            }],
        };

        let result = dc_sweep(&circuit, "V_nonexistent", 0.0, 10.0, 11);
        assert!(result.is_err());
    }

    #[test]
    fn sweep_too_few_points() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 0.0,
                waveform: None,
            }],
        };

        let result = dc_sweep(&circuit, "V1", 0.0, 10.0, 1);
        assert!(result.is_err());
    }

    #[test]
    fn sweep_curve_extraction() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
        };

        let result = dc_sweep(&circuit, "V1", 0.0, 5.0, 6).unwrap();
        let curve = result.node_voltage_curve("n1");
        assert_eq!(curve.len(), 6);

        let i_curve = result.component_current_curve("R1");
        assert_eq!(i_curve.len(), 6);
        // At 5V, I = 5/1000 = 5mA
        assert_relative_eq!(i_curve.last().unwrap().1, 0.005, epsilon = 1e-6);
    }
}
