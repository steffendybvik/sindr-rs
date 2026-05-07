//! Temperature sweep analysis.
//!
//! Varies junction temperature across a range and solves the circuit at each
//! temperature point. IS is re-scaled using the SPICE formula at each step.
//!
//! Mirrors the dc_sweep.rs pattern exactly: clones the circuit, mutates the
//! temperature field on all IS-bearing components, and calls solve_circuit.

use sindr_devices::diode::temperature_scale_is;

use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::results::SimulationResult;

/// One temperature sweep data point.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct TempSweepPoint {
    /// Temperature for this point (Kelvin).
    pub temperature_kelvin: f64,
    /// Simulation result at this temperature.
    pub result: SimulationResult,
}

/// Result of a temperature sweep analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct TempSweepResult {
    pub temp_start: f64,
    pub temp_end: f64,
    pub points: Vec<TempSweepPoint>,
}

/// Reference temperature (K) — SPICE standard for IS at room temperature.
const T0: f64 = 300.15;

/// Scale the IS value for a given component temperature relative to T0.
///
/// - xti = 3.0 for BJTs (SPICE standard)
/// - xti = 2.0 for diodes (SPICE standard)
/// - eg  = 1.11 eV for silicon
fn scale_is(is_t0: f64, t: f64, xti: f64) -> f64 {
    temperature_scale_is(is_t0, t, T0, 1.11, xti)
}

/// Solve the circuit across a temperature range.
///
/// For each temperature step:
/// 1. Clone the circuit.
/// 2. Set `temperature` on all IS-bearing components (Bjt, Diode, Led, ZenerDiode,
///    SchottkyDiode, Photodiode).
/// 3. Solve the modified circuit.
///
/// `temperature` is the junction temperature, not the thermistor temperature.
/// T0 = 300.15 K (reference IS temperature).
///
/// # Arguments
/// - `circuit`:    Circuit to sweep (not modified).
/// - `temp_start`: Start temperature in Kelvin (e.g. 250.0).
/// - `temp_end`:   End temperature in Kelvin (e.g. 400.0).
/// - `num_steps`:  Number of evaluation points (must be >= 2).
pub fn temperature_sweep(
    circuit: &Circuit,
    temp_start: f64,
    temp_end: f64,
    num_steps: usize,
) -> Result<TempSweepResult, SimError> {
    if num_steps < 2 {
        return Err(SimError::InvalidComponent(
            "temperature_sweep: num_steps must be >= 2".into(),
        ));
    }

    let step_size = (temp_end - temp_start) / (num_steps as f64 - 1.0);
    let mut points = Vec::with_capacity(num_steps);

    for i in 0..num_steps {
        let t = temp_start + i as f64 * step_size;
        let mut sweep_circuit = circuit.clone();

        // Scale IS for all temperature-sensitive components
        for component in &mut sweep_circuit.components {
            match component {
                CircuitElement::Bjt { temperature, .. } => {
                    *temperature = t;
                }
                CircuitElement::Diode { temperature, .. } => {
                    *temperature = t;
                }
                CircuitElement::Led { temperature, .. } => {
                    *temperature = t;
                }
                CircuitElement::ZenerDiode { temperature, .. } => {
                    *temperature = t;
                }
                CircuitElement::SchottkyDiode { temperature, .. } => {
                    *temperature = t;
                }
                CircuitElement::Photodiode { temperature, .. } => {
                    *temperature = t;
                }
                _ => {}
            }
        }

        let result = crate::solve_circuit(&sweep_circuit)?;
        points.push(TempSweepPoint {
            temperature_kelvin: t,
            result,
        });
    }

    Ok(TempSweepResult {
        temp_start,
        temp_end,
        points,
    })
}

// Silence unused import warning: scale_is is reserved for future direct use
// (currently the scaling is done inside stamp.rs via the component temperature field)
#[allow(unused)]
fn _dummy_scale_is_use() -> f64 {
    scale_is(1e-14, 350.0, 3.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::Circuit;
    use crate::circuit::CircuitElement;
    use sindr_devices::bjt::BjtKind;

    /// A simple resistor circuit swept 250K → 350K with 11 steps returns 11 points.
    #[test]
    fn temp_sweep_returns_num_steps_points() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
        };

        let result = temperature_sweep(&circuit, 250.0, 350.0, 11);
        assert!(result.is_ok(), "temperature_sweep should succeed: {:?}", result.err());
        let result = result.unwrap();
        assert_eq!(result.points.len(), 11, "Expected 11 points");
        assert_eq!(result.points[0].temperature_kelvin, 250.0);
        assert_eq!(result.points[10].temperature_kelvin, 350.0);
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
                CircuitElement::VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::VoltageSource {
                    id: "Vbb".into(),
                    nodes: ["base_in".into(), "0".into()],
                    voltage: 0.7,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "Rb".into(),
                    nodes: ["base_in".into(), "base".into()],
                    resistance: 10_000.0,
                },
                CircuitElement::Resistor {
                    id: "Rc".into(),
                    nodes: ["vcc".into(), "collector".into()],
                    resistance: 1_000.0,
                },
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["base".into(), "collector".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15, // will be overridden in sweep
                    parasitic_caps: None,
                },
            ],
        };

        let result = temperature_sweep(&circuit, 250.0, 350.0, 3);
        assert!(result.is_ok(), "temp sweep BJT should succeed: {:?}", result.err());
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

        let ic_low = get_ic(&result.points[0]);  // 250K
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
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 1000.0,
                },
            ],
        };
        let result = temperature_sweep(&circuit, 300.0, 400.0, 1);
        assert!(result.is_err(), "Should error with num_steps < 2");
    }
}
