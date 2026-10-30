//! DC sensitivity analysis, as SPICE's `.sens`.
//!
//! The derivative of one DC output with respect to each component parameter,
//! at the operating point. With the linearised system `A` and the output
//! `y = cᵀx`, one adjoint solve `Aᵀλ = c` gives every parameter's derivative
//! as `dy/dp = λᵀ(∂b/∂p − (∂A/∂p)·x)`, so the cost doesn't grow with the
//! number of parameters.

use std::collections::HashMap;

use nalgebra::DVector;
use num_complex::Complex64;

use crate::ac_analysis::{bias_point, linearize_at_dc, small_signal_system};
use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::transfer_function::{independent_vsource_branch, Port, TfOutput};

/// Sensitivity-analysis configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct SensConfig {
    /// The output whose sensitivities are wanted.
    pub output: TfOutput,
}

/// Which parameter of a component a [`SensEntry`] is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensParameter {
    /// A resistor's resistance (Ω).
    Resistance,
    /// A voltage source's DC voltage (V).
    Voltage,
    /// A current source's DC current (A).
    Current,
    /// A VCVS's gain (V/V).
    VoltageGain,
    /// A VCCS's transconductance (S).
    Transconductance,
}

/// The sensitivity of the output to one component parameter.
#[derive(Debug, Clone, PartialEq)]
pub struct SensEntry {
    /// Id of the component.
    pub component_id: String,
    /// Which of its parameters.
    pub parameter: SensParameter,
    /// The parameter's value, in its own unit.
    pub value: f64,
    /// d(output)/d(parameter): the output's unit (V or A) per the
    /// parameter's unit.
    pub sensitivity: f64,
    /// Change in the output, in its own unit, for a 1 % rise in the
    /// parameter: `sensitivity · value / 100`.
    pub per_percent: f64,
}

/// Result of a sensitivity analysis.
#[derive(Debug, Clone, PartialEq)]
pub struct SensResult {
    /// One entry per supported parameter, in circuit order. Components with
    /// no entry: see [`solve_sensitivity`].
    pub entries: Vec<SensEntry>,
    /// Configuration used to produce this result.
    pub config: SensConfig,
}

impl SensResult {
    /// The entry for the component `id`.
    pub fn get(&self, id: &str) -> Option<&SensEntry> {
        self.entries.iter().find(|e| e.component_id == id)
    }
}

/// Runs a DC sensitivity analysis, as SPICE's `.sens`.
///
/// Reports resistors' resistance, voltage and current sources' DC value,
/// VCVS gain and VCCS transconductance. Other components stay in the
/// linearised circuit but get no entry: device-model parameters (IS, β, ...),
/// CCVS and CCCS gains, capacitors and inductors, which have no effect at DC
/// apart from the 1 TΩ leak a capacitor is given.
///
/// # Errors
///
/// - [`SimError::InvalidComponent`] if the output node is not in the circuit,
///   or a current output does not name an independent voltage source
/// - Any [`SimError`] from the DC operating-point solve
/// - [`SimError::SingularMatrix`] if the linearised system is singular
///
/// # Example
///
/// ```
/// use sindr::{solve_sensitivity, Circuit, CircuitElement, SensConfig, TfOutput};
///
/// // 4 V over 1 kΩ / 3 kΩ: V(out) = V1 · R2 / (R1 + R2) = 3 V.
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "0".into()],
///             voltage: 4.0,
///             waveform: None,
///         },
///         CircuitElement::Resistor {
///             id: "R1".into(),
///             nodes: ["in".into(), "out".into()],
///             resistance: 1_000.0,
///         },
///         CircuitElement::Resistor {
///             id: "R2".into(),
///             nodes: ["out".into(), "0".into()],
///             resistance: 3_000.0,
///         },
///     ],
/// };
///
/// let config = SensConfig {
///     output: TfOutput::Voltage { node: "out".into(), reference: None },
/// };
/// let sens = solve_sensitivity(&circuit, &config)?;
///
/// assert!((sens.get("V1").unwrap().sensitivity - 0.75).abs() < 1e-12);
/// // dV(out)/dR1 = −V1 · R2 / (R1 + R2)² = −0.75 mV/Ω
/// assert!((sens.get("R1").unwrap().sensitivity + 0.75e-3).abs() < 1e-12);
/// # Ok::<(), sindr::SimError>(())
/// ```
pub fn solve_sensitivity(circuit: &Circuit, config: &SensConfig) -> Result<SensResult, SimError> {
    solve_sensitivity_with_initial_voltages(circuit, config, &HashMap::new())
}

/// Like [`solve_sensitivity`], seeding the Newton–Raphson solve for the DC
/// bias point with node voltages (V) keyed by node name, as SPICE's
/// `.nodeset`. Unknown node names are ignored.
///
/// # Errors
///
/// As [`solve_sensitivity`].
pub fn solve_sensitivity_with_initial_voltages(
    circuit: &Circuit,
    config: &SensConfig,
    initial_voltages: &HashMap<String, f64>,
) -> Result<SensResult, SimError> {
    let (node_map, dc_solution) = bias_point(circuit, initial_voltages)?;
    let dc = crate::dc_equivalent(circuit);
    let ss_models = linearize_at_dc(&dc, &node_map, &dc_solution);
    let system = small_signal_system(&dc, &node_map, &dc_solution, &ss_models, 0.0);

    let output = Port::output(circuit, &node_map, &system, &config.output)?;
    let adjoint: DVector<Complex64> = system.solve_transpose(&output.excitation(system.size()))?;

    let node = |name: &String| node_map.index(name);
    let v = |n: Option<usize>| n.map_or(0.0, |i| dc_solution[i]);
    let lambda = |n: Option<usize>| n.map_or(0.0, |i| adjoint[i].re);

    let mut entries = Vec::new();
    for component in &circuit.components {
        let (id, parameter, value, sensitivity) = match component {
            CircuitElement::Resistor {
                id,
                nodes,
                resistance,
            } => {
                let (p, q) = (node(&nodes[0]), node(&nodes[1]));
                let dvolts = v(p) - v(q);
                let dlambda = lambda(p) - lambda(q);
                let s = dvolts * dlambda / (resistance * resistance);
                (id, SensParameter::Resistance, *resistance, s)
            }
            CircuitElement::VoltageSource { id, voltage, .. } => {
                let Some(branch) = independent_vsource_branch(circuit, &system, id) else {
                    continue;
                };
                (id, SensParameter::Voltage, *voltage, adjoint[branch].re)
            }
            CircuitElement::CurrentSource {
                id, nodes, current, ..
            } => {
                // Injects into nodes[1] and draws from nodes[0].
                let s = lambda(node(&nodes[1])) - lambda(node(&nodes[0]));
                (id, SensParameter::Current, *current, s)
            }
            CircuitElement::Vcvs {
                id,
                control_nodes,
                gain,
                ..
            } => {
                let Some(branch) = dc.vsource_branch_index(id) else {
                    continue;
                };
                let control = v(node(&control_nodes[0])) - v(node(&control_nodes[1]));
                let s = adjoint[node_map.num_nodes() + branch].re * control;
                (id, SensParameter::VoltageGain, *gain, s)
            }
            CircuitElement::Vccs {
                id,
                nodes,
                control_nodes,
                gm,
            } => {
                let control = v(node(&control_nodes[0])) - v(node(&control_nodes[1]));
                let s = (lambda(node(&nodes[0])) - lambda(node(&nodes[1]))) * control;
                (id, SensParameter::Transconductance, *gm, s)
            }
            _ => continue,
        };
        entries.push(SensEntry {
            component_id: id.clone(),
            parameter,
            value,
            sensitivity,
            per_percent: sensitivity * value / 100.0,
        });
    }

    Ok(SensResult {
        entries,
        config: config.clone(),
    })
}
