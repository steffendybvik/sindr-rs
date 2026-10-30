//! DC small-signal transfer function, as SPICE's `.tf`.
//!
//! Linearises the circuit at its DC operating point, then reports the DC
//! gain from an input source to an output, the resistance the input source
//! sees, and the resistance at the output.

use std::collections::HashMap;

use nalgebra::DVector;
use num_complex::Complex64;

use crate::ac_analysis::{bias_point, linearize_at_dc, small_signal_system, SmallSignalSystem};
use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::node_map::NodeMap;

/// What a transfer function is measured at.
#[derive(Debug, Clone, PartialEq)]
pub enum TfOutput {
    /// `V(node[,reference])`: the voltage of `node` against `reference`,
    /// or against ground when `reference` is `None`.
    Voltage {
        /// Output node.
        node: String,
        /// Reference node; ground when `None`.
        reference: Option<String>,
    },
    /// `I(source)`: the current through an independent voltage source,
    /// from its `nodes[0]` through the source to its `nodes[1]`.
    Current {
        /// Id of the voltage source the current runs through.
        source_id: String,
    },
}

/// Transfer-function analysis configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct TfConfig {
    /// The output the gain is measured at.
    pub output: TfOutput,
    /// Id of the input: an independent voltage or current source.
    pub input_source: String,
}

/// Result of a transfer-function analysis.
///
/// The gain's unit follows the input and output: V/V for a voltage in and
/// out, A/A for a current in and out, Ω or S for a mix.
#[derive(Debug, Clone, PartialEq)]
pub struct TfResult {
    /// Small-signal DC gain: d(output)/d(input).
    pub gain: f64,
    /// Resistance (Ω) the input source sees, looking into the circuit.
    /// Infinite when it draws no small-signal current.
    pub input_resistance: f64,
    /// Resistance (Ω) at the output with every independent source zeroed.
    /// For a voltage output it is the resistance between the output node
    /// and its reference; for a current output, the resistance in series
    /// with the measuring source.
    pub output_resistance: f64,
    /// Configuration used to produce this result.
    pub config: TfConfig,
}

/// Runs a DC small-signal transfer-function analysis, as SPICE's `.tf`.
///
/// # Algorithm
///
/// 1. Solve the DC operating point of `circuit` (capacitors open,
///    inductors shorted).
/// 2. Linearise every nonlinear element at that operating point.
/// 3. Excite the input source with a unit signal, everything else zero,
///    and read the output: that is the gain. The input source's own
///    response gives the input resistance.
/// 4. Drive a unit test current into the output (or a unit voltage on a
///    current output's source) and read the response: that is the output
///    resistance.
///
/// # Errors
///
/// - [`SimError::InvalidComponent`] if the input is not an independent
///   voltage or current source, the output node is not in the circuit, or
///   a current output does not name an independent voltage source
/// - Any [`SimError`] from the DC operating-point solve
/// - [`SimError::SingularMatrix`] if the linearised system is singular
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement};
/// use sindr::transfer_function::{solve_tf, TfConfig, TfOutput};
///
/// // A 1 kΩ / 3 kΩ divider: gain 3/4, Rin 4 kΩ, Rout 1k ‖ 3k = 750 Ω.
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "0".into()],
///             voltage: 1.0,
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
/// let config = TfConfig {
///     output: TfOutput::Voltage { node: "out".into(), reference: None },
///     input_source: "V1".into(),
/// };
/// let tf = solve_tf(&circuit, &config)?;
/// assert!((tf.gain - 0.75).abs() < 1e-9);
/// assert!((tf.input_resistance - 4_000.0).abs() < 1e-6);
/// assert!((tf.output_resistance - 750.0).abs() < 1e-6);
/// # Ok::<(), sindr::SimError>(())
/// ```
pub fn solve_tf(circuit: &Circuit, config: &TfConfig) -> Result<TfResult, SimError> {
    solve_tf_with_initial_voltages(circuit, config, &HashMap::new())
}

/// Like [`solve_tf`], seeding the Newton–Raphson solve for the DC bias
/// point with node voltages (V) keyed by node name, as SPICE's `.nodeset`.
/// Unknown node names are ignored.
///
/// # Errors
///
/// As [`solve_tf`].
pub fn solve_tf_with_initial_voltages(
    circuit: &Circuit,
    config: &TfConfig,
    initial_voltages: &HashMap<String, f64>,
) -> Result<TfResult, SimError> {
    let (node_map, dc_solution) = bias_point(circuit, initial_voltages)?;
    // The bias-point circuit, so an inductor is an exact short (a 0 V
    // source) rather than AC's large conductance at ω = 0.
    let dc = crate::dc_equivalent(circuit);
    let ss_models = linearize_at_dc(&dc, &node_map, &dc_solution);
    let system = small_signal_system(&dc, &node_map, &dc_solution, &ss_models, 0.0);

    let input = Port::input(circuit, &node_map, &system, &config.input_source)?;
    let output = Port::output(circuit, &node_map, &system, &config.output)?;

    // Unit input, every other source zero.
    let x = system.solve(&input.excitation(system.size()))?;
    let gain = output.response(&x);
    let input_resistance = input.resistance(&x);

    // Unit drive at the output port, every independent source zero.
    let x = system.solve(&output.excitation(system.size()))?;
    let output_resistance = output.resistance(&x);

    Ok(TfResult {
        gain,
        input_resistance,
        output_resistance,
        config: config.clone(),
    })
}

/// Branch row of the voltage source `id` of the original `circuit`. The
/// system is built on the DC equivalent, where an inductor is a 0 V source
/// too, and that isn't a source to name.
pub(crate) fn independent_vsource_branch(
    circuit: &Circuit,
    system: &SmallSignalSystem,
    id: &str,
) -> Option<usize> {
    circuit
        .components
        .iter()
        .any(|c| matches!(c, CircuitElement::VoltageSource { id: vid, .. } if vid == id))
        .then(|| system.vsource_branch(id))
        .flatten()
}

/// A port of the linearised system: either the branch of a voltage source
/// (driven with 1 V, read as a current) or a node pair (driven with 1 A,
/// read as a voltage).
pub(crate) enum Port {
    Branch(usize),
    /// Current goes in at `.0` and out at `.1`; `None` is ground.
    Nodes(Option<usize>, Option<usize>),
}

impl Port {
    fn input(
        circuit: &Circuit,
        node_map: &NodeMap,
        system: &SmallSignalSystem,
        id: &str,
    ) -> Result<Self, SimError> {
        if let Some(branch) = independent_vsource_branch(circuit, system, id) {
            return Ok(Port::Branch(branch));
        }
        circuit
            .components
            .iter()
            .find_map(|c| match c {
                // A current source drives current through itself from
                // nodes[0] to nodes[1], so into the circuit at nodes[1].
                CircuitElement::CurrentSource { id: cid, nodes, .. } if cid == id => Some(
                    Port::Nodes(node_map.index(&nodes[1]), node_map.index(&nodes[0])),
                ),
                _ => None,
            })
            .ok_or_else(|| {
                SimError::InvalidComponent(format!(
                    "transfer function: input `{id}` is not an independent voltage or current source"
                ))
            })
    }

    pub(crate) fn output(
        circuit: &Circuit,
        node_map: &NodeMap,
        system: &SmallSignalSystem,
        output: &TfOutput,
    ) -> Result<Self, SimError> {
        match output {
            TfOutput::Voltage { node, reference } => {
                let index = |name: &str| match node_map.index(name) {
                    None if name != circuit.ground_node => Err(SimError::InvalidComponent(
                        format!("transfer function: output node `{name}` is not in the circuit"),
                    )),
                    index => Ok(index),
                };
                let reference = match reference {
                    Some(r) => index(r)?,
                    None => None,
                };
                Ok(Port::Nodes(index(node)?, reference))
            }
            TfOutput::Current { source_id } => independent_vsource_branch(circuit, system, source_id)
                .map(Port::Branch)
                .ok_or_else(|| {
                    SimError::InvalidComponent(format!(
                        "transfer function: output current source `{source_id}` is not an independent voltage source"
                    ))
                }),
        }
    }

    pub(crate) fn excitation(&self, size: usize) -> DVector<Complex64> {
        let mut b = DVector::from_element(size, Complex64::new(0.0, 0.0));
        match *self {
            Port::Branch(k) => b[k] = Complex64::new(1.0, 0.0),
            Port::Nodes(into, out_of) => {
                if let Some(i) = into {
                    b[i] += 1.0;
                }
                if let Some(o) = out_of {
                    b[o] -= 1.0;
                }
            }
        }
        b
    }

    /// The port's own quantity in solution `x`: the branch current, or the
    /// voltage across the node pair.
    fn response(&self, x: &DVector<Complex64>) -> f64 {
        match *self {
            Port::Branch(k) => x[k].re,
            Port::Nodes(p, q) => {
                let v = |n: Option<usize>| n.map_or(0.0, |i| x[i].re);
                v(p) - v(q)
            }
        }
    }

    /// Resistance at the port when the solution `x` is its own unit drive.
    fn resistance(&self, x: &DVector<Complex64>) -> f64 {
        match self {
            // The branch current runs into the source's positive terminal,
            // so the current the source delivers to the circuit is its
            // negative.
            Port::Branch(_) => {
                let delivered = -self.response(x);
                if delivered == 0.0 {
                    f64::INFINITY
                } else {
                    1.0 / delivered
                }
            }
            Port::Nodes(..) => self.response(x),
        }
    }
}
