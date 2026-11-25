//! DC sensitivity analysis, as SPICE's `.sens`.
//!
//! The derivative of one DC output with respect to each component parameter,
//! at the operating point. With the linearised system `A` and the output
//! `y = cᵀx`, one adjoint solve `Aᵀλ = c` gives every parameter's derivative
//! as `dy/dp = λᵀ(∂b/∂p − (∂A/∂p)·x)`, so the cost doesn't grow with the
//! number of parameters.
//!
//! [`solve_ac_sensitivity`] does the same for a small-signal output, one
//! adjoint solve per frequency.

pub(crate) mod ac;
mod device;

pub use ac::{solve_ac_sensitivity, AcSensConfig, AcSensEntry, AcSensPoint, AcSensResult};

use crate::element::{Cccs, Ccvs, CurrentSource, Resistor, Vccs, Vcvs, VoltageSource};
use crate::handle::ComponentId;
use nalgebra::DVector;
use num_complex::Complex64;

use crate::ac_analysis::{
    at_junction_temperatures, bias_point_with_result, linearize_at_dc, small_signal_system,
};
use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::transfer_function::{independent_vsource_branch, Port, TfOutput};

/// Sensitivity-analysis configuration. Build one with [`SensConfig::new`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct SensConfig {
    /// The output whose sensitivities are wanted.
    pub output: TfOutput,
}

impl SensConfig {
    /// The sensitivities of `output`, as SPICE's `.sens output`.
    ///
    /// ```
    /// use sindr::{SensConfig, TfOutput};
    ///
    /// // SPICE `.sens i(Vsense)`.
    /// let config = SensConfig::new(TfOutput::current("Vsense"));
    /// assert_eq!(config.output, TfOutput::Current { source_id: "Vsense".into() });
    /// ```
    pub fn new(output: TfOutput) -> Self {
        Self { output }
    }
}

/// Which parameter of a component a [`SensEntry`] is for.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
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
    /// A CCVS's transresistance `rm` (Ω).
    Transresistance,
    /// A CCCS's current gain `alpha` (A/A).
    CurrentGain,
    /// A capacitor's capacitance (F). AC only: a capacitor is open at DC.
    Capacitance,
    /// An inductor's inductance (H). AC only: an inductor is a short at DC.
    Inductance,
    /// The AC magnitude of the stimulus source (V). AC only.
    AcMagnitude,
    /// A diode's or BJT's saturation current `IS` (A), as given at 27 °C:
    /// the derivative carries IS's temperature scaling.
    SaturationCurrent,
    /// A diode's emission coefficient `N`. It also sets how IS follows
    /// temperature, which the derivative carries.
    EmissionCoefficient,
    /// A diode's series resistance `RS` (Ω).
    SeriesResistance,
    /// A diode's zero-bias junction capacitance `CJO` (F). AC only.
    JunctionCapacitance,
    /// A diode's transit time `TT` (s). AC only.
    TransitTime,
    /// A BJT's forward current gain `BF` (the element's `bf`).
    ForwardBeta,
    /// A BJT's reverse current gain `BR`.
    ReverseBeta,
    /// A BJT's forward emission coefficient `NF`.
    ForwardEmissionCoefficient,
    /// A BJT's reverse emission coefficient `NR`.
    ReverseEmissionCoefficient,
    /// A BJT's forward Early voltage `VAF` (V), reported only while the
    /// Early effect is on (`vaf` above 0).
    EarlyVoltage,
    /// A MOSFET's transconductance parameter `KP` (A/V²), sindr's `kp`
    /// (SPICE's `KP·W/L`).
    TransconductanceParameter,
    /// A MOSFET's threshold voltage `VTO` (V), SPICE-signed.
    ThresholdVoltage,
    /// A MOSFET's channel-length modulation `LAMBDA` (1/V).
    ChannelLengthModulation,
}

/// The sensitivity of the output to one component parameter.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct SensEntry {
    /// Id of the component.
    pub component_id: ComponentId,
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct SensResult {
    /// One entry per supported parameter, in circuit order. Components with
    /// no entry: see [`solve_sensitivity`].
    pub entries: Vec<SensEntry>,
    /// Configuration used to produce this result.
    pub config: SensConfig,
}

impl SensResult {
    /// The entry for the component `id`: its first, for a device with
    /// several parameters (see [`get_parameter`](Self::get_parameter)).
    pub fn get(&self, id: impl AsRef<str>) -> Option<&SensEntry> {
        let id = id.as_ref();
        self.entries.iter().find(|e| e.component_id == id)
    }

    /// The entry for `parameter` of the component `id`.
    ///
    /// ```
    /// use sindr::{solve_sensitivity, Circuit, CircuitElement, SensConfig, SensParameter, TfOutput};
    /// use sindr::element::{Diode, Resistor, VoltageSource};
    ///
    /// let circuit = Circuit {
    ///     ground_node: "0".into(),
    ///     components: vec![
    ///         CircuitElement::VoltageSource(VoltageSource {
    ///             id: "V1".into(),
    ///             nodes: ["in".into(), "0".into()],
    ///             voltage: 5.0,
    ///             waveform: None,
    ///         }),
    ///         CircuitElement::Resistor(Resistor {
    ///             id: "R1".into(),
    ///             nodes: ["in".into(), "a".into()],
    ///             resistance: 1_000.0,
    ///         }),
    ///         CircuitElement::Diode(Diode {
    ///             id: "D1".into(),
    ///             nodes: ["a".into(), "0".into()],
    ///             temperature: 300.15,
    ///             model: None,
    ///         }),
    ///     ],
    ///     ..Default::default()
    /// };
    /// let sens = solve_sensitivity(&circuit, &SensConfig::new(TfOutput::voltage("a")))?;
    /// // A larger IS lowers the forward drop at the same current.
    /// let is = sens.get_parameter("D1", SensParameter::SaturationCurrent).unwrap();
    /// assert!(is.sensitivity < 0.0);
    /// # Ok::<(), sindr::SimError>(())
    /// ```
    ///
    pub fn get_parameter(
        &self,
        id: impl AsRef<str>,
        parameter: SensParameter,
    ) -> Option<&SensEntry> {
        let id = id.as_ref();
        self.entries
            .iter()
            .find(|e| e.component_id == id && e.parameter == parameter)
    }
}

/// Runs a DC sensitivity analysis, as SPICE's `.sens`.
///
/// Reports resistors' resistance, voltage and current sources' DC value,
/// VCVS gain, VCCS transconductance, CCVS transresistance and CCCS current
/// gain, and the device-model parameters: a diode's IS, N and RS, a BJT's
/// BF, IS, BR, NF, NR and VAF, and a MOSFET's KP, VTO and LAMBDA (one entry
/// each; [`SensResult::get_parameter`] picks one out). The derivative is the
/// total one: the operating point moves with the parameter. Other
/// components stay in the linearised circuit but get no entry: other
/// devices (LED, Zener, Schottky, photodiode, JFET, IGBT), a diode's
/// breakdown `BV`/`IBV`, capacitors and inductors, which have no effect at
/// DC apart from the 1 TΩ leak a capacitor is given. A self-heated device
/// is held at the junction temperature its bias balanced at.
///
/// # Errors
///
/// - [`SimError::UnknownNode`] if the output node is not in the circuit
/// - [`SimError::InvalidConfig`] if a current output does not name an
///   independent voltage source
/// - Any [`SimError`] from the DC operating-point solve
/// - [`SimError::SingularMatrix`] if the linearised system is singular
///
/// # Example
///
/// ```
/// use sindr::{solve_sensitivity, Circuit, CircuitElement, SensConfig, TfOutput};
/// use sindr::element::{Resistor, VoltageSource};
///
/// // 4 V over 1 kΩ / 3 kΩ: V(out) = V1 · R2 / (R1 + R2) = 3 V.
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "0".into()],
///             voltage: 4.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R1".into(),
///             nodes: ["in".into(), "out".into()],
///             resistance: 1_000.0,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R2".into(),
///             nodes: ["out".into(), "0".into()],
///             resistance: 3_000.0,
///         }),
///     ],
///     ..Default::default()
/// };
///
/// let config = SensConfig::new(TfOutput::voltage("out"));
/// let sens = solve_sensitivity(&circuit, &config)?;
///
/// assert!((sens.get("V1").unwrap().sensitivity - 0.75).abs() < 1e-12);
/// // dV(out)/dR1 = −V1 · R2 / (R1 + R2)² = −0.75 mV/Ω
/// assert!((sens.get("R1").unwrap().sensitivity + 0.75e-3).abs() < 1e-12);
/// # Ok::<(), sindr::SimError>(())
/// ```
///
pub fn solve_sensitivity(circuit: &Circuit, config: &SensConfig) -> Result<SensResult, SimError> {
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    solve_sensitivity_with(circuit, config, &crate::SolverOptions::default())
}

/// [`solve_sensitivity`] with `opts`.
pub(crate) fn solve_sensitivity_with(
    circuit: &Circuit,
    config: &SensConfig,
    opts: &crate::SolverOptions,
) -> Result<SensResult, SimError> {
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    let (node_map, dc_solution, op) = bias_point_with_result(circuit, opts)?;
    // Each self-heated device at the junction temperature it balanced at.
    let heated = at_junction_temperatures(circuit, &op);
    let dc = crate::dc_equivalent(&heated);
    let ss_models = linearize_at_dc(&dc, &node_map, &dc_solution);
    let system = small_signal_system(&dc, &node_map, &dc_solution, &ss_models, 0.0);

    let output = Port::output(circuit, &node_map, &system, &config.output)?;
    let adjoint: DVector<Complex64> = system.solve_transpose(&output.excitation(system.size()))?;

    let node = |name: &str| node_map.index(name);
    let v = |n: Option<usize>| n.map_or(0.0, |i| dc_solution[i]);
    let lambda = |n: Option<usize>| n.map_or(0.0, |i| adjoint[i].re);

    let mut entries = Vec::new();
    let mut push = |id: &str, parameter, value: f64, sensitivity: f64| {
        entries.push(SensEntry {
            component_id: id.into(),
            parameter,
            value,
            sensitivity,
            per_percent: sensitivity * value / 100.0,
        });
    };
    // The DC current through each controlling source, for CCVS and CCCS.
    let control = |id: &str| op.branch_currents.get(id).copied().unwrap_or(0.0);
    for component in &heated.components {
        let devices = device::dc_parameters(component, &node_map, &dc_solution);
        for d in devices {
            // dy/dp = −λᵀ·∂F/∂p, F being each node's current into the devices.
            let s = -d
                .derivative
                .iter()
                .map(|&(n, di)| lambda(n) * di)
                .sum::<f64>();
            push(component.id(), d.parameter, d.value, s);
        }
        let (id, parameter, value, sensitivity) = match component {
            CircuitElement::Resistor(Resistor {
                id,
                nodes,
                resistance,
            }) => {
                let (p, q) = (node(&nodes[0]), node(&nodes[1]));
                let dvolts = v(p) - v(q);
                let dlambda = lambda(p) - lambda(q);
                let s = dvolts * dlambda / (resistance * resistance);
                (id, SensParameter::Resistance, *resistance, s)
            }
            CircuitElement::VoltageSource(VoltageSource { id, voltage, .. }) => {
                let Some(branch) = independent_vsource_branch(circuit, &system, id) else {
                    continue;
                };
                (id, SensParameter::Voltage, *voltage, adjoint[branch].re)
            }
            CircuitElement::CurrentSource(CurrentSource {
                id, nodes, current, ..
            }) => {
                // Injects into nodes[1] and draws from nodes[0].
                let s = lambda(node(&nodes[1])) - lambda(node(&nodes[0]));
                (id, SensParameter::Current, *current, s)
            }
            CircuitElement::Vcvs(Vcvs {
                id,
                control_nodes,
                gain,
                ..
            }) => {
                let Some(branch) = dc.vsource_branch_index(id) else {
                    continue;
                };
                let control = v(node(&control_nodes[0])) - v(node(&control_nodes[1]));
                let s = adjoint[node_map.num_nodes() + branch].re * control;
                (id, SensParameter::VoltageGain, *gain, s)
            }
            CircuitElement::Vccs(Vccs {
                id,
                nodes,
                control_nodes,
                gm,
            }) => {
                let control = v(node(&control_nodes[0])) - v(node(&control_nodes[1]));
                let s = (lambda(node(&nodes[1])) - lambda(node(&nodes[0]))) * control;
                (id, SensParameter::Transconductance, *gm, s)
            }
            CircuitElement::Ccvs(Ccvs {
                id,
                control_source,
                rm,
                ..
            }) => {
                let Some(branch) = dc.vsource_branch_index(id) else {
                    continue;
                };
                let s = adjoint[node_map.num_nodes() + branch].re * control(control_source);
                (id, SensParameter::Transresistance, *rm, s)
            }
            CircuitElement::Cccs(Cccs {
                id,
                nodes,
                control_source,
                alpha,
            }) => {
                let s =
                    (lambda(node(&nodes[1])) - lambda(node(&nodes[0]))) * control(control_source);
                (id, SensParameter::CurrentGain, *alpha, s)
            }
            _ => continue,
        };
        push(id, parameter, value, sensitivity);
    }

    Ok(SensResult {
        entries,
        config: config.clone(),
    })
}
