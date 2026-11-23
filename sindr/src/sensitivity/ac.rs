//! AC sensitivity, as SPICE's `.sens output ac ...`.

use crate::element::{Capacitor, Cccs, Ccvs, Inductor, Resistor, Vccs, Vcvs, VoltageSource};
use nalgebra::DVector;
use num_complex::Complex64;

use super::{device, SensParameter};
use crate::ac_analysis::{
    at_junction_temperatures, bias_point_with_result, generate_frequencies, incremental_inductors,
    linearize_at_dc, small_signal_system, validate_stimulus, AcConfig,
};
use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::transfer_function::{Port, TfOutput};

/// AC sensitivity configuration. Build one with [`AcSensConfig::new`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct AcSensConfig {
    /// The small-signal output whose sensitivities are wanted.
    pub output: TfOutput,
    /// The frequency sweep, and the voltage source that carries the AC
    /// stimulus, at its magnitude and phase.
    pub ac: AcConfig,
}

impl AcSensConfig {
    /// The sensitivities of `output` over the sweep and stimulus of `ac`,
    /// as SPICE's `.sens output ac ...` with the stimulus on `ac`'s source.
    ///
    /// ```
    /// use sindr::ac_analysis::AcConfig;
    /// use sindr::{AcSensConfig, TfOutput};
    ///
    /// // SPICE `.sens v(out) ac dec 10 1 100k`, V1 carrying `AC 1`.
    /// let ac = AcConfig::new("V1", 1.0, 100e3, 51);
    /// let config = AcSensConfig::new(TfOutput::voltage("out"), ac);
    /// assert_eq!(config.ac.source_id, "V1");
    /// ```
    pub fn new(output: TfOutput, ac: AcConfig) -> Self {
        Self { output, ac }
    }
}

/// The sensitivity of the small-signal output to one parameter, at one
/// frequency.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct AcSensEntry {
    /// Id of the component.
    pub component_id: String,
    /// Which of its parameters.
    pub parameter: SensParameter,
    /// The parameter's value, in its own unit.
    pub value: f64,
    /// d(output)/d(parameter), a phasor: the output's unit (V or A) per the
    /// parameter's unit.
    pub sensitivity: Complex64,
    /// Change in the output phasor for a 1 % rise in the parameter:
    /// `sensitivity · value / 100`.
    pub per_percent: Complex64,
}

/// The output and its sensitivities at one frequency.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct AcSensPoint {
    /// Frequency (Hz).
    pub frequency: f64,
    /// The output phasor (V or A), as [`solve_ac`](crate::ac_analysis::solve_ac)
    /// gives it.
    pub output: Complex64,
    /// One entry per supported parameter, in circuit order.
    pub entries: Vec<AcSensEntry>,
}

impl AcSensPoint {
    /// The entry for the component `id`: its first, for a device with
    /// several parameters (see [`get_parameter`](Self::get_parameter)).
    pub fn get(&self, id: &str) -> Option<&AcSensEntry> {
        self.entries.iter().find(|e| e.component_id == id)
    }

    /// The entry for `parameter` of the component `id`.
    pub fn get_parameter(&self, id: &str, parameter: SensParameter) -> Option<&AcSensEntry> {
        self.entries
            .iter()
            .find(|e| e.component_id == id && e.parameter == parameter)
    }
}

/// Result of an AC sensitivity analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct AcSensResult {
    /// One point per frequency of the sweep.
    pub points: Vec<AcSensPoint>,
    /// Configuration used to produce this result.
    pub config: AcSensConfig,
}

impl AcSensResult {
    /// `(frequency, sensitivity)` of `parameter` of the component `id` at
    /// every frequency where it has an entry.
    pub fn curve(&self, id: &str, parameter: SensParameter) -> Vec<(f64, Complex64)> {
        self.points
            .iter()
            .filter_map(|p| {
                p.get_parameter(id, parameter)
                    .map(|e| (p.frequency, e.sensitivity))
            })
            .collect()
    }
}

/// Runs an AC sensitivity analysis, as SPICE's `.sens output ac ...`.
///
/// At each frequency of `config.ac`, the derivative of the small-signal
/// output phasor with respect to each parameter, from one adjoint solve:
/// with the AC system `A(ω)·x = b` and the output `y = cᵀx`, `Aᵀλ = c`
/// gives `dy/dp = λᵀ(∂b/∂p − (∂A/∂p)·x)` for every parameter at once.
///
/// **The DC operating point is held fixed**, as in ngspice: a parameter
/// moves the small-signal circuit it appears in, not the bias the
/// circuit is linearised at. So a source's DC value has no entry, and a
/// device parameter's entry is its effect on that device's small-signal
/// conductances and capacitances at the bias point, not through the shift
/// of the bias point and every other device's linearisation. For a linear
/// circuit the two agree. For the total DC effect, see
/// [`solve_sensitivity`](crate::solve_sensitivity).
///
/// Entries: resistance, capacitance and inductance (not of a saturating
/// inductor), VCVS, VCCS, CCVS and CCCS gains, the stimulus source's AC
/// magnitude, and the device-model parameters
/// [`solve_sensitivity`](crate::solve_sensitivity) reports, with a diode's
/// `CJO` and `TT` as well when it stores charge.
///
/// # Errors
///
/// - [`SimError::InvalidComponent`] if `config.ac` is invalid (as for
///   [`solve_ac`](crate::ac_analysis::solve_ac)), if the output node is not
///   in the circuit, or a current output does not name an independent
///   voltage source
/// - Any [`SimError`] from the DC operating-point solve
/// - [`SimError::SingularMatrix`] if the small-signal system is singular at
///   any frequency
///
/// # Example
///
/// ```
/// use num_complex::Complex64;
/// use sindr::ac_analysis::AcConfig;
/// use sindr::{solve_ac_sensitivity, AcSensConfig, Circuit, CircuitElement, SensParameter, TfOutput};
/// use sindr::element::{Capacitor, Resistor, VoltageSource};
///
/// // RC low-pass: V(out) = 1 / (1 + jωRC).
/// let (r, c) = (1_000.0, 1e-6);
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "0".into()],
///             voltage: 0.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "R1".into(),
///             nodes: ["in".into(), "out".into()],
///             resistance: r,
///         }),
///         CircuitElement::Capacitor(Capacitor {
///             id: "C1".into(),
///             nodes: ["out".into(), "0".into()],
///             capacitance: c,
///         }),
///     ],
///     ..Default::default()
/// };
///
/// let config = AcSensConfig::new(TfOutput::voltage("out"), AcConfig::new("V1", 159.0, 159.0, 1));
/// let sens = solve_ac_sensitivity(&circuit, &config)?;
///
/// // dV(out)/dR = −jωC / (1 + jωRC)².
/// let jw = Complex64::new(0.0, 2.0 * std::f64::consts::PI * 159.0);
/// let want = -jw * c / ((1.0 + jw * r * c) * (1.0 + jw * r * c));
/// let got = sens.points[0].get_parameter("R1", SensParameter::Resistance).unwrap();
/// assert!((got.sensitivity - want).norm() < 1e-12);
/// # Ok::<(), sindr::SimError>(())
/// ```
///
pub fn solve_ac_sensitivity(
    circuit: &Circuit,
    config: &AcSensConfig,
) -> Result<AcSensResult, SimError> {
    solve_ac_sensitivity_with(circuit, config, &crate::SolverOptions::default())
}

/// [`solve_ac_sensitivity`] with `opts`.
pub(crate) fn solve_ac_sensitivity_with(
    circuit: &Circuit,
    config: &AcSensConfig,
    opts: &crate::SolverOptions,
) -> Result<AcSensResult, SimError> {
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    let ac = &config.ac;
    validate_stimulus(circuit, ac)?;

    let (node_map, dc_solution, op) = bias_point_with_result(circuit, opts)?;
    let small_signal = incremental_inductors(&at_junction_temperatures(circuit, &op), &op);
    let ss_models = linearize_at_dc(&small_signal, &node_map, &dc_solution);
    let num_nodes = node_map.num_nodes();
    let stimulus = Complex64::from_polar(1.0, ac.ac_phase_deg.to_radians());
    let zero = Complex64::new(0.0, 0.0);

    let mut points = Vec::new();
    for frequency in generate_frequencies(ac) {
        let omega = 2.0 * std::f64::consts::PI * frequency;
        let system = small_signal_system(&small_signal, &node_map, &dc_solution, &ss_models, omega);
        let output = Port::output(&small_signal, &node_map, &system, &config.output)?;
        let c = output.excitation(system.size());

        let mut b = DVector::from_element(system.size(), zero);
        let source_branch = system.vsource_branch(&ac.source_id);
        if let Some(k) = source_branch {
            b[k] = stimulus * ac.ac_magnitude;
        }
        let x = system.solve(&b)?;
        let lambda = system.solve_transpose(&c)?;

        let at = |v: &DVector<Complex64>, n: Option<usize>| n.map_or(zero, |i| v[i]);
        let node = |name: &String| node_map.index(name);
        let across = |v: &DVector<Complex64>, nodes: &[String; 2]| {
            at(v, node(&nodes[0])) - at(v, node(&nodes[1]))
        };
        let branch = |id: &str| small_signal.vsource_branch_index(id).map(|k| num_nodes + k);

        let mut entries = Vec::new();
        let mut push = |id: &str, parameter, value: f64, sensitivity: Complex64| {
            entries.push(AcSensEntry {
                component_id: id.to_string(),
                parameter,
                value,
                sensitivity,
                per_percent: sensitivity * value / 100.0,
            });
        };
        // The original elements tell a saturating inductor from a linear one.
        for (component, original) in small_signal.components.iter().zip(&circuit.components) {
            for d in device::ac_parameters(component, &node_map, &dc_solution, omega) {
                // dy/dp = −λᵀ·(∂A/∂p)·x.
                let s: Complex64 = d
                    .derivative
                    .iter()
                    .map(|&(r, k, dy)| at(&lambda, r) * dy * at(&x, k))
                    .sum();
                push(component.id(), d.parameter, d.value, -s);
            }
            // The admittance derivative ∂Y/∂p of a two-terminal element.
            let two_terminal = |nodes: &[String; 2], dy: Complex64| {
                -across(&lambda, nodes) * dy * across(&x, nodes)
            };
            let (id, parameter, value, sensitivity) = match component {
                CircuitElement::Resistor(Resistor {
                    id,
                    nodes,
                    resistance,
                }) => {
                    let dy = Complex64::new(-1.0 / (resistance * resistance), 0.0);
                    (
                        id,
                        SensParameter::Resistance,
                        *resistance,
                        two_terminal(nodes, dy),
                    )
                }
                CircuitElement::Capacitor(Capacitor {
                    id,
                    nodes,
                    capacitance,
                }) => {
                    let dy = Complex64::new(0.0, omega);
                    (
                        id,
                        SensParameter::Capacitance,
                        *capacitance,
                        two_terminal(nodes, dy),
                    )
                }
                CircuitElement::Inductor(Inductor {
                    id,
                    nodes,
                    inductance,
                    ..
                }) => {
                    if matches!(
                        original,
                        CircuitElement::Inductor(Inductor {
                            saturation: Some(_),
                            ..
                        })
                    ) {
                        continue;
                    }
                    // Y = 1/(jωL); at ω = 0 the stamp is a fixed short.
                    let dy = if omega.abs() > 1e-15 {
                        -1.0 / (Complex64::new(0.0, omega) * inductance * inductance)
                    } else {
                        zero
                    };
                    (
                        id,
                        SensParameter::Inductance,
                        *inductance,
                        two_terminal(nodes, dy),
                    )
                }
                CircuitElement::VoltageSource(VoltageSource { id, .. }) if *id == ac.source_id => {
                    let Some(k) = source_branch else { continue };
                    (
                        id,
                        SensParameter::AcMagnitude,
                        ac.ac_magnitude,
                        lambda[k] * stimulus,
                    )
                }
                CircuitElement::Vcvs(Vcvs {
                    id,
                    control_nodes,
                    gain,
                    ..
                }) => {
                    let Some(k) = branch(id) else { continue };
                    let s = lambda[k] * across(&x, control_nodes);
                    (id, SensParameter::VoltageGain, *gain, s)
                }
                CircuitElement::Vccs(Vccs {
                    id,
                    nodes,
                    control_nodes,
                    gm,
                }) => {
                    let s = -across(&lambda, nodes) * across(&x, control_nodes);
                    (id, SensParameter::Transconductance, *gm, s)
                }
                CircuitElement::Ccvs(Ccvs {
                    id,
                    control_source,
                    rm,
                    ..
                }) => {
                    let (Some(k), Some(ctrl)) = (branch(id), branch(control_source)) else {
                        continue;
                    };
                    (id, SensParameter::Transresistance, *rm, lambda[k] * x[ctrl])
                }
                CircuitElement::Cccs(Cccs {
                    id,
                    nodes,
                    control_source,
                    alpha,
                }) => {
                    let Some(ctrl) = branch(control_source) else {
                        continue;
                    };
                    let s = -across(&lambda, nodes) * x[ctrl];
                    (id, SensParameter::CurrentGain, *alpha, s)
                }
                _ => continue,
            };
            push(id, parameter, value, sensitivity);
        }

        let output_value: Complex64 = c.iter().zip(x.iter()).map(|(c, x)| c * x).sum();
        points.push(AcSensPoint {
            frequency,
            output: output_value,
            entries,
        });
    }

    Ok(AcSensResult {
        points,
        config: config.clone(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::{BjtModel, DiodeModel};
    use crate::element::{Bjt, Diode, Mosfet};
    use sindr_devices::bjt::BjtKind;
    use sindr_devices::mosfet::{MosfetKind, MosfetParams};
    use SensParameter as P;

    /// `circuit` with `parameter` of `id` at `x`.
    fn with_parameter(circuit: &Circuit, id: &str, parameter: P, x: f64) -> Circuit {
        let mut c = circuit.clone();
        let Some(element) = c.components.iter_mut().find(|e| e.id() == id) else {
            panic!("no {id}");
        };
        match (element, parameter) {
            (CircuitElement::Resistor(Resistor { resistance, .. }), P::Resistance) => {
                *resistance = x
            }
            (CircuitElement::Capacitor(Capacitor { capacitance, .. }), P::Capacitance) => {
                *capacitance = x
            }
            (CircuitElement::Inductor(Inductor { inductance, .. }), P::Inductance) => {
                *inductance = x
            }
            (CircuitElement::Vcvs(Vcvs { gain, .. }), P::VoltageGain) => *gain = x,
            (CircuitElement::Vccs(Vccs { gm, .. }), P::Transconductance) => *gm = x,
            (CircuitElement::Ccvs(Ccvs { rm, .. }), P::Transresistance) => *rm = x,
            (CircuitElement::Cccs(Cccs { alpha, .. }), P::CurrentGain) => *alpha = x,
            (CircuitElement::Diode(Diode { model, .. }), p) => {
                let m = model.get_or_insert_with(DiodeModel::default);
                match p {
                    P::SaturationCurrent => m.is = x,
                    P::EmissionCoefficient => m.n = x,
                    P::SeriesResistance => m.rs = x,
                    P::JunctionCapacitance => m.cjo = x,
                    P::TransitTime => m.tt = x,
                    _ => panic!("diode {p:?}"),
                }
            }
            (CircuitElement::Bjt(Bjt { bf, .. }), P::ForwardBeta) => *bf = x,
            (CircuitElement::Bjt(Bjt { model, .. }), p) => {
                let m = model.get_or_insert_with(BjtModel::default);
                match p {
                    P::SaturationCurrent => m.is = x,
                    P::ReverseBeta => m.br = x,
                    P::ForwardEmissionCoefficient => m.nf = x,
                    P::ReverseEmissionCoefficient => m.nr = x,
                    P::EarlyVoltage => m.vaf = x,
                    _ => panic!("BJT {p:?}"),
                }
            }
            (CircuitElement::Mosfet(Mosfet { params, .. }), p) => match p {
                P::TransconductanceParameter => params.kp = x,
                P::ThresholdVoltage => params.vto = x,
                P::ChannelLengthModulation => params.lambda = x,
                _ => panic!("MOSFET {p:?}"),
            },
            (e, p) => panic!("{} has no {p:?}", e.id()),
        }
        c
    }

    /// The output phasor of `perturbed` at `frequency`, linearised at the
    /// bias point of `circuit`: the frozen bias of AC sensitivity.
    fn output_at_bias(
        circuit: &Circuit,
        perturbed: &Circuit,
        config: &AcSensConfig,
        f: f64,
    ) -> Complex64 {
        let opts = crate::SolverOptions::default();
        let (node_map, x0, op) = bias_point_with_result(circuit, &opts).unwrap();
        let ss = incremental_inductors(&at_junction_temperatures(perturbed, &op), &op);
        let models = linearize_at_dc(&ss, &node_map, &x0);
        let system =
            small_signal_system(&ss, &node_map, &x0, &models, 2.0 * std::f64::consts::PI * f);
        let mut b = DVector::from_element(system.size(), Complex64::new(0.0, 0.0));
        let k = system.vsource_branch(&config.ac.source_id).unwrap();
        b[k] = Complex64::from_polar(config.ac.ac_magnitude, config.ac.ac_phase_deg.to_radians());
        let x = system.solve(&b).unwrap();
        let c = Port::output(&ss, &node_map, &system, &config.output)
            .unwrap()
            .excitation(system.size());
        c.iter().zip(x.iter()).map(|(c, x)| c * x).sum()
    }

    /// Every entry at every frequency against a central difference of the
    /// output with the bias held, and the output against `solve_ac`.
    fn assert_matches_frozen_bias(
        name: &str,
        circuit: &Circuit,
        config: &AcSensConfig,
        tolerance: f64,
    ) {
        let result = solve_ac_sensitivity(circuit, config).unwrap();
        let ac = crate::ac_analysis::solve_ac(circuit, &config.ac).unwrap();
        assert!(!result.points.is_empty());
        for (point, ac_point) in result.points.iter().zip(&ac.points) {
            if let TfOutput::Voltage {
                node,
                reference: None,
            } = &config.output
            {
                let want = ac_point.node_voltages[node];
                assert!(
                    (point.output - want).norm() <= 1e-12 * want.norm().max(1e-12),
                    "{name}"
                );
            }
            assert!(!point.entries.is_empty(), "{name}");
            for e in &point.entries {
                if e.parameter == P::AcMagnitude {
                    let want = point.output / e.value;
                    assert!(
                        (e.sensitivity - want).norm() <= 1e-9 * want.norm(),
                        "{name}"
                    );
                    continue;
                }
                let h = if e.value == 0.0 {
                    1e-9
                } else {
                    e.value.abs() * 1e-4
                };
                let at = |x: f64| {
                    output_at_bias(
                        circuit,
                        &with_parameter(circuit, &e.component_id, e.parameter, x),
                        config,
                        point.frequency,
                    )
                };
                let fd = if e.value == 0.0 {
                    (at(h) - at(0.0)) / h
                } else {
                    (at(e.value + h) - at(e.value - h)) / (2.0 * h)
                };
                // Each to 1e-4 of itself, or, for one that barely moves
                // the output, to 1e-7 of the output for a 100 % change in
                // the parameter: below that the two solves' round-off wins.
                let error = (e.sensitivity - fd).norm();
                let scale = fd.norm().max(e.sensitivity.norm());
                let negligible = error * e.value.abs() <= 1e-7 * point.output.norm();
                assert!(
                    error <= tolerance * scale || negligible,
                    "{name} at {} Hz: {} {:?}: {} vs {fd}",
                    point.frequency,
                    e.component_id,
                    e.parameter,
                    e.sensitivity
                );
            }
        }
    }

    fn vsource(id: &str, p: &str, v: f64) -> CircuitElement {
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

    fn cap(id: &str, a: &str, b: &str, c: f64) -> CircuitElement {
        CircuitElement::Capacitor(Capacitor {
            id: id.into(),
            nodes: [a.into(), b.into()],
            capacitance: c,
        })
    }

    fn circuit(components: Vec<CircuitElement>) -> Circuit {
        Circuit {
            ground_node: "0".into(),
            components,
            ..Default::default()
        }
    }

    fn config(out: &str, f_start: f64, f_stop: f64) -> AcSensConfig {
        AcSensConfig::new(
            TfOutput::voltage(out),
            AcConfig::new("V1", f_start, f_stop, 5)
                .with_magnitude(0.5)
                .with_phase_deg(30.0),
        )
    }

    #[test]
    fn diode_with_stored_charge_matches_frozen_bias() {
        let model = DiodeModel {
            is: 4e-15,
            n: 1.4,
            rs: 8.0,
            cjo: 5e-12,
            tt: 20e-9,
            ..DiodeModel::default()
        };
        let c = circuit(vec![
            vsource("V1", "in", 2.0),
            res("R1", "in", "a", 1e3),
            CircuitElement::Diode(Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 320.0,
                model: Some(model),
            }),
            cap("C1", "a", "0", 1e-9),
        ]);
        let r = solve_ac_sensitivity(&c, &config("a", 1e3, 100e6)).unwrap();
        let params: Vec<_> = r.points[0]
            .entries
            .iter()
            .filter(|e| e.component_id == "D1")
            .map(|e| e.parameter)
            .collect();
        assert_eq!(
            params,
            [
                P::SaturationCurrent,
                P::EmissionCoefficient,
                P::SeriesResistance,
                P::JunctionCapacitance,
                P::TransitTime
            ]
        );
        // The source's DC value has no AC entry: the bias is held.
        assert!(r.points[0].get("V1").map(|e| e.parameter) == Some(P::AcMagnitude));
        assert_matches_frozen_bias("diode", &c, &config("a", 1e3, 100e6), 1e-4);
    }

    #[test]
    fn bjt_stage_matches_frozen_bias() {
        let model = BjtModel {
            is: 3e-15,
            br: 2.0,
            nf: 1.05,
            nr: 1.1,
            vaf: 60.0,
            ..BjtModel::default()
        };
        for kind in [BjtKind::Npn, BjtKind::Pnp] {
            let s = if kind == BjtKind::Npn { 1.0 } else { -1.0 };
            let c = circuit(vec![
                vsource("VCC", "vcc", s * 12.0),
                vsource("V1", "in", s * 1.5),
                res("RS", "in", "b", 10e3),
                res("RC", "vcc", "c", 3.3e3),
                res("RE", "e", "0", 470.0),
                cap("CE", "e", "0", 10e-6),
                cap("CL", "c", "0", 100e-12),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "e".into()],
                    kind,
                    bf: 120.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: Some(model),
                }),
            ]);
            assert_matches_frozen_bias("BJT", &c, &config("c", 10.0, 10e6), 1e-4);
        }
    }

    #[test]
    fn mosfet_and_controlled_sources_match_frozen_bias() {
        let nmos = MosfetParams {
            kp: 5e-4,
            vto: 0.8,
            lambda: 0.03,
            ..MosfetParams::default_nmos()
        };
        let c = circuit(vec![
            vsource("VDD", "vdd", 10.0),
            vsource("V1", "g", 2.0),
            res("RD", "vdd", "d", 5e3),
            cap("CD", "d", "0", 1e-9),
            CircuitElement::Mosfet(Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind: MosfetKind::Nmos,
                params: nmos.clone(),
                parasitic_caps: None,
                body_diode: None,
            }),
            CircuitElement::Vcvs(Vcvs {
                id: "E1".into(),
                nodes: ["e".into(), "0".into()],
                control_nodes: ["d".into(), "0".into()],
                gain: 0.5,
            }),
            CircuitElement::Inductor(Inductor {
                id: "L1".into(),
                nodes: ["e".into(), "m".into()],
                inductance: 1e-3,
                saturation: None,
            }),
            CircuitElement::VoltageSource(VoltageSource {
                id: "VS".into(),
                nodes: ["m".into(), "n".into()],
                voltage: 0.0,
                waveform: None,
            }),
            res("R2", "n", "0", 2e3),
            CircuitElement::Vccs(Vccs {
                id: "G1".into(),
                nodes: ["0".into(), "out".into()],
                control_nodes: ["e".into(), "0".into()],
                gm: 1e-3,
            }),
            CircuitElement::Ccvs(Ccvs {
                id: "H1".into(),
                nodes: ["h".into(), "0".into()],
                control_source: "VS".into(),
                rm: 500.0,
            }),
            res("R3", "h", "out", 1e3),
            CircuitElement::Cccs(Cccs {
                id: "F1".into(),
                nodes: ["0".into(), "out".into()],
                control_source: "VS".into(),
                alpha: 2.0,
            }),
            res("R4", "out", "0", 1e3),
        ]);
        assert_matches_frozen_bias("MOSFET", &c, &config("out", 100.0, 10e6), 1e-4);
        // A MOSFET in triode, the output a current.
        let triode = circuit(vec![
            vsource("V1", "g", 4.0),
            vsource("VDD", "vdd", 0.5),
            res("RD", "vdd", "d", 100.0),
            cap("CG", "g", "d", 1e-9),
            CircuitElement::Mosfet(Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind: MosfetKind::Nmos,
                params: nmos,
                parasitic_caps: None,
                body_diode: None,
            }),
        ]);
        let current = AcSensConfig::new(TfOutput::current("VDD"), AcConfig::new("V1", 1e3, 1e8, 4));
        assert_matches_frozen_bias("triode", &triode, &current, 1e-4);
    }
}
