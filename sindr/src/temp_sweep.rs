//! Temperature sweep analysis.
//!
//! Varies junction temperature across a range and solves the operating
//! point at each temperature. IS is re-scaled using the SPICE formula at
//! each step, and the thermal voltage follows the temperature.
//!
//! Follows dc_sweep.rs: each point is solved on the circuit's DC equivalent
//! (capacitors open, inductors shorted, sources at t = 0) with the
//! temperature field of every IS-bearing component set, and Newton–Raphson
//! starts from the previous point's node voltages.

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
    /// The operating point at this temperature, as [`solve_op`] gives it.
    /// Its `transient` is always `None`.
    ///
    /// [`solve_op`]: crate::solve_op
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

/// Sweeps junction temperature across a range and solves the operating
/// point at each step.
///
/// At each temperature, the saturation current `IS` of every IS-bearing
/// component (BJT, diode, LED, Zener, Schottky, photodiode) is rescaled
/// using the SPICE temperature formula, and its thermal voltage set to kT/q,
/// before solving. The base circuit is not modified.
///
/// Each point is the DC operating point, as [`solve_op`] solves it and
/// SPICE's `.temp` with `.op` or `.dc` does: capacitors open, inductors
/// shorted and every waveform source at its t = 0 value. Earlier versions
/// ran a circuit with a capacitor, inductor or waveform as a transient from
/// uncharged capacitors at each point, as [`solve_circuit`] does, and
/// reported its end state. Each point's Newton–Raphson starts from the
/// previous point's node voltages, as [`dc_sweep`] does.
///
/// [`solve_op`]: crate::solve_op
/// [`solve_circuit`]: crate::solve_circuit
/// [`dc_sweep`]: fn@crate::dc_sweep
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
/// - Any [`SimError`] from validating the circuit, or from solving at one
///   of the temperatures (propagated immediately)
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

    crate::validation::validate_circuit(circuit)?;

    let step_size = (temp_end - temp_start) / (num_steps as f64 - 1.0);
    let mut points: Vec<TempSweepPoint> = Vec::with_capacity(num_steps);
    // Only temperatures change from point to point: the nodes stay.
    let mut work = crate::dc_equivalent(circuit);
    let node_map = crate::node_map::NodeMap::from_circuit(&work);
    let mut point_opts = opts.clone();

    for i in 0..num_steps {
        let t = temp_start + i as f64 * step_size;
        let at = |e: SimError| e.at(crate::SolvePoint::Temperature { kelvin: t });

        // Scale IS for all temperature-sensitive components
        for component in &mut work.components {
            match component {
                CircuitElement::Bjt(Bjt { temperature, .. })
                | CircuitElement::Diode(Diode { temperature, .. })
                | CircuitElement::Led(Led { temperature, .. })
                | CircuitElement::ZenerDiode(ZenerDiode { temperature, .. })
                | CircuitElement::SchottkyDiode(SchottkyDiode { temperature, .. })
                | CircuitElement::Photodiode(Photodiode { temperature, .. }) => {
                    *temperature = t;
                }
                _ => {}
            }
        }

        // A self-heated part sits in an ambient at the swept temperature.
        // The circuit is flat here: a block's parts, a variant's included,
        // are top-level parts under their instance path.
        let thermals = work.parts.values_mut().filter_map(|p| p.thermal.as_mut());
        for thermal in thermals {
            thermal.ambient = t;
        }

        if let Some(previous) = points.last() {
            point_opts
                .nodeset
                .clone_from(&previous.result.node_voltages);
        }
        let solve = |dc: &Circuit| crate::solve_dc_circuit_on(dc, &node_map, &point_opts);
        let mut result = match crate::thermal::Heated::from_circuit(&work).map_err(at)? {
            Some(heated) => crate::thermal::solve_dc(&work, &heated, solve),
            None => solve(&work),
        }
        .map_err(at)?;
        crate::strip_secondary_windings(&mut result);
        points.push(TempSweepPoint {
            temperature: t,
            result,
        });
    }

    crate::composite::fold_all(original, points.iter_mut().map(|p| &mut p.result));
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

    /// A 5 V source into three 10 kΩ / 10 µF RC sections and a diode,
    /// optionally through an inductor and with a sine on the source.
    fn rc_ladder_into_diode(reactive_extras: bool) -> Circuit {
        use crate::element::{Capacitor, Diode, Inductor};
        let mut components = vec![CircuitElement::VoltageSource(VoltageSource {
            id: "V1".into(),
            nodes: ["n0".into(), "0".into()],
            voltage: 5.0,
            // Zero at t = 0, so the operating point sees 5 V.
            waveform: reactive_extras.then_some(crate::waveform::Waveform::Sine {
                amplitude: 1.0,
                frequency: 1e3,
                offset: 0.0,
                phase: 0.0,
            }),
        })];
        for k in 1..=3 {
            components.push(CircuitElement::Resistor(Resistor {
                id: format!("R{k}"),
                nodes: [format!("n{}", k - 1).into(), format!("n{k}").into()],
                resistance: 10e3,
            }));
            components.push(CircuitElement::Capacitor(Capacitor {
                id: format!("C{k}"),
                nodes: [format!("n{k}").into(), "0".into()],
                capacitance: 10e-6,
            }));
        }
        let anode = if reactive_extras {
            components.push(CircuitElement::Inductor(Inductor {
                id: "L1".into(),
                nodes: ["n3".into(), "a".into()],
                inductance: 1e-3,
                saturation: None,
            }));
            "a"
        } else {
            "n3"
        };
        components.push(CircuitElement::Diode(Diode {
            id: "D1".into(),
            nodes: [anode.into(), "0".into()],
            temperature: 300.15,
            model: None,
        }));
        Circuit::new("0", components)
    }

    /// Each point is the operating point at its temperature, not the end of
    /// a power-on transient: the ladder's capacitors were still charging
    /// after 5 τ of one section (0.4 mV, 1.6 % of the diode current off).
    #[test]
    fn each_point_is_the_operating_point() {
        for extras in [false, true] {
            let circuit = rc_ladder_into_diode(extras);
            let sweep =
                temperature_sweep(&circuit, &TempSweepConfig::new(250.0, 400.0, 7)).unwrap();
            assert_eq!(sweep.points.len(), 7);
            for point in &sweep.points {
                let mut at_t = circuit.clone();
                for c in &mut at_t.components {
                    if let CircuitElement::Diode(d) = c {
                        d.temperature = point.temperature;
                    }
                }
                let op = crate::solve_op(&at_t).unwrap();
                assert!(point.result.transient.is_none());
                // To Newton's tolerance: the sweep starts each point from
                // the last, `solve_op` from scratch.
                for (node, v) in &op.node_voltages {
                    let got = point.result.node_voltages[node];
                    assert!(
                        (got - v).abs() < 1e-6,
                        "{node} at {} K: {got} vs {v}",
                        point.temperature
                    );
                }
                let current = |r: &SimulationResult| r.component("D1").unwrap().current_through;
                let (got, want) = (current(&point.result), current(&op));
                assert!((got - want).abs() <= 1e-6 * want.abs(), "{got} vs {want}");
                if extras {
                    let il = point.result.branch_current("L1").unwrap();
                    assert!((il - want).abs() <= 1e-6 * want.abs(), "i(L1) {il}");
                }
            }
        }
    }
}
