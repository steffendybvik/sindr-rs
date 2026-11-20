//! A circuit simulator you embed in Rust: build a circuit as a typed value,
//! solve it natively or in WebAssembly, and get back numbers checked against
//! ngspice. Pure Rust, no C libraries, no global state. Besides voltages and
//! currents it answers design questions: whether each part stays inside its
//! ratings ([`check_ratings`]) and whether a spec passes ([`check_specs`]).
//! Accuracy against ngspice is tabled in the
//! [repository](https://github.com/steffendybvik/sindr-rs/blob/main/docs/ACCURACY.md).
//!
//! `sindr` solves a [`Circuit`] — a list of components and a ground node —
//! and returns voltages, currents, and power for every component. The solver
//! picks the right path automatically:
//!
//! - **Linear DC** — direct solve of the MNA system.
//! - **Nonlinear DC** — Newton–Raphson when diodes, BJTs, MOSFETs, etc. are present.
//! - **Transient** — Gear-2 (or trapezoidal or backward-Euler) timestepping
//!   when capacitors, inductors, or time-varying sources are present.
//! - **AC small-signal** — sinusoidal-steady-state via [`ac_analysis::solve_ac`].
//! - **Transfer function** — DC small-signal gain, input and output
//!   resistance (SPICE's `.tf`) via [`solve_tf`].
//! - **DC sensitivity** — derivative of an output with respect to each
//!   component parameter (SPICE's `.sens`) via [`solve_sensitivity`].
//! - **DC sweep** — operating point at each value of a swept voltage source,
//!   via [`fn@dc_sweep`].
//! - **Temperature sweep** — operating-point sweep over junction temperature
//!   via [`temperature_sweep`].
//!
//! Device physics (diode, BJT, MOSFET, IGBT, JFET, varactor companion models)
//! live in the companion crate
//! [`sindr-devices`](https://crates.io/crates/sindr-devices). `sindr`
//! re-exports the few enums you'll typically need ([`BjtKind`],
//! [`MosfetKind`], [`JfetKind`], etc.).
//!
//! # Quick start
//!
//! How much current does a red LED draw from 5 V through 220 Ω?
//!
//! ```
//! use sindr::{solve_circuit, Circuit, CircuitElement};
//!
//! let circuit = Circuit::new("gnd", vec![
//!     CircuitElement::VoltageSource { id: "V1".into(), nodes: ["vcc".into(), "gnd".into()], voltage: 5.0, waveform: None },
//!     CircuitElement::Resistor { id: "R1".into(), nodes: ["vcc".into(), "a".into()], resistance: 220.0 },
//!     CircuitElement::Led { id: "D1".into(), nodes: ["a".into(), "gnd".into()], color: "red".into(), temperature: 300.15 },
//! ]);
//! let result = solve_circuit(&circuit)?;
//! let led = result.component_results.iter().find(|c| c.id == "D1").unwrap();
//! println!("{:.1} mA, {:.1} mW", led.current_through * 1e3, led.power * 1e3);
//!
//! // About 1.8 V across the LED leaves about 3.2 V across R1.
//! assert!((led.current_through - 3.2 / 220.0).abs() < 1.5e-3);
//! # Ok::<(), sindr::SimError>(())
//! ```
//!
//! # Data model and hierarchy
//!
//! [`Circuit`] and everything in it lives in [`circuit`], [`design`] and
//! [`waveform`]. Besides its flat `components`, a
//! circuit can place [`Block`]s through [`Instance`]s, name `globals`, and
//! carry [`Part`] data (reference, symbol, footprint) the solver ignores.
//! Every analysis flattens the hierarchy first ([`Circuit::flatten`]: `X1.R1`,
//! `X1.mid`) and reports results under the flattened names. A broken
//! hierarchy is [`SimError::Design`].
//!
//! # Capabilities
//!
//! ## Analyses
//!
//! | Analysis | Entry point | Notes |
//! |---|---|---|
//! | Operating point (linear or nonlinear DC) | [`solve_circuit`] | Newton–Raphson for nonlinear devices |
//! | Transient | [`solve_transient`] | Gear-2 (default), trapezoidal or backward Euler ([`IntegrationMethod`]), steps up to the [`TransientConfig`]'s `tstep`, shortened by truncation-error control and landing on waveform breakpoints. [`solve_circuit`] also takes this path when the circuit has C, L or waveform sources, choosing 5 τ and τ/50 itself |
//! | AC small-signal | [`ac_analysis::solve_ac`] | Needs an [`ac_analysis::AcConfig`]. [`solve_circuit`] **never** runs AC |
//! | Transfer function (`.tf`) | [`solve_tf`] | DC small-signal gain, input and output resistance at the operating point. Input: a voltage or current source; output: a node voltage or a voltage source's current ([`TfOutput`]) |
//! | DC sensitivity (`.sens`) | [`solve_sensitivity`] | Adjoint method: d(output)/d(parameter) for resistances, source values, VCVS gain and VCCS gm. No device-model parameters, no AC sensitivity |
//! | Noise (`.noise`) | [`solve_noise`] | Thermal, shot and flicker noise at an output voltage, referred to an input source, with a per-source breakdown and band totals |
//! | DC sweep | [`fn@dc_sweep`], [`dc_sweep_nested`] | One voltage or current source, or two nested (`.dc V1 … V2 …`) |
//! | Temperature sweep | [`temperature_sweep`] | Re-solves the operating point at each junction temperature |
//! | Solver options (`.options`, `.nodeset`) | [`Solver`], [`SolverOptions`] | Tolerances, iteration budget, gmin and source stepping, and a nodeset seeding Newton for every analysis's bias point. The free functions use the defaults |
//! | Self-heating | [`solve_circuit`], [`solve_transient`] | A part with [`Thermal`] data heats by its dissipation; DC iterates to a balance, transient carries a thermal RC. The small-signal analyses linearise at the heated bias point. Results in `junction_temperatures`; no balance is [`SimError::ThermalRunaway`] |
//! | Ratings check | [`check_ratings`] | Each part against its absolute maximum [`Ratings`], at the operating point or every transient step. Missing ratings report unverified |
//! | Measurements and specs (`.meas`) | [`check_specs`] | [`Measurement`]s (value at x, min/max/avg/RMS, crossings, delay, rise/fall, settling, −3 dB) on any result; [`Spec`] limits give pass/fail and margin |
//! | Initial conditions (`.ic`, `UIC`) | [`TransientConfig::with_initial_voltages`], [`TransientConfig::with_initial_current`], [`TransientConfig::with_start`] | The state a transient starts from: switched on at t = 0 (default) or the DC operating point ([`TransientStart`]) |
//!
//! Not available: distortion, pole-zero, AC sensitivity, Monte Carlo,
//! periodic steady state, S-parameters, variable-order integration.
//!
//! ## Device models
//!
//! | Device | Model |
//! |---|---|
//! | Diode, LED, Schottky, photodiode | Shockley; IS and V_T = kT/q follow each device's `temperature` (SPICE IS(T), XTI = 3) |
//! | Zener | A SPICE diode with breakdown `BV = vz` at 5 mA and RS = 1 Ω |
//! | BJT (NPN/PNP) | Ebers–Moll with Early voltage; not Gummel–Poon |
//! | MOSFET (NMOS/PMOS) | SPICE Level 1 (Shichman–Hodges), body tied to source, SPICE-signed VTO (negative for an enhancement PMOS); no BSIM |
//! | JFET (N/P) | Shichman–Hodges square law |
//! | IGBT | MOSFET-like gate control, square-law collector current |
//! | Varactor | Voltage-dependent junction capacitance (transient; AC at the bias point) |
//! | Op-amp | VCVS of gain 1e5, or with an [`OpAmpModel`]: single-pole GBW, slew rate, rail clamp, output resistance, offset, bias current |
//! | Comparator | Gain 1e5, output clamped to its rails; no GBW, slew rate or hysteresis |
//! | Transformer | Linear coupled inductors |
//! | Battery | Open-circuit voltage by state of charge behind R0 and an optional R1‖C1; state of charge falls with charge drawn ([`battery::BatteryParams`]; Li-ion and CR2032 presets) |
//! | Voltage regulator | Piecewise linear: dropout, quiescent current, current limit, line and load regulation ([`RegulatorParams`]; LM7805 and AMS1117 presets) |
//! | Crystal | Butterworth–Van Dyke: `Rm`–`Lm`–`Cm` beside `C0` ([`CrystalParams`]) |
//! | Ferrite bead | `R_dc` into `L ∥ R_ac` (∥ `C`) ([`FerriteBeadParams`]) |
//!
//! Parameters are Rust structs, not SPICE `.model` cards.
//!
//! ## Convergence aids
//!
//! Junction voltage limiting and Newton damping on every nonlinear solve,
//! with junctions starting at their critical voltage, as in SPICE. Newton
//! converges only once node voltages and every diode, LED and BJT junction
//! voltage have settled. When plain Newton–Raphson fails, the DC solve retries with gmin stepping
//! (1e-2 S down to 1e-12 S), then with source stepping (every source ramped
//! up from zero); [`SimulationResult::convergence_aid`] says which one was
//! needed. A transient step whose Newton solve fails is retried at half the
//! step.
//!
//! ## Netlist import for editors
//!
//! [`import`] takes a schematic tool's netlist in the caller's own ids:
//! components with a kind, a value (`"4k7"`, `"100nF"`) and named
//! terminals, and nets of `(component, terminal)` pins. A [`import::Bench`]
//! adds what a schematic leaves out (ground, supplies, signal sources,
//! loads, the analysis), and [`import::simulate`] runs it. Results are
//! keyed by the caller's component and net ids, and every
//! [`import::ImportError`] names the part, net or bench item at fault.
//!
//! ## SPICE input (feature `spice`)
//!
//! [`spice`] reads a SPICE3 subset into a [`Circuit`]: R, L, C, V, I, D, Q
//! and X elements; `.subckt`, `.model`, `.param`, `.include`, `.lib`,
//! `.ic` and `.nodeset`; and the analysis cards `.op`, `.dc`, `.ac`, `.tf`,
//! `.sens` and `.tran`. Not supported: MOSFET (M)
//! and JFET (J) elements, controlled sources (E, F, G, H), coupled inductors
//! (K), `B` sources, most `.model` parameters (diodes always use IS = 1e-14,
//! N = 1; BJTs honour only BF), and netlist export.
//!
//! The full list of what sindr doesn't do is in
//! [LIMITATIONS.md](https://github.com/steffendybvik/sindr-rs/blob/main/LIMITATIONS.md).
//!
//! # Conventions
//!
//! - **Ground node** must exist on at least one component. Its voltage is
//!   defined as 0 V — every other voltage is reported relative to it.
//! - **Node names** are arbitrary strings (`"0"`, `"gnd"`, `"vcc"`, …).
//!   Components share a node simply by referencing the same string.
//! - **SI units** throughout: V, A, Ω, F, H, s, K.
//! - **Sign conventions** are documented per [`CircuitElement`] variant.
//!
//! # Cargo features
//!
//! - `serde` *(default)* — `Serialize`/`Deserialize` impls on the public
//!   types.
//! - `examples` *(default, implies `serde`)* — exposes [`get_examples`] and
//!   the built-in named circuits (voltage divider, BJT amp, RC transient,
//!   etc.).
//! - `spice` — the SPICE3 netlist parser in [`spice`] (see below).
//!
//! ```toml
//! [dependencies]
//! sindr = "0.1"
//!
//! # No serde, no built-in examples:
//! sindr = { version = "0.1", default-features = false }
//! ```
//!
//! # Transient results
//!
//! A circuit with capacitors, inductors or waveform sources takes the
//! transient path, and [`SimulationResult::transient`] holds the time series.
//! [`solve_circuit`] picks the duration (5 τ) and step (τ/50) from the
//! circuit's time constants; to choose them, as SPICE's `.tran` does, call
//! [`solve_transient`] with a [`TransientConfig`].
//!
//! ```
//! use sindr::{Circuit, CircuitElement, solve_circuit};
//!
//! // 5 V step into R = 1 kΩ, C = 1 µF: τ = 1 ms.
//! let circuit = Circuit {
//!     ground_node: "0".into(),
//!     components: vec![
//!         CircuitElement::VoltageSource {
//!             id: "V1".into(),
//!             nodes: ["in".into(), "0".into()],
//!             voltage: 5.0,
//!             waveform: None,
//!         },
//!         CircuitElement::Resistor {
//!             id: "R1".into(),
//!             nodes: ["in".into(), "out".into()],
//!             resistance: 1_000.0,
//!         },
//!         CircuitElement::Capacitor {
//!             id: "C1".into(),
//!             nodes: ["out".into(), "0".into()],
//!             capacitance: 1e-6,
//!         },
//!     ],
//!     ..Default::default()
//! };
//!
//! let result = solve_circuit(&circuit).unwrap();
//! let tran = result.transient.expect("reactive circuit takes the transient path");
//!
//! // The capacitor charges from 0 V towards 5 V.
//! let first = &tran.timesteps[0];
//! let last = tran.timesteps.last().unwrap();
//! assert!(first.node_voltages["out"] < 1.0);
//! assert!((last.node_voltages["out"] - 5.0).abs() < 0.1);
//! ```
//!
//! # SPICE netlist input
//!
//! Parsing SPICE3 `.cir` decks into a [`Circuit`] lives behind the optional
//! `spice` feature (off by default, so the heavier parser dependencies stay
//! out of the baseline build):
//!
//! ```toml
//! [dependencies]
//! sindr = { version = "0.1", features = ["spice"] }
//! ```
//!
//! (`no_run`: it reads `amp.cir`, which doesn't exist where doctests run.)
//!
//! ```no_run
//! # #[cfg(feature = "spice")]
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let netlist = sindr::spice::parse_file("amp.cir")?;
//! let result = sindr::solve_circuit(&netlist.circuit)?;
//! # Ok(())
//! # }
//! # #[cfg(not(feature = "spice"))]
//! # fn main() {}
//! ```
//!
//! See the [`spice`] module for a runnable example, the supported subset and
//! strictness options.
//!
//! # Where to look next
//!
//! - [`Circuit`] / [`CircuitElement`] — the input format
//! - [`solve_circuit`] — the headline entry point
//! - [`SimulationResult`] — what you get back
//! - [`Waveform`] — time-varying source shapes
//! - [`SimError`] — what can go wrong

#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![warn(rustdoc::broken_intra_doc_links, rustdoc::missing_crate_level_docs)]
#![warn(clippy::missing_errors_doc, clippy::missing_panics_doc)]
#![cfg_attr(docsrs, feature(doc_cfg))]

pub mod examples;
#[cfg(feature = "examples")]
#[cfg_attr(docsrs, doc(cfg(feature = "examples")))]
pub use examples::{get_examples, ExampleCircuit};

pub mod ac_analysis;
mod battery_state;
pub mod circuit;
mod composite;
pub mod csv;
pub mod dc_sweep;
pub mod design;
mod device_params;
pub mod error;
pub mod import;
mod integration;
pub mod measure;
// The matrix type is public only for sindr's own `mna_solve` bench, behind
// the unstable `internals` feature.
#[cfg(feature = "internals")]
#[doc(hidden)]
pub mod mna;
#[cfg(not(feature = "internals"))]
mod mna;
mod newton_raphson;
mod node_map;
pub mod noise;
mod op_amp;
pub mod ratings;
pub mod results;
pub mod sensitivity;
pub mod solver;
#[cfg(feature = "spice")]
#[cfg_attr(docsrs, doc(cfg(feature = "spice")))]
pub mod spice;
mod stamp;
#[cfg(test)]
mod stamp_jacobian_tests;
pub mod temp_sweep;
mod thermal;
pub mod transfer_function;
mod transient;
pub mod validation;
pub mod waveform;

pub use ac_analysis::{solve_ac, AcConfig, AcPoint, AcResult, FrequencySpacing};
pub use circuit::{
    BjtModel, Circuit, CircuitElement, CrystalParams, DiodeModel, FerriteBeadParams, OpAmpModel,
    SpdtPosition,
};
pub use dc_sweep::{dc_sweep, dc_sweep_nested, DcSweepResult, NestedDcSweepResult};
pub use design::{Block, DesignError, Instance, Part, Ratings, Thermal};
pub use error::{SimError, SolvePoint};
pub use measure::{check_specs, Measure, Measurement, Spec, SpecReport};
pub use noise::{solve_noise, NoiseConfig, NoiseResult};
pub use ratings::{check_ratings, RatingCheck, RatingKind, RatingStatus, RatingsReport};
pub use results::{
    BjtResult, ComponentResult, ConvergenceAid, McuResult, MosfetResult, OpAmpResult, RelayResult,
    SimulationResult, TimestepSnapshot, TransientData, TransientStats,
};
pub use sensitivity::{solve_sensitivity, SensConfig, SensEntry, SensParameter, SensResult};
pub use solver::{Solver, SolverOptions};
pub use temp_sweep::{temperature_sweep, TempSweepPoint, TempSweepResult};
pub use transfer_function::{solve_tf, TfConfig, TfOutput, TfResult};
pub use transient::{solve_transient, IntegrationMethod, TransientConfig, TransientStart};
pub use validation::validate_circuit;
pub use waveform::Waveform;

// Device physics re-exports from sindr-devices — convenience for callers
// constructing CircuitElement variants without an extra crate import.
/// Battery equivalent circuits and presets, for [`CircuitElement::Battery`].
pub use sindr_devices::battery;
pub use sindr_devices::bjt::BjtKind;
pub use sindr_devices::igbt::IgbtParams;
pub use sindr_devices::jfet::JfetKind;
pub use sindr_devices::mosfet::MosfetKind;
/// Linear regulator parameters and presets, for [`CircuitElement::VoltageRegulator`]'s `model`.
pub use sindr_devices::regulator;
pub use sindr_devices::regulator::RegulatorParams;
pub use sindr_devices::varactor::VaractorParams;

pub(crate) use mna::MnaSystem;
use node_map::NodeMap as NM;

/// The circuit the solver runs: `circuit` flattened, with each
/// [`CircuitElement::ZenerDiode`] as the SPICE diode it stands for
/// ([`device_params::zener_model`]) and each MOSFET body diode as a
/// [`CircuitElement::Diode`] `<id>.body`. Every analysis starts here, so they
/// all see one zener model; ratings and export keep the zener as it is.
pub(crate) fn solver_circuit(circuit: &Circuit) -> Result<std::borrow::Cow<'_, Circuit>, SimError> {
    let flat = circuit.flattened()?;
    let lowers = |c: &CircuitElement| {
        composite::is_composite(c)
            || matches!(
                c,
                CircuitElement::ZenerDiode { .. }
                    | CircuitElement::Mosfet {
                        body_diode: Some(_),
                        ..
                    }
            )
    };
    if !flat.components.iter().any(lowers) {
        return Ok(flat);
    }
    let mut owned = flat.into_owned();
    let mut lowered = Vec::with_capacity(owned.components.len());
    for element in std::mem::take(&mut owned.components) {
        match composite::lower(&element)? {
            Some(parts) => lowered.extend(parts),
            None => lowered.push(element),
        }
    }
    owned.components = lowered;
    let mut seen = std::collections::HashSet::new();
    for element in &owned.components {
        if !seen.insert(element.id()) {
            return Err(SimError::InvalidComponent(format!(
                "`{}`: two components share this id once switches, crystals and beads \
                 are split into their parts",
                element.id()
            )));
        }
    }
    let mut body_diodes = Vec::new();
    for element in &mut owned.components {
        if let CircuitElement::Mosfet {
            id,
            nodes,
            kind,
            body_diode,
            ..
        } = element
        {
            if let Some(model) = body_diode.take() {
                let (anode, cathode) = match kind {
                    MosfetKind::Nmos => (&nodes[2], &nodes[1]),
                    MosfetKind::Pmos => (&nodes[1], &nodes[2]),
                };
                body_diodes.push(CircuitElement::Diode {
                    id: format!("{id}.body"),
                    nodes: [anode.clone(), cathode.clone()],
                    temperature: sindr_devices::diode::T_NOM,
                    model: Some(model),
                });
            }
        }
    }
    for diode in &body_diodes {
        if owned.components.iter().any(|c| c.id() == diode.id()) {
            return Err(SimError::InvalidComponent(format!(
                "`{}`: a component already has the id a body diode is solved under",
                diode.id()
            )));
        }
    }
    owned.components.extend(body_diodes);
    for element in &mut owned.components {
        if let CircuitElement::ZenerDiode {
            id,
            nodes,
            vz,
            temperature,
        } = element
        {
            if !(vz.is_finite() && *vz > 0.0) {
                return Err(SimError::InvalidComponent(format!(
                    "`{id}`: vz {vz} V must be a positive, finite number"
                )));
            }
            *element = CircuitElement::Diode {
                id: id.clone(),
                nodes: nodes.clone(),
                temperature: *temperature,
                model: Some(device_params::zener_model(*vz)),
            };
        }
    }
    Ok(std::borrow::Cow::Owned(owned))
}

/// The automatic transient config [`solve_circuit`] runs, checked like a
/// user's: a circuit whose time constants and waveforms ask for more than
/// `opts.max_transient_steps` gets an error naming them, not a run that
/// never ends.
fn auto_transient_config(
    circuit: &Circuit,
    opts: &SolverOptions,
) -> Result<TransientConfig, SimError> {
    let config = TransientConfig::auto(circuit);
    config
        .validate(opts.max_transient_steps)
        .map_err(|e| match e {
            SimError::InvalidComponent(msg) => SimError::InvalidComponent(format!(
                "{msg} (automatic span {} s and step {} s; set them with solve_transient)",
                config.tstop, config.tstep
            )),
            other => other,
        })?;
    Ok(config)
}

/// Solves a circuit end-to-end and returns voltages, currents, and power for
/// every component.
///
/// The solver picks the analysis path automatically based on what the circuit
/// contains:
///
/// | Circuit contains | Path |
/// |---|---|
/// | Only resistors + sources | Linear DC (single matrix solve) |
/// | Diodes / BJTs / MOSFETs / etc. | Nonlinear DC (Newton–Raphson) |
/// | Capacitors / inductors / waveforms | Transient (Gear-2) |
/// | Reactive **and** nonlinear | Transient nonlinear |
///
/// For frequency-domain analysis, use [`ac_analysis::solve_ac`] directly.
/// For parameter sweeps, see [`fn@dc_sweep`] and [`temperature_sweep`].
///
/// # Errors
///
/// Returns [`SimError`] if the circuit fails validation (no ground node,
/// floating nodes, invalid component values, etc.) or if the solver itself
/// fails to converge or produces a singular matrix. See [`SimError`] for the
/// full list. A transient whose automatic span and step would take more than
/// 10⁷ steps is [`SimError::InvalidComponent`]; run it with
/// [`solve_transient`] and a coarser [`TransientConfig`].
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement, solve_circuit};
///
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource {
///             id: "V1".into(),
///             nodes: ["n1".into(), "0".into()],
///             voltage: 5.0,
///             waveform: None,
///         },
///         CircuitElement::Resistor {
///             id: "R1".into(),
///             nodes: ["n1".into(), "0".into()],
///             resistance: 1_000.0,
///         },
///     ],
///     ..Default::default()
/// };
///
/// let result = solve_circuit(&circuit).unwrap();
/// // I = V/R = 5 mA flowing through R1
/// let r1 = result.component_results.iter().find(|c| c.id == "R1").unwrap();
/// assert!((r1.current_through.abs() - 0.005).abs() < 1e-9);
/// ```
pub fn solve_circuit(circuit: &Circuit) -> Result<SimulationResult, SimError> {
    solve_circuit_with(circuit, &SolverOptions::default())
}

/// [`solve_circuit`] with `opts`.
pub(crate) fn solve_circuit_with(
    circuit: &Circuit,
    opts: &SolverOptions,
) -> Result<SimulationResult, SimError> {
    let mut result = solve_circuit_parts(circuit, opts)?;
    composite::fold(circuit, &mut result);
    Ok(result)
}

/// [`solve_circuit_with`], with composite elements still in their parts.
fn solve_circuit_parts(
    circuit: &Circuit,
    opts: &SolverOptions,
) -> Result<SimulationResult, SimError> {
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    validation::validate_circuit(circuit)?;

    let node_map = NM::from_circuit(circuit);
    let num_nodes = node_map.num_nodes();
    let num_vsources = circuit.count_voltage_sources();

    if circuit.has_reactive_elements() || circuit.has_waveform_sources() {
        // The transient path doesn't consume the nodeset.
        let config = auto_transient_config(circuit, opts)?;
        return transient::run_transient(
            circuit,
            &node_map,
            num_nodes,
            num_vsources,
            &config,
            opts,
        );
    }

    solve_dc_heated(circuit, |circuit| {
        if circuit.has_nonlinear_elements() {
            newton_raphson::solve_nonlinear(circuit, &node_map, num_nodes, num_vsources, opts)
        } else {
            let mut system = MnaSystem::new(num_nodes, num_vsources);
            stamp::stamp_circuit(circuit, &mut system, &node_map, None)?;
            let solution = system.solve()?;
            Ok(results::extract_results(
                circuit, &node_map, &solution, num_nodes,
            ))
        }
    })
}

/// `solve` on flat `circuit`, iterated to a self-heating balance when any
/// part has [`Thermal`] data.
pub(crate) fn solve_dc_heated(
    circuit: &Circuit,
    mut solve: impl FnMut(&Circuit) -> Result<SimulationResult, SimError>,
) -> Result<SimulationResult, SimError> {
    match thermal::Heated::from_circuit(circuit)? {
        Some(heated) => thermal::solve_dc(circuit, &heated, solve),
        None => solve(circuit),
    }
}

/// DC operating point of flat `circuit` with capacitors open and inductors
/// shorted, whatever else it contains, at its self-heating balance (the
/// junction temperatures [`solve_circuit`] reports). [`solve_circuit`]
/// sends any reactive circuit down the transient path, whose end state
/// (5 τ) is only close to the bias point; small-signal analysis needs the
/// bias point itself. `opts.nodeset` seeds Newton–Raphson.
pub(crate) fn solve_dc_operating_point(
    circuit: &Circuit,
    opts: &SolverOptions,
) -> Result<SimulationResult, SimError> {
    validation::validate_circuit(circuit)?;
    solve_dc_heated(&dc_equivalent(circuit), |dc| solve_dc_circuit(dc, opts))
}

/// The DC solution of `dc`, a circuit with no reactive elements left (as
/// [`dc_equivalent`] gives), without self-heating. `opts.nodeset` seeds
/// Newton–Raphson.
pub(crate) fn solve_dc_circuit(
    dc: &Circuit,
    opts: &SolverOptions,
) -> Result<SimulationResult, SimError> {
    let node_map = NM::from_circuit(dc);
    let num_nodes = node_map.num_nodes();
    let num_vsources = dc.count_voltage_sources();
    if dc.has_nonlinear_elements() {
        newton_raphson::solve_nonlinear(dc, &node_map, num_nodes, num_vsources, opts)
    } else {
        let mut system = MnaSystem::new(num_nodes, num_vsources);
        stamp::stamp_circuit(dc, &mut system, &node_map, None)?;
        let solution = system.solve()?;
        Ok(results::extract_results(
            dc, &node_map, &solution, num_nodes,
        ))
    }
}

/// `circuit` as it stands at DC: capacitors and varactors open (a 1 TΩ leak), inductors and
/// transformer windings shorted (0 V sources). The bias point and the `.tf`
/// small-signal system are both solved on it. It keeps `circuit`'s part
/// data, which carries the thermal data the heated bias point needs.
pub(crate) fn dc_equivalent(circuit: &Circuit) -> Circuit {
    Circuit {
        ground_node: circuit.ground_node.clone(),
        components: circuit
            .components
            .iter()
            .flat_map(|c| {
                vec![match c {
                    // A 1 TΩ leak rather than nothing, so a node between two
                    // capacitors or varactors isn't singular. The same leak as
                    // the varactor's DC stamp, which `.tf`'s ω = 0 small-signal
                    // stamp (its capacitance only) would otherwise drop.
                    CircuitElement::Capacitor { id, nodes, .. }
                    | CircuitElement::Varactor { id, nodes, .. } => CircuitElement::Resistor {
                        id: id.clone(),
                        nodes: nodes.clone(),
                        resistance: 1e12,
                    },
                    // An exact short: a 0 V source. A tiny resistor would put a
                    // ~1e9 S entry beside milli-siemens ones and cost the LU
                    // about twelve digits.
                    CircuitElement::Inductor { id, nodes, .. } => CircuitElement::VoltageSource {
                        id: id.clone(),
                        nodes: nodes.clone(),
                        voltage: 0.0,
                        waveform: None,
                    },
                    // Both windings shorted, one 0 V source each: they fill the
                    // transformer's two branch rows, in its place.
                    CircuitElement::Transformer { id, nodes, .. } => {
                        let [p1, q1, p2, q2] = nodes.clone();
                        return vec![
                            CircuitElement::VoltageSource {
                                id: id.clone(),
                                nodes: [p1, q1],
                                voltage: 0.0,
                                waveform: None,
                            },
                            CircuitElement::VoltageSource {
                                id: format!("{id}#2"),
                                nodes: [p2, q2],
                                voltage: 0.0,
                                waveform: None,
                            },
                        ];
                    }
                    other => other.clone(),
                }]
            })
            .collect(),
        parts: circuit.parts.clone(),
        globals: circuit.globals.clone(),
        ..Default::default()
    }
}

/// Like [`solve_circuit`] but seeds the Newton–Raphson initial guess with
/// the supplied per-node voltages (V), keyed by node name, as SPICE's
/// `.NODESET`. Use a [`Solver`] whose [`SolverOptions::nodeset`] holds them.
///
/// # Errors
///
/// As [`solve_circuit`].
#[deprecated(
    since = "0.1.0",
    note = "use `Solver::new(SolverOptions::default().with_nodeset(..)).circuit(..)`"
)]
pub fn solve_circuit_with_initial_voltages(
    circuit: &Circuit,
    initial_voltages: &std::collections::HashMap<String, f64>,
) -> Result<SimulationResult, SimError> {
    solve_circuit_with(
        circuit,
        &SolverOptions::default().with_nodeset(initial_voltages.clone()),
    )
}
