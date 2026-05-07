//! Pure-Rust circuit simulator. SPICE-style MNA solver with first-class
//! semiconductor models.
//!
//! `sindr` solves a [`Circuit`] — a list of components and a ground node —
//! and returns voltages, currents, and power for every component. The solver
//! picks the right path automatically:
//!
//! - **Linear DC** — direct solve of the MNA system.
//! - **Nonlinear DC** — Newton–Raphson when diodes, BJTs, MOSFETs, etc. are present.
//! - **Transient** — backward-Euler timestepping when capacitors, inductors,
//!   or time-varying sources are present.
//! - **AC small-signal** — sinusoidal-steady-state via [`ac_analysis::solve_ac`].
//! - **DC sweep** — parameter sweep over a component value via [`fn@dc_sweep`].
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
//! Build a voltage divider, solve it, read the divided voltage:
//!
//! ```
//! use sindr::{Circuit, CircuitElement, solve_circuit};
//!
//! let circuit = Circuit {
//!     ground_node: "0".into(),
//!     components: vec![
//!         CircuitElement::VoltageSource {
//!             id: "V1".into(),
//!             nodes: ["n1".into(), "0".into()],
//!             voltage: 10.0,
//!             waveform: None,
//!         },
//!         CircuitElement::Resistor {
//!             id: "R1".into(),
//!             nodes: ["n1".into(), "n2".into()],
//!             resistance: 1_000.0,
//!         },
//!         CircuitElement::Resistor {
//!             id: "R2".into(),
//!             nodes: ["n2".into(), "0".into()],
//!             resistance: 2_000.0,
//!         },
//!     ],
//! };
//!
//! let result = solve_circuit(&circuit).unwrap();
//!
//! // V_n2 = 10 V * R2/(R1+R2) = 10 * 2/3 ≈ 6.667 V
//! let v_n2 = result.node_voltages["n2"];
//! assert!((v_n2 - 6.6667).abs() < 1e-3);
//! ```
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
//!   types. Disable for embedded / no-allocator targets.
//! - `examples` — exposes the `examples` module with built-in named
//!   circuits (voltage divider, BJT amp, RC transient, etc.).
//!
//! ```toml
//! [dependencies]
//! sindr = "0.1"
//!
//! # No serde:
//! sindr = { version = "0.1", default-features = false }
//! ```
//!
//! # Where to look next
//!
//! - [`Circuit`] / [`CircuitElement`] — the input format
//! - [`solve_circuit`] — the headline entry point
//! - [`SimulationResult`] — what you get back
//! - [`Waveform`] — time-varying source shapes
//! - [`SimError`] — what can go wrong

pub mod examples;
#[cfg(feature = "examples")]
pub use examples::{get_examples, ExampleCircuit};

pub mod ac_analysis;
pub mod circuit;
pub mod dc_sweep;
pub mod error;
pub mod mna;
pub mod newton_raphson;
pub mod node_map;
pub mod results;
pub mod stamp;
pub mod temp_sweep;
pub mod transient;
pub mod validation;
pub mod waveform;

pub use circuit::{Circuit, CircuitElement};
pub use dc_sweep::{dc_sweep, DcSweepResult};
pub use error::SimError;
pub use mna::MnaSystem;
pub use node_map::NodeMap;
pub use results::{
    BjtResult, ComponentResult, McuResult, MosfetResult, OpAmpResult, RelayResult,
    SimulationResult, TimestepSnapshot, TransientData,
};
pub use temp_sweep::{temperature_sweep, TempSweepPoint, TempSweepResult};
pub use validation::validate_circuit;
pub use waveform::Waveform;

// Device physics re-exports from sindr-devices — convenience for callers
// constructing CircuitElement variants without an extra crate import.
pub use sindr_devices::bjt::BjtKind;
pub use sindr_devices::igbt::IgbtParams;
pub use sindr_devices::jfet::JfetKind;
pub use sindr_devices::mosfet::MosfetKind;
pub use sindr_devices::varactor::VaractorParams;

use node_map::NodeMap as NM;

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
/// | Capacitors / inductors / waveforms | Transient (backward Euler) |
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
/// full list.
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
/// };
///
/// let result = solve_circuit(&circuit).unwrap();
/// // I = V/R = 5 mA flowing through R1
/// let r1 = result.component_results.iter().find(|c| c.id == "R1").unwrap();
/// assert!((r1.current_through.abs() - 0.005).abs() < 1e-9);
/// ```
pub fn solve_circuit(circuit: &Circuit) -> Result<SimulationResult, SimError> {
    validation::validate_circuit(circuit)?;

    let node_map = NM::from_circuit(circuit);
    let num_nodes = node_map.num_nodes();
    let num_vsources = circuit.count_voltage_sources();

    if circuit.has_reactive_elements() || circuit.has_waveform_sources() {
        if circuit.has_nonlinear_elements() {
            return transient::solve_transient_nonlinear(
                circuit,
                &node_map,
                num_nodes,
                num_vsources,
            );
        }
        return transient::solve_transient(circuit, &node_map, num_nodes, num_vsources);
    }

    if circuit.has_nonlinear_elements() {
        newton_raphson::solve_nonlinear(circuit, &node_map, num_nodes, num_vsources)
    } else {
        let mut system = MnaSystem::new(num_nodes, num_vsources);
        stamp::stamp_circuit(circuit, &mut system, &node_map, None)?;
        let solution = system.solve()?;
        Ok(results::extract_results(
            circuit, &node_map, &solution, num_nodes,
        ))
    }
}
