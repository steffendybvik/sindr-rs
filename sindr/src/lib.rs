//! MNA circuit solver: Newton-Raphson, transient, AC analysis, DC sweep.
//!
//! Depends on sindr-devices for linearised device companion models.

#[cfg(feature = "examples")]
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
pub use temp_sweep::{temperature_sweep, TempSweepPoint, TempSweepResult};
pub use error::SimError;
pub use mna::MnaSystem;
pub use node_map::NodeMap;
pub use results::{BjtResult, ComponentResult, McuResult, MosfetResult, OpAmpResult, RelayResult, SimulationResult, TimestepSnapshot, TransientData};
pub use validation::validate_circuit;
pub use waveform::Waveform;

// Device physics re-exports from sindr-devices — consumed by circuit.rs and stamp.rs
pub use sindr_devices::bjt::BjtKind;
pub use sindr_devices::mosfet::MosfetKind;
pub use sindr_devices::varactor::VaractorParams;
pub use sindr_devices::igbt::IgbtParams;
pub use sindr_devices::jfet::JfetKind;

use node_map::NodeMap as NM;

/// Solve a circuit end-to-end: build node map, stamp, solve, extract results.
pub fn solve_circuit(circuit: &Circuit) -> Result<SimulationResult, SimError> {
    validation::validate_circuit(circuit)?;

    let node_map = NM::from_circuit(circuit);
    let num_nodes = node_map.num_nodes();
    let num_vsources = circuit.count_voltage_sources();

    if circuit.has_reactive_elements() || circuit.has_waveform_sources() {
        if circuit.has_nonlinear_elements() {
            return transient::solve_transient_nonlinear(
                circuit, &node_map, num_nodes, num_vsources,
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
        Ok(results::extract_results(circuit, &node_map, &solution, num_nodes))
    }
}
