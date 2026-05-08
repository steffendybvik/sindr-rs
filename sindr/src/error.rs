//! Error type returned by the solver.

use thiserror::Error;

/// Anything that can go wrong while validating or solving a circuit.
///
/// Returned by [`solve_circuit`](crate::solve_circuit), [`dc_sweep`](fn@crate::dc_sweep),
/// [`temperature_sweep`](crate::temperature_sweep), and the AC analysis
/// entry point.
///
/// Variants split into three groups:
///
/// - **Topology / validation errors** ([`NoGround`](Self::NoGround),
///   [`DisconnectedNodes`](Self::DisconnectedNodes),
///   [`FloatingNode`](Self::FloatingNode)) — caught before solving.
/// - **Numerical errors** ([`SingularMatrix`](Self::SingularMatrix),
///   [`InvalidSolution`](Self::InvalidSolution),
///   [`ConvergenceFailed`](Self::ConvergenceFailed)) — the matrix has no
///   unique solution or Newton–Raphson failed to converge.
/// - **Configuration errors** ([`InvalidComponent`](Self::InvalidComponent),
///   [`InvalidResistance`](Self::InvalidResistance)) — a component value
///   that the solver explicitly rejects.
#[derive(Debug, Error)]
pub enum SimError {
    /// No ground node was defined. Every circuit needs a reference node.
    #[error(
        "No ground node defined in circuit. Every circuit needs a ground (reference) node \
         — try connecting a component terminal to node \"0\"."
    )]
    NoGround,

    /// One or more nodes have no path to the rest of the circuit.
    #[error(
        "These nodes are not connected to the rest of the circuit: {0:?}. \
         Make sure every node has a path to ground through components."
    )]
    DisconnectedNodes(Vec<String>),

    /// A node has fewer than two component terminals attached and so can't
    /// carry current.
    #[error(
        "Node \"{0}\" has fewer than 2 connections and cannot carry current. \
         Connect it to at least 2 components."
    )]
    FloatingNode(String),

    /// The MNA matrix is singular — typically a voltage-source loop or a
    /// subcircuit that's not tied to ground.
    #[error(
        "Circuit matrix is singular — the circuit has no unique solution. \
         Common causes: voltage sources in a loop, or a subcircuit disconnected from ground."
    )]
    SingularMatrix,

    /// The solver produced NaN or infinity, usually due to component values
    /// spanning an extreme dynamic range.
    #[error(
        "Solver produced invalid values (NaN or Infinity). This usually means component \
         values span an extreme range. Try checking for very small or very large values."
    )]
    InvalidSolution,

    /// A component definition is malformed (wrong number of terminals,
    /// unknown id, etc.). Carries a human-readable description.
    #[error("Invalid component: {0}")]
    InvalidComponent(String),

    /// A resistor has zero or negative resistance.
    #[error(
        "Resistor \"{0}\" has zero or negative resistance, which is not physically meaningful."
    )]
    InvalidResistance(String),

    /// Newton–Raphson exhausted its iteration budget without converging.
    #[error(
        "Newton-Raphson failed to converge after 100 iterations. \
         The circuit may have no valid operating point."
    )]
    ConvergenceFailed,
}
