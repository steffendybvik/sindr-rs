use thiserror::Error;

#[derive(Debug, Error)]
pub enum SimError {
    #[error(
        "No ground node defined in circuit. Every circuit needs a ground (reference) node \
         — try connecting a component terminal to node \"0\"."
    )]
    NoGround,

    #[error(
        "These nodes are not connected to the rest of the circuit: {0:?}. \
         Make sure every node has a path to ground through components."
    )]
    DisconnectedNodes(Vec<String>),

    #[error(
        "Node \"{0}\" has fewer than 2 connections and cannot carry current. \
         Connect it to at least 2 components."
    )]
    FloatingNode(String),

    #[error(
        "Circuit matrix is singular — the circuit has no unique solution. \
         Common causes: voltage sources in a loop, or a subcircuit disconnected from ground."
    )]
    SingularMatrix,

    #[error(
        "Solver produced invalid values (NaN or Infinity). This usually means component \
         values span an extreme range. Try checking for very small or very large values."
    )]
    InvalidSolution,

    #[error("Invalid component: {0}")]
    InvalidComponent(String),

    #[error(
        "Resistor \"{0}\" has zero or negative resistance, which is not physically meaningful."
    )]
    InvalidResistance(String),

    #[error(
        "Newton-Raphson failed to converge after 100 iterations. \
         The circuit may have no valid operating point."
    )]
    ConvergenceFailed,

    #[error("Unsupported circuit configuration: {0}")]
    UnsupportedCircuit(String),
}
