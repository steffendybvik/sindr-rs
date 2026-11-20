//! Error type returned by the solver.

use thiserror::Error;

/// Anything that can go wrong while validating or solving a circuit.
///
/// Returned by [`solve_circuit`](crate::solve_circuit),
/// [`solve_transient`](crate::solve_transient),
/// [`solve_ac`](crate::ac_analysis::solve_ac), [`solve_tf`](crate::solve_tf),
/// [`solve_sensitivity`](crate::solve_sensitivity),
/// [`dc_sweep`](fn@crate::dc_sweep) and
/// [`temperature_sweep`](crate::temperature_sweep).
///
/// Variants split into four groups:
///
/// - **Design errors** ([`Design`](Self::Design)) — the hierarchy can't be
///   flattened.
/// - **Topology / validation errors** ([`NoGround`](Self::NoGround),
///   [`DisconnectedNodes`](Self::DisconnectedNodes),
///   [`FloatingNode`](Self::FloatingNode)) — caught before solving.
/// - **Numerical errors** ([`SingularMatrix`](Self::SingularMatrix),
///   [`InvalidSolution`](Self::InvalidSolution),
///   [`ConvergenceFailed`](Self::ConvergenceFailed)) — the matrix has no
///   unique solution or Newton–Raphson failed to converge.
/// - **Configuration errors** ([`InvalidComponent`](Self::InvalidComponent),
///   [`InvalidResistance`](Self::InvalidResistance)) — a component value or
///   analysis setting that the solver explicitly rejects.
#[derive(Debug, Error)]
pub enum SimError {
    /// The design hierarchy can't be flattened: an unknown block, a
    /// port-count mismatch, a recursive block or a repeated id.
    #[error("{0}")]
    Design(#[from] sindr_core::DesignError),

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

    /// A component definition or analysis setting is malformed: a device
    /// `temperature` or BJT `bf` that isn't positive and finite, an id that
    /// names no suitable source or component, a non-finite or out-of-range
    /// sweep or transient setting, a transient of more than 10⁷ steps or a
    /// sweep of more than 10⁷ points, a bad initial condition. Carries a
    /// human-readable description naming the offending id or field.
    #[error("Invalid component: {0}")]
    InvalidComponent(String),

    /// A resistor has zero or negative resistance.
    #[error(
        "Resistor \"{0}\" has zero or negative resistance, which is not physically meaningful."
    )]
    InvalidResistance(String),

    /// Newton–Raphson exhausted its iteration budget without converging,
    /// in DC after the gmin- and source-stepping fallbacks failed too.
    ///
    /// Carries the iteration count reached and the largest per-node voltage
    /// step `max_i |Vᵢ_new − Vᵢ_prev|` (V) at the final iteration. A step
    /// near the convergence tolerance suggests slow convergence — try
    /// scaling component values or supplying initial conditions. A large
    /// step usually means the circuit has no valid operating point (e.g. a
    /// nonlinear element with no DC path to ground).
    ///
    /// Note: this is the maximum *Newton step* between consecutive
    /// iterations, not a KCL residual `|F(x)|`.
    #[error(
        "Newton-Raphson failed to converge after {iterations} iterations \
         (max node-voltage step {max_step_volts:.3e} V). \
         The circuit may have no valid operating point."
    )]
    ConvergenceFailed {
        /// Number of Newton iterations executed before giving up.
        iterations: usize,
        /// Largest per-node Newton step on the final iteration (V): the max
        /// over node indices of `|V_new[i] − V_prev[i]|`.
        max_step_volts: f64,
    },

    /// Self-heating has no stable balance: a part's dissipation heats its
    /// junction, which raises its current, faster than its thermal path can
    /// carry the heat away. The real part would be destroyed.
    #[error(
        "thermal runaway in `{part}`: its junction passed {temperature:.0} K with no stable \
         operating point. Lower its dissipation, add emitter degeneration, or give it a \
         lower thermal resistance (heatsink)."
    )]
    ThermalRunaway {
        /// The part that ran away (the hottest one whose temperature feeds
        /// back into its model).
        part: String,
        /// Its junction temperature when the solve gave up (K).
        temperature: f64,
    },

    /// A [`Measurement`](crate::measure::Measurement) couldn't be taken on
    /// a result: an unknown node or component, a window outside the
    /// result, or a level the signal never crosses.
    #[error("measurement `{name}`: {reason}")]
    Measurement {
        /// The measurement's name.
        name: String,
        /// Why it couldn't be taken.
        reason: String,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convergence_failed_carries_diagnostics() {
        let err = SimError::ConvergenceFailed {
            iterations: 100,
            max_step_volts: 1.234e-3,
        };

        // Pattern-destructure: both fields are accessible as named struct fields.
        match err {
            SimError::ConvergenceFailed {
                iterations,
                max_step_volts,
            } => {
                assert_eq!(iterations, 100);
                assert!((max_step_volts - 1.234e-3).abs() < 1e-12);
            }
            _ => panic!("expected ConvergenceFailed variant"),
        }
    }

    #[test]
    fn convergence_failed_display_includes_iterations_and_step() {
        let err = SimError::ConvergenceFailed {
            iterations: 42,
            max_step_volts: 7.5e-4,
        };
        let msg = err.to_string();
        assert!(
            msg.contains("42 iterations"),
            "Display should mention iteration count: {msg}"
        );
        assert!(
            msg.contains("7.500e-4") || msg.contains("step"),
            "Display should mention step size: {msg}"
        );
    }

    #[test]
    fn other_variants_still_format_cleanly() {
        // Sanity check that adding fields to one variant didn't break others.
        assert!(SimError::NoGround.to_string().contains("ground"));
        assert!(SimError::SingularMatrix.to_string().contains("singular"));
        assert!(SimError::DisconnectedNodes(vec!["n1".into(), "n2".into()])
            .to_string()
            .contains("n1"));
    }
}
