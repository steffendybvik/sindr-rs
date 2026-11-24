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
#[non_exhaustive]
pub enum SimError {
    /// The design hierarchy can't be flattened: an unknown block, a
    /// port-count mismatch, a recursive block or a repeated id.
    #[error("{0}")]
    Design(#[from] crate::DesignError),

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

    /// A net no component names: a typo in a [`CircuitBuilder`] element, or
    /// a name [`Circuit::node_id`] can't find.
    ///
    /// [`CircuitBuilder`]: crate::CircuitBuilder
    /// [`Circuit::node_id`]: crate::Circuit::node_id
    #[error("{}", unknown_node_message(node, component.as_deref()))]
    UnknownNode {
        /// The net's name.
        node: String,
        /// The component that names it, when one does.
        component: Option<String>,
    },

    /// A component id nothing in the circuit has, as
    /// [`Circuit::component_id`] looks it up.
    ///
    /// [`Circuit::component_id`]: crate::Circuit::component_id
    #[error("No component \"{0}\" in the circuit.")]
    UnknownComponent(String),

    /// A resistor's resistance is zero, negative or not finite.
    #[error(
        "Resistor \"{0}\" has a zero, negative or non-finite resistance, which is not physically meaningful."
    )]
    InvalidResistance(String),

    /// Newton–Raphson exhausted its iteration budget without converging,
    /// in DC after the gmin- and source-stepping fallbacks failed too, or in
    /// transient after halving the step down to its floor.
    ///
    /// Says where the circuit is stuck: the node whose voltage was still
    /// moving most on the last iteration, the components on that node, and
    /// the point of the analysis that failed. The message names them, so a
    /// designer or an agent knows which part to look at.
    ///
    /// A final step near the convergence tolerance suggests slow
    /// convergence: try initial conditions (`.nodeset`) or scaling values. A
    /// large one usually means there's no valid operating point there, such
    /// as a nonlinear element with no DC path to ground.
    #[error(
        "Newton-Raphson failed to converge at {at} after {iterations} iterations{}. \
         The circuit may have no valid operating point there.",
        stuck(.node, .components, .max_step_volts)
    )]
    ConvergenceFailed {
        /// Number of Newton iterations executed before giving up.
        iterations: usize,
        /// Largest per-node Newton step on the final iteration (V): the max
        /// over nodes of `|V_new − V_prev|`. A Newton step, not a KCL
        /// residual.
        max_step_volts: f64,
        /// The node that took that step: where the circuit won't settle.
        /// `None` only when no iteration ran.
        node: Option<String>,
        /// Ids of the components with a terminal on [`node`](Self::ConvergenceFailed::node),
        /// in circuit order, with blocks flattened (`X1.R2`).
        components: Vec<String>,
        /// Where in the analysis it failed.
        at: SolvePoint,
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

    /// A transient's step stayed far below `tstep` (three decades or more)
    /// for thousands of steps in a row, the solution still changing too
    /// fast to follow. A node driven with no DC path to ground (a current
    /// source into a capacitor or an open end) ramps without limit like
    /// this. SPICE's "timestep too small". A run whose step settles a
    /// little higher stops this way too once it has taken
    /// [`max_transient_steps`](crate::SolverOptions::max_transient_steps)
    /// steps short of `tstop`.
    #[error(
        "time step too small at t = {time:.3e} s: the step stayed near {step:.1e} s and \
         the solution still changes too fast to follow. Check for a node driven with no \
         DC path to ground, such as a current source into a capacitor or an open end."
    )]
    TimestepTooSmall {
        /// Simulated time when the run stopped (s).
        time: f64,
        /// The last step taken (s).
        step: f64,
    },

    /// A progress hook asked the run to stop
    /// ([`solve_transient_with_progress`](crate::solve_transient_with_progress)).
    #[error("cancelled at t = {time:.3e} s")]
    Cancelled {
        /// Simulated time of the last accepted step (s).
        time: f64,
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

    /// A [`Fourier`](crate::fourier::Fourier) analysis or
    /// [`Fft`](crate::fourier::Fft) couldn't be taken: no transient, an
    /// unknown signal, a run shorter than one period, or settings it can't
    /// use.
    #[error("spectrum: {reason}")]
    Spectrum {
        /// Why it couldn't be taken.
        reason: String,
    },

    /// A behavioural source's expression isn't finite (it overflowed, or
    /// left its domain) at the voltages and currents it read.
    ///
    /// During a solve this is a failed iterate, not a final answer: Newton
    /// shortens the step, and gmin and source stepping, or a shorter
    /// transient step, are tried before this is returned.
    #[error("`{component}`'s expression `{expression}` isn't finite at {inputs}")]
    NotFinite {
        /// The behavioural source.
        component: String,
        /// Its expression.
        expression: String,
        /// What it read, as `v(a) = 20, time = 0`.
        inputs: String,
    },
}

impl SimError {
    /// A failure a smaller step or a convergence aid can get past.
    pub(crate) fn is_retryable(&self) -> bool {
        matches!(
            self,
            SimError::ConvergenceFailed { .. } | SimError::NotFinite { .. }
        )
    }
}

/// Where in an analysis a solve failed, carried by
/// [`SimError::ConvergenceFailed`].
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SolvePoint {
    /// The DC operating point, including one AC, `.tf`, `.sens` or `.noise`
    /// linearise around and the one a transient starts from.
    OperatingPoint,
    /// A DC sweep point: `source` set to `value` (V or A).
    DcSweep {
        /// The swept source's id.
        source: String,
        /// Its value at the failing point.
        value: f64,
    },
    /// A temperature sweep point (K).
    Temperature {
        /// The device temperature at the failing point (K).
        kelvin: f64,
    },
    /// A transient step to `time` (s), after the step was halved to its
    /// floor.
    Transient {
        /// The time the failing step was solving for (s).
        time: f64,
    },
}

impl std::fmt::Display for SolvePoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SolvePoint::OperatingPoint => write!(f, "the DC operating point"),
            SolvePoint::DcSweep { source, value } => {
                write!(f, "DC sweep point {source} = {value:.6e}")
            }
            SolvePoint::Temperature { kelvin } => {
                write!(f, "temperature sweep point {kelvin:.2} K")
            }
            SolvePoint::Transient { time } => write!(f, "transient time {time:.6e} s"),
        }
    }
}

impl SimError {
    /// This error with a convergence failure's [`SolvePoint`] set to
    /// `point`. Other errors pass through unchanged.
    pub(crate) fn at(self, point: SolvePoint) -> Self {
        match self {
            SimError::ConvergenceFailed {
                iterations,
                max_step_volts,
                node,
                components,
                ..
            } => SimError::ConvergenceFailed {
                iterations,
                max_step_volts,
                node,
                components,
                at: point,
            },
            other => other,
        }
    }
}

/// The message's "where": ": node `x` (on D1, R2) still moving 1.2e-1 V".
fn stuck(node: &Option<String>, components: &[String], step: &f64) -> String {
    /// More than this many components are elided.
    const SHOWN: usize = 6;
    let Some(node) = node else {
        return String::new();
    };
    let mut on = components
        .iter()
        .take(SHOWN)
        .map(String::as_str)
        .collect::<Vec<_>>()
        .join(", ");
    if components.len() > SHOWN {
        on.push_str(&format!(" and {} more", components.len() - SHOWN));
    }
    let on = if on.is_empty() {
        String::new()
    } else {
        format!(" (on {on})")
    };
    format!(": node `{node}`{on} was still moving {step:.3e} V per iteration")
}

fn unknown_node_message(node: &str, component: Option<&str>) -> String {
    match component {
        Some(c) => format!(
            "\"{c}\" connects to net \"{node}\", which was never declared. \
             Declare it with `CircuitBuilder::node`, or check the name for a typo."
        ),
        None => format!("No net \"{node}\" in the circuit."),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_handles_name_what_is_missing() {
        let from_builder = SimError::UnknownNode {
            node: "vout".into(),
            component: Some("R1".into()),
        };
        assert_eq!(
            from_builder.to_string(),
            "\"R1\" connects to net \"vout\", which was never declared. \
             Declare it with `CircuitBuilder::node`, or check the name for a typo."
        );
        let lookup = SimError::UnknownNode {
            node: "vout".into(),
            component: None,
        };
        assert_eq!(lookup.to_string(), "No net \"vout\" in the circuit.");
        assert_eq!(
            SimError::UnknownComponent("Q9".into()).to_string(),
            "No component \"Q9\" in the circuit."
        );
    }

    #[test]
    fn convergence_failed_carries_diagnostics() {
        let err = SimError::ConvergenceFailed {
            iterations: 100,
            max_step_volts: 1.234e-3,
            node: None,
            components: Vec::new(),
            at: SolvePoint::OperatingPoint,
        };

        match err {
            SimError::ConvergenceFailed {
                iterations,
                max_step_volts,
                ..
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
            node: Some("out".into()),
            components: (1..=8).map(|i| format!("D{i}")).collect(),
            at: SolvePoint::Transient { time: 1.5e-3 },
        };
        let msg = err.to_string();
        assert!(msg.contains("42 iterations"), "{msg}");
        assert!(msg.contains("7.500e-4 V"), "{msg}");
        assert!(msg.contains("node `out`"), "{msg}");
        assert!(
            msg.contains("(on D1, D2, D3, D4, D5, D6 and 2 more)"),
            "{msg}"
        );
        assert!(msg.contains("transient time 1.500000e-3 s"), "{msg}");
    }

    #[test]
    fn at_sets_the_point_and_leaves_other_errors_alone() {
        let err = SimError::ConvergenceFailed {
            iterations: 1,
            max_step_volts: 1.0,
            node: None,
            components: Vec::new(),
            at: SolvePoint::OperatingPoint,
        }
        .at(SolvePoint::DcSweep {
            source: "V1".into(),
            value: 2.0,
        });
        assert!(err.to_string().contains("DC sweep point V1 = 2.000000e0"));
        assert!(matches!(
            SimError::SingularMatrix.at(SolvePoint::OperatingPoint),
            SimError::SingularMatrix
        ));
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

    #[test]
    fn convergence_message_at_each_point_and_without_components() {
        let failed =
            |node: Option<&str>, components: Vec<String>, at| SimError::ConvergenceFailed {
                iterations: 3,
                max_step_volts: 0.5,
                node: node.map(String::from),
                components,
                at,
            };
        assert_eq!(
            failed(Some("x"), vec![], SolvePoint::OperatingPoint).to_string(),
            "Newton-Raphson failed to converge at the DC operating point after 3 iterations: \
             node `x` was still moving 5.000e-1 V per iteration. \
             The circuit may have no valid operating point there."
        );
        assert_eq!(
            failed(None, vec![], SolvePoint::Temperature { kelvin: 398.15 }).to_string(),
            "Newton-Raphson failed to converge at temperature sweep point 398.15 K after \
             3 iterations. The circuit may have no valid operating point there."
        );
        let many: Vec<String> = (1..=8).map(|k| format!("R{k}")).collect();
        let msg = failed(Some("x"), many, SolvePoint::OperatingPoint).to_string();
        assert!(
            msg.contains("node `x` (on R1, R2, R3, R4, R5, R6 and 2 more)"),
            "{msg}"
        );
    }
}
