//! A behavioural source: a voltage or current set by an expression.

use crate::circuit::BehaviouralOutput;
use crate::expression::Expression;

use super::Device;

/// Behavioural source, SPICE's `B n+ n- V={expr}` or `I={expr}`: a
/// voltage or current set by an [`Expression`] of node voltages,
/// branch currents and time.
///
/// A voltage output holds `V(nodes[0]) − V(nodes[1])` at the
/// expression, and like a [`CircuitElement::VoltageSource`] has a
/// branch current others can read with `i(id)`. A current output
/// drives the expression's value through the source from `nodes[0]` to
/// `nodes[1]`, as a [`CircuitElement::CurrentSource`] does. Newton
/// stamps it with the expression's exact gradient, AC analysis
/// linearises it at the bias point, and `time` reads the transient's
/// time (0 elsewhere).
///
/// ```
/// use sindr::{BehaviouralOutput, CircuitElement};
/// use sindr::expression::Expression;
/// use sindr::element::Behavioural;
///
/// let multiplier = CircuitElement::Behavioural(Behavioural {
///     id: "b1".into(),
///     nodes: ["out".into(), "0".into()],
///     output: BehaviouralOutput::Voltage,
///     expr: Expression::parse("v(a)*v(b)")?,
/// });
/// # Ok::<(), sindr::expression::ExpressionError>(())
/// ```
///
/// [`CircuitElement::VoltageSource`]: crate::CircuitElement::VoltageSource
/// [`CircuitElement::CurrentSource`]: crate::CircuitElement::CurrentSource
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Behavioural {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[n+, n-]`.
    pub nodes: [String; 2],
    /// Whether the expression is a voltage or a current.
    pub output: BehaviouralOutput,
    /// The value, in volts or amperes.
    pub expr: Expression,
}

impl Device for Behavioural {
    device_identity!();

    /// The nodes the expression reads count as connections too.
    fn all_nodes(&self) -> Vec<&String> {
        let mut all: Vec<&String> = self.nodes.iter().collect();
        for node in self.expr.nodes() {
            if !all.contains(&node) {
                all.push(node);
            }
        }
        all
    }

    /// A voltage output's branch current; a current output adds none.
    fn branches(&self) -> usize {
        match self.output {
            BehaviouralOutput::Voltage => 1,
            BehaviouralOutput::Current => 0,
        }
    }

    fn is_nonlinear(&self) -> bool {
        true
    }
}
