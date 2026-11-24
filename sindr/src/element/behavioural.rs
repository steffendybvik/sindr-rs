//! A behavioural source: a voltage or current set by an expression.

use crate::handle::NodeId;
use nalgebra::DVector;

use crate::circuit::{BehaviouralOutput, Circuit, CircuitElement};
use crate::error::SimError;
use crate::expression::Expression;
use crate::mna::MnaSystem;
use crate::node_map::NodeMap;
use crate::stamp::stamp_voltage_source_at;

use super::{AcStamp, DcStamp, Device, TranStamp};
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::expression::Variable;
#[cfg(feature = "spice")]
use crate::spice::export::{Deck, ExportError};

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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Behavioural {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[n+, n-]`.
    pub nodes: [NodeId; 2],
    /// Whether the expression is a voltage or a current.
    pub output: BehaviouralOutput,
    /// The value, in volts or amperes.
    pub expr: Expression,
}

impl Device for Behavioural {
    device_identity!();

    /// Its Jacobian at the bias point; the constant term is a DC source
    /// and drops out of the small-signal system. The bias solve checked
    /// the circuit, so this can't fail here.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let (jacobian, _) = behavioural_linearisation(
            s.circuit,
            s.node_map,
            &self.id,
            &self.expr,
            s.dc_solution,
            0.0,
        )
        .unwrap_or_default();
        let [p, q] = s.nodes();
        match self.output {
            BehaviouralOutput::Current => {
                for (k, g) in &jacobian {
                    let g = Complex64::new(*g, 0.0);
                    s.add(p, Some(*k), g);
                    s.add(q, Some(*k), -g);
                }
            }
            BehaviouralOutput::Voltage => {
                let branch = s.branch;
                s.branch_pair(branch, [p, q]);
                for (k, g) in &jacobian {
                    s.a[(branch, *k)] -= Complex64::new(*g, 0.0);
                }
            }
        }
    }

    /// A voltage output's two terminals; a current output fixes no
    /// voltage.
    fn dc_paths<'a>(&'a self, _: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        match self.output {
            BehaviouralOutput::Voltage => {
                let [p, q] = &self.nodes;
                vec![[p, q]]
            }
            BehaviouralOutput::Current => Vec::new(),
        }
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        // Linear first pass: linearised at all-zero unknowns.
        let zero;
        let x = match s.v_prev {
            Some(x) => x,
            None => {
                zero = DVector::zeros(s.system.b.len());
                &zero
            }
        };
        let nodes = s.nodes();
        stamp_behavioural_at(
            s.system,
            s.circuit,
            s.node_map,
            &self.id,
            nodes,
            self.output,
            &self.expr,
            x,
            0.0,
            s.branch,
        )
    }

    /// Its expression at the step's end, linearised at the iterate.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let nodes = s.nodes();
        stamp_behavioural_at(
            s.system,
            s.circuit,
            s.node_map,
            &self.id,
            nodes,
            self.output,
            &self.expr,
            s.v_prev,
            s.t,
            s.branch,
        )
    }

    /// The nodes the expression reads count as connections too.
    fn all_nodes(&self) -> Vec<&NodeId> {
        let mut all: Vec<&NodeId> = self.nodes.iter().collect();
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

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'B'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        // The expression reads nodes and sources by their deck names.
        let mut deck_names = std::collections::HashMap::new();
        for variable in self.expr.variables() {
            match variable {
                Variable::Voltage { node, reference } => {
                    for n in std::iter::once(node).chain(reference) {
                        deck_names.insert(n.to_string(), deck.node(n)?);
                    }
                }
                Variable::Current { source } => {
                    let name = deck.name_of(source).map(str::to_string).ok_or_else(|| {
                        ExportError::BadName {
                            what: "element",
                            name: self.id.clone(),
                            reason: format!(
                                "its expression reads i({source}), which isn't in the circuit"
                            ),
                        }
                    })?;
                    deck_names.insert(source.to_string(), name);
                }
                _ => {}
            }
        }
        let mut expr = self.expr.clone();
        expr.rename(|n| deck_names[n].clone(), |s| deck_names[s].clone());
        let key = match self.output {
            BehaviouralOutput::Current => 'I',
            _ => 'V',
        };
        let nodes = deck.nodes(&self.nodes)?;
        deck.line(format!("{name} {nodes} {key}={{{expr}}}"));
        Ok(())
    }
}

/// Checks every behavioural expression is finite at the solution `x`.
///
/// Newton accepts its last step without stamping it, so a source whose
/// expression left its domain on that step (`asin` just past 1) would
/// otherwise be reported at a point where it has no value.
///
/// # Errors
///
/// [`SimError::NotFinite`] for the first one that isn't.
pub(crate) fn behavioural_finite_at(
    circuit: &Circuit,
    node_map: &NodeMap,
    x: &DVector<f64>,
    t: f64,
) -> Result<(), SimError> {
    for component in &circuit.components {
        if let CircuitElement::Behavioural(Behavioural { id, expr, .. }) = component {
            behavioural_linearisation(circuit, node_map, id, expr, x, t)?;
        }
    }
    Ok(())
}

/// A behavioural source's expression linearised at `x` (the MNA unknowns)
/// and time `t`: `f(x) ≈ c + Σ jacobian[k].1 · x[jacobian[k].0]`.
///
/// `v(n, m)` reads `x[n] − x[m]`, `i(src)` the branch column of `src`,
/// `time` is `t`.
///
/// # Errors
///
/// [`SimError::InvalidComponent`] when `i(src)` names no voltage-defined
/// element, or the expression isn't finite at `x`. An infinite slope at a
/// finite value is taken as 0.
pub(crate) fn behavioural_linearisation(
    circuit: &Circuit,
    node_map: &NodeMap,
    id: &str,
    expr: &Expression,
    x: &DVector<f64>,
    t: f64,
) -> Result<(Vec<(usize, f64)>, f64), SimError> {
    use crate::expression::Variable;
    let num_nodes = node_map.num_nodes();
    let at = |k: usize| x.get(k).copied().unwrap_or(0.0);
    let mut columns: Vec<Vec<(usize, f64)>> = Vec::with_capacity(expr.variables().len());
    let mut values = Vec::with_capacity(expr.variables().len());
    for variable in expr.variables() {
        let (cols, value) = match variable {
            Variable::Voltage { node, reference } => {
                let mut cols = Vec::new();
                let mut value = 0.0;
                if let Some(k) = node_map.index(node) {
                    cols.push((k, 1.0));
                    value += at(k);
                }
                if let Some(k) = reference.as_deref().and_then(|r| node_map.index(r)) {
                    cols.push((k, -1.0));
                    value -= at(k);
                }
                (cols, value)
            }
            Variable::Current { source } => {
                let branch = circuit.vsource_branch_index(source).ok_or_else(|| {
                    SimError::InvalidComponent(format!(
                        "`{id}` reads i({source}), but `{source}` isn't a voltage-defined \
                         element (voltage source, VCVS, CCVS, op-amp, regulator or \
                         behavioural voltage source)"
                    ))
                })?;
                let k = num_nodes + branch;
                (vec![(k, 1.0)], at(k))
            }
            Variable::Time => (Vec::new(), t),
        };
        columns.push(cols);
        values.push(value);
    }
    let mut gradient = vec![0.0; values.len()];
    let f = expr.evaluate_with_gradient(&values, &mut gradient);
    // A finite value with an infinite slope (sqrt at 0, say) is stamped
    // flat for this iteration, so Newton can move off it.
    for g in &mut gradient {
        if !g.is_finite() {
            *g = 0.0;
        }
    }
    if !f.is_finite() {
        return Err(SimError::NotFinite {
            component: id.to_string(),
            expression: expr.to_string(),
            inputs: expr
                .variables()
                .iter()
                .zip(&values)
                .map(|(v, x)| format!("{} = {x}", variable_name(v)))
                .collect::<Vec<_>>()
                .join(", "),
        });
    }
    let mut jacobian: Vec<(usize, f64)> = Vec::new();
    for (cols, g) in columns.iter().zip(&gradient) {
        for (k, coefficient) in cols {
            match jacobian.iter_mut().find(|(c, _)| c == k) {
                Some((_, j)) => *j += g * coefficient,
                None => jacobian.push((*k, g * coefficient)),
            }
        }
    }
    let c = f - jacobian.iter().map(|(k, j)| j * at(*k)).sum::<f64>();
    Ok((jacobian, c))
}

fn variable_name(variable: &crate::expression::Variable) -> String {
    use crate::expression::Variable;
    match variable {
        Variable::Voltage {
            node,
            reference: None,
        } => format!("v({node})"),
        Variable::Voltage {
            node,
            reference: Some(r),
        } => format!("v({node},{r})"),
        Variable::Current { source } => format!("i({source})"),
        Variable::Time => "time".to_string(),
    }
}

/// Stamps a behavioural source linearised at `x` and time `t`. A voltage
/// output takes branch row `branch`; a current output takes none.
///
/// Terminals as matrix indices (`None` is ground).
#[allow(clippy::too_many_arguments)]
pub(crate) fn stamp_behavioural_at(
    system: &mut MnaSystem,
    circuit: &Circuit,
    node_map: &NodeMap,
    id: &str,
    [p, q]: [Option<usize>; 2],
    output: BehaviouralOutput,
    expr: &Expression,
    x: &DVector<f64>,
    t: f64,
    branch: usize,
) -> Result<(), SimError> {
    let (jacobian, c) = behavioural_linearisation(circuit, node_map, id, expr, x, t)?;
    match output {
        BehaviouralOutput::Current => {
            // Out of n+, into n-: f leaves p, so p's row gets +∂f/∂x.
            for (k, j) in &jacobian {
                if let Some(pi) = p {
                    system.a[(pi, *k)] += j;
                }
                if let Some(qi) = q {
                    system.a[(qi, *k)] -= j;
                }
            }
            if let Some(pi) = p {
                system.b[pi] -= c;
            }
            if let Some(qi) = q {
                system.b[qi] += c;
            }
        }
        _ => {
            // V(p) − V(q) − ∇f·x = c.
            stamp_voltage_source_at(system, [p, q], c, branch);
            for (k, j) in &jacobian {
                system.a[(branch, *k)] -= j;
            }
        }
    }
    Ok(())
}
