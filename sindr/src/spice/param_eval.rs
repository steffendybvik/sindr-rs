//! `.param` arithmetic evaluator with lexical scoping.
//!
//! Operates on the [`ParamExpr`](crate::spice::ast::ParamExpr) AST shaped by the
//! grammar. Pure: takes a [`ParamScope`] (a stack of frames) and folds
//! the expression to an `f64`. The grammar layer is responsible for
//! turning failures into [`crate::spice::SpiceParseError`]s with proper spans;
//! at this layer we surface a small private [`EvalError`].
//!
//! Scoping rules:
//! - `ParamScope` is a stack of `HashMap<String, f64>` frames. Lookup
//!   walks innermost-outward, so an inner-frame definition shadows an
//!   outer one (subcircuit instance overrides outer params).
//! - Within a single `.param` list, forward references resolve via a
//!   topological-sort dependency walk before evaluation. References to
//!   outer-scope names are resolved through `scope.get` and are not part
//!   of the dep graph. Cycles among the bindings raise
//!   [`EvalError::Circular`].

#![allow(dead_code)] // some helpers are exercised only via tests

use std::collections::{HashMap, HashSet};

use miette::NamedSource;

use crate::spice::ast::{BinOp, ParamExpr};
use crate::spice::error::SpiceParseError;

/// Lexically-scoped parameter table. Push a new frame when entering a
/// `.subckt` instantiation; pop it when leaving.
#[derive(Debug, Default)]
pub(crate) struct ParamScope {
    frames: Vec<HashMap<String, f64>>,
}

impl ParamScope {
    /// New scope with one empty top-level frame.
    pub fn new() -> Self {
        Self {
            frames: vec![HashMap::new()],
        }
    }

    /// Push an empty frame (entering a nested instance).
    pub fn push_scope(&mut self) {
        self.frames.push(HashMap::new());
    }

    /// Pop the innermost frame. No-op if only one frame remains.
    pub fn pop_scope(&mut self) {
        if self.frames.len() > 1 {
            self.frames.pop();
        }
    }

    /// Define `name = value` in the innermost frame, shadowing any
    /// outer-frame definition.
    pub fn define(&mut self, name: &str, value: f64) {
        if let Some(top) = self.frames.last_mut() {
            top.insert(name.to_string(), value);
        }
    }

    /// Look up `name`, walking frames innermost-outward.
    pub fn get(&self, name: &str) -> Option<f64> {
        for frame in self.frames.iter().rev() {
            if let Some(v) = frame.get(name) {
                return Some(*v);
            }
        }
        None
    }

    /// Flatten every frame into a single map. Inner frames shadow outer
    /// (innermost wins, matching `get`'s lookup order). Used by the
    /// flattener to capture the scope visible at expansion time so the
    /// build pass can evaluate `ParamExpr` without re-walking the
    /// hierarchy.
    pub fn snapshot_flat(&self) -> HashMap<String, f64> {
        let mut out = HashMap::new();
        for frame in self.frames.iter() {
            for (k, v) in frame {
                out.insert(k.clone(), *v);
            }
        }
        out
    }
}

/// Errors raised by [`eval`] / [`eval_param_list`]. Lifted to
/// [`crate::spice::SpiceParseError`] by the grammar layer with proper spans.
#[derive(Debug, PartialEq)]
pub(crate) enum EvalError {
    /// Reference to a parameter name that is not in scope.
    UndefinedParam(String),
    /// `x / 0` at runtime.
    DivByZero,
    /// Cyclic dependencies among bindings in the same `.param` list.
    Circular(Vec<String>),
}

/// Lift an [`EvalError`] to a [`SpiceParseError`], preserving the variant so
/// callers get a precise diagnostic (undefined-parameter, circular-reference)
/// rather than a generic syntax error. `src_name` names the synthetic miette
/// source shown in diagnostics (e.g. `"<build>"`, `"<param>"`); param
/// expressions reach this layer without per-node spans, so the span is a
/// zero-width placeholder.
pub(crate) fn eval_error_to_parse_error(e: EvalError, src_name: &str) -> SpiceParseError {
    let src = NamedSource::new(src_name, String::new());
    let bad_span = (0, 0).into();
    match e {
        EvalError::UndefinedParam(name) => SpiceParseError::UndefinedParam {
            name,
            src,
            bad_span,
        },
        EvalError::DivByZero => SpiceParseError::Syntax {
            message: "division by zero in parameter expression".to_string(),
            src,
            bad_span,
        },
        EvalError::Circular(cycle) => SpiceParseError::CircularParam {
            cycle,
            src,
            bad_span,
        },
    }
}

/// Evaluate a single expression against `scope`.
pub(crate) fn eval(expr: &ParamExpr, scope: &ParamScope) -> Result<f64, EvalError> {
    match expr {
        ParamExpr::Number(f) => Ok(*f),
        ParamExpr::Ref(name) => scope
            .get(name)
            .ok_or_else(|| EvalError::UndefinedParam(name.clone())),
        ParamExpr::Neg(inner) => Ok(-eval(inner, scope)?),
        ParamExpr::Pow(base, exp) => Ok(eval(base, scope)?.powf(eval(exp, scope)?)),
        ParamExpr::BinOp(l, op, r) => {
            let lv = eval(l, scope)?;
            let rv = eval(r, scope)?;
            match op {
                BinOp::Add => Ok(lv + rv),
                BinOp::Sub => Ok(lv - rv),
                BinOp::Mul => Ok(lv * rv),
                BinOp::Div => {
                    if rv == 0.0 {
                        Err(EvalError::DivByZero)
                    } else {
                        Ok(lv / rv)
                    }
                }
            }
        }
    }
}

/// Evaluate a list of `(name, expr)` pairs and define each into the
/// innermost frame of `scope`. Forward references within the list are
/// supported (we run a topological sort on the dependency graph first).
/// Cycles among list members raise [`EvalError::Circular`].
pub(crate) fn eval_param_list(
    bindings: &[(String, ParamExpr)],
    scope: &mut ParamScope,
) -> Result<(), EvalError> {
    if bindings.is_empty() {
        return Ok(());
    }

    // Collect names defined in this list (later wins on duplicates — the
    // grammar should reject those, but keep last-write semantics safe).
    let mut name_to_idx: HashMap<&str, usize> = HashMap::new();
    for (i, (name, _)) in bindings.iter().enumerate() {
        name_to_idx.insert(name.as_str(), i);
    }

    // Build dependency graph: edge i -> j means binding i depends on
    // binding j (i.e. expr_i references name_j defined in this list).
    let n = bindings.len();
    let mut deps: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (i, (_, expr)) in bindings.iter().enumerate() {
        let mut refs = HashSet::new();
        collect_refs(expr, &mut refs);
        for r in refs {
            if let Some(&j) = name_to_idx.get(r.as_str()) {
                if i != j {
                    deps[i].push(j);
                }
            }
        }
    }

    // Kahn's algorithm: build in-degrees on the *dependents* side.
    // dependents[j] = list of i that need j to be evaluated first.
    let mut dependents: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut in_degree: Vec<usize> = vec![0; n];
    for i in 0..n {
        for &j in &deps[i] {
            dependents[j].push(i);
            in_degree[i] += 1;
        }
    }

    let mut ready: Vec<usize> = (0..n).filter(|&i| in_degree[i] == 0).collect();
    let mut order: Vec<usize> = Vec::with_capacity(n);
    while let Some(i) = ready.pop() {
        order.push(i);
        for &k in &dependents[i] {
            in_degree[k] -= 1;
            if in_degree[k] == 0 {
                ready.push(k);
            }
        }
    }

    if order.len() != n {
        // Whatever is left in non-zero in-degree forms a cycle.
        let cycle: Vec<String> = (0..n)
            .filter(|i| in_degree[*i] > 0)
            .map(|i| bindings[i].0.clone())
            .collect();
        return Err(EvalError::Circular(cycle));
    }

    for i in order {
        let (name, expr) = &bindings[i];
        let v = eval(expr, scope)?;
        scope.define(name, v);
    }
    Ok(())
}

/// Walk `expr` collecting every `Ref` name into `out`.
fn collect_refs(expr: &ParamExpr, out: &mut HashSet<String>) {
    match expr {
        ParamExpr::Number(_) => {}
        ParamExpr::Ref(name) => {
            out.insert(name.clone());
        }
        ParamExpr::Neg(inner) => collect_refs(inner, out),
        ParamExpr::Pow(a, b) => {
            collect_refs(a, out);
            collect_refs(b, out);
        }
        ParamExpr::BinOp(l, _, r) => {
            collect_refs(l, out);
            collect_refs(r, out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn n(x: f64) -> ParamExpr {
        ParamExpr::Number(x)
    }
    fn r(name: &str) -> ParamExpr {
        ParamExpr::Ref(name.to_string())
    }
    fn add(a: ParamExpr, b: ParamExpr) -> ParamExpr {
        ParamExpr::BinOp(Box::new(a), BinOp::Add, Box::new(b))
    }
    fn mul(a: ParamExpr, b: ParamExpr) -> ParamExpr {
        ParamExpr::BinOp(Box::new(a), BinOp::Mul, Box::new(b))
    }
    fn pow(a: ParamExpr, b: ParamExpr) -> ParamExpr {
        ParamExpr::Pow(Box::new(a), Box::new(b))
    }

    #[test]
    fn literal_number() {
        let s = ParamScope::new();
        assert_eq!(eval(&n(1.5), &s).unwrap(), 1.5);
    }

    #[test]
    fn ref_hit() {
        let mut s = ParamScope::new();
        s.define("r_base", 1000.0);
        assert_eq!(eval(&r("r_base"), &s).unwrap(), 1000.0);
    }

    #[test]
    fn ref_miss_returns_undefined() {
        let s = ParamScope::new();
        assert_eq!(
            eval(&r("nope"), &s),
            Err(EvalError::UndefinedParam("nope".to_string()))
        );
    }

    #[test]
    fn arithmetic_expression() {
        // (1k + 2k) * 2 = 6000
        let s = ParamScope::new();
        let expr = mul(add(n(1000.0), n(2000.0)), n(2.0));
        assert_eq!(eval(&expr, &s).unwrap(), 6000.0);
    }

    #[test]
    fn pow_right_assoc() {
        // 2 ^ (3 ^ 2) = 2 ^ 9 = 512
        // The grammar produces right-associated Pow trees; we verify the
        // evaluator computes them correctly given that shape.
        let s = ParamScope::new();
        let expr = pow(n(2.0), pow(n(3.0), n(2.0)));
        assert_eq!(eval(&expr, &s).unwrap(), 512.0);
    }

    #[test]
    fn forward_ref_within_list() {
        let mut s = ParamScope::new();
        let bindings = vec![("a".to_string(), r("b")), ("b".to_string(), n(7.0))];
        eval_param_list(&bindings, &mut s).unwrap();
        assert_eq!(s.get("a"), Some(7.0));
        assert_eq!(s.get("b"), Some(7.0));
    }

    #[test]
    fn circular_bindings_detected() {
        let mut s = ParamScope::new();
        let bindings = vec![("a".to_string(), r("b")), ("b".to_string(), r("a"))];
        match eval_param_list(&bindings, &mut s) {
            Err(EvalError::Circular(names)) => {
                assert!(names.contains(&"a".to_string()));
                assert!(names.contains(&"b".to_string()));
            }
            other => panic!("expected Circular, got {other:?}"),
        }
    }

    #[test]
    fn nested_scope_lookup_and_pop() {
        let mut s = ParamScope::new();
        s.define("vcc", 5.0);
        s.push_scope();
        s.define("gain", 100.0);
        let expr = add(r("vcc"), r("gain"));
        assert_eq!(eval(&expr, &s).unwrap(), 105.0);
        s.pop_scope();
        assert_eq!(
            eval(&r("gain"), &s),
            Err(EvalError::UndefinedParam("gain".to_string()))
        );
    }

    #[test]
    fn instance_override_shadows_outer() {
        let mut s = ParamScope::new();
        s.define("r_load", 1000.0);
        s.push_scope();
        s.define("r_load", 2000.0);
        assert_eq!(s.get("r_load"), Some(2000.0));
        s.pop_scope();
        assert_eq!(s.get("r_load"), Some(1000.0));
    }

    #[test]
    fn div_by_zero_surfaces() {
        let s = ParamScope::new();
        let expr = ParamExpr::BinOp(Box::new(n(1.0)), BinOp::Div, Box::new(n(0.0)));
        assert_eq!(eval(&expr, &s), Err(EvalError::DivByZero));
    }

    #[test]
    fn outer_ref_not_part_of_dep_graph() {
        // Outer scope defines `vcc`; the .param list refers to it but does
        // not redefine it, so no cycle/dep edge is created.
        let mut s = ParamScope::new();
        s.define("vcc", 12.0);
        let bindings = vec![("v_half".to_string(), mul(r("vcc"), n(0.5)))];
        eval_param_list(&bindings, &mut s).unwrap();
        assert_eq!(s.get("v_half"), Some(6.0));
    }
}
