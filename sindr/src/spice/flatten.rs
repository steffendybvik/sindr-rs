//! Subcircuit flattener.
//!
//! Walks a `Vec<RawCard>` (as produced by the parser) and expands every
//! `X<inst>` subcircuit instance inline, producing a flat element list with
//! `.`-separated hierarchical naming for both nodes and component ids.
//!
//! Algorithm:
//!
//! 1. **Pass 1 — collect definitions.** Walk the cards once moving every
//!    `.subckt` into a `name -> RawSubckt` table, every `.model` into a
//!    `name -> RawModel` table, every top-level `.param` list into a
//!    deferred-evaluation queue, and every `.tran/.dc/.ac/.op` into the
//!    analyses list. Top-level element lines go into `top_elements`.
//!
//! 2. **Pass 2 — evaluate top params, then expand.** Run the deferred
//!    `.param` lists against the outer `ParamScope`. Then walk
//!    `top_elements`: subckt instances recurse into `expand_instance`;
//!    other elements are emitted directly with their current scope
//!    snapshot.
//!
//! Recursion into `expand_instance` builds a fresh scope frame, populates
//! it from the subckt's `defaults` then the instance's overrides
//! (overrides win), establishes a `port -> caller_node` mapping, and
//! emits each body card with renamed nodes / ids:
//!
//! * Ground node `"0"` is **never** prefixed.
//! * A node matching a subckt port maps to the caller's node.
//! * Any other node is prefixed with `instance_path.join(".")`.
//! * Component ids are prefixed similarly.
//!
//! Recursion into the same subckt name on the active expansion stack
//! raises [`SpiceParseError::Syntax`].

#![allow(dead_code)] // some helpers are exercised only via tests

use std::collections::{HashMap, HashSet};

use miette::NamedSource;

use crate::spice::ast::{
    ParamExpr, RawAnalysis, RawCard, RawElement, RawElementBody, RawModel, RawSubckt,
};
use crate::spice::error::{ParseWarning, SpiceParseError};
use crate::spice::param_eval::{eval_error_to_parse_error, eval_param_list, ParamScope};
use crate::spice::source_map::{HierarchyPath, SourceMap};

/// Result of flattening: the elements to lower, the model table, populated
/// source-map, and the top-level analyses (still raw — analysis directive
/// arguments are evaluated by `build.rs`).
#[derive(Debug)]
pub(crate) struct Flattened {
    pub elements: Vec<FlatElement>,
    pub models: HashMap<String, RawModel>,
    pub source_map: SourceMap,
    pub analyses: Vec<RawAnalysis>,
    /// Snapshot of the outer-most parameter scope after top-level `.param`
    /// evaluation. Analysis directive arguments are evaluated against this.
    pub top_params: HashMap<String, f64>,
}

/// One device-instance line lifted out of all subckt nesting. The
/// `scope_snapshot` carries every parameter visible at the point of
/// expansion, so `build.rs` can evaluate `value` expressions without
/// re-walking the hierarchy.
#[derive(Debug)]
pub(crate) struct FlatElement {
    pub raw: RawElement,
    pub instance_path: Vec<String>,
    pub scope_snapshot: HashMap<String, f64>,
}

/// Flatten `cards` into a [`Flattened`] form.
///
/// `strict` is currently honoured only via the warnings vector — callers
/// pass it through so future lenient downgrades can branch on it.
pub(crate) fn flatten(
    cards: Vec<RawCard>,
    _strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<Flattened, SpiceParseError> {
    // ---- Pass 1: collect definitions ----
    let mut subckts: HashMap<String, RawSubckt> = HashMap::new();
    let mut models: HashMap<String, RawModel> = HashMap::new();
    let mut top_param_lists: Vec<Vec<(String, ParamExpr)>> = Vec::new();
    let mut top_analyses: Vec<RawAnalysis> = Vec::new();
    let mut top_elements: Vec<RawElement> = Vec::new();

    for card in cards {
        match card {
            RawCard::Subckt(s) => {
                subckts.insert(s.name.clone(), s);
            }
            RawCard::Model(m) => {
                models.insert(m.name.clone(), m);
            }
            RawCard::Param(list) => top_param_lists.push(list),
            RawCard::Analysis(a) => top_analyses.push(a),
            RawCard::Element(e) => top_elements.push(e),
            RawCard::Include(_) | RawCard::Lib { .. } => {
                // The preprocessor resolves these; if one survives this far
                // it is a parser/preprocessor bug. Skip with a warning.
                warnings.push(ParseWarning {
                    message: "stray .include/.lib in flattener input".to_string(),
                    span: None,
                    file: None,
                });
            }
            RawCard::Ends(_) => {
                // The parser collects `.ends` into its parent subckt; an
                // `.ends` reaching this point would have already errored
                // there. Defensive skip.
                warnings.push(ParseWarning {
                    message: "stray .ends in flattener input".to_string(),
                    span: None,
                    file: None,
                });
            }
        }
    }

    // ---- Pass 2: evaluate top-level params ----
    let mut scope = ParamScope::new();
    for list in &top_param_lists {
        if let Err(e) = eval_param_list(list, &mut scope) {
            return Err(eval_error_to_parse_error(e, "<param>"));
        }
    }

    // Snapshot top-level params for analysis-directive evaluation later.
    let top_params_snapshot = snapshot_scope(&scope);

    // ---- Pass 2 cont: expand top-level elements ----
    let mut out_elements: Vec<FlatElement> = Vec::new();
    let mut source_map: SourceMap = HashMap::new();
    let mut active_stack: HashSet<String> = HashSet::new();

    for el in top_elements {
        if matches!(el.body, RawElementBody::Subckt { .. }) {
            expand_instance(
                el,
                &[],
                &subckts,
                &mut scope,
                &mut active_stack,
                &mut out_elements,
                &mut source_map,
            )?;
        } else {
            // Top-level non-X element: emit directly.
            let snapshot = snapshot_scope(&scope);
            out_elements.push(FlatElement {
                raw: el,
                instance_path: Vec::new(),
                scope_snapshot: snapshot,
            });
        }
    }

    Ok(Flattened {
        elements: out_elements,
        models,
        source_map,
        analyses: top_analyses,
        top_params: top_params_snapshot,
    })
}

/// Recursive expansion of one `X<inst>` subckt instance.
fn expand_instance(
    inst: RawElement,
    parent_path: &[String],
    subckts: &HashMap<String, RawSubckt>,
    scope: &mut ParamScope,
    active_stack: &mut HashSet<String>,
    out: &mut Vec<FlatElement>,
    source_map: &mut SourceMap,
) -> Result<(), SpiceParseError> {
    let (subckt_name, instance_overrides) = match &inst.body {
        RawElementBody::Subckt { name, params } => (name.clone(), params.clone()),
        _ => unreachable!("expand_instance called on non-subckt element"),
    };

    let def = match subckts.get(&subckt_name) {
        Some(d) => d,
        None => {
            return Err(SpiceParseError::UndefinedSubckt {
                name: subckt_name.clone(),
                src: span_to_named_source(&inst),
                bad_span: span_to_source_span(&inst),
            });
        }
    };

    // Recursion check.
    if !active_stack.insert(subckt_name.clone()) {
        return Err(SpiceParseError::Syntax {
            message: format!("subcircuit recursion detected in `{subckt_name}`"),
            src: span_to_named_source(&inst),
            bad_span: span_to_source_span(&inst),
        });
    }

    // Arity check between caller-side nodes and declared ports.
    if inst.nodes.len() != def.ports.len() {
        active_stack.remove(&subckt_name);
        return Err(SpiceParseError::ArityMismatch {
            card: inst.id.clone(),
            expected: format!("{} port nodes", def.ports.len()),
            got: inst.nodes.len(),
            src: span_to_named_source(&inst),
            bad_span: span_to_source_span(&inst),
        });
    }

    // Build the port -> caller-node mapping.
    let mut port_map: HashMap<String, String> = HashMap::new();
    for (port, caller_node) in def.ports.iter().zip(inst.nodes.iter()) {
        port_map.insert(port.clone(), caller_node.clone());
    }

    // New scope frame for the body. Defaults evaluated in *parent* scope
    // first, then instance overrides win.
    scope.push_scope();
    if let Err(e) = eval_param_list(&def.defaults, scope) {
        scope.pop_scope();
        active_stack.remove(&subckt_name);
        return Err(eval_error_to_parse_error(e, "<param>"));
    }
    if let Err(e) = eval_param_list(&instance_overrides, scope) {
        scope.pop_scope();
        active_stack.remove(&subckt_name);
        return Err(eval_error_to_parse_error(e, "<param>"));
    }

    // Build the instance path for renaming.
    let mut instance_path = parent_path.to_vec();
    instance_path.push(inst.id.clone());

    // Walk the body.
    for body_card in &def.body {
        match body_card {
            RawCard::Element(child) => {
                if matches!(child.body, RawElementBody::Subckt { .. }) {
                    // For nested X-instances, do NOT pre-prefix the id —
                    // recurse with the parent's instance_path and let the
                    // recursive call append the raw child id once. We
                    // still need to map the child's port nodes through
                    // the current port_map / prefix rules so caller-side
                    // node names are correct.
                    let mut nested = child.clone();
                    nested.nodes = nested
                        .nodes
                        .iter()
                        .map(|nd| rename_node(nd, &instance_path.join("."), &port_map))
                        .collect();
                    expand_instance(
                        nested,
                        &instance_path,
                        subckts,
                        scope,
                        active_stack,
                        out,
                        source_map,
                    )?;
                } else {
                    let renamed = rename_element(child, &instance_path, &port_map);
                    // Populate source_map for the renamed (non-port,
                    // non-ground) nodes so callers can recover hierarchy.
                    for (renamed_node, original_node) in
                        renamed.nodes.iter().zip(child.nodes.iter())
                    {
                        if original_node == "0" {
                            continue; // ground passthrough
                        }
                        if port_map.contains_key(original_node) {
                            continue; // mapped through to caller — already in caller's namespace
                        }
                        source_map
                            .entry(renamed_node.clone())
                            .or_insert_with(|| HierarchyPath {
                                instance_path: instance_path.clone(),
                                original_node: original_node.clone(),
                            });
                    }
                    let snapshot = snapshot_scope(scope);
                    out.push(FlatElement {
                        raw: renamed,
                        instance_path: instance_path.clone(),
                        scope_snapshot: snapshot,
                    });
                }
            }
            RawCard::Param(list) => {
                if let Err(e) = eval_param_list(list, scope) {
                    scope.pop_scope();
                    active_stack.remove(&subckt_name);
                    return Err(eval_error_to_parse_error(e, "<param>"));
                }
            }
            RawCard::Subckt(_) | RawCard::Model(_) => {
                // Nested .subckt / .model definitions are not supported
                // (they would change visibility rules); skip with a
                // warning. The grammar already accepts them, so this
                // keeps lowering robust to vendor decks with nested
                // definitions even if we cannot use them.
                // No-op for now.
            }
            RawCard::Analysis(_) | RawCard::Include(_) | RawCard::Lib { .. } | RawCard::Ends(_) => {
                // Ignore inside subckt bodies.
            }
        }
    }

    scope.pop_scope();
    active_stack.remove(&subckt_name);
    Ok(())
}

/// Apply hierarchical renaming to a body element.
fn rename_element(
    src: &RawElement,
    instance_path: &[String],
    port_map: &HashMap<String, String>,
) -> RawElement {
    let prefix = instance_path.join(".");
    let new_id = if prefix.is_empty() {
        src.id.clone()
    } else {
        format!("{prefix}.{}", src.id)
    };
    let new_nodes = src
        .nodes
        .iter()
        .map(|nd| rename_node(nd, &prefix, port_map))
        .collect();

    // Source-instance bodies need their X-instance subckt name preserved
    // and any ParamExpr left untouched (they are evaluated in build.rs
    // against the FlatElement::scope_snapshot). The rest just clones.
    RawElement {
        id: new_id,
        prefix: src.prefix,
        nodes: new_nodes,
        body: src.body.clone(),
        span: src.span.clone(),
    }
}

fn rename_node(nd: &str, prefix: &str, port_map: &HashMap<String, String>) -> String {
    if nd == "0" {
        return "0".to_string();
    }
    if let Some(caller) = port_map.get(nd) {
        return caller.clone();
    }
    if prefix.is_empty() {
        nd.to_string()
    } else {
        format!("{prefix}.{nd}")
    }
}

/// Take a complete snapshot of every name visible in the scope. Used so
/// downstream `build.rs` can evaluate `ParamExpr` against the exact frame
/// stack present at expansion time without re-walking the hierarchy.
fn snapshot_scope(scope: &ParamScope) -> HashMap<String, f64> {
    // ParamScope intentionally exposes only `get`/`define`; we build a
    // snapshot by replaying every defined name through `get` is not
    // possible without name discovery. Instead, since ParamScope is
    // crate-private we add a small helper here that pokes at the frames
    // via a public method we add next to it. To keep the seam minimal,
    // we use the new `snapshot_flat` method on ParamScope.
    scope.snapshot_flat()
}

fn span_to_named_source(el: &RawElement) -> NamedSource<String> {
    NamedSource::new(&*el.span.file, String::new())
}

fn span_to_source_span(el: &RawElement) -> miette::SourceSpan {
    (el.span.start, el.span.end.saturating_sub(el.span.start)).into()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::spice::ast::{ParamExpr, RawElement, RawElementBody, RawSubckt, Span};

    fn span() -> Span {
        Span {
            start: 0,
            end: 0,
            file: Arc::from("test.cir"),
        }
    }

    fn r_element(id: &str, n0: &str, n1: &str, value: f64) -> RawElement {
        RawElement {
            id: id.to_string(),
            prefix: 'r',
            nodes: vec![n0.to_string(), n1.to_string()],
            body: RawElementBody::Passive {
                value: ParamExpr::Number(value),
            },
            span: span(),
        }
    }

    fn r_element_expr(id: &str, n0: &str, n1: &str, expr: ParamExpr) -> RawElement {
        RawElement {
            id: id.to_string(),
            prefix: 'r',
            nodes: vec![n0.to_string(), n1.to_string()],
            body: RawElementBody::Passive { value: expr },
            span: span(),
        }
    }

    fn x_element(
        id: &str,
        nodes: &[&str],
        sub: &str,
        params: Vec<(String, ParamExpr)>,
    ) -> RawElement {
        RawElement {
            id: id.to_string(),
            prefix: 'x',
            nodes: nodes.iter().map(|s| s.to_string()).collect(),
            body: RawElementBody::Subckt {
                name: sub.to_string(),
                params,
            },
            span: span(),
        }
    }

    fn subckt(name: &str, ports: &[&str], body: Vec<RawCard>) -> RawSubckt {
        RawSubckt {
            name: name.to_string(),
            ports: ports.iter().map(|s| s.to_string()).collect(),
            defaults: vec![],
            body,
            span: span(),
        }
    }

    fn subckt_with_defaults(
        name: &str,
        ports: &[&str],
        defaults: Vec<(String, ParamExpr)>,
        body: Vec<RawCard>,
    ) -> RawSubckt {
        RawSubckt {
            name: name.to_string(),
            ports: ports.iter().map(|s| s.to_string()).collect(),
            defaults,
            body,
            span: span(),
        }
    }

    #[test]
    fn forward_defined_subckt_resolves() {
        // X1 uses `mysub` before mysub is declared.
        let cards = vec![
            RawCard::Element(x_element("X1", &["a", "b"], "mysub", vec![])),
            RawCard::Subckt(subckt(
                "mysub",
                &["p", "q"],
                vec![RawCard::Element(r_element("R1", "p", "q", 1000.0))],
            )),
        ];
        let mut warnings = Vec::new();
        let flat = flatten(cards, true, &mut warnings).unwrap();
        assert_eq!(flat.elements.len(), 1);
        assert_eq!(flat.elements[0].raw.id, "X1.R1");
        assert_eq!(flat.elements[0].raw.nodes, vec!["a", "b"]);
    }

    #[test]
    fn ground_passthrough_not_prefixed() {
        // Body contains `R1 in 0 1k`; instance maps in -> a, ports = [in].
        let cards = vec![
            RawCard::Subckt(subckt(
                "mysub",
                &["in"],
                vec![RawCard::Element(r_element("R1", "in", "0", 1000.0))],
            )),
            RawCard::Element(x_element("X1", &["a"], "mysub", vec![])),
        ];
        let mut warnings = Vec::new();
        let flat = flatten(cards, true, &mut warnings).unwrap();
        assert_eq!(flat.elements.len(), 1);
        assert_eq!(flat.elements[0].raw.nodes, vec!["a", "0"]);
    }

    #[test]
    fn nested_two_deep_hierarchical_naming() {
        // outer X1 -> uses `outer`; outer body has X2 -> uses `inner`;
        // inner body has R1 on internal node n1 to ground.
        let cards = vec![
            RawCard::Subckt(subckt(
                "inner",
                &["p"],
                vec![RawCard::Element(r_element("R1", "n1", "p", 100.0))],
            )),
            RawCard::Subckt(subckt(
                "outer",
                &["q"],
                vec![RawCard::Element(x_element("X2", &["q"], "inner", vec![]))],
            )),
            RawCard::Element(x_element("X1", &["a"], "outer", vec![])),
        ];
        let mut warnings = Vec::new();
        let flat = flatten(cards, true, &mut warnings).unwrap();
        assert_eq!(flat.elements.len(), 1);
        // Internal node n1 should appear at X1.X2.n1.
        let nodes = &flat.elements[0].raw.nodes;
        assert_eq!(nodes[0], "X1.X2.n1");
        assert_eq!(nodes[1], "a"); // port p mapped through both layers
        assert_eq!(flat.elements[0].raw.id, "X1.X2.R1");
    }

    #[test]
    fn instance_override_wins_over_default() {
        // .subckt rc in out r=1k ... R1 in out {r}
        let cards = vec![
            RawCard::Subckt(subckt_with_defaults(
                "rc",
                &["in", "out"],
                vec![("r".to_string(), ParamExpr::Number(1000.0))],
                vec![RawCard::Element(r_element_expr(
                    "R1",
                    "in",
                    "out",
                    ParamExpr::Ref("r".to_string()),
                ))],
            )),
            RawCard::Element(x_element(
                "X1",
                &["a", "b"],
                "rc",
                vec![("r".to_string(), ParamExpr::Number(2000.0))],
            )),
        ];
        let mut warnings = Vec::new();
        let flat = flatten(cards, true, &mut warnings).unwrap();
        // Snapshot should contain r = 2000.0 (instance override wins).
        assert_eq!(flat.elements[0].scope_snapshot.get("r"), Some(&2000.0));
    }

    #[test]
    fn default_used_when_instance_omits_param() {
        let cards = vec![
            RawCard::Subckt(subckt_with_defaults(
                "rc",
                &["in", "out"],
                vec![("r".to_string(), ParamExpr::Number(1000.0))],
                vec![RawCard::Element(r_element_expr(
                    "R1",
                    "in",
                    "out",
                    ParamExpr::Ref("r".to_string()),
                ))],
            )),
            RawCard::Element(x_element("X1", &["a", "b"], "rc", vec![])),
        ];
        let mut warnings = Vec::new();
        let flat = flatten(cards, true, &mut warnings).unwrap();
        assert_eq!(flat.elements[0].scope_snapshot.get("r"), Some(&1000.0));
    }

    #[test]
    fn outer_param_visible_inside_subckt_body() {
        // .param vcc=5; subckt uses {vcc} for R value.
        let cards = vec![
            RawCard::Param(vec![("vcc".to_string(), ParamExpr::Number(5.0))]),
            RawCard::Subckt(subckt(
                "rc",
                &["in", "out"],
                vec![RawCard::Element(r_element_expr(
                    "R1",
                    "in",
                    "out",
                    ParamExpr::Ref("vcc".to_string()),
                ))],
            )),
            RawCard::Element(x_element("X1", &["a", "b"], "rc", vec![])),
        ];
        let mut warnings = Vec::new();
        let flat = flatten(cards, true, &mut warnings).unwrap();
        assert_eq!(flat.elements[0].scope_snapshot.get("vcc"), Some(&5.0));
    }

    #[test]
    fn arity_mismatch_errors() {
        // mysub declares 3 ports; instance passes only 2.
        let cards = vec![
            RawCard::Subckt(subckt(
                "mysub",
                &["a", "b", "c"],
                vec![RawCard::Element(r_element("R1", "a", "b", 100.0))],
            )),
            RawCard::Element(x_element("X1", &["x", "y"], "mysub", vec![])),
        ];
        let mut warnings = Vec::new();
        let err = flatten(cards, true, &mut warnings).unwrap_err();
        assert!(matches!(err, SpiceParseError::ArityMismatch { .. }));
    }

    #[test]
    fn undefined_subckt_errors() {
        let cards = vec![RawCard::Element(x_element(
            "X1",
            &["a", "b"],
            "ghost",
            vec![],
        ))];
        let mut warnings = Vec::new();
        let err = flatten(cards, true, &mut warnings).unwrap_err();
        assert!(matches!(err, SpiceParseError::UndefinedSubckt { .. }));
    }

    #[test]
    fn recursive_subckt_errors() {
        // mysub instantiates itself.
        let cards = vec![
            RawCard::Subckt(subckt(
                "mysub",
                &["a", "b"],
                vec![RawCard::Element(x_element(
                    "X1",
                    &["a", "b"],
                    "mysub",
                    vec![],
                ))],
            )),
            RawCard::Element(x_element("X0", &["p", "q"], "mysub", vec![])),
        ];
        let mut warnings = Vec::new();
        let err = flatten(cards, true, &mut warnings).unwrap_err();
        match err {
            SpiceParseError::Syntax { message, .. } => {
                assert!(message.contains("recursion"), "got: {message}");
            }
            other => panic!("expected Syntax recursion error, got {other:?}"),
        }
    }

    #[test]
    fn source_map_records_internal_node() {
        // Body: R1 n2 0 1k. Instance X1 a b mysub. Internal n2 should
        // map back to instance_path ["X1"], original_node "n2".
        let cards = vec![
            RawCard::Subckt(subckt(
                "mysub",
                &["a", "b"],
                vec![RawCard::Element(r_element("R1", "n2", "0", 1000.0))],
            )),
            RawCard::Element(x_element("X1", &["x", "y"], "mysub", vec![])),
        ];
        let mut warnings = Vec::new();
        let flat = flatten(cards, true, &mut warnings).unwrap();
        let path = flat
            .source_map
            .get("X1.n2")
            .expect("X1.n2 should be in source_map");
        assert_eq!(path.instance_path, vec!["X1".to_string()]);
        assert_eq!(path.original_node, "n2");
    }
}
