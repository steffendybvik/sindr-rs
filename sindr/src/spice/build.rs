//! Lift flattened raw forms ([`crate::spice::flatten::Flattened`]) into the public
//! [`crate::spice::ParsedNetlist`] (sindr `Circuit` + `AnalysisRequest`s).
//!
//! Per-element mapping rules:
//!
//! * **R / L / C** — evaluate `value` against the element's scope snapshot
//!   and emit the matching `CircuitElement`.
//! * **V / I** — evaluate the optional DC term and translate the optional
//!   `RawWaveform` (PULSE / SIN / PWL) into `crate::Waveform`. SPICE's
//!   `td/tr/tf/pw/period` for PULSE map directly; SIN's `theta` (damping)
//!   is dropped with a warning in lenient mode and an error in strict mode
//!   when non-zero.
//! * **D** — look up `.model`; kind must be `D`. Other kinds raise
//!   `UnsupportedModelType`. `IS`/`N` parameters are silently dropped
//!   today — `crate::CircuitElement::Diode` does not yet expose them.
//! * **Q** — look up `.model`; kind must be `NPN` or `PNP`. **SPICE node
//!   order on the Q line is `[c, b, e]`; sindr's `Bjt::nodes` is
//!   `[b, c, e]`** — transpose at this layer.
//! * **X** — should not appear in `flat.elements` (the flattener already
//!   expanded them). Hitting one here is an internal bug.

#![allow(dead_code)] // some helpers are exercised only via tests

use std::collections::HashMap;

use crate::circuit::BjtParasiticCaps;
use crate::{BjtKind, Circuit, CircuitElement, Waveform};
use miette::NamedSource;

use crate::spice::analysis::{AcSweep, AnalysisRequest};
use crate::spice::ast::{
    AcSweepKind, ParamExpr, RawAnalysis, RawElementBody, RawModel, RawWaveform, SourceKind,
};
use crate::spice::error::{ParseWarning, SpiceParseError};
use crate::spice::flatten::{FlatElement, Flattened};
use crate::spice::param_eval::{eval, EvalError, ParamScope};
use crate::spice::ParsedNetlist;

/// Lift a `Flattened` form into a public [`ParsedNetlist`].
pub(crate) fn build_netlist(
    flat: Flattened,
    title: String,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<ParsedNetlist, SpiceParseError> {
    let mut components: Vec<CircuitElement> = Vec::with_capacity(flat.elements.len());

    for fe in &flat.elements {
        let element = build_element(fe, &flat.models, strict, warnings)?;
        components.push(element);
    }

    let mut analyses: Vec<AnalysisRequest> = Vec::with_capacity(flat.analyses.len());
    let top_scope = scope_from_snapshot(&flat.top_params);
    for raw in &flat.analyses {
        analyses.push(build_analysis(raw, &top_scope)?);
    }

    Ok(ParsedNetlist {
        circuit: Circuit {
            ground_node: "0".to_string(),
            components,
        },
        analyses,
        warnings: warnings.clone(),
        source_map: flat.source_map,
        title,
    })
}

fn build_element(
    fe: &FlatElement,
    models: &HashMap<String, RawModel>,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<CircuitElement, SpiceParseError> {
    let scope = scope_from_snapshot(&fe.scope_snapshot);
    let raw = &fe.raw;
    match &raw.body {
        RawElementBody::Passive { value } => {
            let v = eval_or_lift(value, &scope)?;
            let nodes = two_nodes(&raw.nodes, &raw.id)?;
            match raw.prefix {
                'r' => Ok(CircuitElement::Resistor {
                    id: raw.id.clone(),
                    nodes,
                    resistance: v,
                }),
                'l' => Ok(CircuitElement::Inductor {
                    id: raw.id.clone(),
                    nodes,
                    inductance: v,
                }),
                'c' => Ok(CircuitElement::Capacitor {
                    id: raw.id.clone(),
                    nodes,
                    capacitance: v,
                }),
                other => Err(SpiceParseError::Syntax {
                    message: format!("internal: unexpected passive prefix `{other}`"),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                }),
            }
        }

        RawElementBody::Source { kind, dc, waveform } => {
            let nodes = two_nodes(&raw.nodes, &raw.id)?;
            let dc_value = match dc {
                Some(expr) => eval_or_lift(expr, &scope)?,
                None => 0.0,
            };
            let waveform = match waveform {
                Some(rw) => Some(translate_waveform(rw, &scope, strict, warnings)?),
                None => None,
            };
            // If a waveform is present, the static `voltage`/`current`
            // field is the DC offset (added to waveform.evaluate(t) by
            // the solver). With no waveform, it is just the DC value.
            match kind {
                SourceKind::Voltage => Ok(CircuitElement::VoltageSource {
                    id: raw.id.clone(),
                    nodes,
                    voltage: dc_value,
                    waveform,
                }),
                SourceKind::Current => Ok(CircuitElement::CurrentSource {
                    id: raw.id.clone(),
                    nodes,
                    current: dc_value,
                    waveform,
                }),
            }
        }

        RawElementBody::Diode { model } => {
            let m = lookup_model(model, models, raw)?;
            let kind_lc = m.kind.to_ascii_lowercase();
            if kind_lc != "d" {
                return Err(SpiceParseError::UnsupportedModelType {
                    kind: m.kind.clone(),
                    name: m.name.clone(),
                    src: NamedSource::new(&*raw.span.file, String::new()),
                    bad_span: span_of(raw),
                });
            }
            // IS / N parameters in the .model are silently dropped today —
            // crate::CircuitElement::Diode exposes only `temperature`.
            // A future warning could surface here once the public API
            // grows the relevant fields.
            let nodes = two_nodes(&raw.nodes, &raw.id)?;
            Ok(CircuitElement::Diode {
                id: raw.id.clone(),
                nodes,
                temperature: 300.15,
            })
        }

        RawElementBody::Bjt { model, substrate } => {
            let m = lookup_model(model, models, raw)?;
            let kind_lc = m.kind.to_ascii_lowercase();
            let bjt_kind = match kind_lc.as_str() {
                "npn" => BjtKind::Npn,
                "pnp" => BjtKind::Pnp,
                _ => {
                    return Err(SpiceParseError::UnsupportedModelType {
                        kind: m.kind.clone(),
                        name: m.name.clone(),
                        src: NamedSource::new(&*raw.span.file, String::new()),
                        bad_span: span_of(raw),
                    });
                }
            };
            // SPICE3 Q-line node order is [collector, base, emitter];
            // crate::Bjt::nodes is [base, collector, emitter] — transpose.
            if raw.nodes.len() < 3 {
                return Err(SpiceParseError::ArityMismatch {
                    card: raw.id.clone(),
                    expected: "3 nodes (collector base emitter)".to_string(),
                    got: raw.nodes.len(),
                    src: NamedSource::new(&*raw.span.file, String::new()),
                    bad_span: span_of(raw),
                });
            }
            if substrate.is_some() {
                warnings.push(ParseWarning {
                    message: format!(
                        "BJT `{}` substrate node ignored (sindr 4-terminal BJT not yet supported)",
                        raw.id
                    ),
                    span: None,
                    file: Some(raw.span.file.to_string()),
                });
            }
            let nodes = [
                /* base */ raw.nodes[1].clone(),
                /* collector */ raw.nodes[0].clone(),
                /* emitter */ raw.nodes[2].clone(),
            ];
            let bf = m
                .params
                .iter()
                .find(|(k, _)| k.eq_ignore_ascii_case("bf"))
                .map(|(_, e)| eval_or_lift(e, &scope))
                .transpose()?
                .unwrap_or(100.0);
            Ok(CircuitElement::Bjt {
                id: raw.id.clone(),
                nodes,
                kind: bjt_kind,
                bf,
                temperature: 300.15,
                parasitic_caps: None::<BjtParasiticCaps>,
            })
        }

        RawElementBody::Subckt { name, .. } => Err(SpiceParseError::Syntax {
            message: format!(
                "internal: unexpanded subckt instance `{}` referencing `{name}` reached build pass",
                raw.id
            ),
            src: NamedSource::new("<build>", String::new()),
            bad_span: (0, 0).into(),
        }),
    }
}

fn translate_waveform(
    rw: &RawWaveform,
    scope: &ParamScope,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<Waveform, SpiceParseError> {
    match rw {
        RawWaveform::Pulse(args) => {
            // SPICE PULSE: v1 v2 td tr tf pw per
            if args.len() != 7 {
                return Err(SpiceParseError::ArityMismatch {
                    card: "PULSE".to_string(),
                    expected: "7 args (v1 v2 td tr tf pw per)".to_string(),
                    got: args.len(),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                });
            }
            let evals: Vec<f64> = args
                .iter()
                .map(|e| eval_or_lift(e, scope))
                .collect::<Result<Vec<_>, _>>()?;
            Ok(Waveform::Pulse {
                v1: evals[0],
                v2: evals[1],
                delay: evals[2],
                rise_time: evals[3],
                fall_time: evals[4],
                pulse_width: evals[5],
                period: evals[6],
            })
        }
        RawWaveform::Sin(args) => {
            // SPICE SIN: vo va freq [td] [theta]
            if !(3..=5).contains(&args.len()) {
                return Err(SpiceParseError::ArityMismatch {
                    card: "SIN".to_string(),
                    expected: "3..=5 args (vo va freq [td] [theta])".to_string(),
                    got: args.len(),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                });
            }
            let evals: Vec<f64> = args
                .iter()
                .map(|e| eval_or_lift(e, scope))
                .collect::<Result<Vec<_>, _>>()?;
            let offset = evals[0];
            let amplitude = evals[1];
            let frequency = evals[2];
            // td (delay) is dropped today — crate::Waveform::Sine has no
            // delay field. Phase carries radians, td would need to be
            // converted to phase = -2*pi*f*td. Do that conversion so we
            // do not silently lose the value.
            let phase = if evals.len() >= 4 {
                -2.0 * std::f64::consts::PI * frequency * evals[3]
            } else {
                0.0
            };
            if evals.len() == 5 && evals[4] != 0.0 {
                if strict {
                    return Err(SpiceParseError::Syntax {
                        message: "SIN damping factor (theta) is not supported".to_string(),
                        src: NamedSource::new("<build>", String::new()),
                        bad_span: (0, 0).into(),
                    });
                }
                warnings.push(ParseWarning {
                    message: "SIN damping factor (theta) ignored".to_string(),
                    span: None,
                    file: None,
                });
            }
            Ok(Waveform::Sine {
                amplitude,
                frequency,
                offset,
                phase,
            })
        }
        RawWaveform::Pwl(pairs) => {
            let mut points: Vec<(f64, f64)> = Vec::with_capacity(pairs.len());
            for (t_expr, v_expr) in pairs {
                points.push((eval_or_lift(t_expr, scope)?, eval_or_lift(v_expr, scope)?));
            }
            // Reject decreasing time values.
            for w in points.windows(2) {
                if w[1].0 < w[0].0 {
                    return Err(SpiceParseError::Syntax {
                        message: "PWL time values must be non-decreasing".to_string(),
                        src: NamedSource::new("<build>", String::new()),
                        bad_span: (0, 0).into(),
                    });
                }
            }
            Ok(Waveform::Pwl { points })
        }
    }
}

fn build_analysis(
    raw: &RawAnalysis,
    scope: &ParamScope,
) -> Result<AnalysisRequest, SpiceParseError> {
    match raw {
        RawAnalysis::Op => Ok(AnalysisRequest::Op),
        RawAnalysis::Tran { args } => {
            if args.len() != 2 && args.len() != 3 {
                return Err(SpiceParseError::ArityMismatch {
                    card: ".tran".to_string(),
                    expected: "2 or 3 args (tstep tstop [tstart])".to_string(),
                    got: args.len(),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                });
            }
            let tstep = eval_or_lift(&args[0], scope)?;
            let tstop = eval_or_lift(&args[1], scope)?;
            let tstart = if args.len() == 3 {
                Some(eval_or_lift(&args[2], scope)?)
            } else {
                None
            };
            Ok(AnalysisRequest::Tran {
                tstep,
                tstop,
                tstart,
            })
        }
        RawAnalysis::Dc { source, args } => {
            if args.len() != 3 {
                return Err(SpiceParseError::ArityMismatch {
                    card: ".dc".to_string(),
                    expected: "3 args after source name (start stop step)".to_string(),
                    got: args.len(),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                });
            }
            let start = eval_or_lift(&args[0], scope)?;
            let stop = eval_or_lift(&args[1], scope)?;
            let step = eval_or_lift(&args[2], scope)?;
            Ok(AnalysisRequest::Dc {
                source: source.clone(),
                start,
                stop,
                step,
            })
        }
        RawAnalysis::Ac {
            sweep,
            points,
            fstart,
            fstop,
        } => {
            let pts = eval_or_lift(points, scope)?;
            if pts < 1.0 || !pts.is_finite() {
                return Err(SpiceParseError::Syntax {
                    message: "`.ac` points must be a positive integer".to_string(),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                });
            }
            Ok(AnalysisRequest::Ac {
                sweep: match sweep {
                    AcSweepKind::Dec => AcSweep::Dec,
                    AcSweepKind::Oct => AcSweep::Oct,
                    AcSweepKind::Lin => AcSweep::Lin,
                },
                points: pts as usize,
                fstart: eval_or_lift(fstart, scope)?,
                fstop: eval_or_lift(fstop, scope)?,
            })
        }
    }
}

fn lookup_model<'a>(
    name: &str,
    models: &'a HashMap<String, RawModel>,
    raw: &crate::spice::ast::RawElement,
) -> Result<&'a RawModel, SpiceParseError> {
    models
        .get(name)
        .ok_or_else(|| SpiceParseError::UndefinedModel {
            name: name.to_string(),
            src: NamedSource::new(&*raw.span.file, String::new()),
            bad_span: span_of(raw),
        })
}

fn two_nodes(nodes: &[String], id: &str) -> Result<[String; 2], SpiceParseError> {
    if nodes.len() != 2 {
        return Err(SpiceParseError::ArityMismatch {
            card: id.to_string(),
            expected: "2 nodes".to_string(),
            got: nodes.len(),
            src: NamedSource::new("<build>", String::new()),
            bad_span: (0, 0).into(),
        });
    }
    Ok([nodes[0].clone(), nodes[1].clone()])
}

fn span_of(raw: &crate::spice::ast::RawElement) -> miette::SourceSpan {
    (raw.span.start, raw.span.end.saturating_sub(raw.span.start)).into()
}

fn scope_from_snapshot(snapshot: &HashMap<String, f64>) -> ParamScope {
    let mut s = ParamScope::new();
    for (k, v) in snapshot {
        s.define(k, *v);
    }
    s
}

fn eval_or_lift(expr: &ParamExpr, scope: &ParamScope) -> Result<f64, SpiceParseError> {
    eval(expr, scope).map_err(|e| match e {
        EvalError::UndefinedParam(name) => SpiceParseError::UndefinedParam {
            name,
            src: NamedSource::new("<build>", String::new()),
            bad_span: (0, 0).into(),
        },
        EvalError::DivByZero => SpiceParseError::Syntax {
            message: "division by zero in parameter expression".to_string(),
            src: NamedSource::new("<build>", String::new()),
            bad_span: (0, 0).into(),
        },
        EvalError::Circular(cycle) => SpiceParseError::CircularParam {
            cycle,
            src: NamedSource::new("<build>", String::new()),
            bad_span: (0, 0).into(),
        },
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::spice::ast::{
        BinOp, ParamExpr, RawAnalysis, RawCard, RawElement, RawElementBody, RawModel, RawWaveform,
        SourceKind, Span,
    };
    use crate::spice::flatten::flatten;

    fn span() -> Span {
        Span {
            start: 0,
            end: 0,
            file: Arc::from("test.cir"),
        }
    }

    fn r_card(id: &str, n0: &str, n1: &str, value: f64) -> RawCard {
        RawCard::Element(RawElement {
            id: id.to_string(),
            prefix: 'r',
            nodes: vec![n0.to_string(), n1.to_string()],
            body: RawElementBody::Passive {
                value: ParamExpr::Number(value),
            },
            span: span(),
        })
    }

    fn build_from_cards(cards: Vec<RawCard>) -> Result<ParsedNetlist, SpiceParseError> {
        let mut warnings = Vec::new();
        let flat = flatten(cards, true, &mut warnings)?;
        build_netlist(flat, "test".to_string(), true, &mut warnings)
    }

    #[test]
    fn build_simple_resistor_circuit() {
        let cards = vec![r_card("R1", "a", "0", 1000.0)];
        let netlist = build_from_cards(cards).unwrap();
        assert_eq!(netlist.circuit.components.len(), 1);
        match &netlist.circuit.components[0] {
            CircuitElement::Resistor {
                id,
                nodes,
                resistance,
            } => {
                assert_eq!(id, "R1");
                assert_eq!(nodes, &["a".to_string(), "0".to_string()]);
                assert_eq!(*resistance, 1000.0);
            }
            other => panic!("expected Resistor, got {other:?}"),
        }
    }

    #[test]
    fn bjt_node_order_transposed_from_spice() {
        // SPICE: Q1 c b e qmod (collector, base, emitter)
        // sindr expects [base, collector, emitter]
        let cards = vec![
            RawCard::Model(RawModel {
                name: "qmod".to_string(),
                kind: "NPN".to_string(),
                params: vec![("BF".to_string(), ParamExpr::Number(200.0))],
                span: span(),
            }),
            RawCard::Element(RawElement {
                id: "Q1".to_string(),
                prefix: 'q',
                nodes: vec!["c".to_string(), "b".to_string(), "e".to_string()],
                body: RawElementBody::Bjt {
                    model: "qmod".to_string(),
                    substrate: None,
                },
                span: span(),
            }),
        ];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::Bjt {
                nodes, bf, kind, ..
            } => {
                // The transposition: SPICE [c, b, e] -> sindr [b, c, e]
                assert_eq!(nodes[0], "b", "sindr nodes[0] must be base");
                assert_eq!(nodes[1], "c", "sindr nodes[1] must be collector");
                assert_eq!(nodes[2], "e", "sindr nodes[2] must be emitter");
                assert_eq!(*bf, 200.0);
                assert!(matches!(kind, BjtKind::Npn));
            }
            other => panic!("expected Bjt, got {other:?}"),
        }
    }

    #[test]
    fn diode_missing_model_errors() {
        let cards = vec![RawCard::Element(RawElement {
            id: "D1".to_string(),
            prefix: 'd',
            nodes: vec!["a".to_string(), "k".to_string()],
            body: RawElementBody::Diode {
                model: "ghost".to_string(),
            },
            span: span(),
        })];
        let err = build_from_cards(cards).unwrap_err();
        assert!(matches!(err, SpiceParseError::UndefinedModel { .. }));
    }

    #[test]
    fn unsupported_model_kind_errors_in_strict_mode() {
        let cards = vec![
            RawCard::Model(RawModel {
                name: "nmos1".to_string(),
                kind: "NMOS".to_string(),
                params: vec![],
                span: span(),
            }),
            // Reference it via a D-line so the lookup runs.
            RawCard::Element(RawElement {
                id: "D1".to_string(),
                prefix: 'd',
                nodes: vec!["a".to_string(), "k".to_string()],
                body: RawElementBody::Diode {
                    model: "nmos1".to_string(),
                },
                span: span(),
            }),
        ];
        let err = build_from_cards(cards).unwrap_err();
        assert!(matches!(err, SpiceParseError::UnsupportedModelType { .. }));
    }

    #[test]
    fn pwl_waveform_lifts_to_sindr_pwl() {
        // Vsig n 0 PWL(0 0 1m 5)
        let cards = vec![RawCard::Element(RawElement {
            id: "Vsig".to_string(),
            prefix: 'v',
            nodes: vec!["n".to_string(), "0".to_string()],
            body: RawElementBody::Source {
                kind: SourceKind::Voltage,
                dc: None,
                waveform: Some(RawWaveform::Pwl(vec![
                    (ParamExpr::Number(0.0), ParamExpr::Number(0.0)),
                    (ParamExpr::Number(1e-3), ParamExpr::Number(5.0)),
                ])),
            },
            span: span(),
        })];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::VoltageSource { waveform, .. } => match waveform {
                Some(Waveform::Pwl { points }) => {
                    assert_eq!(points.len(), 2);
                    assert_eq!(points[0], (0.0, 0.0));
                    assert_eq!(points[1], (1e-3, 5.0));
                }
                other => panic!("expected Pwl waveform, got {other:?}"),
            },
            other => panic!("expected VoltageSource, got {other:?}"),
        }
    }

    #[test]
    fn tran_analysis_evaluates() {
        let cards = vec![RawCard::Analysis(RawAnalysis::Tran {
            args: vec![ParamExpr::Number(1e-6), ParamExpr::Number(1e-3)],
        })];
        let netlist = build_from_cards(cards).unwrap();
        assert_eq!(netlist.analyses.len(), 1);
        match &netlist.analyses[0] {
            AnalysisRequest::Tran {
                tstep,
                tstop,
                tstart,
            } => {
                assert_eq!(*tstep, 1e-6);
                assert_eq!(*tstop, 1e-3);
                assert!(tstart.is_none());
            }
            other => panic!("expected Tran, got {other:?}"),
        }
    }

    #[test]
    fn ac_analysis_evaluates() {
        let cards = vec![RawCard::Analysis(RawAnalysis::Ac {
            sweep: AcSweepKind::Dec,
            points: ParamExpr::Number(50.0),
            fstart: ParamExpr::Number(10.0),
            fstop: ParamExpr::Number(1e5),
        })];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.analyses[0] {
            AnalysisRequest::Ac {
                sweep,
                points,
                fstart,
                fstop,
            } => {
                assert!(matches!(sweep, AcSweep::Dec));
                assert_eq!(*points, 50);
                assert_eq!(*fstart, 10.0);
                assert_eq!(*fstop, 1e5);
            }
            other => panic!("expected Ac, got {other:?}"),
        }
    }

    #[test]
    fn passive_value_uses_param_expression() {
        // .param r=2k; R1 a 0 {r}
        let cards = vec![
            RawCard::Param(vec![("r".to_string(), ParamExpr::Number(2000.0))]),
            RawCard::Element(RawElement {
                id: "R1".to_string(),
                prefix: 'r',
                nodes: vec!["a".to_string(), "0".to_string()],
                body: RawElementBody::Passive {
                    value: ParamExpr::Ref("r".to_string()),
                },
                span: span(),
            }),
        ];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::Resistor { resistance, .. } => assert_eq!(*resistance, 2000.0),
            other => panic!("expected Resistor, got {other:?}"),
        }
    }

    #[test]
    fn dc_analysis_param_expression_evaluates() {
        // .param vcc=12; .dc V1 0 {vcc} 1
        let cards = vec![
            RawCard::Param(vec![("vcc".to_string(), ParamExpr::Number(12.0))]),
            RawCard::Analysis(RawAnalysis::Dc {
                source: "V1".to_string(),
                args: vec![
                    ParamExpr::Number(0.0),
                    ParamExpr::Ref("vcc".to_string()),
                    ParamExpr::Number(1.0),
                ],
            }),
        ];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.analyses[0] {
            AnalysisRequest::Dc {
                source,
                start,
                stop,
                step,
            } => {
                assert_eq!(source, "V1");
                assert_eq!(*start, 0.0);
                assert_eq!(*stop, 12.0);
                assert_eq!(*step, 1.0);
            }
            other => panic!("expected Dc, got {other:?}"),
        }
    }

    #[test]
    fn pwl_decreasing_time_errors() {
        let cards = vec![RawCard::Element(RawElement {
            id: "Vsig".to_string(),
            prefix: 'v',
            nodes: vec!["n".to_string(), "0".to_string()],
            body: RawElementBody::Source {
                kind: SourceKind::Voltage,
                dc: None,
                waveform: Some(RawWaveform::Pwl(vec![
                    (ParamExpr::Number(1e-3), ParamExpr::Number(0.0)),
                    (ParamExpr::Number(0.0), ParamExpr::Number(5.0)),
                ])),
            },
            span: span(),
        })];
        let err = build_from_cards(cards).unwrap_err();
        assert!(matches!(err, SpiceParseError::Syntax { .. }));
    }

    #[test]
    fn pulse_waveform_full_args() {
        // PULSE 0 5 1u 1u 1u 1m 2m
        let cards = vec![RawCard::Element(RawElement {
            id: "Vp".to_string(),
            prefix: 'v',
            nodes: vec!["n".to_string(), "0".to_string()],
            body: RawElementBody::Source {
                kind: SourceKind::Voltage,
                dc: None,
                waveform: Some(RawWaveform::Pulse(vec![
                    ParamExpr::Number(0.0),
                    ParamExpr::Number(5.0),
                    ParamExpr::Number(1e-6),
                    ParamExpr::Number(1e-6),
                    ParamExpr::Number(1e-6),
                    ParamExpr::Number(1e-3),
                    ParamExpr::Number(2e-3),
                ])),
            },
            span: span(),
        })];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::VoltageSource { waveform, .. } => match waveform {
                Some(Waveform::Pulse {
                    v1,
                    v2,
                    delay,
                    rise_time,
                    fall_time,
                    pulse_width,
                    period,
                }) => {
                    assert_eq!(*v1, 0.0);
                    assert_eq!(*v2, 5.0);
                    assert_eq!(*delay, 1e-6);
                    assert_eq!(*rise_time, 1e-6);
                    assert_eq!(*fall_time, 1e-6);
                    assert_eq!(*pulse_width, 1e-3);
                    assert_eq!(*period, 2e-3);
                }
                other => panic!("expected Pulse waveform, got {other:?}"),
            },
            other => panic!("expected VoltageSource, got {other:?}"),
        }
    }

    #[test]
    fn sin_waveform_three_args() {
        let cards = vec![RawCard::Element(RawElement {
            id: "Vs".to_string(),
            prefix: 'v',
            nodes: vec!["n".to_string(), "0".to_string()],
            body: RawElementBody::Source {
                kind: SourceKind::Voltage,
                dc: None,
                waveform: Some(RawWaveform::Sin(vec![
                    ParamExpr::Number(0.0),
                    ParamExpr::Number(1.0),
                    ParamExpr::Number(1000.0),
                ])),
            },
            span: span(),
        })];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::VoltageSource { waveform, .. } => match waveform {
                Some(Waveform::Sine {
                    amplitude,
                    frequency,
                    offset,
                    phase,
                }) => {
                    assert_eq!(*amplitude, 1.0);
                    assert_eq!(*frequency, 1000.0);
                    assert_eq!(*offset, 0.0);
                    assert_eq!(*phase, 0.0);
                }
                other => panic!("expected Sine waveform, got {other:?}"),
            },
            other => panic!("expected VoltageSource, got {other:?}"),
        }
    }

    #[test]
    fn dc_value_round_trip() {
        // V1 a 0 12  → bare DC, no waveform
        let cards = vec![RawCard::Element(RawElement {
            id: "V1".to_string(),
            prefix: 'v',
            nodes: vec!["a".to_string(), "0".to_string()],
            body: RawElementBody::Source {
                kind: SourceKind::Voltage,
                dc: Some(ParamExpr::Number(12.0)),
                waveform: None,
            },
            span: span(),
        })];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::VoltageSource {
                voltage, waveform, ..
            } => {
                assert_eq!(*voltage, 12.0);
                assert!(waveform.is_none());
            }
            other => panic!("expected VoltageSource, got {other:?}"),
        }
    }

    #[test]
    fn arithmetic_param_expression_in_resistor() {
        // R1 a 0 (1k + 500)
        let cards = vec![RawCard::Element(RawElement {
            id: "R1".to_string(),
            prefix: 'r',
            nodes: vec!["a".to_string(), "0".to_string()],
            body: RawElementBody::Passive {
                value: ParamExpr::BinOp(
                    Box::new(ParamExpr::Number(1000.0)),
                    BinOp::Add,
                    Box::new(ParamExpr::Number(500.0)),
                ),
            },
            span: span(),
        })];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::Resistor { resistance, .. } => assert_eq!(*resistance, 1500.0),
            other => panic!("expected Resistor, got {other:?}"),
        }
    }
}
