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
//!   `UnsupportedModelType`. `IS`, `N`, `RS`, `CJO` (alias `CJ0`, `CJ`),
//!   `VJ` (alias `PB`), `M` (alias `MJ`), `FC`, `TT`, `BV` and `IBV` fill a
//!   `DiodeModel`.
//! * **Q** — look up `.model`; kind must be `NPN` or `PNP`. `BF` sets `bf`;
//!   `IS`, `BR`, `NF`, `NR` and `VAF` (alias `VA`) fill a `BjtModel`.
//!   Any other D or Q card parameter is a warning in lenient mode and an
//!   error in strict mode, naming it. **SPICE node
//!   order on the Q line is `[c, b, e]`; sindr's `Bjt::nodes` is
//!   `[b, c, e]`** — transpose at this layer.
//! * **E / G / F / H** — evaluate the gain and emit `Vcvs` / `Vccs` /
//!   `Cccs` / `Ccvs` with the nodes as written (sindr uses SPICE's
//!   directions). An `F`/`H` control must be a `V` line in the same scope.
//! * **K** — after its scope's elements are built, replace the two `L`
//!   lines it names with one `Transformer` named after the `K` line.
//! * **X** — should not appear in `flat.elements` (the flattener already
//!   expanded them). Hitting one here is an internal bug.

use crate::element::{
    Behavioural, Bjt, Capacitor, Cccs, Ccvs, CurrentSource, Diode, Inductor, Jfet, Mosfet,
    Resistor, Transformer, Vccs, Vcvs, VoltageSource,
};
use crate::handle::{ComponentId, NodeId};
use std::collections::{BTreeMap, HashMap, HashSet};

use crate::circuit::{
    BjtModel, BjtParasiticCaps, DiodeModel, JfetParasiticCaps, MosfetParasiticCaps,
};
use crate::design::{Block, Instance};
use crate::devices::mosfet::MosfetParams;
use crate::{BjtKind, Circuit, CircuitElement, JfetKind, MosfetKind, Waveform};
use miette::NamedSource;

use crate::spice::analysis::{AcStimulus, AcSweep, AnalysisRequest, MeasureRequest};
use crate::spice::ast::{
    AcSweepKind, NodeValueKind, ParamExpr, PolyControls, RawAnalysis, RawCrossing, RawElementBody,
    RawMeasure, RawMeasureKind, RawModel, RawWaveform, SourceKind,
};
use crate::spice::error::{ParseWarning, SpiceParseError};
use crate::spice::flatten::{FlatElement, Flattened, InstanceRecord};
use crate::spice::param_eval::{eval, eval_error_to_parse_error, ParamScope};
use crate::spice::ParsedNetlist;

/// Upper bound on `.ac` sweep point count. Far above any realistic sweep, but
/// finite so an absurd value (e.g. a fat-fingered exponent) is rejected with a
/// diagnostic instead of saturating the `as usize` cast and aborting the
/// process in `Vec::with_capacity`.
const MAX_AC_POINTS: usize = 10_000_000;

/// Lift a `Flattened` form into a public [`ParsedNetlist`].
pub(crate) fn build_netlist(
    flat: Flattened,
    title: String,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<ParsedNetlist, SpiceParseError> {
    // Each element is built with its nodes as its own subckt body writes
    // them, then filed under its instance; `assemble` turns that into
    // blocks. Building from the renamed id keeps diagnostics naming the
    // full path (`x1.r3`).
    let mut scopes: HashMap<&[String], Vec<CircuitElement>> = HashMap::new();
    // `K` lines, applied per scope once all its `L` lines are built.
    let mut couplings: Vec<Coupling<'_>> = Vec::new();
    for fe in &flat.elements {
        if let RawElementBody::Coupling { inductors, k } = &fe.raw.body {
            let scope = scope_from_snapshot(&fe.scope_snapshot);
            let k = eval_or_lift(k, &scope).map_err(|e| e.locate(&fe.raw.span))?;
            couplings.push(Coupling {
                id: local_id(&fe.raw.id, &fe.instance_path),
                inductors,
                k,
                fe,
            });
            continue;
        }
        let mut raw = fe.raw.clone();
        raw.nodes.clone_from(&fe.local_nodes);
        let local = FlatElement {
            raw,
            local_nodes: Vec::new(),
            instance_path: Vec::new(),
            scope_snapshot: fe.scope_snapshot.clone(),
        };
        let mut element = build_element(&local, &flat.models, strict, warnings)
            .map_err(|e| e.locate(&fe.raw.span))?;
        *element.id_mut() = local_id(&fe.raw.id, &fe.instance_path);
        scopes
            .entry(fe.instance_path.as_slice())
            .or_default()
            .push(element);
    }
    for coupling in couplings {
        let elements = scopes
            .entry(coupling.fe.instance_path.as_slice())
            .or_default();
        couple(elements, coupling)?;
    }
    for fe in &flat.elements {
        if let RawElementBody::CurrentControlled { source, .. } = &fe.raw.body {
            check_control_source(&scopes, fe, source)?;
        }
    }
    let mut ac_stimuli = Vec::new();
    for fe in &flat.elements {
        if let RawElementBody::Source {
            kind,
            ac: Some(spec),
            ..
        } = &fe.raw.body
        {
            let scope = scope_from_snapshot(&fe.scope_snapshot);
            let value_or = |expr: &Option<ParamExpr>, default: f64| match expr {
                Some(e) => eval_or_lift(e, &scope).map_err(|e| e.locate(&fe.raw.span)),
                None => Ok(default),
            };
            ac_stimuli.push(AcStimulus {
                source: fe.raw.id.clone(),
                current: *kind == SourceKind::Current,
                magnitude: value_or(&spec.magnitude, 1.0)?,
                phase_deg: value_or(&spec.phase, 0.0)?,
            });
        }
    }
    let mut circuit = assemble(scopes, &flat.instances);
    circuit.globals.clone_from(&flat.globals);
    let components = circuit
        .flattened()
        .map_err(|e| card_error(format!("internal: {e}")))?
        .into_owned()
        .components;

    let mut analyses: Vec<AnalysisRequest> = Vec::with_capacity(flat.analyses.len());
    let top_scope = scope_from_snapshot(&flat.top_params);
    for (raw, span) in &flat.analyses {
        analyses.push(
            build_analysis(raw, &top_scope)
                .and_then(|a| check_sweep_source(a, &components))
                .map_err(|e| e.locate(span))?,
        );
    }

    // `.ic` / `.nodeset`: a later card overrides an earlier one per node.
    let mut initial_conditions = HashMap::new();
    let mut nodesets = HashMap::new();
    for (kind, values, span) in &flat.node_values {
        let map = match kind {
            NodeValueKind::Ic => &mut initial_conditions,
            NodeValueKind::Nodeset => &mut nodesets,
        };
        for (node, expr) in values {
            let v = eval_or_lift(expr, &top_scope).map_err(|e| e.locate(span))?;
            map.insert(node.clone(), v);
        }
    }

    let mut options = build_options(&flat.options, &top_scope, strict, warnings)?;
    options.nodeset.clone_from(&nodesets);

    let measurements = flat
        .measures
        .iter()
        .map(|(raw, span)| build_measure(raw, &top_scope).map_err(|e| e.locate(span)))
        .collect::<Result<Vec<_>, _>>()?;

    Ok(ParsedNetlist {
        circuit,
        analyses,
        initial_conditions,
        nodesets,
        options,
        measurements,
        ac_stimuli,
        warnings: warnings.clone(),
        source_map: flat.source_map,
        title,
    })
}

/// An element's id inside its subckt body: `x1.x2.r3` → `r3`.
fn local_id(id: &str, instance_path: &[String]) -> String {
    let prefix_len = instance_path.iter().map(|p| p.len() + 1).sum::<usize>();
    id.get(prefix_len..).unwrap_or(id).to_string()
}

/// A `K` line waiting for its scope's `L` lines.
struct Coupling<'a> {
    id: String,
    inductors: &'a [String; 2],
    k: f64,
    fe: &'a FlatElement,
}

/// Replace the two inductors a `K` line names with one
/// [`CircuitElement::Transformer`], where the first of them stood. Each
/// inductor's first node is its dot, as in SPICE.
fn couple(
    elements: &mut Vec<CircuitElement>,
    coupling: Coupling<'_>,
) -> Result<(), SpiceParseError> {
    let Coupling {
        id,
        inductors: [l1, l2],
        k,
        fe,
    } = coupling;
    let fail = |message: String| card_error(message).locate(&fe.raw.span);
    if l1 == l2 {
        return Err(fail(format!("`{id}` couples `{l1}` to itself")));
    }
    let find = |name: &str| {
        elements
            .iter()
            .enumerate()
            .find_map(|(i, e)| match e {
                CircuitElement::Inductor(Inductor {
                    id,
                    nodes,
                    inductance,
                    ..
                }) if id == name => Some((i, nodes.clone(), *inductance)),
                _ => None,
            })
            .ok_or_else(|| {
                fail(format!(
                    "`{id}` couples `{name}`, which isn't an `L` line in the same scope \
                     (or another `K` line already couples it)"
                ))
            })
    };
    let (i1, [p1, q1], l1) = find(l1)?;
    let (i2, [p2, q2], l2) = find(l2)?;
    let transformer = CircuitElement::Transformer(Transformer {
        id,
        nodes: [p1, q1, p2, q2],
        l1,
        l2,
        k,
        saturation: None,
    });
    let (first, last) = (i1.min(i2), i1.max(i2));
    elements.remove(last);
    elements[first] = transformer;
    Ok(())
}

/// An `F` or `H` line's controlling source must be a `V` line in the same
/// scope: the solver would otherwise leave the output at zero.
fn check_control_source(
    scopes: &HashMap<&[String], Vec<CircuitElement>>,
    fe: &FlatElement,
    source: &str,
) -> Result<(), SpiceParseError> {
    let found = scopes
        .get(fe.instance_path.as_slice())
        .is_some_and(|elements| {
            elements
                .iter()
                .any(|e| matches!(e, CircuitElement::VoltageSource(VoltageSource { id, .. }) if id == source))
        });
    if found {
        return Ok(());
    }
    Err(card_error(format!(
        "`{}` is controlled by `{source}`, which isn't a `V` line in the same scope",
        local_id(&fe.raw.id, &fe.instance_path)
    ))
    .locate(&fe.raw.span))
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
                'r' => Ok(CircuitElement::Resistor(Resistor {
                    id: raw.id.clone(),
                    nodes,
                    resistance: v,
                })),
                'l' => Ok(CircuitElement::Inductor(Inductor {
                    id: raw.id.clone(),
                    nodes,
                    inductance: v,
                    saturation: None,
                })),
                'c' => Ok(CircuitElement::Capacitor(Capacitor {
                    id: raw.id.clone(),
                    nodes,
                    capacitance: v,
                })),
                other => Err(SpiceParseError::Syntax {
                    message: format!("internal: unexpected passive prefix `{other}`"),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                }),
            }
        }

        RawElementBody::Source {
            kind, dc, waveform, ..
        } => {
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
                SourceKind::Voltage => Ok(CircuitElement::VoltageSource(VoltageSource {
                    id: raw.id.clone(),
                    nodes,
                    voltage: dc_value,
                    waveform,
                })),
                SourceKind::Current => Ok(CircuitElement::CurrentSource(CurrentSource {
                    id: raw.id.clone(),
                    nodes,
                    current: dc_value,
                    waveform,
                })),
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
            let params = model_params(m, DIODE_KEYS, JUNCTION_NEUTRAL, &scope, strict, warnings)?;
            let nodes = two_nodes(&raw.nodes, &raw.id)?;
            let model = (!params.is_empty()).then(|| {
                let mut model = DiodeModel::default();
                for (key, value) in params {
                    match key {
                        "is" => model.is = value,
                        "n" => model.n = value,
                        "rs" => model.rs = value,
                        "cjo" => model.cjo = value,
                        "vj" => model.vj = value,
                        "m" => model.m = value,
                        "fc" => model.fc = value,
                        "tt" => model.tt = value,
                        "bv" => model.bv = Some(value),
                        "kf" => model.kf = value,
                        "af" => model.af = value,
                        "eg" => model.eg = value,
                        "xti" => model.xti = value,
                        _ => model.ibv = value,
                    }
                }
                model
            });
            Ok(CircuitElement::Diode(Diode {
                id: raw.id.clone(),
                nodes,
                temperature: 300.15,
                model,
            }))
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
                /* base */ NodeId::from(&raw.nodes[1]),
                /* collector */ NodeId::from(&raw.nodes[0]),
                /* emitter */ NodeId::from(&raw.nodes[2]),
            ];
            let params = model_params(m, BJT_KEYS, JUNCTION_NEUTRAL, &scope, strict, warnings)?;
            let mut bf = 100.0;
            let mut model = None::<BjtModel>;
            for (key, value) in params {
                if key == "bf" {
                    bf = value;
                    continue;
                }
                let model = model.get_or_insert_with(BjtModel::default);
                match key {
                    "is" => model.is = value,
                    "br" => model.br = value,
                    "nf" => model.nf = value,
                    "nr" => model.nr = value,
                    "kf" => model.kf = value,
                    "af" => model.af = value,
                    "eg" => model.eg = value,
                    "xti" => model.xti = value,
                    _ => model.vaf = value,
                }
            }
            Ok(CircuitElement::Bjt(Bjt {
                id: raw.id.clone(),
                nodes,
                kind: bjt_kind,
                bf,
                temperature: 300.15,
                parasitic_caps: None::<BjtParasiticCaps>,
                model,
            }))
        }

        RawElementBody::Mosfet {
            model,
            params,
            flags,
        } => build_mosfet(raw, model, params, flags, models, &scope, strict, warnings),

        RawElementBody::Jfet { model, area, flags } => build_jfet(
            raw,
            model,
            area.as_ref(),
            flags,
            models,
            &scope,
            strict,
            warnings,
        ),

        RawElementBody::Poly {
            current,
            controls,
            coefficients,
        } => {
            let coefficients = coefficients
                .iter()
                .map(|c| eval_or_lift(c, &scope))
                .collect::<Result<Vec<_>, _>>()?;
            let variables: Vec<String> = match controls {
                PolyControls::Voltages(pairs) => {
                    pairs.iter().map(|(p, n)| format!("v({p},{n})")).collect()
                }
                PolyControls::Currents(sources) => {
                    sources.iter().map(|s| format!("i({s})")).collect()
                }
            };
            let text = poly_text(&variables, &coefficients);
            let expr = crate::expression::Expression::parse(&text)
                .map_err(|e| card_error(format!("`{}`: {e}", raw.id)))?;
            Ok(CircuitElement::Behavioural(Behavioural {
                id: raw.id.clone(),
                nodes: two_nodes(&raw.nodes, &raw.id)?,
                output: if *current {
                    crate::BehaviouralOutput::Current
                } else {
                    crate::BehaviouralOutput::Voltage
                },
                expr,
            }))
        }

        RawElementBody::Behavioural { current, text } => {
            let expr = crate::expression::Expression::parse_with(text, &|name| scope.get(name))
                .map_err(|e| card_error(format!("`{}`: {e}", raw.id)))?;
            Ok(CircuitElement::Behavioural(Behavioural {
                id: raw.id.clone(),
                nodes: two_nodes(&raw.nodes, &raw.id)?,
                output: if *current {
                    crate::BehaviouralOutput::Current
                } else {
                    crate::BehaviouralOutput::Voltage
                },
                expr,
            }))
        }

        // sindr's controlled sources take SPICE's node order and current
        // direction, so the nodes go across as written.
        RawElementBody::VoltageControlled { value } => {
            let v = eval_or_lift(value, &scope)?;
            let [p, n, cp, cn]: [String; 4] =
                raw.nodes.clone().try_into().map_err(|_| {
                    card_error(format!("`{}` needs four nodes: n+ n- nc+ nc-", raw.id))
                })?;
            let (id, nodes, control_nodes) = (
                raw.id.clone(),
                [p, n].map(NodeId::from),
                [cp, cn].map(NodeId::from),
            );
            Ok(if raw.prefix == 'e' {
                CircuitElement::Vcvs(Vcvs {
                    id,
                    nodes,
                    control_nodes,
                    gain: v,
                })
            } else {
                CircuitElement::Vccs(Vccs {
                    id,
                    nodes,
                    control_nodes,
                    gm: v,
                })
            })
        }

        RawElementBody::CurrentControlled { source, value } => {
            let v = eval_or_lift(value, &scope)?;
            let nodes = two_nodes(&raw.nodes, &raw.id)?;
            let (id, control_source) = (raw.id.clone(), ComponentId::from(source));
            Ok(if raw.prefix == 'h' {
                CircuitElement::Ccvs(Ccvs {
                    id,
                    nodes,
                    control_source,
                    rm: v,
                })
            } else {
                CircuitElement::Cccs(Cccs {
                    id,
                    nodes,
                    control_source,
                    alpha: v,
                })
            })
        }

        RawElementBody::Coupling { .. } => Err(card_error(format!(
            "internal: `K` line `{}` reached the element pass",
            raw.id
        ))),

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

/// SPICE2's `POLY` polynomial of `variables` as expression text. The
/// coefficients run by total degree, and within one degree over the
/// variables' index combinations in nondecreasing order: for two,
/// `p0 + p1·x1 + p2·x2 + p3·x1² + p4·x1·x2 + p5·x2² + p6·x1³ + …`. A
/// one-dimensional polynomial with a single coefficient is its linear term,
/// as in SPICE. Zero coefficients are left out.
fn poly_text(variables: &[String], coefficients: &[f64]) -> String {
    let coefficients: Vec<f64> = if variables.len() == 1 && coefficients.len() == 1 {
        vec![0.0, coefficients[0]]
    } else {
        coefficients.to_vec()
    };
    let n = variables.len();
    let mut terms = Vec::new();
    // `combo` is the current term's variable indices, nondecreasing.
    let mut combo: Vec<usize> = Vec::new();
    for &c in &coefficients {
        if c != 0.0 {
            let mut term = format!("{c:e}");
            for &k in &combo {
                term.push('*');
                term.push_str(&variables[k]);
            }
            terms.push(term);
        }
        // Next combination: the last index that can still grow does, and
        // everything after it restarts at its value; past the last of a
        // degree, the next degree starts at all zeros.
        match (0..combo.len()).rev().find(|&i| combo[i] + 1 < n) {
            Some(i) => {
                let next = combo[i] + 1;
                for slot in &mut combo[i..] {
                    *slot = next;
                }
            }
            None => combo = vec![0; combo.len() + 1],
        }
    }
    if terms.is_empty() {
        return "0".to_string();
    }
    terms.join("+")
}

/// A syntax error with no span of its own: the caller locates it on the card.
/// `.model D` parameters sindr's diode uses.
const DIODE_KEYS: &[&str] = &[
    "is", "n", "rs", "cjo", "vj", "m", "fc", "tt", "bv", "ibv", "kf", "af", "eg", "xti",
];

/// SPICE's alternative names for `.model` parameters, and the name sindr
/// reads them as.
const KEY_ALIASES: &[(&str, &str)] = &[
    ("va", "vaf"),
    ("cj0", "cjo"),
    ("cj", "cjo"),
    ("pb", "vj"),
    ("mj", "m"),
];

/// Diode and BJT card parameters sindr has no field for, read only at the
/// value it assumes: `TNOM`, the temperature a card's values are at, of
/// 27 °C.
const JUNCTION_NEUTRAL: &[(&str, f64)] = &[("tnom", 27.0)];

/// `.model NPN`/`PNP` parameters sindr's Ebers–Moll transistor uses.
const BJT_KEYS: &[&str] = &["is", "bf", "br", "nf", "nr", "vaf", "kf", "af", "eg", "xti"];

/// `.model NMOS`/`PMOS` parameters sindr's level-1 MOSFET uses. `vj` and
/// `m` are `PB` and `MJ` under [`KEY_ALIASES`]; `cbs` is the source–bulk
/// junction, shorted with the bulk tied to the source.
const MOSFET_KEYS: &[&str] = &[
    "level", "kp", "vto", "lambda", "gamma", "phi", "ld", "cgso", "cgdo", "cgbo", "is", "cbd",
    "cbs", "vj", "m", "fc", "rd", "rs",
];

/// `.model NJF`/`PJF` parameters sindr's JFET uses. `vj` is `PB` under
/// [`KEY_ALIASES`]. `is`, `lambda`, `rd`, `rs` and `b` are read only at the
/// value that leaves SPICE's model the same as sindr's.
const JFET_KEYS: &[&str] = &[
    "vto", "beta", "is", "lambda", "rd", "rs", "cgs", "cgd", "vj", "m", "fc", "b",
];

/// MOSFET card parameters sindr's model has no field for, read only at the
/// value that switches them off in SPICE.
const MOSFET_NEUTRAL: &[(&str, f64)] = &[("rd", 0.0), ("rs", 0.0)];

/// The same for a JFET card.
const JFET_NEUTRAL: &[(&str, f64)] = &[
    ("is", 0.0),
    ("lambda", 0.0),
    ("rd", 0.0),
    ("rs", 0.0),
    ("b", 1.0),
];

/// M-line geometry that only scales parameters sindr rejects (`JS`, `CJ`,
/// `CJSW`, `RSH`), so dropping it changes nothing.
const MOSFET_GEOMETRY: &[&str] = &["ad", "as", "pd", "ps", "nrd", "nrs"];

/// Something on a device line sindr can't honour: an error in strict mode,
/// otherwise a warning, once per message.
fn unsupported(
    message: String,
    file: &str,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<(), SpiceParseError> {
    if strict {
        return Err(card_error(message));
    }
    let message = format!("{message}; ignored");
    if !warnings.iter().any(|w| w.message == message) {
        warnings.push(ParseWarning {
            message,
            span: None,
            file: Some(file.to_string()),
        });
    }
    Ok(())
}

/// The model card of a FET line, checked against the kinds it may be.
fn fet_model<'a>(
    raw: &crate::spice::ast::RawElement,
    model: &str,
    models: &'a HashMap<String, RawModel>,
    kinds: [&str; 2],
) -> Result<(&'a RawModel, bool), SpiceParseError> {
    let m = lookup_model(model, models, raw)?;
    let kind = m.kind.to_ascii_lowercase();
    if kind != kinds[0] && kind != kinds[1] {
        return Err(SpiceParseError::UnsupportedModelType {
            kind: m.kind.clone(),
            name: m.name.clone(),
            src: NamedSource::new(&*raw.span.file, String::new()),
            bad_span: span_of(raw),
        });
    }
    Ok((m, kind == kinds[1]))
}

/// A card's parameters as a map, with each `neutral` key checked.
fn fet_params(
    m: &RawModel,
    known: &[&'static str],
    neutral: &[(&str, f64)],
    scope: &ParamScope,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<HashMap<&'static str, f64>, SpiceParseError> {
    let params: HashMap<_, _> = model_params(m, known, &[], scope, strict, warnings)?
        .into_iter()
        .collect();
    for (key, neutral) in neutral {
        if let Some(value) = params.get(key).filter(|v| **v != *neutral) {
            unsupported(
                format!(
                    "`.model {}` {}={value}: sindr's {} model has no {}",
                    m.name,
                    key.to_ascii_uppercase(),
                    m.kind.to_ascii_uppercase(),
                    key.to_ascii_uppercase()
                ),
                &m.span.file,
                strict,
                warnings,
            )?;
        }
    }
    Ok(params)
}

/// Bare flags on a FET line (`OFF`) have no meaning to sindr.
fn fet_flags(
    raw: &crate::spice::ast::RawElement,
    flags: &[String],
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<(), SpiceParseError> {
    for flag in flags {
        unsupported(
            format!(
                "`{}`: `{}` not supported",
                raw.id,
                flag.to_ascii_uppercase()
            ),
            &raw.span.file,
            strict,
            warnings,
        )?;
    }
    Ok(())
}

/// Lower an `M` line onto sindr's three-terminal level-1 MOSFET, as SPICE
/// level 1 with ngspice's defaults (`W = L = 100 µm`, `KP = 2e-5`):
///
/// * `kp = KP·W/(L − 2·LD)·M`; `VTO`, `LAMBDA`, `GAMMA`, `PHI` as written.
/// * `CGSO·W + CGBO·(L − 2·LD)` and `CGDO·W`, times `M`, are the fixed gate
///   capacitances (the bulk is the source, so gate–bulk is gate–source).
/// * The drain–bulk junction (`IS`, default 1e-14 A, `CBD`, `PB`, `MJ`,
///   `FC`, times `M`) is the body diode.
///
/// A `LEVEL` other than 1 is an error. A bulk node other than the source
/// and any other card parameter are errors in strict mode and warnings
/// otherwise.
#[allow(clippy::too_many_arguments)]
fn build_mosfet(
    raw: &crate::spice::ast::RawElement,
    model: &str,
    instance: &[(String, ParamExpr)],
    flags: &[String],
    models: &HashMap<String, RawModel>,
    scope: &ParamScope,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<CircuitElement, SpiceParseError> {
    let (m, pmos) = fet_model(raw, model, models, ["nmos", "pmos"])?;
    let card = fet_params(m, MOSFET_KEYS, MOSFET_NEUTRAL, scope, strict, warnings)?;
    let get = |key: &str, default: f64| card.get(key).copied().unwrap_or(default);
    let file = &*raw.span.file;
    // Another level is a different model, not a parameter to drop: read as
    // level 1 it would solve to the wrong answer, so even lenient mode
    // refuses it.
    let level = get("level", 1.0);
    if level != 1.0 {
        return Err(card_error(format!(
            "`.model {}` LEVEL={level}: sindr's MOSFET is level 1",
            m.name
        )));
    }
    let [d, g, s, b]: [String; 4] = raw.nodes.clone().try_into().map_err(|_| {
        card_error(format!(
            "`{}` needs four nodes: drain gate source bulk",
            raw.id
        ))
    })?;
    if b != s {
        unsupported(
            format!(
                "`{}` bulk `{b}` isn't its source `{s}`: sindr's MOSFET ties the body to the source",
                raw.id
            ),
            file,
            strict,
            warnings,
        )?;
    }
    fet_flags(raw, flags, strict, warnings)?;

    let (mut w, mut l, mut mult) = (100e-6, 100e-6, 1.0);
    for (key, expr) in instance {
        let value = eval_or_lift(expr, scope)?;
        match key.to_ascii_lowercase().as_str() {
            "w" => w = value,
            "l" => l = value,
            "m" => mult = value,
            other if MOSFET_GEOMETRY.contains(&other) => {}
            other => unsupported(
                format!(
                    "`{}`: instance parameter `{}` not supported",
                    raw.id,
                    other.to_ascii_uppercase()
                ),
                file,
                strict,
                warnings,
            )?,
        }
    }
    let l_eff = l - 2.0 * get("ld", 0.0);
    if !(w > 0.0 && l_eff > 0.0 && mult > 0.0) {
        return Err(card_error(format!(
            "`{}` needs W, L − 2·LD and M above 0 (W={w}, L−2·LD={l_eff}, M={mult})",
            raw.id
        )));
    }

    let params = MosfetParams {
        kp: get("kp", 2e-5) * w / l_eff * mult,
        vto: get("vto", 0.0),
        lambda: get("lambda", 0.0),
        gamma: get("gamma", 0.0),
        phi: get("phi", 0.6),
        ..MosfetParams::default()
    };
    let cgs = (get("cgso", 0.0) * w + get("cgbo", 0.0) * l_eff) * mult;
    let cgd = get("cgdo", 0.0) * w * mult;
    let parasitic_caps = (cgs > 0.0 || cgd > 0.0).then_some(MosfetParasiticCaps { cgs, cgd });
    let is = get("is", 1e-14) * mult;
    let cbd = get("cbd", 0.0) * mult;
    let body_diode = (is > 0.0 || cbd > 0.0).then(|| DiodeModel {
        is,
        cjo: cbd,
        vj: get("vj", 0.8),
        m: get("m", 0.5),
        fc: get("fc", 0.5),
        ..DiodeModel::default()
    });
    Ok(CircuitElement::Mosfet(Mosfet {
        id: raw.id.clone(),
        nodes: [g, d, s].map(NodeId::from),
        kind: if pmos {
            MosfetKind::Pmos
        } else {
            MosfetKind::Nmos
        },
        params,
        parasitic_caps,
        body_diode,
    }))
}

/// Lower a `J` line onto sindr's JFET: `idss = BETA·VTO²·area`, and the
/// pinch-off `vp` is `VTO` (default −2 V), signed as sindr's
/// [`JfetKind`] expects (negative for N-channel, positive for P-channel).
/// `CGS·area` and `CGD·area` with `PB`, `M` and `FC` are the gate
/// capacitances. sindr's JFET has no gate junction current, channel-length
/// modulation or series resistance, so a card that sets them is refused in
/// strict mode.
#[allow(clippy::too_many_arguments)]
fn build_jfet(
    raw: &crate::spice::ast::RawElement,
    model: &str,
    area: Option<&ParamExpr>,
    flags: &[String],
    models: &HashMap<String, RawModel>,
    scope: &ParamScope,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<CircuitElement, SpiceParseError> {
    let (m, pjf) = fet_model(raw, model, models, ["njf", "pjf"])?;
    let card = fet_params(m, JFET_KEYS, JFET_NEUTRAL, scope, strict, warnings)?;
    fet_flags(raw, flags, strict, warnings)?;
    let [d, g, s]: [String; 3] =
        raw.nodes.clone().try_into().map_err(|_| {
            card_error(format!("`{}` needs three nodes: drain gate source", raw.id))
        })?;
    let vto = card.get("vto").copied().unwrap_or(-2.0);
    let beta = card.get("beta").copied().unwrap_or(1e-4);
    let area = area
        .map(|a| eval_or_lift(a, scope))
        .transpose()?
        .unwrap_or(1.0);
    if !(vto < 0.0 && beta > 0.0 && area > 0.0) {
        return Err(card_error(format!(
            "`{}` needs VTO below 0 and BETA and area above 0 (VTO={vto}, BETA={beta}, area={area})",
            raw.id
        )));
    }
    let get = |key: &str, default: f64| card.get(key).copied().unwrap_or(default);
    let defaults = JfetParasiticCaps::default();
    let caps = JfetParasiticCaps {
        cgs: get("cgs", 0.0) * area,
        cgd: get("cgd", 0.0) * area,
        pb: get("vj", defaults.pb),
        m: get("m", defaults.m),
        fc: get("fc", defaults.fc),
    };
    Ok(CircuitElement::Jfet(Jfet {
        id: raw.id.clone(),
        nodes: [g, d, s].map(NodeId::from),
        kind: if pjf {
            JfetKind::PChannel
        } else {
            JfetKind::NChannel
        },
        idss: beta * vto * vto * area,
        vp: if pjf { -vto } else { vto },
        parasitic_caps: caps.stores_charge().then_some(caps),
    }))
}

/// `.model` parameters that describe the part rather than its behaviour
/// (vendor model libraries add them). They are dropped without a word,
/// even in strict mode, as SPICE simulators drop them.
const ANNOTATION_KEYS: &[&str] = &["mfg", "type", "vceo", "icrating", "iave", "vpk"];

/// The parameters of card `m` that `known` lists, as `(key, value)` in card
/// order, so a repeated key's last value wins when applied in turn. Keys are
/// lowercased and [`KEY_ALIASES`] read as the name they stand for.
///
/// Any other parameter can't be honoured: an error naming it in strict mode,
/// otherwise one warning per card listing them all.
fn model_params(
    m: &RawModel,
    known: &[&'static str],
    neutral: &[(&str, f64)],
    scope: &ParamScope,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<Vec<(&'static str, f64)>, SpiceParseError> {
    let mut params = Vec::new();
    let mut ignored = Vec::new();
    let mut off_neutral = false;
    for (key, expr) in &m.params {
        let lower = key.to_ascii_lowercase();
        let lower = KEY_ALIASES
            .iter()
            .find(|(alias, _)| *alias == lower)
            .map_or(lower, |(_, name)| name.to_string());
        if let Some((_, at)) = neutral.iter().find(|(k, _)| *k == lower) {
            let value = eval_or_lift(expr, scope)?;
            if (value - at).abs() > 1e-9 * at.abs() {
                ignored.push(format!("{}={value}", key.to_ascii_uppercase()));
                off_neutral = true;
            }
            continue;
        }
        match known.iter().find(|k| **k == lower) {
            Some(k) => params.push((*k, eval_or_lift(expr, scope)?)),
            None if ANNOTATION_KEYS.contains(&lower.as_str()) => {}
            None => ignored.push(key.to_ascii_uppercase()),
        }
    }
    if ignored.is_empty() {
        return Ok(params);
    }
    let what = format!(
        "`.model {}` parameter{} {}",
        m.name,
        if ignored.len() == 1 { "" } else { "s" },
        ignored.join(", ")
    );
    let hint = if off_neutral {
        "; its values are at TNOM=27"
    } else {
        ""
    };
    if strict {
        return Err(card_error(format!(
            "{what} not supported by sindr's {} model{hint}",
            m.kind.to_ascii_uppercase()
        )));
    }
    let message = format!("{what} ignored (sindr's model doesn't use them{hint})");
    if !warnings.iter().any(|w| w.message == message) {
        warnings.push(ParseWarning {
            message,
            span: None,
            file: Some(m.span.file.to_string()),
        });
    }
    Ok(params)
}

/// Turn per-instance element lists into a [`Circuit`] with one [`Block`]
/// per distinct subckt body. Instances that bind different `.param`
/// values can give one subckt different bodies; each gets its own block,
/// named `name`, `name_2`, `name_3`, ….
fn assemble(
    mut scopes: HashMap<&[String], Vec<CircuitElement>>,
    records: &[InstanceRecord],
) -> Circuit {
    // Pass 1, bottom-up: group instances whose bodies are identical into
    // classes. Children come after their parent in pre-order, so walking
    // backwards classifies every child before its parent's body is built.
    // Instances refer to their block by class index until pass 2.
    let mut class_of: Vec<usize> = vec![0; records.len()];
    let mut classes: Vec<(&str, Block)> = Vec::new();
    let children = |path: &[String], class_of: &[usize]| -> Vec<Instance> {
        records
            .iter()
            .zip(class_of)
            .filter(|(r, _)| r.path.len() == path.len() + 1 && r.path.starts_with(path))
            .map(|(r, class)| Instance {
                id: r.path.last().cloned().unwrap_or_default(),
                block: class.to_string(),
                nodes: r.local_nodes.clone(),
                variant: None,
            })
            .collect()
    };
    for (i, record) in records.iter().enumerate().rev() {
        let block = Block {
            ports: record.ports.clone(),
            components: scopes.remove(record.path.as_slice()).unwrap_or_default(),
            instances: children(&record.path, &class_of),
            ..Block::default()
        };
        class_of[i] = match classes
            .iter()
            .position(|(name, b)| *name == record.subckt && *b == block)
        {
            Some(class) => class,
            None => {
                classes.push((&record.subckt, block));
                classes.len() - 1
            }
        };
    }

    // Pass 2: name classes in order of first use: `name`, `name_2`, ….
    let subckt_names: HashSet<&str> = records.iter().map(|r| r.subckt.as_str()).collect();
    let mut names: Vec<Option<String>> = vec![None; classes.len()];
    let mut taken: HashSet<String> = HashSet::new();
    for &class in &class_of {
        if names[class].is_some() {
            continue;
        }
        let subckt = classes[class].0;
        let mut name = subckt.to_string();
        let mut k = 1;
        while taken.contains(&name) || (k > 1 && subckt_names.contains(name.as_str())) {
            k += 1;
            name = format!("{subckt}_{k}");
        }
        taken.insert(name.clone());
        names[class] = Some(name);
    }
    let name_of = |class: &str| -> String {
        class
            .parse::<usize>()
            .ok()
            .and_then(|c| names.get(c).cloned().flatten())
            .unwrap_or_default()
    };
    let rename = |instances: &mut Vec<Instance>| {
        for instance in instances {
            instance.block = name_of(&instance.block);
        }
    };

    let mut blocks = BTreeMap::new();
    for (class, (_, mut block)) in classes.into_iter().enumerate() {
        rename(&mut block.instances);
        if let Some(name) = names[class].clone() {
            blocks.insert(name, block);
        }
    }
    let mut instances = children(&[], &class_of);
    rename(&mut instances);

    Circuit {
        components: scopes.remove([].as_slice()).unwrap_or_default(),
        ground_node: "0".to_string(),
        instances,
        blocks,
        ..Circuit::default()
    }
}

/// The [`SolverOptions`](crate::SolverOptions) the deck's `.options`
/// cards set; a later card overrides an earlier one. `reltol`, `vntol`,
/// `trtol` and `itl1` are read. Any other option isn't modelled: an error in
/// strict mode, a warning (and ignored) otherwise.
fn build_options(
    cards: &[crate::spice::flatten::OptionsCard],
    scope: &ParamScope,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<crate::SolverOptions, SpiceParseError> {
    let mut options = crate::SolverOptions::default();
    for (pairs, span) in cards {
        for (name, expr) in pairs {
            let key = name.to_ascii_lowercase();
            let value = |what: &str| -> Result<f64, SpiceParseError> {
                let expr = expr.as_ref().ok_or_else(|| {
                    card_error(format!("`.options` {what} needs a value: `{what}=<value>`"))
                })?;
                let v = eval_or_lift(expr, scope)?;
                if v <= 0.0 {
                    return Err(card_error(format!(
                        "`.options` {what} must be positive (got {v})"
                    )));
                }
                Ok(v)
            };
            match key.as_str() {
                "reltol" => options.reltol = value("reltol").map_err(|e| e.locate(span))?,
                "vntol" => options.vntol = value("vntol").map_err(|e| e.locate(span))?,
                "trtol" => options.trtol = value("trtol").map_err(|e| e.locate(span))?,
                "itl1" => {
                    let v = value("itl1").map_err(|e| e.locate(span))?;
                    if v.fract() != 0.0 {
                        return Err(card_error(format!(
                            "`.options` itl1 must be a whole number of iterations (got {v})"
                        ))
                        .locate(span));
                    }
                    options.max_iterations = v as usize;
                }
                _ => {
                    let message = format!(
                        "`.options` {name} isn't supported; sindr reads reltol, vntol, trtol and itl1"
                    );
                    if strict {
                        return Err(card_error(message).locate(span));
                    }
                    warnings.push(ParseWarning {
                        message,
                        span: Some((span.start, span.end - span.start).into()),
                        file: Some(span.file.to_string()),
                    });
                }
            }
        }
    }
    Ok(options)
}

fn card_error(message: String) -> SpiceParseError {
    SpiceParseError::Syntax {
        message,
        src: NamedSource::new("<build>", String::new()),
        bad_span: (0, 0).into(),
    }
}

/// A `.dc` card must sweep a source the deck defines: an independent voltage
/// or current source. Caught here, it points at the card; left to the solver,
/// it fails with no line.
fn check_sweep_source(
    analysis: AnalysisRequest,
    components: &[CircuitElement],
) -> Result<AnalysisRequest, SpiceParseError> {
    if let AnalysisRequest::Dc { source, outer, .. } = &analysis {
        let sources = std::iter::once(source).chain(outer.as_ref().map(|o| &o.source));
        for source in sources {
            let found = components.iter().any(|c| {
                matches!(
                    c,
                    CircuitElement::VoltageSource(VoltageSource { .. })
                        | CircuitElement::CurrentSource(CurrentSource { .. })
                ) && c.id().eq_ignore_ascii_case(source)
            });
            if !found {
                return Err(card_error(format!(
                    "`.dc` sweeps `{source}`, which is not a voltage or current source in the deck"
                )));
            }
        }
        if let Some(o) = outer {
            if o.source.eq_ignore_ascii_case(source) {
                return Err(card_error(format!(
                    "`.dc` sweeps `{source}` twice; the two sources must differ"
                )));
            }
        }
    }
    Ok(analysis)
}

/// `start stop step` of one `.dc` source, evaluated and checked.
fn build_dc_axis(
    args: &[ParamExpr],
    scope: &ParamScope,
) -> Result<(f64, f64, f64), SpiceParseError> {
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
    // A zero step never reaches stop, and one pointing away from it
    // never gets there either.
    if start != stop && (step == 0.0 || (stop - start).signum() != step.signum()) {
        return Err(card_error(format!(
            "`.dc` step {step} doesn't move from {start} towards {stop}"
        )));
    }
    Ok((start, stop, step))
}

fn build_measure(raw: &RawMeasure, scope: &ParamScope) -> Result<MeasureRequest, SpiceParseError> {
    use crate::measure::{Crossing, Measure, Measurement};
    let value = |expr: &ParamExpr| eval_or_lift(expr, scope);
    let crossing = |c: &RawCrossing| -> Result<Crossing, SpiceParseError> {
        Ok(Crossing {
            signal: c.signal.clone(),
            level: value(&c.level)?,
            edge: c.edge,
            occurrence: c.occurrence,
        })
    };
    let measure = match &raw.kind {
        RawMeasureKind::Stat(signal, stat) => Measure::stat(signal.clone(), *stat),
        RawMeasureKind::Find(signal, at) => Measure::find(signal.clone(), value(at)?),
        RawMeasureKind::When(c) => Measure::When(crossing(c)?),
        RawMeasureKind::Delay(trig, targ) => Measure::Delay {
            trig: crossing(trig)?,
            targ: crossing(targ)?,
        },
    };
    let mut measurement = Measurement::new(raw.name.clone(), measure);
    measurement.from = raw.from.as_ref().map(value).transpose()?;
    measurement.to = raw.to.as_ref().map(value).transpose()?;
    Ok(MeasureRequest {
        analysis: raw.analysis,
        measurement,
    })
}

fn build_analysis(
    raw: &RawAnalysis,
    scope: &ParamScope,
) -> Result<AnalysisRequest, SpiceParseError> {
    match raw {
        RawAnalysis::Op => Ok(AnalysisRequest::Op),
        RawAnalysis::Tran { args, uic } => {
            if !(2..=4).contains(&args.len()) {
                return Err(SpiceParseError::ArityMismatch {
                    card: ".tran".to_string(),
                    expected: "2 to 4 args (tstep tstop [tstart [tmax]])".to_string(),
                    got: args.len(),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                });
            }
            let tstep = eval_or_lift(&args[0], scope)?;
            let tstop = eval_or_lift(&args[1], scope)?;
            let tstart = match args.get(2) {
                Some(arg) => Some(eval_or_lift(arg, scope)?),
                None => None,
            };
            let tmax = match args.get(3) {
                Some(arg) => Some(eval_or_lift(arg, scope)?),
                None => None,
            };
            Ok(AnalysisRequest::Tran {
                tstep,
                tstop,
                tstart,
                tmax,
                uic: *uic,
            })
        }
        RawAnalysis::Dc {
            source,
            args,
            outer,
        } => {
            let (start, stop, step) = build_dc_axis(args, scope)?;
            let outer = match outer {
                Some((source, args)) => {
                    let (start, stop, step) = build_dc_axis(args, scope)?;
                    Some(super::analysis::DcSweepSpec {
                        source: source.clone(),
                        start,
                        stop,
                        step,
                    })
                }
                None => None,
            };
            Ok(AnalysisRequest::Dc {
                source: source.clone(),
                start,
                stop,
                step,
                outer,
            })
        }
        RawAnalysis::Sens { output } => Ok(AnalysisRequest::Sens {
            output: output.clone(),
        }),
        RawAnalysis::SensAc {
            output,
            sweep,
            points,
            fstart,
            fstop,
        } => {
            // The same sweep checks as `.ac`.
            let sweep = RawAnalysis::Ac {
                sweep: *sweep,
                points: points.clone(),
                fstart: fstart.clone(),
                fstop: fstop.clone(),
            };
            match build_analysis(&sweep, scope)? {
                AnalysisRequest::Ac {
                    sweep,
                    points,
                    fstart,
                    fstop,
                } => Ok(AnalysisRequest::SensAc {
                    output: output.clone(),
                    sweep,
                    points,
                    fstart,
                    fstop,
                }),
                _ => Err(card_error("`.sens ... ac`: not an AC sweep".into())),
            }
        }
        RawAnalysis::Pz {
            nodes,
            transfer,
            find,
        } => Ok(AnalysisRequest::Pz {
            input: nodes[0].clone(),
            input_reference: nodes[1].clone(),
            output: nodes[2].clone(),
            output_reference: nodes[3].clone(),
            transfer: *transfer,
            find: *find,
        }),
        RawAnalysis::Tf { output, input } => Ok(AnalysisRequest::Tf {
            output: output.clone(),
            input: input.clone(),
        }),
        RawAnalysis::Noise {
            node,
            reference,
            input,
            sweep,
            points,
            fstart,
            fstop,
        } => {
            let pts = eval_or_lift(points, scope)?;
            if pts < 1.0 || pts > MAX_AC_POINTS as f64 {
                return Err(card_error(format!(
                    "`.noise` points must be a positive integer in 1..={MAX_AC_POINTS} (got {pts})"
                )));
            }
            let fstart = eval_or_lift(fstart, scope)?;
            let fstop = eval_or_lift(fstop, scope)?;
            if fstart <= 0.0 {
                return Err(card_error(format!(
                    "`.noise` fstart must be above 0, where flicker noise is finite (got {fstart})"
                )));
            }
            if fstart > fstop {
                return Err(card_error(format!(
                    "`.noise` fstart {fstart} is above fstop {fstop}"
                )));
            }
            Ok(AnalysisRequest::Noise {
                node: node.clone(),
                reference: reference.clone(),
                input: input.clone(),
                sweep: match sweep {
                    AcSweepKind::Dec => AcSweep::Dec,
                    AcSweepKind::Oct => AcSweep::Oct,
                    AcSweepKind::Lin => AcSweep::Lin,
                },
                points: pts as usize,
                fstart,
                fstop,
            })
        }
        RawAnalysis::Ac {
            sweep,
            points,
            fstart,
            fstop,
        } => {
            let pts = eval_or_lift(points, scope)?;
            // Bound the count before the `as usize` cast: a float→int cast
            // saturates (e.g. `1e18 as usize == usize::MAX`), which would make
            // the downstream `Vec::with_capacity` abort the process.
            if pts < 1.0 || pts > MAX_AC_POINTS as f64 {
                return Err(SpiceParseError::Syntax {
                    message: format!(
                        "`.ac` points must be a positive integer in 1..={MAX_AC_POINTS} (got {pts})"
                    ),
                    src: NamedSource::new("<build>", String::new()),
                    bad_span: (0, 0).into(),
                });
            }
            let fstart = eval_or_lift(fstart, scope)?;
            let fstop = eval_or_lift(fstop, scope)?;
            let log = !matches!(sweep, AcSweepKind::Lin);
            if (log && fstart <= 0.0) || fstart < 0.0 {
                return Err(card_error(format!(
                    "`.ac` fstart must be {} (got {fstart})",
                    if log {
                        "above 0 for a log sweep"
                    } else {
                        "at least 0"
                    }
                )));
            }
            if fstart > fstop {
                return Err(card_error(format!(
                    "`.ac` fstart {fstart} is above fstop {fstop}"
                )));
            }
            Ok(AnalysisRequest::Ac {
                sweep: match sweep {
                    AcSweepKind::Dec => AcSweep::Dec,
                    AcSweepKind::Oct => AcSweep::Oct,
                    AcSweepKind::Lin => AcSweep::Lin,
                },
                points: pts as usize,
                fstart,
                fstop,
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

fn two_nodes(nodes: &[String], id: &str) -> Result<[NodeId; 2], SpiceParseError> {
    if nodes.len() != 2 {
        return Err(SpiceParseError::ArityMismatch {
            card: id.to_string(),
            expected: "2 nodes".to_string(),
            got: nodes.len(),
            src: NamedSource::new("<build>", String::new()),
            bad_span: (0, 0).into(),
        });
    }
    Ok([NodeId::from(&nodes[0]), NodeId::from(&nodes[1])])
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

/// Evaluates `expr`, rejecting NaN and ±∞: `1e400` or `{1/0}` would
/// otherwise reach the solver as an infinite component value, and an
/// analysis directive as a degenerate sweep.
fn eval_or_lift(expr: &ParamExpr, scope: &ParamScope) -> Result<f64, SpiceParseError> {
    let v = eval(expr, scope).map_err(|e| eval_error_to_parse_error(e, "<build>"))?;
    if !v.is_finite() {
        return Err(card_error(format!("value is {v}, not a finite number")));
    }
    Ok(v)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {

    #[test]
    fn poly_coefficients_follow_spice2s_order() {
        let x = |n: usize| (1..=n).map(|k| format!("x{k}")).collect::<Vec<_>>();
        // Two dimensions, to the cubic.
        assert_eq!(
            poly_text(&x(2), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]),
            "1e0+2e0*x1+3e0*x2+4e0*x1*x1+5e0*x1*x2+6e0*x2*x2\
             +7e0*x1*x1*x1+8e0*x1*x1*x2+9e0*x1*x2*x2+1e1*x2*x2*x2"
        );
        // Three dimensions, second degree.
        let three = poly_text(&x(3), &[0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert_eq!(
            three,
            "1e0*x1*x1+2e0*x1*x2+3e0*x1*x3+4e0*x2*x2+5e0*x2*x3+6e0*x3*x3"
        );
        // One coefficient in one dimension is the linear term.
        assert_eq!(poly_text(&x(1), &[3.0]), "3e0*x1");
        assert_eq!(poly_text(&x(1), &[3.0, 0.0, -2.0]), "3e0+-2e0*x1*x1");
        assert_eq!(poly_text(&x(2), &[0.0]), "0");
    }
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
            src: Arc::default(),
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
            CircuitElement::Resistor(Resistor {
                id,
                nodes,
                resistance,
            }) => {
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
            CircuitElement::Bjt(Bjt {
                nodes, bf, kind, ..
            }) => {
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
                ac: None,
                waveform: Some(RawWaveform::Pwl(vec![
                    (ParamExpr::Number(0.0), ParamExpr::Number(0.0)),
                    (ParamExpr::Number(1e-3), ParamExpr::Number(5.0)),
                ])),
            },
            span: span(),
        })];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::VoltageSource(VoltageSource { waveform, .. }) => match waveform {
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
        let cards = vec![RawCard::Analysis(
            RawAnalysis::Tran {
                args: vec![ParamExpr::Number(1e-6), ParamExpr::Number(1e-3)],
                uic: false,
            },
            Span::detached(),
        )];
        let netlist = build_from_cards(cards).unwrap();
        assert_eq!(netlist.analyses.len(), 1);
        match &netlist.analyses[0] {
            AnalysisRequest::Tran {
                tstep,
                tstop,
                tstart,
                ..
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
        let cards = vec![RawCard::Analysis(
            RawAnalysis::Ac {
                sweep: AcSweepKind::Dec,
                points: ParamExpr::Number(50.0),
                fstart: ParamExpr::Number(10.0),
                fstop: ParamExpr::Number(1e5),
            },
            Span::detached(),
        )];
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
    fn ac_points_absurd_value_is_rejected_not_panicked() {
        // A fat-fingered exponent saturates the `as usize` cast to usize::MAX,
        // which would abort in Vec::with_capacity. It must surface as an error.
        let cards = vec![RawCard::Analysis(
            RawAnalysis::Ac {
                sweep: AcSweepKind::Dec,
                points: ParamExpr::Number(1e18),
                fstart: ParamExpr::Number(10.0),
                fstop: ParamExpr::Number(1e5),
            },
            Span::detached(),
        )];
        let err = build_from_cards(cards).unwrap_err();
        assert!(
            matches!(err, SpiceParseError::Syntax { .. }),
            "expected Syntax error for out-of-range points, got {err:?}"
        );
    }

    #[test]
    fn ac_points_just_above_max_is_rejected() {
        let cards = vec![RawCard::Analysis(
            RawAnalysis::Ac {
                sweep: AcSweepKind::Dec,
                points: ParamExpr::Number((MAX_AC_POINTS + 1) as f64),
                fstart: ParamExpr::Number(10.0),
                fstop: ParamExpr::Number(1e5),
            },
            Span::detached(),
        )];
        assert!(build_from_cards(cards).is_err());
    }

    #[test]
    fn tran_nonfinite_arg_is_rejected() {
        let cards = vec![RawCard::Analysis(
            RawAnalysis::Tran {
                args: vec![ParamExpr::Number(f64::INFINITY), ParamExpr::Number(1e-3)],
                uic: false,
            },
            Span::detached(),
        )];
        let err = build_from_cards(cards).unwrap_err();
        assert!(
            matches!(err, SpiceParseError::Syntax { .. }),
            "expected Syntax error for non-finite tstep, got {err:?}"
        );
    }

    #[test]
    fn dc_nonfinite_arg_is_rejected() {
        let cards = vec![RawCard::Analysis(
            RawAnalysis::Dc {
                source: "V1".to_string(),
                args: vec![
                    ParamExpr::Number(0.0),
                    ParamExpr::Number(f64::NAN),
                    ParamExpr::Number(1.0),
                ],
                outer: None,
            },
            Span::detached(),
        )];
        assert!(build_from_cards(cards).is_err());
    }

    #[test]
    fn undefined_param_keeps_precise_variant_through_build() {
        // Regression: the param-error lift must preserve UndefinedParam rather
        // than collapsing to a generic Syntax error.
        let cards = vec![RawCard::Element(RawElement {
            id: "R1".to_string(),
            prefix: 'r',
            nodes: vec!["a".to_string(), "0".to_string()],
            body: RawElementBody::Passive {
                value: ParamExpr::Ref("missing".to_string()),
            },
            span: span(),
        })];
        let err = build_from_cards(cards).unwrap_err();
        assert!(
            matches!(err, SpiceParseError::UndefinedParam { .. }),
            "expected UndefinedParam, got {err:?}"
        );
    }

    #[test]
    fn undefined_param_in_param_list_keeps_precise_variant_through_flatten() {
        // The flatten-layer lift must also preserve UndefinedParam (previously
        // it flattened every EvalError into Syntax).
        let cards = vec![RawCard::Param(
            vec![("a".to_string(), ParamExpr::Ref("missing".to_string()))],
            Span::detached(),
        )];
        let err = build_from_cards(cards).unwrap_err();
        assert!(
            matches!(err, SpiceParseError::UndefinedParam { .. }),
            "expected UndefinedParam from flatten, got {err:?}"
        );
    }

    #[test]
    fn passive_value_uses_param_expression() {
        // .param r=2k; R1 a 0 {r}
        let cards = vec![
            RawCard::Param(
                vec![("r".to_string(), ParamExpr::Number(2000.0))],
                Span::detached(),
            ),
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
            CircuitElement::Resistor(Resistor { resistance, .. }) => {
                assert_eq!(*resistance, 2000.0)
            }
            other => panic!("expected Resistor, got {other:?}"),
        }
    }

    #[test]
    fn dc_analysis_param_expression_evaluates() {
        // .param vcc=12; V1 a 0 0; .dc V1 0 {vcc} 1
        let cards = vec![
            RawCard::Element(RawElement {
                id: "V1".to_string(),
                prefix: 'v',
                nodes: vec!["a".to_string(), "0".to_string()],
                body: RawElementBody::Source {
                    kind: SourceKind::Voltage,
                    dc: Some(ParamExpr::Number(0.0)),
                    ac: None,
                    waveform: None,
                },
                span: span(),
            }),
            RawCard::Param(
                vec![("vcc".to_string(), ParamExpr::Number(12.0))],
                Span::detached(),
            ),
            RawCard::Analysis(
                RawAnalysis::Dc {
                    source: "V1".to_string(),
                    args: vec![
                        ParamExpr::Number(0.0),
                        ParamExpr::Ref("vcc".to_string()),
                        ParamExpr::Number(1.0),
                    ],
                    outer: None,
                },
                Span::detached(),
            ),
        ];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.analyses[0] {
            AnalysisRequest::Dc {
                source,
                start,
                stop,
                step,
                ..
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
                ac: None,
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
                ac: None,
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
            CircuitElement::VoltageSource(VoltageSource { waveform, .. }) => match waveform {
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
                ac: None,
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
            CircuitElement::VoltageSource(VoltageSource { waveform, .. }) => match waveform {
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
                ac: None,
                waveform: None,
            },
            span: span(),
        })];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::VoltageSource(VoltageSource {
                voltage, waveform, ..
            }) => {
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
            CircuitElement::Resistor(Resistor { resistance, .. }) => {
                assert_eq!(*resistance, 1500.0)
            }
            other => panic!("expected Resistor, got {other:?}"),
        }
    }

    fn build_lenient(cards: Vec<RawCard>) -> (ParsedNetlist, Vec<ParseWarning>) {
        let mut warnings = Vec::new();
        let flat = flatten(cards, false, &mut warnings).unwrap();
        let netlist = build_netlist(flat, "test".to_string(), false, &mut warnings).unwrap();
        (netlist, warnings)
    }

    fn model_card(name: &str, kind: &str, params: &[(&str, f64)]) -> RawCard {
        RawCard::Model(RawModel {
            name: name.to_string(),
            kind: kind.to_string(),
            params: params
                .iter()
                .map(|(k, v)| (k.to_string(), ParamExpr::Number(*v)))
                .collect(),
            span: span(),
        })
    }

    fn diode_card(id: &str, model: &str) -> RawCard {
        RawCard::Element(RawElement {
            id: id.to_string(),
            prefix: 'd',
            nodes: vec!["a".to_string(), "0".to_string()],
            body: RawElementBody::Diode {
                model: model.to_string(),
            },
            span: span(),
        })
    }

    fn bjt_card(model: &str) -> RawCard {
        RawCard::Element(RawElement {
            id: "Q1".to_string(),
            prefix: 'q',
            nodes: vec!["c".to_string(), "b".to_string(), "e".to_string()],
            body: RawElementBody::Bjt {
                model: model.to_string(),
                substrate: None,
            },
            span: span(),
        })
    }

    #[test]
    fn diode_card_fills_the_model() {
        let cards = vec![
            model_card(
                "d1n4148",
                "D",
                &[("IS", 2.52e-9), ("n", 1.752), ("Rs", 0.568)],
            ),
            diode_card("D1", "d1n4148"),
        ];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::Diode(Diode { model, .. }) => assert_eq!(
                *model,
                Some(DiodeModel {
                    is: 2.52e-9,
                    n: 1.752,
                    rs: 0.568,
                    ..DiodeModel::default()
                })
            ),
            other => panic!("expected Diode, got {other:?}"),
        }
    }

    #[test]
    fn diode_card_fills_charge_and_breakdown_under_any_alias() {
        let cards = vec![
            model_card(
                "d",
                "D",
                &[
                    ("CJ0", 4e-12),
                    ("PB", 0.6),
                    ("MJ", 0.33),
                    ("FC", 0.4),
                    ("TT", 5.76e-9),
                    ("BV", 100.0),
                    ("IBV", 1e-4),
                ],
            ),
            diode_card("D1", "d"),
        ];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::Diode(Diode { model, .. }) => assert_eq!(
                *model,
                Some(DiodeModel {
                    cjo: 4e-12,
                    vj: 0.6,
                    m: 0.33,
                    fc: 0.4,
                    tt: 5.76e-9,
                    bv: Some(100.0),
                    ibv: 1e-4,
                    ..DiodeModel::default()
                })
            ),
            other => panic!("expected Diode, got {other:?}"),
        }
    }

    #[test]
    fn card_without_parameters_keeps_the_built_in_model() {
        let cards = vec![model_card("dsi", "D", &[]), diode_card("D1", "dsi")];
        let netlist = build_from_cards(cards).unwrap();
        assert!(matches!(
            &netlist.circuit.components[0],
            CircuitElement::Diode(Diode { model: None, .. })
        ));
    }

    #[test]
    fn bjt_card_fills_bf_and_the_model() {
        let cards = vec![
            model_card(
                "q",
                "PNP",
                &[
                    ("IS", 1e-15),
                    ("BF", 250.0),
                    ("BR", 3.0),
                    ("NF", 1.1),
                    ("NR", 1.2),
                    ("VA", 80.0),
                ],
            ),
            bjt_card("q"),
        ];
        let netlist = build_from_cards(cards).unwrap();
        match &netlist.circuit.components[0] {
            CircuitElement::Bjt(Bjt {
                bf, kind, model, ..
            }) => {
                assert_eq!(*bf, 250.0);
                assert!(matches!(kind, BjtKind::Pnp));
                assert_eq!(
                    *model,
                    Some(BjtModel {
                        is: 1e-15,
                        br: 3.0,
                        nf: 1.1,
                        nr: 1.2,
                        vaf: 80.0,
                        ..BjtModel::default()
                    })
                );
            }
            other => panic!("expected Bjt, got {other:?}"),
        }
    }

    #[test]
    fn bf_alone_leaves_the_bjt_model_unset() {
        let cards = vec![model_card("q", "NPN", &[("BF", 50.0)]), bjt_card("q")];
        let netlist = build_from_cards(cards).unwrap();
        assert!(matches!(
            &netlist.circuit.components[0],
            CircuitElement::Bjt(Bjt { bf, model: None, .. }) if *bf == 50.0
        ));
    }

    #[test]
    fn unused_parameters_are_an_error_in_strict_mode() {
        for (kind, element) in [("D", diode_card("D1", "m")), ("NPN", bjt_card("m"))] {
            let cards = vec![
                model_card("m", kind, &[("IS", 1e-14), ("TNOM", 25.0), ("VAR", 10.0)]),
                element,
            ];
            let err = build_from_cards(cards).unwrap_err().to_string();
            assert!(
                err.contains("TNOM=25, VAR") && err.contains("m") && err.contains("TNOM=27"),
                "{kind}: {err}"
            );
        }
    }

    #[test]
    fn unused_parameters_warn_once_per_card_in_lenient_mode() {
        let cards = vec![
            model_card("m", "D", &[("IS", 1e-12), ("TNOM", 25.0), ("VAR", 2.0)]),
            diode_card("D1", "m"),
            diode_card("D2", "m"),
        ];
        let (netlist, warnings) = build_lenient(cards);
        let about_card: Vec<_> = warnings
            .iter()
            .filter(|w| w.message.contains(".model m"))
            .collect();
        assert_eq!(about_card.len(), 1, "{warnings:?}");
        assert!(
            about_card[0].message.contains("TNOM=25, VAR"),
            "{warnings:?}"
        );
        // What is used still applies.
        assert!(matches!(
            &netlist.circuit.components[1],
            CircuitElement::Diode(Diode { model: Some(m), .. }) if m.is == 1e-12
        ));
    }

    #[test]
    fn part_annotations_are_dropped_silently() {
        let cards = vec![
            model_card(
                "q",
                "NPN",
                &[("BF", 300.0), ("Vceo", 40.0), ("Icrating", 0.2)],
            ),
            bjt_card("q"),
        ];
        assert!(build_from_cards(cards).is_ok());
    }
}
