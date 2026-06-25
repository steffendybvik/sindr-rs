//! winnow grammar that turns preprocessed SPICE lines into [`RawCard`] AST nodes.
//!
//! This module is the bridge between the SPICE3 surface syntax and the
//! internal AST. It is pure parsing: no expression evaluation, no `.subckt`
//! flattening, no `Circuit` construction. Span tracking is mandatory — every
//! AST node carries a byte range so callers get caret-style diagnostics.
//!
//! ## Input contract
//!
//! The parser consumes a [`Preprocessed`] value built by the preprocessor
//! pass. Each [`LogicalLine`] is a single SPICE card after comment stripping
//! and `+` continuation joining; the preprocessor has also case-folded the
//! text. The grammar therefore never has to deal with case, comments, or
//! line continuations.
//!
//! ## Span policy
//!
//! Element / card parsers run on the local [`LogicalLine::text`] string; the
//! returned spans are *local* to that string. The top-level driver
//! ([`parse`]) translates them to absolute spans inside the original source
//! file via `LogicalLine::byte_range.start`.

#![allow(dead_code)] // many helpers are exercised via tests only

use std::ops::Range;
use std::sync::Arc;

use miette::{NamedSource, SourceSpan};
use winnow::ascii::space0;
use winnow::combinator::{alt, delimited, opt, preceded, repeat};
use winnow::error::{ContextError, ErrMode, ParserError};
use winnow::stream::{LocatingSlice, Location, Stream};
use winnow::token::{one_of, take_while};
use winnow::{ModalResult, Parser};

use crate::spice::ast::{
    AcSweepKind, BinOp, ParamExpr, RawAnalysis, RawCard, RawElement, RawElementBody, RawModel,
    RawSubckt, RawWaveform, SourceKind, Span,
};
use crate::spice::error::SpiceParseError;
use crate::spice::preprocess::{LogicalLine, Preprocessed};
use crate::spice::si::parse_number as parse_si_number;

// ---------------------------------------------------------------------------
// Lexical primitives
// ---------------------------------------------------------------------------

type Input<'a> = LocatingSlice<&'a str>;

/// Skip ASCII spaces / tabs (NOT newlines — preprocessor already joined lines).
fn ws(input: &mut Input<'_>) -> ModalResult<()> {
    space0.void().parse_next(input)
}

/// Convert a `Range<usize>` produced by `with_span` into a [`Span`] tied to `file`.
fn span_from(range: Range<usize>, file: &Arc<str>) -> Span {
    Span {
        start: range.start,
        end: range.end,
        file: file.clone(),
    }
}

/// SPICE identifier: leading ASCII letter, then `[a-z0-9_]*`. Case-folding is
/// already done by the preprocessor.
fn ident(input: &mut Input<'_>) -> ModalResult<(String, Range<usize>)> {
    (
        one_of(|c: char| c.is_ascii_alphabetic() || c == '_'),
        take_while(0.., |c: char| c.is_ascii_alphanumeric() || c == '_'),
    )
        .take()
        .with_span()
        .map(|(s, span): (&str, Range<usize>)| (s.to_string(), span))
        .parse_next(input)
}

/// Node name: SPICE allows nodes that are pure digits (`0`, `12`) or
/// identifiers, plus `.` for hierarchical names that subckt instances may
/// reference.
fn node_name(input: &mut Input<'_>) -> ModalResult<(String, Range<usize>)> {
    take_while(1.., |c: char| {
        c.is_ascii_alphanumeric() || c == '_' || c == '.' || c == '#'
    })
    .with_span()
    .map(|(s, span): (&str, Range<usize>)| (s.to_string(), span))
    .parse_next(input)
}

/// Parse a numeric literal (with SI suffix) into [`ParamExpr::Number`].
fn number_expr(input: &mut Input<'_>) -> ModalResult<(ParamExpr, Range<usize>)> {
    let start_offset = input.current_token_start();
    let s: &str = &*input;
    let (value, len) = match parse_si_number(s) {
        Some(v) => v,
        None => {
            return Err(ErrMode::from_input(input));
        }
    };
    // Advance the underlying slice by `len` bytes.
    let _ = input.next_slice(len);
    let end_offset = input.current_token_start();
    Ok((
        ParamExpr::Number(value),
        start_offset..end_offset.max(start_offset + len),
    ))
}

/// Parse a `name=value` parameter assignment.
fn param_assign(input: &mut Input<'_>) -> ModalResult<(String, ParamExpr)> {
    let (name, _) = ident(input)?;
    ws(input)?;
    '='.parse_next(input)?;
    ws(input)?;
    let (expr, _) = value_expr(input)?;
    Ok((name, expr))
}

// ---------------------------------------------------------------------------
// ParamExpr grammar (used inside `{...}` and bare in `.param`)
// ---------------------------------------------------------------------------
//
// Precedence (low → high):
//   1. add  / sub          (left-assoc)
//   2. mul  / div          (left-assoc)
//   3. pow  (`^`)          (right-assoc)
//   4. unary minus
//   5. atom: number | ident | `(expr)`

fn expr_atom(input: &mut Input<'_>) -> ModalResult<ParamExpr> {
    ws(input)?;
    alt((
        // `(expr)`
        delimited(('(', ws), expr_add.map(|e| e), (ws, ')')),
        // unary minus
        preceded(('-', ws), expr_atom).map(|e| ParamExpr::Neg(Box::new(e))),
        // numeric literal
        number_expr.map(|(e, _)| e),
        // identifier reference
        ident.map(|(name, _)| ParamExpr::Ref(name)),
    ))
    .parse_next(input)
}

fn expr_pow(input: &mut Input<'_>) -> ModalResult<ParamExpr> {
    let lhs = expr_atom(input)?;
    ws(input)?;
    if opt('^').parse_next(input)?.is_some() {
        // Right-associative recursion.
        ws(input)?;
        let rhs = expr_pow(input)?;
        Ok(ParamExpr::Pow(Box::new(lhs), Box::new(rhs)))
    } else {
        Ok(lhs)
    }
}

fn expr_mul(input: &mut Input<'_>) -> ModalResult<ParamExpr> {
    let mut acc = expr_pow(input)?;
    loop {
        ws(input)?;
        let op = opt(alt(('*', '/'))).parse_next(input)?;
        match op {
            Some('*') => {
                ws(input)?;
                let rhs = expr_pow(input)?;
                acc = ParamExpr::BinOp(Box::new(acc), BinOp::Mul, Box::new(rhs));
            }
            Some('/') => {
                ws(input)?;
                let rhs = expr_pow(input)?;
                acc = ParamExpr::BinOp(Box::new(acc), BinOp::Div, Box::new(rhs));
            }
            _ => return Ok(acc),
        }
    }
}

fn expr_add(input: &mut Input<'_>) -> ModalResult<ParamExpr> {
    let mut acc = expr_mul(input)?;
    loop {
        ws(input)?;
        let op = opt(alt(('+', '-'))).parse_next(input)?;
        match op {
            Some('+') => {
                ws(input)?;
                let rhs = expr_mul(input)?;
                acc = ParamExpr::BinOp(Box::new(acc), BinOp::Add, Box::new(rhs));
            }
            Some('-') => {
                ws(input)?;
                let rhs = expr_mul(input)?;
                acc = ParamExpr::BinOp(Box::new(acc), BinOp::Sub, Box::new(rhs));
            }
            _ => return Ok(acc),
        }
    }
}

/// Parse a `{...}` braced expression — the full ParamExpr grammar.
fn brace_expr(input: &mut Input<'_>) -> ModalResult<(ParamExpr, Range<usize>)> {
    delimited(('{', ws), expr_add, (ws, '}'))
        .with_span()
        .parse_next(input)
}

/// Parse either a `{expr}` or a bare numeric literal — the most common value form.
fn value_expr(input: &mut Input<'_>) -> ModalResult<(ParamExpr, Range<usize>)> {
    alt((brace_expr, number_expr)).parse_next(input)
}

// ---------------------------------------------------------------------------
// Element line parsers (Task 2)
// ---------------------------------------------------------------------------

/// Parse comma-or-space-separated value expressions inside `(...)`.
fn paren_value_list(input: &mut Input<'_>) -> ModalResult<Vec<ParamExpr>> {
    type Item = ((ParamExpr, Range<usize>), (), Option<(char, ())>);
    delimited(
        ('(', ws),
        repeat(0.., (value_expr, ws, opt((',', ws)))).map(|items: Vec<Item>| {
            items
                .into_iter()
                .map(|((e, _), _, _)| e)
                .collect::<Vec<_>>()
        }),
        (ws, ')'),
    )
    .parse_next(input)
}

/// Element ID: prefix character + remainder of identifier.
fn element_id(input: &mut Input<'_>) -> ModalResult<(String, char, Range<usize>)> {
    // `verify_map` returns `None` (a parse error) for an empty identifier,
    // which also makes `chars().next()` total — no unwrap needed.
    ident
        .verify_map(|(s, span)| {
            let prefix = s.chars().next()?;
            Some((s, prefix, span))
        })
        .parse_next(input)
}

/// Parse `R / L / C` passive: `<id> n+ n- <value>`.
fn parse_passive(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    ws(input)?;
    let (n_pos, _) = node_name(input)?;
    ws(input)?;
    let (n_neg, _) = node_name(input)?;
    ws(input)?;
    let (value, val_span) = value_expr(input)?;
    ws(input)?;
    // Strict mode: any trailing tokens are an error.
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawElement {
        id,
        prefix,
        nodes: vec![n_pos, n_neg],
        body: RawElementBody::Passive { value },
        span: Span {
            start: id_span.start,
            end: val_span.end,
            file: Arc::from(""),
        },
    })
}

/// Match a literal keyword. The preprocessor has already lowercased the
/// input, so this just matches the lowercase literal.
fn keyword<'a>(kw: &'static str) -> impl Parser<Input<'a>, &'a str, ErrMode<ContextError>> {
    move |input: &mut Input<'a>| -> ModalResult<&'a str> {
        let mut k = kw;
        k.parse_next(input)
    }
}

/// Try to parse a waveform tail: PULSE / SIN / PWL — returns `None` if the
/// next token is not a waveform keyword.
fn parse_waveform(input: &mut Input<'_>) -> ModalResult<Option<RawWaveform>> {
    ws(input)?;
    if input.is_empty() {
        return Ok(None);
    }
    let cp = input.checkpoint();
    // Try each waveform keyword followed by `(`.
    if keyword("pulse").parse_next(input).is_ok() {
        ws(input)?;
        let args = paren_value_list(input)?;
        if args.len() != 7 {
            return Err(ErrMode::from_input(input));
        }
        return Ok(Some(RawWaveform::Pulse(args)));
    }
    input.reset(&cp);
    if keyword("sin").parse_next(input).is_ok() {
        ws(input)?;
        let args = paren_value_list(input)?;
        if !(3..=5).contains(&args.len()) {
            return Err(ErrMode::from_input(input));
        }
        return Ok(Some(RawWaveform::Sin(args)));
    }
    input.reset(&cp);
    if keyword("pwl").parse_next(input).is_ok() {
        ws(input)?;
        let args = paren_value_list(input)?;
        if args.is_empty() || args.len() % 2 != 0 {
            return Err(ErrMode::from_input(input));
        }
        let pairs = args
            .chunks_exact(2)
            .map(|c| (c[0].clone(), c[1].clone()))
            .collect();
        return Ok(Some(RawWaveform::Pwl(pairs)));
    }
    input.reset(&cp);
    Ok(None)
}

/// Parse `V / I` source: `<id> n+ n- [DC] <value> | <waveform>`.
fn parse_source(input: &mut Input<'_>, kind: SourceKind) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    ws(input)?;
    let (n_pos, _) = node_name(input)?;
    ws(input)?;
    let (n_neg, _) = node_name(input)?;
    ws(input)?;
    // Optional `dc` keyword.
    let _ = opt(keyword("dc")).parse_next(input)?;
    ws(input)?;
    // Try waveform first; if no waveform, expect a value.
    let wf = parse_waveform(input)?;
    let (dc, waveform) = if wf.is_some() {
        (None, wf)
    } else {
        let (val, _) = value_expr(input)?;
        (Some(val), None)
    };
    ws(input)?;
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawElement {
        id,
        prefix,
        nodes: vec![n_pos, n_neg],
        body: RawElementBody::Source { kind, dc, waveform },
        span: Span {
            start: id_span.start,
            end: id_span.end,
            file: Arc::from(""),
        },
    })
}

/// Parse `D` diode: `<id> n_anode n_cathode <model>`.
fn parse_diode(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    ws(input)?;
    let (n_a, _) = node_name(input)?;
    ws(input)?;
    let (n_k, _) = node_name(input)?;
    ws(input)?;
    let (model, _) = ident(input)?;
    ws(input)?;
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawElement {
        id,
        prefix,
        nodes: vec![n_a, n_k],
        body: RawElementBody::Diode { model },
        span: Span {
            start: id_span.start,
            end: id_span.end,
            file: Arc::from(""),
        },
    })
}

/// Parse `Q` BJT: `<id> nc nb ne [ns] <model>`. **Node order is preserved as
/// parsed (collector, base, emitter, [substrate]).** Transposition to sindr's
/// `[base, collector, emitter]` order is the lowering pass's job.
fn parse_bjt(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    ws(input)?;
    // Collect 4 or 5 identifier-shaped tokens; the last is the model name.
    let mut tokens: Vec<String> = Vec::new();
    for _ in 0..5 {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        match node_name(input) {
            Ok((tok, _)) => tokens.push(tok),
            Err(_) => break,
        }
    }
    ws(input)?;
    if !input.is_empty() || tokens.len() < 4 || tokens.len() > 5 {
        return Err(ErrMode::from_input(input));
    }
    // `tokens.len()` is 4 or 5 here (checked above), so both pops are
    // guaranteed `Some`; handle the `None` arm without panicking anyway.
    let Some(model) = tokens.pop() else {
        return Err(ErrMode::from_input(input));
    };
    let substrate = if tokens.len() == 4 {
        tokens.pop()
    } else {
        None
    };
    // tokens now holds [nc, nb, ne].
    Ok(RawElement {
        id,
        prefix,
        nodes: tokens,
        body: RawElementBody::Bjt { model, substrate },
        span: Span {
            start: id_span.start,
            end: id_span.end,
            file: Arc::from(""),
        },
    })
}

/// Parse `X` subckt instance: `<id> [nodes...] <subckt_name> [name=val ...]`.
fn parse_subckt_instance(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    ws(input)?;
    // Greedy: collect identifier-or-node tokens; if a `=` follows, it's actually
    // a `name=val` (back up). The last bare token before `name=val` (or EOL)
    // is the subckt name; everything before it is nodes.
    let mut tokens: Vec<String> = Vec::new();
    let mut params: Vec<(String, ParamExpr)> = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        let cp = input.checkpoint();
        // Try `name=val` first.
        if let Ok(pair) = param_assign.parse_next(input) {
            params.push(pair);
            continue;
        }
        input.reset(&cp);
        // Otherwise consume one bare token.
        match node_name(input) {
            Ok((tok, _)) => tokens.push(tok),
            Err(_) => break,
        }
    }
    ws(input)?;
    if !input.is_empty() || tokens.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    let Some(name) = tokens.pop() else {
        return Err(ErrMode::from_input(input));
    };
    Ok(RawElement {
        id,
        prefix,
        nodes: tokens,
        body: RawElementBody::Subckt { name, params },
        span: Span {
            start: id_span.start,
            end: id_span.end,
            file: Arc::from(""),
        },
    })
}

// ---------------------------------------------------------------------------
// Control card parsers (Task 3)
// ---------------------------------------------------------------------------

/// Match a leading dotted card keyword. The dot has already been consumed by
/// the dispatcher; this matches the bare keyword.
fn dot_card_kind(input: &mut Input<'_>) -> ModalResult<(String, Range<usize>)> {
    take_while(1.., |c: char| c.is_ascii_alphanumeric())
        .with_span()
        .map(|(s, span): (&str, Range<usize>)| (s.to_string(), span))
        .parse_next(input)
}

/// Parse `.subckt NAME [ports...] [param=default ...]`.
fn parse_card_subckt(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    let (name, name_span) = ident(input)?;
    let mut ports: Vec<String> = Vec::new();
    let mut defaults: Vec<(String, ParamExpr)> = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        let cp = input.checkpoint();
        if let Ok(pair) = param_assign.parse_next(input) {
            defaults.push(pair);
            continue;
        }
        input.reset(&cp);
        match node_name(input) {
            Ok((p, _)) => ports.push(p),
            Err(_) => break,
        }
    }
    Ok(RawCard::Subckt(RawSubckt {
        name,
        ports,
        defaults,
        body: Vec::new(),
        span: Span {
            start: name_span.start,
            end: name_span.end,
            file: Arc::from(""),
        },
    }))
}

/// Parse `.ends [name]`.
fn parse_card_ends(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    let name = if input.is_empty() {
        None
    } else {
        opt(ident).parse_next(input)?.map(|(s, _)| s)
    };
    ws(input)?;
    Ok(RawCard::Ends(name))
}

/// Parse `.model NAME KIND [(] k=v ... [)]`. Parens are optional (vendor-tolerant).
fn parse_card_model(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    let (name, name_span) = ident(input)?;
    ws(input)?;
    let (kind, _) = ident(input)?;
    ws(input)?;
    // Optional `(`.
    let had_paren = opt('(').parse_next(input)?.is_some();
    let mut params: Vec<(String, ParamExpr)> = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        if had_paren {
            let cp = input.checkpoint();
            if opt(')').parse_next(input)?.is_some() {
                break;
            }
            input.reset(&cp);
        }
        match param_assign.parse_next(input) {
            Ok(pair) => params.push(pair),
            Err(_) => break,
        }
    }
    ws(input)?;
    Ok(RawCard::Model(RawModel {
        name,
        kind,
        params,
        span: Span {
            start: name_span.start,
            end: name_span.end,
            file: Arc::from(""),
        },
    }))
}

/// Parse `.param name=expr [name2=expr2 ...]`.
fn parse_card_param(input: &mut Input<'_>) -> ModalResult<RawCard> {
    let mut bindings: Vec<(String, ParamExpr)> = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        match param_assign.parse_next(input) {
            Ok(pair) => bindings.push(pair),
            Err(_) => break,
        }
    }
    if bindings.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Param(bindings))
}

/// Parse `.tran tstep tstop [tstart]`.
fn parse_card_tran(input: &mut Input<'_>) -> ModalResult<RawCard> {
    let mut args: Vec<ParamExpr> = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        match value_expr(input) {
            Ok((v, _)) => args.push(v),
            Err(_) => break,
        }
    }
    if args.len() < 2 || args.len() > 3 {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Analysis(RawAnalysis::Tran { args }))
}

/// Parse `.dc source start stop step`.
fn parse_card_dc(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    let (source, _) = ident(input)?;
    let mut args: Vec<ParamExpr> = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        match value_expr(input) {
            Ok((v, _)) => args.push(v),
            Err(_) => break,
        }
    }
    if args.len() != 3 {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Analysis(RawAnalysis::Dc { source, args }))
}

/// Parse `.ac dec|oct|lin points fstart fstop`.
fn parse_card_ac(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    let (sweep_kw, _) = ident(input)?;
    let sweep = match sweep_kw.as_str() {
        "dec" => AcSweepKind::Dec,
        "oct" => AcSweepKind::Oct,
        "lin" => AcSweepKind::Lin,
        _ => return Err(ErrMode::from_input(input)),
    };
    ws(input)?;
    let (points, _) = value_expr(input)?;
    ws(input)?;
    let (fstart, _) = value_expr(input)?;
    ws(input)?;
    let (fstop, _) = value_expr(input)?;
    ws(input)?;
    Ok(RawCard::Analysis(RawAnalysis::Ac {
        sweep,
        points,
        fstart,
        fstop,
    }))
}

/// Parse `.op` (no args).
fn parse_card_op(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    Ok(RawCard::Analysis(RawAnalysis::Op))
}

// ---------------------------------------------------------------------------
// Per-line dispatch
// ---------------------------------------------------------------------------

/// Outcome of dispatching a single logical line. The driver uses this to
/// build the final `Vec<RawCard>` and to handle subckt body collection.
#[derive(Debug)]
enum LineOutcome {
    /// A normal card (element, model, .param, .tran, .dc, .ac, .op, .subckt header, .ends).
    Card(RawCard),
}

/// Dispatch a single preprocessed line to the appropriate parser. Errors
/// returned here are raw winnow errors; the driver lifts them into
/// [`SpiceParseError`] with absolute spans.
fn parse_one_card_local(text: &str) -> Result<RawCard, LocalError> {
    let trimmed = text.trim();
    let Some(first) = trimmed.chars().next() else {
        return Err(LocalError::Empty);
    };
    if first == '.' {
        // Control card: dispatch by keyword.
        let mut input = LocatingSlice::new(&trimmed[1..]);
        let (kind, kind_span) = dot_card_kind(&mut input).map_err(|_| LocalError::Syntax {
            message: "expected a control-card keyword after `.`".to_string(),
            span: 0..trimmed.len(),
        })?;
        let result = match kind.as_str() {
            "subckt" => parse_card_subckt(&mut input),
            "ends" | "endl" => parse_card_ends(&mut input),
            "model" => parse_card_model(&mut input),
            "param" => parse_card_param(&mut input),
            "tran" => parse_card_tran(&mut input),
            "dc" => parse_card_dc(&mut input),
            "ac" => parse_card_ac(&mut input),
            "op" => parse_card_op(&mut input),
            "include" | "lib" => {
                // Should have been resolved by the preprocessor.
                return Err(LocalError::Syntax {
                    message: "internal: unresolved include reached parser".to_string(),
                    span: 0..trimmed.len(),
                });
            }
            "end" => {
                // Top-level `.end` terminates the deck — caller may treat as EOF.
                return Err(LocalError::Empty);
            }
            other => {
                return Err(LocalError::UnknownCard {
                    card: other.to_string(),
                    // span is `.kind` — shift back by 1 for the leading dot.
                    span: 0..(1 + kind_span.end),
                });
            }
        };
        result.map_err(|_| LocalError::Syntax {
            message: format!("malformed `.{kind}` directive"),
            span: 0..trimmed.len(),
        })
    } else if first.is_ascii_alphabetic() {
        let prefix = first;
        let mut input = LocatingSlice::new(trimmed);
        let parsed = match prefix {
            'r' | 'l' | 'c' => parse_passive(&mut input),
            'v' => parse_source(&mut input, SourceKind::Voltage),
            'i' => parse_source(&mut input, SourceKind::Current),
            'd' => parse_diode(&mut input),
            'q' => parse_bjt(&mut input),
            'x' => parse_subckt_instance(&mut input),
            'm' => {
                return Err(LocalError::UnsupportedDevice {
                    device: "MOSFET (M)".to_string(),
                    span: 0..1,
                });
            }
            'j' => {
                return Err(LocalError::UnsupportedDevice {
                    device: "JFET (J)".to_string(),
                    span: 0..1,
                });
            }
            'b' => {
                return Err(LocalError::UnsupportedDevice {
                    device: "behavioural source (B)".to_string(),
                    span: 0..1,
                });
            }
            'k' | 's' => {
                return Err(LocalError::UnsupportedDevice {
                    device: format!("device prefix '{prefix}'"),
                    span: 0..1,
                });
            }
            other => {
                return Err(LocalError::UnknownDevice {
                    prefix: other,
                    span: 0..1,
                });
            }
        };
        parsed
            .map(RawCard::Element)
            .map_err(|_| LocalError::Syntax {
                message: format!("malformed `{prefix}` element"),
                span: 0..trimmed.len(),
            })
    } else {
        Err(LocalError::Syntax {
            message: format!("unexpected leading character '{first}'"),
            span: 0..1,
        })
    }
}

#[derive(Debug)]
enum LocalError {
    Empty,
    UnknownDevice { prefix: char, span: Range<usize> },
    UnknownCard { card: String, span: Range<usize> },
    UnsupportedDevice { device: String, span: Range<usize> },
    Syntax { message: String, span: Range<usize> },
}

/// Recoverable parser errors are downgraded to warnings in lenient mode.
fn is_recoverable(err: &LocalError) -> bool {
    matches!(
        err,
        LocalError::UnknownDevice { .. }
            | LocalError::UnsupportedDevice { .. }
            | LocalError::UnknownCard { .. }
    )
}

/// Lift a [`LocalError`] (whose spans are local to the trimmed line text)
/// into a [`SpiceParseError`] with absolute spans.
fn lift_error(err: LocalError, line: &LogicalLine) -> SpiceParseError {
    let (local, msg_span_start) = trim_offset(&line.text);
    let absolute = |r: &Range<usize>| -> SourceSpan {
        let abs_start = line.byte_range.start + msg_span_start + r.start;
        let abs_end = line.byte_range.start + msg_span_start + r.end;
        (abs_start, abs_end.saturating_sub(abs_start)).into()
    };
    let _ = local;
    let src = || NamedSource::new(&*line.file, (*line.src).clone());
    match err {
        LocalError::Empty => SpiceParseError::Syntax {
            message: "empty card after preprocessing".to_string(),
            src: src(),
            bad_span: absolute(&(0..line.text.len())),
        },
        LocalError::UnknownDevice { prefix, span } => SpiceParseError::UnknownDevice {
            prefix,
            src: src(),
            bad_span: absolute(&span),
        },
        LocalError::UnknownCard { card, span } => SpiceParseError::UnknownCard {
            card,
            src: src(),
            bad_span: absolute(&span),
        },
        LocalError::UnsupportedDevice { device, span } => SpiceParseError::UnsupportedDevice {
            device,
            src: src(),
            bad_span: absolute(&span),
        },
        LocalError::Syntax { message, span } => SpiceParseError::Syntax {
            message,
            src: src(),
            bad_span: absolute(&span),
        },
    }
}

/// Returns `(trimmed_str, leading_ws_byte_count)` so span lifting can account
/// for whitespace the parsers themselves never see.
fn trim_offset(s: &str) -> (&str, usize) {
    let trimmed = s.trim_start();
    let offset = s.len() - trimmed.len();
    (trimmed.trim_end(), offset)
}

// ---------------------------------------------------------------------------
// Top-level driver (Task 3)
// ---------------------------------------------------------------------------

/// Parse a [`Preprocessed`] netlist into a flat `Vec<RawCard>` in strict mode.
/// `.subckt` bodies are collected into their parent `RawSubckt::body`.
pub(crate) fn parse(pre: &Preprocessed) -> Result<Vec<RawCard>, SpiceParseError> {
    let mut warnings = Vec::new();
    parse_with_options(pre, true, &mut warnings)
}

/// Parse a [`Preprocessed`] netlist into a flat `Vec<RawCard>`.
///
/// In strict mode (default), `UnknownDevice`, `UnsupportedDevice`, and
/// `UnknownCard` are fatal. In lenient mode they downgrade to
/// [`crate::spice::error::ParseWarning`]s and the offending card is dropped.
pub(crate) fn parse_with_options(
    pre: &Preprocessed,
    strict: bool,
    warnings: &mut Vec<crate::spice::error::ParseWarning>,
) -> Result<Vec<RawCard>, SpiceParseError> {
    let mut top: Vec<RawCard> = Vec::new();
    // Stack of in-progress subckts and their declaring line (for diagnostics).
    let mut stack: Vec<(RawSubckt, LogicalLine)> = Vec::new();

    for line in &pre.lines {
        let card = match parse_one_card_local(&line.text) {
            Ok(c) => c,
            Err(LocalError::Empty) => continue,
            Err(e) => {
                if !strict && is_recoverable(&e) {
                    let lifted = lift_error(e, line);
                    warnings.push(crate::spice::error::ParseWarning {
                        message: format!("{lifted}"),
                        span: Some((line.byte_range.start, line.byte_range.len()).into()),
                        file: Some(line.file.to_string()),
                    });
                    continue;
                }
                return Err(lift_error(e, line));
            }
        };
        match card {
            RawCard::Subckt(sub) => {
                stack.push((sub, line.clone()));
            }
            RawCard::Ends(_name) => {
                let (closed, _open_line) = match stack.pop() {
                    Some(s) => s,
                    None => {
                        return Err(SpiceParseError::Syntax {
                            message: "`.ends` without matching `.subckt`".to_string(),
                            src: NamedSource::new(&*line.file, (*line.src).clone()),
                            bad_span: (line.byte_range.start, line.byte_range.len()).into(),
                        });
                    }
                };
                push_card(&mut top, &mut stack, RawCard::Subckt(closed));
            }
            other => push_card(&mut top, &mut stack, other),
        }
    }

    if let Some((sub, open_line)) = stack.pop() {
        return Err(SpiceParseError::Syntax {
            message: format!("unclosed `.subckt {}` at end of input", sub.name),
            src: NamedSource::new(&*open_line.file, (*open_line.src).clone()),
            bad_span: (open_line.byte_range.start, open_line.byte_range.len()).into(),
        });
    }

    Ok(top)
}

/// Push a finished card into the innermost open `.subckt` body, or onto the
/// top-level vector if no subckt is open.
fn push_card(top: &mut Vec<RawCard>, stack: &mut [(RawSubckt, LogicalLine)], card: RawCard) {
    if let Some((current, _)) = stack.last_mut() {
        current.body.push(card);
    } else {
        top.push(card);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn line(text: &str) -> LogicalLine {
        LogicalLine {
            text: text.to_string(),
            start_line: 1,
            file: Arc::from("test.cir"),
            src: Arc::new(text.to_string()),
            byte_range: 0..text.len(),
        }
    }

    fn pre(lines: &[&str]) -> Preprocessed {
        let mut out = Preprocessed {
            title: "test".to_string(),
            ..Preprocessed::default()
        };
        let mut offset = 0usize;
        for (i, l) in lines.iter().enumerate() {
            out.lines.push(LogicalLine {
                text: l.to_string(),
                start_line: i + 1,
                file: Arc::from("test.cir"),
                src: Arc::new(lines.join("\n")),
                byte_range: offset..offset + l.len(),
            });
            offset += l.len() + 1; // include the newline join
        }
        out
    }

    fn must_num(e: &ParamExpr) -> f64 {
        match e {
            ParamExpr::Number(v) => *v,
            other => panic!("expected Number, got {other:?}"),
        }
    }

    // -- lex -----------------------------------------------------------------

    #[test]
    fn lex_si_number_simple() {
        assert_eq!(parse_si_number("1k"), Some((1000.0, 2)));
        assert_eq!(parse_si_number("4.7uf"), Some((4.7e-6, 5)));
        assert_eq!(parse_si_number("1meg"), Some((1e6, 4)));
        assert_eq!(parse_si_number("1m"), Some((1e-3, 2)));
        assert_eq!(parse_si_number("1mil"), Some((25.4e-6, 4)));
        assert_eq!(parse_si_number("100"), Some((100.0, 3)));
        assert_eq!(parse_si_number("2.5e3"), Some((2500.0, 5)));
        assert_eq!(parse_si_number("abc"), None);
    }

    #[test]
    fn lex_number_expr() {
        let mut input = LocatingSlice::new("1k");
        let (e, _) = number_expr(&mut input).unwrap();
        assert!((must_num(&e) - 1000.0).abs() < 1e-9);

        let mut input = LocatingSlice::new("4.7uf");
        let (e, _) = number_expr(&mut input).unwrap();
        assert!((must_num(&e) - 4.7e-6).abs() < 1e-12);
    }

    #[test]
    fn lex_brace_expr_arith() {
        let mut input = LocatingSlice::new("{1k + 2k}");
        let (e, _) = brace_expr(&mut input).unwrap();
        match e {
            ParamExpr::BinOp(l, BinOp::Add, r) => {
                assert!((must_num(&l) - 1000.0).abs() < 1e-9);
                assert!((must_num(&r) - 2000.0).abs() < 1e-9);
            }
            other => panic!("expected Add, got {other:?}"),
        }
    }

    #[test]
    fn lex_brace_expr_pow_right_assoc() {
        // 2^3^2 should be 2^(3^2) = 512, not (2^3)^2 = 64.
        let mut input = LocatingSlice::new("{2^3^2}");
        let (e, _) = brace_expr(&mut input).unwrap();
        // Verify shape: outer Pow(2, Pow(3, 2)).
        match e {
            ParamExpr::Pow(base, exp) => {
                assert!((must_num(&base) - 2.0).abs() < 1e-9);
                match *exp {
                    ParamExpr::Pow(b2, e2) => {
                        assert!((must_num(&b2) - 3.0).abs() < 1e-9);
                        assert!((must_num(&e2) - 2.0).abs() < 1e-9);
                    }
                    other => panic!("expected inner Pow, got {other:?}"),
                }
            }
            other => panic!("expected Pow, got {other:?}"),
        }
    }

    #[test]
    fn lex_brace_expr_ref() {
        let mut input = LocatingSlice::new("{r_base * 2}");
        let (e, _) = brace_expr(&mut input).unwrap();
        match e {
            ParamExpr::BinOp(l, BinOp::Mul, r) => {
                match *l {
                    ParamExpr::Ref(name) => assert_eq!(name, "r_base"),
                    other => panic!("expected Ref, got {other:?}"),
                }
                assert!((must_num(&r) - 2.0).abs() < 1e-9);
            }
            other => panic!("expected Mul, got {other:?}"),
        }
    }

    #[test]
    fn lex_param_assign_both_forms() {
        let mut input = LocatingSlice::new("r=1k");
        let (name, expr) = param_assign(&mut input).unwrap();
        assert_eq!(name, "r");
        assert!((must_num(&expr) - 1000.0).abs() < 1e-9);

        let mut input = LocatingSlice::new("r={1k+2k}");
        let (name, expr) = param_assign(&mut input).unwrap();
        assert_eq!(name, "r");
        assert!(matches!(expr, ParamExpr::BinOp(_, BinOp::Add, _)));
    }

    // -- elements ------------------------------------------------------------

    #[test]
    fn elements_passive_resistor() {
        let card = parse_one_card_local("r1 a b 1k").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        assert_eq!(el.id, "r1");
        assert_eq!(el.prefix, 'r');
        assert_eq!(el.nodes, vec!["a".to_string(), "b".to_string()]);
        match el.body {
            RawElementBody::Passive { value } => {
                assert!((must_num(&value) - 1000.0).abs() < 1e-9);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn elements_passive_capacitor_si() {
        let card = parse_one_card_local("c1 n1 0 4.7u").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        assert_eq!(el.prefix, 'c');
        match el.body {
            RawElementBody::Passive { value } => {
                assert!((must_num(&value) - 4.7e-6).abs() < 1e-12);
            }
            _ => panic!(),
        }
    }

    #[test]
    fn elements_v_source_dc_explicit() {
        let card = parse_one_card_local("vin in 0 dc 5").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        match el.body {
            RawElementBody::Source {
                kind: SourceKind::Voltage,
                dc: Some(v),
                waveform: None,
            } => {
                assert!((must_num(&v) - 5.0).abs() < 1e-9);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_v_source_dc_implicit() {
        let card = parse_one_card_local("vin in 0 5").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        match el.body {
            RawElementBody::Source {
                dc: Some(v),
                waveform: None,
                ..
            } => {
                assert!((must_num(&v) - 5.0).abs() < 1e-9);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_v_source_sin() {
        let card = parse_one_card_local("vsig in 0 sin(0 1 1k)").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        match el.body {
            RawElementBody::Source {
                waveform: Some(RawWaveform::Sin(args)),
                dc: None,
                ..
            } => {
                assert_eq!(args.len(), 3);
                assert!((must_num(&args[2]) - 1000.0).abs() < 1e-9);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_v_source_pulse_full() {
        let card = parse_one_card_local("vclk clk 0 pulse(0 5 0 1n 1n 5u 10u)").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        match el.body {
            RawElementBody::Source {
                waveform: Some(RawWaveform::Pulse(args)),
                ..
            } => {
                assert_eq!(args.len(), 7);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_v_source_pwl_pairs() {
        let card = parse_one_card_local("vsource n 0 pwl(0 0 1m 5 2m 5)").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        match el.body {
            RawElementBody::Source {
                waveform: Some(RawWaveform::Pwl(pairs)),
                ..
            } => {
                assert_eq!(pairs.len(), 3);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_pulse_arity_error() {
        // 3 args instead of 7 — should fail.
        let result = parse_one_card_local("vclk clk 0 pulse(0 5 0)");
        assert!(result.is_err(), "expected arity error, got {result:?}");
    }

    #[test]
    fn elements_diode() {
        let card = parse_one_card_local("d1 a k dmod").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        assert_eq!(el.prefix, 'd');
        assert_eq!(el.nodes, vec!["a".to_string(), "k".to_string()]);
        match el.body {
            RawElementBody::Diode { model } => assert_eq!(model, "dmod"),
            _ => panic!(),
        }
    }

    #[test]
    fn elements_bjt_three_node_preserves_order() {
        // q1 c b e qmod — nodes preserved in c,b,e order; transposition is the
        // lowering pass's job.
        let card = parse_one_card_local("q1 c b e qmod").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        assert_eq!(el.prefix, 'q');
        assert_eq!(
            el.nodes,
            vec!["c".to_string(), "b".to_string(), "e".to_string()]
        );
        match el.body {
            RawElementBody::Bjt {
                model,
                substrate: None,
            } => assert_eq!(model, "qmod"),
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_bjt_with_substrate() {
        let card = parse_one_card_local("q1 c b e s qmod").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        match el.body {
            RawElementBody::Bjt {
                model,
                substrate: Some(s),
            } => {
                assert_eq!(model, "qmod");
                assert_eq!(s, "s");
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_subckt_instance_with_params() {
        let card = parse_one_card_local("x1 a b c myfilter r=1k").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        assert_eq!(el.prefix, 'x');
        assert_eq!(
            el.nodes,
            vec!["a".to_string(), "b".to_string(), "c".to_string()]
        );
        match el.body {
            RawElementBody::Subckt { name, params } => {
                assert_eq!(name, "myfilter");
                assert_eq!(params.len(), 1);
                assert_eq!(params[0].0, "r");
                assert!((must_num(&params[0].1) - 1000.0).abs() < 1e-9);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_unsupported_mosfet() {
        let result = parse_one_card_local("m1 d g s mosmod");
        match result {
            Err(LocalError::UnsupportedDevice { device, .. }) => {
                assert!(device.contains("MOSFET"), "device = {device:?}");
            }
            other => panic!("expected UnsupportedDevice(MOSFET), got {other:?}"),
        }
    }

    #[test]
    fn elements_unknown_prefix() {
        let result = parse_one_card_local("z1 a b zmod");
        match result {
            Err(LocalError::UnknownDevice { prefix, .. }) => {
                assert_eq!(prefix, 'z');
            }
            other => panic!("expected UnknownDevice('z'), got {other:?}"),
        }
    }

    // -- control cards ------------------------------------------------------

    #[test]
    fn cards_tran() {
        let card = parse_one_card_local(".tran 1u 1m").unwrap();
        match card {
            RawCard::Analysis(RawAnalysis::Tran { args }) => {
                assert_eq!(args.len(), 2);
                assert!((must_num(&args[0]) - 1e-6).abs() < 1e-15);
                assert!((must_num(&args[1]) - 1e-3).abs() < 1e-12);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_ac_dec() {
        let card = parse_one_card_local(".ac dec 50 10 100k").unwrap();
        match card {
            RawCard::Analysis(RawAnalysis::Ac {
                sweep,
                points,
                fstart,
                fstop,
            }) => {
                assert_eq!(sweep, AcSweepKind::Dec);
                assert!((must_num(&points) - 50.0).abs() < 1e-9);
                assert!((must_num(&fstart) - 10.0).abs() < 1e-9);
                assert!((must_num(&fstop) - 100e3).abs() < 1e-6);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_op() {
        let card = parse_one_card_local(".op").unwrap();
        assert!(matches!(card, RawCard::Analysis(RawAnalysis::Op)));
    }

    #[test]
    fn cards_dc_sweep() {
        let card = parse_one_card_local(".dc vin 0 5 0.1").unwrap();
        match card {
            RawCard::Analysis(RawAnalysis::Dc { source, args }) => {
                assert_eq!(source, "vin");
                assert_eq!(args.len(), 3);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_model_with_parens() {
        let card = parse_one_card_local(".model dmod d (is=1e-14 n=1)").unwrap();
        match card {
            RawCard::Model(m) => {
                assert_eq!(m.name, "dmod");
                assert_eq!(m.kind, "d");
                assert_eq!(m.params.len(), 2);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_model_without_parens() {
        let card = parse_one_card_local(".model dmod d is=1e-14 n=1").unwrap();
        match card {
            RawCard::Model(m) => {
                assert_eq!(m.name, "dmod");
                assert_eq!(m.kind, "d");
                assert_eq!(m.params.len(), 2);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_subckt_header() {
        let card = parse_one_card_local(".subckt rc in out r=1k").unwrap();
        match card {
            RawCard::Subckt(sub) => {
                assert_eq!(sub.name, "rc");
                assert_eq!(sub.ports, vec!["in".to_string(), "out".to_string()]);
                assert_eq!(sub.defaults.len(), 1);
                assert_eq!(sub.defaults[0].0, "r");
                assert!((must_num(&sub.defaults[0].1) - 1000.0).abs() < 1e-9);
                assert!(sub.body.is_empty());
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_param_list() {
        let card = parse_one_card_local(".param a=1k b={2*3}").unwrap();
        match card {
            RawCard::Param(bindings) => {
                assert_eq!(bindings.len(), 2);
                assert_eq!(bindings[0].0, "a");
                assert!((must_num(&bindings[0].1) - 1000.0).abs() < 1e-9);
                assert_eq!(bindings[1].0, "b");
                assert!(matches!(bindings[1].1, ParamExpr::BinOp(_, BinOp::Mul, _)));
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_unknown_card_errors() {
        let result = parse_one_card_local(".foo bar");
        match result {
            Err(LocalError::UnknownCard { card, .. }) => assert_eq!(card, "foo"),
            other => panic!("expected UnknownCard, got {other:?}"),
        }
    }

    #[test]
    fn cards_include_passes_through_as_internal_error() {
        // The preprocessor should consume `.include`s. If one reaches the
        // parser, raise a clear `Syntax` error.
        let result = parse_one_card_local(".include 'foo.cir'");
        assert!(matches!(result, Err(LocalError::Syntax { .. })));
    }

    // -- driver -------------------------------------------------------------

    #[test]
    fn driver_subckt_body_collects() {
        let pre = pre(&[
            ".subckt rc in out r=1k",
            "r1 in mid 1k",
            "c1 mid out 1u",
            ".ends rc",
            "r_top a b 100",
        ]);
        let cards = parse(&pre).expect("parse");
        assert_eq!(cards.len(), 2);
        match &cards[0] {
            RawCard::Subckt(sub) => {
                assert_eq!(sub.name, "rc");
                assert_eq!(sub.body.len(), 2);
            }
            other => panic!("got {other:?}"),
        }
        assert!(matches!(&cards[1], RawCard::Element(e) if e.prefix == 'r'));
    }

    #[test]
    fn driver_unmatched_ends_errors() {
        let pre = pre(&[".ends"]);
        let err = parse(&pre).unwrap_err();
        match err {
            SpiceParseError::Syntax { message, .. } => {
                assert!(
                    message.contains("`.ends` without matching"),
                    "msg = {message}"
                );
            }
            other => panic!("expected Syntax, got {other:?}"),
        }
    }

    #[test]
    fn driver_unclosed_subckt_errors() {
        let pre = pre(&[".subckt rc in out", "r1 in out 1k"]);
        let err = parse(&pre).unwrap_err();
        match err {
            SpiceParseError::Syntax { message, .. } => {
                assert!(message.contains("unclosed"), "msg = {message}");
            }
            other => panic!("expected Syntax, got {other:?}"),
        }
    }

    #[test]
    fn driver_error_includes_named_source() {
        let pre = pre(&["z1 a b zmod"]);
        let err = parse(&pre).unwrap_err();
        let rendered = format!("{err:?}");
        assert!(rendered.contains("UnknownDevice"), "rendered = {rendered}");
    }

    #[test]
    fn driver_line_consumed_helper() {
        // sanity: the test helper builds a usable LogicalLine.
        let l = line("r1 a b 1k");
        assert_eq!(l.text, "r1 a b 1k");
        assert_eq!(l.byte_range, 0..9);
    }
}
