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
    AcSweepKind, BinOp, NodeValueKind, ParamExpr, PolyControls, RawAcSpec, RawAnalysis, RawCard,
    RawCrossing, RawElement, RawElementBody, RawMeasure, RawMeasureKind, RawModel, RawSubckt,
    RawWaveform, SourceKind, Span,
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

/// The span of a whole logical line in its source file.
fn line_span(line: &LogicalLine) -> Span {
    Span {
        start: line.byte_range.start,
        end: line.byte_range.end,
        file: line.file.clone(),
        src: line.src.clone(),
    }
}

/// Replace a card's line-local span with its source line, so errors raised
/// after parsing (undefined model, undefined parameter) can point at it.
/// The logical text is comment-stripped and continuation-joined, so local
/// offsets don't map onto the file; the whole line does.
fn anchor_to_line(card: RawCard, line: &LogicalLine) -> RawCard {
    match card {
        RawCard::Element(mut e) => {
            e.span = line_span(line);
            RawCard::Element(e)
        }
        RawCard::Model(mut m) => {
            m.span = line_span(line);
            RawCard::Model(m)
        }
        RawCard::Param(bindings, _) => RawCard::Param(bindings, line_span(line)),
        RawCard::Analysis(analysis, _) => RawCard::Analysis(analysis, line_span(line)),
        RawCard::NodeValues(kind, values, _) => RawCard::NodeValues(kind, values, line_span(line)),
        RawCard::Measure(measure, _) => RawCard::Measure(measure, line_span(line)),
        RawCard::Four(f0, signals, _) => RawCard::Four(f0, signals, line_span(line)),
        RawCard::Options(options, _) => RawCard::Options(options, line_span(line)),
        other => other,
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
/// reference. Letters and digits may be any Unicode ones (`å`, `µ`), as
/// ngspice reads them.
fn node_name(input: &mut Input<'_>) -> ModalResult<(String, Range<usize>)> {
    take_while(1.., |c: char| {
        c.is_alphanumeric() || c == '_' || c == '.' || c == '#'
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
//   3. unary minus
//   4. pow  (`^` or `**`)  (right-assoc; its exponent may be negated)
//   5. atom: number | ident | `(expr)`
//
// `^` binds tighter than unary minus, so `-2^2` is −4 and `2^-2` is 0.25,
// as in ngspice (checked against ngspice 47, `.param` and `B` sources
// alike) and in sindr's behavioural expressions.

/// Deepest nesting of `(`, unary minus and `^` an expression may have. The
/// grammar recurses once per level, so an unbounded depth would let a long run
/// of `(((…` overflow the stack, which `catch_unwind` can't recover from.
const MAX_EXPR_DEPTH: usize = 256;

thread_local! {
    static EXPR_DEPTH: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Holds one level of expression nesting; dropping it gives the level back.
struct DepthGuard;

impl DepthGuard {
    fn enter(input: &Input<'_>) -> ModalResult<Self> {
        let depth = EXPR_DEPTH.with(|d| {
            let next = d.get() + 1;
            d.set(next);
            next
        });
        let guard = DepthGuard;
        if depth > MAX_EXPR_DEPTH {
            // Cut, so `alt` doesn't retry every branch at every level.
            return Err(ErrMode::Cut(ContextError::from_input(input)));
        }
        Ok(guard)
    }
}

impl Drop for DepthGuard {
    fn drop(&mut self) {
        EXPR_DEPTH.with(|d| d.set(d.get() - 1));
    }
}

fn expr_atom(input: &mut Input<'_>) -> ModalResult<ParamExpr> {
    let _depth = DepthGuard::enter(input)?;
    ws(input)?;
    alt((
        // `(expr)`
        delimited(('(', ws), expr_add.map(|e| e), (ws, ')')),
        // numeric literal
        number_expr.map(|(e, _)| e),
        // identifier reference
        ident.map(|(name, _)| ParamExpr::Ref(name)),
    ))
    .parse_next(input)
}

/// Unary minus, looser than `^`: `-2^2` is `-(2^2)`.
fn expr_unary(input: &mut Input<'_>) -> ModalResult<ParamExpr> {
    ws(input)?;
    if opt('-').parse_next(input)?.is_some() {
        // Only a minus nests, so only it counts towards the depth cap.
        let _depth = DepthGuard::enter(input)?;
        return Ok(ParamExpr::Neg(Box::new(expr_unary(input)?)));
    }
    expr_pow(input)
}

fn expr_pow(input: &mut Input<'_>) -> ModalResult<ParamExpr> {
    let _depth = DepthGuard::enter(input)?;
    let lhs = expr_atom(input)?;
    ws(input)?;
    if opt(alt(("**", "^"))).parse_next(input)?.is_some() {
        // Right-associative, and the exponent may be negated: `2^-2`.
        ws(input)?;
        let rhs = expr_unary(input)?;
        Ok(ParamExpr::Pow(Box::new(lhs), Box::new(rhs)))
    } else {
        Ok(lhs)
    }
}

fn expr_mul(input: &mut Input<'_>) -> ModalResult<ParamExpr> {
    let mut acc = expr_unary(input)?;
    loop {
        ws(input)?;
        let op = opt(alt(('*', '/'))).parse_next(input)?;
        match op {
            Some('*') => {
                ws(input)?;
                let rhs = expr_unary(input)?;
                acc = ParamExpr::BinOp(Box::new(acc), BinOp::Mul, Box::new(rhs));
            }
            Some('/') => {
                ws(input)?;
                let rhs = expr_unary(input)?;
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

/// A line ending in `value`, with `id_span` for its start.
fn element_line(
    id: String,
    prefix: char,
    id_span: Range<usize>,
    nodes: Vec<String>,
    body: RawElementBody,
    end: usize,
) -> RawElement {
    RawElement {
        id,
        prefix,
        nodes,
        body,
        span: Span {
            start: id_span.start,
            end,
            file: Arc::from(""),
            src: Arc::default(),
        },
    }
}

/// `n` node names, each after whitespace.
fn nodes_n(input: &mut Input<'_>, n: usize) -> ModalResult<Vec<String>> {
    let mut nodes = Vec::with_capacity(n);
    for _ in 0..n {
        ws(input)?;
        nodes.push(node_name(input)?.0);
    }
    Ok(nodes)
}

/// A trailing value expression and the end of the line.
fn final_value(input: &mut Input<'_>) -> ModalResult<(ParamExpr, Range<usize>)> {
    ws(input)?;
    let value = value_expr(input)?;
    ws(input)?;
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(value)
}

/// Parse `E` / `G` linear voltage-controlled sources:
/// `<id> n+ n- nc+ nc- <value>`.
fn parse_voltage_controlled(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    let nodes = nodes_n(input, 4)?;
    let (value, val_span) = final_value(input)?;
    let body = RawElementBody::VoltageControlled { value };
    Ok(element_line(id, prefix, id_span, nodes, body, val_span.end))
}

/// Parse `F` / `H` linear current-controlled sources:
/// `<id> n+ n- <vname> <value>`.
fn parse_current_controlled(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    let nodes = nodes_n(input, 2)?;
    ws(input)?;
    let (source, _) = ident(input)?;
    let (value, val_span) = final_value(input)?;
    let body = RawElementBody::CurrentControlled { source, value };
    Ok(element_line(id, prefix, id_span, nodes, body, val_span.end))
}

/// Parse `K` mutual inductance: `<id> <lname1> <lname2> <k>`.
fn parse_coupling(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    ws(input)?;
    let (l1, _) = ident(input)?;
    ws(input)?;
    let (l2, _) = ident(input)?;
    let (k, val_span) = final_value(input)?;
    let body = RawElementBody::Coupling {
        inductors: [l1, l2],
        k,
    };
    Ok(element_line(
        id,
        prefix,
        id_span,
        Vec::new(),
        body,
        val_span.end,
    ))
}

/// The nonlinear forms of `E`/`G`/`F`/`H` (`POLY`, `VALUE`, `TABLE`,
/// `LAPLACE`). sindr reads `POLY`, and `VALUE` on `E` and `G`, as
/// behavioural sources and doesn't model the rest.
fn behavioural_form(line: &str) -> Option<&'static str> {
    line.split(|c: char| c.is_whitespace() || c == '=' || c == '(' || c == '{')
        .skip(1)
        .find_map(|token| match token {
            "poly" => Some("POLY"),
            "value" => Some("VALUE"),
            "table" => Some("TABLE"),
            "laplace" => Some("LAPLACE"),
            _ => None,
        })
}

/// Parse a behavioural source: `B<id> n+ n- V=expr` / `I=expr`, or
/// `E<id>`/`G<id> n+ n- VALUE=expr` (`value` is `true`). The expression
/// runs to the end of the card, braces optional.
fn parse_behavioural(input: &mut Input<'_>, value: bool) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    ws(input)?;
    let (n_pos, _) = node_name(input)?;
    ws(input)?;
    let (n_neg, _) = node_name(input)?;
    ws(input)?;
    let rest: &str = winnow::token::rest.parse_next(input)?;
    let (key, text) = rest
        .split_once('=')
        .ok_or_else(|| ErrMode::from_input(input))?;
    let current = match (key.trim(), value) {
        ("v", false) => false,
        ("i", false) => true,
        ("value", true) => prefix == 'g',
        _ => return Err(ErrMode::from_input(input)),
    };
    let text: String = text.chars().filter(|c| !matches!(c, '{' | '}')).collect();
    let text = text.trim().to_string();
    if text.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawElement {
        id,
        prefix,
        nodes: vec![n_pos, n_neg],
        body: RawElementBody::Behavioural { current, text },
        span: Span {
            start: id_span.start,
            end: id_span.end,
            file: Arc::from(""),
            src: Arc::default(),
        },
    })
}

/// Skips whitespace and the `(`, `)` and `,` that some decks put around
/// and between `POLY` controls and coefficients.
fn poly_separators(input: &mut Input<'_>) -> ModalResult<()> {
    loop {
        ws(input)?;
        if opt(one_of(['(', ')', ','])).parse_next(input)?.is_none() {
            return Ok(());
        }
    }
}

/// Parse `E`/`G`/`F`/`H` `POLY`: `<id> n+ n- POLY(n) controls… coefficients…`,
/// where the controls are `n` node pairs (E, G) or `n` `V` line names
/// (F, H).
fn parse_poly(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    let nodes = nodes_n(input, 2)?;
    ws(input)?;
    keyword("poly").parse_next(input)?;
    ws(input)?;
    '('.parse_next(input)?;
    ws(input)?;
    let dims: &str = take_while(1.., |c: char| c.is_ascii_digit()).parse_next(input)?;
    let dims: usize = dims.parse().map_err(|_| ErrMode::from_input(input))?;
    ws(input)?;
    ')'.parse_next(input)?;
    if dims == 0 || dims > 16 {
        return Err(ErrMode::from_input(input));
    }
    let controls = if matches!(prefix, 'e' | 'g') {
        let mut pairs = Vec::with_capacity(dims);
        for _ in 0..dims {
            poly_separators(input)?;
            let (p, _) = node_name(input)?;
            poly_separators(input)?;
            let (n, _) = node_name(input)?;
            pairs.push((p, n));
        }
        PolyControls::Voltages(pairs)
    } else {
        let mut sources = Vec::with_capacity(dims);
        for _ in 0..dims {
            poly_separators(input)?;
            sources.push(node_name(input)?.0);
        }
        PolyControls::Currents(sources)
    };
    let mut coefficients = Vec::new();
    let mut end = id_span.end;
    loop {
        poly_separators(input)?;
        if input.is_empty() {
            break;
        }
        let (value, span) = value_expr(input)?;
        coefficients.push(value);
        end = span.end;
    }
    if coefficients.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    let body = RawElementBody::Poly {
        current: matches!(prefix, 'g' | 'f'),
        controls,
        coefficients,
    };
    Ok(element_line(id, prefix, id_span, nodes, body, end))
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
            src: Arc::default(),
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
            .as_chunks::<2>()
            .0
            .iter()
            .map(|[a, b]| (a.clone(), b.clone()))
            .collect();
        return Ok(Some(RawWaveform::Pwl(pairs)));
    }
    input.reset(&cp);
    Ok(None)
}

/// A keyword that ends at a word boundary, so `ac` doesn't match `acx`.
fn word<'a>(kw: &'static str) -> impl Parser<Input<'a>, &'a str, ErrMode<ContextError>> {
    move |input: &mut Input<'a>| -> ModalResult<&'a str> {
        let cp = input.checkpoint();
        let got = keyword(kw).parse_next(input)?;
        if input
            .chars()
            .next()
            .is_some_and(|c| c.is_alphanumeric() || c == '_')
        {
            input.reset(&cp);
            return Err(ErrMode::from_input(input));
        }
        Ok(got)
    }
}

/// A value if one comes next; `None` (consuming nothing) otherwise.
fn opt_value(input: &mut Input<'_>) -> ModalResult<Option<ParamExpr>> {
    let cp = input.checkpoint();
    match value_expr(input) {
        Ok((v, _)) => Ok(Some(v)),
        Err(ErrMode::Backtrack(_)) => {
            input.reset(&cp);
            Ok(None)
        }
        Err(e) => Err(e),
    }
}

/// Parse `V / I` source: `<id> n+ n- [[DC] v] [AC [mag [phase]]] [waveform]`,
/// the three specs in any order and each at most once, as SPICE3 reads
/// them. No spec at all is 0 DC.
fn parse_source(input: &mut Input<'_>, kind: SourceKind) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    ws(input)?;
    let (n_pos, _) = node_name(input)?;
    ws(input)?;
    let (n_neg, _) = node_name(input)?;
    let mut dc = None;
    let mut ac = None;
    let mut waveform = None;
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        if opt(word("dc")).parse_next(input)?.is_some() {
            ws(input)?;
            let (val, _) = value_expr(input)?;
            if dc.replace(val).is_some() {
                return Err(ErrMode::from_input(input));
            }
        } else if opt(word("ac")).parse_next(input)?.is_some() {
            ws(input)?;
            let magnitude = opt_value(input)?;
            ws(input)?;
            let phase = if magnitude.is_some() {
                opt_value(input)?
            } else {
                None
            };
            if ac.replace(RawAcSpec { magnitude, phase }).is_some() {
                return Err(ErrMode::from_input(input));
            }
        } else if let Some(wf) = parse_waveform(input)? {
            if waveform.replace(wf).is_some() {
                return Err(ErrMode::from_input(input));
            }
        } else {
            let (val, _) = value_expr(input)?;
            if dc.replace(val).is_some() {
                return Err(ErrMode::from_input(input));
            }
        }
    }
    Ok(RawElement {
        id,
        prefix,
        nodes: vec![n_pos, n_neg],
        body: RawElementBody::Source {
            kind,
            dc,
            ac,
            waveform,
        },
        span: Span {
            start: id_span.start,
            end: id_span.end,
            file: Arc::from(""),
            src: Arc::default(),
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
            src: Arc::default(),
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
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    let (model, substrate) = match tokens.as_slice() {
        [_, _, _, model] => (model.clone(), None),
        [_, _, _, substrate, model] => (model.clone(), Some(substrate.clone())),
        _ => return Err(ErrMode::from_input(input)),
    };
    tokens.truncate(3);
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
            src: Arc::default(),
        },
    })
}

/// Parse `M` MOSFET: `<id> nd ng ns nb <model> [name=val ...] [flag ...]`.
/// **Node order is preserved as parsed (drain, gate, source, bulk).** The
/// lowering pass reorders to sindr's `[gate, drain, source]` and checks the
/// bulk.
fn parse_mosfet(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    let mut nodes = Vec::with_capacity(4);
    for _ in 0..4 {
        ws(input)?;
        nodes.push(node_name(input)?.0);
    }
    ws(input)?;
    let (model, _) = node_name(input)?;
    let (params, flags) = instance_tail(input)?;
    Ok(RawElement {
        id,
        prefix,
        nodes,
        body: RawElementBody::Mosfet {
            model,
            params,
            flags,
        },
        span: Span {
            start: id_span.start,
            end: id_span.end,
            file: Arc::from(""),
            src: Arc::default(),
        },
    })
}

/// Parse `J` JFET: `<id> nd ng ns <model> [area] [flag ...]`. **Node order
/// is preserved as parsed (drain, gate, source).**
fn parse_jfet(input: &mut Input<'_>) -> ModalResult<RawElement> {
    let (id, prefix, id_span) = element_id(input)?;
    let mut nodes = Vec::with_capacity(3);
    for _ in 0..3 {
        ws(input)?;
        nodes.push(node_name(input)?.0);
    }
    ws(input)?;
    let (model, _) = node_name(input)?;
    ws(input)?;
    let area = opt(value_expr).parse_next(input)?.map(|(e, _)| e);
    let (params, flags) = instance_tail(input)?;
    if !params.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawElement {
        id,
        prefix,
        nodes,
        body: RawElementBody::Jfet { model, area, flags },
        span: Span {
            start: id_span.start,
            end: id_span.end,
            file: Arc::from(""),
            src: Arc::default(),
        },
    })
}

/// The rest of a device line after its model name: `name=val` pairs and
/// bare flags, in any order, to the end of the line.
#[allow(clippy::type_complexity)]
fn instance_tail(input: &mut Input<'_>) -> ModalResult<(Vec<(String, ParamExpr)>, Vec<String>)> {
    let mut params = Vec::new();
    let mut flags = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            return Ok((params, flags));
        }
        let cp = input.checkpoint();
        if let Ok(pair) = param_assign.parse_next(input) {
            params.push(pair);
            continue;
        }
        input.reset(&cp);
        flags.push(node_name(input)?.0);
    }
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
    let name = match tokens.pop() {
        Some(name) if input.is_empty() => name,
        _ => return Err(ErrMode::from_input(input)),
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
            src: Arc::default(),
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
    let (name, _) = ident(input)?;
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
    ws(input)?;
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Subckt(RawSubckt {
        name,
        ports,
        defaults,
        body: Vec::new(),
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
        // Vendor libraries often separate parameters with commas.
        if opt(',').parse_next(input)?.is_some() {
            continue;
        }
        // Anything that isn't `name=value` is an error: stopping here would
        // drop it and every parameter after it without a word.
        params.push(param_assign.parse_next(input)?);
    }
    ws(input)?;
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Model(RawModel {
        name,
        kind,
        params,
        span: Span {
            start: name_span.start,
            end: name_span.end,
            file: Arc::from(""),
            src: Arc::default(),
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
        let cp = input.checkpoint();
        match param_assign.parse_next(input) {
            Ok(pair) => bindings.push(pair),
            Err(_) => {
                // Rewind: a half-read `b=` must count as input left over.
                input.reset(&cp);
                break;
            }
        }
    }
    ws(input)?;
    if bindings.is_empty() || !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Param(bindings, Span::detached()))
}

/// Parse `.options` / `.option` / `.opt`: `name=value` pairs and bare flags,
/// in any order.
fn parse_card_options(input: &mut Input<'_>) -> ModalResult<RawCard> {
    let mut options: Vec<(String, Option<ParamExpr>)> = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        let cp = input.checkpoint();
        match param_assign.parse_next(input) {
            Ok((name, expr)) => options.push((name, Some(expr))),
            Err(_) => {
                input.reset(&cp);
                let (name, _) = ident(input)?;
                options.push((name, None));
            }
        }
    }
    Ok(RawCard::Options(options, Span::detached()))
}

/// Parse `.global node [node ...]`.
fn parse_card_global(input: &mut Input<'_>) -> ModalResult<RawCard> {
    let mut nets = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        nets.push(node_name(input)?.0);
    }
    if nets.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Global(nets))
}

/// Parse `.ic` / `.nodeset`: one or more `V(node)=value`.
fn parse_card_node_values(input: &mut Input<'_>, kind: NodeValueKind) -> ModalResult<RawCard> {
    let mut values: Vec<(String, ParamExpr)> = Vec::new();
    loop {
        ws(input)?;
        if input.is_empty() {
            break;
        }
        // Case is already folded, so `V(` arrives as `v(`.
        ('v', ws, '(', ws).parse_next(input)?;
        let (node, _) = node_name(input)?;
        (ws, ')', ws, '=', ws).parse_next(input)?;
        let (value, _) = value_expr(input)?;
        values.push((node, value));
    }
    if values.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::NodeValues(kind, values, Span::detached()))
}

/// Parse `.tran tstep tstop [tstart [tmax]] [uic]`.
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
    // Case is already folded.
    let uic = !input.is_empty() && {
        let (word, _) = ident(input)?;
        ws(input)?;
        if word != "uic" || !input.is_empty() {
            return Err(ErrMode::from_input(input));
        }
        true
    };
    if args.len() < 2 || args.len() > 4 {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Analysis(
        RawAnalysis::Tran { args, uic },
        Span::detached(),
    ))
}

/// Parse `.dc source start stop step`.
fn parse_card_dc(input: &mut Input<'_>) -> ModalResult<RawCard> {
    let sweep = |input: &mut Input<'_>| -> ModalResult<(String, Vec<ParamExpr>)> {
        ws(input)?;
        let (source, _) = ident(input)?;
        let mut args: Vec<ParamExpr> = Vec::new();
        while args.len() < 3 {
            ws(input)?;
            let (v, _) = value_expr(input)?;
            args.push(v);
        }
        ws(input)?;
        Ok((source, args))
    };
    let (source, args) = sweep(input)?;
    let outer = if input.is_empty() {
        None
    } else {
        Some(sweep(input)?)
    };
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Analysis(
        RawAnalysis::Dc {
            source,
            args,
            outer,
        },
        Span::detached(),
    ))
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
    Ok(RawCard::Analysis(
        RawAnalysis::Ac {
            sweep,
            points,
            fstart,
            fstop,
        },
        Span::detached(),
    ))
}

/// Parse an output variable: `v(node[,ref])` or `i(vsource)`.
fn output_variable(input: &mut Input<'_>) -> ModalResult<crate::TfOutput> {
    // Case is already folded, so `V(` arrives as `v(`.
    alt((
        delimited(
            ('v', ws, '(', ws),
            (node_name, opt(preceded((ws, ',', ws), node_name))),
            (ws, ')'),
        )
        .map(|((node, _), reference)| crate::TfOutput::Voltage {
            node: node.into(),
            reference: reference.map(|(r, _)| r.into()),
        }),
        delimited(('i', ws, '(', ws), ident, (ws, ')')).map(|(source_id, _)| {
            crate::TfOutput::Current {
                source_id: source_id.into(),
            }
        }),
    ))
    .parse_next(input)
}

/// Parse `.sens v(node[,ref])` or `.sens i(vsource)`, either followed by
/// `ac <sweep> points fstart fstop` for AC sensitivity.
fn parse_card_sens(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    let output = output_variable(input)?;
    ws(input)?;
    if input.is_empty() {
        return Ok(RawCard::Analysis(
            RawAnalysis::Sens { output },
            Span::detached(),
        ));
    }
    let (ac, _) = ident(input)?;
    if ac != "ac" {
        return Err(ErrMode::from_input(input));
    }
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
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Analysis(
        RawAnalysis::SensAc {
            output,
            sweep,
            points,
            fstart,
            fstop,
        },
        Span::detached(),
    ))
}

/// Parse `.tf v(node[,ref]) source` or `.tf i(vsource) source`.
fn parse_card_tf(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    let output = output_variable(input)?;
    ws(input)?;
    let (source, _) = ident(input)?;
    ws(input)?;
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Analysis(
        RawAnalysis::Tf {
            output,
            input: source,
        },
        Span::detached(),
    ))
}

/// Parse `.noise v(node[,ref]) source dec|oct|lin points fstart fstop
/// [pts_per_summary]`. The summary interval is ngspice's printing detail
/// and is read and ignored.
fn parse_card_noise(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    let crate::TfOutput::Voltage { node, reference } = output_variable(input)? else {
        // Noise is measured at a voltage, never a current.
        return Err(ErrMode::from_input(input));
    };
    ws(input)?;
    let (source, _) = ident(input)?;
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
    if !input.is_empty() {
        value_expr(input)?;
        ws(input)?;
    }
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Analysis(
        RawAnalysis::Noise {
            node: node.into_string(),
            reference: reference.map(crate::NodeId::into_string),
            input: source,
            sweep,
            points,
            fstart,
            fstop,
        },
        Span::detached(),
    ))
}

/// Parse `.pz in in_ref out out_ref vol|cur pol|zer|pz`.
fn parse_card_pz(input: &mut Input<'_>) -> ModalResult<RawCard> {
    let mut nodes: [String; 4] = Default::default();
    for node in &mut nodes {
        ws(input)?;
        *node = node_name(input)?.0;
    }
    ws(input)?;
    let transfer = match ident(input)?.0.as_str() {
        "vol" => crate::PzTransfer::VoltageGain,
        "cur" => crate::PzTransfer::Transimpedance,
        _ => return Err(ErrMode::from_input(input)),
    };
    ws(input)?;
    let find = match ident(input)?.0.as_str() {
        "pol" => crate::PzFind::Poles,
        "zer" => crate::PzFind::Zeros,
        "pz" => crate::PzFind::Both,
        _ => return Err(ErrMode::from_input(input)),
    };
    ws(input)?;
    if !input.is_empty() {
        return Err(ErrMode::from_input(input));
    }
    Ok(RawCard::Analysis(
        RawAnalysis::Pz {
            nodes,
            transfer,
            find,
        },
        Span::detached(),
    ))
}

/// Split a `.meas` card into words: `=` stands alone, and parentheses,
/// braces and quotes keep what they hold in one word (`v(a, b)`,
/// `{tstop/2}`). A quoted `'expr'` becomes `{expr}`.
fn meas_words(text: &str) -> Result<Vec<String>, String> {
    let mut words = Vec::new();
    let mut chars = text.chars().peekable();
    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
            continue;
        }
        if c == '=' {
            chars.next();
            words.push("=".to_string());
            continue;
        }
        if c == '\'' {
            chars.next();
            let mut expr = String::from("{");
            loop {
                match chars.next() {
                    Some('\'') => break,
                    Some(ch) => expr.push(ch),
                    None => return Err("unclosed quote".to_string()),
                }
            }
            expr.push('}');
            words.push(expr);
            continue;
        }
        let mut word = String::new();
        let mut depth = 0usize;
        while let Some(&ch) = chars.peek() {
            if depth == 0 && (ch.is_whitespace() || ch == '=') {
                break;
            }
            match ch {
                '(' | '{' => depth += 1,
                ')' | '}' => {
                    depth = depth
                        .checked_sub(1)
                        .ok_or_else(|| format!("unbalanced `{ch}`"))?;
                }
                _ => {}
            }
            word.push(ch);
            chars.next();
        }
        if depth != 0 {
            return Err(format!("unbalanced brackets in `{word}`"));
        }
        words.push(word);
    }
    Ok(words)
}

/// A `.meas` signal: `v(n)`, `v(a,b)`, `vdb(n)`, `vp(n)`, `vm(n)` or `i(x)`.
fn meas_signal(word: &str) -> Result<crate::measure::Signal, String> {
    use crate::measure::{Signal, View};
    let bad = || {
        format!(
            "expected a signal such as `v(out)`, `v(a,b)`, `vdb(out)`, `vp(out)` or `i(r1)`, found `{word}`"
        )
    };
    let (func, args) = word.split_once('(').ok_or_else(bad)?;
    let args = args.strip_suffix(')').ok_or_else(bad)?;
    let args: Vec<&str> = args.split(',').map(str::trim).collect();
    if args.iter().any(|a| a.is_empty()) || args.len() > 2 {
        return Err(bad());
    }
    // `split` yields at least one piece, and at most two passed the check.
    let voltage = |view: View| {
        let mut signal = match args.get(1) {
            None => Signal::v(args[0]),
            Some(r) => Signal::v_diff(args[0], *r),
        };
        signal.view = view;
        signal
    };
    match func {
        "v" | "vm" => Ok(voltage(View::Value)),
        "vdb" => Ok(voltage(View::Db)),
        "vp" => Ok(voltage(View::Phase)),
        "i" if args.len() == 1 => Ok(Signal::i(args[0])),
        "vr" | "vi" => Err(format!(
            "`{word}`: real and imaginary parts aren't supported; use `vm`, `vdb` or `vp`"
        )),
        _ => Err(bad()),
    }
}

/// A value word: a number with an optional SI suffix, or `{expr}`.
fn meas_value(word: &str) -> Result<ParamExpr, String> {
    let mut input = LocatingSlice::new(word);
    match value_expr(&mut input) {
        Ok((expr, _)) if input.is_empty() => Ok(expr),
        _ => Err(format!(
            "expected a number or `{{expression}}`, found `{word}`"
        )),
    }
}

/// The `key=value` options after a `.meas` measurement, up to `stop` (the
/// `targ` keyword) or the end.
fn meas_options<'w>(
    words: &mut std::iter::Peekable<impl Iterator<Item = &'w String>>,
    stop: Option<&str>,
) -> Result<Vec<(String, String)>, String> {
    let mut options = Vec::new();
    while let Some(key) = words.peek() {
        if Some(key.as_str()) == stop {
            break;
        }
        let key = words.next().map(String::as_str).unwrap_or_default();
        if words.next().map(String::as_str) != Some("=") {
            return Err(format!("expected `{key}=<value>`"));
        }
        let value = words
            .next()
            .ok_or_else(|| format!("`{key}=` needs a value"))?;
        options.push((key.to_string(), value.clone()));
    }
    Ok(options)
}

/// `rise=n`, `fall=n` or `cross=n|last` from a crossing's options.
fn meas_edge(
    options: &[(String, String)],
) -> Result<(crate::measure::Edge, crate::measure::Occurrence), String> {
    use crate::measure::{Edge, Occurrence};
    let mut found = None;
    for (key, value) in options {
        let edge = match key.as_str() {
            "rise" => Edge::Rise,
            "fall" => Edge::Fall,
            "cross" => Edge::Cross,
            _ => continue,
        };
        if found.is_some() {
            return Err("give only one of `rise`, `fall` and `cross`".to_string());
        }
        let occurrence = if value == "last" {
            Occurrence::Last
        } else {
            match value.parse::<usize>() {
                Ok(n) if n >= 1 => Occurrence::Nth(n),
                _ => {
                    return Err(format!(
                        "`{key}=` takes a count from 1 or `last`, found `{value}`"
                    ))
                }
            }
        };
        found = Some((edge, occurrence));
    }
    Ok(found.unwrap_or((Edge::Cross, Occurrence::Nth(1))))
}

/// Reject options a form doesn't take.
fn meas_only(options: &[(String, String)], allowed: &[&str]) -> Result<(), String> {
    match options.iter().find(|(k, _)| !allowed.contains(&k.as_str())) {
        Some((key, _)) => Err(format!("`{key}=` isn't supported here")),
        None => Ok(()),
    }
}

fn meas_option(options: &[(String, String)], key: &str) -> Result<Option<ParamExpr>, String> {
    options
        .iter()
        .find(|(k, _)| k == key)
        .map(|(_, v)| meas_value(v))
        .transpose()
}

/// `<signal>=<level>` or `<signal> val=<level>`-style crossing, then its options.
fn meas_crossing(
    signal: &str,
    level: ParamExpr,
    options: &[(String, String)],
) -> Result<RawCrossing, String> {
    let (edge, occurrence) = meas_edge(options)?;
    Ok(RawCrossing {
        signal: meas_signal(signal)?,
        level,
        edge,
        occurrence,
    })
}

/// Parse `.four <f0> <signal> [<signal> …]`; see
/// [`crate::spice::FourierRequest`].
fn parse_card_four(text: &str) -> Result<RawCard, String> {
    use crate::measure::View;

    let words = meas_words(text)?;
    let mut it = words.iter();
    let f0 = meas_value(it.next().ok_or("expected a fundamental frequency")?)?;
    let signals = it
        .map(|word| {
            let signal = meas_signal(word)?;
            if signal.view == View::Value {
                Ok(signal)
            } else {
                Err(format!(
                    "`{word}`: Fourier analysis reads a voltage or a current, such as `v(out)` or `i(r1)`"
                ))
            }
        })
        .collect::<Result<Vec<_>, String>>()?;
    if signals.is_empty() {
        return Err("expected a signal such as `v(out)` after the frequency".to_string());
    }
    Ok(RawCard::Four(f0, signals, Span::detached()))
}

/// Parse `.meas tran|ac|dc <name> …`; see [`crate::spice::MeasureRequest`].
fn parse_card_meas(text: &str) -> Result<RawCard, String> {
    use crate::measure::Stat;
    use crate::spice::MeasureAnalysis;

    let words = meas_words(text)?;
    let mut it = words.iter().peekable();
    let analysis = match it.next().map(String::as_str) {
        Some("tran") => MeasureAnalysis::Tran,
        Some("ac") => MeasureAnalysis::Ac,
        Some("dc") => MeasureAnalysis::Dc,
        Some(other) => return Err(format!("measures `tran`, `ac` or `dc`, not `{other}`")),
        None => return Err("expected `tran`, `ac` or `dc`".to_string()),
    };
    let name = it
        .next()
        .filter(|w| w.as_str() != "=")
        .ok_or("expected a name for the measurement")?
        .clone();
    let keyword = it.next().ok_or("expected what to measure")?.clone();
    let range = ["from", "to"];
    let (kind, options) = match keyword.as_str() {
        "avg" | "rms" | "min" | "max" | "pp" | "integ" | "integral" => {
            let stat = match keyword.as_str() {
                "avg" => Stat::Avg,
                "rms" => Stat::Rms,
                "min" => Stat::Min,
                "max" => Stat::Max,
                "pp" => Stat::PeakToPeak,
                _ => Stat::Integral,
            };
            let signal = meas_signal(it.next().ok_or("expected a signal")?)?;
            let options = meas_options(&mut it, None)?;
            meas_only(&options, &range)?;
            (RawMeasureKind::Stat(signal, stat), options)
        }
        "find" => {
            let signal = meas_signal(it.next().ok_or("expected a signal")?)?;
            if it.peek().is_some_and(|w| w.as_str() == "when") {
                return Err(
                    "`find … when` isn't supported; measure the `when` and then `find … at=`"
                        .to_string(),
                );
            }
            let options = meas_options(&mut it, None)?;
            meas_only(&options, &["at"])?;
            let at = meas_option(&options, "at")?.ok_or("`find` needs `at=<x>`")?;
            (RawMeasureKind::Find(signal, at), Vec::new())
        }
        "when" => {
            let signal = it.next().ok_or("expected `<signal>=<level>`")?.clone();
            if it.next().map(String::as_str) != Some("=") {
                return Err("expected `<signal>=<level>`".to_string());
            }
            let level = it.next().ok_or("expected a level after `=`")?;
            let level = meas_value(level)
                .map_err(|e| format!("{e} (crossing another signal isn't supported)"))?;
            let options = meas_options(&mut it, None)?;
            meas_only(&options, &["rise", "fall", "cross", "from", "to"])?;
            (
                RawMeasureKind::When(meas_crossing(&signal, level, &options)?),
                options,
            )
        }
        "trig" => {
            let trig_signal = it.next().ok_or("expected a signal after `trig`")?.clone();
            let trig_options = meas_options(&mut it, Some("targ"))?;
            if it.next().map(String::as_str) != Some("targ") {
                return Err("`trig` needs a `targ`".to_string());
            }
            let targ_signal = it.next().ok_or("expected a signal after `targ`")?.clone();
            let targ_options = meas_options(&mut it, None)?;
            let crossing = |signal: &str, options: &[(String, String)], which: &str| {
                meas_only(options, &["val", "rise", "fall", "cross"])
                    .map_err(|e| format!("{e} (on `{which}`)"))?;
                let level = meas_option(options, "val")?
                    .ok_or_else(|| format!("`{which}` needs `val=<level>`"))?;
                meas_crossing(signal, level, options)
            };
            (
                RawMeasureKind::Delay(
                    crossing(&trig_signal, &trig_options, "trig")?,
                    crossing(&targ_signal, &targ_options, "targ")?,
                ),
                Vec::new(),
            )
        }
        "deriv" | "derivative" | "param" | "integral_of" => {
            return Err(format!("`{keyword}` measurements aren't supported"));
        }
        other => return Err(format!("unknown measurement `{other}`")),
    };
    Ok(RawCard::Measure(
        RawMeasure {
            analysis,
            name,
            kind,
            from: meas_option(&options, "from")?,
            to: meas_option(&options, "to")?,
        },
        Span::detached(),
    ))
}

/// Parse `.op` (no args).
fn parse_card_op(input: &mut Input<'_>) -> ModalResult<RawCard> {
    ws(input)?;
    Ok(RawCard::Analysis(RawAnalysis::Op, Span::detached()))
}

// ---------------------------------------------------------------------------
// Per-line dispatch
// ---------------------------------------------------------------------------

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
            "tf" => parse_card_tf(&mut input),
            "sens" => parse_card_sens(&mut input),
            "noise" => parse_card_noise(&mut input),
            "pz" => parse_card_pz(&mut input),
            "meas" | "measure" => {
                return parse_card_meas(&trimmed[1 + kind_span.end..]).map_err(|message| {
                    LocalError::Syntax {
                        message: format!("`.{kind}`: {message}"),
                        span: 0..trimmed.len(),
                    }
                });
            }
            "four" => {
                return parse_card_four(&trimmed[1 + kind_span.end..]).map_err(|message| {
                    LocalError::Syntax {
                        message: format!("`.{kind}`: {message}"),
                        span: 0..trimmed.len(),
                    }
                });
            }
            "ic" => parse_card_node_values(&mut input, NodeValueKind::Ic),
            "nodeset" => parse_card_node_values(&mut input, NodeValueKind::Nodeset),
            "options" | "option" | "opt" | "opts" => parse_card_options(&mut input),
            "global" => parse_card_global(&mut input),
            // The preprocessor splices every well-formed one, so only a
            // malformed card or a stray section marker gets here.
            "include" => {
                return Err(LocalError::Syntax {
                    message: "`.include` takes exactly one file path".to_string(),
                    span: 0..trimmed.len(),
                });
            }
            "lib" => {
                return Err(LocalError::Syntax {
                    message: "`.lib` takes a file path and an optional section; \
                              `.lib <section>` alone only opens a section in a library file"
                        .to_string(),
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
            'e' | 'g' | 'f' | 'h' => {
                let form = behavioural_form(trimmed);
                if form == Some("POLY") {
                    let parsed = parse_poly(&mut input);
                    return parsed
                        .map(RawCard::Element)
                        .map_err(|_| LocalError::Syntax {
                            message: format!("malformed `{prefix}` POLY element"),
                            span: 0..trimmed.len(),
                        });
                }
                if form == Some("VALUE") && matches!(prefix, 'e' | 'g') {
                    let parsed = parse_behavioural(&mut input, true);
                    return parsed
                        .map(RawCard::Element)
                        .map_err(|_| LocalError::Syntax {
                            message: format!("malformed `{prefix}` element"),
                            span: 0..trimmed.len(),
                        });
                }
                if let Some(form) = form {
                    return Err(LocalError::UnsupportedDevice {
                        device: format!(
                            "{form} form of a controlled source ({})",
                            prefix.to_ascii_uppercase()
                        ),
                        span: 0..trimmed.len(),
                    });
                }
                if matches!(prefix, 'e' | 'g') {
                    parse_voltage_controlled(&mut input)
                } else {
                    parse_current_controlled(&mut input)
                }
            }
            'k' => parse_coupling(&mut input),
            'm' => parse_mosfet(&mut input),
            'j' => parse_jfet(&mut input),
            'b' => parse_behavioural(&mut input, false),
            's' => {
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
#[cfg(test)]
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
            Ok(c) => anchor_to_line(c, line),
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
            // As in ngspice, `.global` names nets for the whole deck even
            // when written inside a `.subckt`, so it always goes to the top.
            RawCard::Global(nets) => top.push(RawCard::Global(nets)),
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
        for l in lines {
            out.lines.push(LogicalLine {
                text: l.to_string(),
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
                ac: None,
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
    fn elements_mosfet_keeps_spice_order_params_and_flags() {
        let card = parse_one_card_local("m1 d g s b 2n7002 w=20u l=2u off m=2").unwrap();
        let RawCard::Element(el) = card else { panic!() };
        assert_eq!(el.prefix, 'm');
        assert_eq!(el.nodes, ["d", "g", "s", "b"]);
        match el.body {
            RawElementBody::Mosfet {
                model,
                params,
                flags,
            } => {
                assert_eq!(model, "2n7002");
                let names: Vec<_> = params.iter().map(|(k, _)| k.as_str()).collect();
                assert_eq!(names, ["w", "l", "m"]);
                assert!((must_num(&params[0].1) - 20e-6).abs() < 1e-15);
                assert_eq!(flags, ["off"]);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_mosfet_needs_four_nodes_and_a_model() {
        assert!(matches!(
            parse_one_card_local("m1 d g s mosmod"),
            Err(LocalError::Syntax { .. })
        ));
    }

    #[test]
    fn elements_jfet_with_and_without_area() {
        let RawCard::Element(el) = parse_one_card_local("j1 d g s jmod").unwrap() else {
            panic!()
        };
        assert_eq!(el.nodes, ["d", "g", "s"]);
        match el.body {
            RawElementBody::Jfet { model, area, flags } => {
                assert_eq!(model, "jmod");
                assert!(area.is_none() && flags.is_empty());
            }
            other => panic!("got {other:?}"),
        }
        let RawCard::Element(el) = parse_one_card_local("j2 d g s jmod 2.5 off").unwrap() else {
            panic!()
        };
        match el.body {
            RawElementBody::Jfet { area, flags, .. } => {
                assert!((must_num(&area.unwrap()) - 2.5).abs() < 1e-12);
                assert_eq!(flags, ["off"]);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn elements_jfet_refuses_named_parameters() {
        assert!(matches!(
            parse_one_card_local("j1 d g s jmod area=2"),
            Err(LocalError::Syntax { .. })
        ));
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
            RawCard::Analysis(RawAnalysis::Tran { args, uic }, _) => {
                assert!(!uic);
                assert_eq!(args.len(), 2);
                assert!((must_num(&args[0]) - 1e-6).abs() < 1e-15);
                assert!((must_num(&args[1]) - 1e-3).abs() < 1e-12);
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_tran_uic_and_tmax() {
        let card = parse_one_card_local(".tran 1u 1m 0 10n uic").unwrap();
        match card {
            RawCard::Analysis(RawAnalysis::Tran { args, uic }, _) => {
                assert!(uic);
                assert_eq!(args.len(), 4);
                assert!((must_num(&args[3]) - 1e-8).abs() < 1e-20);
            }
            other => panic!("got {other:?}"),
        }
        assert!(parse_one_card_local(".tran 1u 1m bogus").is_err());
        assert!(parse_one_card_local(".tran 1u 1m uic 5").is_err());
    }

    #[test]
    fn cards_ac_dec() {
        let card = parse_one_card_local(".ac dec 50 10 100k").unwrap();
        match card {
            RawCard::Analysis(
                RawAnalysis::Ac {
                    sweep,
                    points,
                    fstart,
                    fstop,
                },
                _,
            ) => {
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
        assert!(matches!(card, RawCard::Analysis(RawAnalysis::Op, _)));
    }

    #[test]
    fn cards_dc_sweep() {
        let card = parse_one_card_local(".dc vin 0 5 0.1").unwrap();
        match card {
            RawCard::Analysis(
                RawAnalysis::Dc {
                    source,
                    args,
                    outer,
                },
                _,
            ) => {
                assert_eq!(source, "vin");
                assert_eq!(args.len(), 3);
                assert!(outer.is_none());
            }
            other => panic!("got {other:?}"),
        }
    }

    #[test]
    fn cards_dc_two_sources() {
        let card = parse_one_card_local(".dc vds 0 10 1 vgs 0 5 0.5").unwrap();
        match card {
            RawCard::Analysis(RawAnalysis::Dc { source, outer, .. }, _) => {
                assert_eq!(source, "vds");
                let (outer, args) = outer.unwrap();
                assert_eq!(outer, "vgs");
                assert_eq!(args.len(), 3);
            }
            other => panic!("got {other:?}"),
        }
        assert!(parse_one_card_local(".dc vds 0 10").is_err());
        assert!(parse_one_card_local(".dc vds 0 10 1 vgs 0 5").is_err());
        assert!(parse_one_card_local(".dc vds 0 10 1 vgs 0 5 1 2").is_err());
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
            RawCard::Param(bindings, _) => {
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
    fn driver_global_inside_subckt_goes_to_the_top() {
        let pre = pre(&[
            ".subckt rc in out",
            ".global vcc vee",
            ".ends",
            ".global gnd2",
        ]);
        let cards = parse(&pre).expect("parse");
        let globals: Vec<Vec<String>> = cards
            .iter()
            .filter_map(|c| match c {
                RawCard::Global(nets) => Some(nets.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(globals, [vec!["vcc", "vee"], vec!["gnd2"]]);
        assert!(
            matches!(&cards[0], RawCard::Global(_)),
            "hoisted out of the body"
        );
        assert!(matches!(&cards[1], RawCard::Subckt(s) if s.body.is_empty()));
    }

    #[test]
    fn driver_global_without_nets_errors() {
        assert!(matches!(
            parse(&pre(&[".global"])),
            Err(SpiceParseError::Syntax { .. })
        ));
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
