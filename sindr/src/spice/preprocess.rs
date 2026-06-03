//! SPICE preprocessing pass: title extraction, comment stripping,
//! continuation joining, case folding, and `.include` / `.lib` resolution.
//!
//! This stage runs before the winnow grammar so that the grammar only ever
//! sees one logical line at a time, in lowercase, with no comments and
//! with all transitively included files already spliced in.
//!
//! Three SPICE traps are handled here:
//! - **Title line**: SPICE3 *unconditionally* treats the first non-blank
//!   line of the top-level file as a title and excludes it from grammar
//!   input — even if it looks like a card. Missing this swallows a real
//!   element line on every netlist that lacks a title.
//! - **`Meg` vs `m`**: not handled here directly (see [`crate::spice::si`]),
//!   but case folding happens here so the grammar matches `meg` exactly.
//! - **`.lib file section`**: two-argument `.lib` only splices the lines
//!   between `.lib <section>` and `.endl` markers in the included file.

#![allow(dead_code)] // some helpers are exercised only via tests

use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use miette::{NamedSource, SourceSpan};

use crate::spice::error::{ParseWarning, SpiceParseError};

/// One logical SPICE line after preprocessing.
///
/// "Logical" means: comments stripped, `+`-continuations joined into the
/// preceding line, ASCII-lowercased. The original source text and byte
/// span are preserved so the parser can build [`miette::SourceSpan`]s
/// without re-reading the file.
#[derive(Debug, Clone)]
pub(crate) struct LogicalLine {
    /// Joined, comment-stripped, case-folded text.
    pub text: String,
    /// 1-based line number of the *start* of this logical line in `file`.
    pub start_line: usize,
    /// Source filename (for diagnostic rendering).
    pub file: Arc<str>,
    /// Original source text of the file this line came from.
    pub src: Arc<String>,
    /// Byte range in `src` covering the original (pre-fold) text of this
    /// logical line (start of first physical line through end of last
    /// continuation line). Suitable for `#[label]` SourceSpan construction.
    pub byte_range: std::ops::Range<usize>,
}

/// Result of preprocessing a top-level netlist (and its includes).
#[derive(Debug, Default)]
pub(crate) struct Preprocessed {
    /// Title line (first non-blank line of the top-level file). May be
    /// empty if the file has no non-blank lines.
    pub title: String,
    /// All logical lines in declaration order, with includes spliced in
    /// place.
    pub lines: Vec<LogicalLine>,
}

/// Maximum recursive include depth before we give up.
const MAX_INCLUDE_DEPTH: usize = 32;

/// Preprocess a netlist supplied as an in-memory string.
///
/// `base_dir` controls how relative `.include` / `.lib` paths resolve.
/// In strict mode, a relative include without a `base_dir` is an error
/// ([`SpiceParseError::RelativeIncludeInString`]); in lenient mode it
/// resolves against the current working directory and emits a warning.
pub(crate) fn preprocess_str(
    src: &str,
    filename: &str,
    base_dir: Option<&Path>,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<Preprocessed, SpiceParseError> {
    let mut visited = HashSet::new();
    let file_arc: Arc<str> = Arc::from(filename);
    let src_arc: Arc<String> = Arc::new(src.to_string());

    let (title, lines) = preprocess_inner(
        src_arc.clone(),
        file_arc,
        true, // is_top_level → extract title
        base_dir,
        strict,
        0,
        &mut visited,
        warnings,
    )?;

    Ok(Preprocessed { title, lines })
}

/// Preprocess a netlist read from disk. Relative `.include` paths
/// resolve against `path`'s parent directory.
pub(crate) fn preprocess_file(
    path: &Path,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<Preprocessed, SpiceParseError> {
    let src = fs::read_to_string(path).map_err(|source| SpiceParseError::IncludeIo {
        path: path.to_path_buf(),
        source,
    })?;
    let base_dir = path.parent().map(Path::to_path_buf);
    preprocess_str(
        &src,
        &path.display().to_string(),
        base_dir.as_deref(),
        strict,
        warnings,
    )
}

/// Recursive worker.
///
/// Returns `(title, lines)`. `title` is empty for non-top-level calls
/// (included files do not contribute a title — their first line is a
/// regular card).
#[allow(clippy::too_many_arguments)]
fn preprocess_inner(
    src: Arc<String>,
    file: Arc<str>,
    is_top_level: bool,
    base_dir: Option<&Path>,
    strict: bool,
    depth: usize,
    visited: &mut HashSet<PathBuf>,
    warnings: &mut Vec<ParseWarning>,
) -> Result<(String, Vec<LogicalLine>), SpiceParseError> {
    if depth > MAX_INCLUDE_DEPTH {
        return Err(SpiceParseError::Syntax {
            message: format!("include depth exceeded ({MAX_INCLUDE_DEPTH})"),
            src: NamedSource::new(file.as_ref(), src.as_ref().clone()),
            bad_span: SourceSpan::from(0..0),
        });
    }

    // First pass: split into physical lines with byte ranges, drop full-line
    // and trailing comments, identify the title.
    let physical = collect_physical_lines(&src);

    let mut title = String::new();
    let mut title_taken = false;

    // Walk physical lines and either start a new logical line, append to
    // the prior one (continuation), or splice in an include.
    let mut out_lines: Vec<LogicalLine> = Vec::new();
    let mut current: Option<PendingLine> = None;

    for phys in physical {
        // Title detection (top-level only).
        if is_top_level && !title_taken {
            if phys.is_blank {
                continue;
            }
            // First non-blank line — this is the title, regardless of
            // whether it looks like a `*` comment.
            title = phys.raw.trim().to_string();
            title_taken = true;
            continue;
        }

        // Inside the body: skip pure-blank or pure-comment lines.
        if phys.is_blank || phys.body.trim().is_empty() {
            continue;
        }

        let body_trim_start = leading_ws_len(&phys.body);
        let first_non_ws = phys.body.as_bytes().get(body_trim_start).copied();

        // Continuation line: `+` as first non-whitespace char appends to
        // the current logical line.
        if first_non_ws == Some(b'+') {
            if let Some(cur) = current.as_mut() {
                let after_plus = body_trim_start + 1;
                let extra = phys.body[after_plus..].trim();
                if !extra.is_empty() {
                    cur.text.push(' ');
                    cur.text.push_str(extra);
                }
                cur.byte_range.end = phys.range.end;
                continue;
            }
            // Continuation with nothing to continue — ignore (lenient) or
            // just treat as a normal line. SPICE3 actually errors here;
            // we emit a warning to keep going.
            warnings.push(ParseWarning {
                message: "continuation line `+` with no preceding card; ignored".to_string(),
                span: Some(SourceSpan::from(phys.range.clone())),
                file: Some(file.as_ref().to_string()),
            });
            continue;
        }

        // New logical line. Flush current, then either splice include or
        // start a fresh PendingLine.
        if let Some(cur) = current.take() {
            out_lines.push(cur.finish(file.clone(), src.clone()));
        }

        let folded = phys.body.trim().to_ascii_lowercase();

        // Detect include / lib directives.
        if let Some(directive) = parse_include_or_lib(&folded) {
            // Splice the included file in place.
            handle_include(
                directive,
                &phys,
                &file,
                &src,
                base_dir,
                strict,
                depth,
                visited,
                warnings,
                &mut out_lines,
            )?;
            continue;
        }

        current = Some(PendingLine {
            text: folded,
            start_line: phys.line_number,
            byte_range: phys.range.clone(),
        });
    }

    if let Some(cur) = current {
        out_lines.push(cur.finish(file.clone(), src.clone()));
    }

    Ok((title, out_lines))
}

/// In-flight logical line under construction (continuations may extend it).
struct PendingLine {
    text: String,
    start_line: usize,
    byte_range: std::ops::Range<usize>,
}

impl PendingLine {
    fn finish(self, file: Arc<str>, src: Arc<String>) -> LogicalLine {
        LogicalLine {
            text: self.text,
            start_line: self.start_line,
            file,
            src,
            byte_range: self.byte_range,
        }
    }
}

/// One physical line of input after comment-handling.
struct PhysicalLine {
    /// 1-based line number in the file.
    line_number: usize,
    /// Original raw line text (without trailing newline).
    raw: String,
    /// Body after `;`-trimming. Whole-line `*` comments yield body = "".
    body: String,
    /// True if the original raw line was empty / whitespace-only.
    is_blank: bool,
    /// Byte range in the source string covering this raw line (excluding
    /// the trailing newline).
    range: std::ops::Range<usize>,
}

fn collect_physical_lines(src: &str) -> Vec<PhysicalLine> {
    let mut out = Vec::new();
    let mut line_no = 1usize;
    let mut cursor = 0usize;
    let bytes = src.as_bytes();

    while cursor <= bytes.len() {
        // Find end of this line (LF or end-of-string).
        let mut end = cursor;
        while end < bytes.len() && bytes[end] != b'\n' {
            end += 1;
        }
        let raw_slice = &src[cursor..end];
        // Strip a trailing CR for CRLF inputs.
        let raw = raw_slice
            .strip_suffix('\r')
            .unwrap_or(raw_slice)
            .to_string();

        let is_blank = raw.trim().is_empty();
        let body = strip_comments(&raw);

        out.push(PhysicalLine {
            line_number: line_no,
            raw,
            body,
            is_blank,
            range: cursor..end,
        });

        if end >= bytes.len() {
            break;
        }
        cursor = end + 1;
        line_no += 1;
    }

    out
}

/// Drop full-line `*` comments and trailing `;` comments.
fn strip_comments(line: &str) -> String {
    let leading_ws = leading_ws_len(line);
    if line.as_bytes().get(leading_ws).copied() == Some(b'*') {
        return String::new();
    }
    if let Some(idx) = line.find(';') {
        line[..idx].to_string()
    } else {
        line.to_string()
    }
}

fn leading_ws_len(s: &str) -> usize {
    s.bytes().take_while(|b| matches!(b, b' ' | b'\t')).count()
}

/// Recognised include-shaped directive after case folding.
enum IncludeDirective {
    Include {
        path: String,
    },
    Lib {
        path: String,
        section: Option<String>,
    },
}

/// Parse `.include <path>` / `.lib <path> [section]` from a folded line.
///
/// Returns `None` if the line is not one of these directives.
fn parse_include_or_lib(folded: &str) -> Option<IncludeDirective> {
    let rest = if let Some(r) = folded.strip_prefix(".include") {
        r
    } else if let Some(r) = folded.strip_prefix(".lib") {
        // Two-arg `.lib` is the include form; one-arg too. The card-form
        // `.lib name` (declaration) does not appear in the SPICE3 subset
        // we support, so any `.lib` is an include.
        return parse_lib_args(r);
    } else {
        return None;
    };

    // `.include` requires exactly one quoted-or-bare path.
    let trimmed = rest.trim();
    let path = unquote_one(trimmed)?;
    Some(IncludeDirective::Include { path })
}

fn parse_lib_args(rest: &str) -> Option<IncludeDirective> {
    let trimmed = rest.trim();
    // Peel off one (possibly quoted) token.
    let (first, after) = peel_quoted_or_bare(trimmed)?;
    let section_part = after.trim();

    // A bare single token with no path-like characters is a *section marker*
    // (e.g. `.lib slow` opening a section inside a library file), not an
    // include. Leave it in the line stream so extract_lib_section can find it.
    let was_quoted = trimmed.starts_with('\'') || trimmed.starts_with('"');
    let looks_like_path = first.contains('/') || first.contains('\\') || first.contains('.');
    if section_part.is_empty() && !was_quoted && !looks_like_path {
        return None;
    }

    let section = if section_part.is_empty() {
        None
    } else {
        Some(unquote_one(section_part).unwrap_or_else(|| section_part.to_string()))
    };
    Some(IncludeDirective::Lib {
        path: first,
        section,
    })
}

/// Peel off one possibly-quoted token from the start of `s`. Returns
/// `(token_text_unquoted, remainder_after_token)`.
fn peel_quoted_or_bare(s: &str) -> Option<(String, &str)> {
    let bytes = s.as_bytes();
    if bytes.is_empty() {
        return None;
    }
    let quote = match bytes[0] {
        b'\'' => Some(b'\''),
        b'"' => Some(b'"'),
        _ => None,
    };
    if let Some(q) = quote {
        // Find the matching close quote.
        let close = s.as_bytes()[1..].iter().position(|b| *b == q)?;
        let content = s[1..1 + close].to_string();
        let rest = &s[1 + close + 1..];
        Some((content, rest))
    } else {
        let end = bytes
            .iter()
            .position(|b| matches!(b, b' ' | b'\t'))
            .unwrap_or(bytes.len());
        Some((s[..end].to_string(), &s[end..]))
    }
}

fn unquote_one(s: &str) -> Option<String> {
    let (token, rest) = peel_quoted_or_bare(s)?;
    if !rest.trim().is_empty() {
        return None;
    }
    Some(token)
}

/// Resolve and splice an include / lib directive.
#[allow(clippy::too_many_arguments)]
fn handle_include(
    directive: IncludeDirective,
    phys: &PhysicalLine,
    file: &Arc<str>,
    src: &Arc<String>,
    base_dir: Option<&Path>,
    strict: bool,
    depth: usize,
    visited: &mut HashSet<PathBuf>,
    warnings: &mut Vec<ParseWarning>,
    out_lines: &mut Vec<LogicalLine>,
) -> Result<(), SpiceParseError> {
    let (raw_path, section) = match directive {
        IncludeDirective::Include { path } => (path, None),
        IncludeDirective::Lib { path, section } => (path, section),
    };

    let resolved = resolve_include_path(&raw_path, phys, file, src, base_dir, strict, warnings)?;

    // Cycle detection: canonicalize when possible, else use as-is.
    let canon = fs::canonicalize(&resolved).unwrap_or_else(|_| resolved.clone());
    if !visited.insert(canon.clone()) {
        return Err(SpiceParseError::Syntax {
            message: format!("include cycle detected at {}", resolved.display()),
            src: NamedSource::new(file.as_ref(), src.as_ref().clone()),
            bad_span: SourceSpan::from(phys.range.clone()),
        });
    }

    let included_src = fs::read_to_string(&resolved).map_err(|e| {
        if e.kind() == std::io::ErrorKind::NotFound {
            SpiceParseError::IncludeNotFound {
                path: resolved.clone(),
                src: NamedSource::new(file.as_ref(), src.as_ref().clone()),
                bad_span: SourceSpan::from(phys.range.clone()),
            }
        } else {
            SpiceParseError::IncludeIo {
                path: resolved.clone(),
                source: e,
            }
        }
    })?;

    let included_file_arc: Arc<str> = Arc::from(resolved.display().to_string());
    let included_src_arc: Arc<String> = Arc::new(included_src);
    let inner_base = resolved.parent().map(Path::to_path_buf);

    let (_inner_title, inner_lines) = preprocess_inner(
        included_src_arc,
        included_file_arc,
        false, // included files have no title
        inner_base.as_deref(),
        strict,
        depth + 1,
        visited,
        warnings,
    )?;

    // Pop ourselves from the visited set so siblings can include the same
    // file later (the cycle invariant is per-path, not per-tree-traversal).
    visited.remove(&canon);

    let to_splice = match section {
        None => inner_lines,
        Some(section_name) => match extract_lib_section(&inner_lines, &section_name) {
            Some(lines) => lines,
            None => {
                return Err(SpiceParseError::LibSectionNotFound {
                    section: section_name,
                    file: resolved,
                    src: NamedSource::new(file.as_ref(), src.as_ref().clone()),
                    bad_span: SourceSpan::from(phys.range.clone()),
                });
            }
        },
    };

    out_lines.extend(to_splice);
    Ok(())
}

fn resolve_include_path(
    raw_path: &str,
    phys: &PhysicalLine,
    file: &Arc<str>,
    src: &Arc<String>,
    base_dir: Option<&Path>,
    strict: bool,
    warnings: &mut Vec<ParseWarning>,
) -> Result<PathBuf, SpiceParseError> {
    let p = Path::new(raw_path);
    if p.is_absolute() {
        return Ok(p.to_path_buf());
    }
    if let Some(base) = base_dir {
        return Ok(base.join(p));
    }
    if strict {
        return Err(SpiceParseError::RelativeIncludeInString {
            include: raw_path.to_string(),
            src: NamedSource::new(file.as_ref(), src.as_ref().clone()),
            bad_span: SourceSpan::from(phys.range.clone()),
        });
    }
    // Lenient: resolve against cwd and warn.
    warnings.push(ParseWarning {
        message: format!(
            "relative include '{raw_path}' resolved against current working directory"
        ),
        span: Some(SourceSpan::from(phys.range.clone())),
        file: Some(file.as_ref().to_string()),
    });
    let cwd = std::env::current_dir().map_err(|source| SpiceParseError::IncludeIo {
        path: p.to_path_buf(),
        source,
    })?;
    Ok(cwd.join(p))
}

/// Pull out the lines bounded by `.lib <section>` ... `.endl` in an
/// already-preprocessed (lowercased) line stream.
fn extract_lib_section(lines: &[LogicalLine], section: &str) -> Option<Vec<LogicalLine>> {
    let target = section.to_ascii_lowercase();
    let mut out = Vec::new();
    let mut inside = false;
    for line in lines {
        let t = line.text.trim_start();
        if !inside {
            // `.lib <name>` opens a section.
            if let Some(rest) = t.strip_prefix(".lib") {
                let arg = rest.trim();
                // Must be a single identifier (no path), case-insensitive match.
                if !arg.is_empty() && !arg.contains(char::is_whitespace) && arg == target {
                    inside = true;
                }
            }
        } else {
            // `.endl` closes the current section.
            if t.starts_with(".endl") {
                return Some(out);
            }
            out.push(line.clone());
        }
    }
    if inside {
        // Section opened but never closed — accept what we have.
        Some(out)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pp(src: &str) -> Preprocessed {
        let mut warnings = Vec::new();
        preprocess_str(src, "<test>", None, true, &mut warnings).expect("preprocess succeeded")
    }

    #[test]
    fn captures_title_excludes_from_lines() {
        let p = pp("My title\nR1 a b 1k\n");
        assert_eq!(p.title, "My title");
        assert_eq!(p.lines.len(), 1);
        assert_eq!(p.lines[0].text, "r1 a b 1k");
    }

    #[test]
    fn title_can_start_with_star() {
        // First non-blank line wins, even if it looks like a comment.
        let p = pp("* this is the title\nR1 a b 1k\n");
        assert_eq!(p.title, "* this is the title");
        assert_eq!(p.lines.len(), 1);
    }

    #[test]
    fn full_line_and_inline_comments_stripped() {
        let p = pp("title\n* full comment\nR1 a b 1k ; trailing comment\n");
        assert_eq!(p.lines.len(), 1);
        assert_eq!(p.lines[0].text.trim(), "r1 a b 1k");
    }

    #[test]
    fn continuation_joins_into_previous_line() {
        let p = pp("title\nR1 a b\n+ 1k\n");
        assert_eq!(p.lines.len(), 1);
        assert_eq!(p.lines[0].text, "r1 a b 1k");
        assert_eq!(p.lines[0].start_line, 2);
    }

    #[test]
    fn case_folding_is_uniform() {
        let p = pp("title\nVin In 0 DC 5\n.TRAN 1u 1m\n");
        assert_eq!(p.lines.len(), 2);
        assert_eq!(p.lines[0].text, "vin in 0 dc 5");
        assert_eq!(p.lines[1].text, ".tran 1u 1m");
    }

    #[test]
    fn relative_include_in_string_strict_errors() {
        let mut warnings = Vec::new();
        let err = preprocess_str(
            "title\n.include 'sub.cir'\n",
            "<test>",
            None,
            true,
            &mut warnings,
        )
        .expect_err("strict + relative include should error");
        match err {
            SpiceParseError::RelativeIncludeInString { include, .. } => {
                assert_eq!(include, "sub.cir");
            }
            other => panic!("wrong error variant: {other:?}"),
        }
    }

    #[test]
    fn byte_range_covers_logical_line() {
        let src = "title\nR1 a b 1k\n";
        let p = pp(src);
        let line = &p.lines[0];
        let original = &line.src[line.byte_range.clone()];
        assert_eq!(original, "R1 a b 1k");
    }

    #[test]
    fn include_resolves_relative_to_file_dir() {
        // Use a fixture pair under sindr/tests/fixtures/.
        let main = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/include_main.cir");
        let mut warnings = Vec::new();
        let p =
            preprocess_file(&main, true, &mut warnings).expect("include resolution should succeed");
        // Title from main file, then included card.
        assert_eq!(p.title, "main title");
        let texts: Vec<&str> = p.lines.iter().map(|l| l.text.as_str()).collect();
        assert!(
            texts.iter().any(|t| t.contains("rincl")),
            "expected included resistor in lines: {texts:?}"
        );
    }

    #[test]
    fn lib_section_extracts_only_named_section() {
        let main =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/lib_main.cir");
        let mut warnings = Vec::new();
        let p = preprocess_file(&main, true, &mut warnings)
            .expect("lib section extraction should succeed");
        let texts: Vec<&str> = p.lines.iter().map(|l| l.text.as_str()).collect();
        // `slow` section content present, `fast` section content absent.
        assert!(
            texts.iter().any(|t| t.contains("rslow")),
            "expected slow-section line in {texts:?}"
        );
        assert!(
            !texts.iter().any(|t| t.contains("rfast")),
            "did not expect fast-section line in {texts:?}"
        );
    }

    #[test]
    fn include_cycle_is_caught() {
        let main =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/cycle_a.cir");
        let mut warnings = Vec::new();
        let err =
            preprocess_file(&main, true, &mut warnings).expect_err("self-cycle should be caught");
        match err {
            SpiceParseError::Syntax { message, .. } => {
                assert!(
                    message.contains("cycle"),
                    "expected cycle message, got: {message}"
                );
            }
            other => panic!("expected Syntax error, got {other:?}"),
        }
    }
}
