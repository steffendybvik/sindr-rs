//! SPICE SI / engineering-suffix decoding.
//!
//! SPICE3 numeric literals carry an optional case-insensitive suffix
//! (`k`, `meg`, `u`, `pF`, ...) followed by arbitrary trailing alphabetic
//! "decoration" that is silently ignored: `4.7uF` parses as 4.7e-6 with
//! the `F` discarded.
//!
//! The order of the suffix table matters: longer prefixes must be tried
//! before any shorter prefix that is a substring of them. The infamous
//! example is `meg` vs `m` — without longest-match, `1Meg` would parse
//! as 1e-3 with a leftover `eg`.

#![allow(dead_code)] // some helpers are exercised only via tests

/// Suffix table — longest prefixes first, case-insensitive at lookup.
///
/// `mil` (25.4 µm) is a SPICE convention for thousandths of an inch and
/// must precede `m` for the same longest-match reason as `meg`.
const SUFFIXES: &[(&str, f64)] = &[
    ("meg", 1e6),
    ("mil", 25.4e-6),
    ("t", 1e12),
    ("g", 1e9),
    ("k", 1e3),
    ("m", 1e-3),
    ("u", 1e-6),
    ("n", 1e-9),
    ("p", 1e-12),
    ("f", 1e-15),
];

/// Returns `(multiplier, suffix_len_in_bytes)`.
///
/// Returns `(1.0, 0)` if no suffix matched. The lookup is
/// case-insensitive and tries the [`SUFFIXES`] table in declaration order
/// so longest matches win.
pub(crate) fn match_si_suffix(s: &str) -> (f64, usize) {
    for &(suffix, mult) in SUFFIXES {
        if let Some(head) = s.get(..suffix.len()) {
            if head.eq_ignore_ascii_case(suffix) {
                return (mult, suffix.len());
            }
        }
    }
    (1.0, 0)
}

/// Parse a leading numeric literal with optional SI suffix and trailing
/// alphabetic "decoration" (ignored).
///
/// Returns `(value, total_bytes_consumed)` or `None` if the input does
/// not start with a number.
///
/// Accepts: `1`, `1.5`, `.5`, `1e6`, `1.5E-3`, `2k`, `1MEG`, `4.7uF`,
/// `10kohm`. After the suffix, any trailing ASCII alphabetic characters
/// are consumed and discarded so that unit-bearing values like `4.7uF`
/// or `10kohm` round-trip cleanly.
pub(crate) fn parse_number(s: &str) -> Option<(f64, usize)> {
    if s.is_empty() {
        return None;
    }
    let bytes = s.as_bytes();
    let mut idx = 0;

    // Optional leading sign.
    if matches!(bytes.first(), Some(b'+') | Some(b'-')) {
        idx += 1;
    }

    // Mantissa: digits and at most one decimal point.
    let mut saw_digit = false;
    let mut saw_dot = false;
    while idx < bytes.len() {
        let c = bytes[idx];
        if c.is_ascii_digit() {
            saw_digit = true;
            idx += 1;
        } else if c == b'.' && !saw_dot {
            saw_dot = true;
            idx += 1;
        } else {
            break;
        }
    }
    if !saw_digit {
        return None;
    }

    // Optional exponent: e/E followed by optional sign and at least one digit.
    // If no digits follow, leave idx where it is so the trailing `e` falls
    // through to the alphabetic-decoration sink. (No SPICE suffix starts with
    // `e`, so this is unambiguous.)
    if idx < bytes.len() && (bytes[idx] == b'e' || bytes[idx] == b'E') {
        let mut j = idx + 1;
        if j < bytes.len() && (bytes[j] == b'+' || bytes[j] == b'-') {
            j += 1;
        }
        let exp_digits_start = j;
        while j < bytes.len() && bytes[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_digits_start {
            idx = j;
        }
    }

    let value: f64 = s[..idx].parse().ok()?;

    // SI suffix on the remainder.
    let (mult, suffix_len) = match_si_suffix(&s[idx..]);
    idx += suffix_len;

    // Trailing alphabetic decoration: SPICE discards `4.7uF`'s `F`,
    // `10kohm`'s `ohm`, etc.
    while idx < bytes.len() && bytes[idx].is_ascii_alphabetic() {
        idx += 1;
    }

    Some((value * mult, idx))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn val(s: &str) -> f64 {
        let (v, n) = parse_number(s).expect("parse_number returned None");
        assert_eq!(n, s.len(), "parse_number did not consume the full input");
        v
    }

    #[test]
    fn plain_integer() {
        assert_eq!(val("100"), 100.0);
    }

    #[test]
    fn k_suffix() {
        assert_eq!(val("1k"), 1000.0);
    }

    // `M` vs `Meg`: `Meg` (case-insensitive) is 1e6, a bare `m` is 1e-3.
    // They must never be confused — this is the most common SPICE numeric trap.
    #[test]
    fn meg_is_1e6_all_cases() {
        assert_eq!(val("1Meg"), 1e6);
        assert_eq!(val("1MEG"), 1e6);
        assert_eq!(val("1meg"), 1e6);
    }

    #[test]
    fn lowercase_m_is_milli() {
        assert_eq!(val("1m"), 1e-3);
    }

    #[test]
    fn mil_is_thousandths_of_inch() {
        // 25.4 µm — must beat the bare `m` suffix.
        assert!((val("1mil") - 25.4e-6).abs() < 1e-12);
    }

    #[test]
    fn micro_with_unit_decoration() {
        assert!((val("4.7uF") - 4.7e-6).abs() < 1e-18);
    }

    #[test]
    fn kohm_decoration() {
        assert_eq!(val("10kohm"), 1e4);
    }

    #[test]
    fn scientific_notation() {
        assert_eq!(val("2.5e3"), 2500.0);
        assert_eq!(val("1e-3"), 1e-3);
    }

    #[test]
    fn leading_dot() {
        assert_eq!(val(".5"), 0.5);
    }

    #[test]
    fn non_numeric_returns_none() {
        assert!(parse_number("abc").is_none());
        assert!(parse_number("").is_none());
    }

    #[test]
    fn signed() {
        assert_eq!(val("-2k"), -2000.0);
        assert_eq!(val("+1.5"), 1.5);
    }

    #[test]
    fn match_suffix_longest_wins() {
        assert_eq!(match_si_suffix("Meg"), (1e6, 3));
        assert_eq!(match_si_suffix("meg"), (1e6, 3));
        assert_eq!(match_si_suffix("mil"), (25.4e-6, 3));
        assert_eq!(match_si_suffix("m"), (1e-3, 1));
        assert_eq!(match_si_suffix(""), (1.0, 0));
        assert_eq!(match_si_suffix("xyz"), (1.0, 0));
    }
}
