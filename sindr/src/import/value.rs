//! Component values as a schematic writes them: `10k`, `4.7 µF`, `4k7`, `1M`.

/// Powers of ten by prefix. Case matters, as on a schematic: `M` is mega and
/// `m` milli (SPICE reads both as milli), `f` is femto and `F` farad.
const PREFIXES: &[(&str, i32)] = &[
    ("meg", 6),
    ("Meg", 6),
    ("MEG", 6),
    ("T", 12),
    ("G", 9),
    ("M", 6),
    ("k", 3),
    ("K", 3),
    ("m", -3),
    ("u", -6),
    ("µ", -6), // U+00B5 micro sign
    ("μ", -6), // U+03BC Greek mu
    ("n", -9),
    ("p", -12),
    ("f", -15),
];

/// Units a value may end in, compared without case.
const UNITS: &[&str] = &[
    "ohms", "ohm", "\u{3a9}", "\u{2126}", "hz", "f", "h", "v", "a", "w", "s",
];

/// Reads a component value the way a schematic writes it, to a number in
/// SI base units. `None` when the text isn't a value.
///
/// Takes a number with an optional exponent, an optional prefix, and an
/// optional unit: `10k`, `4.7uF`, `4.7 µF`, `100nF`, `2.2mH`, `1e-6`,
/// `5V`, `10 kΩ`. Prefixes are case-sensitive as on a schematic, so `1M`
/// is 1 MΩ, not the 1 mΩ a SPICE deck would read; `meg` is mega too.
/// The prefix (or `R` for 1, or the unit) may stand in for the decimal
/// point, as on a resistor's marking: `4k7` is 4.7 k, `4R7` is 4.7, `2M2`
/// is 2.2 M, `5V1` is 5.1 V.
///
/// ```
/// use sindr::import::parse_value;
///
/// assert_eq!(parse_value("10k"), Some(10e3));
/// assert_eq!(parse_value("4k7"), Some(4.7e3));
/// assert_eq!(parse_value("1M"), Some(1e6));
/// assert_eq!(parse_value("100 nF"), Some(100e-9));
/// assert_eq!(parse_value("5V1"), Some(5.1));
/// assert_eq!(parse_value("BC547"), None);
/// ```
pub fn parse_value(text: &str) -> Option<f64> {
    let s: String = text.chars().filter(|c| !c.is_whitespace()).collect();
    let (sign, rest) = match s.strip_prefix('-') {
        Some(r) => (-1.0, r),
        None => (1.0, s.strip_prefix('+').unwrap_or(&s)),
    };

    let int_len = rest.bytes().take_while(u8::is_ascii_digit).count();
    let (int_part, rest) = rest.split_at(int_len);

    let mut mantissa = int_part.to_string();
    let mut rest = rest;
    let mut has_dot = false;
    if let Some(r) = rest.strip_prefix('.') {
        let frac_len = r.bytes().take_while(u8::is_ascii_digit).count();
        mantissa.push('.');
        mantissa.push_str(&r[..frac_len]);
        rest = &r[frac_len..];
        has_dot = true;
    }
    if !mantissa.bytes().any(|b| b.is_ascii_digit()) {
        return None;
    }

    // An exponent only when digits follow, so `1e` isn't one.
    if let Some(after_e) = rest.strip_prefix(['e', 'E']) {
        let signed = after_e.strip_prefix(['+', '-']).unwrap_or(after_e);
        let digits = signed.bytes().take_while(u8::is_ascii_digit).count();
        if digits > 0 {
            let exp_len = 1 + (after_e.len() - signed.len()) + digits;
            mantissa.push_str(&rest[..exp_len]);
            rest = &rest[exp_len..];
            has_dot = true; // no RKM digits after an exponent
        }
    }

    let mut exponent = 0;
    let mut marker = false;
    if let Some((prefix, e)) = PREFIXES.iter().find(|(p, _)| rest.starts_with(p)) {
        exponent = *e;
        rest = &rest[prefix.len()..];
        marker = true;
    } else if let Some(r) = rest.strip_prefix(['R', 'r']) {
        rest = r;
        marker = true;
    }

    // RKM: digits after the prefix are the fraction (`4k7`). A bare unit
    // can mark the decimal point too (`5V1`).
    let mut fraction = "";
    if !has_dot {
        if marker {
            let n = rest.bytes().take_while(u8::is_ascii_digit).count();
            fraction = &rest[..n];
            rest = &rest[n..];
        } else if let Some(unit) = UNITS.iter().find(|u| starts_with_ignore_case(rest, u)) {
            let after = &rest[unit.len()..];
            let n = after.bytes().take_while(u8::is_ascii_digit).count();
            if n > 0 && n == after.len() {
                fraction = after;
                rest = &rest[..unit.len()];
            }
        }
    }

    if !rest.is_empty()
        && !UNITS
            .iter()
            .any(|u| rest.eq_ignore_ascii_case(u) || rest == *u)
    {
        return None;
    }

    let number = if fraction.is_empty() {
        mantissa
    } else {
        format!("{mantissa}.{fraction}")
    };
    // Scale in the text, not by multiplying, so `100n` is exactly 1e-7.
    let value: f64 = if number.contains(['e', 'E']) {
        number.parse::<f64>().ok()? * 10f64.powi(exponent)
    } else {
        format!("{number}e{exponent}").parse().ok()?
    };
    let value = sign * value;
    value.is_finite().then_some(value)
}

fn starts_with_ignore_case(s: &str, prefix: &str) -> bool {
    s.get(..prefix.len())
        .is_some_and(|head| head.eq_ignore_ascii_case(prefix))
}

#[cfg(test)]
mod tests {
    use super::parse_value;
    use proptest::prelude::*;

    fn close(text: &str, want: f64) {
        let got = parse_value(text).unwrap_or_else(|| panic!("{text:?} didn't parse"));
        assert!(
            (got - want).abs() <= want.abs() * 1e-12,
            "{text:?}: got {got}, want {want}"
        );
    }

    #[test]
    fn plain_and_prefixed() {
        close("100", 100.0);
        close("1.5", 1.5);
        close(".5", 0.5);
        close("10k", 10e3);
        close("10K", 10e3);
        close("2.2m", 2.2e-3);
        close("47u", 47e-6);
        close("100n", 100e-9);
        close("22p", 22e-12);
        close("1G", 1e9);
        close("-12", -12.0);
        close("+5", 5.0);
        close("1e-6", 1e-6);
        close("1.5E3", 1.5e3);
    }

    #[test]
    fn capital_m_is_mega_and_meg_too() {
        close("1M", 1e6);
        close("1Meg", 1e6);
        close("1meg", 1e6);
        close("1m", 1e-3);
    }

    #[test]
    fn micro_in_every_spelling() {
        close("4.7uF", 4.7e-6);
        close("4.7µF", 4.7e-6);
        close("4.7μF", 4.7e-6);
        close("4.7 µF", 4.7e-6);
    }

    #[test]
    fn units() {
        close("10kΩ", 10e3);
        close("10 kohm", 10e3);
        close("1F", 1.0);
        close("1f", 1e-15);
        close("10mH", 10e-3);
        close("5V", 5.0);
        close("100mA", 0.1);
        close("32.768kHz", 32.768e3);
    }

    #[test]
    fn rkm_markings() {
        close("4k7", 4.7e3);
        close("4R7", 4.7);
        close("0R", 0.0);
        close("2M2", 2.2e6);
        close("1n5", 1.5e-9);
        close("3V3", 3.3);
        close("5V1", 5.1);
    }

    #[test]
    fn not_values() {
        for text in [
            "", "BC547", "k", "red", "10x", "1.2.3", "4k7k", "--1", "NE555",
            // An `e` with no exponent digits after it isn't an exponent.
            "1e", "2e+", "3E-", "1ex",
        ] {
            assert_eq!(parse_value(text), None, "{text:?}");
        }
    }

    proptest! {
        #[test]
        fn never_panics(text in "\\PC{0,16}") {
            let _ = parse_value(&text);
        }

        #[test]
        fn plain_numbers_round_trip(x in -1e9f64..1e9) {
            let got = parse_value(&format!("{x}")).unwrap();
            prop_assert!((got - x).abs() <= x.abs() * 1e-12);
        }
    }
}
