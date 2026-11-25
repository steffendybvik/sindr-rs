//! End-to-end error paths: a bad deck must fail with the right error, and the
//! error's label must point at the offending text.
//!
//! Only compiled when the `spice` feature is enabled.
#![cfg(feature = "spice")]

use miette::Diagnostic;
use sindr::element::{Capacitor, Diode};
use sindr::spice::{parse_file, parse_str, SpiceParseError};

/// The text each label of `err` covers in `deck`.
fn labelled(err: &SpiceParseError, deck: &str) -> Vec<String> {
    err.labels()
        .into_iter()
        .flatten()
        .map(|l| {
            deck.get(l.offset()..l.offset() + l.len())
                .unwrap_or("<out of range>")
                .to_string()
        })
        .collect()
}

#[test]
fn unknown_device_points_at_its_prefix() {
    let deck = "* t\nV1 a 0 1\nZ1 a 0 1\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert!(matches!(
        err,
        SpiceParseError::UnknownDevice { prefix: 'z', .. }
    ));
    assert_eq!(labelled(&err, deck), ["Z"]);
}

#[test]
fn relative_include_from_a_string_points_at_the_directive() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 1k\n.include 'nope.cir'\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert!(matches!(
        err,
        SpiceParseError::RelativeIncludeInString { .. }
    ));
    assert_eq!(labelled(&err, deck), [".include 'nope.cir'"]);
}

#[test]
fn missing_include_names_the_path() {
    let dir = std::env::temp_dir().join(format!("sindr-errors-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let main = dir.join("main.cir");
    std::fs::write(
        &main,
        "* t\nV1 a 0 1\nR1 a 0 1k\n.include 'gone.cir'\n.end\n",
    )
    .unwrap();

    let err = parse_file(&main).unwrap_err();
    std::fs::remove_dir_all(&dir).unwrap();
    match err {
        SpiceParseError::IncludeNotFound { path, .. } => {
            assert!(path.ends_with("gone.cir"), "{}", path.display())
        }
        other => panic!("expected IncludeNotFound, got {other:?}"),
    }
}

#[test]
fn undefined_model_is_reported() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 1k\nD1 a 0 nomodel\n.end\n";
    match parse_str(deck).unwrap_err() {
        SpiceParseError::UndefinedModel { name, .. } => assert_eq!(name, "nomodel"),
        other => panic!("expected UndefinedModel, got {other:?}"),
    }
}

#[test]
fn undefined_model_points_at_the_element() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 1k\nD1 a 0 nomodel\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert_eq!(labelled(&err, deck), ["D1 a 0 nomodel"]);
}

#[test]
fn undefined_param_is_reported() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 {missing*2}\n.end\n";
    match parse_str(deck).unwrap_err() {
        SpiceParseError::UndefinedParam { name, .. } => assert_eq!(name, "missing"),
        other => panic!("expected UndefinedParam, got {other:?}"),
    }
}

#[test]
fn undefined_param_points_at_the_reference() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 {missing*2}\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert_eq!(labelled(&err, deck), ["R1 a 0 {missing*2}"]);
}

#[test]
fn circular_params_name_the_cycle() {
    let deck = "* t\n.param a={b+1} b={a*2}\nV1 x 0 1\nR1 x 0 {a}\n.end\n";
    match parse_str(deck).unwrap_err() {
        SpiceParseError::CircularParam { cycle, .. } => {
            let mut names = cycle.clone();
            names.sort();
            names.dedup();
            assert_eq!(names, ["a", "b"]);
        }
        other => panic!("expected CircularParam, got {other:?}"),
    }
}

#[test]
fn invalid_ac_point_count_points_at_the_directive() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 1k\n.ac dec 0 1 1k\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert_eq!(labelled(&err, deck), [".ac dec 0 1 1k"]);
}

#[test]
fn undefined_param_in_a_directive_points_at_it() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 1k\n.tran 1u {tstop}\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert!(matches!(err, SpiceParseError::UndefinedParam { .. }));
    assert_eq!(labelled(&err, deck), [".tran 1u {tstop}"]);
}

#[test]
fn circular_params_point_at_the_param_card() {
    let deck = "* t\n.param a={b+1} b={a*2}\nV1 x 0 1\nR1 x 0 {a}\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert_eq!(labelled(&err, deck), [".param a={b+1} b={a*2}"]);
}

#[test]
fn undefined_model_inside_a_subckt_points_at_its_body_line() {
    let deck =
        "* t\nV1 in 0 1\nX1 in 0 cell\n.subckt cell p q\nR1 p q 1k\nD1 p q ghost\n.ends\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert!(matches!(err, SpiceParseError::UndefinedModel { .. }));
    assert_eq!(labelled(&err, deck), ["D1 p q ghost"]);
}

#[test]
fn error_in_an_included_file_points_into_that_file() {
    let dir = std::env::temp_dir().join(format!("sindr-errors-inc-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("parts.inc"), "* models\nD9 a 0 nomodel\n").unwrap();
    std::fs::write(
        dir.join("main.cir"),
        "* t\nV1 a 0 1\nR1 a 0 1k\n.include 'parts.inc'\n.end\n",
    )
    .unwrap();

    let err = parse_file(dir.join("main.cir")).unwrap_err();
    std::fs::remove_dir_all(&dir).unwrap();
    let source = err.source_code().expect("error carries its source");
    let label = err.labels().into_iter().flatten().next().expect("a label");
    let contents = source.read_span(label.inner(), 0, 0).unwrap();
    assert_eq!(
        std::str::from_utf8(contents.data()).unwrap(),
        "D9 a 0 nomodel"
    );
    assert!(
        contents.name().is_some_and(|n| n.ends_with("parts.inc")),
        "{:?}",
        contents.name()
    );
}

#[test]
fn deeply_nested_expression_is_an_error_not_a_stack_overflow() {
    // 5,000 levels used to overflow a 2 MB thread's stack, which no
    // `catch_unwind` can recover from. Run on such a thread to prove it can't.
    let open = "(".repeat(5_000);
    let close = ")".repeat(5_000);
    let deck = format!("* t\nV1 a 0 1\nR1 a 0 {{{open}1k{close}}}\n.end\n");
    let result = std::thread::Builder::new()
        .stack_size(2 * 1024 * 1024)
        .spawn(move || parse_str(&deck).is_err())
        .unwrap()
        .join()
        .expect("parser must not overflow the stack");
    assert!(result);
}

#[test]
fn moderately_nested_expression_still_parses() {
    let open = "(".repeat(100);
    let close = ")".repeat(100);
    let deck = format!("* t\nV1 a 0 1\nR1 a 0 {{{open}1k{close}}}\n.end\n");
    parse_str(&deck).expect("100 levels is well inside the cap");
}

/// Analysis cards that can't run fail at parse time, pointing at the card,
/// rather than later in the solver with no line.
#[test]
fn unrunnable_analysis_cards_point_at_the_card() {
    for card in [
        ".dc V1 0 1 0",
        ".dc V1 0 1 -0.1",
        ".dc VX 0 1 0.1",
        ".ac dec 10 100 1",
        ".ac dec 10 0 1k",
    ] {
        let deck = format!("* t\nV1 a 0 1\nR1 a 0 1k\n{card}\n.end\n");
        let err = parse_str(&deck).expect_err(card);
        assert!(
            labelled(&err, &deck).iter().any(|l| l.contains(card)),
            "{card}: {err:?} labels {:?}",
            labelled(&err, &deck)
        );
    }
    // Still fine: a sweep to itself, a downward sweep, a linear sweep from 0.
    for card in [".dc V1 1 1 0", ".dc V1 1 0 -0.1", ".ac lin 10 0 1k"] {
        let deck = format!("* t\nV1 a 0 1\nR1 a 0 1k\n{card}\n.end\n");
        parse_str(&deck).unwrap_or_else(|e| panic!("{card}: {e:?}"));
    }
}

#[test]
fn infinite_value_is_an_error() {
    for value in ["1e400", "{1/0}"] {
        let deck = format!("* t\nV1 a 0 1\nR1 a 0 {value}\n.end\n");
        assert!(parse_str(&deck).is_err(), "{value}");
    }
}

#[test]
fn a_suffix_is_atto() {
    let deck = "* t\nV1 a 0 1\nC1 a 0 3a\nR1 a 0 1k\n.end\n";
    let parsed = parse_str(deck).unwrap();
    let c = parsed
        .circuit
        .components
        .iter()
        .find_map(|c| match c {
            sindr::CircuitElement::Capacitor(Capacitor { capacitance, .. }) => Some(*capacitance),
            _ => None,
        })
        .unwrap();
    assert!((c - 3e-18).abs() < 1e-30, "{c}");
}

// ----- Every error variant, and the parser's silent drops ------------------

#[test]
fn unknown_card_names_it() {
    let err = parse_str("* t\nV1 a 0 1\nR1 a 0 1k\n.foo\n.end\n").unwrap_err();
    assert!(matches!(err, SpiceParseError::UnknownCard { ref card, .. } if card == "foo"));
}

#[test]
fn unsupported_device_names_its_prefix() {
    let err = parse_str("* t\nV1 a 0 1\nR1 a 0 1k\nS1 a 0 c 0 sw\n.end\n").unwrap_err();
    assert!(matches!(err, SpiceParseError::UnsupportedDevice { .. }));
}

#[test]
fn undefined_subckt_names_it() {
    let err = parse_str("* t\nV1 a 0 1\nR1 a 0 1k\nX1 a 0 nosuch\n.end\n").unwrap_err();
    assert!(matches!(err, SpiceParseError::UndefinedSubckt { ref name, .. } if name == "nosuch"));
}

#[test]
fn subckt_instance_with_the_wrong_port_count() {
    for instance in ["X1 a foo", "X1 a 0 b foo"] {
        let deck = format!("* t\n.subckt foo a b\nR1 a b 1k\n.ends\nV1 a 0 1\n{instance}\n.end\n");
        let err = parse_str(&deck).unwrap_err();
        assert!(
            matches!(err, SpiceParseError::ArityMismatch { .. }),
            "{instance}: {err:?}"
        );
    }
}

#[test]
fn ends_without_subckt() {
    let err = parse_str("* t\nV1 a 0 1\nR1 a 0 1k\n.ends\n.end\n").unwrap_err();
    assert!(matches!(err, SpiceParseError::Syntax { .. }));
}

#[test]
fn model_parameter_sindr_does_not_model_is_an_error_in_strict_mode() {
    let deck = "* t\n.model m d(is=1e-14 zzz=3)\nV1 a 0 1\nR1 a b 1\nD1 b 0 m\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert!(err.to_string().contains("ZZZ"), "{err}");
}

/// A `.model` card used to stop at the first token it couldn't read and
/// keep what came before, so `n=2` after a comma or a typo was dropped
/// without a word.
#[test]
fn model_card_with_a_bad_parameter_is_an_error() {
    for params in ["d(is=)", "d(is=1e-10 junk n=2)", "d (is=1e-10) n=2"] {
        let deck = format!("* t\n.model m {params}\nV1 a 0 1\nR1 a b 1\nD1 b 0 m\n.end\n");
        let err = parse_str(&deck).unwrap_err();
        assert!(
            matches!(err, SpiceParseError::Syntax { .. }),
            "{params}: {err:?}"
        );
    }
}

#[test]
fn model_card_parameters_may_be_comma_separated() {
    let deck = "* t\n.model m d(is=1e-10, n=2)\nV1 a 0 1\nR1 a b 1\nD1 b 0 m\n.end\n";
    let parsed = parse_str(deck).expect("parses");
    let model = parsed
        .circuit
        .components
        .iter()
        .find_map(|c| match c {
            sindr::CircuitElement::Diode(Diode { model, .. }) => *model,
            _ => None,
        })
        .expect("D1 has a model");
    assert_eq!((model.is, model.n), (1e-10, 2.0));
}

#[test]
fn param_card_with_trailing_junk_is_an_error() {
    for card in [".param a=1 b=", ".param a=1 junk"] {
        let deck = format!("* t\n{card}\nV1 x 0 {{a}}\nR1 x 0 1k\n.end\n");
        let err = parse_str(&deck).unwrap_err();
        assert!(
            matches!(err, SpiceParseError::Syntax { .. }),
            "{card}: {err:?}"
        );
    }
}

#[test]
fn subckt_header_with_trailing_junk_is_an_error() {
    let deck = "* t\n.subckt foo a b r={1+\nR1 a b 1k\n.ends\nV1 a 0 1\nX1 a 0 foo\n.end\n";
    let err = parse_str(deck).unwrap_err();
    assert!(matches!(err, SpiceParseError::Syntax { .. }), "{err:?}");
}

/// The fuzz target's seed decks (`fuzz/seeds/parse`) parse, or fail with an
/// error: never a panic. They run here so CI checks them on stable.
#[test]
fn fuzz_seed_decks_do_not_panic() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../fuzz/seeds/parse");
    let mut seen = 0;
    for entry in std::fs::read_dir(&dir).expect("seed dir") {
        let bytes = std::fs::read(entry.expect("entry").path()).expect("seed");
        // The first byte picks strict or lenient mode.
        let Some((&mode, rest)) = bytes.split_first() else {
            continue;
        };
        let Ok(src) = std::str::from_utf8(rest) else {
            continue;
        };
        let opts = sindr::spice::ParseOptions::default().with_strict(mode & 1 == 0);
        let _ = sindr::spice::parse_str_with_options(src, opts);
        seen += 1;
    }
    assert!(seen > 0, "no seeds in {}", dir.display());
}

// ----- Malformed element and directive lines -------------------------------

/// `* t\nV1 a 0 1\nR1 a 0 1k\n<line>\n.end\n`, parsed strictly.
fn with_line(line: &str) -> (String, SpiceParseError) {
    let deck = format!("* t\nV1 a 0 1\nR1 a 0 1k\n{line}\n.end\n");
    let err = parse_str(&deck).expect_err(line);
    (deck, err)
}

/// Lines the grammar rejects: a `Syntax` error naming the element letter or
/// the card, labelled over the whole line.
#[test]
fn malformed_lines_are_syntax_errors_over_the_line() {
    for (line, what) in [
        // Trailing tokens after the value or model.
        ("R2 a 0 1k 2k", "malformed `r` element"),
        ("D2 a 0 dm extra", "malformed `d` element"),
        ("E1 a 0 a 0 2 extra", "malformed `e` element"),
        // Too few nodes for the letter.
        ("R2 a 1k", "malformed `r` element"),
        ("E1 a b 2", "malformed `e` element"),
        ("F1 a 0 1", "malformed `f` element"),
        ("Q1 c b qmod", "malformed `q` element"),
        // A token that can't be a node ends the node list early.
        ("Q1 c b e {x} qmod", "malformed `q` element"),
        ("X1 a sub (x)", "malformed `x` element"),
        // `ac1` isn't the `AC` keyword followed by 1.
        ("V2 b 0 ac1", "malformed `v` element"),
        ("V2 b 0 ac {", "malformed `v` element"),
        // Waveform argument counts SPICE3 defines.
        ("V2 b 0 PULSE(0 1 0)", "malformed `v` element"),
        ("V2 b 0 SIN(0 1)", "malformed `v` element"),
        ("V2 b 0 SIN(0 1 1k 0 0 7 1)", "malformed `v` element"),
        ("V2 b 0 PWL(0 1 1m)", "malformed `v` element"),
        // Directives.
        (".tran 1u", "malformed `.tran` directive"),
        (".tran 1u 1m 0 1u 2u", "malformed `.tran` directive"),
        (".tran 1u 1m foo", "malformed `.tran` directive"),
        (".tran 1u 1m uic extra", "malformed `.tran` directive"),
        (".dc V1 0 1", "malformed `.dc` directive"),
        (".dc V1 0 1 0.1 V1", "malformed `.dc` directive"),
        (".ac foo 10 1 1k", "malformed `.ac` directive"),
        (
            ".noise v(a) v1 dec 10 1 1k 5 6",
            "malformed `.noise` directive",
        ),
        (".", "expected a control-card keyword after `.`"),
    ] {
        let (deck, err) = with_line(line);
        match &err {
            SpiceParseError::Syntax { message, .. } => assert_eq!(message, what, "{line}"),
            other => panic!("{line}: expected Syntax, got {other:?}"),
        }
        assert_eq!(labelled(&err, &deck), [line], "{line}");
    }
}

/// Values the grammar accepts but the builder rejects once evaluated; each
/// points at its card.
#[test]
fn values_out_of_range_name_the_field() {
    for (line, what) in [
        (
            ".noise v(a) v1 dec 0 1 1k",
            "`.noise` points must be a positive integer in 1..=10000000 (got 0)",
        ),
        (
            ".noise v(a) v1 dec 10 0 1k",
            "`.noise` fstart must be above 0, where flicker noise is finite (got 0)",
        ),
        (
            ".noise v(a) v1 lin 10 2k 1k",
            "`.noise` fstart 2000 is above fstop 1000",
        ),
        (
            ".ac lin 10 -1 10",
            "`.ac` fstart must be at least 0 (got -1)",
        ),
        (".ac dec 10 1 1e400", "value is inf, not a finite number"),
        (".tran 1u {-1e400}", "value is -inf, not a finite number"),
        (
            ".ic v(a)={1e400-1e400}",
            "value is NaN, not a finite number",
        ),
    ] {
        let (deck, err) = with_line(line);
        match &err {
            SpiceParseError::Syntax { message, .. } => assert_eq!(message, what, "{line}"),
            other => panic!("{line}: expected Syntax, got {other:?}"),
        }
        assert_eq!(labelled(&err, &deck), [line], "{line}");
    }
}

/// An undefined parameter anywhere an expression is evaluated late (a
/// waveform, `.meas`, `.ic`, `.options`, a subckt's parameters) names the
/// parameter and points at the line that used it.
#[test]
fn undefined_params_point_at_the_line_that_uses_them() {
    for (line, labelled_text) in [
        (
            "V2 p 0 PULSE(0 {nope} 0 1n 1n 1u 2u)",
            "V2 p 0 PULSE(0 {nope} 0 1n 1n 1u 2u)",
        ),
        ("V2 p 0 SIN(0 {nope} 1k)", "V2 p 0 SIN(0 {nope} 1k)"),
        ("V2 p 0 ac {nope}", "V2 p 0 ac {nope}"),
        (
            ".meas tran x find v(a) at={nope}",
            ".meas tran x find v(a) at={nope}",
        ),
        (
            ".meas tran x when v(a)={nope}",
            ".meas tran x when v(a)={nope}",
        ),
        (
            ".meas tran x avg v(a) from={nope}",
            ".meas tran x avg v(a) from={nope}",
        ),
        (
            ".meas tran x trig v(a) val=1 targ v(a) val={nope}",
            ".meas tran x trig v(a) val=1 targ v(a) val={nope}",
        ),
        (".ic v(a)={nope}", ".ic v(a)={nope}"),
        (".options reltol={nope}", ".options reltol={nope}"),
        (
            ".subckt s p q R={nope}\nR9 p q {R}\n.ends\nX1 a 0 s",
            "X1 a 0 s",
        ),
        (
            ".subckt s p q R=1\nR9 p q {R}\n.ends\nX1 a 0 s R={nope}",
            "X1 a 0 s R={nope}",
        ),
        (
            ".subckt s p q\n.param k={nope}\nR9 p q 1k\n.ends\nX1 a 0 s",
            ".param k={nope}",
        ),
    ] {
        let (deck, err) = with_line(line);
        assert!(
            matches!(&err, SpiceParseError::UndefinedParam { name, .. } if name == "nope"),
            "{line}: {err:?}"
        );
        assert_eq!(labelled(&err, &deck), [labelled_text], "{line}");
    }
}

#[test]
fn a_bjt_on_a_diode_model_is_an_unsupported_model_type() {
    let (deck, err) = with_line("Q1 c b e dmod\n.model dmod D");
    match &err {
        SpiceParseError::UnsupportedModelType { kind, name, .. } => {
            assert_eq!((kind.as_str(), name.as_str()), ("d", "dmod"));
        }
        other => panic!("expected UnsupportedModelType, got {other:?}"),
    }
    assert_eq!(labelled(&err, &deck), ["Q1 c b e dmod"]);
}

/// The `TABLE` and `LAPLACE` forms of `E`/`G`/`F`/`H` are unsupported
/// devices: fatal in strict mode, a warning (and no element) in lenient
/// mode. (`B` lines, `VALUE` and `POLY` are read; see
/// `spice_behavioural.rs`.)
#[test]
fn behavioural_forms_are_unsupported_devices() {
    use sindr::spice::{parse_str_with_options, ParseOptions};
    for (line, device) in [
        (
            "E1 a 0 a 0 table {v(a)} = (0,0) (1,1)",
            "TABLE form of a controlled source (E)",
        ),
        (
            "G1 a 0 laplace {v(a)} = {1/s}",
            "LAPLACE form of a controlled source (G)",
        ),
    ] {
        let (_, err) = with_line(line);
        assert!(
            matches!(&err, SpiceParseError::UnsupportedDevice { device: d, .. } if d == device),
            "{line}: {err:?}"
        );
        let deck = format!("* t\nV1 a 0 1\nR1 a 0 1k\n{line}\n.end\n");
        let lenient =
            parse_str_with_options(&deck, ParseOptions::default().with_strict(false)).unwrap();
        assert_eq!(lenient.circuit.components.len(), 2, "{line}");
        let warnings: Vec<_> = lenient.warnings.iter().map(|w| &w.message).collect();
        assert_eq!(
            warnings,
            [&format!("unsupported device '{device}'")],
            "{line}"
        );
    }
}

/// `.meas` cards the parser can't read, with the reason it gives.
#[test]
fn malformed_meas_cards_say_why() {
    for (card, why) in [
        (".meas", "expected `tran`, `ac` or `dc`"),
        (".meas tran x avg v(a))", "unbalanced `)`"),
        (".meas tran x avg (v(a)", "unbalanced brackets in `(v(a)`"),
        (".meas tran x avg 'v(a)", "unclosed quote"),
        (".meas tran x avg v()", "found `v()`"),
        (".meas tran x avg v(a,b,c)", "found `v(a,b,c)`"),
        (".meas tran x avg q(a)", "found `q(a)`"),
        (".meas tran x avg i(a,b)", "found `i(a,b)`"),
        (".meas tran x avg v(a) from", "expected `from=<value>`"),
        (".meas tran x avg v(a) from=", "`from=` needs a value"),
        (".meas tran x avg v(a) from=abc", "expected a number"),
        (".meas tran x when v(a)", "expected `<signal>=<level>`"),
        (".meas tran x when v(a) 1", "expected `<signal>=<level>`"),
        (".meas tran x when v(a)=", "expected a level after `=`"),
        (".meas tran x when v(a)=1 cross=0", "takes a count from 1"),
        (
            ".meas tran x when v(a)=1 at=1",
            "`at=` isn't supported here",
        ),
        (
            ".meas tran x trig v(a) rise=1 targ v(a) val=1",
            "`trig` needs `val=<level>`",
        ),
        (
            ".meas tran x trig v(a) val=1 targ v(a) foo=1",
            "`foo=` isn't supported here (on `targ`)",
        ),
        (".meas tran x trig", "expected a signal after `trig`"),
        (
            ".meas tran x trig v(a) val=1 targ",
            "expected a signal after `targ`",
        ),
        (".meas tran x bogus v(a)", "unknown measurement `bogus`"),
        (".meas tran", "expected a name for the measurement"),
        (".meas tran x", "expected what to measure"),
        (".meas tran x find", "expected a signal"),
    ] {
        let (deck, err) = with_line(card);
        match &err {
            SpiceParseError::Syntax { message, .. } => {
                assert!(message.starts_with("`.meas`: "), "{card}: {message}");
                assert!(message.contains(why), "{card}: {message}");
            }
            other => panic!("{card}: expected Syntax, got {other:?}"),
        }
        assert_eq!(labelled(&err, &deck), [card], "{card}");
    }
}

/// An include card the preprocessor couldn't read used to reach the parser
/// as "internal: unresolved include reached parser", as if sindr were at
/// fault.
#[test]
fn malformed_include_cards_say_what_they_take() {
    for (line, what) in [
        (".include", "`.include` takes exactly one file path"),
        (
            ".include a.cir b.cir",
            "`.include` takes exactly one file path",
        ),
        (".include 'a.cir", "`.include` takes exactly one file path"),
        (".lib", "`.lib` takes a file path"),
        (".lib \"parts.lib", "`.lib` takes a file path"),
        (".lib fast", "`.lib <section>` alone only opens a section"),
    ] {
        let (deck, err) = with_line(line);
        match &err {
            SpiceParseError::Syntax { message, .. } => {
                assert!(message.contains(what), "{line}: {message}")
            }
            other => panic!("{line}: expected Syntax, got {other:?}"),
        }
        assert_eq!(labelled(&err, &deck), [line], "{line}");
    }
}

/// The nesting cap holds inside an `AC` spec too, where the magnitude is
/// optional: running into it is an error, not "no magnitude".
#[test]
fn deeply_nested_ac_magnitude_is_an_error() {
    let open = "(".repeat(5_000);
    let close = ")".repeat(5_000);
    let (_, err) = with_line(&format!("V2 b 0 ac {{{open}1{close}}}\nR2 b 0 1"));
    assert!(matches!(err, SpiceParseError::Syntax { .. }), "{err:?}");
}
