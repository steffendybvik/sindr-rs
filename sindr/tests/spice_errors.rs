//! End-to-end error paths: a bad deck must fail with the right error, and the
//! error's label must point at the offending text.
//!
//! Only compiled when the `spice` feature is enabled.
#![cfg(feature = "spice")]

use miette::Diagnostic;
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
#[ignore = "SIN-29: build-stage errors lose their source location"]
fn undefined_model_points_at_the_element() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 1k\nD1 a 0 nomodel\n.end\n";
    let err = parse_str(deck).unwrap_err();
    let labels = labelled(&err, deck);
    assert!(
        labels
            .iter()
            .any(|l| l.contains("nomodel") || l.contains("D1")),
        "labels {labels:?}"
    );
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
#[ignore = "SIN-29: build-stage errors lose their source location"]
fn undefined_param_points_at_the_reference() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 {missing*2}\n.end\n";
    let err = parse_str(deck).unwrap_err();
    let labels = labelled(&err, deck);
    assert!(
        labels.iter().any(|l| l.contains("missing")),
        "labels {labels:?}"
    );
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
fn invalid_ac_point_count_is_rejected() {
    let deck = "* t\nV1 a 0 1\nR1 a 0 1k\n.ac dec 0 1 1k\n.end\n";
    assert!(parse_str(deck).is_err());
}
