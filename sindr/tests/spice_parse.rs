//! What the parser reads a deck as: `.meas` forms, `.param` ordering, line
//! continuations, includes and libraries, and the warnings lenient (and
//! strict) parsing leaves behind.
//!
//! Only compiled when the `spice` feature is enabled.
#![cfg(feature = "spice")]

use sindr::element::{Bjt, Diode, Resistor, VoltageSource};
use std::path::{Path, PathBuf};

use sindr::measure::{Crossing, Measure, Signal, Stat};
use sindr::spice::{
    parse_file, parse_str, parse_str_with_options, MeasureAnalysis, ParseOptions, ParsedNetlist,
    SpiceParseError,
};
use sindr::{CircuitElement, Waveform};

fn lenient() -> ParseOptions {
    ParseOptions::default().with_strict(false)
}

fn warnings(netlist: &ParsedNetlist) -> Vec<&str> {
    netlist
        .warnings
        .iter()
        .map(|w| w.message.as_str())
        .collect()
}

fn element<'a>(netlist: &'a ParsedNetlist, id: &str) -> &'a CircuitElement {
    netlist
        .circuit
        .components
        .iter()
        .find(|c| c.id() == id)
        .unwrap_or_else(|| panic!("no `{id}` in {:?}", netlist.circuit.components))
}

fn resistance(netlist: &ParsedNetlist, id: &str) -> f64 {
    match element(netlist, id) {
        CircuitElement::Resistor(Resistor { resistance, .. }) => *resistance,
        other => panic!("`{id}` isn't a resistor: {other:?}"),
    }
}

/// A fresh, empty directory under the system temp dir.
fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("sindr-parse-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

// ----- .meas ----------------------------------------------------------------

#[test]
fn meas_statistics_and_views() {
    let deck = "* m\nV1 a 0 1\nR1 a 0 1k\n\
                .meas tran r rms v(a)\n\
                .meas tran lo min v(a,0)\n\
                .meas tran q integ i(r1) from='1m' to={2*1m}\n\
                .meas tran q2 integral v(a)\n\
                .meas ac ph max vp(a)\n\
                .meas ac mag max vm(a)\n\
                .meas dc top max v(a)\n\
                .end\n";
    let m = parse_str(deck).unwrap().measurements;
    let got: Vec<_> = m
        .iter()
        .map(|r| (r.analysis, r.measurement.clone()))
        .collect();
    let stat = |name: &str, signal, stat| {
        sindr::measure::Measurement::new(name, Measure::stat(signal, stat))
    };
    assert_eq!(
        got,
        vec![
            (MeasureAnalysis::Tran, stat("r", Signal::v("a"), Stat::Rms)),
            (
                MeasureAnalysis::Tran,
                stat("lo", Signal::v_diff("a", "0"), Stat::Min)
            ),
            (
                MeasureAnalysis::Tran,
                // A quoted value is an expression, as `{…}` is.
                stat("q", Signal::i("r1"), Stat::Integral).window(1e-3, 2e-3)
            ),
            (
                MeasureAnalysis::Tran,
                stat("q2", Signal::v("a"), Stat::Integral)
            ),
            (
                MeasureAnalysis::Ac,
                stat("ph", Signal::v("a").phase(), Stat::Max)
            ),
            (MeasureAnalysis::Ac, stat("mag", Signal::v("a"), Stat::Max)),
            (MeasureAnalysis::Dc, stat("top", Signal::v("a"), Stat::Max)),
        ]
    );
}

#[test]
fn meas_crossings_take_param_levels_and_windows() {
    let deck = "* m\nV1 a 0 1\nR1 a 0 1k\n.param half=0.5 t0=1m\n\
                .meas tran t when v(a)={half} cross=2 from={t0}\n\
                .meas tran d trig v(a) val={half} fall=1 targ v(a) val=0.25 cross=last\n\
                .end\n";
    let m = parse_str(deck).unwrap().measurements;
    assert_eq!(
        m[0].measurement.measure,
        Measure::When(Crossing::new(Signal::v("a"), 0.5).nth(2))
    );
    assert_eq!(
        (m[0].measurement.from, m[0].measurement.to),
        (Some(1e-3), None)
    );
    assert_eq!(
        m[1].measurement.measure,
        Measure::Delay {
            trig: Crossing::new(Signal::v("a"), 0.5).falling(),
            targ: Crossing::new(Signal::v("a"), 0.25).last(),
        }
    );
}

// ----- .param ---------------------------------------------------------------

/// A `.param` list is evaluated in dependency order, not as written, and a
/// reference inside `^` or an operator counts as a dependency.
#[test]
fn params_resolve_in_dependency_order() {
    let deck = "* p\n.param y={x^2} s={x+y} x=3\n.param x2={x}\n\
                V1 a 0 1\nR1 a 0 {y}\nR2 a 0 {s}\nR3 a 0 {x2^x}\n.end\n";
    let netlist = parse_str(deck).unwrap();
    assert_eq!(resistance(&netlist, "r1"), 9.0);
    assert_eq!(resistance(&netlist, "r2"), 12.0);
    assert_eq!(resistance(&netlist, "r3"), 27.0);
}

/// A `.param` inside a `.subckt` is local to each instance, and sees the
/// instance's own parameters.
#[test]
fn params_inside_a_subckt_are_per_instance() {
    let deck = "* p\n.subckt load a R=1k\n.param twice={2*R}\nR1 a 0 {twice}\n.ends\n\
                V1 in 0 1\nX1 in load\nX2 in load R=5k\n.end\n";
    let flat = parse_str(deck)
        .unwrap()
        .circuit
        .flattened()
        .unwrap()
        .into_owned();
    let r = |id: &str| match flat.components.iter().find(|c| c.id() == id) {
        Some(CircuitElement::Resistor(Resistor { resistance, .. })) => *resistance,
        other => panic!("{id}: {other:?}"),
    };
    assert_eq!((r("x1.r1"), r("x2.r1")), (2e3, 10e3));
}

/// A card that redefines a name in terms of itself reads the earlier value.
#[test]
fn a_param_may_refer_to_its_earlier_value() {
    let deck = "* p\n.param r=1k\n.param r={r*2}\nV1 a 0 1\nR1 a 0 {r}\n.end\n";
    assert_eq!(resistance(&parse_str(deck).unwrap(), "r1"), 2e3);
}

// ----- Elements -------------------------------------------------------------

/// A Q line's optional substrate node is dropped with a warning (in both
/// modes), and the rest go into sindr's `[base, collector, emitter]`.
#[test]
fn bjt_substrate_node_is_dropped_with_a_warning() {
    let deck =
        "* q\n.model qm NPN(BF=50)\nV1 c 0 5\nR1 c b 100k\nQ1 c b 0 sub qm\nR2 sub 0 1\n.end\n";
    for netlist in [
        parse_str(deck).unwrap(),
        parse_str_with_options(deck, lenient()).unwrap(),
    ] {
        match element(&netlist, "q1") {
            CircuitElement::Bjt(Bjt { nodes, bf, .. }) => {
                assert_eq!(nodes, &["b", "c", "0"]);
                assert_eq!(*bf, 50.0);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(
            warnings(&netlist),
            ["BJT `q1` substrate node ignored (sindr 4-terminal BJT not yet supported)"]
        );
    }
}

/// `SIN(vo va f td theta phase)`: held at `vo + va·sin(phase)` until
/// `td`, then a sine starting there and decaying at `theta`; the phase is
/// in degrees, as in ngspice.
#[test]
fn sin_reads_delay_damping_and_phase() {
    let deck = "* s\nV1 a 0 SIN(0.5 2 1k 0.25m 3 30)\nR1 a 0 1k\n.end\n";
    let netlist = parse_str(deck).unwrap();
    assert!(netlist.warnings.is_empty());
    match element(&netlist, "v1") {
        CircuitElement::VoltageSource(VoltageSource {
            waveform:
                Some(Waveform::DampedSine {
                    amplitude,
                    frequency,
                    offset,
                    delay,
                    damping,
                    phase,
                }),
            ..
        }) => {
            assert_eq!((*amplitude, *frequency, *offset), (2.0, 1e3, 0.5));
            assert_eq!((*delay, *damping), (0.25e-3, 3.0));
            assert!(
                (phase - std::f64::consts::PI / 6.0).abs() < 1e-15,
                "{phase}"
            );
        }
        other => panic!("{other:?}"),
    }

    // No delay and no damping is a plain sine, its phase in radians.
    let plain = "* s\nV1 a 0 SIN(0 1 1k 0 0 90)\nR1 a 0 1k\n.end\n";
    match element(&parse_str(plain).unwrap(), "v1") {
        CircuitElement::VoltageSource(VoltageSource {
            waveform: Some(Waveform::Sine { phase, .. }),
            ..
        }) => assert_eq!(*phase, std::f64::consts::FRAC_PI_2),
        other => panic!("{other:?}"),
    }

    let seven = "* s\nV1 a 0 SIN(0 1 1k 0 0 0 1)\nR1 a 0 1k\n.end\n";
    assert!(parse_str(seven).is_err());
}

/// `.noise` reads all three sweep kinds, and an optional points-per-summary
/// count after `fstop`, which sindr has no use for.
#[test]
fn noise_reads_every_sweep_kind() {
    use sindr::spice::{AcSweep, AnalysisRequest};
    for (sweep, kind) in [
        ("dec", AcSweep::Dec),
        ("oct", AcSweep::Oct),
        ("lin", AcSweep::Lin),
    ] {
        let deck = format!("* n\nV1 a 0 0\nR1 a 0 1k\n.noise v(a) v1 {sweep} 10 1 1k 5\n.end\n");
        match &parse_str(&deck).unwrap().analyses[..] {
            [AnalysisRequest::Noise {
                sweep, points: 10, ..
            }] => assert_eq!(*sweep, kind),
            other => panic!("{sweep}: {other:?}"),
        }
    }
}

// ----- Lines ----------------------------------------------------------------

/// A `+` line continues the card before it, even when it's empty; one with
/// no card before it is dropped with a warning.
#[test]
fn continuation_lines() {
    let deck = "* c\n+ R9 a 0 5\nV1 a 0 1\nR1 a 0\n+\n+ 2k\n.end\n";
    let netlist = parse_str(deck).unwrap();
    assert_eq!(resistance(&netlist, "r1"), 2e3);
    assert!(netlist.circuit.components.iter().all(|c| c.id() != "r9"));
    assert_eq!(
        warnings(&netlist),
        ["continuation line `+` with no preceding card; ignored"]
    );
    let span = netlist.warnings[0].span.expect("warning has a span");
    assert_eq!(
        &deck[span.offset()..span.offset() + span.len()],
        "+ R9 a 0 5"
    );
}

/// sindr reads `.subckt` only at the top level. One inside a `.subckt`
/// is named in a warning; a `.model` there is read (next test).
#[test]
fn nested_definitions_are_skipped_with_a_warning() {
    let deck = "* n\n.subckt amp in out\n.model dloc D(IS=1e-14)\n\
                .subckt inner p q\nR9 p q 1k\n.ends\nR1 in out 1k\n.ends\n\
                V1 a 0 1\nX1 a 0 amp\n.end\n";
    let netlist = parse_str(deck).unwrap();
    assert_eq!(
        warnings(&netlist),
        [
            "`.subckt inner` inside `.subckt amp` is ignored; sindr reads `.subckt` \
          definitions only at the top level"
        ]
    );
    // The body's own elements still expand.
    let flat = netlist.circuit.flattened().unwrap();
    assert!(flat.components.iter().any(|c| c.id() == "x1.r1"));
}

/// A `.model` inside a `.subckt` is that body's own: its elements use it
/// ahead of a top-level card of the same name, and nothing outside sees it.
#[test]
fn subckt_models_are_local_and_shadow_top_level_ones() {
    let deck = "* m
.model d1n D(IS=1e-9)
                .subckt clamp a k
D1 a k d1n
.model d1n D(IS=2e-14 N=1.5)
.ends
                V1 a 0 1
X1 a 0 clamp
D2 a 0 d1n
.end
";
    let netlist = parse_str(deck).unwrap();
    assert!(netlist.warnings.is_empty(), "{:?}", netlist.warnings);
    let flat = netlist.circuit.flatten().unwrap();
    let model = |id: &str| {
        flat.components
            .iter()
            .find_map(|c| match c {
                CircuitElement::Diode(Diode { id: i, model, .. }) if i == id => *model,
                _ => None,
            })
            .unwrap_or_else(|| panic!("no diode {id}"))
    };
    assert_eq!((model("x1.d1").is, model("x1.d1").n), (2e-14, 1.5));
    assert_eq!(model("d2").is, 1e-9);
    // Outside the subckt, its card isn't there.
    let outside = "* m
.subckt clamp a k
.model dloc D
R1 a k 1k
.ends
                   V1 a 0 1
X1 a 0 clamp
D2 a 0 dloc
.end
";
    assert!(matches!(
        parse_str(outside),
        Err(SpiceParseError::UndefinedModel { name, .. }) if name == "dloc"
    ));
}

// ----- .include and .lib ----------------------------------------------------

#[test]
fn absolute_include_works_from_a_string() {
    let dir = temp_dir("abs");
    let sub = dir.join("sub.cir");
    std::fs::write(&sub, "R2 a 0 3k\n").unwrap();
    let deck = format!(
        "* abs\nV1 a 0 1\nR1 a 0 1k\n.include '{}'\n.end\n",
        sub.display()
    );
    let netlist = parse_str(&deck).unwrap();
    std::fs::remove_dir_all(&dir).unwrap();
    assert_eq!(resistance(&netlist, "r2"), 3e3);
}

#[test]
fn include_search_path_resolves_relative_includes_from_a_string() {
    let dir = temp_dir("search");
    std::fs::write(dir.join("sub.cir"), "R2 a 0 3k\n").unwrap();
    let deck = "* rel\nV1 a 0 1\nR1 a 0 1k\n.include sub.cir\n.end\n";
    let netlist =
        parse_str_with_options(deck, ParseOptions::default().with_include_search_path(&dir))
            .unwrap();
    std::fs::remove_dir_all(&dir).unwrap();
    assert_eq!(resistance(&netlist, "r2"), 3e3);
    assert!(netlist.warnings.is_empty());
}

/// Lenient mode resolves a relative include from a string against the
/// working directory, and says so. Tests run in the crate directory.
#[test]
fn lenient_relative_include_resolves_against_the_working_directory() {
    let deck = "* rel\nV1 in 0 1\n.include tests/fixtures/rc.cir\n.end\n";
    assert!(matches!(
        parse_str(deck),
        Err(SpiceParseError::RelativeIncludeInString { .. })
    ));
    let netlist = parse_str_with_options(deck, lenient()).unwrap();
    assert_eq!(
        warnings(&netlist),
        ["relative include 'tests/fixtures/rc.cir' resolved against current working directory"]
    );
    assert!(netlist.circuit.components.len() > 1);
}

#[test]
fn lib_sections() {
    let dir = temp_dir("lib");
    std::fs::write(
        dir.join("parts.lib"),
        "* parts\n.lib fast\nR2 a 0 1k\n.endl\n.lib slow\nR2 a 0 9k\n",
    )
    .unwrap();
    let deck =
        |section: &str| format!("* lib\nV1 a 0 1\nR1 a 0 1k\n.lib \"parts.lib\" {section}\n.end\n");
    let parse = |section: &str| {
        parse_str_with_options(
            &deck(section),
            ParseOptions::default().with_include_search_path(&dir),
        )
        .map_err(Box::new)
    };
    assert_eq!(resistance(&parse("fast").unwrap(), "r2"), 1e3);
    // A section left open runs to the end of the file.
    assert_eq!(resistance(&parse("slow").unwrap(), "r2"), 9e3);
    // With no section, `.lib` includes the whole file.
    std::fs::write(dir.join("plain.lib"), "R2 a 0 4k\n").unwrap();
    let whole = parse_str_with_options(
        "* lib\nV1 a 0 1\nR1 a 0 1k\n.lib plain.lib\n.end\n",
        ParseOptions::default().with_include_search_path(&dir),
    )
    .unwrap();
    assert_eq!(resistance(&whole, "r2"), 4e3);
    let missing = parse("typ").unwrap_err();
    std::fs::remove_dir_all(&dir).unwrap();
    match *missing {
        SpiceParseError::LibSectionNotFound { section, file, .. } => {
            assert_eq!(section, "typ");
            assert!(file.ends_with("parts.lib"), "{}", file.display());
        }
        other => panic!("expected LibSectionNotFound, got {other:?}"),
    }
}

/// A chain of includes deeper than the limit is an error, not a stack
/// overflow.
#[test]
fn include_depth_is_bounded() {
    let dir = temp_dir("depth");
    let file = |i: usize| dir.join(format!("f{i}.cir"));
    // f0 includes f1 includes … f40, which holds a resistor.
    for i in 0..40 {
        std::fs::write(file(i), format!(".include f{}.cir\n", i + 1)).unwrap();
    }
    std::fs::write(file(40), "R2 a 0 1k\n").unwrap();
    let main = dir.join("main.cir");
    std::fs::write(
        &main,
        "* deep\nV1 a 0 1\nR1 a 0 1k\n.include f0.cir\n.end\n",
    )
    .unwrap();
    let err = parse_file(&main).unwrap_err();

    // Ten deep is fine.
    std::fs::write(file(10), "R2 a 0 1k\n").unwrap();
    let shallow = parse_file(&main);
    std::fs::remove_dir_all(&dir).unwrap();
    match err {
        SpiceParseError::Syntax { message, .. } => {
            assert_eq!(message, "include depth exceeded (32)")
        }
        other => panic!("expected Syntax, got {other:?}"),
    }
    assert_eq!(resistance(&shallow.unwrap(), "r2"), 1e3);
}

/// An include that exists but can't be read as text is an I/O error naming
/// the path, not "not found".
#[test]
fn unreadable_include_is_an_io_error() {
    let dir = temp_dir("io");
    std::fs::create_dir_all(dir.join("adir.cir")).unwrap();
    let main = dir.join("main.cir");
    std::fs::write(
        &main,
        "* io\nV1 a 0 1\nR1 a 0 1k\n.include adir.cir\n.end\n",
    )
    .unwrap();
    let err = parse_file(&main).unwrap_err();
    std::fs::remove_dir_all(&dir).unwrap();
    match err {
        SpiceParseError::IncludeIo { path, .. } => {
            assert!(path.ends_with(Path::new("adir.cir")), "{}", path.display())
        }
        other => panic!("expected IncludeIo, got {other:?}"),
    }
}

/// A lenient warning about `.model` parameters sindr ignores names the file
/// the card is in; it used to name none (an empty string).
#[test]
fn ignored_model_parameters_warning_names_the_file() {
    let dir = temp_dir("model");
    let main = dir.join("main.cir");
    std::fs::write(
        &main,
        "* m\n.model m D(IS=1e-14 VAR=1.1)\nV1 a 0 1\nR1 a b 1k\nD1 b 0 m\n.end\n",
    )
    .unwrap();
    let netlist = sindr::spice::parse_file_with_options(&main, lenient()).unwrap();
    std::fs::remove_dir_all(&dir).unwrap();
    assert_eq!(
        warnings(&netlist),
        ["`.model m` parameter VAR ignored (sindr's model doesn't use them)"]
    );
    assert_eq!(
        netlist.warnings[0].file.as_deref(),
        Some(main.display().to_string().as_str())
    );
}

/// `EG` and `XTI` set how a junction's IS follows temperature, and are
/// read onto the model as given: silicon's, or a Schottky's.
#[test]
fn temperature_coefficients_are_read() {
    let deck = "* t\n.model sk D(IS=1e-8 EG=0.69 XTI=2 TNOM=27)\n\
                .model q1 PNP(IS=1e-15 EG=1.12 XTI=3.5)\n\
                V1 a 0 1\nR1 a b 1k\nD1 b 0 sk\nQ1 0 b a q1\n.end\n";
    let netlist = parse_str(deck).unwrap();
    assert!(netlist.warnings.is_empty(), "{:?}", warnings(&netlist));
    match element(&netlist, "d1") {
        CircuitElement::Diode(Diode { model: Some(m), .. }) => {
            assert_eq!((m.eg, m.xti), (0.69, 2.0));
        }
        other => panic!("{other:?}"),
    }
    match element(&netlist, "q1") {
        CircuitElement::Bjt(Bjt { model: Some(m), .. }) => assert_eq!((m.eg, m.xti), (1.12, 3.5)),
        other => panic!("{other:?}"),
    }
}

/// `TNOM` is the temperature a card's values are at; sindr's are at 27 °C,
/// so any other is refused in strict mode and read as 27 in lenient mode.
#[test]
fn another_nominal_temperature_is_refused() {
    let deck = "* t\n.model d1 D(IS=1e-14 TNOM=25)\nV1 a 0 1\nR1 a b 1k\nD1 b 0 d1\n.end\n";
    let err = parse_str(deck).unwrap_err().to_string();
    assert!(err.contains("TNOM=25") && err.contains("TNOM=27"), "{err}");
    let netlist = parse_str_with_options(deck, lenient()).unwrap();
    assert_eq!(
        warnings(&netlist),
        [
            "`.model d1` parameter TNOM=25 ignored (sindr's model doesn't use them; its values \
          are at TNOM=27)"
        ]
    );
}

// ----- instance parameters ---------------------------------------------------

/// `m`, `scale` and `tc1`/`tc2` at an instance `temp` or `dtemp` set an R
/// line's value as ngspice does; these are the resistances ngspice 47
/// reports for the same lines (`temp` wins over `dtemp`).
#[test]
fn resistor_instance_parameters_set_its_value() {
    let deck = "* r\nV1 a 0 1\n\
                R1 a 0 1k m=2\n\
                R2 a 0 1k tc1=0.01 tc2=1e-4 temp=77\n\
                R3 a 0 1k tc1=0.01 dtemp=10\n\
                R4 a 0 1k scale=3\n\
                R5 a 0 1k tc1=0.01 temp=77 dtemp=10\n\
                R6 a 0 1k tc1=0.01\n\
                .op\n.end\n";
    let netlist = parse_str(deck).unwrap();
    for (id, want) in [
        ("r1", 500.0),
        ("r2", 1750.0),
        ("r3", 1100.0),
        ("r4", 3000.0),
        ("r5", 1500.0),
        ("r6", 1000.0),
    ] {
        assert!(
            (resistance(&netlist, id) - want).abs() < 1e-9,
            "{id}: {}",
            resistance(&netlist, id)
        );
    }
}

/// A D line's area, `m`, `temp` and `dtemp`, against the currents ngspice
/// 47 solves for the same lines across 1 V.
#[test]
fn diode_area_multiplier_and_temperature() {
    let deck = "* d\nV2 d 0 1\n\
                D1 d 0 dmod 2\n\
                D2 d 0 dmod m=3\n\
                D3 d 0 dmod temp=77\n\
                D4 d 0 dmod dtemp=50\n\
                D5 d 0 dmod area=2 pj=1u\n\
                .model dmod d(is=1e-14 rs=10 ibv=1e-3 bv=50)\n.op\n.end\n";
    let netlist = parse_str(deck).unwrap();
    let op = sindr::solve_op(&netlist.circuit).unwrap();
    for (id, want) in [
        ("d1", 0.052_113_7),
        ("d2", 0.078_166_9),
        ("d3", 0.032_917_8),
        ("d4", 0.032_917_8),
        ("d5", 0.052_113_7),
    ] {
        let got = op.current(id).unwrap();
        assert!(
            (got - want).abs() < 1e-4 * want,
            "{id}: {got} A, ngspice {want} A"
        );
    }
    match element(&netlist, "d1") {
        CircuitElement::Diode(Diode { model: Some(m), .. }) => {
            assert_eq!((m.is, m.rs, m.ibv), (2e-14, 5.0, 2e-3));
        }
        other => panic!("{other:?}"),
    }
}

/// `IC=` on C and L lines, with `m` and `scale`, start a `uic` transient
/// where ngspice 47 starts it: at 1 ms v(a) = 1.43306, i(l1) = 5.13417e-4
/// and v(c) = 0.367879. Without `uic` ngspice ignores them, and so does
/// sindr.
#[test]
fn capacitor_and_inductor_initial_conditions_apply_with_uic() {
    let lines = "C1 a 0 1u ic=2 m=2 tc1=0.01 dtemp=50\nR1 a 0 1k\n\
                 L1 b 0 1m ic=1m m=2 scale=3\nR2 b 0 1\n\
                 C2 0 c 1u ic=-1\nR3 c 0 1k\n";
    let run = |tran: &str| {
        let deck = format!("* ic\n{lines}{tran}\n.end\n");
        let netlist = parse_str(&deck).unwrap();
        let config = netlist.transient_config().unwrap();
        let result = sindr::solve_transient(&netlist.circuit, &config).unwrap();
        let tran = result.transient.unwrap();
        (
            tran.voltage_at("a", 1e-3).unwrap(),
            tran.current_at("l1", 1e-3).unwrap(),
            tran.voltage_at("c", 1e-3).unwrap(),
        )
    };
    let (va, il, vc) = run(".tran 1u 2m uic");
    assert!((va - 1.433_06).abs() < 2e-3, "v(a) {va}");
    assert!((il.abs() - 5.134_17e-4).abs() < 2e-6, "i(l1) {il}");
    assert!((vc - 0.367_879).abs() < 1e-3, "v(c) {vc}");
    let (va, il, vc) = run(".tran 1u 2m");
    assert!(va.abs() < 1e-9 && il.abs() < 1e-9 && vc.abs() < 1e-9);
}

/// What sindr can't honour on an instance line is an error naming it in
/// strict mode and a warning in lenient mode.
#[test]
fn unsupported_instance_parameters_are_named() {
    for (line, name) in [
        ("R1 a 0 1k noisy=0", "NOISY"),
        ("R1 a 0 1k ic=1", "IC"),
        ("C1 a 0 1u cmod", "CMOD"),
        ("D1 a 0 dmod off", "OFF"),
        ("D1 a 0 dmod ic=0.6", "IC"),
        ("C1 a b 1u ic=1\nR2 b 0 1k", "neither end"),
        (
            "L1 a 0 1m ic=1\nL2 b 0 1m\nR2 b 0 1\nK1 l1 l2 0.9",
            "coupled",
        ),
    ] {
        let deck = format!("* u\nV1 a 0 1\nR9 a 0 1k\n{line}\n.model dmod d\n.end\n");
        let err = parse_str(&deck).unwrap_err().to_string();
        assert!(err.contains(name), "{line}: {err}");
        let netlist = parse_str_with_options(&deck, lenient()).unwrap();
        assert!(
            warnings(&netlist).iter().any(|w| w.contains(name)),
            "{line}: {:?}",
            warnings(&netlist)
        );
    }
    for line in ["R1 a 0 1k m=0", "D1 a 0 dmod -1", "D1 a 0 dmod temp=-300"] {
        let deck = format!("* u\nV1 a 0 1\nR9 a 0 1k\n{line}\n.model dmod d\n.end\n");
        assert!(parse_str_with_options(&deck, lenient()).is_err(), "{line}");
    }
}

/// `IC=` inside a subckt lands under the flattened names.
#[test]
fn initial_conditions_inside_a_subckt_use_flattened_names() {
    let deck = "* s\n.subckt cell p\nR1 p n 1k\nC1 n 0 1u ic=2\nL1 n 0 1m ic=1m\n.ends\n\
                V1 a 0 1\nX1 a cell\n.tran 1u 1m uic\n.end\n";
    let netlist = parse_str(deck).unwrap();
    assert_eq!(netlist.uic_voltages.get("x1.n"), Some(&2.0));
    assert_eq!(netlist.uic_currents.get("x1.l1"), Some(&1e-3));
    let config = netlist.transient_config().unwrap();
    assert_eq!(config.initial_voltages.get("x1.n"), Some(&2.0));
    assert_eq!(config.initial_currents.get("x1.l1"), Some(&1e-3));
}
