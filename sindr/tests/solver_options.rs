//! `SolverOptions` and `Solver`: the defaults match the free functions, each
//! option changes what it says, and bad options are rejected.

use std::collections::HashMap;

use sindr::{
    solve_circuit, solve_transient, BjtKind, Circuit, CircuitElement, ConvergenceAid, SimError,
    Solver, SolverOptions, TransientConfig,
};

fn vsource(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn diode(id: &str, a: &str, k: &str) -> CircuitElement {
    CircuitElement::Diode {
        id: id.into(),
        nodes: [a.into(), k.into()],
        temperature: 300.15,
        model: None,
    }
}

fn npn(id: &str, b: &str, c: &str, e: &str) -> CircuitElement {
    CircuitElement::Bjt {
        id: id.into(),
        nodes: [b.into(), c.into(), e.into()],
        kind: BjtKind::Npn,
        bf: 100.0,
        temperature: 300.15,
        parasitic_caps: None,
        model: None,
    }
}

/// 5 V through 1 kΩ into a diode.
fn diode_circuit() -> Circuit {
    Circuit::new(
        "0",
        vec![
            vsource("V1", "vcc", "0", 5.0),
            resistor("R1", "vcc", "a", 1e3),
            diode("D1", "a", "0"),
        ],
    )
}

/// The current mirror from `convergence_aids.rs`: plain Newton–Raphson and
/// gmin stepping both fail, and only source stepping finds the answer.
fn current_mirror() -> Circuit {
    Circuit::new(
        "0",
        vec![
            vsource("VCC", "vcc", "0", 30.0),
            resistor("RREF", "vcc", "m", 1e3),
            npn("Q1", "m", "m", "0"),
            npn("Q2", "m", "out", "0"),
            resistor("RL", "vcc", "out", 1e6),
        ],
    )
}

fn options(f: impl FnOnce(&mut SolverOptions)) -> Solver {
    let mut o = SolverOptions::default();
    f(&mut o);
    Solver::new(o)
}

#[test]
fn default_solver_matches_the_free_functions() {
    let c = diode_circuit();
    assert_eq!(
        Solver::default().circuit(&c).unwrap().node_voltages,
        solve_circuit(&c).unwrap().node_voltages
    );
    let rc = Circuit::new(
        "0",
        vec![
            vsource("V1", "in", "0", 1.0),
            resistor("R1", "in", "out", 1e3),
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    );
    let config = TransientConfig::new(5e-3, 1e-5);
    let free = solve_transient(&rc, &config).unwrap().transient.unwrap();
    let ours = Solver::default()
        .transient(&rc, &config)
        .unwrap()
        .transient
        .unwrap();
    assert_eq!(free.timesteps.len(), ours.timesteps.len());
    assert_eq!(
        free.timesteps.last().unwrap().node_voltages,
        ours.timesteps.last().unwrap().node_voltages
    );
}

/// A tighter `reltol` lands closer to the exact diode voltage: a reference
/// solved at `1e-12` sits within the default's bound, and the tight solve
/// within a much smaller one.
#[test]
fn reltol_tightens_the_newton_answer() {
    let c = diode_circuit();
    let exact = options(|o| {
        o.reltol = 1e-12;
        o.vntol = 1e-15;
    })
    .circuit(&c)
    .unwrap()
    .node_voltages["a"];
    let default = solve_circuit(&c).unwrap().node_voltages["a"];
    let tight = options(|o| o.reltol = 1e-9)
        .circuit(&c)
        .unwrap()
        .node_voltages["a"];
    assert!(
        (default - exact).abs() < 1e-6 + 1e-3 * exact,
        "{default} vs {exact}"
    );
    assert!((tight - exact).abs() < 1e-8, "{tight} vs {exact}");
}

/// With every convergence aid off and one iteration allowed, the error says
/// so and names a node and the parts on it.
#[test]
fn iteration_budget_is_reported_and_names_the_node() {
    let err = options(|o| {
        o.max_iterations = 1;
        o.gmin_steps.clear();
        o.source_stepping = false;
    })
    .circuit(&diode_circuit())
    .unwrap_err();
    match err {
        SimError::ConvergenceFailed {
            iterations,
            node,
            components,
            ..
        } => {
            assert_eq!(iterations, 1);
            let node = node.expect("a node is named");
            assert!(!components.is_empty(), "parts on node {node}");
        }
        other => panic!("expected ConvergenceFailed, got {other:?}"),
    }
}

#[test]
fn source_stepping_can_be_turned_off() {
    let mirror = current_mirror();
    assert_eq!(
        solve_circuit(&mirror).unwrap().convergence_aid,
        Some(ConvergenceAid::SourceStepping)
    );
    let err = options(|o| o.source_stepping = false)
        .circuit(&mirror)
        .unwrap_err();
    match err {
        SimError::ConvergenceFailed { node, .. } => {
            let node = node.expect("a node is named");
            assert!(["m", "out", "vcc"].contains(&node.as_str()), "{node}");
        }
        other => panic!("expected ConvergenceFailed, got {other:?}"),
    }
}

/// The nodeset reaches the bias point of every small-signal analysis too.
#[test]
fn nodeset_seeds_the_ac_bias_point() {
    let c = diode_circuit();
    let config = sindr::AcConfig {
        f_start: 10.0,
        f_stop: 1e3,
        num_points: 3,
        spacing: sindr::FrequencySpacing::Logarithmic,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    };
    let mut seed = HashMap::new();
    seed.insert("a".to_string(), 0.65);
    let plain = sindr::solve_ac(&c, &config).unwrap();
    let seeded = Solver::new(SolverOptions::default().with_nodeset(seed))
        .ac(&c, &config)
        .unwrap();
    let (p, s) = (plain.gain_curve("a"), seeded.gain_curve("a"));
    for ((_, gp), (_, gs)) in p.iter().zip(&s) {
        assert!((gp - gs).abs() < 1e-3, "{gp} vs {gs}");
    }
}

#[test]
fn transient_step_cap_and_trtol_apply() {
    let rc = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(sindr::Waveform::Sine {
                    offset: 0.0,
                    amplitude: 1.0,
                    frequency: 1e3,
                    phase: 0.0,
                }),
            },
            resistor("R1", "in", "out", 1e3),
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    );
    let config = TransientConfig::new(5e-3, 1e-5);
    let err = options(|o| o.max_transient_steps = 100)
        .transient(&rc, &config)
        .unwrap_err();
    assert!(err.to_string().contains("allowed"), "{err}");

    let steps = |trtol: f64| {
        options(|o| o.trtol = trtol)
            .transient(&rc, &config)
            .unwrap()
            .transient
            .unwrap()
            .stats
            .accepted_steps
    };
    assert!(steps(0.01) > steps(7.0), "a tighter trtol takes more steps");
}

#[test]
fn bad_options_are_rejected() {
    let c = diode_circuit();
    type Break = Box<dyn Fn(&mut SolverOptions)>;
    let cases: Vec<(&str, Break)> = vec![
        ("reltol", Box::new(|o| o.reltol = 0.0)),
        ("vntol", Box::new(|o| o.vntol = f64::NAN)),
        ("trtol", Box::new(|o| o.trtol = -1.0)),
        ("max_iterations", Box::new(|o| o.max_iterations = 0)),
        ("gmin step", Box::new(|o| o.gmin_steps = vec![1e-3, 0.0])),
        ("min_source_step", Box::new(|o| o.min_source_step = 1.0)),
        (
            "max_transient_steps",
            Box::new(|o| o.max_transient_steps = 0),
        ),
        (
            "nodeset",
            Box::new(|o| {
                o.nodeset.insert("a".into(), f64::INFINITY);
            }),
        ),
    ];
    for (what, f) in cases {
        let mut o = SolverOptions::default();
        f(&mut o);
        let err = Solver::new(o).circuit(&c).unwrap_err();
        assert!(
            matches!(&err, SimError::InvalidComponent(m) if m.contains(what)),
            "{what}: {err}"
        );
    }
}

#[allow(deprecated)]
#[test]
fn deprecated_seeded_functions_still_work() {
    let mut seed = HashMap::new();
    seed.insert("a".to_string(), 0.7);
    let old = sindr::solve_circuit_with_initial_voltages(&diode_circuit(), &seed).unwrap();
    let new = Solver::new(SolverOptions::default().with_nodeset(seed))
        .circuit(&diode_circuit())
        .unwrap();
    assert_eq!(old.node_voltages, new.node_voltages);
}

#[cfg(feature = "serde")]
#[test]
fn options_round_trip_through_json_and_fill_defaults() {
    let mut o = SolverOptions::default();
    o.reltol = 1e-5;
    o.nodeset.insert("a".into(), 0.6);
    let json = serde_json::to_string(&o).unwrap();
    assert_eq!(serde_json::from_str::<SolverOptions>(&json).unwrap(), o);

    let partial: SolverOptions = serde_json::from_str(r#"{"max_iterations": 7}"#).unwrap();
    assert_eq!(partial.max_iterations, 7);
    assert_eq!(partial.reltol, SolverOptions::default().reltol);
}

#[cfg(feature = "spice")]
mod spice_options {
    use sindr::spice::{parse_str, parse_str_with_options, ParseOptions};

    const DECK: &str = "* opts\nV1 in 0 5\nR1 in a 1k\nD1 a 0 dmod\n.model dmod D\n";

    #[test]
    fn known_options_are_read() {
        let deck = format!(
            "{DECK}.options reltol=1e-5 vntol=1n\n.opt trtol=3 ITL1=250\n.nodeset v(a)=0.6\n.op\n.end\n"
        );
        let nl = parse_str(&deck).unwrap();
        assert_eq!(nl.options.reltol, 1e-5);
        assert_eq!(nl.options.vntol, 1e-9);
        assert_eq!(nl.options.trtol, 3.0);
        assert_eq!(nl.options.max_iterations, 250);
        assert_eq!(nl.options.nodeset.get("a"), Some(&0.6));
        nl.solver().circuit(&nl.circuit).unwrap();
    }

    #[test]
    fn a_later_card_wins() {
        let deck = format!("{DECK}.options reltol=1e-4\n.options reltol=1e-6\n.end\n");
        assert_eq!(parse_str(&deck).unwrap().options.reltol, 1e-6);
    }

    #[test]
    fn unsupported_options_fail_strict_and_warn_lenient() {
        let deck = format!("{DECK}.options abstol=1p noacct\n.op\n.end\n");
        let err = parse_str(&deck).unwrap_err().to_string();
        assert!(err.contains("abstol"), "{err}");

        let opts = ParseOptions {
            strict: false,
            include_search_path: None,
        };
        let nl = parse_str_with_options(&deck, opts).unwrap();
        assert_eq!(nl.warnings.len(), 2, "{:?}", nl.warnings);
        assert!(nl.warnings[1].message.contains("noacct"));
    }

    #[test]
    fn bad_values_are_errors() {
        for card in [".options reltol=0", ".options itl1=2.5", ".options reltol"] {
            let deck = format!("{DECK}{card}\n.end\n");
            assert!(parse_str(&deck).is_err(), "{card}");
        }
    }
}
