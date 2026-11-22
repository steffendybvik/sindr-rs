//! `SolverOptions` and `Solver`: the defaults match the free functions, each
//! option changes what it says, and bad options are rejected.

use std::collections::HashMap;

use sindr::dc_sweep::DcSweepAxis;
use sindr::{
    solve_ac_sensitivity, solve_circuit, solve_loop_gain, solve_pz, solve_transient, AcSensConfig,
    BjtKind, Circuit, CircuitElement, ConvergenceAid, LoopGainConfig, NoiseConfig, PzConfig,
    SensConfig, SimError, SolvePoint, Solver, SolverOptions, TfConfig, TfOutput, TransientConfig,
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
    let config = sindr::AcConfig::new("V1", 10.0, 1e3, 3);
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

#[test]
fn nodeset_seeds_every_analysis_to_the_same_answer() {
    // The diode circuit has one bias point, so a seeded solver lands where
    // the plain free functions do, to within Newton's tolerance.
    let close = |a: f64, b: f64| (a - b).abs() <= 1e-5 * a.abs().max(b.abs()) + 1e-12;
    let mut seed = HashMap::new();
    seed.insert("a".to_string(), 0.7);
    let solver = Solver::new(SolverOptions::default().with_nodeset(seed));
    let c = diode_circuit();
    let plain = sindr::solve_circuit(&c).unwrap();
    let seeded = solver.circuit(&c).unwrap();
    for (node, v) in &plain.node_voltages {
        assert!(close(seeded.node_voltages[node], *v), "{node}");
    }

    let out = TfOutput::voltage("a");
    let tf = TfConfig::new(out.clone(), "V1");
    let (plain, seeded) = (
        sindr::solve_tf(&c, &tf).unwrap(),
        solver.tf(&c, &tf).unwrap(),
    );
    assert!(close(plain.gain, seeded.gain));
    assert!(close(plain.input_resistance, seeded.input_resistance));
    assert!(close(plain.output_resistance, seeded.output_resistance));

    let sens = SensConfig::new(out);
    let (plain, seeded) = (
        sindr::solve_sensitivity(&c, &sens).unwrap(),
        solver.sensitivity(&c, &sens).unwrap(),
    );
    assert_eq!(plain.entries.len(), seeded.entries.len());
    for (p, s) in plain.entries.iter().zip(&seeded.entries) {
        assert_eq!(p.component_id, s.component_id);
        assert!(close(p.sensitivity, s.sensitivity));
    }

    let noise = NoiseConfig::new("a", "V1", 10.0, 1e3, 3);
    let (plain, seeded) = (
        sindr::solve_noise(&c, &noise).unwrap(),
        solver.noise(&c, &noise).unwrap(),
    );
    for (p, s) in plain.points.iter().zip(&seeded.points) {
        assert!(close(p.output_psd, s.output_psd));
    }

    let ac = sindr::AcConfig::new("V1", 10.0, 1e3, 3);
    let (plain, seeded) = (
        sindr::solve_ac(&c, &ac).unwrap(),
        solver.ac(&c, &ac).unwrap(),
    );
    for ((_, p), (_, s)) in plain.gain_curve("a").iter().zip(seeded.gain_curve("a")) {
        assert!(close(*p, s));
    }
    for ((_, p), (_, s)) in plain.phase_curve("a").iter().zip(seeded.phase_curve("a")) {
        assert!(close(*p, s));
    }
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

        let opts = ParseOptions::default().with_strict(false);
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

/// Every analysis a `Solver` runs, on `c` (which has `V1` and a node `a`),
/// each failure tagged with the analysis' name.
fn every_analysis(solver: &Solver, c: &Circuit) -> Vec<(&'static str, Result<(), SimError>)> {
    let out = TfOutput::Voltage {
        node: "a".into(),
        reference: None,
    };
    let tf = TfConfig::new(out.clone(), "V1");
    let noise = NoiseConfig::new("a", "V1", 10.0, 1e3, 3);
    let ac = sindr::AcConfig::new("V1", 10.0, 1e3, 3);
    let inner = DcSweepAxis::new("V1", 5.0, 5.001, 2);
    let outer = DcSweepAxis::new("V2", 1.0, 1.001, 2);
    vec![
        ("op", solver.op(c).map(drop)),
        ("ac", solver.ac(c, &ac).map(drop)),
        ("noise", solver.noise(c, &noise).map(drop)),
        ("tf", solver.tf(c, &tf).map(drop)),
        (
            "pz",
            solver
                .pz(c, &PzConfig::voltage_gain("vcc", "0", "a", "0"))
                .map(drop),
        ),
        (
            "loop_gain",
            solver
                .loop_gain(c, &LoopGainConfig::new("VP", 10.0, 1e3, 3))
                .map(drop),
        ),
        (
            "ac_sensitivity",
            solver
                .ac_sensitivity(c, &AcSensConfig::new(out.clone(), ac.clone()))
                .map(drop),
        ),
        (
            "sensitivity",
            solver.sensitivity(c, &SensConfig::new(out)).map(drop),
        ),
        (
            "dc_sweep",
            solver.dc_sweep(c, "V1", 5.0, 5.001, 2).map(drop),
        ),
        (
            "dc_sweep_nested",
            solver.dc_sweep_nested(c, &inner, &outer).map(drop),
        ),
        (
            "temperature_sweep",
            solver.temperature_sweep(c, 300.15, 300.25, 2).map(drop),
        ),
    ]
}

/// The diode circuit with a second source, `V2`, for the nested sweep,
/// feeding `a` through a 0 V probe, `VP`, for the loop gain.
fn two_source_diode_circuit() -> Circuit {
    let mut c = diode_circuit();
    c.components.push(vsource("V2", "b", "0", 1.0));
    c.components.push(resistor("R2", "b", "p", 10e3));
    c.components.push(vsource("VP", "a", "p", 0.0));
    c
}

/// No convergence aids and a budget of `iterations`.
fn starved(iterations: usize) -> SolverOptions {
    let mut o = SolverOptions::default();
    o.max_iterations = iterations;
    o.gmin_steps.clear();
    o.source_stepping = false;
    o
}

#[test]
fn the_solver_keeps_its_options() {
    let o = starved(3);
    assert_eq!(Solver::new(o.clone()).options(), &o);
    assert_eq!(Solver::default().options(), &SolverOptions::default());
}

/// Every wrapper validates its options before solving.
#[test]
fn every_analysis_rejects_bad_options() {
    let mut o = SolverOptions::default();
    o.reltol = -1.0;
    for (name, result) in every_analysis(&Solver::new(o), &two_source_diode_circuit()) {
        assert!(
            matches!(&result, Err(SimError::InvalidComponent(m)) if m.contains("reltol")),
            "{name}: {result:?}"
        );
    }
}

/// The options reach the bias point of every analysis: one Newton step with
/// no aids can't find the diode from 0 V, and the error says where, the
/// first point of a sweep included. Two steps from 0 V aren't enough either,
/// but from a nodeset at the answer they are, so the nodeset reaches every
/// analysis too.
#[test]
fn every_analysis_runs_with_the_solvers_options() {
    let c = two_source_diode_circuit();
    for (name, result) in every_analysis(&Solver::default(), &c) {
        assert!(result.is_ok(), "{name}: {result:?}");
    }
    for (name, result) in every_analysis(&Solver::new(starved(1)), &c) {
        let want = match name {
            // The nested sweep names the inner source's point.
            "dc_sweep" | "dc_sweep_nested" => SolvePoint::DcSweep {
                source: "V1".into(),
                value: 5.0,
            },
            "temperature_sweep" => SolvePoint::Temperature { kelvin: 300.15 },
            _ => SolvePoint::OperatingPoint,
        };
        match result {
            Err(SimError::ConvergenceFailed { iterations, at, .. }) => {
                assert_eq!(iterations, 1, "{name}");
                assert_eq!(at, want, "{name}");
            }
            other => panic!("{name}: expected ConvergenceFailed, got {other:?}"),
        }
    }

    for (name, result) in every_analysis(&Solver::new(starved(2)), &c) {
        assert!(result.is_err(), "{name} needs more than two steps from 0 V");
    }
    let exact = solve_circuit(&c).unwrap().node_voltages;
    let mut seeded = starved(2);
    seeded.nodeset = exact.clone();
    for (name, result) in every_analysis(&Solver::new(seeded), &c) {
        assert!(result.is_ok(), "{name}: {result:?}");
    }
}

/// The diode circuit with a capacitor on `a`, so the analyses that look at
/// frequency have a pole to find.
fn diode_rc_circuit() -> Circuit {
    let mut c = two_source_diode_circuit();
    c.components.push(CircuitElement::Capacitor {
        id: "C1".into(),
        nodes: ["a".into(), "0".into()],
        capacitance: 1e-9,
    });
    c
}

#[test]
fn default_solver_matches_the_free_frequency_analyses() {
    let c = diode_rc_circuit();
    let solver = Solver::default();

    let pz = PzConfig::voltage_gain("vcc", "0", "a", "0");
    let free = solve_pz(&c, &pz).unwrap();
    assert_eq!(free.poles.len(), 1, "{:?}", free.poles);
    assert_eq!(solver.pz(&c, &pz).unwrap(), free);

    let lg = LoopGainConfig::new("VP", 10.0, 1e6, 5);
    assert_eq!(
        solver.loop_gain(&c, &lg).unwrap(),
        solve_loop_gain(&c, &lg).unwrap()
    );

    let out = TfOutput::voltage("a");
    let sens = AcSensConfig::new(out, sindr::AcConfig::new("V1", 10.0, 1e6, 5));
    let free = solve_ac_sensitivity(&c, &sens).unwrap();
    assert_eq!(free.points.len(), 5);
    assert_eq!(solver.ac_sensitivity(&c, &sens).unwrap(), free);
}

/// A nodeset at the answer lets a starved solver find the same poles, loop
/// gain and AC sensitivities as the default one.
#[test]
fn nodeset_seeds_the_frequency_analyses() {
    let c = diode_rc_circuit();
    let mut seeded = starved(2);
    seeded.nodeset = solve_circuit(&c).unwrap().node_voltages;
    let (starved, seeded) = (Solver::new(starved(2)), Solver::new(seeded));

    let pz = PzConfig::voltage_gain("vcc", "0", "a", "0");
    assert!(starved.pz(&c, &pz).is_err());
    let (got, want) = (seeded.pz(&c, &pz).unwrap(), solve_pz(&c, &pz).unwrap());
    let rel = (got.poles[0] - want.poles[0]).norm() / want.poles[0].norm();
    assert!(rel < 1e-6, "{:?} vs {:?}", got.poles, want.poles);

    let lg = LoopGainConfig::new("VP", 10.0, 1e6, 5);
    assert!(starved.loop_gain(&c, &lg).is_err());
    let got = seeded.loop_gain(&c, &lg).unwrap();
    let want = solve_loop_gain(&c, &lg).unwrap();
    for (g, w) in got.points.iter().zip(&want.points) {
        // No feedback reaches `a` back through VP, so T is zero to rounding.
        assert!(
            (g.gain - w.gain).norm() <= 1e-6 * w.gain.norm() + 1e-12,
            "{g:?} vs {w:?}"
        );
    }

    let sens = AcSensConfig::new(
        TfOutput::voltage("a"),
        sindr::AcConfig::new("V1", 10.0, 1e6, 5),
    );
    assert!(starved.ac_sensitivity(&c, &sens).is_err());
    assert_eq!(
        seeded.ac_sensitivity(&c, &sens).unwrap().points.len(),
        solve_ac_sensitivity(&c, &sens).unwrap().points.len()
    );
}
