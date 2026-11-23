//! Monte Carlo and corner analysis against closed forms: each run's
//! measurement from its own drawn values, the spread of a known
//! distribution, and the extremes of a divider at its corners.

use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::element::{Capacitor, CurrentSource, Resistor, VoltageSource};
use sindr::measure::{Measure, Measurement, Signal, Stat};
use sindr::variation::{
    corners, monte_carlo, Corner, Corners, Distribution, MonteCarloConfig, Parameter, Tolerance,
};
use sindr::{solve_op, Block, Circuit, CircuitElement, Instance, SimError};

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn v(id: &str, p: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), "0".into()],
        voltage: volts,
        waveform: None,
    })
}

/// 10 V across r1 (10 k) over r2 (10 k).
fn divider() -> Circuit {
    Circuit::new(
        "0",
        vec![
            v("v1", "in", 10.0),
            r("r1", "in", "out", 10e3),
            r("r2", "out", "0", 10e3),
        ],
    )
}

fn vout() -> Measurement {
    Measurement::new("vout", Measure::stat(Signal::v("out"), Stat::Avg))
}

fn close(got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1e-30),
        "got {got}, want {want}"
    );
}

/// Every run's output is the divider of the values it drew, and those
/// stay in the uniform band.
#[test]
fn each_run_solves_its_own_draw() {
    let tolerances = vec![
        Tolerance::uniform("r1", Parameter::Resistance, 0.05),
        Tolerance::uniform("r2", Parameter::Resistance, 0.05),
        Tolerance::uniform("v1", Parameter::Voltage, 0.02),
    ];
    let config = MonteCarloConfig::new(tolerances, 300).with_seed(11);
    let result = monte_carlo(&divider(), &config, &[vout()], solve_op).unwrap();
    assert_eq!(result.runs.len(), 300);
    for run in &result.runs {
        let [r1, r2, vs] = run.values[..] else {
            panic!()
        };
        assert!((9.5e3..=10.5e3).contains(&r1) && (9.5e3..=10.5e3).contains(&r2));
        assert!((9.8..=10.2).contains(&vs));
        close(
            *run.measurements[0].as_ref().unwrap(),
            vs * r2 / (r1 + r2),
            1e-9,
        );
    }
    let stats = result.stats("vout").unwrap();
    assert_eq!((stats.measured, stats.failed), (300, 0));
    close(stats.mean, 5.0, 0.005);
    let lowest = 9.8 * 9.5e3 / (9.5e3 + 10.5e3);
    let highest = 10.2 * 10.5e3 / (10.5e3 + 9.5e3);
    assert!(stats.min >= lowest && stats.max <= highest);
}

/// A 3 % part at 3σ in V = I·R spreads V by 1 %.
#[test]
fn gaussian_spread_matches_its_sigma() {
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::CurrentSource(CurrentSource {
                id: "i1".into(),
                nodes: ["0".into(), "a".into()],
                current: 1e-3,
                waveform: None,
            }),
            r("r1", "a", "0", 1e3),
        ],
    );
    let config = MonteCarloConfig::new(
        vec![Tolerance::new("r1", Parameter::Resistance, 0.03)],
        4000,
    )
    .with_seed(3);
    let va = Measurement::new("va", Measure::stat(Signal::v("a"), Stat::Avg));
    let result = monte_carlo(&circuit, &config, &[va], solve_op).unwrap();
    let stats = result.stats("va").unwrap();
    close(stats.mean, 1.0, 1e-3);
    close(stats.std_dev, 0.01, 0.05);
}

#[test]
fn a_seed_repeats_and_another_differs() {
    let tolerances = Tolerance::on_every(&divider(), Parameter::Resistance, 0.05);
    let run = |seed| {
        let config = MonteCarloConfig::new(tolerances.clone(), 50).with_seed(seed);
        monte_carlo(&divider(), &config, &[vout()], solve_op).unwrap()
    };
    assert_eq!(run(1), run(1));
    assert_ne!(run(1).runs, run(2).runs);
}

/// The divider's extremes sit at two of its four corners.
#[test]
fn every_corner_bounds_the_divider() {
    let tolerances = Tolerance::on_every(&divider(), Parameter::Resistance, 0.05);
    let result = corners(&divider(), &tolerances, &Corners::All, &[vout()], |c| {
        solve_op(c)
    })
    .unwrap();
    assert_eq!(result.runs.len(), 4);
    let stats = result.stats("vout").unwrap();
    close(stats.min, 10.0 * 0.95 / 2.0, 1e-9);
    close(stats.max, 10.0 * 1.05 / 2.0, 1e-9);
    // Each corner's values are the band's ends.
    for run in &result.runs {
        assert!(run.values.iter().all(|x| *x == 9.5e3 || *x == 10.5e3));
    }
}

#[test]
fn listed_corners_run_as_given() {
    let tolerances = Tolerance::on_every(&divider(), Parameter::Resistance, 0.1);
    let list = Corners::List(vec![
        vec![Corner::Nominal, Corner::Nominal],
        vec![Corner::High, Corner::Low],
    ]);
    let result = corners(&divider(), &tolerances, &list, &[vout()], solve_op).unwrap();
    assert_eq!(result.runs[0].values, [10e3, 10e3]);
    close(*result.runs[0].measurements[0].as_ref().unwrap(), 5.0, 1e-9);
    close(
        *result.runs[1].measurements[0].as_ref().unwrap(),
        10.0 * 9e3 / 20e3,
        1e-9,
    );
}

/// Any analysis: an RC low-pass's −3 dB point follows each run's C.
#[test]
fn wraps_an_ac_analysis() {
    let circuit = Circuit::new(
        "0",
        vec![
            v("vin", "in", 0.0),
            r("r1", "in", "out", 1e3),
            CircuitElement::Capacitor(Capacitor {
                id: "c1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 100e-9,
            }),
        ],
    );
    let config = MonteCarloConfig::new(
        vec![Tolerance::uniform("c1", Parameter::Capacitance, 0.1)],
        20,
    );
    let bw = Measurement::new("f3db", Measure::bandwidth(Signal::v("out")));
    let ac = AcConfig::new("vin", 10.0, 1e6, 801);
    let result = monte_carlo(&circuit, &config, &[bw], |c| solve_ac(c, &ac)).unwrap();
    for run in &result.runs {
        let f = *run.measurements[0].as_ref().unwrap();
        let want = 1.0 / (2.0 * std::f64::consts::PI * 1e3 * run.values[0]);
        close(f, want, 0.01);
    }
}

/// A run whose analysis fails is recorded, and the rest still count.
#[test]
fn failed_runs_are_recorded_not_fatal() {
    let tolerances = vec![Tolerance::uniform("r1", Parameter::Resistance, 0.05)];
    let config = MonteCarloConfig::new(tolerances, 100).with_seed(5);
    let picky = |c: &Circuit| {
        let r1 = c.components.iter().find(|e| e.id() == "r1").unwrap();
        match r1 {
            CircuitElement::Resistor(Resistor { resistance, .. }) if *resistance > 10e3 => {
                Err(SimError::InvalidComponent("too big".to_string()))
            }
            _ => solve_op(c),
        }
    };
    let result = monte_carlo(&divider(), &config, &[vout()], picky).unwrap();
    let stats = result.stats("vout").unwrap();
    assert_eq!(stats.measured + stats.failed, 100);
    assert!(stats.failed > 20 && stats.measured > 20, "{stats:?}");
    let failed = result.runs.iter().find(|r| r.values[0] > 10e3).unwrap();
    assert!(failed.measurements[0]
        .as_ref()
        .unwrap_err()
        .contains("too big"));
    // Yield: the fraction measured inside a window.
    let all = result.yield_within("vout", 0.0, 10.0).unwrap();
    close(all, stats.measured as f64 / 100.0, 1e-12);
    assert_eq!(result.yield_within("nope", 0.0, 1.0), None);
}

/// A tolerance inside an instance names its flattened id.
#[test]
fn on_every_reaches_into_the_hierarchy() {
    let block = Block {
        ports: vec!["a".into(), "b".into()],
        components: vec![r("r1", "a", "b", 10e3)],
        ..Default::default()
    };
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![v("v1", "in", 10.0), r("r2", "out", "0", 10e3)],
        blocks: [("res".to_string(), block)].into_iter().collect(),
        instances: vec![Instance::new("x1", "res", ["in", "out"])],
        ..Default::default()
    };
    let tolerances = Tolerance::on_every(&circuit, Parameter::Resistance, 0.05);
    let ids: Vec<_> = tolerances.iter().map(|t| t.component.as_str()).collect();
    assert_eq!(ids, ["r2", "x1.r1"]);
    let result = corners(&circuit, &tolerances, &Corners::All, &[vout()], |c| {
        solve_op(c)
    })
    .unwrap();
    close(result.stats("vout").unwrap().max, 10.0 * 1.05 / 2.0, 1e-9);
}

#[test]
fn bad_configurations_are_errors() {
    let measure = [vout()];
    let mc = |tolerances: Vec<Tolerance>, runs| {
        monte_carlo(
            &divider(),
            &MonteCarloConfig::new(tolerances, runs),
            &measure,
            solve_op,
        )
        .unwrap_err()
        .to_string()
    };
    let res = |id: &str, rel| Tolerance::new(id, Parameter::Resistance, rel);
    assert!(mc(vec![res("r9", 0.1)], 10).contains("isn't in the circuit"));
    assert!(
        mc(vec![Tolerance::new("v1", Parameter::Resistance, 0.1)], 10).contains("no Resistance")
    );
    assert!(mc(vec![res("r1", 1.0)], 10).contains("[0, 1)"));
    assert!(mc(vec![res("r1", -0.1)], 10).contains("[0, 1)"));
    assert!(mc(
        vec![res("r1", 0.1).with_distribution(Distribution::Gaussian { sigmas: 0.0 })],
        10
    )
    .contains("sigmas"));
    assert!(mc(vec![res("r1", 0.1)], 0).contains("at least one run"));

    let many: Vec<_> = (0..17).map(|_| res("r1", 0.1)).collect();
    let err = corners(&divider(), &many, &Corners::All, &measure, solve_op).unwrap_err();
    assert!(err.to_string().contains("2^17"));
    let err = corners(
        &divider(),
        &[res("r1", 0.1)],
        &Corners::List(vec![vec![Corner::Low, Corner::High]]),
        &measure,
        solve_op,
    )
    .unwrap_err();
    assert!(err.to_string().contains("names 2 tolerances, not 1"));
}

/// A measurement no run can take gets NaN statistics, all failed.
#[test]
fn an_unmeasurable_measurement_reports_nan() {
    let missing = Measurement::new("vx", Measure::stat(Signal::v("nowhere"), Stat::Avg));
    let config = MonteCarloConfig::new(
        Tolerance::on_every(&divider(), Parameter::Resistance, 0.01),
        3,
    );
    let result = monte_carlo(&divider(), &config, &[missing], solve_op).unwrap();
    let stats = result.stats("vx").unwrap();
    assert!(stats.mean.is_nan() && stats.std_dev.is_nan() && stats.min.is_nan());
    assert_eq!((stats.measured, stats.failed), (0, 3));
}

#[test]
fn one_run_has_no_spread() {
    let config = MonteCarloConfig::new(
        Tolerance::on_every(&divider(), Parameter::Resistance, 0.01),
        1,
    );
    let result = monte_carlo(&divider(), &config, &[vout()], solve_op).unwrap();
    assert_eq!(result.stats("vout").unwrap().std_dev, 0.0);
}

#[cfg(feature = "serde")]
#[test]
fn configuration_and_result_serialise() {
    let config = MonteCarloConfig::new(
        vec![Tolerance::uniform("r1", Parameter::Resistance, 0.05)],
        4,
    )
    .with_seed(9);
    let json = serde_json::to_value(&config).unwrap();
    assert_eq!(json["tolerances"][0]["parameter"], "resistance");
    assert_eq!(json["tolerances"][0]["distribution"], "uniform");
    let back: MonteCarloConfig = serde_json::from_value(json).unwrap();
    assert_eq!(back, config);
    let result = monte_carlo(&divider(), &config, &[vout()], solve_op).unwrap();
    let json = serde_json::to_value(&result).unwrap();
    assert!(json["runs"][0]["measurements"][0]["Ok"].is_number());
}
