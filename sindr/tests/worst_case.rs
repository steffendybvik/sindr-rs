//! Worst case over tolerances: specs and ratings judged on every run of a
//! Monte Carlo or corner analysis, against the divider's closed form at
//! the same corners.

use sindr::element::{Resistor, VoltageSource};
use sindr::measure::{Measure, Measurement, Signal, Spec, Stat};
use sindr::variation::{
    corners, monte_carlo, ratings_over, Corner, Corners, MonteCarloConfig, Parameter, Sampling,
    Tolerance,
};
use sindr::{
    check_ratings, solve_op, Circuit, CircuitElement, Part, RatingKind, RatingStatus, Ratings,
    SimError,
};

/// 10 V across r1 over r2, both 10 kΩ; r2 rated for 2.6 mW (it takes
/// 2.5 mW at nominal).
fn divider() -> Circuit {
    let r = |id: &str, a: &str, b: &str| {
        CircuitElement::Resistor(Resistor {
            id: id.into(),
            nodes: [a.into(), b.into()],
            resistance: 10e3,
        })
    };
    let mut c = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "v1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            }),
            r("r1", "in", "out"),
            r("r2", "out", "0"),
        ],
    );
    c.parts.insert(
        "r2".into(),
        Part {
            ratings: Ratings {
                power: Some(2.6e-3),
                ..Ratings::default()
            },
            ..Part::default()
        },
    );
    c
}

fn tolerances() -> Vec<Tolerance> {
    vec![
        Tolerance::new("r1", Parameter::Resistance, 0.05),
        Tolerance::new("v1", Parameter::Voltage, 0.05),
    ]
}

fn vout_spec() -> Spec {
    Spec::new(Measurement::new(
        "vout",
        Measure::stat(Signal::v("out"), Stat::Avg),
    ))
    .between(4.8, 5.2)
}

fn close(got: f64, want: f64) {
    assert!(
        (got - want).abs() <= 1e-9 * want.abs().max(1e-12),
        "got {got}, want {want}"
    );
}

/// The four corners by hand: (r1, v1) and r2's power and vout there.
fn by_hand() -> Vec<(f64, f64, f64, f64)> {
    [(0.95, 0.95), (1.05, 0.95), (0.95, 1.05), (1.05, 1.05)]
        .iter()
        .map(|&(a, b)| {
            let (r1, v) = (10e3 * a, 10.0 * b);
            let i = v / (r1 + 10e3);
            (r1, v, i * i * 10e3, i * 10e3)
        })
        .collect()
}

#[test]
fn specs_over_corners_name_the_worst_run() {
    let spec = vout_spec();
    let result = corners(
        &divider(),
        &tolerances(),
        &Corners::All,
        std::slice::from_ref(&spec.measurement),
        solve_op,
    )
    .unwrap();
    let [spread] = &result.check_specs(&[spec])[..] else {
        panic!()
    };
    // vout = v·10k/(r1 + 10k): outside 4.8–5.2 only at (r1 high, v low)
    // and (r1 low, v high).
    let hand = by_hand();
    let fails = hand.iter().filter(|h| !(4.8..=5.2).contains(&h.3)).count();
    assert_eq!(
        (spread.passed, spread.failed, spread.unmeasured),
        (4 - fails, fails, 0)
    );
    close(spread.yield_fraction, (4 - fails) as f64 / 4.0);
    let worst = spread.worst.as_ref().unwrap();
    let margin = |v: f64| (v - 4.8).min(5.2 - v);
    let want = hand
        .iter()
        .map(|h| margin(h.3))
        .fold(f64::INFINITY, f64::min);
    close(worst.margin.unwrap(), want);
    assert!(worst.margin.unwrap() < 0.0);
    // Its condition is that run's values.
    assert_eq!(worst.values, result.runs[worst.run].values);
    close(
        worst.value,
        *result.runs[worst.run].measurements[0].as_ref().unwrap(),
    );
}

#[test]
fn a_spec_never_measured_is_all_unmeasured() {
    let result = monte_carlo(
        &divider(),
        &MonteCarloConfig::new(tolerances(), 5),
        &[],
        solve_op,
    )
    .unwrap();
    let [spread] = &result.check_specs(&[vout_spec()])[..] else {
        panic!()
    };
    assert_eq!((spread.passed, spread.unmeasured), (0, 5));
    assert_eq!(spread.yield_fraction, 0.0);
    assert!(spread.worst.is_none());
}

/// r2 takes (v/(r1 + r2))²·r2; it's over 2.6 mW at the corners where r1
/// is low and v high.
#[test]
fn ratings_over_corners_find_the_worst_part_and_its_condition() {
    let sampling = Sampling::Corners {
        tolerances: tolerances(),
        which: Corners::All,
    };
    let spread = ratings_over(&divider(), &sampling, solve_op).unwrap();
    assert_eq!(spread.runs, 4);
    assert!(spread.failed_runs.is_empty());
    let r2 = spread
        .checks
        .iter()
        .find(|c| c.part == "r2" && c.rating == RatingKind::Power)
        .unwrap();
    let hand = by_hand();
    let worst = hand.iter().map(|h| h.2).fold(0.0, f64::max);
    close(r2.worst.unwrap(), worst);
    close(
        r2.margin_percent.unwrap(),
        100.0 * (2.6e-3 - worst) / 2.6e-3,
    );
    let over = hand.iter().filter(|h| h.2 > 2.6e-3).count();
    assert!(over > 0);
    assert_eq!(r2.exceeded_runs, over);
    assert_eq!(r2.status, RatingStatus::Exceeded);
    // The worst run is r1 low and v high.
    assert_eq!(r2.values, [9.5e3, 10.5]);
    assert_eq!(r2.time, None);
    assert!(!spread.is_safe());
    assert_eq!(spread.exceeded().len(), 1);
    assert_eq!(spread.tightest().unwrap().part, "r2");
    // r1 has no ratings: unverified, never a pass.
    assert!(spread
        .checks
        .iter()
        .filter(|c| c.part == "r1")
        .all(|c| c.status == RatingStatus::Unverified));
}

/// The same run twice merges into one entry per part and rating, in the
/// order and with the values one check of that run gives.
#[test]
fn repeated_runs_merge_into_one_check_each() {
    let mut c = divider();
    let rated = Part {
        ratings: Ratings {
            power: Some(2e-3),
            voltage: Some(50.0),
            current: Some(1e-3),
            ..Ratings::default()
        },
        ..Part::default()
    };
    for id in ["v1", "r1", "r2"] {
        c.parts.insert(id.into(), rated.clone());
    }
    let nominal = vec![Corner::Nominal, Corner::Nominal];
    let sampling = Sampling::Corners {
        tolerances: tolerances(),
        which: Corners::List(vec![nominal.clone(), nominal]),
    };
    let spread = ratings_over(&c, &sampling, solve_op).unwrap();
    let once = check_ratings(&c, &solve_op(&c).unwrap()).unwrap();
    assert!(once.checks.len() > 3);
    let merged: Vec<_> = spread
        .checks
        .iter()
        .map(|w| {
            (
                w.part.as_str(),
                w.rating,
                w.worst,
                w.status,
                w.exceeded_runs,
                w.run,
            )
        })
        .collect();
    let expected: Vec<_> = once
        .checks
        .iter()
        .map(|r| {
            let exceeded = usize::from(r.status == RatingStatus::Exceeded) * 2;
            (
                r.part.as_str(),
                r.rating,
                r.worst,
                r.status,
                exceeded,
                r.worst.map(|_| 0),
            )
        })
        .collect();
    assert_eq!(merged, expected);
}

#[test]
fn ratings_within_at_every_corner_stay_within() {
    let mut c = divider();
    c.parts.get_mut("r2").unwrap().ratings.power = Some(3e-3);
    let sampling = Sampling::Corners {
        tolerances: tolerances(),
        which: Corners::List(vec![vec![Corner::Low, Corner::High]]),
    };
    let spread = ratings_over(&c, &sampling, solve_op).unwrap();
    let r2 = spread.checks.iter().find(|c| c.part == "r2").unwrap();
    assert_eq!(
        (r2.status, r2.exceeded_runs, r2.run),
        (RatingStatus::Within, 0, Some(0))
    );
    close(r2.worst.unwrap(), by_hand()[2].2);
}

#[test]
fn monte_carlo_ratings_record_failed_runs() {
    let sampling = Sampling::MonteCarlo(MonteCarloConfig::new(tolerances(), 40).with_seed(4));
    let picky = |c: &Circuit| match &c.components[1] {
        CircuitElement::Resistor(Resistor { resistance, .. }) if *resistance > 10e3 => {
            Err(SimError::InvalidComponent {
                id: "R2".into(),
                reason: "too big".into(),
            })
        }
        _ => solve_op(c),
    };
    let spread = ratings_over(&divider(), &sampling, picky).unwrap();
    assert_eq!(spread.runs, 40);
    assert!(!spread.failed_runs.is_empty());
    assert!(spread
        .failed_runs
        .iter()
        .all(|(_, why)| why.contains("too big")));
    assert!(!spread.is_safe());
    // The worst of the runs that solved is still found.
    let r2 = spread.checks.iter().find(|c| c.part == "r2").unwrap();
    assert!(r2.worst.unwrap() > 2.5e-3);
}

#[test]
fn ratings_over_a_bad_sampling_is_an_error() {
    let sampling = Sampling::MonteCarlo(MonteCarloConfig::new(tolerances(), 0));
    assert!(ratings_over(&divider(), &sampling, solve_op).is_err());
    let sampling = Sampling::Corners {
        tolerances: vec![Tolerance::new("nope", Parameter::Resistance, 0.1)],
        which: Corners::All,
    };
    assert!(ratings_over(&divider(), &sampling, solve_op).is_err());
}

#[cfg(feature = "serde")]
#[test]
fn sampling_serialises() {
    let sampling = Sampling::Corners {
        tolerances: tolerances(),
        which: Corners::List(vec![vec![Corner::Low, Corner::Nominal]]),
    };
    let json = serde_json::to_value(&sampling).unwrap();
    assert_eq!(json["Corners"]["which"]["list"][0][1], "nominal");
    let back: Sampling = serde_json::from_value(json).unwrap();
    assert_eq!(back, sampling);
}
