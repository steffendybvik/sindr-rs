//! Every analysis beyond the transient and Newton benches, each scaling one
//! parameter (circuit size or sweep length). Names are
//! `<area>/<parameter>/<n>`. CI's bench gate runs these.
//!
//!     cargo bench -p sindr --bench analyses

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::noise::{solve_noise, NoiseConfig};
use sindr::{
    check_ratings, dc_sweep, solve_circuit, solve_sensitivity, solve_tf, temperature_sweep,
    BjtKind, Circuit, CircuitElement, Part, Ratings, SensConfig, TfConfig, TfOutput,
};

fn r(id: String, a: String, b: String, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id,
        nodes: [a, b],
        resistance,
    }
}

fn source(id: &str, node: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [node.into(), "0".into()],
        voltage,
        waveform: None,
    }
}

/// `n` RC stages from `V1`: linear, so AC, `.tf` and `.sens` skip Newton.
fn rc_ladder(n: usize) -> Circuit {
    let mut components = vec![source("V1", "n0", 1.0)];
    for i in 1..=n {
        components.push(r(
            format!("R{i}"),
            format!("n{}", i - 1),
            format!("n{i}"),
            100.0,
        ));
        components.push(CircuitElement::Capacitor {
            id: format!("C{i}"),
            nodes: [format!("n{i}"), "0".into()],
            capacitance: 1e-9,
        });
    }
    Circuit::new("0", components)
}

/// `n` diode–resistor stages from `V1`, every part rated for power.
fn diode_chain(n: usize) -> Circuit {
    let mut components = vec![source("V1", "n0", 5.0)];
    for i in 1..=n {
        components.push(r(
            format!("R{i}"),
            format!("n{}", i - 1),
            format!("n{i}"),
            1e3,
        ));
        components.push(r(format!("RP{i}"), "n0".into(), format!("n{i}"), 10e3));
        components.push(CircuitElement::Diode {
            id: format!("D{i}"),
            nodes: [format!("n{i}"), "0".into()],
            temperature: 300.15,
            model: None,
        });
    }
    let mut c = Circuit::new("0", components);
    let rated = Part {
        ratings: Ratings {
            power: Some(0.25),
            reverse_voltage: Some(50.0),
            forward_current: Some(0.1),
            ..Ratings::default()
        },
        ..Part::default()
    };
    for e in &c.components.clone() {
        c.parts.insert(e.id().to_string(), rated.clone());
    }
    c
}

/// A common-emitter NPN stage: Newton in the bias point, and resistor,
/// shot and flicker noise sources.
fn bjt_stage() -> Circuit {
    Circuit::new(
        "0",
        vec![
            source("VCC", "vcc", 10.0),
            source("VIN", "in", 0.0),
            r("RS".into(), "in".into(), "b".into(), 1e3),
            r("RB".into(), "vcc".into(), "b".into(), 470e3),
            r("RC".into(), "vcc".into(), "c".into(), 2.2e3),
            r("RE".into(), "e".into(), "0".into(), 220.0),
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "e".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            },
        ],
    )
}

fn ac_config(points: usize) -> AcConfig {
    AcConfig {
        f_start: 10.0,
        f_stop: 100e6,
        num_points: points,
        spacing: FrequencySpacing::Logarithmic,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
        ac_phase_deg: 0.0,
    }
}

fn v_out(node: String) -> TfOutput {
    TfOutput::Voltage {
        node,
        reference: None,
    }
}

fn bench(c: &mut Criterion) {
    let mut group = c.benchmark_group("ac/points");
    let ladder = rc_ladder(100);
    for points in [10, 100, 1_000] {
        let config = ac_config(points);
        group.bench_with_input(BenchmarkId::from_parameter(points), &points, |b, _| {
            b.iter(|| solve_ac(&ladder, &config).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("ac/stages");
    for stages in [10, 100, 1_000] {
        let circuit = rc_ladder(stages);
        let config = ac_config(100);
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| solve_ac(&circuit, &config).unwrap())
        });
    }
    group.finish();

    // Each point re-solves Newton, warm-started from the last.
    let mut group = c.benchmark_group("dc_sweep/points");
    let chain = diode_chain(10);
    for points in [10, 100, 1_000] {
        group.bench_with_input(BenchmarkId::from_parameter(points), &points, |b, &n| {
            b.iter(|| dc_sweep(&chain, "V1", 0.0, 5.0, n).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("temperature_sweep/points");
    let stage = bjt_stage();
    for points in [10, 100] {
        group.bench_with_input(BenchmarkId::from_parameter(points), &points, |b, &n| {
            b.iter(|| temperature_sweep(&stage, 233.15, 398.15, n).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("tf/stages");
    for stages in [10, 100, 1_000] {
        let circuit = rc_ladder(stages);
        let config = TfConfig {
            output: v_out(format!("n{stages}")),
            input_source: "V1".into(),
        };
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| solve_tf(&circuit, &config).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("sensitivity/stages");
    for stages in [10, 100, 1_000] {
        let circuit = diode_chain(stages);
        let config = SensConfig {
            output: v_out(format!("n{stages}")),
        };
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| solve_sensitivity(&circuit, &config).unwrap())
        });
    }
    group.finish();

    // A transistor stage: the RC ladder in the noise bench skips Newton and
    // transistor noise sources.
    let mut group = c.benchmark_group("noise_bjt/points");
    for points in [10, 100, 1_000] {
        let config = NoiseConfig {
            output: "c".into(),
            reference: None,
            input_source: "VIN".into(),
            f_start: 1.0,
            f_stop: 10e6,
            num_points: points,
            spacing: FrequencySpacing::Logarithmic,
        };
        group.bench_with_input(BenchmarkId::from_parameter(points), &points, |b, _| {
            b.iter(|| solve_noise(&stage, &config).unwrap())
        });
    }
    group.finish();

    // Checking alone, on a solved operating point.
    let mut group = c.benchmark_group("ratings/stages");
    for stages in [100, 1_000] {
        let circuit = diode_chain(stages);
        let op = solve_circuit(&circuit).unwrap();
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| check_ratings(&circuit, &op).unwrap())
        });
    }
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
