//! Every analysis beyond the transient and Newton benches, each scaling one
//! parameter (circuit size or sweep length). Names are
//! `<area>/<parameter>/<n>`. CI's bench gate runs these.
//!
//!     cargo bench -p sindr --bench analyses

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::element::{Bjt, Capacitor, Diode, Resistor, VoltageSource};
use sindr::import::{
    Bench, BenchSource, Component, ComponentKind, ImportOptions, Net, Netlist, UnknownKinds,
};
use sindr::noise::{solve_noise, NoiseConfig};
use sindr::variation::{ratings_over, MonteCarloConfig, Parameter, Sampling, Tolerance};
use sindr::{
    check_ratings, dc_sweep, solve_circuit, solve_sensitivity, solve_tf, temperature_sweep,
    BjtKind, Block, Circuit, CircuitElement, Instance, Part, Ratings, SensConfig, TfConfig,
    TfOutput,
};
use sindr::{DcSweepAxis, TempSweepConfig};

fn r(id: String, a: String, b: String, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id,
        nodes: [a.into(), b.into()],
        resistance,
    })
}

fn source(id: &str, node: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [node.into(), "0".into()],
        voltage,
        waveform: None,
    })
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
        components.push(CircuitElement::Capacitor(Capacitor {
            id: format!("C{i}"),
            nodes: [format!("n{i}").into(), "0".into()],
            capacitance: 1e-9,
        }));
    }
    Circuit::new("0", components)
}

/// [`rc_ladder`] with each RC stage an instance of one block, so every
/// analysis flattens it first.
fn rc_ladder_blocks(n: usize) -> Circuit {
    let stage = Block {
        ports: vec!["a".into(), "b".into()],
        components: vec![
            r("R".into(), "a".into(), "b".into(), 100.0),
            CircuitElement::Capacitor(Capacitor {
                id: "C".into(),
                nodes: ["b".into(), "0".into()],
                capacitance: 1e-9,
            }),
        ],
        ..Default::default()
    };
    let mut c = Circuit::new("0", vec![source("V1", "n0", 1.0)]);
    c.blocks.insert("stage".into(), stage);
    for i in 1..=n {
        c.instances.push(Instance::new(
            format!("X{i}"),
            "stage",
            [format!("n{}", i - 1), format!("n{i}")],
        ));
    }
    c
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
        components.push(CircuitElement::Diode(Diode {
            id: format!("D{i}"),
            nodes: [format!("n{i}").into(), "0".into()],
            temperature: 300.15,
            model: None,
        }));
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
            CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "e".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }),
        ],
    )
}

fn ac_config(points: usize) -> AcConfig {
    AcConfig::new("V1", 10.0, 100e6, points)
}

fn v_out(node: String) -> TfOutput {
    TfOutput::Voltage {
        node: node.into(),
        reference: None,
    }
}

/// A board as an editor sends it: `n` resistor dividers off a 5 V rail,
/// `n / 4` two-pin connectors and `n / 4` eight-pin ICs sindr doesn't
/// model, each IC on the rail, ground and six divider midpoints.
fn board(n: usize) -> Netlist {
    let mut components = Vec::new();
    let mut vcc = Vec::new();
    let mut gnd = Vec::new();
    let mut mids: Vec<Vec<(String, String)>> = vec![Vec::new(); n];
    for (i, mid) in mids.iter_mut().enumerate() {
        let (top, bottom) = (format!("RT{i}"), format!("RB{i}"));
        components
            .push(Component::new(&top, ComponentKind::from_name("resistor")).with_value("10k"));
        components
            .push(Component::new(&bottom, ComponentKind::from_name("resistor")).with_value("10k"));
        vcc.push((top.clone(), "1".to_string()));
        mid.push((top, "2".to_string()));
        mid.push((bottom.clone(), "1".to_string()));
        gnd.push((bottom, "2".to_string()));
    }
    for k in 0..n / 4 {
        let j = format!("J{k}");
        components.push(Component::new(&j, ComponentKind::from_name("connector")));
        mids[4 * k].push((j.clone(), "1".to_string()));
        gnd.push((j, "2".to_string()));
        let u = format!("U{k}");
        components.push(Component::new(&u, ComponentKind::from_name("ic")).with_value("MCU"));
        vcc.push((u.clone(), "8".to_string()));
        gnd.push((u.clone(), "1".to_string()));
        for p in 0..6 {
            mids[(4 * k + p) % n].push((u.clone(), (p + 2).to_string()));
        }
    }
    let mut nets = vec![Net::new("vcc", vcc), Net::new("gnd", gnd)];
    nets.extend(
        mids.into_iter()
            .enumerate()
            .map(|(i, pins)| Net::new(format!("m{i}"), pins)),
    );
    Netlist { components, nets }
}

fn bench(c: &mut Criterion) {
    let mut group = c.benchmark_group("import/parts");
    let bench = Bench::new("gnd").with_source(BenchSource::voltage("vs", "vcc", 5.0));
    let options = ImportOptions::new().with_unknown_kinds(UnknownKinds::BlackBox);
    for n in [250, 1_000, 4_000] {
        let netlist = board(n);
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| netlist.to_circuit_with(&bench, &options).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("ac/points");
    let ladder = rc_ladder(100);
    for points in [10, 100, 1_000] {
        let config = ac_config(points);
        group.bench_with_input(BenchmarkId::from_parameter(points), &points, |b, _| {
            b.iter(|| solve_ac(&ladder, &config).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("ac_hierarchical/points");
    let ladder = rc_ladder_blocks(400);
    for points in [100, 1_000] {
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
            b.iter(|| dc_sweep(&chain, &DcSweepAxis::new("V1", 0.0, 5.0, n)).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("temperature_sweep/points");
    let stage = bjt_stage();
    for points in [10, 100] {
        group.bench_with_input(BenchmarkId::from_parameter(points), &points, |b, &n| {
            b.iter(|| temperature_sweep(&stage, &TempSweepConfig::new(233.15, 398.15, n)).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("tf/stages");
    for stages in [10, 100, 1_000] {
        let circuit = rc_ladder(stages);
        let config = TfConfig::new(v_out(format!("n{stages}")), "V1");
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| solve_tf(&circuit, &config).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("sensitivity/stages");
    for stages in [10, 100, 1_000] {
        let circuit = diode_chain(stages);
        let config = SensConfig::new(v_out(format!("n{stages}")));
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| solve_sensitivity(&circuit, &config).unwrap())
        });
    }
    group.finish();

    // Sparse from 100 stages: each point solves the system and its adjoint.
    let mut group = c.benchmark_group("noise/stages");
    for stages in [10, 100, 1_000] {
        let circuit = rc_ladder(stages);
        let config = NoiseConfig::new(format!("n{stages}"), "V1", 10.0, 100e6, 100);
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| solve_noise(&circuit, &config).unwrap())
        });
    }
    group.finish();

    // A transistor stage: the RC ladder in the noise bench skips Newton and
    // transistor noise sources.
    let mut group = c.benchmark_group("noise_bjt/points");
    for points in [10, 100, 1_000] {
        let config = NoiseConfig::new("c", "VIN", 1.0, 10e6, points);
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

    // Every part checked on each of 20 Monte Carlo runs, and the checks
    // merged into each one's worst.
    let mut group = c.benchmark_group("ratings_over/stages");
    group.sample_size(10);
    for stages in [100, 1_000] {
        let circuit = diode_chain(stages);
        let sampling = Sampling::MonteCarlo(MonteCarloConfig::new(
            vec![Tolerance::new("R1", Parameter::Resistance, 0.05)],
            20,
        ));
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| ratings_over(&circuit, &sampling, solve_circuit).unwrap())
        });
    }
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
