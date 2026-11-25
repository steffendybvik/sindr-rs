//! The SPICE front end on large decks: parsing, flattening a nested
//! `.subckt` hierarchy, and writing a circuit back out. Names are
//! `<area>/<parameter>/<n>`. CI's bench gate runs these. Sizes stop at
//! 1 000 stages: at 10 000, parsing and export are allocation-bound and
//! swung 40-50% between runs on CI's shared runners, more than the gate
//! allows, while 100 against 1 000 still shows a change in scaling.
//!
//!     cargo bench -p sindr --features spice --bench spice

use std::fmt::Write as _;

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use sindr::spice::{parse_str, to_netlist};

/// A flat deck: `n` RC stages and a diode on every tenth node.
fn flat_deck(n: usize) -> String {
    let mut d = String::from("flat\nV1 n0 0 5\n");
    for i in 1..=n {
        let p = i - 1;
        writeln!(d, "R{i} n{p} n{i} 1k\nC{i} n{i} 0 1n").unwrap();
        if i % 10 == 0 {
            writeln!(d, "D{i} n{i} 0 dd").unwrap();
        }
    }
    d + ".model dd D(IS=1e-14 N=1.8 RS=0.5)\n.end\n"
}

/// `n` instances of a stage that itself instances a divider: two levels
/// of `.subckt`, with a `.param` per stage.
fn nested_deck(n: usize) -> String {
    let mut d = String::from(
        "nested\n\
         .subckt div top bot\nR1 top mid 1k\nR2 mid bot 1k\n.ends\n\
         .subckt stage in out rl=1k\nX1 in out div\nRL out 0 {rl}\n.ends\n\
         V1 n0 0 5\n",
    );
    for i in 1..=n {
        writeln!(d, "X{i} n{} n{i} stage rl={}", i - 1, 1000 + i).unwrap();
    }
    d + ".end\n"
}

fn bench(c: &mut Criterion) {
    let mut group = c.benchmark_group("spice_parse/stages");
    for stages in [100, 1_000] {
        let deck = flat_deck(stages);
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| parse_str(&deck).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("spice_parse_nested/instances");
    for instances in [100, 1_000] {
        let deck = nested_deck(instances);
        group.bench_with_input(
            BenchmarkId::from_parameter(instances),
            &instances,
            |b, _| b.iter(|| parse_str(&deck).unwrap()),
        );
    }
    group.finish();

    let mut group = c.benchmark_group("flatten/instances");
    for instances in [100, 1_000] {
        let circuit = parse_str(&nested_deck(instances)).unwrap().circuit;
        group.bench_with_input(
            BenchmarkId::from_parameter(instances),
            &instances,
            |b, _| b.iter(|| circuit.flatten().unwrap()),
        );
    }
    group.finish();

    let mut group = c.benchmark_group("spice_export/stages");
    for stages in [100, 1_000] {
        let circuit = parse_str(&flat_deck(stages)).unwrap().circuit;
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| to_netlist(&circuit, "bench").unwrap())
        });
    }
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
