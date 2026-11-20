//! Transient cost: an RC ladder at 10 to 1 000 stages over a fixed span,
//! and a 50 Hz half-wave rectifier with a filter cap over five cycles
//! (diode switching each cycle, so step control and Newton both work).
//!
//!     cargo bench -p sindr --bench transient

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig, Waveform};

fn r(id: String, a: String, b: String, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id,
        nodes: [a, b],
        resistance,
    }
}

fn c(id: String, a: String, b: String, capacitance: f64) -> CircuitElement {
    CircuitElement::Capacitor {
        id,
        nodes: [a, b],
        capacitance,
    }
}

/// 1 kΩ / 1 nF stages driven by a 1 V step: τ = 1 µs per stage.
fn rc_ladder(stages: usize) -> Circuit {
    let mut components = vec![CircuitElement::VoltageSource {
        id: "V1".into(),
        nodes: ["n0".into(), "0".into()],
        voltage: 0.0,
        waveform: Some(Waveform::Pulse {
            v1: 0.0,
            v2: 1.0,
            delay: 0.0,
            rise_time: 1e-9,
            fall_time: 1e-9,
            pulse_width: 1.0,
            period: 2.0,
        }),
    }];
    for i in 1..=stages {
        components.push(r(
            format!("R{i}"),
            format!("n{}", i - 1),
            format!("n{i}"),
            1e3,
        ));
        components.push(c(format!("C{i}"), format!("n{i}"), "0".into(), 1e-9));
    }
    Circuit::new("0", components)
}

/// 10 V, 50 Hz into a diode, 1000 µF and 100 Ω.
fn rectifier() -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["ac".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Sine {
                    amplitude: 10.0,
                    frequency: 50.0,
                    offset: 0.0,
                    phase: 0.0,
                }),
            },
            r("Rs".into(), "ac".into(), "a".into(), 1.0),
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["a".into(), "out".into()],
                temperature: 300.15,
                model: None,
            },
            c("C1".into(), "out".into(), "0".into(), 1e-3),
            r("RL".into(), "out".into(), "0".into(), 100.0),
        ],
    )
}

fn bench(c: &mut Criterion) {
    let mut group = c.benchmark_group("transient_rc_ladder");
    group.sample_size(10);
    for stages in [10, 100, 1_000] {
        let circuit = rc_ladder(stages);
        // 20 µs at 100 ns: 200 steps before step control.
        let config = TransientConfig::new(20e-6, 100e-9);
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| solve_transient(&circuit, &config).unwrap())
        });
    }
    group.finish();

    let mut group = c.benchmark_group("transient_rectifier");
    group.sample_size(10);
    let circuit = rectifier();
    // Five cycles at 20 µs: 5 000 steps before step control.
    let config = TransientConfig::new(0.1, 20e-6);
    group.bench_function("5_cycles", |b| {
        b.iter(|| solve_transient(&circuit, &config).unwrap())
    });
    group.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
