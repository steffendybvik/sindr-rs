//! Transient cost: an RC ladder at 10 to 1 000 stages over a fixed span,
//! a 50 Hz half-wave rectifier with a filter cap over five cycles
//! (diode switching each cycle, so step control and Newton both work), 100
//! and 1 000 charge-storing diodes switching off one source, a transient
//! session a microcontroller drives for 10 s at 1 kHz, and CSV export of the
//! ladder's waveforms.
//!
//!     cargo bench -p sindr --bench transient

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use sindr::element::{Capacitor, Diode, Resistor, VoltageSource};
use sindr::{
    solve_transient, Circuit, CircuitElement, DiodeModel, TransientConfig, TransientSession,
    Waveform,
};

fn r(id: String, a: String, b: String, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id,
        nodes: [a.into(), b.into()],
        resistance,
    })
}

fn c(id: String, a: String, b: String, capacitance: f64) -> CircuitElement {
    CircuitElement::Capacitor(Capacitor {
        id,
        nodes: [a.into(), b.into()],
        capacitance,
    })
}

/// 1 kΩ / 1 nF stages driven by a 1 V step: τ = 1 µs per stage.
fn rc_ladder(stages: usize) -> Circuit {
    let mut components = vec![CircuitElement::VoltageSource(VoltageSource {
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
    })];
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
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["ac".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Sine {
                    amplitude: 10.0,
                    frequency: 50.0,
                    offset: 0.0,
                    phase: 0.0,
                }),
            }),
            r("Rs".into(), "ac".into(), "a".into(), 1.0),
            CircuitElement::Diode(Diode {
                id: "D1".into(),
                nodes: ["a".into(), "out".into()],
                temperature: 300.15,
                model: None,
            }),
            c("C1".into(), "out".into(), "0".into(), 1e-3),
            r("RL".into(), "out".into(), "0".into(), 100.0),
        ],
    )
}

/// 5 V, 10 kHz through 1 kΩ into each of `n` diodes with junction
/// capacitance and transit time, so every one stores charge.
fn charged_diodes(n: usize) -> Circuit {
    let model = DiodeModel {
        cjo: 4e-12,
        tt: 10e-9,
        ..Default::default()
    };
    let mut components = vec![CircuitElement::VoltageSource(VoltageSource {
        id: "V1".into(),
        nodes: ["ac".into(), "0".into()],
        voltage: 0.0,
        waveform: Some(Waveform::Sine {
            amplitude: 5.0,
            frequency: 10e3,
            offset: 0.0,
            phase: 0.0,
        }),
    })];
    for i in 1..=n {
        components.push(r(format!("R{i}"), "ac".into(), format!("a{i}"), 1e3));
        components.push(CircuitElement::Diode(Diode {
            id: format!("D{i}"),
            nodes: [format!("a{i}").into(), "0".into()],
            temperature: 300.15,
            model: Some(model),
        }));
    }
    Circuit::new("0", components)
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

    let mut group = c.benchmark_group("transient_charged_diodes");
    group.sample_size(10);
    // One cycle at 2 µs: 50 steps before step control.
    let config = TransientConfig::new(100e-6, 2e-6);
    for n in [100, 1_000] {
        let circuit = charged_diodes(n);
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| solve_transient(&circuit, &config).unwrap())
        });
    }
    group.finish();
}

/// A pin driving 1 kΩ into 1 µF for 10 s at 1 kHz, as firmware on an
/// emulated microcontroller would: each millisecond the pin is written (high
/// for two, low for two) and the filtered output read, 10 000 updates.
fn mcu_session(circuit: &Circuit, config: &TransientConfig) -> f64 {
    let mut sim = TransientSession::new(circuit, config).unwrap();
    sim.keep_steps(false);
    let mut adc = 0.0;
    for k in 1..=10_000u32 {
        sim.advance_to(f64::from(k) * 1e-3).unwrap();
        adc += sim.voltage("out").unwrap();
        let level = if (k / 2) % 2 == 0 { 5.0 } else { 0.0 };
        sim.set_source("PIN", level).unwrap();
    }
    adc
}

fn pin_rc() -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "PIN".into(),
                nodes: ["pin".into(), "0".into()],
                voltage: 0.0,
                waveform: None,
            }),
            r("R1".into(), "pin".into(), "out".into(), 1e3),
            c("C1".into(), "out".into(), "0".into(), 1e-6),
        ],
    )
}

fn bench_session(c: &mut Criterion) {
    let mut group = c.benchmark_group("transient_session");
    group.sample_size(10);
    let circuit = pin_rc();
    let config = TransientConfig::new(10.0, 100e-6);
    group.bench_function("10s_1khz_updates", |b| {
        b.iter(|| mcu_session(&circuit, &config))
    });
    group.finish();
}

fn bench_to_csv(c: &mut Criterion) {
    let mut group = c.benchmark_group("transient_to_csv");
    group.sample_size(10);
    for stages in [100, 1_000] {
        let data = solve_transient(&rc_ladder(stages), &TransientConfig::new(20e-6, 100e-9))
            .unwrap()
            .transient
            .unwrap();
        group.bench_with_input(BenchmarkId::from_parameter(stages), &stages, |b, _| {
            b.iter(|| data.to_csv())
        });
    }
    group.finish();
}

criterion_group!(benches, bench, bench_session, bench_to_csv);
criterion_main!(benches);
