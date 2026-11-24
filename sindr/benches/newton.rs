//! Newton–Raphson cost on DC operating points with many nonlinear elements:
//! a chain of diode–resistor stages, and a row of BJT common-emitter stages,
//! each base fed from the collector before it, at 10 to 1 000 devices.
//!
//!     cargo bench -p sindr --bench newton

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use sindr::element::{Bjt, Diode, Resistor, VoltageSource};
use sindr::{solve_circuit, BjtKind, Circuit, CircuitElement};

fn r(id: String, a: String, b: String, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id,
        nodes: [a.into(), b.into()],
        resistance,
    })
}

fn supply(node: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: format!("V{node}"),
        nodes: [node.into(), "0".into()],
        voltage: volts,
        waveform: None,
    })
}

/// Each stage: 1 kΩ from the previous node, a diode to ground. Every diode
/// sits near 0.6 V, so each one needs its own Newton iterations.
fn diode_chain(n: usize) -> Circuit {
    let mut components = vec![supply("n0", 5.0)];
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
    Circuit::new("0", components)
}

/// Common-emitter stages: 100 kΩ base feed from the stage before, 1 kΩ
/// collector load to 5 V, 100 Ω emitter degeneration.
fn bjt_row(n: usize) -> Circuit {
    let mut components = vec![supply("vcc", 5.0), supply("in", 1.0)];
    for i in 1..=n {
        let feed = if i == 1 {
            "in".to_string()
        } else {
            format!("c{}", i - 1)
        };
        components.push(r(format!("RB{i}"), feed, format!("b{i}"), 100e3));
        components.push(r(format!("RC{i}"), "vcc".into(), format!("c{i}"), 1e3));
        components.push(r(format!("RE{i}"), format!("e{i}"), "0".into(), 100.0));
        components.push(CircuitElement::Bjt(Bjt {
            id: format!("Q{i}"),
            nodes: [
                format!("b{i}").into(),
                format!("c{i}").into(),
                format!("e{i}").into(),
            ],
            kind: BjtKind::Npn,
            bf: 100.0,
            temperature: 300.15,
            parasitic_caps: None,
            model: None,
        }));
    }
    Circuit::new("0", components)
}

fn bench(c: &mut Criterion) {
    for (name, build) in [
        ("newton_diode_chain", diode_chain as fn(usize) -> Circuit),
        ("newton_bjt_row", bjt_row),
    ] {
        let mut group = c.benchmark_group(name);
        group.sample_size(10);
        for n in [10, 100, 1_000] {
            let circuit = build(n);
            group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
                b.iter(|| solve_circuit(&circuit).unwrap())
            });
        }
        group.finish();
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
