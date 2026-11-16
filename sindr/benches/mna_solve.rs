//! MNA solve cost against circuit size, dense and sparse: a resistor ladder
//! and an RC mesh at 10 to 10 000 nodes. `DENSE_MAX` in `src/mna.rs` comes
//! from where the two cross.
//!
//!     cargo bench -p sindr --bench mna_solve

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use nalgebra::DVector;
use sindr::mna::MnaMatrix;

/// Conductance matrix of a ladder of 1 kΩ series resistors with a 10 kΩ
/// shunt at each node, driven by a source row on node 0.
fn ladder(a: &mut MnaMatrix<f64>, n: usize) {
    for i in 0..n {
        a[(i, i)] += 1e-4;
        if i + 1 < n {
            let g = 1e-3;
            a[(i, i)] += g;
            a[(i + 1, i + 1)] += g;
            a[(i, i + 1)] -= g;
            a[(i + 1, i)] -= g;
        }
    }
    a[(0, n)] = 1.0;
    a[(n, 0)] = 1.0;
}

/// A √n × √n grid of 1 kΩ resistors with a capacitor companion (C/dt) to
/// ground at every node, driven at one corner.
fn rc_mesh(a: &mut MnaMatrix<f64>, n: usize) {
    let side = (n as f64).sqrt() as usize;
    let idx = |r: usize, c: usize| r * side + c;
    let g = 1e-3;
    for r in 0..side {
        for c in 0..side {
            let i = idx(r, c);
            a[(i, i)] += 1.0; // C/dt: 1 µF over a 1 µs step
            for (rr, cc) in [(r + 1, c), (r, c + 1)] {
                if rr < side && cc < side {
                    let j = idx(rr, cc);
                    a[(i, i)] += g;
                    a[(j, j)] += g;
                    a[(i, j)] -= g;
                    a[(j, i)] -= g;
                }
            }
        }
    }
    let m = side * side;
    a[(0, m)] = 1.0;
    a[(m, 0)] = 1.0;
}

fn bench(c: &mut Criterion) {
    for (name, build, sizes) in [
        (
            "ladder",
            ladder as fn(&mut MnaMatrix<f64>, usize),
            [10, 100, 1_000, 10_000],
        ),
        ("rc_mesh", rc_mesh, [9, 100, 1_024, 10_000]),
    ] {
        let mut group = c.benchmark_group(name);
        group.sample_size(10);
        for n in sizes {
            let size = n + 1;
            let b = DVector::from_fn(size, |i, _| if i == size - 1 { 1.0 } else { 0.0 });
            // Dense stops at 1 000 nodes: 10 000 would be an 800 MB matrix.
            if n <= 1_100 {
                group.bench_with_input(BenchmarkId::new("dense", n), &n, |bench, &n| {
                    bench.iter(|| {
                        let mut a = MnaMatrix::dense(size);
                        build(&mut a, n);
                        a.solve(&b).unwrap()
                    })
                });
            }
            group.bench_with_input(BenchmarkId::new("sparse", n), &n, |bench, &n| {
                bench.iter(|| {
                    let mut a = MnaMatrix::sparse(size);
                    build(&mut a, n);
                    a.solve(&b).unwrap()
                })
            });
        }
        group.finish();
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
