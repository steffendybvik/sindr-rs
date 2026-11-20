//! sindr against ngspice on the same decks: time per call when embedded,
//! parallel throughput, and how both scale with circuit size.
//!
//! ```text
//! cargo run --release -p sindr --example vs_ngspice --features spice
//! cargo run --release -p sindr --example vs_ngspice --features spice -- --quick
//! ```
//!
//! Needs `ngspice` on the PATH (macOS: `brew install ngspice`). It isn't
//! part of CI: the runners have no ngspice, and this measures ngspice as much
//! as sindr. The README's "Against ngspice" section quotes its output.
//!
//! Both simulators read the same netlist at their default tolerances. sindr
//! is timed in-process, as a program embedding it calls it. ngspice is timed
//! three ways: spawning `ngspice -b` per run (what a program shelling out to
//! it pays, fastest of several), the same minus the fastest start on a
//! one-resistor deck (its solve alone, roughly), and, for throughput, a `.control` loop that runs every
//! op point in one ngspice process (the fastest way to drive it from batch
//! mode).

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::spice::{parse_file, parse_str};
use sindr::{solve_circuit, solve_transient, Circuit, CircuitElement};

fn golden_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

fn scratch() -> PathBuf {
    let dir = std::env::temp_dir().join(format!("sindr-vs-ngspice-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Mean time of `f`, repeated until `budget` has passed (at least 3 runs).
fn time_in_process(budget: Duration, mut f: impl FnMut()) -> Duration {
    f(); // warm up
    let start = Instant::now();
    let mut runs = 0u32;
    while runs < 3 || start.elapsed() < budget {
        f();
        runs += 1;
    }
    start.elapsed() / runs
}

/// Fastest of `runs` runs of `ngspice -b deck`. The fastest, not the
/// median, because the startup it's compared against is the same, and
/// subtracting one noisy figure from another hides a short solve.
fn time_ngspice(deck: &Path, runs: usize) -> Duration {
    (0..runs)
        .map(|_| {
            let start = Instant::now();
            run_ngspice(deck);
            start.elapsed()
        })
        .min()
        .unwrap()
}

/// ngspice's time beyond starting up, or "< 1 ms" inside the noise.
fn solve_only(ng: Duration, startup: Duration) -> String {
    let d = ng.saturating_sub(startup);
    if d < Duration::from_millis(1) {
        "< 1 ms".into()
    } else {
        fmt(d)
    }
}

fn run_ngspice(deck: &Path) {
    let status = Command::new("ngspice")
        .arg("-b")
        .arg(deck)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .expect("ngspice on the PATH");
    assert!(status.success(), "ngspice failed on {}", deck.display());
}

/// `netlist` (without `.end`) with `control` as its `.control` block.
fn write_deck(dir: &Path, name: &str, netlist: &str, control: &str) -> PathBuf {
    let body: String = netlist
        .lines()
        .filter(|l| !l.trim().eq_ignore_ascii_case(".end"))
        .map(|l| format!("{l}\n"))
        .collect();
    let path = dir.join(format!("{name}.cir"));
    std::fs::write(&path, format!("{body}.control\n{control}\n.endc\n.end\n")).unwrap();
    path
}

fn fmt(d: Duration) -> String {
    let s = d.as_secs_f64();
    if s >= 1.0 {
        format!("{s:.2} s")
    } else if s >= 1e-3 {
        format!("{:.2} ms", s * 1e3)
    } else {
        format!("{:.0} µs", s * 1e6)
    }
}

fn ratio(slow: Duration, fast: Duration) -> String {
    format!("{:.0}×", slow.as_secs_f64() / fast.as_secs_f64())
}

struct Settings {
    budget: Duration,
    spawns: usize,
    mc_runs: usize,
    sizes: &'static [usize],
}

fn main() {
    let quick = std::env::args().any(|a| a == "--quick");
    let s = if quick {
        Settings {
            budget: Duration::from_millis(100),
            spawns: 5,
            mc_runs: 200,
            sizes: &[10, 100],
        }
    } else {
        Settings {
            budget: Duration::from_millis(500),
            spawns: 21,
            mc_runs: 1000,
            sizes: &[10, 100, 1000, 3000],
        }
    };
    let dir = scratch();
    let version = Command::new("ngspice")
        .arg("--version")
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).into_owned())
        .unwrap_or_default();
    let version = version
        .lines()
        .find(|l| l.contains("ngspice-"))
        .unwrap_or("ngspice")
        .trim_matches(|c: char| c == '*' || c.is_whitespace());
    println!("{version}, {} threads available\n", threads());

    let empty = write_deck(&dir, "empty", "startup\nR1 a 0 1\n", "op");
    let startup = time_ngspice(&empty, s.spawns);
    println!(
        "Starting ngspice on a one-resistor deck: {}\n",
        fmt(startup)
    );

    per_call(&s, &dir, startup);
    throughput(&s, &dir);
    scaling(&s, &dir, startup);
    let _ = std::fs::remove_dir_all(&dir);
}

fn threads() -> usize {
    std::thread::available_parallelism().map_or(1, |n| n.get())
}

// ------------------------------------------------------------ per call

fn per_call(s: &Settings, dir: &Path, startup: Duration) {
    println!("## Time per call (golden decks)\n");
    println!("| Case | sindr in-process | ngspice -b | ngspice minus startup | vs spawn |");
    println!("|---|---:|---:|---:|---:|");
    let ac = AcConfig {
        f_start: 100.0,
        f_stop: 100e3,
        num_points: 61,
        spacing: FrequencySpacing::Logarithmic,
        source_id: "v1".into(),
        ac_magnitude: 1.0,
    };
    let cases: [(&str, &str, &str); 4] = [
        ("op_diode_series", "op", "op"),
        ("op_bjt_ce", "op", "op"),
        (
            "ac_rlc_bandpass",
            "ac, 61 points",
            "alter @v1[acmag] = 1\nac dec 20 100 100k",
        ),
        ("tran_halfwave", "tran, 5 ms of a rectifier", "run"),
    ];
    for (case, label, control) in cases {
        let path = golden_dir().join(format!("{case}.cir"));
        let netlist = parse_file(&path).unwrap();
        let circuit = netlist.circuit.clone();
        let sindr = match case {
            "ac_rlc_bandpass" => time_in_process(s.budget, || {
                solve_ac(&circuit, &ac).unwrap();
            }),
            "tran_halfwave" => {
                let config = netlist.transient_config().unwrap();
                time_in_process(s.budget, || {
                    solve_transient(&circuit, &config).unwrap();
                })
            }
            _ => time_in_process(s.budget, || {
                solve_circuit(&circuit).unwrap();
            }),
        };
        let deck = std::fs::read_to_string(&path).unwrap();
        let ng = time_ngspice(&write_deck(dir, case, &deck, control), s.spawns);
        println!(
            "| `{case}` ({label}) | {} | {} | {} | {} |",
            fmt(sindr),
            fmt(ng),
            solve_only(ng, startup),
            ratio(ng, sindr),
        );
    }
    println!();
}

// ---------------------------------------------------------- throughput

/// `op_bjt_ce` with the base resistor drawn ±10 %: a Monte Carlo of op
/// points. A small LCG keeps the example free of a rand dependency.
fn monte_carlo_values(n: usize) -> Vec<f64> {
    let mut state: u64 = 0x2545_f491_4f6c_dd1d;
    (0..n)
        .map(|_| {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let unit = (state >> 11) as f64 / (1u64 << 53) as f64;
            470e3 * (0.9 + 0.2 * unit)
        })
        .collect()
}

fn with_rb(base: &Circuit, rb: f64) -> Circuit {
    let mut c = base.clone();
    for e in &mut c.components {
        if let CircuitElement::Resistor { id, resistance, .. } = e {
            if id.eq_ignore_ascii_case("rb") {
                *resistance = rb;
            }
        }
    }
    c
}

fn throughput(s: &Settings, dir: &Path) {
    let n = s.mc_runs;
    println!("## Parallel throughput: {n} op points of `op_bjt_ce`, RB ±10 %\n");
    let path = golden_dir().join("op_bjt_ce.cir");
    let base = parse_file(&path).unwrap().circuit;
    let values = monte_carlo_values(n);

    println!("| Runner | Wall time | Op points/s |");
    println!("|---|---:|---:|");
    let row = |label: &str, d: Duration| {
        println!(
            "| {label} | {} | {:.0} |",
            fmt(d),
            n as f64 / d.as_secs_f64()
        );
    };

    let mut counts = vec![1, 2, 4, threads()];
    counts.dedup();
    for t in counts.into_iter().filter(|&t| t <= threads()) {
        let start = Instant::now();
        std::thread::scope(|scope| {
            for chunk in values.chunks(n.div_ceil(t)) {
                let base = &base;
                scope.spawn(move || {
                    for &rb in chunk {
                        solve_circuit(&with_rb(base, rb)).unwrap();
                    }
                });
            }
        });
        row(
            &format!("sindr, {t} thread(s), one process"),
            start.elapsed(),
        );
    }

    // One ngspice process, every op point from a .control loop.
    let deck = std::fs::read_to_string(&path).unwrap();
    let mut control = String::new();
    for rb in &values {
        writeln!(control, "alter rb = {rb}\nop\ndestroy all").unwrap();
    }
    let looped = write_deck(dir, "mc_loop", &deck, control.trim_end());
    let start = Instant::now();
    run_ngspice(&looped);
    row("ngspice, one process, .control loop", start.elapsed());

    // A process per op point, as many at once as there are cores.
    let decks: Vec<PathBuf> = values
        .iter()
        .enumerate()
        .map(|(i, rb)| {
            write_deck(
                dir,
                &format!("mc_{i}"),
                &deck,
                &format!("alter rb = {rb}\nop"),
            )
        })
        .collect();
    let start = Instant::now();
    std::thread::scope(|scope| {
        for chunk in decks.chunks(decks.len().div_ceil(threads())) {
            scope.spawn(move || chunk.iter().for_each(|d| run_ngspice(d)));
        }
    });
    row(
        &format!("ngspice -b per point, {} at once", threads()),
        start.elapsed(),
    );
    println!();
}

// ------------------------------------------------------------- scaling

/// `n` diode–resistor stages (the `newton` bench's chain), as a deck.
fn diode_chain(n: usize) -> String {
    let mut d = format!("diode chain, {n} stages\nV1 n0 0 5\n");
    for i in 1..=n {
        let p = i - 1;
        writeln!(d, "R{i} n{p} n{i} 1k\nRP{i} n0 n{i} 10k\nD{i} n{i} 0 dd").unwrap();
    }
    d + ".model dd D(IS=1e-14 N=1)\n.end\n"
}

/// `n` RC stages driven by a 1 V step, 200 fixed steps of 0.1 µs.
fn rc_ladder(n: usize) -> String {
    let mut d = format!("rc ladder, {n} stages\nV1 n0 0 PULSE(0 1 0 1n 1n 1 2)\n");
    for i in 1..=n {
        let p = i - 1;
        writeln!(d, "R{i} n{p} n{i} 100\nC{i} n{i} 0 1n").unwrap();
    }
    d + ".tran 0.1u 20u 0 0.1u\n.end\n"
}

fn scaling(s: &Settings, dir: &Path, startup: Duration) {
    println!("## Scaling with size\n");
    println!("| Circuit | Stages | Nodes | sindr | ngspice -b | ngspice minus startup |");
    println!("|---|---:|---:|---:|---:|---:|");
    let spawns = s.spawns.min(5);
    for &n in s.sizes {
        let text = diode_chain(n);
        let circuit = parse_str(&text).unwrap().circuit;
        let sindr = time_in_process(s.budget, || {
            solve_circuit(&circuit).unwrap();
        });
        let ng = time_ngspice(&write_deck(dir, "chain", &text, "op"), spawns);
        println!(
            "| Diode chain, op | {n} | {} | {} | {} | {} |",
            n + 1,
            fmt(sindr),
            fmt(ng),
            solve_only(ng, startup),
        );
    }
    for &n in s.sizes {
        let text = rc_ladder(n);
        let netlist = parse_str(&text).unwrap();
        let config = netlist.transient_config().unwrap();
        let sindr = time_in_process(s.budget, || {
            solve_transient(&netlist.circuit, &config).unwrap();
        });
        let ng = time_ngspice(&write_deck(dir, "ladder", &text, "run"), spawns);
        println!(
            "| RC ladder, 200-step tran | {n} | {} | {} | {} | {} |",
            n + 1,
            fmt(sindr),
            fmt(ng),
            solve_only(ng, startup),
        );
    }
    println!();
}
