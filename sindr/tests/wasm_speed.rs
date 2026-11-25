//! The same solves timed natively and in a browser's WebAssembly, for the
//! browser-against-native ratio. Ignored by default; run each side with
//!
//! ```text
//! cargo test -p sindr --release --all-features --test wasm_speed -- --ignored --nocapture
//! CARGO_TARGET_WASM32_UNKNOWN_UNKNOWN_RUNNER=wasm-bindgen-test-runner \
//!     cargo test -p sindr --release --all-features --target wasm32-unknown-unknown --test wasm_speed -- --include-ignored
//! ```
//!
//! The second runs in Node (V8, Chrome's engine). With `WASM_BINDGEN_USE_BROWSER=1
//! NO_HEADLESS=1` the runner serves the same test on a local page to open
//! in any browser instead.

#![cfg(feature = "spice")]

use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::spice::parse_str;
use sindr::{solve_circuit, solve_transient, Circuit, TransientConfig, TransientSession};

#[cfg(target_arch = "wasm32")]
use wasm_bindgen_test::wasm_bindgen_test as test_case;

#[cfg(not(target_arch = "wasm32"))]
use core::prelude::v1::test as test_case;

/// Milliseconds on a monotonic-enough clock.
#[cfg(target_arch = "wasm32")]
fn now_ms() -> f64 {
    js_sys::Date::now()
}

#[cfg(not(target_arch = "wasm32"))]
fn now_ms() -> f64 {
    use std::sync::OnceLock;
    use std::time::Instant;
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_secs_f64() * 1e3
}

#[cfg(target_arch = "wasm32")]
macro_rules! report {
    ($($t:tt)*) => { wasm_bindgen_test::console_log!($($t)*) };
}

#[cfg(not(target_arch = "wasm32"))]
macro_rules! report {
    ($($t:tt)*) => { println!($($t)*) };
}

/// Mean time per run of `f` (ms), over at least a second and three runs.
fn time(name: &str, mut f: impl FnMut()) {
    f(); // warm up
    let (start, mut runs) = (now_ms(), 0u32);
    while runs < 3 || now_ms() - start < 1000.0 {
        f();
        runs += 1;
    }
    let mean = (now_ms() - start) / f64::from(runs);
    report!("speed {name}: {mean:.3} ms ({runs} runs)");
}

fn circuit(deck: &str) -> Circuit {
    parse_str(deck).expect("deck parses").circuit
}

/// 50 resistors and 50 diodes: each stage a series 1 kΩ and a diode to
/// ground, so Newton works on every node.
fn diode_ladder() -> Circuit {
    let mut deck = String::from("diode ladder\nV1 n0 0 5\n");
    for i in 1..=50 {
        deck += &format!("R{i} n{} n{i} 1k\nD{i} n{i} 0 dmod\n", i - 1);
    }
    deck += ".model dmod D(IS=1e-14 N=1)\n.end\n";
    circuit(&deck)
}

/// A 50 Hz half-wave rectifier into 100 µF ‖ 1 kΩ, five cycles in 20 µs
/// steps: 5 000 steps.
fn rectifier() -> Circuit {
    circuit(
        "rectifier\nV1 ac 0 SIN(0 10 50)\nD1 ac out dmod\nC1 out 0 100u\nR1 out 0 1k\n\
         .model dmod D(IS=1e-14 N=1)\n.end\n",
    )
}

/// 100 RC stages (1 kΩ, 1 nF).
fn rc_ladder() -> Circuit {
    let mut deck = String::from("rc ladder\nV1 n0 0 0\n");
    for i in 1..=100 {
        deck += &format!("R{i} n{} n{i} 1k\nC{i} n{i} 0 1n\n", i - 1);
    }
    deck += ".end\n";
    circuit(&deck)
}

#[test_case]
#[ignore = "timing, run by hand"]
fn speed() {
    let ladder = diode_ladder();
    let op = solve_circuit(&ladder).unwrap();
    assert!(op.node_voltages["n1"] > 0.5);
    time("operating_point/100_parts", || {
        solve_circuit(&ladder).unwrap();
    });

    let rect = rectifier();
    let config = TransientConfig::new(0.1, 20e-6);
    let steps = solve_transient(&rect, &config)
        .unwrap()
        .transient
        .unwrap()
        .timesteps
        .len();
    assert!(steps >= 5_000, "{steps} steps");
    time("transient/rectifier_5000_steps", || {
        solve_transient(&rect, &config).unwrap();
    });

    let rc = rc_ladder();
    // The parser's id for `V1`, whatever case it keeps.
    let source = rc
        .components
        .iter()
        .map(|c| c.id().to_string())
        .find(|id| id.eq_ignore_ascii_case("v1") || id.ends_with("V1"))
        .unwrap();
    let ac = AcConfig::new(source, 10.0, 1e6, 100);
    assert_eq!(solve_ac(&rc, &ac).unwrap().points.len(), 100);
    time("ac/rc_ladder_100_stages_100_points", || {
        solve_ac(&rc, &ac).unwrap();
    });

    // A pin a microcontroller writes every millisecond for 10 s, into
    // 1 kΩ and 1 µF, its output read back each time: 10 000 updates.
    let pin = circuit("pin rc\nVPIN pin 0 0\nR1 pin out 1k\nC1 out 0 1u\n.end\n");
    let source = pin.components[0].id().to_string();
    let config = TransientConfig::new(10.0, 100e-6);
    let session = || {
        let mut sim = TransientSession::new(&pin, &config).unwrap();
        sim.keep_steps(false);
        let mut adc = 0.0;
        for k in 1..=10_000u32 {
            sim.advance_to(f64::from(k) * 1e-3).unwrap();
            adc += sim.voltage("out").unwrap();
            let level = if (k / 2) % 2 == 0 { 5.0 } else { 0.0 };
            sim.set_source(&source, level).unwrap();
        }
        adc
    };
    // High half the time: the output averages about half the rail.
    assert!((session() / 10_000.0 - 2.5).abs() < 0.2);
    time("session/10s_1khz_updates", || {
        session();
    });
}
