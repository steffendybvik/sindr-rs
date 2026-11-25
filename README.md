# sindr

[![CI](https://github.com/steffendybvik/sindr-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/steffendybvik/sindr-rs/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/sindr.svg)](https://crates.io/crates/sindr)
[![docs.rs](https://docs.rs/sindr/badge.svg)](https://docs.rs/sindr)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](#license)

A circuit simulator you embed in Rust: build a circuit as a typed value, solve it natively or in the browser, and get back numbers checked against ngspice.

## Why sindr

- **Typed in, typed out.** A `Circuit` is a Rust value (or JSON), results are structs, and errors are a `SimError` enum you can match on, not text on stderr.
- **Pure Rust, no C.** No `unsafe` in any crate (each has `#![forbid(unsafe_code)]`), no libngspice to install, no global state: run as many simulations in parallel threads as you like.
- **Builds for WebAssembly.** `wasm32-unknown-unknown` with every feature on, so the same engine runs in a web app.
- **Answers design questions.** `check_ratings` says whether each part stays inside its absolute maximum ratings, self-heating flags thermal runaway, and `check_specs` turns `.meas`-style measurements into pass/fail with margins.
- **Real parts as primitives.** 39 element types: the usual R, L, C, sources and semiconductors, plus LEDs, zeners, linear and switching regulators, op-amps, comparators, relays, fuses, switches, potentiometers, thermistors, photoresistors, transformers, batteries, crystals, ferrite beads, DC motors, MCU pins and logic gates.
- **Checked against ngspice.** Every analysis ngspice also runs, except `.tf`, has golden cases solved by both (below).

## Quick start

```toml
[dependencies]
sindr = "0.3"
```

How much current does a red LED draw from 5 V through 220 Ω?

```rust
use sindr::{solve_circuit, Circuit, CircuitElement};
use sindr::element::{Led, Resistor, VoltageSource};

fn main() -> Result<(), sindr::SimError> {
    let circuit = Circuit::new("gnd", vec![
        CircuitElement::VoltageSource(VoltageSource { id: "V1".into(), nodes: ["vcc".into(), "gnd".into()], voltage: 5.0, waveform: None }),
        CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["vcc".into(), "a".into()], resistance: 220.0 }),
        CircuitElement::Led(Led { id: "D1".into(), nodes: ["a".into(), "gnd".into()], color: "red".into(), temperature: 300.15 }),
    ]);
    let result = solve_circuit(&circuit)?;
    let led = result.component("D1").expect("D1 is in the circuit");
    println!("{:.1} mA, {:.1} mW", led.current_through * 1e3, led.power * 1e3);
    Ok(())
}
```

The same circuit with typed handles: `CircuitBuilder` declares each net once and `build` refuses an element wired to a net it never declared, so a typo fails instead of leaving a floating node.

```rust
use sindr::{solve_circuit, CircuitBuilder};
use sindr::element::{Led, Resistor, VoltageSource};

fn main() -> Result<(), sindr::SimError> {
    let mut b = CircuitBuilder::with_ground("gnd");
    let (gnd, vcc, a) = (b.ground(), b.node("vcc"), b.node("a"));
    b.add(VoltageSource::dc("V1", [vcc.clone(), gnd.clone()], 5.0));
    b.add(Resistor { id: "R1".into(), nodes: [vcc, a.clone()], resistance: 220.0 });
    let d1 = b.add(Led { id: "D1".into(), nodes: [a, gnd], color: "red".into(), temperature: 300.15 });
    let result = solve_circuit(&b.build()?)?;
    let led = result.component(&d1).expect("D1 is in the circuit");
    println!("{:.1} mA, {:.1} mW", led.current_through * 1e3, led.power * 1e3);
    Ok(())
}
```

More in [`sindr/examples/`](./sindr/examples): RC transient, BJT amplifier, Bode plot, DC sweep, a SPICE deck.

## Crates

| Crate | Purpose |
|-------|---------|
| [`sindr`](./sindr) | The simulator and its data model (`Circuit`, elements, waveforms, design hierarchy, part data): operating point, transient (also stepped from outside, as a session), AC, DC and temperature sweeps, `.tf`, `.sens`, `.noise`, `.pz`, loop gain, `.four` and FFT, `.meas`, ratings checks, SPICE front end |
| [`sindr-devices`](./sindr-devices) | Device physics: diode family, BJT, MOSFET, JFET, IGBT, varactor, thermistor, photoresistor, regulator and switching-converter models |

`sindr` depends on `sindr-devices`. The split lets you use the device-physics models with your own solver, without the `nalgebra` linear-algebra stack.

Both build for `wasm32-unknown-unknown`, with default and all features, so sindr runs in a browser. CI builds them and solves an operating point, an AC sweep, a transient and a SPICE deck under Node on every pull request.

## Accuracy

Each analysis but `.tf` is checked against [ngspice](https://ngspice.sourceforge.io/) 47 on golden circuits solved by both: diodes with their `.model` cards and charge, BJTs, MOSFETs, JFETs, LEDs, controlled sources, transformers, subcircuits. The worst deviation of any quantity at any point, per analysis:

<!-- accuracy:start -->
| Analysis | Cases | Worst relative error | Worst other |
|---|---|---|---|
| Operating point | 22 | 7e-6 | — |
| DC sweep | 10 | 4e-5 | — |
| Temperature sweep | 4 | 8e-7 | — |
| AC | 15 | 3e-6 | 1e-4° phase |
| Diode charge (AC) | 2 | 2e-4 | — |
| Noise | 4 | 7e-7 | — |
| Pole-zero | 2 | 1e-9 | — |
| Transient | 15 | 3e-3 | 2.6 % of full scale |
<!-- accuracy:end -->

The transient's worst is a switching BJT's collector on a 3 µs edge, where a 0.4 mV base error becomes 2.6 % of the current; its other cases stay under 0.15 %. Its worst relative error, 3e-3, is the averaged buck converter's inductor-ripple estimate against a switched buck in ngspice: a model against a different circuit, not the solver's error. Every case, and the tolerance its test holds it to, is in [`docs/ACCURACY.md`](./docs/ACCURACY.md); how the cases are built is in [`sindr/tests/golden/README.md`](./sindr/tests/golden/README.md). Both tables come from the test suite (`scripts/accuracy_table.py`).

## Speed

Medians on an Apple M1 Pro, release build, one thread ([details](./sindr/README.md#performance)):

| Circuit | Time |
|---|---:|
| Operating point, 100 diodes and resistors | 0.31 ms |
| Transient, 100-stage RC ladder, 200 steps | 6.6 ms |
| Half-wave rectifier, five 50 Hz cycles (5 000 steps) | 8 ms |

CI fails a pull request that makes any benchmark more than 25 % slower.

### Against ngspice

The same netlists in both, at their default tolerances, on the same M1 Pro
with ngspice 47 ([`vs_ngspice`](./sindr/examples/vs_ngspice.rs); needs
ngspice, so it isn't in CI). sindr is called in-process. ngspice is either
spawned per run with `ngspice -b`, or driven by one `.control` loop.

| Per call | sindr | ngspice -b |
|---|---:|---:|
| Operating point, BJT stage | 7 µs | 14 ms (all but < 1 ms is startup) |
| AC, series RLC, 61 points | 48 µs | 14 ms (same) |
| Transient, 5 ms of a rectifier | 35 ms | 56 ms (42 ms solving) |

| 1 000 op points, base resistor ±10 % | Time | Op points/s |
|---|---:|---:|
| sindr, 1 thread | 8.8 ms | 114 000 |
| sindr, 4 threads, one process | 3.4 ms | 290 000 |
| ngspice, one process, `.control` loop | 36 ms | 28 000 |
| ngspice -b per point, 10 at once | 2.0 s | 500 |

| Size | sindr | ngspice, solve only |
|---|---:|---:|
| Operating point, 1 000-diode chain | 3.2 ms | 17 ms |
| Operating point, 3 000-diode chain | 9.6 ms | 87 ms |
| Transient, 1 000-stage RC ladder, 200 steps | 72 ms | 32 ms |
| Transient, 3 000-stage RC ladder, 200 steps | 213 ms | 91 ms |

Embedded, a small solve costs microseconds against ngspice's 14 ms start,
and sindr has no global state, so solves run in parallel threads. ngspice
solves large transients a little over twice as fast: past several hundred
nodes, a long transient is ngspice's ground ([limits](./LIMITATIONS.md#solver--numerics)).

## When to use sindr

| | sindr | A full SPICE simulator |
|---|---|---|
| What it is | A simulator built to embed, with design checks | A standalone program or C library that runs netlists |
| Main input | `Circuit` values or JSON; a SPICE3 subset (feature `spice`) | SPICE netlists in its own dialect |
| WebAssembly, no C library | Yes | Usually not |
| Ratings, self-heating, pass/fail specs | Built in | Scripted by hand, if at all |
| Analyses | op, DC, AC, transient, temperature, `.tf`, `.sens` (DC and AC), `.noise`, `.pz`, loop gain, `.meas`, `.four` and FFT | Those and more: distortion |
| Device models | Shockley diode, Ebers–Moll BJT, MOSFET Level 1, JFET, IGBT, op-amp macromodel, behavioural sources, and part-level elements (LED, regulator, relay, battery …) | **Far more:** Gummel–Poon, BSIM, transmission lines, XSPICE code models |
| SPICE dialect coverage | A practical subset | Full |

Use a full SPICE simulator to run existing decks and vendor model libraries as they are. Use sindr to put a simulator inside a Rust program or a web app, describe circuits as data, and ask whether the parts survive.

## SPICE netlist input

A SPICE3 netlist parser ships inside `sindr` as the `spice` feature, on by default like every feature but the unstable `internals` (`default-features = false` and a `features` list keeps only the ones you pick):

```toml
[dependencies]
sindr = "0.3"
```

```rust,ignore
let netlist = sindr::spice::parse_file("amp.cir")?;
let result = sindr::solve_circuit(&netlist.circuit)?;
```

It reads a practical subset of Berkeley SPICE3f5 decks (R, L, C, V, I, D, Q, M, J, B, X, linear E/G/F/H (and `VALUE`, `POLY`), K coupling, `.subckt`, `.global`, `.model`, `.tran`, `.dc`, `.ac`, `.tf`, `.sens`, `.noise`, `.pz`, `.meas`, `.four`, `.op`, `.ic`, `.nodeset`, `.options`, `.param`, `.include`, `.lib`), keeps `.subckt` hierarchy as blocks and instances (flattened with `.`-separated naming when simulated), and returns a `SourceMap` to recover original instance paths. `to_netlist` writes a `Circuit` back out, `.subckt`s included. See the [`sindr::spice`](https://docs.rs/sindr/latest/sindr/spice/) module docs for the exact supported subset and strictness options.

`sindr::spice::to_netlist` goes the other way: it writes a `Circuit` as a SPICE deck (R, L, C, V, I with waveforms, D and Q with `.model` cards, E/G/F/H, transformers as L–L–K, and exact R/D equivalents for switches, fuses, potentiometers, LEDs and the like) that ngspice runs, and that `parse_str` reads back. Elements with no SPICE equivalent are an `ExportError`.

A runnable walkthrough (parse → solve → dispatch directives → render a diagnostic) lives in [`sindr/examples/parse_netlist.rs`](./sindr/examples/parse_netlist.rs):

```bash
cargo run --example parse_netlist
```

## Status

`0.3.1`, [published on crates.io](https://crates.io/crates/sindr). Until 1.0, a breaking change bumps the minor version (`0.4.0`) and anything else the patch (`0.3.2`), so `sindr = "0.3"` never breaks on `cargo update`. The road to 1.0 is in [`ROADMAP.md`](./ROADMAP.md).

## Scope & limitations

sindr is an analog circuit simulator. Digital logic (beyond gates at the analog level), HDL ingestion, MCU simulation (pair your own with a `TransientSession`), schematic capture, BSIM-class and Gummel–Poon device models, and coupled thermal networks (beyond one thermal RC per part) are **not** supported today. Vendor models load only as level-1 cards or subcircuits built from sindr's elements. SPICE netlist import is available as a parser for a practical subset (see above), not full SPICE3 coverage. See [`LIMITATIONS.md`](./LIMITATIONS.md) for the full breakdown of what's out-of-scope versus not-yet-implemented.

Where it's heading on the way to 1.0 is in [`ROADMAP.md`](./ROADMAP.md). To work on sindr itself, see [`CONTRIBUTING.md`](./CONTRIBUTING.md). AI coding agents (Claude Code, Codex, Cursor, etc.) should also read [`AGENTS.md`](./AGENTS.md) for layout, conventions, and pitfalls.

## License

Dual-licensed under either of:

- [MIT License](./LICENSE-MIT) ([https://opensource.org/licenses/MIT](https://opensource.org/licenses/MIT))
- [Apache License, Version 2.0](./LICENSE-APACHE) ([http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual-licensed as above, without any additional terms or
conditions.
