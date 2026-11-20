# AGENTS.md — orientation for AI coding agents

This file briefs an AI coding assistant (Claude Code, Codex, Cursor, etc.) working on `sindr-rs`. Read this before making changes. For human-facing capability docs see [`README.md`](./README.md); for what sindr **cannot** do see [`LIMITATIONS.md`](./LIMITATIONS.md).

## What this repo is

A pure-Rust analog circuit simulator. SPICE-style MNA (Modified Nodal Analysis) solver plus semiconductor companion models. Library only — no CLI, no GUI, no `no_std`. Pre-1.0 (0.x).

## Workspace layout

```
sindr-rs/
├── sindr-core/         # data model: Circuit, CircuitElement, Waveform, design hierarchy (no solver dep)
│   └── src/{circuit,design,waveform}.rs   # waveform: sine, pulse, square, triangle, PWM, PWL
├── sindr/              # MNA solver crate (depends on sindr-core, sindr-devices)
│   ├── src/
│   │   ├── lib.rs              # public API surface, re-exports
│   │   ├── mna.rs              # MNA matrix assembly
│   │   ├── stamp.rs            # per-element matrix stamping
│   │   ├── newton_raphson.rs   # nonlinear DC solver
│   │   ├── transient.rs        # transient stepper (TransientConfig, solve_transient)
│   │   ├── integration.rs      # Gear-2 / trapezoidal / backward-Euler formulas
│   │   ├── ac_analysis.rs      # small-signal AC (and the linearised system .tf reuses)
│   │   ├── transfer_function.rs # .tf: DC small-signal gain, Rin, Rout
│   │   ├── sensitivity.rs      # .sens: DC adjoint sensitivity
│   │   ├── dc_sweep.rs         # parameter sweep
│   │   ├── temp_sweep.rs       # temperature sweep
│   │   ├── results.rs          # SimulationResult, ComponentResult, etc.
│   │   ├── node_map.rs         # node-name → matrix-index mapping
│   │   ├── ratings.rs          # check_ratings: absolute-maximum ratings vs. a result
│   │   ├── measure.rs          # measurements (.meas) and specs: check_specs, pass/fail and margin
│   │   ├── thermal.rs          # self-heating: DC balance iteration, transient thermal RC
│   │   ├── validation.rs       # circuit pre-flight checks
│   │   ├── error.rs            # SimError variants
│   │   ├── import/             # netlist-and-bench import for editors: Netlist, Bench, simulate, parse_value
│   │   ├── examples.rs         # named example circuits, runnable from list_examples / run_example
│   │   └── spice/              # SPICE3 netlist parser (optional `spice` feature)
│   │       └── {mod,parser,preprocess,flatten,build,ast,param_eval,si,
│   │            analysis,error,source_map}.rs
│   ├── examples/               # cargo run --example targets
│   └── tests/                  # integration tests (+ spice fixtures/)
├── sindr-devices/      # device physics (no solver dep)
│   └── src/{diode,bjt,mosfet,igbt,jfet,varactor,schottky,zener,led,
│            photodiode,photoresistor,thermistor}.rs
├── docs/RELEASING.md    # where capabilities are documented + release checklist
├── scripts/gen_llms_full.py  # regenerates llms-full.txt
├── llms.txt            # short index for LLMs (llmstxt.org)
├── llms-full.txt       # generated: READMEs, LIMITATIONS, public API, examples
└── .planning/          # local GSD workflow artefacts (gitignored) — IGNORE for normal code work
```

`sindr` depends on `sindr-core` and `sindr-devices`; `sindr-core` depends only on `sindr-devices` (plus `thiserror`, optional `serde`). `sindr` re-exports `sindr_core::{circuit, design, waveform}`, so `crate::circuit` inside `sindr` is the core module. Every public analysis entry point flattens a hierarchical `Circuit` first (`circuit.flattened()?`); the internal `mna`/`stamp`/`newton_raphson` code assumes a flat one. SPICE netlist parsing lives in `sindr::spice` behind the optional, default-off `spice` feature, which pulls in `winnow` + `miette`.

## Commands

```bash
cargo build                                   # workspace build
cargo test --workspace --all-features         # all unit, integration + doctests (CI)
cargo test --workspace --no-default-features  # serde/examples off (CI)
cargo test -p sindr                           # solver crate only
cargo test -p sindr-devices                   # device-physics crate only (includes README doctests)
cargo run --example voltage_divider           # run a named example
cargo run --example list_examples             # list every bundled example circuit
cargo doc --workspace --no-deps --all-features --open  # API docs (CI uses RUSTDOCFLAGS=-D warnings)
cargo clippy --workspace --all-targets --all-features -- -D warnings  # lint (CI)
cargo fmt --all                               # format
cargo bench -p sindr --features spice --bench transient --bench newton --bench analyses --bench spice -- --quick  # benchmarks (CI gates PRs at +25 %)
python3 scripts/gen_llms_full.py              # regenerate llms-full.txt (CI checks it)
```

See [`CONTRIBUTING.md`](./CONTRIBUTING.md) for the version-bump and CHANGELOG rule CI enforces on PRs.

Most tests live inline in `#[cfg(test)] mod tests` blocks; cross-cutting integration tests live in `sindr/tests/` (the SPICE end-to-end suite is gated on the `spice` feature).

## Conventions

- **Edition 2021, MSRV 1.89** (`rust-version` in both manifests; CI tests on 1.89 and stable). Don't use std/language features newer than 1.89.
- **No `unsafe`.** This is a numerical library; we have no reason for it.
- **No `unwrap()` / `expect()` in non-test code paths.** Return `SimError` instead.
- **Errors flow through `SimError`** (see `sindr/src/error.rs`). When adding a new failure mode, prefer extending the enum over `String` errors.
- **`f64` everywhere.** No generic numeric backend, no `f32`.
- **Serde is feature-gated** but on by default. Any new public type that is part of `Circuit` or `SimulationResult` must derive `Serialize`/`Deserialize` under `#[cfg(feature = "serde")]`. Component types use `snake_case` tags (see existing `#[serde(rename_all = "snake_case")]`).
- **Public API stability**: the crates are at `0.2.0`, plain 0.x until 1.0. A breaking change bumps the minor version (`0.3.0`), anything else the patch (`0.2.1`). Breaking changes are allowed, but call them out in the commit message and the CHANGELOG.
- **Comments**: lean. Don't restate what well-named code already says. Do explain *why* for non-obvious numerical choices (e.g. damping factors, why `k ≤ 0.999`, why `gmin` thresholds).
- **No GSD planning-artefact references in source.** Strip `EX-NN`, `Pitfall N`, `RESEARCH.md` / `PLAN.md` mentions. They belong in `.planning/`, not in code or commits.

## Adding a new circuit element

1. Add a variant to `CircuitElement` in `sindr-core/src/circuit.rs` (with serde tags), and to its `nodes`/`nodes_mut`/`id`/`id_mut` matches.
2. If it has device physics, add a module in `sindr-devices/src/` exposing a companion-model function.
3. Implement matrix stamping in `sindr/src/stamp.rs` (and Newton-Raphson contribution if nonlinear).
4. If it stores state across timesteps (charge, current), thread it through the transient state in `sindr/src/transient.rs`.
5. Extend `SimulationResult` (`sindr/src/results.rs`) only if the element produces results that don't fit `ComponentResult`.
6. Add a unit test that exercises a known analytical answer.
7. Document it in `sindr/README.md` under the appropriate section.
8. If it changes solver routing (forces transient / nonlinear), update the routing table in `sindr/README.md`.

## Adding a new device model in `sindr-devices`

1. New module under `sindr-devices/src/`.
2. Define a `Params` struct with sensible defaults via `Default` or a named constructor (e.g. `DiodeParams::silicon()`).
3. Companion-model function returns `(g_eq, i_eq)` for two-terminal devices, or a struct with all linearised conductances/currents for multi-terminal.
4. Make `Params` `#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]`.
5. Re-export from `sindr-devices/src/lib.rs`.
6. Test against an analytical or canonical reference point.

## Common pitfalls when generating circuits via this API

These trip up human users *and* code-generating agents. If you are producing example circuits or tests:

- **One node must equal `Circuit::ground_node`.** Otherwise → `SimError::NoGround`.
- **Every node must have a DC path to ground.** Floating sub-circuits (e.g. capacitor stack with no DC return) → `FloatingNode`. Add a large bleeder resistor if the topology calls for it.
- **Every node needs at least two connections.** A node reached by a single component (a dangling resistor end) is also `FloatingNode`.
- **No loops of voltage sources or inductors.** An inductor is a short at DC, so a loop of inductors (or of inductors and voltage sources) is `SingularMatrix`, in AC too, which solves the DC operating point first. Put a small resistor in the loop.
- **Transformer `k` must satisfy `k ≤ 0.999`.** `k = 1.0` is mathematically singular. Default is `0.999`.
- **Voltage source `nodes[0]` is the positive terminal.** Easy to flip and end up with negative supply rails.
- **Current source convention**: current flows from `nodes[0]` toward `nodes[1]`.
- **Op-amp / comparator**: nodes are `[in_plus, in_minus, out]`, output referred to ground. Without `model` (and for every comparator) it's a VCVS with gain `1e5`: no GBW, no slew rate, and `v_pos`/`v_neg` are **not** enforced. `model: Some(OpAmpModel { .. })` adds a GBW pole, slew rate, output resistance, offset, bias current and the rail clamp. A modelled op-amp takes two MNA branch slots (`CircuitElement::branch_slots`), its output current then its gain-stage voltage, and makes the circuit nonlinear (Newton). It doesn't make the circuit reactive, so `solve_circuit` stays DC.
- **BJT nodes**: `[base, collector, emitter]`. **MOSFET nodes**: `[gate, drain, source]`. **IGBT nodes**: `[gate, collector, emitter]`. Order matters; mismatched order silently produces wrong results.
- **Switches** are `0.01 Ω` closed / `1 GΩ` open — not ideal. In high-impedance circuits prefer explicit resistors.
- **Capacitors / inductors / varactors / transformers / waveforms automatically trigger transient analysis.** Don't expect a pure DC solve if any are present.
- **`solve_circuit` never runs AC.** For a frequency response call `sindr::ac_analysis::solve_ac` with an `AcConfig` naming the voltage source that carries the AC stimulus. For DC gain, input and output resistance (`.tf`) call `sindr::solve_tf`; for DC sensitivities (`.sens`), `sindr::solve_sensitivity`.
- **`solve_circuit` picks the transient span itself:** 5·τ_max and τ_max/50 from every R·C and L/R pair, with a 1 µs floor on dt (so τ below ~0.2 µs gives a single step). When the span matters, call `solve_transient` with a `TransientConfig`; for a SPICE deck, `AnalysisRequest::transient_config()` turns its `.tran` card into one.
- **Junction `temperature` fields are kelvin** (300.15 K = 27 °C, the nominal). IS and the thermal voltage kT/q both follow it.
- **PMOS `vto` is SPICE-signed:** negative for an enhancement PMOS, as `MosfetParams::default_pmos()` has it.
- **The default integrator is Gear-2:** second order, no ringing, slightly dissipative over many periods. For ringing amplitude in a resonant tank use `IntegrationMethod::Trapezoidal`, but not on circuits with time constants far below the step, where it rings. `IntegrationMethod::BackwardEuler` reproduces results from before alpha.7.
- **Voltage regulators drop out.** `VoltageRegulator { model: Some(RegulatorParams::lm7805()), .. }` needs about 2 V of headroom and draws its output plus 4.3 mA from the input; check `voltage_across` (`Vin − Vout`) and `power` on its result. `model: None` is ideal but can't boost.
- **Self-heating is opt-in per part:** set `Part::thermal` (`Thermal::junction_to_ambient(rθja)` or `Thermal::on_heatsink(rθjc, rθca)`). Then the element's `temperature` field is ignored, and `SimError::ThermalRunaway` means the design has no thermal balance (add degeneration or a heatsink, don't retry). Only junction devices and thermistors feel the heat; MOSFET/IGBT models don't.
- **Check ratings before calling a design done.** `check_ratings(&circuit, &result)` compares every part with the `Ratings` on its `Part`; for anything with switching or start-up, run it on a transient, since the DC operating point misses the peaks. `report.is_safe()` is false while any needed rating is missing, so fill them in rather than ignoring unverified checks.
- **Judge a design with specs, not by eye.** `check_specs(&result, &[Spec::new(Measurement::new("vout", Measure::stat(Signal::v("out"), Stat::Final))).between(4.9, 5.1)])` gives each spec's value, pass/fail and margin; attach `check_ratings` with `SpecReport::with_ratings`. `SpecStatus::Unmeasured` (an unknown node, a level never crossed) is never a pass: read its `reason`. Node and component names in a parsed SPICE deck are lowercase.
- **`ConvergenceFailed { iterations, max_step_volts, node, components, at }`**: `node` is where the circuit won't settle (the largest final Newton step), `components` what's on it, `at` the `SolvePoint` (operating point, DC or temperature sweep point, transient time); the message names all three. Usually a nonlinear element with no DC path to ground, or component values orders of magnitude outside typical ranges. `max_step_volts` is the max Newton step (not a KCL residual). The solver already retries with gmin stepping (`SolverOptions::gmin_steps`, `1e-2` → `1e-10` then none) and then source stepping before giving up; `SimulationResult::convergence_aid` says which one a solve needed. If neither rescues it, perturb the circuit, give a starting guess in `SolverOptions::nodeset`, or raise `max_iterations`, and run it through a `Solver` — don't blindly retry.

When in doubt, run `cargo run --example list_examples` and copy the closest existing example; the bundled circuits are known-good.

## Things NOT to do

- **Don't add `unsafe`.**
- **Don't add a new dependency without a clear need.** `nalgebra`, `faer` (sparse LU above 100 unknowns), `serde`, `thiserror` are the load-bearing ones; `winnow` + `miette` back the SPICE parser and are pulled in only by the optional `spice` feature.
- **Don't introduce a CLI or GUI in `sindr`, `sindr-core` or `sindr-devices`** — those are explicit non-goals; propose a sibling crate instead. Schematic and PCB tools are meant to build on `sindr-core`'s design model (blocks, instances, parts) from their own crates, not inside these. Heavy *optional* capabilities (like SPICE netlist parsing) belong behind a default-off feature flag with their deps marked `optional`, following the `spice` precedent.
- **Don't implement BSIM / Gummel-Poon / VBIC** speculatively. They are large undertakings; coordinate via an issue first.
- **Don't bypass `SimError`** with `panic!` / `unwrap()` in solver code.
- **Don't write to `.planning/`** unless you are explicitly running a GSD command. It is workflow state, not source.
- **Don't edit `llms-full.txt` by hand.** It's generated: `python3 scripts/gen_llms_full.py`. CI fails if it's stale.
- **Don't commit `target/`, `*.swp`, or editor scratch files.** `.gitignore` covers the basics; check `git status` before committing.
- **Don't squash existing public-API types into private ones** without a deprecation note in the commit message.

## Commit & PR conventions

- Each release is staged on a `v<version>` branch (e.g. `v0.3.0`) that already carries the version bump. Branch off it and open PRs against it, not `main`; the staging branch merges into `main` when the release ships.
- One commit per logical change. Squash GSD's atomic per-task commits at end of phase.
- Commit message: imperative, lowercase first word, no trailing period in subject.
- Co-author trailer is fine; do not add hype-marketing footers.
- Run `cargo fmt --all` and `cargo clippy --workspace --all-targets --all-features -- -D warnings` before committing.

## Where to look first for common questions

| Question | File |
|----------|------|
| "What components exist?" | `sindr-core/src/circuit.rs` (`CircuitElement` enum) |
| "What does the solver do for circuit X?" | routing table in `sindr/README.md`, plus `sindr/src/lib.rs::solve_circuit` |
| "How is element Y stamped?" | `sindr/src/stamp.rs` |
| "Why did this fail?" | `sindr/src/error.rs` (`SimError` variants) |
| "What does sindr *not* do?" | [`LIMITATIONS.md`](./LIMITATIONS.md) |
| "What's a known-good example circuit?" | `sindr/src/examples.rs` and `sindr/examples/` |
