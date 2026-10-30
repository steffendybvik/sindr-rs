# AGENTS.md — orientation for AI coding agents

This file briefs an AI coding assistant (Claude Code, Codex, Cursor, etc.) working on `sindr-rs`. Read this before making changes. For human-facing capability docs see [`README.md`](./README.md); for what sindr **cannot** do see [`LIMITATIONS.md`](./LIMITATIONS.md).

## What this repo is

A pure-Rust analog circuit simulator. SPICE-style MNA (Modified Nodal Analysis) solver plus semiconductor companion models. Library only — no CLI, no GUI, no `no_std`. Pre-release alpha.

## Workspace layout

```
sindr-rs/
├── sindr/              # MNA solver crate (depends on sindr-devices)
│   ├── src/
│   │   ├── lib.rs              # public API surface, re-exports
│   │   ├── circuit.rs          # CircuitElement enum — every supported component
│   │   ├── mna.rs              # MNA matrix assembly
│   │   ├── stamp.rs            # per-element matrix stamping
│   │   ├── newton_raphson.rs   # nonlinear DC solver
│   │   ├── transient.rs        # transient stepper (TransientConfig, solve_transient)
│   │   ├── integration.rs      # Gear-2 / trapezoidal / backward-Euler formulas
│   │   ├── ac_analysis.rs      # small-signal AC
│   │   ├── dc_sweep.rs         # parameter sweep
│   │   ├── temp_sweep.rs       # temperature sweep
│   │   ├── waveform.rs         # Waveform enum (sine, square, pulse, sawtooth)
│   │   ├── results.rs          # SimulationResult, ComponentResult, etc.
│   │   ├── node_map.rs         # node-name → matrix-index mapping
│   │   ├── validation.rs       # circuit pre-flight checks
│   │   ├── error.rs            # SimError variants
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

`sindr` depends on `sindr-devices`. `sindr-devices` has no dependency on `sindr` and is independently testable. SPICE netlist parsing lives in `sindr::spice` behind the optional, default-off `spice` feature, which pulls in `winnow` + `miette`.

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
python3 scripts/gen_llms_full.py              # regenerate llms-full.txt (CI checks it)
```

See [`CONTRIBUTING.md`](./CONTRIBUTING.md) for the version-bump and CHANGELOG rule CI enforces on PRs.

Most tests live inline in `#[cfg(test)] mod tests` blocks; cross-cutting integration tests live in `sindr/tests/` (the SPICE end-to-end suite is gated on the `spice` feature).

## Conventions

- **Edition 2021, MSRV 1.87** (`rust-version` in both manifests; CI tests on 1.87 and stable). Don't use std/language features newer than 1.87.
- **No `unsafe`.** This is a numerical library; we have no reason for it.
- **No `unwrap()` / `expect()` in non-test code paths.** Return `SimError` instead.
- **Errors flow through `SimError`** (see `sindr/src/error.rs`). When adding a new failure mode, prefer extending the enum over `String` errors.
- **`f64` everywhere.** No generic numeric backend, no `f32`.
- **Serde is feature-gated** but on by default. Any new public type that is part of `Circuit` or `SimulationResult` must derive `Serialize`/`Deserialize` under `#[cfg(feature = "serde")]`. Component types use `snake_case` tags (see existing `#[serde(rename_all = "snake_case")]`).
- **Public API stability**: the crate is `0.1.0-alpha.7` — breaking changes are allowed, but call them out in the commit message.
- **Comments**: lean. Don't restate what well-named code already says. Do explain *why* for non-obvious numerical choices (e.g. damping factors, why `k ≤ 0.999`, why `gmin` thresholds).
- **No GSD planning-artefact references in source.** Strip `EX-NN`, `Pitfall N`, `RESEARCH.md` / `PLAN.md` mentions. They belong in `.planning/`, not in code or commits.

## Adding a new circuit element

1. Add a variant to `CircuitElement` in `sindr/src/circuit.rs` (with serde tags).
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
- **Op-amp / comparator**: nodes are `[in_plus, in_minus, out]`. Macromodel is a saturating VCVS with gain `1e5` — no GBW, no slew rate. Don't try to predict realistic closed-loop bandwidth.
- **BJT nodes**: `[base, collector, emitter]`. **MOSFET nodes**: `[gate, drain, source]`. **IGBT nodes**: `[gate, collector, emitter]`. Order matters; mismatched order silently produces wrong results.
- **Switches** are `0.01 Ω` closed / `1 GΩ` open — not ideal. In high-impedance circuits prefer explicit resistors.
- **Capacitors / inductors / varactors / transformers / waveforms automatically trigger transient analysis.** Don't expect a pure DC solve if any are present.
- **`solve_circuit` never runs AC.** For a frequency response call `sindr::ac_analysis::solve_ac` with an `AcConfig` naming the voltage source that carries the AC stimulus.
- **`solve_circuit` picks the transient span itself:** 5·τ_max and τ_max/50 from every R·C and L/R pair, with a 1 µs floor on dt (so τ below ~0.2 µs gives a single step). When the span matters, call `solve_transient` with a `TransientConfig`; for a SPICE deck, `AnalysisRequest::transient_config()` turns its `.tran` card into one.
- **Junction `temperature` fields are kelvin** (300.15 K = 27 °C, the nominal). IS and the thermal voltage kT/q both follow it.
- **PMOS `vto` is SPICE-signed:** negative for an enhancement PMOS, as `MosfetParams::default_pmos()` has it.
- **The default integrator is Gear-2:** second order, no ringing, slightly dissipative over many periods. For ringing amplitude in a resonant tank use `IntegrationMethod::Trapezoidal`, but not on circuits with time constants far below the step, where it rings. `IntegrationMethod::BackwardEuler` reproduces results from before alpha.7.
- **`ConvergenceFailed { iterations, max_step_volts }`**: usually a nonlinear element with no DC path to ground, or component values orders of magnitude outside typical ranges. `max_step_volts` is the max Newton step (not a KCL residual). The solver already retries with gmin stepping (homotopy ladder `1e-2` → `1e-12`) before giving up; no source stepping. If gmin stepping doesn't rescue it, perturb the circuit or pass a starting guess via `solve_circuit_with_initial_voltages` — don't blindly retry.

When in doubt, run `cargo run --example list_examples` and copy the closest existing example; the bundled circuits are known-good.

## Things NOT to do

- **Don't add `unsafe`.**
- **Don't add a new dependency without a clear need.** `nalgebra`, `serde`, `thiserror` are the load-bearing ones; `winnow` + `miette` back the SPICE parser and are pulled in only by the optional `spice` feature.
- **Don't introduce a CLI or GUI in `sindr` or `sindr-devices`** — those are explicit non-goals; propose a sibling crate instead. Schematic capture and EDA-file import are likewise out of scope. Heavy *optional* capabilities (like SPICE netlist parsing) belong behind a default-off feature flag with their deps marked `optional`, following the `spice` precedent.
- **Don't implement BSIM / Gummel-Poon / VBIC** speculatively. They are large undertakings; coordinate via an issue first.
- **Don't bypass `SimError`** with `panic!` / `unwrap()` in solver code.
- **Don't write to `.planning/`** unless you are explicitly running a GSD command. It is workflow state, not source.
- **Don't edit `llms-full.txt` by hand.** It's generated: `python3 scripts/gen_llms_full.py`. CI fails if it's stale.
- **Don't commit `target/`, `*.swp`, or editor scratch files.** `.gitignore` covers the basics; check `git status` before committing.
- **Don't squash existing public-API types into private ones** without a deprecation note in the commit message.

## Commit & PR conventions

- Each release is staged on a `v<version>` branch (e.g. `v0.1.0-alpha.7`) that already carries the version bump. Branch off it and open PRs against it, not `main`; the staging branch merges into `main` when the release ships.
- One commit per logical change. Squash GSD's atomic per-task commits at end of phase.
- Commit message: imperative, lowercase first word, no trailing period in subject.
- Co-author trailer is fine; do not add hype-marketing footers.
- Run `cargo fmt --all` and `cargo clippy --workspace --all-targets --all-features -- -D warnings` before committing.

## Where to look first for common questions

| Question | File |
|----------|------|
| "What components exist?" | `sindr/src/circuit.rs` (`CircuitElement` enum) |
| "What does the solver do for circuit X?" | routing table in `sindr/README.md`, plus `sindr/src/lib.rs::solve_circuit` |
| "How is element Y stamped?" | `sindr/src/stamp.rs` |
| "Why did this fail?" | `sindr/src/error.rs` (`SimError` variants) |
| "What does sindr *not* do?" | [`LIMITATIONS.md`](./LIMITATIONS.md) |
| "What's a known-good example circuit?" | `sindr/src/examples.rs` and `sindr/examples/` |
