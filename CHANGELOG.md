# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0-alpha.7] - Unreleased

### Added

- Sparse MNA solve: systems above 100 unknowns are stored sparsely and
  factorised with `faer`'s sparse LU, reusing the symbolic analysis while the
  sparsity pattern doesn't change, for DC, transient and AC alike. A
  10 000-node ladder solves in ~4 ms; dense took ~80 ms at 1 000 nodes.
  Benchmark: `cargo bench -p sindr --bench mna_solve`.
- Source stepping, the DC fallback after gmin stepping: every source ramped
  up from zero, warm-starting each step. `SimulationResult::convergence_aid`
  reports which fallback (`GminStepping` or `SourceStepping`) a solve needed.
- Transient initial conditions (`.ic` with `UIC`):
  `TransientConfig::with_initial_voltages` and `with_initial_current`.
- `ac_analysis::solve_ac_with_initial_voltages`: a `.nodeset` for the AC bias
  point.
- `sindr::spice` reads `.ic V(node)=…` and `.nodeset V(node)=…` into
  `ParsedNetlist::initial_conditions` and `nodesets`.
- Transient timestep control: steps are shortened where the local truncation
  error would exceed SPICE's default tolerances, and land exactly on every
  corner of a source waveform (`Waveform::breakpoints`), restarting the
  integration there. `TransientData::stats` counts accepted, LTE-rejected and
  Newton-rejected steps; `TransientConfig::with_fixed_step()` opts out.
- Trapezoidal and Gear-2 (BDF2) transient integration, chosen with
  `TransientConfig::with_method(IntegrationMethod::…)`. Both are second
  order; trapezoidal keeps an LC tank's amplitude, Gear-2 doesn't ring on
  fast time constants.
- `sindr::solve_transient(circuit, &TransientConfig)`: transient analysis
  over a chosen span and step (`TransientConfig::new(tstop, tstep)`,
  optionally `.with_tmax(…)`), as SPICE's `.tran`. `solve_circuit` keeps
  choosing them from the circuit's time constants (`TransientConfig::auto`).
- `spice::AnalysisRequest::transient_config()` turns a parsed `.tran` card
  into a `TransientConfig`.
- Crate docs list sindr's capabilities: analyses with their entry points,
  device models, convergence aids and the supported SPICE subset. On docs.rs
  the `spice` and `examples` items carry feature badges.
- `llms.txt` and a generated `llms-full.txt` for LLMs and agents, and
  `docs/RELEASING.md`, the per-release documentation checklist.
- Examples `temperature_sweep` and `initial_voltages`.
- `sindr_devices::bjt::{bjt_terminal_currents, bjt_terminal_jacobian, BjtJacobian}`:
  the BJT's large-signal terminal currents (Early effect included) and their
  exact, uncapped Jacobian, valid in every operating region.
- `MosfetParams::in_nmos_frame`: a device's parameters in the N-channel frame
  that `mosfet_companion` works in.
- `sindr_devices::diode::{thermal_voltage, K_OVER_Q, T_NOM}` and a `vt()`
  method on `DiodeParams`, `BjtParams`, `ZenerParams` and
  `PhotodiodeParams`: the thermal voltage kT/q at the junction temperature.

### Changed

- **Breaking:** `MnaSystem::a` is an `MnaMatrix` (dense or sparse) instead of
  a nalgebra `DMatrix`. Index it as before (`a[(r, c)]`); use
  `MnaMatrix::to_dense()` for a `DMatrix`.
- Transient snapshots are the accepted steps, so their number varies: every
  multiple of `tstep` is still there, plus shorter steps near edges and
  after switch-on.
- Transients integrate with **Gear-2 by default**, including through
  `solve_circuit`, instead of backward Euler. Results are much closer to the
  exact solution (0.03 % against ngspice where BE was 0.4 %) and change
  accordingly. `IntegrationMethod::BackwardEuler` reproduces the old ones.
- **Breaking:** `sindr::transient::solve_transient` now takes
  `(&Circuit, &TransientConfig)`, and `solve_transient_nonlinear` is gone:
  linear and nonlinear circuits share one stepper.
- Transient steps evaluate sources at the step's end time, as backward Euler
  requires. Before, sources were sampled at the step's start, so waveforms
  lagged by one step. The run now ends exactly at its duration instead of one
  step past it.
- A nonlinear transient keeps the requested step. It used to double the step
  after five easy Newton solves, up to 10× (with no accuracy check), and
  report the original step in `TransientData::time_step`. Newton now starts
  each step from the previous solution instead of 0 V.
- **Breaking:** a PMOS's `MosfetParams::vto` is now SPICE-signed: negative
  for an enhancement PMOS, as `MosfetParams::default_pmos()` already was.
  Before, the solver expected a positive magnitude, so `default_pmos()` gave a
  normally-on device that conducted at V_SG = 0. If you set a positive `vto`
  on a PMOS, negate it.
- **Breaking:** junction models now use the thermal voltage at their own
  `temperature` field instead of a fixed constant. `temperature_scale_is`
  takes the emission coefficient `N` as a sixth argument; `critical_voltage`
  and `limit_voltage` take the thermal voltage as an extra last argument;
  `ZenerParams` and `PhotodiodeParams` gain a `temperature` field.
- `V_T` is now kT/q at 300.15 K (0.0258652 V), the temperature devices
  default to. It was 0.025851 V, kT/q at 300 K rounded, which shifted every
  junction voltage by about 5e-4 relative. Diode IS now scales with SPICE's
  default XTI = 3 (was 2).
- A MOSFET's body is now tied to its source, as in a discrete three-terminal
  part, so `Vbs = 0` and there is no body effect. Before, the body sat at
  ground: an NMOS with its source above ground (a source follower, a
  high-side switch) had its threshold raised through `gamma`, and a PMOS with
  its source at VDD had it lowered. Results change for any MOSFET whose
  source isn't at ground.

### Fixed

- Transformers in transient modelled negative inductance (the winding rows
  had V = −L·dI/dt). At the large steps sindr used to take, the unstable mode
  this created happened to look settled; at short steps the secondary blew
  up. A winding is now V = L·dI/dt, so it is a short at DC.
- Varactors in transient: the companion injected its history current with
  the wrong sign, so a varactor charged through a resistor settled about
  1e-3 short of the source voltage.
- A nonlinear transient with a `VoltageRegulator` listed before a
  `Transformer` read the transformer's winding currents from the wrong rows,
  so the result depended on component order.
- AC analysis linearises IGBTs at their operating point. Before, an IGBT was
  an open circuit in AC, so a gate-driven IGBT stage showed no gain.
- Junction temperature scaling. `temperature_scale_is` multiplied the bandgap
  exponent by XTI, so IS was far off away from 27 °C (a BJT's IS at 350 K was
  1.5e-6 A instead of 7e-12 A), and the thermal voltage stayed at its 300 K
  value. A silicon diode's forward drop now falls by about 1.7 mV/K as it
  warms, matching ngspice to 1e-6 from −23 °C to 127 °C.
- `sindr_devices::diode::diode_companion` and `diode_current` with series
  resistance (`DiodeParams.rs > 0`): the junction voltage is now solved from
  the implicit diode equation, and the companion returns the terminal
  conductance `g_j / (1 + Rs·g_j)`. Before, a forward-biased diode with Rs was
  treated as off.
- Diodes, LEDs, Zener and Schottky diodes, photodiodes and BJTs away from
  300.15 K are now evaluated with the same temperature-scaled `IS` everywhere:
  AC linearisation, NR voltage limiting, transient stamping, and the reported
  DC and transient component currents. Before, several of these used the
  nominal-temperature model, so (for example) a diode's `current_through`
  didn't match the operating point that was solved. Photodiode dark current
  now scales with `temperature` too, as `temperature_sweep` documents.
- AC analysis now biases at the DC operating point (capacitors open,
  inductors shorted). Before, any circuit with a capacitor, inductor or
  waveform source was linearised at the transient's 5τ state, ~0.7 % short of
  the bias for a coupling capacitor.
- AC analysis now has small-signal models for JFETs (N and P), Zener,
  Schottky and photodiodes, and varactors (their capacitance at the bias).
  Before, these were open circuits in AC, so e.g. a JFET amplifier had no
  gain.
- SPICE errors raised after parsing now point at the offending line:
  undefined models and parameters, parameter cycles, and bad analysis
  directive values. Before, they carried no source text and a span at the
  start of the file. Lines inside `.include`d files point into those files.
- `solve_ac` validates its `AcConfig` and returns
  `SimError::InvalidComponent` for 0 points, non-finite values,
  `f_start > f_stop`, `f_start <= 0` with logarithmic spacing, and
  `f_start < 0` with linear spacing. Before, these panicked or produced NaN
  frequencies. A 1-point sweep now solves `f_start` alone, as SPICE does.
- `dc_sweep` rejects non-finite endpoints, and `temperature_sweep` rejects
  non-finite temperatures and temperatures at or below 0 K.
- The BJT DC stamp is now the exact Newton linearisation of its currents.
  Junction conductances were capped at 10 S and kept growing past the
  exponential clamp, so above ~0.8 V of Vbe the Jacobian no longer matched the
  current and Newton slowed to a crawl. The stamp now uses
  `bjt_terminal_jacobian`, the same model as AC analysis, Early effect
  included; `bjt_companion` conductances are exact slopes.
- P-channel JFETs were stamped and reported with the Norton current of the
  mirrored N-channel frame, so DC, transient and the reported drain current
  were wrong. A P-channel stage is now the exact mirror of its N-channel twin.
- DC operating points with a forward-biased Schottky diode or photodiode, or
  any diode next to a BJT, MOSFET or Zener, failed with `ConvergenceFailed`.
  The initial guess left the whole supply across the junction; junctions now
  start at their critical voltage, as in SPICE.
- Diodes in series (two or more junctions sharing a node) failed DC with
  `SingularMatrix`. Voltage limiting corrected one junction at a time, so the
  next junction's correction on the shared node pushed the first one volts
  forward. When that happens the solver now takes a damped Newton step that
  keeps every junction within its limit. Only a junction that moved forward
  triggers it, so a junction stepping back keeps its own limit (and a BJT
  its ±10·nV_T step clamp).
- AC analysis now linearises BJTs with the full terminal Jacobian of the DC
  model (gπ, gm, gμ and g_o), instead of a forward-active approximation that
  used the B–C junction conductance as the output conductance. Gains in
  saturation and reverse-active were wrong, in saturation even in sign.
- AC analysis now applies the BJT's junction temperature (IS scaling) as the
  DC solve does.
- MOSFET body-effect transconductance `gmb` is now zero once the body is
  forward-biased past PHI, matching `threshold_voltage`, which stops
  depending on Vbs there. Before, a clamp made `gmb` about 200× `gm`. Every
  PMOS with its source above ground hit this, because its body sits at
  ground, and Newton got a source-column Jacobian entry ~200× too large.
- IGBT operating points were wrong whenever `Vge ≠ Vce`. The stamp put `gm`
  in as a collector–emitter conductance while `i_eq` linearised it around
  `Vge`, so Newton settled on a point that isn't a solution: a switch driven
  at 10 V into 100 Ω from 12 V read `Vce = 7.5 V` instead of 4.8 mV. `gm` is
  now stamped as a gate-controlled source, and `IgbtCompanion::g_ce` is the
  real `∂Ids/∂Vce` (zero in saturation) instead of `ids / vce_sat`. A
  forward-biased IGBT's `Vce` is now limited per Newton step so it can't jump
  into reverse blocking. `IgbtParams::vce_sat` is no longer used.
- Newton–Raphson (DC and transient) now also requires every diode, LED and
  BJT junction voltage to settle within `V_ABSTOL + RELTOL·n·V_T` before
  declaring convergence. The node-voltage tolerance alone scales with a
  node's absolute voltage, so junctions far from ground could stop several
  mV short: a diode near a 10 V rail was off by 1.2% in current, a
  saturated PNP near 5 V by 1.5%. Costs about 9% more iterations.

## [0.1.0-alpha.6] - 2026-06-03

### Added

- SPICE3 netlist parsing, now built in to `sindr` behind the optional `spice`
  feature (`sindr = { version = "0.1", features = ["spice"] }`). Exposes
  `sindr::spice::{parse_file, parse_str, ParsedNetlist, AnalysisRequest, …}`.
  Reads a practical subset of Berkeley SPICE3f5 decks (R, L, C, V, I, D, Q, X,
  `.subckt`, `.model`, `.tran`, `.dc`, `.ac`, `.op`, `.param`, `.include`,
  `.lib`), flattens hierarchy with `.`-separated node naming, and returns a
  `SourceMap` for recovering original instance paths. Strict by default;
  lenient mode downgrades unsupported cards to collected warnings.

### Changed

- The standalone `sindr-spice` crate has been folded into `sindr` as the
  `spice` feature. The parser dependencies (`winnow`, `miette`) are pulled in
  only when the feature is enabled, so the default dependency tree is
  unchanged. Migration: replace `sindr_spice::` with `sindr::spice::` and drop
  the separate `sindr-spice` dependency in favour of the feature.

## [0.1.0-alpha.5] - 2026-05-10

### Added

- `solve_circuit_with_initial_voltages` — SPICE-style `.NODESET` seeding for
  nonlinear DC solves. Pass a `HashMap<String, f64>` to pre-warm the Newton
  initial guess.
- Gmin-stepping homotopy fallback for nonlinear DC. When plain Newton–Raphson
  fails to converge, the solver now automatically retries with a geometric
  ladder of gmin shunts (`1e-2` → `1e-12`), warm-starting between steps.
- Module-level `//!` documentation on `circuit`, `mna`, `stamp`, `node_map`,
  `transient`, `results`, `validation`. Spelled out `[anode, cathode]` on
  `Diode` / `Led` and clarified `Capacitor` / `Inductor` polarity.
- Math-correctness integration test suite (`sindr/tests/math_correctness.rs`):
  KCL residual, Shockley-equation, power-balance, and seed/gmin-path
  agreement checks across linear and nonlinear DC. Two known limitations
  locked in as regression tests (supply-rail seed; series-diode singular
  Jacobian).

### Changed

- **Breaking:** `SimError::ConvergenceFailed` now carries `iterations: usize`
  and `max_step_volts: f64` fields with the iteration count and largest
  per-node Newton step at the point of failure. (`max_step_volts` is a step
  magnitude, not a KCL residual.) Match arms must be updated:
  `ConvergenceFailed { .. }` instead of `ConvergenceFailed`.
- README "Status" line corrected — the crate is published on crates.io.

## [0.1.0-alpha.4] - 2026-05-08

### Changed

- README tagline + crate description for `sindr` rewritten to
  "Rust circuit simulator. SPICE-style MNA solver with built-in
  semiconductor device models." (replaces "first-class semiconductor
  models", which overclaimed SPICE-grade fidelity).

### Removed

- **Breaking:** `SimError::UnsupportedCircuit(String)` variant. It was
  declared but never produced by the solver — pure dead code. Downstream
  pattern matches against this variant must be removed. If a future
  topology check needs to reject an unsupported configuration, a
  variant will be reintroduced with a real call site and a triggering
  test.

### Added

- README badges for CI, crates.io, docs.rs, and licence.
- `CLAUDE.md` at the repo root pointing to `AGENTS.md` as the canonical
  agent-orientation document.
- `.github/dependabot.yml` for weekly cargo + GitHub Actions updates.

## [0.1.0-alpha.3] - 2026-05-07

### Changed

- Declared MSRV (`rust-version = "1.87"`) on both crates. Required by
  `nalgebra 0.34.2` (which declares MSRV 1.87) and the transitive
  `nalgebra-macros 0.3.0` (which uses Rust edition 2024, requiring
  Cargo 1.85+).
- Pinned workspace path dependencies with explicit `version` fields so
  `cargo publish` can resolve `sindr-devices` to a registry coordinate
  when publishing `sindr`.
- One-time `rustfmt` sweep across the workspace.

### Added

- `[package.metadata.docs.rs]` blocks in both crates so docs.rs renders
  feature-gated items with `--cfg docsrs`.
- `NOTICE` shipped inside each crate's package (the root `NOTICE` is not
  picked up by `cargo package`).
- `CHANGELOG.md` (Keep a Changelog format).
- GitHub Actions CI workflow: `cargo fmt --check`, `cargo clippy -D warnings`,
  test matrix on stable + MSRV with `--all-features` and
  `--no-default-features`, `cargo doc -D warnings`.
- Quick voltage-divider example in the README.

## [0.1.0-alpha.2] - 2026-05-07

### Fixed

- Transient timestep array values were off-by-one against their time labels.
  Each snapshot is now stamped with its post-step time, so `(t, V)` pairs from
  `transient.timesteps` plot correctly. ([#4], [#5])

### Changed

- `transient.timesteps` no longer includes a `t=0` entry; samples now run from
  `dt` to slightly past `duration`. The math of the simulation itself was
  already correct — only the time-axis labelling changed.

## [0.1.0-alpha.1] - 2026-05

### Added

- Initial pre-release. MNA solver with Newton-Raphson convergence, transient
  (Backward Euler), AC analysis, DC sweep, temperature sweep. SPICE-style
  semiconductor models (diode, BJT, MOSFET, JFET, IGBT, varactor, zener,
  Schottky, LED, photodiode, photoresistor, thermistor).

[Unreleased]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.7...HEAD
[0.1.0-alpha.7]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.6...v0.1.0-alpha.7
[0.1.0-alpha.6]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.6
[0.1.0-alpha.5]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.5
[0.1.0-alpha.4]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.4
[0.1.0-alpha.3]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.3
[0.1.0-alpha.2]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.2
[0.1.0-alpha.1]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.1
[#4]: https://github.com/steffendybvik/sindr-rs/issues/4
[#5]: https://github.com/steffendybvik/sindr-rs/pull/5
