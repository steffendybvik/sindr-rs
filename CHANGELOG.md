# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0-alpha.8] - Unreleased

### Added

- Transient and Newton benchmarks (`cargo bench -p sindr --bench transient
  --bench newton`), with a CI job that fails a PR more than 25 % slower than
  its base, and a Performance section in the README with the numbers.
- SPICE `.model` cards now set device parameters: `IS`, `N`, `RS` on `D`
  cards, and `IS`, `BF`, `BR`, `NF`, `NR`, `VAF` (alias `VA`) on `NPN`/`PNP`
  cards. Any other card parameter is an error in strict mode and a warning
  per card in lenient mode, naming it; before, every parameter but `BF`
  was dropped silently, even in strict mode.
- ngspice goldens for nonlinear transient and device AC:
  `tran_halfwave` (sine into a rectifier with a filter cap),
  `tran_bjt_switch` (pulse through an RC into an NPN switch),
  `tran_rlc_ring`, `ac_diode` (a biased 1N4148 with its charge),
  `ac_mosfet_cs`, `ac_bjt_caps` and `ac_jfet`.
- `DiodeModel` (IS, N, RS) and `BjtModel` (IS, BR, NF, NR, VAF), as the
  optional `model` field of `CircuitElement::Diode` and `CircuitElement::Bjt`.
  DC, AC, transient, `.tf`, `.sens` and results all use them, so a diode's
  series resistance can now be set from a `Circuit`. Validation rejects
  non-positive IS, N, BR, NF, NR and negative RS or VAF.
- SPICE netlist export: `sindr::spice::to_netlist(&circuit, title)` writes a
  `Circuit` as a SPICE3 deck that ngspice runs and `parse_str` reads back,
  with `ExportError` for what it can't express. Covers R, L, C, V/I
  (square, triangle and PWM as the equivalent `PULSE`), D and Q with
  `.model` cards, E/G/F/H, transformers as two L and a K, and exact
  equivalents for switches, pushbuttons, fuses, potentiometers,
  photoresistors, thermistors, LEDs and Schottky diodes. Every built-in
  example it can write round-trips to the same solution, and ngspice
  goldens generated from exported decks (`op_exported_controlled`,
  `ac_exported_transformer`) pin the controlled-source signs.
- ngspice goldens `dc_diode_1n4148` (a 1N4148 card with `RS`) and
  `op_bjt_card`.
- Diode charge storage and breakdown from `.model D` cards: `CJO` (alias
  `CJ0`, `CJ`), `VJ` (`PB`), `M` (`MJ`), `FC`, `TT`, `BV` and `IBV`, as new
  `DiodeModel` fields. The junction charge (depletion plus `TT·I_d`
  diffusion) is a charge-based companion in transient, so a diode now shows
  reverse recovery, and the diode's result current includes its charging
  current. In AC the diode is RS in series with `g_j + jωC_j`. Breakdown is
  SPICE's exponential with ngspice's `BV`/`IBV` knee fit, in every analysis,
  with Newton limiting past the knee. A diode that stores charge makes a
  circuit reactive, so `solve_circuit` runs it as a transient. Export writes
  the new parameters when they're set. Validation rejects negative CJO or TT,
  non-positive VJ, BV or IBV, and M above 0.9 or FC above 0.95.
  `sindr_devices::diode` gains `Breakdown`, `JunctionCharge`,
  `junction_current`, `diode_charge` and `junction_small_signal`;
  `DiodeParams` has a `breakdown` field (breaking for struct literals).
- ngspice goldens `tran_diode_recovery` (1N4148 reverse recovery),
  `cv_1n4148` and `cv_1n5819` (small-signal admittance across bias) and
  `dc_diode_breakdown`.
- New crate `sindr-core`: the data model (`Circuit`, `CircuitElement`,
  `Waveform`) with no solver dependencies, so design tools can use it alone.
  `sindr` re-exports it, so `sindr::Circuit` paths still work.
- Design hierarchy on `Circuit`: `blocks` (`Block`: ports, components,
  instances), `instances` (`Instance`), `globals`, and `parts` (`Part`:
  reference, value, symbol, footprint, free-form fields, ignored by the
  simulator). `Circuit::flatten` expands instances with `.`-joined names;
  every analysis entry point flattens first. `Circuit::new`,
  `Circuit::nets`, and `CircuitElement::id_mut`/`nodes_mut` for renaming.
- Op-amp macromodel: `OpAmpModel` as the optional `model` of
  `CircuitElement::OpAmp`. It has open-loop gain, a dominant pole from the
  gain-bandwidth product, slew-rate limiting, output resistance, input
  offset and bias current, and an output clamped to `v_pos`/`v_neg` with
  anti-windup. DC, AC, transient, `.tf` and `.sens` use it; a closed-loop
  −3 dB point lands at GBW / gain and a large step rises at the slew rate.
  Without a model the op-amp is unchanged. `CircuitElement::branch_slots`
  gives each element's MNA branch unknowns.
- SPICE import keeps `.subckt` structure as blocks and instances; a subckt
  whose instances bind different `.param` values becomes one block per
  distinct body (`name`, `name_2`, …). Export writes blocks as
  `.subckt`/`.ends` and instances as `X` lines, and round-trips.
- Absolute-maximum ratings checks: `Part::ratings` (`Ratings`: power,
  voltage, current, diode reverse voltage and forward current, BJT/IGBT
  Vce/Vcb/Veb/Ic, MOSFET/JFET Vds/Vgs/Id), and `check_ratings(&circuit,
  &result)`, which reports each part's worst value, when it happened, the
  limit and the margin, at the operating point or at every transient step.
  A rating a part needs but doesn't set is unverified, never a pass.
- Self-heating: `Part::thermal` (`Thermal`: `rth_ja`, or `rth_jc` + `rth_ca`
  for a heatsink, optional `cth`, `ambient`). `solve_circuit` iterates the
  DC operating point until each part's junction temperature matches
  `ambient + P·Rθ`, and `solve_transient` carries a thermal RC per part. The
  temperature feeds back into diodes, LEDs, Zeners, Schottkys, photodiodes,
  BJTs and thermistors. `junction_temperatures` on `SimulationResult` and
  `TimestepSnapshot`, `Ratings::tj_max` checked against them, and
  `SimError::ThermalRunaway` when no balance exists. `temperature_sweep`
  sets the ambient of self-heated parts.
- Linear regulator model: `VoltageRegulator` takes an optional `model`
  (`RegulatorParams`: dropout, dropout resistance, quiescent current,
  current limit, line regulation, output resistance), with
  `RegulatorParams::lm7805()` and `::ams1117()` datasheet presets. AC
  passes input ripple through line regulation. Ratings checks need its
  input voltage, output current and power.
- Measurements and specs (`sindr::measure`). A `Measurement` takes one
  number from a transient, AC, DC sweep, temperature sweep or operating
  point result: the value at an x, min, max, average, RMS, peak-to-peak,
  integral or final value, a crossing (`WHEN`), a delay (`TRIG`/`TARG`),
  a 10–90 % rise or fall time, a settling time or a −3 dB point. A `Spec`
  adds limits. `check_specs` reports each spec's value, limits, pass/fail
  and margin, with the reason when it can't be measured, and
  `SpecReport::with_ratings` attaches the ratings checks.
- SPICE `.meas` / `.measure` cards (`tran`, `ac`, `dc`: `AVG`, `RMS`,
  `MIN`, `MAX`, `PP`, `INTEG`, `FIND … AT`, `WHEN`, `TRIG … TARG`, with
  `FROM`/`TO`) are parsed into `ParsedNetlist::measurements`.

- CSV export: `to_csv()` on `TransientData`, `AcResult`, `DcSweepResult`
  and `TempSweepResult`. One row per point, with the scale first, then
  `v(<node>)` and `i(<id>)` columns (`re(…)`/`im(…)` for AC). Numbers read
  back exactly. No new dependency.
- Transient start from the DC operating point: `TransientConfig::start`
  and `with_start(TransientStart::OperatingPoint)`. Capacitors start
  charged, and inductors and transformer windings carry their DC currents,
  with sources at their t = 0 value and any `.ic` nodes and inductor
  currents held while it's solved, as SPICE's `.tran` without `UIC`. The
  default stays `TransientStart::InitialConditions` (switched on at t = 0),
  so existing runs don't change.
- SPICE `.tran` reads `tmax` and `uic` (`.tran tstep tstop [tstart [tmax]]
  [uic]`), and `transient_config()` starts from the operating point unless
  `uic` is given, as ngspice does. `uic` used to be dropped without a word,
  and a `tmax` was a parse error.
- `.tran`'s `tstart` is applied: `TransientConfig::tstart` and
  `with_tstart(t)` integrate from 0 as before but keep only the steps from
  `t` on, with one landing on `t`. It used to be read and ignored.
- `ParsedNetlist::transient_config()`: the deck's first `.tran` with its
  `.ic` cards as initial voltages, so a deck runs as ngspice runs it.
- ngspice goldens `tran_ic_op` and `tran_ic_uic` (`.ic` without and with
  `UIC`, on the same deck) and `tran_tstart`, run from the deck's own
  `.tran` and `.ic`.
- Two-source DC sweeps: `dc_sweep_nested(circuit, inner, outer)` with
  `DcSweepAxis` gives a `DcSweepResult` along the inner source for each
  value of the outer one, for device curve families. SPICE `.dc` takes a
  second source (`.dc VDS 0 5 0.1 VGS 0 3 0.5`), and
  `AnalysisRequest::dc_axes()` turns a card into axes, counting points the
  way SPICE steps (callers used to truncate `(stop − start) / step`).
- `dc_sweep` sweeps current sources too; `.dc` accepted them, but running
  the sweep failed.
- Noise analysis, SPICE's `.noise`: `solve_noise` with `NoiseConfig` gives
  output and input-referred noise density, each source's contribution and
  band totals. It covers thermal noise (resistors, potentiometers,
  thermistors, photoresistors, FET channels), shot noise (diodes and BJT
  junctions) and flicker noise (`KF`, `AF`). Validated against ngspice on
  an RC filter (kT/C) and a BJT common-emitter stage. `.noise v(out[,ref])
  src dec|oct|lin n f1 f2` is parsed, and `KF`/`AF` are read from `D` and
  `NPN`/`PNP` cards and written by `to_netlist`.

### Changed

- **Breaking:** `AnalysisRequest::Tran` has new `tmax` and `uic` fields;
  patterns need `..`. A deck's `transient_config()` now starts from the
  operating point unless the card says `uic`.
- **Breaking:** `AnalysisRequest::Dc` has a new `outer` field; patterns
  need `..`.
- `dc_sweep` starts each point's Newton–Raphson from the previous point,
  as SPICE does, so a curve stays on one branch of a circuit with several
  operating points. Results move only within Newton's tolerance.
- **Breaking:** `DiodeModel`, `BjtModel` and `MosfetParams` have new `kf`
  and `af` fields (struct literals need `..Default::default()`; JSON
  without them still loads), and `AnalysisRequest` has a new `Noise`
  variant.
- **Breaking:** `SimError::Measurement` is a new variant, and
  `ParsedNetlist` has a new `measurements` field.
- `AcPoint::node_voltages` now includes the ground node (0 V), as
  `SimulationResult::node_voltages` does.
- **Breaking:** `CircuitElement::Diode` and `CircuitElement::Bjt` have a new
  `model` field; struct literals need `model: None` for the old behaviour.
  JSON without it still loads.
- **Breaking:** `Circuit` has new fields (`instances`, `blocks`, `parts`,
  `globals`), so struct literals need `..Default::default()` or
  `Circuit::new`. JSON is unchanged for a flat circuit, and old JSON loads.
- **Breaking:** `ParsedNetlist::circuit` keeps `.subckt` hierarchy instead
  of flattening it; call `circuit.flatten()` for the flat element list.
  `SourceMap` and the flattened names are unchanged.
- **Breaking:** `CircuitElement::OpAmp` has a new `model` field; struct
  literals need `model: None` for the old behaviour. JSON without it still
  loads. The docs no longer claim the unmodelled op-amp saturates at its
  rails: it never did.
- **Breaking:** `SimError::Design` and `ExportError::Design` for a hierarchy
  that can't be flattened.
- `Part` is no longer `Eq` (its ratings are `f64`).
- **Breaking:** `VoltageRegulator` has a new `model` field (struct literals
  need `model: None`), and is nonlinear. Even with no model it now draws its
  output current from the input and can't put out more than its input; it
  used to be an ideal source with the input unconnected. Its
  `ComponentResult` is now the pass element (`Vin − Vout`, output current,
  dissipation) instead of the output voltage and branch current.
- **Breaking:** `SimError::ThermalRunaway` is a new variant, and
  `SimulationResult` and `TimestepSnapshot` have a new
  `junction_temperatures` field (struct literals need it; JSON without it
  still loads).
- `PartialEq` on `CircuitElement`, `Waveform`, `MosfetParams`,
  `IgbtParams` and `VaractorParams`.

### Removed

- **Breaking:** `newton_raphson::I_ABSTOL`, which nothing read. Newton
  converges on node voltages (`V_ABSTOL`, `RELTOL`).

### Fixed

- AC analysis now includes a BJT's and a MOSFET's `parasitic_caps` (Cbe,
  Cbc, Cgs, Cgd). They were stamped in transient only, so a transistor
  amplifier's AC response had no Miller pole and stayed flat to any
  frequency. `.noise` goes through the same small-signal system and changes
  with it. New ngspice goldens `ac_bjt_caps` and `ac_mosfet_cs` pin them.
- A hot LED's forward voltage now falls about 2 mV/K at its rated current,
  as a real one does. IS scaled with silicon's 1.11 eV bandgap, below every
  LED's Vf, so Vf rose with temperature and a self-heated LED drew less
  current as it warmed. Each colour now has an effective EG fitted to
  −2 mV/K. Temperature sweeps and self-heated results for LEDs change.
- A `.model` card stopped at the first parameter it couldn't read and kept
  the ones before it, so `d(is=1e-10, n=2)` (commas, as many vendor
  libraries write them) silently lost `n`, and `is=` with no value or a
  stray word lost the rest of the card. Commas now separate parameters, and
  anything else that isn't `name=value` is a syntax error. `.param` and
  `.subckt` headers with trailing text are errors too, where they used to
  drop it.
- Validation now refuses values the models can't use, naming the
  component and field, before the solver turns them into a bare
  `InvalidSolution` or a wrong answer: a NaN or infinite resistance
  (`InvalidResistance`; an infinite one solved as an open), a non-finite
  source value or controlled-source gain, a negative or non-finite
  capacitance, a zero, negative or non-finite inductance (or transformer
  winding), and non-finite relay, potentiometer, photoresistor, fuse and
  op-amp/comparator rail values. A zero capacitor is still an open.
- **Breaking:** two components with the same id are an `InvalidComponent`.
  Results, controlled sources and sweeps look components up by id, so the
  second was silently shadowed by the first. A SPICE deck that names two
  elements alike now fails to solve.

## [0.1.0-alpha.7] - 2026-11-16

### Added

- Transfer-function analysis, SPICE's `.tf`: `solve_tf` with a `TfConfig`
  returns the DC small-signal gain from a voltage or current source to a
  node voltage or a voltage source's current, with the input and output
  resistance. `sindr::spice` reads `.tf v(out[,ref]) src` and
  `.tf i(vsrc) src` into `AnalysisRequest::Tf` (`tf_config()`).
- DC sensitivity analysis, SPICE's `.sens`: `solve_sensitivity` with a
  `SensConfig` returns the derivative of one output with respect to each
  resistance, source value, VCVS gain and VCCS gm, by one adjoint solve.
  `sindr::spice` reads `.sens v(out[,ref])` and `.sens i(vsrc)` into
  `AnalysisRequest::Sens` (`sens_config()`).
- `MnaMatrix::transpose`.

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
  that `mosfet_companion` works in, and `MosfetKind::nmos_frame`: a
  three-terminal device's `(vgs, vds)` in that frame.
- `sindr_devices::diode::{thermal_voltage, K_OVER_Q, T_NOM}` and a `vt()`
  method on `DiodeParams`, `BjtParams`, `ZenerParams` and
  `PhotodiodeParams`: the thermal voltage kT/q at the junction temperature.

### Changed

- **Breaking:** nalgebra 0.35. `MnaMatrix::to_dense`, `MnaSystem::b` and
  `MnaSystem::solve` use its `DMatrix`/`DVector`, so callers on nalgebra
  0.34 need to upgrade too.
- MSRV is Rust 1.89 (was 1.87), which nalgebra 0.35 requires.
- **Breaking:** `spice::AnalysisRequest` has `Tf` and `Sens` variants, so a
  `match` over it needs arms for them.
- **Breaking:** new public fields `SimulationResult::convergence_aid`,
  `TransientData::stats` and `ParsedNetlist::{initial_conditions, nodesets}`
  mean code that builds these structs with a literal must set them.
- A device `temperature` that is zero, negative or not finite, and a BJT
  `bf` that isn't positive, are now rejected up front as `InvalidComponent`
  naming the component and field. They used to fail mid-solve as a bare
  `InvalidSolution` or a convergence failure.
- `sindr::spice` rejects, at the card's line: a `.dc` whose step is zero or
  points away from stop, a `.dc` sweeping a source the deck doesn't define,
  an `.ac` with fstart above fstop or (for `dec`/`oct`) at or below 0, and
  any value that evaluates to NaN or ±∞ (`1e400`, `{1/0}`). These used to
  parse and fail later with no line, or reach the solver as infinite values.
- The SPICE suffix `a` is atto (1e-18), as in ngspice. `1a` read as 1.

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

- A relay coil with inductance was stamped in transient as its resistance
  and inductance in parallel, so the coil became a short once current built
  up: 12 V through 100 Ω into a 500 Ω, 0.1 H coil settled near 0 V across it
  instead of 10 V. They're now in series, as in DC.
- A SPICE expression nested a few thousand levels deep (`{((((…))))}`)
  overflowed the parser's stack and aborted the process. Nesting deeper than
  256 levels is now a parse error.
- A diode with series resistance in reverse bias, when `RS·IS` is many
  `N·V_T`, returned an unconverged junction voltage, and past about 18 V of
  `RS·IS` NaN. With `RS = 0`, an overflowing junction conductance was hidden
  as GMIN instead of staying infinite.
- `.tf` with a misspelt input or output now reports it before solving the
  bias point, so it isn't masked by a DC failure. A node between two
  varactors, held only by their DC leak, was singular in `.tf`.

- A PULSE with period 0, SPICE's one-shot pulse, never fired: the waveform
  took `t % 0`, got NaN and stayed at `v1` for the whole run. A zero,
  negative or non-finite period now means a single pulse, and
  `Waveform::period()` returns `None` for it.
- Transient: a step ending on a source's jump (a square, PWM or zero-rise
  PULSE edge) took the value after the jump, so the edge landed one step
  early. At a fixed 100 µs step, a 1 kHz square into a 100 µs RC read
  `out = 0.21` at its first falling edge instead of 0.99. That step now
  sees the source's value just before the edge.
- Transient snapshots reported a waveform source's DC term as its voltage
  (or current) and power, ignoring the waveform. They now report its value
  at that time.
- Transient initial conditions that name no node or inductor (a typo), sit
  on a transformer or aren't finite were ignored or failed later as
  `InvalidSolution`. They're now `InvalidComponent` naming the key.
- Every transient and sweep is bounded. `solve_circuit` and
  `solve_circuit_with_initial_voltages` now check their automatic transient
  config like `solve_transient` does, so a circuit whose time constants ask
  for 5·10⁸ steps errors instead of running out of memory. Waveform
  breakpoints count against the 10⁷-step cap, and a source with more
  breakpoints than are kept is an error naming it instead of being stepped
  over. An AC, DC or temperature sweep of more than 10⁷ points returns
  `InvalidComponent` instead of panicking with "capacity overflow".
- JFETs in the ohmic region with the drain below the source had the wrong
  sign of `gm`: `JfetCompanion::gm` took the absolute value of
  `2·IDSS·Vds/Vp²`. DC still converged, but on the wrong Jacobian, and AC
  and `.tf` reported a gain of the wrong sign (an N-JFET at Vds < 0 gave
  −0.0487 where the DC slope is +0.0491). `gm` is now negative there.
- AC and `.tf` always linearised a relay's contact as open, while DC closes
  it once the coil is at pickup, so a relay switching a signal path showed
  ~1e-6 gain instead of the real one. They now take the contact state from
  the DC bias point.
- `solve_ac` with a `source_id` that names no voltage source (a typo, or a
  current source) returned 0 V everywhere. It's now `InvalidComponent`
  naming the id, as `dc_sweep` and `.tf` already did.
- AC and `.tf` on a circuit with a transformer: the bias point left the
  transformer's two branch rows empty, so both failed with
  `SingularMatrix`. The DC equivalent now shorts each winding with a 0 V
  source, and AC stamps the coupled pair, `V = jω·[[L1, M], [M, L2]]·I`,
  so the secondary carries signal. Before, AC dropped the mutual inductance
  and shifted the branch rows of every source after a transformer.
- Transformers in transient modelled negative inductance (the winding rows
  had V = −L·dI/dt). At the large steps sindr used to take, the unstable mode
  this created happened to look settled; at short steps the secondary blew
  up. A winding is now V = L·dI/dt, so it is a short at DC.
- Varactors in transient: the companion injected its history current with
  the wrong sign, so a varactor charged through a resistor settled about
  1e-3 short of the source voltage.
- Diode, LED and Schottky reverse leakage. The companion floored its
  conductance at GMIN but built its current source from the unfloored one,
  which added a GMIN conductance across every reverse-biased junction (twice
  ngspice's leakage). The floor now only shapes the Newton steps.
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
  into reverse blocking; a limited step never counts as converged, so an
  IGBT that really is reverse-blocking still gets there.
  `IgbtParams::vce_sat` is no longer used.
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

[Unreleased]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.8...HEAD
[0.1.0-alpha.8]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.7...v0.1.0-alpha.8
[0.1.0-alpha.7]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.6...v0.1.0-alpha.7
[0.1.0-alpha.6]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.6
[0.1.0-alpha.5]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.5
[0.1.0-alpha.4]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.4
[0.1.0-alpha.3]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.3
[0.1.0-alpha.2]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.2
[0.1.0-alpha.1]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.1
[#4]: https://github.com/steffendybvik/sindr-rs/issues/4
[#5]: https://github.com/steffendybvik/sindr-rs/pull/5
