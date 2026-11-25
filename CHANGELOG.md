# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0]

A typed API (element structs, node and component handles, error causes,
sweep configs, result enums), one file per device, a sparse solver that
stamps into a fixed layout (Newton about twice as fast, large transients
nearly so), new analyses and elements, and a schematic import that
simulates a board while it's still being drawn. Serialised circuit JSON
and SPICE input are unchanged.

### Migrating from 0.2

- **Elements are structs.** Each `CircuitElement` variant wraps a struct
  from `sindr::element`:
  `CircuitElement::Resistor { id, nodes, resistance }` becomes
  `CircuitElement::Resistor(Resistor { id, nodes, resistance })`, with
  `use sindr::element::Resistor;`. Match the same way:
  `CircuitElement::Resistor(Resistor { resistance, .. })`.
- **Nodes and components are handles.** Terminals are `[NodeId; N]`, a
  controlled source's `control_source` is a `ComponentId`, and analysis
  configs take `NodeId`/`ComponentId`. `"n1".into()` still compiles; a
  `String` needs `.into()` or `NodeId::from`. `AcSensEntry::component_id`
  is a `ComponentId` (`.as_str()` for the name).
- **Errors have typed causes.** `SimError::InvalidComponent(msg)` is
  `InvalidComponent { id, reason }`. `InvalidResistance` is gone: a bad
  resistance is `InvalidComponent`. A bad analysis setting is
  `InvalidConfig { config: ConfigKind, reason }`, a repeated id
  `DuplicateId`, and a name nothing has `UnknownNode` or
  `UnknownComponent`. Match on the variant instead of the message.
- **Sweeps take configs.** `dc_sweep(c, "V1", 0.0, 5.0, 51)` is
  `dc_sweep(c, &DcSweepAxis::new("V1", 0.0, 5.0, 51))`, and
  `temperature_sweep(c, 250.0, 400.0, 4)` is
  `temperature_sweep(c, &TempSweepConfig::new(250.0, 400.0, 4))`, on
  `Solver` too. `TempSweepPoint::temperature_kelvin` is `temperature`,
  `TempSweepResult::{temp_start, temp_end}` are `{start, stop}`, and
  `import::Analysis::DcSweep { .. }` is `Analysis::DcSweep(DcSweepAxis)`
  (its JSON has `source_id` and `num_points`).
- **Regions are enums.** `BjtResult::region` is a `BjtRegion` and
  `MosfetResult::region` a `MosfetRegion`: compare with
  `BjtRegion::Active`, or call `.as_str()` for the old string. They
  serialise as the same lowercase strings. `LedColor` and
  `RegulatorRegion` are `#[non_exhaustive]`, so a `match` needs a
  wildcard arm.
- **Two structs gained a field.** `BenchSource` has `assumed` (use
  `..BenchSource::voltage(..)` or set `assumed: false`), and `Instance`
  has `variant` (`variant: None`, or `Instance::new`). Both default in
  JSON.
- **Every feature is on by default.** `sindr = "0.3"` gets serde, JSON
  Schema, SPICE, examples and threaded Monte Carlo. To keep a build
  small, set `default-features = false` and list the features you want.
- **`0V` is a ground.** A net named `0V` is no longer given a 0 V
  supply. If it isn't the bench's ground, tie it with `with_tie`, or
  validation fails with `FloatingNode`.
- **Stuck transients stop.** A transient whose step sits at its floor
  stops with `SimError::TimestepTooSmall` instead of running for
  minutes.

### Added

- `TransientSession`: a transient held open between calls, for a
  circuit paired with firmware, a user or a changing environment. It
  follows FMI 3.0 co-simulation: `advance_to`, `set_source` and
  `set_param` at the time reached, and `run_until` with an early return
  on threshold-crossing events.
- `solve_transient_with_progress` runs a hook after every accepted step;
  returning `Flow::Stop` cancels the run with `SimError::Cancelled`.
- `sindr::fourier`: `Fourier` (SPICE `.four`: harmonics, phases and THD,
  as ngspice computes them) and `Fft` (a windowed radix-2 FFT of a
  uniform resample, scaled to peak amplitude), reading a `Signal` from a
  transient or a `(t, v)` trace. The SPICE parser reads `.four` cards
  into `ParsedNetlist::fourier`.
- `SwitchingRegulator`: an averaged buck or boost converter. It holds its
  set voltage and draws `Vout·Iout/(η·Vin)` from its input, with an
  efficiency constant or over load, a current limit, dropout and UVLO.
  `ConverterParams::ripple_current` estimates the inductor ripple, and
  `converter::feedback_voltage` gives a set point from a divider.
- `DcMotor`: a brushed DC motor with armature R, L and back-EMF, and a
  shaft with inertia, viscous friction and a load torque, which a
  `TransientSession` can change.
- `DigitalPin`: the electrical side of an MCU or logic-IC pin, after
  the IBIS I/O buffer, with mode, level and pull settable through a
  session. `LogicGate`: a CMOS gate at the analog level, with `vil`/`vih`
  or Schmitt inputs, the pin's output stage and a propagation delay.
- `CircuitBuilder` hands out `NodeId` and `ComponentId` handles and
  refuses an undeclared net or a repeated id. Results take a handle or a
  name.
- A block can hold several variants of one body (ideal, behavioural,
  detailed), and each `Instance` picks one with `with_variant`.
- `schemars` feature: `JsonSchema` for every serde type in both crates.
- `sindr::VERSION` and `sindr_devices::VERSION`.
- `import::supply_names` reports which supplies net names imply, the
  supply-like names it left alone, and the nets that look like grounds.
- `UnconnectedTerminals::Skip` leaves out components with unconnected
  required terminals, so a schematic being drawn still simulates.
- `Model::Subckt` uses a vendor `.subckt` as a part's model, with a pin
  map from its terminals to the subcircuit's ports.
- The import ties nets that only reach IC pins to ground through 1 TΩ
  and lists them, so a board with black-box ICs solves.

### Changed

- **Breaking:** each `CircuitElement` variant wraps a struct from
  `sindr::element`.
- **Breaking:** terminals, controlled-source controls and analysis
  configs take `NodeId`/`ComponentId` handles.
- **Breaking:** `SimError` has typed causes: `InvalidComponent { id,
  reason }`, `InvalidConfig { config, reason }`, `DuplicateId`,
  `UnknownNode` and `UnknownComponent`.
- **Breaking:** `dc_sweep` and `temperature_sweep` take a `DcSweepAxis`
  and a `TempSweepConfig`, with renamed result fields.
- **Breaking:** `BjtResult::region` and `MosfetResult::region` are
  enums; `LedColor` and `RegulatorRegion` are `#[non_exhaustive]`.
- **Breaking:** `BenchSource` has an `assumed` field and `Instance` a
  `variant` field.
- Every feature but `internals` is on by default.
- A net named `0V` counts as a ground, not a 0 V supply.
- `Bench::with_supplies_from_net_names` doesn't put a second source on a
  net the bench already drives, and tied nets get one supply between
  them. A supply it assumes is marked `assumed`.
- Sparse systems (above 100 unknowns) stamp into a fixed layout recorded
  on the first solve and refactor numerically into a reused LU: Newton
  on 1 000 stages is about twice as fast, AC 2.4×, and a 1 000-stage RC
  ladder transient 1.8×. Grounded sources are substituted out before a
  sparse factorisation, so a supply rail shared by every part no longer
  fills the LU in (`newton_bjt_row/1000`: 401 → 33 ms before the layout
  change). Node names, control branches, sensed nets and AC terminals
  are resolved once per solve, not on every iteration.
- Each device's DC, transient, AC and noise models, ratings, SPICE line
  and results live in its own file in `sindr::element`. Results are
  unchanged.

### Removed

- `SimError::InvalidResistance`: a bad resistance is
  `SimError::InvalidComponent`.

### Fixed

- A circuit with no unknowns (only elements from ground to ground)
  solves to 0 V and 0 A instead of panicking.
- A transient that can't be followed at any step (a current source into
  a node with no DC path) stops with `TimestepTooSmall` instead of
  running for minutes. Only steps stuck within 10× the step floor count,
  so a fast circuit with a coarse `tstep` still runs, and accepted steps
  are capped at the step budget.
- `dc_sweep` accepts a one-point axis (`start == stop`), as
  `DcSweepAxis::from_step` and a deck's `.dc` can produce.
- The progress hook's step has crystals, ferrite beads and SPDT switches
  folded back, as the result's timesteps do.
- `to_netlist` refuses a circuit with two elements of one id instead of
  writing a deck that doesn't parse.
- The import's floating-net tie counts subcircuit ports, and follows only
  DC paths, so a group grounded only through a capacitor, a current
  source or a FET gate gets its tie.
- A `Model::Subckt` part's internal nodes no longer join the outer
  circuit's ground or another model's `.global` by name.
- Named-net supplies no longer short a net the bench already drives,
  which made the operating point a `SingularMatrix`.

## [0.2.0] - 2026-11-22

### Added

- Results are read by name: `SimulationResult::voltage`, `current`,
  `branch_current` and `component`, and `bjt`, `mosfet`, `op_amp` and
  `relay` for device results, each `None` for a name the result doesn't
  have. Ground reads `0.0`. `TimestepSnapshot` has `voltage`, `current`
  and `component`.
- `SimulationResult::instance("X1")` reads inside a block instance or
  SPICE subcircuit by its instance path: `instance("X1").voltage("sw")`
  is node `X1.sw`. `instance` chains (`instance("X1").instance("X2")`)
  or takes a nested path (`"X1.X2"`); `InstanceResult::name` gives the
  flat name for the curve accessors.
- `TransientData::node_voltage_curve` and `component_current_curve`
  give a waveform as `(t, value)` pairs, and `voltage_at` and
  `current_at` a value at any time, interpolated between steps.
- `AcPoint::voltage` and `magnitude`, and `AcResult::node_voltage_curve`
  (complex) and `magnitude_curve`, next to the existing gain and phase
  helpers.
- `TempSweepResult::node_voltage_curve` and `component_current_curve`,
  as `DcSweepResult` has.
- The SPICE parser reads `M` and `J` lines with level-1 `NMOS`/`PMOS`
  and `NJF`/`PJF` cards. `kp` is `KP·W/(L − 2·LD)·M`, the overlap
  capacitances become the gate caps, and the drain–bulk junction becomes
  the body diode. `idss` is `BETA·VTO²·area`. A `LEVEL` other than 1 is
  an error. A bulk node off the source, and parameters sindr's FETs don't
  have, are errors in strict mode and warnings in lenient mode.
- `to_netlist` writes MOSFETs and JFETs, and they parse back to the same
  circuit. A body diode SPICE's junction can't hold goes out as its own
  `D` line.
- `DiodeModel` and `BjtModel` have `eg` and `xti`: the bandgap and
  exponent `IS` follows temperature with, as SPICE's `IS(T)`. They
  default to silicon's 1.11 eV and 3, so circuits that leave them out
  are unchanged. The SPICE parser reads `EG` and `XTI` on any diode or
  BJT card, a Schottky's `EG=0.69 XTI=2` included, and `to_netlist`
  writes them when they aren't the defaults. `TNOM=27` parses in strict
  mode too; another `TNOM` is an error there.
- A `.model` inside a `.subckt` is read, local to that subcircuit, so
  vendor model files that keep their cards inside the subcircuit load.
  Before, it was ignored with a warning.
- `CircuitElement::Behavioural`: SPICE's `B` source, a voltage or
  current set by an `Expression` of node voltages (`v(n)`, `v(n,m)`),
  branch currents (`i(src)`), `time`, arithmetic, comparisons, `?:` and
  ngspice's functions. Newton stamps its exact Jacobian by forward-mode
  automatic differentiation, AC linearises it at the bias point, and
  transient reads `time`. In JSON the expression is its text.
- An expression that overflows or leaves its domain on the way to the
  answer (`exp` of the whole supply on the first step, `ln` of a node
  that starts at 0 V) no longer ends the solve. Newton shortens the step,
  gmin and source stepping or a shorter transient step take over, and
  the answer is checked for finite values before it's returned.
  `SimError::NotFinite` names the source, its expression and what it
  read, when it can't be got past.
- `sindr::expression`: `Expression::parse`, `variables` and `evaluate`.
- The SPICE parser reads `B` lines and the `VALUE` form of `E`/`G` into
  behavioural sources, with `.param` names resolved inside the
  expression. `to_netlist` writes them back as `B` lines.
- The `POLY(n)` form of `E`, `G`, `F` and `H` is read too, as a
  behavioural source holding SPICE2's polynomial of `n` controlling
  voltages or currents.
- `sindr::variation`: Monte Carlo (`monte_carlo`) and corner (`corners`)
  analysis around any analysis. A `Tolerance` varies a resistance,
  capacitance, inductance or source value uniformly or Gaussian, from a
  seeded generator so runs repeat. Results give each run's values and
  measurements, and each measurement's mean, standard deviation, min, max
  and yield. A run that fails is recorded, not fatal. The `parallel`
  feature spreads runs across threads.
- Worst case over tolerances: `VariationResult::check_specs` judges each
  spec on every run, and `ratings_over` checks every part's ratings on
  every run of a Monte Carlo or corner `Sampling`. Each reports its worst
  run with the tolerance values behind it.
- `CircuitElement::Transformer` takes an optional `saturation`, on a
  magnetising branch: leakage `(1 − k)·L` on each winding and `k·L1` on
  the primary carrying the shared core flux. A transformer driven past
  its volt-second rating, or with a DC bias as in a forward or flyback
  converter, now draws a magnetising current that climbs once the core
  saturates. AC and noise use the incremental inductance at the DC
  bias. Without it, the transformer is the same linear coupled pair.
- `sindr::import` takes a netlist as an editor exports it.
  `ComponentKind::from_name` reads symbol ids, ignoring case and treating
  `-`, `_` and spaces alike, with aliases such as `resistor-iec`,
  `op-amp`, `push-button`, `switch-spst` and `header-<n>`; terminal names
  match the same way, and an op-amp also takes `Non-inverting`,
  `Inverting`, `Output`, `Positive supply` and `Negative supply`.
  `ImportOptions::with_terminal_names` passes each component's
  terminal-name table to `Netlist::to_circuit_with` and
  `import::simulate_with`, so nets may name terminals by id; a bare pin
  number still never picks a polarised terminal.
- `ComponentKind::BlackBox` for parts sindr can't simulate, such as ICs:
  no element, but its terminals stay bench attach points.
  `UnknownKinds::BlackBox` imports unknown kinds that way instead of
  refusing them, which stays the default. `ImportedCircuit::black_boxes`
  lists every black box with the nets it's on.
- `ComponentKind::DualOpAmp`, `QuadOpAmp`, `DualComparator` and
  `QuadComparator`: one package whose two or four units share its supply
  pins. Each unit imports as its own element, `<id>.a`, `<id>.b` …, with
  terminals tagged by unit as datasheets write them (`1IN+`, `IN1-`,
  `INA+`, `-IN B`, `OUT A`, `2OUT`). A spare unit with nothing connected
  adds no element. `ComponentKind::units` gives a package's unit kind and
  count.
- `ComponentKind::Crystal` and `ComponentKind::FerriteBead` import as
  `CircuitElement::Crystal` and `CircuitElement::FerriteBead`, from a
  frequency, ESR and shunt capacitance, or an impedance at 100 MHz and a
  DC resistance.
- JFET gate capacitances: `CircuitElement::Jfet` has an optional
  `parasitic_caps` (`JfetParasiticCaps`), SPICE's gate junctions `CGS` and
  `CGD` graded by `PB`, `M` and `FC`. Transient integrates their charge,
  and AC and noise use the capacitance at the bias point. The SPICE parser
  reads `CGS`, `CGD` (times the area factor), `PB`, `M` and `FC` from an
  `NJF`/`PJF` card, and `to_netlist` writes them back. Checked against
  ngspice on a common-source stage's Miller pole (`ac_jfet_caps`).
- AC sensitivity: `solve_ac_sensitivity` (and `Solver::ac_sensitivity`)
  gives, at each frequency of an `AcConfig`, the derivative of a
  small-signal output phasor with respect to R, C, L, the controlled
  sources' gains, the stimulus's AC magnitude and device-model
  parameters, from one adjoint solve per frequency. As in ngspice, the DC
  operating point is held fixed. The SPICE parser reads `.sens v(out) ac
  dec 10 1 1meg` into `AnalysisRequest::SensAc`, and
  `ParsedNetlist::ac_sens_config` drives it from the deck's `AC` spec.
- `.sens` reports device-model parameters: a diode's IS, N and RS, a
  BJT's BF, IS, BR, NF, NR and VAF, and a MOSFET's KP, VTO and LAMBDA (in
  AC a diode's CJO and TT as well), from analytic derivatives, IS's
  temperature scaling included. CCVS transresistance and CCCS current
  gain get entries too. `SensResult::get_parameter` picks one parameter
  of a device out. Checked against central differences of full re-solves
  and against ngspice.
- `sindr-devices`: `diode_derivatives`, `bjt_derivatives` and
  `mosfet_derivatives` give the exact derivatives of each model's
  currents and conductances with respect to its parameters, and
  `JunctionCharge::depletion_capacitance_slope` the slope of the
  depletion capacitance.
- Pole-zero analysis, SPICE's `.pz`: `solve_pz` with a `PzConfig` gives
  the poles and zeros (rad/s) of the voltage gain or transimpedance from
  one node pair to another, as the generalised eigenvalues of the
  linearised `G + sC` pencil. Infinite eigenvalues are deflated exactly
  before QZ, so a capacitor across a source or an inductor in series with
  a current source leaves no spurious roots. The SPICE parser reads
  `.pz in in_ref out out_ref vol|cur pol|zer|pz` into
  `AnalysisRequest::Pz`, and `pz_config()` turns it into a `PzConfig`.
- Loop gain at a break point: `solve_loop_gain` with a `LoopGainConfig`
  naming a 0 V probe source in series with the loop gives the loop gain
  `T` over frequency by Tian's double injection, exact for loops that
  carry signal both ways through the probe, with the gain and phase
  crossovers and the phase and gain margins, each refined by bisection.
- `Solver::pz` and `Solver::loop_gain` run them with solver options.
- Two bench loads. `LoadKind::CurrentSink { amps }` draws a constant
  current from one attach point to the other (ground by default), such as
  an IC's supply current. `LoadKind::TimedSwitch` is closed through
  `on_resistance` until `at` and then open at `off_resistance` (its
  leakage), or the reverse with `starts_closed: false`: an open-drain
  line held low and released, for a bus's rise time. The operating point,
  DC sweeps and AC take it at t = 0, and a transient lands a step on
  `at`. `LoadKind::current_sink`, `released_at`, `closed_at` and
  `with_off_resistance` build them; in JSON they are
  `{"current_sink": {"amps": 0.005}}` and `{"timed_switch":
  {"on_resistance": 25, "at": 1e-6}}`, the off resistance defaulting to
  1 GΩ and `starts_closed` to true.
- A transient lands a step on every time a behavioural source compares
  `time` with a constant (`time < 1u`), as it does on a waveform's
  corners.
- A reference-design test suite, `sindr/tests/reference_linear_supply.rs`:
  a 7805 from 12 V feeding an AMS1117-3.3 at 500 mA, with datasheet
  ratings and thermal paths on every part. It checks dissipation and
  junction temperature against hand calculations, start-up inrush, and
  that sindr flags each broken variant: no heatsink, the AMS1117 on 12 V,
  a 6 V input, and missing capacitors.
- `sindr::import` builds models from part data and says when it
  couldn't. `ImportedCircuit::models` records each component's
  `ModelSource` (its model, value, parameters, datasheet figures, or
  sindr's default), and `ImportedCircuit::warnings` carries an
  `ImportWarning` for every component that got the default, such as a
  transistor whose value is only a part number. Warnings don't stop the
  import.
- Datasheet constructors: `DiodeModel::from_datasheet` (Vf at If, or two
  points), `DiodeModel::led_from_datasheet`,
  `DiodeModel::zener_from_datasheet` (Vz at Iz), `BjtModel::from_datasheet`
  (hFE, and Vce(sat) at Ic/Ib) and `MosfetParams::from_datasheet` (Vth at
  Id, Rds(on) at Vgs). Figures are `DatasheetValue`s with optional min,
  typ and max, in `sindr::datasheet` (and `sindr_devices::datasheet`).
  `sindr::import` reads them from parameters: `vf`/`if`, `vz`/`iz`,
  `hfe`, `vce_sat`, `vth`, `rds_on` and their test conditions, each figure
  also as `_min`, `_typ` and `_max`.
- LEDs read their colour in any case, from `color`, `colour` or a colour
  word in the value text (`"Amber"`, `"LED red 3mm"`). Amber (2.05 V at
  20 mA) and orange (2.0 V) join the colours, through
  `sindr_devices::led::COLORS`, `forward_voltage_of` and `color_in`; an
  LED element's `color` takes them too.
- `import::Value::Quantity`: a typed quantity as a parts library stores
  it, a number in base units with its `Unit` and the text shown. Its unit
  is checked against the parameter's.
- `sindr_devices::regulator::regulator_edges`: the signed distances from
  a regulator's operating point to the edges between the pieces of its
  characteristic.
- `TransientStats::jumped_steps` counts the regenerative edges a
  transient jumped (see Fixed).

### Changed

- **Breaking:** `CircuitElement::Transformer` has a `saturation` field,
  so a literal needs `saturation: None`. With serde a missing field is
  `None`.
- **Breaking:** `DiodeModel` and `BjtModel` gained fields, so a struct
  literal that names every field needs `eg` and `xti` too. One that ends
  in `..Default::default()` is unaffected.
- **Breaking:** `CircuitElement::Jfet` has a `parasitic_caps` field, so
  a literal needs `parasitic_caps: None`. With serde a missing field is
  `None`.
- `import::parse_value` reads a unit after a space as a unit: `"2 m"`
  is now 2 metres, not 2 milli. `"2m"` is still 2 milli.

### Fixed

- The `led-circuit` example (`examples::led_circuit()`) is a red `Led`
  (`LED1`), not a fixed 2 V source (`V_LED`). Swept with `dc_sweep`, the
  stand-in conducted backwards below 2 V (6 mA at 0 V); the LED blocks
  until its forward voltage and draws about 20 mA at 9 V. "(Simplified)"
  is gone from its name.
- Newton no longer settles on a stale piece of a `VoltageRegulator`'s
  characteristic. A step within the node tolerance could stop on the
  piece the previous iterate was on, so a regulator leaving its 0 V floor
  on a ramped input lagged by a millivolt and caught up in one short
  step: a spike up to the current limit into its output capacitor (1.1 A
  instead of 0.26 A for 22 µF at 12 V/ms), which `check_ratings` then
  reported. A solve now converges only once every regulator is on the
  same piece, and the same quiescent-current ramp, as the iterate it was
  stamped from.
- `import::parse_value` no longer runs a number and a unit together
  across a space: `"0.01 1"` (a ratio) is 0.01, not 0.011, and `"300 K"`
  is 300 kelvin, not 300 k. `300K` is still 300 k.
- An LED whose colour was capitalised (`"Green"`) or amber came out red.
- A linear regulator fed through an inductor or an LC input filter now
  simulates in transient, from switch-on and on a hot-plugged input.
  Newton failed to converge at the first step, because each step crossed
  every edge of the regulator's piecewise characteristic at once; a step
  now moves a regulator one piece at a time.
- A two-transistor LED flasher, and other regenerative circuits whose
  transistors have no junction capacitance, run their transient through
  every switching edge instead of stopping with `ConvergenceFailed`. A
  step that fails at the shortest length follows the jump by
  pseudo-transient continuation and ends on a plain solve, so the answer
  has nothing added. Junction limiting no longer moves a node a voltage
  source holds, which kept an LED off the supply rail at switch-on. The
  flasher's period matches ngspice to 0.006 % (`tran_flasher`).

## [0.1.0] - 2026-11-21

The first plain release. Versions are now `0.x` with no prereleases: a
breaking change bumps the minor version, anything else the patch, so
`sindr = "0.1"` keeps working through `cargo update`. This release was
staged as `0.1.0-alpha.8`.

### Added

- `solve_op` and `Solver::op`: the DC operating point as SPICE's `.op`
  solves it, with capacitors open, inductors shorted and each source at
  its t = 0 value. A deck's `.op` maps onto it.
- The SPICE parser reads linear `E`, `G`, `F` and `H` lines into
  `Vcvs`, `Vccs`, `Cccs` and `Ccvs`, and a `K` line with its two `L`
  lines into one `Transformer`. Every deck `to_netlist` writes now parses
  back, and so do vendor decks with linear controlled sources. The
  `POLY`/`VALUE`/`TABLE`/`LAPLACE` forms are unsupported devices.
- V and I lines read `[DC] v`, `AC [mag [phase]]` and a waveform in any
  order, as SPICE3 does; before, any `AC` spec was a syntax error.
  `ParsedNetlist::ac_stimuli` lists the `AC` specs, and
  `ParsedNetlist::ac_config()` builds the `AcConfig` for the deck's `.ac`
  from its source line, so a vendor or textbook AC deck runs as written.
  `AnalysisRequest::ac_config(source)` gives the sweep alone.
- Node names may use any Unicode letters and digits (`å`, `µ`), as in
  ngspice.
- `AcConfig::ac_phase_deg`: the AC source's phase, which shifts every
  node's phase as SPICE's `AC mag phase` does. `AcConfig::new` and serde
  default it to 0; `with_phase_deg` sets it.
- `SolverOptions` and `Solver`. The tolerances (`reltol`,
  `vntol`, `trtol`), the Newton iteration budget, the gmin-stepping
  ladder, source stepping, the transient step cap and a `nodeset` are
  options now, not constants. `Solver::new(options)` runs every analysis
  with them (`.circuit`, `.transient`, `.ac`, `.noise`, `.tf`,
  `.sensitivity`, `.dc_sweep`, `.dc_sweep_nested`, `.temperature_sweep`).
  The free functions are `Solver::default()` and their answers are
  unchanged.
- SPICE `.options` (also `.option`, `.opt`): `RELTOL`, `VNTOL`, `TRTOL`
  and `ITL1` fill `ParsedNetlist::options`, together with the `.nodeset`
  cards; `ParsedNetlist::solver()` runs the deck with them. Any other
  option is an error in strict mode and a warning otherwise.

- A polarised capacitor's reverse voltage. Set `Ratings::reverse_voltage`
  on a capacitor's part and `check_ratings` flags `nodes[1]` (the `-`
  terminal) rising above `nodes[0]` (the `+`) by more than it, the way it
  does a diode's reverse voltage.
- `examples/vs_ngspice.rs` (feature `spice`): sindr against ngspice on
  the same decks, per call, in parallel and by size. The README's
  "Against ngspice" quotes it, and LIMITATIONS gives the transient
  ceiling.
- **Breaking:** `SimError::ConvergenceFailed` says where the circuit is
  stuck: `node` (the largest final Newton step), `components` (the ids on
  it) and `at`, a new `SolvePoint` (operating point, DC sweep or
  temperature point, transient time). The message names them. Patterns
  that list the variant's fields need `..`.
- CI builds every crate for `wasm32-unknown-unknown` and solves a circuit
  there under Node (`sindr/tests/wasm.rs`).
- Serde for every analysis result and config: `AcResult`, `AcPoint`,
  `NoiseResult` and `NoiseConfig`, `DcSweepResult` and the nested sweep,
  `TfResult`/`TfConfig`, `SensResult`/`SensConfig`. Every `CircuitElement`
  and result type is tested through JSON and back, with circuit
  snapshots in `sindr/tests/serde/`. `NoiseSource::branch` is a `String`
  (it was `&'static str`, which can't be deserialized).
- **Breaking:** three new `CircuitElement`s. `SwitchSpdt` (`[common, a,
  b]`, `position: SpdtPosition`), `Crystal` (`CrystalParams`: the
  Butterworth–Van Dyke model from frequency, ESR, C0 and Cm) and
  `FerriteBead` (`FerriteBeadParams`: R_dc into L ∥ R_ac, optional C,
  with `from_datasheet(z_100mhz, r_dc)`). Each is solved as its network
  in every analysis; results, ratings (crystal drive level as `power`,
  bead `current`) and SPICE export use the element's own id. Checked
  against ngspice. Exhaustive matches on `CircuitElement` need the new
  arms.
- `sindr::import`: simulate a schematic tool's netlist without writing a
  SPICE deck. A `Netlist` of components (the caller's id, a
  `ComponentKind`, a value such as `"4k7"`, parameters, an optional model
  and `Part`) and nets of `(component, terminal)` pins, plus a `Bench`
  (ground, sources and loads on nets or terminals, ties, the analysis),
  becomes a `Circuit` keyed by the caller's ids (`Netlist::to_circuit`)
  or runs (`import::simulate`). Terminals match by name, values read as a
  schematic writes them (`import::parse_value`: `4k7`, `100nF`, `1M` is
  mega), and every `ImportError` names the component, net or bench item.
  All of it is serde.
- `TransientConfig`, `AcConfig` and `FrequencySpacing` derive
  `Serialize`/`Deserialize`, and `AcConfig` derives `PartialEq`.
- **Breaking:** a power MOSFET's body diode. `CircuitElement::Mosfet` has
  a new `body_diode: Option<DiodeModel>` field (struct literals need it,
  JSON may omit it): source to drain for NMOS, drain to source for PMOS,
  solved in every analysis as a diode `<id>.body` whose current the
  results report. An ngspice golden (`dc_mosfet_body_diode`) checks it
  freewheeling.
- **Breaking:** `CircuitElement::Battery`, an equivalent-circuit cell:
  open-circuit voltage by state of charge behind a series resistance and
  an optional RC pair (`sindr::battery::BatteryParams`, with Li-ion/LiPo
  and CR2032 presets). DC, transient and AC take it. In transient the
  state of charge falls with the charge drawn, and `TimestepSnapshot`
  and `SimulationResult` have a new `state_of_charge` map by battery id.
  Exhaustive matches on `CircuitElement` and struct literals of those
  result types need updating.
- **Breaking:** saturating inductors. `CircuitElement::Inductor` has a new
  `saturation: Option<InductorSaturation>` field (`None` keeps today's
  linear inductor; struct literals need it, JSON may omit it). With
  `Some`, the flux follows `L_sat·i + (L0 − L_sat)·I_sat·tanh(i/I_sat)`:
  the transient integrates flux and solves for the current by Newton, and
  AC and noise use the incremental inductance at the DC current. The
  model lives in `sindr_devices::inductor`. No hysteresis or core loss,
  and transformers stay linear.
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
  input voltage, output current and power. A negative `voltage` is a
  negative regulator (7905, 7912): the same characteristic mirrored
  through the ground pin. A zero `voltage` is rejected.
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

- **Breaking:** the public API leaves room to grow, so a new variant,
  result field or config option is no longer a breaking change.
  - `#[non_exhaustive]` enums (match them with a `_` arm):
    `CircuitElement`, `Waveform`, `FrequencySpacing`, `SensParameter`,
    `TfOutput`, `measure::Occurrence`, and in `spice` `SpiceParseError`,
    `ExportError`, `AnalysisRequest`, `AcSweep` and `MeasureAnalysis`,
    and in `import` `Analysis`, `Simulation`, `SourceKind`, `LoadKind`,
    `Attach`, `ComponentKind`, `Model` and `Value`. `CircuitElement`'s
    variants stay open, so struct-variant literals still compile.
  - `#[non_exhaustive]` results, read but not built outside sindr:
    `SimulationResult` and the per-component, transient, AC, noise, `.tf`,
    `.sens`, sweep, measurement, spec and ratings results, and
    `ParsedNetlist`, `ParseWarning`, `ImportedCircuit` and `Imported`.
  - `AcConfig`, `NoiseConfig`, `TfConfig`, `SensConfig` and
    `DcSweepAxis` are `#[non_exhaustive]` with constructors:
    `AcConfig::new(source, f_start, f_stop, points)` with
    `with_spacing`, `with_magnitude` and `with_phase_deg`;
    `NoiseConfig::new(output, input, f_start, f_stop, points)` with
    `with_reference` and `with_spacing`; `TfConfig::new(output, input)`;
    `SensConfig::new(output)`. `TfOutput::voltage`, `voltage_between`
    and `current` build the outputs. The fields stay readable.
  - `measure::Measurable` is sealed.
- **Breaking:** the deprecated `solve_circuit_with_initial_voltages` and
  the `solve_ac`, `solve_noise`, `solve_tf` and `solve_sensitivity`
  `_with_initial_voltages` variants are removed. Use
  `Solver::new(SolverOptions::default().with_nodeset(..))`.
- **Breaking:** the `examples` feature is off by default and no longer
  pulls in `serde`; `get_examples` needs `features = ["examples"]`. The
  example circuits in `sindr::examples` are always there.
- **Breaking:** `results::extract_results` is private, and the `csv`
  module is too; the `to_csv` methods are unchanged.
- `sindr::devices` re-exports `sindr-devices`, so its `*Params` types
  need no second dependency.
- **Breaking:** `SimError`, `SolvePoint` and `spice::ParseOptions` are
  `#[non_exhaustive]`, so new error variants and parser options can come
  in a minor release. Match `SimError` with a `_` arm, and build
  `ParseOptions` with `ParseOptions::default().with_strict(false)` and
  `.with_include_search_path(dir)` instead of a struct literal.
- **Breaking (results):** `Vccs` and `Cccs` drive their current the way
  `CurrentSource` and SPICE's `G` and `F` do: a positive `gm·V_control` or
  `alpha·I_control` flows through the source from `nodes[0]` to
  `nodes[1]`. They drove it the other way, so the same `nodes` meant
  opposite directions on neighbouring variants, and `to_netlist` swapped
  their nodes to make up for it. To keep a circuit's old behaviour, swap
  each `Vccs` and `Cccs` element's two `nodes`. Exported decks no longer
  swap them.
- **Deprecated:** `solve_circuit_with_initial_voltages` and the
  `solve_ac`, `solve_noise`, `solve_tf` and `solve_sensitivity`
  `_with_initial_voltages` variants. Use a `Solver` whose
  `SolverOptions::nodeset` holds the seed.
- **Breaking:** `ParsedNetlist` has a new `options` field.

- **Breaking:** solver internals are private. `mna`, `stamp`,
  `newton_raphson`, `node_map` and `transient` are no longer public
  modules, and `MnaSystem` and `NodeMap` are gone from the root; every
  analysis entry point and config is still re-exported there.
  The backward-Euler `stamp_capacitor_companion` and
  `stamp_inductor_companion`, which the solver didn't use, are removed.
  `solve_ac`, `AcConfig`, `AcPoint`, `AcResult` and
  `FrequencySpacing` are now re-exported at the root like the other
  analyses.
- **Breaking (results):** a `ZenerDiode` is now a SPICE diode with
  breakdown `BV = vz` at `IBV` = 5 mA and `RS` = 1 Ω, in every analysis.
  Its reverse knee used to be a piecewise `1/rz` clamp whose conductance
  jumped from almost nothing to 1 S at `−vz`, which no SPICE model
  matches. A zener now holds `vz` at 5 mA and a few tens of millivolts
  less well below it, and a new ngspice golden (`dc_zener`) checks it
  through the knee. Forward voltage picks up the 1 Ω.
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
- **Breaking:** `McuResult` and `SimulationResult::mcu_results`. No
  element ever filled them: sindr has no microcontroller element.

### Fixed

- A modelled regulator with no input stays off. Below the dropout voltage
  its output could fall under the ground pin (an unpowered LM7805 drove
  −2 V into 1 kΩ), and its quiescent current, drawn whatever the input,
  pulled an undriven input negative. The output now floors at the ground
  pin, and the quiescent current ramps in from 0 V to the dropout voltage.
  `regulator_dissipation` takes the set voltage, and
  `regulator_quiescent` gives the current drawn.
- `solve_noise` names a missing reference node as the reference; before,
  the error called it the output node.
- `AnalysisRequest::ac_config` and `noise_config` sweep the frequencies
  ngspice does. A `.ac dec` over a decade or more is ⌊points·decades⌋
  equal log steps to `fstop`, and every other `dec` or `oct` sweep steps
  by 10^(1/points) or 2^(1/points) from `fstart` and stops at the last
  point not past `fstop`. Before, a band that wasn't a whole number of
  decades or octaves got `round(points·span) + 1` points spread from
  `fstart` to `fstop`: 18 for `.ac dec 10 1 50`, where ngspice has 17.
- The SPICE parser warns about a `.model` or `.subckt` inside a `.subckt`,
  which it skips; before, it dropped them without a word, so an element
  using one failed as undefined with no hint why.
- A `.include` or `.lib` card without a readable path is a syntax error
  that says what the card takes, not "internal: unresolved include
  reached parser". The lenient warning about ignored `.model` parameters
  names the deck's file, where it named none.
- `dc_sweep` and `dc_sweep_nested` solve each point as an operating point
  (capacitors open, inductors shorted), as SPICE's `.dc` does. Before, a
  circuit with a capacitor, inductor or waveform source ran a transient
  at every point and reported its 5 τ end state. A swept source's own
  waveform is ignored.
- AC, `.tf`, `.sens` and `.noise` bias a source with a waveform at its
  t = 0 value, as SPICE does, where they took its DC offset alone.
- The DC self-heating balance no longer reports a stable part as
  `ThermalRunaway`. Its plain `T ← ambient + P·Rθ` update swung wider on
  every pass when negative feedback was stronger than −1, as in a heated
  NTC fed from a current source, until it gave up. Each part now takes a
  damped secant step, and runaway is reported only for a part still
  heating at the 1000 K limit.
- AC, `.tf`, `.sens` and `.noise` now linearise a self-heated circuit at
  the bias point `solve_circuit` reports, each device at its balanced
  junction temperature. They solved the bias point at ambient, so a BJT
  heated to 350 K kept its 300 K gm. Circuits without `Thermal` data are
  unchanged.
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
- A P-channel JFET's current and power in a transient were wrong: the
  transient mirrored its voltages into the N-channel frame before the model,
  which mirrors them again. A self-biased P-JFET reported 12.7 mA where it
  conducted 4.0 mA. Node voltages were right; only the reported
  `current_through` and `power` were off. Transient and DC now share one
  calculation.
- `solve_transient` (and `solve_circuit` when it routes to transient) left
  `op_amp_results` and `relay_results` empty. They now hold the last step's
  values, as DC does.
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
- **Breaking (results):** a comparator's output now stays between its
  `v_neg` and `v_pos` rails. It was a linear VCVS of gain 1e5 that ignored
  them, so 10 mV of overdrive put 1 kV on the output. It is now the
  op-amp macromodel's rail clamp with gain 1e5 and no pole, so it takes
  Newton, two MNA branch slots (output current, then gain-stage voltage)
  and, at a rail, has zero small-signal gain in AC. A comparator whose
  `v_pos` is not above `v_neg` is an `InvalidComponent`.
- The SPICE parser reads `.global`, so a deck `to_netlist` writes for a
  circuit with `globals` parses back. It used to be an unknown card: an
  error in strict mode, and in lenient mode dropped, so a global net used
  inside a `.subckt` took the instance prefix (`x1.vcc`) and came apart
  from the top-level net. `.global` holds for the whole deck wherever it
  is written, inside a `.subckt` too, and fills `Circuit::globals`.

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

[Unreleased]: https://github.com/steffendybvik/sindr-rs/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/steffendybvik/sindr-rs/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.7...v0.1.0
[0.1.0-alpha.7]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.6...v0.1.0-alpha.7
[0.1.0-alpha.6]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.6
[0.1.0-alpha.5]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.5
[0.1.0-alpha.4]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.4
[0.1.0-alpha.3]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.3
[0.1.0-alpha.2]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.2
[0.1.0-alpha.1]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.1
[#4]: https://github.com/steffendybvik/sindr-rs/issues/4
[#5]: https://github.com/steffendybvik/sindr-rs/pull/5
