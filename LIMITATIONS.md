# Scope & Limitations

sindr is an **analog circuit simulator**: a SPICE-style MNA solver with semiconductor companion models, written in pure Rust. This document is the honest counterpart to the READMEs — it lists capabilities sindr does **not** have today, so you can decide whether it fits your use case before investing.

Status: `0.3.1`, before 1.0. Anything below may change.

## What sindr is

- DC, nonlinear DC (Newton-Raphson), transient (Gear-2 (default), trapezoidal or backward Euler), AC, noise, transfer function (`.tf`), DC and AC sensitivity (`.sens`), pole-zero (`.pz`), loop gain and stability margins, DC sweep, temperature sweep, Fourier (`.four`) and FFT of transients, and a transient stepped from outside (`TransientSession`)
- Passives, sources, switches, controlled sources, op-amp / comparator macromodels, transformer, crystal, ferrite bead, digital I/O pin (IBIS parametric buffer), logic gates at the analog level (thresholds, a pin's output stage, propagation delay)
- Diode family (Si, LED, Zener, Schottky, photodiode, varactor), BJT (Ebers-Moll), MOSFET (Level-1), IGBT, JFET
- Thermistor, photoresistor, potentiometer, relay, fuse, linear voltage regulator (dropout, quiescent current, current limit, line and load regulation), averaged buck and boost converter (power balance, efficiency over load, current limit, dropout, UVLO), battery (open-circuit voltage by state of charge, series resistance, optional RC pair)
- Programmatic Rust API + serde JSON for circuits and results

If your problem is "given an analog topology built in Rust, what are the node voltages and branch currents (possibly over time, frequency, or temperature)?", sindr is in scope.

## Out of scope (by design, today)

These are non-goals for the current crate split. They may live in sibling crates later, but they will not appear in `sindr` itself.

| Area | Status |
|------|--------|
| Digital logic simulation (flip-flops, clocked sequential, event-driven kernel; gates only as the analog `LogicGate`) | Out of scope |
| HDL ingestion (Verilog, VHDL, SystemVerilog, Verilog-A, VHDL-AMS, SystemC-AMS) | Out of scope |
| Microcontroller / instruction-set simulation | Out of scope |
| PCB layout, routing, parasitics extraction | Out of scope |
| Schematic capture GUI | Out of scope |
| Electromagnetic / field solvers (FEM, FDTD, method-of-moments) | Out of scope |
| Mechanical / multi-physics co-simulation | Out of scope |

For embedded-system design, the practical implication is that sindr can simulate the **analog portions** of a board (power stages, sensor front-ends, signal conditioning, filters) but not the digital MCU + firmware behaviour. To pair the two, run the firmware in your own emulator and drive the circuit with a `TransientSession`: advance it a tick at a time, set the pins' sources, read the ADC nodes, and stop on a threshold crossing for an interrupt.

## Not yet supported, but in scope

These are gaps a SPICE user would reasonably expect. They are plausible additions; none are committed.

### Netlist & ecosystem

- **SPICE netlist import: partial.** The `spice` feature (`sindr::spice`, on by default) parses a practical subset of SPICE3f5 decks — R, L, C, V, I, D, Q, M, J, B (behavioural, with the `VALUE` form of E/G and the `POLY` form of E/G/F/H), X, linear E/G/F/H, K (two coupled inductors; no three-winding coupling), plus `.subckt`, `.global`, `.model`, `.tran`, `.dc`, `.ac`, `.tf`, `.sens`, `.noise`, `.pz`, `.meas`, `.four`, `.op`, `.ic`, `.nodeset`, `.options`, `.param`, `.include`, `.lib` — into a `Circuit`. `.options` reads `RELTOL`, `VNTOL`, `TRTOL` and `ITL1`; any other option is rejected in strict mode and ignored with a warning in lenient mode. V and I lines take `[DC] v`, `AC [mag [phase]]` and a `PULSE`/`SIN`/`PWL` waveform in any order; `ParsedNetlist::ac_config` drives `.ac` from the one V line with an `AC` spec, so a deck with an `AC` spec on an I line or on several sources needs its `AcConfig` set by hand (AC analysis drives a single voltage source). Full SPICE3 coverage and the `TABLE`/`LAPLACE` forms of E/G/F/H are **not** implemented. `.model` cards are read only for what sindr's models have: `IS`, `N`, `RS`, `CJO`, `VJ`, `M`, `FC`, `TT`, `BV`, `IBV`, `KF`, `AF`, `EG`, `XTI` on diodes and `IS`, `BF`, `BR`, `NF`, `NR`, `VAF`, `KF`, `AF`, `EG`, `XTI` on BJTs. `EG` and `XTI` set how `IS` follows temperature, as SPICE's `IS(T)`; `TNOM` is read only at 27 °C, the temperature sindr's values are at. Temperature moves `IS` and the thermal voltage only: `VJ`, `CJO` and `BF` (`XTB`) stay at their 27 °C values. Anything else (BJT junction capacitance, Gummel–Poon high-injection terms) is rejected in strict mode and ignored with a warning in lenient mode. Vendor model libraries that rely on BSIM / Gummel-Poon model cards remain unreachable.
- **SPICE netlist export: partial.** `sindr::spice::to_netlist` writes R, L, C, V, I, D, Q, M, J, B, E, G, F, H and transformers (as L–L–K), plus exact equivalents for switches, fuses, potentiometers, photoresistors, thermistors, LEDs and Schottky diodes, and crystals, ferrite beads, SPDT switches, digital pins, logic gates and DC motors as the parts they're solved as. A MOSFET with flicker noise, IGBT, zener, op-amp, comparator, relay, photodiode, varactor, regulator, switching regulator, battery and a saturating inductor or transformer have no export, and neither does a junction away from 27 °C or a BJT with parasitic capacitances. No analysis cards or `AC` specs are written. sindr's parser reads every line it writes back.
- **Hierarchy: definitions with evaluated values only.** `Circuit` has blocks (each with optional variants of its body, which an instance picks), instances, global nets and part data, and the `spice` parser keeps `.subckt` structure. Blocks hold evaluated values, not parameter expressions, so a subckt whose instances bind different `.param`s becomes one block per distinct body (`name`, `name_2`, …). Reference designators are per block definition, with no per-instance annotation.
- **Model cards: Shockley and Ebers–Moll subsets only.** `Diode` and `Bjt` take a `DiodeModel` / `BjtModel` (which a SPICE `.model` card fills in), but there are no BSIM / Gummel-Poon / VBIC / HICUM models, MOSFET and JFET take level-1 `NMOS`/`PMOS` and `NJF`/`PJF` cards (a JFET's `VTO`, `BETA` and gate capacitances `CGS`, `CGD`, `PB`, `M`, `FC`), and the other devices take only their Rust parameter structs. A manufacturer card matches sindr's behaviour only when it uses no parameters beyond those. A diode's charge sits across its terminals in transient, at the junction voltage the diode current sets across RS; the drop the charging current itself makes across RS is left out, which matters only for large RS. AC is exact (RS in series with the junction). CJO and VJ don't scale with temperature, and the reverse leakage between 0 and breakdown is Shockley's `−IS` rather than ngspice's cubic. The BJT's Early effect (`VAF`) is `ic·(1 + vce/VAF)`, not SPICE's Gummel–Poon base charge, and `VAR` isn't modelled.
- **Behavioural sources are expressions, not a device language.** `CircuitElement::Behavioural` (SPICE's `B`) is one voltage or current set by an expression of node voltages, branch currents and time. There is no Verilog-A, no internal state or charge (`ddt`, `idt`), and no XSPICE code models; a new device with its own state means writing a Rust companion model. Steps and comparisons in an expression have zero slope, so a hard switch can stall Newton where a smooth one (`tanh`) converges. An expression has no device limiting (SPICE's `pnjlim`): an exponential written as a `B` source is reached by halving steps that overflow and then source stepping, so it takes more iterations than the built-in diode, and an expression's value is good to the solver's `reltol` rather than to rounding.

### Analyses

| Analysis | Supported | Notes |
|----------|-----------|-------|
| `.op` (operating point) | ✅ | via `solve_op`: capacitors open, inductors shorted, sources at their t = 0 value |
| `.dc` (DC sweep) | ✅ | one source (`dc_sweep`) or two nested (`dc_sweep_nested`), voltage or current; each point an operating point, as `.op` |
| `.tran` (transient) | ✅ | Gear-2, trapezoidal or backward Euler (`solve_transient`). Steps up to `tstep`, shortened by local truncation error and landing on waveform breakpoints. `tstart` keeps the result from there on |
| `.ac` (small-signal AC) | ✅ | swept frequency response |
| `.tf` (transfer function) | ✅ | DC small-signal gain, input and output resistance (`solve_tf`) |
| Temperature sweep (`temperature_sweep`) | ✅ | each junction device's `temperature`; no `.temp` card |
| `.noise` (noise analysis) | ✅ | thermal, shot and flicker (`solve_noise`), at 27 °C. Op-amps, regulators, controlled sources and a diode's `RS` are noiseless; no BJT base-resistance noise |
| `.disto` / harmonic distortion | ❌ | |
| `.pz` (pole-zero) | ✅ | poles and zeros of a voltage gain or transimpedance between node pairs (`solve_pz`). A repeated root is only accurate to about √ε; a root more than ~1e13 times the circuit's own frequency scale counts as infinite |
| `.sens` (sensitivity) | ⚠️ | DC (`solve_sensitivity`) and AC (`solve_ac_sensitivity`, `.sens ... ac`), for resistance, source values, all four controlled-source gains, and diode (IS, N, RS), BJT (BF, IS, BR, NF, NR, VAF) and MOSFET (KP, VTO, LAMBDA) model parameters; in AC also C, L, the stimulus's AC magnitude and a diode's CJO and TT. AC holds the bias point fixed, as ngspice does: a parameter's effect through the operating point (and so on every other device's small-signal conductances) is left out, and a source's DC value has no AC entry. No entries for LED, Schottky, photodiode, JFET or IGBT parameters, a diode's BV/IBV, VJ/M/FC, or a transformer. A zener, and a MOSFET's body diode (`<id>.body`), get a diode's IS, N and RS. A self-heated device is held at its balanced junction temperature (the thermal loop's dependence on the parameter is left out) |
| Monte Carlo / worst-case / corner analysis | ✅ | `variation::monte_carlo` and `variation::corners`, over resistance, capacitance, inductance and source values (uniform or Gaussian, independent, seeded). Specs (`check_specs`) and ratings (`ratings_over`) at their worst run. No correlated tolerances, no device-model parameters, no `.mc` / `.step` cards, no derating or SOA yet |
| `.four` (Fourier analysis) | ✅ | `sindr::fourier::Fourier`: harmonics, phases and THD of a transient signal, as ngspice's (last period, linear resample, 200 points by default). A deck's `.four f0 v(out) i(r1)` card parses into `ParsedNetlist::fourier`, one request per signal; ngspice's `nfreqs` and `fourgridsize` options aren't read, so a card always gets 9 harmonics on 200 points |
| `.fft` on transient output | ✅ | `sindr::fourier::Fft`: a radix-2 FFT of a linear resample onto a power of two of points, windowed (Hann by default; rectangular, Hamming, Blackman, flat-top), one-sided and scaled to peak amplitude. No zero padding, no averaging of segments (Welch) |
| `.meas` (measurements) | ⚠️ | `sindr::measure`: statistics, `FIND … AT`, `WHEN`, `TRIG`/`TARG`, plus rise/fall, settling and −3 dB in the API. No `FIND … WHEN`, `DERIV` or `PARAM=`; no currents on AC results |
| Periodic steady-state (PSS / shooting / harmonic balance) | ❌ | relevant for switching converters, RF |
| S-parameter / two-port extraction | ❌ | |
| Stability / loop-gain (Middlebrook, Tian) | ✅ | Tian's double injection at a 0 V probe source in series with the loop (`solve_loop_gain`): loop gain, crossovers, phase and gain margins. One break point at a time; no all-loops (`stb`-style) search |

### Device models

- **Op-amp macromodel: single pole.** `OpAmpModel` gives open-loop gain, a dominant pole from GBW, slew rate, output resistance, input offset and bias current, and a rail clamp. There is no second pole or phase margin, no output current limit, no supply current, no input common-mode range, no noise and no CMRR/PSRR. Vendor op-amp models can't be loaded. Without a `model` the op-amp is a gain-1e5 VCVS with no rail clamp.
- **Batteries are an equivalent circuit, with no chemistry.** `Battery` has an open-circuit voltage curve, a series resistance and one RC pair. Capacity doesn't fall at high current (Peukert) or in the cold, resistance doesn't rise as the cell empties, and there's no aging, self-discharge, charging limit or hysteresis between charge and discharge curves. A battery is noiseless in `.noise`, and SPICE export refuses it.
- **Regulators are linear and piecewise.** `VoltageRegulator` has dropout, quiescent current, a current limit and line and load regulation, positive or negative (a negative `voltage` mirrors the characteristic, for a 79xx), but no foldback, thermal shutdown, PSRR that falls with frequency, or start-up behaviour. It has no control loop either, so its output capacitor's value and ESR don't affect stability (an LDO that oscillates without its output capacitor on a real board is stable here) and don't change the load-step response; capacitors on a regulator show up only as inrush and hold-up. Its temperature doesn't feed back: self-heating reports its junction temperature, but its output doesn't drift with it. No references or adjustable regulators (LM317 with its divider).
- **Switching converters are averaged.** `SwitchingRegulator` (buck, boost) is its average over a switching period: no output ripple (only a `ripple_current` estimate), no control loop or compensation, so no load-step ringing or loop stability, no soft start, discontinuous-conduction mode, foldback or thermal shutdown. In a transient it reaches each operating point at once. Its efficiency is what you give it, not derived from switch and inductor losses. No buck-boost, flyback or inverting topologies; the switched circuit, built from switches, diodes and an inductor, covers those.
- **MOSFET is Level 1 only.** No BSIM (3 / 4 / 6 / -CMG), no EKV, no PSP. Short-channel effects, velocity saturation, mobility degradation and gate leakage are absent.
- **MOSFETs have three terminals, with the body tied to the source** (as in a discrete part), so the body effect (`gamma`, `phi`) applies only in reverse mode, when the drain is pulled past the source and the body sits forward-biased against the end acting as the source, as in SPICE. There's no fourth body terminal for IC-style biasing to a rail.
- **BJT is Ebers-Moll with optional Early voltage.** No Gummel-Poon, no VBIC, no HICUM. No high-injection, base-width modulation in saturation, charge-storage beyond the optional Cbe/Cbc parasitic caps.
- **JFET gate capacitances are depletion charge only, at 27 °C.** `parasitic_caps` gives CGS and CGD as SPICE's graded junctions (`PB`, `M`, `FC`), but the gate junction conducts no current, and `PB` and the capacitances don't follow temperature. ngspice's JFET grades at `M = 0.5` whatever the card says, so a deck with another `M` solves differently there.
- **A MOSFET's body diode is DC and junction charge only.** `body_diode` takes a full diode card (IS, N, RS, CJO, TT, BV), solved as a diode `<id>.body`; there's no separate reverse-recovery model beyond TT, and no body diode on IGBTs.
- **IGBT is a MOSFET square law.** `Igbt` is the Level-1 MOSFET current from gate to collector, cut off for reverse V_CE. There's no bipolar output stage, so no ~0.7 V on-state knee, no tail current at turn-off and no output conductance in saturation, and it has no gate or junction capacitances, so it switches as fast as its drive and load allow. `IgbtParams::vce_sat` is kept for old circuits but unused. Its conduction loss misses the knee's share, and its switching loss is what external capacitors give it, with no tail.
- **No SiC / GaN device models.**
- **Digital pins are IBIS's parametric buffer, DC plus `c_comp`.** A pin's output stage is a resistance to each rail and its clamps are Shockley diodes: no `.ibs` V-I or V-t tables, so a driver doesn't current-limit as a real one does into a short, and edges are only the RC of the output resistance and `c_comp`, with no ramp rate. There's no package R, L or C, no slew-rate control or drive-strength setting, no supply current from the chip's core, and the thresholds are fixed volts that don't follow a sagging supply. A session reads the input once per step.
- **Logic gates are a smooth input stage, a slew-limited delay and a pin's output stage.** Inputs switch over a tenth of the `vil`–`vih` band, at fixed volts that don't follow a sagging supply. `tpd` is one number for both edges, independent of load and supply, and the output's edge is the delay's slew plus the RC of the output resistance and the load. There's no quiescent or shoot-through supply current, no output enable or tri-state, and no flip-flops or latches; a Schmitt input's hysteresis is a 1 ns internal state.
- **Crystals and ferrite beads are linear networks.** A crystal is the Butterworth–Van Dyke model: no overtones, spurious modes, drive-level dependence or aging. A ferrite bead is R + L ∥ R (∥ C) with no DC-bias derating, so its impedance doesn't fall with current as a real bead's does.
- **Magnetics: saturating cores, no more.** `Inductor` can saturate along a smooth single-valued curve (`InductorSaturation`), and so can a `Transformer`'s magnetising branch, which carries the shared core flux behind each winding's leakage. There's no hysteresis, remanence, core loss or temperature dependence, a transformer has two windings, and SPICE export refuses a saturating core.
- **No transmission lines** (lossless or lossy / W-element).
- **DC motors are linear and brushed.** `DcMotor` has armature R and L, back-EMF, inertia, viscous friction and a load torque. There's no Coulomb or static friction (a stalled motor starts at any current), no brush drop, no armature reaction or magnetic saturation, no winding heating, and no commutation ripple. No brushless, stepper or servo motors.
- **No piezo / MEMS electro-mechanical coupling.** The DC motor is the one electromechanical element.
- **No switched-capacitor primitives** (sample-and-hold, charge-redistribution).

### Solver & numerics

- **Fixed-order integrators only:** Gear-2 (the default), trapezoidal and backward Euler, chosen with `TransientConfig::with_method`. No variable-order Gear. Gear-2 damps lightly-damped LC tanks slightly over many periods; trapezoidal keeps their amplitude but can ring on time constants much shorter than the step; backward Euler damps strongly.
- **Sparse LU above 100 unknowns** (`faer`), dense below. A 10 000-node resistor ladder solves in about 4 ms and a 100×100 RC mesh in about 15 ms per solve. The pattern is recorded and analysed once, and each later Newton iteration or step refactors numerically into it; a step that changes the pattern records it again. There's no partial refactorisation of only the columns that changed, and a linear transient still refactors at every step.
- **Convergence aids: gmin stepping, then source stepping.** When plain Newton–Raphson fails on a nonlinear DC solve, sindr retries with a gmin-stepping homotopy (`1e-2` S down to the `1e-12` floor), then by ramping every source up from zero, warm-starting between steps. `SimulationResult::convergence_aid` says which one was needed. No pseudo-transient continuation. `ConvergenceFailed` names the node that won't settle, the components on it and the point of the analysis (`SolvePoint`), with the last Newton step (not a KCL residual).
- **`.nodeset`** (`SolverOptions::nodeset`) seeds the DC operating point and the bias point of AC, noise, `.tf`, `.sens` and every sweep, through a `Solver`.
- **A transient starts switched on at t = 0 unless asked otherwise:** `TransientConfig::new` starts from the initial conditions alone (SPICE's `UIC`), where SPICE starts from the DC operating point. `TransientStart::OperatingPoint` does what SPICE does, and a deck's `.tran` maps onto it (`uic` onto the other). The operating point it starts from is solved without self-heating. Initial winding currents on a transformer can't be set, so a coupled SPICE `L` line takes no `IC=`, and a `C` line's `IC=` is read only with one end at ground, as the other end's starting voltage, which every capacitor on that node then starts from. With `tstart`, sindr's first snapshot is at `tstart` itself, where ngspice's is its first step past it.
- **Truncation-error control is per node voltage** (SPICE-style RELTOL 1e-3, VNTOL 1 µV, TRTOL 7 by default; set them in `SolverOptions`), not per charge or flux. The step never exceeds `tstep`, so a coarse `tstep` buys speed only between edges.
- **No checkpointing / restart** of long transient runs.
- **Large transients are slower than ngspice.** On a 1 000-node RC ladder, 200 steps take 72 ms in sindr and about 32 ms in ngspice's solve (2.2×, `examples/vs_ngspice.rs`). Each step stamps into a fixed sparse layout and refactors it numerically; most of the gap left is recording each step's results by name. sindr wins up to several hundred nodes, where ngspice's 14 ms start dominates, and on DC operating points at every size measured (a 3 000-diode chain: 9.6 ms against 87 ms). Long transients of thousands of nodes are ngspice's ground.
- **No multithreading** inside one solve. Separate solves do run in parallel threads.
- **Automatic differentiation for behavioural sources only.** A `Behavioural` source's Jacobian comes from forward-mode automatic differentiation of its expression. Device models are differentiated by hand, and so are the sensitivities: adjoint sensitivity (DC, and AC at a fixed bias) covers the parameters in the table above. There's no differentiation of a whole circuit with respect to arbitrary inputs.
- **No interval / affine arithmetic** for guaranteed bounds.

### Self-heating & thermal

- **Self-heating is one thermal RC per part.** A part with `Part::thermal` runs at `ambient + P·Rθ` (iterated to a balance in DC, a thermal RC state in transient). There's no thermal coupling between parts (a shared heatsink, neighbours on a board), and no multi-pole Foster/Cauer ladder for the junction-to-case path.
- **Only junction devices and the thermistor feel their temperature:** diodes, LEDs, Zeners, Schottkys, photodiodes, BJTs and thermistors. MOSFETs, JFETs, IGBTs and resistors report a junction temperature (and so get Tj-max checks), but their models have no temperature in them, so heating doesn't move their operating point. Power-MOSFET and IGBT runaway is not predicted.
- **An LED's temperature coefficient is one typical figure:** Vf falls 2 mV/K at its rated 20 mA for every colour, from an effective EG per colour. Real parts range from about −1.5 to −4 mV/K.
- **Self-heating applies to `solve_op`, `solve_circuit`, `solve_transient` and `dc_sweep`** (and the same on a `Solver`), to `temperature_sweep`, which sets the ambient, and to the bias point of AC, `.tf`, `.sens` and `.noise`, which linearise each device at its balanced junction temperature. Each DC sweep point balances on its own.
- In a transient the electrical step solves at the temperature from the step before. Thermal time constants are far longer than the electrical steps, so the lag doesn't matter unless the step is coarse compared with Rθ·Cθ.

### Tooling & UX

- **No CLI**. sindr is a library only.
- **No schematic input**. Circuits are written as Rust `Vec<CircuitElement>` (or JSON via serde).
- **No parsers for editors' own netlist files.** `sindr::import` takes a netlist an editor has already read (components, values, nets), not the editor's file format.
- **No part-number lookup.** `sindr::import` builds a model from what a part's data says (a colour, datasheet figures such as `vf` at `if`, `hfe`, `vce_sat`, `vth`, `rds_on`), not from its part number: a `PMBT3904` with no figures gets the default NPN, with a warning in `ImportedCircuit::warnings`.
- **Datasheet fits are as simple as the models.** A BJT's gain is the same at every collector current (`hfe_ic` doesn't move it), and with no collector resistance in the model a Vce(sat) fit gives a low BR. A MOSFET fit is Level 1 through one threshold and one on-resistance point, so the transfer curve between them is a square law. A zener fit shifts only the breakdown voltage; its 1 Ω dynamic resistance is fixed. An LED given by `vf` at `if` imports as a diode element with the fitted model, not as an LED element.
- **ICs import as black boxes.** A part `sindr::import` has no kind for (a microcontroller, a timer) can come in as a `ComponentKind::BlackBox`, which simulates nothing: its pins draw and drive no current, so a net only it holds up floats. Model the part's behaviour with bench sources and loads, or leave it out of the analysis.
- **Waveform export is CSV only.** Transient, AC and sweep results have `to_csv()`; there's no VCD, Touchstone or Common Simulation Data Format.
- **No built-in plotting**. Examples like `bode_plot.rs` produce data; rendering is on the user.
- **No interactive REPL or notebook integration.**
- **No `no_std` support.** `nalgebra` and `std::collections::HashMap` are pulled in.

## Numerical caveats worth knowing

- Transformer coupling `k = 1.0` is mathematically singular — clamp to `≤ 0.999`.
- Switches are modelled as `0.01 Ω` / `1 GΩ` resistors — they are not ideal, and very high-impedance circuits may need explicit values.
- An op-amp without a `model` is a VCVS with gain `1e5` and no frequency response or rail clamp, so its closed-loop bandwidth isn't realistic. Give the op-amp an `OpAmpModel` for GBW and slew rate. Its stage starts from 0 V at t = 0, like the capacitors, so a transient opens with a short settle unless you seed it.
- A comparator has gain `1e5` and its output clamped to `v_neg`/`v_pos`, but no propagation delay, hysteresis or open-collector output: it switches the instant its inputs cross, and a slow, noisy input can chatter.
- For accurate ringing amplitude in a resonant circuit, use `IntegrationMethod::Trapezoidal` with a step well below the period: Gear-2 and especially backward Euler damp it.
- Photodiode / photoresistor / thermistor "stimulus" parameters (irradiance, light level, temperature) have no waveform: they are constant through a `solve_transient`. A `TransientSession` can change them between steps (`set_param`).

## Useful workarounds

- Need a vendor BJT / MOSFET? A level-1 MOSFET or JFET subcircuit loads as is; otherwise fit Ebers-Moll / Level-1 parameters from the datasheet. Accuracy will be modest.
- Need digital control around an analog plant? Drive it with a `TransientSession` from your own loop: advance a control period, read node voltages, set sources, switches and pots, and use `run_until` to stop where a comparator would trip.
- Need a second pole or phase margin on an op-amp? Build it from VCVS stages and RC networks around an `OpAmp` with an `OpAmpModel`.
- Need a tolerance on a device-model parameter, or correlated parts? Loop in Rust over your own variations of the `Circuit`; sindr is fast enough for thousands of solves on small circuits.

## Reporting gaps

If a missing capability is blocking real work, open an issue describing the **circuit and analysis you want to run**, not just the feature name — concrete use cases drive prioritisation.
