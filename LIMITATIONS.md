# Scope & Limitations

sindr is an **analog circuit simulator**: a SPICE-style MNA solver with semiconductor companion models, written in pure Rust. This document is the honest counterpart to the READMEs — it lists capabilities sindr does **not** have today, so you can decide whether it fits your use case before investing.

Status: pre-release alpha (`0.1.0-alpha.8`). Anything below may change.

## What sindr is

- DC, nonlinear DC (Newton-Raphson), transient (Gear-2 (default), trapezoidal or backward Euler), AC, DC sweep, temperature sweep
- Passives, sources, switches, controlled sources, op-amp / comparator macromodels, transformer
- Diode family (Si, LED, Zener, Schottky, photodiode, varactor), BJT (Ebers-Moll), MOSFET (Level-1), IGBT, JFET
- Thermistor, photoresistor, potentiometer, relay, fuse, linear voltage regulator (dropout, quiescent current, current limit, line and load regulation)
- Programmatic Rust API + serde JSON for circuits and results

If your problem is "given an analog topology built in Rust, what are the node voltages and branch currents (possibly over time, frequency, or temperature)?", sindr is in scope.

## Out of scope (by design, today)

These are non-goals for the current crate split. They may live in sibling crates later, but they will not appear in `sindr` itself.

| Area | Status |
|------|--------|
| Digital logic simulation (gates, flip-flops, clocked sequential, event-driven kernel) | Out of scope |
| HDL ingestion (Verilog, VHDL, SystemVerilog, Verilog-A, VHDL-AMS, SystemC-AMS) | Out of scope |
| Microcontroller / instruction-set simulation | Out of scope |
| PCB layout, routing, parasitics extraction | Out of scope |
| Schematic capture GUI | Out of scope |
| Electromagnetic / field solvers (FEM, FDTD, method-of-moments) | Out of scope |
| Mechanical / multi-physics co-simulation | Out of scope |

For embedded-system design, the practical implication is that sindr can simulate the **analog portions** of a board (power stages, sensor front-ends, signal conditioning, filters) but not the digital MCU + firmware behaviour. Co-simulation with a digital tool is not supported.

## Not yet supported, but in scope

These are gaps a SPICE user would reasonably expect. They are plausible additions; none are committed.

### Netlist & ecosystem

- **SPICE netlist import: partial.** The optional `spice` feature (`sindr::spice`) parses a practical subset of SPICE3f5 decks — R, L, C, V, I, D, Q, X, plus `.subckt`, `.model`, `.tran`, `.dc`, `.ac`, `.tf`, `.sens`, `.op`, `.ic`, `.nodeset`, `.param`, `.include`, `.lib` — into a `Circuit`. Full SPICE3 coverage and behavioural (`B`) sources are **not** implemented. `.model` cards are read only for what sindr's models have: `IS`, `N`, `RS`, `CJO`, `VJ`, `M`, `FC`, `TT`, `BV`, `IBV` on diodes and `IS`, `BF`, `BR`, `NF`, `NR`, `VAF` on BJTs. Anything else (temperature coefficients such as `EG`/`XTI`, noise terms, BJT junction capacitance, Gummel–Poon high-injection terms) is rejected in strict mode and ignored with a warning in lenient mode. Vendor model libraries (TI, Infineon, ON Semi, Wolfspeed, etc.) that rely on BSIM / Gummel-Poon model cards remain unreachable.
- **SPICE netlist export: partial.** `sindr::spice::to_netlist` writes R, L, C, V, I, D, Q, E, G, F, H and transformers (as L–L–K), plus exact equivalents for switches, fuses, potentiometers, photoresistors, thermistors, LEDs and Schottky diodes. MOSFET, JFET, IGBT, zener, op-amp, comparator, relay, photodiode, varactor and regulator have no export, and neither does a junction away from 27 °C or a BJT with parasitic capacitances. No analysis cards are written. sindr's parser reads back only the R, L, C, V, I, D, Q part.
- **Hierarchy: definitions with evaluated values only.** `Circuit` has blocks, instances, global nets and part data (`sindr-core`), and the `spice` parser keeps `.subckt` structure. Blocks hold evaluated values, not parameter expressions, so a subckt whose instances bind different `.param`s becomes one block per distinct body (`name`, `name_2`, …). Reference designators are per block definition, with no per-instance annotation. The parser doesn't read `.global`, although export writes it.
- **Model cards: Shockley and Ebers–Moll subsets only.** `Diode` and `Bjt` take a `DiodeModel` / `BjtModel` (which a SPICE `.model` card fills in), but there are no BSIM / Gummel-Poon / VBIC / HICUM models, and MOSFET, JFET and the other devices take only their Rust parameter structs. A manufacturer card matches sindr's behaviour only when it uses no parameters beyond those. A diode's charge sits across its terminals in transient, at the junction voltage the diode current sets across RS; the drop the charging current itself makes across RS is left out, which matters only for large RS. AC is exact (RS in series with the junction). CJO and VJ don't scale with temperature, and the reverse leakage between 0 and breakdown is Shockley's `−IS` rather than ngspice's cubic. The BJT's Early effect (`VAF`) is `ic·(1 + vce/VAF)`, not SPICE's Gummel–Poon base charge, and `VAR` isn't modelled.
- **No Verilog-A / behavioural device language.** Adding a new device means writing a Rust companion model.

### Analyses

| Analysis | Supported | Notes |
|----------|-----------|-------|
| `.op` (operating point) | ✅ | via `solve_circuit` |
| `.dc` (DC sweep) | ✅ | one source at a time |
| `.tran` (transient) | ✅ | Gear-2, trapezoidal or backward Euler (`solve_transient`). Steps up to `tstep`, shortened by local truncation error and landing on waveform breakpoints. `tstart` ignored |
| `.ac` (small-signal AC) | ✅ | swept frequency response |
| `.tf` (transfer function) | ✅ | DC small-signal gain, input and output resistance (`solve_tf`) |
| `.temp` (temperature sweep) | ✅ | junction-device temp parameter |
| `.noise` (noise analysis) | ❌ | no thermal/shot/flicker noise stamps |
| `.disto` / harmonic distortion | ❌ | |
| `.pz` (pole-zero) | ❌ | |
| `.sens` (sensitivity) | ⚠️ | DC only (`solve_sensitivity`), for resistance, source values, VCVS gain and VCCS gm. No device-model parameters, CCVS or CCCS gains, or AC sensitivity |
| Monte Carlo / worst-case / corner analysis | ❌ | no parameter-distribution sampling |
| `.fft` on transient output | ❌ | user-side only |
| Two-source / nested DC sweep | ❌ | single source per call |
| Periodic steady-state (PSS / shooting / harmonic balance) | ❌ | relevant for switching converters, RF |
| S-parameter / two-port extraction | ❌ | |
| Stability / loop-gain (middlebrook, Tian) | ❌ | |

### Device models

- **Op-amp macromodel: single pole.** `OpAmpModel` gives open-loop gain, a dominant pole from GBW, slew rate, output resistance, input offset and bias current, and a rail clamp. There is no second pole or phase margin, no output current limit, no supply current, no input common-mode range, no noise and no CMRR/PSRR. Vendor op-amp models can't be loaded. Without a `model` the op-amp is a gain-1e5 VCVS with no rail clamp.
- **Regulators are linear and piecewise.** `VoltageRegulator` has dropout, quiescent current, a current limit and line and load regulation, but no foldback, thermal shutdown, PSRR that falls with frequency, or start-up behaviour. No references, adjustable regulators (LM317 with its divider) or DC-DC controllers.
- **MOSFET is Level 1 only.** No BSIM (3 / 4 / 6 / -CMG), no EKV, no PSP. Short-channel effects, velocity saturation, mobility degradation and gate leakage are absent.
- **MOSFETs have three terminals, with the body tied to the source** (as in a discrete part), so the body effect (`gamma`, `phi`) never applies in a circuit. There's no fourth body terminal for IC-style biasing to a rail.
- **BJT is Ebers-Moll with optional Early voltage.** No Gummel-Poon, no VBIC, no HICUM. No high-injection, base-width modulation in saturation, charge-storage beyond the optional Cbe/Cbc parasitic caps.
- **No JFET parasitic capacitances.** (BJT and MOSFET have optional Cbe/Cbc and Cgs/Cgd; JFET doesn't.)
- **No power-MOSFET reverse body diode** as a separate stamped element (must be added explicitly as a `Diode`).
- **No SiC / GaN device models.**
- **No magnetics with saturation / hysteresis.** `Transformer` is linear coupled inductors. No B-H curve, no core loss, no saturating inductor, no gapped core.
- **No transmission lines** (lossless or lossy / W-element).
- **No crystal / quartz resonator macromodel** (must be built from L/C/R by hand).
- **No piezo / MEMS / electro-mechanical coupling.**
- **No batteries with state-of-charge / equivalent-circuit aging.** Only ideal voltage/current sources.
- **No switched-capacitor primitives** (sample-and-hold, charge-redistribution).

### Solver & numerics

- **Fixed-order integrators only:** Gear-2 (the default), trapezoidal and backward Euler, chosen with `TransientConfig::with_method`. No variable-order Gear. Gear-2 damps lightly-damped LC tanks slightly over many periods; trapezoidal keeps their amplitude but can ring on time constants much shorter than the step; backward Euler damps strongly.
- **Sparse LU above 100 unknowns** (`faer`), dense below. A 10 000-node resistor ladder solves in about 4 ms and a 100×100 RC mesh in about 15 ms per solve. No KLU-style partial refactorisation.
- **Convergence aids: gmin stepping, then source stepping.** When plain Newton–Raphson fails on a nonlinear DC solve, sindr retries with a gmin-stepping homotopy (`1e-2` S down to the `1e-12` floor), then by ramping every source up from zero, warm-starting between steps. `SimulationResult::convergence_aid` says which one was needed. No pseudo-transient continuation. `ConvergenceFailed` carries the iteration count and the last Newton step (not a KCL residual) to help diagnose.
- **`.nodeset`** seeds the DC operating point (`solve_circuit_with_initial_voltages`) and the bias point of AC, `.tf` and `.sens` (`solve_ac_with_initial_voltages`, `solve_tf_with_initial_voltages`, `solve_sensitivity_with_initial_voltages`).
- **A transient starts switched on at t = 0 unless asked otherwise:** `TransientConfig::new` starts from the initial conditions alone (SPICE's `UIC`), where SPICE starts from the DC operating point. `TransientStart::OperatingPoint` does what SPICE does, and a deck's `.tran` maps onto it (`uic` onto the other). The operating point it starts from is solved without self-heating. Initial winding currents on a transformer can't be set, and inductor `IC=` on SPICE element lines isn't parsed. `.tran`'s `tstart` is read but not applied.
- **Truncation-error control is per node voltage** (SPICE-style RELTOL 1e-3, VNTOL 1 µV, TRTOL 7), not per charge or flux, and the tolerances can't be changed yet. The step never exceeds `tstep`, so a coarse `tstep` buys speed only between edges.
- **No checkpointing / restart** of long transient runs.
- **No multithreading** in the solver.
- **No automatic differentiation.** Adjoint sensitivity exists for DC only, over the parameters in the table above.
- **No interval / affine arithmetic** for guaranteed bounds.

### Self-heating & thermal

- **Self-heating is one thermal RC per part.** A part with `Part::thermal` runs at `ambient + P·Rθ` (iterated to a balance in DC, a thermal RC state in transient). There's no thermal coupling between parts (a shared heatsink, neighbours on a board), and no multi-pole Foster/Cauer ladder for the junction-to-case path.
- **Only junction devices and the thermistor feel their temperature:** diodes, LEDs, Zeners, Schottkys, photodiodes, BJTs and thermistors. MOSFETs, JFETs, IGBTs and resistors report a junction temperature (and so get Tj-max checks), but their models have no temperature in them, so heating doesn't move their operating point. Power-MOSFET and IGBT runaway is not predicted.
- **An LED's temperature coefficient is one typical figure:** Vf falls 2 mV/K at its rated 20 mA for every colour, from an effective EG per colour. Real parts range from about −1.5 to −4 mV/K.
- **Self-heating applies to `solve_circuit`, `solve_circuit_with_initial_voltages` and `solve_transient`** (and `temperature_sweep`, which sets the ambient). AC, `.tf`, `.sens` and `dc_sweep` linearise around the unheated operating point.
- In a transient the electrical step solves at the temperature from the step before. Thermal time constants are far longer than the electrical steps, so the lag doesn't matter unless the step is coarse compared with Rθ·Cθ.

### Tooling & UX

- **No CLI**. sindr is a library only.
- **No schematic input**. Circuits are written as Rust `Vec<CircuitElement>` (or JSON via serde).
- **No KiCad / Eagle / Altium netlist import.**
- **Waveform export is CSV only.** Transient, AC and sweep results have `to_csv()`; there's no VCD, Touchstone or Common Simulation Data Format.
- **No built-in plotting**. Examples like `bode_plot.rs` produce data; rendering is on the user.
- **No interactive REPL or notebook integration.**
- **No web / WASM target tested.** It may build, but it is not a supported configuration.
- **No `no_std` support.** `nalgebra` and `std::collections::HashMap` are pulled in.

## Numerical caveats worth knowing

- Transformer coupling `k = 1.0` is mathematically singular — clamp to `≤ 0.999`.
- Switches are modelled as `0.01 Ω` / `1 GΩ` resistors — they are not ideal, and very high-impedance circuits may need explicit values.
- An op-amp without a `model`, and every comparator, is a VCVS with gain `1e5` and no frequency response or rail clamp, so its closed-loop bandwidth isn't realistic. Give the op-amp an `OpAmpModel` for GBW and slew rate. Its stage starts from 0 V at t = 0, like the capacitors, so a transient opens with a short settle unless you seed it.
- For accurate ringing amplitude in a resonant circuit, use `IntegrationMethod::Trapezoidal` with a step well below the period: Gear-2 and especially backward Euler damp it.
- Photodiode / photoresistor / thermistor "stimulus" parameters (irradiance, light level, temperature) are **constants per solve**, not time-varying inputs. There is no environmental waveform.

## Useful workarounds

- Need a vendor BJT / MOSFET? Fit Ebers-Moll / Level-1 parameters from the datasheet. Accuracy will be modest.
- Need digital control around an analog plant? Drive the analog circuit step-by-step from your own Rust loop, sampling node voltages between transient calls and updating sources / switches between them. This is not co-simulation, but it works for slow control loops.
- Need a second pole or phase margin on an op-amp? Build it from VCVS stages and RC networks around an `OpAmp` with an `OpAmpModel`.
- Need Monte Carlo? Loop in Rust over randomised `CircuitElement` parameter values; sindr is fast enough for thousands of solves on small circuits.

## Reporting gaps

If a missing capability is blocking real work, open an issue describing the **circuit and analysis you want to run**, not just the feature name — concrete use cases drive prioritisation.
