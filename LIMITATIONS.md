# Scope & Limitations

sindr is an **analog circuit simulator**: a SPICE-style MNA solver with semiconductor companion models, written in pure Rust. This document is the honest counterpart to the READMEs — it lists capabilities sindr does **not** have today, so you can decide whether it fits your use case before investing.

Status: pre-release alpha (`0.1.0-alpha.5`). Anything below may change.

## What sindr is

- DC, nonlinear DC (Newton-Raphson), transient (Backward Euler), AC, DC sweep, temperature sweep
- Passives, sources, switches, controlled sources, op-amp / comparator macromodels, transformer
- Diode family (Si, LED, Zener, Schottky, photodiode, varactor), BJT (Ebers-Moll), MOSFET (Level-1), IGBT, JFET
- Thermistor, photoresistor, potentiometer, relay
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

- **No SPICE netlist import.** `.cir` / `.net` / `.lib` / `.subckt` files cannot be loaded. Vendor model libraries (TI, Infineon, ON Semi, Wolfspeed, etc.) are unreachable.
- **No SPICE netlist export.** Cannot hand a circuit off to ngspice / LTspice / Xyce.
- **No subcircuit / hierarchical netlists.** Every circuit is a flat `Vec<CircuitElement>`. There is no `.subckt`-style block, no parameter-passing instance, no hierarchical node naming.
- **No model cards.** Devices take parameters as Rust structs, not BSIM / Gummel-Poon / VBIC / HICUM cards. There is no way to point at a manufacturer model and get matching behaviour.
- **No Verilog-A / behavioural device language.** Adding a new device means writing a Rust companion model.

### Analyses

| Analysis | Supported | Notes |
|----------|-----------|-------|
| `.op` (operating point) | ✅ | via `solve_circuit` |
| `.dc` (DC sweep) | ✅ | one source at a time |
| `.tran` (transient) | ✅ | Backward Euler, adaptive dt |
| `.ac` (small-signal AC) | ✅ | swept frequency response |
| `.temp` (temperature sweep) | ✅ | junction-device temp parameter |
| `.noise` (noise analysis) | ❌ | no thermal/shot/flicker noise stamps |
| `.disto` / harmonic distortion | ❌ | |
| `.pz` (pole-zero) | ❌ | |
| `.sens` (sensitivity) | ❌ | |
| Monte Carlo / worst-case / corner analysis | ❌ | no parameter-distribution sampling |
| `.fft` on transient output | ❌ | user-side only |
| Two-source / nested DC sweep | ❌ | single source per call |
| Periodic steady-state (PSS / shooting / harmonic balance) | ❌ | relevant for switching converters, RF |
| S-parameter / two-port extraction | ❌ | |
| Stability / loop-gain (middlebrook, Tian) | ❌ | |

### Device models

- **No op-amp macromodel beyond ideal-with-rails.** No GBW, slew rate, input bias current, offset, output impedance, current limiting, or supply current. Real op-amp models from vendors cannot be loaded.
- **No regulator / reference / DC-DC controller models.**
- **MOSFET is Level 1 only.** No BSIM (3 / 4 / 6 / -CMG), no EKV, no PSP. Short-channel effects, velocity saturation, mobility degradation, body effect, gate leakage are absent.
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

- **Backward Euler is the only transient integrator.** No trapezoidal, no Gear-2, no variable-order. BE is L-stable but numerically dissipative — expect amplitude decay on lightly-damped LC tanks. This is not tunable.
- **Dense LU factorisation** (via `nalgebra`). No sparse solver. Practical circuit size is bounded by `O(N²)` memory and `O(N³)` factorisation; large boards (hundreds of nodes) will be slow.
- **Convergence aids: gmin stepping + Newton damping only.** When plain Newton–Raphson fails on a nonlinear DC solve, sindr automatically retries with a gmin-stepping homotopy (geometric ladder from `1e-2` down to the `1e-12` floor, warm-starting between steps). No source stepping, no pseudo-transient continuation, no general homotopy. `ConvergenceFailed` carries the iteration count and final residual to help diagnose.
- **`.nodeset`-style seeding via `solve_circuit_with_initial_voltages`.** Pass a `HashMap<String, f64>` of node-name → voltage to pre-warm the Newton initial guess. No `.ic` (force-fixed initial conditions) directive yet.
- **No `uic` (use-initial-conditions) skip-DC option** for transient.
- **No checkpointing / restart** of long transient runs.
- **No multithreading** in the solver.
- **No automatic differentiation / adjoint sensitivity.**
- **No interval / affine arithmetic** for guaranteed bounds.

### Self-heating & thermal

- Temperature is a **per-device parameter**, not a state variable. The temp-sweep analysis re-solves at each temperature; it does not couple junction power dissipation back into junction temperature within a single solve.
- **No electro-thermal network**, no thermal RC ladders, no `T(t)` co-simulated with `V(t)`. This matters for IGBT / power-MOSFET / LED designs where self-heating shifts the operating point.

### Tooling & UX

- **No CLI**. sindr is a library only.
- **No schematic input**. Circuits are written as Rust `Vec<CircuitElement>` (or JSON via serde).
- **No KiCad / Eagle / Altium netlist import.**
- **No waveform export** to VCD, CSV, Touchstone, or Common Simulation Data Format. Users marshal `TransientData` themselves.
- **No built-in plotting**. Examples like `bode_plot.rs` produce data; rendering is on the user.
- **No interactive REPL or notebook integration.**
- **No web / WASM target tested.** It may build, but it is not a supported configuration.
- **No `no_std` support.** `nalgebra` and `std::collections::HashMap` are pulled in.

## Numerical caveats worth knowing

- Transformer coupling `k = 1.0` is mathematically singular — clamp to `≤ 0.999`.
- Switches are modelled as `0.01 Ω` / `1 GΩ` resistors — they are not ideal, and very high-impedance circuits may need explicit values.
- The op-amp / comparator macromodel is a saturating VCVS with gain `1e5` — it has no frequency response, so AC and transient closed-loop bandwidth predictions are not realistic.
- Backward Euler will under-damp resonant tanks; if you need accurate ringing amplitude, this is the wrong tool today.
- Photodiode / photoresistor / thermistor "stimulus" parameters (irradiance, light level, temperature) are **constants per solve**, not time-varying inputs. There is no environmental waveform.

## Useful workarounds

- Need a vendor BJT / MOSFET? Fit Ebers-Moll / Level-1 parameters from the datasheet. Accuracy will be modest.
- Need digital control around an analog plant? Drive the analog circuit step-by-step from your own Rust loop, sampling node voltages between transient calls and updating sources / switches between them. This is not co-simulation, but it works for slow control loops.
- Need a Bode plot of a closed loop with a real op-amp? Replace the `OpAmp` element with a manually-built VCVS + RC pole network that approximates the part you care about.
- Need Monte Carlo? Loop in Rust over randomised `CircuitElement` parameter values; sindr is fast enough for thousands of solves on small circuits.

## Reporting gaps

If a missing capability is blocking real work, open an issue describing the **circuit and analysis you want to run**, not just the feature name — concrete use cases drive prioritisation.
