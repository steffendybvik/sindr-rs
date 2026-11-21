# Roadmap

Where sindr is heading from 0.1.0 to 1.0. It's a plan, not a promise of
dates. For what sindr does today see [`README.md`](./README.md); for what it
doesn't, [`LIMITATIONS.md`](./LIMITATIONS.md); for what shipped,
[`CHANGELOG.md`](./CHANGELOG.md). Contributions are welcome: start with
[`CONTRIBUTING.md`](./CONTRIBUTING.md), and open an issue to talk through
anything bigger than a fix.

## What sindr is for

sindr is a circuit simulator you embed. A circuit is a typed Rust value,
results are structs, and errors are an enum you can match on. It runs
natively and in the browser (`wasm32-unknown-unknown`), has no global state,
and uses no `unsafe`. It answers three questions for the application around
it:

1. **What happens electrically?** Operating point, DC sweep, AC, transient,
   noise, `.tf` and `.sens`.
2. **Will the parts survive?** Absolute-maximum ratings and self-heating.
3. **Does the design meet its spec?** Measurements and pass/fail specs with
   margins.

ngspice is the ruler, not the rival. Results are checked against ngspice
golden files, and a model counts as done when it agrees.

**Not on the roadmap:** a full SPICE dialect, a schematic or PCB editor, a
waveform viewer, HDL or digital simulation, BSIM-class models, or a parts
library of sindr's own.

## The bar every release meets

Checked in CI, so no release lowers it:

- every ngspice golden and the mathematical suite (KCL residuals, power
  balance, closed forms) pass;
- each source file stays above its line-coverage floor;
- no benchmark gets more than 25 % slower;
- `cargo-semver-checks` agrees with the version bump;
- stable, MSRV and `wasm32-unknown-unknown` all build and pass.

## Milestones

| | Milestone | Version | What it means |
|---|---|---|---|
| 1 | **Editor-ready** | 0.1.x | An editor's netlist and part data simulate without the user writing a deck or a model parameter |
| 2 | **Where it will fail** | 0.1.x | Design verification good enough to stake a board on |
| 3 | **Cheap devices** | 0.1.x | Adding a device touches one place in the code |
| 4 | **Typed API** | 0.2 | Wrong circuits are hard to build, and errors are typed |
| 5 | **Extensible models** | 0.3 | Devices from outside the crate, and the models boards need |
| 6 | **1.0** | 1.0 | Stable for the whole of 1.x |

Milestones 1–3 only add to the API, so they ship as 0.1.x patches and can
overlap.

### 1. Editor-ready

- Real boards, expressed as an editor exports them (components, values as
  text, nets) plus a test bench, simulated end to end through
  `sindr::import` and kept as reference designs in the test suite.
- Models built from the numbers a parts library holds: an LED's forward
  voltage at a current, a transistor's gain, a MOSFET's threshold and
  on-resistance. Each is checked to reproduce its datasheet points.
- Results say when a component fell back to a default model, the way a part
  with no ratings reports *Unverified*, so nothing defaults silently.
- A way to describe what an IC pin looks like electrically (a driven level, a
  supply current), and clear errors for what sindr can't simulate.

**Done when** example boards simulate from a netlist and a bench with no model
parameter in the input, checked against ngspice or a closed form.

### 2. Where it will fail

- Worst case: corners and Monte Carlo over component tolerances.
- Derating: checking against a derated limit, not only the absolute maximum.
- Temperature-dependent ratings, using the junction temperatures
  self-heating already computes.
- Safe-operating-area checks for transistors.
- Each violation names the condition it happens under (the corner, the time,
  the temperature) and how severe it is.

**Done when** ratings and spec checks report the worst part and its margin
across tolerances and temperature, each new check verified against a
datasheet example.

### 3. Cheap devices

Each device's behaviour (DC, AC, transient, results, ratings, export) in one
place, internal only, with every numerical result unchanged.

### 4. Typed API (0.2)

- Typed node and component handles; names stay as labels, and an unknown name
  is an error at build time instead of a silent new node.
- Typed error causes instead of message strings.
- Sweeps configured the same way as every other analysis.

**Done when** no public error is a bare string, and a migration guide is in
the CHANGELOG.

### 5. Extensible models (0.3)

- A public interface for devices defined outside sindr, designed and
  prototyped before it's committed, with an external-device example crate.
- Gummel–Poon BJTs, behavioural (`B`) sources, JFET capacitances and a
  saturating transformer core.

### 6. 1.0

1.0 ships when all of these hold:

1. **Stable API.** Typed handles, `#[non_exhaustive]` where it matters,
   internals hidden, solver options, `cargo-semver-checks` in CI.
2. **Common parts covered.** Every common schematic symbol maps to an element
   with an ngspice golden or a hand-derived check.
3. **The analyses an editor needs.** Operating point, DC sweep, AC and
   transient, plus ratings checks, all checked against ngspice.
4. **Never panics, always explains.** Any input gives a result or a typed
   error that names the part. The SPICE parser is fuzzed.
5. **Fast enough to run live.** The operating point of a 100-part board under
   1 ms, a small transient under 100 ms, and the browser within 3× native.
6. **Used in a real tool.** An editor imports its own netlist through sindr's
   import API and simulates its boards end to end.

## Versions

Plain `0.x` from 0.1.0: a breaking change bumps the minor version, anything
else the patch. The last `0.x` before 1.0 is the freeze candidate.

## After 1.0

All additive, so fine within 1.x:

- Layout-aware physics from a board's stackup: trace impedance, thermal,
  power integrity
- Electrical rule checks an editor can run on a schematic
- Pole-zero and loop gain, AC sensitivity
- Higher-level MOSFET models, if boards need them
