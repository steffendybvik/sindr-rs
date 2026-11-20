# Roadmap

Where sindr is heading on the way to 1.0. It's a plan, not a promise of
dates. For what sindr does today see [`README.md`](./README.md); for what it
doesn't, [`LIMITATIONS.md`](./LIMITATIONS.md). Contributions are welcome:
start with [`CONTRIBUTING.md`](./CONTRIBUTING.md), and open an issue to talk
through anything bigger than a fix.

## What sindr is for

sindr is a circuit simulator you embed. A circuit is a typed Rust value,
results are structs, and errors are an enum you can match on. It runs
natively and in the browser (`wasm32-unknown-unknown`), has no global state,
and uses no `unsafe`. It answers design questions, not only waveforms:
whether a part stays inside its ratings, whether a spec passes, which part
fails.

ngspice is the ruler, not the rival. Results are checked against ngspice
golden files, and a model counts as done when it agrees. sindr doesn't aim
for ngspice's breadth: BSIM-class models, vendor model libraries, very large
IC netlists and RF analyses are out of scope for 1.0.

## 1.0

> An editor can simulate any schematic built from common parts, natively and
> in the browser. Results are checked against ngspice, failures name the
> part, and the API won't break for the whole of 1.x.

1.0 ships when all of these hold:

1. **Stable API.** Typed node and component handles, `#[non_exhaustive]`
   where it matters, internals hidden, solver options, and
   `cargo-semver-checks` in CI.
2. **Common parts covered.** Every common schematic symbol (passives,
   sources, diodes, LEDs, transistors, op-amps, regulators, switches,
   connectors, crystals) maps to an element with an ngspice golden or a
   hand-derived check.
3. **The analyses an editor needs.** Operating point, DC sweep, AC and
   transient, plus ratings checks, all checked against ngspice.
4. **Never panics, always explains.** Any input gives a result or a typed
   error that names the part. The SPICE parser is fuzzed.
5. **Fast enough to run live.** Draft targets: the operating point of a
   100-part board under 1 ms, and a small transient under 100 ms, with the
   browser within 3× native. Benchmarks are published in the README.
6. **Used in a real tool.** An editor imports its own netlist through
   sindr's import API and simulates its boards end to end.

## Versions

No more prereleases after `0.1.0-alpha.8`. From `0.2.0` on, versions are
plain `0.x`: a breaking change bumps the minor version, anything else the
patch. The last `0.x` before 1.0 is the freeze candidate.

## Now

- Netlist-and-bench import API for editors: components, terminals and nets
  in, a `Circuit` and a test bench out, with errors that name the part
- Release `0.2.0`
- README and docs.rs front page with the accuracy table, benchmarks and an
  honest comparison with other options
- Coverage of every common schematic symbol, and the elements still
  missing: SPDT switch, crystal, ferrite bead

## Next

**Stability**
- Convergence errors that name the node and component that won't settle
- `#![forbid(unsafe_code)]` in every crate
- Solver options: tolerances, iteration budget, gmin ladder
- CI builds for `wasm32-unknown-unknown`, `cargo-semver-checks` and
  `cargo-deny`

**Type safety (breaking, before the freeze)**
- Typed node and component handles in place of string ids
- A smaller public module surface
- Possibly unit newtypes (ohms, farads, volts) at the public API

**Performance**
- Benchmarks against ngspice: time per call when embedded, and throughput
  across threads
- Benchmarks for every analysis and the SPICE front end

**Physics**
- Gummel-Poon BJT, so vendor `.model` cards load
- JFET junction capacitances
- More of the SPICE dialect: DC and AC specs on sources, non-ASCII node names

**Tests**
- Serde round trips for every circuit and result type
- One end-to-end test per device

## After 1.0

All additive, so fine within 1.x:

- Monte Carlo and corner analysis, pole-zero and loop gain, AC sensitivity
- Layout-aware physics from a board's stackup: trace impedance, thermal,
  power integrity
- Electrical rule checks an editor can run on a schematic
- Higher-level MOSFET models (Level 2/3, BSIM)
