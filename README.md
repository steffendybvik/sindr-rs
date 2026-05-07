# sindr

Pure-Rust circuit simulator. SPICE-style MNA solver with first-class semiconductor models.

> *Sindr* — Old Norse for "sparks", and the dwarf smith who forged Mjölnir, Gungnir, and Draupnir. The maker of the gods' tools.

## Crates

| Crate | Purpose |
|-------|---------|
| [`sindr`](./sindr) | MNA solver: DC, transient, AC, DC sweep, temperature sweep |
| [`sindr-devices`](./sindr-devices) | Device physics: diode, BJT, MOSFET, IGBT, JFET, varactor companion models |

`sindr` depends on `sindr-devices`. The split lets you use the device-physics models with your own solver if you don't want the `nalgebra` linear-algebra stack.

## Status

Pre-release alpha (`0.1.0-alpha.1`). API is liable to change. Not yet on crates.io.

## Scope & limitations

sindr is an analog circuit simulator. Digital logic, HDL ingestion, MCU simulation, schematic capture, SPICE netlist import, vendor model cards, noise/Monte-Carlo/pole-zero analyses, BSIM-class device models, and self-heating co-simulation are **not** supported today. See [`LIMITATIONS.md`](./LIMITATIONS.md) for the full breakdown of what's out-of-scope versus not-yet-implemented.

## License

Dual-licensed under either of:

- [MIT License](./LICENSE-MIT) ([https://opensource.org/licenses/MIT](https://opensource.org/licenses/MIT))
- [Apache License, Version 2.0](./LICENSE-APACHE) ([http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual-licensed as above, without any additional terms or
conditions.
