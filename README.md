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

## License

MIT — see [LICENSE-MIT](./LICENSE-MIT).
