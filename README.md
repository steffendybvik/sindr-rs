# sindr

Pure-Rust circuit simulator. SPICE-style MNA solver with first-class semiconductor models.

## Crates

| Crate | Purpose |
|-------|---------|
| [`sindr`](./sindr) | MNA solver: DC, transient, AC, DC sweep, temperature sweep |
| [`sindr-devices`](./sindr-devices) | Device physics: diode, BJT, MOSFET, IGBT, JFET, varactor companion models |

`sindr` depends on `sindr-devices`. The split lets you use the device-physics models with your own solver if you don't want the `nalgebra` linear-algebra stack.

## Quick example

A 10 V source across a 1 kΩ / 2 kΩ divider — `n2` should sit at 6.667 V:

```rust
use sindr::{Circuit, CircuitElement, solve_circuit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2_000.0,
            },
        ],
    };

    let result = solve_circuit(&circuit)?;
    println!("V(n2) = {:.4} V", result.node_voltages["n2"]);
    Ok(())
}
```

More circuits (RC transient, BJT amplifier, AC Bode plot, DC sweep) live in [`sindr/examples/`](./sindr/examples).

## Status

Pre-release alpha (`0.1.0-alpha.3`). API is liable to change. Not yet on crates.io.

## Scope & limitations

sindr is an analog circuit simulator. Digital logic, HDL ingestion, MCU simulation, schematic capture, SPICE netlist import, vendor model cards, noise/Monte-Carlo/pole-zero analyses, BSIM-class device models, and self-heating co-simulation are **not** supported today. See [`LIMITATIONS.md`](./LIMITATIONS.md) for the full breakdown of what's out-of-scope versus not-yet-implemented.

AI coding agents (Claude Code, Codex, Cursor, etc.) working on this repo should read [`AGENTS.md`](./AGENTS.md) for layout, conventions, and pitfalls.

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
