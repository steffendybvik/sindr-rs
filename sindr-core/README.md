# sindr-core

[![crates.io](https://img.shields.io/crates/v/sindr-core.svg)](https://crates.io/crates/sindr-core)
[![docs.rs](https://docs.rs/sindr-core/badge.svg)](https://docs.rs/sindr-core)

The circuit and design data model behind [`sindr`](https://crates.io/crates/sindr), without the solver. It holds `Circuit`, every `CircuitElement`, `Waveform`, and the design layer: blocks and instances, net scoping, and part data (reference, value, symbol, footprint). It depends on `sindr-devices` (device parameter types), `thiserror` and optional `serde`. There is no linear algebra.

Use it to build, store or exchange circuits in a design tool. `sindr` re-exports all of it, so `sindr::Circuit` is the same type and simulates directly.

```toml
[dependencies]
sindr-core = "0.1.0-alpha.8"
```

## Hierarchy

A `Block` is a reusable definition, what `.subckt` is in SPICE. An `Instance` places one and binds its ports to the caller's nets. Inside a block a net is a port, ground, a global (`Circuit::globals`) or local to each instance. `Circuit::flatten` expands it all, naming elements and local nets with the instance path (`X1.R1`, `X1.mid`). sindr's analyses flatten for you.

```rust
use sindr_core::{Block, Circuit, CircuitElement, Instance, Part};

let r = |id: &str, a: &str, b: &str| CircuitElement::Resistor {
    id: id.into(),
    nodes: [a.into(), b.into()],
    resistance: 1e3,
};

let mut circuit = Circuit::new(
    "0",
    vec![CircuitElement::VoltageSource {
        id: "V1".into(),
        nodes: ["in".into(), "0".into()],
        voltage: 10.0,
        waveform: None,
    }],
);
circuit.blocks.insert(
    "divider".into(),
    Block {
        ports: vec!["top".into(), "out".into()],
        components: vec![r("R1", "top", "out"), r("R2", "out", "0")],
        ..Block::default()
    },
);
circuit.instances.push(Instance::new("X1", "divider", ["in", "mid"]));
circuit.parts.insert(
    "V1".into(),
    Part { symbol: Some("Simulation_SPICE:VDC".into()), ..Part::default() },
);

let flat = circuit.flatten().unwrap();
let ids: Vec<&str> = flat.components.iter().map(|c| c.id()).collect();
assert_eq!(ids, ["V1", "X1.R1", "X1.R2"]);
```

`Part` data is ignored by the simulator, except `Part::ratings`: absolute maximum `Ratings` that `sindr::check_ratings` compares a simulation against. Empty hierarchy and part fields are left out of JSON, so a flat circuit serialises exactly as before.

## License

MIT OR Apache-2.0.
