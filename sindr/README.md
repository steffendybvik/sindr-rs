# sindr

MNA (Modified Nodal Analysis) circuit solver. Build a circuit, call `solve_circuit`, get voltages, currents, and power for every component.

Supports: resistive DC, nonlinear DC (Newton-Raphson), transient (backward Euler), AC analysis, DC sweep, and temperature sweep. The solver picks the right path automatically based on what components are in the circuit.

Device physics (BJT, MOSFET, diode models) live in the companion crate [`sindr-devices`](../sindr-devices).

## Add to your project

```toml
[dependencies]
sindr = { path = "../sindr" }
```

Serde support is on by default. To disable:

```toml
sindr = { path = "../sindr", default-features = false }
```

## Quick start

```rust
use sindr::{Circuit, CircuitElement, solve_circuit};

let circuit = Circuit {
    ground_node: "gnd".into(),
    components: vec![
        CircuitElement::VoltageSource {
            id: "V1".into(),
            nodes: ["n1".into(), "gnd".into()],
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
            nodes: ["n2".into(), "gnd".into()],
            resistance: 2_000.0,
        },
    ],
};

let result = solve_circuit(&circuit)?;

println!("n2 = {:.3} V", result.node_voltages["n2"]);  // 6.667 V
println!("R1 current = {:.3} mA", result.component_results
    .iter().find(|c| c.id == "R1").unwrap().current_through * 1000.0);
```

## Building circuits

Every component has an `id` (arbitrary string) and `nodes` (node names as strings). One node must match `ground_node` — it is held at 0 V.

### Passive components

```rust
CircuitElement::Resistor   { id, nodes: [String; 2], resistance: f64 }
CircuitElement::Capacitor  { id, nodes: [String; 2], capacitance: f64 }
CircuitElement::Inductor   { id, nodes: [String; 2], inductance: f64 }
```

Capacitors and inductors trigger transient analysis automatically. In pure DC (no reactive elements, no waveforms), they are treated as open circuits.

### Sources

```rust
CircuitElement::VoltageSource { id, nodes: [String; 2], voltage: f64, waveform: Option<Waveform> }
CircuitElement::CurrentSource { id, nodes: [String; 2], current: f64, waveform: Option<Waveform> }
```

`nodes[0]` is the positive terminal for voltage sources; current flows from `nodes[0]` toward `nodes[1]` for current sources.

A `Waveform` (sine, square, sawtooth, pulse) on any source triggers transient analysis. See [`waveform.rs`](src/waveform.rs) for the `Waveform` enum.

### Switches

```rust
CircuitElement::Switch     { id, nodes: [String; 2], closed: bool }
CircuitElement::Pushbutton { id, nodes: [String; 2], closed: bool }
```

Modeled as a 0.01 Ω resistor when closed, 1 GΩ when open.

### Diodes

```rust
CircuitElement::Diode         { id, nodes: [String; 2] }            // silicon, Vf ≈ 0.7 V
CircuitElement::Led           { id, nodes: [String; 2], color: String } // "red"|"green"|"blue"|"yellow"|"white"
CircuitElement::ZenerDiode    { id, nodes: [String; 2], vz: f64 }   // breakdown voltage
CircuitElement::SchottkyDiode { id, nodes: [String; 2] }            // Vf ≈ 0.3 V
CircuitElement::Photodiode    { id, nodes: [String; 2], irradiance: f64 } // irradiance in W
```

All diodes use Newton-Raphson automatically when present. Each diode variant accepts an optional `temperature: f64` field (K, default 300.15 K) which scales IS using the SPICE temperature formula.

### Varactor diode

```rust
CircuitElement::Varactor {
    id: String,
    nodes: [String; 2],  // [anode, cathode]
    params: VaractorParams, // { cj0, phi, m }
}
```

Varactors are purely reactive — treated as open circuit in DC, and stamped as a voltage-dependent capacitor `C_j(V) = cj0 / (1 - V/phi)^m` each transient timestep. Always triggers transient analysis.

### Transistors

```rust
// BJT — nodes: [base, collector, emitter]
CircuitElement::Bjt {
    id,
    nodes: [String; 3],
    kind: BjtKind,                          // BjtKind::Npn or BjtKind::Pnp
    bf: f64,                                // forward beta, default 100
    temperature: f64,                       // junction temp (K), default 300.15 K
    parasitic_caps: Option<BjtParasiticCaps>, // { cbe, cbc } in Farads; None = disabled
}

// MOSFET — nodes: [gate, drain, source]
CircuitElement::Mosfet {
    id,
    nodes: [String; 3],
    kind: MosfetKind,                          // MosfetKind::Nmos or MosfetKind::Pmos
    params: MosfetParams,                      // threshold voltage, mobility, etc.
    parasitic_caps: Option<MosfetParasiticCaps>, // { cgs, cgd } in Farads; None = disabled
}

// IGBT — nodes: [gate, collector, emitter]
CircuitElement::Igbt {
    id,
    nodes: [String; 3],
    params: IgbtParams, // { vth, k, vce_sat }
}
```

`BjtKind`, `MosfetKind`, `BjtParasiticCaps`, `MosfetParasiticCaps`, `IgbtParams` are re-exported from `sindr` directly.

When `parasitic_caps` is set, Cbe/Cbc (BJT) or Cgs/Cgd (MOSFET) are stamped as Backward Euler capacitor companions each transient timestep, with per-junction voltage state tracked internally. This automatically triggers transient analysis.

**NPN common-emitter example:**

```rust
use sindr::{Circuit, CircuitElement, BjtKind, solve_circuit};

let circuit = Circuit {
    ground_node: "0".into(),
    components: vec![
        CircuitElement::VoltageSource {
            id: "Vcc".into(), nodes: ["vcc".into(), "0".into()], voltage: 10.0, waveform: None,
        },
        CircuitElement::Resistor {
            id: "Rc".into(), nodes: ["vcc".into(), "collector".into()], resistance: 1_000.0,
        },
        CircuitElement::Resistor {
            id: "Rb".into(), nodes: ["vcc".into(), "base".into()], resistance: 470_000.0,
        },
        CircuitElement::Bjt {
            id: "Q1".into(),
            nodes: ["base".into(), "collector".into(), "0".into()],
            kind: BjtKind::Npn,
            bf: 100.0,
            temperature: 300.15,
            parasitic_caps: None,
        },
    ],
};

let result = solve_circuit(&circuit)?;

let q1 = result.bjt_results.iter().find(|b| b.id == "Q1").unwrap();
println!("Vbe = {:.3} V, Vce = {:.3} V, Ic = {:.3} mA, region = {}",
    q1.vbe, q1.vce, q1.ic * 1000.0, q1.region);
```

### Transformer (coupled inductors)

```rust
CircuitElement::Transformer {
    id: String,
    nodes: [String; 4],  // [p1, q1, p2, q2] — primary (p1,q1), secondary (p2,q2)
    l1: f64,             // primary inductance (H)
    l2: f64,             // secondary inductance (H)
    k: f64,              // coupling coefficient [0, 0.999]; default 0.999 (near-ideal)
}
```

Mutual inductance M = k·√(L1·L2). In DC analysis both windings are stamped as short circuits. In transient, two branch current unknowns are added to the MNA system. k=1 is mathematically singular — use k≤0.999.

### Controlled sources

```rust
// Voltage-controlled voltage source: V_out = gain × V_ctrl
CircuitElement::Vcvs { id, nodes: [out+, out-], control_nodes: [ctrl+, ctrl-], gain: f64 }

// Voltage-controlled current source: I_out = gm × V_ctrl
CircuitElement::Vccs { id, nodes: [out_from, out_to], control_nodes: [ctrl+, ctrl-], gm: f64 }

// Current-controlled voltage source: V_out = rm × I_ctrl
CircuitElement::Ccvs { id, nodes: [out+, out-], control_source: String, rm: f64 }

// Current-controlled current source: I_out = alpha × I_ctrl
CircuitElement::Cccs { id, nodes: [out_from, out_to], control_source: String, alpha: f64 }
```

`control_source` is the `id` of the voltage source whose branch current is sensed.

### Op-amp / Comparator

```rust
// nodes: [in_plus, in_minus, out]
CircuitElement::OpAmp      { id, nodes: [String; 3], v_pos: f64, v_neg: f64 } // supply rails, default ±15 V
CircuitElement::Comparator { id, nodes: [String; 3], v_pos: f64, v_neg: f64 } // default 5 V / 0 V
```

Both are modeled as a high-gain VCVS (gain = 1×10⁵) that saturates at the supply rails.

### Sensors / misc

```rust
CircuitElement::Photoresistor { id, nodes: [String; 2], light_level: f64 } // 0.0 (dark, ~1 MΩ) – 1.0 (bright, ~1 kΩ)
CircuitElement::Thermistor    { id, nodes: [String; 2], temperature: f64 } // Kelvin, default 298.15 K (25 °C)
CircuitElement::Potentiometer { id, nodes: [top, wiper, bottom], resistance: f64, position: f64 } // position 0.0–1.0
CircuitElement::Relay {
    id,
    nodes: [String; 4],     // [coil+, coil-, contact1, contact2]
    coil_resistance: f64,
    pickup_voltage: f64,
    inductance: f64,        // coil inductance (H); 0.0 = purely resistive (default)
}
```

When `inductance > 0`, the relay coil is modeled as an RL circuit in transient analysis (L stamped as a Backward Euler inductor companion). This triggers transient automatically.

## Reading results

`solve_circuit` returns `Result<SimulationResult, SimError>`.

```rust
pub struct SimulationResult {
    pub node_voltages:    HashMap<String, f64>,  // every node including ground (0.0)
    pub branch_currents:  HashMap<String, f64>,  // voltage source branch currents by id
    pub component_results: Vec<ComponentResult>, // V, I, P for every component
    pub bjt_results:      Vec<BjtResult>,        // present only if circuit has BJTs
    pub mosfet_results:   Vec<MosfetResult>,     // present only if circuit has MOSFETs
    pub op_amp_results:   Vec<OpAmpResult>,      // present only if circuit has op-amps
    pub relay_results:    Vec<RelayResult>,      // present only if circuit has relays
    pub transient:        Option<TransientData>, // present only for reactive/waveform circuits
}

pub struct ComponentResult {
    pub id:              String,
    pub voltage_across:  f64,
    pub current_through: f64,
    pub power:           f64,
}

pub struct BjtResult {
    pub id: String, pub vbe: f64, pub vce: f64,
    pub ib: f64, pub ic: f64, pub ie: f64,
    pub power: f64, pub region: String, // "active" | "saturation" | "cutoff" | "reverse_active"
}

pub struct MosfetResult {
    pub id: String, pub vgs: f64, pub vds: f64,
    pub id_current: f64, pub power: f64, pub region: String, // "off" | "linear" | "saturation"
}
```

For transient circuits, `result.transient` contains a time series:

```rust
pub struct TransientData {
    pub timesteps: Vec<TimestepSnapshot>,
    pub time_step: f64,
    pub duration:  f64,
}

pub struct TimestepSnapshot {
    pub time:              f64,
    pub node_voltages:     HashMap<String, f64>,
    pub component_results: Vec<ComponentResult>,
}
```

## DC sweep

Sweep a voltage source across a range and collect operating points at each step.

```rust
use sindr::{Circuit, CircuitElement, dc_sweep};

let sweep = dc_sweep(&circuit, "V1", 0.0, 5.0, 51)?;

let v_curve = sweep.node_voltage_curve("n2");
let i_curve = sweep.component_current_curve("D1");
```

## Temperature sweep

Solve a circuit at a series of junction temperatures and collect operating points. Useful for BJT Ic vs. temperature curves and thermal characterisation.

```rust
use sindr::{Circuit, temperature_sweep, TempSweepResult};

// All junction devices (BJT, diode, LED, Zener, Schottky, Photodiode) have their
// temperature field overridden at each step.
let result = temperature_sweep(&circuit, 250.0, 350.0, 11)?; // 250 K → 350 K, 11 points

for point in &result.points {
    println!("T = {:.1} K, Ic = {:.3} mA", point.temperature,
        point.bjt_results.iter().find(|b| b.id == "Q1").map_or(0.0, |b| b.ic) * 1000.0);
}
```

```rust
pub struct TempSweepResult {
    pub points: Vec<TempSweepPoint>,
}

pub struct TempSweepPoint {
    pub temperature:      f64,
    pub node_voltages:    HashMap<String, f64>,
    pub component_results: Vec<ComponentResult>,
    pub bjt_results:      Vec<BjtResult>,
}
```

`num_steps` must be ≥ 2.

## Error handling

```rust
pub enum SimError {
    NoGround,
    DisconnectedNodes(String),
    FloatingNode(String),
    SingularMatrix,
    InvalidSolution,
    InvalidComponent(String),
    InvalidResistance(String),
    ConvergenceFailed { iterations: usize, max_step_volts: f64 },
}
```

`ConvergenceFailed` means Newton-Raphson did not converge — usually caused by a nonlinear circuit with no DC path to ground, or component values far outside typical operating ranges. The `max_step_volts` field gives the largest per-node Newton step (V) on the final iteration, useful for distinguishing slow convergence from genuine divergence. (Note: this is a step magnitude `max_i |V_new[i] − V_prev[i]|`, not a KCL residual `|F(x)|`.)

## Solver routing

`solve_circuit` picks a path automatically — you never select it manually:

| Circuit contains | Solver used |
|---|---|
| Only linear elements | Direct MNA (LU factorisation) |
| Nonlinear elements (diodes, BJTs, MOSFETs, IGBTs…) | Newton-Raphson iteration |
| Reactive elements (C, L, Varactor, Transformer) or waveform sources | Backward Euler transient |
| Both reactive and nonlinear | Transient with per-step Newton-Raphson |

The transient solver uses adaptive timestepping: dt halves on Newton-Raphson convergence failure, and doubles after 5 consecutive successful timesteps (clamped to 10× the initial dt).

## Serde

With the default `serde` feature, `Circuit` and `SimulationResult` (and all nested types) implement `Serialize`/`Deserialize`. Component types use snake_case tags:

```json
{
  "ground_node": "0",
  "components": [
    { "type": "voltage_source", "id": "V1", "nodes": ["n1", "0"], "voltage": 10.0 },
    { "type": "resistor",       "id": "R1", "nodes": ["n1", "0"], "resistance": 1000.0 },
    { "type": "bjt",  "id": "Q1", "nodes": ["b", "c", "e"], "kind": "npn", "bf": 100 },
    { "type": "transformer", "id": "T1", "nodes": ["p1","q1","p2","q2"], "l1": 1e-3, "l2": 4e-3 },
    { "type": "varactor", "id": "D1", "nodes": ["a", "k"], "params": { "cj0": 10e-12, "phi": 0.7, "m": 0.5 } }
  ]
}
```

Fields with defaults (`bf`, `temperature`, `k`, `coil_resistance`, `position`, `v_pos`, `v_neg`, `parasitic_caps`) can be omitted from JSON and will take their defaults.
