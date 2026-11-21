# sindr

MNA (Modified Nodal Analysis) circuit solver. Build a circuit, call `solve_circuit`, get voltages, currents, and power for every component.

Supports: resistive DC, nonlinear DC (Newton-Raphson), transient (Gear-2, trapezoidal or backward Euler), AC analysis, noise (`.noise`), transfer function (`.tf`), DC sensitivity (`.sens`), DC sweep, and temperature sweep. The solver picks the right path automatically based on what components are in the circuit.

Device physics (BJT, MOSFET, diode models) live in the companion crate [`sindr-devices`](../sindr-devices).

## Add to your project

```toml
[dependencies]
sindr = "0.1"
```

Serde support is on by default. To disable:

```toml
sindr = { version = "0.1", default-features = false }
```

To parse SPICE netlists, enable the optional `spice` feature (off by default):

```toml
sindr = { version = "0.1", features = ["spice"] }
```

```rust,ignore
let netlist = sindr::spice::parse_file("amp.cir")?;
let result = sindr::solve_circuit(&netlist.circuit)?;
```

It reads a practical subset of Berkeley SPICE3f5 decks (R, L, C, V, I, D, Q, B, X, linear E/G/F/H (and `VALUE`), K coupling, `.subckt`, `.global`, `.model`, `.tran`, `.dc`, `.ac`, `.tf`, `.sens`, `.noise`, `.meas`, `.op`, `.ic`, `.nodeset`, `.options`, `.param`, `.include`, `.lib`), keeping `.subckt` hierarchy as blocks and instances, which are flattened with `.`-separated naming when simulated. See the `sindr::spice` module docs for the supported subset and strictness options, or run the worked example:

```bash
cargo run --example parse_netlist --features spice
```

`sindr::spice::to_netlist` goes the other way: it writes a `Circuit` as a SPICE deck (R, L, C, V, I with waveforms, D and Q with `.model` cards, E/G/F/H, transformers as L–L–K, and exact R/D equivalents for switches, fuses, potentiometers, LEDs and the like) that ngspice runs, and that `parse_str` reads back. Elements with no SPICE equivalent are an `ExportError`.

## Quick start

```rust,no_run
# use sindr::*;
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
    ..Default::default()
};

let result = solve_circuit(&circuit)?;

println!("n2 = {:.3} V", result.node_voltages["n2"]);  // 6.667 V
println!("R1 current = {:.3} mA", result.component_results
    .iter().find(|c| c.id == "R1").unwrap().current_through * 1000.0);
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Building circuits

Every component has an `id` (arbitrary string) and `nodes` (node names as strings). One node must match `ground_node` — it is held at 0 V.

### Passive components

```rust,ignore
CircuitElement::Resistor   { id, nodes: [String; 2], resistance: f64 }
CircuitElement::Capacitor  { id, nodes: [String; 2], capacitance: f64 }
CircuitElement::Inductor   { id, nodes: [String; 2], inductance: f64, saturation: Option<InductorSaturation> }
```

Capacitors and inductors trigger transient analysis automatically. In pure DC (no reactive elements, no waveforms), they are treated as open circuits.

An inductor with `saturation: Some(InductorSaturation { i_sat, l_sat })` has a core that saturates: its flux is `λ(i) = l_sat·i + (inductance − l_sat)·i_sat·tanh(i/i_sat)`, so its inductance falls from `inductance` at zero current towards `l_sat` past `i_sat` (to 42 % of the way at `i_sat`, 7 % at `2·i_sat`). A datasheet's saturation current, where L has dropped 20–30 %, is about `0.6·i_sat`. The transient integrates the flux, and AC and noise use the incremental inductance `dλ/di` at the DC current. There's no hysteresis or core loss; add a resistor for loss.

```rust,no_run
# use sindr::*;
use sindr_devices::inductor::InductorSaturation;

// 10 µH that saturates around 3 A, keeping 1 µH when saturated.
let l1 = CircuitElement::Inductor {
    id: "L1".into(),
    nodes: ["sw".into(), "out".into()],
    inductance: 10e-6,
    saturation: Some(InductorSaturation { i_sat: 3.0 / 0.6, l_sat: 1e-6 }),
};
```

### Sources

```rust,ignore
CircuitElement::VoltageSource { id, nodes: [String; 2], voltage: f64, waveform: Option<Waveform> }
CircuitElement::CurrentSource { id, nodes: [String; 2], current: f64, waveform: Option<Waveform> }
```

`nodes[0]` is the positive terminal for voltage sources. A current source's current flows through it from `nodes[0]` to `nodes[1]`: it is drawn out of `nodes[0]` and pushed into `nodes[1]`, as SPICE's `I` does.

A `Waveform` (sine, pulse, square, triangle, PWM, PWL) on any source triggers transient analysis. See [`waveform.rs`](src/waveform.rs) for the `Waveform` enum.

### Battery

```rust,ignore
CircuitElement::Battery { id, nodes: [String; 2], params: BatteryParams, state_of_charge: f64 }
```

`nodes` are `[positive, negative]`. The cell is an equivalent circuit: open-circuit voltage by state of charge (`params.ocv`, interpolated), behind a series resistance `r_series` and an optional RC pair (`params.rc`) for the slow sag after a load step and the recovery after it. Its terminal voltage is `OCV(SoC) − R0·i − v1`.

- **DC:** the open-circuit voltage at `state_of_charge` behind `R0 + R1`.
- **Transient:** the RC pair charges, and the state of charge falls by the charge drawn, `dSoC/dt = −i/(3600·capacity)`. Each step's `state_of_charge` (and the result's, at the end) holds it by battery id. Below 0 the cell is over-discharged and holds its 0 % voltage. A battery with an RC pair makes `solve_circuit` run a transient.
- **AC:** the impedance `R0 + R1‖C1`.

`current_through` flows `nodes[0] → nodes[1]` inside the cell, as for a voltage source, so it is negative while discharging. Presets: `BatteryParams::lithium_ion(capacity_ah, r_series)` (a typical Li-ion/LiPo rest curve, 3.27–4.20 V), `BatteryParams::cr2032()` (a 225 mAh coin cell with its pulse-recovery pair) and `BatteryParams::constant(volts, capacity_ah, r_series)`.

```rust,no_run
# use sindr::*;
use sindr::battery::BatteryParams;

// A 500 mAh LiPo at 80 %, 0.15 Ω.
let cell = CircuitElement::Battery {
    id: "B1".into(),
    nodes: ["vbat".into(), "0".into()],
    params: BatteryParams::lithium_ion(0.5, 0.15),
    state_of_charge: 0.8,
};
```

### Switches

```rust,ignore
CircuitElement::Switch     { id, nodes: [String; 2], closed: bool }
CircuitElement::Pushbutton { id, nodes: [String; 2], closed: bool }
```

Modeled as a 0.01 Ω resistor when closed, 1 GΩ when open.

```rust,ignore
CircuitElement::SwitchSpdt { id, nodes: [String; 3], position: SpdtPosition }  // [common, a, b]
```

A single-pole double-throw switch: common to `a` (`SpdtPosition::A`, the default) or to `b`, each path a `Switch`. Its result is the path in use: common minus the selected throw, and the current from common into it.

### Crystal and ferrite bead

```rust,ignore
CircuitElement::Crystal     { id, nodes: [String; 2], params: CrystalParams }
CircuitElement::FerriteBead { id, nodes: [String; 2], params: FerriteBeadParams }
```

- **Crystal:** the Butterworth–Van Dyke model, `Rm`–`Lm`–`Cm` in series beside `C0`, with `Lm` set by the series-resonant `frequency`. `CrystalParams::from_datasheet(frequency, esr, c0)` takes `Cm = C0/250`, typical of AT-cut crystals; set `cm` when you know it. Open at DC. Its result's `power` is what `Rm` dissipates, the drive level, which `Ratings::power` limits. The load capacitance a datasheet names is the two capacitors on your board, not part of the crystal.
- **Ferrite bead:** `r_dc` in series with `l ∥ r_ac` (∥ `c`, when set). `FerriteBeadParams::from_datasheet(z_100mhz, r_dc)` fits `l` and `r_ac` to the impedance at 100 MHz; fit all four to the curve when the peak matters. `Ratings::current` is its rated current.

Both, and the SPDT switch, are solved as those networks: results and ratings come back under the element's own id, and `to_netlist` writes the network.

### Diodes

```rust,ignore
CircuitElement::Diode         { id, nodes: [String; 2], model: Option<DiodeModel> } // silicon (Vf ≈ 0.7 V) unless a SPICE-style model is set
CircuitElement::Led           { id, nodes: [String; 2], color: String } // "red"|"green"|"blue"|"yellow"|"white"
CircuitElement::ZenerDiode    { id, nodes: [String; 2], vz: f64 }   // breakdown voltage at 5 mA
CircuitElement::SchottkyDiode { id, nodes: [String; 2] }            // Vf ≈ 0.3 V
CircuitElement::Photodiode    { id, nodes: [String; 2], irradiance: f64 } // irradiance in W
```

All diodes use Newton-Raphson automatically when present. `Diode`'s optional `model` is what a SPICE `.model D` card fills in: `DiodeModel { is, n, rs }` (defaults `1e-14`, `1`, `0`), the junction charge `cjo`, `vj`, `m`, `fc`, `tt` (defaults `0`, `1`, `0.5`, `0.5`, `0`) and breakdown `bv`, `ibv` (defaults none, 1 mA). With `cjo` or `tt` set the diode stores charge, so it shows reverse recovery in transient and a capacitance in AC, and the circuit takes the transient path; its `current_through` then includes the charging current, while `power` stays the dissipation. `bv` adds SPICE's exponential reverse breakdown, fitted to pass `ibv` near `−bv` as ngspice does. A `ZenerDiode` is exactly such a diode: `bv = vz`, `ibv` = 5 mA (a small zener's test current) and `rs` = 1 Ω, so it matches ngspice with the card `D(IS=1e-14 N=1 RS=1 BV=vz IBV=5m)`. Each diode variant accepts an optional `temperature: f64` field (K, default 300.15 K): IS scales with the SPICE temperature formula and the thermal voltage is kT/q at that temperature.

### Varactor diode

```rust,ignore
CircuitElement::Varactor {
    id: String,
    nodes: [String; 2],  // [anode, cathode]
    params: VaractorParams, // { cj0, phi, m }
}
```

Varactors are purely reactive — treated as open circuit in DC, as their junction capacitance at the bias point in AC, and stamped as a voltage-dependent capacitor `C_j(V) = cj0 / (1 - V/phi)^m` each transient timestep. Always triggers transient analysis.

### Transistors

```rust,ignore
// BJT — nodes: [base, collector, emitter]
CircuitElement::Bjt {
    id,
    nodes: [String; 3],
    kind: BjtKind,                          // BjtKind::Npn or BjtKind::Pnp
    bf: f64,                                // forward beta, default 100
    temperature: f64,                       // junction temp (K), default 300.15 K
    parasitic_caps: Option<BjtParasiticCaps>, // { cbe, cbc } in Farads; None = disabled
    model: Option<BjtModel>,                // IS, BR, NF, NR, VAF; None = built-in defaults
}

// MOSFET — nodes: [gate, drain, source]; body tied to source
CircuitElement::Mosfet {
    id,
    nodes: [String; 3],
    kind: MosfetKind,                          // MosfetKind::Nmos or MosfetKind::Pmos
    params: MosfetParams,                      // KP, VTO, LAMBDA, GAMMA, PHI; VTO SPICE-signed (negative for PMOS)
    parasitic_caps: Option<MosfetParasiticCaps>, // { cgs, cgd } in Farads; None = disabled
    body_diode: Option<DiodeModel>,            // power MOSFET's body diode; None = none
}

// IGBT — nodes: [gate, collector, emitter]
CircuitElement::Igbt {
    id,
    nodes: [String; 3],
    params: IgbtParams, // { vth, k, vce_sat }
}

// JFET — nodes: [gate, drain, source]
CircuitElement::Jfet {
    id,
    nodes: [String; 3],
    kind: JfetKind, // JfetKind::NChannel or JfetKind::PChannel
    idss: f64,      // drain saturation current at Vgs = 0 (A)
    vp: f64,        // pinch-off voltage (V), negative for N-channel
}
```

A MOSFET's `body_diode` is a SPICE diode card (`DiodeModel`) from source to drain for NMOS, drain to source for PMOS. It conducts when a bridge or inductive load pulls the drain past the source with the channel off. Every analysis solves it as a `Diode` named `<id>.body`, which is where its current shows in the results; `bv` on it sets the part's avalanche voltage.

`BjtKind`, `MosfetKind`, `JfetKind`, `IgbtParams` and `VaractorParams` are re-exported from `sindr` directly. `BjtParasiticCaps` and `MosfetParasiticCaps` live in `sindr::circuit`, and `MosfetParams` in `sindr_devices::mosfet`.

When `parasitic_caps` is set, Cbe/Cbc (BJT) or Cgs/Cgd (MOSFET) are stamped as capacitor companions each transient timestep, with per-junction voltage state tracked internally, and as jωC in AC. This automatically triggers transient analysis.

**NPN common-emitter example:**

```rust,no_run
# use sindr::*;
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
            model: None,
        },
    ],
    ..Default::default()
};

let result = solve_circuit(&circuit)?;

let q1 = result.bjt_results.iter().find(|b| b.id == "Q1").unwrap();
println!("Vbe = {:.3} V, Vce = {:.3} V, Ic = {:.3} mA, region = {}",
    q1.vbe, q1.vce, q1.ic * 1000.0, q1.region);
# Ok::<(), Box<dyn std::error::Error>>(())
```

### Transformer (coupled inductors)

```rust,ignore
CircuitElement::Transformer {
    id: String,
    nodes: [String; 4],  // [p1, q1, p2, q2] — primary (p1,q1), secondary (p2,q2)
    l1: f64,             // primary inductance (H)
    l2: f64,             // secondary inductance (H)
    k: f64,              // coupling coefficient [0, 0.999]; default 0.999 (near-ideal)
}
```

Mutual inductance M = k·√(L1·L2). In DC analysis both windings are short circuits. In AC and transient, two branch current unknowns (one per winding) are added to the MNA system, so the windings are coupled through M. k=1 is mathematically singular — use k≤0.999.

### Controlled sources

```rust,ignore
// Voltage-controlled voltage source: V_out = gain × V_ctrl
CircuitElement::Vcvs { id, nodes: [out+, out-], control_nodes: [ctrl+, ctrl-], gain: f64 }

// Voltage-controlled current source: I_out = gm × V_ctrl
CircuitElement::Vccs { id, nodes: [out_from, out_to], control_nodes: [ctrl+, ctrl-], gm: f64 }

// Current-controlled voltage source: V_out = rm × I_ctrl
CircuitElement::Ccvs { id, nodes: [out+, out-], control_source: String, rm: f64 }

// Current-controlled current source: I_out = alpha × I_ctrl
CircuitElement::Cccs { id, nodes: [out_from, out_to], control_source: String, alpha: f64 }

// Behavioural source (SPICE's B): a voltage or current set by an expression
CircuitElement::Behavioural { id, nodes: [n+, n-], output: BehaviouralOutput, expr: Expression }
```

A behavioural source's `expr` is `Expression::parse("v(a)*v(b)")?` and the like: `v(n)`, `v(n,m)`, `i(source)` (the branch current of a voltage source, VCVS, CCVS, op-amp, regulator or behavioural voltage source), `time`, arithmetic, comparisons, `c ? a : b` and ngspice's functions (`exp`, `tanh`, `limit`, `min`, `u`, …). `BehaviouralOutput::Voltage` holds `V(n+) − V(n-)` at the expression; `Current` drives it from `n+` to `n-`. Newton gets the exact Jacobian by automatic differentiation; AC linearises at the bias point. In JSON the expression is its text: `{"type": "behavioural", "output": "voltage", "expr": "v(a)*v(b)", …}`.

`control_source` is the `id` of the voltage source whose branch current is sensed, positive from its `+` terminal through it to `-`, as in SPICE.

The current sources follow SPICE's `G` and `F` and sindr's own `CurrentSource`: a positive `I_out` flows through the source from `out_from` to `out_to`, so it is drawn out of `out_from` and pushed into `out_to`.

### Op-amp / Comparator

```rust,ignore
// nodes: [in_plus, in_minus, out]
CircuitElement::OpAmp      { id, nodes: [String; 3], v_pos: f64, v_neg: f64, model: Option<OpAmpModel> } // rails, default ±15 V
CircuitElement::Comparator { id, nodes: [String; 3], v_pos: f64, v_neg: f64 } // output high / low, default ±15 V
```

With `model: None` the op-amp is a high-gain VCVS (gain 1×10⁵) with no frequency response, and the rails aren't enforced. A comparator has the same gain and no frequency response, but its output is clamped to `v_neg`/`v_pos`, so overdrive beyond `v_pos / 1e5` (50 µV for a 5 V rail) pins it to a rail in DC, transient and AC (where its gain at a rail is zero). It has no hysteresis or open-collector output; `v_pos` must be above `v_neg`. An `OpAmpModel` turns the op-amp into a macromodel. It has open-loop `gain` (default 1e5), a dominant pole from `gbw` (default 1 MHz), a `slew_rate` limit (default 0.5 V/µs), `r_out`, `offset_voltage` and `bias_current`, and its output clamps to `v_pos`/`v_neg` with anti-windup. A non-inverting stage of gain G then has its −3 dB point near `gbw / G`, and a large step rises at the slew rate:

```rust
use sindr::{CircuitElement, OpAmpModel};

let u1 = CircuitElement::OpAmp {
    id: "U1".into(),
    nodes: ["in".into(), "fb".into(), "out".into()],
    v_pos: 12.0,
    v_neg: -12.0,
    model: Some(OpAmpModel { gbw: Some(3e6), slew_rate: Some(13e6), r_out: 50.0, ..OpAmpModel::default() }),
};
```

The pole and slew limit act in transient and AC runs. An op-amp doesn't count as reactive, so `solve_circuit` still runs DC for a circuit whose only dynamics are op-amps; call `solve_transient` to see them.

### Sensors / misc

```rust,ignore
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
CircuitElement::Fuse             { id, nodes: [String; 2], rating: f64, blown: bool } // 1 mΩ intact, 1 GΩ blown; rating is display-only
CircuitElement::VoltageRegulator { id, nodes: [input, output, gnd], voltage: f64, model: Option<RegulatorParams> }
```

When `inductance > 0`, the relay coil is modeled as an RL circuit in transient analysis (L stamped as an inductor companion). This triggers transient automatically.

`VoltageRegulator` is a linear regulator. With headroom it holds `voltage` above its ground pin (a negative `voltage` is a negative regulator such as a 7905: the same characteristic mirrored, with the input below ground and the output current flowing in at the output pin); without it the output follows the input down by the dropout; past the current limit it holds the current, short circuits included. It draws its output current plus its quiescent current from the input. `model: None` is ideal (no dropout, no quiescent current, no limit) but still never puts out more than it gets in. `RegulatorParams` sets `dropout` (V, plus `dropout_resistance` Ω for one that grows with load), `quiescent_current`, `current_limit`, `line_regulation` (V/V) and `output_resistance` (Ω, load regulation). `RegulatorParams::lm7805()` and `RegulatorParams::ams1117()` are datasheet presets. Its result is the pass element: `voltage_across` is `Vin − Vout`, `current_through` the output current, and `power` the dissipation `(Vin − Vout)·Iout + Vin·Iq`, which the ratings checks and self-heating use. In AC, input ripple reaches the output through line regulation (all of it in dropout). Foldback and thermal shutdown are not modelled.

## Hierarchy and part data

A `Block` is a reusable definition, what `.subckt` is in SPICE. An `Instance` places one and binds its ports to the caller's nets. Inside a block a net is a port, ground, a global (`Circuit::globals`) or local to each instance. `Circuit::flatten` expands it all, naming elements and local nets with the instance path (`X1.R1`, `X1.mid`). Every analysis flattens for you.

```rust,ignore
use sindr::{Block, Circuit, CircuitElement, Instance, Part};

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

`Part` data is ignored by the simulator, except `Part::ratings`: absolute maximum `Ratings` that `check_ratings` compares a simulation against. Empty hierarchy and part fields are left out of JSON, so a flat circuit serialises exactly as before.

## Reading results

`solve_circuit` returns `Result<SimulationResult, SimError>`.

```rust,ignore
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
    pub power: f64, pub region: String, // "active" | "saturation" | "cutoff"
}

pub struct MosfetResult {
    pub id: String, pub vgs: f64, pub vds: f64,
    pub id_current: f64, pub power: f64, pub region: String, // "cutoff" | "triode" | "saturation"
}
```

For transient circuits, `result.transient` contains a time series. `solve_circuit` picks the duration (5 τ) and step (τ/50) from the circuit's time constants; to set them yourself, as SPICE's `.tran tstep tstop` does, use `solve_transient`:

```rust,ignore
use sindr::{solve_transient, TransientConfig};

// 10 ms in 1 µs steps; optionally cap the step with .with_tmax(…)
let result = solve_transient(&circuit, &TransientConfig::new(10e-3, 1e-6))?;
```

Each snapshot is the end of one step, with sources evaluated at that time; the last is at `tstop`. The default integrator is Gear-2 (second order, no ringing); `.with_method(IntegrationMethod::Trapezoidal)` keeps an LC tank's amplitude exactly, and `IntegrationMethod::BackwardEuler` matches sindr before 0.1.0-alpha.7. Every method takes its first step as backward Euler.

A run starts switched on at t = 0 by default: capacitors uncharged and inductors at 0 A, unless `.with_initial_voltages(…)` or `.with_initial_current(…)` say otherwise (SPICE's `.ic` with `UIC`), so power-on inrush shows. `.with_start(TransientStart::OperatingPoint)` starts from the DC operating point instead, as SPICE's `.tran` without `UIC` does: each source at its t = 0 value, capacitors charged and inductors and transformer windings carrying its currents, with any initial voltages and currents held while it's solved. A SPICE deck's `ParsedNetlist::transient_config()` sets the run up the way ngspice does: the start, `.ic`, `tmax`, and `tstart` (`.with_tstart(t)`: integrate from 0, keep the steps from `t` on).

Steps are at most `tstep` long and every multiple of `tstep` gets a snapshot. In between, the stepper shortens steps where the local truncation error would exceed SPICE's default tolerances, and lands exactly on each corner of a source waveform (pulse edges, PWL points, square-wave flips), so a 1 ns edge in a 1 ms run is resolved without 1 ns steps everywhere. `result.transient.stats` counts accepted and rejected steps; `.with_fixed_step()` turns the error control off.


```rust,ignore
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

### CSV export

`TransientData`, `AcResult`, `DcSweepResult` and `TempSweepResult` each have `to_csv()`. It writes one row per point: the scale first (`time`, `frequency`, the swept source's id, or `temperature` in K), then `v(<node>)` for each node in name order, then `i(<id>)` (each component's `current_through`) in circuit order. AC writes each node as `re(v(<node>))` and `im(v(<node>))`. Numbers read back as the same `f64`, and names with commas or quotes are quoted.

```rust,no_run
# use sindr::*;
# let result = solve_circuit(&Circuit::default())?;
std::fs::write("rc.csv", result.transient.unwrap().to_csv())?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

## AC analysis

Small-signal frequency response around the DC operating point. Nonlinear devices are linearised at the operating point, then each frequency is a complex MNA solve.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};

// 51 points from 1 Hz to 100 kHz, driving V1 (the voltage source that
// carries the AC stimulus) with 1 V at 0°.
let config = AcConfig::new("V1", 1.0, 100_000.0, 51)
    .with_spacing(FrequencySpacing::Logarithmic); // the default; or Linear
let result = solve_ac(&circuit, &config)?;

let gain = result.gain_curve("n_out");   // (Hz, dB)
let phase = result.phase_curve("n_out"); // (Hz, degrees)
# Ok::<(), Box<dyn std::error::Error>>(())
```

`num_points` counts both endpoints; with 1, only `f_start` is solved. Invalid configs (0 points, non-finite values, `f_start > f_stop`, `f_start <= 0` with logarithmic spacing) return `SimError::InvalidComponent`.

## Transfer function

SPICE's `.tf`: the small-signal DC gain from an input source to an output, the resistance the input source sees, and the output resistance, all at the DC operating point (capacitors open, inductors shorted, nonlinear devices linearised).

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::{solve_tf, TfConfig, TfOutput};

// Output V(out), or TfOutput::current("Vsense"); input a voltage or current source.
let config = TfConfig::new(TfOutput::voltage("out"), "V1");
let tf = solve_tf(&circuit, &config)?;
println!("{} V/V, Rin {} Ω, Rout {} Ω", tf.gain, tf.input_resistance, tf.output_resistance);
# Ok::<(), Box<dyn std::error::Error>>(())
```

The input resistance is infinite when the input source draws no small-signal current. Every other independent source is zeroed for the output resistance. A current output (`I(Vx)`) runs from the source's `nodes[0]` through it to `nodes[1]`, as SPICE's.

## DC sensitivity

SPICE's `.sens`: the derivative of one DC output with respect to each component parameter, at the operating point. One adjoint solve covers them all, however many there are.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::{solve_sensitivity, SensConfig, TfOutput};

let config = SensConfig::new(TfOutput::voltage("out")); // or TfOutput::current("Vsense")
let sens = solve_sensitivity(&circuit, &config)?;

for e in &sens.entries {
    println!("{}: {} per unit, {} per +1 %", e.component_id, e.sensitivity, e.per_percent);
}
# Ok::<(), Box<dyn std::error::Error>>(())
```

Entries cover resistance, voltage and current source values, VCVS gain and VCCS gm. Other components get no entry: device-model parameters, CCVS and CCCS gains, capacitors and inductors. There is no AC sensitivity.

## Noise

SPICE's `.noise`: thermal noise of resistances and FET channels, shot noise of junction currents, and flicker (1/f) noise where a model sets `KF`, at an output voltage and referred back to an input source. One adjoint solve per frequency gives every source's share.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::{solve_noise, NoiseConfig};

// V(out) referred to V1, 41 log-spaced points from 10 Hz to 100 kHz.
let noise = solve_noise(&circuit, &NoiseConfig::new("out", "V1", 10.0, 100e3, 41))?;

println!("{:.2} µV rms at the output", noise.total_output_noise * 1e6);
for (source, total) in noise.sources.iter().zip(&noise.source_totals) {
    println!("  {} ({:?}): {:.2} µV", source.component, source.kind, total * 1e6);
}
# Ok::<(), Box<dyn std::error::Error>>(())
```

Each point has the output and input-referred densities (`output_density()`, `input_density()`, per √Hz) and each source's contribution. The totals integrate over the band as ngspice does. Noise is at 27 °C. Sources, controlled sources, op-amps, comparators, regulators and a diode's series resistance are noiseless.

## DC sweep

Sweep a voltage or current source across a range and collect operating points at each step. Each point's Newton–Raphson starts from the one before, as in SPICE.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::{Circuit, CircuitElement, dc_sweep};

let sweep = dc_sweep(&circuit, "V1", 0.0, 5.0, 51)?;

let v_curve = sweep.node_voltage_curve("n2");
let i_curve = sweep.component_current_curve("D1");
# Ok::<(), Box<dyn std::error::Error>>(())
```

Two sources, one inside the other (SPICE's `.dc VDS 0 5 0.1 VGS 0 3 0.5`), draw a family of device curves: one `DcSweepResult` along the inner source per value of the outer one.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::{dc_sweep_nested, dc_sweep::DcSweepAxis};

let family = dc_sweep_nested(
    &circuit,
    &DcSweepAxis::from_step("VDS", 0.0, 5.0, 0.1)?, // inner, fastest
    &DcSweepAxis::from_step("VGS", 0.0, 3.0, 0.5)?, // outer
)?;
for curve in &family.curves {
    let id_vs_vds = curve.sweep.component_current_curve("M1");
    println!("VGS = {} V: {} points", curve.outer_value, id_vs_vds.len());
}
# Ok::<(), Box<dyn std::error::Error>>(())
```

A parsed `.dc` card gives its axes with `analysis.dc_axes()`.

## Temperature sweep

Solve a circuit at a series of junction temperatures and collect operating points. Useful for BJT Ic vs. temperature curves and thermal characterisation.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::{Circuit, temperature_sweep, TempSweepResult};

// All junction devices (BJT, diode, LED, Zener, Schottky, Photodiode) have their
// temperature field overridden at each step.
let result = temperature_sweep(&circuit, 250.0, 350.0, 11)?; // 250 K → 350 K, 11 points

for point in &result.points {
    let ic = point.result.bjt_results.iter().find(|b| b.id == "Q1").map_or(0.0, |b| b.ic);
    println!("T = {:.1} K, Ic = {:.3} mA", point.temperature_kelvin, ic * 1000.0);
}
# Ok::<(), Box<dyn std::error::Error>>(())
```

```rust,ignore
pub struct TempSweepResult {
    pub temp_start: f64,              // K
    pub temp_end:   f64,              // K
    pub points:     Vec<TempSweepPoint>,
}

pub struct TempSweepPoint {
    pub temperature_kelvin: f64,
    pub result:             SimulationResult, // the full operating point at this temperature
}
```

`num_steps` must be ≥ 2, and both temperatures finite and above 0 K.

## Self-heating

Give a part a thermal path and the solver heats its junction by what it dissipates, then feeds that temperature back into the device: IS and kT/q for diodes, LEDs, Zeners, Schottkys, photodiodes and BJTs, and the resistance of a thermistor. The DC solve iterates until power and temperature agree. A transient carries a thermal RC state per part (`Cθ·dT/dt = P − (T − ambient)/Rθ`), starting cold at ambient.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::{solve_circuit, Part, SimError, Thermal};

// SOT-23 on its own (~500 K/W), or a TO-220 on a heatsink.
circuit.parts.insert("Q2".into(), Part { thermal: Some(Thermal::junction_to_ambient(500.0)), ..Part::default() });
circuit.parts.insert("Q3".into(), Part { thermal: Some(Thermal::on_heatsink(5.0, 10.0)), ..Part::default() });

match solve_circuit(&circuit) {
    Ok(result) => println!("Tj(Q2) = {:.1} K", result.junction_temperatures["Q2"]),
    Err(SimError::ThermalRunaway { part, temperature }) => println!("{part} runs away past {temperature:.0} K"),
    Err(e) => return Err(e),
}
# Ok::<(), SimError>(())
```

`Thermal` takes `rth_ja` (K/W), or `rth_jc` plus `rth_ca` for a part on a heatsink, an optional `cth` (J/K; without it a transient's junction follows its power with no lag), and `ambient` (K, default 300.15). With it, the element's own `temperature` field is ignored. `SimulationResult::junction_temperatures` and each `TimestepSnapshot`'s carry the result, and `Ratings::tj_max` is checked against them. When no balance exists (a junction past 1000 K, or the iteration never settles), the solve returns `SimError::ThermalRunaway`. AC, `.tf`, `.sens` and `.noise` solve the same heated bias point and linearise each device at its junction temperature. MOSFETs, JFETs, IGBTs and resistors report a junction temperature, but their models don't depend on it.

## Ratings checks

Check a solved circuit against each part's absolute maximum ratings, so nothing gets powered past its datasheet limits. Ratings live on the component's `Part` (`circuit.parts`, keyed by component id). `check_ratings` looks at the DC operating point, or at every step of a transient, where inrush, inductive kickback and start-up peaks show up.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::{check_ratings, solve_transient, Part, Ratings, RatingStatus, TransientConfig};

circuit.parts.insert("Q1".into(), Part {
    ratings: Ratings { vce: Some(45.0), ic: Some(0.1), power: Some(0.5), ..Ratings::default() },
    ..Part::default()
});

let result = solve_transient(&circuit, &TransientConfig::new(1e-3, 1e-6))?;
let report = check_ratings(&circuit, &result)?;
for c in report.exceeded() {
    println!("{} {:?}: {:.3} {} at t = {:?} (limit {:?}, margin {:.0}%)",
        c.part, c.rating, c.worst.unwrap(), c.rating.unit(), c.time, c.limit,
        c.margin_percent.unwrap_or(f64::NAN));
}
assert!(report.is_safe()); // false while anything is exceeded *or* unverified
# Ok::<(), Box<dyn std::error::Error>>(())
```

Each component kind has ratings it needs, checked whether set or not; a missing one is `RatingStatus::Unverified`, never a pass:

| Component | Needs |
|---|---|
| resistor, potentiometer, thermistor, photoresistor | `power` |
| capacitor | `voltage` |
| inductor | `current` |
| diode, Schottky, LED, photodiode | `reverse_voltage`, `forward_current`, `power` |
| Zener | `power` |
| varactor | `reverse_voltage` |
| BJT | `vce`, `vcb`, `veb`, `ic`, `power` |
| MOSFET, JFET | `vds`, `vgs`, `id`, `power` |
| IGBT | `vce`, `vgs` (gate–emitter), `ic`, `power` |
| anything with `Part::thermal` | also `tj_max` |

Any other rating set on a part is checked too: `power`, `voltage` and `current` on any component (on the port its `ComponentResult` reports), the rest on the devices they belong to. All limits are magnitudes, so PNP and PMOS parts use the same positive numbers. Transistor power is `|V·I|` on the Vce/Ic or Vds/Id port, without base or gate dissipation.

## Measurements and specs

Turn a result into numbers, and the numbers into a verdict. A `Measurement` reads a `Signal` (`v(out)`, `v(a, b)`, `i(R1)`, in dB or phase on AC) off any result: a transient, AC, DC or temperature sweep, or an operating point. A `Spec` adds limits, and `check_specs` reports each one's value, limits, pass/fail and margin.

```rust,no_run
# use sindr::*;
# let mut circuit = Circuit::default();
use sindr::measure::{check_specs, Crossing, Measure, Measurement, Signal, Spec, Stat};

let tran = solve_transient(&circuit, &TransientConfig::new(10e-3, 1e-6))?;
let specs = [
    Spec::new(Measurement::new("vout", Measure::stat(Signal::v("out"), Stat::Avg)).window(5e-3, 10e-3))
        .between(4.9, 5.1),
    Spec::new(Measurement::new("ripple", Measure::stat(Signal::v("out"), Stat::PeakToPeak)).window(5e-3, 10e-3))
        .at_most(0.05),
    Spec::new(Measurement::new("settle", Measure::settling(Signal::v("out"), 0.05))).at_most(2e-3),
];
let report = check_specs(&tran, &specs).with_ratings(check_ratings(&circuit, &tran)?);
for c in report.failed() {
    println!("{}: {:?} outside {:?}..{:?} by {:?}", c.name, c.value, c.min, c.max, c.margin);
}
assert!(report.passed()); // every spec passed and every part is within its ratings
# Ok::<(), Box<dyn std::error::Error>>(())
```

| `Measure` | SPICE `.meas` | Gives |
|---|---|---|
| `Find { at }` | `FIND v(out) AT=1m` | value at x, interpolated |
| `Stat` | `AVG`, `RMS`, `MIN`, `MAX`, `PP`, `INTEG` (and `Final`) | statistic over the window (`FROM=`/`TO=`) |
| `When(Crossing)` | `WHEN v(out)=2.5 RISE=2` | x of the nth rising, falling or any crossing, or the last |
| `Delay { trig, targ }` | `TRIG … VAL= … TARG … VAL=` | x(targ) − x(trig) |
| `Transition` | | rise or fall time between fractions of the swing (10–90 %) |
| `Settling { band }` | | time from the window start until it stays within ±band of its final value |
| `Bandwidth { drop_db }` | `WHEN vdb(out)=-3 FALL=1` | first x after the peak `drop_db` below it (−3 dB point) |

A measurement that can't be taken (an unknown node, a level never crossed, a phase on a transient) is `SpecStatus::Unmeasured` with a reason, never a pass; `Measurement::on` returns it as `SimError::Measurement`. Average and RMS are weighted by step width, so the transient's uneven steps don't skew them. With the `spice` feature, `.meas tran|ac|dc` cards land in `ParsedNetlist::measurements`, each with the analysis it measures.

## Importing a schematic netlist

`sindr::import` is for editors: map your netlist onto it and simulate a schematic without writing a SPICE deck. A `Netlist` holds components (your id, a `ComponentKind`, a value as a number or text, named parameters, an optional model and `Part`) and nets (your id, an optional display name, and the `(component, terminal)` pins on it). A `Bench` adds what the schematic leaves out: the ground net, sources and loads attached to nets or terminals (a connector pin, a test point), nets to tie together (an analog ground to ground), and the analysis to run.

```rust
use sindr::import::{simulate, Analysis, Bench, BenchSource, Component, ComponentKind, Net, Netlist, Simulation};

let netlist = Netlist {
    components: vec![
        Component::new("c1", ComponentKind::Resistor).with_value("10k"),
        Component::new("c2", ComponentKind::Resistor).with_value("4k7"),
    ],
    nets: vec![
        Net::new("vin", [("c1", "1")]),
        Net::new("vout", [("c1", "2"), ("c2", "1")]),
        Net::new("gnd", [("c2", "2")]),
    ],
};
let bench = Bench::new("gnd")
    .with_source(BenchSource::voltage("supply", "vin", 5.0))
    .with_analysis(Analysis::OperatingPoint);
let Simulation::OperatingPoint(op) = simulate(&netlist, &bench).unwrap().result else { unreachable!() };
println!("vout = {:.3} V", op.node_voltages["vout"]);
```

- **Ids carry through.** Element ids are your component ids and node names your net ids, so results land on your parts. Every `ImportError` names the component, net or bench item: an unknown kind, a terminal the kind doesn't have, a terminal on two nets or on none, a value that isn't one.
- **Terminals by name, never pin number:** `anode`/`cathode` (or `A`/`K`), `B`/`C`/`E`, `G`/`D`/`S`, `in+`/`in-`/`out`, case ignored. Unpolarised two-terminal parts also take `1` and `2`. `ComponentKind::terminals` lists each kind's names. A MOSFET's `body` must sit on its source's net; an op-amp's `V+`/`V-` pins set its rails from the bench supplies on those nets.
- **Values as schematics write them**, through `import::parse_value`: `4k7`, `4R7`, `2M2`, `100nF`, `4.7 µF`, `10 kΩ`. Case matters as on a schematic, so `1M` is a megohm (SPICE would read a milliohm).
- **Connectors, test points, net ties and mounting holes** add no element; a bench attaches to their terminals.
- **Power ports are only names.** A net called `3V3` isn't driven unless the bench says so. `Bench::with_supplies_from_net_names` adds a supply for every net whose name states a voltage (`3V3`, `+5V`, `-12V`), on request only.
- Everything is serde, so a netlist and bench can come in as JSON. The `ComponentKind` docs list each kind's value, parameters and model.

## Examples

One small program per capability in [`examples/`](./examples), run with `cargo run -p sindr --example <name>`:

| Example | Shows |
|---|---|
| `voltage_divider` | Linear DC operating point |
| `bjt_amplifier` | Nonlinear DC with a BJT |
| `transient_rc` | Transient step response |
| `bode_plot` | AC analysis |
| `transfer_function` | Transfer function (`.tf`) |
| `sensitivity` | DC sensitivity (`.sens`) |
| `dc_sweep_diode` | DC sweep (diode I–V curve) |
| `temperature_sweep` | Temperature sweep |
| `initial_voltages` | Seeding Newton–Raphson (`.nodeset`) |
| `import_netlist` | Importing a schematic netlist with a test bench |
| `parse_netlist` | SPICE input (`--features spice`) |
| `list_examples`, `run_example` | The built-in named circuits (`--features examples`) |

## Error handling

```rust,ignore
pub enum SimError {
    Design(DesignError),
    NoGround,
    DisconnectedNodes(Vec<String>),
    FloatingNode(String),
    SingularMatrix,
    InvalidSolution,
    InvalidComponent(String),
    InvalidResistance(String),
    ConvergenceFailed {
        iterations: usize,
        max_step_volts: f64,
        node: Option<String>,
        components: Vec<String>,
        at: SolvePoint,
    },
    ThermalRunaway { part: String, temperature: f64 },
    Measurement { name: String, reason: String },
}
```

`ConvergenceFailed` means Newton-Raphson did not converge — usually caused by a nonlinear circuit with no DC path to ground, or component values far outside typical operating ranges. The `max_step_volts` field gives the largest per-node Newton step (V) on the final iteration, useful for distinguishing slow convergence from genuine divergence. `node` is the node that took it, `components` the parts on that node, and `at` where the analysis failed (`SolvePoint::OperatingPoint`, `DcSweep`, `Temperature` or `Transient { time }`), so the message points at the part to fix. (Note: this is a step magnitude `max_i |V_new[i] − V_prev[i]|`, not a KCL residual `|F(x)|`.)

`ThermalRunaway` means self-heating found no thermal balance (`temperature` is the junction's, in K, when the solve gave up), `Measurement` that a measurement couldn't be taken on a result, and `Design` that the block hierarchy can't be flattened.

## Solver routing

`solve_circuit` picks a path automatically — you never select it manually:

| Circuit contains | Solver used |
|---|---|
| Only linear elements | Direct MNA (LU factorisation) |
| Nonlinear elements (diodes, BJTs, MOSFETs, IGBTs, comparators…) | Newton-Raphson iteration |
| Reactive elements (C, L, Varactor, Transformer, parasitic caps, a diode whose model sets `cjo` or `tt`) or waveform sources | Transient (Gear-2 by default) |
| Both reactive and nonlinear | Transient with per-step Newton-Raphson |

The transient solver steps up to `tstep` (or `tmax`, if smaller), shortens a step where the local truncation error would exceed SPICE's default tolerances (RELTOL 1e-3, VNTOL 1 µV, TRTOL 7), and lands exactly on every corner of a source waveform. A step whose Newton-Raphson solve fails is halved and retried.

## Performance

Medians from `cargo bench -p sindr --bench transient --bench newton` on an Apple M1 Pro (release build, one thread). The circuits are in `sindr/benches/`.

| Case | 10 | 100 | 1 000 |
|---|---:|---:|---:|
| DC, diode–resistor chain (devices) | 37 µs | 0.93 ms | 8.7 ms |
| DC, cascaded BJT common-emitter stages (devices) | 0.18 ms | 6.7 ms | 440 ms |
| Transient, RC ladder over 200 steps (stages) | 0.95 ms | 15 ms | 143 ms |

A 50 Hz half-wave rectifier with a filter cap over five cycles (5 000 steps) takes 9 ms. The cascaded BJT row grows faster than linearly, because each stage's bias depends on the one before it and Newton needs more iterations as the chain lengthens. `benches/mna_solve.rs` has the raw dense and sparse solve times up to 10 000 nodes (`--features internals`).

Pull requests run these benchmarks against their base on the same runner, and CI fails if any case is more than 25 % slower.

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
