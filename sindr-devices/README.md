# sindr-devices

Pure electronics device physics companion models for MNA circuit simulation.

`sindr-devices` is a standalone workspace crate that implements the nonlinear device models used by the `sindr` solver. It has no dependency on the solver — pure physics math, independently testable.

## Devices

| Module | Device | Model |
|--------|--------|-------|
| `diode` | Silicon diode | Shockley companion with series resistance and temperature IS scaling |
| `bjt` | BJT (NPN/PNP) | Ebers-Moll companion with Early voltage |
| `mosfet` | MOSFET (NMOS/PMOS) | Level-1 companion with body effect |
| `jfet` | JFET (N/P-channel) | Shichman-Hodges |
| `igbt` | IGBT | MOSFET square-law collector current, blocks reverse V_CE |
| `led` | LED | Diode with colour-dependent forward voltage |
| `zener` | Zener diode | Shockley plus reverse-breakdown branch |
| `schottky` | Schottky diode | Shockley with Schottky IS/N (~0.3 V forward) |
| `varactor` | Varactor diode | Voltage-dependent junction capacitance C_j(V) |
| `thermistor` | NTC thermistor | Beta model R(T) = R₀·exp(β·(1/T − 1/T₀)) |
| `photodiode` | Photodiode | Diode + photocurrent offset |
| `photoresistor` | Photoresistor (LDR) | Light-level-dependent resistance |

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
sindr-devices = "0.1.0-alpha.7"
```

Two-terminal companions return `(g_eq, i_eq)`: the linearised conductance (S)
and the Norton current source (A) at the operating point. Multi-terminal
devices return a companion struct.

### Diode

```rust
use sindr_devices::diode::{diode_companion, temperature_scale_is, DiodeParams};

let params = DiodeParams::silicon(); // IS=1e-14, N=1.0, rs=0.0, 300.15 K
let (g_eq, i_eq) = diode_companion(0.65, &params);
assert!(g_eq > 0.0);

// Temperature-scaled IS (SPICE formula): IS, T, Tnom, Eg, XTI
let is_at_125c = temperature_scale_is(1e-14, 398.15, 300.15, 1.11, 3.0);
assert!(is_at_125c > 1e-14);
```

`DiodeParams` fields: `is` (A), `n` (ideality), `rs` (series resistance, Ω),
`temperature` (K).

### BJT

```rust
use sindr_devices::bjt::{bjt_companion, BjtParams};

let params = BjtParams::new(100.0); // BF=100, IS=1e-14, no Early effect
let c = bjt_companion(0.65, -5.0, &params); // Vbe, Vbc
// c.g_be, c.g_bc, c.g_ce (Early output conductance, 0 when vaf = 0), c.ic, c.ib
assert!(c.ic > 0.0);
```

`BjtParams` fields: `is`, `bf`, `nf`, `br`, `nr`, `vaf` / `var` (Early
voltages, V; 0.0 = infinite), `temperature` (K).

### MOSFET

```rust
use sindr_devices::mosfet::{mosfet_companion, MosfetParams};

let params = MosfetParams::default_nmos(); // or default_pmos()
let c = mosfet_companion(2.0, 3.0, 0.0, &params); // Vgs, Vds, Vbs
// c.id, c.gm, c.gds, c.gmb (body effect), c.vth, c.region
assert!(c.id > 0.0);
```

### JFET

```rust
use sindr_devices::jfet::{jfet_companion, JfetKind};

let c = jfet_companion(-0.5, 5.0, JfetKind::NChannel, 10e-3, -2.0); // Vgs, Vds, kind, IDSS, Vp
// c.gm, c.gds, c.i_eq
assert!(c.gm > 0.0);
```

### IGBT

```rust
use sindr_devices::igbt::{igbt_companion, IgbtParams};

let params = IgbtParams::default(); // vth=5.0, k=5.0
let c = igbt_companion(10.0, 5.0, &params); // Vge, Vce
// c.gm = ∂Ids/∂Vge, c.g_ce = ∂Ids/∂Vce, c.ids, c.i_eq
assert!(c.ids > 0.0);
```

### Varactor

```rust
use sindr_devices::varactor::{junction_capacitance, varactor_companion, VaractorParams};

let params = VaractorParams { cj0: 10e-12, phi: 0.7, m: 0.5 };
let c_j = junction_capacitance(-2.0, &params); // reverse-biased capacitance (F)
assert!(c_j < params.cj0);

// Backward Euler companion from the previous timestep's voltage; dt = 0.0 gives a DC open circuit.
let (g_eq, i_eq) = varactor_companion(-2.0, 1e-6, &params);
```

### Other diodes and sensors

```rust
use sindr_devices::led::{led_params, LedColor};
use sindr_devices::photodiode::{photodiode_companion, PhotodiodeParams};
use sindr_devices::photoresistor::{ldr_resistance, PhotoresistorParams};
use sindr_devices::schottky::{schottky_companion, SchottkyParams};
use sindr_devices::thermistor::{thermistor_resistance, ThermistorParams};
use sindr_devices::zener::{zener_companion, ZenerParams};

let red = led_params(LedColor::Red); // DiodeParams for use with diode_companion
let (g, i) = schottky_companion(0.3, &SchottkyParams::default()); // IS=1e-8, N=1.0
let (g, i) = zener_companion(-5.5, &ZenerParams::new(5.1)); // Vz = 5.1 V
let (g, i) = photodiode_companion(-1.0, 0.1, &PhotodiodeParams::default()); // 0.1 W irradiance

let r_85c = thermistor_resistance(358.15, &ThermistorParams::ntc_10k()); // ~2.5 kΩ
let r_dim = ldr_resistance(0.5, &PhotoresistorParams::default()); // 0.0 dark – 1.0 bright
assert!(r_85c < 10_000.0 && r_dim > 1_000.0);
```

## Optional features

| Feature | Enables |
|---------|---------|
| `serde` | `Serialize`/`Deserialize` on all param structs |

```toml
sindr-devices = { version = "0.1.0-alpha.7", features = ["serde"] }
```
