# sindr-devices

Pure electronics device physics companion models for MNA circuit simulation.

`sindr-devices` is a standalone workspace crate that implements the nonlinear device models used by the `sindr` solver. It has no dependency on the solver — pure physics math, independently testable.

## Devices

| Module | Device | Model |
|--------|--------|-------|
| `diode` | Silicon diode | Shockley companion model with series resistance and temperature IS scaling |
| `bjt` | BJT transistor (NPN/PNP) | Ebers-Moll companion model with Early voltage |
| `mosfet` | MOSFET (NMOS/PMOS) | Level-1 MOSFET companion model |
| `varactor` | Varactor diode | Voltage-dependent junction capacitance C_j(V) |
| `igbt` | IGBT | MOSFET gate control with BJT output conductance |
| `schottky` | Schottky diode | Shockley with Schottky IS/N (~0.3 V forward) |
| `thermistor` | NTC thermistor | Beta model R(T) = R₀·exp(β·(1/T − 1/T₀)) |
| `photodiode` | Photodiode | Diode + photocurrent offset |

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
sindr-devices = { workspace = true }
```

### Diode companion model

```rust
use sindr_devices::diode::{DiodeParams, diode_companion, temperature_scale_is};

let params = DiodeParams::silicon(); // IS=1e-14, N=1.0, rs=0.0, temperature=300.15 K
let (g_eq, i_eq) = diode_companion(0.65, &params);
// g_eq — linearised conductance (S)
// i_eq — Norton current source (A)

// Temperature-dependent IS (SPICE formula)
let is_at_125c = temperature_scale_is(1e-14, 398.15, 300.15, 1.11, 3.0);
```

`DiodeParams` fields:
- `is: f64` — saturation current (A)
- `n: f64` — ideality factor
- `rs: f64` — series resistance (Ω), default 0.0; single-step Newton correction applied internally
- `temperature: f64` — junction temperature (K), default 300.15 K

### BJT companion model

```rust
use sindr_devices::bjt::{BjtParams, BjtKind, bjt_companion};

let params = BjtParams::default(); // IS=1e-15, BF=100, BR=1, vaf=0.0 (no Early effect)
let companion = bjt_companion(0.65, -5.0, &params);
// companion.g_ce — Early output conductance (A/V); 0.0 when vaf=0
// companion.{gbe, gbc, ice, ibc, ibe, ...}
```

`BjtParams` fields:
- `is, bf, br` — Ebers-Moll parameters
- `vaf: f64` — forward Early voltage (V); 0.0 = infinite (no Early effect)
- `var: f64` — reverse Early voltage (V); 0.0 = infinite
- `temperature: f64` — junction temperature (K), default 300.15 K

### MOSFET companion model

```rust
use sindr_devices::mosfet::{MosfetParams, MosfetKind, mosfet_companion};

let params = MosfetParams::default(); // Vto=0.7, Kp=2e-4, Lambda=0.01
let companion = mosfet_companion(2.0, 3.0, MosfetKind::Nmos, &params);
// companion.gds, companion.ids, companion.region (Cutoff/Triode/Saturation)
```

### Varactor junction capacitance

```rust
use sindr_devices::varactor::{VaractorParams, junction_capacitance, varactor_companion};

let params = VaractorParams { cj0: 10e-12, phi: 0.7, m: 0.5 };
let c_j = junction_capacitance(-2.0, &params); // reverse-biased capacitance

// Transient companion (dt=0.0 → DC open circuit)
let (g_eq, i_eq) = varactor_companion(v_prev, dt, &params);
```

`VaractorParams` fields: `cj0` (zero-bias cap, F), `phi` (built-in potential, V), `m` (grading coefficient).

### IGBT companion model

```rust
use sindr_devices::igbt::{IgbtParams, igbt_companion};

let params = IgbtParams::default(); // vth=5.0, k=5.0, vce_sat=2.0
let companion = igbt_companion(vge, vce, &params);
// companion.gm    — transconductance (A/V)
// companion.ids   — collector current (A)
// companion.g_ce  — output conductance = ids / vce_sat (A/V)
// companion.region — "cutoff" | "triode" | "saturation"
```

### Schottky diode

```rust
use sindr_devices::schottky::{SchottkyParams, schottky_companion};

let params = SchottkyParams::default(); // IS=1e-8, N=1.05 → ~0.3 V forward
let companion = schottky_companion(0.3, &params);
```

### NTC thermistor

```rust
use sindr_devices::thermistor::{ThermistorParams, thermistor_resistance};

let params = ThermistorParams::ntc_10k(); // R0=10 kΩ, beta=3950 K, T0=298.15 K
let r_at_85c = thermistor_resistance(358.15, &params); // ~2.5 kΩ
```

### Photodiode

```rust
use sindr_devices::photodiode::{PhotodiodeParams, photodiode_companion};

let params = PhotodiodeParams::default(); // responsivity=0.5 A/W, IS=1e-11
let companion = photodiode_companion(vd, 0.1 /* W irradiance */, &params);
// i_ph = responsivity * irradiance flows as reverse photocurrent
```

## Optional features

| Feature | Enables |
|---------|---------|
| `serde` | `Serialize`/`Deserialize` on all param structs |

```toml
sindr-devices = { workspace = true, features = ["serde"] }
```
