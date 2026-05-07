//! Pure-Rust electronics device physics: linearised companion models for
//! semiconductor and passive components used by MNA-based circuit simulators.
//!
//! `sindr-devices` provides the small-signal linearisation that
//! Newton–Raphson solvers need at every iteration: given an applied voltage,
//! each device returns a `(g_eq, i_eq)` pair — an equivalent conductance and
//! Norton-current source. Stamp those into the MNA matrix and the solver
//! converges on the operating point.
//!
//! This crate has **no dependency** on a particular solver. Use it standalone
//! if you're building your own MNA implementation, or pair it with the
//! companion crate [`sindr`](https://crates.io/crates/sindr) for an
//! end-to-end simulator.
//!
//! # Devices
//!
//! | Module | Device | Model |
//! |---|---|---|
//! | [`diode`] | Silicon diode | Shockley with series resistance + temperature IS scaling |
//! | [`bjt`] | BJT (NPN/PNP) | Ebers–Moll with Early voltage |
//! | [`mosfet`] | MOSFET (NMOS/PMOS) | Level-1 |
//! | [`jfet`] | JFET (N/P-channel) | Shichman–Hodges |
//! | [`igbt`] | IGBT | MOSFET gate control + BJT output conductance |
//! | [`led`] | LED | Diode with colour-dependent forward voltage |
//! | [`zener`] | Zener diode | Shockley + reverse-breakdown branch |
//! | [`schottky`] | Schottky diode | Shockley with low-N, low-IS parameters |
//! | [`varactor`] | Varactor | Voltage-dependent junction capacitance |
//! | [`thermistor`] | NTC thermistor | Beta model `R(T) = R₀·exp(β·(1/T − 1/T₀))` |
//! | [`photodiode`] | Photodiode | Diode + photocurrent offset |
//! | [`photoresistor`] | Photoresistor (LDR) | Light-level-dependent resistance |
//!
//! # Quick example
//!
//! ```
//! use sindr_devices::diode::{DiodeParams, diode_companion};
//!
//! let params = DiodeParams::silicon();          // IS=1e-14, N=1.0, rs=0.0
//! let v_applied = 0.65;                          // forward bias, volts
//! let (g_eq, i_eq) = diode_companion(v_applied, &params);
//!
//! // Stamp g_eq into Y(p,p), Y(q,q), -g_eq into Y(p,q)/Y(q,p),
//! // and i_eq into b(p)/-i_eq into b(q) in your MNA system.
//! assert!(g_eq > 0.0);
//! ```
//!
//! # Conventions
//!
//! - **Voltages and currents** are in SI units (V, A).
//! - **Temperature** is in kelvin. Default is 300.15 K (≈ 27 °C, the SPICE
//!   default).
//! - **Sign convention** for two-terminal companions: `v_applied` is the
//!   voltage from `nodes[0]` to `nodes[1]`, and `i_eq` is the current
//!   flowing into `nodes[0]`.
//! - **Companion form**: each `*_companion` function returns the linearised
//!   `(g_eq, i_eq)` at the operating point — the form Newton–Raphson stamps
//!   directly.

pub mod bjt;
pub mod diode;
pub mod igbt;
pub mod jfet;
pub mod led;
pub mod mosfet;
pub mod photodiode;
pub mod photoresistor;
pub mod schottky;
pub mod thermistor;
pub mod varactor;
pub mod zener;
