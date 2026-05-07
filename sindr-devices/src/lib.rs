//! Pure electronics device physics for MNA circuit simulation.
//!
//! Provides linearized companion models for diodes, BJTs, and MOSFETs,
//! suitable for use in Newton-Raphson simulation engines.

pub mod diode;
pub mod bjt;
pub mod mosfet;
pub mod led;
pub mod zener;
pub mod schottky;
pub mod thermistor;
pub mod photodiode;
pub mod photoresistor;
pub mod varactor;
pub mod igbt;
pub mod jfet;
