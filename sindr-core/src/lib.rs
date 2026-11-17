//! The circuit and design data model behind
//! [`sindr`](https://crates.io/crates/sindr), without the solver.
//!
//! - [`circuit`]: [`Circuit`] and every [`CircuitElement`] sindr simulates
//! - [`design`]: hierarchy ([`Block`], [`Instance`]), part data ([`Part`])
//!   and [`Circuit::flatten`]
//! - [`waveform`]: time-varying source [`Waveform`]s
//!
//! The only dependencies are `sindr-devices` (device parameter types),
//! `thiserror` and, behind the default `serde` feature, `serde`. Design
//! tools can build and exchange circuits with this crate alone; `sindr`
//! re-exports all of it and simulates them.
//!
//! ```
//! use sindr_core::{Circuit, CircuitElement, Part};
//!
//! let mut circuit = Circuit::new(
//!     "0",
//!     vec![CircuitElement::Resistor {
//!         id: "R1".into(),
//!         nodes: ["in".into(), "0".into()],
//!         resistance: 10e3,
//!     }],
//! );
//! circuit.parts.insert(
//!     "R1".into(),
//!     Part { value: Some("10k".into()), footprint: Some("R_0603".into()), ..Part::default() },
//! );
//! assert_eq!(circuit.nets(), ["0", "in"]);
//! ```

#![forbid(unsafe_code)]
#![cfg_attr(docsrs, feature(doc_cfg))]

pub mod circuit;
pub mod design;
pub mod waveform;

pub use circuit::{BjtModel, BjtParasiticCaps, Circuit, CircuitElement, DiodeModel, MosfetParasiticCaps};
pub use design::{Block, DesignError, Instance, Part};
pub use waveform::Waveform;
