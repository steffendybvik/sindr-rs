//! A light-emitting diode, its forward drop set by its colour.

use crate::error::SimError;

use super::{Check, Device};

/// LED. `nodes: [anode, cathode]`. Forward voltage drop depends on
/// `color` (e.g. `"red"` ≈ 1.8 V, `"blue"` ≈ 3.2 V).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Led {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Emission colour, any case: `"red"`, `"orange"`, `"amber"`,
    /// `"yellow"`, `"green"`, `"blue"` or `"white"`
    /// ([`led::COLORS`](sindr_devices::led::COLORS)); sets the forward
    /// voltage drop. Any other name is red.
    pub color: String,
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(
        feature = "serde",
        serde(default = "super::default_junction_temperature")
    )]
    pub temperature: f64,
}

impl Device for Led {
    device_identity!();

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).positive("temperature", self.temperature, " K")
    }
}
