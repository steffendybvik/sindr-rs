//! A voltage-controlled voltage source, SPICE's `E`.

use crate::error::SimError;

use super::{Check, Device};

/// Voltage-Controlled Voltage Source: V_out = mu * V_control.
/// nodes: [out+, out-], control_nodes: [ctrl+, ctrl-]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Vcvs {
    /// Unique component identifier.
    pub id: String,
    /// Output terminals `[out+, out-]`.
    pub nodes: [String; 2],
    /// Sensing terminals `[ctrl+, ctrl-]`.
    pub control_nodes: [String; 2],
    /// Voltage gain μ (V/V): `V_out = gain * V_control`.
    pub gain: f64, // mu
}

impl Device for Vcvs {
    device_identity!();

    fn all_nodes(&self) -> Vec<&String> {
        let [a, b] = &self.nodes;
        let [c, d] = &self.control_nodes;
        vec![a, b, c, d]
    }

    fn branches(&self) -> usize {
        1
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).finite("gain", self.gain, "")
    }
}
