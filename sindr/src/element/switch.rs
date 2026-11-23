//! A two-terminal switch, open or closed.

use super::Device;

/// Switch between two nodes. Modeled as a resistor with very low (closed)
/// or very high (open) resistance for DC analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Switch {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// When true the switch conducts (closed); when false it is open.
    pub closed: bool,
}

impl Device for Switch {
    device_identity!();
}
