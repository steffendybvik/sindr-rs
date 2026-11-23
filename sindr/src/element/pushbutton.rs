//! A momentary pushbutton, pressed or released.

use super::Device;

/// Pushbutton (momentary switch): identical behaviour to Switch.
/// When closed = true, current flows; when false, open circuit.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Pushbutton {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`.
    pub nodes: [String; 2],
    /// When true the button conducts (pressed); when false it is open.
    pub closed: bool,
}

impl Device for Pushbutton {
    device_identity!();
}
