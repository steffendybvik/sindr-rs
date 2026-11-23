//! A linear voltage regulator with dropout, quiescent current and a
//! current limit.

use crate::error::SimError;

use super::{Check, Device};

/// Linear voltage regulator: holds the output at `voltage` above its
/// ground pin while it has headroom, and draws its output current plus
/// its quiescent current from the input.
///
/// Piecewise linear (see `sindr_devices::regulator`): out of headroom
/// the output follows the input down by the dropout; past the current
/// limit it holds the current. `model` sets dropout, quiescent current,
/// current limit and line and load regulation;
/// `RegulatorParams::lm7805()` and `::ams1117()` are datasheet presets.
/// Its result is the pass element: `Vin − Vout`, the output current,
/// and the power it dissipates.
///
/// `nodes: [input, output, gnd]`
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct VoltageRegulator {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[input, output, gnd]`.
    pub nodes: [String; 3], // [input, output, gnd]
    /// Regulated output voltage (V) from the ground pin. E.g. 5.0 for a
    /// 7805, or −5.0 for a 7905, whose input sits below ground and whose
    /// output current flows in at the output pin.
    pub voltage: f64,
    /// Dropout, quiescent current, limit and regulation. `None` is an
    /// ideal regulator that still can't put out more than it gets in.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub model: Option<sindr_devices::regulator::RegulatorParams>,
}

impl Device for VoltageRegulator {
    device_identity!();

    /// Its output current.
    fn branches(&self) -> usize {
        1
    }

    /// Dropout and the current limit make it piecewise linear.
    fn is_nonlinear(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        let voltage = self.voltage;
        // Negative for a 79xx-style regulator; zero regulates nothing.
        if !voltage.is_finite() || voltage == 0.0 {
            return Err(check.fail(format_args!(
                "voltage {voltage} V must be a non-zero, finite number"
            )));
        }
        if let Some(m) = &self.model {
            check.non_negative("model.dropout", m.dropout, " V")?;
            check.non_negative("model.dropout_resistance", m.dropout_resistance, " Ω")?;
            check.non_negative("model.quiescent_current", m.quiescent_current, " A")?;
            check.non_negative("model.line_regulation", m.line_regulation, " V/V")?;
            check.non_negative("model.output_resistance", m.output_resistance, " Ω")?;
            if let Some(limit) = m.current_limit {
                check.positive("model.current_limit", limit, " A")?;
            }
        }
        Ok(())
    }
}
