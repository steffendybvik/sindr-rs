//! A relay: a coil whose voltage opens or closes a contact pair.

use nalgebra::DVector;

use crate::error::SimError;
use crate::node_map::NodeMap;
use crate::stamp::{stamp_resistor_at, SWITCH_R_CLOSED, SWITCH_R_OPEN};

use super::{Check, DcStamp, Device};

/// Relay: 4-terminal component with a coil and a contact pair.
/// nodes: [coil+, coil-, contact1, contact2]
/// Contact closes when |V_coil| >= pickup_voltage. AC and `.tf` take the
/// contact as open or closed at the DC bias point.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Relay {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[coil+, coil-, contact1, contact2]`.
    pub nodes: [String; 4],
    /// Coil resistance (Ω). Default 500.
    #[cfg_attr(feature = "serde", serde(default = "default_relay_coil_resistance"))]
    pub coil_resistance: f64,
    /// Coil voltage magnitude at which the contact closes (V).
    pub pickup_voltage: f64,
    /// Coil inductance in Henry, in series with `coil_resistance` in
    /// transient. 0.0 = purely resistive coil.
    #[cfg_attr(feature = "serde", serde(default))]
    pub inductance: f64,
}

impl Device for Relay {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        // The coil is a resistive load; its inductance is the transient's.
        let [coil_a, coil_b, contact_a, contact_b] = s.nodes();
        stamp_resistor_at(s.system, [coil_a, coil_b], self.coil_resistance);
        let contact = relay_contact_resistance_at([coil_a, coil_b], self.pickup_voltage, s.v_prev);
        stamp_resistor_at(s.system, [contact_a, contact_b], contact);
        Ok(())
    }

    fn inductor_histories(&self) -> usize {
        usize::from(self.inductance > 0.0)
    }

    /// The contact switches on the coil voltage, which takes Newton
    /// iterations to settle.
    fn is_nonlinear(&self) -> bool {
        true
    }

    fn is_reactive(&self) -> bool {
        self.inductance > 0.0
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.positive("coil_resistance", self.coil_resistance, " Ω")?;
        check.positive("pickup_voltage", self.pickup_voltage, " V")?;
        check.non_negative("inductance", self.inductance, " H")
    }
}

#[cfg(feature = "serde")]
fn default_relay_coil_resistance() -> f64 {
    500.0
}

/// Resistance of a relay's contact (`nodes[2]`–`nodes[3]`): closed when the
/// coil voltage `nodes[0]`–`nodes[1]` in `v` is at least `pickup_voltage` in
/// magnitude, open otherwise and when there is no `v` yet.
pub(crate) fn relay_contact_resistance(
    node_map: &NodeMap,
    nodes: &[String; 4],
    pickup_voltage: f64,
    v: Option<&DVector<f64>>,
) -> f64 {
    let coil = [node_map.index(&nodes[0]), node_map.index(&nodes[1])];
    relay_contact_resistance_at(coil, pickup_voltage, v)
}

/// [`relay_contact_resistance`] with the coil's terminals as matrix
/// indices (`None` is ground).
pub(crate) fn relay_contact_resistance_at(
    [a, b]: [Option<usize>; 2],
    pickup_voltage: f64,
    v: Option<&DVector<f64>>,
) -> f64 {
    let closed = v.is_some_and(|v| {
        let at = |i: Option<usize>| i.map_or(0.0, |i| v[i]);
        (at(a) - at(b)).abs() >= pickup_voltage
    });
    if closed {
        SWITCH_R_CLOSED
    } else {
        SWITCH_R_OPEN
    }
}
