//! A relay: a coil whose voltage opens or closes a contact pair.

use crate::handle::NodeId;
use nalgebra::DVector;

use crate::error::SimError;
use crate::stamp::{stamp_companion_at, stamp_resistor_at, SWITCH_R_CLOSED, SWITCH_R_OPEN};

use super::{AcStamp, Check, DcStamp, Device, Report, TranStamp};
use crate::results::RelayResult;

#[cfg(feature = "spice")]
use crate::spice::export::{unsupported, Deck, ExportError};

/// Relay: 4-terminal component with a coil and a contact pair.
/// nodes: [coil+, coil-, contact1, contact2]
/// Contact closes when |V_coil| >= pickup_voltage. AC and `.tf` take the
/// contact as open or closed at the DC bias point.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Relay {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[coil+, coil-, contact1, contact2]`.
    pub nodes: [NodeId; 4],
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

    /// Its coil's voltage and current, and after a DC solve whether the
    /// contact has picked up. With inductance, the coil current over a
    /// step is the inductor's, and the inductor sees the coil voltage less
    /// the resistive drop.
    fn report(&self, r: &mut Report<'_>) {
        let coil_voltage = r.across(0, 1);
        let coil_current = match r.step() {
            Some(step) if self.inductance > 0.0 => {
                let history = r.inductor(0);
                let (g, i_hist) = step.integ.inductor(self.inductance, &history);
                let (g_s, i_s) = series_resistance(g, i_hist, self.coil_resistance);
                let current = g_s * coil_voltage + i_s;
                let v_inductor = coil_voltage - self.coil_resistance * current;
                r.set_inductor(0, history.push(current, v_inductor));
                current
            }
            _ => coil_voltage / self.coil_resistance,
        };
        r.port(&self.id, coil_voltage, coil_current);
        r.relay(|| RelayResult {
            id: self.id.clone(),
            coil_voltage,
            contact_closed: coil_voltage.abs() >= self.pickup_voltage,
        });
    }

    /// The coil's resistance, and the contact open or closed as at the DC
    /// bias point.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let [coil_a, coil_b, contact_a, contact_b] = s.nodes();
        s.conductance([coil_a, coil_b], 1.0 / self.coil_resistance);
        let r =
            relay_contact_resistance_at([coil_a, coil_b], self.pickup_voltage, Some(s.dc_solution));
        s.conductance([contact_a, contact_b], 1.0 / r);
    }

    /// The coil's ends and the contact's; nothing joins the coil to the
    /// contact.
    fn dc_paths<'a>(&'a self, _: &'a NodeId) -> Vec<[&'a NodeId; 2]> {
        let [coil_a, coil_b, contact_a, contact_b] = &self.nodes;
        vec![[coil_a, coil_b], [contact_a, contact_b]]
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        // The coil is a resistive load; its inductance is the transient's.
        let [coil_a, coil_b, contact_a, contact_b] = s.nodes();
        stamp_resistor_at(s.system, [coil_a, coil_b], self.coil_resistance);
        let contact = relay_contact_resistance_at([coil_a, coil_b], self.pickup_voltage, s.v_prev);
        stamp_resistor_at(s.system, [contact_a, contact_b], contact);
        Ok(())
    }

    /// The coil's inductance, if it has one, in series with its
    /// resistance over the step; the contact as at DC.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let [coil_a, coil_b, contact_a, contact_b] = s.nodes();
        if self.inductance > 0.0 {
            let (g, i_hist) = s.integ.inductor(self.inductance, s.inductor(0));
            let (g, i_hist) = series_resistance(g, i_hist, self.coil_resistance);
            stamp_companion_at(s.system, [coil_a, coil_b], g, i_hist);
        } else {
            stamp_resistor_at(s.system, [coil_a, coil_b], self.coil_resistance);
        }
        let contact =
            relay_contact_resistance_at([coil_a, coil_b], self.pickup_voltage, Some(s.v_prev));
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

    #[cfg(feature = "spice")]
    fn write_spice(&self, _: &mut Deck<'_>, _: &str) -> Result<(), ExportError> {
        Err(unsupported(
            &self.id,
            "a voltage-operated contact has no SPICE3 primitive",
        ))
    }
}

#[cfg(feature = "serde")]
fn default_relay_coil_resistance() -> f64 {
    500.0
}

/// A companion `i = g·v + i_hist` in series with a resistance `r`, as one
/// companion across both: from `v = r·i + v_L` and `i = g·v_L + i_hist`,
/// `i = g/(1 + g·r)·v + i_hist/(1 + g·r)`. A relay coil is its winding's
/// resistance in series with its inductance.
pub(crate) fn series_resistance(g: f64, i_hist: f64, r: f64) -> (f64, f64) {
    let k = 1.0 + g * r;
    (g / k, i_hist / k)
}

/// Resistance of a relay's contact: closed when the coil voltage from
/// `a` to `b` (matrix indices, `None` is ground) in `v` is at least
/// `pickup_voltage` in magnitude, open otherwise and when there is no `v`
/// yet.
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
