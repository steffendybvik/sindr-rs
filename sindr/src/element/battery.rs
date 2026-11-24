//! A battery: open-circuit voltage by state of charge behind its internal
//! resistance and an optional RC pair.

use crate::battery_state;
use crate::error::SimError;
use crate::stamp::{stamp_companion_at, stamp_resistor_at};

use super::{AcStamp, Check, DcStamp, Device, TranStamp};
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::spice::export::{unsupported, Deck, ExportError};

/// A battery: open-circuit voltage by state of charge behind a series
/// resistance and an optional RC pair ([`BatteryParams`]). At DC it is
/// its open-circuit voltage at `state_of_charge` behind `R0 + R1`. In
/// transient the RC pair charges and the state of charge falls with the
/// charge drawn, reported per step in `state_of_charge`. In AC it is
/// the impedance `R0 + R1‖C1`. `current_through` flows
/// `nodes[0] → nodes[1]` inside the cell, so it is negative while the
/// battery discharges, as for a voltage source.
///
/// [`BatteryParams`]: sindr_devices::battery::BatteryParams
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Battery {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[positive, negative]`.
    pub nodes: [String; 2],
    /// The cell's equivalent circuit.
    pub params: sindr_devices::battery::BatteryParams,
    /// State of charge at the start, 0 (empty) to 1 (full).
    pub state_of_charge: f64,
}

impl Device for Battery {
    device_identity!();

    /// Its open-circuit voltage is a DC source, zero in AC; what is left
    /// is `Z = R0 + R1‖C1`. In the pencil form that is a branch current
    /// `i` through R0 into an internal node `m`, then R1‖C1 from `m` to
    /// `nodes[1]`.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let [p, q] = s.nodes();
        let params = &self.params;
        if let (true, Some(pair)) = (s.pencil, params.rc) {
            let (i, m) = (s.aux(), s.aux());
            let one = Complex64::new(1.0, 0.0);
            if let Some(p) = p {
                s.a[(p, i)] += one;
                s.a[(i, p)] += one;
            }
            s.a[(i, m)] -= one;
            s.a[(i, i)] -= Complex64::new(params.r_series, 0.0);
            s.a[(m, i)] -= one;
            let y = Complex64::new(1.0 / pair.r, 0.0) + s.s * pair.c;
            s.admittance([Some(m), q], y);
            return;
        }
        let mut z = Complex64::new(params.r_series, 0.0);
        if let Some(pair) = params.rc {
            z += Complex64::new(pair.r, 0.0) / (Complex64::new(1.0, 0.0) + s.s * pair.r * pair.c);
        }
        s.admittance([p, q], Complex64::new(1.0, 0.0) / z);
    }

    fn pencil_rows(&self) -> usize {
        if self.params.rc.is_some() {
            2
        } else {
            0
        }
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let (g, i_hist) = battery_state::dc_norton(&self.params, self.state_of_charge);
        let [p, q] = s.nodes();
        stamp_resistor_at(s.system, [p, q], 1.0 / g);
        // The Norton current: −i_hist out of nodes[1] into nodes[0]'s side.
        if let Some(p) = p {
            s.system.b[p] -= i_hist;
        }
        if let Some(q) = q {
            s.system.b[q] += i_hist;
        }
        Ok(())
    }

    /// Its Norton form over the step, from the state of charge and RC pair
    /// it starts the step with.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let soc = s.history(&self.id, battery_state::SOC).x1;
        let rc = s.history(&self.id, battery_state::RC);
        let (g, i_hist) = battery_state::step_norton(&self.params, soc, &rc, s.integ);
        let nodes = s.nodes();
        stamp_companion_at(s.system, nodes, g, i_hist);
        Ok(())
    }

    fn is_reactive(&self) -> bool {
        self.params.rc.is_some()
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        let params = &self.params;
        check.positive("params.capacity", params.capacity, " Ah")?;
        check.positive("params.r_series", params.r_series, " Ω")?;
        if let Some(pair) = params.rc {
            check.positive("params.rc.r", pair.r, " Ω")?;
            check.positive("params.rc.c", pair.c, " F")?;
        }
        let bad_curve = params.ocv.is_empty()
            || params
                .ocv
                .iter()
                .any(|&(soc, v)| !soc.is_finite() || !v.is_finite())
            || params.ocv.windows(2).any(|w| w[1].0 <= w[0].0);
        if bad_curve {
            return Err(check.fail(
                "params.ocv must be finite (state of charge, volts) points \
                 with the state of charge rising",
            ));
        }
        check.within("state_of_charge", self.state_of_charge, 1.0)
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, _: &mut Deck<'_>, _: &str) -> Result<(), ExportError> {
        Err(unsupported(
            &self.id,
            "a battery's state of charge has no SPICE3 primitive",
        ))
    }
}
