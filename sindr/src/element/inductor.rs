//! An inductor, linear or with a saturating core.

use crate::handle::NodeId;
use sindr_devices::inductor;

use crate::error::SimError;
use crate::stamp::stamp_companion_at;

use super::{AcStamp, Check, DcStamp, Device, Report, TranStamp};
use crate::ratings::RatingKind;
use num_complex::Complex64;

#[cfg(feature = "spice")]
use crate::spice::export::{num, unsupported, Deck, ExportError};

/// Inductor. Stamped as **open-circuit in pure DC analysis** (sindr
/// solves DC as a single algebraic step, not as t→∞ of a transient);
/// integrates `dI/dt` properly during transient. The DC bias point that
/// AC, `.tf` and `.sens` linearise around shorts it instead. `voltage_across` is
/// reported as `nodes[0] − nodes[1]`, `current_through` flows
/// `nodes[0] → nodes[1]`. To get steady-state current, run a
/// transient long enough to settle.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Inductor {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[a, b]`; `current_through` flows `nodes[0] → nodes[1]`.
    pub nodes: [NodeId; 2],
    /// Inductance (H): the small-signal value, at zero current.
    pub inductance: f64,
    /// Core saturation: the inductance falls towards `l_sat` as the
    /// current passes `i_sat`. `None` is a linear inductor.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub saturation: Option<sindr_devices::inductor::InductorSaturation>,
}

impl Device for Inductor {
    device_identity!();

    /// A short at DC, where it reports nothing across or through it. A
    /// saturating core's history holds its flux.
    fn report(&self, r: &mut Report<'_>) {
        let Some(step) = r.step() else {
            r.component(&self.id, 0.0, 0.0, 0.0);
            return;
        };
        let v = r.across(0, 1);
        let history = r.inductor(0);
        let i = match &self.saturation {
            None => {
                let (g, i_hist) = step.integ.inductor(self.inductance, &history);
                let i = g * v + i_hist;
                r.set_inductor(0, history.push(i, v));
                i
            }
            Some(sat) => {
                let lambda = step.integ.flux(v, &history);
                r.set_inductor(0, history.push(lambda, v));
                inductor::current_from_flux(lambda, self.inductance, sat)
            }
        };
        r.port(&self.id, v, i);
    }

    /// `1/(sL)`, or in the pencil form the branch `V_p − V_q = sL·i`,
    /// which is affine in `s`. At `s = 0` a short.
    fn stamp_ac(&self, s: &mut AcStamp<'_>) {
        let [p, q] = s.nodes();
        if s.pencil {
            let i = s.aux();
            s.branch_pair(i, [p, q]);
            s.a[(i, i)] -= s.s * self.inductance;
            return;
        }
        let y = if s.s.norm() > 1e-15 {
            Complex64::new(1.0, 0.0) / (s.s * self.inductance)
        } else {
            Complex64::new(1e12, 0.0)
        };
        s.admittance([p, q], y);
    }

    fn pencil_rows(&self) -> usize {
        1
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        &[RatingKind::Current]
    }

    fn stamp_dc(&self, _: &mut DcStamp<'_>) -> Result<(), SimError> {
        // Nothing of its own: the DC analyses short it with a 0 V source
        // first (`dc_equivalent`).
        Ok(())
    }

    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let history = s.inductor(0);
        let nodes = s.nodes();
        let (g, i_hist) = match &self.saturation {
            None => s.integ.inductor(self.inductance, history),
            Some(sat) => {
                // Its history holds fluxes. At the Newton iterate's
                // voltage the step ends at flux λ(v), carrying i(λ);
                // di/dv = (dλ/dv)/L(i).
                let [a, b] = nodes;
                let v = s.voltage(a) - s.voltage(b);
                let lambda = s.integ.flux(v, history);
                let i = inductor::current_from_flux(lambda, self.inductance, sat);
                let l = inductor::incremental_inductance(i, self.inductance, sat);
                let g = s.integ.flux_per_volt() / l;
                (g, i - g * v)
            }
        };
        stamp_companion_at(s.system, nodes, g, i_hist);
        Ok(())
    }

    fn inductor_histories(&self) -> usize {
        1
    }

    fn is_nonlinear(&self) -> bool {
        self.saturation.is_some()
    }

    fn is_reactive(&self) -> bool {
        true
    }

    /// A zero inductance would divide by zero in the companion model, so
    /// it must be positive.
    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        check.positive("inductance", self.inductance, " H")?;
        if let Some(sat) = &self.saturation {
            check.positive("saturation.i_sat", sat.i_sat, " A")?;
            check.positive("saturation.l_sat", sat.l_sat, " H")?;
            if sat.l_sat > self.inductance {
                return Err(check.fail(format_args!(
                    "saturation.l_sat {} H must not exceed the inductance {} H",
                    sat.l_sat, self.inductance
                )));
            }
        }
        Ok(())
    }

    #[cfg(feature = "spice")]
    fn spice_letter(&self) -> char {
        'L'
    }

    #[cfg(feature = "spice")]
    fn write_spice(&self, deck: &mut Deck<'_>, name: &str) -> Result<(), ExportError> {
        if self.saturation.is_some() {
            return Err(unsupported(
                &self.id,
                "a saturating core has no SPICE3 primitive",
            ));
        }
        let value = num(&self.id, "inductance", self.inductance)?;
        deck.two_terminal(name, &self.nodes, value)
    }
}
