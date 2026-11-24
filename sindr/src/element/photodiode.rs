//! A photodiode: a junction diode in parallel with a photocurrent.

use crate::device_params;
use crate::error::SimError;

use super::{AcBias, AcStamp, Check, Conductance, DcStamp, Device};
use crate::noise::NoiseSources;
use crate::ratings::{RatingKind, RatingProbe};

/// Photodiode: diode + photocurrent source driven by irradiance.
/// nodes: [anode, cathode]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Photodiode {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[anode, cathode]`.
    pub nodes: [String; 2],
    /// Incident irradiance in W (optical power, not W/m²).
    /// 0.0 = dark, 0.1 = ~50mA photocurrent with default responsivity.
    #[cfg_attr(feature = "serde", serde(default))]
    pub irradiance: f64,
    /// Junction temperature (K), positive and finite. Default 300.15 K.
    /// Sets IS (SPICE temperature scaling) and the thermal voltage kT/q.
    #[cfg_attr(
        feature = "serde",
        serde(default = "super::default_junction_temperature")
    )]
    pub temperature: f64,
}

impl Device for Photodiode {
    device_identity!();

    /// The photocurrent is a constant source, with no AC part.
    fn linearise(&self, b: &AcBias<'_>) -> Vec<Conductance> {
        let params = device_params::photodiode(self.temperature);
        let (g_d, _) =
            sindr_devices::photodiode::photodiode_companion(b.across(), self.irradiance, &params);
        b.two_terminal(g_d)
    }

    /// All of it is in [`linearise`](Device::linearise).
    fn stamp_ac(&self, _: &mut AcStamp<'_>) {}

    fn noise(&self, n: &mut NoiseSources<'_>) {
        let [a, b] = &self.nodes;
        let i = n.current(&self.id);
        n.shot(&self.id, "", [a, b], i);
    }

    fn needed_ratings(&self) -> &'static [RatingKind] {
        use RatingKind::*;
        &[ReverseVoltage, ForwardCurrent, Power]
    }

    fn rating(&self, kind: RatingKind, m: &RatingProbe<'_>) -> Option<f64> {
        m.junction_rating(kind)
    }

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        let Some(vp) = s.v_prev else {
            return Ok(());
        };
        let params = device_params::photodiode(self.temperature);
        let [p, q] = s.nodes();
        let v_a = p.map_or(0.0, |i| vp[i]);
        let v_c = q.map_or(0.0, |i| vp[i]);
        let v_d = v_a - v_c;
        let (g_d, i_eq) =
            sindr_devices::photodiode::photodiode_companion(v_d, self.irradiance, &params);
        // Norton equivalent: conductance between anode and cathode, and a
        // current source.
        let system = &mut *s.system;
        if let Some(pi) = p {
            system.a[(pi, pi)] += g_d;
        }
        if let Some(qi) = q {
            system.a[(qi, qi)] += g_d;
        }
        if let (Some(pi), Some(qi)) = (p, q) {
            system.a[(pi, qi)] -= g_d;
            system.a[(qi, pi)] -= g_d;
        }
        if let Some(pi) = p {
            system.b[pi] -= i_eq;
        }
        if let Some(qi) = q {
            system.b[qi] += i_eq;
        }
        Ok(())
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        Check::new(&self.id).positive("temperature", self.temperature, " K")
    }
}
