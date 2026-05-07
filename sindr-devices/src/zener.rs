//! Zener diode model: piecewise companion for Newton-Raphson simulation.
//!
//! Forward bias: standard Shockley diode.
//! Reverse breakdown: linearised clamp at -Vz.

use crate::diode::V_T;

/// Zener diode parameters.
#[derive(Debug, Clone)]
pub struct ZenerParams {
    /// Breakdown voltage (positive, e.g. 5.1 for a BZX55C5V1).
    pub vz: f64,
    /// Breakdown region dynamic resistance (Ω). Smaller = sharper knee.
    pub rz: f64,
    /// Saturation current for the forward diode region.
    pub is: f64,
}

impl ZenerParams {
    pub fn new(vz: f64) -> Self {
        Self {
            vz,
            rz: 1.0,
            is: 1e-14,
        }
    }
}

impl Default for ZenerParams {
    fn default() -> Self {
        Self::new(5.1)
    }
}

/// Piecewise Zener companion model. Returns `(g_eq, i_eq)` for MNA stamping.
///
/// - `v_d > -vz`: standard Shockley companion (clamped to avoid overflow).
/// - `v_d <= -vz`: linearised 1/rz clamp driving the junction to -Vz.
pub fn zener_companion(v_d: f64, params: &ZenerParams) -> (f64, f64) {
    const CLAMP: f64 = 40.0;

    if v_d > -params.vz {
        let nv_t = V_T; // emission coefficient = 1 for silicon
        let v_clamped = v_d.min(CLAMP * nv_t);
        let exp_v = (v_clamped / nv_t).exp();
        let i_d = params.is * (exp_v - 1.0);
        let g_d = (params.is / nv_t) * exp_v;
        let i_eq = i_d - g_d * v_clamped;
        (g_d, i_eq)
    } else {
        // Breakdown: I = (v_d + Vz) / rz
        // Companion form: I = g_z * v_d + i_eq, where i_eq = Vz / rz
        let g_z = 1.0 / params.rz;
        let i_eq = params.vz / params.rz;
        (g_z, i_eq)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_bias_conducts() {
        let p = ZenerParams::new(5.1);
        let (g, _) = zener_companion(0.65, &p);
        assert!(g > 1e-3);
    }

    #[test]
    fn reverse_before_breakdown_low_conductance() {
        let p = ZenerParams::new(5.1);
        let (g, _) = zener_companion(-1.0, &p);
        assert!(g < 1e-3); // still in Shockley region, near zero
    }

    #[test]
    fn breakdown_clamps_at_vz() {
        let p = ZenerParams::new(5.1);
        let (g, i_eq) = zener_companion(-6.0, &p);
        // At v_d = -Vz: I = g * (-Vz) + i_eq should equal 0
        let i_at_vz = g * (-p.vz) + i_eq;
        assert!(
            i_at_vz.abs() < 1e-10,
            "Current at -Vz should be zero, got {i_at_vz}"
        );
    }
}
