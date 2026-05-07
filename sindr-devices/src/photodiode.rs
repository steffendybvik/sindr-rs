//! Photodiode companion model.
//!
//! Model: I = IS*(exp(Vd/VT)-1) - I_photo
//! I_photo = responsivity * irradiance (photocurrent, flows in reverse direction)
//!
//! In dark (irradiance=0), behaves identically to a silicon diode.
//! With irradiance, produces reverse photocurrent that can drive loads.

use crate::diode::{DiodeParams, diode_companion};

/// Photodiode parameters.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct PhotodiodeParams {
    /// Dark saturation current (A).
    pub is: f64,
    /// Ideality factor (~1.0 for photodiodes).
    pub n: f64,
    /// Responsivity (A/W). Converts irradiance to photocurrent.
    /// Typical silicon photodiode: 0.4–0.7 A/W.
    pub responsivity: f64,
}

impl PhotodiodeParams {
    /// Silicon photodiode: IS = 1e-12 A, N = 1.0, responsivity = 0.5 A/W.
    pub fn silicon() -> Self {
        Self { is: 1e-12, n: 1.0, responsivity: 0.5 }
    }
}

impl Default for PhotodiodeParams {
    fn default() -> Self { Self::silicon() }
}

/// Linearized companion model for a photodiode at operating point v_d.
///
/// Returns (g_d, i_eq) where i_eq includes the photocurrent offset.
/// Photocurrent flows in reverse direction (opposite to diode forward current).
pub fn photodiode_companion(v_d: f64, irradiance: f64, params: &PhotodiodeParams) -> (f64, f64) {
    let (g_d, i_eq_dark) = diode_companion(v_d, &DiodeParams { is: params.is, n: params.n, rs: 0.0, temperature: 300.15 });
    let i_photo = params.responsivity * irradiance.max(0.0);
    // Photocurrent subtracts from the dark diode current (reverse direction)
    (g_d, i_eq_dark - i_photo)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn photodiode_dark_equals_standard_diode() {
        let params = PhotodiodeParams::silicon();
        let (g_photo, i_photo) = photodiode_companion(0.5, 0.0, &params);
        let (g_diode, i_diode) = diode_companion(0.5, &crate::diode::DiodeParams { is: params.is, n: params.n, rs: 0.0, temperature: 300.15 });
        assert_relative_eq!(g_photo, g_diode, max_relative = 1e-10);
        assert_relative_eq!(i_photo, i_diode, max_relative = 1e-10);
    }

    #[test]
    fn photocurrent_reduces_i_eq() {
        let params = PhotodiodeParams::silicon();
        // At irradiance = 0.1 W/m², I_photo = 0.5 * 0.1 = 50 mA
        let (_, i_eq_dark) = photodiode_companion(-0.5, 0.0, &params);
        let (_, i_eq_lit) = photodiode_companion(-0.5, 0.1, &params);
        // i_eq should be reduced by ~50 mA (photocurrent)
        let delta = i_eq_dark - i_eq_lit;
        assert!(
            (delta - 0.05).abs() < 0.001,
            "Photocurrent should be ~50mA at 0.1 W irradiance, got delta={}",
            delta
        );
    }

    #[test]
    fn negative_irradiance_clamped_to_zero() {
        let params = PhotodiodeParams::silicon();
        let (g1, i1) = photodiode_companion(0.0, 0.0, &params);
        let (g2, i2) = photodiode_companion(0.0, -10.0, &params);
        assert_relative_eq!(g1, g2, max_relative = 1e-10);
        assert_relative_eq!(i1, i2, max_relative = 1e-10);
    }
}
