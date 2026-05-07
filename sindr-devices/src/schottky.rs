//! Schottky diode companion model.
//!
//! Schottky diodes use the same Shockley equation as silicon diodes
//! but with higher saturation current (IS ≈ 1e-8 A) and lower forward
//! voltage (~0.3V vs ~0.6V silicon). Reuses diode_companion().

use crate::diode::{DiodeParams, diode_companion};

/// Schottky diode parameters.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct SchottkyParams {
    /// Saturation current (A). Higher than silicon due to lower barrier.
    pub is: f64,
    /// Emission coefficient. ~1.0 for Schottky (more ideal than silicon).
    pub n: f64,
}

impl SchottkyParams {
    /// Standard BAT85 Schottky: IS = 1e-8 A, N = 1.0 → Vf ≈ 0.3V at 10 mA.
    pub fn standard() -> Self {
        Self { is: 1e-8, n: 1.0 }
    }
}

impl Default for SchottkyParams {
    fn default() -> Self { Self::standard() }
}

/// Linearized companion model for a Schottky diode at operating point v_d.
///
/// Returns (g_d, i_eq): conductance and equivalent Norton current.
/// Delegates to diode_companion with Schottky parameters.
pub fn schottky_companion(v_d: f64, params: &SchottkyParams) -> (f64, f64) {
    diode_companion(v_d, &DiodeParams { is: params.is, n: params.n, rs: 0.0, temperature: 300.15 })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schottky_forward_voltage_lower_than_silicon() {
        // At 10 mA, silicon Vf ≈ 0.6V, Schottky Vf ≈ 0.3V
        // Test that at v_d = 0.3, Schottky conducts significantly more than silicon
        let params = SchottkyParams::standard();
        let (g_s, _i_s) = schottky_companion(0.3, &params);
        // g > GMIN means device is conducting (not just leakage)
        assert!(g_s > 1e-6, "Schottky should conduct at 0.3V, got g={}", g_s);

        // Silicon diode at same voltage conducts much less
        let (g_si, _) = diode_companion(0.3, &crate::diode::DiodeParams { is: 1e-14, n: 1.0, rs: 0.0, temperature: 300.15 });
        assert!(g_s > g_si * 1000.0, "Schottky should have >> 1000x conductance of silicon at 0.3V");
    }

    #[test]
    fn schottky_reverse_bias_near_zero() {
        let params = SchottkyParams::standard();
        let (g_d, i_eq) = schottky_companion(-5.0, &params);
        // At -5V reverse bias, conductance is essentially zero (exp(-193) ≈ 0)
        assert!(g_d < 1e-6, "Reverse bias conductance should be near zero, got g={}", g_d);
        assert!(i_eq.abs() < 1e-6, "Reverse i_eq should be tiny");
    }
}
