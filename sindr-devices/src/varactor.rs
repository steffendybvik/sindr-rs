//! Varactor diode model: voltage-dependent junction capacitance.
//!
//! A varactor is a reverse-biased diode whose junction capacitance varies with
//! applied voltage. In DC analysis it is an open circuit. In transient analysis
//! the capacitance is evaluated at the previous timestep voltage (freeze-and-stamp).

/// Varactor diode parameters.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VaractorParams {
    /// Zero-bias junction capacitance (F). Typical: 10e-12 (10 pF).
    pub cj0: f64,
    /// Built-in junction potential (V). Typical: 0.7 for Si.
    pub phi: f64,
    /// Grading coefficient (dimensionless). Typical: 0.5 (abrupt junction).
    pub m: f64,
}

impl Default for VaractorParams {
    fn default() -> Self {
        Self { cj0: 10e-12, phi: 0.7, m: 0.5 }
    }
}

/// Compute junction capacitance at voltage `v`.
///
/// C_j(V) = C_j0 / (1 - V/phi)^m
///
/// Clamped: V must not reach phi (singularity). Clamp V to 0.9 * phi max.
/// For reverse bias (V < 0), capacitance increases correctly.
/// For forward bias (V approaching phi), capacitance would diverge — clamp.
pub fn junction_capacitance(v: f64, params: &VaractorParams) -> f64 {
    let v_clamped = v.min(0.9 * params.phi);
    let denom = (1.0 - v_clamped / params.phi).powf(params.m);
    params.cj0 / denom.max(1e-6) // prevent division by near-zero
}

/// Compute the transient companion model for the varactor at `v_prev`.
///
/// Returns `(g_eq, i_eq)` where:
/// - g_eq = C_j(v_prev) / dt  (backward Euler capacitor conductance)
/// - i_eq = -g_eq * v_prev    (capacitor history current — note sign convention)
///
/// In DC analysis, stamp as open circuit: return (0.0, 0.0).
/// Caller passes dt = 0.0 to signal DC, returning open circuit.
pub fn varactor_companion(v_prev: f64, dt: f64, params: &VaractorParams) -> (f64, f64) {
    if dt <= 0.0 {
        // DC analysis: varactor is open circuit
        return (0.0, 0.0);
    }
    let cj = junction_capacitance(v_prev, params);
    let g_eq = cj / dt;
    let i_eq = -g_eq * v_prev; // history current: I_eq = -C/dt * V_prev
    (g_eq, i_eq)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn varactor_dc_is_open_circuit() {
        let params = VaractorParams::default();
        let (g_eq, i_eq) = varactor_companion(0.0, 0.0, &params);
        assert_eq!(g_eq, 0.0);
        assert_eq!(i_eq, 0.0);
    }

    #[test]
    fn varactor_dc_is_open_circuit_at_nonzero_voltage() {
        let params = VaractorParams::default();
        let (g_eq, i_eq) = varactor_companion(-5.0, 0.0, &params);
        assert_eq!(g_eq, 0.0);
        assert_eq!(i_eq, 0.0);
    }

    #[test]
    fn varactor_capacitance_decreases_at_reverse_bias() {
        // At 0V bias, C_j = cj0. At reverse bias (V < 0), (1 - V/phi) > 1,
        // so denominator > 1, meaning C_j(V<0) < C_j(0) for the standard model.
        // The varactor capacitance increases toward 0V from reverse bias.
        let params = VaractorParams::default();
        let c_at_zero = junction_capacitance(0.0, &params);
        let c_at_reverse = junction_capacitance(-2.0, &params);
        // At 0V: denom = 1.0; at -2V: denom = (1 + 2/0.7)^0.5 > 1, so C(-2) < C(0)
        assert!(c_at_zero > c_at_reverse,
            "C(0V)={} should be > C(-2V)={}", c_at_zero, c_at_reverse);
    }

    #[test]
    fn varactor_transient_g_eq_is_cj_over_dt() {
        let params = VaractorParams::default();
        let v_prev = 0.0;
        let dt = 1e-6;
        let (g_eq, i_eq) = varactor_companion(v_prev, dt, &params);
        // At v=0: C_j = cj0 / 1.0 = cj0
        let expected_g = params.cj0 / dt;
        assert!((g_eq - expected_g).abs() < 1e-20,
            "g_eq={} expected {}", g_eq, expected_g);
        // i_eq = -g_eq * v_prev = 0 at v=0
        assert_eq!(i_eq, 0.0);
    }

    #[test]
    fn varactor_capacitance_clamped_near_phi() {
        let params = VaractorParams::default();
        // Forward bias approaching phi should clamp and return finite value > cj0
        let c_clamped = junction_capacitance(0.65, &params);
        assert!(c_clamped.is_finite(), "capacitance should be finite");
        assert!(c_clamped > params.cj0,
            "forward-biased C={} should be > cj0={}", c_clamped, params.cj0);
    }
}
