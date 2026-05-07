//! NTC thermistor resistance model (Beta model).
//!
//! Thermistors have temperature-dependent resistance but are passive —
//! resistance is computed once from the temperature parameter, not from
//! circuit voltages. No NR iteration needed.

/// NTC thermistor parameters (Beta model).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct ThermistorParams {
    /// Resistance at reference temperature T0 (Ω).
    pub r0: f64,
    /// Beta coefficient (K). Typically 3000–5000 for NTC.
    pub beta: f64,
    /// Reference temperature (K). Standard: 298.15 K (25°C).
    pub t0: f64,
}

impl ThermistorParams {
    /// Standard 10 kΩ NTC thermistor (Epcos B57164K0103K000 or equivalent).
    /// R0 = 10kΩ at 25°C, Beta = 3950 K.
    pub fn ntc_10k() -> Self {
        Self { r0: 10_000.0, beta: 3950.0, t0: 298.15 }
    }
}

impl Default for ThermistorParams {
    fn default() -> Self { Self::ntc_10k() }
}

/// Compute thermistor resistance at a given temperature (Kelvin).
///
/// Uses the Beta model: R(T) = R0 * exp(beta * (1/T - 1/T0))
///
/// Does not panic; clamps T to a minimum of 1 K to avoid division by zero.
pub fn thermistor_resistance(temperature_k: f64, params: &ThermistorParams) -> f64 {
    let t = temperature_k.max(1.0);
    params.r0 * (params.beta * (1.0 / t - 1.0 / params.t0)).exp()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn ntc_at_reference_temperature_equals_r0() {
        let params = ThermistorParams::ntc_10k();
        let r = thermistor_resistance(298.15, &params);
        assert_relative_eq!(r, 10_000.0, max_relative = 1e-10);
    }

    #[test]
    fn ntc_increases_as_temperature_drops() {
        let params = ThermistorParams::ntc_10k();
        let r_25c = thermistor_resistance(298.15, &params);
        let r_10c = thermistor_resistance(283.15, &params);
        let r_50c = thermistor_resistance(323.15, &params);
        assert!(r_10c > r_25c, "NTC resistance should increase as temp drops");
        assert!(r_50c < r_25c, "NTC resistance should decrease as temp rises");
    }

    #[test]
    fn ntc_0c_reasonable_value() {
        let params = ThermistorParams::ntc_10k();
        let r_0c = thermistor_resistance(273.15, &params);
        // At 0°C, typical 10k NTC is ~27-33 kΩ
        assert!(r_0c > 20_000.0 && r_0c < 40_000.0, "R at 0°C = {} (expected 20-40 kΩ)", r_0c);
    }
}
