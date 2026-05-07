//! Photoresistor (LDR) model: resistance as a function of illuminance.
//!
//! Uses logarithmic interpolation between dark and bright endpoints,
//! matching real LDR behaviour (resistance halves roughly every decade of lux).

/// Photoresistor parameters.
#[derive(Debug, Clone)]
pub struct PhotoresistorParams {
    /// Resistance in darkness (Ω). Default: 1 MΩ.
    pub r_dark: f64,
    /// Resistance at full illumination (Ω). Default: 1 kΩ.
    pub r_bright: f64,
}

impl Default for PhotoresistorParams {
    fn default() -> Self {
        Self { r_dark: 1_000_000.0, r_bright: 1_000.0 }
    }
}

/// Compute LDR resistance from a normalised light level.
///
/// `light_level` is clamped to `[0.0, 1.0]`:
/// - `0.0` → `params.r_dark`
/// - `1.0` → `params.r_bright`
///
/// Interpolation is logarithmic: `R = r_dark * (r_bright / r_dark) ^ light_level`
pub fn ldr_resistance(light_level: f64, params: &PhotoresistorParams) -> f64 {
    let t = light_level.clamp(0.0, 1.0);
    params.r_dark * (params.r_bright / params.r_dark).powf(t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn dark_returns_r_dark() {
        let p = PhotoresistorParams::default();
        assert_relative_eq!(ldr_resistance(0.0, &p), p.r_dark, epsilon = 1e-6);
    }

    #[test]
    fn bright_returns_r_bright() {
        let p = PhotoresistorParams::default();
        assert_relative_eq!(ldr_resistance(1.0, &p), p.r_bright, epsilon = 1e-6);
    }

    #[test]
    fn midpoint_is_geometric_mean() {
        let p = PhotoresistorParams::default();
        let r_mid = ldr_resistance(0.5, &p);
        let expected = (p.r_dark * p.r_bright).sqrt();
        assert_relative_eq!(r_mid, expected, max_relative = 1e-9);
    }

    #[test]
    fn clamps_above_one() {
        let p = PhotoresistorParams::default();
        assert_relative_eq!(ldr_resistance(2.0, &p), p.r_bright, epsilon = 1e-6);
    }

    #[test]
    fn clamps_below_zero() {
        let p = PhotoresistorParams::default();
        assert_relative_eq!(ldr_resistance(-1.0, &p), p.r_dark, epsilon = 1e-6);
    }
}
