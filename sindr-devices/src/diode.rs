//! Diode model: Shockley equation, companion model, voltage limiting.
//!
//! Provides the mathematical foundation for Newton-Raphson simulation
//! of diodes and LEDs. Each NR iteration linearizes the diode at its
//! current operating point into a companion conductance + current source.

/// Thermal voltage at 300 K: kT/q
pub const V_T: f64 = 0.025851;

/// Minimum conductance shunt to prevent singular Jacobian in reverse bias.
pub const GMIN: f64 = 1e-12;

/// Diode parameters (Shockley model).
#[derive(Debug, Clone)]
pub struct DiodeParams {
    /// Saturation current (A).
    pub is: f64,
    /// Emission coefficient (dimensionless).
    pub n: f64,
    /// Series resistance (Ω). Default 0.0 — no effect on existing models.
    pub rs: f64,
    /// Junction temperature (K). Default 300.15 K = 27°C.
    pub temperature: f64,
}

impl DiodeParams {
    /// Standard silicon diode: IS = 1e-14 A, N = 1.0.
    pub fn silicon() -> Self {
        Self {
            is: 1e-14,
            n: 1.0,
            rs: 0.0,
            temperature: 300.15,
        }
    }

    /// LED with a given forward voltage at 20 mA.
    ///
    /// IS is derived from the Shockley equation: IS = 0.020 / (exp(Vf / (N * VT)) - 1)
    /// with N = 2.0 (typical LED emission coefficient).
    pub fn led(forward_voltage: f64) -> Self {
        let n = 2.0;
        let is = 0.020 / ((forward_voltage / (n * V_T)).exp() - 1.0);
        Self {
            is,
            n,
            rs: 0.0,
            temperature: 300.15,
        }
    }

    /// LED parameters by colour name.
    ///
    /// Maps common LED colours to their typical forward voltage.
    pub fn for_led_color(color: &str) -> Self {
        let vf = match color {
            "red" => 1.8,
            "green" => 2.2,
            "blue" => 3.2,
            "yellow" => 2.0,
            "white" => 3.0,
            _ => 1.8, // default to red
        };
        Self::led(vf)
    }
}

/// Scale saturation current IS for junction temperature.
///
/// IS(T) = IS(T0) * (T/T0)^XTI * exp(XTI * Eg / (k/q) * (1/T0 - 1/T))
///
/// This follows the SPICE formula (Sedra & Smith eq 4.11).
///
/// Arguments:
/// - `is_t0`: IS at reference temperature T0
/// - `t`: Junction temperature (K)
/// - `t0`: Reference temperature (K), typically 300.15
/// - `eg`: Bandgap energy (eV): 1.11 for Si, 0.69 for Ge, 1.42 for GaAs
/// - `xti`: Temperature exponent: 3.0 for BJT diodes, 2.0 for regular diodes
pub fn temperature_scale_is(is_t0: f64, t: f64, t0: f64, eg: f64, xti: f64) -> f64 {
    // k/q = V_T / 300.0 (approximately, since V_T = kT/q at 300K)
    let k_over_q = V_T / 300.0;
    (t / t0).powf(xti) * ((xti * eg / k_over_q) * (1.0 / t0 - 1.0 / t)).exp() * is_t0
}

/// Compute the companion model (linearised) at operating point `v_d`.
///
/// Returns `(g_d, i_eq)` where:
/// - `g_d` is the companion conductance (dI/dV at v_d)
/// - `i_eq` is the companion current source (I_d - g_d * v_d)
///
/// These stamp into MNA exactly like a resistor (g_d) and current source (i_eq).
///
/// When `params.rs > 0`, a single-step Newton correction accounts for the
/// voltage drop across the series resistance (v_j = v_d - i_d_prev * rs).
pub fn diode_companion(v_d: f64, params: &DiodeParams) -> (f64, f64) {
    let nv_t = params.n * V_T;

    // Series resistance correction: v_j = v_d - i_d_prev * rs
    // Use v_d as initial estimate for i_d_prev (single linearization step).
    let i_d_prev = params.is * ((v_d / nv_t).exp() - 1.0);
    let v_j = if params.rs > 0.0 {
        // Clamp to prevent extreme negative junction voltages
        (v_d - i_d_prev * params.rs).max(-5.0 * nv_t)
    } else {
        v_d
    };

    let exp_vj = (v_j / nv_t).exp();

    // Diode current at junction voltage: I_d = IS * (exp(V_j / (N*V_T)) - 1)
    let i_d = params.is * (exp_vj - 1.0);

    // Companion conductance: g_d = (IS / (N*V_T)) * exp(V_j / (N*V_T))
    let g_d = (params.is / nv_t) * exp_vj;

    // Companion current source: I_eq = I_d - g_d * V_d (use terminal voltage v_d)
    let i_eq = i_d - g_d * v_d;

    (g_d.max(GMIN), i_eq)
}

/// Compute the Shockley diode current at voltage `v_d`.
pub fn diode_current(v_d: f64, params: &DiodeParams) -> f64 {
    let nv_t = params.n * V_T;
    params.is * ((v_d / nv_t).exp() - 1.0)
}

/// Critical voltage for voltage limiting.
///
/// V_crit = N * V_T * ln(N * V_T / (sqrt(2) * IS))
pub fn critical_voltage(is: f64, n: f64) -> f64 {
    n * V_T * (n * V_T / (std::f64::consts::SQRT_2 * is)).ln()
}

/// SPICE-style voltage limiting to prevent NR overshoot.
///
/// If the proposed new voltage `v_new` exceeds V_crit above `v_old`,
/// the step is compressed logarithmically to keep the exponential
/// in the Shockley equation from overflowing.
pub fn limit_voltage(v_new: f64, v_old: f64, v_crit: f64, n: f64) -> f64 {
    let nv_t = n * V_T;

    if v_new > v_crit && (v_new - v_old).abs() > 2.0 * nv_t {
        if v_old > 0.0 {
            let arg = 1.0 + (v_new - v_old) / nv_t;
            if arg > 0.0 {
                v_old + nv_t * arg.ln()
            } else {
                v_crit
            }
        } else {
            v_crit
        }
    } else {
        v_new
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn silicon_params() {
        let p = DiodeParams::silicon();
        assert_eq!(p.is, 1e-14);
        assert_eq!(p.n, 1.0);
        assert_eq!(p.rs, 0.0);
        assert_eq!(p.temperature, 300.15);
    }

    #[test]
    fn led_color_mapping() {
        let red = DiodeParams::for_led_color("red");
        let green = DiodeParams::for_led_color("green");
        let blue = DiodeParams::for_led_color("blue");
        let yellow = DiodeParams::for_led_color("yellow");
        let white = DiodeParams::for_led_color("white");
        let unknown = DiodeParams::for_led_color("magenta");

        // All LEDs have N = 2.0
        assert_eq!(red.n, 2.0);
        assert_eq!(green.n, 2.0);
        assert_eq!(blue.n, 2.0);
        assert_eq!(yellow.n, 2.0);
        assert_eq!(white.n, 2.0);
        assert_eq!(unknown.n, 2.0);

        // Unknown defaults to red
        assert_eq!(unknown.is, red.is);

        // IS should be positive and decrease as Vf increases (higher Vf -> smaller IS)
        assert!(red.is > 0.0);
        assert!(green.is > 0.0);
        assert!(red.is > green.is); // lower Vf -> higher IS
        assert!(green.is > blue.is);
    }

    #[test]
    fn diode_companion_at_zero_voltage() {
        let params = DiodeParams::silicon();
        let (g_d, i_eq) = diode_companion(0.0, &params);

        // At V_d = 0: g_d_raw = IS/(N*VT) ≈ 3.87e-13, clamped to GMIN=1e-12
        // g_d should equal max(IS/(N*VT), GMIN)
        let g_raw = params.is / (params.n * V_T);
        let expected_g = g_raw.max(GMIN);
        assert_relative_eq!(g_d, expected_g, epsilon = 1e-20);

        // I_d = 0 at v=0, so i_eq = I_d - g_d * v_d = 0 - g_d * 0 = 0
        assert_relative_eq!(i_eq, 0.0, epsilon = 1e-20);
    }

    #[test]
    fn diode_companion_at_forward_bias() {
        let params = DiodeParams::silicon();
        let v_d = 0.65;
        let (g_d, i_eq) = diode_companion(v_d, &params);

        // g_d and current should be positive and substantial at 0.65V
        assert!(g_d > 0.0);
        assert!(g_d > 1e-3); // Should be in the mS range or higher

        // i_eq = I_d - g_d * V_d; for forward bias this is negative
        // (the companion current source opposes the conductance)
        let i_d = diode_current(v_d, &params);
        assert_relative_eq!(i_eq, i_d - g_d * v_d, epsilon = 1e-15);
    }

    #[test]
    fn diode_companion_at_reverse_bias() {
        let params = DiodeParams::silicon();
        let v_d = -1.0;
        let (g_d, _i_eq) = diode_companion(v_d, &params);

        // At reverse bias, conductance should be extremely small
        assert!(g_d > 0.0);
        assert!(g_d < 1e-10);
    }

    #[test]
    fn critical_voltage_is_positive() {
        let v_crit = critical_voltage(1e-14, 1.0);
        assert!(v_crit > 0.0);
        // For silicon diode, V_crit should be around 0.6-0.8V
        assert!(v_crit > 0.5);
        assert!(v_crit < 1.0);
    }

    #[test]
    fn limit_voltage_passes_small_steps() {
        let v_crit = critical_voltage(1e-14, 1.0);
        // Small step from 0.6 to 0.61: should pass through unchanged
        let limited = limit_voltage(0.61, 0.6, v_crit, 1.0);
        assert_relative_eq!(limited, 0.61, epsilon = 1e-15);
    }

    #[test]
    fn limit_voltage_clamps_large_steps() {
        let v_crit = critical_voltage(1e-14, 1.0);
        // Large step from 0.6 to 5.0: should be clamped
        let limited = limit_voltage(5.0, 0.6, v_crit, 1.0);
        assert!(limited < 5.0);
        assert!(limited > 0.6);
    }

    #[test]
    fn limit_voltage_negative_unchanged() {
        let v_crit = critical_voltage(1e-14, 1.0);
        // Negative voltages (reverse bias) should pass through
        let limited = limit_voltage(-1.0, -0.5, v_crit, 1.0);
        assert_relative_eq!(limited, -1.0, epsilon = 1e-15);
    }

    /// temperature_scale_is at reference temperature T0 should return is_t0 (ratio = 1).
    #[test]
    fn temperature_scale_is_at_reference_temp() {
        let is_t0 = 1e-14;
        let t0 = 300.15;
        let result = temperature_scale_is(is_t0, t0, t0, 1.11, 3.0);
        // At T = T0, (T/T0)^xti = 1 and exp term = 0 → result = is_t0
        assert_relative_eq!(result, is_t0, max_relative = 1e-10);
    }

    /// IS at 350K must be greater than IS at 300K for silicon (positive temperature coefficient).
    #[test]
    fn temperature_scale_is_increases_with_temp() {
        let is_t0 = 1e-14;
        let t0 = 300.15;
        let is_300 = temperature_scale_is(is_t0, 300.15, t0, 1.11, 3.0);
        let is_350 = temperature_scale_is(is_t0, 350.0, t0, 1.11, 3.0);
        assert!(
            is_350 > is_300,
            "IS should increase with temperature: IS(350K)={} IS(300K)={}",
            is_350,
            is_300
        );
    }

    /// rs=0.0 should give the same result as the original companion model (backward compatible).
    #[test]
    fn diode_companion_rs_zero_unchanged() {
        let params = DiodeParams::silicon(); // rs=0.0 by default
        let v_d = 0.65;
        let (g_d, i_eq) = diode_companion(v_d, &params);

        // Recompute expected values using the original formula (no Rs correction)
        let nv_t = params.n * V_T;
        let exp_vd = (v_d / nv_t).exp();
        let expected_g_d = ((params.is / nv_t) * exp_vd).max(GMIN);
        let expected_i_d = params.is * (exp_vd - 1.0);
        let expected_i_eq = expected_i_d - expected_g_d * v_d;

        assert_relative_eq!(g_d, expected_g_d, epsilon = 1e-15);
        assert_relative_eq!(i_eq, expected_i_eq, epsilon = 1e-15);
    }

    /// rs > 0 should reduce effective junction voltage and thus reduce g_d vs rs=0.
    #[test]
    fn diode_companion_rs_nonzero_reduces_current() {
        let v_d = 0.65;
        let params_no_rs = DiodeParams::silicon();
        let mut params_with_rs = DiodeParams::silicon();
        params_with_rs.rs = 10.0;

        let (g_d_no_rs, _) = diode_companion(v_d, &params_no_rs);
        let (g_d_with_rs, _) = diode_companion(v_d, &params_with_rs);

        assert!(
            g_d_with_rs < g_d_no_rs,
            "Rs>0 should reduce g_d: g_d_no_rs={} g_d_with_rs={}",
            g_d_no_rs,
            g_d_with_rs
        );
    }
}
