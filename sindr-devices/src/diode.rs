//! Diode model: Shockley equation, companion model, voltage limiting.
//!
//! Provides the mathematical foundation for Newton-Raphson simulation
//! of diodes and LEDs. Each NR iteration linearizes the diode at its
//! current operating point into a companion conductance + current source.

/// Boltzmann constant over the elementary charge, k/q (V/K).
pub const K_OVER_Q: f64 = 8.617_333_262e-5;

/// Nominal temperature (K) the device parameters are given at: 27 °C.
pub const T_NOM: f64 = 300.15;

/// Thermal voltage kT/q at the nominal temperature [`T_NOM`] (V).
pub const V_T: f64 = K_OVER_Q * T_NOM;

/// Thermal voltage kT/q (V) at temperature `t` (K).
pub fn thermal_voltage(t: f64) -> f64 {
    K_OVER_Q * t
}

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
    /// Thermal voltage kT/q (V) at the junction temperature.
    pub fn vt(&self) -> f64 {
        thermal_voltage(self.temperature)
    }

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
/// IS(T) = IS(T0) * (T/T0)^(XTI/N) * exp(Eg / (N * k/q) * (1/T0 - 1/T))
///
/// This is the SPICE formula. For a BJT, SPICE scales IS with N = 1.
///
/// Arguments:
/// - `is_t0`: IS at reference temperature T0 (A)
/// - `t`: Junction temperature (K)
/// - `t0`: Reference temperature (K), typically [`T_NOM`]
/// - `eg`: Bandgap energy (eV): 1.11 for Si, 0.69 for Ge, 1.42 for GaAs
/// - `xti`: Temperature exponent: SPICE's default is 3.0 for diodes and BJTs
/// - `n`: Emission coefficient (1.0 for a BJT)
pub fn temperature_scale_is(is_t0: f64, t: f64, t0: f64, eg: f64, xti: f64, n: f64) -> f64 {
    let exponent = xti / n * (t / t0).ln() + eg / (n * K_OVER_Q) * (1.0 / t0 - 1.0 / t);
    is_t0 * exponent.exp()
}

/// Compute the companion model (linearised) at operating point `v_d`.
///
/// Returns `(g_d, i_eq)` where:
/// - `g_d` is the companion conductance (dI/dV at v_d)
/// - `i_eq` is the companion current source (I_d - g_d * v_d)
///
/// These stamp into MNA exactly like a resistor (g_d) and current source (i_eq).
///
/// When `params.rs > 0`, `v_d` is the terminal voltage across junction and
/// series resistance together. The junction voltage is solved from
/// `I = IS * (exp((v_d - I*RS) / (N*V_T)) - 1)`, with V_T at
/// `params.temperature`, and the terminal conductance `g_j / (1 + RS*g_j)`
/// is returned, so `g_d * v_d + i_eq` is the terminal current at `v_d`.
pub fn diode_companion(v_d: f64, params: &DiodeParams) -> (f64, f64) {
    let nv_t = params.n * params.vt();
    let v_j = junction_voltage(v_d, params);
    let exp_vj = (v_j / nv_t).exp();

    // Diode current at junction voltage: I_d = IS * (exp(V_j / (N*V_T)) - 1)
    let i_d = params.is * (exp_vj - 1.0);

    // Junction conductance g_j = (IS / (N*V_T)) * exp(V_j / (N*V_T)), seen
    // through RS at the terminals: g_d = g_j / (1 + RS*g_j)
    let g_j = (params.is / nv_t) * exp_vj;
    let g_d = g_j / (1.0 + params.rs * g_j);

    // Companion current source: I_eq = I_d - g_d * V_d
    let i_eq = i_d - g_d * v_d;

    (g_d.max(GMIN), i_eq)
}

/// Compute the diode current at terminal voltage `v_d`, including the drop
/// across `params.rs`.
pub fn diode_current(v_d: f64, params: &DiodeParams) -> f64 {
    let nv_t = params.n * params.vt();
    params.is * ((junction_voltage(v_d, params) / nv_t).exp() - 1.0)
}

/// Junction voltage `v_j` for terminal voltage `v_d`: the root of
/// `f(v_j) = v_j + RS * IS * (exp(v_j / (N*V_T)) - 1) - v_d`.
///
/// `f` is increasing and convex, so Newton started above the root converges
/// monotonically down to it without overshooting. The start is an upper bound:
/// for `v_d >= 0` the current is at most `v_d / RS`, which bounds `v_j` by
/// `N*V_T * ln(1 + v_d / (RS*IS))` (and by `v_d`); for `v_d < 0` the current is
/// above `-IS`, so `v_j < v_d + RS*IS`.
fn junction_voltage(v_d: f64, params: &DiodeParams) -> f64 {
    let rs = params.rs;
    if rs <= 0.0 {
        return v_d;
    }
    let is = params.is;
    let nv_t = params.n * params.vt();

    let mut v_j = if v_d >= 0.0 {
        v_d.min(nv_t * (v_d / (rs * is)).ln_1p())
    } else {
        v_d + rs * is
    };
    for _ in 0..100 {
        let exp_vj = (v_j / nv_t).exp();
        let f = v_j + rs * is * (exp_vj - 1.0) - v_d;
        let df = 1.0 + rs * is / nv_t * exp_vj;
        let step = f / df;
        v_j -= step;
        if step.abs() <= 1e-15 * (1.0 + v_j.abs()) {
            break;
        }
    }
    v_j
}

/// Critical voltage for voltage limiting.
///
/// V_crit = N * V_T * ln(N * V_T / (sqrt(2) * IS)), with `vt` the thermal
/// voltage at the junction temperature.
pub fn critical_voltage(is: f64, n: f64, vt: f64) -> f64 {
    n * vt * (n * vt / (std::f64::consts::SQRT_2 * is)).ln()
}

/// SPICE-style voltage limiting to prevent NR overshoot.
///
/// If the proposed new voltage `v_new` exceeds V_crit above `v_old`,
/// the step is compressed logarithmically to keep the exponential
/// in the Shockley equation from overflowing. `vt` is the thermal voltage at
/// the junction temperature.
pub fn limit_voltage(v_new: f64, v_old: f64, v_crit: f64, n: f64, vt: f64) -> f64 {
    let nv_t = n * vt;

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
        let g_raw = params.is / (params.n * params.vt());
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
        let v_crit = critical_voltage(1e-14, 1.0, V_T);
        assert!(v_crit > 0.0);
        // For silicon diode, V_crit should be around 0.6-0.8V
        assert!(v_crit > 0.5);
        assert!(v_crit < 1.0);
    }

    #[test]
    fn limit_voltage_passes_small_steps() {
        let v_crit = critical_voltage(1e-14, 1.0, V_T);
        // Small step from 0.6 to 0.61: should pass through unchanged
        let limited = limit_voltage(0.61, 0.6, v_crit, 1.0, V_T);
        assert_relative_eq!(limited, 0.61, epsilon = 1e-15);
    }

    #[test]
    fn limit_voltage_clamps_large_steps() {
        let v_crit = critical_voltage(1e-14, 1.0, V_T);
        // Large step from 0.6 to 5.0: should be clamped
        let limited = limit_voltage(5.0, 0.6, v_crit, 1.0, V_T);
        assert!(limited < 5.0);
        assert!(limited > 0.6);
    }

    #[test]
    fn limit_voltage_negative_unchanged() {
        let v_crit = critical_voltage(1e-14, 1.0, V_T);
        // Negative voltages (reverse bias) should pass through
        let limited = limit_voltage(-1.0, -0.5, v_crit, 1.0, V_T);
        assert_relative_eq!(limited, -1.0, epsilon = 1e-15);
    }

    /// temperature_scale_is at reference temperature T0 should return is_t0 (ratio = 1).
    #[test]
    fn temperature_scale_is_at_reference_temp() {
        let is_t0 = 1e-14;
        let t0 = 300.15;
        let result = temperature_scale_is(is_t0, t0, t0, 1.11, 3.0, 1.0);
        // At T = T0, (T/T0)^xti = 1 and exp term = 0 → result = is_t0
        assert_relative_eq!(result, is_t0, max_relative = 1e-10);
    }

    /// IS at 350K must be greater than IS at 300K for silicon (positive temperature coefficient).
    #[test]
    fn temperature_scale_is_increases_with_temp() {
        let is_t0 = 1e-14;
        let t0 = 300.15;
        let is_300 = temperature_scale_is(is_t0, 300.15, t0, 1.11, 3.0, 1.0);
        let is_350 = temperature_scale_is(is_t0, 350.0, t0, 1.11, 3.0, 1.0);
        assert!(
            is_350 > is_300,
            "IS should increase with temperature: IS(350K)={} IS(300K)={}",
            is_350,
            is_300
        );
    }

    /// SPICE's IS(T) at 350 K from 1e-14 A at 300.15 K, XTI = 3, Eg = 1.11 eV:
    /// `(T/T0)^(XTI/N) * exp(Eg/(N*k/q) * (1/T0 - 1/T))`, worked out by hand.
    #[test]
    fn temperature_scale_is_matches_spice_formula() {
        let n1 = temperature_scale_is(1e-14, 350.0, 300.15, 1.11, 3.0, 1.0);
        assert_relative_eq!(n1, 7.157_360_875e-12, max_relative = 1e-9);
        let n2 = temperature_scale_is(1e-14, 350.0, 300.15, 1.11, 3.0, 2.0);
        assert_relative_eq!(n2, 2.675_324_443e-13, max_relative = 1e-9);
    }

    /// The thermal voltage follows the junction temperature.
    #[test]
    fn thermal_voltage_scales_with_temperature() {
        assert_relative_eq!(thermal_voltage(300.15), V_T, max_relative = 1e-15);
        let hot = DiodeParams {
            temperature: 400.0,
            ..DiodeParams::silicon()
        };
        assert_relative_eq!(hot.vt(), 0.034_469_333, max_relative = 1e-8);
    }

    /// rs=0.0 should give the same result as the original companion model (backward compatible).
    #[test]
    fn diode_companion_rs_zero_unchanged() {
        let params = DiodeParams::silicon(); // rs=0.0 by default
        let v_d = 0.65;
        let (g_d, i_eq) = diode_companion(v_d, &params);

        // Recompute expected values using the original formula (no Rs correction)
        let nv_t = params.n * params.vt();
        let exp_vd = (v_d / nv_t).exp();
        let expected_g_d = ((params.is / nv_t) * exp_vd).max(GMIN);
        let expected_i_d = params.is * (exp_vd - 1.0);
        let expected_i_eq = expected_i_d - expected_g_d * v_d;

        assert_relative_eq!(g_d, expected_g_d, epsilon = 1e-15);
        assert_relative_eq!(i_eq, expected_i_eq, epsilon = 1e-15);
    }

    /// Terminal current of a diode with series resistance, by bisection on
    /// `I = IS * (exp((v - I*RS) / (N*V_T)) - 1)`: independent of the Newton
    /// solve in `junction_voltage`.
    fn rs_current_by_bisection(v: f64, params: &DiodeParams) -> f64 {
        let nv_t = params.n * params.vt();
        let residual = |i: f64| params.is * (((v - i * params.rs) / nv_t).exp() - 1.0) - i;
        // The current lies between -IS and max(v, 0) / RS; the residual falls as I rises.
        let (mut lo, mut hi) = (-params.is, v.max(0.0) / params.rs + params.is);
        for _ in 0..400 {
            let mid = 0.5 * (lo + hi);
            if residual(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    const RS_BIASES: [f64; 9] = [-5.0, -0.5, -0.01, 0.0, 0.3, 0.65, 0.8, 2.0, 10.0];

    /// With rs > 0, the companion reproduces the implicit-solve current at the
    /// linearisation point: g_d * v + i_eq == I(v).
    #[test]
    fn diode_companion_rs_matches_implicit_current() {
        for rs in [0.01, 1.0, 10.0, 1e3] {
            let mut params = DiodeParams::silicon();
            params.rs = rs;
            for v in RS_BIASES {
                let expected = rs_current_by_bisection(v, &params);
                let (g_d, i_eq) = diode_companion(v, &params);
                let tol = 1e-9 * expected.abs() + 1e-20;
                assert!(
                    (g_d * v + i_eq - expected).abs() <= tol + GMIN * v.abs(),
                    "rs={rs} v={v}: companion current {} vs implicit {expected}",
                    g_d * v + i_eq
                );
                let i = diode_current(v, &params);
                assert!(
                    (i - expected).abs() <= tol,
                    "rs={rs} v={v}: diode_current {i} vs implicit {expected}"
                );
            }
        }
    }

    /// With rs > 0, the companion conductance is the terminal dI/dV, which
    /// equals g_j / (1 + RS*g_j); checked against a central finite difference
    /// of the implicit-solve current.
    #[test]
    fn diode_companion_rs_conductance_matches_finite_difference() {
        for rs in [0.01, 1.0, 10.0, 1e3] {
            let mut params = DiodeParams::silicon();
            params.rs = rs;
            for v in RS_BIASES {
                let h = 1e-6;
                let fd = (rs_current_by_bisection(v + h, &params)
                    - rs_current_by_bisection(v - h, &params))
                    / (2.0 * h);
                let (g_d, _) = diode_companion(v, &params);
                assert!(
                    (g_d - fd.max(GMIN)).abs() <= 1e-5 * fd.abs() + GMIN,
                    "rs={rs} v={v}: g_d {g_d} vs finite difference {fd}"
                );
                assert!(g_d <= 1.0 / rs, "rs={rs} v={v}: g_d {g_d} above 1/RS");
            }
        }
    }

    /// Forward bias with rs > 0 conducts: at 0.8 V through 10 Ω the diode is
    /// well on, with a terminal conductance close to (but below) 1/RS.
    #[test]
    fn diode_companion_rs_forward_bias_conducts() {
        let mut params = DiodeParams::silicon();
        params.rs = 10.0;
        let (g_d, _) = diode_companion(0.8, &params);
        let i = diode_current(0.8, &params);
        assert!(
            i > 1e-3,
            "diode should conduct at 0.8 V through 10 Ω, got {i} A"
        );
        assert!(
            g_d > 0.05 && g_d < 0.1,
            "g_d {g_d} should approach 1/RS = 0.1 S"
        );
    }
}
