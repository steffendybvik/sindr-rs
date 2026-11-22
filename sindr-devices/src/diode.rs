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
    /// Junction temperature (K). Default 300.15 K = 27°C. Sets the thermal
    /// voltage [`DiodeParams::vt`]; `is` is used as given, so scale it with
    /// [`temperature_scale_is`] for a junction away from [`T_NOM`].
    pub temperature: f64,
    /// Reverse breakdown (SPICE `BV`/`IBV`). `None`: the junction never
    /// breaks down, as in SPICE without `BV`.
    pub breakdown: Option<Breakdown>,
}

/// Reverse breakdown of a diode, as a SPICE card's `BV` and `IBV` give it.
///
/// Breakdown adds `−IS·exp(−(v_j + xbv)/(N·V_T))` to the junction current,
/// with `xbv` fitted as ngspice fits it: the current at `v_j = −bv` is
/// `−ibv`, less about `IS·bv/V_T` that ngspice's fit carries. Away from the
/// knee this is the ngspice current to within `IS`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Breakdown {
    /// Breakdown voltage BV (V), positive.
    pub bv: f64,
    /// Current at `−bv`, IBV (A), positive. SPICE's default is 1 mA.
    pub ibv: f64,
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
            breakdown: None,
        }
    }

    /// LED with a given forward voltage (V) at 20 mA.
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
            breakdown: None,
        }
    }

    /// LED parameters by colour name, case ignored, as
    /// [`led::COLORS`](crate::led::COLORS) lists them. Unknown names are red.
    pub fn for_led_color(color: &str) -> Self {
        crate::led::led_params_from_str(color)
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
/// - `g_d` is the companion conductance (dI/dV at v_d), floored at [`GMIN`]
/// - `i_eq` is the companion current source (I_d - g_d * v_d)
///
/// `i_eq` is built from the same floored `g_d`, so `g_d * v_d + i_eq` is
/// always the diode current: the floor shapes the Newton steps in deep
/// reverse bias without adding a GMIN leakage to the converged current.
///
/// These stamp into MNA exactly like a resistor (g_d) and current source (i_eq).
///
/// When `params.rs > 0`, `v_d` is the terminal voltage across junction and
/// series resistance together. The junction voltage is solved from
/// `I = IS * (exp((v_d - I*RS) / (N*V_T)) - 1)`, with V_T at
/// `params.temperature`, and the terminal conductance `g_j / (1 + RS*g_j)`
/// is returned, so `g_d * v_d + i_eq` is the terminal current at `v_d`.
///
/// Units and signs: `v_d` is anode − cathode (V), `g_d` is in S and `i_eq`
/// in A, both for current flowing anode → cathode.
pub fn diode_companion(v_d: f64, params: &DiodeParams) -> (f64, f64) {
    let v_j = junction_voltage(v_d, params);
    let (i_d, g_j) = junction_current(v_j, params);

    // Junction conductance seen through RS at the terminals:
    // g_d = g_j / (1 + RS*g_j), floored at GMIN so the Jacobian never loses
    // the diode in deep reverse. Written as 1/(1/g_j + RS) so an overflowing
    // g_j gives 1/RS, not inf/inf. RS = 0 skips it so an overflowing g_j
    // stays infinite rather than turning into NaN that the floor would hide.
    let g_d = if params.rs > 0.0 {
        1.0 / (1.0 / g_j + params.rs)
    } else {
        g_j
    }
    .max(GMIN);

    // Companion current source: I_eq = I_d - g_d * V_d, from the same g_d
    // that is stamped, so g_d * V_d + I_eq is I_d and the floor only shapes
    // the Newton steps, not the converged current.
    let i_eq = i_d - g_d * v_d;

    (g_d, i_eq)
}

/// Compute the diode current (A, anode → cathode) at terminal voltage
/// `v_d` (V, anode − cathode), including the drop across `params.rs`.
pub fn diode_current(v_d: f64, params: &DiodeParams) -> f64 {
    junction_current(junction_voltage(v_d, params), params).0
}

/// Current (A) and conductance (S) of the junction itself at junction
/// voltage `v_j`: Shockley, plus the breakdown branch when `params` has one.
pub fn junction_current(v_j: f64, params: &DiodeParams) -> (f64, f64) {
    let nv_t = params.n * params.vt();
    let exp_vj = (v_j / nv_t).exp();
    let mut i = params.is * (exp_vj - 1.0);
    let mut g = params.is / nv_t * exp_vj;
    if let Some(xbv) = breakdown_knee(params) {
        let exp_bv = (-(v_j + xbv) / nv_t).exp();
        i -= params.is * exp_bv;
        g += params.is / nv_t * exp_bv;
    }
    (i, g)
}

/// `xbv`, the offset of the breakdown exponential, fitted so the current at
/// `−bv` is `−ibv`. This is ngspice's fit (`DIOtemp`), including its floor
/// on IBV, so knees match ngspice.
fn breakdown_knee(params: &DiodeParams) -> Option<f64> {
    let Breakdown { bv, ibv } = params.breakdown?;
    let vt = params.vt();
    let nv_t = params.n * vt;
    let is = params.is;
    if ibv < is * bv / vt {
        return Some(bv);
    }
    let mut xbv = bv - nv_t * (1.0 + ibv / is).ln();
    for _ in 0..25 {
        xbv = bv - nv_t * (ibv / is + 1.0 - xbv / vt).ln();
        let fit = is * (((bv - xbv) / nv_t).exp() - 1.0 + xbv / vt);
        if (ibv - fit).abs() <= 1e-3 * ibv {
            break;
        }
    }
    Some(xbv)
}

/// Junction voltage `v_j` for terminal voltage `v_d`: the root of
/// `f(v_j) = v_j + RS * I(v_j) - v_d`.
///
/// `f` is increasing and the root lies between 0 and `v_d` (the current has
/// the sign of `v_d`), so Newton is kept inside that bracket and falls back
/// to bisection when a step leaves it or the exponential overflows. Newton
/// starts from a bound on the root on the side it converges from without
/// overshooting: for `v_d >= 0` the current is at most `v_d / RS`, which
/// bounds `v_j` by `N*V_T * ln(1 + v_d / (RS*IS))`; for `v_d < 0`, the root
/// is below `v_d + RS*IS` and below 0, or, with breakdown, above where the
/// breakdown current alone reaches `|v_d| / RS`. A start on the far side of
/// an exponential makes Newton crawl one `N*V_T` per step, and past about
/// 700 `N*V_T` the exponential overflows.
fn junction_voltage(v_d: f64, params: &DiodeParams) -> f64 {
    let rs = params.rs;
    if rs <= 0.0 {
        return v_d;
    }
    let is = params.is;
    let nv_t = params.n * params.vt();
    let (mut lo, mut hi) = if v_d >= 0.0 { (0.0, v_d) } else { (v_d, 0.0) };

    let mut v_j = if v_d >= 0.0 {
        v_d.min(nv_t * (v_d / (rs * is)).ln_1p())
    } else if let Some(xbv) = breakdown_knee(params) {
        // In breakdown the current is at most |v_d| / RS too, which bounds
        // v_j from below; from there Newton climbs to the root without
        // overshooting into the exponential.
        v_d.max(-xbv - nv_t * (-v_d / (rs * is)).ln_1p())
    } else {
        (v_d + rs * is).min(0.0)
    };
    for _ in 0..200 {
        let (i, g) = junction_current(v_j, params);
        let f = v_j + rs * i - v_d;
        if f > 0.0 {
            hi = v_j;
        } else {
            lo = v_j;
        }
        let next = v_j - f / (1.0 + rs * g);
        let next = if next.is_finite() && next >= lo && next <= hi {
            next
        } else {
            0.5 * (lo + hi)
        };
        let step = next - v_j;
        v_j = next;
        if step.abs() <= 1e-15 * (1.0 + v_j.abs()) || hi - lo <= 1e-15 * (1.0 + v_j.abs()) {
            break;
        }
    }
    v_j
}

/// Charge-storage parameters of a junction, as a SPICE diode card's `CJO`,
/// `VJ`, `M`, `FC` and `TT` give them. The default stores no charge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JunctionCharge {
    /// Zero-bias depletion capacitance CJO (F). Default 0.
    pub cjo: f64,
    /// Junction potential VJ (V). Default 1.
    pub vj: f64,
    /// Grading coefficient M, in `[0, 1)`. Default 0.5.
    pub m: f64,
    /// Forward-bias depletion coefficient FC, in `[0, 1)`: above `FC·VJ` the
    /// depletion capacitance is continued linearly. Default 0.5.
    pub fc: f64,
    /// Transit time TT (s): the diffusion charge is `TT·I_d`. Default 0.
    pub tt: f64,
}

impl Default for JunctionCharge {
    fn default() -> Self {
        Self {
            cjo: 0.0,
            vj: 1.0,
            m: 0.5,
            fc: 0.5,
            tt: 0.0,
        }
    }
}

impl JunctionCharge {
    /// Whether the junction stores any charge at all.
    pub fn is_none(&self) -> bool {
        self.cjo == 0.0 && self.tt == 0.0
    }

    /// The slope dC/dv (F/V) of the depletion capacitance at junction
    /// voltage `v` (V): the diffusion part, `TT·g_d`, is the caller's.
    pub fn depletion_capacitance_slope(&self, v: f64) -> f64 {
        if self.cjo <= 0.0 {
            return 0.0;
        }
        let (vj, m, fc) = (self.vj, self.m, self.fc);
        if v < fc * vj {
            self.cjo * m / vj * (1.0 - v / vj).powf(-m - 1.0)
        } else {
            self.cjo / (1.0 - fc).powf(1.0 + m) * m / vj
        }
    }

    /// Stored charge (C) and capacitance dQ/dV (F) at junction voltage `v`
    /// (V), carrying current `i_d` (A) with conductance `g_d` (S).
    ///
    /// The SPICE junction charge: depletion `CJO·VJ·(1 − (1 − v/VJ)^(1−M))/(1−M)`
    /// up to `FC·VJ`, continued with a linear capacitance above it so it
    /// stays finite in forward bias, plus diffusion `TT·I_d`.
    pub fn charge(&self, v: f64, i_d: f64, g_d: f64) -> (f64, f64) {
        let (mut q, mut c) = (self.tt * i_d, self.tt * g_d);
        if self.cjo > 0.0 {
            let (vj, m, fc) = (self.vj, self.m, self.fc);
            let v_fc = fc * vj;
            if v < v_fc {
                let arg = 1.0 - v / vj;
                let sarg = arg.powf(-m);
                q += self.cjo * vj * (1.0 - arg * sarg) / (1.0 - m);
                c += self.cjo * sarg;
            } else {
                let f1 = vj * (1.0 - (1.0 - fc).powf(1.0 - m)) / (1.0 - m);
                let f2 = (1.0 - fc).powf(1.0 + m);
                let f3 = 1.0 - fc * (1.0 + m);
                q += self.cjo
                    * (f1 + (f3 * (v - v_fc) + m / (2.0 * vj) * (v * v - v_fc * v_fc)) / f2);
                c += self.cjo / f2 * (f3 + m * v / vj);
            }
        }
        (q, c)
    }
}

/// The junction behind RS at terminal voltage `v_d` (V): its conductance
/// `g_j` (S) and capacitance `c_j` (F). In AC the diode is RS in series with
/// `g_j + jωC_j`.
pub fn junction_small_signal(
    v_d: f64,
    params: &DiodeParams,
    charge: &JunctionCharge,
) -> (f64, f64) {
    let v_j = junction_voltage(v_d, params);
    let (i_d, g_j) = junction_current(v_j, params);
    (g_j, charge.charge(v_j, i_d, g_j).1)
}

/// Stored charge (C) and its derivative with respect to the terminal
/// voltage (F) of a diode at terminal voltage `v_d` (V, anode − cathode).
///
/// The charge is the junction's at the junction voltage the diode current
/// sets across RS. The drop the charging current itself makes across RS is
/// left out, so with RS > 0 the charge sits across the terminals.
pub fn diode_charge(v_d: f64, params: &DiodeParams, charge: &JunctionCharge) -> (f64, f64) {
    let v_j = junction_voltage(v_d, params);
    let (i_d, g_j) = junction_current(v_j, params);
    let (q, c_j) = charge.charge(v_j, i_d, g_j);
    // dv_j/dv_d through RS; an overflowing g_j means v_j is pinned.
    let dvj = if params.rs > 0.0 {
        1.0 / (1.0 + params.rs * g_j)
    } else {
        1.0
    };
    (q, if dvj.is_finite() { c_j * dvj } else { 0.0 })
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

/// The derivatives of a diode's operating point with respect to one of its
/// parameters, at a fixed terminal voltage: what a sensitivity analysis
/// needs of the device.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct DiodeDerivative {
    /// ∂I/∂p: terminal current (A, anode → cathode) per unit of the
    /// parameter.
    pub current: f64,
    /// ∂v_j/∂p: voltage across the junction behind RS (V) per unit of the
    /// parameter.
    pub junction_voltage: f64,
    /// ∂g_j/∂p: the junction's conductance (S) per unit of the parameter.
    pub junction_conductance: f64,
}

/// [`DiodeDerivative`]s with respect to each parameter of [`DiodeParams`]
/// that shapes the DC curve, from [`diode_derivatives`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct DiodeDerivatives {
    /// With respect to `is` (per A), `n` held.
    pub is: DiodeDerivative,
    /// With respect to `n`, `is` held.
    pub n: DiodeDerivative,
    /// With respect to `rs` (per Ω).
    pub rs: DiodeDerivative,
}

/// The exact derivatives of the diode at terminal voltage `v_d` (V, anode −
/// cathode) with respect to `params.is`, `params.n` and `params.rs`, the
/// breakdown branch included: the junction voltage moves with the parameter
/// so that `v_j + RS·I = v_d` still holds.
///
/// `is` and `n` are taken as they are in `params`; a caller whose IS follows
/// temperature chains through its own scaling.
pub fn diode_derivatives(v_d: f64, params: &DiodeParams) -> DiodeDerivatives {
    let v_j = junction_voltage(v_d, params);
    let (i, g) = junction_current(v_j, params);
    let (is, n, rs) = (params.is, params.n, params.rs);
    let nv_t = n * params.vt();

    // Partials at fixed v_j: (∂I/∂is, ∂g/∂is), (∂I/∂n, ∂g/∂n), and ∂g/∂v_j.
    let e = (v_j / nv_t).exp();
    let g_f = is * e / nv_t;
    let mut d_is = (e - 1.0, e / nv_t);
    let mut d_n = (-g_f * v_j / n, g_f * (-v_j / (n * nv_t) - 1.0 / n));
    let mut dg_dvj = g_f / nv_t;
    if let Some(xbv) = breakdown_knee(params) {
        let eb = (-(v_j + xbv) / nv_t).exp();
        let g_b = is * eb / nv_t;
        // The knee moves with IS and N where it is fitted to IBV.
        let (dx_dis, dx_dn) = match params.breakdown {
            Some(Breakdown { bv, ibv }) if ibv >= is * bv / params.vt() => {
                let ep = ((bv - xbv) / nv_t).exp();
                let g_x = is * (-ep / nv_t + 1.0 / params.vt());
                let g_is = ep - 1.0 + xbv / params.vt();
                let g_n = -is * ep * (bv - xbv) / (n * nv_t);
                (-g_is / g_x, -g_n / g_x)
            }
            _ => (0.0, 0.0),
        };
        // I_b = −is·eb and g_b = is·eb/(n·V_T), each through xbv as well.
        d_is.0 += -eb + g_b * dx_dis;
        d_is.1 += eb / nv_t - g_b / nv_t * dx_dis;
        d_n.0 += -g_b * (v_j + xbv) / n + g_b * dx_dn;
        d_n.1 += g_b * ((v_j + xbv) / (n * nv_t) - 1.0 / n) - g_b / nv_t * dx_dn;
        dg_dvj -= g_b / nv_t;
    }

    // With v_j = v_d − RS·I: dI·(1 + RS·g) = ∂I/∂p − g·I·∂RS/∂p.
    let denominator = 1.0 + rs * g;
    let total = |(di, dg): (f64, f64), d_rs: f64| {
        let current = (di - g * i * d_rs) / denominator;
        let junction_voltage = -rs * current - i * d_rs;
        DiodeDerivative {
            current,
            junction_voltage,
            junction_conductance: dg + dg_dvj * junction_voltage,
        }
    };
    DiodeDerivatives {
        is: total(d_is, 0.0),
        n: total(d_n, 0.0),
        rs: total((0.0, 0.0), 1.0),
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

    /// The linearisation reproduces the diode current at its own operating
    /// point, deep reverse bias included, where `g_d` sits on the GMIN floor.
    #[test]
    fn diode_companion_reproduces_current_at_operating_point() {
        let mut with_rs = DiodeParams::silicon();
        with_rs.rs = 10.0;
        for params in [DiodeParams::silicon(), with_rs] {
            for v_d in [-50.0, -5.0, -0.5, 0.0, 0.3, 0.65, 0.8] {
                let (g_d, i_eq) = diode_companion(v_d, &params);
                let i_d = diode_current(v_d, &params);
                assert!(
                    (g_d * v_d + i_eq - i_d).abs() <= 1e-12 * i_d.abs() + 1e-24,
                    "rs = {}, v = {v_d}: g·v + i_eq = {}, I_d = {i_d}",
                    params.rs,
                    g_d * v_d + i_eq
                );
            }
        }
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

    /// When RS*IS is many N*V_T, reverse bias still solves exactly. Starting
    /// Newton at `v_d + RS*IS` used to stall at the iteration cap and, past
    /// about 18 V of RS*IS, overflow to NaN.
    #[test]
    fn diode_rs_reverse_bias_with_large_rs_is() {
        for (is, rs) in [(1e-6, 1e7), (1e-6, 1e8), (1e-3, 1e5)] {
            let mut params = DiodeParams::silicon();
            params.is = is;
            params.rs = rs;
            for v in [-0.01, -0.5, -5.0, -50.0] {
                let expected = rs_current_by_bisection(v, &params);
                let i = diode_current(v, &params);
                assert!(
                    (i - expected).abs() <= 1e-9 * expected.abs() + 1e-20,
                    "is={is} rs={rs} v={v}: diode_current {i} vs implicit {expected}"
                );
                let (g_d, i_eq) = diode_companion(v, &params);
                assert!(g_d.is_finite() && i_eq.is_finite(), "is={is} rs={rs} v={v}");
            }
        }
    }

    /// With RS = 0, an overflowing junction conductance stays infinite instead
    /// of turning into NaN and being floored to GMIN.
    #[test]
    fn diode_companion_rs_zero_overflow_is_not_floored() {
        let params = DiodeParams::silicon();
        let (g_d, _) = diode_companion(100.0, &params);
        assert!(g_d > 1.0, "g_d = {g_d}");
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

    fn card_1n4148() -> (DiodeParams, JunctionCharge) {
        (
            DiodeParams {
                is: 2.52e-9,
                n: 1.752,
                rs: 0.568,
                temperature: T_NOM,
                breakdown: Some(Breakdown {
                    bv: 100.0,
                    ibv: 100e-6,
                }),
            },
            JunctionCharge {
                cjo: 4e-12,
                m: 0.4,
                tt: 20e-9,
                ..JunctionCharge::default()
            },
        )
    }

    #[test]
    fn breakdown_passes_ibv_at_bv() {
        let (mut p, _) = card_1n4148();
        p.rs = 0.0;
        let i = diode_current(-100.0, &p);
        // ngspice's fit carries an extra `IS·xbv/V_T` (about 10 µA here), so
        // at −BV the current is IBV less that, as the dc_diode_breakdown
        // golden confirms.
        let want = -(100e-6 - p.is * 100.0 / p.vt());
        assert_relative_eq!(i, want, max_relative = 2e-3);
        // Well short of breakdown the reverse current is IS.
        assert_relative_eq!(diode_current(-50.0, &p), -p.is, max_relative = 1e-6);
    }

    #[test]
    fn junction_voltage_solves_through_rs_in_breakdown() {
        let (p, _) = card_1n4148();
        for v_d in [-150.0, -100.5, -50.0, -0.3, 0.0, 0.4, 0.8, 1.5] {
            let v_j = junction_voltage(v_d, &p);
            let (i, _) = junction_current(v_j, &p);
            assert!(
                (v_j + p.rs * i - v_d).abs() <= 1e-9 * (1.0 + v_d.abs()),
                "v_d {v_d}: v_j {v_j}, i {i}"
            );
        }
    }

    #[test]
    fn companion_slope_matches_the_current_in_breakdown() {
        let (p, _) = card_1n4148();
        for v_d in [-101.0, -100.2, -99.9] {
            let (g, _) = diode_companion(v_d, &p);
            let h = 1e-6;
            let fd = (diode_current(v_d + h, &p) - diode_current(v_d - h, &p)) / (2.0 * h);
            assert_relative_eq!(g, fd, max_relative = 1e-5);
        }
    }

    #[test]
    fn charge_is_continuous_at_fc_and_its_slope_is_the_capacitance() {
        let (p, charge) = card_1n4148();
        let v_fc = charge.fc * charge.vj;
        let below = charge.charge(v_fc - 1e-12, 0.0, 0.0);
        let above = charge.charge(v_fc + 1e-12, 0.0, 0.0);
        assert_relative_eq!(below.0, above.0, max_relative = 1e-9);
        assert_relative_eq!(below.1, above.1, max_relative = 1e-9);
        // Zero bias stores no charge and has CJO.
        assert_eq!(charge.charge(0.0, 0.0, 0.0), (0.0, charge.cjo));

        for v_d in [-20.0, -1.0, 0.0, 0.3, 0.5, 0.6, 0.7, 0.9] {
            let (_, c) = diode_charge(v_d, &p, &charge);
            let h = 1e-6;
            let fd = (diode_charge(v_d + h, &p, &charge).0 - diode_charge(v_d - h, &p, &charge).0)
                / (2.0 * h);
            assert_relative_eq!(c, fd, max_relative = 1e-5);
        }
    }

    #[test]
    fn reverse_capacitance_follows_the_grading() {
        let (p, charge) = card_1n4148();
        let (g_j, c_j) = junction_small_signal(-5.0, &p, &charge);
        assert!(g_j < 1e-9);
        // CJO / (1 + 5/VJ)^M, the diffusion part being negligible.
        assert_relative_eq!(c_j, 4e-12 / 6.0_f64.powf(0.4), max_relative = 1e-6);
    }

    #[test]
    fn a_junction_stores_charge_with_either_depletion_or_diffusion() {
        assert!(JunctionCharge::default().is_none());
        let depletion = JunctionCharge {
            cjo: 1e-12,
            ..JunctionCharge::default()
        };
        assert!(!depletion.is_none());
        let diffusion = JunctionCharge {
            tt: 5e-9,
            ..JunctionCharge::default()
        };
        assert!(!diffusion.is_none());
        // Diffusion only: Q = TT·I_d and C = TT·g_d, exactly.
        assert_eq!(
            diffusion.charge(0.65, 2e-3, 0.08),
            (5e-9 * 2e-3, 5e-9 * 0.08)
        );
    }

    /// With no series resistance the terminals are the junction: the
    /// diode's dQ/dV is the junction capacitance itself.
    #[test]
    fn without_rs_the_terminal_capacitance_is_the_junction_capacitance() {
        let (p, charge) = card_1n4148();
        let p = DiodeParams { rs: 0.0, ..p };
        for v_d in [-5.0, 0.0, 0.6] {
            let (i_d, g_d) = junction_current(v_d, &p);
            let (q, c) = diode_charge(v_d, &p, &charge);
            assert_eq!((q, c), charge.charge(v_d, i_d, g_d), "v_d = {v_d}");
        }
    }

    /// As documented on [`Breakdown`]: the current at −BV is −IBV less the
    /// `IS·BV/V_T` ngspice's fit carries, to the fit's 0.1 %. Taking
    /// `xbv ≈ bv` in that term costs `IS·(bv − xbv)/V_T`, under 3e-4 of
    /// IBV for these cards.
    #[test]
    fn breakdown_carries_ibv_at_minus_bv() {
        for (bv, ibv, is) in [
            (100.0, 100e-6, 2.52e-9),
            (5.1, 5e-3, 1e-14),
            (5.1, 1.0, 1e-14),
            (30.0, 1e-9, 1e-14),
        ] {
            let p = DiodeParams {
                is,
                breakdown: Some(Breakdown { bv, ibv }),
                ..DiodeParams::silicon()
            };
            let i = diode_current(-bv, &p);
            assert_relative_eq!(-i, ibv - is * bv / p.vt(), max_relative = 1.5e-3);
        }
    }

    /// Whether `want` is the derivative of `f` in one parameter of `params`
    /// at `x`: the central difference's change matches `want`'s to 1e-5, or
    /// to a few ulps of `f` where the change is below its resolution.
    fn matches_central_difference(
        want: f64,
        params: &DiodeParams,
        set: fn(&mut DiodeParams, f64),
        x: f64,
        f: impl Fn(&DiodeParams) -> f64,
    ) -> bool {
        let h = x.abs() * 1e-6;
        let at = |x: f64| {
            let mut p = params.clone();
            set(&mut p, x);
            f(&p)
        };
        let (hi, lo) = (at(x + h), at(x - h));
        let (got, expected) = (hi - lo, want * 2.0 * h);
        (got - expected).abs() <= 1e-5 * expected.abs() + 8.0 * f64::EPSILON * hi.abs()
    }

    #[test]
    fn diode_derivatives_match_central_differences() {
        let forward = DiodeParams {
            is: 2e-14,
            n: 1.7,
            rs: 15.0,
            temperature: 320.0,
            breakdown: None,
        };
        let breakdown = DiodeParams {
            rs: 2.0,
            breakdown: Some(Breakdown { bv: 5.1, ibv: 1e-3 }),
            ..forward.clone()
        };
        for (params, v_d) in [
            (&forward, 0.75),
            (&forward, -0.3),
            (&breakdown, -5.3),
            (&breakdown, 0.7),
        ] {
            let d = diode_derivatives(v_d, params);
            let v_j = |p: &DiodeParams| junction_voltage(v_d, p);
            let g_j = |p: &DiodeParams| junction_current(junction_voltage(v_d, p), p).1;
            type Set = fn(&mut DiodeParams, f64);
            let cases: [(&str, DiodeDerivative, Set, f64); 3] = [
                ("is", d.is, |p, x| p.is = x, params.is),
                ("n", d.n, |p, x| p.n = x, params.n),
                ("rs", d.rs, |p, x| p.rs = x, params.rs),
            ];
            for (name, want, set, x) in cases {
                let current = |p: &DiodeParams| diode_current(v_d, p);
                assert!(
                    matches_central_difference(want.current, params, set, x, current),
                    "I by {name} at {v_d} V"
                );
                assert!(
                    matches_central_difference(want.junction_voltage, params, set, x, v_j),
                    "v_j by {name} at {v_d} V"
                );
                assert!(
                    matches_central_difference(want.junction_conductance, params, set, x, g_j),
                    "g_j by {name} at {v_d} V"
                );
            }
        }
    }

    #[test]
    fn depletion_capacitance_slope_matches_central_differences() {
        let charge = JunctionCharge {
            cjo: 2e-12,
            vj: 0.8,
            m: 0.4,
            fc: 0.5,
            tt: 0.0,
        };
        for v in [-2.0, 0.1, 0.6] {
            let h = 1e-6;
            let c = |v: f64| charge.charge(v, 0.0, 0.0).1;
            let fd = (c(v + h) - c(v - h)) / (2.0 * h);
            assert_relative_eq!(
                charge.depletion_capacitance_slope(v),
                fd,
                max_relative = 1e-6
            );
        }
    }
}
