//! Saturating inductor core: inductance that falls as the current rises.
//!
//! A ferrite or iron core holds its initial inductance `L0` at small
//! currents and falls towards an air-core floor `l_sat` once the flux
//! approaches saturation. The flux linkage is
//!
//! ```text
//! λ(i) = l_sat·i + (L0 − l_sat)·i_sat·tanh(i / i_sat)
//! L(i) = dλ/di = l_sat + (L0 − l_sat)·sech²(i / i_sat)
//! ```
//!
//! so L is `L0` at zero current, `l_sat + 0.42·(L0 − l_sat)` at `i_sat`
//! and `l_sat + 0.07·(L0 − l_sat)` at `2·i_sat`. The curve is
//! single-valued: no hysteresis, remanence or core loss. Put a resistor in
//! series or parallel for loss.
//!
//! A simulator integrates the flux (`v = dλ/dt`) and recovers the current
//! with [`current_from_flux`].

/// Saturation of an inductor whose small-signal inductance is `L0`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct InductorSaturation {
    /// Current (A) at which the core saturates: the knee of `tanh`. A
    /// datasheet's saturation current (the current at which L has fallen
    /// 20–30 %) is roughly `0.6·i_sat` here.
    pub i_sat: f64,
    /// Inductance (H) left once the core is fully saturated, positive and
    /// at most `L0`. An air-core winding keeps about `L0/μr`.
    pub l_sat: f64,
}

impl InductorSaturation {
    /// Saturation at `i_sat` (A), keeping a tenth of `l0` (H) when
    /// saturated.
    pub fn at(i_sat: f64, l0: f64) -> Self {
        Self {
            i_sat,
            l_sat: 0.1 * l0,
        }
    }
}

/// Flux linkage (Wb) of an inductor with small-signal inductance `l0` (H)
/// carrying `i` (A).
pub fn flux(i: f64, l0: f64, s: &InductorSaturation) -> f64 {
    s.l_sat * i + (l0 - s.l_sat) * s.i_sat * (i / s.i_sat).tanh()
}

/// Incremental inductance `dλ/di` (H) at current `i` (A).
pub fn incremental_inductance(i: f64, l0: f64, s: &InductorSaturation) -> f64 {
    let sech = 1.0 / (i / s.i_sat).cosh();
    s.l_sat + (l0 - s.l_sat) * sech * sech
}

/// The current (A) that carries flux `lambda` (Wb): the inverse of [`flux`].
///
/// λ(i) is odd and strictly increasing (its slope is at least `l_sat`), so
/// the root is unique. Newton's method from the saturated-line estimate,
/// kept inside a shrinking bracket, finds it to a relative 1e-12 in a few
/// iterations.
pub fn current_from_flux(lambda: f64, l0: f64, s: &InductorSaturation) -> f64 {
    if lambda == 0.0 {
        return 0.0;
    }
    let sign = lambda.signum();
    let target = lambda.abs();
    // λ(i) ≥ l_sat·i, and λ(i) ≥ L0·i for i up to where tanh bends; the
    // root lies between the two lines' answers.
    let (mut lo, mut hi) = (target / l0, target / s.l_sat);
    let mut i = lo.max((target - (l0 - s.l_sat) * s.i_sat) / s.l_sat);
    for _ in 0..100 {
        let f = flux(i, l0, s) - target;
        if f > 0.0 {
            hi = i;
        } else {
            lo = i;
        }
        let next = i - f / incremental_inductance(i, l0, s);
        let next = if next > lo && next < hi {
            next
        } else {
            0.5 * (lo + hi)
        };
        if (next - i).abs() <= 1e-12 * next.abs().max(f64::MIN_POSITIVE) {
            return sign * next;
        }
        i = next;
    }
    sign * i
}

#[cfg(test)]
mod tests {
    use super::*;

    const L0: f64 = 10e-3;

    fn sat() -> InductorSaturation {
        InductorSaturation {
            i_sat: 2.0,
            l_sat: 1e-3,
        }
    }

    #[test]
    fn small_currents_see_l0() {
        let s = sat();
        assert!((incremental_inductance(0.0, L0, &s) - L0).abs() < 1e-15);
        assert!((flux(1e-6, L0, &s) / 1e-6 - L0).abs() < 1e-12);
    }

    #[test]
    fn large_currents_see_l_sat() {
        let s = sat();
        assert!((incremental_inductance(40.0, L0, &s) - s.l_sat).abs() < 1e-12);
        // At i_sat: l_sat + (L0 − l_sat)·sech²(1).
        let sech1 = 1.0 / 1f64.cosh();
        let want = s.l_sat + (L0 - s.l_sat) * sech1 * sech1;
        assert!((incremental_inductance(2.0, L0, &s) - want).abs() < 1e-15);
    }

    #[test]
    fn flux_is_odd() {
        let s = sat();
        for i in [0.1, 1.0, 3.0, 50.0] {
            assert_eq!(flux(-i, L0, &s), -flux(i, L0, &s));
        }
    }

    #[test]
    fn current_from_flux_inverts_flux() {
        let s = sat();
        for i in [
            -100.0, -5.0, -2.0, -0.3, 0.0, 1e-9, 0.01, 1.0, 2.0, 2.5, 7.0, 1e3,
        ] {
            let back = current_from_flux(flux(i, L0, &s), L0, &s);
            assert!(
                (back - i).abs() <= 1e-10 * i.abs().max(1e-9),
                "i = {i}: got {back}"
            );
        }
    }

    #[test]
    fn hard_saturation_still_inverts() {
        let s = InductorSaturation {
            i_sat: 0.5,
            l_sat: 1e-9,
        };
        for i in [0.4, 0.6, 2.0, 30.0] {
            let back = current_from_flux(flux(i, L0, &s), L0, &s);
            assert!((back - i).abs() <= 1e-9 * i, "i = {i}: got {back}");
        }
    }
}
