//! Integration formulas for transient companion models.
//!
//! Every reactive element relates a state `q` to its rate `f = dq/dt`: a
//! capacitor's charge `C·v` and its current, an inductor's flux `L·i` and its
//! voltage. Each method discretises that, over a step of length `h`, as
//!
//! ```text
//!   f_n = a0·q_n + a1·q_{n−1} + a2·q_{n−2} − b1·f_{n−1}
//! ```
//!
//! | Method | a0 | a1 | a2 | b1 |
//! |---|---|---|---|---|
//! | Backward Euler | 1/h | −1/h | 0 | 0 |
//! | Trapezoidal | 2/h | −2/h | 0 | 1 |
//! | Gear-2 (BDF2), ω = h/h_prev | (1+2ω)/(h(1+ω)) | −(1+ω)/h | ω²/(h(1+ω)) | 0 |
//!
//! so an element becomes a conductance plus a current source fixed by its
//! [`History`].

/// Numerical integration method for transient analysis.
///
/// | Method | Order | Damping | Use it for |
/// |---|---|---|---|
/// | [`BackwardEuler`](Self::BackwardEuler) | 1 | Strong: oscillations decay that shouldn't | Robustness; matching older sindr results |
/// | [`Trapezoidal`](Self::Trapezoidal) | 2 | None: an LC tank keeps its amplitude | Resonant circuits. Can ring on time constants much shorter than the step |
/// | [`Gear2`](Self::Gear2) | 2 | Slight, and strong on fast time constants | The default: accurate and doesn't ring |
///
/// Every method takes its first step as backward Euler, since the
/// second-order ones need a history that doesn't exist at t = 0.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum IntegrationMethod {
    /// Backward Euler: first order, L-stable, numerically damped.
    BackwardEuler,
    /// Trapezoidal rule: second order, A-stable, no numerical damping.
    Trapezoidal,
    /// Gear-2 (second-order backward differentiation, BDF2): second order,
    /// L-stable. Handles step-size changes.
    #[default]
    Gear2,
}

/// Coefficients of one step's integration formula (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Integrator {
    a0: f64,
    a1: f64,
    a2: f64,
    b1: f64,
}

impl Integrator {
    /// Backward Euler over a step of `h`.
    pub(crate) fn backward_euler(h: f64) -> Self {
        Self {
            a0: 1.0 / h,
            a1: -1.0 / h,
            a2: 0.0,
            b1: 0.0,
        }
    }

    /// `method` over a step of `h`, the previous step having been `h_prev`
    /// (`None` on the first step, which is always backward Euler).
    pub(crate) fn for_step(method: IntegrationMethod, h: f64, h_prev: Option<f64>) -> Self {
        let Some(h_prev) = h_prev else {
            return Self::backward_euler(h);
        };
        match method {
            IntegrationMethod::BackwardEuler => Self::backward_euler(h),
            IntegrationMethod::Trapezoidal => Self {
                a0: 2.0 / h,
                a1: -2.0 / h,
                a2: 0.0,
                b1: 1.0,
            },
            IntegrationMethod::Gear2 => {
                let w = h / h_prev;
                Self {
                    a0: (1.0 + 2.0 * w) / (h * (1.0 + w)),
                    a1: -(1.0 + w) / h,
                    a2: w * w / (h * (1.0 + w)),
                    b1: 0.0,
                }
            }
        }
    }

    /// Companion of a capacitance `c` (F): `(g, i_hist)` with the current
    /// from the first terminal to the second `i = g·v + i_hist`.
    pub(crate) fn capacitor(&self, c: f64, h: &History) -> (f64, f64) {
        (
            c * self.a0,
            c * (self.a1 * h.x1 + self.a2 * h.x2) - self.b1 * h.y1,
        )
    }

    /// Companion of an inductance `l` (H): `(g, i_hist)` with the current
    /// from the first terminal to the second `i = g·v + i_hist`.
    pub(crate) fn inductor(&self, l: f64, h: &History) -> (f64, f64) {
        let z = l * self.a0;
        (
            1.0 / z,
            -(l * (self.a1 * h.x1 + self.a2 * h.x2) - self.b1 * h.y1) / z,
        )
    }

    /// Coupled windings with inductance matrix `l` (H) and histories `h`, in
    /// the branch form `v_k + Σ_j l_kj·a0·i_j = rhs_k`: returns `(l·a0, rhs)`.
    ///
    /// The branch rows use the convention `v = −L·di/dt` (the winding
    /// current is the MNA branch current), so `q = −L·i` and `f = v`.
    pub(crate) fn coupled(&self, l: [[f64; 2]; 2], h: [&History; 2]) -> ([[f64; 2]; 2], [f64; 2]) {
        let mut z = [[0.0; 2]; 2];
        let mut rhs = [0.0; 2];
        for k in 0..2 {
            for j in 0..2 {
                z[k][j] = l[k][j] * self.a0;
                rhs[k] -= l[k][j] * (self.a1 * h[j].x1 + self.a2 * h[j].x2);
            }
            rhs[k] -= self.b1 * h[k].y1;
        }
        (z, rhs)
    }
}

/// The last two states and the last rate of one reactive element: a
/// capacitor's voltages and current, or an inductor's currents and voltage.
/// All zero at t = 0: capacitors uncharged, inductors at rest.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct History {
    /// State one step back (V for a capacitor, A for an inductor).
    pub(crate) x1: f64,
    /// State two steps back.
    pub(crate) x2: f64,
    /// Rate one step back (A for a capacitor, V for an inductor).
    pub(crate) y1: f64,
}

impl History {
    /// The history after a step that ended at state `x` with rate `y`.
    pub(crate) fn push(self, x: f64, y: f64) -> Self {
        Self {
            x1: x,
            x2: self.x1,
            y1: y,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each formula is exact on the polynomials of its order: q = t for all,
    /// q = t² for the second-order ones (f = 2t).
    #[test]
    fn formulas_are_exact_to_their_order() {
        let (h_prev, h) = (0.3, 0.2);
        let (t2, t1, t0) = (0.0, h_prev, h_prev + h);
        let check = |method, q: fn(f64) -> f64, f: fn(f64) -> f64| {
            let i = Integrator::for_step(method, h, Some(h_prev));
            let got = i.a0 * q(t0) + i.a1 * q(t1) + i.a2 * q(t2) - i.b1 * f(t1);
            (got - f(t0)).abs()
        };
        for m in [
            IntegrationMethod::BackwardEuler,
            IntegrationMethod::Trapezoidal,
            IntegrationMethod::Gear2,
        ] {
            assert!(check(m, |t| t, |_| 1.0) < 1e-12, "{m:?} on q = t");
        }
        for m in [IntegrationMethod::Trapezoidal, IntegrationMethod::Gear2] {
            assert!(check(m, |t| t * t, |t| 2.0 * t) < 1e-12, "{m:?} on q = t²");
        }
    }

    /// Backward Euler reproduces the classic companions: a capacitor is C/h
    /// with history −(C/h)·v_prev, an inductor h/L with history i_prev.
    #[test]
    fn backward_euler_matches_the_classic_companions() {
        let be = Integrator::backward_euler(1e-3);
        let h = History {
            x1: 2.0,
            x2: 7.0,
            y1: 5.0,
        };
        let (g, i) = be.capacitor(1e-6, &h);
        assert!((g - 1e-3).abs() < 1e-18 && (i + 2e-3).abs() < 1e-15);
        let (g, i) = be.inductor(1e-2, &h);
        assert!((g - 0.1).abs() < 1e-15 && (i - 2.0).abs() < 1e-12);
    }

    #[test]
    fn first_step_is_backward_euler() {
        for m in [IntegrationMethod::Trapezoidal, IntegrationMethod::Gear2] {
            assert_eq!(
                Integrator::for_step(m, 0.1, None),
                Integrator::backward_euler(0.1)
            );
        }
    }
}
