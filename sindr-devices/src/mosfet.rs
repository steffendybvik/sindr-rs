//! MOSFET Level 1 (Shichman-Hodges) model with body effect.
//!
//! Implements NMOS and PMOS transistors with three operating regions:
//! cutoff, triode (linear), and saturation. The companion model linearizes
//! the device at each NR iteration for MNA stamping.
//!
//! `sindr` treats every MOSFET as a three-terminal part with its body tied
//! to its source (Vbs = 0), so the body effect never applies there; the
//! model keeps it for callers that drive the body separately.

use crate::datasheet::{positive, DatasheetError, MosfetDatasheet};

/// MOSFET type (NMOS or PMOS).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MosfetKind {
    /// N-channel MOSFET (electrons are majority carriers in the channel).
    #[cfg_attr(feature = "serde", serde(rename = "nmos"))]
    Nmos,
    /// P-channel MOSFET (holes are majority carriers in the channel).
    #[cfg_attr(feature = "serde", serde(rename = "pmos"))]
    Pmos,
}

impl MosfetKind {
    /// Gate–source and drain–source voltages `(vgs, vds)` of a three-terminal
    /// MOSFET with node voltages `vg`, `vd`, `vs`, in the N-channel frame that
    /// [`mosfet_companion`] works in: mirrored (`vsg`, `vsd`) for a PMOS.
    ///
    /// A three-terminal MOSFET is a discrete part, whose body is tied to its
    /// source inside the package, so its `vbs` is always 0.
    pub fn nmos_frame(self, vg: f64, vd: f64, vs: f64) -> (f64, f64) {
        match self {
            MosfetKind::Nmos => (vg - vs, vd - vs),
            MosfetKind::Pmos => (vs - vg, vs - vd),
        }
    }
}

/// Operating region of a MOSFET.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MosfetRegion {
    /// Below threshold — channel off, negligible drain current.
    Cutoff,
    /// Triode (linear) region — `Vds` below saturation, device acts as a voltage-controlled resistor.
    Triode,
    /// Saturation region — channel pinched off, drain current set by `Vgs`.
    Saturation,
}

impl MosfetRegion {
    /// Lowercase string label for the region (`"cutoff"`, `"triode"`, `"saturation"`).
    pub fn as_str(&self) -> &'static str {
        match self {
            MosfetRegion::Cutoff => "cutoff",
            MosfetRegion::Triode => "triode",
            MosfetRegion::Saturation => "saturation",
        }
    }
}

/// MOSFET device parameters (Level 1 Shichman-Hodges).
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct MosfetParams {
    /// Transconductance parameter (A/V^2). SPICE KP parameter.
    pub kp: f64,
    /// Zero-bias threshold voltage (V). SPICE VTO parameter, signed as in
    /// SPICE: positive for an enhancement NMOS, negative for an enhancement
    /// PMOS (see [`MosfetParams::in_nmos_frame`]).
    pub vto: f64,
    /// Channel-length modulation (1/V). SPICE LAMBDA parameter.
    pub lambda: f64,
    /// Body effect coefficient (V^0.5). SPICE GAMMA parameter.
    pub gamma: f64,
    /// Surface potential (V). SPICE PHI parameter.
    pub phi: f64,
    /// Flicker-noise coefficient. Noise analysis puts `KF·|Id|^AF / f`
    /// (A²/Hz) across drain–source. sindr has no channel geometry, so this
    /// is SPICE's `KF/(Cox·L²)`. Default 0: no flicker noise.
    #[cfg_attr(feature = "serde", serde(default))]
    pub kf: f64,
    /// Flicker-noise current exponent AF. Default 1.
    #[cfg_attr(feature = "serde", serde(default = "default_af"))]
    pub af: f64,
}

#[cfg(feature = "serde")]
fn default_af() -> f64 {
    1.0
}

impl MosfetParams {
    /// Default NMOS parameters (typical educational values).
    pub fn default_nmos() -> Self {
        Self {
            kp: 2e-4,     // 200 uA/V^2
            vto: 0.7,     // 0.7V threshold
            lambda: 0.02, // 0.02 /V
            gamma: 0.4,   // 0.4 V^0.5
            phi: 0.6,     // 0.6V
            kf: 0.0,
            af: 1.0,
        }
    }

    /// Default PMOS parameters (typical educational values).
    pub fn default_pmos() -> Self {
        Self {
            kp: 1e-4,  // 100 uA/V^2 (typically half of NMOS)
            vto: -0.7, // -0.7V threshold (negative for PMOS)
            lambda: 0.02,
            gamma: 0.4,
            phi: 0.6,
            kf: 0.0,
            af: 1.0,
        }
    }
}

impl MosfetParams {
    /// These parameters in the N-channel frame that [`mosfet_companion`]
    /// works in. Callers mirror a PMOS's voltages (`vsg`, `vsd`, …) before
    /// calling it; this mirrors the threshold to match, so a SPICE-signed
    /// VTO (−0.7 V for [`MosfetParams::default_pmos`]) becomes the +0.7 V
    /// the N-channel equations expect.
    pub fn in_nmos_frame(&self, kind: MosfetKind) -> MosfetParams {
        match kind {
            MosfetKind::Nmos => self.clone(),
            MosfetKind::Pmos => MosfetParams {
                vto: -self.vto,
                ..self.clone()
            },
        }
    }
}

impl MosfetParams {
    /// Level-1 parameters fitted to a datasheet's threshold and
    /// on-resistance: KP and VTO are solved so that the drain current at
    /// `Vgs = Vds = vth` is `vth_id`, and the drain–source resistance at
    /// `Vgs = rds_on_vgs` is `rds_on` (at `rds_on_id` when given, else at a
    /// vanishing drain current). LAMBDA, GAMMA and PHI keep the
    /// [`default_nmos`](Self::default_nmos) / [`default_pmos`](Self::default_pmos)
    /// values; VTO is signed for `kind`, as SPICE signs it.
    ///
    /// ```
    /// use sindr_devices::datasheet::MosfetDatasheet;
    /// use sindr_devices::mosfet::{MosfetKind, MosfetParams};
    ///
    /// // A small-signal NMOS: Vgs(th) 2.1 V at 250 µA, 1.2 Ω at 10 V.
    /// let ds = MosfetDatasheet::new(2.1, 250e-6, 1.2, 10.0);
    /// let p = MosfetParams::from_datasheet(MosfetKind::Nmos, &ds).unwrap();
    /// assert!(p.vto > 0.0 && p.vto < 2.1);
    /// ```
    ///
    /// # Errors
    ///
    /// [`DatasheetError`] when a figure is missing or not positive, when
    /// `rds_on_vgs` isn't above the threshold, or when no Level-1 device
    /// meets both figures (an `rds_on_id` so high that the part leaves the
    /// triode region).
    pub fn from_datasheet(kind: MosfetKind, ds: &MosfetDatasheet) -> Result<Self, DatasheetError> {
        let vth = ds.vth.require("vth")?.abs();
        let id_th = positive("vth_id", ds.vth_id.abs())?;
        let rds = positive("rds_on", ds.rds_on.require("rds_on")?.abs())?;
        let vgs_on = positive("rds_on_vgs", ds.rds_on_vgs.abs())?;
        let id_on = ds
            .rds_on_id
            .map(|i| positive("rds_on_id", i.abs()))
            .transpose()?;
        let d = vgs_on - vth;
        if d <= 0.0 || d.is_nan() {
            return Err(DatasheetError::Invalid {
                figure: "rds_on_vgs",
                reason: "must be above the threshold voltage",
            });
        }
        let base = Self::default_nmos();
        let vds_on = id_on.map(|i| i * rds);
        // `a` is the overdrive at `vgs_on`; VTO = vgs_on − a. KP follows
        // from the on-resistance, then `a` is solved so the threshold
        // point carries `id_th`.
        let params_at = |a: f64| -> Self {
            let kp = match (id_on, vds_on) {
                (Some(i), Some(v)) => i / ((a * v - 0.5 * v * v) * (1.0 + base.lambda * v)),
                _ => 1.0 / (rds * a),
            };
            Self {
                kp,
                vto: vgs_on - a,
                ..base.clone()
            }
        };
        let excess = |a: f64| mosfet_companion(vth, vth, 0.0, &params_at(a)).id - id_th;

        let mut lo = d.max(vds_on.unwrap_or(0.0)) * (1.0 + 1e-12) + 1e-12;
        if excess(lo) >= 0.0 {
            return Err(DatasheetError::Invalid {
                figure: "rds_on",
                reason: "can't be met with this threshold by a Level-1 MOSFET",
            });
        }
        let mut hi = 2.0 * lo + 1.0;
        while excess(hi) < 0.0 {
            hi *= 2.0;
            if hi > 1e6 {
                return Err(DatasheetError::Invalid {
                    figure: "vth_id",
                    reason: "can't be met with this on-resistance by a Level-1 MOSFET",
                });
            }
        }
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if excess(mid) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let fitted = params_at(0.5 * (lo + hi));
        Ok(match kind {
            MosfetKind::Nmos => fitted,
            MosfetKind::Pmos => Self {
                vto: -fitted.vto,
                ..fitted
            },
        })
    }
}

impl Default for MosfetParams {
    fn default() -> Self {
        Self::default_nmos()
    }
}

/// Companion model output for MNA stamping.
pub struct MosfetCompanion {
    /// Drain current at the operating point (A), drain → source, in the
    /// NMOS frame: callers mirror a PMOS (negate) to get its physical current.
    pub id: f64,
    /// Transconductance ∂Id/∂Vgs (S).
    pub gm: f64,
    /// Output conductance ∂Id/∂Vds (S).
    pub gds: f64,
    /// Body transconductance ∂Id/∂Vbs (S), from the body effect. Zero when
    /// Vbs >= PHI, where the threshold stops depending on Vbs.
    pub gmb: f64,
    /// Operating region.
    pub region: MosfetRegion,
    /// Effective threshold voltage after the body effect (V), NMOS frame.
    pub vth: f64,
}

/// Compute threshold voltage with body effect.
///
/// Vth = VTO + GAMMA * (sqrt(PHI - Vbs) - sqrt(PHI))
///
/// For NMOS with Vbs <= 0: Vth increases (harder to turn on). Once the body
/// is forward-biased past PHI (Vbs >= PHI) the sqrt term is clamped to 0
/// and Vth no longer depends on Vbs.
/// For PMOS, caller passes sign-adjusted Vbs and
/// [`MosfetParams::in_nmos_frame`] parameters. With the body tied to the
/// source, as in sindr, Vbs = 0 and Vth = VTO.
pub fn threshold_voltage(vbs: f64, params: &MosfetParams) -> f64 {
    let phi_minus_vbs = params.phi - vbs;
    // Clamp to prevent sqrt of negative (shouldn't happen physically)
    let sqrt_term = if phi_minus_vbs > 0.0 {
        phi_minus_vbs.sqrt()
    } else {
        0.0
    };
    params.vto + params.gamma * (sqrt_term - params.phi.sqrt())
}

/// Evaluate the Level 1 MOSFET model at a given operating point.
///
/// Works in the N-channel frame. For NMOS: Vgs, Vds, Vbs are used directly.
/// For PMOS: pass the mirrored voltages (Vsg, Vsd, Vsb; see
/// [`MosfetKind::nmos_frame`]) and [`MosfetParams::in_nmos_frame`]
/// parameters, so the equations work identically.
///
/// `gmb` is zero once the body is forward-biased past PHI, matching
/// [`threshold_voltage`].
pub fn mosfet_companion(vgs: f64, vds: f64, vbs: f64, params: &MosfetParams) -> MosfetCompanion {
    let vth = threshold_voltage(vbs, params);
    let vov = vgs - vth; // overdrive voltage

    if vov <= 0.0 {
        // Cutoff region
        return MosfetCompanion {
            id: 0.0,
            gm: 0.0,
            gds: 0.0,
            gmb: 0.0,
            region: MosfetRegion::Cutoff,
            vth,
        };
    }

    // Body effect transconductance factor: dVth/dVbs. Zero once the body is
    // forward-biased past PHI, where `threshold_voltage` clamps the sqrt term
    // and Vth stops depending on Vbs.
    let phi_minus_vbs = params.phi - vbs;
    let dvth_dvbs = if params.gamma > 0.0 && phi_minus_vbs > 0.0 {
        -params.gamma / (2.0 * phi_minus_vbs.sqrt())
    } else {
        0.0
    };

    if vds < vov {
        // Triode region: Id = KP * [(Vgs-Vth)*Vds - Vds^2/2] * (1 + LAMBDA*Vds)
        let id = params.kp * ((vov * vds) - (vds * vds / 2.0)) * (1.0 + params.lambda * vds);

        // Partial derivatives
        let gm = params.kp * vds * (1.0 + params.lambda * vds);
        let gds = params.kp * (vov - vds) * (1.0 + params.lambda * vds)
            + params.kp * ((vov * vds) - (vds * vds / 2.0)) * params.lambda;
        let gmb = -gm * dvth_dvbs;

        MosfetCompanion {
            id,
            gm,
            gds,
            gmb,
            region: MosfetRegion::Triode,
            vth,
        }
    } else {
        // Saturation region: Id = (KP/2) * (Vgs-Vth)^2 * (1 + LAMBDA*Vds)
        let id = (params.kp / 2.0) * vov * vov * (1.0 + params.lambda * vds);

        // Partial derivatives
        let gm = params.kp * vov * (1.0 + params.lambda * vds);
        let gds = (params.kp / 2.0) * vov * vov * params.lambda;
        let gmb = -gm * dvth_dvbs;

        MosfetCompanion {
            id,
            gm,
            gds,
            gmb,
            region: MosfetRegion::Saturation,
            vth,
        }
    }
}

/// The derivatives of a MOSFET's drain current, transconductance and output
/// conductance with respect to one parameter, at fixed terminal voltages, in
/// the N-channel frame of [`mosfet_companion`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct MosfetDerivative {
    /// ∂Id/∂p: drain current (A) per unit of the parameter.
    pub id: f64,
    /// ∂gm/∂p (S) per unit of the parameter.
    pub gm: f64,
    /// ∂gds/∂p (S) per unit of the parameter.
    pub gds: f64,
}

/// [`MosfetDerivative`]s with respect to `kp`, `vto` and `lambda`, from
/// [`mosfet_derivatives`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct MosfetDerivatives {
    /// With respect to `kp` (per A/V²).
    pub kp: MosfetDerivative,
    /// With respect to `vto` (per V) as `params` has it: in the N-channel
    /// frame, so a PMOS caller negates it for its SPICE-signed VTO.
    pub vto: MosfetDerivative,
    /// With respect to `lambda` (per 1/V).
    pub lambda: MosfetDerivative,
}

/// The exact derivatives of [`mosfet_companion`]'s `id`, `gm` and `gds` at
/// `vgs`, `vds`, `vbs` (V, N-channel frame) with respect to `kp`, `vto` and
/// `lambda`, for sensitivity analysis. All zero in cutoff.
pub fn mosfet_derivatives(
    vgs: f64,
    vds: f64,
    vbs: f64,
    params: &MosfetParams,
) -> MosfetDerivatives {
    let vov = vgs - threshold_voltage(vbs, params);
    let (kp, lambda) = (params.kp, params.lambda);
    let clm = 1.0 + lambda * vds;
    if vov <= 0.0 {
        return MosfetDerivatives::default();
    }
    if vds < vov {
        // Id = KP·(vov·vds − vds²/2)·(1 + λ·vds); ∂vov/∂VTO = −1.
        let shape = vov * vds - vds * vds / 2.0;
        MosfetDerivatives {
            kp: MosfetDerivative {
                id: shape * clm,
                gm: vds * clm,
                gds: (vov - vds) * clm + shape * lambda,
            },
            vto: MosfetDerivative {
                id: -kp * vds * clm,
                gm: 0.0,
                gds: -kp * clm - kp * vds * lambda,
            },
            lambda: MosfetDerivative {
                id: kp * shape * vds,
                gm: kp * vds * vds,
                gds: kp * (vov - vds) * vds + kp * shape,
            },
        }
    } else {
        // Id = (KP/2)·vov²·(1 + λ·vds).
        MosfetDerivatives {
            kp: MosfetDerivative {
                id: vov * vov / 2.0 * clm,
                gm: vov * clm,
                gds: vov * vov / 2.0 * lambda,
            },
            vto: MosfetDerivative {
                id: -kp * vov * clm,
                gm: -kp * clm,
                gds: -kp * vov * lambda,
            },
            lambda: MosfetDerivative {
                id: kp / 2.0 * vov * vov * vds,
                gm: kp * vov * vds,
                gds: kp / 2.0 * vov * vov,
            },
        }
    }
}

/// Detect MOSFET operating region from terminal voltages.
pub fn detect_region(vgs: f64, vds: f64, vth: f64) -> MosfetRegion {
    let vov = vgs - vth;
    if vov <= 0.0 {
        MosfetRegion::Cutoff
    } else if vds < vov {
        MosfetRegion::Triode
    } else {
        MosfetRegion::Saturation
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn nmos_cutoff() {
        let params = MosfetParams::default_nmos();
        let comp = mosfet_companion(0.0, 5.0, 0.0, &params);
        assert_eq!(comp.region, MosfetRegion::Cutoff);
        assert_relative_eq!(comp.id, 0.0, epsilon = 1e-15);
        assert_relative_eq!(comp.gm, 0.0, epsilon = 1e-15);
    }

    #[test]
    fn nmos_saturation() {
        let params = MosfetParams::default_nmos();
        // Vgs=3V, Vds=5V, Vbs=0: Vov=3-0.7=2.3V, Vds>Vov => saturation
        let comp = mosfet_companion(3.0, 5.0, 0.0, &params);
        assert_eq!(comp.region, MosfetRegion::Saturation);

        // Id = (KP/2) * Vov^2 * (1 + lambda*Vds)
        let expected_id = (2e-4 / 2.0) * 2.3 * 2.3 * (1.0 + 0.02 * 5.0);
        assert_relative_eq!(comp.id, expected_id, epsilon = 1e-10);
        assert!(comp.gm > 0.0);
        assert!(comp.gds > 0.0);
    }

    #[test]
    fn nmos_triode() {
        let params = MosfetParams::default_nmos();
        // Vgs=3V, Vds=0.5V, Vbs=0: Vov=2.3V, Vds<Vov => triode
        let comp = mosfet_companion(3.0, 0.5, 0.0, &params);
        assert_eq!(comp.region, MosfetRegion::Triode);

        let expected_id = 2e-4 * ((2.3 * 0.5) - (0.5 * 0.5 / 2.0)) * (1.0 + 0.02 * 0.5);
        assert_relative_eq!(comp.id, expected_id, epsilon = 1e-10);
    }

    #[test]
    fn body_effect_increases_vth() {
        let params = MosfetParams::default_nmos();
        let vth_no_body = threshold_voltage(0.0, &params);
        let vth_with_body = threshold_voltage(-2.0, &params); // Vbs = -2V for NMOS
        assert!(
            vth_with_body > vth_no_body,
            "Body effect should increase Vth: {} vs {}",
            vth_with_body,
            vth_no_body
        );
    }

    #[test]
    fn body_effect_gmb_nonzero() {
        let params = MosfetParams::default_nmos();
        let comp = mosfet_companion(3.0, 5.0, -1.0, &params);
        assert!(
            comp.gmb.abs() > 0.0,
            "gmb should be nonzero with body effect"
        );
    }

    #[test]
    fn saturation_id_vs_vgs() {
        // Id should increase with Vgs (quadratic)
        let params = MosfetParams::default_nmos();
        let comp1 = mosfet_companion(2.0, 5.0, 0.0, &params);
        let comp2 = mosfet_companion(3.0, 5.0, 0.0, &params);
        assert!(comp2.id > comp1.id);
    }

    #[test]
    fn channel_length_modulation() {
        // With lambda > 0, Id should increase slightly with Vds in saturation
        let params = MosfetParams::default_nmos();
        let comp1 = mosfet_companion(3.0, 2.5, 0.0, &params);
        let comp2 = mosfet_companion(3.0, 5.0, 0.0, &params);
        assert!(comp2.id > comp1.id, "CLM: Id should increase with Vds");
    }

    #[test]
    fn pmos_with_sign_flip() {
        // PMOS: caller flips signs so equations work as NMOS
        // Physical: Vsg=3V, Vsd=5V => effective Vgs=3V, Vds=5V
        let params = MosfetParams {
            kp: 1e-4,
            vto: 0.7, // Note: caller handles sign, so VTO is positive here
            lambda: 0.02,
            gamma: 0.4,
            phi: 0.6,
            ..MosfetParams::default()
        };
        let comp = mosfet_companion(3.0, 5.0, 0.0, &params);
        assert_eq!(comp.region, MosfetRegion::Saturation);
        assert!(comp.id > 0.0);
    }

    #[test]
    fn detect_region_matches_companion() {
        let params = MosfetParams::default_nmos();

        // Cutoff
        let vth = threshold_voltage(0.0, &params);
        assert_eq!(detect_region(0.0, 5.0, vth), MosfetRegion::Cutoff);

        // Saturation
        assert_eq!(detect_region(3.0, 5.0, vth), MosfetRegion::Saturation);

        // Triode
        assert_eq!(detect_region(3.0, 0.5, vth), MosfetRegion::Triode);
    }

    /// gm, gds and gmb must be the derivatives of the drain current the
    /// companion returns, including with the body forward-biased past PHI
    /// (a PMOS with its body at ground and source at VDD sits there).
    #[test]
    fn companion_conductances_match_finite_difference() {
        let params = MosfetParams {
            vto: 0.7,
            ..MosfetParams::default_nmos()
        };
        let id = |vgs: f64, vds: f64, vbs: f64| mosfet_companion(vgs, vds, vbs, &params).id;
        let h = 1e-6;
        for vbs in [-3.0, -0.5, 0.0, 0.3, 2.0, 5.0] {
            for (vgs, vds) in [(0.3, 2.0), (3.0, 0.5), (3.0, 5.0), (1.5, 0.2), (2.0, 4.0)] {
                let c = mosfet_companion(vgs, vds, vbs, &params);
                let fd_gm = (id(vgs + h, vds, vbs) - id(vgs - h, vds, vbs)) / (2.0 * h);
                let fd_gds = (id(vgs, vds + h, vbs) - id(vgs, vds - h, vbs)) / (2.0 * h);
                let fd_gmb = (id(vgs, vds, vbs + h) - id(vgs, vds, vbs - h)) / (2.0 * h);
                for (name, analytic, fd) in [
                    ("gm", c.gm, fd_gm),
                    ("gds", c.gds, fd_gds),
                    ("gmb", c.gmb, fd_gmb),
                ] {
                    let tol = 1e-5 * analytic.abs().max(fd.abs()) + 1e-12;
                    assert!(
                        (analytic - fd).abs() <= tol,
                        "{name} at vgs={vgs}, vds={vds}, vbs={vbs}: analytic={analytic:e}, finite difference={fd:e}"
                    );
                }
            }
        }
    }

    #[test]
    fn mosfet_derivatives_match_central_differences() {
        let params = MosfetParams {
            kp: 3e-4,
            vto: 0.9,
            lambda: 0.03,
            ..MosfetParams::default_nmos()
        };
        type Set = fn(&mut MosfetParams, f64);
        type Pick = fn(&MosfetDerivatives) -> MosfetDerivative;
        let cases: [(Set, Pick, f64); 3] = [
            (|p, x| p.kp = x, |d| d.kp, params.kp),
            (|p, x| p.vto = x, |d| d.vto, params.vto),
            (|p, x| p.lambda = x, |d| d.lambda, params.lambda),
        ];
        // Saturation, triode, and cutoff.
        for (vgs, vds) in [(2.0, 3.0), (2.5, 0.4), (0.5, 2.0)] {
            let d = mosfet_derivatives(vgs, vds, 0.0, &params);
            for (set, pick, x) in cases {
                let h = x * 1e-6;
                let at = |x: f64| {
                    let mut p = params.clone();
                    set(&mut p, x);
                    let c = mosfet_companion(vgs, vds, 0.0, &p);
                    [c.id, c.gm, c.gds]
                };
                let (hi, lo) = (at(x + h), at(x - h));
                let want = pick(&d);
                for (k, got) in [want.id, want.gm, want.gds].into_iter().enumerate() {
                    let fd = (hi[k] - lo[k]) / (2.0 * h);
                    assert_relative_eq!(got, fd, max_relative = 1e-6, epsilon = 1e-12);
                }
            }
        }
    }

    /// The fitted part reproduces both datasheet figures to 1e-6 relative.
    #[test]
    fn from_datasheet_meets_threshold_and_on_resistance() {
        for (kind, sign) in [(MosfetKind::Nmos, 1.0), (MosfetKind::Pmos, -1.0)] {
            for id_on in [None, Some(0.5)] {
                let mut ds = MosfetDatasheet::new(sign * 2.1, 250e-6, 0.28, sign * 10.0);
                ds.rds_on_id = id_on;
                let p = MosfetParams::from_datasheet(kind, &ds).unwrap();
                let n = p.in_nmos_frame(kind);
                let at_th = mosfet_companion(2.1, 2.1, 0.0, &n).id;
                assert_relative_eq!(at_th, 250e-6, max_relative = 1e-6);
                let rds = match id_on {
                    Some(i) => (i * 0.28) / mosfet_companion(10.0, i * 0.28, 0.0, &n).id,
                    None => 1.0 / mosfet_companion(10.0, 0.0, 0.0, &n).gds,
                };
                assert_relative_eq!(rds, 0.28, max_relative = 1e-6);
                assert_eq!(p.vto.signum(), sign);
            }
        }
    }

    #[test]
    fn from_datasheet_refuses_rds_on_below_threshold() {
        let ds = MosfetDatasheet::new(2.0, 250e-6, 1.0, 1.5);
        assert!(MosfetParams::from_datasheet(MosfetKind::Nmos, &ds).is_err());
        let empty = MosfetDatasheet::default();
        assert!(MosfetParams::from_datasheet(MosfetKind::Nmos, &empty).is_err());
    }
}
