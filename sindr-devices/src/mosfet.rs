//! MOSFET Level 1 (Shichman-Hodges) model with body effect.
//!
//! Implements NMOS and PMOS transistors with three operating regions:
//! cutoff, triode (linear), and saturation. The companion model linearizes
//! the device at each NR iteration for MNA stamping.

/// MOSFET type (NMOS or PMOS).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MosfetKind {
    #[cfg_attr(feature = "serde", serde(rename = "nmos"))]
    Nmos,
    #[cfg_attr(feature = "serde", serde(rename = "pmos"))]
    Pmos,
}

/// Operating region of a MOSFET.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MosfetRegion {
    Cutoff,
    Triode,
    Saturation,
}

impl MosfetRegion {
    pub fn as_str(&self) -> &'static str {
        match self {
            MosfetRegion::Cutoff => "cutoff",
            MosfetRegion::Triode => "triode",
            MosfetRegion::Saturation => "saturation",
        }
    }
}

/// MOSFET device parameters (Level 1 Shichman-Hodges).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
pub struct MosfetParams {
    /// Transconductance parameter (A/V^2). SPICE KP parameter.
    pub kp: f64,
    /// Zero-bias threshold voltage (V). SPICE VTO parameter.
    pub vto: f64,
    /// Channel-length modulation (1/V). SPICE LAMBDA parameter.
    pub lambda: f64,
    /// Body effect coefficient (V^0.5). SPICE GAMMA parameter.
    pub gamma: f64,
    /// Surface potential (V). SPICE PHI parameter.
    pub phi: f64,
}

impl MosfetParams {
    /// Default NMOS parameters (typical educational values).
    pub fn default_nmos() -> Self {
        Self {
            kp: 2e-4,    // 200 uA/V^2
            vto: 0.7,    // 0.7V threshold
            lambda: 0.02, // 0.02 /V
            gamma: 0.4,  // 0.4 V^0.5
            phi: 0.6,    // 0.6V
        }
    }

    /// Default PMOS parameters (typical educational values).
    pub fn default_pmos() -> Self {
        Self {
            kp: 1e-4,     // 100 uA/V^2 (typically half of NMOS)
            vto: -0.7,    // -0.7V threshold (negative for PMOS)
            lambda: 0.02,
            gamma: 0.4,
            phi: 0.6,
        }
    }
}

impl Default for MosfetParams {
    fn default() -> Self {
        Self::default_nmos()
    }
}

/// Companion model output for MNA stamping.
pub struct MosfetCompanion {
    /// Drain current at operating point.
    pub id: f64,
    /// Transconductance: dId/dVgs.
    pub gm: f64,
    /// Output conductance: dId/dVds.
    pub gds: f64,
    /// Body transconductance: dId/dVbs (from body effect).
    pub gmb: f64,
    /// Operating region.
    pub region: MosfetRegion,
    /// Effective threshold voltage (after body effect).
    pub vth: f64,
}

/// Compute threshold voltage with body effect.
///
/// Vth = VTO + GAMMA * (sqrt(PHI - Vbs) - sqrt(PHI))
///
/// For NMOS with Vbs <= 0: Vth increases (harder to turn on).
/// For PMOS, caller passes sign-adjusted Vbs.
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
/// All voltages should be sign-adjusted by the caller for PMOS
/// (i.e., pass |Vgs|, |Vds|, |Vbs| with appropriate polarity handling).
///
/// For NMOS: Vgs, Vds, Vbs are used directly.
/// For PMOS: caller negates terminal voltages so the equations work identically.
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

    // Body effect transconductance factor: dVth/dVbs
    let phi_minus_vbs = (params.phi - vbs).max(1e-6);
    let dvth_dvbs = if params.gamma > 0.0 {
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
        assert!(comp.gmb.abs() > 0.0, "gmb should be nonzero with body effect");
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
}
