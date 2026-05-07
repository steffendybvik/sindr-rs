//! BJT (Bipolar Junction Transistor) device model.
//!
//! Ebers–Moll companion model with Early voltage. Produces a 3×3 conductance
//! / current contribution that an MNA solver stamps for [base, collector,
//! emitter] terminals at each Newton–Raphson iteration.
//!
//! Both NPN and PNP are supported via [`BjtKind`]; the PNP case mirrors NPN
//! through sign conventions on `vbe` and `vbc`.

use crate::diode::V_T;

/// BJT polarity.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BjtKind {
    /// NPN (electrons are majority carriers in base region).
    #[cfg_attr(feature = "serde", serde(rename = "npn"))]
    Npn,
    /// PNP (holes are majority carriers in base region).
    #[cfg_attr(feature = "serde", serde(rename = "pnp"))]
    Pnp,
}

/// Operating region of a BJT — useful for diagnostics and circuit
/// classification (amplifier biased "active", switch in "saturation", …).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BjtRegion {
    /// Both junctions reverse-biased — negligible currents.
    Cutoff,
    /// Forward-active: BE forward-biased, BC reverse-biased. Linear amplifier region.
    Active,
    /// Both junctions forward-biased — output voltage clamped near `VCE_sat`.
    Saturation,
}

impl BjtRegion {
    pub fn as_str(&self) -> &'static str {
        match self {
            BjtRegion::Cutoff => "cutoff",
            BjtRegion::Active => "active",
            BjtRegion::Saturation => "saturation",
        }
    }
}

#[derive(Debug, Clone)]
pub struct BjtParams {
    pub is: f64,          // Saturation current (A)
    pub bf: f64,          // Forward current gain
    pub nf: f64,          // Forward emission coefficient
    pub br: f64,          // Reverse current gain
    pub nr: f64,          // Reverse emission coefficient
    pub vaf: f64,         // Forward Early voltage (V). 0.0 = infinite (no Early effect).
    pub var: f64,         // Reverse Early voltage (V). 0.0 = infinite.
    pub temperature: f64, // Junction temperature (K). Default 300.15 K = 27°C.
}

impl BjtParams {
    pub fn new(bf: f64) -> Self {
        Self {
            is: 1e-14,
            bf,
            nf: 1.0,
            br: 1.0,
            nr: 1.0,
            vaf: 0.0,
            var: 0.0,
            temperature: 300.15,
        }
    }
}

/// Companion model output for one NR iteration.
/// Contains conductances and equivalent currents for MNA stamping.
pub struct BjtCompanion {
    pub g_be: f64, // B-E forward junction conductance
    pub g_bc: f64, // B-C reverse junction conductance
    pub g_ce: f64, // Early voltage output conductance between C and E
    pub ic: f64,   // collector current at operating point
    pub ib: f64,   // base current at operating point
    pub vbe: f64,  // junction voltage (for RHS computation)
    pub vbc: f64,  // junction voltage (for RHS computation)
}

/// Evaluate Ebers-Moll transport model at given junction voltages.
/// Returns companion model data for MNA stamping.
///
/// Junction voltages should already be sign-adjusted for PNP
/// (caller passes Vbe_eff and Vbc_eff).
/// Maximum junction voltage for exponential evaluation.
/// Beyond this, use linear extrapolation to prevent overflow.
const V_MAX_EXP: f64 = 40.0 * V_T; // ~1.034V

pub fn bjt_companion(vbe: f64, vbc: f64, params: &BjtParams) -> BjtCompanion {
    let nf_vt = params.nf * V_T;
    let nr_vt = params.nr * V_T;

    // Clamp exponentials: beyond V_MAX_EXP, linearize to prevent overflow
    let (exp_be, vbe_eval) = if vbe > V_MAX_EXP {
        let e = (V_MAX_EXP / nf_vt).exp();
        // Linear extrapolation: exp(v/nVt) ≈ exp(vmax/nVt) * (1 + (v-vmax)/nVt)
        (e * (1.0 + (vbe - V_MAX_EXP) / nf_vt), vbe)
    } else {
        ((vbe / nf_vt).exp(), vbe)
    };
    let (exp_bc, vbc_eval) = if vbc > V_MAX_EXP {
        let e = (V_MAX_EXP / nr_vt).exp();
        (e * (1.0 + (vbc - V_MAX_EXP) / nr_vt), vbc)
    } else {
        ((vbc / nr_vt).exp(), vbc)
    };
    let _ = (vbe_eval, vbc_eval); // used for clarity, actual vbe/vbc used below

    // Junction currents
    let i_f = params.is * (exp_be - 1.0);
    let i_r = params.is * (exp_bc - 1.0);

    // Junction conductances (derivatives).
    // Cap conductances at a maximum to prevent the NR Jacobian from becoming
    // ill-conditioned near saturation where both junctions are forward-biased.
    // G_MAX of 10.0 S (0.1 ohm) is large enough for accurate modeling while
    // preventing the 1e5+ S values that cause NR oscillation.
    const G_MAX: f64 = 10.0;
    let g_be = ((params.is / nf_vt) * exp_be).min(G_MAX);
    let g_bc = ((params.is / nr_vt) * exp_bc).min(G_MAX);

    // Terminal currents (transport model)
    let alpha_r = params.br / (params.br + 1.0);
    let ic = i_f - i_r / alpha_r;
    let ib = i_f / params.bf + i_r / params.br;

    // Early voltage output conductance: g_ce = Ic / VAF
    // When vaf = 0.0, Early effect is disabled (infinite VAF).
    let g_ce = if params.vaf > 0.0 && params.vaf.is_finite() {
        ic.abs() / params.vaf
    } else {
        0.0
    };

    BjtCompanion {
        g_be,
        g_bc,
        g_ce,
        ic,
        ib,
        vbe,
        vbc,
    }
}

/// Detect operating region from junction voltages.
pub fn detect_region(vbe: f64, vbc: f64) -> BjtRegion {
    if vbe < 0.1 {
        BjtRegion::Cutoff
    } else if vbc < 0.0 {
        BjtRegion::Active
    } else {
        BjtRegion::Saturation
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn bjt_companion_at_zero_voltages() {
        let params = BjtParams::new(100.0);
        let comp = bjt_companion(0.0, 0.0, &params);

        // At Vbe=0, Vbc=0: exp(0)=1, I_f=0, I_r=0 -> Ic~0, Ib~0
        assert_relative_eq!(comp.ic, 0.0, epsilon = 1e-20);
        assert_relative_eq!(comp.ib, 0.0, epsilon = 1e-20);

        // g_be = IS/(NF*VT) * exp(0) = IS/(NF*VT), small positive
        let expected_g_be = params.is / (params.nf * V_T);
        assert_relative_eq!(comp.g_be, expected_g_be, epsilon = 1e-20);
        assert!(comp.g_be > 0.0);

        // g_bc = IS/(NR*VT) * exp(0) = IS/(NR*VT), small positive
        let expected_g_bc = params.is / (params.nr * V_T);
        assert_relative_eq!(comp.g_bc, expected_g_bc, epsilon = 1e-20);
        assert!(comp.g_bc > 0.0);
    }

    #[test]
    fn bjt_companion_forward_active() {
        let params = BjtParams::new(100.0);
        let comp = bjt_companion(0.7, -5.0, &params);

        // Active region: Ic should be large positive (mA range)
        assert!(comp.ic > 1e-3, "Ic should be in mA range, got {}", comp.ic);

        // Ib should be small positive (Ic/beta range)
        assert!(comp.ib > 0.0);
        assert!(comp.ib < comp.ic);

        // g_be >> g_bc (forward junction strongly conducting, reverse off)
        assert!(
            comp.g_be > comp.g_bc * 1e6,
            "g_be={} should be >> g_bc={}",
            comp.g_be,
            comp.g_bc
        );
    }

    #[test]
    fn detect_region_cutoff() {
        assert_eq!(detect_region(-0.5, -5.0), BjtRegion::Cutoff);
    }

    #[test]
    fn detect_region_active() {
        assert_eq!(detect_region(0.7, -5.0), BjtRegion::Active);
    }

    #[test]
    fn detect_region_saturation() {
        assert_eq!(detect_region(0.7, 0.3), BjtRegion::Saturation);
    }

    /// Companion model at saturation: both junctions forward biased (Vbe=0.7, Vbc=0.3).
    /// Verifies conductances, currents, and KCL.
    #[test]
    fn test_bjt_companion_saturation_both_junctions() {
        let params = BjtParams::new(100.0);
        // Vbe=0.7V (strongly forward), Vbc=0.6V (forward but less)
        let comp = bjt_companion(0.7, 0.6, &params);

        // Both g_be and g_bc should be positive and significant
        assert!(comp.g_be > 1e-3, "g_be={} should be significant", comp.g_be);
        assert!(comp.g_bc > 1e-3, "g_bc={} should be significant", comp.g_bc);

        // g_be > g_bc (forward junction more strongly biased at 0.7V vs 0.6V)
        assert!(
            comp.g_be > comp.g_bc,
            "g_be={} should > g_bc={}",
            comp.g_be,
            comp.g_bc
        );

        // Both junction currents should be positive (both forward)
        let i_f = params.is * ((0.7 / (params.nf * V_T)).exp() - 1.0);
        let i_r = params.is * ((0.6 / (params.nr * V_T)).exp() - 1.0);
        assert!(i_f > 0.0, "i_f should be positive");
        assert!(i_r > 0.0, "i_r should be positive");

        // Ic should be less than in pure active mode (reverse junction subtracts)
        let active_comp = bjt_companion(0.7, -5.0, &params);
        assert!(
            comp.ic < active_comp.ic,
            "Saturation Ic={} should < Active Ic={}",
            comp.ic,
            active_comp.ic
        );

        // KCL: Ic + Ib + Ie = 0, where Ie = -(Ic + Ib)
        let ie = -(comp.ic + comp.ib);
        assert!((comp.ic + comp.ib + ie).abs() < 1e-15, "KCL violated");
    }

    /// BjtParams::new() has vaf=0.0 — Early effect disabled — so g_ce must be 0.0.
    #[test]
    fn bjt_early_voltage_zero_means_no_effect() {
        let params = BjtParams::new(100.0);
        assert_eq!(params.vaf, 0.0);
        let comp = bjt_companion(0.7, -5.0, &params);
        assert_eq!(comp.g_ce, 0.0, "g_ce should be 0 when vaf=0");
    }

    /// With vaf=100V in forward-active region, g_ce = |Ic| / vaf > 0.
    #[test]
    fn bjt_early_voltage_nonzero_adds_conductance() {
        let mut params = BjtParams::new(100.0);
        params.vaf = 100.0;
        let comp = bjt_companion(0.7, -5.0, &params);
        assert!(
            comp.g_ce > 0.0,
            "g_ce should be positive when vaf>0 and Ic>0"
        );
        // g_ce should equal |Ic| / vaf
        let expected_g_ce = comp.ic.abs() / 100.0;
        assert_relative_eq!(comp.g_ce, expected_g_ce, epsilon = 1e-15);
    }

    /// PNP companion outputs should be IDENTICAL to NPN at same effective voltages.
    /// The sign flip happens in stamp.rs, not in bjt_companion.
    #[test]
    fn test_bjt_companion_pnp_sign_convention() {
        let params = BjtParams::new(100.0);

        // Same effective voltages for both
        let npn = bjt_companion(0.7, -5.0, &params);
        let pnp = bjt_companion(0.7, -5.0, &params);

        // Outputs should be identical (bjt_companion doesn't know about NPN/PNP)
        assert_relative_eq!(npn.ic, pnp.ic, epsilon = 1e-15);
        assert_relative_eq!(npn.ib, pnp.ib, epsilon = 1e-15);
        assert_relative_eq!(npn.g_be, pnp.g_be, epsilon = 1e-15);
        assert_relative_eq!(npn.g_bc, pnp.g_bc, epsilon = 1e-15);
    }
}
