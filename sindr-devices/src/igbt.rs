//! IGBT (Insulated Gate Bipolar Transistor) simplified model.
//!
//! An IGBT combines MOSFET gate control with BJT output characteristics.
//! This educational model uses MOSFET Level-1 gate control plus an on-state
//! voltage drop (VCE_sat) and output conductance g_ce = Ic / VCE_sat.
//!
//! Nodes: [gate, collector, emitter] — same ordering convention as BJT.
//! Gate controls current like a MOSFET; collector/emitter behave like BJT.
//!
//! Simplified model is appropriate for educational simulation.
//! Full IGBT models (IGBTv3) have 20+ parameters — out of scope.

/// IGBT parameters (simplified model).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IgbtParams {
    /// Gate threshold voltage (V). Typical: 4.0–6.0 V.
    pub vth: f64,
    /// Transconductance parameter (A/V²). Typical: 5.0.
    pub k: f64,
    /// On-state collector-emitter saturation voltage (V). Typical: 2.0.
    pub vce_sat: f64,
}

impl Default for IgbtParams {
    fn default() -> Self {
        Self { vth: 5.0, k: 5.0, vce_sat: 2.0 }
    }
}

/// IGBT companion model output.
pub struct IgbtCompanion {
    /// Channel conductance (transconductance linearised around operating point).
    pub gm: f64,
    /// Output conductance between collector and emitter (g_ce = ids / vce_sat).
    pub g_ce: f64,
    /// Drain/collector current at operating point.
    pub ids: f64,
    /// Companion current source value for MNA stamping.
    pub i_eq: f64,
}

/// Evaluate IGBT companion model at gate-emitter voltage `vge` and
/// collector-emitter voltage `vce`.
///
/// Gate control: MOSFET Level-1 (square-law) saturation model.
/// Output: BJT-like g_ce = ids / vce_sat (finite output impedance).
///
/// Returns companion model for MNA stamping. The stamp adds:
/// - gm between C and E (as conductance)
/// - g_ce between C and E (output conductance)
/// - i_eq as current source from E to C
pub fn igbt_companion(vge: f64, vce: f64, params: &IgbtParams) -> IgbtCompanion {
    let vgs_eff = vge - params.vth;

    if vgs_eff <= 0.0 {
        // Cutoff: gate not driven, no collector current
        return IgbtCompanion { gm: 0.0, g_ce: 0.0, ids: 0.0, i_eq: 0.0 };
    }

    // MOSFET Level-1 saturation (VCE_sat is the saturation boundary)
    let ids = if vce >= vgs_eff {
        // Saturation region: Ids = K/2 * (Vge - Vth)^2
        0.5 * params.k * vgs_eff * vgs_eff
    } else {
        // Triode region: Ids = K * (Vge - Vth - Vce/2) * Vce
        params.k * (vgs_eff - vce / 2.0) * vce
    };
    let ids = ids.max(0.0);

    // Linearised transconductance gm = dIds/dVge
    let gm = if vce >= vgs_eff {
        params.k * vgs_eff // saturation: gm = K * (Vge - Vth)
    } else {
        params.k * vce // triode: gm = K * Vce
    };
    let gm = gm.max(0.0);

    // Output conductance: g_ce = Ids / VCE_sat (BJT-like finite output impedance)
    let g_ce = if params.vce_sat > 0.0 && ids > 0.0 {
        ids / params.vce_sat
    } else {
        0.0
    };

    // Companion current source: I_eq = Ids - gm * vge - g_ce * vce
    let i_eq = ids - gm * vge - g_ce * vce;

    IgbtCompanion { gm, g_ce, ids, i_eq }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn igbt_cutoff_when_vge_below_vth() {
        let params = IgbtParams::default(); // vth = 5.0
        // vge = 0.0 < vth: cutoff region
        let c = igbt_companion(0.0, 10.0, &params);
        assert_eq!(c.ids, 0.0);
        assert_eq!(c.gm, 0.0);
        assert_eq!(c.g_ce, 0.0);
        assert_eq!(c.i_eq, 0.0);
    }

    #[test]
    fn igbt_cutoff_at_threshold() {
        let params = IgbtParams::default(); // vth = 5.0
        // vge = vth exactly: vgs_eff = 0 → cutoff
        let c = igbt_companion(5.0, 10.0, &params);
        assert_eq!(c.ids, 0.0);
        assert_eq!(c.gm, 0.0);
    }

    #[test]
    fn igbt_conducts_in_saturation() {
        let params = IgbtParams::default(); // vth=5, k=5
        // vge=10 → vgs_eff=5; vce=8 >= vgs_eff=5 → saturation
        let c = igbt_companion(10.0, 8.0, &params);
        assert!(c.ids > 0.0, "ids should be positive in saturation, got {}", c.ids);
        assert!(c.gm > 0.0, "gm should be positive in saturation, got {}", c.gm);
        // ids = 0.5 * 5 * 5^2 = 62.5 A
        let expected_ids = 0.5 * 5.0 * 5.0_f64.powi(2);
        assert!((c.ids - expected_ids).abs() < 1e-9,
            "ids={} expected {}", c.ids, expected_ids);
    }

    #[test]
    fn igbt_conducts_in_triode() {
        let params = IgbtParams::default(); // vth=5, k=5
        // vge=10 → vgs_eff=5; vce=2 < vgs_eff=5 → triode region
        let c = igbt_companion(10.0, 2.0, &params);
        assert!(c.ids > 0.0, "ids should be positive in triode, got {}", c.ids);
        // ids = k * (vgs_eff - vce/2) * vce = 5 * (5 - 1) * 2 = 40 A
        let expected_ids = 5.0 * (5.0 - 2.0 / 2.0) * 2.0;
        assert!((c.ids - expected_ids).abs() < 1e-9,
            "ids={} expected {}", c.ids, expected_ids);
    }

    #[test]
    fn igbt_g_ce_is_ids_over_vce_sat() {
        let params = IgbtParams::default(); // vce_sat=2.0
        // Saturation: vge=10, vce=8
        let c = igbt_companion(10.0, 8.0, &params);
        let expected_g_ce = c.ids / params.vce_sat;
        assert!((c.g_ce - expected_g_ce).abs() < 1e-9,
            "g_ce={} expected ids/vce_sat={}", c.g_ce, expected_g_ce);
    }
}
