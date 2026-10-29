//! IGBT (Insulated Gate Bipolar Transistor) simplified model.
//!
//! An IGBT combines MOSFET gate control with BJT output characteristics.
//! This educational model uses the MOSFET Level-1 square law for the
//! collector current, and blocks reverse collector-emitter voltage.
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
    ///
    /// Not used by the current I–V model: the collector current follows the
    /// square law alone. Kept so existing circuits still deserialize.
    pub vce_sat: f64,
}

impl Default for IgbtParams {
    fn default() -> Self {
        Self {
            vth: 5.0,
            k: 5.0,
            vce_sat: 2.0,
        }
    }
}

/// IGBT companion model output.
pub struct IgbtCompanion {
    /// Transconductance ∂Ids/∂Vge (S).
    pub gm: f64,
    /// Output conductance ∂Ids/∂Vce (S): zero in saturation and cutoff.
    pub g_ce: f64,
    /// Drain/collector current at operating point.
    pub ids: f64,
    /// Companion current source value for MNA stamping:
    /// `ids − gm·vge − g_ce·vce`.
    pub i_eq: f64,
}

/// Evaluate IGBT companion model at gate-emitter voltage `vge` and
/// collector-emitter voltage `vce`.
///
/// Collector current (MOSFET Level-1), with `vov = vge − vth`:
/// - cutoff (`vov ≤ 0`) or reverse (`vce ≤ 0`): 0
/// - saturation (`vce ≥ vov`): `K/2 · vov²`
/// - triode (`0 < vce < vov`): `K · (vov − vce/2) · vce`
///
/// `gm` and `g_ce` are the exact partial derivatives of that current, so the
/// linearisation `Ic ≈ gm·vge + g_ce·vce + i_eq` is a Newton step. The stamp
/// puts `gm` in as a gate-controlled current source and `g_ce` as a C–E
/// conductance.
pub fn igbt_companion(vge: f64, vce: f64, params: &IgbtParams) -> IgbtCompanion {
    let vov = vge - params.vth;

    let (ids, gm, g_ce) = if vov <= 0.0 || vce <= 0.0 {
        // Cutoff, or reverse-biased C–E: no collector current
        (0.0, 0.0, 0.0)
    } else if vce >= vov {
        // Saturation: Ids = K/2 * vov^2
        (0.5 * params.k * vov * vov, params.k * vov, 0.0)
    } else {
        // Triode: Ids = K * (vov - vce/2) * vce
        (
            params.k * (vov - vce / 2.0) * vce,
            params.k * vce,
            params.k * (vov - vce),
        )
    };

    IgbtCompanion {
        gm,
        g_ce,
        ids,
        i_eq: ids - gm * vge - g_ce * vce,
    }
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
        assert!(
            c.ids > 0.0,
            "ids should be positive in saturation, got {}",
            c.ids
        );
        assert!(
            c.gm > 0.0,
            "gm should be positive in saturation, got {}",
            c.gm
        );
        // ids = 0.5 * 5 * 5^2 = 62.5 A
        let expected_ids = 0.5 * 5.0 * 5.0_f64.powi(2);
        assert!(
            (c.ids - expected_ids).abs() < 1e-9,
            "ids={} expected {}",
            c.ids,
            expected_ids
        );
    }

    #[test]
    fn igbt_conducts_in_triode() {
        let params = IgbtParams::default(); // vth=5, k=5
                                            // vge=10 → vgs_eff=5; vce=2 < vgs_eff=5 → triode region
        let c = igbt_companion(10.0, 2.0, &params);
        assert!(
            c.ids > 0.0,
            "ids should be positive in triode, got {}",
            c.ids
        );
        // ids = k * (vgs_eff - vce/2) * vce = 5 * (5 - 1) * 2 = 40 A
        let expected_ids = 5.0 * (5.0 - 2.0 / 2.0) * 2.0;
        assert!(
            (c.ids - expected_ids).abs() < 1e-9,
            "ids={} expected {}",
            c.ids,
            expected_ids
        );
    }

    /// gm and g_ce must be the derivatives of ids, and i_eq must make the
    /// linearisation reproduce ids at its own operating point.
    #[test]
    fn igbt_companion_matches_finite_difference() {
        let params = IgbtParams::default(); // vth=5, k=5
        let ids = |vge: f64, vce: f64| igbt_companion(vge, vce, &params).ids;
        let h = 1e-6;
        // Cutoff, reverse, triode, saturation.
        for (vge, vce) in [
            (3.0, 10.0),
            (10.0, -2.0),
            (10.0, 2.0),
            (8.0, 1.0),
            (10.0, 8.0),
        ] {
            let c = igbt_companion(vge, vce, &params);
            let fd_gm = (ids(vge + h, vce) - ids(vge - h, vce)) / (2.0 * h);
            let fd_gce = (ids(vge, vce + h) - ids(vge, vce - h)) / (2.0 * h);
            assert!(
                (c.gm - fd_gm).abs() <= 1e-6 * fd_gm.abs() + 1e-9,
                "gm at ({vge}, {vce}): {} vs {fd_gm}",
                c.gm
            );
            assert!(
                (c.g_ce - fd_gce).abs() <= 1e-6 * fd_gce.abs() + 1e-9,
                "g_ce at ({vge}, {vce}): {} vs {fd_gce}",
                c.g_ce
            );
            let i_lin = c.gm * vge + c.g_ce * vce + c.i_eq;
            assert!(
                (i_lin - c.ids).abs() <= 1e-9 * c.ids.abs() + 1e-12,
                "companion at ({vge}, {vce})"
            );
        }
    }
}
