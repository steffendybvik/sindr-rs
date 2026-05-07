//! JFET device model: Shockley square-law.
//! N-channel and P-channel supported via sign conventions.

const GMIN: f64 = 1e-12; // minimum conductance floor

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JfetKind {
    #[cfg_attr(feature = "serde", serde(rename = "nchannel"))]
    NChannel,
    #[cfg_attr(feature = "serde", serde(rename = "pchannel"))]
    PChannel,
}

/// Newton-Raphson linearised companion for JFET
pub struct JfetCompanion {
    pub gm: f64,   // transconductance dId/dVgs
    pub gds: f64,  // drain-source conductance dId/dVds
    pub i_eq: f64, // equivalent current for MNA stamp
}

/// Compute JFET companion at operating point (vgs, vds).
/// Nodes order: [gate, drain, source] — same as MOSFET convention.
/// N-channel: Vp < 0 (typically -2 to -6 V), Idss > 0.
pub fn jfet_companion(vgs: f64, vds: f64, kind: JfetKind, idss: f64, vp: f64) -> JfetCompanion {
    let (vgs, vds, idss, vp) = match kind {
        JfetKind::NChannel => (vgs, vds, idss, vp),
        JfetKind::PChannel => (-vgs, -vds, idss, -vp.abs()), // sign-flip for P-channel
    };

    // Clamp vgs to avoid numerical issues
    let vgs = vgs.max(vp - 1.0); // don't go too deep into cutoff

    if vgs <= vp {
        // Cutoff region: Id = 0
        return JfetCompanion {
            gm: GMIN,
            gds: GMIN,
            i_eq: 0.0,
        };
    }

    let vgs_norm = 1.0 - vgs / vp; // (1 - Vgs/Vp)

    if vds >= vgs - vp {
        // Saturation region: Id = Idss * (1 - Vgs/Vp)²
        let id = idss * vgs_norm * vgs_norm;
        // gm = dId/dVgs = 2*Idss*(1 - Vgs/Vp)*(-1/Vp) = -2*Idss*vgs_norm/Vp
        let gm = (-2.0 * idss * vgs_norm / vp).max(GMIN);
        let gds = GMIN;
        let i_eq = id - gm * vgs - gds * vds;
        JfetCompanion { gm, gds, i_eq }
    } else {
        // Triode (ohmic) region: Id = Idss/Vp² * (2*(Vgs-Vp)*Vds - Vds²)
        let vp_sq = vp * vp;
        let id = idss / vp_sq * (2.0 * (vgs - vp) * vds - vds * vds);
        // gm = dId/dVgs = 2*Idss*Vds/Vp²
        let gm = (2.0 * idss * vds / vp_sq).abs().max(GMIN);
        // gds = dId/dVds = 2*Idss*(Vgs-Vp-Vds)/Vp²
        let gds = (2.0 * idss * (vgs - vp - vds) / vp_sq).max(GMIN);
        let i_eq = id - gm * vgs - gds * vds;
        JfetCompanion { gm, gds, i_eq }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nchannel_saturation_at_vgs_zero() {
        // Vgs=0, Vds > |Vp|: Id should equal Idss
        let c = jfet_companion(0.0, 5.0, JfetKind::NChannel, 10e-3, -2.0);
        // Id = Idss * (1 - 0/(-2))² = Idss * 1 = 10 mA
        // Verify via i_eq + gm*0 + gds*5 ≈ 10 mA
        let id_approx = c.i_eq + c.gm * 0.0 + c.gds * 5.0;
        assert!(
            (id_approx - 10e-3).abs() < 1e-4,
            "Id at Vgs=0 should be ~Idss=10mA, got {}",
            id_approx
        );
    }

    #[test]
    fn nchannel_cutoff() {
        // Vgs = Vp (pinch-off)
        let c = jfet_companion(-2.0, 5.0, JfetKind::NChannel, 10e-3, -2.0);
        assert!(
            c.i_eq.abs() < 1e-9,
            "Cutoff Id should be ~0, got {}",
            c.i_eq
        );
    }
}
