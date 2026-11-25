//! JFET (Junction Field-Effect Transistor) device model.
//!
//! Shichman–Hodges square-law model. N-channel and P-channel are both
//! supported via [`JfetKind`]; the P-channel case mirrors N-channel through
//! sign conventions on `vgs` and `vds`.
//!
//! At each Newton–Raphson iteration, [`jfet_companion`] returns a
//! linearised contribution that an MNA solver stamps for the [gate, drain,
//! source] terminals.

const GMIN: f64 = 1e-12; // minimum conductance floor

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
/// JFET channel polarity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JfetKind {
    /// N-channel JFET (pinch-off voltage `Vp < 0`).
    #[cfg_attr(feature = "serde", serde(rename = "nchannel"))]
    NChannel,
    /// P-channel JFET (pinch-off voltage `Vp > 0`); sign-mirrored from N-channel.
    #[cfg_attr(feature = "serde", serde(rename = "pchannel"))]
    PChannel,
}

/// Linearised JFET companion model — the output of one Newton–Raphson
/// iteration ready for MNA stamping.
pub struct JfetCompanion {
    /// Transconductance `∂Id/∂Vgs` (S).
    pub gm: f64,
    /// Drain–source conductance `∂Id/∂Vds` (S).
    pub gds: f64,
    /// Equivalent Norton current for the MNA right-hand side (A), drain →
    /// source, in the N-channel frame (a P-channel caller negates it).
    pub i_eq: f64,
}

/// Compute JFET companion at operating point (vgs, vds).
/// Nodes order: [gate, drain, source] — same as MOSFET convention.
/// N-channel: Vp < 0 (typically -2 to -6 V), Idss > 0 (A). `vgs`, `vds` and
/// `vp` are in V.
///
/// The channel is symmetric: with the drain below the source (`vds < 0` in
/// the N-channel frame) the drain acts as the source, and the model is
/// evaluated with the two swapped (`vgd`, `−vds`), as SPICE does in reverse
/// mode. `gm` and `gds` stay the partial derivatives with respect to the
/// labelled `vgs` and `vds`, so in the ohmic region `gm = 2·Idss·Vds/Vp²`
/// keeps its sign and is negative when the drain is below the source.
///
/// P-channel is evaluated in the mirrored N-channel frame (`-vgs`, `-vds`,
/// `-|vp|`): `gm` and `gds` carry over to the physical device unchanged, but
/// `i_eq` is the mirrored frame's, so the physical Norton current is
/// `-i_eq`.
pub fn jfet_companion(vgs: f64, vds: f64, kind: JfetKind, idss: f64, vp: f64) -> JfetCompanion {
    let (vgs, vds, idss, vp) = match kind {
        JfetKind::NChannel => (vgs, vds, idss, vp),
        JfetKind::PChannel => (-vgs, -vds, idss, -vp.abs()), // sign-flip for P-channel
    };

    let on = if vds < 0.0 {
        // Id(vgs, vds) = −F(vgs − vds, −vds).
        forward_drain_current(vgs - vds, -vds, idss, vp).map(|(id, gm, gds)| (-id, -gm, gm + gds))
    } else {
        forward_drain_current(vgs, vds, idss, vp)
    };
    let Some((id, gm, gds)) = on else {
        // Cutoff region: Id = 0
        return JfetCompanion {
            gm: GMIN,
            gds: GMIN,
            i_eq: 0.0,
        };
    };
    JfetCompanion {
        gm,
        gds,
        i_eq: id - gm * vgs - gds * vds,
    }
}

/// `(Id, gm, gds)` in the N-channel frame with `vds >= 0`, or `None` in
/// cutoff.
fn forward_drain_current(vgs: f64, vds: f64, idss: f64, vp: f64) -> Option<(f64, f64, f64)> {
    if vgs <= vp {
        return None;
    }

    let vgs_norm = 1.0 - vgs / vp; // (1 - Vgs/Vp)

    if vds >= vgs - vp {
        // Saturation region: Id = Idss * (1 - Vgs/Vp)²
        let id = idss * vgs_norm * vgs_norm;
        // gm = dId/dVgs = 2*Idss*(1 - Vgs/Vp)*(-1/Vp) = -2*Idss*vgs_norm/Vp
        let gm = (-2.0 * idss * vgs_norm / vp).max(GMIN);
        Some((id, gm, GMIN))
    } else {
        // Triode (ohmic) region: Id = Idss/Vp² * (2*(Vgs-Vp)*Vds - Vds²)
        let vp_sq = vp * vp;
        let id = idss / vp_sq * (2.0 * (vgs - vp) * vds - vds * vds);
        // gm = dId/dVgs = 2*Idss*Vds/Vp². Not floored: gm is off the
        // diagonal, so a GMIN floor would only make it disagree with Id.
        let gm = 2.0 * idss * vds / vp_sq;
        // gds = dId/dVds = 2*Idss*(Vgs-Vp-Vds)/Vp²
        let gds = (2.0 * idss * (vgs - vp - vds) / vp_sq).max(GMIN);
        Some((id, gm, gds))
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

    /// In the ohmic region with the drain below the source, raising Vgs
    /// makes Id more negative: gm matches the central difference of Id.
    #[test]
    fn triode_gm_follows_id_with_the_drain_below_the_source() {
        let id = |vgs: f64, vds: f64| {
            let c = jfet_companion(vgs, vds, JfetKind::NChannel, 10e-3, -2.0);
            c.i_eq + c.gm * vgs + c.gds * vds
        };
        let (vgs, vds, h) = (-0.5, -0.5, 1e-6);
        let c = jfet_companion(vgs, vds, JfetKind::NChannel, 10e-3, -2.0);
        let fd = (id(vgs + h, vds) - id(vgs - h, vds)) / (2.0 * h);
        assert!(c.gm < 0.0, "gm {}", c.gm);
        assert!((c.gm - fd).abs() < 1e-9, "gm {} vs {fd}", c.gm);
    }

    /// gm and gds are the partial derivatives of Id in every region, the
    /// drain below the source included, for both channel types.
    #[test]
    fn conductances_match_finite_difference() {
        for (kind, sign) in [(JfetKind::NChannel, 1.0), (JfetKind::PChannel, -1.0)] {
            let id = |vgs: f64, vds: f64| {
                let c = jfet_companion(vgs, vds, kind, 10e-3, sign * -2.0);
                sign * c.i_eq + c.gm * vgs + c.gds * vds
            };
            let h = 1e-7;
            for (vgs, vds) in [
                (-0.5, 3.0),
                (-0.5, 0.5),
                (-0.5, -0.5),
                (-3.0, -2.0),
                (-3.0, -4.0),
            ] {
                let (vgs, vds) = (sign * vgs, sign * vds);
                let c = jfet_companion(vgs, vds, kind, 10e-3, sign * -2.0);
                let fd_gm = (id(vgs + h, vds) - id(vgs - h, vds)) / (2.0 * h);
                let fd_gds = (id(vgs, vds + h) - id(vgs, vds - h)) / (2.0 * h);
                assert!((c.gm - fd_gm).abs() < 1e-7, "gm {} vs {fd_gm}", c.gm);
                assert!((c.gds - fd_gds).abs() < 1e-7, "gds {} vs {fd_gds}", c.gds);
            }
        }
    }

    /// Swapping drain and source negates the drain current. With the gate
    /// pinched off against the source but not the drain, the channel
    /// conducts from the source end.
    #[test]
    fn reverse_mode_mirrors_forward_mode() {
        let id = |vgs: f64, vds: f64| {
            let c = jfet_companion(vgs, vds, JfetKind::NChannel, 10e-3, -2.0);
            c.i_eq + c.gm * vgs + c.gds * vds
        };
        for (vgs, vds) in [(-0.5, 3.0), (-0.5, 0.5), (0.0, 1.0)] {
            assert!((id(vgs - vds, -vds) + id(vgs, vds)).abs() < 1e-15);
        }
        // Vgs = −3 V is past pinch-off, but Vgd = 0: Idss flows source → drain.
        assert!((id(-3.0, -3.0) + 10e-3).abs() < 1e-12);
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
