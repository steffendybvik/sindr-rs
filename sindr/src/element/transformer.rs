//! A transformer: two coupled windings, linear or with a saturating core.

use sindr_devices::inductor;

use crate::error::SimError;
use crate::mna::MnaSystem;
use crate::stamp::stamp_voltage_source_at;

use super::{Check, DcStamp, Device, TranStamp};

/// Ideal transformer with coupled inductors.
///
/// nodes: [p1, q1, p2, q2] where:
///   - [p1, q1]: primary winding (+ and - terminals)
///   - [p2, q2]: secondary winding (+ and - terminals)
///
/// Coupling: M = k * sqrt(L1 * L2) where k ∈ [0, 1).
/// k=1 is ideal (no leakage), but mathematically singular — use k<=0.999.
///
/// In DC analysis: both windings stamped as short circuits; the bias point
/// for AC, `.tf` and `.sens` shorts each with a 0 V source.
/// In AC: the coupled pair `V = jω·[[L1, M], [M, L2]]·I`.
/// In transient: coupled-inductor companion with 2 branch current unknowns,
/// each winding `V = L·dI/dt` plus the mutual term. The windings start at
/// 0 A; an initial current on a transformer is rejected.
///
/// With `saturation`, the core is modelled as a leakage inductance
/// `(1 − k)·L` on each winding and a magnetising inductance `k·L1` on the
/// primary carrying the shared core flux, driven by the magnetising
/// current `i1 + n·i2` (n = √(L2/L1), the turns ratio). That branch
/// saturates as a saturating [`Inductor`] does, so a
/// winding driven past its volt-second rating, or with a DC bias, draws
/// a magnetising current that climbs once the core saturates. Without
/// it the pair is linear and identical to the coupled inductors above.
///
/// [`Inductor`]: super::Inductor
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Transformer {
    /// Unique component identifier.
    pub id: String,
    /// Terminals `[p1, q1, p2, q2]`: primary `[p1, q1]`, secondary `[p2, q2]`.
    pub nodes: [String; 4], // [p1, q1, p2, q2]
    /// Primary winding inductance (H).
    pub l1: f64, // Primary inductance (H)
    /// Secondary winding inductance (H).
    pub l2: f64, // Secondary inductance (H)
    /// Coupling coefficient (unitless) in `[0, 0.999]`. Default 0.999 (near-ideal).
    #[cfg_attr(feature = "serde", serde(default = "default_coupling"))]
    pub k: f64, // Coupling coefficient [0, 0.999]. Default 0.999 (near-ideal).
    /// Core saturation, on the magnetising branch: `i_sat` is the
    /// magnetising current referred to the primary, and `l_sat` at
    /// most the magnetising inductance `k·l1`. `None` is a linear core.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub saturation: Option<sindr_devices::inductor::InductorSaturation>,
}

impl Device for Transformer {
    device_identity!();

    fn stamp_dc(&self, s: &mut DcStamp<'_>) -> Result<(), SimError> {
        // Both windings shorted, each a 0 V source on one of its two branch
        // rows, as `dc_equivalent` has them.
        let [p1, q1, p2, q2] = s.nodes();
        stamp_voltage_source_at(s.system, [p1, q1], 0.0, s.branch);
        stamp_voltage_source_at(s.system, [p2, q2], 0.0, s.branch + 1);
        Ok(())
    }

    /// The coupled windings' companion over the step, on its two branch
    /// rows.
    fn stamp_transient(&self, s: &mut TranStamp<'_>) -> Result<(), SimError> {
        let (k1, k2) = (s.branch, s.branch + 1);
        let h = [s.inductor(0), s.inductor(1)];
        let (z, rhs) = match &self.saturation {
            None => s
                .integ
                .coupled(transformer_inductances(self.l1, self.l2, self.k), h),
            // Its histories hold fluxes. Linearised at the Newton
            // iterate's winding currents.
            Some(sat) => {
                let at = |k: usize| s.v_prev.get(k).copied().unwrap_or(0.0);
                let i = [at(k1), at(k2)];
                let (lambda, l) = transformer_flux(self.l1, self.l2, self.k, sat, i);
                s.integ.coupled_nonlinear(l, lambda, i, h)
            }
        };
        let nodes = s.nodes();
        stamp_transformer_companion_at(s.system, nodes, z, rhs, k1, k2);
        Ok(())
    }

    /// One winding current each, primary then secondary.
    fn branches(&self) -> usize {
        2
    }

    /// Its winding currents aren't a source's current: nothing reads them
    /// by the transformer's id.
    fn exposes_branch_current(&self) -> bool {
        false
    }

    /// One per winding, primary then secondary.
    fn inductor_histories(&self) -> usize {
        2
    }

    fn is_nonlinear(&self) -> bool {
        self.saturation.is_some()
    }

    fn is_reactive(&self) -> bool {
        true
    }

    fn validate(&self) -> Result<(), SimError> {
        let check = Check::new(&self.id);
        let (l1, l2, k) = (self.l1, self.l2, self.k);
        check.positive("l1", l1, " H")?;
        check.positive("l2", l2, " H")?;
        check.finite("k", k, "")?;
        if let Some(sat) = &self.saturation {
            if k <= 0.0 {
                return Err(check.fail(format_args!(
                    "a saturating core needs coupling, but k is {k}"
                )));
            }
            check.positive("saturation.i_sat", sat.i_sat, " A")?;
            check.positive("saturation.l_sat", sat.l_sat, " H")?;
            let magnetising = k * l1;
            if sat.l_sat > magnetising {
                return Err(check.fail(format_args!(
                    "saturation.l_sat {} H must not exceed the magnetising \
                     inductance k·l1 = {magnetising} H",
                    sat.l_sat,
                )));
            }
        }
        Ok(())
    }
}

#[cfg(feature = "serde")]
fn default_coupling() -> f64 {
    0.999
}

/// A transformer's inductance matrix `[[L1, M], [M, L2]]`, M = k·√(L1·L2).
fn transformer_inductances(l1: f64, l2: f64, k: f64) -> [[f64; 2]; 2] {
    let m = k * (l1 * l2).sqrt();
    [[l1, m], [m, l2]]
}

/// Winding fluxes (Wb) and incremental inductance matrix `dλ/di` (H) of a
/// transformer with a saturating core, at winding currents `i` (A).
///
/// Each winding has leakage `(1 − k)·L`; the magnetising inductance `k·L1`
/// on the primary carries `im = i1 + n·i2`, n = √(L2/L1), and saturates as
/// `sat`. Its flux links the primary once and the secondary n times:
/// `λ1 = (1 − k)·L1·i1 + λm(im)`, `λ2 = (1 − k)·L2·i2 + n·λm(im)`. With a
/// linear λm = k·L1·im these are the coupled inductors of
/// [`transformer_inductances`].
pub(crate) fn transformer_flux(
    l1: f64,
    l2: f64,
    k: f64,
    sat: &inductor::InductorSaturation,
    i: [f64; 2],
) -> ([f64; 2], [[f64; 2]; 2]) {
    let n = (l2 / l1).sqrt();
    let lm = k * l1;
    let im = i[0] + n * i[1];
    let flux_m = inductor::flux(im, lm, sat);
    let lm_inc = inductor::incremental_inductance(im, lm, sat);
    let (ll1, ll2) = ((1.0 - k) * l1, (1.0 - k) * l2);
    (
        [ll1 * i[0] + flux_m, ll2 * i[1] + n * flux_m],
        [
            [ll1 + lm_inc, n * lm_inc],
            [n * lm_inc, ll2 + n * n * lm_inc],
        ],
    )
}

/// Stamp a coupled-inductor transformer as two branch-current unknowns,
/// its terminals `[p1, q1, p2, q2]` as matrix indices (`None` is ground).
///
/// `k1`/`k2` are the MNA rows of the primary and secondary currents, each
/// flowing from the winding's first terminal to its second. Each winding's
/// branch row is `V(p) − V(q) + Σ_j z[k][j]·I_j = rhs[k]`, with `z` and `rhs`
/// from the integration formula (`Integrator::coupled`). For backward Euler,
/// `z = −L/dt` and `rhs = −(L/dt)·I_prev`, i.e. `V = L·(I − I_prev)/dt`.
fn stamp_transformer_companion_at(
    system: &mut MnaSystem,
    nodes: [Option<usize>; 4],
    z: [[f64; 2]; 2],
    rhs: [f64; 2],
    k1: usize,
    k2: usize,
) {
    let rows = [k1, k2];
    for (w, &k) in rows.iter().enumerate() {
        let p = nodes[2 * w];
        let q = nodes[2 * w + 1];
        if let Some(pi) = p {
            system.a[(pi, k)] += 1.0;
            system.a[(k, pi)] += 1.0;
        }
        if let Some(qi) = q {
            system.a[(qi, k)] -= 1.0;
            system.a[(k, qi)] -= 1.0;
        }
        for (j, &col) in rows.iter().enumerate() {
            system.a[(k, col)] += z[w][j];
        }
        system.b[k] += rhs[w];
    }
}
