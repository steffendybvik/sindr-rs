//! Models fitted to datasheet figures.
//!
//! The figures are [`sindr_devices::datasheet`]'s, re-exported here:
//! [`DiodeDatasheet`], [`ZenerDatasheet`], [`BjtDatasheet`] and
//! [`MosfetDatasheet`], each figure a [`DatasheetValue`] with optional
//! minimum, typical and maximum. The constructors sit on the models they
//! build:
//!
//! | Constructor | Fits | Through |
//! |---|---|---|
//! | [`DiodeModel::from_datasheet`] | IS (N = 1), or IS and N with a second point | `vf` at `if_` (and `vf2` at `if2`) |
//! | [`DiodeModel::led_from_datasheet`] | IS (N = 2) or IS and N, and EG for −2 mV/K | `vf` at `if_` (and `vf2` at `if2`) |
//! | [`DiodeModel::zener_from_datasheet`] | BV of sindr's zener card | `vz` at `iz` |
//! | [`BjtModel::from_datasheet`] | the element's `bf`, and BR | `hfe`; `vce_sat` at `vce_sat_ic` / `vce_sat_ib` |
//! | [`MosfetParams::from_datasheet`] | KP and VTO | `vth` at `vth_id`; `rds_on` at `rds_on_vgs` |
//!
//! Figures are read at 27 °C, the temperature sindr's model values are
//! given at.

pub use sindr_devices::datasheet::{
    BjtDatasheet, DatasheetError, DatasheetValue, DiodeDatasheet, MosfetDatasheet, ZenerDatasheet,
};
pub use sindr_devices::mosfet::MosfetParams;

use crate::circuit::{BjtModel, DiodeModel};
use crate::device_params;
use sindr_devices::diode::{diode_current, V_T};

fn positive(figure: &'static str, v: f64) -> Result<f64, DatasheetError> {
    if v.is_finite() && v > 0.0 {
        Ok(v)
    } else {
        Err(DatasheetError::Invalid {
            figure,
            reason: "must be positive",
        })
    }
}

/// IS and N through the datasheet's forward point(s); `n` when there is
/// one point.
fn forward_fit(ds: &DiodeDatasheet, n: f64) -> Result<(f64, f64), DatasheetError> {
    let vf = positive("vf", ds.vf.require("vf")?)?;
    let i1 = positive("if", ds.if_)?;
    let n = match (ds.vf2, ds.if2) {
        (None, None) => n,
        (Some(vf2), Some(i2)) => {
            let vf2 = positive("vf2", vf2.require("vf2")?)?;
            let i2 = positive("if2", i2)?;
            let guess = (vf2 - vf) / (V_T * (i2 / i1).ln());
            if !(guess.is_finite() && guess > 0.0) {
                return Err(DatasheetError::Invalid {
                    figure: "vf2",
                    reason: "must rise with current from the first point",
                });
            }
            // The guess drops Shockley's −1; bisect for the N that keeps
            // it, so both points hold exactly. The current ratio between
            // the two voltages falls as N rises.
            let ratio = |n: f64| (vf2 / (n * V_T)).exp_m1() / (vf / (n * V_T)).exp_m1();
            let want = i2 / i1;
            let (mut lo, mut hi) = (guess * 0.5, guess * 2.0);
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if (ratio(mid) > want) == (vf2 > vf) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            0.5 * (lo + hi)
        }
        (None, Some(_)) => return Err(DatasheetError::Missing { figure: "vf2" }),
        (Some(_), None) => return Err(DatasheetError::Missing { figure: "if2" }),
    };
    let is = i1 / (vf / (n * V_T)).exp_m1();
    if !(is.is_finite() && is > 0.0) {
        return Err(DatasheetError::Invalid {
            figure: "vf",
            reason: "gives no usable saturation current",
        });
    }
    Ok((is, n))
}

impl DiodeModel {
    /// A diode through its datasheet forward voltage: IS so that it
    /// conducts `if_` at `vf`, with N = 1 and no series resistance. With a
    /// second point (`vf2` at `if2`), N is fitted too, so the diode passes
    /// through both. Other parameters are the defaults.
    ///
    /// ```
    /// use sindr::circuit::DiodeModel;
    /// use sindr::datasheet::DiodeDatasheet;
    ///
    /// // 1N4148: 0.62 V at 1 mA, 0.72 V at 10 mA (typical).
    /// let ds = DiodeDatasheet::new(0.62, 1e-3).with_second_point(0.72, 10e-3);
    /// let m = DiodeModel::from_datasheet(&ds).unwrap();
    /// assert!(m.n > 1.5 && m.n < 2.0);
    /// ```
    ///
    /// # Errors
    ///
    /// [`DatasheetError`] when `vf` or `if_` is missing or not positive,
    /// when only half of the second point is given, or when `vf2` doesn't
    /// rise with current from `vf`.
    pub fn from_datasheet(ds: &DiodeDatasheet) -> Result<Self, DatasheetError> {
        let (is, n) = forward_fit(ds, 1.0)?;
        Ok(Self {
            is,
            n,
            ..Self::default()
        })
    }

    /// An LED through its datasheet forward voltage: as
    /// [`from_datasheet`](Self::from_datasheet), with N = 2 (sindr's LED
    /// presets' emission coefficient) when there is one point, and EG set
    /// so `vf` falls 2 mV/K at `if_`, as sindr's LED colours do. Use it as
    /// a [`CircuitElement::Diode`](crate::CircuitElement::Diode)'s model.
    ///
    /// ```
    /// use sindr::circuit::DiodeModel;
    /// use sindr::datasheet::DiodeDatasheet;
    ///
    /// // An amber LED: 2.05 V at 20 mA.
    /// let m = DiodeModel::led_from_datasheet(&DiodeDatasheet::new(2.05, 20e-3)).unwrap();
    /// assert_eq!(m.n, 2.0);
    /// ```
    ///
    /// # Errors
    ///
    /// As [`from_datasheet`](Self::from_datasheet).
    pub fn led_from_datasheet(ds: &DiodeDatasheet) -> Result<Self, DatasheetError> {
        let (is, n) = forward_fit(ds, 2.0)?;
        Ok(Self {
            is,
            n,
            eg: device_params::led_eg(ds.vf.require("vf")?),
            ..Self::default()
        })
    }

    /// The zener card sindr stamps a
    /// [`CircuitElement::ZenerDiode`](crate::CircuitElement::ZenerDiode)
    /// as (IBV = 5 mA, RS = 1 Ω), with BV solved so that it carries `iz`
    /// in reverse at `vz`. Its `bv` is the element's `vz`: a datasheet's
    /// Vz at a test current other than 5 mA, or with the 1 Ω drop, shifts
    /// it a little.
    ///
    /// ```
    /// use sindr::circuit::DiodeModel;
    /// use sindr::datasheet::ZenerDatasheet;
    ///
    /// // 1N4733A: 5.1 V at 49 mA.
    /// let m = DiodeModel::zener_from_datasheet(&ZenerDatasheet::new(5.1, 49e-3)).unwrap();
    /// let vz = m.bv.unwrap();
    /// assert!(vz < 5.1 && vz > 4.9);
    /// ```
    ///
    /// # Errors
    ///
    /// [`DatasheetError`] when `vz` or `iz` is missing or not positive.
    pub fn zener_from_datasheet(ds: &ZenerDatasheet) -> Result<Self, DatasheetError> {
        let vz = positive("vz", ds.vz.require("vz")?)?;
        let iz = positive("iz", ds.iz)?;
        // Reverse current at −vz with breakdown `bv`; falls as `bv` rises.
        let excess = |bv: f64| -diode_current(-vz, &device_params::zener_diode(bv, 300.15)) - iz;
        let mut lo = (vz - iz * 1.0 - 1.0).max(1e-3);
        let mut hi = vz + 2.0;
        if excess(lo) <= 0.0 || excess(hi) >= 0.0 {
            return Err(DatasheetError::Invalid {
                figure: "iz",
                reason: "can't be reached at vz by sindr's zener",
            });
        }
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if excess(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Ok(device_params::zener_model(0.5 * (lo + hi)))
    }
}

impl BjtModel {
    /// A transistor through its datasheet gain and saturation voltage:
    /// the element's `bf` (returned first) is `hfe`, and the model's BR is
    /// solved so that at collector current `vce_sat_ic` and base current
    /// `vce_sat_ib` the collector–emitter voltage is `vce_sat`. Without
    /// Vce(sat), BR stays at 1. Other parameters are the defaults, so the
    /// Early effect stays off and the gain is the same at every current.
    ///
    /// The Ebers–Moll saturation voltage, with NF = NR = 1, is
    /// `VT·ln((1 + (1 + β)/BR) / (1 − β/BF))` with `β = Ic/Ib`, solved here
    /// for BR. sindr's transistor has no collector resistance, so a
    /// datasheet's Vce(sat) (which includes it) gives a low BR.
    ///
    /// ```
    /// use sindr::circuit::BjtModel;
    /// use sindr::datasheet::{BjtDatasheet, DatasheetValue};
    ///
    /// // 2N3904: hFE 100–300 at 10 mA, Vce(sat) 0.2 V max at 10 mA / 1 mA.
    /// let ds = BjtDatasheet::new(DatasheetValue::range(100.0, 300.0), 10e-3)
    ///     .with_vce_sat(DatasheetValue { max: Some(0.2), ..Default::default() }, 10e-3, 1e-3);
    /// let (bf, model) = BjtModel::from_datasheet(&ds).unwrap();
    /// assert_eq!(bf, 200.0);
    /// assert!(model.br > 0.0);
    /// ```
    ///
    /// # Errors
    ///
    /// [`DatasheetError`] when `hfe` is missing or not positive, when
    /// Vce(sat) comes without both currents, when `ic / ib` isn't below
    /// `hfe` (the part wouldn't saturate), or when `vce_sat` is too low to
    /// reach at that overdrive.
    pub fn from_datasheet(ds: &BjtDatasheet) -> Result<(f64, Self), DatasheetError> {
        let bf = positive("hfe", ds.hfe.require("hfe")?)?;
        let Some(vce_sat) = ds.vce_sat else {
            return Ok((bf, Self::default()));
        };
        let vce_sat = positive("vce_sat", vce_sat.require("vce_sat")?)?;
        let ic = positive(
            "vce_sat_ic",
            ds.vce_sat_ic.ok_or(DatasheetError::Missing {
                figure: "vce_sat_ic",
            })?,
        )?;
        let ib = positive(
            "vce_sat_ib",
            ds.vce_sat_ib.ok_or(DatasheetError::Missing {
                figure: "vce_sat_ib",
            })?,
        )?;
        let beta = ic / ib;
        if beta >= bf {
            return Err(DatasheetError::Invalid {
                figure: "vce_sat_ib",
                reason: "is too small to saturate the transistor at hfe",
            });
        }
        let denom = (1.0 - beta / bf) * (vce_sat / V_T).exp() - 1.0;
        let br = (1.0 + beta) / denom;
        if !(br.is_finite() && br > 0.0) {
            return Err(DatasheetError::Invalid {
                figure: "vce_sat",
                reason: "is too low for an Ebers–Moll transistor at this overdrive",
            });
        }
        Ok((
            bf,
            Self {
                br,
                ..Self::default()
            },
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::{Bjt, CurrentSource, VoltageSource};
    use crate::{solve_op, BjtKind, Circuit, CircuitElement};
    use sindr_devices::diode::T_NOM;

    fn forward_current(model: &DiodeModel, v: f64) -> f64 {
        diode_current(v, &device_params::diode(Some(model), T_NOM))
    }

    /// One point: the diode conducts `if` at `vf` to 1e-9 relative.
    #[test]
    fn diode_through_one_point() {
        let m = DiodeModel::from_datasheet(&DiodeDatasheet::new(0.7, 10e-3)).unwrap();
        assert_eq!(m.n, 1.0);
        let i = forward_current(&m, 0.7);
        assert!((i / 10e-3 - 1.0).abs() < 1e-9, "{i}");
    }

    /// Two points: both reproduced to 1e-9 relative.
    #[test]
    fn diode_through_two_points() {
        let ds = DiodeDatasheet::new(0.62, 1e-3).with_second_point(0.72, 10e-3);
        let m = DiodeModel::from_datasheet(&ds).unwrap();
        for (v, i) in [(0.62, 1e-3), (0.72, 10e-3)] {
            let got = forward_current(&m, v);
            assert!((got / i - 1.0).abs() < 1e-9, "{v} V: {got}");
        }
        let falling = DiodeDatasheet::new(0.72, 1e-3).with_second_point(0.62, 10e-3);
        assert!(DiodeModel::from_datasheet(&falling).is_err());
        assert!(DiodeModel::from_datasheet(&DiodeDatasheet::default()).is_err());
    }

    /// An LED's point is reproduced to 1e-9 relative, and its Vf falls
    /// 2 mV/K there (within 0.1 mV/K).
    #[test]
    fn led_through_its_point_with_the_led_tempco() {
        let m = DiodeModel::led_from_datasheet(&DiodeDatasheet::new(2.05, 20e-3)).unwrap();
        let i = forward_current(&m, 2.05);
        assert!((i / 20e-3 - 1.0).abs() < 1e-9, "{i}");
        let vf = |t: f64| {
            let p = device_params::diode(Some(&m), t);
            p.n * p.vt() * (20e-3 / p.is).ln_1p()
        };
        let tempco = (vf(T_NOM + 1.0) - vf(T_NOM - 1.0)) / 2.0;
        assert!((tempco + 2e-3).abs() < 1e-4, "{tempco} V/K");
    }

    /// The zener carries `iz` at `vz` in reverse, to 1e-6 relative.
    #[test]
    fn zener_through_vz_at_iz() {
        for (vz, iz) in [(5.1, 49e-3), (3.3, 5e-3), (12.0, 1e-3)] {
            let m = DiodeModel::zener_from_datasheet(&ZenerDatasheet::new(vz, iz)).unwrap();
            let p = device_params::zener_diode(m.bv.unwrap(), T_NOM);
            let got = -diode_current(-vz, &p);
            assert!((got / iz - 1.0).abs() < 1e-6, "{vz} V at {iz} A: {got}");
        }
    }

    fn bjt_op(bf: f64, model: BjtModel, ib: f64, collector: CircuitElement) -> (f64, f64) {
        let circuit = Circuit::new(
            "0",
            vec![
                CircuitElement::CurrentSource(CurrentSource {
                    id: "ib".into(),
                    nodes: ["0".into(), "b".into()],
                    current: ib,
                    waveform: None,
                }),
                collector,
                CircuitElement::Bjt(Bjt {
                    id: "q".into(),
                    nodes: ["b".into(), "c".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf,
                    temperature: T_NOM,
                    parasitic_caps: None,
                    model: Some(model),
                }),
            ],
        );
        let r = solve_op(&circuit).unwrap();
        let q = r.component_results.iter().find(|c| c.id == "q").unwrap();
        (r.node_voltages["c"], q.current_through)
    }

    /// In the active region Ic/Ib is `hfe` to 1e-6 relative; driven at the
    /// datasheet's Ic and Ib, Vce is `vce_sat` to within 0.5 mV.
    #[test]
    fn bjt_through_hfe_and_vce_sat() {
        let ds = BjtDatasheet::new(DatasheetValue::range(100.0, 300.0), 10e-3).with_vce_sat(
            DatasheetValue {
                max: Some(0.2),
                ..Default::default()
            },
            10e-3,
            1e-3,
        );
        let (bf, model) = BjtModel::from_datasheet(&ds).unwrap();
        assert_eq!(bf, 200.0);

        let supply = CircuitElement::VoltageSource(VoltageSource {
            id: "vc".into(),
            nodes: ["c".into(), "0".into()],
            voltage: 5.0,
            waveform: None,
        });
        let ib = 20e-6;
        let r = solve_op(&Circuit::new(
            "0",
            vec![
                CircuitElement::CurrentSource(CurrentSource {
                    id: "ib".into(),
                    nodes: ["0".into(), "b".into()],
                    current: ib,
                    waveform: None,
                }),
                supply,
                CircuitElement::Bjt(Bjt {
                    id: "q".into(),
                    nodes: ["b".into(), "c".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf,
                    temperature: T_NOM,
                    parasitic_caps: None,
                    model: Some(model),
                }),
            ],
        ))
        .unwrap();
        let ic = -r
            .component_results
            .iter()
            .find(|c| c.id == "vc")
            .unwrap()
            .current_through;
        assert!((ic.abs() / ib / 200.0 - 1.0).abs() < 1e-6, "Ic {ic}");

        let forced = CircuitElement::CurrentSource(CurrentSource {
            id: "ic".into(),
            nodes: ["0".into(), "c".into()],
            current: 10e-3,
            waveform: None,
        });
        let (vce, _) = bjt_op(bf, model, 1e-3, forced);
        assert!((vce - 0.2).abs() < 5e-4, "Vce(sat) {vce}");
    }

    #[test]
    fn bjt_refuses_figures_it_cant_meet() {
        // Ic/Ib above hFE: never saturates.
        let ds = BjtDatasheet::new(50.0, 10e-3).with_vce_sat(0.2, 10e-3, 0.1e-3);
        assert!(BjtModel::from_datasheet(&ds).is_err());
        // Vce(sat) half given.
        let mut half = BjtDatasheet::new(100.0, 10e-3);
        half.vce_sat = Some(0.2.into());
        assert!(BjtModel::from_datasheet(&half).is_err());
        // hFE alone keeps the default model.
        let (bf, m) = BjtModel::from_datasheet(&BjtDatasheet::new(150.0, 2e-3)).unwrap();
        assert_eq!((bf, m), (150.0, BjtModel::default()));
    }
}
