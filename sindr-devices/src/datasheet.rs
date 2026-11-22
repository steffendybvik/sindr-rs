//! Datasheet figures a device model can be fitted to.
//!
//! A datasheet gives a part's characteristics as figures at test
//! conditions: a diode's forward voltage at a current, a transistor's gain
//! at a collector current, a MOSFET's threshold at a drain current. These
//! structs hold such figures as an editor's parts library stores them:
//! quantities in volts, amps, ohms or plain ratios, each figure with its
//! minimum, typical and maximum as separate optional values. The
//! `from_datasheet` constructors next to each model fit the model through
//! them: [`MosfetParams::from_datasheet`](crate::mosfet::MosfetParams::from_datasheet)
//! here, and the diode, LED, zener and BJT ones on `sindr`'s `DiodeModel`
//! and `BjtModel`.
//!
//! A figure's [`nominal`](DatasheetValue::nominal) value is what a fit
//! uses: the typical value, else the middle of minimum and maximum, else
//! whichever of the two is given.

use std::fmt;

/// One datasheet figure: its minimum, typical and maximum, any of which
/// may be missing. All in the same unit, the base unit of the quantity
/// (V, A, Ω) or a plain ratio.
///
/// Set the fields you need and fill the rest with `..Default::default()`,
/// so a field added in a later release doesn't break the literal.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DatasheetValue {
    /// Minimum.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub min: Option<f64>,
    /// Typical.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub typ: Option<f64>,
    /// Maximum.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub max: Option<f64>,
}

impl DatasheetValue {
    /// A figure with only a typical value.
    pub fn typical(typ: f64) -> Self {
        Self {
            typ: Some(typ),
            ..Self::default()
        }
    }

    /// A figure given as a range.
    pub fn range(min: f64, max: f64) -> Self {
        Self {
            min: Some(min),
            max: Some(max),
            typ: None,
        }
    }

    /// The value a fit uses: the typical value, else the middle of minimum
    /// and maximum, else whichever of them is given. `None` when the figure
    /// is empty.
    pub fn nominal(&self) -> Option<f64> {
        match (self.min, self.typ, self.max) {
            (_, Some(typ), _) => Some(typ),
            (Some(min), None, Some(max)) => Some(0.5 * (min + max)),
            (Some(v), None, None) | (None, None, Some(v)) => Some(v),
            (None, None, None) => None,
        }
    }

    /// Whether no value is given.
    pub fn is_empty(&self) -> bool {
        self.min.is_none() && self.typ.is_none() && self.max.is_none()
    }

    /// [`nominal`](Self::nominal), for the figure named `figure`.
    ///
    /// # Errors
    ///
    /// [`DatasheetError::Missing`] naming `figure` when it's empty;
    /// [`DatasheetError::Invalid`] when it isn't finite.
    pub fn require(&self, figure: &'static str) -> Result<f64, DatasheetError> {
        let v = self.nominal().ok_or(DatasheetError::Missing { figure })?;
        if v.is_finite() {
            Ok(v)
        } else {
            Err(DatasheetError::Invalid {
                figure,
                reason: "isn't a finite number",
            })
        }
    }
}

impl From<f64> for DatasheetValue {
    fn from(typ: f64) -> Self {
        Self::typical(typ)
    }
}

/// Why a model can't be fitted to datasheet figures.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum DatasheetError {
    /// A figure the fit needs is missing.
    Missing {
        /// The figure, as the struct's field names it.
        figure: &'static str,
    },
    /// A figure is out of range: a current that isn't positive, a second
    /// forward point below the first.
    Invalid {
        /// The figure.
        figure: &'static str,
        /// What's wrong with it.
        reason: &'static str,
    },
}

impl fmt::Display for DatasheetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DatasheetError::Missing { figure } => write!(f, "datasheet figure {figure} is missing"),
            DatasheetError::Invalid { figure, reason } => {
                write!(f, "datasheet figure {figure} {reason}")
            }
        }
    }
}

impl std::error::Error for DatasheetError {}

/// A positive, finite test condition.
pub(crate) fn positive(figure: &'static str, v: f64) -> Result<f64, DatasheetError> {
    if v.is_finite() && v > 0.0 {
        Ok(v)
    } else {
        Err(DatasheetError::Invalid {
            figure,
            reason: "must be positive",
        })
    }
}

/// A diode's or LED's forward characteristic: `vf` at `if_`, and
/// optionally a second point `vf2` at `if2`.
///
/// Set the fields you need and fill the rest with `..Default::default()`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DiodeDatasheet {
    /// Forward voltage Vf (V) at [`if_`](Self::if_).
    pub vf: DatasheetValue,
    /// Test current If (A), positive.
    #[cfg_attr(feature = "serde", serde(rename = "if"))]
    pub if_: f64,
    /// A second forward voltage (V), at [`if2`](Self::if2). With it the
    /// emission coefficient is fitted too.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub vf2: Option<DatasheetValue>,
    /// The second point's test current (A), positive and not `if_`.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub if2: Option<f64>,
}

impl DiodeDatasheet {
    /// `vf` (V) at `if_` (A).
    pub fn new(vf: impl Into<DatasheetValue>, if_: f64) -> Self {
        Self {
            vf: vf.into(),
            if_,
            ..Self::default()
        }
    }

    /// Adds a second point, `vf2` (V) at `if2` (A).
    pub fn with_second_point(mut self, vf2: impl Into<DatasheetValue>, if2: f64) -> Self {
        self.vf2 = Some(vf2.into());
        self.if2 = Some(if2);
        self
    }
}

/// A zener's breakdown: `vz` at the test current `iz`.
///
/// Set the fields you need and fill the rest with `..Default::default()`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ZenerDatasheet {
    /// Zener voltage Vz (V), positive, at [`iz`](Self::iz).
    pub vz: DatasheetValue,
    /// Test current Iz (A), positive: 5 mA for a BZX84, 20 mA for many
    /// 1N47xx parts.
    pub iz: f64,
}

impl ZenerDatasheet {
    /// `vz` (V) at `iz` (A).
    pub fn new(vz: impl Into<DatasheetValue>, iz: f64) -> Self {
        Self { vz: vz.into(), iz }
    }
}

/// A bipolar transistor's gain and saturation: `hfe` at `hfe_ic`, and
/// optionally `vce_sat` at `vce_sat_ic` with base current `vce_sat_ib`.
/// Currents are magnitudes, the same for an NPN and a PNP.
///
/// Set the fields you need and fill the rest with `..Default::default()`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BjtDatasheet {
    /// DC current gain hFE (ratio), at [`hfe_ic`](Self::hfe_ic).
    pub hfe: DatasheetValue,
    /// The collector current hFE is given at (A). The Ebers–Moll model's
    /// gain doesn't change with current, so this is kept for the record
    /// and doesn't move the fit; 0 when not given.
    pub hfe_ic: f64,
    /// Collector–emitter saturation voltage (V), at
    /// [`vce_sat_ic`](Self::vce_sat_ic) and
    /// [`vce_sat_ib`](Self::vce_sat_ib).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub vce_sat: Option<DatasheetValue>,
    /// The collector current Vce(sat) is given at (A).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub vce_sat_ic: Option<f64>,
    /// The base current Vce(sat) is given at (A); typically `ic / 10`.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub vce_sat_ib: Option<f64>,
}

impl BjtDatasheet {
    /// `hfe` at `ic` (A).
    pub fn new(hfe: impl Into<DatasheetValue>, ic: f64) -> Self {
        Self {
            hfe: hfe.into(),
            hfe_ic: ic,
            ..Self::default()
        }
    }

    /// Adds Vce(sat) `vce_sat` (V) at collector current `ic` and base
    /// current `ib` (A).
    pub fn with_vce_sat(mut self, vce_sat: impl Into<DatasheetValue>, ic: f64, ib: f64) -> Self {
        self.vce_sat = Some(vce_sat.into());
        self.vce_sat_ic = Some(ic);
        self.vce_sat_ib = Some(ib);
        self
    }
}

/// A MOSFET's threshold and on-resistance: gate threshold `vth` at drain
/// current `vth_id` (with the gate tied to the drain, as datasheets
/// measure it), and `rds_on` at gate–source voltage `rds_on_vgs`,
/// optionally at drain current `rds_on_id`.
///
/// Voltages and currents are read as magnitudes, so a P-channel part's
/// figures may be given with the datasheet's minus signs or without.
///
/// Set the fields you need and fill the rest with `..Default::default()`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MosfetDatasheet {
    /// Gate threshold voltage Vgs(th) (V), at [`vth_id`](Self::vth_id).
    pub vth: DatasheetValue,
    /// The drain current the threshold is given at (A): 250 µA or 1 mA on
    /// most datasheets.
    pub vth_id: f64,
    /// Drain–source on-resistance Rds(on) (Ω), at
    /// [`rds_on_vgs`](Self::rds_on_vgs).
    pub rds_on: DatasheetValue,
    /// The gate–source voltage Rds(on) is given at (V).
    pub rds_on_vgs: f64,
    /// The drain current Rds(on) is given at (A). Without it the fit uses
    /// the on-resistance at a vanishing drain current.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub rds_on_id: Option<f64>,
}

impl MosfetDatasheet {
    /// Threshold `vth` (V) at `vth_id` (A), and on-resistance `rds_on`
    /// (Ω) at `rds_on_vgs` (V).
    pub fn new(
        vth: impl Into<DatasheetValue>,
        vth_id: f64,
        rds_on: impl Into<DatasheetValue>,
        rds_on_vgs: f64,
    ) -> Self {
        Self {
            vth: vth.into(),
            vth_id,
            rds_on: rds_on.into(),
            rds_on_vgs,
            rds_on_id: None,
        }
    }

    /// Sets [`rds_on_id`](Self::rds_on_id).
    pub fn with_rds_on_id(mut self, id: f64) -> Self {
        self.rds_on_id = Some(id);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_prefers_typical_then_middle() {
        assert_eq!(DatasheetValue::typical(2.0).nominal(), Some(2.0));
        assert_eq!(DatasheetValue::range(100.0, 300.0).nominal(), Some(200.0));
        let max_only = DatasheetValue {
            max: Some(0.2),
            ..Default::default()
        };
        assert_eq!(max_only.nominal(), Some(0.2));
        assert_eq!(DatasheetValue::default().nominal(), None);
        assert!(DatasheetValue::default().require("vf").is_err());
    }
}
