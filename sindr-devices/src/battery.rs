//! Equivalent-circuit battery: open-circuit voltage by state of charge,
//! series resistance and an optional RC pair.
//!
//! ```text
//!  +  ──[ R0 ]──┬──[ R1 ]──┬──(OCV(SoC))──  −
//!               └──[ C1 ]──┘
//! ```
//!
//! The terminal voltage is `OCV(SoC) − R0·i − v1`, with `i` the discharge
//! current (out of the positive terminal) and `v1` the RC pair's voltage,
//! `C1·dv1/dt = i − v1/R1`. The pair gives the slow sag and recovery after
//! a load step. State of charge falls as `dSoC/dt = −i/(3600·capacity)`.
//!
//! Aging, temperature and current-dependent capacity (Peukert) are not
//! modelled.

/// The RC pair: the polarisation that makes a loaded cell sag over seconds
/// and recover after the load goes.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct BatteryRc {
    /// Polarisation resistance (Ω).
    pub r: f64,
    /// Polarisation capacitance (F).
    pub c: f64,
}

/// A battery's equivalent circuit.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct BatteryParams {
    /// Capacity (Ah): the charge from full (SoC 1) to empty (SoC 0).
    pub capacity: f64,
    /// Open-circuit (rest) voltage against state of charge: `(SoC, volts)`
    /// pairs with SoC rising from 0 to 1, read by linear interpolation.
    pub ocv: Vec<(f64, f64)>,
    /// Series (ohmic) resistance (Ω), positive.
    pub r_series: f64,
    /// Optional RC pair for the slow part of the sag.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub rc: Option<BatteryRc>,
}

impl BatteryParams {
    /// A cell with a flat `volts` at every state of charge.
    pub fn constant(volts: f64, capacity: f64, r_series: f64) -> Self {
        Self {
            capacity,
            ocv: vec![(0.0, volts), (1.0, volts)],
            r_series,
            rc: None,
        }
    }

    /// A single lithium-ion or lithium-polymer cell: a typical rest-voltage
    /// curve, 3.27 V empty to 4.20 V full (3.84 V at half charge), with the
    /// capacity (Ah) and series resistance (Ω) of the part. A small pouch
    /// cell is around 0.1–0.3 Ω, an 18650 around 0.03–0.05 Ω.
    pub fn lithium_ion(capacity: f64, r_series: f64) -> Self {
        let ocv = [
            (0.00, 3.27),
            (0.05, 3.61),
            (0.10, 3.69),
            (0.15, 3.71),
            (0.20, 3.73),
            (0.25, 3.75),
            (0.30, 3.77),
            (0.35, 3.79),
            (0.40, 3.80),
            (0.45, 3.82),
            (0.50, 3.84),
            (0.55, 3.85),
            (0.60, 3.87),
            (0.65, 3.91),
            (0.70, 3.95),
            (0.75, 3.98),
            (0.80, 4.02),
            (0.85, 4.08),
            (0.90, 4.11),
            (0.95, 4.15),
            (1.00, 4.20),
        ];
        Self {
            capacity,
            ocv: ocv.to_vec(),
            r_series,
            rc: None,
        }
    }

    /// A CR2032-class lithium coin cell: 225 mAh, about 3.0 V for most of
    /// its life and falling off over the last tenth, with 15 Ω series
    /// resistance and a 25 Ω, 0.2 F pulse-recovery pair. Approximate,
    /// from typical published discharge curves; a coin cell's resistance
    /// rises steeply as it empties and in the cold, which isn't modelled.
    pub fn cr2032() -> Self {
        Self {
            capacity: 0.225,
            ocv: vec![
                (0.00, 2.00),
                (0.05, 2.60),
                (0.10, 2.80),
                (0.20, 2.90),
                (0.50, 2.95),
                (0.90, 3.00),
                (1.00, 3.20),
            ],
            r_series: 15.0,
            rc: Some(BatteryRc { r: 25.0, c: 0.2 }),
        }
    }
}

/// Open-circuit voltage (V) at state of charge `soc`, interpolated along
/// the curve. Outside 0..1 it holds the end values.
pub fn open_circuit_voltage(soc: f64, params: &BatteryParams) -> f64 {
    let ocv = &params.ocv;
    let Some(&(s0, v0)) = ocv.first() else {
        return 0.0;
    };
    if soc <= s0 {
        return v0;
    }
    for pair in ocv.windows(2) {
        let ((sa, va), (sb, vb)) = (pair[0], pair[1]);
        if soc <= sb {
            return if sb > sa {
                va + (vb - va) * (soc - sa) / (sb - sa)
            } else {
                vb
            };
        }
    }
    ocv.last().map_or(v0, |&(_, v)| v)
}

/// The resistance (Ω) a battery shows at DC: `R0`, plus `R1` once the RC
/// pair has settled.
pub fn dc_resistance(params: &BatteryParams) -> f64 {
    params.r_series + params.rc.map_or(0.0, |rc| rc.r)
}

/// State of charge after `amp_hours` (Ah) is drawn from `soc`.
pub fn discharged(soc: f64, amp_hours: f64, params: &BatteryParams) -> f64 {
    soc - amp_hours / params.capacity
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_between_points() {
        let p = BatteryParams::lithium_ion(1.0, 0.1);
        assert_eq!(open_circuit_voltage(1.0, &p), 4.20);
        assert_eq!(open_circuit_voltage(0.5, &p), 3.84);
        assert!((open_circuit_voltage(0.525, &p) - 3.845).abs() < 1e-12);
    }

    #[test]
    fn holds_the_end_values_outside_0_to_1() {
        let p = BatteryParams::lithium_ion(1.0, 0.1);
        assert_eq!(open_circuit_voltage(-0.2, &p), 3.27);
        assert_eq!(open_circuit_voltage(1.3, &p), 4.20);
    }

    #[test]
    fn curves_rise_with_charge() {
        for p in [
            BatteryParams::lithium_ion(1.0, 0.1),
            BatteryParams::cr2032(),
        ] {
            assert!(p
                .ocv
                .windows(2)
                .all(|w| w[0].0 < w[1].0 && w[0].1 <= w[1].1));
        }
    }

    #[test]
    fn discharge_takes_charge_over_capacity() {
        let p = BatteryParams::cr2032();
        assert!((discharged(1.0, 0.045, &p) - 0.8).abs() < 1e-12);
    }
}
