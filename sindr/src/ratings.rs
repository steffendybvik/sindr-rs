//! Absolute-maximum ratings checks: did any part go past its datasheet
//! limits?
//!
//! [`check_ratings`] compares a [`SimulationResult`] against the
//! [`Ratings`] on each component's [`Part`](crate::Part) (looked up by
//! component id in [`Circuit::parts`]). It looks at the DC operating point,
//! or at every step of a transient, since peaks (inrush, inductive
//! kickback, start-up) are what kill parts. For each part and rating it
//! reports the worst value, when it happened, the limit and the margin left.
//!
//! Each component kind has ratings it needs, which are checked whether set
//! or not. A needed rating that isn't set comes back
//! [`RatingStatus::Unverified`], never as a pass:
//!
//! | Component | Needs |
//! |---|---|
//! | resistor, potentiometer, thermistor, photoresistor | `power` |
//! | capacitor | `voltage` |
//! | inductor | `current` |
//! | diode, Schottky, LED, photodiode | `reverse_voltage`, `forward_current`, `power` |
//! | Zener | `power` |
//! | varactor | `reverse_voltage` |
//! | BJT | `vce`, `vcb`, `veb`, `ic`, `power` |
//! | MOSFET, JFET | `vds`, `vgs`, `id`, `power` |
//! | IGBT | `vce`, `vgs` (gate–emitter), `ic`, `power` |
//!
//! Any other rating a part sets is checked as well. `power`, `voltage` and
//! `current` work on every component, measured on the port its
//! [`ComponentResult`] reports (Vce and Ic for a BJT). A rating the
//! component has no way to measure, such as `vds` on a resistor, is
//! unverified. Transistor power is `|V·I|` on that port, so a BJT's base
//! dissipation is left out.
//!
//! ```
//! use sindr::{check_ratings, solve_circuit, Circuit, CircuitElement, Part, Ratings};
//!
//! let mut circuit = Circuit::new(
//!     "0",
//!     vec![
//!         CircuitElement::VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 12.0, waveform: None },
//!         CircuitElement::Resistor { id: "R1".into(), nodes: ["in".into(), "0".into()], resistance: 100.0 },
//!     ],
//! );
//! // A 1/8 W resistor across 12 V dissipates 1.44 W.
//! circuit.parts.insert(
//!     "R1".into(),
//!     Part { ratings: Ratings { power: Some(0.125), ..Ratings::default() }, ..Part::default() },
//! );
//!
//! let result = solve_circuit(&circuit).unwrap();
//! let report = check_ratings(&circuit, &result).unwrap();
//! let r1 = &report.exceeded()[0];
//! assert_eq!(r1.part, "R1");
//! assert!((r1.worst.unwrap() - 1.44).abs() < 1e-9);
//! assert!(!report.is_safe());
//! ```

use std::collections::HashMap;

use crate::circuit::{Circuit, CircuitElement};
use crate::design::Ratings;
use crate::error::SimError;
use crate::results::{ComponentResult, SimulationResult};

/// One absolute-maximum rating. Each is a magnitude; see [`Ratings`] for
/// what it measures.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum RatingKind {
    /// Power dissipation (W).
    Power,
    /// Voltage across the part (V).
    Voltage,
    /// Current through the part (A).
    Current,
    /// Diode reverse voltage (V).
    ReverseVoltage,
    /// Diode forward current (A).
    ForwardCurrent,
    /// Collector–emitter voltage (V).
    Vce,
    /// Collector–base voltage (V).
    Vcb,
    /// Emitter–base voltage (V).
    Veb,
    /// Collector current (A).
    Ic,
    /// Drain–source voltage (V).
    Vds,
    /// Gate–source (or IGBT gate–emitter) voltage (V).
    Vgs,
    /// Drain current (A).
    Id,
}

impl RatingKind {
    const ALL: [RatingKind; 12] = [
        RatingKind::Power,
        RatingKind::Voltage,
        RatingKind::Current,
        RatingKind::ReverseVoltage,
        RatingKind::ForwardCurrent,
        RatingKind::Vce,
        RatingKind::Vcb,
        RatingKind::Veb,
        RatingKind::Ic,
        RatingKind::Vds,
        RatingKind::Vgs,
        RatingKind::Id,
    ];

    /// The limit `ratings` sets for this kind, if any.
    pub fn limit(self, ratings: &Ratings) -> Option<f64> {
        match self {
            RatingKind::Power => ratings.power,
            RatingKind::Voltage => ratings.voltage,
            RatingKind::Current => ratings.current,
            RatingKind::ReverseVoltage => ratings.reverse_voltage,
            RatingKind::ForwardCurrent => ratings.forward_current,
            RatingKind::Vce => ratings.vce,
            RatingKind::Vcb => ratings.vcb,
            RatingKind::Veb => ratings.veb,
            RatingKind::Ic => ratings.ic,
            RatingKind::Vds => ratings.vds,
            RatingKind::Vgs => ratings.vgs,
            RatingKind::Id => ratings.id,
        }
    }

    /// SI unit symbol: `"W"`, `"V"` or `"A"`.
    pub fn unit(self) -> &'static str {
        match self {
            RatingKind::Power => "W",
            RatingKind::Voltage
            | RatingKind::ReverseVoltage
            | RatingKind::Vce
            | RatingKind::Vcb
            | RatingKind::Veb
            | RatingKind::Vds
            | RatingKind::Vgs => "V",
            RatingKind::Current | RatingKind::ForwardCurrent | RatingKind::Ic | RatingKind::Id => {
                "A"
            }
        }
    }
}

/// How a part did against one rating.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RatingStatus {
    /// The worst value stayed at or under the limit.
    Within,
    /// The worst value went over the limit.
    Exceeded,
    /// Can't tell: the rating isn't set, or sindr can't measure it on this
    /// component.
    Unverified,
}

/// One part checked against one rating.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct RatingCheck {
    /// Component id, flattened (`X1.Q1`).
    pub part: String,
    /// Which rating.
    pub rating: RatingKind,
    /// The limit, in the rating's unit. `None` when the part doesn't set it.
    pub limit: Option<f64>,
    /// The worst (largest) value the part saw, in the rating's unit. `None`
    /// when sindr can't measure it on this component.
    pub worst: Option<f64>,
    /// When the worst value happened (s). `None` for a DC operating point.
    pub time: Option<f64>,
    /// Margin left, as a percentage of the limit: `100·(limit − worst)/limit`.
    /// Negative when exceeded; `None` when unverified.
    pub margin_percent: Option<f64>,
    /// Within, exceeded or unverified.
    pub status: RatingStatus,
}

/// Every rating checked on every part, in circuit order.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RatingsReport {
    /// One entry per part and rating.
    pub checks: Vec<RatingCheck>,
}

impl RatingsReport {
    /// Checks whose worst value went over the limit.
    pub fn exceeded(&self) -> Vec<&RatingCheck> {
        self.with_status(RatingStatus::Exceeded)
    }

    /// Checks that couldn't be verified.
    pub fn unverified(&self) -> Vec<&RatingCheck> {
        self.with_status(RatingStatus::Unverified)
    }

    /// `true` only when every check is within its limit: nothing exceeded
    /// and nothing unverified.
    pub fn is_safe(&self) -> bool {
        self.checks.iter().all(|c| c.status == RatingStatus::Within)
    }

    /// The checks for one part.
    pub fn for_part<'a>(&'a self, part: &'a str) -> impl Iterator<Item = &'a RatingCheck> + 'a {
        self.checks.iter().filter(move |c| c.part == part)
    }

    fn with_status(&self, status: RatingStatus) -> Vec<&RatingCheck> {
        self.checks.iter().filter(|c| c.status == status).collect()
    }
}

/// Ratings a component kind needs, checked even when unset.
fn needed(element: &CircuitElement) -> &'static [RatingKind] {
    use RatingKind::*;
    match element {
        CircuitElement::Resistor { .. }
        | CircuitElement::Potentiometer { .. }
        | CircuitElement::Thermistor { .. }
        | CircuitElement::Photoresistor { .. } => &[Power],
        CircuitElement::Capacitor { .. } => &[Voltage],
        CircuitElement::Inductor { .. } => &[Current],
        CircuitElement::Diode { .. }
        | CircuitElement::SchottkyDiode { .. }
        | CircuitElement::Led { .. }
        | CircuitElement::Photodiode { .. } => &[ReverseVoltage, ForwardCurrent, Power],
        CircuitElement::ZenerDiode { .. } => &[Power],
        CircuitElement::Varactor { .. } => &[ReverseVoltage],
        CircuitElement::Bjt { .. } => &[Vce, Vcb, Veb, Ic, Power],
        CircuitElement::Mosfet { .. } | CircuitElement::Jfet { .. } => &[Vds, Vgs, Id, Power],
        CircuitElement::Igbt { .. } => &[Vce, Vgs, Ic, Power],
        _ => &[],
    }
}

fn is_diode(element: &CircuitElement) -> bool {
    matches!(
        element,
        CircuitElement::Diode { .. }
            | CircuitElement::SchottkyDiode { .. }
            | CircuitElement::Led { .. }
            | CircuitElement::Photodiode { .. }
            | CircuitElement::ZenerDiode { .. }
            | CircuitElement::Varactor { .. }
    )
}

/// The value of `kind` on `element` at one instant, or `None` when it
/// can't be measured on this component.
fn measure(
    element: &CircuitElement,
    kind: RatingKind,
    port: Option<&ComponentResult>,
    voltages: &HashMap<String, f64>,
) -> Option<f64> {
    let v = |i: usize| voltages.get(element.nodes().get(i)?).copied();
    let between = |a: usize, b: usize| Some((v(a)? - v(b)?).abs());
    let value = match (kind, element) {
        (RatingKind::Power, _) => port?.power.max(0.0),
        (RatingKind::Voltage, _) => port?.voltage_across.abs(),
        (RatingKind::Current, _) => port?.current_through.abs(),
        (RatingKind::ReverseVoltage, e) if is_diode(e) => (-port?.voltage_across).max(0.0),
        (RatingKind::ForwardCurrent, e) if is_diode(e) => port?.current_through.max(0.0),
        // [base, collector, emitter] and [gate, collector, emitter]
        (RatingKind::Vce, CircuitElement::Bjt { .. } | CircuitElement::Igbt { .. }) => {
            between(1, 2)?
        }
        (RatingKind::Vcb, CircuitElement::Bjt { .. }) => between(1, 0)?,
        (RatingKind::Veb, CircuitElement::Bjt { .. }) => between(2, 0)?,
        (RatingKind::Ic, CircuitElement::Bjt { .. } | CircuitElement::Igbt { .. }) => {
            port?.current_through.abs()
        }
        // [gate, drain, source]
        (RatingKind::Vds, CircuitElement::Mosfet { .. } | CircuitElement::Jfet { .. }) => {
            between(1, 2)?
        }
        (
            RatingKind::Vgs,
            CircuitElement::Mosfet { .. }
            | CircuitElement::Jfet { .. }
            | CircuitElement::Igbt { .. },
        ) => between(0, 2)?,
        (RatingKind::Id, CircuitElement::Mosfet { .. } | CircuitElement::Jfet { .. }) => {
            port?.current_through.abs()
        }
        _ => return None,
    };
    value.is_finite().then_some(value)
}

/// One instant: when (`None` for DC), node voltages, per-component results.
struct Sample<'a> {
    time: Option<f64>,
    voltages: &'a HashMap<String, f64>,
    ports: HashMap<&'a str, &'a ComponentResult>,
}

impl<'a> Sample<'a> {
    fn new(
        time: Option<f64>,
        voltages: &'a HashMap<String, f64>,
        results: &'a [ComponentResult],
    ) -> Self {
        Self {
            time,
            voltages,
            ports: results.iter().map(|r| (r.id.as_str(), r)).collect(),
        }
    }
}

/// Check every component of `circuit` against the [`Ratings`] on its
/// [`Part`](crate::Part), using `result` from any solve of that circuit.
///
/// With a transient in `result`, every timestep is checked (the first is
/// the starting operating point); otherwise the DC operating point is. See
/// the [module docs](self) for which ratings each component needs.
///
/// # Errors
///
/// [`SimError::Design`] when the circuit's hierarchy can't be flattened.
pub fn check_ratings(
    circuit: &Circuit,
    result: &SimulationResult,
) -> Result<RatingsReport, SimError> {
    let flat = circuit.flattened()?;
    let samples: Vec<Sample<'_>> = match &result.transient {
        Some(tran) => tran
            .timesteps
            .iter()
            .map(|s| Sample::new(Some(s.time), &s.node_voltages, &s.component_results))
            .collect(),
        None => vec![Sample::new(
            None,
            &result.node_voltages,
            &result.component_results,
        )],
    };

    let none = Ratings::default();
    let mut checks = Vec::new();
    for element in &flat.components {
        let id = element.id();
        let ratings = flat.parts.get(id).map_or(&none, |p| &p.ratings);
        let needs = needed(element);
        for kind in RatingKind::ALL {
            let limit = kind.limit(ratings);
            if limit.is_none() && !needs.contains(&kind) {
                continue;
            }
            let mut worst: Option<(f64, Option<f64>)> = None;
            for sample in &samples {
                let port = sample.ports.get(id).copied();
                if let Some(value) = measure(element, kind, port, sample.voltages) {
                    if worst.is_none_or(|(w, _)| value > w) {
                        worst = Some((value, sample.time));
                    }
                }
            }
            checks.push(judge(id, kind, limit, worst));
        }
    }
    Ok(RatingsReport { checks })
}

fn judge(
    part: &str,
    rating: RatingKind,
    limit: Option<f64>,
    worst: Option<(f64, Option<f64>)>,
) -> RatingCheck {
    let (status, margin_percent) = match (limit, worst) {
        (Some(limit), Some((value, _))) if limit > 0.0 => {
            let status = if value > limit {
                RatingStatus::Exceeded
            } else {
                RatingStatus::Within
            };
            (status, Some(100.0 * (limit - value) / limit))
        }
        // A zero rating can't be divided by; anything above it is over.
        (Some(limit), Some((value, _))) if value > limit => (RatingStatus::Exceeded, None),
        (Some(_), Some(_)) => (RatingStatus::Within, None),
        _ => (RatingStatus::Unverified, None),
    };
    RatingCheck {
        part: part.to_string(),
        rating,
        limit,
        worst: worst.map(|(w, _)| w),
        time: worst.and_then(|(_, t)| t),
        margin_percent,
        status,
    }
}
