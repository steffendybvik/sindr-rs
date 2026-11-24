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
//! | voltage regulator | `voltage` (input, from its ground pin), `current` (output), `power` |
//! | digital pin | `current` (into or out of the pin) |
//! | digital pin's port | `current`: the larger of what its pins source and what they sink |
//!
//! A digital pin's [`port`](crate::element::DigitalPin::port) is checked
//! as a part of its own: the `current` on the [`Part`](crate::Part) named
//! for the port limits the sum over its pins, as a datasheet limits the
//! current a port sources and sinks.
//!
//! A part with [`Thermal`](crate::Thermal) data also needs `tj_max`, checked
//! against the junction temperature self-heating reports. Without thermal
//! data there is no junction temperature, so a `tj_max` is unverified.
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
//! use sindr::element::{Resistor, VoltageSource};
//!
//! let mut circuit = Circuit::new(
//!     "0",
//!     vec![
//!         CircuitElement::VoltageSource(VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 12.0, waveform: None }),
//!         CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["in".into(), "0".into()], resistance: 100.0 }),
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
//!

use std::collections::HashMap;

use crate::circuit::{Circuit, CircuitElement};
use crate::design::Ratings;
use crate::element::DigitalPin;
use crate::error::SimError;
use crate::results::{ComponentResult, SimulationResult};

/// One absolute-maximum rating. Each is a magnitude; see [`Ratings`] for
/// what it measures.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
    /// Reverse voltage (V): a diode's, or a polarised capacitor's.
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
    /// Junction temperature (K), from self-heating.
    Tj,
}

impl RatingKind {
    const ALL: [RatingKind; 13] = [
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
        RatingKind::Tj,
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
            RatingKind::Tj => ratings.tj_max,
        }
    }

    /// SI unit symbol: `"W"`, `"V"`, `"A"` or `"K"`.
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
            RatingKind::Tj => "K",
        }
    }
}

/// How a part did against one rating.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Default, PartialEq)]
#[non_exhaustive]
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

/// The value of `kind` on `element` at one instant, or `None` when it
/// can't be measured on this component.
fn measure(
    element: &CircuitElement,
    kind: RatingKind,
    port: Option<&ComponentResult>,
    sample: &Sample<'_>,
) -> Option<f64> {
    let value = match kind {
        RatingKind::Tj => *sample.junction_temperatures.get(element.id())?,
        RatingKind::Power => port?.power.max(0.0),
        _ => element.device().rating(
            kind,
            &RatingProbe {
                nodes: element.nodes(),
                voltages: sample.voltages,
                port,
            },
        )?,
    };
    value.is_finite().then_some(value)
}

/// One element at one instant, for [`Device::rating`]: its port result
/// and the node voltages its terminals sit at.
///
/// [`Device::rating`]: crate::element::Device::rating
pub(crate) struct RatingProbe<'a> {
    nodes: &'a [String],
    voltages: &'a HashMap<String, f64>,
    port: Option<&'a ComponentResult>,
}

impl RatingProbe<'_> {
    /// The element's port result: the voltage across it, the current
    /// through it and its power. `None` when the solve gave none.
    pub(crate) fn port(&self) -> Option<&ComponentResult> {
        self.port
    }

    /// `|V(nodes[a]) − V(nodes[b])|`.
    pub(crate) fn between(&self, a: usize, b: usize) -> Option<f64> {
        let v = |i: usize| self.voltages.get(self.nodes.get(i)?).copied();
        Some((v(a)? - v(b)?).abs())
    }

    /// What every element's port gives: its voltage rating against the
    /// magnitude of the voltage across it, its current rating against the
    /// current through it. `None` for any other kind.
    pub(crate) fn port_rating(&self, kind: RatingKind) -> Option<f64> {
        match kind {
            RatingKind::Voltage => Some(self.port?.voltage_across.abs()),
            RatingKind::Current => Some(self.port?.current_through.abs()),
            _ => None,
        }
    }

    /// A junction's ratings: reverse voltage and forward current from its
    /// port, `nodes[0]` the anode. Others as [`port_rating`](Self::port_rating).
    pub(crate) fn junction_rating(&self, kind: RatingKind) -> Option<f64> {
        match kind {
            RatingKind::ReverseVoltage => Some((-self.port?.voltage_across).max(0.0)),
            RatingKind::ForwardCurrent => Some(self.port?.current_through.max(0.0)),
            _ => self.port_rating(kind),
        }
    }
}

/// One instant: when (`None` for DC), node voltages, per-component results.
struct Sample<'a> {
    time: Option<f64>,
    voltages: &'a HashMap<String, f64>,
    ports: HashMap<&'a str, &'a ComponentResult>,
    junction_temperatures: &'a HashMap<String, f64>,
}

impl<'a> Sample<'a> {
    fn new(
        time: Option<f64>,
        voltages: &'a HashMap<String, f64>,
        results: &'a [ComponentResult],
        junction_temperatures: &'a HashMap<String, f64>,
    ) -> Self {
        Self {
            time,
            voltages,
            ports: results.iter().map(|r| (r.id.as_str(), r)).collect(),
            junction_temperatures,
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
            .map(|s| {
                Sample::new(
                    Some(s.time),
                    &s.node_voltages,
                    &s.component_results,
                    &s.junction_temperatures,
                )
            })
            .collect(),
        None => vec![Sample::new(
            None,
            &result.node_voltages,
            &result.component_results,
            &result.junction_temperatures,
        )],
    };

    let none = Ratings::default();
    let mut checks = Vec::new();
    for element in &flat.components {
        let id = element.id();
        let part = flat.parts.get(id);
        let ratings = part.map_or(&none, |p| &p.ratings);
        let heated = part.is_some_and(|p| p.thermal.is_some());
        let needs = element.device().needed_ratings();
        for kind in RatingKind::ALL {
            let limit = kind.limit(ratings);
            let needed = needs.contains(&kind) || (kind == RatingKind::Tj && heated);
            if limit.is_none() && !needed {
                continue;
            }
            let mut worst: Option<(f64, Option<f64>)> = None;
            for sample in &samples {
                let port = sample.ports.get(id).copied();
                if let Some(value) = measure(element, kind, port, sample) {
                    if worst.is_none_or(|(w, _)| value > w) {
                        worst = Some((value, sample.time));
                    }
                }
            }
            checks.push(judge(id, kind, limit, worst));
        }
    }
    checks.extend(port_checks(&flat, &samples));
    Ok(RatingsReport { checks })
}

/// Each digital-pin port's current: the larger of the sums its pins source
/// and sink, against the `current` on the part named for the port.
fn port_checks(flat: &Circuit, samples: &[Sample<'_>]) -> Vec<RatingCheck> {
    let mut ports: std::collections::BTreeMap<&str, Vec<&str>> = Default::default();
    for element in &flat.components {
        if let CircuitElement::DigitalPin(DigitalPin {
            id,
            port: Some(port),
            ..
        }) = element
        {
            ports.entry(port).or_default().push(id);
        }
    }
    ports
        .into_iter()
        .map(|(port, pins)| {
            let limit = flat.parts.get(port).and_then(|p| p.ratings.current);
            let mut worst: Option<(f64, Option<f64>)> = None;
            for sample in samples {
                let (mut sourced, mut sunk) = (0.0, 0.0);
                for pin in &pins {
                    if let Some(r) = sample.ports.get(pin) {
                        // Into the pin is positive.
                        sunk += r.current_through.max(0.0);
                        sourced += (-r.current_through).max(0.0);
                    }
                }
                let value = f64::max(sourced, sunk);
                if value.is_finite() && worst.is_none_or(|(w, _)| value > w) {
                    worst = Some((value, sample.time));
                }
            }
            judge(port, RatingKind::Current, limit, worst)
        })
        .collect()
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
