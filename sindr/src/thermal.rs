//! Self-heating: junction temperature from dissipated power, fed back into
//! the devices.
//!
//! A component whose [`Part`](crate::Part) has [`Thermal`] data runs at
//! `T = ambient + P·Rθ` at DC. The operating point is iterated until power
//! and temperature agree. In a transient each such part carries a thermal RC
//! state, `Cθ·dT/dt = P − (T − ambient)/Rθ`, advanced after every accepted
//! step (backward Euler), and the next step solves at the new temperature.
//!
//! Diodes, LEDs, Zeners, Schottkys, photodiodes, BJTs and thermistors take
//! the temperature back (IS(T) and kT/q, or the thermistor's resistance).
//! Other parts (resistors, MOSFETs, IGBTs, …) have no temperature in their
//! model, so their junction temperature is reported but changes nothing.

use std::collections::HashMap;

use crate::circuit::{Circuit, CircuitElement};
use crate::design::Thermal;
use crate::error::SimError;
use crate::results::{ComponentResult, SimulationResult};

/// A junction that gets this hot while the solve is still looking for a
/// balance has run away: silicon models mean nothing past ~600 K, and a
/// real part is long gone.
pub(crate) const RUNAWAY_KELVIN: f64 = 1000.0;

/// DC balance: temperatures agree with their power to this (K).
const TOLERANCE_KELVIN: f64 = 1e-3;

/// DC balance iterations before giving up. Negative feedback is damped to
/// a secant step and settles in a handful; positive feedback converges
/// geometrically at its loop gain below 1. Still moving after this many
/// means a gain at or above 1 that never reaches the runaway limit, which
/// is reported as runaway too.
const MAX_ITERATIONS: usize = 1000;

struct Entry {
    index: usize,
    id: String,
    rth: f64,
    cth: Option<f64>,
    ambient: f64,
    feeds_back: bool,
}

/// The heated parts of a flat circuit.
pub(crate) struct Heated {
    entries: Vec<Entry>,
}

pub(crate) fn temperature_mut(element: &mut CircuitElement) -> Option<&mut f64> {
    match element {
        CircuitElement::Diode { temperature, .. }
        | CircuitElement::Led { temperature, .. }
        | CircuitElement::ZenerDiode { temperature, .. }
        | CircuitElement::SchottkyDiode { temperature, .. }
        | CircuitElement::Photodiode { temperature, .. }
        | CircuitElement::Bjt { temperature, .. }
        | CircuitElement::Thermistor { temperature, .. } => Some(temperature),
        _ => None,
    }
}

fn positive(id: &str, what: &str, value: f64) -> Result<f64, SimError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(SimError::InvalidComponent(format!(
            "part `{id}`: thermal {what} must be positive and finite (got {value})"
        )))
    }
}

impl Heated {
    /// The components of flat `circuit` with thermal data, or `None` when
    /// there are none.
    pub(crate) fn from_circuit(circuit: &Circuit) -> Result<Option<Self>, SimError> {
        let mut entries = Vec::new();
        for (index, element) in circuit.components.iter().enumerate() {
            let id = element.id();
            let Some(thermal) = circuit.parts.get(id).and_then(|p| p.thermal) else {
                continue;
            };
            entries.push(Self::entry(index, id, &thermal, element)?);
        }
        Ok((!entries.is_empty()).then_some(Self { entries }))
    }

    fn entry(
        index: usize,
        id: &str,
        thermal: &Thermal,
        element: &CircuitElement,
    ) -> Result<Entry, SimError> {
        let rth = thermal.resistance().ok_or_else(|| {
            SimError::InvalidComponent(format!(
                "part `{id}`: thermal needs rth_ja, or rth_jc and rth_ca"
            ))
        })?;
        Ok(Entry {
            index,
            id: id.to_string(),
            rth: positive(id, "resistance", rth)?,
            cth: thermal
                .cth
                .map(|c| positive(id, "capacitance cth", c))
                .transpose()?,
            ambient: positive(id, "ambient", thermal.ambient)?,
            feeds_back: temperature_mut(&mut element.clone()).is_some(),
        })
    }

    /// Every heated part at its ambient: the state at switch-on.
    pub(crate) fn ambient(&self) -> Vec<f64> {
        self.entries.iter().map(|e| e.ambient).collect()
    }

    /// Set each heated device's temperature in `circuit`, which must be the
    /// flat circuit this was built from (or a clone of it).
    pub(crate) fn apply(&self, circuit: &mut Circuit, temps: &[f64]) {
        for (entry, &t) in self.entries.iter().zip(temps) {
            if let Some(temperature) = circuit
                .components
                .get_mut(entry.index)
                .and_then(temperature_mut)
            {
                *temperature = t;
            }
        }
    }

    /// Power each heated part dissipates (W); a part delivering power
    /// doesn't cool below ambient.
    pub(crate) fn powers(&self, results: &[ComponentResult]) -> Vec<f64> {
        let by_id: HashMap<&str, f64> = results.iter().map(|r| (r.id.as_str(), r.power)).collect();
        self.entries
            .iter()
            .map(|e| {
                by_id
                    .get(e.id.as_str())
                    .copied()
                    .filter(|p| p.is_finite())
                    .unwrap_or(0.0)
                    .max(0.0)
            })
            .collect()
    }

    /// Steady-state temperatures for `powers`: `ambient + P·Rθ`.
    pub(crate) fn steady(&self, powers: &[f64]) -> Vec<f64> {
        self.entries
            .iter()
            .zip(powers)
            .map(|(e, p)| e.ambient + p * e.rth)
            .collect()
    }

    /// Temperatures after `dt` seconds at `powers`, from `temps`. Backward
    /// Euler on the thermal RC, which is stable for any step; a part with
    /// no `cth` jumps straight to its steady temperature.
    pub(crate) fn advance(&self, temps: &[f64], powers: &[f64], dt: f64) -> Vec<f64> {
        self.entries
            .iter()
            .zip(temps.iter().zip(powers))
            .map(|(e, (&t, &p))| match e.cth {
                Some(c) => {
                    let a = dt / c;
                    (t + a * (p + e.ambient / e.rth)) / (1.0 + a / e.rth)
                }
                None => e.ambient + p * e.rth,
            })
            .collect()
    }

    /// `ThermalRunaway` for the hottest device past [`RUNAWAY_KELVIN`]
    /// whose temperature feeds back, if any.
    pub(crate) fn check_runaway(&self, temps: &[f64]) -> Result<(), SimError> {
        match self.hottest(temps) {
            Some((id, t)) if t > RUNAWAY_KELVIN => Err(SimError::ThermalRunaway {
                part: id.to_string(),
                temperature: t,
            }),
            _ => Ok(()),
        }
    }

    /// `ThermalRunaway` for a part with feedback that sits at
    /// [`RUNAWAY_KELVIN`] (where [`relax`](Self::relax) holds it) and whose
    /// power there still asks for a hotter junction: no balance below the
    /// limit. Reports the temperature it asked for.
    fn check_runaway_at_limit(&self, temps: &[f64], wanted: &[f64]) -> Result<(), SimError> {
        let runaway = self
            .entries
            .iter()
            .zip(temps.iter().zip(wanted))
            .filter(|(e, (&t, &w))| e.feeds_back && t >= RUNAWAY_KELVIN && (w > t || w.is_nan()))
            .map(|(e, (_, &w))| (e.id.as_str(), if w.is_finite() { w } else { f64::MAX }))
            .max_by(|a, b| a.1.total_cmp(&b.1));
        match runaway {
            Some((id, t)) => Err(SimError::ThermalRunaway {
                part: id.to_string(),
                temperature: t,
            }),
            None => Ok(()),
        }
    }

    /// One DC balance step from `temps` toward `wanted`, the temperatures
    /// their power asks for. A part whose loop gain `k` is negative steps
    /// `(wanted − T)/(1 − k)`, a fraction of the way, which lands on the
    /// balance when the gain is constant; otherwise it takes the full step.
    /// A part with feedback is capped at [`RUNAWAY_KELVIN`], past which its
    /// device model means nothing.
    fn relax(&self, temps: &[f64], wanted: &[f64], gains: &[f64]) -> Vec<f64> {
        self.entries
            .iter()
            .zip(temps.iter().zip(wanted.iter().zip(gains)))
            .map(|(e, (&t, (&w, &k)))| {
                let damping = if k < 0.0 { 1.0 / (1.0 - k) } else { 1.0 };
                let next = t + damping * (w - t);
                // A NaN (a solve that blew up) is held at the limit too.
                if e.feeds_back && (next > RUNAWAY_KELVIN || next.is_nan()) {
                    RUNAWAY_KELVIN
                } else {
                    next
                }
            })
            .collect()
    }

    fn hottest<'a>(&'a self, temps: &[f64]) -> Option<(&'a str, f64)> {
        self.entries
            .iter()
            .zip(temps)
            .filter(|(e, _)| e.feeds_back)
            .map(|(e, &t)| (e.id.as_str(), if t.is_finite() { t } else { f64::MAX }))
            .max_by(|a, b| a.1.total_cmp(&b.1))
    }

    /// Junction temperature per heated part, by id.
    pub(crate) fn report(&self, temps: &[f64]) -> HashMap<String, f64> {
        self.entries
            .iter()
            .zip(temps)
            .map(|(e, &t)| (e.id.clone(), t))
            .collect()
    }
}

/// DC operating point with self-heating: solve, heat each part by its
/// power, solve again at the new temperatures, until they agree.
///
/// Each heated part's balance is `g(T) = F(T) − T = 0`, where
/// `F(T) = ambient + P(T)·Rθ` is the temperature its power at `T` asks
/// for. The plain update `T ← F(T)` only settles when the loop gain
/// `k = dF/dT` lies in (−1, 1). Negative feedback stronger than −1 (a
/// heated NTC fed from a current source, whose power falls as it warms)
/// overshoots further on every pass, so each part instead steps
/// `T ← T + g/(1 − k)`: a secant step on `g`, with `k` estimated from the
/// last two passes. That is Newton's method on `g` per part, with the
/// other parts' coupling through the circuit taken as it stands.
///
/// The step is only ever shortened (`k < 0`); with positive feedback the
/// plain update is kept, so a part whose power keeps climbing still climbs
/// at the pace it always did and a stable one settles where it did before.
/// A part with feedback is held at [`RUNAWAY_KELVIN`]; still asking to be
/// hotter there is a genuine runaway.
///
/// `solve` is the plain DC solve of a flat circuit.
pub(crate) fn solve_dc(
    circuit: &Circuit,
    heated: &Heated,
    mut solve: impl FnMut(&Circuit) -> Result<SimulationResult, SimError>,
) -> Result<SimulationResult, SimError> {
    let mut work = circuit.clone();
    let mut temps = heated.ambient();
    // The previous pass's temperatures and the F they gave, for the secant
    // estimate of each part's loop gain.
    let mut previous: Option<(Vec<f64>, Vec<f64>)> = None;
    let mut gains = vec![0.0; temps.len()];
    for _ in 0..MAX_ITERATIONS {
        heated.apply(&mut work, &temps);
        let mut result = solve(&work)?;
        let wanted = heated.steady(&heated.powers(&result.component_results));
        let moved = wanted
            .iter()
            .zip(&temps)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        if moved < TOLERANCE_KELVIN {
            result.junction_temperatures = heated.report(&wanted);
            return Ok(result);
        }
        heated.check_runaway_at_limit(&temps, &wanted)?;

        if let Some((last_temps, last_wanted)) = &previous {
            for (i, gain) in gains.iter_mut().enumerate() {
                let dt = temps[i] - last_temps[i];
                // Too small a move gives no slope; keep the last estimate.
                if dt.abs() > 1e-9 {
                    *gain = (wanted[i] - last_wanted[i]) / dt;
                }
            }
        }
        let next = heated.relax(&temps, &wanted, &gains);
        previous = Some((temps, wanted));
        temps = next;
    }
    let (part, temperature) = heated
        .hottest(&temps)
        .map_or((String::new(), f64::NAN), |(id, t)| (id.to_string(), t));
    Err(SimError::ThermalRunaway { part, temperature })
}
