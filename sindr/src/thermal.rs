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

/// DC fixed-point iterations before giving up. A stable balance converges
/// geometrically at the loop gain dT/dT < 1; still moving after this many
/// means a gain at or above 1, which is runaway.
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

fn temperature_mut(element: &mut CircuitElement) -> Option<&mut f64> {
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
/// `solve` is the plain DC solve of a flat circuit.
pub(crate) fn solve_dc(
    circuit: &Circuit,
    heated: &Heated,
    mut solve: impl FnMut(&Circuit) -> Result<SimulationResult, SimError>,
) -> Result<SimulationResult, SimError> {
    let mut work = circuit.clone();
    let mut temps = heated.ambient();
    for _ in 0..MAX_ITERATIONS {
        heated.apply(&mut work, &temps);
        let mut result = solve(&work)?;
        let next = heated.steady(&heated.powers(&result.component_results));
        let moved = next
            .iter()
            .zip(&temps)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        if moved < TOLERANCE_KELVIN {
            result.junction_temperatures = heated.report(&next);
            return Ok(result);
        }
        heated.check_runaway(&next)?;
        temps = next;
    }
    let (part, temperature) = heated
        .hottest(&temps)
        .map_or((String::new(), f64::NAN), |(id, t)| (id.to_string(), t));
    Err(SimError::ThermalRunaway { part, temperature })
}
