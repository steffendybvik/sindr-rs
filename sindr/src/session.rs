//! Transient sessions: a transient that runs as far as it's told, takes
//! changes to its inputs between steps, and stops where a signal crosses a
//! level.
//!
//! [`solve_transient`](crate::solve_transient) runs from 0 to `tstop` in one
//! call. A [`TransientSession`] holds the same run open, for a circuit
//! paired with something outside it: firmware on an emulated
//! microcontroller driving a pin, a user dragging a potentiometer, a sensor
//! whose surroundings change. It follows FMI 3.0 co-simulation: the caller
//! advances the circuit to a time ([`advance_to`](TransientSession::advance_to),
//! FMI's `doStep`), changes inputs at the time reached
//! ([`set_source`](TransientSession::set_source),
//! [`set_param`](TransientSession::set_param)), and can ask to be stopped
//! early at an event ([`run_until`](TransientSession::run_until)), as FMI's
//! early return.
//!
//! A change takes effect at the time the session has reached: the step
//! that got there saw the old value, the next one sees the new. The
//! integration restarts there with a short backward-Euler step, as it does
//! at a waveform's corner, so no step integrates across the change.
//!
//! Between calls the session keeps everything a run sets up (the flat
//! circuit, the node map, where each element sits in the matrix, its
//! histories), so a call costs only the steps it takes. Nothing needs
//! threads: in a browser, run it where the rest of the simulation runs.
//!
//! ```
//! use sindr::{Circuit, CircuitElement, Edge, Event, TransientConfig, TransientSession};
//! use sindr::element::{Capacitor, Resistor, VoltageSource};
//!
//! // A pin driving an RC: 1 kΩ into 1 µF, τ = 1 ms.
//! let circuit = Circuit::new("0", vec![
//!     CircuitElement::VoltageSource(VoltageSource {
//!         id: "PIN".into(), nodes: ["pin".into(), "0".into()], voltage: 0.0, waveform: None,
//!     }),
//!     CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["pin".into(), "out".into()],
//!         resistance: 1e3 }),
//!     CircuitElement::Capacitor(Capacitor { id: "C1".into(), nodes: ["out".into(), "0".into()],
//!         capacitance: 1e-6 }),
//! ]);
//! let mut sim = TransientSession::new(&circuit, &TransientConfig::new(10e-3, 10e-6)).unwrap();
//!
//! // Firmware drives the pin high at 1 ms.
//! sim.advance_to(1e-3).unwrap();
//! sim.set_source("PIN", 5.0).unwrap();
//!
//! // Stop where the output crosses half the rail: τ·ln 2 after the edge.
//! let hit = sim
//!     .run_until(10e-3, &[Event::crosses("out", 2.5, Edge::Rising)])
//!     .unwrap()
//!     .expect("the output crosses 2.5 V");
//! assert!((hit.time - (1e-3 + 1e-3 * 2f64.ln())).abs() < 5e-6);
//! assert!((sim.voltage("out").unwrap() - 2.5).abs() < 1e-3);
//!
//! // Every step taken, to plot or log.
//! let steps: Vec<_> = sim.drain_steps().collect();
//! assert!((steps.last().unwrap().time - sim.time()).abs() < 1e-15);
//! ```

use std::collections::HashMap;

use crate::circuit::{Circuit, CircuitElement, SpdtPosition};
use crate::element::{
    CurrentSource, DigitalPin, LogicLevel, Photodiode, Photoresistor, PinMode, Potentiometer, Pull,
    Pushbutton, Resistor, Switch, VoltageSource,
};
use crate::error::{ConfigKind, SimError};
use crate::results::{TimestepSnapshot, TransientStats};
use crate::solver::SolverOptions;
use crate::transient::{
    waveform_breakpoints, Flow, StepState, Stepper, TransientConfig, TransientProgress,
};

/// A transient held open between calls: advance it, change its inputs,
/// stop it on events. See the [module docs](self).
///
/// The session runs from 0 to the config's
/// [`tstop`](TransientConfig::tstop), which bounds it: it can't be advanced
/// past that. Steps from [`tstart`](TransientConfig::tstart) on are kept
/// until [`drain_steps`](Self::drain_steps) takes them, unless
/// [`keep_steps`](Self::keep_steps) says not to.
///
/// After an error other than [`SimError::Cancelled`] the session can't go
/// on, and every later call that steps it fails.
pub struct TransientSession {
    stepper: Stepper,
    /// Solver-circuit element index by id.
    index: HashMap<String, usize>,
    /// The caller's circuit flattened, before lowering, when it has
    /// composites whose parts each step folds back into them.
    composites: Option<Circuit>,
    /// Steps taken and not yet drained.
    steps: Vec<TimestepSnapshot>,
    /// The latest step, when it isn't the last of `steps`.
    last: Option<TimestepSnapshot>,
    keep: bool,
    /// Set by an error the session can't step past.
    failed: Option<f64>,
    /// The circuit's digital pins, as last set.
    pins: Vec<Pin>,
    /// What each pin's input reads at the time reached.
    readings: Vec<LogicLevel>,
}

/// A digital pin the session drives and reads.
struct Pin {
    pin: DigitalPin,
    /// The pad's and the ground terminal's solution rows; `None` for
    /// ground.
    rows: [Option<usize>; 2],
}

impl Pin {
    fn volts(&self, stepper: &Stepper) -> f64 {
        let v = |row: Option<usize>| row.map_or(0.0, |i| stepper.state.solution[i]);
        v(self.rows[0]) - v(self.rows[1])
    }
}

/// A value inside the circuit that a session can change between steps:
/// what a user or the environment does to a part. A source's value is
/// [`set_source`](TransientSession::set_source).
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum Param {
    /// A resistor's resistance (Ω).
    Resistance(f64),
    /// A potentiometer's wiper position, 0.0 (top) to 1.0 (bottom).
    Position(f64),
    /// Whether a switch or pushbutton conducts.
    Closed(bool),
    /// The throw an SPDT switch's common terminal is on.
    Throw(SpdtPosition),
    /// A photoresistor's light level, 0.0 (dark) to 1.0 (bright).
    LightLevel(f64),
    /// A photodiode's incident optical power (W).
    Irradiance(f64),
    /// The temperature (K) of a thermistor, or of a junction device
    /// without self-heating.
    Temperature(f64),
    /// A digital pin's output stage: firmware's pin mode.
    PinMode(PinMode),
    /// The level a digital pin's output drives, `true` for high.
    Level(bool),
    /// A digital pin's weak pull resistor.
    Pull(Pull),
    /// A DC motor's load torque (N·m), opposing rotation when positive.
    LoadTorque(f64),
}

/// Which way a signal must cross an [`Event`]'s level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Edge {
    /// From below the level to at or above it.
    Rising,
    /// From above the level to at or below it.
    Falling,
    /// Either way.
    Either,
}

/// Something [`run_until`](TransientSession::run_until) stops at: a node
/// voltage crossing a level, as a comparator or an interrupt pin sees it.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct Event {
    /// The node watched.
    pub node: String,
    /// The level (V).
    pub level: f64,
    /// Which way the node must cross it.
    pub edge: Edge,
    /// How closely the crossing is located (s): the session stops no more
    /// than this after it. `None` means 10⁻⁴ of the config's
    /// [`tstep`](TransientConfig::tstep).
    pub tolerance: Option<f64>,
}

impl Event {
    /// `node` crossing `level` volts on `edge`.
    pub fn crosses(node: impl Into<String>, level: f64, edge: Edge) -> Self {
        Self {
            node: node.into(),
            level,
            edge,
            tolerance: None,
        }
    }

    /// Locate the crossing to `seconds`.
    pub fn within(mut self, seconds: f64) -> Self {
        self.tolerance = Some(seconds);
        self
    }

    fn crossed(&self, before: f64, after: f64) -> bool {
        let up = before < self.level && after >= self.level;
        let down = before > self.level && after <= self.level;
        match self.edge {
            Edge::Rising => up,
            Edge::Falling => down,
            Edge::Either => up || down,
        }
    }
}

/// Where [`run_until`](TransientSession::run_until) stopped.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct EventHit {
    /// Which of the events given, by index.
    pub event: usize,
    /// The time the session stopped at (s), which is
    /// [`time`](TransientSession::time): the crossing lies within the
    /// event's tolerance before it.
    pub time: f64,
}

/// The run at one accepted step, to go back to.
struct Saved {
    state: StepState,
    steps: usize,
    last: Option<TimestepSnapshot>,
    readings: Vec<LogicLevel>,
}

/// A step an event crossed in: the run before it, and the watched values
/// either side.
type Bracket = (Saved, Vec<f64>, Vec<f64>);

/// The events of one [`run_until`](TransientSession::run_until), with
/// their nodes resolved.
struct Watch<'a> {
    events: &'a [Event],
    /// Each event's node as a solution row; `None` for ground.
    rows: Vec<Option<usize>>,
    tolerances: Vec<f64>,
}

impl Watch<'_> {
    fn values(&self, stepper: &Stepper) -> Vec<f64> {
        self.rows
            .iter()
            .map(|row| row.map_or(0.0, |i| stepper.state.solution[i]))
            .collect()
    }

    /// The event that crosses first between `before` and `after`, with
    /// how far through the step it crosses, by linear interpolation.
    fn first(&self, before: &[f64], after: &[f64]) -> Option<(usize, f64)> {
        self.events
            .iter()
            .enumerate()
            .filter(|(k, e)| e.crossed(before[*k], after[*k]))
            // A crossing means the two sides differ, so the span isn't 0.
            .map(|(k, e)| {
                (
                    k,
                    ((e.level - before[k]) / (after[k] - before[k])).clamp(0.0, 1.0),
                )
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }
}

/// How many times [`TransientSession::run_until`] narrows a crossing by
/// interpolation before it falls back to halving: a node that bends hard
/// inside the bracket can keep interpolation from closing in.
const INTERPOLATED_NARROWINGS: usize = 8;

impl TransientSession {
    /// A session of `circuit` at t = 0, from its initial conditions or
    /// operating point as `config` says.
    ///
    /// # Errors
    ///
    /// As [`solve_transient`](crate::solve_transient) before its first step:
    /// an invalid config or circuit, or an operating point that won't
    /// solve.
    pub fn new(circuit: &Circuit, config: &TransientConfig) -> Result<Self, SimError> {
        Self::with_options(circuit, config, &SolverOptions::default())
    }

    /// [`new`](Self::new) with `opts`.
    pub(crate) fn with_options(
        circuit: &Circuit,
        config: &TransientConfig,
        opts: &SolverOptions,
    ) -> Result<Self, SimError> {
        let flat = crate::solver_circuit(circuit)?.into_owned();
        config.validate(opts.max_transient_steps)?;
        crate::validation::validate_circuit(&flat)?;
        let composites = circuit
            .flattened()
            .ok()
            .map(std::borrow::Cow::into_owned)
            .filter(|c| crate::composite::StepFolder::new(c).is_some());
        let index = flat
            .components
            .iter()
            .enumerate()
            .map(|(k, c)| (c.id().to_string(), k))
            .collect();
        let stepper = Stepper::new(flat, config, opts)?;
        let row = |node: &str| stepper.node_map.index(node);
        let pins: Vec<Pin> = composites
            .iter()
            .flat_map(|c| &c.components)
            .filter_map(|c| match c {
                CircuitElement::DigitalPin(pin) => Some(Pin {
                    rows: [row(&pin.nodes[0]), row(&pin.nodes[2])],
                    pin: pin.clone(),
                }),
                _ => None,
            })
            .collect();
        let readings = pins
            .iter()
            .map(|p| p.pin.reads(p.volts(&stepper), None))
            .collect();
        Ok(Self {
            stepper,
            index,
            composites,
            steps: Vec::new(),
            last: None,
            keep: true,
            failed: None,
            pins,
            readings,
        })
    }

    /// The time reached (s).
    pub fn time(&self) -> f64 {
        self.stepper.state.time
    }

    /// Step counts so far.
    pub fn stats(&self) -> TransientStats {
        self.stepper.state.stats
    }

    /// The voltage (V) at `node` at the time reached, or `None` for a node
    /// the circuit doesn't have. Read straight from the solution, so it's
    /// cheap to call after every step.
    pub fn voltage(&self, node: &str) -> Option<f64> {
        if node == self.stepper.circuit.ground_node {
            return Some(0.0);
        }
        let i = self.stepper.node_map.index(node)?;
        Some(self.stepper.state.solution[i])
    }

    /// The current (A) through component `id` at the latest step, with
    /// the sign of its [`ComponentResult`](crate::ComponentResult); `None`
    /// before the first step or for an id the circuit doesn't have.
    pub fn current(&self, id: &str) -> Option<f64> {
        self.last_step()?
            .component_results
            .iter()
            .find(|c| c.id == id)
            .map(|c| c.current_through)
    }

    /// What digital pin `id`'s input buffer reads at the time reached,
    /// against its `vil` and `vih`, or `None` for an id that isn't a
    /// digital pin. An output reads its own pad, as a microcontroller's
    /// input register does.
    ///
    /// The reading has the thresholds' hysteresis: between them it holds
    /// what the last step read. It's taken after every step, so a pad that
    /// crosses a threshold and comes back within one step isn't seen; to
    /// catch an edge, [`run_until`](Self::run_until) an
    /// [`Event`] at the threshold.
    pub fn logic_level(&self, id: &str) -> Option<LogicLevel> {
        let k = self.pins.iter().position(|p| p.pin.id == id)?;
        Some(self.readings[k])
    }

    /// The latest step taken, kept or not; `None` before the first.
    pub fn last_step(&self) -> Option<&TimestepSnapshot> {
        self.last.as_ref().or(self.steps.last())
    }

    /// Whether to keep each step for [`drain_steps`](Self::drain_steps)
    /// (the default). A long session that only reads
    /// [`voltage`](Self::voltage) can turn it off, so nothing builds up.
    pub fn keep_steps(&mut self, keep: bool) {
        self.keep = keep;
    }

    /// Take the steps kept since the last call, oldest first.
    pub fn drain_steps(&mut self) -> std::vec::Drain<'_, TimestepSnapshot> {
        if self.last.is_none() {
            self.last = self.steps.last().cloned();
        }
        self.steps.drain(..)
    }

    /// Set source `id` (a voltage source's volts, a current source's amps)
    /// to `value` from the time reached on. A waveform it had is dropped:
    /// from now on the caller drives it.
    ///
    /// # Errors
    ///
    /// [`SimError::UnknownComponent`] for an id the circuit doesn't have,
    /// and [`SimError::InvalidConfig`] for one that isn't a voltage or
    /// current source, or a value that isn't finite.
    pub fn set_source(&mut self, id: &str, value: f64) -> Result<(), SimError> {
        if !value.is_finite() {
            return Err(SimError::config(
                ConfigKind::TransientSession,
                format!("`{id}`: a source value must be finite (got {value})"),
            ));
        }
        let k = self.element_index(id)?;
        let mut element = self.stepper.circuit.components[k].clone();
        match &mut element {
            CircuitElement::VoltageSource(VoltageSource {
                voltage, waveform, ..
            }) => {
                *voltage = value;
                *waveform = None;
            }
            CircuitElement::CurrentSource(CurrentSource {
                current, waveform, ..
            }) => {
                *current = value;
                *waveform = None;
            }
            _ => {
                return Err(SimError::config(
                    ConfigKind::TransientSession,
                    format!("`{id}`: not a voltage or current source"),
                ))
            }
        }
        self.replace(k, element)
    }

    /// Change `param` of component `id` from the time reached on.
    ///
    /// # Errors
    ///
    /// [`SimError::UnknownComponent`] for an id the circuit doesn't have,
    /// and [`SimError::InvalidConfig`] for a part `param` doesn't apply to,
    /// or a temperature on a part whose temperature self-heating sets; a
    /// value the part refuses fails as
    /// [`validate_circuit`](crate::validate_circuit) would fail it. A
    /// refused change changes nothing.
    pub fn set_param(&mut self, id: &str, param: Param) -> Result<(), SimError> {
        let wrong = || {
            Err(SimError::config(
                ConfigKind::TransientSession,
                format!("`{id}`: {param:?} doesn't apply to this component"),
            ))
        };
        if let Param::Throw(position) = param {
            // An SPDT switch is solved as its two throws.
            let throws = [("a", SpdtPosition::A), ("b", SpdtPosition::B)];
            let parts: Vec<_> = throws
                .iter()
                .filter_map(|(suffix, pos)| {
                    let k = *self.index.get(&format!("{id}#{suffix}"))?;
                    Some((k, *pos == position))
                })
                .collect();
            if parts.len() != 2 {
                return match self.index.contains_key(id) {
                    true => wrong(),
                    false => self.element_index(id).map(|_| ()),
                };
            }
            for (k, closed) in parts {
                let mut element = self.stepper.circuit.components[k].clone();
                if let CircuitElement::Switch(Switch { closed: c, .. }) = &mut element {
                    *c = closed;
                }
                self.replace(k, element)?;
            }
            return Ok(());
        }

        if let Some(p) = self.pins.iter().position(|p| p.pin.id == id) {
            let mut pin = self.pins[p].pin.clone();
            match param {
                Param::PinMode(mode) => pin.mode = mode,
                Param::Level(level) => pin.level = level,
                Param::Pull(pull) => pin.pull = pull,
                _ => return wrong(),
            }
            for part in crate::composite::pin(&pin)? {
                let k = self.element_index(part.id())?;
                self.replace(k, part)?;
            }
            self.pins[p].pin = pin;
            return Ok(());
        }

        if let Param::LoadTorque(torque) = param {
            // A motor is solved as its parts; the load is `#load`.
            let Some(&k) = self.index.get(&format!("{id}#load")) else {
                return match self.index.contains_key(id) {
                    true => wrong(),
                    false => self.element_index(id).map(|_| ()),
                };
            };
            if !torque.is_finite() {
                return Err(SimError::component(
                    id,
                    format!("params.load_torque {torque} N·m must be finite"),
                ));
            }
            let mut element = self.stepper.circuit.components[k].clone();
            if let CircuitElement::CurrentSource(CurrentSource { current, .. }) = &mut element {
                *current = torque;
            }
            return self.replace(k, element);
        }

        let k = self.element_index(id)?;
        let mut element = self.stepper.circuit.components[k].clone();
        match (&mut element, param) {
            (CircuitElement::Resistor(Resistor { resistance, .. }), Param::Resistance(v)) => {
                *resistance = v;
            }
            (CircuitElement::Potentiometer(Potentiometer { position, .. }), Param::Position(v)) => {
                *position = v;
            }
            (
                CircuitElement::Switch(Switch { closed, .. })
                | CircuitElement::Pushbutton(Pushbutton { closed, .. }),
                Param::Closed(v),
            ) => *closed = v,
            (
                CircuitElement::Photoresistor(Photoresistor { light_level, .. }),
                Param::LightLevel(v),
            ) => *light_level = v,
            (CircuitElement::Photodiode(Photodiode { irradiance, .. }), Param::Irradiance(v)) => {
                *irradiance = v;
            }
            (element, Param::Temperature(v)) => {
                if self
                    .stepper
                    .circuit
                    .parts
                    .get(id)
                    .is_some_and(|p| p.thermal.is_some())
                {
                    return Err(SimError::config(
                        ConfigKind::TransientSession,
                        format!("`{id}`: self-heating sets this part's temperature"),
                    ));
                }
                match crate::thermal::temperature_mut(element) {
                    Some(t) => *t = v,
                    None => return wrong(),
                }
            }
            _ => return wrong(),
        }
        element.device().validate()?;
        self.replace(k, element)
    }

    /// Advance to `t` (s).
    ///
    /// # Errors
    ///
    /// [`SimError::InvalidConfig`] for a `t` before the time reached,
    /// past `tstop` or not finite; otherwise as
    /// [`solve_transient`](crate::solve_transient) for the steps taken.
    pub fn advance_to(&mut self, t: f64) -> Result<(), SimError> {
        self.advance(t, None)
    }

    /// [`advance_to`](Self::advance_to), calling `progress` after every
    /// accepted step, whose [`tstop`](TransientProgress::tstop) is `t`.
    ///
    /// # Errors
    ///
    /// As [`advance_to`](Self::advance_to), and [`SimError::Cancelled`] when
    /// the hook returns [`Flow::Stop`]. A cancelled session stays at the
    /// step it stopped after, and can be advanced again from there.
    pub fn advance_to_with_progress(
        &mut self,
        t: f64,
        mut progress: impl FnMut(&TransientProgress<'_>) -> Flow,
    ) -> Result<(), SimError> {
        self.advance(t, Some(&mut progress))
    }

    /// Advance to `t_max` (s), or stop early at the first of `events` to
    /// happen. The crossing is located to the event's tolerance: the
    /// session stops at most that long after it, with the node past the
    /// level. `None` when nothing happened before `t_max`.
    ///
    /// The session starts watching from the time reached, so a node
    /// already past the level counts only when it crosses again.
    ///
    /// # Errors
    ///
    /// As [`advance_to`](Self::advance_to), and
    /// [`SimError::UnknownNode`] for an event on a node the circuit
    /// doesn't have, and [`SimError::InvalidConfig`] for one with a level
    /// or tolerance that isn't finite (or a tolerance that isn't
    /// positive).
    pub fn run_until(
        &mut self,
        t_max: f64,
        events: &[Event],
    ) -> Result<Option<EventHit>, SimError> {
        self.check_target(t_max)?;
        let watch = self.watch(events)?;
        self.open_window(t_max)?;
        let mut values = watch.values(&self.stepper);
        match self.advance_watching(t_max, &watch, &mut values)? {
            None => Ok(None),
            Some(bracket) => self.locate(&watch, bracket).map(Some),
        }
    }

    fn element_index(&self, id: &str) -> Result<usize, SimError> {
        self.index
            .get(id)
            .copied()
            .ok_or_else(|| SimError::UnknownComponent(id.to_string()))
    }

    /// Put `element` in place of element `k`, as a change at the time
    /// reached. A value it already had changes nothing, and doesn't
    /// restart the integration.
    fn replace(&mut self, k: usize, element: CircuitElement) -> Result<(), SimError> {
        if self.stepper.circuit.components[k] == element {
            return Ok(());
        }
        self.stepper.edit(k, element);
        Ok(())
    }

    fn check_target(&self, t: f64) -> Result<(), SimError> {
        if let Some(time) = self.failed {
            return Err(SimError::config(
                ConfigKind::TransientSession,
                format!("stopped by an error at t = {time:e} s"),
            ));
        }
        let eps = self.stepper.eps();
        let tstop = self.stepper.config.tstop;
        if !t.is_finite() || t < self.time() - eps || t > tstop + eps {
            return Err(SimError::config(
                ConfigKind::TransientSession,
                format!(
                    "can't advance to {t} s from {} s; it ends at tstop = {tstop} s",
                    self.time()
                ),
            ));
        }
        Ok(())
    }

    /// The breakpoints of the circuit's sources from the time reached to
    /// `until`, for the steps of one call. Sources may have changed since
    /// the last, so they're found afresh.
    fn open_window(&mut self, until: f64) -> Result<(), SimError> {
        let from = self.time();
        // A breakpoint right at `until` is still a corner to land on, so
        // the window reaches past it.
        let to = until + self.stepper.h_max;
        self.stepper.breakpoints = waveform_breakpoints(&self.stepper.circuit, from, to)?;
        self.stepper.state.next_bp = 0;
        Ok(())
    }

    fn advance(
        &mut self,
        t: f64,
        mut progress: Option<&mut dyn FnMut(&TransientProgress<'_>) -> Flow>,
    ) -> Result<(), SimError> {
        self.check_target(t)?;
        self.open_window(t)?;
        while !self.stepper.reached(t) {
            self.take_step(t)?;
            if let Some(hook) = progress.as_mut() {
                let report = TransientProgress {
                    time: self.time(),
                    tstop: t,
                    accepted_steps: self.stepper.state.stats.accepted_steps,
                    step: if self.stepper.recording() {
                        self.last_step()
                    } else {
                        None
                    },
                };
                if hook(&report) == Flow::Stop {
                    return Err(SimError::Cancelled { time: self.time() });
                }
            }
        }
        Ok(())
    }

    fn take_step(&mut self, until: f64) -> Result<(), SimError> {
        let mut snapshot = match self.stepper.step(until) {
            Ok(snapshot) => snapshot,
            Err(e) => {
                self.failed = Some(self.time());
                return Err(e);
            }
        };
        if let Some(folder) = self
            .composites
            .as_ref()
            .and_then(crate::composite::StepFolder::new)
        {
            folder.fold(&mut snapshot);
        }
        for (reading, p) in self.readings.iter_mut().zip(&self.pins) {
            *reading = p.pin.reads(p.volts(&self.stepper), Some(*reading));
        }
        if self.keep && self.stepper.recording() {
            self.steps.push(snapshot);
            self.last = None;
        } else {
            self.last = Some(snapshot);
        }
        Ok(())
    }

    fn save(&self, with_last: bool) -> Saved {
        Saved {
            state: self.stepper.state.clone(),
            steps: self.steps.len(),
            last: if with_last {
                self.last_step().cloned()
            } else {
                None
            },
            readings: self.readings.clone(),
        }
    }

    fn rewind(&mut self, saved: Saved) {
        self.stepper.rewind(saved.state);
        self.steps.truncate(saved.steps);
        self.last = saved.last;
        self.readings = saved.readings;
    }

    /// Step to `until` while watching, with `values` the watched nodes at
    /// the time reached. A step across one of the events stops it, and
    /// gives the state before that step with the values either side;
    /// otherwise the session ends at `until` with `values` there.
    fn advance_watching(
        &mut self,
        until: f64,
        watch: &Watch<'_>,
        values: &mut Vec<f64>,
    ) -> Result<Option<Bracket>, SimError> {
        while !self.stepper.reached(until) {
            let before = self.save(false);
            self.take_step(until)?;
            let after = watch.values(&self.stepper);
            if watch.first(values, &after).is_some() {
                let before_values = std::mem::replace(values, after.clone());
                return Ok(Some((before, before_values, after)));
            }
            *values = after;
        }
        Ok(None)
    }

    /// Narrow a step an event crossed in (the state before it, and the
    /// watched values either side; the session is at its end) until it's
    /// no longer than the event's tolerance, and stop at its end.
    fn locate(
        &mut self,
        watch: &Watch<'_>,
        (mut a, mut va, mut vb): Bracket,
    ) -> Result<EventHit, SimError> {
        let mut b = self.save(true);
        for narrowing in 0.. {
            let (ta, tb) = (a.state.time, b.state.time);
            // `b` is past a crossing `a` is before, so there is one.
            let (event, frac) = watch.first(&va, &vb).unwrap_or((0, 0.5));
            let tol = watch.tolerances[event];
            if tb - ta <= tol {
                break;
            }
            // Straddle the interpolated crossing by half the tolerance
            // each side, so one narrowing can close the bracket; halve
            // when interpolation isn't closing in (a node curving hard
            // across the bracket keeps one end where it is). Both targets
            // stay a quarter tolerance inside it, which is room enough as
            // it's wider than the tolerance.
            let tc = if narrowing < INTERPOLATED_NARROWINGS {
                ta + frac * (tb - ta)
            } else {
                0.5 * (ta + tb)
            };
            let targets = [
                (tc - 0.5 * tol).max(ta + 0.25 * tol),
                (tc + 0.5 * tol).min(tb - 0.25 * tol),
            ];
            self.rewind(Saved {
                state: a.state.clone(),
                steps: a.steps,
                last: None,
                readings: a.readings.clone(),
            });
            let mut values = va.clone();
            let mut bracket = None;
            for target in targets {
                // The window `run_until` opened covers the bracket, so its
                // breakpoints still hold.
                match self.advance_watching(target, watch, &mut values)? {
                    Some(found) => {
                        bracket = Some(found);
                        break;
                    }
                    None => {
                        a = self.save(false);
                        va = values.clone();
                    }
                }
            }
            // With nothing crossed by the last target, the crossing is
            // between it (now `a`) and `b`.
            if let Some((before, v0, v1)) = bracket {
                a = before;
                va = v0;
                vb = v1;
                b = self.save(true);
            }
        }
        let time = b.state.time;
        let event = watch.first(&va, &vb).map_or(0, |(k, _)| k);
        self.rewind(b);
        Ok(EventHit { event, time })
    }

    fn watch<'a>(&self, events: &'a [Event]) -> Result<Watch<'a>, SimError> {
        let floor = 1e3 * self.stepper.eps();
        let mut rows = Vec::with_capacity(events.len());
        let mut tolerances = Vec::with_capacity(events.len());
        for e in events {
            let invalid = |what: String| {
                Err(SimError::config(
                    ConfigKind::TransientSession,
                    format!("event on `{}`: {what}", e.node),
                ))
            };
            if !e.level.is_finite() {
                return invalid(format!("level must be finite (got {})", e.level));
            }
            let tol = e.tolerance.unwrap_or(1e-4 * self.stepper.config.tstep);
            if !(tol.is_finite() && tol > 0.0) {
                return invalid(format!("tolerance must be finite and > 0 s (got {tol})"));
            }
            tolerances.push(tol.max(floor));
            if e.node == self.stepper.circuit.ground_node {
                rows.push(None);
                continue;
            }
            match self.stepper.node_map.index(&e.node) {
                Some(i) => rows.push(Some(i)),
                None => return Err(SimError::unknown_node(e.node.as_str())),
            }
        }
        Ok(Watch {
            events,
            rows,
            tolerances,
        })
    }
}
