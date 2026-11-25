//! Solver output types.
//!
//! [`SimulationResult`] is what every analysis path returns: node voltages,
//! branch currents, per-component V/I/P, and (when relevant) per-device
//! operating-point detail and a transient time-series. The optional
//! per-device fields are populated only when the matching component types
//! appear in the circuit.

use crate::element::layout::Layout;
use crate::element::Reported;
use std::collections::HashMap;

use nalgebra::DVector;

use sindr_devices::bjt::BjtRegion;
use sindr_devices::mosfet::MosfetRegion;

use crate::circuit::Circuit;
use crate::node_map::{NodeMap, Terminals};

/// Per-component voltage, current, and power.
///
/// Reported for every component. For a two-terminal one, current flows
/// from `nodes[0]` to `nodes[1]` and positive `power` means it is
/// dissipating energy. Components with more terminals report one port:
/// BJTs and IGBTs give Vce and Ic, MOSFETs and JFETs give Vds and Id, with
/// `power` as the magnitude `|V·I|` (their full results are in
/// [`BjtResult`] and [`MosfetResult`]); a relay gives its coil, a
/// transformer its primary, and a voltage regulator its pass element
/// (`Vin − Vout`, the output current, and its dissipation).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct ComponentResult {
    /// Component id (matches the id on the source [`CircuitElement`](crate::CircuitElement)).
    pub id: String,
    /// Voltage from `nodes[0]` to `nodes[1]` (V).
    pub voltage_across: f64,
    /// Current from `nodes[0]` to `nodes[1]` (A).
    pub current_through: f64,
    /// Instantaneous power dissipation (W). Positive = dissipating.
    pub power: f64,
}

/// Per-BJT operating-point results: terminal voltages, terminal currents,
/// power, and which region of operation the device is in.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct BjtResult {
    /// Component id.
    pub id: String,
    /// Base–emitter voltage (V).
    pub vbe: f64,
    /// Collector–emitter voltage (V).
    pub vce: f64,
    /// Base current (A), into the base: positive for a conducting NPN,
    /// negative for a PNP.
    pub ib: f64,
    /// Collector current (A), into the collector: positive for a
    /// conducting NPN, negative for a PNP.
    pub ic: f64,
    /// Emitter current (A), into the emitter: `−(ic + ib)` by KCL, so
    /// negative for a conducting NPN and positive for a PNP.
    pub ie: f64,
    /// Collector dissipation `|Ic·Vce|` (W). The base's `Ib·Vbe` is left
    /// out.
    pub power: f64,
    /// Operating region. Serialises as `"active"`, `"saturation"` or
    /// `"cutoff"`, as the string it replaced did.
    pub region: BjtRegion,
}

/// Per-MOSFET operating-point results.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct MosfetResult {
    /// Component id.
    pub id: String,
    /// Gate–source voltage (V).
    pub vgs: f64,
    /// Drain–source voltage (V).
    pub vds: f64,
    /// Drain current (A), into the drain: positive for a conducting NMOS,
    /// negative for a PMOS, and the other way round in reverse mode (the
    /// drain pulled past the source).
    pub id_current: f64,
    /// Total dissipation (W).
    pub power: f64,
    /// Operating region; in reverse mode, that of the device with drain and
    /// source swapped. Serialises as `"cutoff"`, `"triode"` or
    /// `"saturation"`, as the string it replaced did.
    pub region: MosfetRegion,
}

/// Per-op-amp / comparator results: input/output voltages.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct OpAmpResult {
    /// Component id.
    pub id: String,
    /// Voltage at the non-inverting input (V).
    pub v_in_plus: f64,
    /// Voltage at the inverting input (V).
    pub v_in_minus: f64,
    /// `v_in_plus - v_in_minus` (V).
    pub v_differential: f64,
    /// Output voltage (V), clipped to the rail range.
    pub v_out: f64,
}

/// Per-relay results: coil voltage and contact state.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct RelayResult {
    /// Component id.
    pub id: String,
    /// Coil voltage, `coil+ − coil−` (`nodes[0] − nodes[1]`) (V).
    pub coil_voltage: f64,
    /// `true` when coil voltage exceeds the pickup threshold.
    pub contact_closed: bool,
}

/// A DC motor's shaft, from its [`DcMotor`](crate::element::DcMotor)
/// element. Its electrical side is its [`ComponentResult`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct MotorResult {
    /// Shaft speed ω (rad/s), positive turning forwards.
    pub speed: f64,
    /// Electromagnetic torque `kt·i` (N·m), before friction, inertia and
    /// load take their share.
    pub torque: f64,
    /// Power converted from electrical to mechanical, `torque · speed` (W):
    /// the electrical power in, less the armature's `i²R` (and, in a
    /// transient, the energy its inductance stores).
    pub mechanical_power: f64,
}

impl MotorResult {
    /// Shaft speed in revolutions per minute.
    pub fn rpm(&self) -> f64 {
        self.speed * 60.0 / std::f64::consts::TAU
    }
}

/// One timestep of a transient simulation: time, node voltages, per-component
/// state at that instant.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct TimestepSnapshot {
    /// Simulation time at this snapshot (s).
    pub time: f64,
    /// Node voltages at this instant (V).
    pub node_voltages: HashMap<String, f64>,
    /// Per-component voltage/current/power at this instant. A source with a
    /// waveform reports its value at this time, not its DC term.
    pub component_results: Vec<ComponentResult>,
    /// Junction temperature (K) of each part with
    /// [`Thermal`](crate::Thermal) data at this instant, by component id.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "HashMap::is_empty", default)
    )]
    pub junction_temperatures: HashMap<String, f64>,
    /// State of charge (0 to 1) of each battery at this instant, by
    /// component id. Below 0 the cell is over-discharged; its voltage holds
    /// the curve's 0 % value.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "HashMap::is_empty", default)
    )]
    pub state_of_charge: HashMap<String, f64>,
    /// Each DC motor's shaft at this instant, by component id.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "HashMap::is_empty", default)
    )]
    pub motors: HashMap<String, MotorResult>,
}

/// Time-series data from a transient simulation.
///
/// Present on [`SimulationResult::transient`] when the circuit contains
/// reactive elements (capacitors, inductors) or time-varying sources.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct TransientData {
    /// Snapshots at each accepted integration step, in time order. They
    /// include every multiple of `time_step`, every waveform breakpoint and
    /// the end time, plus any shorter steps accuracy needed in between.
    pub timesteps: Vec<TimestepSnapshot>,
    /// Requested time step (s): the longest step taken, and the output grid.
    pub time_step: f64,
    /// Total simulated duration (s).
    pub duration: f64,
    /// How the stepper got there.
    #[cfg_attr(feature = "serde", serde(default))]
    pub stats: TransientStats,
}

/// Step counts from one transient run.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct TransientStats {
    /// Steps accepted (one snapshot each).
    pub accepted_steps: usize,
    /// Steps rejected because their local truncation error was too large.
    pub lte_rejections: usize,
    /// Steps retried at half the size because Newton–Raphson didn't converge.
    pub newton_rejections: usize,
    /// Steps Newton–Raphson couldn't take even at the shortest step, solved
    /// by pseudo-transient continuation instead: regenerative edges the
    /// node voltages jump across, such as a multivibrator's when its
    /// transistors have no junction capacitance.
    #[cfg_attr(feature = "serde", serde(default))]
    pub jumped_steps: usize,
}

/// Complete simulation output.
///
/// At minimum, contains node voltages, branch currents (for voltage sources
/// and other branch-current-bearing elements), and per-component results.
/// The optional fields are populated only when the corresponding
/// component types appear in the circuit:
///
/// - `bjt_results` — for [`Bjt`](crate::CircuitElement::Bjt) components
/// - `mosfet_results` — for [`Mosfet`](crate::CircuitElement::Mosfet)
/// - `op_amp_results` — for op-amps and comparators
/// - `relay_results` — for relays
/// - `transient` — for circuits with capacitors, inductors, or waveform
///   sources (DC analysis returns `None`)
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct SimulationResult {
    /// Voltage at every node (V), keyed by node name. Includes the ground
    /// node (always 0.0).
    ///
    /// Indexing with `[]` panics on a missing key, the same as any
    /// [`HashMap`]. If the node name might be absent (e.g. typos, or a
    /// node only referenced via a control terminal), use
    /// `node_voltages.get("n2")` to get an `Option<&f64>` instead.
    pub node_voltages: HashMap<String, f64>,
    /// Branch currents (A) for components that introduce a current unknown
    /// — voltage sources, op-amps, transformers, etc. Keyed by component id.
    ///
    /// The MNA convention: the current entering the element at `nodes[0]`
    /// and leaving at `nodes[1]`, through the element. A source delivering
    /// power reads **negative**: a 5 V supply feeding 5 mA into a load is
    /// `-0.005`. For a VCVS, CCVS, op-amp or comparator it is the current
    /// into the output node (`nodes[0]`, or the op-amp's `out`), so an
    /// output sourcing current is negative too. For a voltage regulator it
    /// is the current into its output pin (negative when it sources); its
    /// [`ComponentResult`] has the output current as a positive number.
    pub branch_currents: HashMap<String, f64>,
    /// Per-component voltage, current, and power for every two-terminal
    /// component.
    pub component_results: Vec<ComponentResult>,
    /// Detailed per-BJT results. Empty when the circuit has no BJTs.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub bjt_results: Vec<BjtResult>,
    /// Detailed per-MOSFET results. Empty when the circuit has no MOSFETs.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub mosfet_results: Vec<MosfetResult>,
    /// Detailed per-op-amp / comparator results.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub op_amp_results: Vec<OpAmpResult>,
    /// Detailed per-relay results.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Vec::is_empty", default)
    )]
    pub relay_results: Vec<RelayResult>,
    /// Transient time-series, present only when the circuit has reactive
    /// elements or waveform sources.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub transient: Option<TransientData>,
    /// The fallback the nonlinear DC solve needed, if plain Newton–Raphson
    /// didn't converge on its own. `None` for linear circuits, transients and
    /// plain Newton.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "Option::is_none", default)
    )]
    pub convergence_aid: Option<ConvergenceAid>,
    /// Self-heated junction temperature (K) of each part with
    /// [`Thermal`](crate::Thermal) data, by component id: the DC balance, or
    /// the end of a transient. Empty when no part has thermal data.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "HashMap::is_empty", default)
    )]
    pub junction_temperatures: HashMap<String, f64>,
    /// State of charge (0 to 1) of each battery, by component id: the
    /// starting value at DC, the last step's after a transient.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "HashMap::is_empty", default)
    )]
    pub state_of_charge: HashMap<String, f64>,
    /// Each DC motor's shaft, by component id: its steady state at DC, the
    /// last step's after a transient.
    #[cfg_attr(
        feature = "serde",
        serde(skip_serializing_if = "HashMap::is_empty", default)
    )]
    pub motors: HashMap<String, MotorResult>,
}

/// A convergence fallback for nonlinear DC, tried in this order when plain
/// Newton–Raphson fails.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConvergenceAid {
    /// Extra conductance to ground on every node, stepped from 1e-2 S down
    /// to zero.
    GminStepping,
    /// Every independent source ramped from near zero up to its full value.
    SourceStepping,
}

/// A component's current: its [`ComponentResult::current_through`], or its
/// branch current when it has no component result.
fn current_in(
    components: &[ComponentResult],
    branches: Option<&HashMap<String, f64>>,
    id: &str,
) -> Option<f64> {
    find_by_id(components, id, |c| &c.id)
        .map(|c| c.current_through)
        .or_else(|| branches.and_then(|b| b.get(id).copied()))
}

fn find_by_id<'a, T>(items: &'a [T], id: &str, key: impl Fn(&T) -> &str) -> Option<&'a T> {
    items.iter().find(|item| key(item) == id)
}

impl SimulationResult {
    /// The voltage at `node` (V), or `None` when the circuit has no such
    /// node. The ground node is in every result and reads `0.0`.
    ///
    /// Inside a block, a local net is named by its instance path
    /// (`"X1.mid"`); [`SimulationResult::instance`] does the joining.
    ///
    /// ```
    /// use sindr::{solve_circuit, Circuit, CircuitElement};
    /// use sindr::element::{Resistor, VoltageSource};
    ///
    /// let circuit = Circuit::new("0", vec![
    ///     CircuitElement::VoltageSource(VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 10.0, waveform: None }),
    ///     CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()], resistance: 1e3 }),
    ///     CircuitElement::Resistor(Resistor { id: "R2".into(), nodes: ["out".into(), "0".into()], resistance: 1e3 }),
    /// ]);
    /// let result = solve_circuit(&circuit)?;
    /// assert!((result.voltage("out").unwrap() - 5.0).abs() < 1e-9);
    /// assert_eq!(result.voltage("0"), Some(0.0));
    /// assert_eq!(result.voltage("nope"), None);
    /// # Ok::<(), sindr::SimError>(())
    /// ```
    ///
    pub fn voltage(&self, node: impl AsRef<str>) -> Option<f64> {
        let node = node.as_ref();
        self.node_voltages.get(node).copied()
    }

    /// The current through component `id` (A): its
    /// [`ComponentResult::current_through`], from `nodes[0]` to `nodes[1]`
    /// for a two-terminal part, the collector or drain current for a
    /// transistor. An id with a branch current but no component result
    /// falls back to its [`branch_current`](Self::branch_current). `None`
    /// when there's no such component.
    ///
    /// This is what a measurement's `Signal::i` reads.
    pub fn current(&self, id: impl AsRef<str>) -> Option<f64> {
        let id = id.as_ref();
        current_in(&self.component_results, Some(&self.branch_currents), id)
    }

    /// The MNA branch current of component `id` (A), with the sign
    /// convention of [`SimulationResult::branch_currents`]: a source
    /// delivering power reads negative. `None` when `id` has no branch
    /// current (resistors, diodes and other parts without a current
    /// unknown) or isn't in the circuit.
    pub fn branch_current(&self, id: impl AsRef<str>) -> Option<f64> {
        let id = id.as_ref();
        self.branch_currents.get(id).copied()
    }

    /// The voltage, current and power of component `id`, or `None` when
    /// there's no such component.
    pub fn component(&self, id: impl AsRef<str>) -> Option<&ComponentResult> {
        let id = id.as_ref();
        find_by_id(&self.component_results, id, |c| &c.id)
    }

    /// The operating point of BJT `id`, or `None` when there's no BJT by
    /// that id.
    pub fn bjt(&self, id: impl AsRef<str>) -> Option<&BjtResult> {
        let id = id.as_ref();
        find_by_id(&self.bjt_results, id, |b| &b.id)
    }

    /// The operating point of MOSFET `id`, or `None` when there's no
    /// MOSFET by that id.
    pub fn mosfet(&self, id: impl AsRef<str>) -> Option<&MosfetResult> {
        let id = id.as_ref();
        find_by_id(&self.mosfet_results, id, |m| &m.id)
    }

    /// The inputs and output of op-amp or comparator `id`, or `None` when
    /// there's no op-amp or comparator by that id.
    pub fn op_amp(&self, id: impl AsRef<str>) -> Option<&OpAmpResult> {
        let id = id.as_ref();
        find_by_id(&self.op_amp_results, id, |o| &o.id)
    }

    /// The coil voltage and contact state of relay `id`, or `None` when
    /// there's no relay by that id.
    pub fn relay(&self, id: impl AsRef<str>) -> Option<&RelayResult> {
        let id = id.as_ref();
        find_by_id(&self.relay_results, id, |r| &r.id)
    }

    /// The results inside a block instance, addressed by its instance path:
    /// `"X1"`, or `"X1.X2"` for an instance nested in another. Chain
    /// [`InstanceResult::instance`] to go deeper.
    ///
    /// Names inside it are the ones written in the block, joined to the
    /// path with [`PATH_SEPARATOR`](crate::design::PATH_SEPARATOR) the way
    /// flattening names them, so `instance("X1").voltage("mid")` reads
    /// node `X1.mid`. A parsed SPICE deck is lowercase: `instance("x1")`.
    ///
    /// ```
    /// use sindr::{solve_circuit, Block, Circuit, CircuitElement, Instance};
    /// use sindr::element::{Resistor, VoltageSource};
    ///
    /// let r = |id: &str, a: &str, b: &str| CircuitElement::Resistor(Resistor {
    ///     id: id.into(), nodes: [a.into(), b.into()], resistance: 1e3,
    /// });
    /// let mut circuit = Circuit::new("0", vec![
    ///     CircuitElement::VoltageSource(VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 4.0, waveform: None }),
    /// ]);
    /// circuit.blocks.insert("ladder".into(), sindr::Block {
    ///     ports: vec!["top".into()],
    ///     components: vec![r("R1", "top", "mid"), r("R2", "mid", "0")],
    ///     ..Block::default()
    /// });
    /// circuit.instances.push(Instance::new("X1", "ladder", ["in"]));
    ///
    /// let result = solve_circuit(&circuit)?;
    /// let x1 = result.instance("X1");
    /// assert!((x1.voltage("mid").unwrap() - 2.0).abs() < 1e-9);
    /// assert!((x1.current("R2").unwrap() - 2e-3).abs() < 1e-12);
    /// // A port is the caller's net, not a node of its own.
    /// assert_eq!(x1.voltage("top"), None);
    /// # Ok::<(), sindr::SimError>(())
    /// ```
    ///
    /// The view never fails to build: an instance that doesn't exist
    /// answers `None` to every lookup. Ports, ground and globals keep the
    /// name they have outside the block (a port is the net it's bound
    /// to), so read those on the parent.
    pub fn instance(&self, path: &str) -> InstanceResult<'_> {
        InstanceResult {
            result: self,
            path: path.to_string(),
        }
    }
}

/// The results inside one block instance: what
/// [`SimulationResult::instance`] returns.
///
/// Each lookup joins the instance path to a name as the block writes it
/// (`"sw"` inside `"X1.X2"` is `"X1.X2.sw"`) and reads the full result.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct InstanceResult<'a> {
    result: &'a SimulationResult,
    path: String,
}

impl<'a> InstanceResult<'a> {
    /// The instance path, e.g. `"X1.X2"`.
    pub fn path(&self) -> &str {
        &self.path
    }

    /// The flat name of `name` inside this instance: `"X1.X2.sw"` for
    /// `"sw"` inside `"X1.X2"`. Use it to read an AC, sweep or transient
    /// result, whose accessors take flat names.
    pub fn name(&self, name: &str) -> String {
        if self.path.is_empty() {
            name.to_string()
        } else {
            format!("{}{}{name}", self.path, crate::design::PATH_SEPARATOR)
        }
    }

    /// The instance at `path` inside this one.
    pub fn instance(&self, path: &str) -> InstanceResult<'a> {
        InstanceResult {
            result: self.result,
            path: self.name(path),
        }
    }

    /// The voltage at local net `node` (V); see
    /// [`SimulationResult::voltage`].
    pub fn voltage(&self, node: impl AsRef<str>) -> Option<f64> {
        let node = node.as_ref();
        self.result.voltage(self.name(node))
    }

    /// The current through component `id` (A); see
    /// [`SimulationResult::current`].
    pub fn current(&self, id: impl AsRef<str>) -> Option<f64> {
        let id = id.as_ref();
        self.result.current(self.name(id))
    }

    /// The branch current of component `id` (A); see
    /// [`SimulationResult::branch_current`].
    pub fn branch_current(&self, id: impl AsRef<str>) -> Option<f64> {
        let id = id.as_ref();
        self.result.branch_current(self.name(id))
    }

    /// The voltage, current and power of component `id`.
    pub fn component(&self, id: impl AsRef<str>) -> Option<&'a ComponentResult> {
        let id = id.as_ref();
        self.result.component(self.name(id))
    }

    /// The operating point of BJT `id`.
    pub fn bjt(&self, id: impl AsRef<str>) -> Option<&'a BjtResult> {
        let id = id.as_ref();
        self.result.bjt(self.name(id))
    }

    /// The operating point of MOSFET `id`.
    pub fn mosfet(&self, id: impl AsRef<str>) -> Option<&'a MosfetResult> {
        let id = id.as_ref();
        self.result.mosfet(self.name(id))
    }

    /// The inputs and output of op-amp or comparator `id`.
    pub fn op_amp(&self, id: impl AsRef<str>) -> Option<&'a OpAmpResult> {
        let id = id.as_ref();
        self.result.op_amp(self.name(id))
    }

    /// The coil voltage and contact state of relay `id`.
    pub fn relay(&self, id: impl AsRef<str>) -> Option<&'a RelayResult> {
        let id = id.as_ref();
        self.result.relay(self.name(id))
    }
}

impl TimestepSnapshot {
    /// The voltage at `node` at this instant (V), or `None` when there's
    /// no such node. Ground reads `0.0`.
    pub fn voltage(&self, node: impl AsRef<str>) -> Option<f64> {
        let node = node.as_ref();
        self.node_voltages.get(node).copied()
    }

    /// The current through component `id` at this instant (A), its
    /// [`ComponentResult::current_through`], or `None` when there's no
    /// such component.
    pub fn current(&self, id: impl AsRef<str>) -> Option<f64> {
        let id = id.as_ref();
        current_in(&self.component_results, None, id)
    }

    /// The voltage, current and power of component `id` at this instant.
    pub fn component(&self, id: impl AsRef<str>) -> Option<&ComponentResult> {
        let id = id.as_ref();
        find_by_id(&self.component_results, id, |c| &c.id)
    }
}

impl TransientData {
    /// A node's waveform as `(time in s, voltage in V)` pairs, one per
    /// step. Empty when there's no such node.
    ///
    /// ```
    /// use sindr::{solve_transient, Circuit, CircuitElement, TransientConfig};
    /// use sindr::element::{Capacitor, Resistor, VoltageSource};
    ///
    /// let circuit = Circuit::new("0", vec![
    ///     CircuitElement::VoltageSource(VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 5.0, waveform: None }),
    ///     CircuitElement::Resistor(Resistor { id: "R1".into(), nodes: ["in".into(), "out".into()], resistance: 1e3 }),
    ///     CircuitElement::Capacitor(Capacitor { id: "C1".into(), nodes: ["out".into(), "0".into()], capacitance: 1e-6 }),
    /// ]);
    /// let result = solve_transient(&circuit, &TransientConfig::new(5e-3, 1e-5))?;
    /// let tran = result.transient.unwrap();
    ///
    /// let wave = tran.node_voltage_curve("out");
    /// assert_eq!(wave.len(), tran.timesteps.len());
    /// // One time constant in: 1 − 1/e of the way to 5 V.
    /// let v_tau = tran.voltage_at("out", 1e-3).unwrap();
    /// assert!((v_tau - 5.0 * (1.0 - (-1.0f64).exp())).abs() < 0.02);
    /// # Ok::<(), sindr::SimError>(())
    /// ```
    ///
    pub fn node_voltage_curve(&self, node: impl AsRef<str>) -> Vec<(f64, f64)> {
        let node = node.as_ref();
        self.timesteps
            .iter()
            .filter_map(|s| s.voltage(node).map(|v| (s.time, v)))
            .collect()
    }

    /// A component's current as `(time in s, current in A)` pairs, one per
    /// step; see [`TimestepSnapshot::current`]. Empty when there's no such
    /// component.
    pub fn component_current_curve(&self, id: impl AsRef<str>) -> Vec<(f64, f64)> {
        let id = id.as_ref();
        self.timesteps
            .iter()
            .filter_map(|s| s.current(id).map(|i| (s.time, i)))
            .collect()
    }

    /// The voltage at `node` at time `t` (V), linearly interpolated between
    /// the steps either side. `None` when there's no such node or `t` is
    /// outside the run.
    pub fn voltage_at(&self, node: impl AsRef<str>, t: f64) -> Option<f64> {
        let node = node.as_ref();
        crate::measure::interpolate(&self.node_voltage_curve(node), t)
    }

    /// The current through component `id` at time `t` (A), linearly
    /// interpolated as [`TransientData::voltage_at`] is.
    pub fn current_at(&self, id: impl AsRef<str>, t: f64) -> Option<f64> {
        let id = id.as_ref();
        crate::measure::interpolate(&self.component_current_curve(id), t)
    }
}

/// Extract simulation results from the MNA solution vector.
///
/// Builds node voltages, branch currents (for voltage sources), and
/// per-component V/I/P using passive sign convention, each element's from
/// its [`Device::report`](crate::element::Device::report).
pub(crate) fn extract_results(
    circuit: &Circuit,
    node_map: &NodeMap,
    solution: &DVector<f64>,
    num_nodes: usize,
) -> SimulationResult {
    // Node voltages: every mapped node + ground = 0.0
    let mut node_voltages = HashMap::new();
    for i in 0..num_nodes {
        if let Some(name) = node_map.node_name(i) {
            node_voltages.insert(name.to_string(), solution[i]);
        }
    }
    node_voltages.insert(circuit.ground_node.clone(), 0.0);

    let layout = Layout::new(circuit);
    let terminals = Terminals::new(circuit, node_map);
    let reported = Reported::collect(circuit, node_map, (&layout, &terminals), solution, None);
    SimulationResult {
        node_voltages,
        branch_currents: reported.branch_currents,
        component_results: reported.components,
        bjt_results: reported.bjts,
        mosfet_results: reported.mosfets,
        op_amp_results: reported.op_amps,
        relay_results: reported.relays,
        convergence_aid: None,
        junction_temperatures: HashMap::new(),
        state_of_charge: reported.state_of_charge,
        motors: HashMap::new(),
        transient: None,
    }
}

// Every test here checks the serde shape.
#[cfg(all(test, feature = "serde"))]
mod tests {
    use super::*;

    /// DC-only SimulationResult (transient: None) serializes WITHOUT a "transient" key.
    #[test]
    fn dc_result_omits_transient_key() {
        let result = SimulationResult {
            node_voltages: HashMap::from([("n1".into(), 5.0)]),
            branch_currents: HashMap::new(),
            component_results: vec![],
            bjt_results: vec![],
            mosfet_results: vec![],
            op_amp_results: vec![],
            relay_results: vec![],
            convergence_aid: None,
            junction_temperatures: HashMap::new(),
            state_of_charge: HashMap::new(),
            motors: HashMap::new(),
            transient: None,
        };
        let json = serde_json::to_string(&result).unwrap();
        assert!(!json.contains("\"transient\""));
        assert!(!json.contains("\"bjt_results\""));
    }

    /// SimulationResult with transient data serializes the timesteps array.
    #[test]
    fn transient_result_includes_timesteps() {
        let result = SimulationResult {
            node_voltages: HashMap::from([("n1".into(), 5.0)]),
            branch_currents: HashMap::new(),
            component_results: vec![],
            bjt_results: vec![],
            mosfet_results: vec![],
            op_amp_results: vec![],
            relay_results: vec![],
            convergence_aid: None,
            junction_temperatures: HashMap::new(),
            state_of_charge: HashMap::new(),
            motors: HashMap::new(),
            transient: Some(TransientData {
                timesteps: vec![
                    TimestepSnapshot {
                        time: 0.0,
                        node_voltages: HashMap::from([("n1".into(), 0.0)]),
                        component_results: vec![],
                        junction_temperatures: HashMap::new(),
                        state_of_charge: HashMap::new(),
                        motors: HashMap::new(),
                    },
                    TimestepSnapshot {
                        time: 0.001,
                        node_voltages: HashMap::from([("n1".into(), 3.2)]),
                        component_results: vec![],
                        junction_temperatures: HashMap::new(),
                        state_of_charge: HashMap::new(),
                        motors: HashMap::new(),
                    },
                ],
                time_step: 0.001,
                duration: 0.002,
                stats: Default::default(),
            }),
        };
        let json = serde_json::to_string(&result).unwrap();
        assert!(json.contains("\"transient\""));
        assert!(json.contains("\"timesteps\""));
        assert!(json.contains("\"time_step\""));
        assert!(json.contains("\"duration\""));

        // Roundtrip: deserialize back
        let parsed: SimulationResult = serde_json::from_str(&json).unwrap();
        let td = parsed.transient.unwrap();
        assert_eq!(td.timesteps.len(), 2);
        assert_eq!(td.time_step, 0.001);
        assert_eq!(td.duration, 0.002);
        assert_eq!(td.timesteps[1].node_voltages["n1"], 3.2);
    }
}

#[cfg(test)]
mod extract_tests {
    use super::*;
    use crate::circuit::CircuitElement;
    use crate::device_params;
    use crate::element::{Capacitor, Inductor, Transformer, Varactor, VoltageSource, ZenerDiode};
    use crate::handle::NodeId;

    /// `extract_results` on elements the analyses lower before solving (a
    /// Zener to a diode, C open, L and transformer windings to 0 V
    /// sources): each still reports what it carries at a given solution,
    /// and a transformer's two branch slots keep later sources reading
    /// their own branch current.
    #[test]
    fn raw_elements_report_their_dc_terminal_values() {
        let two = |a: &str, b: &str| -> [NodeId; 2] { [a.into(), b.into()] };
        let circuit = Circuit::new(
            "0",
            vec![
                CircuitElement::ZenerDiode(ZenerDiode {
                    id: "DZ1".into(),
                    nodes: two("0", "a"),
                    vz: 5.1,
                    temperature: 300.15,
                }),
                CircuitElement::Varactor(Varactor {
                    id: "CV1".into(),
                    nodes: two("b", "a"),
                    params: sindr_devices::varactor::VaractorParams::default(),
                }),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: two("a", "0"),
                    capacitance: 1e-6,
                }),
                CircuitElement::Inductor(Inductor {
                    id: "L1".into(),
                    nodes: two("a", "b"),
                    inductance: 1e-3,
                    saturation: None,
                }),
                CircuitElement::Transformer(Transformer {
                    id: "T1".into(),
                    nodes: ["a".into(), "b".into(), "b".into(), "0".into()],
                    l1: 1e-3,
                    l2: 1e-3,
                    k: 0.99,
                    saturation: None,
                }),
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: two("a", "0"),
                    voltage: 6.0,
                    waveform: None,
                }),
            ],
        );
        let node_map = NodeMap::from_circuit(&circuit);
        let n = node_map.num_nodes();
        // V(a) = 6, V(b) = 2; branches: T1 primary, T1 secondary, V1.
        let mut solution = DVector::zeros(n + 3);
        solution[node_map.index("a").unwrap()] = 6.0;
        solution[node_map.index("b").unwrap()] = 2.0;
        solution[n + 2] = -0.25;
        let result = extract_results(&circuit, &node_map, &solution, n);
        let get = |id: &str| {
            result
                .component_results
                .iter()
                .find(|r| r.id == id)
                .unwrap_or_else(|| panic!("no result for {id}"))
        };

        // The Zener sits 6 V reverse-biased, past its 5.1 V knee: the
        // diode model's breakdown current, reported in the forward sense.
        let dz = get("DZ1");
        let expected =
            sindr_devices::diode::diode_current(-6.0, &device_params::zener_diode(5.1, 300.15));
        assert_eq!(dz.voltage_across, -6.0);
        assert_eq!(dz.current_through, expected);
        assert!(expected < -1e-3, "in breakdown: {expected} A");
        assert_eq!(dz.power, -6.0 * expected);

        let cv = get("CV1");
        assert_eq!(
            (cv.voltage_across, cv.current_through, cv.power),
            (-4.0, 0.0, 0.0)
        );
        for id in ["C1", "L1"] {
            let r = get(id);
            assert_eq!(
                (r.voltage_across, r.current_through, r.power),
                (0.0, 0.0, 0.0)
            );
        }
        let t1 = get("T1");
        assert_eq!(
            (t1.voltage_across, t1.current_through, t1.power),
            (4.0, 0.0, 0.0)
        );
        assert_eq!(get("V1").current_through, -0.25);
        assert_eq!(result.branch_currents["V1"], -0.25);
    }
}
