//! The test bench: what a schematic leaves out, and the analysis to run.

use super::{parse_value, ImportError, ImportOptions, ImportedCircuit, Netlist, Pin, Value};
use crate::ac_analysis::{solve_ac, AcConfig, AcResult};
use crate::circuit::{BehaviouralOutput, Circuit, CircuitElement};
use crate::dc_sweep::{dc_sweep, DcSweepResult};
use crate::element::{Behavioural, Capacitor, CurrentSource, Inductor, Resistor, VoltageSource};
use crate::expression::Expression;
use crate::results::SimulationResult;
use crate::transient::{solve_transient, TransientConfig};
use crate::waveform::Waveform;
use crate::SimError;

/// Where a bench item connects: a net, or a component's terminal (a
/// connector pin, a test point) standing for the net it is on.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Attach {
    /// A net, by its id, or by its display name when no net has that id
    /// and only one net has that name.
    Net(String),
    /// A component's terminal.
    Terminal(Pin),
}

impl From<&str> for Attach {
    fn from(net: &str) -> Self {
        Attach::Net(net.to_string())
    }
}

impl From<String> for Attach {
    fn from(net: String) -> Self {
        Attach::Net(net)
    }
}

impl From<Pin> for Attach {
    fn from(pin: Pin) -> Self {
        Attach::Terminal(pin)
    }
}

/// What a [`BenchSource`] drives.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(tag = "type", rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum SourceKind {
    /// A voltage, `plus` above `minus`.
    Voltage {
        /// DC voltage (V); the offset of any waveform.
        volts: f64,
        /// A time-varying part added on top, for transient.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        waveform: Option<Waveform>,
    },
    /// A current, flowing out of the source into `plus`'s net and back
    /// from `minus`'s: it drives `amps` into `plus`.
    Current {
        /// DC current (A); the offset of any waveform.
        amps: f64,
        /// A time-varying part added on top, for transient.
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        waveform: Option<Waveform>,
    },
}

/// A source the bench adds: a supply on a net, or a signal into an input.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct BenchSource {
    /// Its id, unique among components and bench items. Results are keyed
    /// by it, and an AC or DC sweep names its source by it.
    pub id: String,
    /// Voltage or current, with any waveform.
    #[cfg_attr(feature = "serde", serde(flatten))]
    pub kind: SourceKind,
    /// The positive side.
    pub plus: Attach,
    /// The negative side; `None` is the bench's ground.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub minus: Option<Attach>,
}

impl BenchSource {
    /// A DC voltage source from `plus` to ground.
    pub fn voltage(id: impl Into<String>, plus: impl Into<Attach>, volts: f64) -> Self {
        Self {
            id: id.into(),
            kind: SourceKind::Voltage {
                volts,
                waveform: None,
            },
            plus: plus.into(),
            minus: None,
        }
    }

    /// A DC current source driving `amps` into `plus`, returning through
    /// ground.
    pub fn current(id: impl Into<String>, plus: impl Into<Attach>, amps: f64) -> Self {
        Self {
            id: id.into(),
            kind: SourceKind::Current {
                amps,
                waveform: None,
            },
            plus: plus.into(),
            minus: None,
        }
    }

    /// Sets the negative side.
    pub fn with_minus(mut self, minus: impl Into<Attach>) -> Self {
        self.minus = Some(minus.into());
        self
    }

    /// Adds a waveform on top of the DC value.
    pub fn with_waveform(mut self, waveform: Waveform) -> Self {
        match &mut self.kind {
            SourceKind::Voltage { waveform: w, .. } | SourceKind::Current { waveform: w, .. } => {
                *w = Some(waveform)
            }
        }
        self
    }

    pub(super) fn element(&self, nodes: [String; 2]) -> CircuitElement {
        let id = self.id.clone();
        match &self.kind {
            SourceKind::Voltage { volts, waveform } => {
                CircuitElement::VoltageSource(VoltageSource {
                    id,
                    nodes,
                    voltage: *volts,
                    waveform: waveform.clone(),
                })
            }
            // sindr's current source pushes current from nodes[0] to
            // nodes[1] through itself, so it drives nodes[1]'s net.
            SourceKind::Current { amps, waveform } => {
                let [plus, minus] = nodes;
                CircuitElement::CurrentSource(CurrentSource {
                    id,
                    nodes: [minus, plus],
                    current: *amps,
                    waveform: waveform.clone(),
                })
            }
        }
    }
}

/// What a [`BenchLoad`] is.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum LoadKind {
    /// A resistance (Ω).
    Resistor(f64),
    /// A capacitance (F).
    Capacitor(f64),
    /// An inductance (H).
    Inductor(f64),
    /// A constant current drawn from `a`'s net and returned into `b`'s,
    /// such as an IC's supply current: `amps` flows into the load at `a`
    /// and out of it at `b`. For a current that changes with time, use a
    /// [`SourceKind::Current`] with a waveform, negated: it drives its
    /// current into `plus`, so a sink at `a` is a source on `a` with a
    /// negative value.
    CurrentSink {
        /// The current drawn (A).
        amps: f64,
    },
    /// A switch between `a` and `b` that changes state once, at `at`: an
    /// open-drain output holding a line low and then letting go, or a
    /// short that closes part way through. It is `on_resistance` while
    /// closed and `off_resistance` (the leakage) while open.
    ///
    /// The operating point, a DC sweep and AC take its state at t = 0. A
    /// transient lands a step on `at`; that step still sees the old state,
    /// and the next one the new, as a source's waveform jumps. With `at ≤
    /// 0` it is in its switched state from the start.
    TimedSwitch {
        /// Resistance while closed (Ω).
        on_resistance: f64,
        /// Resistance while open (Ω). In JSON it defaults to 1 GΩ, as a
        /// [`Switch`](CircuitElement::Switch) has.
        #[cfg_attr(feature = "serde", serde(default = "default_off_resistance"))]
        off_resistance: f64,
        /// When it switches (s).
        at: f64,
        /// Closed until `at` and open after it (`true`, the default in
        /// JSON), or open until `at` and closed after it.
        #[cfg_attr(feature = "serde", serde(default = "default_starts_closed"))]
        starts_closed: bool,
    },
}

#[cfg(feature = "serde")]
fn default_off_resistance() -> f64 {
    1e9
}

#[cfg(feature = "serde")]
fn default_starts_closed() -> bool {
    true
}

impl LoadKind {
    /// A [`CurrentSink`](Self::CurrentSink) drawing `amps`.
    pub fn current_sink(amps: f64) -> Self {
        LoadKind::CurrentSink { amps }
    }

    /// A [`TimedSwitch`](Self::TimedSwitch) closed through `on_resistance`
    /// until `at`, then open (1 GΩ): a line held low and released.
    pub fn released_at(at: f64, on_resistance: f64) -> Self {
        LoadKind::TimedSwitch {
            on_resistance,
            off_resistance: 1e9,
            at,
            starts_closed: true,
        }
    }

    /// A [`TimedSwitch`](Self::TimedSwitch) open (1 GΩ) until `at`, then
    /// closed through `on_resistance`.
    pub fn closed_at(at: f64, on_resistance: f64) -> Self {
        LoadKind::TimedSwitch {
            on_resistance,
            off_resistance: 1e9,
            at,
            starts_closed: false,
        }
    }

    /// Sets a [`TimedSwitch`](Self::TimedSwitch)'s off resistance (Ω);
    /// any other kind is returned unchanged.
    pub fn with_off_resistance(mut self, ohms: f64) -> Self {
        if let LoadKind::TimedSwitch { off_resistance, .. } = &mut self {
            *off_resistance = ohms;
        }
        self
    }
}

/// A load the bench adds: what an output drives, a probe's capacitance.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct BenchLoad {
    /// Its id, unique among components and bench items.
    pub id: String,
    /// What it is.
    pub kind: LoadKind,
    /// One side.
    pub a: Attach,
    /// The other side; `None` is the bench's ground.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub b: Option<Attach>,
}

impl BenchLoad {
    /// A load from `a` to ground.
    pub fn new(id: impl Into<String>, kind: LoadKind, a: impl Into<Attach>) -> Self {
        Self {
            id: id.into(),
            kind,
            a: a.into(),
            b: None,
        }
    }

    /// Sets the other side.
    pub fn with_b(mut self, b: impl Into<Attach>) -> Self {
        self.b = Some(b.into());
        self
    }

    pub(super) fn element(&self, nodes: [String; 2]) -> Result<CircuitElement, ImportError> {
        let id = self.id.clone();
        Ok(match self.kind {
            LoadKind::Resistor(resistance) => CircuitElement::Resistor(Resistor {
                id,
                nodes,
                resistance,
            }),
            LoadKind::Capacitor(capacitance) => CircuitElement::Capacitor(Capacitor {
                id,
                nodes,
                capacitance,
            }),
            LoadKind::Inductor(inductance) => CircuitElement::Inductor(Inductor {
                id,
                nodes,
                inductance,
                saturation: None,
            }),
            // A current source pushes from nodes[0] to nodes[1] through
            // itself, so it draws from a's net into b's.
            LoadKind::CurrentSink { amps } => CircuitElement::CurrentSource(CurrentSource {
                id,
                nodes,
                current: amps,
                waveform: None,
            }),
            // A behavioural current v(a, b)·g(t): Newton stamps it as the
            // conductance it is, and the transient lands a step on `at`
            // from the `time < at` it compares.
            LoadKind::TimedSwitch {
                on_resistance,
                off_resistance,
                at,
                starts_closed,
            } => {
                let ohms = |field: &str, r: f64| {
                    if r.is_finite() && r > 0.0 {
                        Ok(1.0 / r)
                    } else {
                        Err(self.bad(field, r, "a positive resistance (Ω)"))
                    }
                };
                let g_on = ohms("on_resistance", on_resistance)?;
                let g_off = ohms("off_resistance", off_resistance)?;
                if !at.is_finite() {
                    return Err(self.bad("at", at, "a finite time (s)"));
                }
                let (before, after) = if starts_closed {
                    (g_on, g_off)
                } else {
                    (g_off, g_on)
                };
                let [a, b] = &nodes;
                let expr = Expression::timed_conductance(a, b, at, before, after);
                CircuitElement::Behavioural(Behavioural {
                    id,
                    nodes,
                    output: BehaviouralOutput::Current,
                    expr,
                })
            }
        })
    }

    fn bad(&self, field: &str, given: f64, expected: &str) -> ImportError {
        ImportError::BadValue {
            component: self.id.clone(),
            field: field.to_string(),
            given: Value::Number(given),
            expected: expected.to_string(),
        }
    }
}

/// The analysis a [`Bench`] runs.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", serde(tag = "type", rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Analysis {
    /// The DC operating point: capacitors open, inductors shorted,
    /// waveforms at their DC value. Self-heating applies.
    OperatingPoint,
    /// A transient run, as [`solve_transient`].
    Transient(TransientConfig),
    /// A small-signal frequency sweep, as [`solve_ac`]. `source_id` names
    /// a bench source.
    Ac(AcConfig),
    /// A DC sweep of one source's value, as [`dc_sweep`].
    DcSweep {
        /// The bench source to sweep.
        source: String,
        /// First value (V or A).
        start: f64,
        /// Last value (V or A).
        stop: f64,
        /// Number of points, both ends included.
        points: usize,
    },
}

/// What a schematic leaves out: the ground net, the sources and loads that
/// drive it, nets to tie together, and the analysis.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq)]
pub struct Bench {
    /// The reference (0 V) net.
    pub ground: Attach,
    /// Pairs of nets to join into one node, such as an analog ground and
    /// the ground.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Vec::is_empty")
    )]
    pub ties: Vec<[Attach; 2]>,
    /// Sources: supplies, signals.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Vec::is_empty")
    )]
    pub sources: Vec<BenchSource>,
    /// Loads.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Vec::is_empty")
    )]
    pub loads: Vec<BenchLoad>,
    /// The analysis [`simulate`] runs.
    pub analysis: Analysis,
}

impl Bench {
    /// A bench grounding `ground`, with nothing attached, running the
    /// operating point.
    pub fn new(ground: impl Into<Attach>) -> Self {
        Self {
            ground: ground.into(),
            ties: Vec::new(),
            sources: Vec::new(),
            loads: Vec::new(),
            analysis: Analysis::OperatingPoint,
        }
    }

    /// Adds a source.
    pub fn with_source(mut self, source: BenchSource) -> Self {
        self.sources.push(source);
        self
    }

    /// Adds a load.
    pub fn with_load(mut self, load: BenchLoad) -> Self {
        self.loads.push(load);
        self
    }

    /// Joins two nets.
    pub fn with_tie(mut self, a: impl Into<Attach>, b: impl Into<Attach>) -> Self {
        self.ties.push([a.into(), b.into()]);
        self
    }

    /// Sets the analysis.
    pub fn with_analysis(mut self, analysis: Analysis) -> Self {
        self.analysis = analysis;
        self
    }

    /// Adds a DC supply to ground on every net of `netlist` whose display
    /// name is a voltage: `3V3`, `+5V`, `5V0`, `12V`, `-12V`, `+3.3V`.
    /// Names that aren't (`VCC`, `VDD`, `GND`) get nothing, and neither do
    /// decorated ones (`3V3_A`, `12V_IN`): those are often rails derived
    /// through a filter or a switch, and a source on them would short it.
    /// Each source's id is `supply:` and the net's id. Never done unless
    /// called: a power port's name is only a hint. [`supply_names`] says
    /// which supplies this assumes and which supply-like names it left for
    /// the caller to set.
    pub fn with_supplies_from_net_names(mut self, netlist: &Netlist) -> Self {
        for supply in supply_names(netlist).assumed {
            self.sources.push(BenchSource::voltage(
                format!("supply:{}", supply.net),
                Attach::Net(supply.net),
                supply.volts,
            ));
        }
        self
    }
}

/// What [`Bench::with_supplies_from_net_names`] makes of a netlist's net
/// names, for a UI to show before simulating ("assumed 3.3 V on 3V3; set
/// VCC").
#[derive(Debug, Clone, Default, PartialEq)]
#[non_exhaustive]
pub struct SupplyNames {
    /// Nets whose name is a voltage, each getting a supply of that voltage,
    /// in netlist order.
    pub assumed: Vec<AssumedSupply>,
    /// Nets whose name looks like a supply but isn't a plain voltage
    /// (`VCC`, `VDD`, `VBAT`, `3V3_A`), in netlist order, by net id: they get
    /// nothing, so the caller sets them or leaves them to the circuit.
    pub unresolved: Vec<String>,
    /// Nets named like a ground (`GND`, `AGND`, `0V`), in netlist order, by
    /// net id. One is the bench's ground; joining the others to it is the
    /// caller's call, with [`Bench::with_tie`].
    pub grounds: Vec<String>,
}

/// A supply assumed from a net's name.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct AssumedSupply {
    /// The net's id.
    pub net: String,
    /// Its display name, which states the voltage.
    pub name: String,
    /// The voltage to ground, in volts.
    pub volts: f64,
}

/// Reads `netlist`'s net display names as power ports: which state a
/// voltage, which look like supplies without one, and which are grounds.
/// Changes nothing; see [`Bench::with_supplies_from_net_names`].
///
/// ```
/// use sindr::import::{supply_names, Net, Netlist};
///
/// let netlist = Netlist {
///     nets: vec![
///         Net::new("n1", [("r1", "1")]).with_name("3V3"),
///         Net::new("n2", [("r1", "2")]).with_name("VCC"),
///         Net::new("n3", [("r2", "1")]).with_name("GND"),
///     ],
///     ..Default::default()
/// };
/// let names = supply_names(&netlist);
/// assert_eq!((names.assumed[0].net.as_str(), names.assumed[0].volts), ("n1", 3.3));
/// assert_eq!(names.unresolved, ["n2"]);
/// assert_eq!(names.grounds, ["n3"]);
/// ```
pub fn supply_names(netlist: &Netlist) -> SupplyNames {
    let mut names = SupplyNames::default();
    for net in &netlist.nets {
        let Some(name) = net.name.as_deref() else {
            continue;
        };
        // A ground first: `0V` states a voltage, but a source on the
        // ground net would sit between ground and itself.
        if is_ground_name(name) {
            names.grounds.push(net.id.clone());
        } else if let Some(volts) = voltage_from_name(name) {
            names.assumed.push(AssumedSupply {
                net: net.id.clone(),
                name: name.to_string(),
                volts,
            });
        } else if is_supply_like_name(name) {
            names.unresolved.push(net.id.clone());
        }
    }
    names
}

/// `GND`, `AGND`, `PGND`, `GND_ISO`, `0V`.
fn is_ground_name(name: &str) -> bool {
    let n = name.trim().to_ascii_uppercase();
    n.contains("GND") || n == "0V" || n == "0"
}

/// A name a power port would carry without stating a plain voltage:
/// `VCC`, `VDD`, `VEE`, `VSS`, `VBAT`, `VIN`, `VBUS`, `V+`, `AVDD`,
/// `+VCC`, or a decorated voltage such as `3V3_A` or `+12V_IN`.
fn is_supply_like_name(name: &str) -> bool {
    let n = name.trim().to_ascii_uppercase();
    let bare = n.trim_start_matches(['+', '-']);
    let rail = [
        "VCC", "VDD", "VEE", "VSS", "VBAT", "VIN", "VBUS", "VSYS", "VREF", "VPP",
    ];
    (bare.starts_with(|c: char| c.is_ascii_digit()) && bare.contains('V'))
        || bare.starts_with("V+")
        || bare.starts_with("V-")
        || rail
            .iter()
            .any(|r| bare.starts_with(r) || bare.ends_with(r))
}

/// The voltage a power net's name states: `3V3`, `+5V`, `-12V`, `3.3V`.
pub(super) fn voltage_from_name(name: &str) -> Option<f64> {
    let t = name.trim();
    let digits = t.trim_start_matches(['+', '-']);
    if !digits.starts_with(|c: char| c.is_ascii_digit()) || !digits.contains(['V', 'v']) {
        return None;
    }
    parse_value(t)
}

/// A netlist built against a bench, and the result of the bench's analysis.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct Imported {
    /// The circuit that ran, and the node of each net.
    pub circuit: ImportedCircuit,
    /// The analysis' result.
    pub result: Simulation,
}

/// The result of a [`Bench`]'s analysis.
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
#[non_exhaustive]
pub enum Simulation {
    /// [`Analysis::OperatingPoint`].
    OperatingPoint(SimulationResult),
    /// [`Analysis::Transient`].
    Transient(SimulationResult),
    /// [`Analysis::Ac`].
    Ac(AcResult),
    /// [`Analysis::DcSweep`].
    DcSweep(DcSweepResult),
}

/// Why [`simulate`] failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum BenchError {
    /// The netlist and bench don't make a circuit.
    #[error(transparent)]
    Import(#[from] ImportError),
    /// The circuit didn't solve.
    #[error(transparent)]
    Sim(#[from] SimError),
}

/// Builds `netlist` against `bench` ([`Netlist::to_circuit`]) and runs the
/// bench's analysis.
///
/// # Errors
///
/// [`BenchError::Import`] when the circuit can't be built,
/// [`BenchError::Sim`] when it doesn't solve.
pub fn simulate(netlist: &Netlist, bench: &Bench) -> Result<Imported, BenchError> {
    simulate_with(netlist, bench, &ImportOptions::default())
}

/// [`simulate`], reading the netlist as `options` say
/// ([`Netlist::to_circuit_with`]): terminal names from each component's
/// table, unknown kinds refused or kept as black boxes. The black boxes
/// are listed in the result's [`ImportedCircuit::black_boxes`].
///
/// ```
/// use sindr::import::{simulate_with, Bench, BenchSource, Component, ComponentKind,
///                     ImportOptions, Net, Netlist, Simulation};
///
/// // An LED whose nets name its terminals by id.
/// let netlist = Netlist {
///     components: vec![
///         Component::new("r1", ComponentKind::from_name("resistor")).with_value("470"),
///         Component::new("d1", ComponentKind::from_name("led")).with_value("Green"),
///     ],
///     nets: vec![
///         Net::new("vcc", [("r1", "1")]),
///         Net::new("a", [("r1", "2"), ("d1", "1")]),
///         Net::new("gnd", [("d1", "2")]),
///     ],
/// };
/// let options = ImportOptions::new().with_terminal_names("d1", [("1", "Anode"), ("2", "Cathode")]);
/// let bench = Bench::new("gnd").with_source(BenchSource::voltage("vs", "vcc", 5.0));
///
/// let Simulation::OperatingPoint(op) = simulate_with(&netlist, &bench, &options)?.result else {
///     unreachable!()
/// };
/// assert!(op.node_voltages["a"] > 1.5 && op.node_voltages["a"] < 2.2);
/// # Ok::<(), sindr::import::BenchError>(())
/// ```
///
/// # Errors
///
/// As [`simulate`].
pub fn simulate_with(
    netlist: &Netlist,
    bench: &Bench,
    options: &ImportOptions,
) -> Result<Imported, BenchError> {
    let circuit = netlist.to_circuit_with(bench, options)?;
    let result = match &bench.analysis {
        Analysis::OperatingPoint => Simulation::OperatingPoint(operating_point(&circuit.circuit)?),
        Analysis::Transient(config) => {
            Simulation::Transient(solve_transient(&circuit.circuit, config)?)
        }
        Analysis::Ac(config) => Simulation::Ac(solve_ac(&circuit.circuit, config)?),
        Analysis::DcSweep {
            source,
            start,
            stop,
            points,
        } => Simulation::DcSweep(dc_sweep(&circuit.circuit, source, *start, *stop, *points)?),
    };
    Ok(Imported { circuit, result })
}

/// The DC operating point with self-heating, whatever the circuit holds.
fn operating_point(circuit: &Circuit) -> Result<SimulationResult, SimError> {
    let flat = crate::solver_circuit(circuit)?;
    crate::solve_dc_operating_point(&flat, &crate::SolverOptions::default())
}
