//! The test bench: what a schematic leaves out, and the analysis to run.

use std::collections::HashMap;

use super::{parse_value, ImportError, ImportedCircuit, Netlist, Pin};
use crate::ac_analysis::{solve_ac, AcConfig, AcResult};
use crate::circuit::{Circuit, CircuitElement};
use crate::dc_sweep::{dc_sweep, DcSweepResult};
use crate::results::SimulationResult;
use crate::transient::{solve_transient, TransientConfig};
use crate::waveform::Waveform;
use crate::SimError;

/// Where a bench item connects: a net, or a component's terminal (a
/// connector pin, a test point) standing for the net it is on.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
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
#[cfg_attr(feature = "serde", serde(tag = "type", rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
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
            SourceKind::Voltage { volts, waveform } => CircuitElement::VoltageSource {
                id,
                nodes,
                voltage: *volts,
                waveform: waveform.clone(),
            },
            // sindr's current source pushes current from nodes[0] to
            // nodes[1] through itself, so it drives nodes[1]'s net.
            SourceKind::Current { amps, waveform } => {
                let [plus, minus] = nodes;
                CircuitElement::CurrentSource {
                    id,
                    nodes: [minus, plus],
                    current: *amps,
                    waveform: waveform.clone(),
                }
            }
        }
    }
}

/// What a [`BenchLoad`] is.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoadKind {
    /// A resistance (Ω).
    Resistor(f64),
    /// A capacitance (F).
    Capacitor(f64),
    /// An inductance (H).
    Inductor(f64),
}

/// A load the bench adds: what an output drives, a probe's capacitance.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

    pub(super) fn element(&self, nodes: [String; 2]) -> CircuitElement {
        let id = self.id.clone();
        match self.kind {
            LoadKind::Resistor(resistance) => CircuitElement::Resistor {
                id,
                nodes,
                resistance,
            },
            LoadKind::Capacitor(capacitance) => CircuitElement::Capacitor {
                id,
                nodes,
                capacitance,
            },
            LoadKind::Inductor(inductance) => CircuitElement::Inductor {
                id,
                nodes,
                inductance,
                saturation: None,
            },
        }
    }
}

/// The analysis a [`Bench`] runs.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "type", rename_all = "snake_case"))]
#[derive(Debug, Clone, PartialEq)]
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
    /// Names that aren't (`VCC`, `VDD`, `GND`) get nothing. Each source's
    /// id is `supply:` and the net's id. Never done unless called: a power
    /// port's name is only a hint.
    pub fn with_supplies_from_net_names(mut self, netlist: &Netlist) -> Self {
        for net in &netlist.nets {
            let Some(volts) = net.name.as_deref().and_then(voltage_from_name) else {
                continue;
            };
            self.sources.push(BenchSource::voltage(
                format!("supply:{}", net.id),
                Attach::Net(net.id.clone()),
                volts,
            ));
        }
        self
    }
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
pub struct Imported {
    /// The circuit that ran, and the node of each net.
    pub circuit: ImportedCircuit,
    /// The analysis' result.
    pub result: Simulation,
}

/// The result of a [`Bench`]'s analysis.
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
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
    let circuit = netlist.to_circuit(bench)?;
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
    crate::validation::validate_circuit(&flat)?;
    let dc = Circuit {
        parts: flat.parts.clone(),
        ..crate::dc_equivalent(&flat)
    };
    crate::solve_dc_heated(&dc, |c| crate::solve_dc_circuit(c, &HashMap::new()))
}
