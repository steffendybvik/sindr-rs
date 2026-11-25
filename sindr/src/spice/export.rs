//! Write a [`Circuit`] as a SPICE deck: [`to_netlist`].

use crate::element::{
    Capacitor, Cccs, CurrentSource, Diode, Inductor, Resistor, Switch, Vcvs, VoltageSource,
};
use std::collections::{BTreeMap, HashMap, HashSet};

use sindr_devices::diode::{DiodeParams, T_NOM};
use thiserror::Error;

use crate::circuit::{BjtModel, DiodeModel};
use crate::design::{Block, DesignError, Instance};
use crate::handle::NodeId;
use crate::stamp::{SWITCH_R_CLOSED, SWITCH_R_OPEN};
use crate::{BjtKind, Circuit, CircuitElement, Waveform};

/// Why [`to_netlist`] couldn't write a circuit.
#[derive(Debug, Clone, PartialEq, Error)]
#[non_exhaustive]
pub enum ExportError {
    /// The element has no SPICE equivalent that sindr can express.
    #[error("`{id}` can't be written as SPICE: {reason}")]
    Unsupported {
        /// The element's id.
        id: String,
        /// What has no SPICE equivalent.
        reason: String,
    },
    /// A node or element name a SPICE deck can't hold, or two names that
    /// SPICE, being case-insensitive, would merge.
    #[error("{what} `{name}` can't be written as SPICE: {reason}")]
    BadName {
        /// `"node"`, `"element"` or `"block"`.
        what: &'static str,
        /// The name as the circuit has it.
        name: String,
        /// What is wrong with it.
        reason: String,
    },
    /// The design hierarchy is broken: see [`DesignError`].
    #[error(transparent)]
    Design(#[from] DesignError),
    /// A value that is NaN or infinite.
    #[error("`{id}`: {field} is {value}, not a finite number")]
    NonFinite {
        /// The element's id.
        id: String,
        /// The field holding the value.
        field: &'static str,
        /// The value.
        value: f64,
    },
}

/// Writes `circuit` as a SPICE deck titled `title`: element lines, then one
/// `.model` card per distinct diode or transistor model, then `.end`. No
/// analysis cards are written.
///
/// The deck is SPICE3, so ngspice reads it, and [`parse_str`](super::parse_str)
/// reads back every element it can parse (R, L, C, V, I, D, Q, M, J) into a
/// circuit that solves the same.
///
/// # What is written
///
/// | Element | As |
/// |---|---|
/// | `Resistor`, `Capacitor`, `Inductor` | `R`, `C`, `L` |
/// | `VoltageSource`, `CurrentSource` | `V`, `I`, with `DC` or a waveform |
/// | `Diode`, `Bjt` | `D`, `Q`, with a `.model` card |
/// | `Mosfet` | `M d g s s` with `W=1 L=1`, so a level-1 card's `KP`, `CGSO` and `CGDO` are sindr's values as they are. A body diode with only `IS`, `CJO`, `VJ`, `M` and `FC` is the card's drain–bulk junction; one with more is a `D<name>_body` line beside it, and the card's `IS` is 0 |
/// | `Jfet` | `J`, with an `NJF`/`PJF` card: `VTO` the pinch-off, `BETA = idss/VTO²`, `IS=0` for sindr's ideal gate, and the gate capacitances as `CGS`, `CGD`, `PB`, `FC` (and `M` when it isn't 0.5, which ngspice ignores) |
/// | `Vcvs`, `Vccs`, `Ccvs`, `Cccs` | `E`, `G`, `H`, `F` |
/// | `Behavioural` | `B n+ n- V={expr}` or `I={expr}`, its nodes and sources by their deck names |
/// | `Transformer` | two `L` and a `K` |
/// | `Switch`, `Pushbutton`, `Fuse` | `R` at the resistance sindr stamps |
/// | `SwitchSpdt` | two `R`, common to each throw |
/// | `Crystal` | `R`, `L`, `C` in series (the motional arm) beside `C` (C0) |
/// | `FerriteBead` | `R` (DC) into `L` ∥ `R` (∥ `C`) |
/// | `DcMotor` | the armature `R`, `L`, a 0 V `V` sensing its current and the back-EMF `E`; the shaft as `F` (`kt·i`), `C` (J), `R` (1/B) and `I` (the load) on a node whose voltage over `b` is the speed |
/// | `Potentiometer` | two `R`, top–wiper and wiper–bottom |
/// | `Photoresistor`, `Thermistor` | `R` at their resistance |
/// | `Led`, `SchottkyDiode` | `D` with their Shockley parameters |
///
/// A source's waveform and its static value are written as one waveform:
/// SPICE ignores a `DC` value during a transient, where sindr adds it.
/// Square, triangle and PWM waves become the `PULSE` that traces the same
/// wave; a sine's phase becomes `SIN`'s phase, in degrees.
///
/// sindr's parser reads every line written here back.
///
/// # Names
///
/// The ground node is written as `0`. An element keeps its id when the id
/// starts with its SPICE letter (`R1` for a resistor); otherwise the letter
/// is put in front (`load` becomes `Rload`). Elements written as two lines
/// add `_top`/`_bottom` (potentiometer) or `_p`/`_s` (transformer windings).
/// SPICE folds case, and sindr's parser reads everything as lowercase.
///
/// # Errors
///
/// [`ExportError::Unsupported`] for an element with no SPICE equivalent:
/// a MOSFET with flicker noise, a JFET with a pinch-off SPICE can't write,
/// IGBT, zener, op-amp, comparator, relay, photodiode,
/// varactor, voltage regulator, a BJT with parasitic capacitances, a junction
/// away from 27 °C (sindr reads no temperature from a deck), or a phased sine
/// of zero frequency. [`ExportError::BadName`] for a name holding whitespace
/// or SPICE punctuation, a node other than ground named `0` or `gnd`, two
/// names that differ only in case, or an F/H source whose controlling source
/// isn't a voltage source. [`ExportError::NonFinite`] for a NaN or infinite
/// value.
///
/// ```
/// use sindr::{Circuit, CircuitElement};
/// use sindr::element::{Resistor, VoltageSource};
///
/// let circuit = Circuit {
///     ground_node: "gnd".into(),
///     components: vec![
///         CircuitElement::VoltageSource(VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "gnd".into()],
///             voltage: 5.0,
///             waveform: None,
///         }),
///         CircuitElement::Resistor(Resistor {
///             id: "load".into(),
///             nodes: ["in".into(), "gnd".into()],
///             resistance: 1e3,
///         }),
///     ],
///     ..Default::default()
/// };
/// let deck = sindr::spice::to_netlist(&circuit, "divider")?;
/// assert_eq!(deck, "divider\nV1 in 0 DC 5\nRload in 0 1000\n.end\n");
/// # Ok::<(), sindr::spice::ExportError>(())
/// ```
///
pub fn to_netlist(circuit: &Circuit, title: &str) -> Result<String, ExportError> {
    circuit.flattened()?;
    let mut deck = Deck::new(&circuit.ground_node, &circuit.blocks);
    let mut lines = Vec::new();
    for global in &circuit.globals {
        check_token("node", global)?;
        lines.push(format!(".global {global}"));
    }
    lines.extend(deck.scope(&circuit.components, &circuit.instances)?);

    let mut block_names = HashMap::<String, String>::new();
    let bodies = circuit.blocks.iter().flat_map(|(block_name, block)| {
        std::iter::once(block.base_variant_name())
            .chain(block.variants.keys().map(String::as_str))
            .filter_map(move |variant| {
                let body = block.variant(variant)?;
                Some((subckt_name(block_name, block, variant), body))
            })
    });
    for (name, body) in bodies {
        check_token("block", &name)?;
        if let Some(other) = block_names.insert(name.to_ascii_lowercase(), name.clone()) {
            return Err(ExportError::BadName {
                what: "block",
                name,
                reason: format!("SPICE folds case, so it is the same block as `{other}`"),
            });
        }
        let lines_of_body = deck.scope(body.components, body.instances)?;
        let mut header = format!(".subckt {name}");
        for port in body.ports {
            if *port == circuit.ground_node {
                return Err(ExportError::BadName {
                    what: "node",
                    name: port.clone(),
                    reason: format!("block `{name}` has ground as a port"),
                });
            }
            header.push(' ');
            header.push_str(&deck.node(port)?);
        }
        lines.push(header);
        lines.extend(lines_of_body);
        lines.push(format!(".ends {name}"));
    }

    let mut out = String::new();
    out.push_str(title.lines().next().unwrap_or(""));
    out.push('\n');
    for line in lines.iter().chain(&deck.models) {
        out.push_str(line);
        out.push('\n');
    }
    out.push_str(".end\n");
    Ok(out)
}

/// The `.subckt` name a variant of a block is written under: the block's
/// name for its own body, `<block>_<variant>` for the others.
fn subckt_name(block_name: &str, block: &Block, variant: &str) -> String {
    if variant == block.base_variant_name() {
        block_name.to_string()
    } else {
        format!("{block_name}_{variant}")
    }
}

pub(crate) struct Deck<'a> {
    ground: &'a str,
    blocks: &'a BTreeMap<String, Block>,
    /// Each element id's SPICE name.
    names: HashMap<&'a str, String>,
    /// Ids of the voltage sources, which F and H may sense.
    voltage_sources: HashSet<&'a str>,
    /// Lowercased node names seen, with the spelling first seen.
    nodes: HashMap<String, String>,
    lines: Vec<String>,
    models: Vec<String>,
    diode_models: Vec<(DiodeModel, String)>,
    bjt_models: Vec<(BjtKind, f64, BjtModel, String)>,
    /// MOSFET and JFET cards written so far: `TYPE(params)` and its name.
    fet_models: Vec<(String, String)>,
}

impl<'a> Deck<'a> {
    fn new(ground: &'a str, blocks: &'a BTreeMap<String, Block>) -> Self {
        Self {
            ground,
            blocks,
            names: HashMap::new(),
            voltage_sources: HashSet::new(),
            nodes: HashMap::new(),
            lines: Vec::new(),
            models: Vec::new(),
            diode_models: Vec::new(),
            bjt_models: Vec::new(),
            fet_models: Vec::new(),
        }
    }

    /// Starts a scope (the top level or one block): names every element and
    /// instance up front, so an F or H line can name a source that comes
    /// after it. Models carry over; they are global in SPICE.
    fn begin_scope(
        &mut self,
        components: &'a [CircuitElement],
        instances: &'a [Instance],
    ) -> Result<(), ExportError> {
        self.names.clear();
        self.voltage_sources.clear();
        self.nodes.clear();
        self.lines.clear();
        let mut taken = HashMap::<String, &str>::new();
        let ids = components
            .iter()
            .map(|c| (c.id(), c.device().spice_letter()))
            .chain(instances.iter().map(|i| (i.id.as_str(), 'x')));
        for (id, letter) in ids {
            check_token("element", id)?;
            // Two elements with one id would share one entry in `names`, so
            // the first would be written under the second's SPICE name.
            if self.names.contains_key(id) {
                return Err(ExportError::BadName {
                    what: "element",
                    name: id.to_string(),
                    reason: "another element has the same id".to_string(),
                });
            }
            let name = if id
                .chars()
                .next()
                .is_some_and(|c| c.eq_ignore_ascii_case(&letter))
            {
                id.to_string()
            } else {
                format!("{}{id}", letter.to_ascii_uppercase())
            };
            if let Some(other) = taken.insert(name.to_ascii_lowercase(), id) {
                return Err(ExportError::BadName {
                    what: "element",
                    name: id.to_string(),
                    reason: format!("its SPICE name `{name}` is also `{other}`'s"),
                });
            }
            self.names.insert(id, name);
        }
        for component in components {
            if matches!(
                component,
                CircuitElement::VoltageSource(VoltageSource { .. })
            ) {
                self.voltage_sources.insert(component.id());
            }
        }
        Ok(())
    }

    /// Writes a whole scope's element and `X` lines, and returns them.
    fn scope(
        &mut self,
        components: &'a [CircuitElement],
        instances: &'a [Instance],
    ) -> Result<Vec<String>, ExportError> {
        self.begin_scope(components, instances)?;
        for component in components {
            self.element(component)?;
        }
        for instance in instances {
            let mut line = self.names[instance.id.as_str()].clone();
            for node in &instance.nodes {
                line.push(' ');
                line.push_str(&self.node(node)?);
            }
            line.push(' ');
            match self.blocks.get(&instance.block) {
                Some(block) => line.push_str(&subckt_name(
                    &instance.block,
                    block,
                    block.picked_variant(instance),
                )),
                None => line.push_str(&instance.block),
            }
            self.line(line);
        }
        Ok(std::mem::take(&mut self.lines))
    }

    pub(crate) fn node(&mut self, node: &str) -> Result<String, ExportError> {
        if node == self.ground {
            return Ok("0".to_string());
        }
        check_token("node", node)?;
        let lower = node.to_ascii_lowercase();
        if lower == "0" || lower == "gnd" {
            return Err(ExportError::BadName {
                what: "node",
                name: node.to_string(),
                reason: "SPICE reads it as ground, and it isn't the circuit's ground".to_string(),
            });
        }
        match self.nodes.get(&lower) {
            Some(seen) if seen != node => Err(ExportError::BadName {
                what: "node",
                name: node.to_string(),
                reason: format!("SPICE folds case, so it is the same node as `{seen}`"),
            }),
            Some(_) => Ok(node.to_string()),
            None => {
                self.nodes.insert(lower, node.to_string());
                Ok(node.to_string())
            }
        }
    }

    pub(crate) fn nodes<const N: usize>(
        &mut self,
        nodes: &[NodeId; N],
    ) -> Result<String, ExportError> {
        let mut out = Vec::with_capacity(N);
        for node in nodes {
            out.push(self.node(node)?);
        }
        Ok(out.join(" "))
    }

    /// `name` for a second line of element `id`, unless another element
    /// already has it.
    pub(crate) fn extra_name(&self, id: &str, name: String) -> Result<String, ExportError> {
        match self
            .names
            .iter()
            .find(|(_, n)| n.eq_ignore_ascii_case(&name))
        {
            Some((other, _)) => Err(ExportError::BadName {
                what: "element",
                name: id.to_string(),
                reason: format!("the SPICE name `{name}` it needs is also `{other}`'s"),
            }),
            None => Ok(name),
        }
    }

    pub(crate) fn line(&mut self, line: String) {
        self.lines.push(line);
    }

    /// A two-terminal line `<name> <a> <b> <value>`.
    pub(crate) fn two_terminal(
        &mut self,
        name: &str,
        nodes: &[NodeId; 2],
        value: String,
    ) -> Result<(), ExportError> {
        let nodes = self.nodes(nodes)?;
        self.line(format!("{name} {nodes} {value}"));
        Ok(())
    }

    /// Writes `component`'s lines under its SPICE name.
    fn element(&mut self, component: &CircuitElement) -> Result<(), ExportError> {
        let name = self.names[component.id()].clone();
        component.device().write_spice(self, &name)
    }

    /// The SPICE name of element `id` in this scope, if it has one.
    pub(crate) fn name_of(&self, id: &str) -> Option<&str> {
        self.names.get(id).map(String::as_str)
    }

    /// An independent source's line: its static value as `DC`, or its
    /// waveform card with the value folded in.
    pub(crate) fn source(
        &mut self,
        id: &str,
        name: &str,
        nodes: &[NodeId; 2],
        value: f64,
        waveform: Option<&Waveform>,
    ) -> Result<(), ExportError> {
        let value = match waveform {
            None => format!("DC {}", num(id, "value", value)?),
            Some(w) => waveform_card(id, w, value)?,
        };
        self.two_terminal(name, nodes, value)
    }

    /// The lines of an element solved as its parts
    /// ([`composite::lower`](crate::composite::lower)): one two-terminal
    /// line per part, named after `name`.
    pub(crate) fn parts(
        &mut self,
        element: &CircuitElement,
        name: &str,
    ) -> Result<(), ExportError> {
        let id = element.id();
        let parts = crate::composite::lower(element)
            .map_err(|e| unsupported(id, &e.to_string()))?
            .unwrap_or_default();
        // `<id>#m1` → `<id>_m1`: one SPICE-safe spelling for the parts'
        // names and the internal nodes.
        let local = |s: &str| s.replace('#', "_");
        // Each part's SPICE name, for an `F` part to name its sensing source.
        let mut written: HashMap<&str, String> = HashMap::new();
        for part in &parts {
            let suffix = part.id().rsplit('#').next().unwrap_or("");
            let letter = part.device().spice_letter();
            let base = name.strip_prefix(['R', 'r']).unwrap_or(name);
            let part_name = self.extra_name(id, format!("{letter}{base}_{suffix}"))?;
            written.insert(part.id(), part_name.clone());
            let nodes = [local(&part.nodes()[0]), local(&part.nodes()[1])].map(NodeId::from);
            let value = match part {
                CircuitElement::VoltageSource(VoltageSource { voltage, .. }) => {
                    format!("DC {}", num(id, "voltage", *voltage)?)
                }
                CircuitElement::Vcvs(Vcvs {
                    control_nodes,
                    gain,
                    ..
                }) => {
                    let control = control_nodes.each_ref().map(|n| NodeId::from(local(n)));
                    let gain = num(id, "gain", *gain)?;
                    self.controlled(&part_name, &nodes, &control, gain)?;
                    continue;
                }
                CircuitElement::Cccs(Cccs {
                    control_source,
                    alpha,
                    ..
                }) => {
                    let Some(control) = written.get(control_source.as_str()) else {
                        return Err(unsupported(id, "a part sensing a later part"));
                    };
                    let control = control.clone();
                    let out = self.nodes(&nodes)?;
                    let alpha = num(id, "alpha", *alpha)?;
                    self.line(format!("{part_name} {out} {control} {alpha}"));
                    continue;
                }
                CircuitElement::Resistor(Resistor { resistance, .. }) => {
                    num(id, "resistance", *resistance)?
                }
                CircuitElement::Inductor(Inductor { inductance, .. }) => {
                    num(id, "inductance", *inductance)?
                }
                CircuitElement::Capacitor(Capacitor { capacitance, .. }) => {
                    num(id, "capacitance", *capacitance)?
                }
                CircuitElement::Switch(Switch { closed, .. }) => {
                    num(id, "resistance", switch_resistance(*closed))?
                }
                CircuitElement::CurrentSource(CurrentSource { current, .. }) => {
                    format!("DC {}", num(id, "current", *current)?)
                }
                CircuitElement::Diode(Diode { model, .. }) => {
                    let model = model.unwrap_or_default();
                    self.diode(id, &part_name, &nodes, model)?;
                    continue;
                }
                CircuitElement::Behavioural(b) => {
                    let mut b = b.clone();
                    b.nodes = nodes;
                    b.expr.rename(local, |s| s.to_string());
                    crate::element::Device::write_spice(&b, self, &part_name)?;
                    continue;
                }
                _ => return Err(unsupported(id, "an unexpected part")),
            };
            self.two_terminal(&part_name, &nodes, value)?;
        }
        Ok(())
    }

    pub(crate) fn diode(
        &mut self,
        id: &str,
        name: &str,
        nodes: &[NodeId; 2],
        model: DiodeModel,
    ) -> Result<(), ExportError> {
        let model_name = match self.diode_models.iter().find(|(m, _)| *m == model) {
            Some((_, n)) => n.clone(),
            None => {
                let n = format!("dmod{}", self.diode_models.len() + 1);
                let mut card = format!(
                    ".model {n} D(IS={} N={} RS={}",
                    num(id, "is", model.is)?,
                    num(id, "n", model.n)?,
                    num(id, "rs", model.rs)?
                );
                // Charge and breakdown only when set, so a plain diode's
                // card reads as it always has.
                let default = DiodeModel::default();
                for (key, field, value, unset) in [
                    ("CJO", "cjo", model.cjo, default.cjo),
                    ("VJ", "vj", model.vj, default.vj),
                    ("M", "m", model.m, default.m),
                    ("FC", "fc", model.fc, default.fc),
                    ("TT", "tt", model.tt, default.tt),
                    ("IBV", "ibv", model.ibv, default.ibv),
                    ("EG", "eg", model.eg, default.eg),
                    ("XTI", "xti", model.xti, default.xti),
                ] {
                    if value != unset {
                        card.push_str(&format!(" {key}={}", num(id, field, value)?));
                    }
                }
                if let Some(bv) = model.bv {
                    card.push_str(&format!(" BV={}", num(id, "bv", bv)?));
                }
                card.push_str(&flicker(id, model.kf, model.af)?);
                card.push(')');
                self.models.push(card);
                self.diode_models.push((model, n.clone()));
                n
            }
        };
        self.two_terminal(name, nodes, model_name)
    }

    pub(crate) fn bjt_model(
        &mut self,
        id: &str,
        kind: BjtKind,
        bf: f64,
        model: BjtModel,
    ) -> Result<String, ExportError> {
        if let Some((.., n)) = self
            .bjt_models
            .iter()
            .find(|(k, b, m, _)| *k == kind && *b == bf && *m == model)
        {
            return Ok(n.clone());
        }
        let card = match kind {
            BjtKind::Npn => "NPN",
            BjtKind::Pnp => "PNP",
        };
        let n = format!("qmod{}", self.bjt_models.len() + 1);
        self.models.push(format!(
            ".model {n} {card}(IS={} BF={} BR={} NF={} NR={} VAF={}{}{})",
            num(id, "is", model.is)?,
            num(id, "bf", bf)?,
            num(id, "br", model.br)?,
            num(id, "nf", model.nf)?,
            num(id, "nr", model.nr)?,
            num(id, "vaf", model.vaf)?,
            flicker(id, model.kf, model.af)?,
            bjt_temperature(id, &model)?
        ));
        self.bjt_models.push((kind, bf, model, n.clone()));
        Ok(n)
    }

    /// The name of the card `TYPE(params)`, writing it on first use.
    pub(crate) fn fet_model(&mut self, prefix: &str, card: String) -> String {
        if let Some((_, n)) = self.fet_models.iter().find(|(c, _)| *c == card) {
            return n.clone();
        }
        let used = self
            .fet_models
            .iter()
            .filter(|(_, n)| n.starts_with(prefix))
            .count();
        let n = format!("{prefix}{}", used + 1);
        self.models.push(format!(".model {n} {card}"));
        self.fet_models.push((card, n.clone()));
        n
    }

    pub(crate) fn controlled(
        &mut self,
        name: &str,
        nodes: &[NodeId; 2],
        control_nodes: &[NodeId; 2],
        value: String,
    ) -> Result<(), ExportError> {
        let out = self.nodes(nodes)?;
        let control = self.nodes(control_nodes)?;
        self.line(format!("{name} {out} {control} {value}"));
        Ok(())
    }

    pub(crate) fn current_controlled(
        &mut self,
        name: &str,
        id: &str,
        nodes: &[NodeId; 2],
        control_source: &str,
        value: String,
    ) -> Result<(), ExportError> {
        if !self.voltage_sources.contains(control_source) {
            return Err(ExportError::BadName {
                what: "element",
                name: id.to_string(),
                reason: format!("its controlling source `{control_source}` isn't a voltage source in the circuit"),
            });
        }
        let control = self.names[control_source].clone();
        let out = self.nodes(nodes)?;
        self.line(format!("{name} {out} {control} {value}"));
        Ok(())
    }
}

/// An [`ExportError::Unsupported`] for element `id`.
pub(crate) fn unsupported(id: &str, reason: &str) -> ExportError {
    ExportError::Unsupported {
        id: id.to_string(),
        reason: reason.to_string(),
    }
}

/// A switch's resistance, as sindr stamps it (Ω).
pub(crate) fn switch_resistance(closed: bool) -> f64 {
    if closed {
        SWITCH_R_CLOSED
    } else {
        SWITCH_R_OPEN
    }
}

/// Rejects a name SPICE would split or misread.
fn check_token(what: &'static str, name: &str) -> Result<(), ExportError> {
    const PUNCTUATION: &[char] = &['(', ')', '=', ',', ';', '{', '}', '\'', '"', '*'];
    let reason = if name.is_empty() {
        "it is empty"
    } else if name.chars().any(char::is_whitespace) {
        "it contains whitespace"
    } else if name.contains(PUNCTUATION) {
        "it contains SPICE punctuation"
    } else if name.starts_with(['.', '+']) {
        "SPICE reads a line starting with `.` or `+` as a card or continuation"
    } else {
        return Ok(());
    };
    Err(ExportError::BadName {
        what,
        name: name.to_string(),
        reason: reason.to_string(),
    })
}

pub(crate) fn nominal(id: &str, temperature: f64) -> Result<(), ExportError> {
    if (temperature - T_NOM).abs() > 1e-6 {
        return Err(ExportError::Unsupported {
            id: id.to_string(),
            reason: format!(
                "its junction is at {temperature} K, and sindr reads no temperature from a deck"
            ),
        });
    }
    Ok(())
}

/// ` EG=… XTI=…` for a BJT card whose `IS` follows temperature other than
/// as silicon's; nothing for the defaults, which are SPICE's too.
fn bjt_temperature(id: &str, model: &BjtModel) -> Result<String, ExportError> {
    let default = BjtModel::default();
    let mut out = String::new();
    for (key, field, value, unset) in [
        ("EG", "eg", model.eg, default.eg),
        ("XTI", "xti", model.xti, default.xti),
    ] {
        if value != unset {
            out.push_str(&format!(" {key}={}", num(id, field, value)?));
        }
    }
    Ok(out)
}

/// ` KF=… AF=…` for a card with flicker noise; nothing when `kf` is 0, so
/// cards without it read as they always have.
pub(crate) fn flicker(id: &str, kf: f64, af: f64) -> Result<String, ExportError> {
    if kf == 0.0 {
        return Ok(String::new());
    }
    Ok(format!(
        " KF={} AF={}",
        num(id, "kf", kf)?,
        num(id, "af", af)?
    ))
}

pub(crate) fn shockley(params: DiodeParams) -> DiodeModel {
    DiodeModel {
        is: params.is,
        n: params.n,
        rs: params.rs,
        ..DiodeModel::default()
    }
}

/// `value` written so it reads back as the same `f64`: plain decimals in a
/// readable range, exponent notation outside it.
pub(crate) fn num(id: &str, field: &'static str, value: f64) -> Result<String, ExportError> {
    if !value.is_finite() {
        return Err(ExportError::NonFinite {
            id: id.to_string(),
            field,
            value,
        });
    }
    let magnitude = value.abs();
    Ok(if value == 0.0 {
        "0".to_string()
    } else if (1e-3..1e6).contains(&magnitude) {
        format!("{value}")
    } else {
        format!("{value:e}")
    })
}

/// A source's waveform card, with the source's static value `dc` folded in.
pub(crate) fn waveform_card(id: &str, waveform: &Waveform, dc: f64) -> Result<String, ExportError> {
    let n = |field, v| num(id, field, v);
    let pulse = |v1: f64, v2: f64, td: f64, tr: f64, tf: f64, pw: f64, per: f64| {
        Ok::<_, ExportError>(format!(
            "PULSE({} {} {} {} {} {} {})",
            n("v1", v1)?,
            n("v2", v2)?,
            n("delay", td)?,
            n("rise_time", tr)?,
            n("fall_time", tf)?,
            n("pulse_width", pw)?,
            n("period", per)?
        ))
    };
    // SPICE's SIN: trailing zero arguments are left out, and the phase is
    // in degrees.
    let sin = |vo: f64, va: f64, freq: f64, td: f64, theta: f64, phase: f64| {
        if freq == 0.0 && (phase != 0.0 || td != 0.0 || theta != 0.0) {
            return Err(ExportError::Unsupported {
                id: id.to_string(),
                reason: "SPICE reads a zero SIN frequency as 1/TSTOP".to_string(),
            });
        }
        let mut args = vec![
            n("offset", vo + dc)?,
            n("amplitude", va)?,
            n("frequency", freq)?,
            n("delay", td)?,
            n("damping", theta)?,
            n("phase", phase.to_degrees())?,
        ];
        while args.len() > 3 && args.last().is_some_and(|a| a == "0") {
            args.pop();
        }
        Ok(format!("SIN({})", args.join(" ")))
    };
    let period = |frequency: f64| {
        if frequency > 0.0 && frequency.is_finite() {
            Ok(1.0 / frequency)
        } else {
            Err(ExportError::Unsupported {
                id: id.to_string(),
                reason: format!("a periodic wave of frequency {frequency} Hz has no PULSE"),
            })
        }
    };
    match waveform {
        Waveform::Sine {
            amplitude,
            frequency,
            offset,
            phase,
        } => sin(*offset, *amplitude, *frequency, 0.0, 0.0, *phase),
        Waveform::DampedSine {
            amplitude,
            frequency,
            offset,
            delay,
            damping,
            phase,
        } => sin(*offset, *amplitude, *frequency, *delay, *damping, *phase),
        Waveform::Pulse {
            v1,
            v2,
            delay,
            rise_time,
            fall_time,
            pulse_width,
            period,
        } => {
            // A single pulse that never falls is SPICE's with its width and
            // period left to default to TSTOP.
            if *pulse_width == f64::INFINITY && !(*period > 0.0 && period.is_finite()) {
                return Ok(format!(
                    "PULSE({} {} {} {} {})",
                    n("v1", v1 + dc)?,
                    n("v2", v2 + dc)?,
                    n("delay", *delay)?,
                    n("rise_time", *rise_time)?,
                    n("fall_time", *fall_time)?
                ));
            }
            // sindr and SPICE both read a non-positive period as one pulse.
            let period = if *period > 0.0 && period.is_finite() {
                *period
            } else {
                0.0
            };
            pulse(
                v1 + dc,
                v2 + dc,
                *delay,
                *rise_time,
                *fall_time,
                *pulse_width,
                period,
            )
        }
        Waveform::Square {
            amplitude,
            frequency,
            offset,
            duty,
        } => {
            let t = period(*frequency)?;
            let mid = offset + dc;
            pulse(mid - amplitude, mid + amplitude, 0.0, 0.0, 0.0, duty * t, t)
        }
        Waveform::Pwm {
            amplitude,
            frequency,
            duty,
            offset,
        } => {
            let t = period(*frequency)?;
            let low = offset + dc;
            pulse(low, low + amplitude, 0.0, 0.0, 0.0, duty * t, t)
        }
        Waveform::Triangle {
            amplitude,
            frequency,
            offset,
        } => {
            let t = period(*frequency)?;
            let mid = offset + dc;
            pulse(
                mid - amplitude,
                mid + amplitude,
                0.0,
                t / 2.0,
                t / 2.0,
                0.0,
                t,
            )
        }
        Waveform::Pwl { points } => {
            if points.is_empty() {
                return Ok(format!("PWL(0 {})", n("value", dc)?));
            }
            let mut pairs = Vec::with_capacity(points.len());
            for (t, v) in points {
                pairs.push(format!("{} {}", n("time", *t)?, n("value", v + dc)?));
            }
            Ok(format!("PWL({})", pairs.join(" ")))
        }
    }
}
