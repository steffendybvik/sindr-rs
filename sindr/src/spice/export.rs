//! Write a [`Circuit`] as a SPICE deck: [`to_netlist`].

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

use sindr_devices::diode::{DiodeParams, T_NOM};
use thiserror::Error;

use crate::circuit::{BjtModel, DiodeModel};
use crate::design::{DesignError, Instance};
use crate::stamp::{ldr_resistance, SWITCH_R_CLOSED, SWITCH_R_OPEN};
use crate::{device_params, BjtKind, Circuit, CircuitElement, Waveform};

/// Resistance of an intact fuse, as sindr stamps it (Ω).
const FUSE_R_INTACT: f64 = 0.001;

/// Why [`to_netlist`] couldn't write a circuit.
#[derive(Debug, Clone, PartialEq, Error)]
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
/// reads back every element it can parse (R, L, C, V, I, D, Q) into a circuit
/// that solves the same.
///
/// # What is written
///
/// | Element | As |
/// |---|---|
/// | `Resistor`, `Capacitor`, `Inductor` | `R`, `C`, `L` |
/// | `VoltageSource`, `CurrentSource` | `V`, `I`, with `DC` or a waveform |
/// | `Diode`, `Bjt` | `D`, `Q`, with a `.model` card |
/// | `Vcvs`, `Vccs`, `Ccvs`, `Cccs` | `E`, `G`, `H`, `F` |
/// | `Transformer` | two `L` and a `K` |
/// | `Switch`, `Pushbutton`, `Fuse` | `R` at the resistance sindr stamps |
/// | `Potentiometer` | two `R`, top–wiper and wiper–bottom |
/// | `Photoresistor`, `Thermistor` | `R` at their resistance |
/// | `Led`, `SchottkyDiode` | `D` with their Shockley parameters |
///
/// A source's waveform and its static value are written as one waveform:
/// SPICE ignores a `DC` value during a transient, where sindr adds it.
/// Square, triangle and PWM waves become the `PULSE` that traces the same
/// wave; a sine's phase becomes `SIN`'s delay.
///
/// sindr's parser reads only R, L, C, V, I, D, Q, so a deck with E, G, F, H
/// or K is for other simulators.
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
/// MOSFET, JFET, IGBT, zener, op-amp, comparator, relay, photodiode,
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
///
/// let circuit = Circuit {
///     ground_node: "gnd".into(),
///     components: vec![
///         CircuitElement::VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "gnd".into()],
///             voltage: 5.0,
///             waveform: None,
///         },
///         CircuitElement::Resistor {
///             id: "load".into(),
///             nodes: ["in".into(), "gnd".into()],
///             resistance: 1e3,
///         },
///     ],
///     ..Default::default()
/// };
/// let deck = sindr::spice::to_netlist(&circuit, "divider")?;
/// assert_eq!(deck, "divider\nV1 in 0 DC 5\nRload in 0 1000\n.end\n");
/// # Ok::<(), sindr::spice::ExportError>(())
/// ```
pub fn to_netlist(circuit: &Circuit, title: &str) -> Result<String, ExportError> {
    circuit.flattened()?;
    let mut deck = Deck::new(&circuit.ground_node);
    let mut lines = Vec::new();
    for global in &circuit.globals {
        check_token("node", global)?;
        lines.push(format!(".global {global}"));
    }
    lines.extend(deck.scope(&circuit.components, &circuit.instances)?);

    let mut block_names = HashMap::<String, &str>::new();
    for (name, block) in &circuit.blocks {
        check_token("block", name)?;
        if let Some(other) = block_names.insert(name.to_ascii_lowercase(), name) {
            return Err(ExportError::BadName {
                what: "block",
                name: name.clone(),
                reason: format!("SPICE folds case, so it is the same block as `{other}`"),
            });
        }
        let body = deck.scope(&block.components, &block.instances)?;
        let mut header = format!(".subckt {name}");
        for port in &block.ports {
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
        lines.extend(body);
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

struct Deck<'a> {
    ground: &'a str,
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
}

impl<'a> Deck<'a> {
    fn new(ground: &'a str) -> Self {
        Self {
            ground,
            names: HashMap::new(),
            voltage_sources: HashSet::new(),
            nodes: HashMap::new(),
            lines: Vec::new(),
            models: Vec::new(),
            diode_models: Vec::new(),
            bjt_models: Vec::new(),
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
            .map(|c| (c.id(), spice_letter(c)))
            .chain(instances.iter().map(|i| (i.id.as_str(), 'x')));
        for (id, letter) in ids {
            check_token("element", id)?;
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
            if matches!(component, CircuitElement::VoltageSource { .. }) {
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
            line.push_str(&instance.block);
            self.line(line);
        }
        Ok(std::mem::take(&mut self.lines))
    }

    fn node(&mut self, node: &str) -> Result<String, ExportError> {
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

    fn nodes<const N: usize>(&mut self, nodes: &[String; N]) -> Result<String, ExportError> {
        let mut out = Vec::with_capacity(N);
        for node in nodes {
            out.push(self.node(node)?);
        }
        Ok(out.join(" "))
    }

    /// `name` for a second line of element `id`, unless another element
    /// already has it.
    fn extra_name(&self, id: &str, name: String) -> Result<String, ExportError> {
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

    fn line(&mut self, line: String) {
        self.lines.push(line);
    }

    /// A two-terminal line `<name> <a> <b> <value>`.
    fn two_terminal(
        &mut self,
        name: &str,
        nodes: &[String; 2],
        value: String,
    ) -> Result<(), ExportError> {
        let nodes = self.nodes(nodes)?;
        self.line(format!("{name} {nodes} {value}"));
        Ok(())
    }

    fn element(&mut self, component: &CircuitElement) -> Result<(), ExportError> {
        let id = component.id();
        let name = self.names[id].clone();
        let unsupported = |reason: &str| ExportError::Unsupported {
            id: id.to_string(),
            reason: reason.to_string(),
        };
        match component {
            CircuitElement::Resistor {
                nodes, resistance, ..
            } => self.two_terminal(&name, nodes, num(id, "resistance", *resistance)?),
            CircuitElement::Capacitor {
                nodes, capacitance, ..
            } => self.two_terminal(&name, nodes, num(id, "capacitance", *capacitance)?),
            CircuitElement::Inductor {
                nodes, inductance, ..
            } => self.two_terminal(&name, nodes, num(id, "inductance", *inductance)?),
            CircuitElement::VoltageSource {
                nodes,
                voltage: value,
                waveform,
                ..
            }
            | CircuitElement::CurrentSource {
                nodes,
                current: value,
                waveform,
                ..
            } => {
                let value = match waveform {
                    None => format!("DC {}", num(id, "value", *value)?),
                    Some(w) => waveform_card(id, w, *value)?,
                };
                self.two_terminal(&name, nodes, value)
            }
            CircuitElement::Switch { nodes, closed, .. }
            | CircuitElement::Pushbutton { nodes, closed, .. } => {
                let r = if *closed {
                    SWITCH_R_CLOSED
                } else {
                    SWITCH_R_OPEN
                };
                self.two_terminal(&name, nodes, num(id, "resistance", r)?)
            }
            CircuitElement::Fuse { nodes, blown, .. } => {
                let r = if *blown { SWITCH_R_OPEN } else { FUSE_R_INTACT };
                self.two_terminal(&name, nodes, num(id, "resistance", r)?)
            }
            CircuitElement::Photoresistor {
                nodes, light_level, ..
            } => {
                let r = ldr_resistance(*light_level);
                self.two_terminal(&name, nodes, num(id, "resistance", r)?)
            }
            CircuitElement::Thermistor {
                nodes, temperature, ..
            } => {
                let params = sindr_devices::thermistor::ThermistorParams::default();
                let r = sindr_devices::thermistor::thermistor_resistance(*temperature, &params);
                self.two_terminal(&name, nodes, num(id, "resistance", r)?)
            }
            CircuitElement::Potentiometer {
                nodes,
                resistance,
                position,
                ..
            } => {
                // The clamp keeps the wiper off the ends, as the stamp does.
                let pos = position.clamp(0.001, 0.999);
                let top = num(id, "resistance", resistance * pos)?;
                let bottom = num(id, "resistance", resistance * (1.0 - pos))?;
                let [t, w, b] = nodes.clone();
                let top_name = self.extra_name(id, format!("{name}_top"))?;
                let bottom_name = self.extra_name(id, format!("{name}_bottom"))?;
                self.two_terminal(&top_name, &[t, w.clone()], top)?;
                self.two_terminal(&bottom_name, &[w, b], bottom)
            }
            CircuitElement::Diode {
                nodes,
                temperature,
                model,
                ..
            } => {
                nominal(id, *temperature)?;
                let model = model.unwrap_or_default();
                self.diode(id, &name, nodes, model)
            }
            CircuitElement::Led {
                nodes,
                color,
                temperature,
                ..
            } => {
                nominal(id, *temperature)?;
                let model = shockley(device_params::led(color, T_NOM));
                self.diode(id, &name, nodes, model)
            }
            CircuitElement::SchottkyDiode {
                nodes, temperature, ..
            } => {
                nominal(id, *temperature)?;
                let model = shockley(device_params::schottky(T_NOM));
                self.diode(id, &name, nodes, model)
            }
            CircuitElement::Bjt {
                nodes,
                kind,
                bf,
                temperature,
                parasitic_caps,
                model,
                ..
            } => {
                nominal(id, *temperature)?;
                if parasitic_caps
                    .as_ref()
                    .is_some_and(|c| c.cbe != 0.0 || c.cbc != 0.0)
                {
                    return Err(unsupported(
                        "fixed parasitic capacitances have no SPICE BJT parameter",
                    ));
                }
                let model_name = self.bjt_model(id, *kind, *bf, model.unwrap_or_default())?;
                // SPICE's Q line is [collector, base, emitter]; sindr's is [b, c, e].
                let [b, c, e] = nodes.clone();
                let nodes = self.nodes(&[c, b, e])?;
                self.line(format!("{name} {nodes} {model_name}"));
                Ok(())
            }
            CircuitElement::Vcvs {
                nodes,
                control_nodes,
                gain,
                ..
            } => self.controlled(&name, nodes, control_nodes, num(id, "gain", *gain)?),
            // sindr's VCCS and CCCS push their current out of `nodes[0]`
            // into the circuit; SPICE's G and F draw it in at `n+`. Checked
            // against ngspice by the `op_exported_controlled` golden.
            CircuitElement::Vccs {
                nodes,
                control_nodes,
                gm,
                ..
            } => self.controlled(&name, &swapped(nodes), control_nodes, num(id, "gm", *gm)?),
            CircuitElement::Ccvs {
                nodes,
                control_source,
                rm,
                ..
            } => {
                let value = num(id, "rm", *rm)?;
                self.current_controlled(&name, id, nodes, control_source, value)
            }
            CircuitElement::Cccs {
                nodes,
                control_source,
                alpha,
                ..
            } => {
                let value = num(id, "alpha", *alpha)?;
                self.current_controlled(&name, id, &swapped(nodes), control_source, value)
            }
            CircuitElement::Transformer {
                nodes, l1, l2, k, ..
            } => {
                let [p1, q1, p2, q2] = nodes.clone();
                let base = name.strip_prefix(['K', 'k']).unwrap_or(&name);
                let primary = self.extra_name(id, format!("L{base}_p"))?;
                let secondary = self.extra_name(id, format!("L{base}_s"))?;
                self.two_terminal(&primary, &[p1, q1], num(id, "l1", *l1)?)?;
                self.two_terminal(&secondary, &[p2, q2], num(id, "l2", *l2)?)?;
                let k = num(id, "k", *k)?;
                self.line(format!("{name} {primary} {secondary} {k}"));
                Ok(())
            }
            CircuitElement::Mosfet { .. } => Err(unsupported(
                "MOSFET cards aren't written yet (sindr's parser has no M element)",
            )),
            CircuitElement::Jfet { .. } => Err(unsupported(
                "JFET cards aren't written yet (sindr's parser has no J element)",
            )),
            CircuitElement::Igbt { .. } => Err(unsupported("SPICE has no IGBT primitive")),
            CircuitElement::ZenerDiode { .. } => Err(unsupported(
                "sindr's breakdown clamp isn't SPICE's BV/IBV breakdown",
            )),
            CircuitElement::OpAmp { .. } | CircuitElement::Comparator { .. } => Err(unsupported(
                "the saturating macromodel has no SPICE primitive",
            )),
            CircuitElement::Relay { .. } => Err(unsupported(
                "a voltage-operated contact has no SPICE3 primitive",
            )),
            CircuitElement::Photodiode { .. } => Err(unsupported(
                "the light-driven current has no SPICE primitive",
            )),
            CircuitElement::Varactor { .. } => Err(unsupported(
                "a SPICE diode with junction capacitance also conducts",
            )),
            CircuitElement::VoltageRegulator { .. } => {
                Err(unsupported("a regulator macromodel has no SPICE primitive"))
            }
        }
    }

    fn diode(
        &mut self,
        id: &str,
        name: &str,
        nodes: &[String; 2],
        model: DiodeModel,
    ) -> Result<(), ExportError> {
        let model_name = match self.diode_models.iter().find(|(m, _)| *m == model) {
            Some((_, n)) => n.clone(),
            None => {
                let n = format!("dmod{}", self.diode_models.len() + 1);
                self.models.push(format!(
                    ".model {n} D(IS={} N={} RS={})",
                    num(id, "is", model.is)?,
                    num(id, "n", model.n)?,
                    num(id, "rs", model.rs)?
                ));
                self.diode_models.push((model, n.clone()));
                n
            }
        };
        self.two_terminal(name, nodes, model_name)
    }

    fn bjt_model(
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
            ".model {n} {card}(IS={} BF={} BR={} NF={} NR={} VAF={})",
            num(id, "is", model.is)?,
            num(id, "bf", bf)?,
            num(id, "br", model.br)?,
            num(id, "nf", model.nf)?,
            num(id, "nr", model.nr)?,
            num(id, "vaf", model.vaf)?
        ));
        self.bjt_models.push((kind, bf, model, n.clone()));
        Ok(n)
    }

    fn controlled(
        &mut self,
        name: &str,
        nodes: &[String; 2],
        control_nodes: &[String; 2],
        value: String,
    ) -> Result<(), ExportError> {
        let out = self.nodes(nodes)?;
        let control = self.nodes(control_nodes)?;
        self.line(format!("{name} {out} {control} {value}"));
        Ok(())
    }

    fn current_controlled(
        &mut self,
        name: &str,
        id: &str,
        nodes: &[String; 2],
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

fn swapped([a, b]: &[String; 2]) -> [String; 2] {
    [b.clone(), a.clone()]
}

/// The SPICE element letter an element is written as.
fn spice_letter(component: &CircuitElement) -> char {
    match component {
        CircuitElement::Capacitor { .. } => 'C',
        CircuitElement::Inductor { .. } => 'L',
        CircuitElement::VoltageSource { .. } => 'V',
        CircuitElement::CurrentSource { .. } => 'I',
        CircuitElement::Diode { .. }
        | CircuitElement::Led { .. }
        | CircuitElement::SchottkyDiode { .. } => 'D',
        CircuitElement::Bjt { .. } => 'Q',
        CircuitElement::Vcvs { .. } => 'E',
        CircuitElement::Vccs { .. } => 'G',
        CircuitElement::Ccvs { .. } => 'H',
        CircuitElement::Cccs { .. } => 'F',
        CircuitElement::Transformer { .. } => 'K',
        CircuitElement::Mosfet { .. } => 'M',
        CircuitElement::Jfet { .. } => 'J',
        _ => 'R',
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

fn nominal(id: &str, temperature: f64) -> Result<(), ExportError> {
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

fn shockley(params: DiodeParams) -> DiodeModel {
    DiodeModel {
        is: params.is,
        n: params.n,
        rs: params.rs,
    }
}

/// `value` written so it reads back as the same `f64`: plain decimals in a
/// readable range, exponent notation outside it.
fn num(id: &str, field: &'static str, value: f64) -> Result<String, ExportError> {
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
fn waveform_card(id: &str, waveform: &Waveform, dc: f64) -> Result<String, ExportError> {
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
        } => {
            let head = format!(
                "SIN({} {} {}",
                n("offset", offset + dc)?,
                n("amplitude", *amplitude)?,
                n("frequency", *frequency)?
            );
            if *phase == 0.0 {
                return Ok(format!("{head})"));
            }
            if *frequency == 0.0 {
                return Err(ExportError::Unsupported {
                    id: id.to_string(),
                    reason: "a phased sine of zero frequency has no SIN delay".to_string(),
                });
            }
            // sindr's parser reads the delay back as phase = −2π·f·td.
            let td = -phase / (2.0 * PI * frequency);
            Ok(format!("{head} {})", n("phase", td)?))
        }
        Waveform::Pulse {
            v1,
            v2,
            delay,
            rise_time,
            fall_time,
            pulse_width,
            period,
        } => {
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
