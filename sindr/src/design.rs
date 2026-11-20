//! Design hierarchy and part data: [`Block`], [`Instance`], [`Part`], and
//! [`Circuit::flatten`].
//!
//! A [`Block`] is a reusable definition, what `.subckt` is in SPICE. An
//! [`Instance`] places a block and binds its ports to the caller's nets,
//! what an `X` line is. Inside a block a net name is a port, ground
//! ([`Circuit::ground_node`]), a global ([`Circuit::globals`]), or local to
//! each instance.
//!
//! Flattening names things the way the SPICE parser always has: elements
//! and local nets get the instance path joined with `.` (`x1.x2.r1`,
//! `x1.mid`), ports become the caller's net, and ground and globals are
//! never prefixed.
//!
//! ```
//! use sindr::{Block, Circuit, CircuitElement, Instance};
//!
//! let divider = Block {
//!     ports: vec!["top".into(), "out".into()],
//!     components: vec![
//!         CircuitElement::Resistor { id: "R1".into(), nodes: ["top".into(), "out".into()], resistance: 1e3 },
//!         CircuitElement::Resistor { id: "R2".into(), nodes: ["out".into(), "0".into()], resistance: 1e3 },
//!     ],
//!     ..Block::default()
//! };
//! let mut circuit = Circuit::new(
//!     "0",
//!     vec![CircuitElement::VoltageSource { id: "V1".into(), nodes: ["in".into(), "0".into()], voltage: 10.0, waveform: None }],
//! );
//! circuit.blocks.insert("divider".into(), divider);
//! circuit.instances.push(Instance::new("X1", "divider", ["in", "mid"]));
//!
//! let flat = circuit.flatten().unwrap();
//! assert!(flat.is_flat());
//! assert_eq!(flat.components[1].id(), "X1.R1");
//! assert_eq!(flat.components[2].nodes(), ["mid", "0"]);
//! ```

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap, HashSet};

use crate::circuit::{Circuit, CircuitElement};

/// Separator between instance-path segments in flattened names.
pub const PATH_SEPARATOR: char = '.';

/// A reusable block definition: what `.subckt` is in SPICE.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Block {
    /// Port nets, in the order [`Instance::nodes`] binds them.
    pub ports: Vec<String>,
    /// Components inside the block, named in the block's own scope.
    #[cfg_attr(feature = "serde", serde(default))]
    pub components: Vec<CircuitElement>,
    /// Instances of other blocks placed inside this one.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Vec::is_empty")
    )]
    pub instances: Vec<Instance>,
    /// Part data for the block's components and instances, keyed by id.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "BTreeMap::is_empty")
    )]
    pub parts: BTreeMap<String, Part>,
}

/// One placement of a [`Block`]: what an `X` line is in SPICE.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
pub struct Instance {
    /// Instance id, unique in its scope (`"X1"`). Prefixes everything inside
    /// it when flattened.
    pub id: String,
    /// Name of the block it places, a key of [`Circuit::blocks`].
    pub block: String,
    /// The caller's nets, bound to [`Block::ports`] by position.
    pub nodes: Vec<String>,
}

impl Instance {
    /// An instance of `block` with `nodes` bound to its ports in order.
    pub fn new<S: Into<String>>(
        id: impl Into<String>,
        block: impl Into<String>,
        nodes: impl IntoIterator<Item = S>,
    ) -> Self {
        Self {
            id: id.into(),
            block: block.into(),
            nodes: nodes.into_iter().map(Into::into).collect(),
        }
    }
}

/// Part data for a component or instance: what a schematic or BOM needs.
/// The simulator ignores it.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Part {
    /// Reference designator (`"R1"`). `None` means the element id is the
    /// reference.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub reference: Option<String>,
    /// Display value (`"10k"`, `"BC547"`).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub value: Option<String>,
    /// Schematic symbol, in whatever form the design tool uses
    /// (`"Device:R"`).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub symbol: Option<String>,
    /// PCB footprint (`"Resistor_SMD:R_0603_1608Metric"`).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub footprint: Option<String>,
    /// Any other fields: manufacturer, MPN, tolerance, …
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "BTreeMap::is_empty"))]
    pub fields: BTreeMap<String, String>,
    /// Absolute maximum ratings from the datasheet. `sindr::check_ratings`
    /// compares a simulation against them.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Ratings::is_empty"))]
    pub ratings: Ratings,
    /// How the part sheds heat. When set, the solver heats its junction by
    /// the power it dissipates; see [`Thermal`].
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub thermal: Option<Thermal>,
}

/// A part's thermal path to ambient, for self-heating.
///
/// With it, the part's junction runs at `ambient + P·Rθ` in DC, or follows
/// `Cθ·dT/dt = P − (T − ambient)/Rθ` in a transient, and the solver feeds
/// that temperature back into the device (IS and kT/q for diodes, LEDs and
/// BJTs; the thermistor's resistance). The element's own `temperature`
/// field is then ignored.
///
/// Rθ is [`rth_jc`](Self::rth_jc) + [`rth_ca`](Self::rth_ca) when both are
/// set (a part on a heatsink), otherwise [`rth_ja`](Self::rth_ja).
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thermal {
    /// Junction to ambient (K/W), the datasheet's Rθja for the part on its
    /// own.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub rth_ja: Option<f64>,
    /// Junction to case (K/W).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub rth_jc: Option<f64>,
    /// Case to ambient through what the part is mounted on: interface pad
    /// plus heatsink (K/W).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub rth_ca: Option<f64>,
    /// Thermal capacitance (J/K). `None` in a transient means the junction
    /// follows its power with no lag.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub cth: Option<f64>,
    /// Ambient temperature (K). Default 300.15 K (27 °C).
    pub ambient: f64,
}

impl Default for Thermal {
    fn default() -> Self {
        Self {
            rth_ja: None,
            rth_jc: None,
            rth_ca: None,
            cth: None,
            ambient: 300.15,
        }
    }
}

impl Thermal {
    /// A free-standing part: junction to ambient `rth_ja` (K/W).
    pub fn junction_to_ambient(rth_ja: f64) -> Self {
        Self {
            rth_ja: Some(rth_ja),
            ..Self::default()
        }
    }

    /// A part on a heatsink: junction to case `rth_jc` plus case to ambient
    /// `rth_ca` (K/W).
    pub fn on_heatsink(rth_jc: f64, rth_ca: f64) -> Self {
        Self {
            rth_jc: Some(rth_jc),
            rth_ca: Some(rth_ca),
            ..Self::default()
        }
    }

    /// The thermal resistance from junction to ambient (K/W) the solver
    /// uses, or `None` when neither path is set.
    pub fn resistance(&self) -> Option<f64> {
        match (self.rth_jc, self.rth_ca) {
            (Some(jc), Some(ca)) => Some(jc + ca),
            _ => self.rth_ja,
        }
    }
}

/// Absolute maximum ratings: the datasheet limits a part must never pass.
///
/// Every limit is a magnitude in SI units and `None` when unknown. A rating
/// a part needs but doesn't have is reported as unverified, never as a
/// pass. Which ratings a component needs depends on its kind; see
/// `sindr::ratings`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Ratings {
    /// Power dissipation (W).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub power: Option<f64>,
    /// Voltage across the part (V): a capacitor's rated voltage, a
    /// resistor's working voltage.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub voltage: Option<f64>,
    /// Current through the part (A): an inductor's saturation current.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub current: Option<f64>,
    /// Reverse voltage (V): a diode's cathode above its anode, or a
    /// polarised capacitor's `nodes[1]` above its `nodes[0]` (the `+`
    /// terminal). Most electrolytics allow 1 V or less; set it on a
    /// polarised capacitor's part to have reverse bias flagged.
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub reverse_voltage: Option<f64>,
    /// Diode forward current, anode to cathode (A).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub forward_current: Option<f64>,
    /// BJT or IGBT collector–emitter voltage, `|Vce|` (V).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub vce: Option<f64>,
    /// BJT collector–base voltage, `|Vcb|` (V).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub vcb: Option<f64>,
    /// BJT emitter–base voltage, `|Veb|` (V).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub veb: Option<f64>,
    /// BJT or IGBT collector current, `|Ic|` (A).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub ic: Option<f64>,
    /// MOSFET or JFET drain–source voltage, `|Vds|` (V).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub vds: Option<f64>,
    /// MOSFET or JFET gate–source voltage, `|Vgs|`; an IGBT's gate–emitter
    /// voltage (V).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub vgs: Option<f64>,
    /// MOSFET or JFET drain current, `|Id|` (A).
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub id: Option<f64>,
    /// Maximum junction temperature (K): 423.15 K for a 150 °C part. Checked
    /// against the junction temperature self-heating reports, so it needs
    /// [`Part::thermal`].
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub tj_max: Option<f64>,
}

impl Ratings {
    /// `true` when no limit is set.
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

/// Why a design can't be flattened.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum DesignError {
    /// An instance names a block that isn't in [`Circuit::blocks`].
    #[error("instance `{instance}` places block `{block}`, which is not defined")]
    UnknownBlock {
        /// Full path of the instance (`x1.x2`).
        instance: String,
        /// The missing block.
        block: String,
    },
    /// An instance binds a different number of nets than the block has ports.
    #[error("instance `{instance}` connects {got} nets, but block `{block}` has {expected} ports")]
    PortCount {
        /// Full path of the instance.
        instance: String,
        /// The block it places.
        block: String,
        /// Ports the block declares.
        expected: usize,
        /// Nets the instance binds.
        got: usize,
    },
    /// A block places itself, directly or through other blocks.
    #[error("instance `{instance}` places block `{block}` inside itself")]
    RecursiveBlock {
        /// Full path of the instance that closes the loop.
        instance: String,
        /// The block that recurses.
        block: String,
    },
    /// Two components or instances in one scope share an id.
    #[error("id `{id}` is used twice in {scope}")]
    DuplicateId {
        /// The repeated id.
        id: String,
        /// `"the top level"` or `"block `name`"`.
        scope: String,
    },
}

impl Circuit {
    /// Expand every instance into a flat circuit with no instances or
    /// blocks. Names follow the module docs; `parts` keys get the same
    /// prefix as their element, and `globals` carry over.
    ///
    /// Top-level components come first, then each instance's expansion in
    /// order, depth-first.
    ///
    /// # Errors
    ///
    /// [`DesignError`] for an unknown block, a port-count mismatch, a
    /// recursive block, or a repeated id in one scope.
    pub fn flatten(&self) -> Result<Circuit, DesignError> {
        Ok(self.flattened()?.into_owned())
    }

    /// [`flatten`](Self::flatten), borrowing `self` when it is already flat.
    ///
    /// # Errors
    ///
    /// As [`flatten`](Self::flatten).
    pub fn flattened(&self) -> Result<Cow<'_, Circuit>, DesignError> {
        if self.is_flat() {
            return Ok(Cow::Borrowed(self));
        }
        check_unique(&self.components, &self.instances, || {
            "the top level".to_string()
        })?;
        let mut out = Circuit {
            components: self.components.clone(),
            ground_node: self.ground_node.clone(),
            parts: self.parts.clone(),
            globals: self.globals.clone(),
            ..Circuit::default()
        };
        let mut expander = Expander {
            design: self,
            stack: Vec::new(),
            out: &mut out,
        };
        for instance in &self.instances {
            expander.expand(instance, "", &HashMap::new())?;
        }
        Ok(Cow::Owned(out))
    }
}

struct Expander<'a> {
    design: &'a Circuit,
    /// Blocks being expanded, outermost first: a repeat is recursion.
    stack: Vec<&'a str>,
    out: &'a mut Circuit,
}

impl<'a> Expander<'a> {
    /// Expand `instance`, which sits in the scope named `parent` (empty at
    /// the top) whose ports bind as `parent_ports`.
    fn expand(
        &mut self,
        instance: &'a Instance,
        parent: &str,
        parent_ports: &HashMap<&str, String>,
    ) -> Result<(), DesignError> {
        let path = join(parent, &instance.id);
        let block =
            self.design
                .blocks
                .get(&instance.block)
                .ok_or_else(|| DesignError::UnknownBlock {
                    instance: path.clone(),
                    block: instance.block.clone(),
                })?;
        if block.ports.len() != instance.nodes.len() {
            return Err(DesignError::PortCount {
                instance: path,
                block: instance.block.clone(),
                expected: block.ports.len(),
                got: instance.nodes.len(),
            });
        }
        if self.stack.contains(&instance.block.as_str()) {
            return Err(DesignError::RecursiveBlock {
                instance: path,
                block: instance.block.clone(),
            });
        }
        check_unique(&block.components, &block.instances, || {
            format!("block `{}`", instance.block)
        })?;

        let ports: HashMap<&str, String> = block
            .ports
            .iter()
            .zip(&instance.nodes)
            .map(|(port, net)| (port.as_str(), self.net(net, parent, parent_ports)))
            .collect();

        for component in &block.components {
            let mut el = component.clone();
            let id = el.id_mut();
            *id = join(&path, id);
            for net in el.nodes_mut() {
                *net = self.net(net, &path, &ports);
            }
            if let Some(control) = el.control_nodes_mut() {
                for net in control.iter_mut() {
                    *net = self.net(net, &path, &ports);
                }
            }
            if let Some(source) = el.control_source_mut() {
                *source = join(&path, source);
            }
            self.out.components.push(el);
        }
        for (id, part) in &block.parts {
            self.out.parts.insert(join(&path, id), part.clone());
        }

        self.stack.push(&instance.block);
        for child in &block.instances {
            self.expand(child, &path, &ports)?;
        }
        self.stack.pop();
        Ok(())
    }

    /// The flat name of `net` as seen inside the scope at `path`.
    fn net(&self, net: &str, path: &str, ports: &HashMap<&str, String>) -> String {
        if net == self.design.ground_node || self.design.globals.iter().any(|g| g == net) {
            return net.to_string();
        }
        match ports.get(net) {
            Some(bound) => bound.clone(),
            None => join(path, net),
        }
    }
}

fn join(path: &str, name: &str) -> String {
    if path.is_empty() {
        name.to_string()
    } else {
        format!("{path}{PATH_SEPARATOR}{name}")
    }
}

fn check_unique(
    components: &[CircuitElement],
    instances: &[Instance],
    scope: impl Fn() -> String,
) -> Result<(), DesignError> {
    let mut seen = HashSet::new();
    let ids = components
        .iter()
        .map(CircuitElement::id)
        .chain(instances.iter().map(|i| i.id.as_str()));
    for id in ids {
        if !seen.insert(id) {
            return Err(DesignError::DuplicateId {
                id: id.to_string(),
                scope: scope(),
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(id: &str, a: &str, b: &str) -> CircuitElement {
        CircuitElement::Resistor {
            id: id.into(),
            nodes: [a.into(), b.into()],
            resistance: 1e3,
        }
    }

    /// `stage(p, q)` holds `inner(a, b)` plus R3; `inner` is a two-resistor
    /// chain through a local `mid`. Same shape as the SPICE source-map test.
    fn nested() -> Circuit {
        let mut c = Circuit::new("0", vec![r("R9", "out", "0")]);
        c.blocks.insert(
            "inner".into(),
            Block {
                ports: vec!["a".into(), "b".into()],
                components: vec![r("R1", "a", "mid"), r("R2", "mid", "b")],
                ..Block::default()
            },
        );
        c.blocks.insert(
            "stage".into(),
            Block {
                ports: vec!["p".into(), "q".into()],
                components: vec![r("R3", "n", "q")],
                instances: vec![Instance::new("X2", "inner", ["p", "n"])],
                ..Block::default()
            },
        );
        c.instances
            .push(Instance::new("X1", "stage", ["in", "out"]));
        c
    }

    fn summary(c: &Circuit) -> Vec<(String, Vec<String>)> {
        c.components
            .iter()
            .map(|e| (e.id().to_string(), e.nodes().to_vec()))
            .collect()
    }

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|x| x.to_string()).collect()
    }

    #[test]
    fn nested_instances_flatten_with_path_prefixes() {
        let flat = nested().flatten().unwrap();
        assert!(flat.is_flat() && flat.blocks.is_empty());
        assert_eq!(
            summary(&flat),
            vec![
                ("R9".into(), s(&["out", "0"])),
                ("X1.R3".into(), s(&["X1.n", "out"])),
                ("X1.X2.R1".into(), s(&["in", "X1.X2.mid"])),
                ("X1.X2.R2".into(), s(&["X1.X2.mid", "X1.n"])),
            ]
        );
    }

    #[test]
    fn a_flat_circuit_is_borrowed() {
        let c = Circuit::new("0", vec![r("R1", "a", "0")]);
        assert!(matches!(c.flattened().unwrap(), Cow::Borrowed(_)));
        assert_eq!(c.flatten().unwrap(), c);
    }

    #[test]
    fn ground_and_globals_are_never_prefixed() {
        let mut c = Circuit::new("gnd", vec![]);
        c.globals.push("vcc".into());
        c.blocks.insert(
            "b".into(),
            Block {
                components: vec![r("R1", "vcc", "gnd")],
                ..Block::default()
            },
        );
        c.instances
            .push(Instance::new("X1", "b", Vec::<String>::new()));
        let flat = c.flatten().unwrap();
        assert_eq!(flat.components[0].nodes(), ["vcc", "gnd"]);
        assert_eq!(flat.globals, ["vcc"]);
    }

    #[test]
    fn controlled_sources_follow_the_prefix() {
        let mut c = Circuit::new("0", vec![]);
        c.blocks.insert(
            "b".into(),
            Block {
                ports: vec!["in".into()],
                components: vec![
                    CircuitElement::Vcvs {
                        id: "E1".into(),
                        nodes: ["o".into(), "0".into()],
                        control_nodes: ["in".into(), "x".into()],
                        gain: 2.0,
                    },
                    CircuitElement::Cccs {
                        id: "F1".into(),
                        nodes: ["o".into(), "0".into()],
                        control_source: "E1".into(),
                        alpha: 1.0,
                    },
                ],
                ..Block::default()
            },
        );
        c.instances.push(Instance::new("X1", "b", ["top"]));
        let flat = c.flatten().unwrap();
        assert_eq!(flat.components[0].all_nodes(), ["X1.o", "0", "top", "X1.x"]);
        match &flat.components[1] {
            CircuitElement::Cccs { control_source, .. } => assert_eq!(control_source, "X1.E1"),
            other => panic!("unexpected {other:?}"),
        }
    }

    #[test]
    fn parts_get_the_element_prefix() {
        let mut c = nested();
        let part = Part {
            footprint: Some("R_0603".into()),
            ..Part::default()
        };
        c.blocks
            .get_mut("inner")
            .unwrap()
            .parts
            .insert("R1".into(), part.clone());
        c.parts.insert("R9".into(), part.clone());
        let flat = c.flatten().unwrap();
        assert_eq!(flat.parts.get("X1.X2.R1"), Some(&part));
        assert_eq!(flat.parts.get("R9"), Some(&part));
    }

    #[test]
    fn unknown_block_names_the_instance_path() {
        let mut c = nested();
        c.blocks.remove("inner");
        assert_eq!(
            c.flatten(),
            Err(DesignError::UnknownBlock {
                instance: "X1.X2".into(),
                block: "inner".into()
            })
        );
    }

    #[test]
    fn port_count_mismatch_is_an_error() {
        let mut c = nested();
        c.instances[0].nodes.pop();
        assert!(matches!(
            c.flatten(),
            Err(DesignError::PortCount {
                expected: 2,
                got: 1,
                ..
            })
        ));
    }

    #[test]
    fn recursion_is_an_error() {
        let mut c = nested();
        c.blocks
            .get_mut("inner")
            .unwrap()
            .instances
            .push(Instance::new("X9", "stage", ["a", "b"]));
        assert!(matches!(
            c.flatten(),
            Err(DesignError::RecursiveBlock { block, .. }) if block == "stage"
        ));
    }

    #[test]
    fn a_block_used_twice_is_not_recursion() {
        let mut c = nested();
        c.instances
            .push(Instance::new("X3", "stage", ["in", "out2"]));
        let flat = c.flatten().unwrap();
        assert_eq!(flat.components.len(), 7);
    }

    #[test]
    fn duplicate_ids_in_one_scope_are_an_error() {
        let mut c = nested();
        c.instances.push(Instance::new("R9", "inner", ["a", "b"]));
        assert!(matches!(
            c.flatten(),
            Err(DesignError::DuplicateId { id, .. }) if id == "R9"
        ));
    }

    #[test]
    fn nets_lists_the_top_level() {
        assert_eq!(nested().nets(), ["0", "in", "out"]);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trips_and_flat_json_is_unchanged() {
        let mut c = nested();
        c.parts.insert(
            "R9".into(),
            Part {
                reference: Some("R1".into()),
                ..Part::default()
            },
        );
        let json = serde_json::to_string(&c).unwrap();
        let back: Circuit = serde_json::from_str(&json).unwrap();
        assert_eq!(back, c);

        let flat = Circuit::new("0", vec![r("R1", "a", "0")]);
        let json = serde_json::to_value(&flat).unwrap();
        let keys: Vec<_> = json.as_object().unwrap().keys().collect();
        assert_eq!(keys, ["components", "ground_node"]);
        let old = r#"{"ground_node":"0","components":[{"type":"resistor","id":"R1","nodes":["a","0"],"resistance":1000.0}]}"#;
        assert_eq!(serde_json::from_str::<Circuit>(old).unwrap(), flat);
    }
}
