//! Typed handles for nets and components, and the builder that hands them
//! out.
//!
//! A [`NodeId`] names a net and a [`ComponentId`] names a component. Element
//! terminals and analysis configs take them, so a net can't be passed where
//! a component is meant, or the other way round. Each is its name underneath:
//! it serialises as a plain string, so circuit and config JSON keep the shape
//! they had with names.
//!
//! [`CircuitBuilder`] is the checked way to make them. Declare each net once
//! with [`node`](CircuitBuilder::node), wire elements to the handles it gives
//! back, and [`build`](CircuitBuilder::build) refuses an element that names a
//! net nobody declared, so a typo is an error rather than a new, floating
//! net:
//!
//! ```
//! use sindr::element::{Resistor, VoltageSource};
//! use sindr::{solve_circuit, CircuitBuilder};
//!
//! let mut b = CircuitBuilder::new();
//! let gnd = b.ground();
//! let vin = b.node("in");
//! let out = b.node("out");
//! b.add(VoltageSource::dc("V1", [vin.clone(), gnd.clone()], 10.0));
//! b.add(Resistor { id: "R1".into(), nodes: [vin, out.clone()], resistance: 1e3 });
//! b.add(Resistor { id: "R2".into(), nodes: [out.clone(), gnd], resistance: 1e3 });
//! let circuit = b.build().unwrap();
//!
//! let result = solve_circuit(&circuit).unwrap();
//! assert!((result.voltage(&out).unwrap() - 5.0).abs() < 1e-9);
//! ```
//!
//! The string API stays alongside, for data: a handle converts from a name
//! (`"out".into()`), and [`Circuit::node_id`](crate::Circuit::node_id) and
//! [`Circuit::component_id`](crate::Circuit::component_id) look one up and
//! fail with [`SimError::UnknownNode`] or [`SimError::UnknownComponent`] when
//! the circuit has no such name. SPICE and schematic import build from names.

use std::borrow::Borrow;
use std::collections::BTreeSet;
use std::fmt;
use std::ops::Deref;

use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;

macro_rules! name_handle {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[cfg_attr(feature = "serde", serde(transparent))]
        #[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
        #[cfg_attr(feature = "schemars", schemars(transparent))]
        #[derive(Debug, Clone, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(String);

        impl $name {
            /// The name it stands for.
            pub fn as_str(&self) -> &str {
                &self.0
            }

            /// The name, owned.
            pub fn into_string(self) -> String {
                self.0
            }

            /// For renaming in place (flattening, import).
            // Not every handle kind is renamed in place.
            #[allow(dead_code)]
            pub(crate) fn name_mut(&mut self) -> &mut String {
                &mut self.0
            }
        }

        impl Deref for $name {
            type Target = str;
            fn deref(&self) -> &str {
                &self.0
            }
        }

        impl AsRef<str> for $name {
            fn as_ref(&self) -> &str {
                &self.0
            }
        }

        impl Borrow<str> for $name {
            fn borrow(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.0)
            }
        }

        impl From<&str> for $name {
            fn from(name: &str) -> Self {
                Self(name.to_owned())
            }
        }

        impl From<&String> for $name {
            fn from(name: &String) -> Self {
                Self(name.clone())
            }
        }

        impl From<String> for $name {
            fn from(name: String) -> Self {
                Self(name)
            }
        }

        impl From<&$name> for $name {
            fn from(handle: &$name) -> Self {
                handle.clone()
            }
        }

        impl From<$name> for String {
            fn from(handle: $name) -> Self {
                handle.0
            }
        }

        impl PartialEq<str> for $name {
            fn eq(&self, other: &str) -> bool {
                self.0 == other
            }
        }

        impl PartialEq<&str> for $name {
            fn eq(&self, other: &&str) -> bool {
                self.0 == *other
            }
        }

        impl PartialEq<String> for $name {
            fn eq(&self, other: &String) -> bool {
                &self.0 == other
            }
        }

        impl PartialEq<$name> for str {
            fn eq(&self, other: &$name) -> bool {
                self == other.0
            }
        }

        impl PartialEq<$name> for &str {
            fn eq(&self, other: &$name) -> bool {
                *self == other.0
            }
        }

        impl PartialEq<$name> for String {
            fn eq(&self, other: &$name) -> bool {
                *self == other.0
            }
        }
    };
}

name_handle!(
    /// A net: an element terminal, a config's output node, a result's key.
    ///
    /// Get one from [`CircuitBuilder::node`], which checks it at build time,
    /// or [`Circuit::node_id`](crate::Circuit::node_id), which checks it
    /// against a circuit; `"out".into()` makes one from a name unchecked.
    NodeId
);

name_handle!(
    /// A component, by its id: the source an analysis drives, the part a
    /// sensitivity or noise result names.
    ///
    /// Get one from [`CircuitBuilder::add`] or
    /// [`Circuit::component_id`](crate::Circuit::component_id), which checks
    /// it against a circuit; `"V1".into()` makes one from a name unchecked.
    ComponentId
);

/// Builds a flat [`Circuit`] from typed handles, checking at
/// [`build`](Self::build) that every net an element names was declared and
/// that no two components share an id. See the [module docs](self).
#[derive(Debug, Clone)]
pub struct CircuitBuilder {
    ground: NodeId,
    nodes: BTreeSet<NodeId>,
    components: Vec<CircuitElement>,
}

impl Default for CircuitBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl CircuitBuilder {
    /// A builder whose ground net is `"0"`, as in SPICE.
    pub fn new() -> Self {
        Self::with_ground("0")
    }

    /// A builder whose ground net has another name, such as `"gnd"`.
    pub fn with_ground(name: impl Into<String>) -> Self {
        let ground = NodeId(name.into());
        Self {
            nodes: BTreeSet::from([ground.clone()]),
            ground,
            components: Vec::new(),
        }
    }

    /// The ground net.
    pub fn ground(&self) -> NodeId {
        self.ground.clone()
    }

    /// Declares the net `name` and returns its handle. Declaring a name
    /// twice gives the same net.
    pub fn node(&mut self, name: impl Into<String>) -> NodeId {
        let id = NodeId(name.into());
        self.nodes.insert(id.clone());
        id
    }

    /// Adds an element and returns its id's handle.
    pub fn add(&mut self, element: impl Into<CircuitElement>) -> ComponentId {
        let element = element.into();
        let id = ComponentId::from(element.id());
        self.components.push(element);
        id
    }

    /// The circuit.
    ///
    /// # Errors
    ///
    /// [`SimError::UnknownNode`] for an element that names a net no call to
    /// [`node`](Self::node) declared, and [`SimError::DuplicateId`] for
    /// two components with one id.
    pub fn build(self) -> Result<Circuit, SimError> {
        let mut ids = BTreeSet::new();
        for element in &self.components {
            if !ids.insert(element.id()) {
                return Err(SimError::DuplicateId {
                    id: element.id().to_string(),
                    lowered: false,
                });
            }
            if let Some(node) = element
                .all_nodes()
                .into_iter()
                .find(|n| !self.nodes.contains(n.as_str()))
            {
                return Err(SimError::UnknownNode {
                    node: node.to_string(),
                    component: Some(element.id().to_owned()),
                });
            }
        }
        Ok(Circuit::new(self.ground.into_string(), self.components))
    }
}

impl Circuit {
    /// The handle of a net this circuit names at the top level (see
    /// [`nets`](Self::nets)).
    ///
    /// # Errors
    ///
    /// [`SimError::UnknownNode`] when no component, instance or ground
    /// names it.
    pub fn node_id(&self, name: &str) -> Result<NodeId, SimError> {
        if self.nets().contains(&name) {
            Ok(NodeId::from(name))
        } else {
            Err(SimError::UnknownNode {
                node: name.to_owned(),
                component: None,
            })
        }
    }

    /// The handle of a top-level component or instance.
    ///
    /// # Errors
    ///
    /// [`SimError::UnknownComponent`] when nothing at the top level has the
    /// id.
    pub fn component_id(&self, id: &str) -> Result<ComponentId, SimError> {
        let known = self.components.iter().any(|c| c.id() == id)
            || self.instances.iter().any(|i| i.id == id);
        if known {
            Ok(ComponentId::from(id))
        } else {
            Err(SimError::UnknownComponent(id.to_owned()))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::{Resistor, VoltageSource};

    fn divider(b: &mut CircuitBuilder, out: &NodeId) {
        let gnd = b.ground();
        let vin = b.node("in");
        b.add(VoltageSource::dc("V1", [vin.clone(), gnd.clone()], 10.0));
        b.add(Resistor {
            id: "R1".into(),
            nodes: [vin, out.clone()],
            resistance: 1e3,
        });
    }

    #[test]
    fn build_refuses_an_undeclared_net() {
        let mut b = CircuitBuilder::new();
        let out = b.node("out");
        divider(&mut b, &out);
        b.add(Resistor {
            id: "R2".into(),
            nodes: ["ouy".into(), b.ground()],
            resistance: 1e3,
        });
        match b.build() {
            Err(SimError::UnknownNode { node, component }) => {
                assert_eq!(node, "ouy");
                assert_eq!(component.as_deref(), Some("R2"));
            }
            other => panic!("expected UnknownNode, got {other:?}"),
        }
    }

    #[test]
    fn build_refuses_a_repeated_id() {
        let mut b = CircuitBuilder::new();
        let out = b.node("out");
        divider(&mut b, &out);
        b.add(Resistor {
            id: "R1".into(),
            nodes: [out, b.ground()],
            resistance: 1e3,
        });
        assert!(matches!(b.build(), Err(SimError::DuplicateId { .. })));
    }

    #[test]
    fn built_circuit_solves_and_looks_up_handles() {
        let mut b = CircuitBuilder::with_ground("gnd");
        let out = b.node("out");
        divider(&mut b, &out);
        let r2 = b.add(Resistor {
            id: "R2".into(),
            nodes: [out.clone(), b.ground()],
            resistance: 1e3,
        });
        let circuit = b.build().unwrap();
        assert_eq!(circuit.ground_node, "gnd");
        assert_eq!(circuit.node_id("out").unwrap(), out);
        assert_eq!(circuit.component_id("R2").unwrap(), r2);
        assert!(matches!(
            circuit.node_id("ouy"),
            Err(SimError::UnknownNode { .. })
        ));
        assert!(matches!(
            circuit.component_id("R9"),
            Err(SimError::UnknownComponent(_))
        ));
        let result = crate::solve_circuit(&circuit).unwrap();
        assert!((result.voltage(&out).unwrap() - 5.0).abs() < 1e-9);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn handles_serialise_as_their_names() {
        let id = NodeId::from("out");
        assert_eq!(serde_json::to_string(&id).unwrap(), "\"out\"");
        let back: ComponentId = serde_json::from_str("\"V1\"").unwrap();
        assert_eq!(back, "V1");
    }
}
