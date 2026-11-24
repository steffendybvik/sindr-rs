//! Elements solved as a network of simpler ones: the SPDT switch (two
//! switches), the crystal (C0 beside Rm–Lm–Cm), the ferrite bead (Rdc
//! into L ∥ Rac ∥ C) and the digital pin (its output stage, clamps, die
//! capacitance and leakage).
//!
//! [`lower`] turns each into its parts before any analysis, with internal
//! ids and nodes `<id>#…`. [`fold`] puts a result back together: one
//! [`ComponentResult`] under the element's own id, from its parts', and the
//! parts and internal nodes gone.

use crate::element::{
    Capacitor, Crystal, CurrentSource, DigitalPin, Diode, FerriteBead, Inductor, Resistor, Switch,
    SwitchSpdt,
};
use std::collections::HashMap;

use crate::circuit::{Circuit, CircuitElement, CrystalParams, FerriteBeadParams, SpdtPosition};
use crate::results::{ComponentResult, SimulationResult, TimestepSnapshot};
use crate::SimError;

/// Whether `element` is solved as a network of others.
pub(crate) fn is_composite(element: &CircuitElement) -> bool {
    matches!(
        element,
        CircuitElement::SwitchSpdt(SwitchSpdt { .. })
            | CircuitElement::Crystal(Crystal { .. })
            | CircuitElement::FerriteBead(FerriteBead { .. })
            | CircuitElement::DigitalPin(DigitalPin { .. })
    )
}

fn positive(id: &str, what: &str, v: f64) -> Result<f64, SimError> {
    if v.is_finite() && v > 0.0 {
        Ok(v)
    } else {
        Err(SimError::InvalidComponent(format!(
            "`{id}`: {what} {v} must be a positive, finite number"
        )))
    }
}

fn part(id: &str, suffix: &str) -> String {
    format!("{id}#{suffix}")
}

/// The parts `element` is solved as, or `None` when it's solved as itself.
///
/// # Errors
///
/// [`SimError::InvalidComponent`] for a parameter that isn't positive and
/// finite.
pub(crate) fn lower(element: &CircuitElement) -> Result<Option<Vec<CircuitElement>>, SimError> {
    let parts = match element {
        CircuitElement::SwitchSpdt(SwitchSpdt {
            id,
            nodes,
            position,
        }) => {
            let [com, a, b] = nodes.clone();
            vec![
                CircuitElement::Switch(Switch {
                    id: part(id, "a"),
                    nodes: [com.clone(), a],
                    closed: *position == SpdtPosition::A,
                }),
                CircuitElement::Switch(Switch {
                    id: part(id, "b"),
                    nodes: [com, b],
                    closed: *position == SpdtPosition::B,
                }),
            ]
        }
        CircuitElement::Crystal(Crystal { id, nodes, params }) => crystal(id, nodes, params)?,
        CircuitElement::FerriteBead(FerriteBead { id, nodes, params }) => bead(id, nodes, params)?,
        CircuitElement::DigitalPin(p) => pin(p)?,
        _ => return Ok(None),
    };
    Ok(Some(parts))
}

fn crystal(
    id: &str,
    nodes: &[String; 2],
    p: &CrystalParams,
) -> Result<Vec<CircuitElement>, SimError> {
    positive(id, "frequency (Hz)", p.frequency)?;
    let esr = positive(id, "esr (Ω)", p.esr)?;
    let cm = positive(id, "cm (F)", p.cm)?;
    if !(p.c0.is_finite() && p.c0 >= 0.0) {
        return Err(SimError::InvalidComponent(format!(
            "`{id}`: c0 {} F must be zero or positive",
            p.c0
        )));
    }
    let lm = positive(id, "motional inductance (H)", p.lm())?;
    let [a, b] = nodes.clone();
    let (m1, m2) = (part(id, "m1"), part(id, "m2"));
    let mut parts = vec![
        CircuitElement::Resistor(Resistor {
            id: part(id, "rm"),
            nodes: [a.clone(), m1.clone()],
            resistance: esr,
        }),
        CircuitElement::Inductor(Inductor {
            id: part(id, "lm"),
            nodes: [m1, m2.clone()],
            inductance: lm,
            saturation: None,
        }),
        CircuitElement::Capacitor(Capacitor {
            id: part(id, "cm"),
            nodes: [m2, b.clone()],
            capacitance: cm,
        }),
    ];
    if p.c0 > 0.0 {
        parts.push(CircuitElement::Capacitor(Capacitor {
            id: part(id, "c0"),
            nodes: [a, b],
            capacitance: p.c0,
        }));
    }
    Ok(parts)
}

fn bead(
    id: &str,
    nodes: &[String; 2],
    p: &FerriteBeadParams,
) -> Result<Vec<CircuitElement>, SimError> {
    let r_dc = positive(id, "r_dc (Ω)", p.r_dc)?;
    let l = positive(id, "l (H)", p.l)?;
    let r_ac = positive(id, "r_ac (Ω)", p.r_ac)?;
    let [a, b] = nodes.clone();
    let mid = part(id, "m");
    let mut parts = vec![
        CircuitElement::Resistor(Resistor {
            id: part(id, "rdc"),
            nodes: [a, mid.clone()],
            resistance: r_dc,
        }),
        CircuitElement::Inductor(Inductor {
            id: part(id, "l"),
            nodes: [mid.clone(), b.clone()],
            inductance: l,
            saturation: None,
        }),
        CircuitElement::Resistor(Resistor {
            id: part(id, "rac"),
            nodes: [mid.clone(), b.clone()],
            resistance: r_ac,
        }),
    ];
    if let Some(c) = p.c {
        parts.push(CircuitElement::Capacitor(Capacitor {
            id: part(id, "c"),
            nodes: [mid, b],
            capacitance: positive(id, "c (F)", c)?,
        }));
    }
    Ok(parts)
}

/// A digital pin's parts: `#up` from the supply and `#down` to ground (the
/// output stage and the pull resistor, [`DigitalPin::stage`]), the clamp
/// diodes `#pc` (pin to supply) and `#gc` (ground to pin), and with a
/// `c_comp` or `leakage` the capacitor `#c` and current source `#leak`.
///
/// The parts are the same whatever the mode, level and pull, which only
/// set the two resistances, so a session changes them in place.
pub(crate) fn pin(p: &DigitalPin) -> Result<Vec<CircuitElement>, SimError> {
    use crate::element::Device;
    p.validate()?;
    let DigitalPin {
        id, nodes, model, ..
    } = p;
    let [pad, vdd, gnd] = nodes.clone();
    let (r_up, r_down) = p.stage();
    let diode = |suffix: &str, nodes: [String; 2]| {
        CircuitElement::Diode(Diode {
            id: part(id, suffix),
            nodes,
            temperature: 300.15,
            model: Some(model.clamp),
        })
    };
    let mut parts = vec![
        CircuitElement::Resistor(Resistor {
            id: part(id, "up"),
            nodes: [vdd.clone(), pad.clone()],
            resistance: r_up,
        }),
        CircuitElement::Resistor(Resistor {
            id: part(id, "down"),
            nodes: [pad.clone(), gnd.clone()],
            resistance: r_down,
        }),
        diode("pc", [pad.clone(), vdd]),
        diode("gc", [gnd.clone(), pad.clone()]),
    ];
    if model.c_comp > 0.0 {
        parts.push(CircuitElement::Capacitor(Capacitor {
            id: part(id, "c"),
            nodes: [pad.clone(), gnd.clone()],
            capacitance: model.c_comp,
        }));
    }
    if model.leakage != 0.0 {
        parts.push(CircuitElement::CurrentSource(CurrentSource {
            id: part(id, "leak"),
            nodes: [pad, gnd],
            current: model.leakage,
            waveform: None,
        }));
    }
    Ok(parts)
}

/// How to rebuild one composite's result from its parts'.
struct Recipe<'a> {
    id: &'a str,
    /// The terminals whose voltage difference is `voltage_across`.
    across: [&'a str; 2],
    /// Parts whose `current_through`, each times its sign, add up to the
    /// terminal current.
    current: Vec<(String, f64)>,
    /// Parts whose `power` adds up to the element's.
    power: Vec<String>,
}

fn recipes(circuit: &Circuit) -> Vec<Recipe<'_>> {
    circuit
        .components
        .iter()
        .filter_map(|c| match c {
            CircuitElement::SwitchSpdt(SwitchSpdt {
                id,
                nodes,
                position,
            }) => {
                let (throw, suffix) = match position {
                    SpdtPosition::A => (&nodes[1], "a"),
                    SpdtPosition::B => (&nodes[2], "b"),
                };
                Some(Recipe {
                    id,
                    across: [&nodes[0], throw],
                    current: vec![(part(id, suffix), 1.0)],
                    power: vec![part(id, "a"), part(id, "b")],
                })
            }
            CircuitElement::Crystal(Crystal { id, nodes, .. }) => Some(Recipe {
                id,
                across: [&nodes[0], &nodes[1]],
                current: vec![(part(id, "rm"), 1.0), (part(id, "c0"), 1.0)],
                power: vec![part(id, "rm")],
            }),
            CircuitElement::FerriteBead(FerriteBead { id, nodes, .. }) => Some(Recipe {
                id,
                across: [&nodes[0], &nodes[1]],
                current: vec![(part(id, "rdc"), 1.0)],
                power: vec![part(id, "rdc"), part(id, "rac")],
            }),
            // Into the pin: what leaves the pad through each part.
            CircuitElement::DigitalPin(DigitalPin { id, nodes, .. }) => {
                let parts = [
                    ("up", -1.0),
                    ("down", 1.0),
                    ("pc", 1.0),
                    ("gc", -1.0),
                    ("c", 1.0),
                    ("leak", 1.0),
                ];
                Some(Recipe {
                    id,
                    across: [&nodes[0], &nodes[2]],
                    current: parts.iter().map(|(s, k)| (part(id, s), *k)).collect(),
                    power: parts.iter().map(|(s, _)| part(id, s)).collect(),
                })
            }
            _ => None,
        })
        .collect()
}

/// Rebuilds the composites of the flat `circuit` in `result` (and in each
/// of its transient steps): one result per composite under its own id, in
/// place of its parts, and no internal nodes.
///
/// `circuit` is the caller's, before flattening or lowering.
pub(crate) fn fold(circuit: &Circuit, result: &mut SimulationResult) {
    let Ok(flat) = circuit.flattened() else {
        return;
    };
    let recipes = recipes(&flat);
    if recipes.is_empty() {
        return;
    }
    fold_one(
        &recipes,
        &result.node_voltages,
        &mut result.component_results,
    );
    strip(&recipes, &mut result.node_voltages);
    strip(&recipes, &mut result.branch_currents);
    if let Some(tran) = &mut result.transient {
        for step in &mut tran.timesteps {
            fold_one(&recipes, &step.node_voltages, &mut step.component_results);
            strip(&recipes, &mut step.node_voltages);
        }
    }
}

/// The composites of a flattened circuit, to fold its transient steps one
/// at a time as they are taken.
pub(crate) struct StepFolder<'a>(Vec<Recipe<'a>>);

impl<'a> StepFolder<'a> {
    /// `flat` is the caller's circuit flattened, before lowering; `None`
    /// when it has no composites, so there is nothing to fold.
    pub(crate) fn new(flat: &'a Circuit) -> Option<Self> {
        let recipes = recipes(flat);
        (!recipes.is_empty()).then_some(Self(recipes))
    }

    /// `step` as [`fold`] leaves it in the result.
    pub(crate) fn fold(&self, step: &mut TimestepSnapshot) {
        fold_one(&self.0, &step.node_voltages, &mut step.component_results);
        strip(&self.0, &mut step.node_voltages);
    }
}

/// Removes the internal nodes or parts (`<id>#…`) of every composite.
fn strip<V>(circuit_recipes: &[Recipe<'_>], map: &mut HashMap<String, V>) {
    map.retain(|key, _| !is_internal(circuit_recipes, key));
}

/// Removes the composites' internal nodes from `map`. `circuit` is the
/// caller's, before flattening or lowering.
pub(crate) fn strip_nodes<V>(circuit: &Circuit, map: &mut HashMap<String, V>) {
    if let Ok(flat) = circuit.flattened() {
        strip(&recipes(&flat), map);
    }
}

fn is_internal(recipes: &[Recipe<'_>], key: &str) -> bool {
    key.split_once('#')
        .is_some_and(|(owner, _)| recipes.iter().any(|r| r.id == owner))
}

fn fold_one(
    recipes: &[Recipe<'_>],
    voltages: &HashMap<String, f64>,
    results: &mut Vec<ComponentResult>,
) {
    let by_id: HashMap<&str, &ComponentResult> =
        results.iter().map(|r| (r.id.as_str(), r)).collect();
    let sum = |ids: &[String]| -> f64 {
        ids.iter()
            .filter_map(|id| by_id.get(id.as_str()))
            .map(|r| r.power)
            .sum()
    };
    let current = |parts: &[(String, f64)]| -> f64 {
        parts
            .iter()
            .filter_map(|(id, sign)| Some(sign * by_id.get(id.as_str())?.current_through))
            .sum()
    };
    let v = |node: &str| voltages.get(node).copied().unwrap_or(0.0);
    let folded: Vec<ComponentResult> = recipes
        .iter()
        .map(|r| ComponentResult {
            id: r.id.to_string(),
            voltage_across: v(r.across[0]) - v(r.across[1]),
            current_through: current(&r.current),
            power: sum(&r.power),
        })
        .collect();
    results.retain(|c| !is_internal(recipes, &c.id));
    results.extend(folded);
}

#[cfg(test)]
mod tests;
