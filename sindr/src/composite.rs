//! Elements solved as a network of simpler ones: the SPDT switch (two
//! switches), the crystal (C0 beside Rm–Lm–Cm), the ferrite bead (Rdc
//! into L ∥ Rac ∥ C) and the digital pin (its output stage, clamps, die
//! capacitance and leakage) and the logic gate (its input stage, delay
//! and a pin's output stage).
//!
//! [`lower`] turns each into its parts before any analysis, with internal
//! ids and nodes `<id>#…`. [`fold`] puts a result back together: one
//! [`ComponentResult`] under the element's own id, from its parts', and the
//! parts and internal nodes gone.

use crate::circuit::BehaviouralOutput;
use crate::element::{
    Behavioural, Capacitor, Crystal, CurrentSource, DigitalPin, Diode, FerriteBead, GateKind,
    Inductor, LogicGate, Resistor, Switch, SwitchSpdt,
};
use crate::expression::Expression;
use crate::handle::NodeId;
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
            | CircuitElement::LogicGate(LogicGate { .. })
    )
}

fn positive(id: &str, what: &str, v: f64) -> Result<f64, SimError> {
    if v.is_finite() && v > 0.0 {
        Ok(v)
    } else {
        Err(SimError::component(
            id,
            format!("{what} {v} must be a positive, finite number"),
        ))
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
        CircuitElement::LogicGate(g) => gate(g)?,
        _ => return Ok(None),
    };
    Ok(Some(parts))
}

fn crystal(
    id: &str,
    nodes: &[NodeId; 2],
    p: &CrystalParams,
) -> Result<Vec<CircuitElement>, SimError> {
    positive(id, "frequency (Hz)", p.frequency)?;
    let esr = positive(id, "esr (Ω)", p.esr)?;
    let cm = positive(id, "cm (F)", p.cm)?;
    if !(p.c0.is_finite() && p.c0 >= 0.0) {
        return Err(SimError::component(
            id,
            format!("c0 {} F must be zero or positive", p.c0),
        ));
    }
    let lm = positive(id, "motional inductance (H)", p.lm())?;
    let [a, b] = nodes.clone();
    let (m1, m2): (NodeId, NodeId) = (part(id, "m1").into(), part(id, "m2").into());
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
    nodes: &[NodeId; 2],
    p: &FerriteBeadParams,
) -> Result<Vec<CircuitElement>, SimError> {
    let r_dc = positive(id, "r_dc (Ω)", p.r_dc)?;
    let l = positive(id, "l (H)", p.l)?;
    let r_ac = positive(id, "r_ac (Ω)", p.r_ac)?;
    let [a, b] = nodes.clone();
    let mid: NodeId = part(id, "m").into();
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
    let diode = |suffix: &str, nodes: [NodeId; 2]| {
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

/// The current the delay's follower slews its capacitor with (A).
const DELAY_I: f64 = 1e-3;
/// How far the follower's node is from its target (V, of the 1 V logic
/// level) when its current is 76 % of `DELAY_I`: the sharpness of its
/// settling.
const DELAY_EPS: f64 = 0.01;
/// The DC path from the delay's input to its follower (Ω): 1 µA a volt,
/// 0.1 % of `DELAY_I`.
const DELAY_R: f64 = 1e6;
/// A plain input's threshold span over its switching width.
const PLAIN_SHARPNESS: f64 = 20.0;
/// A Schmitt state's threshold span over its switching width: the
/// higher, the closer it switches to `vih` and `vil`.
const SCHMITT_SHARPNESS: f64 = 64.0;
/// A Schmitt state's conductance toward its target (S).
const STATE_G: f64 = 1e-3;
/// A Schmitt state's time constant (s): fast beside any gate's delay.
const STATE_TAU: f64 = 1e-9;

/// A logic gate's parts, all referred to its ground terminal `gnd`:
///
/// - Each input `k`'s logic value, 0 to 1: a smooth step from `vil` to
///   `vih`. With Schmitt inputs it is the voltage of `#h<k>`, a source
///   state that feeds its own value back into its threshold, so it holds
///   between the thresholds.
/// - `#f`, a source setting `#x` to the gate's function of those values
///   (V, 0 to 1).
/// - With a `tpd`, `#sd`, `#cd` and `#rd` make `#d` follow `#x` at a
///   limited slew, crossing half way `tpd` after it switches.
/// - The output stage, as the lagged level `y` sets it: `#up` drives
///   `y · v(vdd, out) / r_high` from the supply, `#dn` drives
///   `(1 − y) · v(out, gnd) / r_low` to ground, with `#ru` and `#rd0` at
///   `R_OFF` (1 TΩ) keeping the output's
///   DC path, the clamps `#pc` and `#gc`, and `c_comp` as `#c`.
/// - `c_in` on each input as `#cin<k>`.
pub(crate) fn gate(g: &LogicGate) -> Result<Vec<CircuitElement>, SimError> {
    use crate::element::Device;
    g.validate()?;
    let id = &g.id;
    let m = &g.model;
    let inputs = g.inputs();
    let [out, vdd, gnd] = g.outputs();
    // Expressions are written over placeholder names, then renamed to the
    // circuit's nets, which may hold characters the parser doesn't read.
    let mut names: HashMap<String, NodeId> = HashMap::new();
    for (k, input) in inputs.iter().enumerate() {
        names.insert(format!("in{k}"), input.clone());
        names.insert(format!("h{k}"), part(id, &format!("h{k}")).into());
    }
    for (name, node) in [
        ("out", out.clone()),
        ("vdd", vdd.clone()),
        ("gnd", gnd.clone()),
        ("x", part(id, "x").into()),
        ("d", part(id, "d").into()),
    ] {
        names.insert(name.into(), node);
    }
    let expr = |text: String| -> Result<Expression, SimError> {
        let mut e = Expression::parse(&text).map_err(|e| g.internal_error(e))?;
        e.rename(|n| names[n].to_string(), |s| s.to_string());
        Ok(e)
    };
    let behavioural = |suffix: &str, nodes: [NodeId; 2], output, text: String| {
        Ok::<_, SimError>(CircuitElement::Behavioural(Behavioural {
            id: part(id, suffix),
            nodes,
            output,
            expr: expr(text)?,
        }))
    };
    let mut parts = Vec::new();

    // Each input's logic value. Plain: tanh over (vih − vil) / 20 either
    // side of the midpoint, as steep as a CMOS input's transfer and fully
    // switched well inside vil and vih, so a delay counts from the input's
    // midpoint whatever its edge rate. Schmitt: the
    // state `#h<k>` follows a target whose threshold moves from vih to vil
    // as the state rises. The loop gain of SCHMITT_SHARPNESS / 2 latches it
    // within a few percent of the span of each threshold; its 1 ns
    // capacitor lets it switch over time rather than jump.
    let span = m.vih - m.vil;
    let mut values = Vec::with_capacity(inputs.len());
    for k in 0..inputs.len() {
        if g.schmitt {
            let h: NodeId = part(id, &format!("h{k}")).into();
            parts.push(behavioural(
                &format!("h{k}"),
                [gnd.clone(), h.clone()],
                BehaviouralOutput::Current,
                format!(
                    "({STATE_G:e})*(0.5*(1+tanh((v(in{k},gnd)-({vih:e})+v(h{k},gnd)*({span:e}))/({w:e})))-v(h{k},gnd))",
                    vih = m.vih,
                    w = span / SCHMITT_SHARPNESS,
                ),
            )?);
            parts.push(CircuitElement::Capacitor(Capacitor {
                id: part(id, &format!("ch{k}")),
                nodes: [h, gnd.clone()],
                capacitance: STATE_G * STATE_TAU,
            }));
            values.push(format!("v(h{k},gnd)"));
        } else {
            values.push(format!(
                "0.5*(1+tanh((v(in{k},gnd)-({mid:e}))/({w:e})))",
                mid = 0.5 * (m.vil + m.vih),
                w = span / PLAIN_SHARPNESS,
            ));
        }
    }
    let product = |terms: Vec<String>| {
        terms
            .iter()
            .map(|t| format!("({t})"))
            .collect::<Vec<_>>()
            .join("*")
    };
    let not = |t: &str| format!("(1-({t}))");
    let all = product(values.clone());
    let none = product(values.iter().map(|v| not(v)).collect());
    // Parity: (1 − Π(1 − 2a)) / 2 is 1 for an odd count of 1s.
    let odd = format!(
        "0.5*(1-{})",
        product(values.iter().map(|v| format!("1-2*({v})")).collect())
    );
    let function = match g.kind {
        GateKind::Buffer => values[0].clone(),
        GateKind::Not => not(&values[0]),
        GateKind::And => all,
        GateKind::Nand => not(&all),
        GateKind::Or => not(&none),
        GateKind::Nor => none,
        GateKind::Xor => odd,
        GateKind::Xnor => not(&odd),
    };
    parts.push(behavioural(
        "f",
        [part(id, "x").into(), gnd.clone()],
        BehaviouralOutput::Voltage,
        function,
    )?);
    // The delay: `#d` slews from `#x`'s old level to its new one at
    // 1 / (2 · tpd) V/s, `#sd` driving `#cd` with a current that saturates
    // at DELAY_I, so it crosses half way `tpd` after `#x` switched. A
    // follower, not an RC lag, so the delay is the same however recently
    // the gate last switched. `#rd` gives `#d` its DC path.
    let level = if g.tpd > 0.0 {
        let d: NodeId = part(id, "d").into();
        parts.push(behavioural(
            "sd",
            [gnd.clone(), d.clone()],
            BehaviouralOutput::Current,
            format!("({DELAY_I:e})*tanh((v(x,gnd)-v(d,gnd))/({DELAY_EPS:e}))"),
        )?);
        parts.push(CircuitElement::Capacitor(Capacitor {
            id: part(id, "cd"),
            nodes: [d.clone(), gnd.clone()],
            capacitance: 2.0 * DELAY_I * g.tpd,
        }));
        parts.push(CircuitElement::Resistor(Resistor {
            id: part(id, "rd"),
            nodes: [part(id, "x").into(), d],
            resistance: DELAY_R,
        }));
        "v(d,gnd)"
    } else {
        "v(x,gnd)"
    };

    parts.push(behavioural(
        "up",
        [vdd.clone(), out.clone()],
        BehaviouralOutput::Current,
        format!("({level})*v(vdd,out)/({:e})", m.r_high),
    )?);
    parts.push(behavioural(
        "dn",
        [out.clone(), gnd.clone()],
        BehaviouralOutput::Current,
        format!("(1-({level}))*v(out,gnd)/({:e})", m.r_low),
    )?);
    let r_off = crate::element::R_OFF;
    parts.push(CircuitElement::Resistor(Resistor {
        id: part(id, "ru"),
        nodes: [vdd.clone(), out.clone()],
        resistance: r_off,
    }));
    parts.push(CircuitElement::Resistor(Resistor {
        id: part(id, "rl"),
        nodes: [out.clone(), gnd.clone()],
        resistance: r_off,
    }));
    let diode = |suffix: &str, nodes: [NodeId; 2]| {
        CircuitElement::Diode(Diode {
            id: part(id, suffix),
            nodes,
            temperature: 300.15,
            model: Some(m.clamp),
        })
    };
    parts.push(diode("pc", [out.clone(), vdd.clone()]));
    parts.push(diode("gc", [gnd.clone(), out.clone()]));
    if m.c_comp > 0.0 {
        parts.push(CircuitElement::Capacitor(Capacitor {
            id: part(id, "c"),
            nodes: [out.clone(), gnd.clone()],
            capacitance: m.c_comp,
        }));
    }
    if g.c_in > 0.0 {
        for (k, input) in inputs.iter().enumerate() {
            parts.push(CircuitElement::Capacitor(Capacitor {
                id: part(id, &format!("cin{k}")),
                nodes: [input.clone(), gnd.clone()],
                capacitance: g.c_in,
            }));
        }
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
            // Into the output: what leaves it through the output stage.
            CircuitElement::LogicGate(g) if g.nodes.len() >= 3 => {
                let [out, _, gnd] = g.outputs();
                let stage = [
                    ("up", -1.0),
                    ("dn", 1.0),
                    ("ru", -1.0),
                    ("rl", 1.0),
                    ("pc", 1.0),
                    ("gc", -1.0),
                    ("c", 1.0),
                ];
                Some(Recipe {
                    id: &g.id,
                    across: [out, gnd],
                    current: stage.iter().map(|(s, k)| (part(&g.id, s), *k)).collect(),
                    power: stage.iter().map(|(s, _)| part(&g.id, s)).collect(),
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
