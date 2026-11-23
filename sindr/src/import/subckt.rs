//! A SPICE subcircuit as a part's model: parse the model text, pick the
//! `.subckt`, and bind the component's terminals to its ports.

use std::collections::BTreeMap;

use super::{normalise, Component, ImportError, ImportOptions, ImportWarning, SubcktModel};
use crate::design::{Block, Instance};
use crate::spice::{parse_str_with_options, ParseOptions};

/// A component placed as an instance of its subcircuit model.
pub(super) struct Placed {
    /// The model's blocks, renamed under the component's id so two models
    /// with the same `.subckt` names don't collide.
    pub blocks: BTreeMap<String, Block>,
    /// The instance, under the component's id.
    pub instance: Instance,
    /// The model's `.global` nets.
    pub globals: Vec<String>,
    /// Parse warnings and terminals the pin map leaves out.
    pub warnings: Vec<ImportWarning>,
    /// Mapped terminals on no net, by the pin map's names. When any are,
    /// nothing else is filled in.
    pub unconnected: Vec<String>,
}

/// Places `component` as an instance of `model`. `terminals` is the node
/// of each of the component's connected terminals, by terminal id;
/// `ground` the circuit's ground node, which the model's ground becomes.
pub(super) fn place(
    component: &Component,
    model: &SubcktModel,
    terminals: &BTreeMap<String, String>,
    ground: &str,
    options: &ImportOptions,
) -> Result<Placed, ImportError> {
    let id = &component.id;
    let model_error = |problem: String| ImportError::SubcktModel {
        component: id.clone(),
        problem,
    };
    let pin_error = |pin: &str, problem: &str| ImportError::SubcktPin {
        component: id.clone(),
        pin: pin.to_string(),
        problem: problem.to_string(),
    };

    // The `.subckt` lines in the text: name and port count.
    let declared = subckt_headers(&model.text);
    let Some((name, ports)) = declared
        .iter()
        .find(|(name, _)| name.eq_ignore_ascii_case(&model.name))
        .cloned()
    else {
        let have = if declared.is_empty() {
            "none".to_string()
        } else {
            declared
                .iter()
                .map(|(n, _)| format!("\"{n}\""))
                .collect::<Vec<_>>()
                .join(", ")
        };
        return Err(model_error(format!(
            "has no .subckt \"{}\" (it has {have})",
            model.name
        )));
    };
    if ports != model.pins.len() {
        return Err(model_error(format!(
            ".subckt \"{name}\" has {ports} port{}, and the pin map gives {}",
            if ports == 1 { "" } else { "s" },
            model.pins.len()
        )));
    }

    // The parser keeps a subcircuit as a block once something places it, so
    // place it once; the block that instance names is the model.
    const PLACEHOLDER: &str = "x_import_placeholder";
    let placeholder_nodes: Vec<String> = (0..ports).map(|i| format!("{PLACEHOLDER}_{i}")).collect();
    let deck = format!(
        "* model of {id}\n{}\n{PLACEHOLDER} {} {name}\n",
        model.text,
        placeholder_nodes.join(" ")
    );
    let lenient = ParseOptions {
        strict: false,
        ..ParseOptions::default()
    };
    let parsed = parse_str_with_options(&deck, lenient)
        .map_err(|e| model_error(format!("doesn't parse: {e}")))?;
    let model_ground = parsed.circuit.ground_node.clone();
    let globals = parsed.circuit.globals.clone();
    let chosen = parsed
        .circuit
        .instances
        .iter()
        .find(|i| i.id.eq_ignore_ascii_case(PLACEHOLDER))
        .map(|i| i.block.clone())
        .ok_or_else(|| model_error(format!("doesn't place .subckt \"{name}\"")))?;
    let outside = parsed.circuit.components.len() + parsed.circuit.instances.len() - 1;
    let blocks = parsed.circuit.blocks;

    // Each pin: a terminal id, or a terminal's name in the options' table.
    let names = options.terminal_names.get(id);
    let mut nodes = Vec::with_capacity(ports);
    let mut used = Vec::with_capacity(ports);
    let mut unconnected = Vec::new();
    for pin in &model.pins {
        let by_name = names.and_then(|names| {
            let wanted = normalise(pin);
            let mut ids = names.iter().filter(|(_, n)| normalise(n) == wanted);
            match (ids.next(), ids.next()) {
                (Some((t, _)), None) => Some(t.clone()),
                _ => None,
            }
        });
        let known = names.is_some_and(|n| n.contains_key(pin));
        let terminal = if terminals.contains_key(pin) || known {
            pin.clone()
        } else if let Some(t) = by_name {
            t
        } else {
            return Err(pin_error(pin, "isn't a terminal of the component"));
        };
        if used.contains(&terminal) {
            return Err(pin_error(pin, "is mapped to two ports"));
        }
        match terminals.get(&terminal) {
            Some(node) => nodes.push(node.clone()),
            None => unconnected.push(pin.clone()),
        }
        used.push(terminal);
    }

    let mut warnings: Vec<ImportWarning> = parsed
        .warnings
        .iter()
        .map(|w| ImportWarning {
            component: id.clone(),
            message: format!("subcircuit model: {}", w.message),
        })
        .collect();
    if !unconnected.is_empty() {
        return Ok(Placed {
            blocks: BTreeMap::new(),
            instance: Instance::new(id.clone(), chosen, Vec::<String>::new()),
            globals,
            warnings,
            unconnected,
        });
    }
    for terminal in terminals.keys().filter(|t| !used.contains(t)) {
        warnings.push(ImportWarning {
            component: id.clone(),
            message: format!(
                "terminal \"{terminal}\" is on a net but not in the subcircuit's pin map, so \
                 nothing connects to it"
            ),
        });
    }
    if outside > 0 {
        warnings.push(ImportWarning {
            component: id.clone(),
            message: format!(
                "subcircuit model: {outside} element{} outside any .subckt ignored",
                if outside == 1 { "" } else { "s" }
            ),
        });
    }

    let rename = |name: &str| format!("{id}:{name}");
    let to_ground = |node: &mut String| {
        if *node == model_ground {
            *node = ground.to_string();
        }
    };
    let blocks = blocks
        .into_iter()
        .map(|(name, mut block)| {
            for element in &mut block.components {
                element.nodes_mut().iter_mut().for_each(to_ground);
                if let Some(control) = element.control_nodes_mut() {
                    control.iter_mut().for_each(to_ground);
                }
                if let Some(expr) = element.expression_mut() {
                    expr.rename(
                        |net| {
                            if net == model_ground {
                                ground.to_string()
                            } else {
                                net.to_string()
                            }
                        },
                        |source| source.to_string(),
                    );
                }
            }
            for instance in &mut block.instances {
                instance.block = rename(&instance.block);
                instance.nodes.iter_mut().for_each(to_ground);
            }
            (rename(&name), block)
        })
        .collect();
    Ok(Placed {
        blocks,
        instance: Instance::new(id.clone(), rename(&chosen), nodes),
        globals,
        warnings,
        unconnected,
    })
}

/// Each `.subckt` line in `text`: its name and port count. Continuation
/// lines (`+`) are joined; `params:` and `name=value` tokens aren't ports.
fn subckt_headers(text: &str) -> Vec<(String, usize)> {
    let mut lines: Vec<String> = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        match line.strip_prefix('+') {
            Some(rest) if !lines.is_empty() => {
                let last = lines.last_mut().expect("not empty");
                last.push(' ');
                last.push_str(rest);
            }
            _ => lines.push(line.to_string()),
        }
    }
    lines
        .iter()
        .filter_map(|line| {
            let mut tokens = line.split_whitespace();
            let card = tokens.next()?;
            if !card.eq_ignore_ascii_case(".subckt") {
                return None;
            }
            let name = tokens.next()?.to_string();
            let ports = tokens
                .take_while(|t| !t.contains('=') && !t.eq_ignore_ascii_case("params:"))
                .count();
            Some((name, ports))
        })
        .collect()
}
