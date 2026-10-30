//! Properties every linear circuit must have, checked on random networks
//! with proptest: KCL and power balance, reciprocity, invariance under
//! renaming nodes and reordering components, and linear scaling.

use std::collections::HashMap;

use proptest::prelude::*;
use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{solve_circuit, Circuit, CircuitElement};

/// A random connected network on nodes `n1..=nodes`: a resistor ring from
/// ground through every node and back (so nothing floats or dangles), then
/// extra elements
/// between random node pairs. `kinds` picks each extra element: 0 a
/// resistor, 1 a capacitor, 2 an inductor (reactive ones only if `reactive`).
/// Each inductor sits in series with 1 Ω on an internal node: a loop of
/// inductors alone is a short at DC, so the operating point AC needs would
/// be singular.
#[derive(Debug, Clone)]
struct Network {
    nodes: usize,
    chain: Vec<f64>,
    extras: Vec<(usize, usize, u8, f64)>,
}

fn node(i: usize) -> String {
    if i == 0 {
        "0".into()
    } else {
        format!("n{i}")
    }
}

impl Network {
    fn elements(&self, reactive: bool) -> Vec<CircuitElement> {
        let mut out = Vec::new();
        for (i, &r) in self.chain.iter().enumerate() {
            out.push(CircuitElement::Resistor {
                id: format!("RC{i}"),
                nodes: [node(i), node((i + 1) % (self.nodes + 1))],
                resistance: r,
            });
        }
        for (k, &(a, b, kind, value)) in self.extras.iter().enumerate() {
            let nodes = [node(a), node(b)];
            let id = format!("X{k}");
            match if reactive { kind } else { 0 } {
                1 => out.push(CircuitElement::Capacitor {
                    id,
                    nodes,
                    capacitance: value * 1e-9,
                }),
                2 => {
                    let mid = format!("x{k}");
                    out.push(CircuitElement::Inductor {
                        id,
                        nodes: [nodes[0].clone(), mid.clone()],
                        inductance: value * 1e-6,
                    });
                    out.push(CircuitElement::Resistor {
                        id: format!("XR{k}"),
                        nodes: [mid, nodes[1].clone()],
                        resistance: 1.0,
                    });
                }
                _ => out.push(CircuitElement::Resistor {
                    id,
                    nodes,
                    resistance: value,
                }),
            }
        }
        out
    }
}

fn network() -> impl Strategy<Value = Network> {
    (2usize..7).prop_flat_map(|nodes| {
        (
            Just(nodes),
            prop::collection::vec(10.0f64..1e5, nodes + 1),
            prop::collection::vec((0..=nodes, 0..=nodes, 0u8..3, 10.0f64..1e5), 0..8),
        )
            .prop_map(|(nodes, chain, extras)| Network {
                nodes,
                // Self-loops carry no current and make no sense to stamp.
                extras: extras.into_iter().filter(|e| e.0 != e.1).collect(),
                chain,
            })
    })
}

fn vsource(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
    }
}

/// Resistor currents leaving `node`, from the solved node voltages.
fn resistor_current_out(circuit: &Circuit, v: &HashMap<String, f64>, node: &str) -> f64 {
    circuit
        .components
        .iter()
        .map(|c| match c {
            CircuitElement::Resistor {
                nodes, resistance, ..
            } if nodes[0] == node => (v[&nodes[0]] - v[&nodes[1]]) / resistance,
            CircuitElement::Resistor {
                nodes, resistance, ..
            } if nodes[1] == node => (v[&nodes[1]] - v[&nodes[0]]) / resistance,
            _ => 0.0,
        })
        .sum()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// Driven at the top of the chain: KCL holds at every free node, and
    /// the power the source delivers equals the power the resistors burn.
    #[test]
    fn kcl_and_power_balance(net in network(), volts in -20.0f64..20.0) {
        let top = node(net.nodes);
        let mut elements = net.elements(false);
        elements.push(vsource("V1", &top, "0", volts));
        let c = circuit(elements);
        let v = solve_circuit(&c).unwrap().node_voltages;

        let scale = volts.abs() / 10.0 + 1e-12;
        for i in 1..net.nodes {
            let residual = resistor_current_out(&c, &v, &node(i));
            prop_assert!(residual.abs() <= 1e-9 * scale, "KCL at {}: {residual}", node(i));
        }
        let delivered = volts * resistor_current_out(&c, &v, &top);
        let burned: f64 = c.components.iter().map(|e| match e {
            CircuitElement::Resistor { nodes, resistance, .. } => {
                (v[&nodes[0]] - v[&nodes[1]]).powi(2) / resistance
            }
            _ => 0.0,
        }).sum();
        prop_assert!((delivered - burned).abs() <= 1e-9 * burned.abs().max(1e-15),
            "delivered {delivered} W, dissipated {burned} W");
    }

    /// A passive RLC network is reciprocal: with both ports terminated in
    /// the same R_s, driving port a and reading port b gives the same
    /// complex voltage as driving b and reading a.
    #[test]
    fn rlc_network_is_reciprocal(
        net in network(),
        ports in (1usize..7, 1usize..7),
        f in 10.0f64..1e6,
    ) {
        // Two distinct ports among n1..=nodes.
        let i = ports.0 % net.nodes;
        let j = (i + 1 + ports.1 % (net.nodes - 1)) % net.nodes;
        let (a, b) = (node(1 + i), node(1 + j));
        let drive = |from: &str, to: &str| {
            let mut elements = net.elements(true);
            elements.push(vsource("V1", "src", "0", 0.0));
            elements.push(CircuitElement::Resistor {
                id: "RSRC".into(), nodes: ["src".into(), from.into()], resistance: 50.0,
            });
            elements.push(CircuitElement::Resistor {
                id: "RLOAD".into(), nodes: [to.into(), "0".into()], resistance: 50.0,
            });
            let config = AcConfig {
                f_start: f, f_stop: f, num_points: 1,
                spacing: FrequencySpacing::Linear,
                source_id: "V1".into(), ac_magnitude: 1.0,
            };
            solve_ac(&circuit(elements), &config).unwrap().points[0].node_voltages[to]
        };
        let (ab, ba) = (drive(&a, &b), drive(&b, &a));
        // Round-off only: the two solves pivot differently (and differently
        // again under the sparse LU), so allow a few ulps of cancellation.
        prop_assert!((ab - ba).norm() <= 1e-8 * ab.norm().max(1e-12),
            "{a}→{b}: {ab}, {b}→{a}: {ba} at {f} Hz");
    }

    /// Renaming every node and reversing the component order changes no
    /// node voltage. The rename reverses the nodes' alphabetical order
    /// (`n1..nN` becomes `mN..m1`), so the MNA unknowns really are permuted:
    /// sindr numbers nodes in sorted order.
    #[test]
    fn solution_ignores_node_names_and_component_order(net in network(), volts in -20.0f64..20.0) {
        let top = node(net.nodes);
        let mut elements = net.elements(false);
        elements.push(vsource("V1", &top, "0", volts));
        let original = solve_circuit(&circuit(elements.clone())).unwrap().node_voltages;

        let rename = |n: &String| match n.strip_prefix('n') {
            Some(i) => format!("m{}", net.nodes + 1 - i.parse::<usize>().unwrap()),
            None => n.clone(),
        };
        let renamed: Vec<CircuitElement> = elements.iter().rev().map(|e| {
            let mut e = e.clone();
            match &mut e {
                CircuitElement::Resistor { nodes, .. }
                | CircuitElement::VoltageSource { nodes, .. } => {
                    *nodes = [rename(&nodes[0]), rename(&nodes[1])];
                }
                _ => unreachable!(),
            }
            e
        }).collect();
        let solved = solve_circuit(&circuit(renamed)).unwrap().node_voltages;
        for (n, v) in &original {
            let w = solved[&rename(n)];
            prop_assert!((v - w).abs() <= 1e-12 * volts.abs().max(1.0), "{n}: {v} vs {w}");
        }
    }

    /// Scaling every source by k scales every node voltage by k.
    #[test]
    fn linear_solution_scales_with_sources(
        net in network(),
        volts in -20.0f64..20.0,
        amps in -1e-3f64..1e-3,
        k in -10.0f64..10.0,
    ) {
        let solve = |scale: f64| {
            let mut elements = net.elements(false);
            elements.push(vsource("V1", &node(net.nodes), "0", scale * volts));
            elements.push(CircuitElement::CurrentSource {
                id: "I1".into(), nodes: ["0".into(), node(1)],
                current: scale * amps, waveform: None,
            });
            solve_circuit(&circuit(elements)).unwrap().node_voltages
        };
        let (base, scaled) = (solve(1.0), solve(k));
        let full_scale = base.values().fold(0.0f64, |m, v| m.max(v.abs()));
        for (n, v) in &base {
            prop_assert!((k * v - scaled[n]).abs() <= 1e-9 * (k.abs() * full_scale).max(1e-12),
                "{n}: k·V = {}, V(k·sources) = {}", k * v, scaled[n]);
        }
    }
}
