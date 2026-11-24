//! Where each element's extra MNA unknowns and transient history sit.

use std::collections::HashMap;

use crate::circuit::Circuit;

/// For each element of a circuit, by its index in `components`: its first
/// branch row and its first capacitor and inductor history slots.
///
/// - **Branch rows** are the unknowns after the node voltages, counted in
///   circuit order over [`Device::branches`]: one for a voltage source,
///   VCVS, CCVS, op-amp without a model, regulator or behavioural voltage
///   source, two for a modelled op-amp or a comparator (output current,
///   then gain-stage voltage) and for a transformer (primary current, then
///   secondary). [`branch`](Self::branch) is the offset past the node
///   voltages, the number [`Circuit::vsource_branch_index`] gives.
/// - **Capacitor histories**, counted in circuit order over
///   [`Device::capacitor_histories`]: one for a capacitor and for a
///   varactor.
/// - **Inductor histories**, counted in circuit order over
///   [`Device::inductor_histories`]: one for an inductor, one for a relay
///   whose coil has inductance (none for a purely resistive coil), and two
///   for a transformer (primary, then secondary).
///
/// - **Control branches**: for a CCVS or CCCS, the branch row of the
///   source whose current it reads, so a stamp doesn't search the circuit
///   for it on every iteration.
///
/// Other state a transient carries (junction and parasitic capacitances,
/// a diode's stored charge, a battery's state of charge, an op-amp's gain
/// stage) is kept by component id, not here.
///
/// An element that adds none of a kind gets the slot the next one would,
/// so an index is always the count of that kind before the element. The
/// numbers describe the circuit given: the analyses solve SPDT switches,
/// crystals, ferrite beads and zeners after lowering them to simpler
/// elements, so they need a `Layout` of that lowered circuit.
///
/// [`Device::branches`]: super::Device::branches
/// [`Device::capacitor_histories`]: super::Device::capacitor_histories
/// [`Device::inductor_histories`]: super::Device::inductor_histories
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Layout {
    /// Per element: first branch, capacitor and inductor slot. One more
    /// entry than elements, holding the totals.
    starts: Vec<Slots>,
    /// Per element: the branch row its control source's current is in.
    /// Empty when the circuit has no current-controlled source.
    controls: Vec<Option<usize>>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Slots {
    branch: usize,
    capacitor: usize,
    inductor: usize,
}

impl Layout {
    /// Numbers the branch rows and history slots of `circuit`'s elements.
    pub(crate) fn new(circuit: &Circuit) -> Self {
        let mut next = Slots::default();
        let mut starts = Vec::with_capacity(circuit.components.len() + 1);
        for component in &circuit.components {
            starts.push(next);
            let device = component.device();
            next.branch += device.branches();
            next.capacitor += device.capacitor_histories();
            next.inductor += device.inductor_histories();
        }
        starts.push(next);
        let controls = Self::controls(circuit, &starts);
        Self { starts, controls }
    }

    /// Each element's control branch: the first element with its control
    /// source's id whose branch current others can read, as
    /// [`Circuit::vsource_branch_index`] finds it.
    fn controls(circuit: &Circuit, starts: &[Slots]) -> Vec<Option<usize>> {
        let devices = || circuit.components.iter().map(|c| c.device());
        if devices().all(|d| d.control_source().is_none()) {
            return Vec::new();
        }
        let mut readable: HashMap<&str, usize> = HashMap::new();
        for (k, device) in devices().enumerate() {
            if device.exposes_branch_current() {
                readable.entry(device.id()).or_insert(starts[k].branch);
            }
        }
        devices()
            .map(|d| d.control_source().and_then(|id| readable.get(id).copied()))
            .collect()
    }

    /// Element `k`'s first branch row, counted from the first row past the
    /// node voltages.
    ///
    /// # Panics
    ///
    /// If `k` is past the last element.
    pub(crate) fn branch(&self, k: usize) -> usize {
        self.slots(k).branch
    }

    /// The branch row, counted from the first past the node voltages, of
    /// the current element `k` reads: its control source's, for a CCVS or
    /// CCCS whose source is in the circuit; `None` otherwise.
    pub(crate) fn control(&self, k: usize) -> Option<usize> {
        self.controls.get(k).copied().flatten()
    }

    /// Element `k`'s first capacitor history slot.
    ///
    /// # Panics
    ///
    /// If `k` is past the last element.
    pub(crate) fn capacitor(&self, k: usize) -> usize {
        self.slots(k).capacitor
    }

    /// Element `k`'s first inductor history slot.
    ///
    /// # Panics
    ///
    /// If `k` is past the last element.
    pub(crate) fn inductor(&self, k: usize) -> usize {
        self.slots(k).inductor
    }

    /// Branch rows in the whole circuit.
    #[cfg_attr(
        not(test),
        expect(dead_code, reason = "the transient doesn't use it yet")
    )]
    pub(crate) fn branches(&self) -> usize {
        self.totals().branch
    }

    /// Capacitor histories in the whole circuit.
    #[cfg_attr(
        not(test),
        expect(dead_code, reason = "the transient doesn't use it yet")
    )]
    pub(crate) fn capacitors(&self) -> usize {
        self.totals().capacitor
    }

    /// Inductor histories in the whole circuit.
    #[cfg_attr(
        not(test),
        expect(dead_code, reason = "the transient doesn't use it yet")
    )]
    pub(crate) fn inductors(&self) -> usize {
        self.totals().inductor
    }

    fn slots(&self, k: usize) -> Slots {
        assert!(k + 1 < self.starts.len(), "element {k} is out of range");
        self.starts[k]
    }

    fn totals(&self) -> Slots {
        self.starts.last().copied().unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::{BehaviouralOutput, CircuitElement, OpAmpModel};
    use crate::element::{
        Behavioural, Capacitor, Cccs, Ccvs, Comparator, Inductor, OpAmp, Relay, Resistor,
        Transformer, Varactor, Vcvs, VoltageRegulator, VoltageSource,
    };
    use crate::expression::Expression;

    fn two(id: &str) -> [String; 2] {
        [format!("{id}.a"), "0".into()]
    }

    fn three(id: &str) -> [String; 3] {
        [format!("{id}.a"), format!("{id}.b"), "0".into()]
    }

    /// Every kind of element that adds branch rows or history slots, with
    /// ones that add none between them.
    fn circuit() -> Circuit {
        let resistor = |id: &str| {
            CircuitElement::Resistor(Resistor {
                id: id.into(),
                nodes: two(id),
                resistance: 1.0,
            })
        };
        let behavioural = |id: &str, output| {
            CircuitElement::Behavioural(Behavioural {
                id: id.into(),
                nodes: two(id),
                output,
                expr: Expression::parse("1").unwrap(),
            })
        };
        let op_amp = |id: &str, model| {
            CircuitElement::OpAmp(OpAmp {
                id: id.into(),
                nodes: three(id),
                v_pos: 15.0,
                v_neg: -15.0,
                model,
            })
        };
        let relay = |id: &str, inductance| {
            CircuitElement::Relay(Relay {
                id: id.into(),
                nodes: [format!("{id}.a"), "0".into(), format!("{id}.c"), "0".into()],
                coil_resistance: 500.0,
                pickup_voltage: 3.0,
                inductance,
            })
        };
        let transformer = CircuitElement::Transformer(Transformer {
            id: "T1".into(),
            nodes: ["T1.a".into(), "0".into(), "T1.b".into(), "0".into()],
            l1: 1.0,
            l2: 1.0,
            k: 0.99,
            saturation: None,
        });
        Circuit::new(
            "0",
            vec![
                resistor("R1"),
                CircuitElement::Capacitor(Capacitor {
                    id: "C1".into(),
                    nodes: two("C1"),
                    capacitance: 1e-6,
                }),
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: two("V1"),
                    voltage: 1.0,
                    waveform: None,
                }),
                op_amp("U1", Some(OpAmpModel::default())),
                CircuitElement::Inductor(Inductor {
                    id: "L1".into(),
                    nodes: two("L1"),
                    inductance: 1e-3,
                    saturation: None,
                }),
                transformer,
                relay("K1", 0.0),
                CircuitElement::Vcvs(Vcvs {
                    id: "E1".into(),
                    nodes: two("E1"),
                    control_nodes: two("E1c"),
                    gain: 1.0,
                }),
                relay("K2", 1e-3),
                CircuitElement::Ccvs(Ccvs {
                    id: "H1".into(),
                    nodes: two("H1"),
                    control_source: "V1".into(),
                    rm: 1.0,
                }),
                behavioural("B1", BehaviouralOutput::Current),
                CircuitElement::Varactor(Varactor {
                    id: "CV1".into(),
                    nodes: two("CV1"),
                    params: Default::default(),
                }),
                op_amp("U2", None),
                CircuitElement::Comparator(Comparator {
                    id: "X1".into(),
                    nodes: three("X1"),
                    v_pos: 5.0,
                    v_neg: 0.0,
                }),
                behavioural("B2", BehaviouralOutput::Voltage),
                CircuitElement::VoltageRegulator(VoltageRegulator {
                    id: "REG1".into(),
                    nodes: three("REG1"),
                    voltage: 5.0,
                    model: None,
                }),
                resistor("R2"),
            ],
        )
    }

    /// A CCVS's or CCCS's control branch is the one
    /// `vsource_branch_index` gives for its source, the first of a repeated
    /// id that exposes a branch current; a missing source has none.
    #[test]
    fn controls_agree_with_vsource_branch_index() {
        let mut circuit = circuit();
        let cccs = |id: &str, source: &str| {
            CircuitElement::Cccs(Cccs {
                id: id.into(),
                nodes: two(id),
                control_source: source.into(),
                alpha: 1.0,
            })
        };
        circuit.components.extend([
            cccs("F1", "U2"),
            cccs("F2", "REG1"),
            cccs("F3", "V9"),
            cccs("F4", "R1"),
            CircuitElement::VoltageSource(VoltageSource {
                id: "U2".into(),
                nodes: two("U2b"),
                voltage: 1.0,
                waveform: None,
            }),
        ]);
        let layout = Layout::new(&circuit);
        let mut controlled = 0;
        for (k, component) in circuit.components.iter().enumerate() {
            match component.device().control_source() {
                Some(source) => {
                    controlled += 1;
                    assert_eq!(
                        layout.control(k),
                        circuit.vsource_branch_index(source),
                        "{}",
                        component.id()
                    );
                }
                None => assert_eq!(layout.control(k), None, "{}", component.id()),
            }
        }
        assert_eq!(controlled, 5);
        assert_eq!(layout.control(circuit.components.len() - 3), None);
        assert_eq!(Layout::new(&Circuit::new("0", Vec::new())).control(0), None);
    }

    /// Every element whose branch current others read starts where
    /// `vsource_branch_index` says it does.
    #[test]
    fn branches_agree_with_vsource_branch_index() {
        let circuit = circuit();
        let layout = Layout::new(&circuit);
        let mut sources = 0;
        for (k, component) in circuit.components.iter().enumerate() {
            let expected = circuit.vsource_branch_index(component.id());
            if component.device().exposes_branch_current() {
                assert_eq!(Some(layout.branch(k)), expected, "{}", component.id());
                sources += 1;
            } else {
                assert_eq!(expected, None, "{}", component.id());
            }
        }
        assert_eq!(sources, 8);
        assert_eq!(layout.branches(), circuit.count_voltage_sources());
    }

    /// Branch rows by hand, transformer included.
    #[test]
    fn branch_rows_count_every_earlier_branch() {
        let circuit = circuit();
        let layout = Layout::new(&circuit);
        let rows: Vec<(&str, usize)> = circuit
            .components
            .iter()
            .enumerate()
            .map(|(k, c)| (c.id(), layout.branch(k)))
            .collect();
        assert_eq!(
            rows,
            [
                ("R1", 0),
                ("C1", 0),
                ("V1", 0),
                ("U1", 1),
                ("L1", 3),
                ("T1", 3),
                ("K1", 5),
                ("E1", 5),
                ("K2", 6),
                ("H1", 6),
                ("B1", 7),
                ("CV1", 7),
                ("U2", 7),
                ("X1", 8),
                ("B2", 10),
                ("REG1", 11),
                ("R2", 12),
            ]
        );
        assert_eq!(layout.branches(), 12);
    }

    /// Capacitors and varactors share one numbering; inductors, coils with
    /// inductance and transformer windings another, in circuit order as
    /// the transient walks them.
    #[test]
    fn history_slots_count_capacitors_and_inductors_in_order() {
        let circuit = circuit();
        let layout = Layout::new(&circuit);
        let at = |id: &str| {
            let k = circuit
                .components
                .iter()
                .position(|c| c.id() == id)
                .unwrap();
            (layout.capacitor(k), layout.inductor(k))
        };
        assert_eq!(at("R1"), (0, 0));
        assert_eq!(at("C1"), (0, 0));
        assert_eq!(at("L1"), (1, 0));
        assert_eq!(at("T1"), (1, 1));
        assert_eq!(at("K1"), (1, 3));
        assert_eq!(at("K2"), (1, 3));
        assert_eq!(at("H1"), (1, 4));
        assert_eq!(at("CV1"), (1, 4));
        assert_eq!(at("R2"), (2, 4));
        assert_eq!((layout.capacitors(), layout.inductors()), (2, 4));
    }

    #[test]
    fn an_empty_circuit_has_no_slots() {
        let layout = Layout::new(&Circuit::new("0", Vec::new()));
        assert_eq!(
            (layout.branches(), layout.capacitors(), layout.inductors()),
            (0, 0, 0)
        );
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn an_index_past_the_last_element_panics() {
        Layout::new(&Circuit::new("0", Vec::new())).branch(0);
    }
}
