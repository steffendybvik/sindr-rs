//! `.sens` against closed-form answers, and against finite differences of
//! full DC solves, which use only the DC model.

use approx::{assert_relative_eq, relative_eq};
use sindr::devices::mosfet::MosfetParams;
use sindr::element::{
    Bjt, Capacitor, Cccs, Ccvs, CurrentSource, Diode, Inductor, Mosfet, Relay, Resistor,
    Transformer, Vccs, Vcvs, VoltageSource,
};
use sindr::{
    solve_circuit, solve_sensitivity, BjtKind, BjtModel, Circuit, CircuitElement, DiodeModel,
    MosfetKind, SensConfig, SensParameter, SensResult, SimError, TfOutput,
};

fn vsource(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    })
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    })
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
        ..Default::default()
    }
}

fn v_out(node: &str) -> TfOutput {
    TfOutput::Voltage {
        node: node.into(),
        reference: None,
    }
}

fn sens(circuit: &Circuit, output: TfOutput) -> SensResult {
    solve_sensitivity(circuit, &SensConfig::new(output)).unwrap()
}

fn of(result: &SensResult, id: &str) -> f64 {
    result.get(id).unwrap().sensitivity
}

/// V volts over R1 and R2, out at their junction.
const V: f64 = 8.0;
const R1: f64 = 1e3;
const R2: f64 = 3e3;

fn divider() -> Circuit {
    circuit(vec![
        vsource("V1", "in", "0", V),
        resistor("R1", "in", "out", R1),
        resistor("R2", "out", "0", R2),
    ])
}

#[test]
fn divider_voltage_output() {
    let r = sens(&divider(), v_out("out"));
    let total = R1 + R2;
    assert_relative_eq!(of(&r, "V1"), R2 / total, max_relative = 1e-12);
    assert_relative_eq!(
        of(&r, "R1"),
        -V * R2 / (total * total),
        max_relative = 1e-12
    );
    assert_relative_eq!(of(&r, "R2"), V * R1 / (total * total), max_relative = 1e-12);
}

#[test]
fn entry_carries_value_and_per_percent() {
    let r = sens(&divider(), v_out("out"));
    let e = r.get("R2").unwrap();
    assert_eq!(e.parameter, SensParameter::Resistance);
    assert_eq!(e.value, R2);
    assert_relative_eq!(e.per_percent, e.sensitivity * R2 / 100.0);
    assert_eq!(r.entries.len(), 3);
}

#[test]
fn output_against_a_reference_node() {
    // V(in, out) = V·R1/(R1 + R2).
    let output = TfOutput::Voltage {
        node: "in".into(),
        reference: Some("out".into()),
    };
    let r = sens(&divider(), output);
    let total = R1 + R2;
    assert_relative_eq!(of(&r, "V1"), R1 / total, max_relative = 1e-12);
    assert_relative_eq!(of(&r, "R1"), V * R2 / (total * total), max_relative = 1e-12);
}

#[test]
fn current_output_through_a_voltage_source() {
    // I(V1), from + through the source, is −V/(R1 + R2).
    let output = TfOutput::Current {
        source_id: "V1".into(),
    };
    let r = sens(&divider(), output);
    let total = R1 + R2;
    assert_relative_eq!(of(&r, "V1"), -1.0 / total, max_relative = 1e-12);
    assert_relative_eq!(of(&r, "R1"), V / (total * total), max_relative = 1e-12);
}

#[test]
fn current_source_value() {
    // I1 pushes I into `a`, over R: V(a) = I·R.
    let c = circuit(vec![
        CircuitElement::CurrentSource(CurrentSource {
            id: "I1".into(),
            nodes: ["0".into(), "a".into()],
            current: 2e-3,
            waveform: None,
        }),
        resistor("R1", "a", "0", 5e3),
    ]);
    let r = sens(&c, v_out("a"));
    assert_relative_eq!(of(&r, "I1"), 5e3, max_relative = 1e-12);
    assert_relative_eq!(of(&r, "R1"), 2e-3, max_relative = 1e-12);
}

/// `circuit` with `parameter` of the component `id` set to `x`.
fn with_parameter(circuit: &Circuit, id: &str, parameter: SensParameter, x: f64) -> Circuit {
    use SensParameter as P;
    let mut c = circuit.clone();
    let element = c
        .components
        .iter_mut()
        .find(|e| e.id() == id)
        .unwrap_or_else(|| panic!("no component {id}"));
    match (element, parameter) {
        (CircuitElement::Resistor(Resistor { resistance, .. }), P::Resistance) => *resistance = x,
        (CircuitElement::Capacitor(Capacitor { capacitance, .. }), P::Capacitance) => {
            *capacitance = x
        }
        (CircuitElement::Inductor(Inductor { inductance, .. }), P::Inductance) => *inductance = x,
        (CircuitElement::VoltageSource(VoltageSource { voltage, .. }), P::Voltage) => *voltage = x,
        (CircuitElement::CurrentSource(CurrentSource { current, .. }), P::Current) => *current = x,
        (CircuitElement::Vcvs(Vcvs { gain, .. }), P::VoltageGain) => *gain = x,
        (CircuitElement::Vccs(Vccs { gm, .. }), P::Transconductance) => *gm = x,
        (CircuitElement::Ccvs(Ccvs { rm, .. }), P::Transresistance) => *rm = x,
        (CircuitElement::Cccs(Cccs { alpha, .. }), P::CurrentGain) => *alpha = x,
        (CircuitElement::Diode(Diode { model, .. }), p) => {
            let m = model.get_or_insert_with(DiodeModel::default);
            match p {
                P::SaturationCurrent => m.is = x,
                P::EmissionCoefficient => m.n = x,
                P::SeriesResistance => m.rs = x,
                P::JunctionCapacitance => m.cjo = x,
                P::TransitTime => m.tt = x,
                _ => panic!("diode {p:?}"),
            }
        }
        (CircuitElement::Bjt(Bjt { bf, .. }), P::ForwardBeta) => *bf = x,
        (CircuitElement::Bjt(Bjt { model, .. }), p) => {
            let m = model.get_or_insert_with(BjtModel::default);
            match p {
                P::SaturationCurrent => m.is = x,
                P::ReverseBeta => m.br = x,
                P::ForwardEmissionCoefficient => m.nf = x,
                P::ReverseEmissionCoefficient => m.nr = x,
                P::EarlyVoltage => m.vaf = x,
                _ => panic!("BJT {p:?}"),
            }
        }
        (CircuitElement::Mosfet(Mosfet { params, .. }), p) => match p {
            P::TransconductanceParameter => params.kp = x,
            P::ThresholdVoltage => params.vto = x,
            P::ChannelLengthModulation => params.lambda = x,
            _ => panic!("MOSFET {p:?}"),
        },
        (e, p) => panic!("{} has no {p:?}", e.id()),
    }
    c
}

/// Central difference of `output` for a relative nudge of one parameter,
/// through full DC solves.
fn finite_difference(
    circuit: &Circuit,
    id: &str,
    parameter: SensParameter,
    value: f64,
    output: impl Fn(&sindr::SimulationResult) -> f64,
) -> f64 {
    let with = |x: f64| output(&solve_circuit(&with_parameter(circuit, id, parameter, x)).unwrap());
    if value == 0.0 {
        // A parameter at zero (a diode's RS) can't go below it: one-sided.
        let h = 1e-6;
        return (with(h) - with(0.0)) / h;
    }
    let h = value.abs() * 1e-5;
    (with(value + h) - with(value - h)) / (2.0 * h)
}

/// Every entry of `.sens V(out)` against its finite difference.
fn assert_matches_finite_differences(name: &str, c: &Circuit, out: &str, tolerance: f64) {
    let r = sens(c, v_out(out));
    assert!(!r.entries.is_empty(), "{name}");
    for e in &r.entries {
        let fd = finite_difference(c, &e.component_id, e.parameter, e.value, |s| {
            s.node_voltages[out]
        });
        // A change below a nanovolt per percent is under the full solves'
        // resolution (IS of a diode in breakdown).
        let negligible = e.value != 0.0 && (e.sensitivity - fd).abs() * e.value.abs() < 1e-7;
        assert!(
            negligible || relative_eq!(e.sensitivity, fd, max_relative = tolerance, epsilon = 1e-9),
            "{name}: {} {:?}: {} vs {fd}",
            e.component_id,
            e.parameter,
            e.sensitivity
        );
    }
}

#[test]
fn controlled_sources_match_finite_differences() {
    let c = circuit(vec![
        vsource("V1", "in", "0", 2.0),
        resistor("R1", "in", "x", 1e3),
        resistor("R2", "x", "0", 2e3),
        CircuitElement::Vcvs(Vcvs {
            id: "E1".into(),
            nodes: ["y".into(), "0".into()],
            control_nodes: ["x".into(), "0".into()],
            gain: 3.0,
        }),
        resistor("R3", "y", "z", 500.0),
        resistor("R4", "z", "0", 1.5e3),
        CircuitElement::Vccs(Vccs {
            id: "G1".into(),
            nodes: ["0".into(), "out".into()],
            control_nodes: ["z".into(), "0".into()],
            gm: 2e-3,
        }),
        resistor("R5", "out", "0", 4e3),
    ]);
    assert_matches_finite_differences("controlled sources", &c, "out", 1e-7);
}

#[test]
fn diode_matches_finite_differences() {
    let c = circuit(vec![
        vsource("V1", "in", "0", 5.0),
        resistor("R1", "in", "out", 1e3),
        CircuitElement::Diode(Diode {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            temperature: 300.15,
            model: None,
        }),
    ]);
    assert_matches_finite_differences("diode", &c, "out", 1e-4);
}

#[test]
fn common_emitter_stage_matches_finite_differences() {
    let c = circuit(vec![
        vsource("VCC", "vcc", "0", 12.0),
        vsource("V1", "in", "0", 2.0),
        resistor("RS", "in", "b", 1e3),
        resistor("RC", "vcc", "c", 2.2e3),
        resistor("RE", "e", "0", 470.0),
        CircuitElement::Bjt(Bjt {
            id: "Q1".into(),
            nodes: ["b".into(), "c".into(), "e".into()],
            kind: BjtKind::Npn,
            bf: 100.0,
            temperature: 300.15,
            parasitic_caps: None,
            model: None,
        }),
    ]);
    assert_matches_finite_differences("CE stage", &c, "c", 1e-4);
}

#[test]
fn sparse_system_uses_the_transpose_too() {
    // A ladder above the dense limit of 100 unknowns: N equal resistors from
    // V down to ground, out k resistors from the bottom. dV(out)/dR_j is
    // V·k/N²·(1/R) with a minus for a resistor above `out`, and V·(N−k)/N²/R
    // for one below it.
    const N: usize = 150;
    const K: usize = 40;
    let (r, v) = (1e3, 6.0);
    let name = |i: usize| {
        if i == 0 {
            "0".to_string()
        } else {
            format!("n{i}")
        }
    };
    let mut components = vec![vsource("V1", &name(N), "0", v)];
    for i in 0..N {
        components.push(resistor(&format!("R{i}"), &name(i + 1), &name(i), r));
    }
    let n = N as f64;
    let k = K as f64;
    let result = sens(&circuit(components), v_out(&name(K)));
    assert_relative_eq!(of(&result, "V1"), k / n, max_relative = 1e-9);
    let below = v * (n - k) / (n * n * r);
    let above = -v * k / (n * n * r);
    assert_relative_eq!(of(&result, "R0"), below, max_relative = 1e-9);
    assert_relative_eq!(of(&result, "R100"), above, max_relative = 1e-9);
}

#[test]
fn sparse_asymmetric_system_matches_finite_differences() {
    // Resistors and independent sources alone give a symmetric MNA matrix,
    // where the transpose changes nothing. A VCVS and a VCCS make it
    // asymmetric, and a ladder above 100 unknowns puts it in sparse storage.
    const N: usize = 150;
    let name = |i: usize| {
        if i == 0 {
            "0".to_string()
        } else {
            format!("n{i}")
        }
    };
    let mut components = vec![vsource("V1", &name(N), "0", 6.0)];
    for i in 0..N {
        components.push(resistor(&format!("R{i}"), &name(i + 1), &name(i), 1e3));
    }
    components.extend([
        CircuitElement::Vcvs(Vcvs {
            id: "E1".into(),
            nodes: ["e".into(), "0".into()],
            control_nodes: [name(90), name(30)],
            gain: 2.5,
        }),
        resistor("RE", "e", "out", 2e3),
        CircuitElement::Vccs(Vccs {
            id: "G1".into(),
            nodes: ["out".into(), "0".into()],
            control_nodes: [name(120), "0".into()],
            gm: 1e-4,
        }),
        resistor("RL", "out", "0", 3e3),
    ]);
    let c = circuit(components);
    assert_matches_finite_differences("sparse ladder", &c, "out", 1e-6);
    assert_matches_finite_differences("sparse ladder", &c, &name(60), 1e-6);
}

#[test]
fn closed_relay_contact_matches_finite_differences() {
    // The coil sits at 12 V against a 5 V pickup, so DC closes the contact
    // and the path V2 → R2 → contact → R1 carries the signal.
    let c = circuit(vec![
        vsource("V1", "coil", "0", 12.0),
        vsource("V2", "src", "0", 10.0),
        resistor("R2", "src", "a", 1e3),
        CircuitElement::Relay(Relay {
            id: "K1".into(),
            nodes: ["coil".into(), "0".into(), "a".into(), "b".into()],
            coil_resistance: 500.0,
            pickup_voltage: 5.0,
            inductance: 0.0,
        }),
        resistor("R1", "b", "0", 1e3),
    ]);
    let r = sens(&c, v_out("b"));
    assert_relative_eq!(of(&r, "V2"), 0.5, max_relative = 1e-4);
    assert_matches_finite_differences("closed relay", &c, "b", 1e-4);
}

#[test]
fn transformer_matches_closed_form() {
    // At DC each winding is a short, so V(m) = V1·R3/(R1 + R3) and the
    // secondary's loop carries no current. Closed form rather than finite
    // differences: `solve_circuit` runs a circuit with a transformer as a
    // transient, whose last step is only close to the DC point.
    let c = circuit(vec![
        vsource("V1", "in", "0", 4.0),
        resistor("R1", "in", "m", 1e3),
        resistor("R3", "m", "a", 3e3),
        CircuitElement::Transformer(Transformer {
            id: "T1".into(),
            nodes: ["a".into(), "0".into(), "s".into(), "0".into()],
            l1: 1e-3,
            l2: 4e-3,
            k: 0.99,
            saturation: None,
        }),
        resistor("RL", "s", "0", 100.0),
    ]);
    let r = sens(&c, v_out("m"));
    assert!(r.get("T1").is_none());
    let (v, r1, r3) = (4.0, 1e3, 3e3);
    let total = r1 + r3;
    assert_relative_eq!(of(&r, "V1"), r3 / total, max_relative = 1e-9);
    assert_relative_eq!(of(&r, "R1"), -v * r3 / (total * total), max_relative = 1e-9);
    assert_relative_eq!(of(&r, "R3"), v * r1 / (total * total), max_relative = 1e-9);
    assert_relative_eq!(of(&r, "RL"), 0.0, epsilon = 1e-12);
}

#[test]
fn reactive_parts_get_no_entry_and_leave_the_rest_right() {
    let mut c = divider();
    c.components[1] = resistor("R1", "in", "mid", R1);
    c.components.push(CircuitElement::Inductor(Inductor {
        id: "L1".into(),
        nodes: ["mid".into(), "out".into()],
        inductance: 1e-3,
        saturation: None,
    }));
    c.components.push(CircuitElement::Capacitor(Capacitor {
        id: "C1".into(),
        nodes: ["out".into(), "0".into()],
        capacitance: 1e-6,
    }));
    let r = sens(&c, v_out("out"));
    assert!(r.get("L1").is_none());
    assert!(r.get("C1").is_none());
    assert_relative_eq!(of(&r, "V1"), R2 / (R1 + R2), max_relative = 1e-8);
    assert_relative_eq!(
        of(&r, "R2"),
        V * R1 / ((R1 + R2) * (R1 + R2)),
        max_relative = 1e-8
    );
}

#[test]
fn bad_output_is_an_error() {
    let c = divider();
    for output in [
        v_out("nowhere"),
        TfOutput::Current {
            source_id: "R1".into(),
        },
    ] {
        let err = solve_sensitivity(&c, &SensConfig::new(output));
        assert!(matches!(err, Err(SimError::InvalidComponent(_))), "{err:?}");
    }
}

#[test]
fn config_constructor() {
    let config = sindr::SensConfig::new(TfOutput::current("V1"));
    assert_eq!(
        config.output,
        TfOutput::Current {
            source_id: "V1".into()
        }
    );
}

fn diode(id: &str, a: &str, k: &str, temperature: f64, model: DiodeModel) -> CircuitElement {
    CircuitElement::Diode(Diode {
        id: id.into(),
        nodes: [a.into(), k.into()],
        temperature,
        model: Some(model),
    })
}

fn bjt(
    id: &str,
    nodes: [&str; 3],
    kind: BjtKind,
    temperature: f64,
    model: BjtModel,
) -> CircuitElement {
    CircuitElement::Bjt(Bjt {
        id: id.into(),
        nodes: nodes.map(Into::into),
        kind,
        bf: 80.0,
        temperature,
        parasitic_caps: None,
        model: Some(model),
    })
}

fn mosfet(id: &str, nodes: [&str; 3], kind: MosfetKind, params: MosfetParams) -> CircuitElement {
    CircuitElement::Mosfet(Mosfet {
        id: id.into(),
        nodes: nodes.map(Into::into),
        kind,
        params,
        parasitic_caps: None,
        body_diode: None,
    })
}

/// The parameters `id` has entries for, in order.
fn parameters(r: &SensResult, id: &str) -> Vec<SensParameter> {
    r.entries
        .iter()
        .filter(|e| e.component_id == id)
        .map(|e| e.parameter)
        .collect()
}

#[test]
fn diode_model_parameters_match_finite_differences() {
    use SensParameter as P;
    // Away from 27 °C, so IS's temperature scaling (through N too) counts.
    let model = DiodeModel {
        is: 5e-15,
        n: 1.6,
        rs: 12.0,
        ..DiodeModel::default()
    };
    let c = circuit(vec![
        vsource("V1", "in", "0", 3.0),
        resistor("R1", "in", "a", 470.0),
        diode("D1", "a", "0", 330.0, model),
    ]);
    let r = sens(&c, v_out("a"));
    assert_eq!(
        parameters(&r, "D1"),
        [
            P::SaturationCurrent,
            P::EmissionCoefficient,
            P::SeriesResistance
        ]
    );
    assert_matches_finite_differences("diode model", &c, "a", 1e-4);
}

#[test]
fn diode_in_breakdown_matches_finite_differences() {
    // The knee is fitted to IBV, so it moves with IS and N.
    let model = DiodeModel {
        is: 1e-14,
        n: 1.2,
        rs: 3.0,
        bv: Some(5.6),
        ibv: 1e-3,
        ..DiodeModel::default()
    };
    let c = circuit(vec![
        vsource("V1", "in", "0", 12.0),
        resistor("R1", "in", "k", 1e3),
        diode("D1", "0", "k", 300.15, model),
    ]);
    let r = sens(&c, v_out("k"));
    assert!(r.get("D1").is_some());
    assert_matches_finite_differences("zener", &c, "k", 1e-4);
}

#[test]
fn bjt_model_parameters_match_finite_differences() {
    use SensParameter as P;
    let model = BjtModel {
        is: 2e-15,
        br: 3.0,
        nf: 1.05,
        nr: 1.2,
        vaf: 70.0,
        ..BjtModel::default()
    };
    for kind in [BjtKind::Npn, BjtKind::Pnp] {
        let s = if kind == BjtKind::Npn { 1.0 } else { -1.0 };
        let c = circuit(vec![
            vsource("VCC", "vcc", "0", s * 12.0),
            vsource("V1", "in", "0", s * 2.0),
            resistor("RS", "in", "b", 10e3),
            resistor("RC", "vcc", "c", 2.2e3),
            resistor("RE", "e", "0", 470.0),
            bjt("Q1", ["b", "c", "e"], kind, 320.0, model),
        ]);
        let r = sens(&c, v_out("c"));
        assert_eq!(
            parameters(&r, "Q1"),
            [
                P::ForwardBeta,
                P::SaturationCurrent,
                P::ReverseBeta,
                P::ForwardEmissionCoefficient,
                P::ReverseEmissionCoefficient,
                P::EarlyVoltage,
            ],
        );
        assert_matches_finite_differences("BJT model", &c, "c", 1e-4);
    }
}

#[test]
fn saturated_bjt_matches_finite_differences() {
    // A small RC and a big base drive: both junctions forward, so BR and NR
    // matter as much as BF and NF.
    let model = BjtModel {
        br: 2.0,
        vaf: 40.0,
        ..BjtModel::default()
    };
    let c = circuit(vec![
        vsource("VCC", "vcc", "0", 5.0),
        vsource("V1", "in", "0", 5.0),
        resistor("RB", "in", "b", 2e3),
        resistor("RC", "vcc", "c", 1e3),
        bjt("Q1", ["b", "c", "0"], BjtKind::Npn, 300.15, model),
    ]);
    let r = sens(&c, v_out("c"));
    let br = r.get_parameter("Q1", SensParameter::ReverseBeta).unwrap();
    assert!(br.sensitivity.abs() > 1e-4, "{br:?}");
    assert_matches_finite_differences("saturated BJT", &c, "c", 1e-4);
}

#[test]
fn mosfet_parameters_match_finite_differences() {
    use SensParameter as P;
    let nmos = MosfetParams {
        kp: 4e-4,
        vto: 1.0,
        lambda: 0.02,
        ..MosfetParams::default_nmos()
    };
    let pmos = MosfetParams {
        kp: 2e-4,
        vto: -0.8,
        lambda: 0.03,
        ..MosfetParams::default_pmos()
    };
    // Common source in saturation, and a PMOS high-side switch in triode.
    let saturated = circuit(vec![
        vsource("VDD", "vdd", "0", 10.0),
        vsource("VG", "g", "0", 2.0),
        resistor("RD", "vdd", "d", 10e3),
        resistor("RS", "s", "0", 1e3),
        mosfet("M1", ["g", "d", "s"], MosfetKind::Nmos, nmos),
    ]);
    let triode = circuit(vec![
        vsource("VDD", "vdd", "0", 5.0),
        vsource("VG", "g", "0", 0.0),
        resistor("RL", "d", "0", 2e3),
        mosfet("M1", ["g", "d", "vdd"], MosfetKind::Pmos, pmos),
    ]);
    for (name, c) in [("saturated NMOS", &saturated), ("triode PMOS", &triode)] {
        let r = sens(c, v_out("d"));
        assert_eq!(
            parameters(&r, "M1"),
            [
                P::TransconductanceParameter,
                P::ThresholdVoltage,
                P::ChannelLengthModulation,
            ],
        );
        assert_matches_finite_differences(name, c, "d", 1e-4);
    }
}

#[test]
fn current_controlled_sources_match_finite_differences() {
    let c = circuit(vec![
        vsource("V1", "in", "0", 2.0),
        resistor("R1", "in", "x", 1e3),
        // VS senses R1's current into R2.
        vsource("VS", "x", "y", 0.0),
        resistor("R2", "y", "0", 3e3),
        CircuitElement::Ccvs(Ccvs {
            id: "H1".into(),
            nodes: ["h".into(), "0".into()],
            control_source: "VS".into(),
            rm: 2e3,
        }),
        resistor("R3", "h", "out", 1e3),
        CircuitElement::Cccs(Cccs {
            id: "F1".into(),
            nodes: ["0".into(), "out".into()],
            control_source: "VS".into(),
            alpha: 3.0,
        }),
        resistor("R4", "out", "0", 2e3),
    ]);
    let r = sens(&c, v_out("out"));
    // I(VS) = 0.5 mA, so dV(out)/drm = I(VS) · R4/(R3 + R4) and
    // dV(out)/dalpha = I(VS) · R3‖R4.
    let i = 2.0 / 4e3;
    assert_relative_eq!(of(&r, "H1"), i * 2e3 / 3e3, max_relative = 1e-9);
    assert_relative_eq!(of(&r, "F1"), i * 2e3 / 3.0, max_relative = 1e-9);
    assert_eq!(
        r.get("H1").unwrap().parameter,
        SensParameter::Transresistance
    );
    assert_eq!(r.get("F1").unwrap().parameter, SensParameter::CurrentGain);
    assert_matches_finite_differences("CCVS and CCCS", &c, "out", 1e-7);
}

/// An RC low-pass, H = 1/(1 + jωRC): the AC sensitivity helpers pick out
/// dH/dR = −jωC/(1 + jωRC)² and dH/dC = −jωR/(1 + jωRC)² at each frequency.
#[test]
fn ac_sensitivity_lookups_match_the_rc_closed_form() {
    use num_complex::Complex64;
    use sindr::{solve_ac_sensitivity, AcConfig, AcSensConfig, SensParameter, TfOutput};

    let (r, c) = (1e3, 1e-7);
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "out".into()],
                resistance: r,
            }),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: c,
            }),
        ],
    );
    let config = AcSensConfig::new(TfOutput::voltage("out"), AcConfig::new("V1", 100.0, 1e5, 7));
    let sens = solve_ac_sensitivity(&circuit, &config).unwrap();

    let dr = sens.curve("R1", SensParameter::Resistance);
    let dc = sens.curve("C1", SensParameter::Capacitance);
    assert_eq!((dr.len(), dc.len()), (7, 7));
    for (point, ((f, s_r), (_, s_c))) in sens.points.iter().zip(dr.iter().zip(&dc)) {
        let jw = Complex64::new(0.0, 2.0 * std::f64::consts::PI * f);
        let den = (1.0 + jw * r * c).powi(2);
        let (want_r, want_c) = (-jw * c / den, -jw * r / den);
        assert!(
            (s_r - want_r).norm() <= 1e-9 * want_r.norm(),
            "{f} Hz: {s_r} vs {want_r}"
        );
        assert!(
            (s_c - want_c).norm() <= 1e-9 * want_c.norm(),
            "{f} Hz: {s_c} vs {want_c}"
        );
        let entry = point
            .get_parameter("R1", SensParameter::Resistance)
            .unwrap();
        assert_eq!((entry.value, entry.sensitivity), (r, *s_r));
    }
    assert!(sens.points[0]
        .get_parameter("R1", SensParameter::Capacitance)
        .is_none());
    assert!(sens.curve("nope", SensParameter::Resistance).is_empty());
}
