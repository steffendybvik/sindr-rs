//! A circuit with no stored energy must give the same answer in transient
//! as at its DC operating point, for every element, through every result
//! the two paths build separately: node voltages, each component's voltage,
//! current and power, and the BJT, MOSFET, op-amp and relay results.
//!
//! The transient extracts its per-element results in its own code
//! (`extract_timestep_results`), so this pins it to the DC path one device
//! at a time.
//!
//! Each circuit also goes through JSON and back unchanged, which covers
//! serde for these elements (`serde_round_trip.rs` has the rest).

use sindr::{
    solve_circuit, solve_transient, BjtKind, Circuit, CircuitElement, JfetKind, MosfetKind,
    SimulationResult, TransientConfig,
};
use sindr_devices::igbt::IgbtParams;
use sindr_devices::mosfet::MosfetParams;

fn r(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn v(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

/// A 5 V supply on `vcc` and 2 V on `in`, each with a load so neither node
/// floats, plus `parts`.
fn bench(parts: Vec<CircuitElement>) -> Circuit {
    let mut components = vec![
        v("Vcc", "vcc", "0", 5.0),
        r("Rvcc", "vcc", "0", 10e3),
        v("Vin", "in", "0", 2.0),
        r("Rin", "in", "0", 10e3),
    ];
    components.extend(parts);
    Circuit::new("0", components)
}

fn close(what: &str, dc: f64, tr: f64) {
    let tol = 1e-6 * dc.abs().max(1.0);
    assert!((dc - tr).abs() <= tol, "{what}: DC {dc} but transient {tr}");
}

fn assert_same(name: &str, dc: &SimulationResult, tr: &SimulationResult) {
    let last = tr
        .transient
        .as_ref()
        .and_then(|t| t.timesteps.last())
        .unwrap_or_else(|| panic!("{name}: no transient steps"));
    for (node, &v_dc) in &dc.node_voltages {
        close(
            &format!("{name}: v({node})"),
            v_dc,
            last.node_voltages[node],
        );
    }
    for c_dc in &dc.component_results {
        let c_tr = last
            .component_results
            .iter()
            .find(|c| c.id == c_dc.id)
            .unwrap_or_else(|| panic!("{name}: `{}` missing from the transient", c_dc.id));
        let what = |field: &str| format!("{name}: {}.{field}", c_dc.id);
        close(
            &what("voltage_across"),
            c_dc.voltage_across,
            c_tr.voltage_across,
        );
        close(
            &what("current_through"),
            c_dc.current_through,
            c_tr.current_through,
        );
        close(&what("power"), c_dc.power, c_tr.power);
    }
    assert_eq!(
        dc.component_results.len(),
        last.component_results.len(),
        "{name}: component count"
    );
    assert_eq!(dc.bjt_results.len(), tr.bjt_results.len(), "{name}: BJTs");
    for (a, b) in dc.bjt_results.iter().zip(&tr.bjt_results) {
        let what = |f: &str| format!("{name}: {}.{f}", a.id);
        close(&what("vbe"), a.vbe, b.vbe);
        close(&what("vce"), a.vce, b.vce);
        close(&what("ic"), a.ic, b.ic);
        close(&what("ib"), a.ib, b.ib);
        close(&what("power"), a.power, b.power);
        assert_eq!(a.region, b.region, "{}", what("region"));
    }
    assert_eq!(
        dc.mosfet_results.len(),
        tr.mosfet_results.len(),
        "{name}: MOSFETs"
    );
    for (a, b) in dc.mosfet_results.iter().zip(&tr.mosfet_results) {
        let what = |f: &str| format!("{name}: {}.{f}", a.id);
        close(&what("vgs"), a.vgs, b.vgs);
        close(&what("vds"), a.vds, b.vds);
        close(&what("id_current"), a.id_current, b.id_current);
        close(&what("power"), a.power, b.power);
        assert_eq!(a.region, b.region, "{}", what("region"));
    }
    assert_eq!(
        dc.op_amp_results.len(),
        tr.op_amp_results.len(),
        "{name}: op-amps"
    );
    for (a, b) in dc.op_amp_results.iter().zip(&tr.op_amp_results) {
        close(&format!("{name}: {}.v_out", a.id), a.v_out, b.v_out);
    }
    assert_eq!(
        dc.relay_results.len(),
        tr.relay_results.len(),
        "{name}: relays"
    );
    for (a, b) in dc.relay_results.iter().zip(&tr.relay_results) {
        close(
            &format!("{name}: {}.coil_voltage", a.id),
            a.coil_voltage,
            b.coil_voltage,
        );
        assert_eq!(
            a.contact_closed, b.contact_closed,
            "{name}: {}.contact_closed",
            a.id
        );
    }
}

fn check(name: &str, parts: Vec<CircuitElement>) {
    let circuit = bench(parts);
    #[cfg(feature = "serde")]
    {
        let json = serde_json::to_string(&circuit).unwrap();
        let back: Circuit = serde_json::from_str(&json).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(back, circuit, "{name}: changed in a JSON round trip");
    }
    let dc = solve_circuit(&circuit).unwrap_or_else(|e| panic!("{name}: DC: {e}"));
    let tr = solve_transient(&circuit, &TransientConfig::new(10e-6, 1e-6))
        .unwrap_or_else(|e| panic!("{name}: transient: {e}"));
    assert_same(name, &dc, &tr);
}

#[test]
fn controlled_sources() {
    check(
        "vcvs",
        vec![
            CircuitElement::Vcvs {
                id: "E1".into(),
                nodes: ["o".into(), "0".into()],
                control_nodes: ["in".into(), "0".into()],
                gain: 2.0,
            },
            r("RL", "o", "0", 1e3),
        ],
    );
    check(
        "vccs",
        vec![
            CircuitElement::Vccs {
                id: "G1".into(),
                nodes: ["o".into(), "0".into()],
                control_nodes: ["in".into(), "0".into()],
                gm: 1e-3,
            },
            r("RL", "o", "0", 1e3),
        ],
    );
    check(
        "ccvs",
        vec![
            CircuitElement::Ccvs {
                id: "H1".into(),
                nodes: ["o".into(), "0".into()],
                control_source: "Vin".into(),
                rm: 100.0,
            },
            r("RL", "o", "0", 1e3),
        ],
    );
    check(
        "cccs",
        vec![
            CircuitElement::Cccs {
                id: "F1".into(),
                nodes: ["o".into(), "0".into()],
                control_source: "Vin".into(),
                alpha: 3.0,
            },
            r("RL", "o", "0", 1e3),
        ],
    );
}

#[test]
fn switches_and_resistive_parts() {
    for closed in [true, false] {
        check(
            &format!("switch closed={closed}"),
            vec![
                r("R1", "vcc", "o", 1e3),
                CircuitElement::Switch {
                    id: "S1".into(),
                    nodes: ["o".into(), "0".into()],
                    closed,
                },
            ],
        );
        check(
            &format!("pushbutton closed={closed}"),
            vec![
                r("R1", "vcc", "o", 1e3),
                CircuitElement::Pushbutton {
                    id: "B1".into(),
                    nodes: ["o".into(), "0".into()],
                    closed,
                },
            ],
        );
        check(
            &format!("fuse blown={closed}"),
            vec![
                r("R1", "vcc", "o", 1e3),
                CircuitElement::Fuse {
                    id: "F1".into(),
                    nodes: ["o".into(), "0".into()],
                    rating: 1.0,
                    blown: closed,
                },
            ],
        );
    }
    check(
        "potentiometer",
        vec![
            CircuitElement::Potentiometer {
                id: "P1".into(),
                nodes: ["vcc".into(), "w".into(), "0".into()],
                resistance: 10e3,
                position: 0.3,
            },
            r("RL", "w", "0", 4.7e3),
        ],
    );
    check(
        "photoresistor",
        vec![
            r("R1", "vcc", "o", 10e3),
            CircuitElement::Photoresistor {
                id: "LDR1".into(),
                nodes: ["o".into(), "0".into()],
                light_level: 0.4,
            },
        ],
    );
    check(
        "thermistor",
        vec![
            r("R1", "vcc", "o", 10e3),
            CircuitElement::Thermistor {
                id: "TH1".into(),
                nodes: ["o".into(), "0".into()],
                temperature: 320.0,
            },
        ],
    );
}

#[test]
fn op_amp_and_comparator() {
    check(
        "op-amp follower",
        vec![
            CircuitElement::OpAmp {
                id: "U1".into(),
                nodes: ["in".into(), "o".into(), "o".into()],
                v_pos: 15.0,
                v_neg: -15.0,
                model: None,
            },
            r("RL", "o", "0", 1e3),
        ],
    );
    check(
        "comparator",
        vec![
            v("Vref", "ref", "0", 2.0 - 1e-5),
            CircuitElement::Comparator {
                id: "U2".into(),
                nodes: ["in".into(), "ref".into(), "o".into()],
                v_pos: 5.0,
                v_neg: 0.0,
            },
            r("RL", "o", "0", 1e3),
        ],
    );
}

#[test]
fn junctions() {
    let to_ground = |element: CircuitElement| vec![r("R1", "vcc", "a", 1e3), element];
    check(
        "diode",
        to_ground(CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["a".into(), "0".into()],
            temperature: 300.15,
            model: None,
        }),
    );
    check(
        "zener in breakdown",
        vec![
            v("Vhi", "hi", "0", 12.0),
            r("R1", "hi", "k", 1e3),
            CircuitElement::ZenerDiode {
                id: "D1".into(),
                nodes: ["0".into(), "k".into()],
                vz: 5.1,
                temperature: 300.15,
            },
        ],
    );
    check(
        "schottky",
        to_ground(CircuitElement::SchottkyDiode {
            id: "D1".into(),
            nodes: ["a".into(), "0".into()],
            temperature: 300.15,
        }),
    );
    check(
        "led",
        to_ground(CircuitElement::Led {
            id: "D1".into(),
            nodes: ["a".into(), "0".into()],
            color: "red".into(),
            temperature: 300.15,
        }),
    );
    check(
        "photodiode",
        vec![
            r("R1", "vcc", "k", 10e3),
            CircuitElement::Photodiode {
                id: "D1".into(),
                nodes: ["0".into(), "k".into()],
                irradiance: 1e-4,
                temperature: 300.15,
            },
        ],
    );
}

#[test]
fn transistors() {
    for kind in [BjtKind::Npn, BjtKind::Pnp] {
        let (supply, sense) = match kind {
            BjtKind::Npn => ("vcc", "0"),
            BjtKind::Pnp => ("0", "vcc"),
        };
        check(
            &format!("bjt {kind:?}"),
            vec![
                r("RB", "in", "b", 100e3),
                r("RC", supply, "c", 1e3),
                r("RE", "e", sense, 100.0),
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "e".into()],
                    kind,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: None,
                },
            ],
        );
    }
    for (kind, params) in [
        (MosfetKind::Nmos, MosfetParams::default_nmos()),
        (MosfetKind::Pmos, MosfetParams::default_pmos()),
    ] {
        let (gate, load, source) = match kind {
            MosfetKind::Nmos => ("in", "vcc", "0"),
            MosfetKind::Pmos => ("0", "0", "vcc"),
        };
        check(
            &format!("mosfet {kind:?}"),
            vec![
                r("RG", gate, "g", 1e3),
                r("RGL", "g", "0", 1e6),
                r("RD", load, "d", 1e3),
                CircuitElement::Mosfet {
                    id: "M1".into(),
                    nodes: ["g".into(), "d".into(), source.into()],
                    kind,
                    params,
                    parasitic_caps: None,
                    body_diode: None,
                },
            ],
        );
    }
    for kind in [JfetKind::NChannel, JfetKind::PChannel] {
        let (drain_supply, vp) = match kind {
            JfetKind::NChannel => ("vcc", -2.0),
            JfetKind::PChannel => ("0", 2.0),
        };
        let source_return = if drain_supply == "vcc" { "0" } else { "vcc" };
        check(
            &format!("jfet {kind:?}"),
            vec![
                r("RD", drain_supply, "d", 1e3),
                r("RS", "s", source_return, 100.0),
                r("RG", "g", source_return, 1e6),
                CircuitElement::Jfet {
                    id: "J1".into(),
                    nodes: ["g".into(), "d".into(), "s".into()],
                    kind,
                    idss: 10e-3,
                    vp,
                    parasitic_caps: None,
                },
            ],
        );
    }
    check(
        "igbt",
        vec![
            v("Vg", "g", "0", 15.0),
            r("RG", "g", "0", 10e3),
            r("RC", "vcc", "c", 100.0),
            CircuitElement::Igbt {
                id: "Q1".into(),
                nodes: ["g".into(), "c".into(), "e".into()],
                params: IgbtParams::default(),
            },
            r("RE", "e", "0", 1.0),
        ],
    );
}

#[test]
fn relay_and_regulator() {
    check(
        "relay",
        vec![
            CircuitElement::Relay {
                id: "K1".into(),
                nodes: ["vcc".into(), "0".into(), "in".into(), "o".into()],
                coil_resistance: 500.0,
                pickup_voltage: 4.0,
                inductance: 0.0,
            },
            r("RL", "o", "0", 1e3),
        ],
    );
    check(
        "ideal regulator",
        vec![
            CircuitElement::VoltageRegulator {
                id: "U1".into(),
                nodes: ["vcc".into(), "o".into(), "0".into()],
                voltage: 3.3,
                model: None,
            },
            r("RL", "o", "0", 1e3),
        ],
    );
}
