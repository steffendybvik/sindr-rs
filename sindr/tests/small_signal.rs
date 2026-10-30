//! Small-signal consistency: AC analysis linearises each nonlinear device at
//! its DC operating point, so in a circuit without reactive parts the AC
//! transfer from a source to every node must equal the derivative of the DC
//! solution with respect to that source's voltage.
//!
//! The derivative is a central finite difference of two full DC solves, so it
//! uses only the DC model; nothing here repeats a device formula. A device
//! whose AC stamp drifts from its DC model (a missing or wrong-signed
//! conductance, a different temperature or parameter set) fails here.

use num_complex::Complex64;
use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{solve_circuit, BjtKind, Circuit, CircuitElement, IgbtParams, JfetKind, MosfetKind};
use sindr_devices::mosfet::MosfetParams;

fn vsource(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

/// `circuit` with source `V1` set to `volts`.
fn with_input(circuit: &Circuit, volts: f64) -> Circuit {
    let mut c = circuit.clone();
    for e in &mut c.components {
        if let CircuitElement::VoltageSource { id, voltage, .. } = e {
            if id == "V1" {
                *voltage = volts;
            }
        }
    }
    c
}

/// For each bias of `V1`, the AC response of every node (at 1 kHz; nothing is
/// frequency-dependent) equals the central difference of the DC node voltages.
fn assert_ac_matches_dc(name: &str, circuit: &Circuit, biases: &[f64]) {
    const H: f64 = 1e-5;
    for &v0 in biases {
        let at = with_input(circuit, v0);
        let config = AcConfig {
            f_start: 1e3,
            f_stop: 1e3,
            num_points: 1,
            spacing: FrequencySpacing::Linear,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
        };
        let ac = solve_ac(&at, &config).unwrap_or_else(|e| panic!("{name} at {v0} V: {e}"));
        let ac = &ac.points[0].node_voltages;
        let up = solve_circuit(&with_input(circuit, v0 + H))
            .unwrap()
            .node_voltages;
        let down = solve_circuit(&with_input(circuit, v0 - H))
            .unwrap()
            .node_voltages;

        for (node, gain) in ac {
            let fd = (up[node] - down[node]) / (2.0 * H);
            let err = (*gain - Complex64::new(fd, 0.0)).norm();
            assert!(
                err <= 1e-4 * fd.abs() + 1e-6,
                "{name}, V1 = {v0} V, node {node}: AC {gain}, d(DC)/dV1 {fd}"
            );
        }
    }
}

/// `V1 → 1 kΩ → n2 → device → 0`, the device's anode at `n2`.
fn two_terminal(device: CircuitElement) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "in", "0", 0.0),
            resistor("R1", "in", "n2", 1e3),
            device,
        ],
    }
}

const T_NOM: f64 = 300.15;

#[test]
fn diode_ac_matches_dc() {
    for t in [T_NOM, 350.0] {
        let circuit = two_terminal(CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["n2".into(), "0".into()],
            temperature: t,
        });
        assert_ac_matches_dc(&format!("diode at {t} K"), &circuit, &[-2.0, 0.3, 0.7, 5.0]);
    }
}

#[test]
fn led_ac_matches_dc() {
    let circuit = two_terminal(CircuitElement::Led {
        id: "D1".into(),
        nodes: ["n2".into(), "0".into()],
        color: "green".into(),
        temperature: T_NOM,
    });
    assert_ac_matches_dc("LED", &circuit, &[1.0, 2.2, 5.0]);
}

#[test]
fn schottky_ac_matches_dc() {
    let circuit = two_terminal(CircuitElement::SchottkyDiode {
        id: "D1".into(),
        nodes: ["n2".into(), "0".into()],
        temperature: T_NOM,
    });
    assert_ac_matches_dc("Schottky", &circuit, &[-2.0, 0.2, 5.0]);
}

/// Forward, below breakdown, and in breakdown (−8 V across a 5.1 V Zener).
#[test]
fn zener_ac_matches_dc() {
    let circuit = two_terminal(CircuitElement::ZenerDiode {
        id: "D1".into(),
        nodes: ["n2".into(), "0".into()],
        vz: 5.1,
        temperature: T_NOM,
    });
    assert_ac_matches_dc("Zener", &circuit, &[-8.0, -3.0, 0.7, 5.0]);
}

#[test]
fn photodiode_ac_matches_dc() {
    for irradiance in [0.0, 1e-3] {
        let circuit = two_terminal(CircuitElement::Photodiode {
            id: "D1".into(),
            nodes: ["n2".into(), "0".into()],
            irradiance,
            temperature: T_NOM,
        });
        assert_ac_matches_dc(
            &format!("photodiode at {irradiance} W"),
            &circuit,
            &[-2.0, 0.5, 5.0],
        );
    }
}

/// Emitter-degenerated stage driven at the base through RB. For the PNP
/// every voltage is mirrored: the supply is −5 V.
fn bjt_stage(kind: BjtKind, temperature: f64) -> Circuit {
    let vcc = match kind {
        BjtKind::Npn => 5.0,
        BjtKind::Pnp => -5.0,
    };
    Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "in", "0", 0.0),
            vsource("VCC", "vcc", "0", vcc),
            resistor("RB", "in", "b", 10e3),
            resistor("RC", "vcc", "c", 1e3),
            resistor("RE", "e", "0", 100.0),
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "e".into()],
                kind,
                bf: 100.0,
                temperature,
                parasitic_caps: None,
            },
        ],
    }
}

/// Cutoff, forward-active and saturation.
#[test]
fn npn_ac_matches_dc() {
    for t in [T_NOM, 350.0] {
        assert_ac_matches_dc(
            &format!("NPN at {t} K"),
            &bjt_stage(BjtKind::Npn, t),
            &[0.3, 0.9, 1.5, 6.0],
        );
    }
}

#[test]
fn pnp_ac_matches_dc() {
    assert_ac_matches_dc(
        "PNP",
        &bjt_stage(BjtKind::Pnp, T_NOM),
        &[-0.3, -0.9, -1.5, -6.0],
    );
}

/// Gate driven directly, drain load to the supply, source resistor to ground
/// so the source sits off ground. Every voltage is mirrored for P-channel.
fn fet_stage(device: CircuitElement, sign: f64, vdd: f64) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "g", "0", 0.0),
            vsource("VDD", "vdd", "0", sign * vdd),
            resistor("RD", "vdd", "d", 10e3),
            resistor("RS", "s", "0", 1e3),
            device,
        ],
    }
}

fn mosfet(kind: MosfetKind, params: MosfetParams) -> CircuitElement {
    CircuitElement::Mosfet {
        id: "M1".into(),
        nodes: ["g".into(), "d".into(), "s".into()],
        kind,
        params,
        parasitic_caps: None,
    }
}

/// Cutoff, saturation and triode.
#[test]
fn nmos_ac_matches_dc() {
    let circuit = fet_stage(
        mosfet(MosfetKind::Nmos, MosfetParams::default_nmos()),
        1.0,
        10.0,
    );
    assert_ac_matches_dc("NMOS", &circuit, &[0.3, 2.0, 4.0, 10.0]);
}

#[test]
fn pmos_ac_matches_dc() {
    let circuit = fet_stage(
        mosfet(MosfetKind::Pmos, MosfetParams::default_pmos()),
        -1.0,
        10.0,
    );
    assert_ac_matches_dc("PMOS", &circuit, &[-0.3, -2.0, -4.0, -10.0]);
}

fn jfet(kind: JfetKind) -> CircuitElement {
    CircuitElement::Jfet {
        id: "J1".into(),
        nodes: ["g".into(), "d".into(), "s".into()],
        kind,
        idss: 10e-3,
        vp: -2.0,
    }
}

/// From pinch-off up to V_GS near zero, where the drain load pushes the
/// device into its ohmic region.
#[test]
fn n_jfet_ac_matches_dc() {
    let circuit = fet_stage(jfet(JfetKind::NChannel), 1.0, 12.0);
    assert_ac_matches_dc("N-JFET", &circuit, &[-1.5, 0.0, 1.0, 3.0]);
}

#[test]
fn p_jfet_ac_matches_dc() {
    let circuit = fet_stage(jfet(JfetKind::PChannel), -1.0, 12.0);
    assert_ac_matches_dc("P-JFET", &circuit, &[1.5, 0.0, -1.0, -3.0]);
}

#[test]
fn igbt_ac_matches_dc() {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("V1", "g", "0", 0.0),
            vsource("VCC", "vcc", "0", 20.0),
            resistor("RC", "vcc", "c", 100.0),
            resistor("RE", "e", "0", 10.0),
            CircuitElement::Igbt {
                id: "Q1".into(),
                nodes: ["g".into(), "c".into(), "e".into()],
                params: IgbtParams::default(),
            },
        ],
    };
    assert_ac_matches_dc("IGBT", &circuit, &[3.0, 5.5, 6.0, 15.0]);
}
