//! Newton–Raphson consistency checks for every nonlinear device stamp.
//!
//! For a stamp evaluated at operating point `v`, Newton's method solves
//! `A(v)·x = b(v)`. That is only Newton's method if `A(v)` is the Jacobian of
//! the residual `F(x) = A(x)·x − b(x)` at `x = v`. These tests compute that
//! Jacobian by central finite differences and compare it entry by entry with
//! the stamped matrix, over a grid of bias points per device. They also check
//! KCL: with no terminal on ground, the device's terminal currents sum to zero.
//!
//! The checks don't use any device formula, so a wrong derivative, a missing
//! term or a clamp that changes the slope but not the current all show up.

use nalgebra::{DMatrix, DVector};
use sindr::circuit::{Circuit, CircuitElement};
use sindr::mna::MnaSystem;
use sindr::node_map::NodeMap;
use sindr::stamp::stamp_circuit;
use sindr_devices::bjt::BjtKind;
use sindr_devices::igbt::IgbtParams;
use sindr_devices::jfet::JfetKind;
use sindr_devices::mosfet::{MosfetKind, MosfetParams};

const REL_TOL: f64 = 1e-4;
const ABS_TOL: f64 = 1e-9;

fn circuit_of(element: CircuitElement) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![element],
    }
}

fn stamp_at(
    circuit: &Circuit,
    node_map: &NodeMap,
    v: &DVector<f64>,
) -> (DMatrix<f64>, DVector<f64>) {
    let mut system = MnaSystem::new(node_map.num_nodes(), 0);
    stamp_circuit(circuit, &mut system, node_map, Some(v)).unwrap();
    (system.a, system.b)
}

/// Residual `F(x) = A(x)·x − b(x)`: the device's current out of each node.
fn residual(circuit: &Circuit, node_map: &NodeMap, x: &DVector<f64>) -> DVector<f64> {
    let (a, b) = stamp_at(circuit, node_map, x);
    a * x - b
}

/// Node voltage vector from `(node, volts)` pairs.
fn voltages(node_map: &NodeMap, bias: &[(&str, f64)]) -> DVector<f64> {
    let mut v = DVector::zeros(node_map.num_nodes());
    for (node, volts) in bias {
        v[node_map.index(node).unwrap()] = *volts;
    }
    v
}

/// Compare the stamped matrix with the finite-difference Jacobian of the
/// residual, and check KCL. Returns one line per mismatch.
fn check(circuit: &Circuit, bias: &[(&str, f64)]) -> Vec<String> {
    let node_map = NodeMap::from_circuit(circuit);
    let v = voltages(&node_map, bias);
    let n = v.len();
    let (a, _) = stamp_at(circuit, &node_map, &v);
    let mut failures = Vec::new();

    for j in 0..n {
        let h = 1e-7 * v[j].abs().max(1.0);
        let mut plus = v.clone();
        let mut minus = v.clone();
        plus[j] += h;
        minus[j] -= h;
        let column = (residual(circuit, &node_map, &plus) - residual(circuit, &node_map, &minus))
            / (2.0 * h);
        for i in 0..n {
            let (stamped, fd) = (a[(i, j)], column[i]);
            if (stamped - fd).abs() > REL_TOL * stamped.abs().max(fd.abs()) + ABS_TOL {
                failures.push(format!(
                    "bias {bias:?}: dF[{}]/dV[{}] stamped {stamped:.6e}, finite difference {fd:.6e}",
                    node_map.node_name(i).unwrap(),
                    node_map.node_name(j).unwrap(),
                ));
            }
        }
    }

    let f = residual(circuit, &node_map, &v);
    let scale = f.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    if f.sum().abs() > 1e-9 * scale + 1e-15 {
        failures.push(format!(
            "bias {bias:?}: terminal currents sum to {:.6e}, not 0",
            f.sum()
        ));
    }
    failures
}

fn assert_consistent(circuit: Circuit, biases: &[&[(&str, f64)]]) {
    let failures: Vec<String> = biases
        .iter()
        .flat_map(|bias| check(&circuit, bias))
        .collect();
    assert!(
        failures.is_empty(),
        "{} mismatches:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// Two-terminal biases from anode `a` to cathode `k`, both off ground.
fn two_terminal(volts: &[f64]) -> Vec<Vec<(&'static str, f64)>> {
    volts
        .iter()
        .map(|&v| vec![("a", 1.0 + v), ("k", 1.0)])
        .collect()
}

fn as_slices<'a>(biases: &'a [Vec<(&'static str, f64)>]) -> Vec<&'a [(&'static str, f64)]> {
    biases.iter().map(Vec::as_slice).collect()
}

const DIODE_BIASES: &[f64] = &[-2.0, -0.3, 0.0, 0.3, 0.6, 0.7, 0.8];

#[test]
fn diode_stamp_is_newton_consistent() {
    for temperature in [250.0, 300.15, 400.0] {
        let circuit = circuit_of(CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["a".into(), "k".into()],
            temperature,
        });
        assert_consistent(circuit, &as_slices(&two_terminal(DIODE_BIASES)));
    }
}

#[test]
fn led_stamp_is_newton_consistent() {
    for color in ["red", "green", "blue"] {
        let circuit = circuit_of(CircuitElement::Led {
            id: "D1".into(),
            nodes: ["a".into(), "k".into()],
            color: color.into(),
            temperature: 300.15,
        });
        assert_consistent(
            circuit,
            &as_slices(&two_terminal(&[-2.0, 0.0, 1.0, 1.8, 2.5, 3.2])),
        );
    }
}

#[test]
fn schottky_stamp_is_newton_consistent() {
    let circuit = circuit_of(CircuitElement::SchottkyDiode {
        id: "D1".into(),
        nodes: ["a".into(), "k".into()],
        temperature: 300.15,
    });
    assert_consistent(
        circuit,
        &as_slices(&two_terminal(&[-2.0, 0.0, 0.2, 0.35, 0.5])),
    );
}

#[test]
fn photodiode_stamp_is_newton_consistent() {
    let circuit = circuit_of(CircuitElement::Photodiode {
        id: "D1".into(),
        nodes: ["a".into(), "k".into()],
        irradiance: 1e-3,
        temperature: 300.15,
    });
    assert_consistent(circuit, &as_slices(&two_terminal(DIODE_BIASES)));
}

#[test]
fn zener_stamp_is_newton_consistent() {
    for temperature in [250.0, 300.15, 400.0] {
        let circuit = circuit_of(CircuitElement::ZenerDiode {
            id: "D1".into(),
            nodes: ["a".into(), "k".into()],
            vz: 5.1,
            temperature,
        });
        // Breakdown, reverse, zero, forward, and past the exponential clamp.
        assert_consistent(
            circuit,
            &as_slices(&two_terminal(&[-6.0, -5.5, -3.0, 0.0, 0.6, 0.75, 1.2])),
        );
    }
}

fn bjt(kind: BjtKind, temperature: f64) -> Circuit {
    circuit_of(CircuitElement::Bjt {
        id: "Q1".into(),
        nodes: ["b".into(), "c".into(), "e".into()],
        kind,
        bf: 100.0,
        temperature,
        parasitic_caps: None,
    })
}

/// NPN biases `(vb, vc, ve)` with every terminal off ground; PNP uses them negated.
const BJT_BIASES: &[[f64; 3]] = &[
    [1.0, 6.0, 1.0],  // cutoff
    [1.65, 6.0, 1.0], // forward active
    [1.72, 6.0, 1.0], // forward active, high current
    [1.75, 1.2, 1.0], // saturation
    [1.65, 1.0, 6.0], // reverse active
];

fn bjt_biases(sign: f64) -> Vec<Vec<(&'static str, f64)>> {
    BJT_BIASES
        .iter()
        .map(|[b, c, e]| vec![("b", sign * b), ("c", sign * c), ("e", sign * e)])
        .collect()
}

#[test]
fn npn_stamp_is_newton_consistent() {
    for temperature in [250.0, 300.15, 400.0] {
        assert_consistent(bjt(BjtKind::Npn, temperature), &as_slices(&bjt_biases(1.0)));
    }
}

#[test]
fn pnp_stamp_is_newton_consistent() {
    for temperature in [250.0, 300.15, 400.0] {
        assert_consistent(
            bjt(BjtKind::Pnp, temperature),
            &as_slices(&bjt_biases(-1.0)),
        );
    }
}

#[test]
fn bjt_stamp_past_conductance_cap_is_newton_consistent() {
    let high_current: &[(&str, f64)] = &[("b", 1.8), ("c", 6.0), ("e", 1.0)];
    assert_consistent(bjt(BjtKind::Npn, 300.15), &[high_current]);
}

fn mosfet(kind: MosfetKind, params: MosfetParams) -> Circuit {
    circuit_of(CircuitElement::Mosfet {
        id: "M1".into(),
        nodes: ["g".into(), "d".into(), "s".into()],
        kind,
        params,
        parasitic_caps: None,
    })
}

/// NMOS biases `(vg, vd, vs)` relative to a 1 V offset; PMOS uses them negated.
const FET_BIASES: &[[f64; 3]] = &[
    [0.0, 5.0, 0.0],  // cutoff
    [3.0, 0.5, 0.0],  // triode
    [2.0, 5.0, 0.0],  // saturation
    [3.0, -0.5, 0.0], // drain below source
];

fn fet_biases(sign: f64) -> Vec<Vec<(&'static str, f64)>> {
    FET_BIASES
        .iter()
        .map(|[g, d, s]| {
            vec![
                ("g", sign * g + 1.0),
                ("d", sign * d + 1.0),
                ("s", sign * s + 1.0),
            ]
        })
        .collect()
}

#[test]
fn nmos_stamp_is_newton_consistent() {
    assert_consistent(
        mosfet(MosfetKind::Nmos, MosfetParams::default_nmos()),
        &as_slices(&fet_biases(1.0)),
    );
}

#[test]
fn pmos_stamp_is_newton_consistent() {
    assert_consistent(
        mosfet(MosfetKind::Pmos, MosfetParams::default_pmos()),
        &as_slices(&fet_biases(-1.0)),
    );
}

fn jfet(kind: JfetKind) -> Circuit {
    circuit_of(CircuitElement::Jfet {
        id: "J1".into(),
        nodes: ["g".into(), "d".into(), "s".into()],
        kind,
        idss: 10e-3,
        vp: -2.0,
    })
}

/// N-channel biases `(vg, vd, vs)` relative to a 1 V offset; P-channel uses them negated.
const JFET_BIASES: &[[f64; 3]] = &[
    [-3.0, 5.0, 0.0], // cutoff
    [-0.5, 5.0, 0.0], // saturation
    [-0.5, 0.5, 0.0], // triode
];

fn jfet_biases(sign: f64) -> Vec<Vec<(&'static str, f64)>> {
    JFET_BIASES
        .iter()
        .map(|[g, d, s]| {
            vec![
                ("g", sign * g + 1.0),
                ("d", sign * d + 1.0),
                ("s", sign * s + 1.0),
            ]
        })
        .collect()
}

#[test]
fn n_jfet_stamp_is_newton_consistent() {
    assert_consistent(jfet(JfetKind::NChannel), &as_slices(&jfet_biases(1.0)));
}

#[test]
fn p_jfet_stamp_is_newton_consistent() {
    assert_consistent(jfet(JfetKind::PChannel), &as_slices(&jfet_biases(-1.0)));
}

#[test]
fn igbt_stamp_is_newton_consistent() {
    let circuit = circuit_of(CircuitElement::Igbt {
        id: "Z1".into(),
        nodes: ["g".into(), "c".into(), "e".into()],
        params: IgbtParams::default(),
    });
    let biases: Vec<Vec<(&str, f64)>> = [[3.0, 10.0, 0.0], [8.0, 10.0, 0.0], [8.0, 1.0, 0.0]]
        .iter()
        .map(|[g, c, e]| vec![("g", g + 1.0), ("c", c + 1.0), ("e", e + 1.0)])
        .collect();
    assert_consistent(circuit, &as_slices(&biases));
}
