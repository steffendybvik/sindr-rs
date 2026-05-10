//! Integration tests for `solve_circuit_with_initial_voltages` — sindr's
//! `.NODESET` equivalent. These tests run against the public API only.

use std::collections::HashMap;

use sindr::{solve_circuit, solve_circuit_with_initial_voltages, Circuit, CircuitElement};

fn voltage_divider() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2_000.0,
            },
        ],
    }
}

fn diode_resistor(vsource: f64) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: vsource,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 100.0,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["n2".into(), "0".into()],
                temperature: 300.15,
            },
        ],
    }
}

#[test]
fn empty_seed_matches_plain_solve_linear() {
    let circuit = voltage_divider();
    let plain = solve_circuit(&circuit).unwrap();
    let seeded = solve_circuit_with_initial_voltages(&circuit, &HashMap::new()).unwrap();

    for node in plain.node_voltages.keys() {
        let p = plain.node_voltages[node];
        let s = seeded.node_voltages[node];
        assert!((p - s).abs() < 1e-9, "node {node}: plain={p}, seeded={s}");
    }
}

#[test]
fn empty_seed_matches_plain_solve_nonlinear() {
    let circuit = diode_resistor(5.0);
    let plain = solve_circuit(&circuit).unwrap();
    let seeded = solve_circuit_with_initial_voltages(&circuit, &HashMap::new()).unwrap();

    let v_plain = plain.node_voltages["n2"];
    let v_seed = seeded.node_voltages["n2"];
    assert!(
        (v_plain - v_seed).abs() < 1e-6,
        "diode V(n2): plain={v_plain}, seeded={v_seed}"
    );
}

#[test]
fn linear_circuit_ignores_seed() {
    // Seed must NOT alter the (unique) DC solution of a linear circuit.
    let circuit = voltage_divider();
    let mut seed = HashMap::new();
    seed.insert("n1".to_string(), 999.0);
    seed.insert("n2".to_string(), -42.0);

    let result = solve_circuit_with_initial_voltages(&circuit, &seed).unwrap();
    // V(n2) for the divider = 10 V * 2/3 ≈ 6.667 V
    let v_n2 = result.node_voltages["n2"];
    assert!(
        (v_n2 - 6.6667).abs() < 1e-3,
        "linear DC must ignore seed; got V(n2) = {v_n2}"
    );
}

#[test]
fn unknown_node_names_in_seed_are_silently_ignored() {
    let circuit = diode_resistor(5.0);
    let mut seed = HashMap::new();
    seed.insert("not_a_node".to_string(), 12.34);
    seed.insert("also_missing".to_string(), -1.0);

    // Must not error; should converge to the same answer as a plain solve.
    let result = solve_circuit_with_initial_voltages(&circuit, &seed)
        .expect("unknown seed keys must not cause failure");
    let v_n2 = result.node_voltages["n2"];
    assert!(
        (0.5..=0.9).contains(&v_n2),
        "expected diode forward drop, got V(n2) = {v_n2}"
    );
}

#[test]
fn seed_near_correct_answer_still_converges() {
    let circuit = diode_resistor(5.0);
    let mut seed = HashMap::new();
    seed.insert("n2".to_string(), 0.65); // a hint at the diode drop

    let result = solve_circuit_with_initial_voltages(&circuit, &seed).unwrap();
    let v_n2 = result.node_voltages["n2"];
    assert!(
        (0.5..=0.9).contains(&v_n2),
        "seeded result still physical: V(n2) = {v_n2}"
    );

    // Must agree with the unseeded answer to high precision.
    let plain = solve_circuit(&circuit).unwrap();
    let v_plain = plain.node_voltages["n2"];
    assert!(
        (v_n2 - v_plain).abs() < 1e-6,
        "seeded ({v_n2}) and unseeded ({v_plain}) must agree"
    );
}

#[test]
fn seed_far_from_answer_recovers_via_gmin() {
    // Massively wrong seed — gmin stepping is the safety net.
    let circuit = diode_resistor(5.0);
    let mut seed = HashMap::new();
    seed.insert("n1".to_string(), 1.0e4);
    seed.insert("n2".to_string(), -1.0e4);

    let result = solve_circuit_with_initial_voltages(&circuit, &seed).unwrap();
    let v_n2 = result.node_voltages["n2"];
    assert!(
        (0.5..=0.9).contains(&v_n2),
        "rescued solution must still be physical, got V(n2) = {v_n2}"
    );
}

#[test]
fn seed_does_not_break_transient_path() {
    // Circuits with reactive elements take the transient path; the seed
    // must be silently ignored without breaking the solve.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["n2".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    };

    let mut seed = HashMap::new();
    seed.insert("n2".to_string(), 999.0);

    let result = solve_circuit_with_initial_voltages(&circuit, &seed)
        .expect("transient solve must not error when given a seed");
    assert!(result.transient.is_some(), "expected transient data");
}
