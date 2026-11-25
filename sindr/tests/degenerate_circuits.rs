//! Circuits with nothing to solve still give an answer or a typed error,
//! never a panic.

use sindr::element::Resistor;
use sindr::{
    solve_ac, solve_circuit, solve_transient, AcConfig, Circuit, CircuitElement, TransientConfig,
};

/// A resistor from ground to ground: no unknowns at all.
fn ground_only() -> Circuit {
    Circuit::new(
        "0",
        vec![CircuitElement::Resistor(Resistor {
            id: "R1".into(),
            nodes: ["0".into(), "0".into()],
            resistance: 1e3,
        })],
    )
}

#[test]
fn an_element_from_ground_to_ground_solves_to_zero() {
    let op = solve_circuit(&ground_only()).unwrap();
    assert_eq!(op.voltage("0"), Some(0.0));
    assert_eq!(op.current("R1"), Some(0.0));

    let tran = solve_transient(&ground_only(), &TransientConfig::new(1e-6, 1e-7)).unwrap();
    assert!(!tran.transient.unwrap().timesteps.is_empty());
}

#[test]
fn ac_with_no_unknowns_is_an_error_not_a_panic() {
    // Its stimulus has to be a source in the circuit, and there is none.
    assert!(solve_ac(&ground_only(), &AcConfig::new("V1", 1.0, 1e3, 3)).is_err());
}
