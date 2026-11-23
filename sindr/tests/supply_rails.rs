//! A supply rail shared by many parts is electrically the same as a supply
//! per part: same node voltages, and the shared source carries the sum of
//! the separate ones' currents. Large enough to take the sparse solver,
//! where the rail is substituted before factorising.
//!
//! The two circuits are solved separately, so they agree to Newton's
//! stopping point, not to the last bit: tolerances here are 1e-7 V and
//! 1e-6 relative, well inside sindr's VNTOL (1 µV) and RELTOL (1e-3).

use sindr::element::{Bjt, Resistor, VoltageSource};
use sindr::{solve_op, BjtKind, Circuit, CircuitElement};

fn r(id: String, a: String, b: String, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id,
        nodes: [a, b],
        resistance: ohms,
    })
}

fn supply(id: &str, node: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [node.into(), "0".into()],
        voltage: volts,
        waveform: None,
    })
}

/// `stages` common-emitter stages in a row, each fed from the one before.
/// With `shared`, every collector resistor hangs off one `vcc` rail;
/// otherwise each stage has its own 5 V source `vcc<i>`.
fn row(stages: usize, shared: bool) -> Circuit {
    let mut parts = vec![supply("vin", "in", 1.0)];
    if shared {
        parts.push(supply("vcc", "vcc", 5.0));
    }
    for i in 1..=stages {
        let feed = if i == 1 {
            "in".to_string()
        } else {
            format!("c{}", i - 1)
        };
        let rail = if shared {
            "vcc".to_string()
        } else {
            parts.push(supply(&format!("vcc{i}"), &format!("vcc{i}"), 5.0));
            format!("vcc{i}")
        };
        parts.push(r(format!("RB{i}"), feed, format!("b{i}"), 100e3));
        parts.push(r(format!("RC{i}"), rail, format!("c{i}"), 1e3));
        parts.push(r(format!("RE{i}"), format!("e{i}"), "0".into(), 100.0));
        parts.push(CircuitElement::Bjt(Bjt {
            id: format!("Q{i}"),
            nodes: [format!("b{i}"), format!("c{i}"), format!("e{i}")],
            kind: BjtKind::Npn,
            bf: 100.0,
            temperature: 300.15,
            parasitic_caps: None,
            model: None,
        }));
    }
    Circuit::new("0", parts)
}

#[test]
fn a_shared_rail_solves_as_separate_supplies() {
    let stages = 60; // 3 nodes a stage: well past the dense limit
    let (shared, separate) = (
        solve_op(&row(stages, true)).unwrap(),
        solve_op(&row(stages, false)).unwrap(),
    );
    for i in 1..=stages {
        for node in [format!("b{i}"), format!("c{i}"), format!("e{i}")] {
            let (a, b) = (
                shared.voltage(&node).unwrap(),
                separate.voltage(&node).unwrap(),
            );
            assert!((a - b).abs() < 1e-7, "{node}: {a} vs {b}");
        }
    }
    assert_eq!(shared.voltage("vcc"), Some(5.0));
    let total: f64 = (1..=stages)
        .map(|i| separate.current(&format!("vcc{i}")).unwrap())
        .sum();
    let rail = shared.current("vcc").unwrap();
    assert!(
        (rail - total).abs() <= 1e-6 * total.abs(),
        "{rail} vs {total}"
    );
    // KCL at the rail: the source supplies what the collector resistors draw.
    let drawn: f64 = (1..=stages)
        .map(|i| shared.current(&format!("RC{i}")).unwrap())
        .sum();
    assert!(
        (rail.abs() - drawn.abs()).abs() <= 1e-9 * drawn.abs(),
        "{rail} vs {drawn}"
    );
}
