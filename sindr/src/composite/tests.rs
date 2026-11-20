use super::*;
use crate::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use crate::design::{Part, Ratings};
use crate::{check_ratings, solve_circuit, solve_transient, RatingKind, RatingStatus};
use crate::{TransientConfig, Waveform};

fn v(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    }
}

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    }
}

fn spdt(position: SpdtPosition) -> Circuit {
    Circuit::new(
        "0",
        vec![
            v("v1", "in", "0", 10.0),
            CircuitElement::SwitchSpdt {
                id: "s1".into(),
                nodes: ["in".into(), "a".into(), "b".into()],
                position,
            },
            r("ra", "a", "0", 1e3),
            r("rb", "b", "0", 2e3),
        ],
    )
}

fn result<'a>(r: &'a SimulationResult, id: &str) -> &'a ComponentResult {
    r.component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("no result for {id}"))
}

#[test]
fn spdt_steers_to_the_selected_throw_and_reports_that_path() {
    let res = solve_circuit(&spdt(SpdtPosition::A)).unwrap();
    assert!((res.node_voltages["a"] - 10.0 * 1e3 / (1e3 + 0.01)).abs() < 1e-9);
    assert!(res.node_voltages["b"].abs() < 1e-4);
    let s1 = result(&res, "s1");
    assert!((s1.current_through - 10.0 / (1e3 + 0.01)).abs() < 1e-9);
    assert!(s1.voltage_across.abs() < 1e-3);

    let res = solve_circuit(&spdt(SpdtPosition::B)).unwrap();
    assert!((res.node_voltages["b"] - 10.0).abs() < 1e-4);
    assert!((result(&res, "s1").current_through - 10.0 / (2e3 + 0.01)).abs() < 1e-9);
}

#[test]
fn parts_and_internal_nodes_are_folded_away() {
    let res = solve_circuit(&spdt(SpdtPosition::A)).unwrap();
    assert!(res.component_results.iter().all(|c| !c.id.contains('#')));
    assert_eq!(
        res.component_results
            .iter()
            .filter(|c| c.id == "s1")
            .count(),
        1
    );

    let res = solve_transient(&bead_circuit(), &TransientConfig::new(1e-5, 1e-8)).unwrap();
    assert!(res.node_voltages.keys().all(|n| !n.contains('#')));
    for step in &res.transient.as_ref().unwrap().timesteps {
        assert!(step.node_voltages.keys().all(|n| !n.contains('#')));
        assert!(step.component_results.iter().any(|c| c.id == "fb1"));
        assert!(step.component_results.iter().all(|c| !c.id.contains('#')));
    }
}

fn bead_circuit() -> Circuit {
    Circuit::new(
        "0",
        vec![
            v("v1", "in", "0", 1.0),
            CircuitElement::FerriteBead {
                id: "fb1".into(),
                nodes: ["in".into(), "out".into()],
                params: FerriteBeadParams {
                    r_dc: 0.5,
                    l: 1e-6,
                    r_ac: 600.0,
                    c: None,
                },
            },
            r("rl", "out", "0", 9.5),
        ],
    )
}

#[test]
fn bead_settles_to_its_dc_resistance() {
    // L/R = 1 µH / 10 Ω = 100 ns; 10 µs is a hundred time constants.
    let res = solve_transient(&bead_circuit(), &TransientConfig::new(1e-5, 1e-8)).unwrap();
    let fb = result(&res, "fb1");
    assert!(
        (fb.current_through - 0.1).abs() < 1e-6,
        "{}",
        fb.current_through
    );
    assert!((fb.voltage_across - 0.05).abs() < 1e-6);
    assert!((fb.power - 0.1 * 0.1 * 0.5).abs() < 1e-6);
}

fn ac_impedance(element: CircuitElement, f: f64) -> num_complex::Complex64 {
    // A 1 V source through the element into a 1 Ω sense resistor: the
    // element's impedance is (1 − V(out)) / V(out).
    let circuit = Circuit::new(
        "0",
        vec![v("v1", "in", "0", 0.0), element, r("rs", "out", "0", 1.0)],
    );
    let ac = solve_ac(
        &circuit,
        &AcConfig {
            f_start: f,
            f_stop: f,
            num_points: 1,
            spacing: FrequencySpacing::Linear,
            source_id: "v1".into(),
            ac_magnitude: 1.0,
        },
    )
    .unwrap();
    let out = ac.points[0].node_voltages["out"];
    assert!(ac.points[0].node_voltages.keys().all(|n| !n.contains('#')));
    (1.0 - out) / out
}

#[test]
fn crystal_impedance_is_the_bvd_network() {
    use num_complex::Complex64;
    let p = CrystalParams {
        frequency: 8e6,
        esr: 30.0,
        c0: 4e-12,
        cm: 15e-15,
    };
    let crystal = || CircuitElement::Crystal {
        id: "y1".into(),
        nodes: ["in".into(), "out".into()],
        params: p,
    };
    for f in [1e6, 7.99e6, 8e6, 8.01e6, 8.02e6, 30e6] {
        let w = 2.0 * std::f64::consts::PI * f;
        let j = Complex64::i();
        let motional = p.esr + j * w * p.lm() + 1.0 / (j * w * p.cm);
        let shunt = 1.0 / (j * w * p.c0);
        let want = motional * shunt / (motional + shunt);
        let got = ac_impedance(crystal(), f);
        assert!(
            (got - want).norm() <= 1e-6 * want.norm(),
            "{f} Hz: {got} vs {want}"
        );
    }
    // At series resonance the motional arm is its ESR alone.
    let w = 2.0 * std::f64::consts::PI * p.frequency;
    assert!((w * p.lm() - 1.0 / (w * p.cm)).abs() < 1e-6);
}

#[test]
fn bead_impedance_matches_its_datasheet_point() {
    let p = FerriteBeadParams::from_datasheet(600.0, 0.15);
    let z = ac_impedance(
        CircuitElement::FerriteBead {
            id: "fb1".into(),
            nodes: ["in".into(), "out".into()],
            params: p,
        },
        100e6,
    );
    // jωL ∥ R with ωL = R = 600 Ω has magnitude 600/√2, plus r_dc.
    let want = 0.15 + 600.0 / 2.0;
    assert!((z.re - want).abs() < 1e-3, "{z}");
    assert!((z.im - 300.0).abs() < 1e-3, "{z}");
}

#[test]
fn ratings_see_the_folded_part() {
    let mut circuit = bead_circuit();
    circuit.parts.insert(
        "fb1".into(),
        Part {
            ratings: Ratings {
                current: Some(0.05),
                power: Some(1.0),
                ..Ratings::default()
            },
            ..Part::default()
        },
    );
    let res = solve_transient(&circuit, &TransientConfig::new(1e-5, 1e-8)).unwrap();
    let report = check_ratings(&circuit, &res).unwrap();
    let current = report
        .for_part("fb1")
        .find(|c| c.rating == RatingKind::Current)
        .unwrap();
    assert_eq!(current.status, RatingStatus::Exceeded);
    let power = report
        .for_part("fb1")
        .find(|c| c.rating == RatingKind::Power)
        .unwrap();
    assert_eq!(power.status, RatingStatus::Within);
}

#[test]
fn crystal_drive_level_is_its_esr_power() {
    let p = CrystalParams::from_datasheet(1e6, 100.0, 5e-12);
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "v1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Sine {
                    amplitude: 1.0,
                    frequency: 1e6,
                    phase: 0.0,
                    offset: 0.0,
                }),
            },
            CircuitElement::Crystal {
                id: "y1".into(),
                nodes: ["in".into(), "0".into()],
                params: p,
            },
        ],
    );
    let res = solve_transient(&circuit, &TransientConfig::new(2e-6, 2e-9)).unwrap();
    for step in &res.transient.unwrap().timesteps {
        let y1 = result_in(&step.component_results, "y1");
        assert!(y1.power >= 0.0, "dissipation only, never reactive power");
    }
}

fn result_in<'a>(results: &'a [ComponentResult], id: &str) -> &'a ComponentResult {
    results.iter().find(|c| c.id == id).unwrap()
}

#[test]
fn bad_parameters_name_the_part() {
    let mut circuit = bead_circuit();
    if let CircuitElement::FerriteBead { params, .. } = &mut circuit.components[1] {
        params.l = 0.0;
    }
    match solve_circuit(&circuit) {
        Err(SimError::InvalidComponent(msg)) => {
            assert!(msg.contains("fb1") && msg.contains("l (H)"))
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_part_id_already_taken_is_refused() {
    let mut circuit = spdt(SpdtPosition::A);
    circuit.components.push(r("s1#a", "a", "0", 1e3));
    assert!(matches!(
        solve_circuit(&circuit),
        Err(SimError::InvalidComponent(_))
    ));
}

#[test]
fn stamping_directly_is_an_error_not_a_silent_open() {
    let circuit = spdt(SpdtPosition::A);
    let node_map = crate::node_map::NodeMap::from_circuit(&circuit);
    let mut system = crate::MnaSystem::new(node_map.num_nodes(), circuit.count_voltage_sources());
    assert!(crate::stamp::stamp_circuit(&circuit, &mut system, &node_map, None).is_err());
}

#[cfg(feature = "serde")]
#[test]
fn serde_round_trip() {
    let json = r#"[
        {"type": "switch_spdt", "id": "s1", "nodes": ["c", "a", "b"], "position": "b"},
        {"type": "switch_spdt", "id": "s2", "nodes": ["c", "a", "b"]},
        {"type": "crystal", "id": "y1", "nodes": ["a", "b"],
         "params": {"frequency": 16e6, "esr": 40, "c0": 5e-12, "cm": 2e-14}},
        {"type": "ferrite_bead", "id": "fb1", "nodes": ["a", "b"],
         "params": {"r_dc": 0.1, "l": 1e-6, "r_ac": 600}}
    ]"#;
    let elements: Vec<CircuitElement> = serde_json::from_str(json).unwrap();
    assert!(matches!(
        elements[1],
        CircuitElement::SwitchSpdt {
            position: SpdtPosition::A,
            ..
        }
    ));
    let back: Vec<CircuitElement> =
        serde_json::from_str(&serde_json::to_string(&elements).unwrap()).unwrap();
    assert_eq!(back, elements);
}
