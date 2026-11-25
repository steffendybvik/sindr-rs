use super::*;
use crate::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use crate::design::{Part, Ratings};
use crate::element::{Crystal, FerriteBead, Resistor, SwitchSpdt, VoltageSource};
use crate::{check_ratings, solve_circuit, solve_transient, RatingKind, RatingStatus};
use crate::{TransientConfig, Waveform};

fn v(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    })
}

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn spdt(position: SpdtPosition) -> Circuit {
    Circuit::new(
        "0",
        vec![
            v("v1", "in", "0", 10.0),
            CircuitElement::SwitchSpdt(SwitchSpdt {
                id: "s1".into(),
                nodes: ["in".into(), "a".into(), "b".into()],
                position,
            }),
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
            CircuitElement::FerriteBead(FerriteBead {
                id: "fb1".into(),
                nodes: ["in".into(), "out".into()],
                params: FerriteBeadParams {
                    r_dc: 0.5,
                    l: 1e-6,
                    r_ac: 600.0,
                    c: None,
                },
            }),
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
        &AcConfig::new("v1", f, f, 1).with_spacing(FrequencySpacing::Linear),
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
    let crystal = || {
        CircuitElement::Crystal(Crystal {
            id: "y1".into(),
            nodes: ["in".into(), "out".into()],
            params: p,
        })
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
        CircuitElement::FerriteBead(FerriteBead {
            id: "fb1".into(),
            nodes: ["in".into(), "out".into()],
            params: p,
        }),
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
            CircuitElement::VoltageSource(VoltageSource {
                id: "v1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Sine {
                    amplitude: 1.0,
                    frequency: 1e6,
                    phase: 0.0,
                    offset: 0.0,
                }),
            }),
            CircuitElement::Crystal(Crystal {
                id: "y1".into(),
                nodes: ["in".into(), "0".into()],
                params: p,
            }),
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
    if let CircuitElement::FerriteBead(FerriteBead { params, .. }) = &mut circuit.components[1] {
        params.l = 0.0;
    }
    match solve_circuit(&circuit) {
        Err(SimError::InvalidComponent { id, reason }) => {
            assert!(id == "fb1" && reason.contains("l (H)"))
        }
        other => panic!("{other:?}"),
    }
}

fn crystal_with(c0: f64) -> CircuitElement {
    CircuitElement::Crystal(Crystal {
        id: "y1".into(),
        nodes: ["in".into(), "out".into()],
        params: CrystalParams {
            frequency: 8e6,
            esr: 30.0,
            c0,
            cm: 15e-15,
        },
    })
}

#[test]
fn crystal_without_c0_is_its_motional_arm_alone() {
    // With no shunt capacitance the crystal is Rm–Lm–Cm in series, so at
    // series resonance it is a pure 30 Ω.
    assert_eq!(lower(&crystal_with(0.0)).unwrap().unwrap().len(), 3);
    let z = ac_impedance(crystal_with(0.0), 8e6);
    assert!((z.re - 30.0).abs() < 1e-6 && z.im.abs() < 1e-6, "{z}");
}

#[test]
fn negative_or_nan_c0_is_refused() {
    for c0 in [-1e-12, f64::NAN] {
        match lower(&crystal_with(c0)) {
            Err(SimError::InvalidComponent { id, reason }) => {
                assert!(id == "y1" && reason.contains("c0"), "{reason}")
            }
            other => panic!("c0 = {c0}: {other:?}"),
        }
    }
}

#[test]
fn a_circuit_that_cannot_flatten_is_left_as_it_is() {
    // `fold` and `strip_nodes` run after a solve, which has flattened the
    // circuit already; given one that doesn't flatten they change nothing.
    let mut broken = spdt(SpdtPosition::A);
    broken
        .instances
        .push(crate::design::Instance::new("X1", "missing", ["in"]));
    assert!(broken.flatten().is_err());

    let mut res = solve_circuit(&spdt(SpdtPosition::A)).unwrap();
    res.node_voltages.insert("s1#x".into(), 1.0);
    let before = res.component_results.len();
    fold(&broken, &mut res);
    assert_eq!(res.node_voltages.get("s1#x"), Some(&1.0));
    assert_eq!(res.component_results.len(), before);

    strip_nodes(&broken, [&mut res.node_voltages]);
    assert!(res.node_voltages.contains_key("s1#x"));
    strip_nodes(&spdt(SpdtPosition::A), [&mut res.node_voltages]);
    assert!(!res.node_voltages.contains_key("s1#x"));
}

#[test]
fn a_part_id_already_taken_is_refused() {
    let mut circuit = spdt(SpdtPosition::A);
    circuit.components.push(r("s1#a", "a", "0", 1e3));
    assert!(matches!(
        solve_circuit(&circuit),
        Err(SimError::DuplicateId { lowered: true, .. })
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
        CircuitElement::SwitchSpdt(SwitchSpdt {
            position: SpdtPosition::A,
            ..
        })
    ));
    let back: Vec<CircuitElement> =
        serde_json::from_str(&serde_json::to_string(&elements).unwrap()).unwrap();
    assert_eq!(back, elements);
}

/// An SPDT inside a block, so the sweeps fold an instance's composite.
fn spdt_in_a_block(volts: f64) -> Circuit {
    let inner = spdt(SpdtPosition::A);
    let block = crate::design::Block {
        ports: vec!["in".into()],
        components: inner.components[1..].to_vec(),
        ..Default::default()
    };
    let mut c = Circuit::new("0", vec![v("v1", "in", "0", volts)]);
    c.blocks.insert("sw".into(), block);
    c.instances
        .push(crate::design::Instance::new("x1", "sw", ["in"]));
    c
}

fn assert_folded_like(point: &SimulationResult, want: &SimulationResult) {
    let mut ids: Vec<_> = point.component_results.iter().map(|c| &c.id).collect();
    let mut want_ids: Vec<_> = want.component_results.iter().map(|c| &c.id).collect();
    ids.sort();
    want_ids.sort();
    assert_eq!(ids, want_ids);
    let mut nodes: Vec<_> = point.node_voltages.keys().collect();
    let mut want_nodes: Vec<_> = want.node_voltages.keys().collect();
    nodes.sort();
    want_nodes.sort();
    assert_eq!(nodes, want_nodes);
    for c in &want.component_results {
        let got = result(point, &c.id);
        assert!((got.current_through - c.current_through).abs() < 1e-12);
        assert!((got.voltage_across - c.voltage_across).abs() < 1e-9);
        assert!((got.power - c.power).abs() < 1e-12);
    }
}

#[test]
fn sweeps_fold_every_point_of_a_hierarchical_composite() {
    use crate::{dc_sweep, dc_sweep_nested, solve_op, temperature_sweep};
    use crate::{DcSweepAxis, TempSweepConfig};

    let circuit = spdt_in_a_block(0.0);
    let sweep = dc_sweep(&circuit, &DcSweepAxis::new("v1", 0.0, 10.0, 6)).unwrap();
    for point in &sweep.points {
        let want = solve_op(&spdt_in_a_block(point.sweep_value)).unwrap();
        assert!(want.component_results.iter().all(|c| !c.id.contains('#')));
        assert_folded_like(&point.result, &want);
    }

    let mut two = circuit.clone();
    two.components.push(v("v2", "in2", "0", 0.0));
    two.components.push(r("r2", "in2", "0", 1e3));
    let nested = dc_sweep_nested(
        &two,
        &DcSweepAxis::new("v1", 0.0, 10.0, 3),
        &DcSweepAxis::new("v2", 0.0, 1.0, 2),
    )
    .unwrap();
    for curve in &nested.curves {
        for point in &curve.sweep.points {
            let mut at = two.clone();
            at.components[0] = v("v1", "in", "0", point.sweep_value);
            at.components[1] = v("v2", "in2", "0", curve.outer_value);
            assert_folded_like(&point.result, &solve_op(&at).unwrap());
        }
    }

    let circuit = spdt_in_a_block(5.0);
    let want = solve_op(&circuit).unwrap();
    let temps = temperature_sweep(&circuit, &TempSweepConfig::new(250.0, 350.0, 3)).unwrap();
    for point in &temps.points {
        assert_folded_like(&point.result, &want);
    }

    let ac = solve_ac(&circuit, &AcConfig::new("v1", 1.0, 1e3, 4)).unwrap();
    for point in &ac.points {
        let mut nodes: Vec<_> = point.node_voltages.keys().collect();
        let mut want_nodes: Vec<_> = want.node_voltages.keys().collect();
        nodes.sort();
        want_nodes.sort();
        assert_eq!(nodes, want_nodes);
    }
}
