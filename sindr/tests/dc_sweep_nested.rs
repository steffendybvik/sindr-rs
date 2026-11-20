//! Two-source DC sweeps: device curve families, seeding, limits and the
//! two-source `.dc` card.

use sindr::dc_sweep::DcSweepAxis;
use sindr::{dc_sweep, dc_sweep_nested, Circuit, CircuitElement, MosfetKind};
use sindr_devices::mosfet::MosfetParams;

fn vsource(id: &str, a: &str, b: &str) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [a.into(), b.into()],
        voltage: 0.0,
        waveform: None,
    }
}

/// An NMOS with its source grounded, gate on VGS and drain on VDS.
fn nmos(params: MosfetParams) -> Circuit {
    Circuit::new(
        "0",
        vec![
            vsource("VGS", "g", "0"),
            vsource("VDS", "d", "0"),
            CircuitElement::Mosfet {
                id: "M1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind: MosfetKind::Nmos,
                params,
                parasitic_caps: None,
                body_diode: None,
            },
        ],
    )
}

/// Level 1 drain current with the source and body grounded.
fn level1(p: &MosfetParams, vgs: f64, vds: f64) -> f64 {
    let vov = vgs - p.vto;
    if vov <= 0.0 {
        0.0
    } else if vds < vov {
        p.kp * (vov * vds - vds * vds / 2.0) * (1.0 + p.lambda * vds)
    } else {
        p.kp / 2.0 * vov * vov * (1.0 + p.lambda * vds)
    }
}

/// Id against VDS for each VGS: every point on the Level 1 curve, the
/// inner axis fastest.
#[test]
fn nmos_curve_family_matches_level_1() {
    let p = MosfetParams::default_nmos();
    let sweep = dc_sweep_nested(
        &nmos(p.clone()),
        &DcSweepAxis::from_step("VDS", 0.0, 5.0, 0.25).unwrap(),
        &DcSweepAxis::from_step("VGS", 0.0, 3.0, 0.5).unwrap(),
    )
    .unwrap();
    assert_eq!(sweep.outer_source_id, "VGS");
    assert_eq!(sweep.curves.len(), 7);
    for curve in &sweep.curves {
        assert_eq!(curve.sweep.source_id, "VDS");
        assert_eq!(curve.sweep.points.len(), 21);
        for point in &curve.sweep.points {
            let vgs = curve.outer_value;
            let vds = point.sweep_value;
            assert_eq!(point.result.node_voltages["g"], vgs);
            let m1 = point
                .result
                .mosfet_results
                .iter()
                .find(|m| m.id == "M1")
                .unwrap();
            let want = level1(&p, vgs, vds);
            assert!(
                (m1.id_current - want).abs() < 1e-9 + 1e-6 * want.abs(),
                "Id({vgs}, {vds}) = {}, want {want}",
                m1.id_current
            );
        }
    }
    // The curves fan out: more gate drive, more current at every VDS > 0.
    let at_5v: Vec<f64> = sweep
        .curves
        .iter()
        .map(|c| c.sweep.points.last().unwrap().result.mosfet_results[0].id_current)
        .collect();
    assert!(at_5v.windows(2).all(|w| w[1] >= w[0]), "{at_5v:?}");
}

/// A BJT's output curves come from a base current source: the sweep takes
/// current sources on either axis.
#[test]
fn bjt_curves_sweep_a_base_current() {
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::CurrentSource {
                id: "IB".into(),
                nodes: ["0".into(), "b".into()],
                current: 0.0,
                waveform: None,
            },
            vsource("VCE", "c", "0"),
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["b".into(), "c".into(), "0".into()],
                kind: sindr::BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            },
        ],
    );
    let sweep = dc_sweep_nested(
        &circuit,
        &DcSweepAxis::new("VCE", 0.0, 5.0, 11),
        &DcSweepAxis::new("IB", 10e-6, 30e-6, 3),
    )
    .unwrap();
    for curve in &sweep.curves {
        // In the active region Ic ≈ β·Ib.
        let ic = curve.sweep.points.last().unwrap().result.bjt_results[0].ic;
        let want = 100.0 * curve.outer_value;
        assert!(
            (ic - want).abs() < 0.1 * want,
            "Ic = {ic} at Ib = {}",
            curve.outer_value
        );
    }
}

/// Each point starts from its neighbour, so a sweep gives the same answers
/// as solving each point on its own, here for a diode's I–V curve, to
/// within Newton's tolerance (V_ABSTOL = 1 µV): the two start from
/// different guesses, so they stop at slightly different iterates.
#[test]
fn seeded_points_match_independent_solves() {
    let circuit = Circuit::new(
        "0",
        vec![
            vsource("V1", "a", "0"),
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["a".into(), "b".into()],
                resistance: 100.0,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["b".into(), "0".into()],
                temperature: 300.15,
                model: None,
            },
        ],
    );
    let sweep = dc_sweep(&circuit, "V1", -1.0, 5.0, 25).unwrap();
    for point in &sweep.points {
        let mut alone = circuit.clone();
        if let CircuitElement::VoltageSource { voltage, .. } = &mut alone.components[0] {
            *voltage = point.sweep_value;
        }
        let want = sindr::solve_circuit(&alone).unwrap().node_voltages["b"];
        let got = point.result.node_voltages["b"];
        assert!((got - want).abs() < 1e-6, "v(b) = {got}, want {want}");
    }
}

#[test]
fn the_axes_are_checked() {
    let c = nmos(MosfetParams::default_nmos());
    let vds = DcSweepAxis::new("VDS", 0.0, 1.0, 3);
    let err = |inner: &DcSweepAxis, outer: &DcSweepAxis| {
        dc_sweep_nested(&c, inner, outer).unwrap_err().to_string()
    };
    assert!(err(&vds, &vds).contains("both"));
    assert!(err(&vds, &DcSweepAxis::new("nope", 0.0, 1.0, 3)).contains("not found"));
    assert!(err(&vds, &DcSweepAxis::new("VGS", 0.0, 1.0, 1)).contains("at least 2"));
    // Each axis is within the limit, but not the two together.
    assert!(err(
        &DcSweepAxis::new("VDS", 0.0, 1.0, 10_000),
        &DcSweepAxis::new("VGS", 0.0, 1.0, 10_000),
    )
    .contains("more than"));
    // A single value is fine when start and stop agree.
    let one = dc_sweep_nested(&c, &vds, &DcSweepAxis::new("VGS", 2.0, 2.0, 1)).unwrap();
    assert_eq!(one.curves.len(), 1);
}

#[test]
fn from_step_reads_spice_steps() {
    let a = DcSweepAxis::from_step("V1", 0.0, 1.0, 0.1).unwrap();
    assert_eq!((a.num_points, a.stop), (11, 1.0));
    // A step that doesn't divide the span stops short, as SPICE does.
    let a = DcSweepAxis::from_step("V1", 0.0, 1.0, 0.3).unwrap();
    assert_eq!(a.num_points, 4);
    assert!((a.stop - 0.9).abs() < 1e-12);
    let a = DcSweepAxis::from_step("V1", 5.0, 0.0, -1.0).unwrap();
    assert_eq!((a.num_points, a.stop), (6, 0.0));
    assert_eq!(
        DcSweepAxis::from_step("V1", 2.0, 2.0, 0.0)
            .unwrap()
            .num_points,
        1
    );
    assert!(DcSweepAxis::from_step("V1", 0.0, 1.0, -0.1).is_err());
    assert!(DcSweepAxis::from_step("V1", 0.0, 1.0, 0.0).is_err());
    assert!(DcSweepAxis::from_step("V1", 0.0, 1.0, 1e-12).is_err());
}

#[cfg(feature = "spice")]
mod spice {
    use sindr::spice::parse_str;

    const DECK: &str = "* nmos family\nVGS g 0 0\nVDS d 0 0\nR1 g d 1k\n";

    #[test]
    fn two_source_dc_runs() {
        let netlist = parse_str(&format!("{DECK}.dc VDS 0 5 0.5 VGS 0 3 1\n.end\n")).unwrap();
        let (inner, outer) = netlist.analyses[0].dc_axes().unwrap().unwrap();
        assert_eq!((inner.source_id.as_str(), inner.num_points), ("vds", 11));
        let outer = outer.unwrap();
        assert_eq!((outer.source_id.as_str(), outer.num_points), ("vgs", 4));
        let sweep = sindr::dc_sweep_nested(&netlist.circuit, &inner, &outer).unwrap();
        // R1 carries (VGS − VDS) / 1 kΩ, from g to d.
        let last = sweep.curves[3].sweep.component_current_curve("r1");
        assert!((last[0].1 - 3e-3).abs() < 1e-12);
        assert!((last[10].1 - (3.0 - 5.0) / 1e3).abs() < 1e-12);
    }

    #[test]
    fn two_source_dc_errors() {
        for (card, why) in [
            (".dc VDS 0 5 0.5 VDS 0 3 1", "twice"),
            (
                ".dc VDS 0 5 0.5 VX 0 3 1",
                "not a voltage or current source",
            ),
            (".dc VDS 0 5 0.5 VGS 0 3 -1", "doesn't move"),
        ] {
            let err = parse_str(&format!("{DECK}{card}\n.end\n")).unwrap_err();
            assert!(format!("{err:?}").contains(why), "{card}: {err:?}");
        }
        assert!(parse_str(&format!("{DECK}.dc VDS 0 5 0.5 VGS 0 3\n.end\n")).is_err());
    }
}
