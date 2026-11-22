//! SPICE's `M` (MOSFET) and `J` (JFET) lines: how a level-1 card maps onto
//! sindr's FETs, what strict and lenient mode do with what sindr can't
//! model, and that `to_netlist`'s output for them parses back to the same
//! circuit. ngspice agreement is in `golden.rs`.
//!
//! Only compiled when the `spice` feature is enabled.
#![cfg(feature = "spice")]

use sindr::circuit::{JfetParasiticCaps, MosfetParasiticCaps};
use sindr::devices::mosfet::MosfetParams;
use sindr::spice::{
    parse_str, parse_str_with_options, to_netlist, ExportError, ParseOptions, ParsedNetlist,
    SpiceParseError,
};
use sindr::{Circuit, CircuitElement, DiodeModel, JfetKind, MosfetKind};

fn lenient(deck: &str) -> ParsedNetlist {
    parse_str_with_options(deck, ParseOptions::default().with_strict(false)).unwrap()
}

fn find<'a>(circuit: &'a Circuit, id: &str) -> &'a CircuitElement {
    circuit
        .components
        .iter()
        .find(|c| c.id() == id)
        .unwrap_or_else(|| panic!("no `{id}` in {:#?}", circuit.components))
}

fn strict_error(deck: &str) -> String {
    parse_str(deck)
        .expect_err("strict mode should refuse")
        .to_string()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-12 * b.abs().max(1e-30)
}

const NMOS: &str = "t\nM1 d g s s nch\n.model nch NMOS\n.end\n";

#[test]
fn bare_card_takes_spice_level_1_defaults() {
    let netlist = parse_str(NMOS).unwrap();
    let CircuitElement::Mosfet {
        nodes,
        kind,
        params,
        parasitic_caps,
        body_diode,
        ..
    } = find(&netlist.circuit, "m1")
    else {
        panic!()
    };
    assert_eq!(nodes, &["g".to_string(), "d".into(), "s".into()]);
    assert_eq!(*kind, MosfetKind::Nmos);
    // W = L = 100 µm, so kp is KP's 2e-5.
    assert!(close(params.kp, 2e-5));
    assert_eq!((params.vto, params.lambda, params.gamma), (0.0, 0.0, 0.0));
    assert!(close(params.phi, 0.6));
    assert_eq!(*parasitic_caps, None);
    // SPICE's drain–bulk junction, IS = 1e-14 A, is the body diode.
    let body = body_diode.expect("drain–bulk junction");
    assert!(close(body.is, 1e-14));
    assert_eq!((body.cjo, body.vj, body.m, body.fc), (0.0, 0.8, 0.5, 0.5));
}

#[test]
fn card_and_line_scale_into_the_element() {
    let deck = "t\n\
        M1 d g 0 0 p1 W=10u L=1u M=3 AD=1p AS=1p PD=4u PS=4u NRD=1 NRS=1\n\
        .model p1 PMOS(LEVEL=1 KP=40u VTO=-0.9 LAMBDA=0.05 GAMMA=0.3 PHI=0.7 LD=0.05u\n\
        + CGSO=0.5n CGDO=0.4n CGBO=0.1n IS=0 CBD=2p CBS=2p PB=0.7 MJ=0.33 FC=0.4 RD=0 RS=0)\n\
        .end\n";
    let netlist = parse_str(deck).unwrap();
    let CircuitElement::Mosfet {
        kind,
        params,
        parasitic_caps,
        body_diode,
        ..
    } = find(&netlist.circuit, "m1")
    else {
        panic!()
    };
    let (w, l_eff, m) = (10e-6, 1e-6 - 0.1e-6, 3.0);
    assert_eq!(*kind, MosfetKind::Pmos);
    assert!(close(params.kp, 40e-6 * w / l_eff * m));
    assert!(close(params.vto, -0.9));
    assert!(close(params.lambda, 0.05));
    assert!(close(params.gamma, 0.3));
    assert!(close(params.phi, 0.7));
    let MosfetParasiticCaps { cgs, cgd, .. } = parasitic_caps.clone().expect("caps");
    assert!(close(cgs, (0.5e-9 * w + 0.1e-9 * l_eff) * m));
    assert!(close(cgd, 0.4e-9 * w * m));
    // IS = 0 still leaves CBD's junction capacitance.
    let body = body_diode.expect("CBD");
    assert_eq!(body.is, 0.0);
    assert!(close(body.cjo, 2e-12 * m));
    assert_eq!((body.vj, body.m, body.fc), (0.7, 0.33, 0.4));
}

#[test]
fn no_junction_means_no_body_diode() {
    let netlist = parse_str("t\nM1 d g 0 0 n1\n.model n1 NMOS(IS=0)\n.end\n").unwrap();
    let CircuitElement::Mosfet { body_diode, .. } = find(&netlist.circuit, "m1") else {
        panic!()
    };
    assert_eq!(*body_diode, None);
}

#[test]
fn mosfet_inside_a_subckt_checks_its_own_bulk() {
    let deck = "t\n\
        .subckt sw d g s\n\
        M1 d g s s nch\n\
        .ends\n\
        .model nch NMOS(KP=1m VTO=2)\n\
        X1 out gate 0 sw\n\
        .end\n";
    let netlist = parse_str(deck).unwrap();
    let flat = netlist.circuit.flatten().unwrap();
    let CircuitElement::Mosfet { nodes, .. } = find(&flat, "x1.m1") else {
        panic!()
    };
    assert_eq!(nodes, &["gate".to_string(), "out".into(), "0".into()]);
}

#[test]
fn bulk_off_the_source_is_refused_in_strict_and_warned_in_lenient() {
    let deck = "t\nM1 d g s 0 nch\n.model nch NMOS\n.end\n";
    assert!(strict_error(deck).contains("ties the body to the source"));
    let netlist = lenient(deck);
    assert!(netlist.warnings.iter().any(|w| w.message.contains("bulk")));
    assert!(matches!(
        find(&netlist.circuit, "m1"),
        CircuitElement::Mosfet { .. }
    ));
}

#[test]
fn what_sindr_cant_model_is_refused_in_strict_and_warned_in_lenient() {
    for (deck, says) in [
        ("t\nM1 d g s s n\n.model n NMOS(TOX=10n)\n.end\n", "TOX"),
        ("t\nM1 d g s s n\n.model n NMOS(RD=1)\n.end\n", "RD=1"),
        ("t\nM1 d g s s n OFF\n.model n NMOS\n.end\n", "OFF"),
        ("t\nM1 d g s s n TEMP=50\n.model n NMOS\n.end\n", "TEMP"),
        ("t\nJ1 d g s j\n.model j NJF(LAMBDA=0.01)\n.end\n", "LAMBDA"),
        ("t\nJ1 d g s j\n.model j NJF(IS=1f)\n.end\n", "IS"),
        ("t\nJ1 d g s j\n.model j NJF(RD=10)\n.end\n", "RD"),
        ("t\nJ1 d g s j OFF\n.model j NJF\n.end\n", "OFF"),
    ] {
        let message = strict_error(deck);
        assert!(message.contains(says), "{says}: {message}");
        let netlist = lenient(deck);
        assert!(
            netlist.warnings.iter().any(|w| w.message.contains(says)),
            "{says}: {:?}",
            netlist.warnings
        );
    }
}

#[test]
fn another_level_is_refused_even_in_lenient_mode() {
    let deck = "t\nM1 d g s s n\n.model n NMOS(LEVEL=3 KP=1m)\n.end\n";
    assert!(strict_error(deck).contains("`.model n` LEVEL=3"));
    let lenient = parse_str_with_options(deck, ParseOptions::default().with_strict(false));
    assert!(lenient.unwrap_err().to_string().contains("LEVEL=3"));
}

#[test]
fn neutral_values_pass_in_strict_mode() {
    parse_str("t\nJ1 d g s j\n.model j NJF(LAMBDA=0 RD=0 RS=0 CGS=0 CGD=0 B=1)\n.end\n").unwrap();
    parse_str("t\nM1 d g s s n\n.model n NMOS(LEVEL=1 RD=0 RS=0)\n.end\n").unwrap();
}

#[test]
fn wrong_model_type_is_unsupported() {
    let err = parse_str("t\nM1 d g s s q\n.model q NPN\n.end\n").unwrap_err();
    assert!(matches!(err, SpiceParseError::UnsupportedModelType { .. }));
    let err = parse_str("t\nJ1 d g s n\n.model n NMOS\n.end\n").unwrap_err();
    assert!(matches!(err, SpiceParseError::UnsupportedModelType { .. }));
}

#[test]
fn impossible_geometry_is_an_error_in_both_modes() {
    let deck = "t\nM1 d g s s n L=1u\n.model n NMOS(LD=0.6u)\n.end\n";
    assert!(strict_error(deck).contains("L − 2·LD"));
    let err = parse_str_with_options(deck, ParseOptions::default().with_strict(false));
    assert!(err.is_err());
}

#[test]
fn jfet_maps_beta_and_vto_onto_idss_and_pinch_off() {
    let deck = "t\n\
        J1 d g s nj\n\
        J2 d2 g2 s2 pj 3\n\
        .model nj NJF\n\
        .model pj PJF(VTO=-1.5 BETA=2m)\n\
        .end\n";
    let netlist = parse_str(deck).unwrap();
    let CircuitElement::Jfet {
        nodes,
        kind,
        idss,
        vp,
        ..
    } = find(&netlist.circuit, "j1")
    else {
        panic!()
    };
    assert_eq!(nodes, &["g".to_string(), "d".into(), "s".into()]);
    assert_eq!(*kind, JfetKind::NChannel);
    // SPICE's defaults: VTO = −2 V, BETA = 1e-4 A/V².
    assert!(close(*idss, 1e-4 * 4.0));
    assert!(close(*vp, -2.0));
    let CircuitElement::Jfet { kind, idss, vp, .. } = find(&netlist.circuit, "j2") else {
        panic!()
    };
    assert_eq!(*kind, JfetKind::PChannel);
    assert!(close(*idss, 2e-3 * 1.5 * 1.5 * 3.0));
    // sindr signs a P-channel pinch-off positive.
    assert!(close(*vp, 1.5));
}

/// `CGS` and `CGD` scale with the area factor; `PB` (or `VJ`), `M` and `FC`
/// grade them. A card that leaves the capacitances out has none.
#[test]
fn jfet_card_caps_are_its_gate_junctions() {
    let deck = "t\n\
        J1 d g s nj 2\n\
        J2 d2 g2 s2 pj\n\
        J3 d3 g3 s3 bare\n\
        .model nj NJF(CGS=3p CGD=1p PB=0.7 M=0.33 FC=0.4)\n\
        .model pj PJF(VTO=-1.5 CGD=0.8p)\n\
        .model bare NJF\n\
        .end\n";
    let netlist = parse_str(deck).unwrap();
    let caps = |id: &str| match find(&netlist.circuit, id) {
        CircuitElement::Jfet { parasitic_caps, .. } => *parasitic_caps,
        _ => panic!(),
    };
    let j1 = caps("j1").expect("j1 caps");
    assert!(close(j1.cgs, 6e-12) && close(j1.cgd, 2e-12));
    assert_eq!((j1.pb, j1.m, j1.fc), (0.7, 0.33, 0.4));
    assert_eq!(
        caps("j2"),
        Some(JfetParasiticCaps {
            cgd: 0.8e-12,
            ..Default::default()
        })
    );
    assert_eq!(caps("j3"), None);
}

#[test]
fn jfet_vto_must_be_negative() {
    assert!(strict_error("t\nJ1 d g s j\n.model j NJF(VTO=1)\n.end\n").contains("VTO"));
}

// ------------------------------------------------------------- export

fn round_trip(element: CircuitElement) -> Circuit {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![element],
        ..Default::default()
    };
    let deck = to_netlist(&circuit, "fets").unwrap();
    parse_str(&deck)
        .unwrap_or_else(|e| panic!("{e}\n{deck}"))
        .circuit
}

#[test]
fn mosfet_round_trips_exactly() {
    let original = CircuitElement::Mosfet {
        id: "m1".into(),
        nodes: ["g".into(), "d".into(), "s".into()],
        kind: MosfetKind::Pmos,
        params: MosfetParams {
            kp: 3.7e-3,
            vto: -1.9,
            lambda: 0.013,
            ..MosfetParams::default_pmos()
        },
        parasitic_caps: Some(MosfetParasiticCaps {
            cgs: 1.1e-9,
            cgd: 2.3e-10,
        }),
        body_diode: Some(DiodeModel {
            is: 2e-13,
            cjo: 4e-10,
            vj: 0.75,
            m: 0.4,
            fc: 0.45,
            ..DiodeModel::default()
        }),
    };
    let back = round_trip(original.clone());
    assert_eq!(find(&back, "m1"), &original);
}

#[test]
fn mosfet_without_caps_or_body_diode_round_trips() {
    let original = CircuitElement::Mosfet {
        id: "m1".into(),
        nodes: ["g".into(), "d".into(), "0".into()],
        kind: MosfetKind::Nmos,
        params: MosfetParams::default_nmos(),
        parasitic_caps: None,
        body_diode: None,
    };
    assert_eq!(find(&round_trip(original.clone()), "m1"), &original);
}

#[test]
fn body_diode_spice_junctions_lack_becomes_its_own_diode() {
    let body = DiodeModel {
        is: 1e-12,
        n: 1.3,
        rs: 0.02,
        bv: Some(100.0),
        ..DiodeModel::default()
    };
    let back = round_trip(CircuitElement::Mosfet {
        id: "m1".into(),
        nodes: ["g".into(), "d".into(), "s".into()],
        kind: MosfetKind::Nmos,
        params: MosfetParams::default_nmos(),
        parasitic_caps: None,
        body_diode: Some(body),
    });
    let CircuitElement::Mosfet { body_diode, .. } = find(&back, "m1") else {
        panic!()
    };
    assert_eq!(*body_diode, None);
    // Anode at the source, cathode at the drain, for an NMOS.
    let CircuitElement::Diode { nodes, model, .. } = find(&back, "dm1_body") else {
        panic!()
    };
    assert_eq!(nodes, &["s".to_string(), "d".into()]);
    assert_eq!(*model, Some(body));
}

#[test]
fn mosfet_flicker_noise_isnt_written() {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![CircuitElement::Mosfet {
            id: "m1".into(),
            nodes: ["g".into(), "d".into(), "0".into()],
            kind: MosfetKind::Nmos,
            params: MosfetParams {
                kf: 1e-25,
                ..MosfetParams::default_nmos()
            },
            parasitic_caps: None,
            body_diode: None,
        }],
        ..Default::default()
    };
    assert!(matches!(
        to_netlist(&circuit, "t"),
        Err(ExportError::Unsupported { .. })
    ));
}

#[test]
fn jfets_round_trip() {
    for (kind, vp, idss, want_vp) in [
        (JfetKind::NChannel, -3.1, 7e-3, -3.1),
        (JfetKind::PChannel, 1.7, 4e-3, 1.7),
        // sindr's P-channel reads either sign of `vp` as |vp|.
        (JfetKind::PChannel, -1.7, 4e-3, 1.7),
    ] {
        let back = round_trip(CircuitElement::Jfet {
            id: "j1".into(),
            nodes: ["g".into(), "d".into(), "s".into()],
            kind,
            idss,
            vp,
            parasitic_caps: None,
        });
        assert!(matches!(
            find(&back, "j1"),
            CircuitElement::Jfet {
                parasitic_caps: None,
                ..
            }
        ));
        let CircuitElement::Jfet {
            kind: k,
            idss: i,
            vp: v,
            ..
        } = find(&back, "j1")
        else {
            panic!()
        };
        assert_eq!(*k, kind);
        assert!(close(*i, idss) && close(*v, want_vp), "{kind:?} {vp}");
    }
}

#[test]
fn jfet_gate_caps_round_trip() {
    for m in [0.5, 0.33] {
        let caps = JfetParasiticCaps {
            cgs: 4.7e-12,
            cgd: 1.2e-12,
            pb: 0.75,
            m,
            fc: 0.3,
        };
        let back = round_trip(CircuitElement::Jfet {
            id: "j1".into(),
            nodes: ["g".into(), "d".into(), "s".into()],
            kind: JfetKind::PChannel,
            idss: 4e-3,
            vp: 1.7,
            parasitic_caps: Some(caps),
        });
        let CircuitElement::Jfet {
            parasitic_caps: Some(got),
            ..
        } = find(&back, "j1")
        else {
            panic!("no caps after the round trip")
        };
        assert!(close(got.cgs, caps.cgs) && close(got.cgd, caps.cgd));
        assert_eq!((got.pb, got.m, got.fc), (caps.pb, caps.m, caps.fc));
    }
}

#[test]
fn n_channel_jfet_pinching_off_above_zero_isnt_written() {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![CircuitElement::Jfet {
            id: "j1".into(),
            nodes: ["g".into(), "d".into(), "0".into()],
            kind: JfetKind::NChannel,
            idss: 1e-3,
            vp: 2.0,
            parasitic_caps: None,
        }],
        ..Default::default()
    };
    assert!(matches!(
        to_netlist(&circuit, "t"),
        Err(ExportError::Unsupported { .. })
    ));
}
