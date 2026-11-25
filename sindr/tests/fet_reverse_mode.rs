//! A FET's channel is symmetric: whichever end sits lower acts as the
//! source, so a part wired with drain and source the other way round
//! conducts as SPICE's reverse mode has it, rather than staying off.

use sindr::element::{Jfet, Mosfet, Resistor, VoltageSource};
use sindr::{solve_circuit, Circuit, CircuitElement, JfetKind, MosfetKind, SimulationResult};
use sindr_devices::mosfet::MosfetParams;

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn v(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    })
}

fn mosfet(kind: MosfetKind, params: &MosfetParams, [g, d, s]: [&str; 3]) -> CircuitElement {
    CircuitElement::Mosfet(Mosfet {
        id: "m1".into(),
        nodes: [g.into(), d.into(), s.into()],
        kind,
        params: params.clone(),
        parasitic_caps: None,
        body_diode: None,
    })
}

fn jfet(kind: JfetKind, vp: f64, [g, d, s]: [&str; 3]) -> CircuitElement {
    CircuitElement::Jfet(Jfet {
        id: "j1".into(),
        nodes: [g.into(), d.into(), s.into()],
        kind,
        idss: 10e-3,
        vp,
        parasitic_caps: None,
    })
}

fn current(result: &SimulationResult, id: &str) -> f64 {
    result
        .component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("no result for {id}"))
        .current_through
}

/// The node `hi` pulled to `rail` through 10 kΩ, with `fet` from `hi` to
/// the other end of the channel held at `low`.
fn pass_gate(rail: f64, gate: f64, low: f64, fet: CircuitElement) -> f64 {
    pulled(rail, 10e3, gate, low, fet)
}

/// [`pass_gate`] through `ohms`.
fn pulled(rail: f64, ohms: f64, gate: f64, low: f64, fet: CircuitElement) -> f64 {
    let circuit = Circuit::new(
        "0",
        vec![
            v("vrail", "rail", "0", rail),
            v("vgate", "g", "0", gate),
            v("vlow", "low", "0", low),
            r("rpull", "rail", "hi", ohms),
            fet,
        ],
    );
    solve_circuit(&circuit).unwrap().node_voltages["hi"]
}

/// An NMOS pass gate, gate at 3.3 V, one end at 0 V and the other pulled
/// up to 3.3 V through 10 kΩ, settles where the triode current matches the
/// pull-up's, whichever end is labelled the drain. Before reverse mode the
/// drain-at-0-V wiring left the channel off and the node at 3.3 V.
#[test]
fn nmos_pass_gate_conducts_either_way_round() {
    // No body effect: the body is tied to the labelled source, so swapping
    // the labels would also move the body.
    let p = MosfetParams {
        gamma: 0.0,
        ..MosfetParams::default_nmos()
    };
    let forward = pass_gate(
        3.3,
        3.3,
        0.0,
        mosfet(MosfetKind::Nmos, &p, ["g", "hi", "low"]),
    );
    let reverse = pass_gate(
        3.3,
        3.3,
        0.0,
        mosfet(MosfetKind::Nmos, &p, ["g", "low", "hi"]),
    );
    // Triode: (3.3 − V)/10 kΩ = KP·(2.6·V − V²/2)·(1 + λ·V).
    let excess = |x: f64| p.kp * (2.6 * x - x * x / 2.0) * (1.0 + p.lambda * x) - (3.3 - x) / 10e3;
    let (mut lo, mut hi) = (0.0, 2.6);
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if excess(mid) > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    assert!((forward - lo).abs() < 1e-6, "forward {forward} vs {lo}");
    assert!((reverse - forward).abs() < 1e-9, "{reverse} vs {forward}");
}

/// With the default model's body effect the reversed part's body sits at
/// the upper end, a little forward-biased, which lowers its threshold: it
/// conducts a touch harder, but both wirings pull the node well down.
#[test]
fn nmos_pass_gate_with_body_effect_conducts_either_way_round() {
    let p = MosfetParams::default_nmos();
    let forward = pass_gate(
        3.3,
        3.3,
        0.0,
        mosfet(MosfetKind::Nmos, &p, ["g", "hi", "low"]),
    );
    let reverse = pass_gate(
        3.3,
        3.3,
        0.0,
        mosfet(MosfetKind::Nmos, &p, ["g", "low", "hi"]),
    );
    assert!(forward > 0.4 && forward < 0.8, "forward {forward}");
    assert!(reverse > 0.4 && reverse <= forward, "reverse {reverse}");
}

/// A PMOS pass gate, gate at 0 V, one end at 3.3 V and the other pulled
/// down to 0 V: mirrored, the same either way round.
#[test]
fn pmos_pass_gate_conducts_either_way_round() {
    let p = MosfetParams {
        gamma: 0.0,
        ..MosfetParams::default_pmos()
    };
    let forward = pass_gate(
        0.0,
        0.0,
        3.3,
        mosfet(MosfetKind::Pmos, &p, ["g", "hi", "low"]),
    );
    let reverse = pass_gate(
        0.0,
        0.0,
        3.3,
        mosfet(MosfetKind::Pmos, &p, ["g", "low", "hi"]),
    );
    assert!(forward > 2.0, "forward {forward}");
    assert!((reverse - forward).abs() < 1e-9, "{reverse} vs {forward}");
}

/// An N-channel JFET with its gate and one end on ground and the other end
/// pulled to 12 V through 500 Ω: Vgs = 0 against the grounded end, so it
/// saturates at Idss whichever end is labelled the source, and the node
/// sits at 12 V − 500 Ω·10 mA. Before reverse mode the source-on-top
/// wiring saw Vgs = −V(hi), past pinch-off, and stayed off at 12 V.
#[test]
fn jfet_conducts_either_way_round() {
    let n = |nodes| pulled(12.0, 500.0, 0.0, 0.0, jfet(JfetKind::NChannel, -2.0, nodes));
    let (forward, reverse) = (n(["g", "hi", "low"]), n(["g", "low", "hi"]));
    assert!((forward - 7.0).abs() < 1e-6, "forward {forward}");
    assert!((reverse - forward).abs() < 1e-9, "{reverse} vs {forward}");

    let p = |nodes| pulled(-12.0, 500.0, 0.0, 0.0, jfet(JfetKind::PChannel, 2.0, nodes));
    let (p_forward, p_reverse) = (p(["g", "hi", "low"]), p(["g", "low", "hi"]));
    assert!((p_forward + 7.0).abs() < 1e-6, "p-channel {p_forward}");
    assert!(
        (p_reverse - p_forward).abs() < 1e-9,
        "{p_reverse} vs {p_forward}"
    );
}

/// In the ohmic region too: pulled up through 10 kΩ the node settles where
/// Idss/Vp²·(2·2·V − V²) matches the pull-up's current.
#[test]
fn ohmic_jfet_conducts_either_way_round() {
    let n = |nodes| pass_gate(12.0, 0.0, 0.0, jfet(JfetKind::NChannel, -2.0, nodes));
    let (forward, reverse) = (n(["g", "hi", "low"]), n(["g", "low", "hi"]));
    let excess = |x: f64| 10e-3 / 4.0 * (4.0 * x - x * x) - (12.0 - x) / 10e3;
    let (mut lo, mut hi) = (0.0, 2.0);
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if excess(mid) > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    assert!((forward - lo).abs() < 1e-6, "forward {forward} vs {lo}");
    assert!((reverse - forward).abs() < 1e-9, "{reverse} vs {forward}");
}

/// The current a FET reports with gate, drain and source held at `[vg, va,
/// vb]`, labelled `[g, a, b]` by `fet`.
fn held(volts: [f64; 3], fet: CircuitElement) -> f64 {
    let is_jfet = matches!(fet, CircuitElement::Jfet(_));
    let circuit = Circuit::new(
        "0",
        vec![
            v("vg", "g", "0", volts[0]),
            v("va", "a", "0", volts[1]),
            v("vb", "b", "0", volts[2]),
            fet,
        ],
    );
    let result = solve_circuit(&circuit).unwrap();
    current(&result, if is_jfet { "j1" } else { "m1" })
}

/// Swapping a biased FET's drain and source labels negates the drain
/// current it reports, in every region and for every channel type.
#[test]
fn swapping_drain_and_source_negates_the_current() {
    let nmos = MosfetParams {
        gamma: 0.0,
        ..MosfetParams::default_nmos()
    };
    let pmos = MosfetParams {
        gamma: 0.0,
        ..MosfetParams::default_pmos()
    };
    type Fet = fn(&MosfetParams, [&str; 3]) -> CircuitElement;
    let cases: [(&str, Fet, &MosfetParams, [f64; 3]); 10] = [
        (
            "nmos sat",
            |p, n| mosfet(MosfetKind::Nmos, p, n),
            &nmos,
            [3.0, 5.0, 0.0],
        ),
        (
            "nmos triode",
            |p, n| mosfet(MosfetKind::Nmos, p, n),
            &nmos,
            [3.0, 0.4, 0.0],
        ),
        (
            "nmos on via vgd",
            |p, n| mosfet(MosfetKind::Nmos, p, n),
            &nmos,
            [2.0, 0.0, 3.0],
        ),
        (
            "nmos off",
            |p, n| mosfet(MosfetKind::Nmos, p, n),
            &nmos,
            [0.0, 2.0, 1.0],
        ),
        (
            "pmos sat",
            |p, n| mosfet(MosfetKind::Pmos, p, n),
            &pmos,
            [0.0, 0.0, 5.0],
        ),
        (
            "pmos triode",
            |p, n| mosfet(MosfetKind::Pmos, p, n),
            &pmos,
            [0.0, 4.6, 5.0],
        ),
        (
            "pmos on via vgd",
            |p, n| mosfet(MosfetKind::Pmos, p, n),
            &pmos,
            [1.0, 3.0, 0.0],
        ),
        (
            "njf ohmic",
            |_, n| jfet(JfetKind::NChannel, -2.0, n),
            &nmos,
            [0.0, 0.5, 0.0],
        ),
        (
            "njf sat",
            |_, n| jfet(JfetKind::NChannel, -2.0, n),
            &nmos,
            [-0.5, 5.0, 0.0],
        ),
        (
            "pjf on via vgd",
            |_, n| jfet(JfetKind::PChannel, 2.0, n),
            &nmos,
            [0.0, -3.0, 0.0],
        ),
    ];
    for (name, fet, p, volts) in cases {
        let forward = held(volts, fet(p, ["g", "a", "b"]));
        let reverse = held(volts, fet(p, ["g", "b", "a"]));
        let tol = 1e-12 + 1e-9 * forward.abs();
        assert!(
            (forward + reverse).abs() <= tol,
            "{name}: {forward} vs {reverse}"
        );
    }
}
