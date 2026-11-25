//! Pole-zero analysis against closed-form poles and zeros.

use num_complex::Complex64;
use sindr::element::{
    Battery, Capacitor, CurrentSource, Diode, Inductor, Jfet, OpAmp, Resistor, VoltageSource,
};
use sindr::pole_zero::{solve_pz, PzConfig, PzFind};
use sindr::{Circuit, CircuitElement, OpAmpModel, SimError};

fn v(id: &str, p: &str, n: &str) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: 0.0,
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

fn c(id: &str, a: &str, b: &str, farads: f64) -> CircuitElement {
    CircuitElement::Capacitor(Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance: farads,
    })
}

fn l(id: &str, a: &str, b: &str, henries: f64) -> CircuitElement {
    CircuitElement::Inductor(Inductor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        inductance: henries,
        saturation: None,
    })
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
        ..Default::default()
    }
}

/// Asserts `got` and `want` are the same roots to `rel` of each root's
/// magnitude, in any order.
fn assert_roots(got: &[Complex64], want: &[Complex64], rel: f64) {
    assert_eq!(got.len(), want.len(), "got {got:?}, want {want:?}");
    let mut left: Vec<Complex64> = got.to_vec();
    for w in want {
        let (i, d) = left
            .iter()
            .enumerate()
            .map(|(i, g)| (i, (g - w).norm()))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();
        assert!(
            d <= rel * w.norm(),
            "root {w} off by {:.3e} (relative), got {got:?}",
            d / w.norm()
        );
        left.remove(i);
    }
}

const R: f64 = 10.0;
const L: f64 = 1e-3;
const C: f64 = 1e-6;

/// V1 → R → a → L → b → C → ground.
fn series_rlc() -> Vec<CircuitElement> {
    vec![
        v("V1", "in", "0"),
        r("R1", "in", "a", R),
        l("L1", "a", "b", L),
        c("C1", "b", "0", C),
    ]
}

/// Roots of s² + 2α·s + ω₀².
fn quadratic(two_alpha: f64, omega0_sq: f64) -> [Complex64; 2] {
    let alpha = two_alpha / 2.0;
    let d = Complex64::new(alpha * alpha - omega0_sq, 0.0).sqrt();
    [-alpha + d, -alpha - d]
}

#[test]
fn series_rlc_across_the_capacitor_is_all_pole() {
    let pz = solve_pz(
        &circuit(series_rlc()),
        &PzConfig::voltage_gain("in", "0", "b", "0"),
    )
    .unwrap();
    // H = 1 / (s²LC + sRC + 1).
    assert_roots(&pz.poles, &quadratic(R / L, 1.0 / (L * C)), 1e-12);
    assert!(pz.zeros.is_empty(), "{:?}", pz.zeros);
    assert!(pz.is_stable());
}

#[test]
fn series_rlc_across_the_tank_has_imaginary_zeros() {
    let pz = solve_pz(
        &circuit(series_rlc()),
        &PzConfig::voltage_gain("in", "0", "a", "0"),
    )
    .unwrap();
    // H = (s²LC + 1) / (s²LC + sRC + 1).
    assert_roots(&pz.poles, &quadratic(R / L, 1.0 / (L * C)), 1e-12);
    let w0 = 1.0 / (L * C).sqrt();
    assert_roots(
        &pz.zeros,
        &[Complex64::new(0.0, w0), Complex64::new(0.0, -w0)],
        1e-12,
    );
}

#[test]
fn voltage_gain_without_a_source_drives_its_own() {
    // No V1: the analysis shorts the input pair itself.
    let mut parts = series_rlc();
    parts[0] = r("Rbias", "in", "0", 1e3);
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::voltage_gain("in", "0", "b", "0"),
    )
    .unwrap();
    assert_roots(&pz.poles, &quadratic(R / L, 1.0 / (L * C)), 1e-12);
}

#[test]
fn transimpedance_into_the_capacitor_node() {
    // V1 shorts `in`, so the current into b sees C ‖ (L + R):
    // Z = (R + sL) / (s²LC + sRC + 1), one zero at −R/L.
    let pz = solve_pz(
        &circuit(series_rlc()),
        &PzConfig::transimpedance("b", "0", "b", "0"),
    )
    .unwrap();
    assert_roots(&pz.poles, &quadratic(R / L, 1.0 / (L * C)), 1e-12);
    assert_roots(&pz.zeros, &[Complex64::new(-R / L, 0.0)], 1e-12);
}

#[test]
fn infinite_eigenvalues_of_higher_index_are_deflated() {
    // A capacitor straight across the source and an inductor in series
    // with a current source each give a Jordan block at infinity. Neither
    // may show up as a spurious finite root.
    let mut parts = series_rlc();
    parts.push(c("Cdec", "in", "0", 1e-3));
    parts.push(CircuitElement::CurrentSource(CurrentSource {
        id: "I1".into(),
        nodes: ["0".into(), "x".into()],
        current: 0.0,
        waveform: None,
    }));
    parts.push(l("L2", "x", "b", 1e-2));
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::voltage_gain("in", "0", "a", "0"),
    )
    .unwrap();
    assert_roots(&pz.poles, &quadratic(R / L, 1.0 / (L * C)), 1e-10);
    let w0 = 1.0 / (L * C).sqrt();
    assert_roots(
        &pz.zeros,
        &[Complex64::new(0.0, w0), Complex64::new(0.0, -w0)],
        1e-10,
    );
}

#[test]
fn rc_ladder_with_widely_spread_poles() {
    // Two RC sections a factor 1e6 apart in time constant: the poles are
    // the roots of s²·R1C1R2C2 + s·(R1C1 + R2C2 + R1C2) + 1.
    let (r1, c1, r2, c2) = (1e3, 1e-6, 10.0, 1e-10);
    let parts = vec![
        v("V1", "in", "0"),
        r("R1", "in", "a", r1),
        c("C1", "a", "0", c1),
        r("R2", "a", "out", r2),
        c("C2", "out", "0", c2),
    ];
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::voltage_gain("in", "0", "out", "0"),
    )
    .unwrap();
    let k = r1 * c1 * r2 * c2;
    let want = quadratic((r1 * c1 + r2 * c2 + r1 * c2) / k, 1.0 / k);
    assert_roots(&pz.poles, &want, 1e-10);
    assert!(pz.zeros.is_empty());
}

#[test]
fn follower_pole_is_the_open_loop_pole_times_one_plus_a0() {
    let model = OpAmpModel {
        gain: 1e5,
        gbw: Some(1e6),
        ..Default::default()
    };
    let tau = model.gain / (2.0 * std::f64::consts::PI * 1e6);
    let parts = vec![
        v("V1", "in", "0"),
        CircuitElement::OpAmp(OpAmp {
            id: "U1".into(),
            nodes: ["in".into(), "out".into(), "out".into()],
            v_pos: 15.0,
            v_neg: -15.0,
            model: Some(model),
        }),
        r("RL", "out", "0", 10e3),
    ];
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::voltage_gain("in", "0", "out", "0"),
    )
    .unwrap();
    assert_roots(&pz.poles, &[Complex64::new(-(1.0 + 1e5) / tau, 0.0)], 1e-12);
    assert!(pz.zeros.is_empty());
}

#[test]
fn find_restricts_the_output() {
    let poles = solve_pz(
        &circuit(series_rlc()),
        &PzConfig::voltage_gain("in", "0", "a", "0").with_find(PzFind::Poles),
    )
    .unwrap();
    assert_eq!((poles.poles.len(), poles.zeros.len()), (2, 0));
    let zeros = solve_pz(
        &circuit(series_rlc()),
        &PzConfig::voltage_gain("in", "0", "a", "0").with_find(PzFind::Zeros),
    )
    .unwrap();
    assert_eq!((zeros.poles.len(), zeros.zeros.len()), (0, 2));
}

#[test]
fn resistive_network_has_no_roots() {
    let parts = vec![
        v("V1", "in", "0"),
        r("R1", "in", "out", 1e3),
        r("R2", "out", "0", 1e3),
    ];
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::voltage_gain("in", "0", "out", "0"),
    )
    .unwrap();
    assert!(pz.poles.is_empty() && pz.zeros.is_empty());
}

#[test]
fn bad_ports_are_rejected() {
    let rlc = circuit(series_rlc());
    for config in [
        PzConfig::voltage_gain("nope", "0", "b", "0"),
        PzConfig::voltage_gain("in", "0", "b", "b"),
    ] {
        assert!(
            matches!(
                solve_pz(&rlc, &config),
                Err(SimError::UnknownNode { .. } | SimError::InvalidConfig { .. })
            ),
            "{config:?}"
        );
    }
    // A current into a node V1 holds at 0 V never reaches the output.
    let err = solve_pz(&rlc, &PzConfig::transimpedance("in", "0", "b", "0")).unwrap_err();
    assert!(matches!(err, SimError::InvalidConfig { .. }), "{err:?}");
}

#[cfg(feature = "spice")]
#[test]
fn pz_card_parses_into_a_config() {
    use sindr::spice::{parse_str, AnalysisRequest};
    use sindr::PzTransfer;

    let deck = "* rlc\nV1 in 0 0\nR1 in a 10\nL1 a b 1m\nC1 b 0 1u\n\
                .pz in 0 a 0 vol pz\n.pz B 0 b 0 CUR zer\n.end\n";
    let netlist = parse_str(deck).unwrap();
    assert_eq!(
        netlist.analyses[0],
        AnalysisRequest::Pz {
            input: "in".into(),
            input_reference: "0".into(),
            output: "a".into(),
            output_reference: "0".into(),
            transfer: PzTransfer::VoltageGain,
            find: PzFind::Both,
        }
    );
    let vol = netlist.analyses[0].pz_config().unwrap();
    assert_eq!(vol, PzConfig::voltage_gain("in", "0", "a", "0"));
    let pz = solve_pz(&netlist.circuit, &vol).unwrap();
    assert_eq!((pz.poles.len(), pz.zeros.len()), (2, 2));

    let cur = netlist.analyses[1].pz_config().unwrap();
    assert_eq!(
        cur,
        PzConfig::transimpedance("b", "0", "b", "0").with_find(PzFind::Zeros)
    );
    let pz = solve_pz(&netlist.circuit, &cur).unwrap();
    assert!(pz.poles.is_empty());
    assert_roots(&pz.zeros, &[Complex64::new(-R / L, 0.0)], 1e-12);

    for bad in [
        ".pz in 0 a vol pz",
        ".pz in 0 a 0 volt pz",
        ".pz in 0 a 0 vol both",
        ".pz in 0 a 0 vol pz extra",
    ] {
        let deck = format!("* rlc\nV1 in 0 0\nR1 in a 10\nC1 a 0 1u\n{bad}\n.end\n");
        assert!(parse_str(&deck).is_err(), "{bad}");
    }
}

/// A common-source JFET stage: VDD through RD to the drain, the gate
/// driven through RG, the source grounded. With `caps`, CGS and CGD.
fn jfet_stage(caps: Option<sindr::circuit::JfetParasiticCaps>) -> Circuit {
    circuit(vec![
        v("vin", "in", "0"),
        CircuitElement::VoltageSource(VoltageSource {
            id: "vdd".into(),
            nodes: ["vdd".into(), "0".into()],
            voltage: 15.0,
            waveform: None,
        }),
        r("rg", "in", "g", 100e3),
        r("rd", "vdd", "d", 1e3),
        CircuitElement::Jfet(Jfet {
            id: "j1".into(),
            nodes: ["g".into(), "d".into(), "0".into()],
            kind: sindr::JfetKind::NChannel,
            idss: 10e-3,
            vp: -2.0,
            parasitic_caps: caps,
        }),
    ])
}

#[test]
fn jfet_gate_capacitances_give_the_stage_its_poles_and_miller_zero() {
    let config = PzConfig::voltage_gain("in", "0", "d", "0");
    // Without capacitances the stage is resistive: no poles, no zeros.
    let flat = solve_pz(&jfet_stage(None), &config).unwrap();
    assert!(flat.poles.is_empty(), "{:?}", flat.poles);
    assert!(flat.zeros.is_empty(), "{:?}", flat.zeros);

    // CGS and CGD (M = 0: constant) make it second order, with CGD's
    // feed-forward zero in the right half-plane at gm/CGD.
    let (cgs, cgd) = (10e-12, 2e-12);
    let caps = sindr::circuit::JfetParasiticCaps {
        cgs,
        cgd,
        m: 0.0,
        ..Default::default()
    };
    let pz = solve_pz(&jfet_stage(Some(caps)), &config).unwrap();
    assert_eq!(pz.poles.len(), 2, "{:?}", pz.poles);
    assert!(pz.is_stable());
    // At Vgs = 0 the channel is saturated with gm = 2·Idss/|Vp|.
    let gm = 2.0 * 10e-3 / 2.0;
    assert_roots(&pz.zeros, &[Complex64::new(gm / cgd, 0.0)], 1e-6);
    // The poles' product is 1/(RG·RD·CGS·CGD), from the characteristic
    // polynomial of the two-node circuit.
    let product = pz.poles[0] * pz.poles[1];
    let want = 1.0 / (100e3 * 1e3 * cgs * cgd);
    assert!(
        (product.re - want).abs() < 1e-6 * want && product.im.abs() < 1e-6 * want,
        "{product} vs {want}"
    );
}

#[test]
fn battery_rc_pair_gives_a_pole_and_a_zero() {
    // A cell R0 + R1‖C1 across RL: Z = RL‖(R0 + R1/(1 + s·R1C1)), driven
    // by a current into its positive terminal. The zero is where the
    // cell's own impedance has its zero, −(R0 + R1)/(R0·R1·C1); the pole
    // is −(RL + R0 + R1)/(R1·C1·(RL + R0)).
    let (r0, r1, c1, rl) = (0.1, 0.2, 50.0, 10.0);
    let mut params = sindr_devices::battery::BatteryParams::constant(3.7, 1.0, r0);
    params.rc = Some(sindr_devices::battery::BatteryRc { r: r1, c: c1 });
    let parts = vec![
        CircuitElement::Battery(Battery {
            id: "B1".into(),
            nodes: ["p".into(), "0".into()],
            params,
            state_of_charge: 0.5,
        }),
        r("RL", "p", "0", rl),
    ];
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::transimpedance("p", "0", "p", "0"),
    )
    .unwrap();
    let pole = -(rl + r0 + r1) / (r1 * c1 * (rl + r0));
    let zero = -(r0 + r1) / (r0 * r1 * c1);
    assert_roots(&pz.poles, &[Complex64::new(pole, 0.0)], 1e-9);
    assert_roots(&pz.zeros, &[Complex64::new(zero, 0.0)], 1e-9);
}

#[test]
fn reverse_biased_diode_charge_sits_behind_its_series_resistance() {
    // V1 → R → a → diode (RS, CJO) → ground, reverse biased at −5 V so the
    // junction is its depletion capacitance Cj = CJO·(1 + 5/VJ)^−M and next
    // to no conductance: H = (1 + s·RS·Cj) / (1 + s·(R + RS)·Cj).
    let (rr, rs, cjo) = (1e3, 10.0, 10e-12);
    let model = sindr::DiodeModel {
        rs,
        cjo,
        ..Default::default()
    };
    let cj = cjo * (1.0 + 5.0 / model.vj).powf(-model.m);
    let parts = vec![
        CircuitElement::VoltageSource(VoltageSource {
            id: "V1".into(),
            nodes: ["in".into(), "0".into()],
            voltage: -5.0,
            waveform: None,
        }),
        r("R1", "in", "a", rr),
        CircuitElement::Diode(Diode {
            id: "D1".into(),
            nodes: ["a".into(), "0".into()],
            temperature: 300.15,
            model: Some(model),
        }),
    ];
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::voltage_gain("in", "0", "a", "0"),
    )
    .unwrap();
    assert_roots(
        &pz.poles,
        &[Complex64::new(-1.0 / ((rr + rs) * cj), 0.0)],
        1e-6,
    );
    assert_roots(&pz.zeros, &[Complex64::new(-1.0 / (rs * cj), 0.0)], 1e-6);
}

/// Two RC low-passes off one input, read differentially:
/// H = 1/(1 + sτ1) − 1/(1 + sτ2) = s(τ2 − τ1) / ((1 + sτ1)(1 + sτ2)),
/// so poles at −1/τ1 and −1/τ2 and a zero at the origin.
#[test]
fn differential_output_has_both_poles_and_a_zero_at_the_origin() {
    let (t1, t2) = (1e3 * 1e-6, 4.7e3 * 1e-6);
    let parts = vec![
        v("V1", "in", "0"),
        r("R1", "in", "a", 1e3),
        c("C1", "a", "0", 1e-6),
        r("R2", "in", "b", 4.7e3),
        c("C2", "b", "0", 1e-6),
    ];
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::voltage_gain("in", "0", "a", "b"),
    )
    .unwrap();
    let mut poles: Vec<f64> = pz.poles.iter().map(|p| p.re).collect();
    poles.sort_by(f64::total_cmp);
    assert_eq!(poles.len(), 2);
    assert!((poles[0] + 1.0 / t1).abs() < 1e-9 / t1, "{poles:?}");
    assert!((poles[1] + 1.0 / t2).abs() < 1e-9 / t2, "{poles:?}");
    assert_eq!(pz.zeros.len(), 1, "{:?}", pz.zeros);
    assert!(pz.zeros[0].norm() < 1e-6, "{:?}", pz.zeros);
}

/// A current driven into `a` and out of `b` across C, with R1 and R2 to
/// ground: Z = (R1 + R2) ‖ 1/(sC), one pole at −1/((R1 + R2)·C).
#[test]
fn transimpedance_between_two_nodes() {
    let parts = vec![
        r("R1", "a", "0", 1e3),
        r("R2", "b", "0", 2e3),
        c("C1", "a", "b", 1e-6),
    ];
    let pz = solve_pz(
        &circuit(parts),
        &PzConfig::transimpedance("a", "b", "a", "b"),
    )
    .unwrap();
    assert_eq!(pz.poles.len(), 1, "{:?}", pz.poles);
    let want = -1.0 / (3e3 * 1e-6);
    assert!(
        (pz.poles[0].re - want).abs() < 1e-9 * want.abs() && pz.poles[0].im == 0.0,
        "{:?}",
        pz.poles
    );
    assert!(pz.zeros.is_empty(), "{:?}", pz.zeros);
}
