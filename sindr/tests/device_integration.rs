//! Solve-level integration tests for circuit elements whose MNA stamping was
//! previously only unit-tested or untested end-to-end: the four controlled
//! sources, the ideal voltage regulator, fuse, switch, pushbutton,
//! potentiometer, photoresistor, thermistor and comparator.
//!
//! Each test builds a small circuit, runs `solve_circuit`, and checks the
//! element's defining behaviour against hand-computed values. None of these
//! has an ngspice primitive to golden against, so each closed form is
//! written out above its test. Where a current
//! direction depends on MNA branch-sign conventions the assertion is on the
//! magnitude, which is what the device's defining equation pins down.

use approx::assert_relative_eq;
use sindr::{solve_circuit, Circuit, CircuitElement, SimError, SimulationResult};

fn vsrc(id: &str, p: &str, n: &str, voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform: None,
    }
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn node(r: &SimulationResult, n: &str) -> f64 {
    *r.node_voltages
        .get(n)
        .unwrap_or_else(|| panic!("no node `{n}` in solution"))
}

fn comp_current(r: &SimulationResult, id: &str) -> f64 {
    r.component_results
        .iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("no component `{id}` in results"))
        .current_through
}

// ----- Controlled sources -------------------------------------------------

#[test]
fn vcvs_output_is_gain_times_control_voltage() {
    // V(c) = 2 V, gain = 4  ->  V(out) = 8 V.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "c", "0", 2.0),
            CircuitElement::Vcvs {
                id: "E1".into(),
                nodes: ["out".into(), "0".into()],
                control_nodes: ["c".into(), "0".into()],
                gain: 4.0,
            },
            resistor("Rload", "out", "0", 1_000.0),
        ],
        ..Default::default()
    };
    let r = solve_circuit(&circuit).expect("solve vcvs");
    assert_relative_eq!(node(&r, "c"), 2.0, epsilon = 1e-9);
    assert_relative_eq!(node(&r, "out"), 8.0, epsilon = 1e-6);
    // Defining law holds on the solved solution.
    assert_relative_eq!(node(&r, "out"), 4.0 * node(&r, "c"), epsilon = 1e-6);
}

#[test]
fn vccs_load_current_is_gm_times_control_voltage() {
    // I_out = gm * V(c) = 0.002 S * 2 V = 4 mA, across a 500 Ω load -> 2 V.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "c", "0", 2.0),
            CircuitElement::Vccs {
                id: "G1".into(),
                nodes: ["out".into(), "0".into()],
                control_nodes: ["c".into(), "0".into()],
                gm: 0.002,
            },
            resistor("Rload", "out", "0", 500.0),
        ],
        ..Default::default()
    };
    let r = solve_circuit(&circuit).expect("solve vccs");
    assert_relative_eq!(comp_current(&r, "Rload").abs(), 0.002 * 2.0, epsilon = 1e-6);
    assert_relative_eq!(node(&r, "out").abs(), 0.002 * 2.0 * 500.0, epsilon = 1e-6);
}

#[test]
fn ccvs_output_is_rm_times_control_current() {
    // Controlling source V1 carries 1 V / 100 Ω = 10 mA. rm = 200 Ω -> |V_out| = 2 V.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "c", "0", 1.0),
            resistor("Rsense", "c", "0", 100.0),
            CircuitElement::Ccvs {
                id: "H1".into(),
                nodes: ["out".into(), "0".into()],
                control_source: "V1".into(),
                rm: 200.0,
            },
            resistor("Rload", "out", "0", 1_000.0),
        ],
        ..Default::default()
    };
    let r = solve_circuit(&circuit).expect("solve ccvs");
    let i_ctrl = r.branch_currents["V1"].abs();
    assert_relative_eq!(i_ctrl, 0.01, epsilon = 1e-6);
    assert_relative_eq!(node(&r, "out").abs(), 200.0 * i_ctrl, epsilon = 1e-6);
}

#[test]
fn cccs_output_current_is_alpha_times_control_current() {
    // I_ctrl = 10 mA, alpha = 5 -> I_out = 50 mA across a 40 Ω load -> |V_out| = 2 V.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "c", "0", 1.0),
            resistor("Rsense", "c", "0", 100.0),
            CircuitElement::Cccs {
                id: "F1".into(),
                nodes: ["out".into(), "0".into()],
                control_source: "V1".into(),
                alpha: 5.0,
            },
            resistor("Rload", "out", "0", 40.0),
        ],
        ..Default::default()
    };
    let r = solve_circuit(&circuit).expect("solve cccs");
    assert_relative_eq!(comp_current(&r, "Rload").abs(), 5.0 * 0.01, epsilon = 1e-6);
    assert_relative_eq!(node(&r, "out").abs(), 5.0 * 0.01 * 40.0, epsilon = 1e-6);
}

// ----- Voltage regulator --------------------------------------------------

#[test]
fn voltage_regulator_holds_output_independent_of_input() {
    let make = |vin: f64| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("Vin", "in", "0", vin),
            CircuitElement::VoltageRegulator {
                id: "U1".into(),
                nodes: ["in".into(), "out".into(), "0".into()],
                voltage: 5.0,
                model: None,
            },
            resistor("Rload", "out", "0", 1_000.0),
        ],
        ..Default::default()
    };
    for vin in [9.0, 12.0, 15.0] {
        let r = solve_circuit(&make(vin)).expect("solve regulator");
        assert_relative_eq!(node(&r, "out"), 5.0, epsilon = 1e-6);
    }
}

// ----- Switch -------------------------------------------------------------

#[test]
fn switch_closed_shorts_open_blocks() {
    // 10 V -> 1 kΩ -> node `n` -> switch -> gnd.
    let make = |closed: bool| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            resistor("R1", "t", "n", 1_000.0),
            CircuitElement::Switch {
                id: "S1".into(),
                nodes: ["n".into(), "0".into()],
                closed,
            },
        ],
        ..Default::default()
    };
    let closed = solve_circuit(&make(true)).expect("closed");
    let open = solve_circuit(&make(false)).expect("open");
    assert!(
        node(&closed, "n") < 0.01,
        "closed switch should pull `n` near 0 V, got {}",
        node(&closed, "n")
    );
    assert!(
        node(&open, "n") > 9.99,
        "open switch should leave `n` near the 10 V supply, got {}",
        node(&open, "n")
    );
}

// ----- Fuse ---------------------------------------------------------------

#[test]
fn fuse_intact_conducts_blown_opens() {
    let make = |blown: bool| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            resistor("R1", "t", "n", 1_000.0),
            CircuitElement::Fuse {
                id: "F1".into(),
                nodes: ["n".into(), "0".into()],
                rating: 1.0,
                blown,
            },
        ],
        ..Default::default()
    };
    let intact = solve_circuit(&make(false)).expect("intact");
    let blown = solve_circuit(&make(true)).expect("blown");
    assert!(
        node(&intact, "n") < 0.02,
        "intact fuse should pull `n` near 0 V, got {}",
        node(&intact, "n")
    );
    assert!(
        node(&blown, "n") > 9.99,
        "blown fuse should leave `n` near the 10 V supply, got {}",
        node(&blown, "n")
    );
}

// ----- Potentiometer ------------------------------------------------------

#[test]
fn potentiometer_loaded_wiper_divides_voltage() {
    // top = 10 V, bottom = gnd, R = 1 kΩ, with a 750 Ω load on the wiper.
    // At position p the upper half is R*p (top→wiper) and the lower half is
    // R*(1-p) (wiper→gnd), which sits in parallel with the load.
    //   p = 0.25: r_top = 250, r_bot = 750 ∥ 750 = 375  -> V_w = 10·375/625 = 6.00 V
    //   p = 0.50: r_top = 500, r_bot = 500 ∥ 750 = 300  -> V_w = 10·300/800 = 3.75 V
    let make = |pos: f64| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            CircuitElement::Potentiometer {
                id: "P1".into(),
                nodes: ["t".into(), "w".into(), "0".into()],
                resistance: 1_000.0,
                position: pos,
            },
            resistor("Rload", "w", "0", 750.0),
        ],
        ..Default::default()
    };
    let r25 = solve_circuit(&make(0.25)).expect("p=0.25");
    let r50 = solve_circuit(&make(0.50)).expect("p=0.50");
    assert_relative_eq!(node(&r25, "w"), 6.0, epsilon = 1e-6);
    assert_relative_eq!(node(&r50, "w"), 3.75, epsilon = 1e-6);
}

#[test]
fn potentiometer_unloaded_wiper_is_rejected_as_floating() {
    // Known limitation: validation counts component-terminal incidences per
    // node, so a potentiometer whose wiper has no external connection looks
    // like a degree-1 (floating) node — even though the wiper is internally
    // connected to both ends through the two resistor halves and the circuit
    // is solvable. Locked in as a regression test; if validation learns about
    // a pot's internal wiper connection this should flip to a solve.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            CircuitElement::Potentiometer {
                id: "P1".into(),
                nodes: ["t".into(), "w".into(), "0".into()],
                resistance: 1_000.0,
                position: 0.5,
            },
        ],
        ..Default::default()
    };
    match solve_circuit(&circuit) {
        Err(SimError::FloatingNode(node)) => assert_eq!(node, "w"),
        other => panic!("expected FloatingNode(\"w\"), got {other:?}"),
    }
}

/// Self-biased JFET stage. The P-channel version is the N-channel one with
/// every voltage mirrored about the rail, so its node voltages must mirror
/// too and its drain current must flip sign.
fn jfet_self_bias(kind: sindr::JfetKind) -> Circuit {
    let (vp, rail) = match kind {
        sindr::JfetKind::NChannel => (-2.0, 12.0),
        sindr::JfetKind::PChannel => (2.0, -12.0),
    };
    Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "rail", "0", rail),
            resistor("RD", "rail", "d", 1_000.0),
            resistor("RS", "s", "0", 220.0),
            resistor("RG", "g", "0", 1e6),
            CircuitElement::Jfet {
                id: "J1".into(),
                nodes: ["g".into(), "d".into(), "s".into()],
                kind,
                idss: 10e-3,
                vp,
            },
        ],
        ..Default::default()
    }
}

#[test]
fn p_channel_jfet_mirrors_n_channel() {
    let n = solve_circuit(&jfet_self_bias(sindr::JfetKind::NChannel)).unwrap();
    let p = solve_circuit(&jfet_self_bias(sindr::JfetKind::PChannel)).unwrap();
    for name in ["d", "s", "g"] {
        assert_relative_eq!(node(&p, name), -node(&n, name), epsilon = 1e-9);
    }
    assert_relative_eq!(
        comp_current(&p, "J1"),
        -comp_current(&n, "J1"),
        epsilon = 1e-12
    );
    // N-channel matches ngspice (tests/golden/op_jfet): Vs = 0.79656 V.
    assert_relative_eq!(node(&n, "s"), 0.79656, epsilon = 1e-4);
}

#[test]
fn p_channel_jfet_mirrors_n_channel_in_transient() {
    let with_cap = |kind| {
        let mut c = jfet_self_bias(kind);
        c.components.push(CircuitElement::Capacitor {
            id: "CS".into(),
            nodes: ["s".into(), "0".into()],
            capacitance: 1e-6,
        });
        solve_circuit(&c).unwrap().transient.unwrap()
    };
    let n = with_cap(sindr::JfetKind::NChannel);
    let p = with_cap(sindr::JfetKind::PChannel);
    assert_eq!(n.timesteps.len(), p.timesteps.len());
    for (sn, sp) in n.timesteps.iter().zip(&p.timesteps) {
        for name in ["d", "s"] {
            assert_relative_eq!(
                sp.node_voltages[name],
                -sn.node_voltages[name],
                epsilon = 1e-9
            );
        }
    }
}

fn mosfet(kind: sindr::MosfetKind, params: sindr_devices::mosfet::MosfetParams) -> CircuitElement {
    CircuitElement::Mosfet {
        id: "M1".into(),
        nodes: ["g".into(), "d".into(), "s".into()],
        kind,
        params,
        parasitic_caps: None,
    }
}

#[test]
fn pmos_with_default_params_is_off_at_zero_vsg() {
    // Gate and source both at VDD, drain to ground through 10 kΩ.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "vdd", "0", 10.0),
            resistor("RG", "vdd", "g", 1_000.0),
            resistor("RS", "vdd", "s", 1.0),
            resistor("RD", "d", "0", 10_000.0),
            mosfet(
                sindr::MosfetKind::Pmos,
                sindr_devices::mosfet::MosfetParams::default_pmos(),
            ),
        ],
        ..Default::default()
    };
    let r = solve_circuit(&circuit).unwrap();
    assert!(node(&r, "d").abs() < 1e-6, "Vd = {}", node(&r, "d"));
}

/// Common-source stage with its source at ground, so the body (also at
/// ground) plays no part. The PMOS version, with SPICE-signed VTO and every
/// voltage mirrored, must mirror the NMOS one exactly.
#[test]
fn pmos_stage_mirrors_nmos_stage() {
    use sindr_devices::mosfet::MosfetParams;
    let stage = |kind, params, sign: f64| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("VDD", "vdd", "0", sign * 10.0),
            vsrc("VG", "g", "0", sign * 2.0),
            resistor("RD", "vdd", "d", 20_000.0),
            CircuitElement::Resistor {
                id: "RS".into(),
                nodes: ["s".into(), "0".into()],
                resistance: 1e-3,
            },
            mosfet(kind, params),
        ],
        ..Default::default()
    };
    let nmos = MosfetParams::default_nmos();
    let pmos = MosfetParams {
        vto: -nmos.vto,
        ..nmos.clone()
    };
    let n = solve_circuit(&stage(sindr::MosfetKind::Nmos, nmos, 1.0)).unwrap();
    let p = solve_circuit(&stage(sindr::MosfetKind::Pmos, pmos, -1.0)).unwrap();
    assert!(
        node(&n, "d") < 9.0,
        "NMOS should conduct: Vd = {}",
        node(&n, "d")
    );
    for name in ["d", "g"] {
        assert_relative_eq!(node(&p, name), -node(&n, name), epsilon = 1e-9);
    }
}

// ----- Pushbutton ---------------------------------------------------------
// Closed form: a pressed button is the switch's 0.01 Ω, a released one its
// 1 GΩ, so the divider below sits at 10·0.01/1000.01 V or at 10·1e9/(1e9+1e3).

#[test]
fn pushbutton_pressed_shorts_released_blocks() {
    let make = |closed: bool| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 10.0),
            resistor("R1", "t", "n", 1_000.0),
            CircuitElement::Pushbutton {
                id: "B1".into(),
                nodes: ["n".into(), "0".into()],
                closed,
            },
        ],
        ..Default::default()
    };
    let pressed = solve_circuit(&make(true)).expect("pressed");
    let released = solve_circuit(&make(false)).expect("released");
    assert_relative_eq!(node(&pressed, "n"), 10.0 * 0.01 / 1_000.01, epsilon = 1e-9);
    assert_relative_eq!(
        node(&released, "n"),
        10.0 * 1e9 / (1e9 + 1e3),
        epsilon = 1e-6
    );
    assert_relative_eq!(
        comp_current(&pressed, "B1").abs(),
        10.0 / 1_000.01,
        epsilon = 1e-9
    );
}

// ----- Photoresistor ------------------------------------------------------
// Closed form: R = r_dark·(r_bright/r_dark)^light with the defaults 1 MΩ and
// 1 kΩ, so 1 MΩ dark, 1 kΩ bright and √(1e6·1e3) ≈ 31.6 kΩ at half light.

#[test]
fn photoresistor_divider_follows_light_level() {
    let make = |light_level: f64| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 5.0),
            resistor("R1", "t", "out", 10_000.0),
            CircuitElement::Photoresistor {
                id: "LDR1".into(),
                nodes: ["out".into(), "0".into()],
                light_level,
            },
        ],
        ..Default::default()
    };
    for (light, r_ldr) in [(0.0, 1e6), (0.5, (1e6_f64 * 1e3).sqrt()), (1.0, 1e3)] {
        let r = solve_circuit(&make(light)).expect("solves");
        assert_relative_eq!(
            node(&r, "out"),
            5.0 * r_ldr / (r_ldr + 10_000.0),
            max_relative = 1e-9
        );
    }
}

// ----- Thermistor ---------------------------------------------------------
// Closed form: the built-in NTC is 10 kΩ at 25 °C with β = 3950 K, so
// R(T) = 10k·exp(3950·(1/T − 1/298.15)). A 10 kΩ top resistor halves 5 V
// at 25 °C, and the divider rises as the thermistor cools.

#[test]
fn thermistor_divider_follows_temperature() {
    let make = |temperature: f64| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("V1", "t", "0", 5.0),
            resistor("R1", "t", "out", 10_000.0),
            CircuitElement::Thermistor {
                id: "TH1".into(),
                nodes: ["out".into(), "0".into()],
                temperature,
            },
        ],
        ..Default::default()
    };
    let mut last = f64::INFINITY;
    for t in [273.15, 298.15, 323.15, 373.15] {
        let r_th = 10_000.0 * (3950.0 * (1.0 / t - 1.0 / 298.15_f64)).exp();
        let r = solve_circuit(&make(t)).expect("solves");
        let v = node(&r, "out");
        assert_relative_eq!(v, 5.0 * r_th / (r_th + 10_000.0), max_relative = 1e-9);
        assert!(v < last, "an NTC divider falls as it warms: {v} at {t} K");
        last = v;
    }
    let r = solve_circuit(&make(298.15)).expect("solves");
    assert_relative_eq!(node(&r, "out"), 2.5, epsilon = 1e-9);
}

// ----- Comparator ---------------------------------------------------------
// Closed form: the comparator is a VCVS of gain 1e5 from (in+ − in−) to
// out, with no rail clamp (LIMITATIONS), so a 10 µV overdrive gives ±1 V.
// The rails aren't enforced: this pins today's behaviour, not a real part's.

#[test]
fn comparator_is_a_gain_1e5_vcvs() {
    let make = |overdrive: f64| Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("Vref", "ref", "0", 1.0),
            vsrc("Vin", "in", "0", 1.0 + overdrive),
            CircuitElement::Comparator {
                id: "U1".into(),
                nodes: ["in".into(), "ref".into(), "out".into()],
                v_pos: 5.0,
                v_neg: 0.0,
            },
            resistor("Rload", "out", "0", 10_000.0),
        ],
        ..Default::default()
    };
    for overdrive in [10e-6, -10e-6, 0.0] {
        let r = solve_circuit(&make(overdrive)).expect("solves");
        assert_relative_eq!(node(&r, "out"), 1e5 * overdrive, epsilon = 1e-6);
    }
}

/// The same comparator in transient and AC: it has no state, so a pulse
/// on in+ shows on the output at once, and AC gain is flat at 100 dB.
#[test]
fn comparator_in_transient_and_ac() {
    use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
    use sindr::{solve_transient, TransientConfig, Waveform};

    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsrc("Vref", "ref", "0", 0.0),
            CircuitElement::VoltageSource {
                id: "Vin".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Pulse {
                    v1: 0.0,
                    v2: 10e-6,
                    delay: 1e-6,
                    rise_time: 1e-9,
                    fall_time: 1e-9,
                    pulse_width: 1.0,
                    period: 2.0,
                }),
            },
            CircuitElement::Comparator {
                id: "U1".into(),
                nodes: ["in".into(), "ref".into(), "out".into()],
                v_pos: 5.0,
                v_neg: 0.0,
            },
            resistor("Rload", "out", "0", 10_000.0),
        ],
        ..Default::default()
    };
    let tr = solve_transient(&circuit, &TransientConfig::new(5e-6, 1e-7)).expect("transient");
    let steps = &tr.transient.as_ref().expect("transient data").timesteps;
    let out_at = |t: f64| {
        steps
            .iter()
            .min_by(|a, b| (a.time - t).abs().total_cmp(&(b.time - t).abs()))
            .map(|s| s.node_voltages["out"])
            .expect("a step")
    };
    assert_relative_eq!(out_at(0.5e-6), 0.0, epsilon = 1e-9);
    assert_relative_eq!(out_at(4e-6), 1.0, epsilon = 1e-6);

    let ac = solve_ac(
        &circuit,
        &AcConfig {
            f_start: 1.0,
            f_stop: 1e6,
            num_points: 7,
            spacing: FrequencySpacing::Logarithmic,
            source_id: "Vin".into(),
            ac_magnitude: 1.0,
        },
    )
    .expect("ac");
    for point in &ac.points {
        assert_relative_eq!(point.node_voltages["out"].norm(), 1e5, max_relative = 1e-9);
    }
}
