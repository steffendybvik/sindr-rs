//! Every analysis evaluates a device with the same temperature-scaled model
//! the DC operating point was solved with.
//!
//! - Reported currents: in a source → resistor → device loop, the device's
//!   reported current must equal the resistor's (KCL), in DC and transient
//!   results alike, at temperatures away from nominal.
//! - AC linearisation: the small-signal gain must match a finite difference of
//!   the DC device model, built here independently from `sindr_devices`.

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::{solve_circuit, BjtKind, Circuit, CircuitElement, SimulationResult};
use sindr_devices::bjt::{bjt_companion, BjtParams};
use sindr_devices::diode::{diode_current, temperature_scale_is, DiodeParams};

const R: f64 = 1000.0;
const TEMPERATURES: [f64; 3] = [250.0, 350.0, 400.0];

fn source(voltage: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: "V1".into(),
        nodes: ["n1".into(), "0".into()],
        voltage,
        waveform: None,
    }
}

fn resistor() -> CircuitElement {
    CircuitElement::Resistor {
        id: "R1".into(),
        nodes: ["n1".into(), "n2".into()],
        resistance: R,
    }
}

/// Every diode-like device at `t`, each as `D1` between `n2` and ground.
/// The zener is reversed so it conducts in breakdown.
fn devices(t: f64) -> Vec<CircuitElement> {
    let fwd = || ["n2".to_string(), "0".to_string()];
    vec![
        CircuitElement::Diode {
            id: "D1".into(),
            nodes: fwd(),
            temperature: t,
        },
        CircuitElement::Led {
            id: "D1".into(),
            nodes: fwd(),
            color: "red".into(),
            temperature: t,
        },
        CircuitElement::ZenerDiode {
            id: "D1".into(),
            nodes: fwd(),
            vz: 3.3,
            temperature: t,
        },
        CircuitElement::ZenerDiode {
            id: "D1".into(),
            nodes: ["0".into(), "n2".into()],
            vz: 3.3,
            temperature: t,
        },
        CircuitElement::SchottkyDiode {
            id: "D1".into(),
            nodes: fwd(),
            temperature: t,
        },
        CircuitElement::Photodiode {
            id: "D1".into(),
            nodes: fwd(),
            irradiance: 0.0,
            temperature: t,
        },
    ]
}

/// Current of `id` from n2 towards ground, whichever way round its nodes are.
fn current_to_ground(components: &[sindr::ComponentResult], device: &CircuitElement) -> f64 {
    let i = components
        .iter()
        .find(|c| c.id == "D1")
        .expect("device result")
        .current_through;
    if device.nodes()[0] == "n2" {
        i
    } else {
        -i
    }
}

fn assert_kcl(components: &[sindr::ComponentResult], device: &CircuitElement, what: &str) {
    let i_r = components
        .iter()
        .find(|c| c.id == "R1")
        .unwrap()
        .current_through;
    let i_d = current_to_ground(components, device);
    assert!(
        i_r > 1e-4,
        "{what}: loop should conduct, resistor current {i_r} A"
    );
    assert!(
        // Newton-Raphson stops at a relative voltage tolerance, which leaves
        // up to ~1e-3 relative KCL residual on a steep junction. A device
        // evaluated with the wrong parameters is off by orders of magnitude.
        (i_r - i_d).abs() <= 1e-2 * i_r,
        "{what}: resistor carries {i_r} A but device reports {i_d} A"
    );
}

#[test]
fn dc_reported_device_current_obeys_kcl_off_nominal_temperature() {
    for t in TEMPERATURES {
        for device in devices(t) {
            // The Schottky and photodiode don't converge in DC behind 5 V / 1 kΩ
            // at any temperature yet (SIN-16); the transient test still covers them.
            if matches!(
                device,
                CircuitElement::SchottkyDiode { .. } | CircuitElement::Photodiode { .. }
            ) {
                continue;
            }
            let circuit = Circuit {
                ground_node: "0".into(),
                components: vec![source(5.0), resistor(), device.clone()],
            };
            let result =
                solve_circuit(&circuit).unwrap_or_else(|e| panic!("DC solve {device:?}: {e:?}"));
            assert_kcl(
                &result.component_results,
                &device,
                &format!("DC {device:?}"),
            );
        }
    }
}

#[test]
fn transient_reported_device_current_obeys_kcl_off_nominal_temperature() {
    for t in TEMPERATURES {
        for device in devices(t) {
            // A capacitor across the ideal source sends the solve down the
            // transient path without changing the n2 operating point.
            let circuit = Circuit {
                ground_node: "0".into(),
                components: vec![
                    source(5.0),
                    CircuitElement::Capacitor {
                        id: "C1".into(),
                        nodes: ["n1".into(), "0".into()],
                        capacitance: 1e-6,
                    },
                    resistor(),
                    device.clone(),
                ],
            };
            let result: SimulationResult = solve_circuit(&circuit).expect("transient solve");
            let transient = result.transient.as_ref().expect("transient result");
            let last = transient.timesteps.last().expect("a timestep");
            assert_kcl(
                &last.component_results,
                &device,
                &format!("transient {device:?}"),
            );
        }
    }
}

fn ac_gain_at_n2(circuit: &Circuit) -> f64 {
    let config = AcConfig {
        f_start: 10.0,
        f_stop: 10.0,
        num_points: 1,
        spacing: FrequencySpacing::Linear,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
    };
    let ac = solve_ac(circuit, &config).expect("AC solve");
    ac.points[0].node_voltages["n2"].norm()
}

/// Divider R1 / D1: |H| = 1 / (1 + R·g_d), with g_d the finite-difference
/// slope of the temperature-scaled DC diode at its operating point.
#[test]
fn ac_diode_conductance_matches_dc_model_off_nominal_temperature() {
    for t in TEMPERATURES {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                source(5.0),
                resistor(),
                CircuitElement::Diode {
                    id: "D1".into(),
                    nodes: ["n2".into(), "0".into()],
                    temperature: t,
                },
            ],
        };
        let v_d = solve_circuit(&circuit).unwrap().node_voltages["n2"];

        let mut params = DiodeParams::silicon();
        params.is = temperature_scale_is(params.is, t, 300.15, 1.11, 2.0);
        let h = 1e-6;
        let g_d = (diode_current(v_d + h, &params) - diode_current(v_d - h, &params)) / (2.0 * h);
        let expected = 1.0 / (1.0 + R * g_d);

        let gain = ac_gain_at_n2(&circuit);
        assert!(
            (gain - expected).abs() <= 1e-5 * expected,
            "T={t}: AC gain {gain} vs DC-model {expected}"
        );
    }
}

/// Common emitter with the base driven directly: |H| at the collector is
/// gm·Rc, with gm the finite-difference dIc/dVbe of the temperature-scaled
/// DC model. Vbe is chosen per temperature for Ic ≈ 1 mA (forward active),
/// at temperatures that keep it below the companion's exponential clamp.
#[test]
fn ac_bjt_transconductance_matches_dc_model_off_nominal_temperature() {
    let bf = 100.0;
    let rc = 1000.0;
    for t in [280.0, 320.0, 350.0] {
        let mut params = BjtParams::new(bf);
        params.is = temperature_scale_is(params.is, t, 300.15, 1.11, 3.0);
        let vbe = params.nf * sindr_devices::diode::V_T * (1e-3 / params.is).ln();
        assert!(vbe > 0.0 && vbe < 1.0, "T={t}: Vbe {vbe} out of range");

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource {
                    id: "V1".into(),
                    nodes: ["b".into(), "0".into()],
                    voltage: vbe,
                    waveform: None,
                },
                CircuitElement::VoltageSource {
                    id: "VCC".into(),
                    nodes: ["vcc".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                },
                CircuitElement::Resistor {
                    id: "RC".into(),
                    nodes: ["vcc".into(), "c".into()],
                    resistance: rc,
                },
                CircuitElement::Bjt {
                    id: "Q1".into(),
                    nodes: ["b".into(), "c".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf,
                    temperature: t,
                    parasitic_caps: None,
                },
            ],
        };
        let vc = solve_circuit(&circuit).unwrap().node_voltages["c"];
        let vbc = vbe - vc;

        let h = 1e-6;
        let gm = (bjt_companion(vbe + h, vbc, &params).ic
            - bjt_companion(vbe - h, vbc, &params).ic)
            / (2.0 * h);
        let expected = gm * rc;

        let config = AcConfig {
            f_start: 10.0,
            f_stop: 10.0,
            num_points: 1,
            spacing: FrequencySpacing::Linear,
            source_id: "V1".into(),
            ac_magnitude: 1.0,
        };
        let gain = solve_ac(&circuit, &config).unwrap().points[0].node_voltages["c"].norm();
        assert!(
            (gain - expected).abs() <= 1e-3 * expected,
            "T={t}: AC gain {gain} vs DC-model gm·Rc {expected}"
        );
    }
}
