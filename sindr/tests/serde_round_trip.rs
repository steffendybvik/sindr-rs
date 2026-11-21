//! Circuits and results survive JSON and back: how an editor or agent tool
//! stores circuits and passes results around.
//!
//! Circuits are compared against committed snapshots in `tests/serde/`, so
//! an accidental format change shows up in review. Regenerate them after a
//! deliberate change:
//!
//! ```text
//! SINDR_WRITE_SNAPSHOTS=1 cargo test -p sindr --test serde_round_trip
//! ```
//!
//! Results are checked by round trip only. Their numbers come from a solve,
//! which can differ in the last bits between platforms, so a byte snapshot
//! would be flaky. JSON is compared as values, so `HashMap` order doesn't
//! matter.
#![cfg(feature = "serde")]

use std::path::PathBuf;

use serde::de::DeserializeOwned;
use serde::Serialize;
use serde_json::Value;
use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::measure::Signal;
use sindr::noise::{solve_noise, NoiseConfig};
use sindr::transfer_function::TfOutput;
use sindr::{
    check_ratings, check_specs, dc_sweep, get_examples, solve_circuit, solve_sensitivity, solve_tf,
    solve_transient, temperature_sweep, Block, Circuit, CircuitElement, Instance, Measure,
    Measurement, Part, Ratings, SensConfig, Spec, TfConfig, TransientConfig,
};

/// `value` to JSON, back, and to JSON again: both JSON values must match.
fn round_trip<T: Serialize + DeserializeOwned>(what: &str, value: &T) -> Value {
    let json = serde_json::to_value(value).unwrap_or_else(|e| panic!("{what}: {e}"));
    let back: T = serde_json::from_value(json.clone()).unwrap_or_else(|e| panic!("{what}: {e}"));
    let again = serde_json::to_value(&back).unwrap();
    assert_eq!(json, again, "{what} changed in a JSON round trip");
    json
}

fn snapshot_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/serde")
        .join(format!("{name}.json"))
}

/// `json` against `tests/serde/<name>.json`, or writes it there with
/// `SINDR_WRITE_SNAPSHOTS=1`.
fn snapshot(name: &str, json: &Value) {
    let path = snapshot_path(name);
    if std::env::var_os("SINDR_WRITE_SNAPSHOTS").is_some() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let text = serde_json::to_string_pretty(json).unwrap() + "\n";
        std::fs::write(&path, text).unwrap();
        return;
    }
    let text = std::fs::read_to_string(&path).unwrap_or_else(|_| {
        panic!(
            "no snapshot at {}; run with SINDR_WRITE_SNAPSHOTS=1",
            path.display()
        )
    });
    let committed: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(
        &committed, json,
        "{name}: the JSON format changed. If that's deliberate, regenerate with \
         SINDR_WRITE_SNAPSHOTS=1 and say so in the CHANGELOG."
    );
}

#[test]
fn every_example_circuit() {
    let examples = get_examples();
    assert!(examples.len() > 10);
    for example in examples {
        let json = round_trip(&example.id, &example.circuit);
        snapshot(&format!("circuit_{}", example.id), &json);
    }
}

/// The elements no example or `transient_matches_dc` bench uses.
fn remaining_elements() -> Circuit {
    let n = |a: &str, b: &str| [a.to_string(), b.to_string()];
    Circuit::new(
        "0",
        vec![
            CircuitElement::Battery {
                id: "B1".into(),
                nodes: n("bat", "0"),
                params: sindr::battery::BatteryParams::lithium_ion(2.0, 0.05),
                state_of_charge: 0.8,
            },
            CircuitElement::Inductor {
                id: "L1".into(),
                nodes: n("bat", "sw"),
                inductance: 10e-6,
                saturation: Some(sindr_devices::inductor::InductorSaturation::at(2.0, 10e-6)),
            },
            CircuitElement::SwitchSpdt {
                id: "S1".into(),
                nodes: ["sw".into(), "a".into(), "b".into()],
                position: sindr::circuit::SpdtPosition::B,
            },
            CircuitElement::Varactor {
                id: "D1".into(),
                nodes: n("0", "a"),
                params: sindr::VaractorParams::default(),
            },
            CircuitElement::Crystal {
                id: "Y1".into(),
                nodes: n("a", "b"),
                params: sindr::circuit::CrystalParams {
                    frequency: 16e6,
                    esr: 40.0,
                    c0: 5e-12,
                    cm: 20e-15,
                },
            },
            CircuitElement::FerriteBead {
                id: "FB1".into(),
                nodes: n("b", "0"),
                params: sindr::circuit::FerriteBeadParams {
                    r_dc: 0.1,
                    l: 1e-6,
                    r_ac: 600.0,
                    c: Some(1e-12),
                },
            },
        ],
    )
}

#[test]
fn elements_the_examples_leave_out() {
    let json = round_trip("remaining elements", &remaining_elements());
    snapshot("circuit_remaining_elements", &json);
}

/// A block with a part, instanced twice, and global nets.
fn hierarchy() -> Circuit {
    let mut divider = Block {
        ports: vec!["top".into(), "mid".into(), "bot".into()],
        components: vec![
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["top".into(), "mid".into()],
                resistance: 1e3,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["mid".into(), "bot".into()],
                resistance: 2e3,
            },
        ],
        ..Block::default()
    };
    divider.parts.insert(
        "R1".into(),
        Part {
            ratings: Ratings {
                power: Some(0.25),
                ..Ratings::default()
            },
            ..Part::default()
        },
    );
    let mut c = Circuit::new(
        "0",
        vec![CircuitElement::VoltageSource {
            id: "V1".into(),
            nodes: ["in".into(), "0".into()],
            voltage: 9.0,
            waveform: None,
        }],
    );
    c.blocks.insert("divider".into(), divider);
    c.instances
        .push(Instance::new("X1", "divider", ["in", "a", "0"]));
    c.instances
        .push(Instance::new("X2", "divider", ["a", "b", "0"]));
    c
}

#[test]
fn design_hierarchy() {
    let c = hierarchy();
    let json = round_trip("hierarchy", &c);
    snapshot("circuit_hierarchy", &json);
    // The round-tripped design still solves to the same answer.
    let back: Circuit = serde_json::from_value(json).unwrap();
    let (a, b) = (solve_circuit(&c).unwrap(), solve_circuit(&back).unwrap());
    assert_eq!(a.node_voltages, b.node_voltages);
}

/// 5 V through 1 kΩ into a diode, 1 µF across it.
fn rc_diode() -> Circuit {
    let mut c = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "a".into()],
                resistance: 1e3,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["a".into(), "0".into()],
                temperature: 300.15,
                model: None,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["a".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    );
    c.parts.insert(
        "R1".into(),
        Part {
            ratings: Ratings {
                power: Some(0.25),
                ..Ratings::default()
            },
            ..Part::default()
        },
    );
    c
}

#[test]
fn every_result_type() {
    let c = rc_diode();
    let v_a = TfOutput::Voltage {
        node: "a".into(),
        reference: None,
    };

    let op = solve_circuit(&c).unwrap();
    round_trip("operating point", &op);
    round_trip("ratings", &check_ratings(&c, &op).unwrap());
    let spec = Spec::new(Measurement::new(
        "va",
        Measure::Find {
            signal: Signal::v("a"),
            at: 0.0,
        },
    ))
    .between(0.5, 0.8);
    round_trip("specs", &check_specs(&op, &[spec]));

    let tran = solve_transient(&c, &TransientConfig::new(1e-3, 1e-5)).unwrap();
    round_trip("transient", &tran);

    let ac = AcConfig {
        f_start: 10.0,
        f_stop: 1e6,
        num_points: 11,
        spacing: FrequencySpacing::Logarithmic,
        source_id: "V1".into(),
        ac_magnitude: 1.0,
        ac_phase_deg: 0.0,
    };
    round_trip("ac config", &ac);
    round_trip("ac", &solve_ac(&c, &ac).unwrap());

    let noise = NoiseConfig {
        output: "a".into(),
        reference: None,
        input_source: "V1".into(),
        f_start: 10.0,
        f_stop: 1e6,
        num_points: 11,
        spacing: FrequencySpacing::Logarithmic,
    };
    round_trip("noise config", &noise);
    round_trip("noise", &solve_noise(&c, &noise).unwrap());

    let tf = TfConfig {
        output: v_a.clone(),
        input_source: "V1".into(),
    };
    round_trip("tf config", &tf);
    round_trip("tf", &solve_tf(&c, &tf).unwrap());

    let sens = SensConfig { output: v_a };
    round_trip("sens config", &sens);
    round_trip("sens", &solve_sensitivity(&c, &sens).unwrap());

    round_trip("dc sweep", &dc_sweep(&c, "V1", 0.0, 5.0, 6).unwrap());
    round_trip(
        "temperature sweep",
        &temperature_sweep(&c, 250.0, 350.0, 3).unwrap(),
    );
}
