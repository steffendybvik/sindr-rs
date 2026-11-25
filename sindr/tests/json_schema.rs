//! The schemas the `schemars` feature generates accept the JSON sindr
//! itself writes and reads, and refuse JSON it can't read.
#![cfg(all(feature = "schemars", not(target_arch = "wasm32")))]

use schemars::{schema_for, JsonSchema};
use serde::Serialize;
use serde_json::{json, Value};
use sindr::ac_analysis::{solve_ac, AcConfig, AcResult};
use sindr::import::{Bench, BenchLoad, BenchSource, ImportOptions, LoadKind, Netlist, Pin};
use sindr::{solve_circuit, solve_transient, Circuit, SimulationResult, TransientConfig};

/// The errors from validating `value` against `T`'s schema.
fn errors<T: JsonSchema>(value: &Value) -> Vec<String> {
    let schema = serde_json::to_value(schema_for!(T)).unwrap();
    let validator = jsonschema::validator_for(&schema).expect("a valid schema");
    validator
        .iter_errors(value)
        .map(|e| format!("{} at {}", e, e.instance_path()))
        .collect()
}

fn assert_valid<T: JsonSchema>(value: &Value) {
    let errors = errors::<T>(value);
    assert!(errors.is_empty(), "{errors:#?}\nin {value:#}");
}

fn to_json(value: &impl Serialize) -> Value {
    serde_json::to_value(value).unwrap()
}

fn fixtures() -> Vec<(String, Value)> {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/serde");
    let mut out: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .map(|p| {
            let text = std::fs::read_to_string(&p).unwrap();
            (
                p.display().to_string(),
                serde_json::from_str(&text).unwrap(),
            )
        })
        .collect();
    out.sort_by(|a, b| a.0.cmp(&b.0));
    out
}

#[test]
fn every_circuit_fixture_fits_the_circuit_schema() {
    let fixtures = fixtures();
    assert!(fixtures.len() >= 10);
    for (path, value) in fixtures {
        let errors = errors::<Circuit>(&value);
        assert!(errors.is_empty(), "{path}: {errors:#?}");
    }
}

#[test]
fn the_import_types_fit_their_schemas() {
    let netlist = json!({
        "components": [
            {"id": "R1", "kind": "resistor", "value": "4k7"},
            {"id": "D1", "kind": "led", "value": "Green"},
            {"id": "U1", "kind": "op_amp",
             "model": {"type": "subckt", "text": ".subckt b a b c d e\n.ends", "name": "b",
                       "pins": ["+", "-", "OUT", "V+", "V-"]}},
            {"id": "U2", "kind": "mcu-qfn32"}
        ],
        "nets": [
            {"id": "n1", "name": "+3V3", "pins": [{"component": "R1", "terminal": "1"}]},
            {"id": "n2", "pins": [{"component": "R1", "terminal": "2"},
                                  {"component": "D1", "terminal": "anode"}]}
        ]
    });
    assert_valid::<Netlist>(&netlist);
    // The import reads it, and writes it back to JSON that fits too.
    let read: Netlist = serde_json::from_value(netlist).unwrap();
    assert_valid::<Netlist>(&to_json(&read));

    let bench = Bench::new("n0")
        .with_source(BenchSource::voltage("vin", "n1", 3.3))
        .with_load(BenchLoad::new(
            "probe",
            LoadKind::Resistor(1e6),
            Pin::new("U2", "vdd"),
        ))
        .with_supplies_from_net_names(&read);
    let written = to_json(&bench);
    assert_valid::<Bench>(&written);
    let back: Bench = serde_json::from_value(written).unwrap();
    assert_eq!(back, bench);

    let options = json!({"unknown_kinds": "black_box", "unconnected": "skip",
                         "terminal_names": {"Q1": {"1": "Base"}}});
    assert_valid::<ImportOptions>(&options);
}

#[test]
fn results_fit_their_schemas() {
    let circuit: Circuit = serde_json::from_value(fixtures()[0].1.clone()).unwrap();
    let op = solve_circuit(&circuit).unwrap();
    assert_valid::<SimulationResult>(&to_json(&op));

    let rc: Circuit = serde_json::from_value(json!({
        "ground_node": "0",
        "components": [
            {"type": "voltage_source", "id": "V1", "nodes": ["in", "0"], "voltage": 1.0},
            {"type": "resistor", "id": "R1", "nodes": ["in", "out"], "resistance": 1e3},
            {"type": "capacitor", "id": "C1", "nodes": ["out", "0"], "capacitance": 1e-6}
        ]
    }))
    .unwrap();
    let config = TransientConfig::new(1e-3, 1e-5);
    assert_valid::<TransientConfig>(&to_json(&config));
    let tran = solve_transient(&rc, &config).unwrap();
    assert_valid::<SimulationResult>(&to_json(&tran));

    let ac = AcConfig::new("V1", 10.0, 1e5, 5);
    assert_valid::<AcConfig>(&to_json(&ac));
    let result = solve_ac(&rc, &ac).unwrap();
    assert_valid::<AcResult>(&to_json(&result));
}

#[test]
fn json_sindr_cant_read_is_refused() {
    // A resistor without its resistance, and an unknown element type.
    let missing = json!({"ground_node": "0", "components": [
        {"type": "resistor", "id": "R1", "nodes": ["a", "0"]}]});
    assert!(serde_json::from_value::<Circuit>(missing.clone()).is_err());
    assert!(!errors::<Circuit>(&missing).is_empty());

    let unknown = json!({"ground_node": "0", "components": [
        {"type": "flux_capacitor", "id": "F1", "nodes": ["a", "0"]}]});
    assert!(serde_json::from_value::<Circuit>(unknown.clone()).is_err());
    assert!(!errors::<Circuit>(&unknown).is_empty());
}
