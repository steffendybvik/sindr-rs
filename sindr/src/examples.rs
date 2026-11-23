//! Built-in example circuits.
//!
//! Each circuit is exposed as a named function returning `Circuit` directly,
//! e.g. `sindr::examples::voltage_divider()`. The dynamic metadata-bearing list
//! (`get_examples()` returning `Vec<ExampleCircuit>`) is feature-gated behind
//! `examples` and intended for HTTP/UI consumers that need ids and descriptions.

use crate::circuit::{Circuit, CircuitElement};
use crate::element::{
    Bjt, Capacitor, CurrentSource, Fuse, Led, Resistor, Transformer, VoltageSource,
};
use crate::waveform::Waveform;
use sindr_devices::bjt::BjtKind;

// ============================================================================
// Beginner: pure resistive
// ============================================================================

/// Two resistors in series across a voltage source. Node `n2` shows the divided voltage.
pub fn voltage_divider() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2000.0,
            }),
        ],
        ..Default::default()
    }
}

/// Two resistors in parallel. Current splits inversely proportional to resistance.
pub fn current_divider() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 12.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 3000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R2".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 6000.0,
            }),
        ],
        ..Default::default()
    }
}

/// Three resistors in series. Voltage drops proportional to resistance.
pub fn series_resistors() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 9.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "n3".into()],
                resistance: 2000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R3".into(),
                nodes: ["n3".into(), "0".into()],
                resistance: 3000.0,
            }),
        ],
        ..Default::default()
    }
}

/// A 9 V supply drives a red LED through a 330 Ω current-limiting resistor.
pub fn led_circuit() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 9.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 330.0,
            }),
            CircuitElement::Led(Led {
                id: "LED1".into(),
                nodes: ["n2".into(), "0".into()],
                color: "red".into(),
                temperature: 300.15,
            }),
        ],
        ..Default::default()
    }
}

/// 5 V source through a 1 A fuse and 100 Ω load. Toggle `blown` for the open-fuse case.
pub fn fuse_protection() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            }),
            CircuitElement::Fuse(Fuse {
                id: "F1".into(),
                nodes: ["n1".into(), "n2".into()],
                rating: 1.0,
                blown: false,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 100.0,
            }),
        ],
        ..Default::default()
    }
}

// ============================================================================
// Intermediate: reactive, AC, magnetic coupling
// ============================================================================

/// Voltage source and current source in the same circuit, demonstrating superposition.
pub fn mixed_sources() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            }),
            CircuitElement::CurrentSource(CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "n2".into()],
                current: 0.002,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2000.0,
            }),
        ],
        ..Default::default()
    }
}

/// Classic measurement bridge with four resistors. Bridge voltage is zero when balanced.
pub fn wheatstone_bridge() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R3".into(),
                nodes: ["n1".into(), "n3".into()],
                resistance: 1000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R4".into(),
                nodes: ["n3".into(), "0".into()],
                resistance: 2000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R_bridge".into(),
                nodes: ["n2".into(), "n3".into()],
                resistance: 5000.0,
            }),
        ],
        ..Default::default()
    }
}

/// 5 V amplitude, 1 kHz sinusoidal source driving a 1 kΩ load.
pub fn sine_wave_source() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Sine {
                    amplitude: 5.0,
                    frequency: 1000.0,
                    offset: 0.0,
                    phase: 0.0,
                }),
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1000.0,
            }),
        ],
        ..Default::default()
    }
}

/// First-order RC filter with -3 dB cutoff at ~159 Hz (R=1 kΩ, C=1 µF).
pub fn rc_lowpass_filter() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            }),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["n2".into(), "0".into()],
                capacitance: 1e-6,
            }),
        ],
        ..Default::default()
    }
}

/// 2:1 step-down transformer (L1=1 H, L2=0.25 H, k=0.999). 10 V primary → ~5 V on 100 Ω secondary.
pub fn transformer_stepdown() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            }),
            CircuitElement::Transformer(Transformer {
                id: "T1".into(),
                nodes: ["n1".into(), "0".into(), "n2".into(), "0".into()],
                l1: 1.0,
                l2: 0.25,
                k: 0.999,
                saturation: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R_load".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 100.0,
            }),
        ],
        ..Default::default()
    }
}

// ============================================================================
// Advanced: semiconductors
// ============================================================================

/// Classic NPN BJT amplifier. Base resistor sets bias, collector resistor sets gain.
pub fn npn_common_emitter() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "Vcc".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "Rb".into(),
                nodes: ["vcc".into(), "base".into()],
                resistance: 470_000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "Rc".into(),
                nodes: ["vcc".into(), "collector".into()],
                resistance: 1000.0,
            }),
            CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["base".into(), "collector".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }),
        ],
        ..Default::default()
    }
}

/// BJT driven into saturation to switch a load. Low base resistor ensures full turn-on.
pub fn npn_switch() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "Vcc".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "Rb".into(),
                nodes: ["vcc".into(), "base".into()],
                resistance: 10_000.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "Rc".into(),
                nodes: ["vcc".into(), "collector".into()],
                resistance: 470.0,
            }),
            CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: ["base".into(), "collector".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }),
        ],
        ..Default::default()
    }
}

// ============================================================================
// Metadata aggregator — for HTTP/UI listing.
// Gated behind `examples`; serialisable with `serde`.
// ============================================================================

#[cfg(feature = "examples")]
mod meta {
    use super::*;

    /// A named, simulatable example circuit with display metadata.
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[derive(Debug, Clone)]
    pub struct ExampleCircuit {
        /// Stable identifier used for lookup and routing.
        pub id: String,
        /// Human-readable display name.
        pub name: String,
        /// Short description of what the circuit demonstrates.
        pub description: String,
        /// The simulatable circuit definition.
        pub circuit: Circuit,
    }

    /// Returns all built-in example circuits with their display metadata.
    /// Order matches the difficulty grouping in the frontend.
    pub fn get_examples() -> Vec<ExampleCircuit> {
        vec![
            ExampleCircuit {
                id: "voltage-divider".into(),
                name: "Voltage Divider".into(),
                description: "Two resistors in series across a voltage source. Node n2 shows the divided voltage.".into(),
                circuit: voltage_divider(),
            },
            ExampleCircuit {
                id: "current-divider".into(),
                name: "Current Divider".into(),
                description: "Two resistors in parallel. Current splits inversely proportional to resistance.".into(),
                circuit: current_divider(),
            },
            ExampleCircuit {
                id: "series-resistors".into(),
                name: "Series Resistors".into(),
                description: "Three resistors in series. Voltage drops proportional to resistance.".into(),
                circuit: series_resistors(),
            },
            ExampleCircuit {
                id: "led-circuit".into(),
                name: "LED Circuit".into(),
                description: "A 330Ω current-limiting resistor sets a red LED's current from a 9V supply. Sweep the supply to see the LED block until its forward voltage, then conduct.".into(),
                circuit: led_circuit(),
            },
            ExampleCircuit {
                id: "fuse-protection".into(),
                name: "Fuse Protection Circuit".into(),
                description: "5V source through a 1A fuse and 100Ω load. Click the fuse on the canvas to toggle between intact and blown states.".into(),
                circuit: fuse_protection(),
            },
            ExampleCircuit {
                id: "mixed-sources".into(),
                name: "Mixed Sources".into(),
                description: "Voltage source and current source in the same circuit, demonstrating superposition.".into(),
                circuit: mixed_sources(),
            },
            ExampleCircuit {
                id: "wheatstone-bridge".into(),
                name: "Wheatstone Bridge".into(),
                description: "Classic measurement bridge with four resistors. When balanced (R1/R2 = R3/R4), the bridge voltage is zero.".into(),
                circuit: wheatstone_bridge(),
            },
            ExampleCircuit {
                id: "sine-wave-source".into(),
                name: "Sine Wave Source".into(),
                description: "5V amplitude, 1kHz sinusoidal voltage source driving a 1kΩ load. Double-click the source to edit waveform parameters.".into(),
                circuit: sine_wave_source(),
            },
            ExampleCircuit {
                id: "rc-lowpass-filter".into(),
                name: "RC Low-Pass Filter".into(),
                description: "First-order RC filter with -3dB cutoff at ~159 Hz (R=1kΩ, C=1µF). Use AC Sweep mode to see the Bode plot drop-off.".into(),
                circuit: rc_lowpass_filter(),
            },
            ExampleCircuit {
                id: "transformer-stepdown".into(),
                name: "Transformer Step-Down".into(),
                description: "2:1 step-down transformer (L1=1H, L2=0.25H, k=0.999). 10V primary yields ~5V across the 100Ω secondary load.".into(),
                circuit: transformer_stepdown(),
            },
            ExampleCircuit {
                id: "npn-common-emitter".into(),
                name: "NPN Common Emitter".into(),
                description: "Classic BJT amplifier. Base resistor sets bias current, collector resistor sets gain. Shows active region operation.".into(),
                circuit: npn_common_emitter(),
            },
            ExampleCircuit {
                id: "npn-switch".into(),
                name: "NPN Transistor Switch".into(),
                description: "BJT driven into saturation to switch a load. Low base resistor ensures the transistor fully turns on.".into(),
                circuit: npn_switch(),
            },
        ]
    }
}

#[cfg(feature = "examples")]
pub use meta::{get_examples, ExampleCircuit};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solve_circuit;

    fn all() -> Vec<(&'static str, Circuit)> {
        vec![
            ("voltage_divider", voltage_divider()),
            ("current_divider", current_divider()),
            ("series_resistors", series_resistors()),
            ("led_circuit", led_circuit()),
            ("fuse_protection", fuse_protection()),
            ("mixed_sources", mixed_sources()),
            ("wheatstone_bridge", wheatstone_bridge()),
            ("sine_wave_source", sine_wave_source()),
            ("rc_lowpass_filter", rc_lowpass_filter()),
            ("transformer_stepdown", transformer_stepdown()),
            ("npn_common_emitter", npn_common_emitter()),
            ("npn_switch", npn_switch()),
        ]
    }

    #[test]
    fn every_example_solves_to_finite_values() {
        for (name, circuit) in all() {
            let result = solve_circuit(&circuit).unwrap_or_else(|e| panic!("{name}: {e}"));
            for (node, v) in &result.node_voltages {
                assert!(v.is_finite(), "{name}: V({node}) = {v}");
            }
        }
    }

    #[test]
    fn voltage_divider_matches_closed_form() {
        let result = solve_circuit(&voltage_divider()).unwrap();
        assert!((result.voltage("n2").unwrap() - 10.0 * 2000.0 / 3000.0).abs() < 1e-9);
    }

    #[test]
    fn current_divider_splits_inversely_to_resistance() {
        let result = solve_circuit(&current_divider()).unwrap();
        let current = |id: &str| result.current(id).unwrap().abs();
        assert!((current("R1") - 12.0 / 3000.0).abs() < 1e-12);
        assert!((current("R2") - 12.0 / 6000.0).abs() < 1e-12);
    }

    #[test]
    fn led_example_blocks_below_its_forward_voltage() {
        let sweep = crate::dc_sweep(&led_circuit(), "V1", 0.0, 9.0, 10).unwrap();
        let curve = sweep.component_current_curve("LED1");
        let at = |v: f64| curve.iter().find(|(x, _)| (x - v).abs() < 1e-9).unwrap().1;
        // No current at 0 V and essentially none below a red LED's ~1.8 V.
        assert!(at(0.0).abs() < 1e-9, "I(0 V) = {}", at(0.0));
        assert!(at(1.0).abs() < 1e-6, "I(1 V) = {}", at(1.0));
        // Never backwards.
        assert!(curve.iter().all(|(_, i)| *i > -1e-9));
        // About 20 mA at 9 V through 330 Ω.
        let i9 = at(9.0);
        assert!((0.018..0.024).contains(&i9), "I(9 V) = {i9}");
    }

    #[test]
    fn bjt_examples_sit_in_their_intended_regions() {
        let region = |circuit: Circuit| {
            let result = solve_circuit(&circuit).unwrap();
            result.bjt("Q1").unwrap().region.clone()
        };
        assert_eq!(region(npn_common_emitter()), "active");
        assert_eq!(region(npn_switch()), "saturation");
    }

    #[test]
    fn reactive_and_waveform_examples_take_the_transient_path() {
        for circuit in [
            sine_wave_source(),
            rc_lowpass_filter(),
            transformer_stepdown(),
        ] {
            let result = solve_circuit(&circuit).unwrap();
            assert!(result.transient.is_some());
        }
        assert!(solve_circuit(&voltage_divider())
            .unwrap()
            .transient
            .is_none());
    }

    #[cfg(feature = "examples")]
    #[test]
    fn get_examples_lists_every_circuit_with_unique_kebab_ids() {
        let examples = get_examples();
        assert_eq!(examples.len(), all().len());

        let mut ids: Vec<&str> = examples.iter().map(|e| e.id.as_str()).collect();
        for id in &ids {
            assert!(
                id.chars().all(|c| c.is_ascii_lowercase() || c == '-'),
                "id {id:?} is not kebab-case"
            );
        }
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), examples.len(), "duplicate example ids");

        for example in &examples {
            assert!(!example.name.is_empty() && !example.description.is_empty());
            let fn_name = example.id.replace('-', "_");
            let (_, circuit) = all().into_iter().find(|(n, _)| *n == fn_name).unwrap();
            assert_eq!(
                serde_json::to_string(&example.circuit).unwrap(),
                serde_json::to_string(&circuit).unwrap(),
                "{} doesn't return {fn_name}()",
                example.id
            );
        }
    }
}
