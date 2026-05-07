//! Built-in example circuits.
//!
//! Each circuit is exposed as a named function returning `Circuit` directly,
//! e.g. `sindr::examples::voltage_divider()`. The dynamic metadata-bearing list
//! (`get_examples()` returning `Vec<ExampleCircuit>`) is feature-gated behind
//! `examples` and intended for HTTP/UI consumers that need ids and descriptions.

use crate::circuit::{Circuit, CircuitElement};
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
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2000.0,
            },
        ],
    }
}

/// Two resistors in parallel. Current splits inversely proportional to resistance.
pub fn current_divider() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 12.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 3000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 6000.0,
            },
        ],
    }
}

/// Three resistors in series. Voltage drops proportional to resistance.
pub fn series_resistors() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 9.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "n3".into()],
                resistance: 2000.0,
            },
            CircuitElement::Resistor {
                id: "R3".into(),
                nodes: ["n3".into(), "0".into()],
                resistance: 3000.0,
            },
        ],
    }
}

/// Current-limiting resistor protects an LED (modeled as a 2 V source).
pub fn led_circuit() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 9.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 330.0,
            },
            CircuitElement::VoltageSource {
                id: "V_LED".into(),
                nodes: ["n2".into(), "0".into()],
                voltage: 2.0,
                waveform: None,
            },
        ],
    }
}

/// 5 V source through a 1 A fuse and 100 Ω load. Toggle `blown` for the open-fuse case.
pub fn fuse_protection() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Fuse {
                id: "F1".into(),
                nodes: ["n1".into(), "n2".into()],
                rating: 1.0,
                blown: false,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 100.0,
            },
        ],
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
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            },
            CircuitElement::CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "n2".into()],
                current: 0.002,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2000.0,
            },
        ],
    }
}

/// Classic measurement bridge with four resistors. Bridge voltage is zero when balanced.
pub fn wheatstone_bridge() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 2000.0,
            },
            CircuitElement::Resistor {
                id: "R3".into(),
                nodes: ["n1".into(), "n3".into()],
                resistance: 1000.0,
            },
            CircuitElement::Resistor {
                id: "R4".into(),
                nodes: ["n3".into(), "0".into()],
                resistance: 2000.0,
            },
            CircuitElement::Resistor {
                id: "R_bridge".into(),
                nodes: ["n2".into(), "n3".into()],
                resistance: 5000.0,
            },
        ],
    }
}

/// 5 V amplitude, 1 kHz sinusoidal source driving a 1 kΩ load.
pub fn sine_wave_source() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 0.0,
                waveform: Some(Waveform::Sine {
                    amplitude: 5.0,
                    frequency: 1000.0,
                    offset: 0.0,
                    phase: 0.0,
                }),
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1000.0,
            },
        ],
    }
}

/// First-order RC filter with -3 dB cutoff at ~159 Hz (R=1 kΩ, C=1 µF).
pub fn rc_lowpass_filter() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["n2".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    }
}

/// 2:1 step-down transformer (L1=1 H, L2=0.25 H, k=0.999). 10 V primary → ~5 V on 100 Ω secondary.
pub fn transformer_stepdown() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Transformer {
                id: "T1".into(),
                nodes: ["n1".into(), "0".into(), "n2".into(), "0".into()],
                l1: 1.0,
                l2: 0.25,
                k: 0.999,
            },
            CircuitElement::Resistor {
                id: "R_load".into(),
                nodes: ["n2".into(), "0".into()],
                resistance: 100.0,
            },
        ],
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
            CircuitElement::VoltageSource {
                id: "Vcc".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "Rb".into(),
                nodes: ["vcc".into(), "base".into()],
                resistance: 470_000.0,
            },
            CircuitElement::Resistor {
                id: "Rc".into(),
                nodes: ["vcc".into(), "collector".into()],
                resistance: 1000.0,
            },
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["base".into(), "collector".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
            },
        ],
    }
}

/// BJT driven into saturation to switch a load. Low base resistor ensures full turn-on.
pub fn npn_switch() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "Vcc".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "Rb".into(),
                nodes: ["vcc".into(), "base".into()],
                resistance: 10_000.0,
            },
            CircuitElement::Resistor {
                id: "Rc".into(),
                nodes: ["vcc".into(), "collector".into()],
                resistance: 470.0,
            },
            CircuitElement::Bjt {
                id: "Q1".into(),
                nodes: ["base".into(), "collector".into(), "0".into()],
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
            },
        ],
    }
}

// ============================================================================
// Metadata aggregator — for HTTP/UI listing.
// Gated behind `examples` (which implies `serde`).
// ============================================================================

#[cfg(feature = "examples")]
mod meta {
    use super::*;
    use serde::{Deserialize, Serialize};

    /// A named, simulatable example circuit with display metadata.
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct ExampleCircuit {
        pub id: String,
        pub name: String,
        pub description: String,
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
                name: "LED Circuit (Simplified)".into(),
                description: "A current-limiting resistor protects an LED (modeled as a 2V source). Shows how resistors set current in a circuit.".into(),
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
