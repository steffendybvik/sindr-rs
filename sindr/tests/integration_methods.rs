//! The transient integration methods on circuits where they differ:
//! nonlinear devices, a stiff time constant, a nonlinear capacitor.

use sindr::{
    solve_transient, BjtKind, Circuit, CircuitElement, IntegrationMethod, TransientConfig,
    VaractorParams, Waveform,
};

const METHODS: [IntegrationMethod; 3] = [
    IntegrationMethod::BackwardEuler,
    IntegrationMethod::Trapezoidal,
    IntegrationMethod::Gear2,
];

fn vsource(id: &str, p: &str, n: &str, voltage: f64, waveform: Option<Waveform>) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage,
        waveform,
    }
}

fn resistor(id: &str, a: &str, b: &str, resistance: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    }
}

fn capacitor(id: &str, a: &str, b: &str, capacitance: f64) -> CircuitElement {
    CircuitElement::Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance,
    }
}

fn sine(amplitude: f64, frequency: f64) -> Option<Waveform> {
    Some(Waveform::Sine {
        amplitude,
        frequency,
        offset: 0.0,
        phase: 0.0,
    })
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
    }
}

/// Node `node` over the run.
fn trace(circuit: &Circuit, config: &TransientConfig, node: &str) -> Vec<f64> {
    solve_transient(circuit, config)
        .unwrap_or_else(|e| panic!("{:?}: {e}", config.method))
        .transient
        .unwrap()
        .timesteps
        .iter()
        .map(|s| s.node_voltages[node])
        .collect()
}

/// Half-wave rectifier with a reservoir capacitor: every method converges,
/// and the methods agree on the ripple's peak to within their truncation
/// error.
#[test]
fn rectifier_converges_under_every_method() {
    let rectifier = circuit(vec![
        vsource("V1", "in", "0", 0.0, sine(5.0, 50.0)),
        CircuitElement::Diode {
            id: "D1".into(),
            nodes: ["in".into(), "out".into()],
            temperature: 300.15,
        },
        resistor("RL", "out", "0", 1e3),
        capacitor("C1", "out", "0", 100e-6),
    ]);
    let peaks: Vec<f64> = METHODS
        .iter()
        .map(|&m| {
            let config = TransientConfig::new(60e-3, 20e-6).with_method(m);
            trace(&rectifier, &config, "out")
                .into_iter()
                .fold(f64::MIN, f64::max)
        })
        .collect();
    for (m, p) in METHODS.iter().zip(&peaks) {
        assert!(*p > 4.2 && *p < 4.6, "{m:?}: peak {p}");
    }
    assert!(
        (peaks[1] - peaks[2]).abs() < 5e-3,
        "trap/Gear-2 peaks {peaks:?}"
    );
}

/// A BJT switched hard on and off by a 1 kHz square-ish sine on its base,
/// with Miller-ish parasitic capacitance: converges under every method, and
/// the collector swings between near VCC and saturation.
#[test]
fn bjt_switch_converges_under_every_method() {
    let switch = circuit(vec![
        vsource("V1", "in", "0", 0.0, sine(5.0, 1e3)),
        vsource("VCC", "vcc", "0", 5.0, None),
        resistor("RB", "in", "b", 10e3),
        resistor("RC", "vcc", "c", 1e3),
        CircuitElement::Bjt {
            id: "Q1".into(),
            nodes: ["b".into(), "c".into(), "0".into()],
            kind: BjtKind::Npn,
            bf: 100.0,
            temperature: 300.15,
            parasitic_caps: Some(sindr::circuit::BjtParasiticCaps {
                cbe: 20e-12,
                cbc: 5e-12,
            }),
        },
    ]);
    for m in METHODS {
        let config = TransientConfig::new(3e-3, 2e-6).with_method(m);
        let vc = trace(&switch, &config, "c");
        let (lo, hi) = vc
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), &v| (lo.min(v), hi.max(v)));
        assert!(lo < 0.3 && hi > 4.9, "{m:?}: collector {lo}..{hi}");
    }
}

/// RC with τ = 1 ns stepped at 1 µs: a time constant far shorter than the
/// step, so the exact answer is the source voltage from the first step on.
/// The backward-Euler first step lands within τ/h = 1e-3 of it. Gear-2 and
/// backward Euler then damp that error away; the trapezoidal rule carries
/// it on, flipping sign each step and shrinking only by
/// (1 − h/2τ)/(1 + h/2τ) ≈ −0.996 per step. This is why Gear-2 is the
/// default.
#[test]
fn trapezoidal_rings_on_a_stiff_time_constant_and_gear2_does_not() {
    let stiff = circuit(vec![
        vsource("V1", "in", "0", 1.0, None),
        resistor("R1", "in", "out", 1.0),
        capacitor("C1", "out", "0", 1e-9),
    ]);
    let config = |m| {
        TransientConfig::new(20e-6, 1e-6)
            .with_method(m)
            .with_fixed_step()
    };
    for m in [IntegrationMethod::Gear2, IntegrationMethod::BackwardEuler] {
        let v = trace(&stiff, &config(m), "out");
        assert!(
            v[2..].iter().all(|x| (x - 1.0).abs() < 1e-5),
            "{m:?}: {v:?}"
        );
    }
    let trap = trace(&stiff, &config(IntegrationMethod::Trapezoidal), "out");
    let err: Vec<f64> = trap.iter().map(|x| x - 1.0).collect();
    assert!(
        err[1..].windows(2).all(|w| w[0] * w[1] < 0.0),
        "trapezoidal error should alternate in sign: {err:?}"
    );
    assert!(err.last().unwrap().abs() > 9e-4, "{err:?}");
}

/// A varactor charged through a resistor settles at exactly the source
/// voltage, as any capacitor does. Its companion used to inject its history
/// current with the wrong sign, which left it about 1e-3 short.
#[test]
fn varactor_settles_at_the_source_voltage() {
    let varactor = circuit(vec![
        vsource("V1", "in", "0", -2.0, None),
        resistor("R1", "in", "out", 1e3),
        CircuitElement::Varactor {
            id: "D1".into(),
            nodes: ["out".into(), "0".into()],
            params: VaractorParams::default(),
        },
    ]);
    for m in METHODS {
        let v = trace(
            &varactor,
            &TransientConfig::new(1e-6, 1e-9).with_method(m),
            "out",
        );
        let last = *v.last().unwrap();
        assert!((last + 2.0).abs() < 1e-9, "{m:?}: settled at {last}");
    }
}
