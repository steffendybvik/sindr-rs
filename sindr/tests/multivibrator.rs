//! A two-transistor LED flasher (astable multivibrator) through its
//! transient: an LED and 470 Ω in each collector from 9 V, 47 kΩ base
//! resistors, 4.7 µF and 3.3 µF cross-coupling capacitors.
//!
//! Its transistors have no junction capacitance, so each edge is a jump:
//! the node voltages switch in no time while the capacitors hold theirs.
//! The period is checked against the closed form
//! `T ≈ ln 2 · RB · (C1 + C2)`, and each half period against the RC charge
//! of the base it times.

use sindr::{
    solve_transient, BjtKind, Circuit, CircuitElement, TransientConfig, TransientData,
    TransientStart,
};

const VCC: f64 = 9.0;
const RB: f64 = 47e3;
const C1: f64 = 4.7e-6;
const C2: f64 = 3.3e-6;

/// A junction in each collector: an LED of `color`, or a silicon diode
/// when `color` is `None`.
fn flasher(color: Option<&str>) -> Circuit {
    let junction = |id: &str, k: &str| match color {
        Some(color) => CircuitElement::Led {
            id: id.into(),
            nodes: ["vcc".into(), k.into()],
            color: color.into(),
            temperature: 300.15,
        },
        None => CircuitElement::Diode {
            id: id.into(),
            nodes: ["vcc".into(), k.into()],
            model: None,
            temperature: 300.15,
        },
    };
    let resistor = |id: &str, a: &str, b: &str, resistance: f64| CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance,
    };
    let capacitor = |id: &str, a: &str, b: &str, capacitance: f64| CircuitElement::Capacitor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        capacitance,
    };
    let npn = |id: &str, b: &str, c: &str| CircuitElement::Bjt {
        id: id.into(),
        nodes: [b.into(), c.into(), "0".into()],
        kind: BjtKind::Npn,
        bf: 200.0,
        temperature: 300.15,
        parasitic_caps: None,
        model: None,
    };
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "vs".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: VCC,
                waveform: None,
            },
            junction("d1", "a1"),
            junction("d2", "a2"),
            resistor("r1", "a1", "c1n", 470.0),
            resistor("r2", "a2", "c2n", 470.0),
            resistor("rb1", "vcc", "b1", RB),
            resistor("rb2", "vcc", "b2", RB),
            capacitor("c1", "c1n", "b2", C1),
            capacitor("c2", "c2n", "b1", C2),
            npn("q1", "b1", "c1n"),
            npn("q2", "b2", "c2n"),
        ],
        ..Default::default()
    }
}

fn run(circuit: &Circuit, start: TransientStart) -> TransientData {
    let config = TransientConfig::new(1.2, 1e-4).with_start(start);
    solve_transient(circuit, &config)
        .unwrap_or_else(|e| panic!("{e}"))
        .transient
        .unwrap()
}

/// The times `node` rises through `level` (V), linearly interpolated.
fn rising(tran: &TransientData, node: &str, level: f64) -> Vec<f64> {
    tran.timesteps
        .windows(2)
        .filter_map(|w| {
            let (a, b) = (w[0].node_voltages[node], w[1].node_voltages[node]);
            (a < level && b >= level)
                .then(|| w[0].time + (level - a) / (b - a) * (w[1].time - w[0].time))
        })
        .collect()
}

/// The steady period from the collector's rises, the first (start-up)
/// one left out.
fn period(tran: &TransientData) -> f64 {
    let rises = rising(tran, "c2n", 4.0);
    assert!(rises.len() >= 4, "only {} periods: {rises:?}", rises.len());
    (rises[rises.len() - 1] - rises[1]) / (rises.len() - 2) as f64
}

fn closed_form() -> f64 {
    std::f64::consts::LN_2 * RB * (C1 + C2)
}

/// The ideal formula assumes the collector swings the whole rail, kicking
/// the other base down to about −Vcc, and that the base switches at 0 V.
/// Here the LED holds the off collector near 7.7 V, a volt and a bit below
/// the rail even with a few mA of recharge current through it, so the base
/// is kicked down to only −6.9 V; and with the LED's high slope resistance
/// at low current the loop gain passes one, and the edge starts, at
/// Vbe ≈ 0.5 V. Both shorten the half periods: the period comes out about
/// 9.5 % under the closed form (ngspice gives the same 236 ms, see
/// `tran_flasher` in `tests/golden.rs`). Held to 9.5 % ± 2 %.
#[test]
fn led_flasher_runs_at_its_closed_form_period() {
    for start in [
        TransientStart::InitialConditions,
        TransientStart::OperatingPoint,
    ] {
        let tran = run(&flasher(Some("red")), start);
        let t = period(&tran);
        let deviation = t / closed_form() - 1.0;
        assert!(
            (deviation + 0.095).abs() < 0.02,
            "{start:?}: period {t} s is {:.1} % off the closed form {} s",
            deviation * 100.0,
            closed_form()
        );
        assert!(tran.stats.jumped_steps > 0, "{:?}", tran.stats);

        // The LED that's on carries (9 V − Vf − Vce(sat)) / 470 Ω, with Vf
        // ≈ 1.78 V for the red LED at 15 mA.
        let peak = tran
            .timesteps
            .iter()
            .flat_map(|s| &s.component_results)
            .filter(|c| c.id == "d1")
            .map(|c| c.current_through)
            .fold(0.0, f64::max);
        assert!((14e-3..16e-3).contains(&peak), "{start:?}: LED peak {peak}");
    }
}

/// Between edges, each base charges from where the edge kicked it towards
/// Vcc through RB, until it switches: the half period is
/// `RB · C · ln((Vcc − v_min) / (Vcc − v_edge))`, with the kick `v_min`
/// and the switching point `v_edge` read off the run. The capacitor's
/// other plate, the collector that's on, settles about 0.1 V up from its
/// Vce(sat) right after the edge as the recharge current fades, which
/// lifts the base with it: `v_min` counts that in.
#[test]
fn each_half_period_is_its_bases_rc_charge() {
    let tran = run(&flasher(Some("red")), TransientStart::InitialConditions);
    let s = &tran.timesteps;
    for (base, collector, c) in [("b1", "c2n", C2), ("b2", "c1n", C1)] {
        // The edges that switch this base's transistor off kick the base
        // down by volts in one step.
        let kicks: Vec<usize> = (1..s.len())
            .filter(|&i| s[i - 1].node_voltages[base] - s[i].node_voltages[base] > 3.0)
            .collect();
        assert!(kicks.len() >= 3, "{base}: {kicks:?}");
        for pair in kicks[1..].windows(2) {
            let (from, next) = (pair[0], pair[1]);
            // The base switches its transistor on where it jumps up at the
            // edge that turns the other one off.
            let on = (from + 1..next)
                .find(|&i| s[i].node_voltages[base] - s[i - 1].node_voltages[base] > 0.05)
                .unwrap_or_else(|| panic!("{base}: no turn-on after {}", s[from].time));
            let settled = s[on - 1].node_voltages[collector] - s[from].node_voltages[collector];
            let v_min = s[from].node_voltages[base] + settled;
            let v_edge = s[on - 1].node_voltages[base];
            let measured = s[on].time - s[from].time;
            let rc = RB * c * ((VCC - v_min) / (VCC - v_edge)).ln();
            assert!(
                (measured / rc - 1.0).abs() < 1e-2,
                "{base}: half period {measured} s, RC charge {rc} s (from {v_min} V to {v_edge} V)"
            );
        }
    }
}

/// Silicon diodes in place of the LEDs drop only about 0.7 V, so the
/// collector swings closer to the rail and the period is within a few
/// percent of the closed form.
#[test]
fn diode_flasher_runs_near_its_closed_form_period() {
    let tran = run(&flasher(None), TransientStart::InitialConditions);
    let deviation = period(&tran) / closed_form() - 1.0;
    assert!(deviation.abs() < 0.05, "{:.1} %", deviation * 100.0);
}
