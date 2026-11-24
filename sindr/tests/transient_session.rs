//! Transient sessions: stepped from outside, inputs changed between steps,
//! stopped on events.

use sindr::element::{
    Capacitor, CurrentSource, FerriteBead, Photoresistor, Potentiometer, Resistor, Switch,
    SwitchSpdt, VoltageSource,
};
use sindr::{
    solve_transient, Circuit, CircuitElement, Edge, Event, Flow, Param, SimError, Solver,
    SolverOptions, SpdtPosition, TimestepSnapshot, TransientConfig, TransientSession, Waveform,
};

const TAU: f64 = 1e-3;

/// `PIN` (0 V, or `waveform`) through 1 kΩ into 1 µF at `out`.
fn rc(waveform: Option<Waveform>) -> Circuit {
    Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "PIN".into(),
                nodes: ["pin".into(), "0".into()],
                voltage: 0.0,
                waveform,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["pin".into(), "out".into()],
                resistance: 1e3,
            }),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["out".into(), "0".into()],
                capacitance: 1e-6,
            }),
        ],
    )
}

/// 0 V to 5 V and back every millisecond: low for the first.
fn square() -> Waveform {
    Waveform::Pulse {
        v1: 0.0,
        v2: 5.0,
        delay: 1e-3,
        rise_time: 1e-9,
        fall_time: 1e-9,
        pulse_width: 1e-3,
        period: 2e-3,
    }
}

/// The same square, driven from outside: `PIN` set at each millisecond.
fn toggled(config: &TransientConfig) -> Vec<TimestepSnapshot> {
    let mut sim = TransientSession::new(&rc(None), config).unwrap();
    let ms = (config.tstop / 1e-3).round() as usize;
    for k in 1..=ms {
        sim.advance_to(k as f64 * 1e-3).unwrap();
        sim.set_source("PIN", if k % 2 == 1 { 5.0 } else { 0.0 })
            .unwrap();
    }
    sim.drain_steps().collect()
}

fn at(steps: &[TimestepSnapshot], t: f64, node: &str) -> f64 {
    let i = steps
        .partition_point(|s| s.time < t)
        .clamp(1, steps.len() - 1);
    let (a, b) = (&steps[i - 1], &steps[i]);
    let (va, vb) = (a.node_voltages[node], b.node_voltages[node]);
    va + (vb - va) * (t - a.time) / (b.time - a.time)
}

#[test]
fn a_square_toggled_from_outside_follows_the_same_pulse() {
    let config = TransientConfig::new(10e-3, 10e-6);
    let pulsed = solve_transient(&rc(Some(square())), &config)
        .unwrap()
        .transient
        .unwrap()
        .timesteps;
    let toggled = toggled(&config);
    // The pulse's 1 ns edges are the only difference: 5 V for 1 ns is
    // 5 µV on a 1 ms τ.
    for k in 0..=1000 {
        let t = k as f64 * 10e-6;
        let (want, got) = (at(&pulsed, t, "out"), at(&toggled, t, "out"));
        assert!((got - want).abs() < 2e-4, "t = {t}: {got} vs {want}");
    }
}

#[test]
fn a_change_restarts_the_integration_at_the_time_reached() {
    let config = TransientConfig::new(2e-3, 10e-6);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.advance_to(1e-3).unwrap();
    assert_eq!(sim.voltage("out"), Some(0.0));
    sim.set_source("PIN", 5.0).unwrap();
    // The step that reached 1 ms saw 0 V; the next one starts short, from
    // the new value.
    sim.advance_to(2e-3).unwrap();
    let steps: Vec<_> = sim.drain_steps().collect();
    let edge = steps.iter().position(|s| s.time >= 1e-3 - 1e-15).unwrap();
    assert_eq!(steps[edge].node_voltages["pin"], 0.0);
    assert_eq!(steps[edge + 1].node_voltages["pin"], 5.0);
    assert!(steps[edge + 1].time - steps[edge].time < 1e-6);
    let want = 5.0 * (1.0 - (-1.0f64).exp());
    assert!((sim.voltage("out").unwrap() - want).abs() < 2e-3);
}

#[test]
fn one_advance_is_the_whole_transient() {
    let config = TransientConfig::new(5e-3, 10e-6);
    let circuit = rc(Some(square()));
    let whole = solve_transient(&circuit, &config).unwrap();
    let mut sim = TransientSession::new(&circuit, &config).unwrap();
    sim.advance_to(5e-3).unwrap();
    let steps: Vec<_> = sim.drain_steps().collect();
    let tran = whole.transient.unwrap();
    assert_eq!(steps.len(), tran.timesteps.len());
    for (a, b) in steps.iter().zip(&tran.timesteps) {
        assert_eq!(a.time, b.time);
        assert_eq!(a.node_voltages["out"], b.node_voltages["out"]);
    }
    assert_eq!(sim.stats(), tran.stats);
}

#[test]
fn a_waveform_is_followed_across_many_short_advances() {
    // A 50 µs period sliced into 37 µs calls: each call finds its own
    // window of corners, and every one is landed on.
    let pwm = Waveform::Pulse {
        v1: 0.0,
        v2: 5.0,
        delay: 0.0,
        rise_time: 1e-7,
        fall_time: 1e-7,
        pulse_width: 2e-5,
        period: 5e-5,
    };
    let config = TransientConfig::new(2e-3, 5e-6);
    let circuit = rc(Some(pwm.clone()));
    let whole = solve_transient(&circuit, &config)
        .unwrap()
        .transient
        .unwrap();
    let mut sim = TransientSession::new(&circuit, &config).unwrap();
    let mut t: f64 = 0.0;
    while t < 2e-3 {
        t = (t + 37e-6).min(2e-3);
        sim.advance_to(t).unwrap();
    }
    let steps: Vec<_> = sim.drain_steps().collect();
    for corner in pwm.breakpoints(2e-3) {
        assert!(
            steps.iter().any(|s| (s.time - corner).abs() < 1e-15),
            "no step on the corner at {corner}"
        );
    }
    for s in &whole.timesteps {
        let got = at(&steps, s.time, "out");
        assert!(
            (got - s.node_voltages["out"]).abs() < 1e-3,
            "t = {}",
            s.time
        );
    }
}

#[test]
fn run_until_stops_at_a_rising_crossing() {
    let config = TransientConfig::new(10e-3, 10e-6);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.set_source("PIN", 5.0).unwrap();
    let tol = 1e-9;
    let hit = sim
        .run_until(
            10e-3,
            &[Event::crosses("out", 2.5, Edge::Rising).within(tol)],
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.event, 0);
    assert_eq!(hit.time, sim.time());
    // Past the level, by no more than the tolerance's worth of slope
    // (2.5 V / τ at the crossing).
    let v = sim.voltage("out").unwrap();
    assert!(v >= 2.5 && v - 2.5 <= 2.5 / TAU * tol * 1.01, "{v}");
    // Where the analytic crossing is, to the integration's own accuracy.
    assert!((hit.time - TAU * 2f64.ln()).abs() < 2e-6, "{}", hit.time);
    // The steps tried and undone while narrowing it are gone.
    let steps: Vec<_> = sim.drain_steps().collect();
    assert!(steps.windows(2).all(|w| w[0].time < w[1].time));
    assert_eq!(steps.last().unwrap().time, hit.time);
}

#[test]
fn run_until_stops_at_the_first_of_several_events() {
    let config = TransientConfig::new(10e-3, 10e-6);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.set_source("PIN", 5.0).unwrap();
    let events = [
        Event::crosses("out", 4.0, Edge::Rising),
        Event::crosses("out", 1.0, Edge::Either),
        Event::crosses("out", 0.5, Edge::Falling),
    ];
    let hit = sim.run_until(10e-3, &events).unwrap().unwrap();
    assert_eq!(hit.event, 1);
    assert!((hit.time - TAU * (5.0f64 / 4.0).ln()).abs() < 2e-6);
    // Already past 1 V: the next stop is 4 V.
    let hit = sim.run_until(10e-3, &events).unwrap().unwrap();
    assert_eq!(hit.event, 0);
    assert!((hit.time - TAU * 5f64.ln()).abs() < 2e-6);
    // Then the pin drops, and 1 V and 0.5 V come on the way down.
    sim.set_source("PIN", 0.0).unwrap();
    let t0 = sim.time();
    let hit = sim.run_until(10e-3, &events).unwrap().unwrap();
    assert_eq!(hit.event, 1);
    assert!((hit.time - (t0 + TAU * 4f64.ln())).abs() < 2e-6);
    let hit = sim.run_until(10e-3, &events).unwrap().unwrap();
    assert_eq!(hit.event, 2);
    assert!((hit.time - (t0 + TAU * 8f64.ln())).abs() < 2e-6);
    // Nothing left to cross: it runs to the end.
    assert_eq!(sim.run_until(10e-3, &events).unwrap(), None);
    assert!((sim.time() - 10e-3).abs() < 1e-15);
}

#[test]
fn run_until_finds_a_crossing_inside_a_long_fixed_step() {
    // 100 µs fixed steps on a 1 ms τ: the crossing is located far inside
    // one, to 10 ns.
    let config = TransientConfig::new(5e-3, 100e-6).with_fixed_step();
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.set_source("PIN", 5.0).unwrap();
    let hit = sim
        .run_until(
            5e-3,
            &[Event::crosses("out", 2.5, Edge::Rising).within(1e-8)],
        )
        .unwrap()
        .unwrap();
    let v = sim.voltage("out").unwrap();
    assert!((2.5..2.5 + 2.5 / TAU * 1e-8 * 1.01).contains(&v), "{v}");
    assert!((hit.time - TAU * 2f64.ln()).abs() < 1e-5, "{}", hit.time);
}

#[test]
fn a_potentiometer_turned_mid_run_moves_its_divider() {
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["top".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            }),
            CircuitElement::Potentiometer(Potentiometer {
                id: "RV1".into(),
                nodes: ["top".into(), "w".into(), "0".into()],
                resistance: 10e3,
                position: 0.5,
            }),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["w".into(), "0".into()],
                capacitance: 1e-9,
            }),
        ],
    );
    let config = TransientConfig::new(1e-3, 1e-6);
    let mut sim = TransientSession::new(&circuit, &config).unwrap();
    sim.advance_to(0.5e-3).unwrap();
    assert!((sim.voltage("w").unwrap() - 5.0).abs() < 1e-3);
    // Wiper toward the bottom: less of the 10 V across the lower part.
    sim.set_param("RV1", Param::Position(0.8)).unwrap();
    sim.advance_to(1e-3).unwrap();
    assert!((sim.voltage("w").unwrap() - 2.0).abs() < 1e-3);
}

#[test]
fn switches_photoresistors_and_spdt_throws_take_their_new_state() {
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            }),
            CircuitElement::Switch(Switch {
                id: "S1".into(),
                nodes: ["vcc".into(), "a".into()],
                closed: false,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["a".into(), "0".into()],
                resistance: 1e3,
            }),
            CircuitElement::Photoresistor(Photoresistor {
                id: "LDR1".into(),
                nodes: ["vcc".into(), "l".into()],
                light_level: 0.0,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R2".into(),
                nodes: ["l".into(), "0".into()],
                resistance: 10e3,
            }),
            CircuitElement::SwitchSpdt(SwitchSpdt {
                id: "SW1".into(),
                nodes: ["c".into(), "vcc".into(), "0".into()],
                position: SpdtPosition::B,
            }),
            CircuitElement::Resistor(Resistor {
                id: "R3".into(),
                nodes: ["c".into(), "0".into()],
                resistance: 1e3,
            }),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: ["a".into(), "0".into()],
                capacitance: 1e-12,
            }),
        ],
    );
    let config = TransientConfig::new(2e-6, 1e-8);
    let mut sim = TransientSession::new(&circuit, &config).unwrap();
    sim.advance_to(1e-6).unwrap();
    assert!(sim.voltage("a").unwrap() < 1e-3);
    assert!(sim.voltage("l").unwrap() < 0.1);
    assert!(sim.voltage("c").unwrap().abs() < 1e-3);
    sim.set_param("S1", Param::Closed(true)).unwrap();
    sim.set_param("LDR1", Param::LightLevel(1.0)).unwrap();
    sim.set_param("SW1", Param::Throw(SpdtPosition::A)).unwrap();
    sim.advance_to(2e-6).unwrap();
    assert!(sim.voltage("a").unwrap() > 4.9);
    assert!(sim.voltage("l").unwrap() > 4.0);
    assert!(sim.voltage("c").unwrap() > 4.9);
    // The SPDT is reported as itself, not as its two throws.
    let last = sim.last_step().unwrap();
    assert!(last.component_results.iter().any(|c| c.id == "SW1"));
    assert!(!last
        .component_results
        .iter()
        .any(|c| c.id.starts_with("SW1#")));
    assert!((sim.current("R1").unwrap() - sim.voltage("a").unwrap() / 1e3).abs() < 1e-9);
}

#[test]
fn a_value_it_already_has_doesnt_restart_the_integration() {
    let config = TransientConfig::new(2e-3, 10e-6);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.set_source("PIN", 5.0).unwrap();
    sim.advance_to(1e-3).unwrap();
    let before = sim.stats().accepted_steps;
    // Firmware writing the same pin level every tick.
    for k in 1..=100 {
        sim.set_source("PIN", 5.0).unwrap();
        sim.advance_to(1e-3 + k as f64 * 10e-6).unwrap();
    }
    // One step per tick: no restart's short steps.
    assert_eq!(sim.stats().accepted_steps - before, 100);
}

#[test]
fn bad_changes_are_refused_and_change_nothing() {
    let config = TransientConfig::new(1e-3, 10e-6);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    let invalid = |r: Result<(), SimError>| matches!(r, Err(SimError::InvalidComponent(_)));
    assert!(invalid(sim.set_source("R1", 1.0)));
    assert!(invalid(sim.set_source("NOPE", 1.0)));
    assert!(invalid(sim.set_source("PIN", f64::NAN)));
    assert!(invalid(sim.set_param("C1", Param::Resistance(1.0))));
    // The part's own check, as `validate_circuit` would give.
    assert!(matches!(
        sim.set_param("R1", Param::Resistance(-1.0)),
        Err(SimError::InvalidResistance(id)) if id == "R1"
    ));
    assert!(invalid(sim.set_param("R1", Param::Throw(SpdtPosition::A))));
    assert!(invalid(
        sim.set_param("NOPE", Param::Throw(SpdtPosition::A))
    ));
    assert!(invalid(sim.set_param("NOPE", Param::Closed(true))));
    sim.advance_to(1e-3).unwrap();
    assert_eq!(sim.voltage("out"), Some(0.0));
}

#[test]
fn a_session_only_goes_forward_and_ends_at_tstop() {
    let config = TransientConfig::new(1e-3, 10e-6);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.advance_to(0.5e-3).unwrap();
    sim.advance_to(0.5e-3).unwrap();
    assert!(matches!(
        sim.advance_to(0.4e-3),
        Err(SimError::InvalidComponent(_))
    ));
    assert!(matches!(
        sim.advance_to(2e-3),
        Err(SimError::InvalidComponent(_))
    ));
    assert!(matches!(
        sim.advance_to(f64::NAN),
        Err(SimError::InvalidComponent(_))
    ));
    assert!(matches!(
        sim.run_until(1e-3, &[Event::crosses("nowhere", 1.0, Edge::Rising)]),
        Err(SimError::InvalidComponent(_))
    ));
    assert!(matches!(
        sim.run_until(
            1e-3,
            &[Event::crosses("out", 1.0, Edge::Rising).within(0.0)]
        ),
        Err(SimError::InvalidComponent(_))
    ));
    assert_eq!(sim.voltage("nowhere"), None);
    assert_eq!(sim.voltage("0"), Some(0.0));
    sim.advance_to(1e-3).unwrap();
}

#[test]
fn a_cancelled_advance_can_be_taken_up_again() {
    let config = TransientConfig::new(10e-3, 10e-6);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.set_source("PIN", 5.0).unwrap();
    let mut seen = 0;
    let e = sim
        .advance_to_with_progress(10e-3, |p| {
            seen += 1;
            assert!(p.step.is_some());
            if p.fraction() < 0.3 {
                Flow::Continue
            } else {
                Flow::Stop
            }
        })
        .unwrap_err();
    let SimError::Cancelled { time } = e else {
        panic!("{e}")
    };
    assert_eq!(time, sim.time());
    assert!((3e-3..4e-3).contains(&time));
    assert_eq!(seen, sim.stats().accepted_steps);
    sim.advance_to(10e-3).unwrap();
    assert!((sim.voltage("out").unwrap() - 5.0 * (1.0 - (-10.0f64).exp())).abs() < 2e-3);
}

#[test]
fn steps_are_kept_from_tstart_or_not_at_all() {
    let config = TransientConfig::new(2e-3, 10e-6).with_tstart(1e-3);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.set_source("PIN", 5.0).unwrap();
    sim.advance_to(0.5e-3).unwrap();
    assert_eq!(sim.drain_steps().count(), 0);
    assert!(sim.last_step().is_some());
    sim.advance_to(2e-3).unwrap();
    let steps: Vec<_> = sim.drain_steps().collect();
    assert!((steps[0].time - 1e-3).abs() < 1e-15);
    // Drained, the latest step is still there to read.
    assert_eq!(sim.last_step().unwrap().time, steps.last().unwrap().time);

    let mut sim = TransientSession::new(&rc(None), &TransientConfig::new(1e-3, 10e-6)).unwrap();
    sim.keep_steps(false);
    sim.advance_to(1e-3).unwrap();
    assert_eq!(sim.drain_steps().count(), 0);
    assert!((sim.last_step().unwrap().time - 1e-3).abs() < 1e-15);
}

#[test]
fn a_failed_session_refuses_to_go_on() {
    // A node with no DC path, driven: it ramps without limit (see
    // `transient_adaptive`), and the run stops with TimestepTooSmall.
    let circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::CurrentSource(CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "a".into()],
                current: 1.0,
                waveform: None,
            }),
            CircuitElement::FerriteBead(FerriteBead {
                id: "FB1".into(),
                nodes: ["a".into(), "open".into()],
                params: sindr::FerriteBeadParams::from_datasheet(1e3, 0.1),
            }),
        ],
    );
    let mut sim = TransientSession::new(&circuit, &TransientConfig::new(4e-6, 1e-7)).unwrap();
    let e = sim.advance_to(4e-6).unwrap_err();
    assert!(matches!(e, SimError::TimestepTooSmall { .. }), "{e:?}");
    assert!(matches!(
        sim.advance_to(4e-6),
        Err(SimError::InvalidComponent(_))
    ));
}

#[test]
fn a_solver_opens_sessions_with_its_options() {
    let mut options = SolverOptions::default();
    options.max_transient_steps = 10;
    let solver = Solver::new(options);
    let config = TransientConfig::new(1e-3, 10e-6);
    assert!(solver.transient_session(&rc(None), &config).is_err());
    let solver = Solver::new(SolverOptions::default());
    let mut sim = solver.transient_session(&rc(None), &config).unwrap();
    sim.advance_to(1e-3).unwrap();
}

#[test]
fn current_sources_sensors_and_temperatures_take_their_new_values() {
    use sindr::element::{Led, Photodiode, Thermistor};
    let mut circuit = Circuit::new(
        "0",
        vec![
            CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["vcc".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            }),
            CircuitElement::CurrentSource(CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "i".into()],
                current: 1e-3,
                waveform: None,
            }),
            CircuitElement::Resistor(Resistor {
                id: "RI".into(),
                nodes: ["i".into(), "0".into()],
                resistance: 1e3,
            }),
            CircuitElement::Thermistor(Thermistor {
                id: "TH1".into(),
                nodes: ["vcc".into(), "t".into()],
                temperature: 298.15,
            }),
            CircuitElement::Resistor(Resistor {
                id: "RT".into(),
                nodes: ["t".into(), "0".into()],
                resistance: 10e3,
            }),
            CircuitElement::Photodiode(Photodiode {
                id: "PD1".into(),
                nodes: ["p".into(), "0".into()],
                irradiance: 0.0,
                temperature: 300.15,
            }),
            CircuitElement::Resistor(Resistor {
                id: "RP".into(),
                nodes: ["p".into(), "0".into()],
                resistance: 1e3,
            }),
            CircuitElement::Resistor(Resistor {
                id: "RL".into(),
                nodes: ["vcc".into(), "l".into()],
                resistance: 330.0,
            }),
            CircuitElement::Led(Led {
                id: "D1".into(),
                nodes: ["l".into(), "0".into()],
                color: "red".into(),
                temperature: 300.15,
            }),
        ],
    );
    circuit.parts.insert(
        "D1".into(),
        sindr::Part {
            thermal: Some(sindr::Thermal::junction_to_ambient(400.0)),
            ..Default::default()
        },
    );
    let config = TransientConfig::new(2e-6, 1e-7);
    let mut sim = TransientSession::new(&circuit, &config).unwrap();
    sim.advance_to(1e-6).unwrap();
    let (t_cool, p_dark) = (sim.voltage("t").unwrap(), sim.voltage("p").unwrap());
    // Newton's gmin shunts take a few nV.
    assert!((sim.voltage("i").unwrap() - 1.0).abs() < 1e-6);
    assert!(p_dark.abs() < 1e-6);

    sim.set_source("I1", 2e-3).unwrap();
    // An NTC conducts more warm: more of the 5 V across RT.
    sim.set_param("TH1", Param::Temperature(350.0)).unwrap();
    sim.set_param("PD1", Param::Irradiance(1e-3)).unwrap();
    // Self-heating sets the LED's junction; a resistor has none.
    assert!(matches!(
        sim.set_param("D1", Param::Temperature(350.0)),
        Err(SimError::InvalidComponent(_))
    ));
    assert!(matches!(
        sim.set_param("RT", Param::Temperature(350.0)),
        Err(SimError::InvalidComponent(_))
    ));
    sim.advance_to(2e-6).unwrap();
    assert!((sim.voltage("i").unwrap() - 2.0).abs() < 1e-6);
    assert!(sim.voltage("t").unwrap() > t_cool + 0.5);
    assert!(sim.voltage("p").unwrap() > 0.1);
}

#[test]
fn of_two_events_crossed_in_one_step_the_earlier_wins() {
    // 1 ms fixed steps: the first step passes both 1 V and 2 V.
    let config = TransientConfig::new(5e-3, 1e-3).with_fixed_step();
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.set_source("PIN", 5.0).unwrap();
    let events = [
        Event::crosses("out", 2.0, Edge::Rising),
        Event::crosses("out", 1.0, Edge::Rising),
        // Ground never moves: watched, never hit.
        Event::crosses("0", 0.5, Edge::Either),
    ];
    let hit = sim.run_until(5e-3, &events).unwrap().unwrap();
    assert_eq!(hit.event, 1);
    let v = sim.voltage("out").unwrap();
    assert!((1.0..1.001).contains(&v), "{v}");
    assert!(matches!(
        sim.run_until(5e-3, &[Event::crosses("out", f64::NAN, Edge::Rising)]),
        Err(SimError::InvalidComponent(_))
    ));
}

#[test]
fn a_crossing_where_the_node_bends_hard_is_still_found() {
    // Near its asymptote over 1 ms steps the node curves hard across the
    // step, so straight-line estimates keep landing past the crossing;
    // the narrowing falls back to halving and still gets there.
    let config = TransientConfig::new(10e-3, 1e-3).with_fixed_step();
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    sim.set_source("PIN", 5.0).unwrap();
    let tol = 1e-9;
    let hit = sim
        .run_until(
            10e-3,
            &[Event::crosses("out", 4.9, Edge::Rising).within(tol)],
        )
        .unwrap()
        .unwrap();
    let v = sim.voltage("out").unwrap();
    // The slope there is (5 − 4.9) / τ.
    assert!((4.9..4.9 + 0.1 / TAU * tol * 2.0).contains(&v), "{v}");
    assert!((3e-3..5e-3).contains(&hit.time), "{}", hit.time);
}

#[test]
fn progress_before_tstart_has_no_step() {
    let config = TransientConfig::new(2e-3, 10e-6).with_tstart(1e-3);
    let mut sim = TransientSession::new(&rc(None), &config).unwrap();
    let mut before = 0;
    let mut after = 0;
    sim.advance_to_with_progress(2e-3, |p| {
        if p.time < 1e-3 - 1e-12 {
            assert!(p.step.is_none());
            before += 1;
        } else {
            assert!(p.step.is_some());
            after += 1;
        }
        Flow::Continue
    })
    .unwrap();
    assert!(before > 0 && after > 0);
}
