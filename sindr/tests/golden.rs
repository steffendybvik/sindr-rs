//! End-to-end results checked against ngspice.
//!
//! Each case in `tests/golden/` is a netlist (`<case>.cir`), the ngspice
//! analysis that produced its reference data (`<case>.ctl`), and that data
//! (`<case>.golden`). The goldens are committed; regenerate them with
//! `sindr/tests/golden/regen.sh` (needs ngspice on the PATH). See
//! `tests/golden/README.md` for how cases are built and toleranced.
//!
//! Only compiled when the `spice` feature is enabled.
#![cfg(feature = "spice")]

use std::fmt::Write as _;
use std::path::PathBuf;

use num_complex::Complex64;
use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::spice::{parse_file, to_netlist};
use sindr::{
    dc_sweep, solve_circuit, temperature_sweep, Circuit, CircuitElement, JfetKind, MosfetKind,
    SimulationResult,
};
use sindr_devices::mosfet::MosfetParams;

fn case_path(case: &str, ext: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/golden")
        .join(format!("{case}.{ext}"))
}

/// One `wrdata` table: a header of vector names, then rows. Column 0 is the
/// scale (time, frequency, sweep value).
struct Golden {
    names: Vec<String>,
    rows: Vec<Vec<f64>>,
}

impl Golden {
    fn load(case: &str) -> Self {
        let path = case_path(case, "golden");
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("{}: {e} (run tests/golden/regen.sh)", path.display()));
        let mut lines = text.lines().filter(|l| !l.trim().is_empty());
        let names = lines
            .next()
            .expect("empty golden")
            .split_whitespace()
            .map(str::to_string)
            .collect();
        let rows = lines
            .map(|l| {
                l.split_whitespace()
                    .map(|v| v.parse().expect("number in golden"))
                    .collect()
            })
            .collect();
        Self { names, rows }
    }

    fn column(&self, name: &str) -> Vec<f64> {
        let i = self
            .names
            .iter()
            .position(|n| n == name)
            .unwrap_or_else(|| panic!("no vector {name} in {:?}", self.names));
        self.rows.iter().map(|r| r[i]).collect()
    }

    fn scale(&self) -> Vec<f64> {
        self.rows.iter().map(|r| r[0]).collect()
    }

    /// The single value of an operating-point vector.
    fn op(&self, name: &str) -> f64 {
        self.column(name)[0]
    }
}

/// Accepts `|got − want| ≤ abs + rel·|want|`.
#[derive(Clone, Copy)]
struct Tol {
    rel: f64,
    abs: f64,
}

const OP_V: Tol = Tol {
    rel: 1e-6,
    abs: 1e-9,
};
const OP_I: Tol = Tol {
    rel: 1e-5,
    abs: 1e-12,
};

/// ngspice's JFET adds gate-junction diodes and GMIN shunts that sindr's
/// Shichman-Hodges model omits.
const JFET: (Tol, Tol) = (
    Tol {
        rel: 2e-5,
        abs: 1e-9,
    },
    Tol {
        rel: 2e-5,
        abs: 1e-12,
    },
);

/// Collects every quantity of a case, then fails once with the worst
/// deviation of each quantity that's out of tolerance.
struct Check {
    case: &'static str,
    report: String,
}

impl Check {
    fn new(case: &'static str) -> Self {
        Self {
            case,
            report: String::new(),
        }
    }

    fn series(&mut self, quantity: &str, scale: &[f64], got: &[f64], want: &[f64], tol: Tol) {
        assert_eq!(got.len(), want.len(), "{}: {quantity} length", self.case);
        let mut worst: Option<(f64, usize)> = None;
        for (i, (g, w)) in got.iter().zip(want).enumerate() {
            // A NaN from sindr counts as the worst possible deviation.
            let excess = (g - w).abs() - (tol.abs + tol.rel * w.abs());
            let excess = if excess.is_nan() {
                f64::INFINITY
            } else {
                excess
            };
            if excess > 0.0 && worst.is_none_or(|(e, _)| excess > e) {
                worst = Some((excess, i));
            }
        }
        if let Some((_, i)) = worst {
            let (g, w) = (got[i], want[i]);
            let _ = writeln!(
                self.report,
                "  {quantity} at {:.6e}: sindr {g:.9e}, ngspice {w:.9e} (diff {:.3e}, rel {:.3e})",
                scale[i],
                g - w,
                ((g - w) / w).abs()
            );
        }
    }

    fn value(&mut self, quantity: &str, got: f64, want: f64, tol: Tol) {
        self.series(quantity, &[0.0], &[got], &[want], tol);
    }

    fn finish(self) {
        assert!(
            self.report.is_empty(),
            "{} deviates from ngspice:\n{}",
            self.case,
            self.report
        );
    }
}

fn parse(case: &str) -> Circuit {
    parse_file(case_path(case, "cir"))
        .unwrap_or_else(|e| panic!("{case}: {e}"))
        .circuit
}

/// Both sindr and ngspice report a source's current flowing into its `+`
/// terminal, so a source delivering power reads negative.
fn source_current(result: &SimulationResult, id: &str) -> f64 {
    result.branch_currents[id]
}

/// Compares an operating point: `v(<node>)` and `i(<source>)` vectors.
fn check_op(case: &'static str, circuit: &Circuit, tol: (Tol, Tol)) {
    let golden = Golden::load(case);
    let result = solve_circuit(circuit).unwrap_or_else(|e| panic!("{case}: {e}"));
    let mut check = Check::new(case);
    for name in golden.names.iter().skip(1) {
        let inner = &name[2..name.len() - 1];
        if name.starts_with("v(") {
            check.value(name, result.node_voltages[inner], golden.op(name), tol.0);
        } else {
            check.value(name, source_current(&result, inner), golden.op(name), tol.1);
        }
    }
    check.finish();
}

// ---------------------------------------------------------------- .op

#[test]
fn op_resistive() {
    check_op("op_resistive", &parse("op_resistive"), (OP_V, OP_I));
}

#[test]
fn op_diode() {
    check_op("op_diode", &parse("op_diode"), (OP_V, OP_I));
}

#[test]
fn op_bjt_ce() {
    check_op("op_bjt_ce", &parse("op_bjt_ce"), (OP_V, OP_I));
}

#[test]
fn op_bjt_card() {
    check_op("op_bjt_card", &parse("op_bjt_card"), (OP_V, OP_I));
}

#[test]
fn op_diode_series() {
    check_op("op_diode_series", &parse("op_diode_series"), (OP_V, OP_I));
}

#[test]
fn op_subckt_param() {
    check_op("op_subckt_param", &parse("op_subckt_param"), (OP_V, OP_I));
}

// Devices with no SPICE card in sindr: the circuit is built here, matching
// the ngspice deck element for element.

fn vsource(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    }
}

fn resistor(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    }
}

fn circuit(components: Vec<CircuitElement>) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components,
        ..Default::default()
    }
}

#[test]
fn op_mosfet_cs() {
    check_op(
        "op_mosfet_cs",
        &circuit(vec![
            vsource("vdd", "vdd", "0", 10.0),
            vsource("vg", "g", "0", 3.0),
            resistor("rd", "vdd", "d", 10e3),
            CircuitElement::Mosfet {
                id: "m1".into(),
                nodes: ["g".into(), "d".into(), "0".into()],
                kind: MosfetKind::Nmos,
                params: MosfetParams::default_nmos(),
                parasitic_caps: None,
            },
        ]),
        (OP_V, OP_I),
    );
}

/// A three-terminal MOSFET's body is tied to its source, so a follower
/// sees no body effect; ngspice's `M1 vdd g s s`.
#[test]
fn op_mosfet_follower() {
    check_op(
        "op_mosfet_follower",
        &circuit(vec![
            vsource("vdd", "vdd", "0", 10.0),
            vsource("vg", "g", "0", 5.0),
            resistor("rs", "s", "0", 10e3),
            CircuitElement::Mosfet {
                id: "m1".into(),
                nodes: ["g".into(), "vdd".into(), "s".into()],
                kind: MosfetKind::Nmos,
                params: MosfetParams::default_nmos(),
                parasitic_caps: None,
            },
        ]),
        (OP_V, OP_I),
    );
}

/// PMOS with source and body at VDD; ngspice's `M1 d g vdd vdd`.
#[test]
fn op_pmos_highside() {
    check_op(
        "op_pmos_highside",
        &circuit(vec![
            vsource("vdd", "vdd", "0", 10.0),
            vsource("vg", "g", "0", 5.0),
            resistor("rd", "d", "0", 10e3),
            CircuitElement::Mosfet {
                id: "m1".into(),
                nodes: ["g".into(), "d".into(), "vdd".into()],
                kind: MosfetKind::Pmos,
                params: MosfetParams::default_pmos(),
                parasitic_caps: None,
            },
        ]),
        (OP_V, OP_I),
    );
}

#[test]
fn op_jfet() {
    check_op(
        "op_jfet",
        &circuit(vec![
            vsource("vdd", "vdd", "0", 12.0),
            resistor("rd", "vdd", "d", 1e3),
            resistor("rs", "s", "0", 220.0),
            resistor("rg", "g", "0", 1e6),
            CircuitElement::Jfet {
                id: "j1".into(),
                nodes: ["g".into(), "d".into(), "s".into()],
                kind: JfetKind::NChannel,
                idss: 10e-3,
                vp: -2.0,
            },
        ]),
        JFET,
    );
}

#[test]
fn op_led() {
    check_op(
        "op_led",
        &circuit(vec![
            vsource("v1", "in", "0", 5.0),
            resistor("r1", "in", "out", 150.0),
            CircuitElement::Led {
                id: "d1".into(),
                nodes: ["out".into(), "0".into()],
                color: "red".into(),
                temperature: 300.15,
            },
        ]),
        (OP_V, OP_I),
    );
}

#[test]
fn op_schottky() {
    check_op(
        "op_schottky",
        &circuit(vec![
            vsource("v1", "in", "0", 5.0),
            resistor("r1", "in", "out", 1e3),
            CircuitElement::SchottkyDiode {
                id: "d1".into(),
                nodes: ["out".into(), "0".into()],
                temperature: 300.15,
            },
        ]),
        (OP_V, OP_I),
    );
}

#[test]
fn op_photodiode() {
    check_op(
        "op_photodiode",
        &circuit(vec![
            vsource("v1", "k", "0", 5.0),
            resistor("r1", "k", "a", 10e3),
            CircuitElement::Photodiode {
                id: "d1".into(),
                nodes: ["0".into(), "a".into()],
                irradiance: 1e-4,
                temperature: 300.15,
            },
        ]),
        (OP_V, OP_I),
    );
}

// ---------------------------------------------------------------- .dc

#[test]
fn dc_diode_iv() {
    let golden = Golden::load("dc_diode_iv");
    let sweep = dc_sweep(&parse("dc_diode_iv"), "v1", 0.0, 1.0, 21).unwrap();
    let scale = golden.scale();
    let vb: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["b"])
        .collect();
    let i: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| source_current(&p.result, "v1"))
        .collect();

    let mut check = Check::new("dc_diode_iv");
    check.series("v-sweep", &scale, &sweep_values(&sweep), &scale, OP_V);
    check.series("v(b)", &scale, &vb, &golden.column("v(b)"), OP_V);
    check.series("i(v1)", &scale, &i, &golden.column("i(v1)"), OP_I);
    check.finish();
}

#[test]
fn dc_diode_1n4148() {
    let golden = Golden::load("dc_diode_1n4148");
    let sweep = dc_sweep(&parse("dc_diode_1n4148"), "v1", 0.0, 2.0, 21).unwrap();
    let scale = golden.scale();
    let vb: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["b"])
        .collect();
    let i: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| source_current(&p.result, "v1"))
        .collect();

    let mut check = Check::new("dc_diode_1n4148");
    check.series("v-sweep", &scale, &sweep_values(&sweep), &scale, OP_V);
    check.series("v(b)", &scale, &vb, &golden.column("v(b)"), OP_V);
    check.series("i(v1)", &scale, &i, &golden.column("i(v1)"), OP_I);
    check.finish();
}

fn sweep_values(sweep: &sindr::DcSweepResult) -> Vec<f64> {
    sweep.points.iter().map(|p| p.sweep_value).collect()
}

// ---------------------------------------------------------------- .ac

const AC_LINEAR: Tol = Tol {
    rel: 1e-6,
    abs: 1e-12,
};

/// Runs a log sweep matching ngspice's `ac dec <per_decade> f_start f_stop`
/// and compares `mag(v(node))` and, when the golden has it, phase in degrees.
fn check_ac(
    case: &'static str,
    circuit: &Circuit,
    source: &str,
    node: &str,
    per_decade: usize,
    f: (f64, f64),
    mag_tol: Tol,
) {
    let golden = Golden::load(case);
    let decades = (f.1 / f.0).log10().round() as usize;
    let config = AcConfig {
        f_start: f.0,
        f_stop: f.1,
        num_points: decades * per_decade + 1,
        spacing: FrequencySpacing::Logarithmic,
        source_id: source.into(),
        ac_magnitude: 1.0,
    };
    let result = solve_ac(circuit, &config).unwrap_or_else(|e| panic!("{case}: {e}"));
    let freqs: Vec<f64> = result.points.iter().map(|p| p.frequency).collect();
    let phasors: Vec<_> = result
        .points
        .iter()
        .map(|p| p.node_voltages[node])
        .collect();

    let mut check = Check::new(case);
    let scale = golden.scale();
    check.series(
        "frequency",
        &scale,
        &freqs,
        &scale,
        Tol {
            rel: 1e-8,
            abs: 0.0,
        },
    );
    let mag: Vec<f64> = phasors.iter().map(|v| v.norm()).collect();
    check.series("mag", &scale, &mag, &golden.column("mag"), mag_tol);
    if golden.names.iter().any(|n| n == "ph") {
        let phase: Vec<f64> = phasors.iter().map(|v| v.arg().to_degrees()).collect();
        check.series(
            "phase (deg)",
            &scale,
            &phase,
            &golden.column("ph"),
            Tol {
                rel: 0.0,
                abs: 1e-4,
            },
        );
    }
    check.finish();
}

#[test]
fn ac_rc_lowpass() {
    check_ac(
        "ac_rc_lowpass",
        &parse("ac_rc_lowpass"),
        "v1",
        "out",
        10,
        (1.0, 100e3),
        AC_LINEAR,
    );
}

#[test]
fn ac_rlc_bandpass() {
    check_ac(
        "ac_rlc_bandpass",
        &parse("ac_rlc_bandpass"),
        "v1",
        "out",
        20,
        (100.0, 100e3),
        AC_LINEAR,
    );
}

#[test]
fn ac_bjt_ce() {
    check_ac(
        "ac_bjt_ce",
        &parse("ac_bjt_ce"),
        "vin",
        "c",
        5,
        (10.0, 100e3),
        OP_V,
    );
}

// ------------------------------------------------------- exported decks

/// Checks that `<case>.cir` is the deck `to_netlist` writes for `circuit`,
/// so the golden is ngspice's answer for exactly this circuit. With
/// `SINDR_WRITE_DECKS=1` it writes the deck instead; then run
/// `regen.sh <case>`.
fn exported<'a>(case: &str, circuit: &'a Circuit) -> &'a Circuit {
    let deck = to_netlist(
        circuit,
        &format!("{case}: written by sindr::spice::to_netlist"),
    )
    .unwrap_or_else(|e| panic!("{case}: {e}"));
    let path = case_path(case, "cir");
    if std::env::var_os("SINDR_WRITE_DECKS").is_some() {
        std::fs::write(&path, &deck).unwrap();
    }
    let committed = std::fs::read_to_string(&path).unwrap_or_default();
    assert_eq!(
        committed, deck,
        "{case}.cir isn't what to_netlist writes: rerun with SINDR_WRITE_DECKS=1, then regen.sh {case}"
    );
    circuit
}

fn with_names(id: &str, nodes: [&str; 2]) -> (String, [String; 2]) {
    (id.into(), [nodes[0].into(), nodes[1].into()])
}

/// E, G, H and F sources, a current source, a potentiometer and a switch, written by
/// `to_netlist`: their signs and equivalents against ngspice's.
#[test]
fn op_exported_controlled() {
    let (r, two) = (resistor, with_names);
    let mut components = vec![
        vsource("v1", "in", "0", 2.0),
        r("r1", "in", "a", 1e3),
        r("r2", "a", "0", 2e3),
        r("re", "e", "0", 1e3),
        r("rg", "g", "0", 1e3),
        r("rh", "h", "0", 1e3),
        r("rf", "f", "0", 1e3),
        r("rx", "x", "0", 1e3),
        r("ri", "i", "0", 1e3),
        CircuitElement::CurrentSource {
            id: "i1".into(),
            nodes: ["i".into(), "0".into()],
            current: 1e-3,
            waveform: None,
        },
        CircuitElement::Potentiometer {
            id: "rpot".into(),
            nodes: ["in".into(), "w".into(), "0".into()],
            resistance: 10e3,
            position: 0.3,
        },
        CircuitElement::Switch {
            id: "sw1".into(),
            nodes: ["w".into(), "x".into()],
            closed: true,
        },
    ];
    let (id, nodes) = two("e1", ["e", "0"]);
    components.push(CircuitElement::Vcvs {
        id,
        nodes,
        control_nodes: ["a".into(), "0".into()],
        gain: 3.0,
    });
    let (id, nodes) = two("g1", ["g", "0"]);
    components.push(CircuitElement::Vccs {
        id,
        nodes,
        control_nodes: ["a".into(), "0".into()],
        gm: 1e-3,
    });
    let (id, nodes) = two("h1", ["h", "0"]);
    components.push(CircuitElement::Ccvs {
        id,
        nodes,
        control_source: "v1".into(),
        rm: 100.0,
    });
    let (id, nodes) = two("f1", ["f", "0"]);
    components.push(CircuitElement::Cccs {
        id,
        nodes,
        control_source: "v1".into(),
        alpha: 2.0,
    });
    let circuit = Circuit {
        ground_node: "0".into(),
        components,
        ..Default::default()
    };
    check_op(
        "op_exported_controlled",
        exported("op_exported_controlled", &circuit),
        (OP_V, OP_I),
    );
}

/// A transformer, written by `to_netlist` as two L and a K: its dot
/// convention and coupling against ngspice's.
#[test]
fn ac_exported_transformer() {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            vsource("v1", "in", "0", 0.0),
            resistor("r1", "in", "p", 50.0),
            CircuitElement::Transformer {
                id: "t1".into(),
                nodes: ["p".into(), "0".into(), "s".into(), "0".into()],
                l1: 1e-3,
                l2: 4e-3,
                k: 0.99,
            },
            resistor("rl", "s", "0", 1e3),
        ],
        ..Default::default()
    };
    check_ac(
        "ac_exported_transformer",
        exported("ac_exported_transformer", &circuit),
        "v1",
        "s",
        10,
        (10.0, 1e6),
        AC_LINEAR,
    );
}

// ---------------------------------------------------------------- .tran

/// sindr steps with its default Gear-2 at a fixed dt = τ/50 over 5τ; the
/// golden is an accurate ngspice solution resampled onto that grid. The
/// largest error, about 0.03 % of full scale, comes from the backward-Euler
/// first step; allow 0.1 %. (Backward Euler alone peaks near 0.4 %.)
fn check_tran(case: &'static str, vectors: &[&str]) {
    let result = solve_circuit(&parse(case)).unwrap();
    compare_tran(case, vectors, &result.transient.expect("transient result"));
}

/// Each of `vectors` in `tran` against the golden, resampled onto sindr's
/// steps, to 0.1 % of full scale.
fn compare_tran(case: &'static str, vectors: &[&str], tran: &sindr::TransientData) {
    let golden = Golden::load(case);
    let times: Vec<f64> = tran.timesteps.iter().map(|s| s.time).collect();

    let mut check = Check::new(case);
    let scale = golden.scale();
    for name in vectors {
        let want_all = golden.column(name);
        let full_scale = want_all.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let inner = &name[2..name.len() - 1];
        let got: Vec<f64> = tran
            .timesteps
            .iter()
            .map(|s| {
                if name.starts_with("v(") {
                    s.node_voltages[inner]
                } else {
                    // Snapshots carry per-component currents, not branch
                    // currents; a source's `current_through` has the same
                    // sign convention as `branch_currents`.
                    let c = s.component_results.iter().find(|c| c.id == inner).unwrap();
                    c.current_through
                }
            })
            .collect();
        let want: Vec<f64> = times
            .iter()
            .map(|&t| interpolate(&scale, &want_all, t))
            .collect();
        check.series(
            name,
            &times,
            &got,
            &want,
            Tol {
                rel: 0.0,
                abs: 1e-3 * full_scale,
            },
        );
    }
    check.finish();
}

fn interpolate(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    let i = xs.partition_point(|&v| v < x).clamp(1, xs.len() - 1);
    let (x0, x1, y0, y1) = (xs[i - 1], xs[i], ys[i - 1], ys[i]);
    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
}

#[test]
fn tran_rc_step() {
    check_tran("tran_rc_step", &["v(out)"]);
}

#[test]
fn tran_rl_step() {
    check_tran("tran_rl_step", &["v(a)", "i(v1)"]);
}

/// `.tran` without `UIC`: both start from the DC operating point (the
/// pulse's 1 V low, the cap charged, the inductor carrying its current),
/// sit there until the pulse at 1 ms, then respond to it. The golden is
/// ngspice's own steps (TMAX 1 µs, not resampled), so the pulse edge isn't
/// smeared.
#[test]
fn tran_op_start() {
    let config = sindr::TransientConfig::new(3e-3, 20e-6)
        .with_tmax(1e-6)
        .with_start(sindr::TransientStart::OperatingPoint);
    let result = sindr::solve_transient(&parse("tran_op_start"), &config).unwrap();
    compare_tran(
        "tran_op_start",
        &["v(a)", "i(v1)"],
        &result.transient.unwrap(),
    );
}

// ---------------------------------------------------------------- temperature

#[test]
fn temp_diode() {
    let golden = Golden::load("temp_diode");
    let kelvin: Vec<f64> = golden.scale().iter().map(|c| c + 273.15).collect();
    let sweep = temperature_sweep(
        &parse("temp_diode"),
        kelvin[0],
        *kelvin.last().unwrap(),
        kelvin.len(),
    )
    .unwrap();
    let got: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["out"])
        .collect();
    let mut check = Check::new("temp_diode");
    check.series("v(out)", &kelvin, &got, &golden.column("v(out)"), OP_V);
    check.finish();
}

// ---------------------------------------------------------------- diode charge and breakdown

/// A 1N4148 card with CJO, M, TT, BV and IBV, through 1 kΩ, swept past its
/// 100 V breakdown. ngspice adds a GMIN leakage (1e-12 S) that sindr leaves
/// out, about 0.1 nA here, so currents get 1 nA; at the knee the two
/// models' reverse terms differ by a few parts in 1e5.
#[test]
fn dc_diode_breakdown() {
    let golden = Golden::load("dc_diode_breakdown");
    let sweep = dc_sweep(&parse("dc_diode_breakdown"), "v1", 0.0, -150.0, 31).unwrap();
    let scale = golden.scale();
    let vb: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["b"])
        .collect();
    let i: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| source_current(&p.result, "v1"))
        .collect();

    let mut check = Check::new("dc_diode_breakdown");
    check.series("v(b)", &scale, &vb, &golden.column("v(b)"), OP_V);
    check.series(
        "i(v1)",
        &scale,
        &i,
        &golden.column("i(v1)"),
        Tol {
            rel: 1e-4,
            abs: 1e-9,
        },
    );
    check.finish();
}

/// sindr's small-signal admittance of a diode at each bias of ngspice's
/// sweep, measured in AC at 1 MHz behind the deck's 1 Ω, against what
/// ngspice's junction (`@d1[gd]`, `@d1[cd]`) gives behind the card's RS:
/// `Y = 1/(RS + 1/(gd + jωCd))`. Conductance is compared only above
/// `−3·N·V_T`: below it ngspice swaps Shockley's reverse current for a
/// cubic that sindr doesn't model, which is a different (nA-scale) leakage,
/// not a different charge. It gets 2 pS, since ngspice's gd includes its
/// 1 pS GMIN.
fn check_cv(case: &'static str) {
    let golden = Golden::load(case);
    let base = parse(case);
    let card = base
        .components
        .iter()
        .find_map(|c| match c {
            CircuitElement::Diode { model: Some(m), .. } => Some(*m),
            _ => None,
        })
        .expect("a diode with a card");
    let rs = card.rs;
    let shockley_from = -3.0 * card.n * sindr_devices::diode::V_T;
    let bias = golden.scale();
    let f = 1e6;
    let omega = 2.0 * std::f64::consts::PI * f;
    let want: Vec<Complex64> = golden
        .column("@d1[gd]")
        .iter()
        .zip(golden.column("@d1[cd]"))
        .map(|(&gd, cd)| 1.0 / (rs + 1.0 / Complex64::new(gd, omega * cd)))
        .collect();
    let got: Vec<Complex64> = bias
        .iter()
        .map(|&v| {
            let mut circuit = base.clone();
            for c in &mut circuit.components {
                if let CircuitElement::VoltageSource { id, voltage, .. } = c {
                    if id == "v1" {
                        *voltage = v;
                    }
                }
            }
            let ac = solve_ac(
                &circuit,
                &AcConfig {
                    f_start: f,
                    f_stop: f,
                    num_points: 1,
                    spacing: FrequencySpacing::Linear,
                    source_id: "v1".into(),
                    ac_magnitude: 1.0,
                },
            )
            .unwrap();
            let vb = ac.points[0].node_voltages["b"];
            // 1 V across 1 Ω and the diode: Y = I / V(b), I = (1 − V(b)) / 1 Ω.
            (1.0 - vb) / vb
        })
        .collect();
    let part = |ys: &[Complex64], f: fn(&Complex64) -> f64| ys.iter().map(f).collect::<Vec<_>>();
    let mut check = Check::new(case);
    check.series(
        "C = Im(Y)/ω",
        &bias,
        &part(&got, |y| y.im / (2.0 * std::f64::consts::PI * 1e6)),
        &part(&want, |y| y.im / (2.0 * std::f64::consts::PI * 1e6)),
        Tol {
            rel: 1e-3,
            abs: 1e-16,
        },
    );
    let forward: Vec<usize> = (0..bias.len())
        .filter(|&i| bias[i] >= shockley_from)
        .collect();
    let pick = |xs: &[f64]| forward.iter().map(|&i| xs[i]).collect::<Vec<_>>();
    check.series(
        "G = Re(Y)",
        &pick(&bias),
        &pick(&part(&got, |y| y.re)),
        &pick(&part(&want, |y| y.re)),
        Tol {
            rel: 1e-3,
            abs: 2e-12,
        },
    );
    check.finish();
}

#[test]
fn cv_1n4148() {
    check_cv("cv_1n4148");
}

#[test]
fn cv_1n5819() {
    check_cv("cv_1n5819");
}

/// A 1N4148 (TT = 20 ns, CJO = 4 pF) carried 4.3 mA forward, then switched to
/// −5 V: the stored charge holds it on for about 15 ns of reverse current
/// before it snaps off. ngspice runs the same deck with `uic`, from rest.
#[test]
fn tran_diode_recovery() {
    let case = "tran_diode_recovery";
    let golden = Golden::load(case);
    let parsed = parse_file(case_path(case, "cir")).unwrap();
    let config = parsed.analyses[0].transient_config().unwrap();
    let tran = sindr::solve_transient(&parsed.circuit, &config)
        .unwrap()
        .transient
        .expect("transient result");
    let times: Vec<f64> = tran.timesteps.iter().map(|s| s.time).collect();
    let scale = golden.scale();

    let mut check = Check::new(case);
    for (name, full_scale) in [("v(b)", 5.0), ("i(v1)", 5.6e-3)] {
        let want_all = golden.column(name);
        let got: Vec<f64> = tran
            .timesteps
            .iter()
            .map(|s| {
                if name == "v(b)" {
                    s.node_voltages["b"]
                } else {
                    let c = s.component_results.iter().find(|c| c.id == "v1").unwrap();
                    c.current_through
                }
            })
            .collect();
        let want: Vec<f64> = times
            .iter()
            .map(|&t| interpolate(&scale, &want_all, t))
            .collect();
        check.series(
            name,
            &times,
            &got,
            &want,
            Tol {
                rel: 0.0,
                abs: 2e-2 * full_scale,
            },
        );
    }
    check.finish();

    // The storage phase itself: reverse current through the diode at 110 ns,
    // 9 ns into recovery, within 2 % of ngspice.
    let at = |t: f64| {
        let k = times.partition_point(|&x| x < t);
        tran.timesteps[k]
            .component_results
            .iter()
            .find(|c| c.id == "d1")
            .unwrap()
            .current_through
    };
    let want = -interpolate(&scale, &golden.column("i(v1)"), 110e-9);
    let got = at(110e-9);
    assert!(
        (got - want).abs() <= 0.02 * want.abs(),
        "D1 at 110 ns: sindr {got:e}, ngspice {want:e}"
    );
}
