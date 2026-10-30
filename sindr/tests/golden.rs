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

use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::spice::parse_file;
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

/// Junction devices. sindr's thermal voltage is a constant for 300.00 K
/// (rounded), while the goldens run at the SPICE default 27 °C (300.15 K):
/// that shifts every junction voltage by ~5e-4 relative. Matching ngspice's
/// temperature to sindr's V_T brings these cases under 2e-6. In the
/// exponential region a current's relative error is the voltage one times
/// V/(n·V_T) ≈ 26, hence the wider current tolerance. Tighten both to
/// `OP_V`/`OP_I` once SIN-15 makes V_T follow temperature.
const JUNCTION: (Tol, Tol) = (
    Tol {
        rel: 1e-3,
        abs: 1e-9,
    },
    Tol {
        rel: 2e-2,
        abs: 1e-9,
    },
);

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
    check_op("op_diode", &parse("op_diode"), JUNCTION);
}

#[test]
fn op_bjt_ce() {
    check_op("op_bjt_ce", &parse("op_bjt_ce"), JUNCTION);
}

#[test]
fn op_diode_series() {
    check_op("op_diode_series", &parse("op_diode_series"), JUNCTION);
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
        JUNCTION,
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
        JUNCTION,
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
        JUNCTION,
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
    check.series("v(b)", &scale, &vb, &golden.column("v(b)"), JUNCTION.0);
    check.series("i(v1)", &scale, &i, &golden.column("i(v1)"), JUNCTION.1);
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
    let result = solve_ac(&parse(case), &config).unwrap_or_else(|e| panic!("{case}: {e}"));
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
    check_ac("ac_rc_lowpass", "v1", "out", 10, (1.0, 100e3), AC_LINEAR);
}

#[test]
fn ac_rlc_bandpass() {
    check_ac(
        "ac_rlc_bandpass",
        "v1",
        "out",
        20,
        (100.0, 100e3),
        AC_LINEAR,
    );
}

#[test]
fn ac_bjt_ce() {
    // gm = Ic/V_T, so the gain carries the same V_T offset as JUNCTION.
    check_ac("ac_bjt_ce", "vin", "c", 5, (10.0, 100e3), JUNCTION.0);
}

// ---------------------------------------------------------------- .tran

/// sindr steps with backward Euler at a fixed dt = τ/50 over 5τ; the golden
/// is an accurate ngspice solution resampled onto that grid. BE's global
/// error at that step peaks near 0.4 % of the step height, so allow 1 % of
/// full scale.
fn check_tran(case: &'static str, vectors: &[&str]) {
    let golden = Golden::load(case);
    let result = solve_circuit(&parse(case)).unwrap();
    let tran = result.transient.expect("transient result");
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
                abs: 0.01 * full_scale,
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

// ---------------------------------------------------------------- temperature

#[test]
#[ignore = "SIN-15: IS temperature scaling and V_T(T) are wrong"]
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
