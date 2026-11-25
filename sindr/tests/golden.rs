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

use sindr::element::{
    Bjt, Capacitor, Cccs, Ccvs, Crystal, CurrentSource, Diode, FerriteBead, Jfet, Led, Mosfet,
    Photodiode, Potentiometer, Resistor, SchottkyDiode, Switch, SwitchSpdt, Transformer, Vccs,
    Vcvs, VoltageSource, ZenerDiode,
};
use std::fmt::Write as _;
use std::path::PathBuf;

use num_complex::Complex64;
use sindr::ac_analysis::{solve_ac, AcConfig, FrequencySpacing};
use sindr::noise::{solve_noise, NoiseConfig, NoiseResult};
use sindr::spice::{parse_file, to_netlist};
use sindr::{
    dc_sweep, solve_ac_sensitivity, solve_circuit, solve_op, solve_sensitivity, temperature_sweep,
    Circuit, CircuitElement, JfetKind, MosfetKind, SensParameter, SimulationResult,
};
use sindr::{DcSweepAxis, TempSweepConfig};
use sindr_devices::mosfet::MosfetParams;

#[path = "support/deck_parity.rs"]
mod deck_parity;

/// Checks that `<case>.cir` describes the Rust-built `circuit` (see
/// `support/deck_parity.rs`), so ngspice and sindr solve the same thing.
fn in_deck<'a>(case: &str, circuit: &'a Circuit) -> &'a Circuit {
    let deck = std::fs::read_to_string(case_path(case, "cir")).unwrap();
    deck_parity::in_deck(case, &deck, circuit)
}

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

/// How a quantity's error is measured, for the accuracy table.
#[derive(Clone, Copy)]
enum Unit {
    /// Relative to the reference, floored at `abs / rel`.
    Rel,
    /// As a fraction of this full scale.
    FullScale(f64),
    /// In degrees.
    Deg,
}

/// Collects every quantity of a case, then fails once with the worst
/// deviation of each quantity that's out of tolerance.
///
/// With `SINDR_ACCURACY` set to a file, each case also appends its worst
/// error and tolerance per unit there, a JSON object a line, for
/// `scripts/accuracy_table.py`.
struct Check {
    case: &'static str,
    report: String,
    /// Worst (error, allowed) by unit: relative, full scale, degrees.
    worst: [Option<(f64, f64)>; 3],
}

impl Check {
    fn new(case: &'static str) -> Self {
        Self {
            case,
            report: String::new(),
            worst: [None; 3],
        }
    }

    fn series(&mut self, quantity: &str, scale: &[f64], got: &[f64], want: &[f64], tol: Tol) {
        self.series_in(quantity, scale, got, want, tol, Unit::Rel);
    }

    /// As [`series`](Self::series), to `fraction` of `full_scale`.
    fn series_fs(
        &mut self,
        quantity: &str,
        scale: &[f64],
        got: &[f64],
        want: &[f64],
        fraction: f64,
        full_scale: f64,
    ) {
        let tol = Tol {
            rel: 0.0,
            abs: fraction * full_scale,
        };
        self.series_in(quantity, scale, got, want, tol, Unit::FullScale(full_scale));
    }

    fn series_in(
        &mut self,
        quantity: &str,
        scale: &[f64],
        got: &[f64],
        want: &[f64],
        tol: Tol,
        unit: Unit,
    ) {
        assert_eq!(got.len(), want.len(), "{}: {quantity} length", self.case);
        let mut worst: Option<(f64, usize)> = None;
        let mut largest = 0.0_f64;
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
            let err = match unit {
                Unit::Rel if tol.rel > 0.0 => (g - w).abs() / w.abs().max(tol.abs / tol.rel),
                Unit::Rel => continue,
                Unit::FullScale(fs) => (g - w).abs() / fs,
                Unit::Deg => (g - w).abs(),
            };
            largest = largest.max(if err.is_nan() { f64::INFINITY } else { err });
        }
        let (slot, allowed) = match unit {
            Unit::Rel => (0, tol.rel),
            Unit::FullScale(fs) => (1, tol.abs / fs),
            Unit::Deg => (2, tol.abs),
        };
        if !(matches!(unit, Unit::Rel) && tol.rel == 0.0) {
            let entry = &mut self.worst[slot];
            if entry.is_none_or(|(e, _)| largest > e) {
                *entry = Some((largest, allowed));
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
        if let Some(path) = std::env::var_os("SINDR_ACCURACY") {
            let fields: Vec<String> = ["rel", "fs", "deg"]
                .iter()
                .zip(self.worst)
                .filter_map(|(name, w)| {
                    w.map(|(err, allowed)| format!("\"{name}\": [{err:e}, {allowed:e}]"))
                })
                .collect();
            let line = format!("{{\"case\": \"{}\", {}}}\n", self.case, fields.join(", "));
            use std::io::Write as _;
            let mut file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
                .expect("open SINDR_ACCURACY");
            file.write_all(line.as_bytes())
                .expect("write SINDR_ACCURACY");
        }
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

/// Compares an operating point, as `.op` maps onto [`solve_op`]:
/// `v(<node>)` and `i(<source or inductor>)` vectors.
fn check_op(case: &'static str, circuit: &Circuit, tol: (Tol, Tol)) {
    let golden = Golden::load(case);
    let result = solve_op(circuit).unwrap_or_else(|e| panic!("{case}: {e}"));
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

/// An inductor shorted, capacitors open, and a `PULSE` current source at
/// its t = 0 value, as ngspice's `.op`; `solve_circuit` would report the
/// 5 τ end of a transient instead.
#[test]
fn op_reactive() {
    check_op("op_reactive", &parse("op_reactive"), (OP_V, OP_I));
}

/// B sources and the `VALUE` forms of E and G: a multiplier, `i()` of a
/// source, a current that depends on its own output, and a `.param`
/// inside an expression. Pins `I=`'s direction and `i()`'s sign.
#[test]
fn op_behavioural() {
    check_op("op_behavioural", &parse("op_behavioural"), (OP_V, OP_I));
}

/// `POLY` on E, G, F and H, lowered to behavioural sources: pins SPICE2's
/// coefficient order (`x1², x1·x2, x2²` after the linear terms).
#[test]
fn op_poly() {
    check_op("op_poly", &parse("op_poly"), (OP_V, OP_I));
}

/// A squarer's small-signal gain is its slope at the bias, 2·1.5, into
/// an RC pole.
#[test]
fn ac_behavioural() {
    check_ac(
        "ac_behavioural",
        &parse("ac_behavioural"),
        "vin",
        "out",
        5,
        (10.0, 1e6),
        OP_V,
    );
}

#[test]
fn op_resistive() {
    check_op("op_resistive", &parse("op_resistive"), (OP_V, OP_I));
}

#[test]
fn op_global() {
    check_op("op_global", &parse("op_global"), (OP_V, OP_I));
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
fn op_bjt_pnp() {
    check_op("op_bjt_pnp", &parse("op_bjt_pnp"), (OP_V, OP_I));
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
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    })
}

fn resistor(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
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
        in_deck(
            "op_mosfet_cs",
            &circuit(vec![
                vsource("vdd", "vdd", "0", 10.0),
                vsource("vg", "g", "0", 3.0),
                resistor("rd", "vdd", "d", 10e3),
                CircuitElement::Mosfet(Mosfet {
                    id: "m1".into(),
                    nodes: ["g".into(), "d".into(), "0".into()],
                    kind: MosfetKind::Nmos,
                    params: MosfetParams::default_nmos(),
                    parasitic_caps: None,
                    body_diode: None,
                }),
            ]),
        ),
        (OP_V, OP_I),
    );
}

/// A three-terminal MOSFET's body is tied to its source, so a follower
/// sees no body effect; ngspice's `M1 vdd g s s`.
#[test]
fn op_mosfet_follower() {
    check_op(
        "op_mosfet_follower",
        in_deck(
            "op_mosfet_follower",
            &circuit(vec![
                vsource("vdd", "vdd", "0", 10.0),
                vsource("vg", "g", "0", 5.0),
                resistor("rs", "s", "0", 10e3),
                CircuitElement::Mosfet(Mosfet {
                    id: "m1".into(),
                    nodes: ["g".into(), "vdd".into(), "s".into()],
                    kind: MosfetKind::Nmos,
                    params: MosfetParams::default_nmos(),
                    parasitic_caps: None,
                    body_diode: None,
                }),
            ]),
        ),
        (OP_V, OP_I),
    );
}

/// PMOS with source and body at VDD; ngspice's `M1 d g vdd vdd`.
#[test]
fn op_pmos_highside() {
    check_op(
        "op_pmos_highside",
        in_deck(
            "op_pmos_highside",
            &circuit(vec![
                vsource("vdd", "vdd", "0", 10.0),
                vsource("vg", "g", "0", 5.0),
                resistor("rd", "d", "0", 10e3),
                CircuitElement::Mosfet(Mosfet {
                    id: "m1".into(),
                    nodes: ["g".into(), "d".into(), "vdd".into()],
                    kind: MosfetKind::Pmos,
                    params: MosfetParams::default_pmos(),
                    parasitic_caps: None,
                    body_diode: None,
                }),
            ]),
        ),
        (OP_V, OP_I),
    );
}

#[test]
fn op_jfet() {
    check_op(
        "op_jfet",
        in_deck(
            "op_jfet",
            &circuit(vec![
                vsource("vdd", "vdd", "0", 12.0),
                resistor("rd", "vdd", "d", 1e3),
                resistor("rs", "s", "0", 220.0),
                resistor("rg", "g", "0", 1e6),
                CircuitElement::Jfet(Jfet {
                    id: "j1".into(),
                    nodes: ["g".into(), "d".into(), "s".into()],
                    kind: JfetKind::NChannel,
                    idss: 10e-3,
                    vp: -2.0,
                    parasitic_caps: None,
                }),
            ]),
        ),
        JFET,
    );
}

#[test]
fn op_led() {
    check_op(
        "op_led",
        in_deck(
            "op_led",
            &circuit(vec![
                vsource("v1", "in", "0", 5.0),
                resistor("r1", "in", "out", 150.0),
                CircuitElement::Led(Led {
                    id: "d1".into(),
                    nodes: ["out".into(), "0".into()],
                    color: "red".into(),
                    temperature: 300.15,
                }),
            ]),
        ),
        (OP_V, OP_I),
    );
}

#[test]
fn op_schottky() {
    check_op(
        "op_schottky",
        in_deck(
            "op_schottky",
            &circuit(vec![
                vsource("v1", "in", "0", 5.0),
                resistor("r1", "in", "out", 1e3),
                CircuitElement::SchottkyDiode(SchottkyDiode {
                    id: "d1".into(),
                    nodes: ["out".into(), "0".into()],
                    temperature: 300.15,
                }),
            ]),
        ),
        (OP_V, OP_I),
    );
}

#[test]
fn op_photodiode() {
    check_op(
        "op_photodiode",
        in_deck(
            "op_photodiode",
            &circuit(vec![
                vsource("v1", "k", "0", 5.0),
                resistor("r1", "k", "a", 10e3),
                CircuitElement::Photodiode(Photodiode {
                    id: "d1".into(),
                    nodes: ["0".into(), "a".into()],
                    irradiance: 1e-4,
                    temperature: 300.15,
                }),
            ]),
        ),
        (OP_V, OP_I),
    );
}

// ---------------------------------------------------------------- .dc

/// The `op_reactive` deck swept: every point is an operating point, with
/// the inductor shorted and the capacitors open.
#[test]
fn dc_reactive() {
    let golden = Golden::load("dc_reactive");
    let sweep = dc_sweep(&parse("dc_reactive"), &DcSweepAxis::new("v1", 0.0, 2.0, 5)).unwrap();
    let scale = golden.scale();
    let column = |node: &str| -> Vec<f64> {
        sweep
            .points
            .iter()
            .map(|p| p.result.node_voltages[node])
            .collect()
    };
    let i: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| source_current(&p.result, "v1"))
        .collect();
    let mut check = Check::new("dc_reactive");
    check.series("v-sweep", &scale, &sweep_values(&sweep), &scale, OP_V);
    for node in ["mid", "b", "c"] {
        let name = format!("v({node})");
        check.series(&name, &scale, &column(node), &golden.column(&name), OP_V);
    }
    check.series("i(v1)", &scale, &i, &golden.column("i(v1)"), OP_I);
    check.finish();
}

#[test]
fn dc_diode_iv() {
    let golden = Golden::load("dc_diode_iv");
    let sweep = dc_sweep(&parse("dc_diode_iv"), &DcSweepAxis::new("v1", 0.0, 1.0, 21)).unwrap();
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
    let sweep = dc_sweep(
        &parse("dc_diode_1n4148"),
        &DcSweepAxis::new("v1", 0.0, 2.0, 21),
    )
    .unwrap();
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
    let decades = (f.1 / f.0).log10().round() as usize;
    let config = AcConfig::new(source, f.0, f.1, decades * per_decade + 1);
    check_ac_with(case, circuit, &config, node, mag_tol);
}

/// Runs `config` and compares `mag(v(node))` and, when the golden has it,
/// phase in degrees.
fn check_ac_with(
    case: &'static str,
    circuit: &Circuit,
    config: &AcConfig,
    node: &str,
    mag_tol: Tol,
) {
    let golden = Golden::load(case);
    let result = solve_ac(circuit, config).unwrap_or_else(|e| panic!("{case}: {e}"));
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
        // ngspice's `cph` unwraps the phase; take sindr's turn by turn
        // nearest it.
        let want = golden.column("ph");
        let phase: Vec<f64> = phasors
            .iter()
            .zip(&want)
            .map(|(v, w)| {
                let p = v.arg().to_degrees();
                p + 360.0 * ((w - p) / 360.0).round()
            })
            .collect();
        check.series_in(
            "phase (deg)",
            &scale,
            &phase,
            &want,
            Tol {
                rel: 0.0,
                // A relative error ε in the phasor turns its phase by up
                // to ε rad.
                abs: mag_tol.rel.to_degrees().max(1e-4),
            },
            Unit::Deg,
        );
    }
    check.finish();
}

/// The deck drives its own `.ac` from `V1 in 0 dc 1 ac 2 30`: the
/// stimulus's magnitude and phase come from the source line, the sweep from
/// the `.ac` card, with nothing set by hand.
#[test]
fn ac_deck_stimulus() {
    let netlist = parse_file(case_path("ac_deck_stimulus", "cir")).unwrap();
    let config = netlist.ac_config().unwrap().unwrap();
    assert_eq!((config.ac_magnitude, config.ac_phase_deg), (2.0, 30.0));
    check_ac_with(
        "ac_deck_stimulus",
        &netlist.circuit,
        &config,
        "out",
        AC_LINEAR,
    );
}

/// `.ac dec 10 1 50` spans 1.7 decades. ngspice sweeps ⌊10·1.7⌋ = 16 equal
/// log steps from 1 Hz to exactly 50 Hz, 17 points; the deck's config must
/// sweep the same ones.
#[test]
fn ac_partial_decade() {
    let netlist = parse_file(case_path("ac_partial_decade", "cir")).unwrap();
    let config = netlist.ac_config().unwrap().unwrap();
    assert_eq!(config.num_points, 17);
    assert_eq!(config.f_stop, 50.0);
    check_ac_with(
        "ac_partial_decade",
        &netlist.circuit,
        &config,
        "out",
        AC_LINEAR,
    );
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

/// `ac_bjt_ce` from a 1k source, with constant Cbe and Cbc: gm·RC, then
/// the Miller pole and the right-half-plane zero of Cbc.
#[test]
fn ac_bjt_caps() {
    let circuit = circuit(vec![
        vsource("vcc", "vcc", "0", 10.0),
        vsource("vin", "in", "0", 0.0),
        resistor("rs", "in", "s", 1e3),
        CircuitElement::Capacitor(Capacitor {
            id: "cin".into(),
            nodes: ["s".into(), "b".into()],
            capacitance: 10e-6,
        }),
        resistor("rb", "vcc", "b", 470e3),
        resistor("rc", "vcc", "c", 1e3),
        CircuitElement::Bjt(Bjt {
            id: "q1".into(),
            nodes: ["b".into(), "c".into(), "0".into()],
            kind: sindr::BjtKind::Npn,
            bf: 100.0,
            temperature: 300.15,
            parasitic_caps: Some(sindr::circuit::BjtParasiticCaps {
                cbe: 10e-12,
                cbc: 2e-12,
            }),
            model: None,
        }),
    ]);
    check_ac("ac_bjt_caps", &circuit, "vin", "c", 5, (10.0, 1e9), OP_V);
}

/// A forward-biased 1N4148 (about 4.3 mA): R1 against rd + RS, then the
/// diffusion and junction charge rolling off the top decades.
#[test]
fn ac_diode() {
    check_ac(
        "ac_diode",
        &parse("ac_diode"),
        "v1",
        "a",
        5,
        (1e3, 1e9),
        OP_V,
    );
}

/// Common-source NMOS fed from 10k: gm·RD at low frequency, then the
/// Miller pole of Cgd and the zero where Cgd feeds forward.
#[test]
fn ac_mosfet_cs() {
    let circuit = circuit(vec![
        vsource("vdd", "vdd", "0", 10.0),
        vsource("vg", "in", "0", 3.0),
        resistor("rg", "in", "g", 10e3),
        resistor("rd", "vdd", "d", 10e3),
        CircuitElement::Mosfet(Mosfet {
            id: "m1".into(),
            nodes: ["g".into(), "d".into(), "0".into()],
            kind: MosfetKind::Nmos,
            params: MosfetParams::default_nmos(),
            parasitic_caps: Some(sindr::circuit::MosfetParasiticCaps {
                cgs: 10e-12,
                cgd: 2e-12,
            }),
            body_diode: None,
        }),
    ]);
    check_ac("ac_mosfet_cs", &circuit, "vg", "d", 5, (1e3, 1e9), OP_V);
}

/// Self-biased JFET, AC-coupled into the gate with the source bypassed:
/// the input high-pass, then gm·RD.
#[test]
fn ac_jfet() {
    let circuit = circuit(vec![
        vsource("vdd", "vdd", "0", 12.0),
        vsource("vin", "in", "0", 0.0),
        CircuitElement::Capacitor(Capacitor {
            id: "cin".into(),
            nodes: ["in".into(), "g".into()],
            capacitance: 100e-9,
        }),
        resistor("rg", "g", "0", 1e6),
        resistor("rd", "vdd", "d", 1e3),
        resistor("rs", "s", "0", 220.0),
        CircuitElement::Capacitor(Capacitor {
            id: "cs".into(),
            nodes: ["s".into(), "0".into()],
            capacitance: 100e-6,
        }),
        CircuitElement::Jfet(Jfet {
            id: "j1".into(),
            nodes: ["g".into(), "d".into(), "s".into()],
            kind: JfetKind::NChannel,
            idss: 10e-3,
            vp: -2.0,
            parasitic_caps: None,
        }),
    ]);
    check_ac("ac_jfet", &circuit, "vin", "d", 5, (0.1, 100e3), JFET.0);
}

/// A JFET common-source stage driven from 10 kΩ: the input pole of CGS plus
/// the Miller-multiplied CGD, both at their reverse-biased junction values
/// (`C0/√(1 − v/PB)`), and the feedforward zero of CGD. Built in Rust and
/// read from the deck, whose `NJF` card carries `CGS`, `CGD`, `PB` and `FC`.
#[test]
fn ac_jfet_caps() {
    let circuit = circuit(vec![
        vsource("vdd", "vdd", "0", 12.0),
        vsource("vin", "in", "0", -1.0),
        resistor("rsig", "in", "g", 10e3),
        resistor("rd", "vdd", "d", 1e3),
        CircuitElement::Jfet(Jfet {
            id: "j1".into(),
            nodes: ["g".into(), "d".into(), "0".into()],
            kind: JfetKind::NChannel,
            idss: 10e-3,
            vp: -2.0,
            parasitic_caps: Some(sindr::circuit::JfetParasiticCaps {
                cgs: 10e-12,
                cgd: 2e-12,
                pb: 0.8,
                fc: 0.5,
                ..Default::default()
            }),
        }),
    ]);
    // The gate is reverse-biased, so ngspice's default gate IS adds next to
    // nothing and the case holds to the junction devices' 1e-6.
    let range = (10e3, 1e9);
    check_ac(
        "ac_jfet_caps",
        in_deck("ac_jfet_caps", &circuit),
        "vin",
        "d",
        5,
        range,
        OP_V,
    );
    let parsed = parse("ac_jfet_caps");
    assert!(matches!(
        parsed.components.iter().find(|c| c.id() == "j1"),
        Some(CircuitElement::Jfet(Jfet {
            parasitic_caps: Some(_),
            ..
        }))
    ));
    check_ac("ac_jfet_caps", &parsed, "vin", "d", 5, range, OP_V);
}

// ------------------------------------------------- M and J lines, parsed

/// The FET decks above, read by sindr's parser instead of built in Rust.
/// Their cards leave `IS` at SPICE's 1e-14 A, so each MOSFET also gets
/// that drain–bulk junction as its body diode, reverse-biased here.
#[test]
fn op_fet_decks_parse() {
    for case in ["op_mosfet_cs", "op_mosfet_follower", "op_pmos_highside"] {
        check_op(case, &parse(case), (OP_V, OP_I));
    }
    check_op("op_jfet", &parse("op_jfet"), JFET);
}

#[test]
fn ac_fet_decks_parse() {
    check_ac(
        "ac_mosfet_cs",
        &parse("ac_mosfet_cs"),
        "vg",
        "d",
        5,
        (1e3, 1e9),
        OP_V,
    );
    check_ac(
        "ac_jfet",
        &parse("ac_jfet"),
        "vin",
        "d",
        5,
        (0.1, 100e3),
        JFET.0,
    );
}

/// A level-1 card with what the M line maps beyond the defaults: `W`, `L`,
/// `LD` and `M` into `kp`, `CGSO`/`CGDO`/`CGBO` into the gate capacitances
/// and `CBD`/`PB`/`MJ` into the body diode's junction capacitance.
#[test]
fn ac_mosfet_card() {
    let circuit = parse("ac_mosfet_card");
    let Some(CircuitElement::Mosfet(Mosfet {
        params,
        parasitic_caps: Some(caps),
        body_diode: Some(body),
        ..
    })) = circuit.components.iter().find(|c| c.id() == "m1")
    else {
        panic!("ac_mosfet_card: m1 should be a MOSFET with caps and a body diode");
    };
    let (w, l_eff, m) = (20e-6, 2e-6 - 2.0 * 0.1e-6, 2.0);
    assert!((params.kp - 50e-6 * w / l_eff * m).abs() < 1e-12);
    assert!((caps.cgs - (1e-9 * w + 0.2e-9 * l_eff) * m).abs() < 1e-20);
    assert!((caps.cgd - 0.3e-9 * w * m).abs() < 1e-20);
    assert!((body.cjo - 5e-12 * m).abs() < 1e-20);
    check_ac("ac_mosfet_card", &circuit, "vg", "d", 5, (1e3, 1e9), OP_V);
}

/// A P-channel JFET with an area factor: `IDSS = BETA·VTO²·area`.
#[test]
fn op_pjf() {
    check_op("op_pjf", &parse("op_pjf"), JFET);
}

/// A DC sweep of a parsed deck: `v(<node>)` and `i(<source>)` against
/// ngspice's. Off, ngspice adds GMIN leakage sindr doesn't, hence 1 nA.
fn check_dc_deck(case: &'static str, source: &str, from: f64, to: f64) {
    let golden = Golden::load(case);
    let scale = golden.scale();
    let sweep = dc_sweep(
        &parse(case),
        &DcSweepAxis::new(source, from, to, scale.len()),
    )
    .unwrap_or_else(|e| panic!("{case}: {e}"));
    let mut check = Check::new(case);
    for name in golden.names.iter().skip(1) {
        let inner = &name[2..name.len() - 1];
        let (values, tol): (Vec<f64>, Tol) = if name.starts_with("v(") {
            let v = sweep.points.iter().map(|p| p.result.node_voltages[inner]);
            (v.collect(), OP_V)
        } else {
            let i = sweep
                .points
                .iter()
                .map(|p| source_current(&p.result, inner));
            (
                i.collect(),
                Tol {
                    rel: 1e-4,
                    abs: 1e-9,
                },
            )
        };
        check.series(name, &scale, &values, &golden.column(name), tol);
    }
    check.finish();
}

/// A vendor-style power MOSFET: a `.subckt` holding a gate resistor, a
/// level-1 core, an explicit body diode and its own `.model` cards,
/// switching a 1 Ω load from off to fully on.
#[test]
fn dc_power_mosfet_subckt() {
    check_dc_deck("dc_power_mosfet_subckt", "vg", 0.0, 10.0);
}

/// The same part held off, its drain pulled below its source: the body
/// diode from the subckt's local card conducts amps.
#[test]
fn dc_power_mosfet_body() {
    check_dc_deck("dc_power_mosfet_body", "vd", -3.0, 0.0);
}

/// The body-diode deck parsed: its MOSFET card switches the built-in
/// junction off (`IS=1e-30`) and the explicit `D1` does the conducting.
#[test]
fn dc_mosfet_body_diode_deck_parses() {
    let golden = Golden::load("dc_mosfet_body_diode");
    let sweep = dc_sweep(
        &parse("dc_mosfet_body_diode"),
        &DcSweepAxis::new("v1", -5.0, 5.0, 41),
    )
    .unwrap();
    let vd: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["d"])
        .collect();
    let mut check = Check::new("dc_mosfet_body_diode");
    check.series("v(d)", &golden.scale(), &vd, &golden.column("v(d)"), OP_V);
    check.finish();
}

// ---------------------------------------------------------------- noise

fn noise(
    circuit: &Circuit,
    source: &str,
    node: &str,
    per_decade: usize,
    f: (f64, f64),
) -> NoiseResult {
    let decades = (f.1 / f.0).log10().round() as usize;
    solve_noise(
        circuit,
        &NoiseConfig::new(node, source, f.0, f.1, decades * per_decade + 1),
    )
    .unwrap_or_else(|e| panic!("noise: {e}"))
}

fn check_noise(case: &'static str, result: &NoiseResult, tol: Tol) {
    let golden = Golden::load(case);
    let scale = golden.scale();
    let freqs: Vec<f64> = result.points.iter().map(|p| p.frequency).collect();
    let onoise: Vec<f64> = result.points.iter().map(|p| p.output_density()).collect();
    let inoise: Vec<f64> = result.points.iter().map(|p| p.input_density()).collect();
    let mut check = Check::new(case);
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
    check.series(
        "onoise_spectrum",
        &scale,
        &onoise,
        &golden.column("onoise_spectrum"),
        tol,
    );
    check.series(
        "inoise_spectrum",
        &scale,
        &inoise,
        &golden.column("inoise_spectrum"),
        tol,
    );
    check.finish();
}

/// Resistor thermal noise through an RC: 4kTR at the bottom of the band,
/// rolling off past the pole.
#[test]
fn noise_rc_lowpass() {
    let result = noise(&parse("noise_rc_lowpass"), "v1", "out", 10, (1.0, 100e3));
    check_noise(
        "noise_rc_lowpass",
        &result,
        Tol {
            rel: 1e-6,
            abs: 0.0,
        },
    );
}

/// The band totals, which ngspice integrates as a power law between points.
///
/// Only `onoise_total` is compared. Referred to V1, the RC's noise is R1's
/// flat 4kTR, so `inoise_total` is exactly √(4kTR·(f_stop − f_start)), which
/// sindr gives. ngspice's 1.444 µV is about 11 % high: its input-noise
/// integral applies each point's gain to both ends of the interval before
/// it, which is off wherever the gain is rolling off.
#[test]
fn noise_rc_total() {
    let result = noise(&parse("noise_rc_total"), "v1", "out", 10, (1.0, 100e3));
    let golden = Golden::load("noise_rc_total");
    let mut check = Check::new("noise_rc_total");
    let tol = Tol {
        rel: 1e-4,
        abs: 0.0,
    };
    check.value(
        "onoise_total",
        result.total_output_noise,
        golden.column("onoise_total")[0],
        tol,
    );
    check.finish();

    let four_ktr: f64 = 4.0 * 1.380_649e-23 * 300.15 * 1e3;
    let exact = (four_ktr * (100e3 - 1.0)).sqrt();
    assert!(
        (result.total_input_noise / exact - 1.0).abs() < 1e-9,
        "inoise_total {} vs {exact}",
        result.total_input_noise
    );
}

/// Collector and base shot noise of a biased NPN, through the stage's gain.
#[test]
fn noise_bjt_ce() {
    let result = noise(&parse("noise_bjt_ce"), "vin", "c", 5, (10.0, 100e3));
    check_noise(
        "noise_bjt_ce",
        &result,
        Tol {
            rel: 1e-5,
            abs: 0.0,
        },
    );
}

/// `.noise ... dec 10 1 50`: ngspice's noise sweep steps by 10^(1/10) from
/// 1 Hz and stops at 10^1.6 ≈ 39.8 Hz, the last point not past 50 Hz, unlike
/// `.ac` over the same band.
#[test]
fn noise_partial_decade() {
    let netlist = parse_file(case_path("noise_partial_decade", "cir")).unwrap();
    let config = netlist.analyses[0].noise_config().unwrap();
    assert_eq!(config.num_points, 17);
    let result = solve_noise(&netlist.circuit, &config).unwrap();
    check_noise(
        "noise_partial_decade",
        &result,
        Tol {
            rel: 1e-6,
            abs: 0.0,
        },
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

fn with_names(id: &str, nodes: [&str; 2]) -> (String, [sindr::NodeId; 2]) {
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
        CircuitElement::CurrentSource(CurrentSource {
            id: "i1".into(),
            nodes: ["i".into(), "0".into()],
            current: 1e-3,
            waveform: None,
        }),
        CircuitElement::Potentiometer(Potentiometer {
            id: "rpot".into(),
            nodes: ["in".into(), "w".into(), "0".into()],
            resistance: 10e3,
            position: 0.3,
        }),
        CircuitElement::Switch(Switch {
            id: "sw1".into(),
            nodes: ["w".into(), "x".into()],
            closed: true,
        }),
    ];
    let (id, nodes) = two("e1", ["e", "0"]);
    components.push(CircuitElement::Vcvs(Vcvs {
        id,
        nodes,
        control_nodes: ["a".into(), "0".into()],
        gain: 3.0,
    }));
    let (id, nodes) = two("g1", ["0", "g"]);
    components.push(CircuitElement::Vccs(Vccs {
        id,
        nodes,
        control_nodes: ["a".into(), "0".into()],
        gm: 1e-3,
    }));
    let (id, nodes) = two("h1", ["h", "0"]);
    components.push(CircuitElement::Ccvs(Ccvs {
        id,
        nodes,
        control_source: "v1".into(),
        rm: 100.0,
    }));
    let (id, nodes) = two("f1", ["0", "f"]);
    components.push(CircuitElement::Cccs(Cccs {
        id,
        nodes,
        control_source: "v1".into(),
        alpha: 2.0,
    }));
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
    // The same deck read back by sindr's parser.
    check_op(
        "op_exported_controlled",
        &parse("op_exported_controlled"),
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
            CircuitElement::Transformer(Transformer {
                id: "t1".into(),
                nodes: ["p".into(), "0".into(), "s".into(), "0".into()],
                l1: 1e-3,
                l2: 4e-3,
                k: 0.99,
                saturation: None,
            }),
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
    // The same deck read back by sindr's parser: `L`, `L` and `K` become
    // one transformer again.
    check_ac(
        "ac_exported_transformer",
        &parse("ac_exported_transformer"),
        "v1",
        "s",
        10,
        (10.0, 1e6),
        AC_LINEAR,
    );
}

/// Every FET `to_netlist` writes: both JFET polarities, and a MOSFET of
/// each kind, one body diode on the card's drain–bulk junction and one, with
/// series resistance and breakdown SPICE's junction lacks, as its own `D`.
fn exported_fets() -> Circuit {
    circuit(vec![
        vsource("vdd", "vdd", "0", 12.0),
        resistor("rjd", "vdd", "jd", 1e3),
        resistor("rjs", "js", "0", 220.0),
        resistor("rjg", "jg", "0", 1e6),
        CircuitElement::Jfet(Jfet {
            id: "j1".into(),
            nodes: ["jg".into(), "jd".into(), "js".into()],
            kind: JfetKind::NChannel,
            idss: 10e-3,
            vp: -2.0,
            parasitic_caps: None,
        }),
        resistor("rps", "vdd", "ps", 330.0),
        resistor("rpg", "pg", "vdd", 1e6),
        resistor("rpd", "pd", "0", 1e3),
        CircuitElement::Jfet(Jfet {
            id: "j2".into(),
            nodes: ["pg".into(), "pd".into(), "ps".into()],
            kind: JfetKind::PChannel,
            idss: 12.5e-3,
            vp: 2.5,
            parasitic_caps: None,
        }),
        vsource("vg1", "g1", "0", 3.0),
        resistor("rd1", "vdd", "d1", 10e3),
        CircuitElement::Mosfet(Mosfet {
            id: "m1".into(),
            nodes: ["g1".into(), "d1".into(), "0".into()],
            kind: MosfetKind::Nmos,
            params: MosfetParams::default_nmos(),
            parasitic_caps: None,
            body_diode: Some(sindr::DiodeModel {
                is: 1e-12,
                n: 1.2,
                rs: 0.05,
                bv: Some(60.0),
                ibv: 250e-6,
                ..Default::default()
            }),
        }),
        vsource("vg2", "g2", "0", 5.0),
        resistor("rd2", "d2", "0", 10e3),
        CircuitElement::Mosfet(Mosfet {
            id: "m2".into(),
            nodes: ["g2".into(), "d2".into(), "vdd".into()],
            kind: MosfetKind::Pmos,
            params: MosfetParams::default_pmos(),
            parasitic_caps: Some(sindr::circuit::MosfetParasiticCaps {
                cgs: 10e-12,
                cgd: 2e-12,
            }),
            body_diode: Some(sindr::DiodeModel {
                is: 1e-14,
                cjo: 5e-12,
                vj: 0.8,
                ..Default::default()
            }),
        }),
    ])
}

#[test]
fn op_exported_fets() {
    let circuit = exported_fets();
    check_op(
        "op_exported_fets",
        exported("op_exported_fets", &circuit),
        JFET,
    );
    // Read back, the deck solves the same: the separate body diode is then
    // `dm1_body` beside a junction-less `m1`.
    let parsed = parse("op_exported_fets");
    check_op("op_exported_fets", &parsed, JFET);
    assert!(parsed.components.iter().any(|c| c.id() == "dm1_body"));
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
    compare_tran_within(case, vectors, tran, 1e-3);
}

/// As [`compare_tran`], to `fraction` of each vector's full scale.
fn compare_tran_within(
    case: &'static str,
    vectors: &[&str],
    tran: &sindr::TransientData,
    fraction: f64,
) {
    compare_steps_within(case, vectors, &tran.timesteps, fraction);
}

/// As [`compare_tran_within`], for steps from anywhere: a transient
/// session's.
fn compare_steps_within(
    case: &'static str,
    vectors: &[&str],
    steps: &[sindr::TimestepSnapshot],
    fraction: f64,
) {
    let golden = Golden::load(case);
    let times: Vec<f64> = steps.iter().map(|s| s.time).collect();

    let mut check = Check::new(case);
    let scale = golden.scale();
    for name in vectors {
        let want_all = golden.column(name);
        let full_scale = want_all.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let inner = &name[2..name.len() - 1];
        let got: Vec<f64> = steps
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
        check.series_fs(name, &times, &got, &want, fraction, full_scale);
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

/// The deck's PULSE driven from outside instead: a transient session sets
/// the source to each level at each millisecond, the way firmware drives a
/// pin. The 1 ns edges ngspice integrates are the only difference.
#[test]
fn tran_session_square() {
    let case = "tran_session_square";
    let mut circuit = parse(case);
    for element in &mut circuit.components {
        if let CircuitElement::VoltageSource(source) = element {
            source.waveform = None;
            source.voltage = 0.0;
        }
    }
    let config = sindr::TransientConfig::new(10e-3, 10e-6);
    let mut sim = sindr::TransientSession::new(&circuit, &config).unwrap();
    for k in 1..=10 {
        sim.advance_to(f64::from(k) * 1e-3).unwrap();
        sim.set_source("v1", if k % 2 == 1 { 5.0 } else { 0.0 })
            .unwrap();
    }
    let steps: Vec<_> = sim.drain_steps().collect();
    compare_steps_within(case, &["v(out)"], &steps, 1e-3);
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

/// Runs the deck's own `.tran` (and `.ic`) as ngspice does, through
/// `ParsedNetlist::transient_config`, and compares `vectors` to 0.1 % of
/// full scale.
fn check_deck_tran(case: &'static str, vectors: &[&str]) -> sindr::TransientData {
    check_deck_tran_within(case, vectors, 1e-3)
}

/// As [`check_deck_tran`], to `fraction` of each vector's full scale.
fn check_deck_tran_within(
    case: &'static str,
    vectors: &[&str],
    fraction: f64,
) -> sindr::TransientData {
    let netlist = parse_file(case_path(case, "cir")).unwrap();
    let config = netlist.transient_config().expect(".tran in the deck");
    let tran = sindr::solve_transient(&netlist.circuit, &config)
        .unwrap_or_else(|e| panic!("{case}: {e}"))
        .transient
        .unwrap();
    compare_tran_within(case, vectors, &tran, fraction);
    tran
}

/// `.ic` without `UIC`: `a` is held at 3 V while the operating point is
/// solved, so C1 starts charged to 3 V, and C2, not listed, at its
/// operating point (1 V). Then `a` decays to the divider's 0.5 V.
#[test]
fn tran_ic_op() {
    let tran = check_deck_tran("tran_ic_op", &["v(a)", "v(b)"]);
    assert!((tran.timesteps[0].node_voltages["b"] - 1.0).abs() < 1e-3);
}

/// The same deck with `UIC`: C1 starts at its `.ic` 3 V, and C2, not
/// listed, uncharged, so `b` charges towards 1 V.
#[test]
fn tran_ic_uic() {
    let tran = check_deck_tran("tran_ic_uic", &["v(a)", "v(b)"]);
    assert!(tran.timesteps[0].node_voltages["b"] < 1e-2);
}

/// `.tran … tstart`: integrated from 0, so the pulse train has charged C1
/// for 1.25 ms, but kept only from there on. sindr's first step lands on
/// tstart; ngspice's first saved point is its first step past it.
#[test]
fn tran_tstart() {
    let tran = check_deck_tran("tran_tstart", &["v(a)", "i(v1)"]);
    assert!((tran.timesteps[0].time - 1.25e-3).abs() < 1e-12);
}

/// Sine into a diode and filter cap. The deck runs with `uic`, from rest,
/// as ngspice does; sindr steps adaptively at its defaults, and the golden
/// is ngspice at RELTOL 1e-6 and a short TMAX.
#[test]
fn tran_halfwave() {
    check_deck_tran("tran_halfwave", &["v(b)", "i(v1)"]);
}

/// `.four` on a sine clipped by antiparallel diodes, against ngspice's
/// `fourier 1k v(out)` on the same run (THD 29.0343 %, 200-point grid,
/// linear interpolation, one period). Two checks: `Fourier` on ngspice's
/// own waveform reproduces ngspice's table, which pins the algorithm; and
/// `Fourier` on sindr's waveform agrees with it, which pins the solve.
/// The golden is the waveform; ngspice prints the table it's checked
/// against when the `.ctl` is run, and it is copied in here.
#[test]
fn four_diode_clipper() {
    use sindr::fourier::Fourier;
    use sindr::measure::Signal;

    let case = "four_diode_clipper";
    // (harmonic, magnitude, phase in degrees) from ngspice 47.
    let ngspice = [
        (1, 0.814221, 0.000163373),
        (3, 0.214814, 0.000766763),
        (5, 0.0907753, 0.00218746),
        (7, 0.0372933, 0.0055055),
        (9, 0.0105286, 0.0137194),
    ];
    let golden = Golden::load(case);
    let trace: Vec<(f64, f64)> = golden
        .column("time")
        .into_iter()
        .zip(golden.column("v(out)"))
        .collect();
    let four = Fourier::new(1e3);

    let from_ngspice = four.of(&trace).unwrap();
    assert!(
        (from_ngspice.thd_percent - 29.0343).abs() < 1e-3,
        "{case}: THD of ngspice's waveform {} %",
        from_ngspice.thd_percent
    );
    for (k, mag, phase) in ngspice {
        let h = from_ngspice.harmonic(k).unwrap();
        assert!(
            (h.magnitude - mag).abs() < 1e-5 * mag.max(0.01),
            "{case}: {h:?}"
        );
        assert!((h.phase - phase).abs() < 1e-3, "{case}: {h:?}");
    }
    for k in [2, 4, 6, 8] {
        assert!(from_ngspice.harmonic(k).unwrap().magnitude < 1e-6);
    }

    let netlist = parse_file(case_path(case, "cir")).unwrap();
    // The deck's `.four 1k v(out)` is the analysis built by hand above.
    let [card] = netlist.fourier.as_slice() else {
        panic!("{case}: one `.four` signal, got {:?}", netlist.fourier);
    };
    assert_eq!((&card.fourier, &card.signal), (&four, &Signal::v("out")));
    let config = netlist.transient_config().expect(".tran in the deck");
    let result =
        sindr::solve_transient(&netlist.circuit, &config).unwrap_or_else(|e| panic!("{case}: {e}"));
    // sindr reads 29.067 %: the clipped peaks differ by a fraction of a mV.
    let from_sindr = card.fourier.on(&result, &card.signal).unwrap();
    assert!(
        (from_sindr.thd_percent - 29.0343).abs() < 0.05,
        "{case}: THD of sindr's waveform {} %",
        from_sindr.thd_percent
    );
    for (k, mag, _) in ngspice {
        let h = from_sindr.harmonic(k).unwrap();
        // 0.1 % of the fundamental, the transient goldens' full-scale bound.
        assert!((h.magnitude - mag).abs() < 1e-3 * 0.814221, "{case}: {h:?}");
    }
}

/// The base RC is checked to 0.1 % of full scale. The collector edge is
/// looser: in the active region ic = IS·exp(vbe/Vt), so the base's error
/// (0.4 mV here) comes out about vbe/Vt ≈ 27 times larger in ic, and the
/// edge falls 5 V in 3 µs. Allow 3 %.
#[test]
fn tran_bjt_switch() {
    check_deck_tran("tran_bjt_switch", &["v(b)"]);
    check_deck_tran_within("tran_bjt_switch", &["v(c)", "i(vcc)"], 3e-2);
}

/// Rises of `v` through `level` on the time base `t`, linearly
/// interpolated.
fn rising_through(t: &[f64], v: &[f64], level: f64) -> Vec<f64> {
    (1..t.len())
        .filter(|&i| v[i - 1] < level && v[i] >= level)
        .map(|i| t[i - 1] + (level - v[i - 1]) / (v[i] - v[i - 1]) * (t[i] - t[i - 1]))
        .collect()
}

/// The two-transistor LED flasher from rest. Its transistors have no
/// junction charge, so each edge is a jump that sindr takes by
/// pseudo-transient continuation, and the waveforms can't be compared
/// point by point (an edge a fraction of a millisecond off is volts off).
/// What's compared is what the circuit is for: the period, from each
/// collector's rises through 4 V after the first, to 0.2 %, and the current
/// in the LED that's on (its peak), to 0.5 %. The golden is ngspice at a
/// 10 µs TMAX, resampled to 1 ms, which places a rise to about 10 µs.
#[test]
fn tran_flasher() {
    let case = "tran_flasher";
    let golden = Golden::load(case);
    let config = sindr::TransientConfig::new(1.2, 1e-4);
    let tran = sindr::solve_transient(&parse(case), &config)
        .unwrap_or_else(|e| panic!("{case}: {e}"))
        .transient
        .unwrap();
    let time: Vec<f64> = tran.timesteps.iter().map(|s| s.time).collect();
    let node =
        |n: &str| -> Vec<f64> { tran.timesteps.iter().map(|s| s.node_voltages[n]).collect() };
    let steady_period =
        |rises: &[f64]| (rises[rises.len() - 1] - rises[1]) / (rises.len() - 2) as f64;
    let mut check = Check::new(case);
    for (collector, anode, led) in [("c1n", "a1", "d1"), ("c2n", "a2", "d2")] {
        let ours = rising_through(&time, &node(collector), 4.0);
        let theirs = rising_through(
            &golden.scale(),
            &golden.column(&format!("v({collector})")),
            4.0,
        );
        assert!(theirs.len() >= 4, "{case}: ngspice rises {theirs:?}");
        assert_eq!(
            ours.len(),
            theirs.len(),
            "{case}: {collector} rises {ours:?} vs {theirs:?}"
        );
        check.value(
            &format!("period of v({collector})"),
            steady_period(&ours),
            steady_period(&theirs),
            Tol {
                rel: 2e-3,
                abs: 0.0,
            },
        );

        let peak = |i: Vec<f64>| i.into_iter().fold(0.0, f64::max);
        let ours = peak(
            tran.timesteps
                .iter()
                .map(|s| {
                    s.component_results
                        .iter()
                        .find(|c| c.id == led)
                        .unwrap()
                        .current_through
                })
                .collect(),
        );
        let theirs = peak(
            golden
                .column(&format!("v({anode})"))
                .iter()
                .zip(golden.column(&format!("v({collector})")))
                .map(|(a, c)| (a - c) / 470.0)
                .collect(),
        );
        check.value(
            &format!("peak i({led})"),
            ours,
            theirs,
            Tol {
                rel: 5e-3,
                abs: 0.0,
            },
        );
    }
    check.finish();
}

#[test]
fn tran_rlc_ring() {
    check_deck_tran("tran_rlc_ring", &["v(b)", "i(v1)"]);
}

/// A B-source time function into an RC, from the deck's own `.tran`.
#[test]
fn tran_behavioural() {
    check_deck_tran("tran_behavioural", &["v(s)", "v(out)"]);
}

// ---------------------------------------------------------------- temperature

#[test]
fn temp_diode() {
    let golden = Golden::load("temp_diode");
    let kelvin: Vec<f64> = golden.scale().iter().map(|c| c + 273.15).collect();
    let sweep = temperature_sweep(
        &parse("temp_diode"),
        &TempSweepConfig::new(kelvin[0], *kelvin.last().unwrap(), kelvin.len()),
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

/// A vendor-style card stating silicon's EG, XTI and TNOM, with N and RS
/// off the defaults: it parses in strict mode and follows ngspice across
/// the sweep, since those are the values sindr scales IS with.
#[test]
fn temp_diode_card() {
    let golden = Golden::load("temp_diode_card");
    let kelvin: Vec<f64> = golden.scale().iter().map(|c| c + 273.15).collect();
    let sweep = temperature_sweep(
        &parse("temp_diode_card"),
        &TempSweepConfig::new(kelvin[0], *kelvin.last().unwrap(), kelvin.len()),
    )
    .unwrap();
    let got: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["out"])
        .collect();
    let mut check = Check::new("temp_diode_card");
    check.series("v(out)", &kelvin, &got, &golden.column("v(out)"), OP_V);
    check.finish();
}

/// A Schottky-style card: IS follows EG = 0.69 eV and XTI = 2, so it
/// rises far more slowly with temperature than silicon's.
#[test]
fn temp_schottky_card() {
    let golden = Golden::load("temp_schottky_card");
    let kelvin: Vec<f64> = golden.scale().iter().map(|c| c + 273.15).collect();
    let sweep = temperature_sweep(
        &parse("temp_schottky_card"),
        &TempSweepConfig::new(kelvin[0], *kelvin.last().unwrap(), kelvin.len()),
    )
    .unwrap();
    let got: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["out"])
        .collect();
    let mut check = Check::new("temp_schottky_card");
    check.series("v(out)", &kelvin, &got, &golden.column("v(out)"), OP_V);
    check.finish();
}

/// A BJT card with its own EG and XTI, biased by a base resistor: Vbe
/// falls and the collector current rises with temperature as ngspice's.
#[test]
fn temp_bjt_card() {
    let golden = Golden::load("temp_bjt_card");
    let kelvin: Vec<f64> = golden.scale().iter().map(|c| c + 273.15).collect();
    let sweep = temperature_sweep(
        &parse("temp_bjt_card"),
        &TempSweepConfig::new(kelvin[0], *kelvin.last().unwrap(), kelvin.len()),
    )
    .unwrap();
    let mut check = Check::new("temp_bjt_card");
    for node in ["b", "c"] {
        let got: Vec<f64> = sweep
            .points
            .iter()
            .map(|p| p.result.node_voltages[node])
            .collect();
        let name = format!("v({node})");
        check.series(&name, &kelvin, &got, &golden.column(&name), OP_V);
    }
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
    let sweep = dc_sweep(
        &parse("dc_diode_breakdown"),
        &DcSweepAxis::new("v1", 0.0, -150.0, 31),
    )
    .unwrap();
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

/// A 5.1 V `ZenerDiode` as a shunt regulator through 1 kΩ, swept from
/// forward bias to well past its knee. sindr stamps a zener as the SPICE
/// diode in the deck (BV = vz at IBV = 5 mA, RS = 1 Ω), so ngspice reads
/// the same model. Built in Rust: the deck's `D` card would come back as a
/// plain diode.
#[test]
fn dc_zener() {
    let golden = Golden::load("dc_zener");
    let circuit = circuit(vec![
        vsource("v1", "a", "0", 0.0),
        resistor("r1", "a", "b", 1e3),
        CircuitElement::ZenerDiode(ZenerDiode {
            id: "d1".into(),
            nodes: ["0".into(), "b".into()],
            vz: 5.1,
            temperature: 300.15,
        }),
    ]);
    let sweep = dc_sweep(&circuit, &DcSweepAxis::new("v1", -2.0, 12.0, 29)).unwrap();
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
    // Below the knee both carry picoamps; ngspice's 1 pS GMIN is half of
    // it there, as in `dc_diode_breakdown`, so currents get 1 nA.
    let mut check = Check::new("dc_zener");
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

/// An NMOS held off, its drain pulled below its source through 10 Ω: the
/// body diode carries the current, as in a bridge's freewheeling interval.
/// The gate sits at −5 V so the channel stays off even with source and
/// drain swapped; at 0 V it would turn on near VTO, where ngspice's body
/// effect (bulk at node 0) and sindr's 3-terminal model part ways.
/// ngspice's level-1 bulk junctions are switched off (`IS=1e-30`) and the
/// body diode is an explicit `D` with the card sindr's `body_diode` holds.
#[test]
fn dc_mosfet_body_diode() {
    let golden = Golden::load("dc_mosfet_body_diode");
    let body = sindr::DiodeModel {
        is: 1e-12,
        n: 1.2,
        rs: 0.05,
        bv: Some(60.0),
        ibv: 250e-6,
        ..Default::default()
    };
    let circuit = circuit(vec![
        vsource("v1", "a", "0", 0.0),
        vsource("vg", "g", "0", -5.0),
        resistor("r1", "a", "d", 10.0),
        CircuitElement::Mosfet(Mosfet {
            id: "m1".into(),
            nodes: ["g".into(), "d".into(), "0".into()],
            kind: MosfetKind::Nmos,
            params: MosfetParams::default_nmos(),
            parasitic_caps: None,
            body_diode: Some(body),
        }),
    ]);
    let sweep = dc_sweep(&circuit, &DcSweepAxis::new("v1", -5.0, 5.0, 41)).unwrap();
    let scale = golden.scale();
    let vd: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| p.result.node_voltages["d"])
        .collect();
    let i: Vec<f64> = sweep
        .points
        .iter()
        .map(|p| source_current(&p.result, "v1"))
        .collect();
    let mut check = Check::new("dc_mosfet_body_diode");
    check.series("v(d)", &scale, &vd, &golden.column("v(d)"), OP_V);
    // Off, both carry picoamps; ngspice adds GMIN leakage sindr doesn't.
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
    // The body diode's current is reported under `m1.body`.
    let on = &sweep.points[0].result;
    let body_current = on
        .component_results
        .iter()
        .find(|c| c.id == "m1.body")
        .expect("m1.body result")
        .current_through;
    assert!((body_current - golden.column("i(v1)")[0]).abs() < 1e-3);
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
            CircuitElement::Diode(Diode { model: Some(m), .. }) => Some(*m),
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
                if let CircuitElement::VoltageSource(VoltageSource { id, voltage, .. }) = c {
                    if id == "v1" {
                        *voltage = v;
                    }
                }
            }
            let ac = solve_ac(
                &circuit,
                &AcConfig::new("v1", f, f, 1).with_spacing(FrequencySpacing::Linear),
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
    let config = parsed.transient_config().expect(".tran in the deck");
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
        check.series_fs(name, &times, &got, &want, 2e-2, full_scale);
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

/// An SPDT switch steering 5 V to one of two loads, and a second one in
/// the other position. The deck is the two resistors each one exports as.
#[test]
fn op_exported_spdt() {
    use sindr::circuit::SpdtPosition;
    let spdt = |id: &str, nodes: [&str; 3], position| {
        CircuitElement::SwitchSpdt(SwitchSpdt {
            id: id.into(),
            nodes: nodes.map(sindr::NodeId::from),
            position,
        })
    };
    let circuit = circuit(vec![
        vsource("v1", "in", "0", 5.0),
        resistor("r1", "in", "com", 100.0),
        spdt("s1", ["com", "a", "b"], SpdtPosition::A),
        resistor("ra", "a", "0", 1e3),
        resistor("rb", "b", "0", 2e3),
        spdt("s2", ["com", "c", "d"], SpdtPosition::B),
        resistor("rc", "c", "0", 3e3),
        resistor("rd", "d", "0", 4e3),
    ]);
    check_op(
        "op_exported_spdt",
        exported("op_exported_spdt", &circuit),
        (OP_V, OP_I),
    );
}

/// A 16 MHz crystal in series between 50 Ω source and load, through its
/// series and parallel resonances.
#[test]
fn ac_exported_crystal() {
    let circuit = circuit(vec![
        vsource("v1", "in", "0", 0.0),
        resistor("rs", "in", "a", 50.0),
        CircuitElement::Crystal(Crystal {
            id: "y1".into(),
            nodes: ["a".into(), "out".into()],
            params: sindr::circuit::CrystalParams {
                frequency: 16e6,
                esr: 40.0,
                c0: 5e-12,
                cm: 20e-15,
            },
        }),
        resistor("rl", "out", "0", 50.0),
    ]);
    check_ac(
        "ac_exported_crystal",
        exported("ac_exported_crystal", &circuit),
        "v1",
        "out",
        200,
        (1e6, 1e8),
        AC_LINEAR,
    );
}

/// A ferrite bead with its parallel capacitance, between 50 Ω source and
/// load, from 100 kHz past its self-resonance.
#[test]
fn ac_exported_ferrite_bead() {
    let circuit = circuit(vec![
        vsource("v1", "in", "0", 0.0),
        resistor("rs", "in", "a", 50.0),
        CircuitElement::FerriteBead(FerriteBead {
            id: "fb1".into(),
            nodes: ["a".into(), "out".into()],
            params: sindr::circuit::FerriteBeadParams {
                r_dc: 0.1,
                l: 1e-6,
                r_ac: 600.0,
                c: Some(1e-12),
            },
        }),
        resistor("rl", "out", "0", 50.0),
    ]);
    check_ac(
        "ac_exported_ferrite_bead",
        exported("ac_exported_ferrite_bead", &circuit),
        "v1",
        "out",
        20,
        (1e5, 1e10),
        AC_LINEAR,
    );
}

/// Three digital pins on a 3.3 V supply, each pulled by `v1` from 1.5 V
/// below ground to 1.7 V above the supply through a resistor: a push-pull
/// output driving high (its `r_high` against the source, then its power
/// clamp), an input with its pull-up (both clamps), and an open-drain
/// output pulling low. The deck is the IBIS output stage and clamps the
/// pins export as, so this is the pin's model built from primitives
/// against ngspice's.
#[test]
fn dc_exported_digital_pin() {
    use sindr::element::{DigitalPin, PinMode, PinModel, Pull};
    let pin = |id: &str, pad: &str, mode, level, pull| {
        CircuitElement::DigitalPin(DigitalPin {
            id: id.into(),
            nodes: [pad.into(), "vdd".into(), "0".into()],
            mode,
            level,
            pull,
            port: None,
            model: PinModel::default(),
        })
    };
    let circuit = circuit(vec![
        vsource("vdd", "vdd", "0", 3.3),
        vsource("v1", "ext", "0", 0.0),
        pin("p1", "a", PinMode::PushPull, true, Pull::None),
        resistor("r1", "ext", "a", 100.0),
        pin("p2", "b", PinMode::Input, false, Pull::Up),
        resistor("r2", "ext", "b", 1e3),
        pin("p3", "c", PinMode::OpenDrain, false, Pull::None),
        resistor("r3", "ext", "c", 330.0),
    ]);
    let circuit = exported("dc_exported_digital_pin", &circuit);
    let golden = Golden::load("dc_exported_digital_pin");
    let sweep = dc_sweep(circuit, &DcSweepAxis::new("v1", -1.5, 5.0, 66)).unwrap();
    let scale = golden.scale();
    let node = |n: &str| -> Vec<f64> {
        sweep
            .points
            .iter()
            .map(|p| p.result.node_voltages[n])
            .collect()
    };
    let current = |id: &str| -> Vec<f64> {
        sweep
            .points
            .iter()
            .map(|p| source_current(&p.result, id))
            .collect()
    };
    let mut check = Check::new("dc_exported_digital_pin");
    check.series("v-sweep", &scale, &sweep_values(&sweep), &scale, OP_V);
    for n in ["a", "b", "c"] {
        check.series(
            &format!("v({n})"),
            &scale,
            &node(n),
            &golden.column(&format!("v({n})")),
            OP_V,
        );
    }
    for id in ["v1", "vdd"] {
        check.series(
            &format!("i({id})"),
            &scale,
            &current(id),
            &golden.column(&format!("i({id})")),
            OP_I,
        );
    }
    check.finish();
}

/// The golden vector for a sensitivity entry. Elements, gains and the
/// stimulus as ngspice's `.sens` names them: an element's own value is its
/// name, a controlled source's gain `<id>_gain`, a stimulus `<id>_acmag`.
/// A device-model parameter `<id>_<param>`, the central difference the
/// case's `.ctl` takes of ngspice's operating point.
fn sens_vector(id: &str, parameter: SensParameter) -> String {
    use SensParameter as P;
    let param = match parameter {
        P::VoltageGain | P::Transconductance | P::Transresistance | P::CurrentGain => "gain",
        P::AcMagnitude => "acmag",
        P::SaturationCurrent => "is",
        P::EmissionCoefficient => "n",
        P::SeriesResistance => "rs",
        P::ForwardBeta => "bf",
        P::ReverseBeta => "br",
        P::ForwardEmissionCoefficient => "nf",
        P::ReverseEmissionCoefficient => "nr",
        P::EarlyVoltage => "vaf",
        P::TransconductanceParameter => "kp",
        P::ThresholdVoltage => "vto",
        P::ChannelLengthModulation => "lambda",
        _ => return id.to_string(),
    };
    format!("{id}_{param}")
}

/// A DC `.sens` deck: every vector in its golden against sindr's entry.
fn check_dc_sens(case: &'static str, tol: Tol) {
    let netlist = parse_file(case_path(case, "cir")).unwrap();
    let config = netlist.analyses[0].sens_config().unwrap();
    let sens = solve_sensitivity(&netlist.circuit, &config).unwrap();
    let golden = Golden::load(case);
    let mut check = Check::new(case);
    let mut seen = 0;
    for e in &sens.entries {
        let name = sens_vector(&e.component_id, e.parameter);
        // Column 0 is the scale, whatever its name.
        if !golden.names[1..].contains(&name) {
            continue;
        }
        seen += 1;
        check.value(&name, e.sensitivity, golden.op(&name), tol);
    }
    // The scale column repeats one vector, so it isn't counted.
    assert_eq!(
        seen,
        golden.names.len() - 1,
        "{case}: vectors without an entry"
    );
    check.finish();
}

/// Device-model parameters against ngspice's own central differences (a
/// 0.1 % step). ngspice's `.sens` isn't the reference here: it takes its
/// own finite step, and is 2 % off for a diode's IS and N, and reports 0 for
/// RS, BF, BR, NF and NR.
const SENS_DEVICE: Tol = Tol {
    rel: 1e-5,
    abs: 1e-9,
};

/// A diode's IS, N and RS and a MOSFET's KP, VTO and LAMBDA, with the IS
/// derivative the size of 10¹² V/A.
#[test]
fn sens_dc_diode_mos() {
    check_dc_sens("sens_dc_diode_mos", SENS_DEVICE);
}

/// A BJT's BF, IS, BR, NF and NR.
#[test]
fn sens_dc_bjt() {
    check_dc_sens("sens_dc_bjt", SENS_DEVICE);
}

/// E, G, F and H gains, through the controlling branch current for F and H.
#[test]
fn sens_dc_controlled() {
    check_dc_sens(
        "sens_dc_controlled",
        Tol {
            rel: 1e-6,
            abs: 1e-12,
        },
    );
}

/// AC sensitivity of a linear network to R, C, L, every controlled source's
/// gain and the stimulus, at 1 kHz. A single frequency: ngspice 47 gets only
/// a sweep's first point right, and linearises at a zero bias, so the deck
/// is linear.
#[test]
fn sens_ac_linear() {
    let case = "sens_ac_linear";
    let netlist = parse_file(case_path(case, "cir")).unwrap();
    let config = netlist.ac_sens_config().unwrap().unwrap();
    let sens = solve_ac_sensitivity(&netlist.circuit, &config).unwrap();
    let golden = Golden::load(case);
    let point = &sens.points[0];
    let mut check = Check::new(case);
    check.value(
        "frequency",
        point.frequency,
        golden.scale()[0],
        Tol {
            rel: 1e-12,
            abs: 0.0,
        },
    );
    let mut seen = 0;
    for e in &point.entries {
        let name = sens_vector(&e.component_id, e.parameter);
        let Some(i) = golden.names.iter().position(|n| *n == name) else {
            continue;
        };
        seen += 1;
        let want = Complex64::new(golden.rows[0][i], golden.rows[0][i + 1]);
        // Both parts to 1e-6 of the phasor's magnitude.
        let tol = Tol {
            rel: 0.0,
            abs: 1e-6 * want.norm(),
        };
        check.value(&format!("{name} re"), e.sensitivity.re, want.re, tol);
        check.value(&format!("{name} im"), e.sensitivity.im, want.im, tol);
    }
    // Each vector is two columns (real, imaginary) after the frequency.
    assert_eq!(
        seen,
        (golden.names.len() - 1) / 2,
        "vectors without an entry"
    );
    check.finish();
}

/// The poles and zeros in a `.pz` golden: each `pole(k)` / `zero(k)` is a
/// real and an imaginary column after the scale.
fn pz_golden(case: &str) -> (Vec<Complex64>, Vec<Complex64>) {
    let golden = Golden::load(case);
    let row = &golden.rows[0];
    let (mut poles, mut zeros) = (Vec::new(), Vec::new());
    for k in (1..golden.names.len()).step_by(2) {
        let z = Complex64::new(row[k], row[k + 1]);
        if golden.names[k].starts_with("pole") {
            poles.push(z);
        } else {
            zeros.push(z);
        }
    }
    (poles, zeros)
}

/// Each golden root against the nearest of sindr's, to `rel` relative and
/// `rel·scale` absolute (ngspice writes nine significant digits, and a root
/// at 0 comes out at round-off of the others' size).
fn check_roots(
    check: &mut Check,
    what: &str,
    got: &[Complex64],
    want: &[Complex64],
    rel: f64,
    scale: f64,
) {
    assert_eq!(got.len(), want.len(), "{what}: {got:?} vs {want:?}");
    let tol = Tol {
        rel,
        abs: rel * scale,
    };
    for (k, w) in want.iter().enumerate() {
        let g = got
            .iter()
            .min_by(|a, b| (*a - w).norm().total_cmp(&(*b - w).norm()))
            .unwrap();
        check.value(&format!("{what}({}).re", k + 1), g.re, w.re, tol);
        check.value(&format!("{what}({}).im", k + 1), g.im, w.im, tol);
    }
}

fn check_pz(case: &'static str, config: &sindr::PzConfig) {
    let netlist = parse_file(case_path(case, "cir")).unwrap();
    let pz = sindr::solve_pz(&netlist.circuit, config).unwrap();
    let (poles, zeros) = pz_golden(case);
    let scale = poles
        .iter()
        .chain(&zeros)
        .map(|z| z.norm())
        .fold(0.0, f64::max);
    let mut check = Check::new(case);
    check_roots(&mut check, "pole", &pz.poles, &poles, 1e-8, scale);
    check_roots(&mut check, "zero", &pz.zeros, &zeros, 1e-8, scale);
    check.finish();
}

/// `.pz … vol pz` through an RLC tank with a capacitor bridging it: a
/// complex pole pair and a complex zero pair.
#[test]
fn pz_rlc_vol() {
    check_pz(
        "pz_rlc_vol",
        &sindr::PzConfig::voltage_gain("in", "0", "a", "0"),
    );
}

/// `.pz … cur pz` into an RC-RL ladder: three poles and the inductor's zero
/// at DC.
#[test]
fn pz_ladder_cur() {
    check_pz(
        "pz_ladder_cur",
        &sindr::PzConfig::transimpedance("in", "0", "out", "0"),
    );
}

/// A DC motor started from rest on 12 V against a load: sindr's motor
/// against ngspice's R–L–E armature and F–C–R–I shaft, the deck it exports
/// as. Speed is `v(m1_w)` there, `motors["m1"].speed` here.
#[test]
fn tran_exported_dc_motor() {
    use sindr::element::{DcMotor, DcMotorParams};
    let case = "tran_exported_dc_motor";
    let circuit = circuit(vec![
        vsource("v1", "in", "0", 12.0),
        CircuitElement::DcMotor(DcMotor {
            id: "m1".into(),
            nodes: ["in".into(), "0".into()],
            params: DcMotorParams::new(2.0, 0.02)
                .with_inductance(1e-3)
                .with_inertia(1e-6)
                .with_friction(1e-6)
                .with_load(2e-3),
        }),
    ]);
    let circuit = exported(case, &circuit);
    let config = sindr::TransientConfig::new(30e-3, 20e-6).with_tmax(10e-6);
    let result = sindr::solve_transient(circuit, &config).unwrap();
    let steps = &result.transient.unwrap().timesteps;
    let golden = Golden::load(case);
    let scale = golden.scale();
    let times: Vec<f64> = steps.iter().map(|s| s.time).collect();
    let mut check = Check::new(case);
    let speed: Vec<f64> = steps.iter().map(|s| s.motors["m1"].speed).collect();
    let current: Vec<f64> = steps
        .iter()
        .map(|s| {
            s.component_results
                .iter()
                .find(|c| c.id == "v1")
                .unwrap()
                .current_through
        })
        .collect();
    for (name, got) in [("v(m1_w)", speed), ("i(v1)", current)] {
        let want_all = golden.column(name);
        let full_scale = want_all.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let want: Vec<f64> = times
            .iter()
            .map(|&t| interpolate(&scale, &want_all, t))
            .collect();
        check.series_fs(name, &times, &got, &want, 1e-3, full_scale);
    }
    check.finish();
}

/// The averaged buck's operating point against a switched synchronous buck
/// in ngspice (`tran_switched_buck.cir`), averaged over its last 20
/// periods once settled. The averaged model isn't fitted to it: its set
/// voltage is `D·Vin` less the conduction drop, and its efficiency the
/// conduction loss of the inductor current's RMS (DC plus the model's own
/// ripple estimate) in the switch and inductor resistance. So this checks
/// power balance and the ripple estimate against real switching. The
/// switched deck has no sindr twin (sindr has no averaged-converter SPICE
/// line), so the `.cir` is read by ngspice only.
#[test]
fn tran_switched_buck() {
    use sindr::element::SwitchingRegulator;
    use sindr::{ConverterParams, ConverterTopology};
    let case = "tran_switched_buck";
    let golden = Golden::load(case);
    let (vin, duty, f, l, r_load) = (12.0, 5.0 / 12.0, 200e3, 22e-6, 5.0);
    // Either switch's 20 mΩ plus the inductor's 30 mΩ carries the current
    // all period.
    let r_on = 20e-3 + 30e-3;
    let v_set = duty * vin * r_load / (r_load + r_on);
    let i_out = v_set / r_load;
    let mut model = ConverterParams {
        switching_frequency: Some(f),
        inductance: Some(l),
        ..Default::default()
    };
    let ripple = model
        .ripple_current(ConverterTopology::Buck, vin, v_set)
        .unwrap();
    let p_out = v_set * i_out;
    let loss = (i_out * i_out + ripple * ripple / 12.0) * r_on;
    model.efficiency = p_out / (p_out + loss);
    let c = circuit(vec![
        vsource("vin", "in", "0", vin),
        CircuitElement::SwitchingRegulator(SwitchingRegulator {
            id: "u1".into(),
            nodes: ["in".into(), "out".into(), "0".into()],
            topology: ConverterTopology::Buck,
            voltage: v_set,
            model,
        }),
        resistor("rload", "out", "0", r_load),
    ]);
    let r = solve_circuit(&c).unwrap();
    let at = |name| golden.column(name)[0];
    // ngspice ran long enough to settle: the 20 periods before agree.
    assert!((at("vbefore") - at("vout")).abs() < 1e-5);
    let mut check = Check::new(case);
    // The switched output sits 270 µV lower (5e-5): its 1 ns edges and
    // dead time trim the duty a little.
    check.value(
        "v(out)",
        r.node_voltages["out"],
        at("vout"),
        Tol {
            rel: 2e-4,
            abs: 0.0,
        },
    );
    // Power balance at that efficiency gives the input current to 2e-5.
    check.value(
        "i(vin)",
        r.branch_currents["vin"],
        at("iin"),
        Tol {
            rel: 2e-4,
            abs: 0.0,
        },
    );
    // The ripple estimate is 0.3 % under the switched inductor's.
    check.value(
        "ripple",
        ripple,
        at("iripple"),
        Tol {
            rel: 0.01,
            abs: 0.0,
        },
    );
    check.finish();
}
