//! Arbitrary netlists and benches into `sindr::import`, then the analysis.
//! Nothing may panic: a bad netlist is an `ImportError`, a circuit that
//! won't solve is a `SimError`.
//!
//! The first byte picks the input: odd reads the rest as JSON for a
//! `Netlist` and a `Bench` (the serde path an editor uses), even decodes it
//! into a structured netlist, so most inputs reach the solver.

#![no_main]

use libfuzzer_sys::fuzz_target;
use sindr::import::{
    simulate_with, Analysis, Bench, BenchLoad, BenchSource, Component, ComponentKind,
    ImportOptions, LoadKind, Net, Netlist, UnknownKinds,
};
use sindr::{AcConfig, DcSweepAxis, TransientConfig};

/// Symbol names, canonical and alias, including multi-unit packages, black
/// boxes and one sindr doesn't know.
const KINDS: &[&str] = &[
    "resistor", "resistor-iec", "capacitor", "capacitor-polarized", "inductor", "diode",
    "schottky", "zener", "led", "photodiode", "varactor", "npn", "pnp", "nmos", "pmos",
    "njfet", "pjfet", "igbt", "op-amp", "comparator", "dual-op-amp", "quad-op-amp",
    "potentiometer", "switch", "push-button", "fuse", "relay", "transformer", "regulator",
    "battery", "crystal", "ferrite-bead", "black-box", "thermistor", "photoresistor",
    "voltage-source", "current-source", "connector", "test-point", "header-4", "mcu-qfn32",
];

/// Values as a schematic writes them, and some it shouldn't.
const VALUES: &[&str] = &[
    "1k", "4k7", "100n", "10uF", "1M", "0", "-5", "3V3", "5", "1e300", "1e-300", "nan",
    "inf", "", "abc", "PMBT3904", "Green", "amber", "16MHz", "2 m", "0.01 1", "300 K",
];

/// Parameter names the kinds read, plus one none does.
const PARAMS: &[&str] = &[
    "bf", "hfe", "hfe_ic", "vce_sat", "vce_sat_ic", "vce_sat_ib", "vf", "if", "vf2", "if2",
    "vz", "iz", "vth", "vth_id", "rds_on", "rds_on_vgs", "color", "temperature", "position",
    "k", "l1", "l2", "frequency", "esr", "c0", "z_100mhz", "r_dc", "pickup_voltage",
    "coil_resistance", "state_of_charge", "light_level", "nonsense",
];

/// Numbers to draw from, the awkward ones included.
const NUMBERS: &[f64] = &[
    0.0, 1.0, -1.0, 3.3, 5.0, 12.0, 1e3, 1e-9, 1e12, -1e12, 1e308, f64::MIN_POSITIVE,
    f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.5, 100.0,
];

struct Bytes<'a>(&'a [u8]);

impl Bytes<'_> {
    fn byte(&mut self) -> u8 {
        match self.0.split_first() {
            Some((&b, rest)) => {
                self.0 = rest;
                b
            }
            None => 0,
        }
    }
    fn below(&mut self, n: usize) -> usize {
        if n == 0 {
            0
        } else {
            self.byte() as usize % n
        }
    }
    fn pick<'t, T>(&mut self, items: &'t [T]) -> &'t T {
        &items[self.below(items.len())]
    }
    fn number(&mut self) -> f64 {
        *self.pick(NUMBERS)
    }
}

/// A netlist and bench decoded from bytes: up to 8 components on up to 6
/// nets, any terminal possibly unconnected.
fn decode(data: &[u8]) -> (Netlist, Bench) {
    let mut b = Bytes(data);
    let nets: Vec<String> = (0..1 + b.below(6)).map(|i| format!("n{i}")).collect();
    let mut netlist = Netlist::default();
    let mut pins: Vec<Vec<(String, String)>> = vec![Vec::new(); nets.len()];
    for i in 0..b.below(9) {
        let id = format!("c{i}");
        let kind = ComponentKind::from_name(b.pick(KINDS));
        let mut component = Component::new(id.clone(), kind.clone());
        match b.below(3) {
            0 => {}
            1 => component = component.with_value(*b.pick(VALUES)),
            _ => component = component.with_value(b.number()),
        }
        for _ in 0..b.below(3) {
            let name = *b.pick(PARAMS);
            component = if b.byte() & 1 == 0 {
                component.with_param(name, b.number())
            } else {
                component.with_param(name, *b.pick(VALUES))
            };
        }
        // Each terminal by one of its names, a pin number or junk, on a
        // net or left off.
        let terminals = kind.terminals();
        let count = if terminals.is_empty() { 1 + b.below(4) } else { terminals.len() };
        for t in 0..count {
            if b.below(5) == 0 {
                continue;
            }
            let terminal = match (terminals.get(t), b.below(3)) {
                (Some(names), 0) => b.pick(names).to_string(),
                (_, 1) => (t + 1).to_string(),
                (Some(names), _) => names[0].to_string(),
                (None, _) => format!("p{t}"),
            };
            pins[b.below(nets.len())].push((id.clone(), terminal));
        }
        netlist.components.push(component);
    }
    for (net, pins) in nets.iter().zip(pins) {
        netlist.nets.push(Net::new(net.clone(), pins));
    }

    let mut bench = Bench::new(b.pick(&nets).clone());
    for i in 0..b.below(3) {
        let net = b.pick(&nets).clone();
        let source = if b.byte() & 1 == 0 {
            BenchSource::voltage(format!("v{i}"), net, b.number())
        } else {
            BenchSource::current(format!("i{i}"), net, b.number())
        };
        bench = bench.with_source(source.with_minus(b.pick(&nets).clone()));
    }
    for i in 0..b.below(3) {
        let kind = match b.below(4) {
            0 => LoadKind::Resistor(b.number()),
            1 => LoadKind::Capacitor(b.number()),
            2 => LoadKind::current_sink(b.number()),
            _ => LoadKind::released_at(b.number().abs().min(1.0), b.number()),
        };
        bench = bench.with_load(BenchLoad::new(format!("l{i}"), kind, b.pick(&nets).clone()));
    }
    if b.below(4) == 0 {
        bench = bench.with_tie(b.pick(&nets).clone(), b.pick(&nets).clone());
    }
    // Analyses kept short, so every input finishes quickly.
    bench = bench.with_analysis(match b.below(4) {
        0 => Analysis::OperatingPoint,
        1 => Analysis::Transient(TransientConfig::new(1e-6 * (1 + b.below(10)) as f64, 1e-7)),
        2 => Analysis::Ac(AcConfig::new("v0", 1.0, 1e6, 1 + b.below(8))),
        _ => Analysis::DcSweep(DcSweepAxis::new("v0", b.number(), b.number(), b.below(6))),
    });
    (netlist, bench)
}

fuzz_target!(|data: &[u8]| {
    let Some((&mode, rest)) = data.split_first() else {
        return;
    };
    let options = ImportOptions::new().with_unknown_kinds(if mode & 2 == 0 {
        UnknownKinds::BlackBox
    } else {
        UnknownKinds::Refuse
    });
    let (netlist, bench) = if mode & 1 == 1 {
        let Ok(text) = std::str::from_utf8(rest) else {
            return;
        };
        let (n, b) = text.split_once('\n').unwrap_or((text, "{}"));
        let (Ok(netlist), Ok(bench)) =
            (serde_json::from_str::<Netlist>(n), serde_json::from_str::<Bench>(b))
        else {
            return;
        };
        // A transient asked for in JSON can be arbitrarily long; keep it short.
        if let Analysis::Transient(t) = &bench.analysis {
            if !(t.tstop.is_finite() && t.tstep.is_finite())
                || t.tstep <= 0.0
                || t.tstop / t.tstep > 1e4
            {
                return;
            }
        }
        if let Analysis::DcSweep(axis) = &bench.analysis {
            if axis.num_points > 1_000 {
                return;
            }
        }
        if let Analysis::Ac(ac) = &bench.analysis {
            if ac.num_points > 1_000 {
                return;
            }
        }
        (netlist, bench)
    } else {
        decode(rest)
    };
    // `SINDR_FUZZ_SHOW=1` prints a crashing input's netlist and bench.
    if std::env::var_os("SINDR_FUZZ_SHOW").is_some() {
        eprintln!("{netlist:#?}\n{bench:#?}");
    }
    let _ = netlist.to_circuit_with(&bench, &options);
    let _ = simulate_with(&netlist, &bench, &options);
});
