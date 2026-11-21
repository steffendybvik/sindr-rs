//! CSV export reads back as the same numbers, for every result type.

use std::collections::HashMap;

use sindr::ac_analysis::{solve_ac, AcConfig};
use sindr::{
    dc_sweep, solve_transient, temperature_sweep, Circuit, CircuitElement, TransientConfig,
};

/// Parses CSV as `to_csv` writes it (RFC 4180 quoting, `\n` lines) into a
/// header and rows of optional numbers.
fn read(csv: &str) -> (Vec<String>, Vec<Vec<Option<f64>>>) {
    let mut records = Vec::new();
    let mut record = Vec::new();
    let mut field = String::new();
    let mut quoted = false;
    let mut chars = csv.chars().peekable();
    while let Some(c) = chars.next() {
        match (quoted, c) {
            (true, '"') if chars.peek() == Some(&'"') => {
                field.push('"');
                chars.next();
            }
            (true, '"') => quoted = false,
            (true, c) => field.push(c),
            (false, '"') => quoted = true,
            (false, ',') => record.push(std::mem::take(&mut field)),
            (false, '\n') => {
                record.push(std::mem::take(&mut field));
                records.push(std::mem::take(&mut record));
            }
            (false, c) => field.push(c),
        }
    }
    assert!(field.is_empty() && record.is_empty(), "no final newline");
    let mut records = records.into_iter();
    let header = records.next().expect("a header");
    let rows = records
        .map(|r| {
            assert_eq!(r.len(), header.len(), "ragged row {r:?}");
            r.iter()
                .map(|f| (!f.is_empty()).then(|| f.parse().expect("a number")))
                .collect()
        })
        .collect();
    (header, rows)
}

fn column<'a>(
    header: &[String],
    rows: &'a [Vec<Option<f64>>],
    name: &str,
) -> impl Iterator<Item = f64> + 'a {
    let i = header
        .iter()
        .position(|h| h == name)
        .unwrap_or_else(|| panic!("no column {name} in {header:?}"));
    rows.iter().map(move |r| r[i].expect("a value"))
}

/// An RC low-pass from a 5 V source through 1 kΩ, with an awkward node name.
fn rc() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["in".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["in".into(), "out,1".into()],
                resistance: 1e3,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["out,1".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
        ..Default::default()
    }
}

#[test]
fn transient_round_trips() {
    let tran = solve_transient(&rc(), &TransientConfig::new(5e-3, 1e-4))
        .unwrap()
        .transient
        .unwrap();
    let (header, rows) = read(&tran.to_csv());
    assert_eq!(header[0], "time");
    assert!(header.contains(&"v(out,1)".to_string()), "{header:?}");
    assert_eq!(rows.len(), tran.timesteps.len());
    let times: Vec<f64> = column(&header, &rows, "time").collect();
    let out: Vec<f64> = column(&header, &rows, "v(out,1)").collect();
    let i_c: Vec<f64> = column(&header, &rows, "i(C1)").collect();
    for (k, s) in tran.timesteps.iter().enumerate() {
        assert_eq!(times[k], s.time);
        assert_eq!(out[k], s.node_voltages["out,1"]);
        let c1 = s.component_results.iter().find(|c| c.id == "C1").unwrap();
        assert_eq!(i_c[k], c1.current_through);
    }
    // Components come in circuit order, after the nodes.
    let ids: Vec<&String> = header.iter().filter(|h| h.starts_with("i(")).collect();
    assert_eq!(ids, ["i(V1)", "i(R1)", "i(C1)"]);
}

#[test]
fn ac_round_trips() {
    let ac = solve_ac(&rc(), &AcConfig::new("V1", 10.0, 1e5, 13)).unwrap();
    let (header, rows) = read(&ac.to_csv());
    assert_eq!(header[0], "frequency");
    let f: Vec<f64> = column(&header, &rows, "frequency").collect();
    let re: Vec<f64> = column(&header, &rows, "re(v(out,1))").collect();
    let im: Vec<f64> = column(&header, &rows, "im(v(out,1))").collect();
    for (k, p) in ac.points.iter().enumerate() {
        assert_eq!(f[k], p.frequency);
        assert_eq!(re[k], p.node_voltages["out,1"].re);
        assert_eq!(im[k], p.node_voltages["out,1"].im);
    }
}

/// A diode behind 1 kΩ, swept by its source and by temperature.
fn diode() -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["a".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["a".into(), "b".into()],
                resistance: 1e3,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["b".into(), "0".into()],
                temperature: 300.15,
                model: None,
            },
        ],
        ..Default::default()
    }
}

#[test]
fn sweeps_round_trip() {
    let sweep = dc_sweep(&diode(), "V1", 0.0, 5.0, 11).unwrap();
    let (header, rows) = read(&sweep.to_csv());
    assert_eq!(header[0], "V1");
    let values: Vec<f64> = column(&header, &rows, "V1").collect();
    let vb: Vec<f64> = column(&header, &rows, "v(b)").collect();
    let id: Vec<f64> = column(&header, &rows, "i(D1)").collect();
    for (k, p) in sweep.points.iter().enumerate() {
        assert_eq!(values[k], p.sweep_value);
        assert_eq!(vb[k], p.result.node_voltages["b"]);
        let d1 = p
            .result
            .component_results
            .iter()
            .find(|c| c.id == "D1")
            .unwrap();
        assert_eq!(id[k], d1.current_through);
    }

    let temps = temperature_sweep(&diode(), 250.0, 350.0, 5).unwrap();
    let (header, rows) = read(&temps.to_csv());
    assert_eq!(header[0], "temperature");
    let kelvin: Vec<f64> = column(&header, &rows, "temperature").collect();
    let vb: Vec<f64> = column(&header, &rows, "v(b)").collect();
    let by_t: HashMap<u64, f64> = temps
        .points
        .iter()
        .map(|p| (p.temperature_kelvin.to_bits(), p.result.node_voltages["b"]))
        .collect();
    for (t, v) in kelvin.iter().zip(&vb) {
        assert_eq!(by_t[&t.to_bits()], *v);
    }
}
