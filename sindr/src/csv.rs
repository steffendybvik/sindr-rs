//! CSV export of transient, AC and sweep results.
//!
//! Each result type gets a `to_csv` method that writes one row per point:
//! the scale (time, frequency, swept value or temperature) first, then a
//! column per node voltage, `v(<node>)` with nodes in name order, then
//! for real-valued results a column per component current, `i(<id>)`, in
//! circuit order. AC writes each node voltage as two columns,
//! `re(v(<node>))` and `im(v(<node>))`.
//!
//! Numbers are written so they read back as the same `f64`. A value missing
//! from a row is an empty field. Names that need it are quoted as RFC 4180
//! has it. Lines end in `\n`.

use std::collections::{BTreeSet, HashSet};
use std::fmt::Write as _;

use crate::ac_analysis::AcResult;
use crate::dc_sweep::DcSweepResult;
use crate::results::{ComponentResult, TransientData};
use crate::temp_sweep::TempSweepResult;
use std::collections::HashMap;

impl TransientData {
    /// The waveforms as CSV: `time` (s), then `v(<node>)` (V) and
    /// `i(<id>)` (A, each component's `current_through`), one row per
    /// timestep. See [CSV export](crate#csv-export) for the layout.
    pub fn to_csv(&self) -> String {
        real_table(
            "time",
            self.timesteps
                .iter()
                .map(|s| (s.time, &s.node_voltages, s.component_results.as_slice())),
        )
    }
}

impl DcSweepResult {
    /// The sweep as CSV: the swept source's value, in a column named after
    /// it, then `v(<node>)` (V) and `i(<id>)` (A), one row per point. See
    /// [CSV export](crate#csv-export) for the layout.
    pub fn to_csv(&self) -> String {
        real_table(
            &self.source_id,
            self.points.iter().map(|p| {
                (
                    p.sweep_value,
                    &p.result.node_voltages,
                    p.result.component_results.as_slice(),
                )
            }),
        )
    }
}

impl TempSweepResult {
    /// The sweep as CSV: `temperature` (K), then `v(<node>)` (V) and
    /// `i(<id>)` (A), one row per point. See [CSV export](crate#csv-export)
    /// for the layout.
    pub fn to_csv(&self) -> String {
        real_table(
            "temperature",
            self.points.iter().map(|p| {
                (
                    p.temperature,
                    &p.result.node_voltages,
                    p.result.component_results.as_slice(),
                )
            }),
        )
    }
}

impl AcResult {
    /// The frequency response as CSV: `frequency` (Hz), then for each node
    /// `re(v(<node>))` and `im(v(<node>))` (V, for the source's
    /// [`ac_magnitude`](crate::ac_analysis::AcConfig::ac_magnitude)), one
    /// row per frequency. Magnitude and phase follow from the two parts.
    /// See [CSV export](crate#csv-export) for the layout.
    pub fn to_csv(&self) -> String {
        let nodes: BTreeSet<&str> = self
            .points
            .iter()
            .flat_map(|p| p.node_voltages.keys().map(String::as_str))
            .collect();
        let mut header = vec!["frequency".to_string()];
        for node in &nodes {
            header.push(format!("re(v({node}))"));
            header.push(format!("im(v({node}))"));
        }
        let rows = self.points.iter().map(|p| {
            let mut row = vec![Some(p.frequency)];
            for node in &nodes {
                let v = p.node_voltages.get(*node);
                row.push(v.map(|v| v.re));
                row.push(v.map(|v| v.im));
            }
            row
        });
        render(&header, rows)
    }
}

/// A table of real results: the scale, node voltages, component currents.
fn real_table<'a>(
    scale: &str,
    rows: impl Iterator<Item = (f64, &'a HashMap<String, f64>, &'a [ComponentResult])> + Clone,
) -> String {
    let nodes: BTreeSet<&str> = rows
        .clone()
        .flat_map(|(_, v, _)| v.keys().map(String::as_str))
        .collect();
    let mut seen: HashSet<&str> = HashSet::new();
    let mut ids: Vec<&str> = Vec::new();
    for (_, _, components) in rows.clone() {
        for c in components {
            if seen.insert(&c.id) {
                ids.push(&c.id);
            }
        }
    }
    let header: Vec<String> = std::iter::once(scale.to_string())
        .chain(nodes.iter().map(|n| format!("v({n})")))
        .chain(ids.iter().map(|id| format!("i({id})")))
        .collect();
    let mut first: HashMap<&str, usize> = HashMap::new();
    let rows = rows.map(move |(x, voltages, components)| {
        let mut row = Vec::with_capacity(1 + nodes.len() + ids.len());
        row.push(Some(x));
        row.extend(nodes.iter().map(|n| voltages.get(*n).copied()));
        // Results usually list every component once, in circuit order: read
        // them by position then, and through a first-match index otherwise.
        let in_order =
            components.len() == ids.len() && components.iter().zip(&ids).all(|(c, id)| c.id == *id);
        if in_order {
            row.extend(components.iter().map(|c| Some(c.current_through)));
        } else {
            first.clear();
            for (k, c) in components.iter().enumerate() {
                first.entry(c.id.as_str()).or_insert(k);
            }
            row.extend(
                ids.iter()
                    .map(|id| first.get(id).map(|&k| components[k].current_through)),
            );
        }
        row
    });
    render(&header, rows)
}

fn render(header: &[String], rows: impl Iterator<Item = Vec<Option<f64>>>) -> String {
    let mut out = String::new();
    for (k, h) in header.iter().enumerate() {
        if k > 0 {
            out.push(',');
        }
        out.push_str(&quote(h));
    }
    out.push('\n');
    for row in rows {
        for (k, v) in row.into_iter().enumerate() {
            if k > 0 {
                out.push(',');
            }
            if let Some(v) = v {
                write_number(&mut out, v);
            }
        }
        out.push('\n');
    }
    out
}

/// `name` as a CSV field: quoted, with `"` doubled, when it holds a comma,
/// quote or line break.
fn quote(name: &str) -> String {
    if name.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", name.replace('"', "\"\""))
    } else {
        name.to_string()
    }
}

/// `value` in the shortest form that reads back as the same `f64`: plain
/// decimals in a readable range, exponent notation outside it, appended to
/// `out`.
fn write_number(out: &mut String, value: f64) {
    let magnitude = value.abs();
    if value == 0.0 || !value.is_finite() || (1e-4..1e15).contains(&magnitude) {
        let _ = write!(out, "{value}");
    } else {
        let _ = write!(out, "{value:e}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn number(value: f64) -> String {
        let mut s = String::new();
        write_number(&mut s, value);
        s
    }

    #[test]
    fn numbers_read_back_exactly() {
        for v in [
            0.0,
            -0.0,
            1.0,
            -2.5,
            1e-14,
            3.3e-9,
            0.1 + 0.2,
            123_456.789,
            1e300,
            -4.2e-310,
            f64::MAX,
            f64::MIN_POSITIVE,
        ] {
            let s = number(v);
            assert_eq!(s.parse::<f64>().unwrap(), v, "{v} wrote {s}");
        }
        assert_eq!(number(1e-14), "1e-14");
        assert_eq!(number(0.5), "0.5");
    }

    #[test]
    fn currents_line_up_with_their_columns_in_any_order() {
        let c = |id: &str, i: f64| ComponentResult {
            id: id.into(),
            voltage_across: 0.0,
            current_through: i,
            power: 0.0,
        };
        let v = HashMap::from([("a".to_string(), 1.0)]);
        let rows = [
            vec![c("R1", 1.0), c("R2", 2.0)],
            vec![c("R2", 4.0), c("R1", 3.0), c("R3", 5.0)],
            vec![c("R3", 6.0), c("R3", 7.0)],
        ];
        let csv = real_table(
            "t",
            rows.iter()
                .enumerate()
                .map(|(k, r)| (k as f64, &v, r.as_slice())),
        );
        assert_eq!(
            csv,
            "t,v(a),i(R1),i(R2),i(R3)\n0,1,1,2,\n1,1,3,4,5\n2,1,,,6\n"
        );
    }

    #[test]
    fn awkward_names_are_quoted() {
        assert_eq!(quote("out"), "out");
        assert_eq!(quote("a,b"), "\"a,b\"");
        assert_eq!(quote("say \"hi\""), "\"say \"\"hi\"\"\"");
    }
}
