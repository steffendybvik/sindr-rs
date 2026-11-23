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

use std::collections::BTreeSet;
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
    let mut ids: Vec<&str> = Vec::new();
    for (_, _, components) in rows.clone() {
        for c in components {
            if !ids.contains(&c.id.as_str()) {
                ids.push(&c.id);
            }
        }
    }
    let header: Vec<String> = std::iter::once(scale.to_string())
        .chain(nodes.iter().map(|n| format!("v({n})")))
        .chain(ids.iter().map(|id| format!("i({id})")))
        .collect();
    let rows = rows.map(|(x, voltages, components)| {
        std::iter::once(Some(x))
            .chain(nodes.iter().map(|n| voltages.get(*n).copied()))
            .chain(ids.iter().map(|id| {
                components
                    .iter()
                    .find(|c| c.id == *id)
                    .map(|c| c.current_through)
            }))
            .collect::<Vec<_>>()
    });
    render(&header, rows)
}

fn render(header: &[String], rows: impl Iterator<Item = Vec<Option<f64>>>) -> String {
    let mut out = String::new();
    let fields: Vec<String> = header.iter().map(|h| quote(h)).collect();
    out.push_str(&fields.join(","));
    out.push('\n');
    for row in rows {
        let fields: Vec<String> = row
            .into_iter()
            .map(|v| v.map_or_else(String::new, number))
            .collect();
        out.push_str(&fields.join(","));
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
/// decimals in a readable range, exponent notation outside it.
fn number(value: f64) -> String {
    let magnitude = value.abs();
    let mut s = String::new();
    if value == 0.0 || !value.is_finite() || (1e-4..1e15).contains(&magnitude) {
        let _ = write!(s, "{value}");
    } else {
        let _ = write!(s, "{value:e}");
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn awkward_names_are_quoted() {
        assert_eq!(quote("out"), "out");
        assert_eq!(quote("a,b"), "\"a,b\"");
        assert_eq!(quote("say \"hi\""), "\"say \"\"hi\"\"\"");
    }
}
