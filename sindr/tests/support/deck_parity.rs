//! Parity between a golden's `.cir` and the `Circuit` sindr solves for it.
//!
//! Goldens with MOSFETs, JFETs, LEDs, Schottky diodes and photodiodes are
//! built in Rust: sindr's parser doesn't read `M` or `J` lines and its
//! export doesn't write them, so `to_netlist` can't keep the two in step.
//! [`in_deck`] reads the deck with the few rules these decks use and checks
//! every element against the circuit: names, nodes, values, and each
//! `.model` parameter against the Rust parameters it stands for. A
//! parameter the card leaves out takes its SPICE default.

use sindr::element::{Jfet, Led, Mosfet, Photodiode, Resistor, SchottkyDiode, VoltageSource};
use std::collections::HashMap;

use sindr::{Circuit, CircuitElement, JfetKind, MosfetKind};
use sindr_devices::led::led_params_from_str;
use sindr_devices::photodiode::PhotodiodeParams;
use sindr_devices::schottky::SchottkyParams;

/// sindr's nominal junction temperature, which a deck without `.temp`
/// runs at (27 °C).
const NOMINAL_K: f64 = 300.15;

struct Deck {
    /// Element name (lower case) → its remaining tokens (lower case).
    elements: HashMap<String, Vec<String>>,
    /// Model name → (type, parameters).
    models: HashMap<String, (String, HashMap<String, f64>)>,
}

/// A SPICE number with its scale suffix: `10k`, `1meg`, `2.5m`, `50u`.
fn number(token: &str) -> f64 {
    let t = token.to_ascii_lowercase();
    // `e` belongs to the mantissa (`1e-3`); no suffix starts with it.
    let split = t
        .find(|c: char| c.is_ascii_alphabetic() && c != 'e')
        .unwrap_or(t.len());
    let (mantissa, suffix) = t.split_at(split);
    let scale = match suffix {
        "" => 1.0,
        s if s.starts_with("meg") => 1e6,
        s if s.starts_with('t') => 1e12,
        s if s.starts_with('g') => 1e9,
        s if s.starts_with('k') => 1e3,
        s if s.starts_with("mil") => 25.4e-6,
        s if s.starts_with('m') => 1e-3,
        s if s.starts_with('u') => 1e-6,
        s if s.starts_with('n') => 1e-9,
        s if s.starts_with('p') => 1e-12,
        s if s.starts_with('f') => 1e-15,
        s => panic!("unknown suffix `{s}` in `{token}`"),
    };
    mantissa
        .parse::<f64>()
        .unwrap_or_else(|_| panic!("not a number: `{token}`"))
        * scale
}

fn read(deck: &str) -> Deck {
    let mut elements = HashMap::new();
    let mut models = HashMap::new();
    // The first line is the title.
    for line in deck.lines().skip(1) {
        let line = line.trim().to_ascii_lowercase();
        if line.is_empty() || line.starts_with('*') {
            continue;
        }
        assert!(
            !line.starts_with('+'),
            "continuation lines aren't read: {line}"
        );
        if let Some(rest) = line.strip_prefix(".model") {
            let rest = rest.replace(['(', ')', ','], " ");
            let mut words = rest.split_whitespace();
            let name = words.next().expect("model name").to_string();
            let kind = words.next().expect("model type").to_string();
            let params = words
                .map(|w| {
                    let (k, v) = w
                        .split_once('=')
                        .unwrap_or_else(|| panic!("`{w}` in {line}"));
                    (k.to_string(), number(v))
                })
                .collect();
            models.insert(name, (kind, params));
        } else if line.starts_with('.') {
            assert!(
                line == ".end" || line.starts_with(".op"),
                "parity reads no `{line}`: add it to deck_parity.rs"
            );
        } else {
            let mut words = line.split_whitespace().map(str::to_string);
            let name = words.next().expect("element name");
            elements.insert(name, words.collect());
        }
    }
    Deck { elements, models }
}

fn close(case: &str, what: &str, deck: f64, rust: f64) {
    assert!(
        (deck - rust).abs() <= 1e-9 * deck.abs().max(rust.abs()).max(1e-300),
        "{case}: {what} is {deck} in the deck but {rust} in Rust"
    );
}

impl Deck {
    fn take(&mut self, case: &str, id: &str) -> Vec<String> {
        self.elements
            .remove(&id.to_ascii_lowercase())
            .unwrap_or_else(|| panic!("{case}: `{id}` is in the Rust circuit, not the deck"))
    }

    fn model(&self, case: &str, name: &str) -> &(String, HashMap<String, f64>) {
        self.models
            .get(name)
            .unwrap_or_else(|| panic!("{case}: no `.model {name}`"))
    }

    fn nodes(case: &str, id: &str, deck: &[String], rust: &[&String]) {
        let rust: Vec<String> = rust.iter().map(|n| n.to_ascii_lowercase()).collect();
        assert_eq!(deck, &rust[..], "{case}: `{id}` nodes");
    }

    /// The `D` card a two-terminal junction uses, checked for IS and N.
    fn diode(&mut self, case: &str, id: &str, nodes: &[String; 2], is: f64, n: f64) {
        let words = self.take(case, id);
        Self::nodes(case, id, &words[..2], &[&nodes[0], &nodes[1]]);
        let (kind, params) = self.model(case, &words[2]).clone();
        assert_eq!(kind, "d", "{case}: `{id}` model type");
        close(
            case,
            &format!("{id} IS"),
            params.get("is").copied().unwrap_or(1e-14),
            is,
        );
        close(
            case,
            &format!("{id} N"),
            params.get("n").copied().unwrap_or(1.0),
            n,
        );
        let known = ["is", "n"];
        for key in params.keys() {
            assert!(
                known.contains(&key.as_str()),
                "{case}: `{id}` card sets {key}, which this junction doesn't model"
            );
        }
    }
}

/// Checks that `<case>.cir` describes `circuit`, element for element. See
/// the module docs.
pub fn in_deck<'a>(case: &str, deck: &str, circuit: &'a Circuit) -> &'a Circuit {
    let mut d = read(deck);
    for component in &circuit.components {
        match component {
            CircuitElement::Resistor(Resistor {
                id,
                nodes,
                resistance,
            }) => {
                let words = d.take(case, id);
                let mut want = [nodes[0].to_ascii_lowercase(), nodes[1].to_ascii_lowercase()];
                let mut got = [words[0].clone(), words[1].clone()];
                want.sort();
                got.sort();
                assert_eq!(got, want, "{case}: `{id}` nodes");
                close(case, id, number(&words[2]), *resistance);
            }
            CircuitElement::VoltageSource(VoltageSource {
                id,
                nodes,
                voltage,
                waveform: None,
            }) => {
                let words = d.take(case, id);
                Deck::nodes(case, id, &words[..2], &[&nodes[0], &nodes[1]]);
                let value = words.iter().skip(2).find(|w| *w != "dc").expect("a value");
                close(case, id, number(value), *voltage);
            }
            CircuitElement::Mosfet(Mosfet {
                id,
                nodes,
                kind,
                params,
                parasitic_caps: None,
                body_diode: None,
            }) => {
                // `M d g s b model [W= L=]`; sindr's nodes are [g, d, s] and
                // its body is the source.
                let words = d.take(case, id);
                Deck::nodes(case, id, &words[..3], &[&nodes[1], &nodes[0], &nodes[2]]);
                assert_eq!(words[3], words[2], "{case}: `{id}` body must be its source");
                let mut w_over_l = 1.0;
                for word in &words[5..] {
                    if let Some(w) = word.strip_prefix("w=") {
                        w_over_l *= number(w);
                    } else if let Some(l) = word.strip_prefix("l=") {
                        w_over_l /= number(l);
                    }
                }
                let (mtype, card) = d.model(case, &words[4]).clone();
                let want_type = match kind {
                    MosfetKind::Nmos => "nmos",
                    MosfetKind::Pmos => "pmos",
                };
                assert_eq!(mtype, want_type, "{case}: `{id}` model type");
                let get = |k: &str, default: f64| card.get(k).copied().unwrap_or(default);
                assert_eq!(get("level", 1.0), 1.0, "{case}: `{id}` LEVEL");
                // KP·W/L is the transconductance sindr's `kp` stands for.
                close(
                    case,
                    &format!("{id} KP·W/L"),
                    get("kp", 2e-5) * w_over_l,
                    params.kp,
                );
                close(case, &format!("{id} VTO"), get("vto", 0.0), params.vto);
                close(
                    case,
                    &format!("{id} LAMBDA"),
                    get("lambda", 0.0),
                    params.lambda,
                );
                close(
                    case,
                    &format!("{id} GAMMA"),
                    get("gamma", 0.0),
                    params.gamma,
                );
                close(case, &format!("{id} PHI"), get("phi", 0.6), params.phi);
            }
            CircuitElement::Jfet(Jfet {
                id,
                nodes,
                kind,
                idss,
                vp,
                parasitic_caps,
            }) => {
                // `J d g s model`; sindr's nodes are [g, d, s].
                let words = d.take(case, id);
                Deck::nodes(case, id, &words[..3], &[&nodes[1], &nodes[0], &nodes[2]]);
                let (jtype, card) = d.model(case, &words[3]).clone();
                let want_type = match kind {
                    JfetKind::NChannel => "njf",
                    JfetKind::PChannel => "pjf",
                };
                assert_eq!(jtype, want_type, "{case}: `{id}` model type");
                let vto = card.get("vto").copied().unwrap_or(-2.0);
                let beta = card.get("beta").copied().unwrap_or(1e-4);
                close(case, &format!("{id} VTO"), vto, *vp);
                // Shockley's IDSS = BETA·VTO².
                close(case, &format!("{id} BETA·VTO²"), beta * vto * vto, *idss);
                assert_eq!(
                    card.get("lambda").copied().unwrap_or(0.0),
                    0.0,
                    "{case}: sindr's JFET has no LAMBDA"
                );
                // The gate junctions, at SPICE's defaults when the card
                // leaves them out. ngspice has no M: it grades at 0.5.
                let caps = parasitic_caps.unwrap_or_default();
                let get = |k: &str, default: f64| card.get(k).copied().unwrap_or(default);
                close(case, &format!("{id} CGS"), get("cgs", 0.0), caps.cgs);
                close(case, &format!("{id} CGD"), get("cgd", 0.0), caps.cgd);
                if caps.cgs > 0.0 || caps.cgd > 0.0 {
                    close(case, &format!("{id} PB"), get("pb", 1.0), caps.pb);
                    close(case, &format!("{id} FC"), get("fc", 0.5), caps.fc);
                    close(case, &format!("{id} M"), 0.5, caps.m);
                }
            }
            CircuitElement::Led(Led {
                id,
                nodes,
                color,
                temperature,
            }) => {
                close(case, &format!("{id} temperature"), NOMINAL_K, *temperature);
                let p = led_params_from_str(color);
                d.diode(case, id, nodes, p.is, p.n);
            }
            CircuitElement::SchottkyDiode(SchottkyDiode {
                id,
                nodes,
                temperature,
            }) => {
                close(case, &format!("{id} temperature"), NOMINAL_K, *temperature);
                let p = SchottkyParams::standard();
                d.diode(case, id, nodes, p.is, p.n);
            }
            CircuitElement::Photodiode(Photodiode {
                id,
                nodes,
                irradiance,
                temperature,
            }) => {
                close(case, &format!("{id} temperature"), NOMINAL_K, *temperature);
                let p = PhotodiodeParams::silicon();
                d.diode(case, id, nodes, p.is, p.n);
                // ngspice has no photodiode: the deck carries its
                // photocurrent as a current source from cathode to anode.
                let source = d
                    .elements
                    .iter()
                    .find(|(name, words)| {
                        name.starts_with('i')
                            && words[0] == nodes[1].to_ascii_lowercase()
                            && words[1] == nodes[0].to_ascii_lowercase()
                    })
                    .map(|(name, words)| (name.clone(), number(&words[2])))
                    .unwrap_or_else(|| panic!("{case}: no photocurrent source for `{id}`"));
                close(
                    case,
                    &format!("{id} photocurrent"),
                    source.1,
                    p.responsivity * irradiance,
                );
                d.elements.remove(&source.0);
            }
            other => panic!("{case}: deck_parity can't check {other:?} yet"),
        }
    }
    let mut extra: Vec<_> = d.elements.keys().collect();
    extra.sort();
    assert!(
        extra.is_empty(),
        "{case}: the deck has {extra:?}, which the Rust circuit doesn't"
    );
    circuit
}
