//! Block variants: one block, several implementations of the same ports,
//! picked per instance.

use sindr::element::{Resistor, VoltageSource};
use sindr::{Block, Circuit, CircuitElement, DesignError, Instance, SimError, Variant};

fn r(id: &str, a: &str, b: &str, ohms: f64) -> CircuitElement {
    CircuitElement::Resistor(Resistor {
        id: id.into(),
        nodes: [a.into(), b.into()],
        resistance: ohms,
    })
}

fn v(id: &str, p: &str, n: &str, volts: f64) -> CircuitElement {
    CircuitElement::VoltageSource(VoltageSource {
        id: id.into(),
        nodes: [p.into(), n.into()],
        voltage: volts,
        waveform: None,
    })
}

/// A 5 V supply: ideal is a source, detailed a 10 V source behind a
/// 1k/1k divider (5 V open circuit, 500 Ω out).
fn supply() -> Block {
    let mut block = Block {
        ports: vec!["out".into()],
        components: vec![v("V1", "out", "0", 5.0)],
        base_variant: Some("ideal".into()),
        ..Block::default()
    };
    block.variants.insert(
        "detailed".into(),
        Variant {
            ports: vec!["out".into()],
            components: vec![
                v("V1", "in", "0", 10.0),
                r("R1", "in", "out", 1e3),
                r("R2", "out", "0", 1e3),
            ],
            ..Variant::default()
        },
    );
    block
}

fn design(variant: Option<&str>) -> Circuit {
    let mut c = Circuit::new("0", vec![r("Rload", "vout", "0", 10e3)]);
    c.blocks.insert("supply".into(), supply());
    let mut x1 = Instance::new("X1", "supply", ["vout"]);
    x1.variant = variant.map(Into::into);
    c.instances.push(x1);
    c
}

#[test]
fn one_block_solves_with_either_variant() {
    let ideal = sindr::solve_op(&design(None)).unwrap();
    assert!((ideal.voltage("vout").unwrap() - 5.0).abs() < 1e-9);

    let detailed = sindr::solve_op(&design(Some("detailed"))).unwrap();
    // 10 V · (1k ∥ 10k) / (1k + 1k ∥ 10k)
    let expected = 10.0 * (1e3 * 10e3 / 11e3) / (1e3 + 1e3 * 10e3 / 11e3);
    assert!((detailed.voltage("vout").unwrap() - expected).abs() < 1e-9);
    // Inside the block reads back under the same instance path.
    assert!((detailed.voltage("X1.in").unwrap() - 10.0).abs() < 1e-9);

    let by_name = sindr::solve_op(&design(Some("ideal"))).unwrap();
    assert_eq!(by_name.voltage("vout"), ideal.voltage("vout"));
}

#[test]
fn switching_variant_changes_nothing_outside_the_instance() {
    let flat = |variant| design(variant).flatten().unwrap();
    let ideal = flat(None);
    let detailed = flat(Some("detailed"));
    let outside = |c: &Circuit| -> Vec<CircuitElement> {
        c.components
            .iter()
            .filter(|e| !e.id().starts_with("X1."))
            .cloned()
            .collect()
    };
    assert_eq!(outside(&ideal), outside(&detailed));
    assert_eq!(ideal.ground_node, detailed.ground_node);
    assert_eq!(ideal.globals, detailed.globals);
}

#[test]
fn the_block_default_applies_when_an_instance_names_none() {
    let mut c = design(None);
    c.blocks.get_mut("supply").unwrap().default_variant = Some("detailed".into());
    let ids: Vec<_> = c
        .flatten()
        .unwrap()
        .components
        .iter()
        .map(|e| e.id().to_string())
        .collect();
    assert_eq!(ids, ["Rload", "X1.V1", "X1.R1", "X1.R2"]);
}

#[test]
fn a_port_mismatch_between_variants_names_the_block() {
    let mut c = design(None);
    c.blocks
        .get_mut("supply")
        .unwrap()
        .variants
        .get_mut("detailed")
        .unwrap()
        .ports = vec!["vout".into()];
    let err = c.flatten().unwrap_err();
    assert_eq!(
        err,
        DesignError::VariantPorts {
            block: "supply".into(),
            variant: "detailed".into(),
            expected: vec!["out".into()],
            got: vec!["vout".into()],
        }
    );
    // Whatever variant the instance picks, and through every analysis.
    assert!(matches!(
        sindr::solve_op(&c),
        Err(SimError::Design(DesignError::VariantPorts { .. }))
    ));
}

#[test]
fn an_unknown_variant_names_the_instance() {
    let err = design(Some("behavioural")).flatten().unwrap_err();
    assert_eq!(
        err,
        DesignError::UnknownVariant {
            instance: "X1".into(),
            block: "supply".into(),
            variant: "behavioural".into(),
        }
    );

    let mut c = design(None);
    c.blocks.get_mut("supply").unwrap().default_variant = Some("nope".into());
    assert!(matches!(
        c.flatten(),
        Err(DesignError::UnknownVariant { variant, .. }) if variant == "nope"
    ));
}

#[test]
fn a_variant_cannot_reuse_the_name_of_the_blocks_own_body() {
    let mut c = design(None);
    let block = c.blocks.get_mut("supply").unwrap();
    let detailed = block.variants.remove("detailed").unwrap();
    block.variants.insert("ideal".into(), detailed);
    assert!(matches!(
        c.flatten(),
        Err(DesignError::DuplicateId { id, .. }) if id == "ideal"
    ));
}

#[test]
fn without_a_name_the_blocks_own_body_is_base() {
    let mut c = design(Some("base"));
    c.blocks.get_mut("supply").unwrap().base_variant = None;
    assert_eq!(c.flatten().unwrap().components[1].id(), "X1.V1");
}

#[cfg(feature = "spice")]
#[test]
fn export_writes_every_variant_and_the_instance_names_its_pick() {
    use sindr::spice::{parse_str, to_netlist};

    let deck = to_netlist(&design(Some("detailed")), "variants").unwrap();
    assert!(deck.contains(".subckt supply out\n"), "{deck}");
    assert!(deck.contains(".subckt supply_detailed out\n"), "{deck}");
    assert!(deck.contains("X1 vout supply_detailed\n"), "{deck}");

    // SPICE has no variants: the picked one comes back as a block of its
    // own, and solves the same.
    let parsed = parse_str(&deck).unwrap().circuit;
    assert!(parsed.blocks.contains_key("supply_detailed"));
    let before = sindr::solve_op(&design(Some("detailed"))).unwrap();
    let after = sindr::solve_op(&parsed).unwrap();
    assert!((before.voltage("vout").unwrap() - after.voltage("vout").unwrap()).abs() < 1e-9);

    let ideal = to_netlist(&design(None), "variants").unwrap();
    assert!(ideal.contains("X1 vout supply\n"), "{ideal}");
}

#[cfg(feature = "spice")]
#[test]
fn a_variant_name_that_collides_with_another_block_is_refused() {
    let mut c = design(None);
    c.blocks.insert("supply_detailed".into(), supply());
    assert!(sindr::spice::to_netlist(&c, "clash").is_err());
}

#[cfg(feature = "serde")]
#[test]
fn variants_round_trip_through_json() {
    let c = design(Some("detailed"));
    let json = serde_json::to_string(&c).unwrap();
    assert!(json.contains(r#""variant":"detailed""#));
    assert_eq!(serde_json::from_str::<Circuit>(&json).unwrap(), c);
    // A block with no variants writes none of the new keys.
    let plain = serde_json::to_string(&Block::default()).unwrap();
    assert!(!plain.contains("variant"), "{plain}");
}
