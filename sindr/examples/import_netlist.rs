//! Simulate a schematic tool's netlist with a test bench.
//!
//! A transistor switch: a 9 V battery comes in on connector J1, R1 feeds an
//! LED, and Q1 sinks it when a logic-level signal on test point TP1 drives
//! its base through R2. The netlist says nothing about voltages; the bench
//! adds the supply on J1's pins and the 3.3 V drive on TP1, then runs an
//! operating point. Results come back keyed by the netlist's own ids.
//!
//! Run: `cargo run -p sindr --example import_netlist`

use sindr::import::{
    simulate, Analysis, Bench, BenchSource, Component, ComponentKind, Net, Netlist, Pin, Simulation,
};

fn main() {
    let netlist = Netlist {
        components: vec![
            Component::new("J1", ComponentKind::Connector),
            Component::new("TP1", ComponentKind::TestPoint),
            Component::new("R1", ComponentKind::Resistor).with_value("680"),
            Component::new("D1", ComponentKind::Led).with_param("color", "green"),
            Component::new("Q1", ComponentKind::Npn).with_param("hfe", 200.0),
            Component::new("R2", ComponentKind::Resistor).with_value("4k7"),
        ],
        nets: vec![
            Net::new("net-1", [("J1", "1"), ("R1", "1")]).with_name("VBAT"),
            Net::new("net-2", [("R1", "2"), ("D1", "anode")]),
            Net::new("net-3", [("D1", "cathode"), ("Q1", "C")]),
            Net::new("net-4", [("TP1", "1"), ("R2", "1")]).with_name("DRIVE"),
            Net::new("net-5", [("R2", "2"), ("Q1", "B")]),
            Net::new("net-6", [("J1", "2"), ("Q1", "E")]).with_name("GND"),
        ],
    };

    let bench = Bench::new("GND")
        .with_source(BenchSource::voltage("battery", Pin::new("J1", "1"), 9.0))
        .with_source(BenchSource::voltage("drive", Pin::new("TP1", "1"), 3.3))
        .with_analysis(Analysis::OperatingPoint);

    let imported = match simulate(&netlist, &bench) {
        Ok(imported) => imported,
        Err(e) => {
            eprintln!("can't simulate: {e}");
            std::process::exit(1);
        }
    };
    let Simulation::OperatingPoint(op) = imported.result else {
        unreachable!("the bench asked for an operating point")
    };

    for net in &netlist.nets {
        let node = imported.circuit.node(&net.id).unwrap_or(&net.id);
        let label = net.name.as_deref().unwrap_or(&net.id);
        println!("{label:>6}: {:7.3} V", op.node_voltages[node]);
    }
    for id in ["R1", "D1", "R2"] {
        if let Some(c) = op.component_results.iter().find(|c| c.id == id) {
            println!(
                "{id:>6}: {:7.3} mA, {:6.2} mW",
                c.current_through * 1e3,
                c.power * 1e3
            );
        }
    }
}
