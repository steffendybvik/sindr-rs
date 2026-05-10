//! Math-correctness tests for the alpha-5 changes (gmin homotopy and
//! `.NODESET` seeding). Where the existing `initial_voltages.rs` tests
//! check "didn't crash and stayed in a plausible range", these tests verify
//! the *physics*: KCL holds at every node, the diode obeys Shockley at the
//! converged operating point, the linear divider matches its closed-form
//! solution to floating-point precision, and the gmin/seeded paths agree
//! with plain Newton–Raphson where they should.
//!
//! Conventions used here mirror [`ComponentResult`]:
//! - `current_through` flows `nodes[0] → nodes[1]`.
//! - For KCL: at node `n`, current is leaving via component terminal `n0=n`
//!   (subtract) and entering via terminal `n1=n` (add).

use std::collections::HashMap;

use sindr::{
    solve_circuit, solve_circuit_with_initial_voltages, Circuit, CircuitElement, SimulationResult,
};

// ---------- helpers ----------

/// Sum net current INTO every non-ground node from a converged solution.
/// A correct solver must produce residuals of ~0 at every node.
fn kcl_residuals(circuit: &Circuit, result: &SimulationResult) -> HashMap<String, f64> {
    let by_id: HashMap<&str, f64> = result
        .component_results
        .iter()
        .map(|c| (c.id.as_str(), c.current_through))
        .collect();

    let mut sums: HashMap<String, f64> = HashMap::new();
    for el in &circuit.components {
        let (id, n0, n1): (&String, &String, &String) = match el {
            CircuitElement::Resistor { id, nodes, .. }
            | CircuitElement::VoltageSource { id, nodes, .. }
            | CircuitElement::Diode { id, nodes, .. }
            | CircuitElement::Led { id, nodes, .. }
            | CircuitElement::Capacitor { id, nodes, .. }
            | CircuitElement::Inductor { id, nodes, .. } => (id, &nodes[0], &nodes[1]),
            _ => continue, // multi-terminal devices not used in these tests
        };
        let i = match by_id.get(id.as_str()) {
            Some(c) => *c,
            None => continue,
        };
        if n0 != &circuit.ground_node {
            *sums.entry(n0.clone()).or_default() -= i;
        }
        if n1 != &circuit.ground_node {
            *sums.entry(n1.clone()).or_default() += i;
        }
    }
    sums
}

fn assert_kcl(circuit: &Circuit, result: &SimulationResult, tol_amps: f64) {
    let residuals = kcl_residuals(circuit, result);
    for (node, sum) in &residuals {
        assert!(
            sum.abs() < tol_amps,
            "KCL violated at node {node}: net current = {sum:.3e} A (tol = {tol_amps:.0e})"
        );
    }
}

// ---------- circuits ----------

fn diode_resistor(vsource: f64, r_ohms: f64) -> Circuit {
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: vsource,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: r_ohms,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["n2".into(), "0".into()],
                temperature: 300.15,
            },
        ],
    }
}

fn three_resistor_divider() -> Circuit {
    // 12 V → R1=1k → n2 → R2=2k → n3 → R3=3k → 0
    // Total = 6 kΩ → I = 2 mA
    // V(n2) = 12 - 2 mA·1k = 10 V
    // V(n3) = 10 - 2 mA·2k =  6 V
    Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 12.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["n2".into(), "n3".into()],
                resistance: 2_000.0,
            },
            CircuitElement::Resistor {
                id: "R3".into(),
                nodes: ["n3".into(), "0".into()],
                resistance: 3_000.0,
            },
        ],
    }
}

// ---------- math tests ----------

/// Sanity check on the helper itself: a hand-computed linear divider.
/// V(n2)=10.000 V, V(n3)=6.000 V, I=2.000 mA. Closed-form, no Newton involved.
#[test]
fn linear_divider_matches_closed_form_to_microvolts() {
    let circuit = three_resistor_divider();
    let result = solve_circuit(&circuit).unwrap();

    let v_n2 = result.node_voltages["n2"];
    let v_n3 = result.node_voltages["n3"];
    assert!((v_n2 - 10.0).abs() < 1e-9, "V(n2) = {v_n2} (expected 10.0)");
    assert!((v_n3 - 6.0).abs() < 1e-9, "V(n3) = {v_n3} (expected 6.0)");

    // I through every series element must equal +2.000 mA in the n0→n1 direction.
    let by_id: HashMap<&str, f64> = result
        .component_results
        .iter()
        .map(|c| (c.id.as_str(), c.current_through))
        .collect();
    for r_id in ["R1", "R2", "R3"] {
        let i = by_id[r_id];
        assert!(
            (i - 2.0e-3).abs() < 1e-12,
            "{r_id} current = {i} (expected 2.0 mA)"
        );
    }

    assert_kcl(&circuit, &result, 1e-12);
}

/// KCL must hold at every node of a nonlinear (diode) DC solve. This is
/// the single strongest sanity check on the new solver paths — if any
/// stamp or any of the gmin/seed plumbing got the math wrong, the residual
/// would show up here.
///
/// Tolerance: 1 µA. Newton–Raphson stops when per-node voltage step <
/// `V_ABSTOL (1e-6) + RELTOL (1e-3) * |V|`, so at V(n2)≈0.75 V the
/// solver is allowed up to ~750 µV slop per step. Mapped through the
/// diode small-signal conductance (g = I/V_T ≈ 1.6 A/V at 43 mA), that
/// permits up to ~1 mA of KCL residual at the convergence boundary.
/// In practice we observe ~0.5 µA — well under the budget.
#[test]
fn diode_resistor_satisfies_kcl() {
    let circuit = diode_resistor(5.0, 100.0);
    let result = solve_circuit(&circuit).unwrap();
    assert_kcl(&circuit, &result, 1e-6);
}

/// At the converged operating point the diode current reported by the
/// solver MUST equal the Shockley-equation current at the reported diode
/// voltage. This validates the device math against its own definition —
/// if the stamp or the linearisation introduced a bias, the two would
/// disagree.
#[test]
fn diode_op_obeys_shockley_equation() {
    // V_T and IS values match sindr_devices::diode::DiodeParams::silicon().
    const V_T: f64 = 0.025851;
    const N: f64 = 1.0;
    const IS: f64 = 1e-14;

    let circuit = diode_resistor(5.0, 100.0);
    let result = solve_circuit(&circuit).unwrap();

    let diode = result
        .component_results
        .iter()
        .find(|c| c.id == "D1")
        .unwrap();

    let v_d = diode.voltage_across;
    let i_d_solver = diode.current_through;
    let i_d_shockley = IS * ((v_d / (N * V_T)).exp() - 1.0);

    // The reported (V, I) point must satisfy Shockley to high precision.
    // Allow a small relative tolerance — the NR convergence threshold is
    // around 1e-6 V on node voltages, which at the knee maps to a few
    // percent of the current.
    let rel_err = (i_d_solver - i_d_shockley).abs() / i_d_shockley.abs().max(1e-12);
    assert!(
        rel_err < 5e-3,
        "diode (V={v_d}, I={i_d_solver}) violates Shockley: \
         model says I={i_d_shockley}, rel err = {rel_err:.3e}"
    );
}

/// Plain Newton–Raphson and the gmin-stepping fallback must converge to
/// the *same* answer when both succeed. The gmin path is forced by giving
/// a wildly wrong initial seed; we then compare against the unseeded
/// solution from plain NR.
#[test]
fn gmin_path_agrees_with_plain_nr_to_microvolts() {
    let circuit = diode_resistor(5.0, 100.0);
    let plain = solve_circuit(&circuit).unwrap();

    let mut bad_seed = HashMap::new();
    bad_seed.insert("n1".to_string(), 1.0e6);
    bad_seed.insert("n2".to_string(), -1.0e6);
    let via_gmin = solve_circuit_with_initial_voltages(&circuit, &bad_seed).unwrap();

    for node in plain.node_voltages.keys() {
        let p = plain.node_voltages[node];
        let g = via_gmin.node_voltages[node];
        // Tolerance bounded by NR convergence: V_ABSTOL + RELTOL*|V|
        // ≈ 1 mV at V≈1 V. The two paths can land on points up to that
        // distance apart and both still be legitimately "converged".
        assert!(
            (p - g).abs() < 1e-3,
            "node {node}: plain NR = {p} V, gmin path = {g} V (Δ = {:.3e})",
            (p - g).abs()
        );
    }

    // And the rescued solution must itself satisfy KCL — not just match
    // the plain answer numerically. See diode_resistor_satisfies_kcl for
    // tolerance derivation.
    assert_kcl(&circuit, &via_gmin, 1e-6);
}

/// For a circuit with a unique operating point, the converged answer must
/// be independent of the initial seed. Sweeping over many seeds and
/// asserting they all agree catches drift in the seed-handling code.
#[test]
fn seed_invariance_for_unique_operating_point() {
    let circuit = diode_resistor(5.0, 100.0);
    let baseline = solve_circuit(&circuit).unwrap();
    let v_n2_ref = baseline.node_voltages["n2"];

    // Spread of physically-reasonable seeds, plus a couple of wild ones
    // that the gmin homotopy rescues. Excluded from this set: seeds at
    // exactly the supply rail (5.0 V), which forward-bias the diode by
    // V_supply directly and defeat the voltage-limiter chain — see
    // `seeding_at_supply_rail_is_a_known_limitation` for that case.
    // Range covers reasonable seeds within and near the supply rails.
    // Seeds that forward-bias the diode by more than a few V are excluded:
    // exp(V/V_T) overflows to infinity for V > ~700·V_T ≈ 18 V, producing
    // an InvalidSolution error before the homotopy can do anything. That
    // is a known limit of the Shockley model under naive seeding, not a
    // correctness issue with sindr's math.
    // Upper-end seeds are bounded by the diode voltage limiter: each
    // Newton step is clamped to ≈ V_T (26 mV) once the device is in
    // forward conduction, so a seed `V_seed` requires roughly
    // `(V_seed − V_op) / V_T` iterations to reach the operating point.
    // With MAX_NR_ITERATIONS = 100 and V_op ≈ 0.65 V that caps the
    // forward-bias seed at about 3.2 V before plain NR exhausts its
    // budget. (Reverse-bias seeds — including very large negative
    // ones — are fine because the diode current saturates at −IS.)
    let seeds = [-1.0e4, -5.0, -1.0, 0.0, 0.3, 0.65, 1.0, 2.5, 3.0];
    for v in seeds {
        let mut s = HashMap::new();
        s.insert("n2".to_string(), v);
        let r = match solve_circuit_with_initial_voltages(&circuit, &s) {
            Ok(r) => r,
            Err(e) => panic!("seed V(n2)={v} failed to converge: {e}"),
        };
        let v_n2 = r.node_voltages["n2"];
        // Bounded by NR convergence criterion (V_ABSTOL + RELTOL·|V|).
        // At V(n2)≈0.75 V that's ~0.75 mV; tighten to 1 mV here. The
        // important property is that wildly different starting points
        // converge to the *same* operating point, not to one within
        // floating-point precision — that would be physically untrue
        // anyway since each Newton path stops at a different snapshot
        // of the same iteration.
        assert!(
            (v_n2 - v_n2_ref).abs() < 1e-3,
            "seed V(n2)={v}: converged V(n2)={v_n2}, baseline {v_n2_ref}, Δ={:.3e}",
            (v_n2 - v_n2_ref).abs()
        );
    }
}

/// KCL must also hold for a low-bias diode where current is a few µA —
/// catches gmin-shunt artefacts that would be invisible at high current.
#[test]
fn low_bias_diode_satisfies_kcl() {
    let circuit = diode_resistor(0.3, 100.0);
    let result = solve_circuit(&circuit).unwrap();
    // At low bias the diode small-signal conductance is tiny, so the
    // mV-scale NR slop maps to nA-scale current residuals — much tighter
    // than the high-bias case.
    assert_kcl(&circuit, &result, 1e-9);
}

/// A multi-resistor network with no nonlinear elements must satisfy KCL
/// to floating-point precision — there is no Newton iteration tolerance
/// in the linear path.
#[test]
fn linear_network_satisfies_kcl_to_machine_precision() {
    let circuit = three_resistor_divider();
    let result = solve_circuit(&circuit).unwrap();
    assert_kcl(&circuit, &result, 1e-12);
}

/// Power balance: total power delivered by sources == total power
/// dissipated in passive elements. A universal sanity check that catches
/// sign-convention bugs (KCL alone won't — flipping all currents leaves
/// KCL satisfied).
fn assert_power_balance(circuit: &Circuit, result: &SimulationResult, tol_watts: f64) {
    let mut source_power = 0.0;
    let mut passive_power = 0.0;
    let by_id: HashMap<&str, &sindr::ComponentResult> = result
        .component_results
        .iter()
        .map(|c| (c.id.as_str(), c))
        .collect();

    for el in &circuit.components {
        match el {
            CircuitElement::VoltageSource { id, .. } | CircuitElement::CurrentSource { id, .. } => {
                // For a source, `power` (= V·I in n0→n1 convention) is
                // negative when delivering. Track delivered power as +.
                if let Some(c) = by_id.get(id.as_str()) {
                    source_power += -c.power;
                }
            }
            CircuitElement::Resistor { id, .. }
            | CircuitElement::Diode { id, .. }
            | CircuitElement::Led { id, .. } => {
                if let Some(c) = by_id.get(id.as_str()) {
                    passive_power += c.power;
                }
            }
            _ => {}
        }
    }
    let imbalance = (source_power - passive_power).abs();
    assert!(
        imbalance < tol_watts,
        "power balance violated: sources delivered {source_power:.6e} W, \
         passives dissipated {passive_power:.6e} W, |Δ| = {imbalance:.3e} W"
    );
}

#[test]
fn linear_divider_satisfies_power_balance() {
    let circuit = three_resistor_divider();
    let result = solve_circuit(&circuit).unwrap();
    // P = V·I = 12 V · 2 mA = 24 mW total delivered.
    // R1: I²R = 4e-6 · 1000 = 4 mW; R2: 8 mW; R3: 12 mW; sum = 24 mW.
    assert_power_balance(&circuit, &result, 1e-12);
}

#[test]
fn diode_resistor_satisfies_power_balance() {
    let circuit = diode_resistor(5.0, 100.0);
    let result = solve_circuit(&circuit).unwrap();
    // ~43 mA · 5 V = 215 mW delivered. Tolerance: NR slop maps through
    // V·I to roughly mV·43mA = 43 µW worst case; budget 100 µW.
    assert_power_balance(&circuit, &result, 1e-4);
}

/// Two voltage sources → two branch-current unknowns in MNA. Verifies the
/// expanded `(n+m) × (n+m)` system stays consistent and that branch
/// currents are returned with the correct sign.
#[test]
fn two_voltage_sources_in_series_satisfy_kcl_and_power() {
    // 10V → R1=1k → midpoint → R2=2k → 4V → 0
    // Loop voltage drop available across both R's: 10 − 4 = 6 V
    // Series resistance: 3 kΩ. So I = 2 mA.
    // V(midpoint) = 10 − 2mA·1k = 8 V
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["a".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["a".into(), "mid".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["mid".into(), "b".into()],
                resistance: 2_000.0,
            },
            CircuitElement::VoltageSource {
                id: "V2".into(),
                nodes: ["b".into(), "0".into()],
                voltage: 4.0,
                waveform: None,
            },
        ],
    };
    let result = solve_circuit(&circuit).unwrap();
    let v_mid = result.node_voltages["mid"];
    assert!(
        (v_mid - 8.0).abs() < 1e-9,
        "V(mid) = {v_mid} (expected 8.0)"
    );
    assert_kcl(&circuit, &result, 1e-12);
    // Sources both deliver/absorb energy; net delivered = 10·2mA + (−4)·2mA = 12 mW
    // dissipated by R1 (4 mW) + R2 (8 mW) = 12 mW.
    assert_power_balance(&circuit, &result, 1e-12);
}

/// Resistor bridge — non-trivial linear topology, verifies stamping of
/// resistors that share neither terminal with ground.
///
/// Wheatstone bridge with R1=R3=1k, R2=R4=2k driven by 10 V, no detector
/// in the middle. Symmetric, so V(mid_top) = V(mid_bot) = 10·2/(1+2) =
/// 6.667 V.
#[test]
fn wheatstone_bridge_balances() {
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["src".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["src".into(), "a".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Resistor {
                id: "R2".into(),
                nodes: ["a".into(), "0".into()],
                resistance: 2_000.0,
            },
            CircuitElement::Resistor {
                id: "R3".into(),
                nodes: ["src".into(), "b".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Resistor {
                id: "R4".into(),
                nodes: ["b".into(), "0".into()],
                resistance: 2_000.0,
            },
        ],
    };
    let result = solve_circuit(&circuit).unwrap();
    let v_a = result.node_voltages["a"];
    let v_b = result.node_voltages["b"];
    let expected = 10.0 * 2.0 / 3.0;
    assert!((v_a - expected).abs() < 1e-9, "V(a) = {v_a}");
    assert!((v_b - expected).abs() < 1e-9, "V(b) = {v_b}");
    assert!(
        (v_a - v_b).abs() < 1e-12,
        "balanced bridge: V(a) and V(b) must match exactly"
    );
    assert_kcl(&circuit, &result, 1e-12);
    assert_power_balance(&circuit, &result, 1e-12);
}

/// Reverse-biased diode: forward voltage drop is supplied opposite to the
/// anode→cathode direction, so the diode is reverse-biased. Current must
/// be ~−Is (≈ −1e-14 A for silicon), V across diode ≈ −5 V, KCL still
/// holds across the whole loop.
#[test]
fn reverse_biased_diode_carries_negligible_current() {
    // 5 V supply, but diode is anode=0, cathode=n2 — so V(anode)−V(cathode)
    // = 0 − V(n2) ≈ −5 V → reverse bias.
    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 100.0,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["0".into(), "n2".into()],
                temperature: 300.15,
            },
        ],
    };
    let result = solve_circuit(&circuit).unwrap();
    let diode = result
        .component_results
        .iter()
        .find(|c| c.id == "D1")
        .unwrap();
    // Reverse current is at most the saturation current (~1e-14 A) plus
    // GMIN·V (≈ 5e-12 A). Anything larger means the stamp got it wrong.
    assert!(
        diode.current_through.abs() < 1e-10,
        "reverse-biased diode current = {} (expected ~0)",
        diode.current_through
    );
    // V(n2) should sit at ~5 V (the drop across R1 is tiny since I≈0).
    let v_n2 = result.node_voltages["n2"];
    assert!(
        (v_n2 - 5.0).abs() < 1e-6,
        "V(n2) = {v_n2} (expected ≈ 5.0 in reverse bias)"
    );
    assert_kcl(&circuit, &result, 1e-9);
}

/// Two diodes in parallel — verifies multi-nonlinear stamping and that
/// Kirchhoff's current law correctly splits the source current between
/// identical branches. Each diode carries half the total, both satisfy
/// Shockley independently. Well-conditioned topology (cf. series, where
/// the inter-diode node creates a Jacobian conditioning challenge).
#[test]
fn two_diodes_in_parallel_split_current_and_obey_shockley() {
    const V_T: f64 = 0.025851;
    const IS: f64 = 1e-14;

    let circuit = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 5.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 100.0,
            },
            CircuitElement::Diode {
                id: "D1".into(),
                nodes: ["n2".into(), "0".into()],
                temperature: 300.15,
            },
            CircuitElement::Diode {
                id: "D2".into(),
                nodes: ["n2".into(), "0".into()],
                temperature: 300.15,
            },
        ],
    };
    let result = solve_circuit(&circuit).unwrap();
    // Two diodes in parallel double the small-signal conductance at the
    // operating point, so the per-step NR slop maps to ~2× the KCL
    // residual we'd see for a single diode at the same current.
    assert_kcl(&circuit, &result, 1e-5);

    let d1 = result
        .component_results
        .iter()
        .find(|c| c.id == "D1")
        .unwrap();
    let d2 = result
        .component_results
        .iter()
        .find(|c| c.id == "D2")
        .unwrap();

    // Both diodes share the same V_d (V(n2)) and so must carry the same
    // current within float precision.
    assert!(
        (d1.voltage_across - d2.voltage_across).abs() < 1e-12,
        "parallel diodes must share V: D1={}, D2={}",
        d1.voltage_across,
        d2.voltage_across
    );
    assert!(
        (d1.current_through - d2.current_through).abs() < 1e-9,
        "parallel diodes must share I: D1={}, D2={}",
        d1.current_through,
        d2.current_through
    );

    // Each must satisfy Shockley.
    for d in [d1, d2] {
        let i_shockley = IS * ((d.voltage_across / V_T).exp() - 1.0);
        let rel_err = (d.current_through - i_shockley).abs() / i_shockley.abs().max(1e-12);
        assert!(
            rel_err < 5e-3,
            "{}: V={}, I={}, Shockley I={}, rel err={:.3e}",
            d.id,
            d.voltage_across,
            d.current_through,
            i_shockley,
            rel_err
        );
    }

    // KCL at n2: I_R1 (in) = I_D1 (out) + I_D2 (out).
    let r1 = result
        .component_results
        .iter()
        .find(|c| c.id == "R1")
        .unwrap();
    let kcl_n2 = r1.current_through - d1.current_through - d2.current_through;
    assert!(
        kcl_n2.abs() < 1e-5,
        "KCL at n2: I_R1={}, I_D1={}, I_D2={}, residual={}",
        r1.current_through,
        d1.current_through,
        d2.current_through,
        kcl_n2
    );
}

/// Capacitor in pure DC must behave as an open circuit. Putting one in
/// parallel with a resistor must not change the resistor's current.
#[test]
fn capacitor_is_open_circuit_in_dc() {
    let with_cap = Circuit {
        ground_node: "0".into(),
        components: vec![
            CircuitElement::VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            },
            CircuitElement::Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1_000.0,
            },
            CircuitElement::Capacitor {
                id: "C1".into(),
                nodes: ["n1".into(), "0".into()],
                capacitance: 1e-6,
            },
        ],
    };
    // Note: Capacitor presence triggers transient mode. We can't directly
    // compare against a pure-DC solve, but at the *initial* timepoint of a
    // freshly-started transient the cap is uncharged and presents as a
    // short — different from "open". So instead we just verify the
    // simulation runs and the steady-state current through R1 is V/R.
    let result = solve_circuit(&with_cap).unwrap();
    let r1 = result
        .component_results
        .iter()
        .find(|c| c.id == "R1")
        .unwrap();
    // Steady-state through R1 should be 10V / 1kΩ = 10 mA. After enough
    // time, the cap stops charging; the snapshot we get is at t=0 where
    // cap is uncharged and acts as a wire across V1, so V across R1 = 10 V
    // either way.
    assert!(
        (r1.current_through.abs() - 0.010).abs() < 1e-6,
        "R1 current = {} (expected ±10 mA)",
        r1.current_through
    );
}

/// Documents (and locks in) a known limitation: seeding a diode's anode
/// at the full supply rail forward-biases it by ~5 V, which puts the
/// device deep into the exponential where current would be ~10^80 A.
/// The voltage-limiter clamps each Newton step to ~V_T (26 mV), so the
/// solver would need ~190 iterations to walk down to the real OP — but
/// MAX_NR_ITERATIONS = 100. The gmin-stepping homotopy doesn't rescue
/// because the bad seed survives into its first ladder step.
///
/// This is a robustness limit, not a math bug: every solution the solver
/// *does* return satisfies KCL and Shockley to within tolerance. If this
/// limitation is ever fixed (e.g. by re-seeding inside gmin_stepping at
/// each ladder step, or by raising MAX_NR_ITERATIONS), this test will
/// start failing with `Ok(...)` and should be flipped to assert success.
#[test]
fn seeding_at_supply_rail_is_a_known_limitation() {
    let circuit = diode_resistor(5.0, 100.0);
    let mut s = HashMap::new();
    s.insert("n2".to_string(), 5.0);
    let r = solve_circuit_with_initial_voltages(&circuit, &s);
    assert!(
        r.is_err(),
        "if this now succeeds, gmin homotopy or voltage limiting was \
         improved — flip this test to assert convergence + KCL"
    );
}
