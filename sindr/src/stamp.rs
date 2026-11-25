//! Component → MNA matrix stamps.
//!
//! Each element's DC and transient stamps live with its device in
//! [`crate::element`]:
//! a routine that writes its conductance / constraint contributions into
//! the [`MnaSystem`]. For nonlinear devices, the stamp evaluates the
//! device's companion model at the current operating point passed in via
//! the optional `v_prev` solution vector — Newton–Raphson re-stamps every
//! iteration as the operating point moves.
//!
//! [`stamp_circuit`] is the DC dispatcher used by every analysis path; the
//! helpers several elements share (`stamp_resistor_at`, `stamp_voltage_source_at`,
//! the transient's `stamp_companion_at`, …) are also exposed for the
//! transient and AC pipelines.

use nalgebra::DVector;

use sindr_devices::diode::{self, DiodeParams};

use crate::circuit::Circuit;
use crate::element::{layout::Layout, DcStamp};
use crate::error::SimError;
#[cfg(test)]
use crate::handle::NodeId;
use crate::mna::MnaSystem;
use crate::node_map::{NodeMap, Terminals};

/// Switch closed resistance (very low, near short circuit).
pub const SWITCH_R_CLOSED: f64 = 0.01;
/// Switch open resistance (very high, near open circuit).
pub const SWITCH_R_OPEN: f64 = 1e9;

/// Stamp all components in a circuit into the MNA system.
///
/// Iterates through every component and stamps its DC model. Branch rows
/// follow the node voltages, numbered in the order the elements that take
/// them appear in the component list.
///
/// When `v_prev` is `Some`, nonlinear components (diodes, LEDs, BJTs, MOSFETs,
/// JFETs, IGBTs, ...) are stamped using their companion model evaluated at the
/// given operating point.
/// When `v_prev` is `None`, nonlinear components are skipped (linear solve).
///
/// # Errors
///
/// [`SimError::InvalidComponent`] for a resistor with zero or negative
/// resistance.
pub fn stamp_circuit(
    circuit: &Circuit,
    system: &mut MnaSystem,
    node_map: &NodeMap,
    v_prev: Option<&DVector<f64>>,
) -> Result<(), SimError> {
    let layout = Layout::new(circuit);
    let terminals = Terminals::new(circuit, node_map);
    stamp_circuit_with(circuit, system, node_map, &layout, &terminals, v_prev)
}

/// [`stamp_circuit`] with `circuit`'s branch rows numbered and terminals
/// resolved already, for a loop that stamps the same circuit many times.
pub(crate) fn stamp_circuit_with(
    circuit: &Circuit,
    system: &mut MnaSystem,
    node_map: &NodeMap,
    layout: &Layout,
    terminals: &Terminals,
    v_prev: Option<&DVector<f64>>,
) -> Result<(), SimError> {
    for (k, component) in circuit.components.iter().enumerate() {
        let mut s = DcStamp::new(system, node_map, layout, terminals, k, v_prev);
        component.device().stamp_dc(&mut s)?;
    }
    Ok(())
}

/// Stamp a resistor (conductance `g = 1/R`) into the G submatrix.
///
/// For nodes `p` and `q` with conductance `g`:
/// ```text
///   A[p,p] += g,  A[q,q] += g,  A[p,q] -= g,  A[q,p] -= g
/// ```
/// Ground entries (where index is `None`) are skipped.
///
/// Terminals are matrix indices (`None` is ground).
pub(crate) fn stamp_resistor_at(
    system: &mut MnaSystem,
    [p, q]: [Option<usize>; 2],
    resistance: f64,
) {
    let g = 1.0 / resistance;

    if let Some(pi) = p {
        system.a[(pi, pi)] += g;
    }
    if let Some(qi) = q {
        system.a[(qi, qi)] += g;
    }
    if let (Some(pi), Some(qi)) = (p, q) {
        system.a[(pi, qi)] -= g;
        system.a[(qi, pi)] -= g;
    }
}

/// Stamp an independent voltage source into B, C submatrices and RHS.
///
/// `nodes[0]` is the positive terminal (`p`), `nodes[1]` the negative (`q`).
/// Branch index `k` is the row/column for this source in the MNA system.
///
/// Stamps:
/// ```text
///   A[p,k] += 1, A[q,k] -= 1, A[k,p] += 1, A[k,q] -= 1, b[k] = voltage
/// ```
///
/// Terminals are matrix indices (`None` is ground).
pub(crate) fn stamp_voltage_source_at(
    system: &mut MnaSystem,
    [p, q]: [Option<usize>; 2],
    voltage: f64,
    branch: usize,
) {
    // B submatrix (node rows, branch column)
    if let Some(pi) = p {
        system.a[(pi, branch)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(qi, branch)] -= 1.0;
    }

    // C submatrix (branch row, node columns)
    if let Some(pi) = p {
        system.a[(branch, pi)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(branch, qi)] -= 1.0;
    }

    // RHS: voltage value
    system.b[branch] = voltage;
}

/// Stamp an independent current source into the RHS vector.
///
/// Current flows from `nodes[0]` toward `nodes[1]`.
/// KCL convention: current *entering* a node is positive.
/// ```text
///   b[to]   += current   (current enters the to-node)
///   b[from] -= current   (current leaves the from-node)
/// ```
///
/// Terminals are matrix indices (`None` is ground).
pub(crate) fn stamp_current_source_at(
    system: &mut MnaSystem,
    [from, to]: [Option<usize>; 2],
    current: f64,
) {
    if let Some(ti) = to {
        system.b[ti] += current;
    }
    if let Some(fi) = from {
        system.b[fi] -= current;
    }
}

/// Stamp a transient companion branch between matrix indices `[p, q]`
/// (`None` is ground): conductance `g` plus the history current `i_hist`,
/// so the current from `p` to `q` is `g·v + i_hist`. Every capacitor,
/// inductor and stored charge over one step takes this form.
pub(crate) fn stamp_companion_at(
    system: &mut MnaSystem,
    [p, q]: [Option<usize>; 2],
    g: f64,
    i_hist: f64,
) {
    if let Some(pi) = p {
        system.a[(pi, pi)] += g;
        system.b[pi] -= i_hist;
    }
    if let Some(qi) = q {
        system.a[(qi, qi)] += g;
        system.b[qi] += i_hist;
    }
    if let (Some(pi), Some(qi)) = (p, q) {
        system.a[(pi, qi)] -= g;
        system.a[(qi, pi)] -= g;
    }
}

/// [`stamp_diode_companion_at`] by node name.
#[cfg(test)]
pub(crate) fn stamp_diode_companion(
    system: &mut MnaSystem,
    node_map: &NodeMap,
    nodes: &[NodeId; 2],
    v_prev: &DVector<f64>,
    params: &DiodeParams,
) {
    stamp_diode_companion_at(
        system,
        [node_map.index(&nodes[0]), node_map.index(&nodes[1])],
        v_prev,
        params,
    );
}

/// Stamp a diode/LED companion model into the MNA system.
///
/// Evaluates the companion model at the current operating point (from v_prev)
/// and stamps the resulting conductance and current source.
///
/// nodes[0] = anode, nodes[1] = cathode.
///
/// Terminals as matrix indices (`None` is ground).
pub(crate) fn stamp_diode_companion_at(
    system: &mut MnaSystem,
    [p, q]: [Option<usize>; 2],
    v_prev: &DVector<f64>,
    params: &DiodeParams,
) {
    // Compute diode voltage from operating point
    let v_anode = p.map_or(0.0, |i| v_prev[i]);
    let v_cathode = q.map_or(0.0, |i| v_prev[i]);
    let v_d = v_anode - v_cathode;

    // Get companion model parameters
    let (g_d, i_eq) = diode::diode_companion(v_d, params);

    // Stamp conductance (same pattern as resistor with g = g_d)
    if let Some(pi) = p {
        system.a[(pi, pi)] += g_d;
    }
    if let Some(qi) = q {
        system.a[(qi, qi)] += g_d;
    }
    if let (Some(pi), Some(qi)) = (p, q) {
        system.a[(pi, qi)] -= g_d;
        system.a[(qi, pi)] -= g_d;
    }

    // Stamp current source (current flows anode -> cathode)
    if let Some(pi) = p {
        system.b[pi] -= i_eq;
    }
    if let Some(qi) = q {
        system.b[qi] += i_eq;
    }
}

/// Stamp a Voltage-Controlled Voltage Source (VCVS).
///
/// V_out = mu * V_control
/// nodes: [out+, out-], control_nodes: [ctrl+, ctrl-]
///
/// MNA stamp: adds a branch equation V(out+) - V(out-) = mu * (V(ctrl+) - V(ctrl-))
///
/// Terminals as matrix indices (`None` is ground).
pub(crate) fn stamp_vcvs_at(
    system: &mut MnaSystem,
    [p, q]: [Option<usize>; 2],
    [cp, cq]: [Option<usize>; 2],
    gain: f64,
    branch: usize,
) {
    // B submatrix: node rows, branch column
    if let Some(pi) = p {
        system.a[(pi, branch)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(qi, branch)] -= 1.0;
    }

    // C submatrix: branch row - V(p) - V(q) - mu*V(cp) + mu*V(cq) = 0
    if let Some(pi) = p {
        system.a[(branch, pi)] += 1.0;
    }
    if let Some(qi) = q {
        system.a[(branch, qi)] -= 1.0;
    }
    if let Some(cpi) = cp {
        system.a[(branch, cpi)] -= gain;
    }
    if let Some(cqi) = cq {
        system.a[(branch, cqi)] += gain;
    }
    // RHS = 0 (no independent voltage)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::circuit::{Circuit, CircuitElement};
    use crate::element::bjt::stamp_bjt_companion;
    use crate::element::igbt::stamp_igbt_companion;
    use crate::element::jfet::stamp_jfet_companion;
    use crate::element::mosfet::stamp_mosfet_companion;
    use crate::element::{
        Bjt, Capacitor, Crystal, CurrentSource, Diode, FerriteBead, Igbt, Inductor, Jfet, Led,
        Mosfet, Photodiode, Resistor, SchottkyDiode, Transformer, Varactor, VoltageSource,
        ZenerDiode,
    };
    use approx::assert_relative_eq;
    use sindr_devices::bjt::{BjtKind, BjtParams};
    use sindr_devices::jfet::JfetKind;
    use sindr_devices::mosfet::{MosfetKind, MosfetParams};

    /// A transformer at DC shorts both windings on its own two branch
    /// rows, and a voltage source after it still gets the next row.
    #[test]
    fn dc_transformer_takes_its_two_branch_rows() {
        let r = |id: &str, a: &str, b: &str, ohms: f64| {
            CircuitElement::Resistor(Resistor {
                id: id.into(),
                nodes: [a.into(), b.into()],
                resistance: ohms,
            })
        };
        let v = |id: &str, a: &str, volts: f64| {
            CircuitElement::VoltageSource(VoltageSource {
                id: id.into(),
                nodes: [a.into(), "0".into()],
                voltage: volts,
                waveform: None,
            })
        };
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                v("V1", "a", 2.0),
                r("R1", "a", "p", 1e3),
                CircuitElement::Transformer(Transformer {
                    id: "T1".into(),
                    nodes: ["p".into(), "0".into(), "s".into(), "0".into()],
                    l1: 1e-3,
                    l2: 1e-3,
                    k: 0.99,
                    saturation: None,
                }),
                r("R2", "s", "0", 1e3),
                v("V2", "b", 3.0),
                r("R3", "b", "0", 1e3),
            ],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n = node_map.num_nodes();
        let mut system = MnaSystem::new(n, circuit.count_voltage_sources());
        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();
        let x = system.solve().unwrap();
        let at = |node: &str| x[node_map.index(node).unwrap()];
        assert_relative_eq!(at("p"), 0.0, epsilon = 1e-12);
        assert_relative_eq!(at("s"), 0.0, epsilon = 1e-12);
        assert_relative_eq!(at("b"), 3.0, epsilon = 1e-12);
        // Rows: V1, then the two windings, then V2. The primary carries
        // V1's 2 mA through R1.
        assert_relative_eq!(x[n + 1], 2e-3, epsilon = 1e-12);
        assert_relative_eq!(x[n + 3], -3e-3, epsilon = 1e-12);
    }

    /// Resistor stamp isolation: R=1k between n1 and ground.
    /// A matrix is 1x1 (one non-ground node, zero vsources).
    /// A[0,0] should equal 0.001.
    #[test]
    fn stamp_resistor_to_ground() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 1000.0,
            })],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();

        assert_eq!(system.size(), 1);
        assert_relative_eq!(system.a[(0, 0)], 0.001, epsilon = 1e-15);
    }

    /// Resistor stamp between two non-ground nodes: R=1k between n1 and n2.
    /// A[0,0]=0.001, A[1,1]=0.001, A[0,1]=-0.001, A[1,0]=-0.001.
    #[test]
    fn stamp_resistor_between_nodes() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "n2".into()],
                resistance: 1000.0,
            })],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();

        let i1 = node_map.index("n1").unwrap();
        let i2 = node_map.index("n2").unwrap();
        assert_relative_eq!(system.a[(i1, i1)], 0.001, epsilon = 1e-15);
        assert_relative_eq!(system.a[(i2, i2)], 0.001, epsilon = 1e-15);
        assert_relative_eq!(system.a[(i1, i2)], -0.001, epsilon = 1e-15);
        assert_relative_eq!(system.a[(i2, i1)], -0.001, epsilon = 1e-15);
    }

    /// VoltageSource stamp: V=10V, n1(+), ground(-). 1 vsource.
    /// A[0,1]=1.0, A[1,0]=1.0, b[1]=10.0.
    #[test]
    fn stamp_voltage_source_to_ground() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::VoltageSource(VoltageSource {
                id: "V1".into(),
                nodes: ["n1".into(), "0".into()],
                voltage: 10.0,
                waveform: None,
            })],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), circuit.count_voltage_sources());

        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();

        // n1 is index 0, vsource branch is index 1
        assert_relative_eq!(system.a[(0, 1)], 1.0, epsilon = 1e-15);
        assert_relative_eq!(system.a[(1, 0)], 1.0, epsilon = 1e-15);
        assert_relative_eq!(system.b[1], 10.0, epsilon = 1e-15);
    }

    /// CurrentSource stamp: I=0.002A from ground to n1.
    /// b[0] = 0.002 (current injected into n1).
    #[test]
    fn stamp_current_source_from_ground() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::CurrentSource(CurrentSource {
                id: "I1".into(),
                nodes: ["0".into(), "n1".into()],
                current: 0.002,
                waveform: None,
            })],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        stamp_circuit(&circuit, &mut system, &node_map, None).unwrap();

        assert_relative_eq!(system.b[0], 0.002, epsilon = 1e-15);
    }

    /// Invalid resistance (zero) returns SimError::InvalidComponent.
    #[test]
    fn stamp_zero_resistance_returns_error() {
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Resistor(Resistor {
                id: "R1".into(),
                nodes: ["n1".into(), "0".into()],
                resistance: 0.0,
            })],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let mut system = MnaSystem::new(node_map.num_nodes(), 0);

        let result = stamp_circuit(&circuit, &mut system, &node_map, None);
        assert!(result.is_err());
        match result.unwrap_err() {
            SimError::InvalidComponent { id, reason } => {
                assert_eq!(id, "R1");
                assert!(reason.contains("resistance"), "{reason}");
            }
            other => panic!("expected InvalidComponent, got: {other}"),
        }
    }

    /// BJT stamp test: NPN in active region (Vb=0.7, Vc=5.0, Ve=0).
    /// Verifies all 9 G-matrix entries and 3 RHS entries against hand-calculated values.
    #[test]
    fn stamp_bjt_companion_matrix_entries() {
        use sindr_devices::bjt::{BjtKind, BjtParams};
        use sindr_devices::diode::V_T;

        // Circuit: BJT (base=n1, collector=n2, emitter=GND) + Vcc voltage source
        // We need nodes n1 (base) and n2 (collector); emitter is ground.
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["n2".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rb".into(),
                    nodes: ["n1".into(), "0".into()],
                    resistance: 100000.0,
                }),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["n1".into(), "n2".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: None,
                }),
            ],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n_nodes = node_map.num_nodes();
        let n_vsources = circuit.count_voltage_sources();
        let size = n_nodes + n_vsources;

        let mut system = MnaSystem::new(n_nodes, n_vsources);

        // Set up v_prev: Vb=0.7, Vc=5.0, Ve=0 (ground)
        let mut v_prev = nalgebra::DVector::zeros(size);
        let bi = node_map.index("n1").unwrap();
        let ci = node_map.index("n2").unwrap();
        v_prev[bi] = 0.7;
        v_prev[ci] = 5.0;

        // Stamp only the BJT
        let params = BjtParams::new(100.0);
        let nodes: [NodeId; 3] = ["n1".into(), "n2".into(), "0".into()];
        stamp_bjt_companion(
            &mut system,
            &node_map,
            &nodes,
            &v_prev,
            &params,
            BjtKind::Npn,
        );

        // Hand-calculate expected companion values at Vbe=0.7, Vbc=0.7-5.0=-4.3
        let vbe = 0.7;
        let vbc = -4.3;
        let nf_vt = 1.0 * V_T;
        let nr_vt = 1.0 * V_T;
        let exp_be = (vbe / nf_vt).exp();
        let exp_bc = (vbc / nr_vt).exp();
        let g_be = (1e-14 / nf_vt) * exp_be;
        let g_bc = (1e-14 / nr_vt) * exp_bc;
        let bf = 100.0;
        let br = 1.0;
        let alpha_r = br / (br + 1.0);

        // G matrix: Row B (base)
        assert_relative_eq!(system.a[(bi, bi)], g_be / bf + g_bc / br, epsilon = 1e-15);
        assert_relative_eq!(system.a[(bi, ci)], -g_bc / br, epsilon = 1e-15);
        // G[B,E] is ground, skipped

        // G matrix: Row C (collector) -- CORRECTED signs per docs/bjt-ebers-moll.md
        assert_relative_eq!(system.a[(ci, bi)], g_be - g_bc / alpha_r, epsilon = 1e-15);
        assert_relative_eq!(system.a[(ci, ci)], g_bc / alpha_r, epsilon = 1e-15);
        // G[C,E] is ground, skipped

        // Diagonal entries should be positive (self-conductance)
        assert!(system.a[(bi, bi)] > 0.0, "G[B,B] should be positive");
        // G[C,C] is positive (g_bc/alpha_r term, corrected sign)
        assert!(system.a[(ci, ci)] >= 0.0, "G[C,C] should be non-negative");

        // Cross-coupling: G[C,B] should contain the g_be term (large in active region)
        assert!(
            system.a[(ci, bi)] > 0.1,
            "G[C,B] should be substantial in active region, got {}",
            system.a[(ci, bi)]
        );

        // RHS should be non-zero (companion current sources present)
        assert!(system.b[bi].abs() > 1e-10, "RHS[B] should be non-zero");
        assert!(system.b[ci].abs() > 1e-10, "RHS[C] should be non-zero");

        // Verify RHS values against hand-calculated companion currents
        let i_f = 1e-14 * (exp_be - 1.0);
        let i_r = 1e-14 * (exp_bc - 1.0);
        let ic = i_f - i_r / alpha_r;
        let ib = i_f / bf + i_r / br;

        let ib_eq = ib - (g_be / bf) * vbe - (g_bc / br) * vbc;
        let ic_eq = ic - g_be * vbe + (g_bc / alpha_r) * vbc;

        assert_relative_eq!(system.b[bi], -ib_eq, epsilon = 1e-15);
        assert_relative_eq!(system.b[ci], -ic_eq, epsilon = 1e-15);
        // Emitter is ground, so no RHS entry for it (skipped)
    }

    /// KCL column-sum test: each column of the 3x3 BJT Jacobian submatrix
    /// must sum to zero (current into each node sums to zero).
    #[test]
    fn test_bjt_stamp_kcl_column_sum() {
        use sindr_devices::bjt::{BjtKind, BjtParams};

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vbb".into(),
                    nodes: ["n_b".into(), "0".into()],
                    voltage: 0.7,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rc".into(),
                    nodes: ["n_c".into(), "0".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["n_b".into(), "n_c".into(), "0".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: None,
                }),
            ],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n_nodes = node_map.num_nodes();
        let n_vsources = circuit.count_voltage_sources();
        let size = n_nodes + n_vsources;

        let mut system = MnaSystem::new(n_nodes, n_vsources);

        let mut v_prev = nalgebra::DVector::zeros(size);
        let bi = node_map.index("n_b").unwrap();
        let ci = node_map.index("n_c").unwrap();
        v_prev[bi] = 0.7;
        v_prev[ci] = 0.0;

        let params = BjtParams::new(100.0);
        let nodes: [NodeId; 3] = ["n_b".into(), "n_c".into(), "0".into()];
        stamp_bjt_companion(
            &mut system,
            &node_map,
            &nodes,
            &v_prev,
            &params,
            BjtKind::Npn,
        );

        let params2 = BjtParams::new(100.0);
        let bf = params2.bf;
        let br = params2.br;
        let alpha_f = bf / (bf + 1.0);
        let alpha_r = br / (br + 1.0);

        let vbe = 0.7;
        let vbc = 0.7 - 0.0;
        let comp = sindr_devices::bjt::bjt_companion(vbe, vbc, &params2);
        let g_be = comp.g_be;
        let g_bc = comp.g_bc;

        assert!(
            g_bc > 1e-6,
            "g_bc={} should be significant in saturation",
            g_bc
        );

        let col_b = (g_be / bf + g_bc / br) + (g_be - g_bc / alpha_r) + (-g_be / alpha_f + g_bc);
        assert!(
            col_b.abs() < 1e-12,
            "Column B should sum to 0 (KCL), got {}",
            col_b
        );

        let col_c = (-g_bc / br) + (g_bc / alpha_r) + (-g_bc);
        assert!(
            col_c.abs() < 1e-12,
            "Column C should sum to 0 (KCL), got {}",
            col_c
        );

        let col_e = (-g_be / bf) + (-g_be) + (g_be / alpha_f);
        assert!(
            col_e.abs() < 1e-12,
            "Column E should sum to 0 (KCL), got {}",
            col_e
        );

        assert_relative_eq!(system.a[(bi, bi)], g_be / bf + g_bc / br, epsilon = 1e-12);
        assert_relative_eq!(system.a[(bi, ci)], -g_bc / br, epsilon = 1e-12);
        assert_relative_eq!(system.a[(ci, bi)], g_be - g_bc / alpha_r, epsilon = 1e-12);
        assert_relative_eq!(system.a[(ci, ci)], g_bc / alpha_r, epsilon = 1e-12);
    }

    /// Stamp entries for NPN in active mode (Vbe=0.7V, Vbc=-5.0V, BF=100, BR=1).
    #[test]
    fn test_bjt_stamp_entries_npn_active() {
        use sindr_devices::bjt::{BjtKind, BjtParams};

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["n_vcc".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rb".into(),
                    nodes: ["n_b".into(), "0".into()],
                    resistance: 100_000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Re".into(),
                    nodes: ["n_e".into(), "0".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rc".into(),
                    nodes: ["n_c".into(), "0".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["n_b".into(), "n_c".into(), "n_e".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: None,
                }),
            ],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n_nodes = node_map.num_nodes();
        let n_vsources = circuit.count_voltage_sources();
        let size = n_nodes + n_vsources;

        let mut system = MnaSystem::new(n_nodes, n_vsources);

        let bi = node_map.index("n_b").unwrap();
        let ci = node_map.index("n_c").unwrap();
        let ei = node_map.index("n_e").unwrap();

        let mut v_prev = nalgebra::DVector::zeros(size);
        v_prev[bi] = 0.7;
        v_prev[ci] = 5.0;
        v_prev[ei] = 0.0;

        let params = BjtParams::new(100.0);
        let nodes: [NodeId; 3] = ["n_b".into(), "n_c".into(), "n_e".into()];
        stamp_bjt_companion(
            &mut system,
            &node_map,
            &nodes,
            &v_prev,
            &params,
            BjtKind::Npn,
        );

        let vbe = 0.7;
        let vbc = 0.7 - 5.0;
        let comp = sindr_devices::bjt::bjt_companion(vbe, vbc, &params);
        let g_be = comp.g_be;
        let g_bc = comp.g_bc;
        let bf = 100.0;
        let br = 1.0;
        let alpha_f = bf / (bf + 1.0);
        let alpha_r = br / (br + 1.0);

        assert_relative_eq!(system.a[(bi, bi)], g_be / bf + g_bc / br, epsilon = 1e-12);
        assert_relative_eq!(system.a[(bi, ci)], -g_bc / br, epsilon = 1e-12);
        assert_relative_eq!(system.a[(bi, ei)], -g_be / bf, epsilon = 1e-12);

        assert_relative_eq!(system.a[(ci, bi)], g_be + g_bc / alpha_r, epsilon = 1e-6);

        assert_relative_eq!(system.a[(ci, ei)], -g_be, epsilon = 1e-12);

        assert_relative_eq!(system.a[(ei, ei)], g_be / alpha_f, epsilon = 1e-12);
    }

    /// Stamp entries for NPN in saturation (Vbe=0.7V, Vbc=0.3V).
    #[test]
    fn test_bjt_stamp_entries_npn_saturation() {
        use sindr_devices::bjt::{BjtKind, BjtParams};

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["n_vcc".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rb".into(),
                    nodes: ["n_b".into(), "0".into()],
                    resistance: 100_000.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Re".into(),
                    nodes: ["n_e".into(), "0".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "Rc".into(),
                    nodes: ["n_c".into(), "0".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Bjt(Bjt {
                    id: "Q1".into(),
                    nodes: ["n_b".into(), "n_c".into(), "n_e".into()],
                    kind: BjtKind::Npn,
                    bf: 100.0,
                    temperature: 300.15,
                    parasitic_caps: None,
                    model: None,
                }),
            ],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n_nodes = node_map.num_nodes();
        let n_vsources = circuit.count_voltage_sources();
        let size = n_nodes + n_vsources;

        let mut system = MnaSystem::new(n_nodes, n_vsources);

        let bi = node_map.index("n_b").unwrap();
        let ci = node_map.index("n_c").unwrap();
        let ei = node_map.index("n_e").unwrap();

        let mut v_prev = nalgebra::DVector::zeros(size);
        v_prev[bi] = 0.7;
        v_prev[ci] = 0.0;
        v_prev[ei] = 0.0;

        let params = BjtParams::new(100.0);
        let nodes: [NodeId; 3] = ["n_b".into(), "n_c".into(), "n_e".into()];
        stamp_bjt_companion(
            &mut system,
            &node_map,
            &nodes,
            &v_prev,
            &params,
            BjtKind::Npn,
        );

        let vbe = 0.7;
        let vbc = 0.7;
        let comp = sindr_devices::bjt::bjt_companion(vbe, vbc, &params);
        let g_be = comp.g_be;
        let g_bc = comp.g_bc;
        let bf = 100.0;
        let br = 1.0;
        let alpha_f = bf / (bf + 1.0);
        let alpha_r = br / (br + 1.0);

        assert!(
            g_bc > 1e-3,
            "g_bc={} should be significant in saturation",
            g_bc
        );

        assert_relative_eq!(system.a[(bi, bi)], g_be / bf + g_bc / br, epsilon = 0.01);
        assert_relative_eq!(system.a[(bi, ci)], -g_bc / br, epsilon = 0.01);
        assert_relative_eq!(system.a[(bi, ei)], -g_be / bf, epsilon = 0.01);

        assert_relative_eq!(system.a[(ci, bi)], g_be - g_bc / alpha_r, epsilon = 0.01);
        assert_relative_eq!(system.a[(ci, ci)], g_bc / alpha_r, epsilon = 0.01);
        assert_relative_eq!(system.a[(ci, ei)], -g_be, epsilon = 0.01);

        assert_relative_eq!(system.a[(ei, bi)], -g_be / alpha_f + g_bc, epsilon = 0.01);
        assert_relative_eq!(system.a[(ei, ci)], -g_bc, epsilon = 0.01);
        assert_relative_eq!(system.a[(ei, ei)], g_be / alpha_f, epsilon = 0.01);
    }

    /// Transformer DC solve: 10V source on primary → DC solve completes without panic.
    /// Primary and secondary are both near-short-circuits (1e-9 Ohm) in DC.
    #[test]
    fn transformer_dc_solve() {
        use crate::solve_circuit;

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["p1".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["p1".into(), "n1".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Transformer(Transformer {
                    id: "T1".into(),
                    nodes: ["n1".into(), "0".into(), "n2".into(), "0".into()],
                    l1: 1e-3,
                    l2: 4e-3,
                    k: 0.999,
                    saturation: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R2".into(),
                    nodes: ["n2".into(), "0".into()],
                    resistance: 1000.0,
                }),
            ],
            ..Default::default()
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Transformer DC circuit solve failed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        // At DC the primary winding is a short, so after the switch-on
        // transient n1 has fallen from its initial jump to ~0 V.
        let tran = result.transient.unwrap();
        let first = tran.timesteps[0].node_voltages["n1"];
        let v_n1 = result.node_voltages["n1"];
        assert!(first > 1.0, "primary should jump at switch-on, got {first}");
        assert!(
            v_n1.abs() < 0.05,
            "primary should settle near 0 V, got {v_n1}"
        );
    }

    /// Varactor DC stamp: treated as 1e12 Ω shunt (open circuit).
    /// Simple circuit: V1=5V, varactor between n1 and ground.
    /// DC solve should return result without panic.
    #[test]
    fn varactor_dc_circuit_solves() {
        use crate::solve_circuit;

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                CircuitElement::VoltageSource(VoltageSource {
                    id: "V1".into(),
                    nodes: ["n1".into(), "0".into()],
                    voltage: 5.0,
                    waveform: None,
                }),
                CircuitElement::Resistor(Resistor {
                    id: "R1".into(),
                    nodes: ["n1".into(), "n2".into()],
                    resistance: 1000.0,
                }),
                CircuitElement::Varactor(Varactor {
                    id: "CV1".into(),
                    nodes: ["n2".into(), "0".into()],
                    params: sindr_devices::varactor::VaractorParams::default(),
                }),
            ],
            ..Default::default()
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "Varactor DC circuit solve failed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        // n1 = 5V (voltage source), n2 should be very close to 5V (varactor = open circuit = 1e12 Ohm)
        let v_n1 = result.node_voltages["n1"];
        assert_relative_eq!(v_n1, 5.0, epsilon = 1e-6);
        // Varactor result should be present
        let cv1 = result
            .component_results
            .iter()
            .find(|c| c.id == "CV1")
            .unwrap();
        // Voltage across varactor is approximately 5V (n2 ≈ 5V due to 1e12 shunt)
        assert!(
            cv1.voltage_across > 4.9,
            "Varactor voltage should be ~5V, got {}",
            cv1.voltage_across
        );
    }

    /// IGBT cutoff test: Vge=0 (gate at 0V, below threshold) → ids=0 in result.
    #[test]
    fn igbt_cutoff_circuit() {
        use crate::solve_circuit;

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                // Gate at 0V (tied to ground via large resistor effectively at 0V)
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vg".into(),
                    nodes: ["gate".into(), "0".into()],
                    voltage: 0.0,
                    waveform: None,
                }),
                // Collector supply: 12V
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["coll".into(), "0".into()],
                    voltage: 12.0,
                    waveform: None,
                }),
                // Load resistor
                CircuitElement::Resistor(Resistor {
                    id: "Rload".into(),
                    nodes: ["coll".into(), "0".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Igbt(Igbt {
                    id: "T1".into(),
                    nodes: ["gate".into(), "coll".into(), "0".into()],
                    params: sindr_devices::igbt::IgbtParams::default(), // vth=5.0
                }),
            ],
            ..Default::default()
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "IGBT cutoff circuit failed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        let t1 = result
            .component_results
            .iter()
            .find(|c| c.id == "T1")
            .unwrap();
        // With Vge=0 < Vth=5V, IGBT is in cutoff: ids = 0
        assert_relative_eq!(t1.current_through, 0.0, epsilon = 1e-9);
    }

    /// IGBT active test: Vge=10V, Vce=12V → non-zero collector current.
    #[test]
    fn igbt_active_circuit() {
        use crate::solve_circuit;

        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![
                // Gate at 10V (above threshold of 5V)
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vg".into(),
                    nodes: ["gate".into(), "0".into()],
                    voltage: 10.0,
                    waveform: None,
                }),
                // Collector supply
                CircuitElement::VoltageSource(VoltageSource {
                    id: "Vcc".into(),
                    nodes: ["n_vcc".into(), "0".into()],
                    voltage: 12.0,
                    waveform: None,
                }),
                // Load resistor between supply and collector
                CircuitElement::Resistor(Resistor {
                    id: "Rload".into(),
                    nodes: ["n_vcc".into(), "coll".into()],
                    resistance: 100.0,
                }),
                CircuitElement::Igbt(Igbt {
                    id: "T1".into(),
                    nodes: ["gate".into(), "coll".into(), "0".into()],
                    params: sindr_devices::igbt::IgbtParams::default(), // vth=5, k=5
                }),
            ],
            ..Default::default()
        };

        let result = solve_circuit(&circuit);
        assert!(
            result.is_ok(),
            "IGBT active circuit failed: {:?}",
            result.err()
        );
        let result = result.unwrap();
        let t1 = result
            .component_results
            .iter()
            .find(|c| c.id == "T1")
            .unwrap();
        // With Vge=10 > Vth=5, IGBT should conduct
        assert!(
            t1.current_through > 0.0,
            "IGBT should have non-zero collector current, got {}",
            t1.current_through
        );
    }

    /// Physical terminal currents (into base, collector, emitter) of the BJT
    /// model at node voltages `v = [vb, vc, ve]`.
    fn bjt_currents(kind: BjtKind, params: &BjtParams, v: [f64; 3]) -> [f64; 3] {
        let sign = match kind {
            BjtKind::Npn => 1.0,
            BjtKind::Pnp => -1.0,
        };
        let (ib, ic) = sindr_devices::bjt::bjt_terminal_currents(
            sign * (v[0] - v[2]),
            sign * (v[0] - v[1]),
            params,
        );
        [sign * ib, sign * ic, -sign * (ib + ic)]
    }

    /// Every stamped row must be the Newton linearisation of its terminal
    /// current, including past the old 10 S conductance cap and past the
    /// exponential clamp at 40·V_T.
    #[test]
    fn bjt_stamp_is_newton_consistent() {
        let nodes: [NodeId; 3] = ["b".into(), "c".into(), "e".into()];
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: nodes.clone(),
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            })],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let idx: Vec<usize> = nodes.iter().map(|n| node_map.index(n).unwrap()).collect();

        let plain = BjtParams::new(100.0);
        let early = BjtParams {
            vaf: 50.0,
            ..BjtParams::new(100.0)
        };
        // Junction voltages (vbe, vbc) in the device's own polarity.
        let biases = [
            (0.65, -4.0), // forward-active
            (0.80, -4.0), // dIc/dVbe ≈ 10.6 S, past the old cap
            (1.20, -4.0), // past the exp clamp
            (0.75, 0.60), // saturation
            (1.10, 1.05), // both junctions past the clamp
        ];
        for kind in [BjtKind::Npn, BjtKind::Pnp] {
            let sign = if kind == BjtKind::Npn { 1.0 } else { -1.0 };
            for params in [&plain, &early] {
                for (vbe, vbc) in biases {
                    // Put the emitter at 1 V so no terminal is ground.
                    let ve = 1.0;
                    let v = [ve + sign * vbe, ve + sign * (vbe - vbc), ve];
                    let mut x = nalgebra::DVector::zeros(node_map.num_nodes());
                    for (i, &k) in idx.iter().enumerate() {
                        x[k] = v[i];
                    }
                    let mut system = MnaSystem::new(node_map.num_nodes(), 0);
                    stamp_bjt_companion(&mut system, &node_map, &nodes, &x, params, kind);

                    let currents = bjt_currents(kind, params, v);
                    for (row, &r) in idx.iter().enumerate() {
                        let label = format!(
                            "{kind:?} vaf={} vbe={vbe} vbc={vbc} row {}",
                            params.vaf, nodes[row]
                        );
                        let linear = system.a.to_dense().row(r).dot(&x.transpose()) - system.b[r];
                        assert_relative_eq!(
                            linear,
                            currents[row],
                            max_relative = 1e-9,
                            epsilon = 1e-15
                        );
                        for (col, &k) in idx.iter().enumerate() {
                            let h = 1e-6;
                            let (mut up, mut down) = (v, v);
                            up[col] += h;
                            down[col] -= h;
                            let fd = (bjt_currents(kind, params, up)[row]
                                - bjt_currents(kind, params, down)[row])
                                / (2.0 * h);
                            let stamped = system.a[(r, k)];
                            assert!(
                                (stamped - fd).abs() <= 1e-5 * fd.abs().max(1e-9),
                                "{label}: d/dV{} stamped {stamped:e}, finite difference {fd:e}",
                                nodes[col]
                            );
                        }
                    }
                }
            }
        }
    }

    /// Physical drain current (into the drain) of the JFET model. A P-channel
    /// device is the N-channel square law mirrored: Id = −I_N(vs−vg, vs−vd).
    fn jfet_drain_current(kind: JfetKind, vg: f64, vd: f64, vs: f64) -> f64 {
        let n_current = |vgs: f64, vds: f64| {
            let c = sindr_devices::jfet::jfet_companion(vgs, vds, JfetKind::NChannel, 10e-3, -2.0);
            c.gm * vgs + c.gds * vds + c.i_eq
        };
        match kind {
            JfetKind::NChannel => n_current(vg - vs, vd - vs),
            JfetKind::PChannel => -n_current(vs - vg, vs - vd),
        }
    }

    /// The stamped drain row must be the Newton linearisation of the drain
    /// current: its value at the operating point and its partial derivatives.
    #[test]
    fn jfet_stamp_is_newton_consistent() {
        let nodes: [NodeId; 3] = ["g".into(), "d".into(), "s".into()];
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Jfet(Jfet {
                id: "J1".into(),
                nodes: nodes.clone(),
                kind: JfetKind::NChannel,
                idss: 10e-3,
                vp: -2.0,
                parasitic_caps: None,
            })],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let idx = |n: &str| node_map.index(n).unwrap();

        // (kind, vg, vd, vs): saturation and triode for each channel type,
        // forward and reverse.
        let cases = [
            (JfetKind::NChannel, 4.0, 8.0, 5.0),
            (JfetKind::NChannel, 4.0, 5.5, 5.0),
            (JfetKind::NChannel, 4.0, 2.0, 5.0),
            (JfetKind::NChannel, 4.0, 4.5, 5.0),
            (JfetKind::PChannel, 6.0, 2.0, 5.0),
            (JfetKind::PChannel, 6.0, 4.5, 5.0),
            (JfetKind::PChannel, 6.0, 8.0, 5.0),
            (JfetKind::PChannel, 6.0, 5.5, 5.0),
        ];
        for (kind, vg, vd, vs) in cases {
            let vp = match kind {
                JfetKind::NChannel => -2.0,
                JfetKind::PChannel => 2.0,
            };
            let mut v = nalgebra::DVector::zeros(node_map.num_nodes());
            v[idx("g")] = vg;
            v[idx("d")] = vd;
            v[idx("s")] = vs;
            let mut system = MnaSystem::new(node_map.num_nodes(), 0);
            stamp_jfet_companion(&mut system, &node_map, &nodes, kind, 10e-3, vp, &v);

            let row = idx("d");
            let linear_id = system.a.to_dense().row(row).dot(&v.transpose()) - system.b[row];
            let id = jfet_drain_current(kind, vg, vd, vs);
            let label = format!("{kind:?} at vg={vg} vd={vd} vs={vs}");
            assert_relative_eq!(linear_id, id, max_relative = 1e-9, epsilon = 1e-12);

            let h = 1e-6;
            let partials = [
                (
                    "g",
                    (jfet_drain_current(kind, vg + h, vd, vs)
                        - jfet_drain_current(kind, vg - h, vd, vs))
                        / (2.0 * h),
                ),
                (
                    "d",
                    (jfet_drain_current(kind, vg, vd + h, vs)
                        - jfet_drain_current(kind, vg, vd - h, vs))
                        / (2.0 * h),
                ),
                (
                    "s",
                    (jfet_drain_current(kind, vg, vd, vs + h)
                        - jfet_drain_current(kind, vg, vd, vs - h))
                        / (2.0 * h),
                ),
            ];
            for (node, fd) in partials {
                let stamped = system.a[(row, idx(node))];
                assert!(
                    (stamped - fd).abs() <= 1e-6 * fd.abs().max(1e-6),
                    "{label}: dId/dV{node} stamped {stamped:e}, finite difference {fd:e}"
                );
            }
        }
    }

    /// Stamping at `v` and evaluating at `v` gives the device's terminal
    /// currents, so the stamped Jacobian columns must match a finite
    /// difference of those currents. Covers PMOS and NMOS, each also in
    /// reverse mode (drain past the source), where the body tied to the
    /// labelled source is forward-biased against the end acting as one.
    #[test]
    fn mosfet_stamp_is_newton_consistent() {
        use sindr_devices::mosfet::{MosfetKind, MosfetParams};

        let nodes: [NodeId; 3] = ["g".into(), "d".into(), "s".into()];
        let params = MosfetParams {
            vto: 0.7,
            ..MosfetParams::default_nmos()
        };
        // (kind, [vg, vd, vs]) across saturation and triode, forward and
        // reverse.
        let cases = [
            (MosfetKind::Pmos, [2.0, 1.0, 5.0]),
            (MosfetKind::Pmos, [0.0, 4.5, 5.0]),
            (MosfetKind::Pmos, [1.0, 0.0, 3.0]),
            (MosfetKind::Pmos, [1.0, 3.0, 0.0]),
            (MosfetKind::Pmos, [0.0, 3.3, 3.0]),
            (MosfetKind::Nmos, [3.0, 5.0, 0.5]),
            (MosfetKind::Nmos, [2.0, 0.2, -1.0]),
            (MosfetKind::Nmos, [2.0, 0.0, 3.0]),
            (MosfetKind::Nmos, [3.3, 0.0, 0.4]),
        ];
        for (kind, bias) in cases {
            let circuit = Circuit {
                ground_node: "0".into(),
                components: vec![CircuitElement::Mosfet(Mosfet {
                    id: "M1".into(),
                    nodes: nodes.clone(),
                    kind,
                    params: params.clone(),
                    parasitic_caps: None,
                    body_diode: None,
                })],
                ..Default::default()
            };
            let node_map = NodeMap::from_circuit(&circuit);
            let n = node_map.num_nodes();
            let idx: Vec<usize> = nodes.iter().map(|x| node_map.index(x).unwrap()).collect();
            let mut v = DVector::zeros(n);
            for (k, &i) in idx.iter().enumerate() {
                v[i] = bias[k];
            }
            let stamp_at = |v: &DVector<f64>| {
                let mut system = MnaSystem::new(n, 0);
                stamp_mosfet_companion(&mut system, &node_map, &nodes, v, &params, kind);
                system
            };
            let currents = |v: &DVector<f64>| {
                let system = stamp_at(v);
                &system.a.to_dense() * v - &system.b
            };
            let system = stamp_at(&v);
            let h = 1e-6;
            for &col in &idx {
                let mut vp = v.clone();
                let mut vm = v.clone();
                vp[col] += h;
                vm[col] -= h;
                let fd = (currents(&vp) - currents(&vm)) / (2.0 * h);
                for &row in &idx {
                    let stamped = system.a[(row, col)];
                    let tol = 1e-5 * stamped.abs().max(fd[row].abs()) + 1e-12;
                    assert!(
                        (stamped - fd[row]).abs() <= tol,
                        "{kind:?} at {bias:?}: dI[{row}]/dV[{col}] stamped {stamped:e}, finite difference {:e}",
                        fd[row]
                    );
                }
            }
        }
    }

    #[test]
    fn bjt_companion_conductances_are_slopes_of_its_currents() {
        let params = BjtParams::new(100.0);
        let alpha_r = params.br / (params.br + 1.0);
        for (vbe, vbc) in [(0.65, -2.0), (0.85, -2.0), (1.3, -2.0), (0.7, 1.2)] {
            let c = sindr_devices::bjt::bjt_companion(vbe, vbc, &params);
            // Large enough that round-off on kiloamp clamped currents stays
            // well below the slope; truncation is ~h²/(6·V_T²).
            let h = 1e-5;
            let ic = |vbe, vbc| sindr_devices::bjt::bjt_companion(vbe, vbc, &params).ic;
            // ic = i_f − i_r/α_R, so ∂ic/∂vbe = g_be and ∂ic/∂vbc = −g_bc/α_R.
            let dvbe = (ic(vbe + h, vbc) - ic(vbe - h, vbc)) / (2.0 * h);
            let dvbc = (ic(vbe, vbc + h) - ic(vbe, vbc - h)) / (2.0 * h);
            assert_relative_eq!(c.g_be, dvbe, max_relative = 1e-5);
            assert_relative_eq!(-c.g_bc / alpha_r, dvbc, max_relative = 1e-5);
        }
    }

    /// Stamping at `v` and evaluating at `v` gives the IGBT's terminal
    /// currents, so the stamped Jacobian columns (gate included) must match
    /// a finite difference of those currents.
    #[test]
    fn igbt_stamp_is_newton_consistent() {
        use sindr_devices::igbt::IgbtParams;

        let nodes: [NodeId; 3] = ["g".into(), "c".into(), "e".into()];
        let params = IgbtParams::default();
        let circuit = Circuit {
            ground_node: "0".into(),
            components: vec![CircuitElement::Igbt(Igbt {
                id: "Q1".into(),
                nodes: nodes.clone(),
                params: params.clone(),
            })],
            ..Default::default()
        };
        let node_map = NodeMap::from_circuit(&circuit);
        let n = node_map.num_nodes();
        let idx: Vec<usize> = nodes.iter().map(|x| node_map.index(x).unwrap()).collect();
        // [vg, vc, ve]: saturation, triode, emitter off ground, cutoff.
        for bias in [
            [8.0, 10.0, 0.0],
            [10.0, 2.0, 0.0],
            [12.0, 4.0, 1.0],
            [3.0, 10.0, 0.0],
        ] {
            let mut v = DVector::zeros(n);
            for (k, &i) in idx.iter().enumerate() {
                v[i] = bias[k];
            }
            let stamp_at = |v: &DVector<f64>| {
                let mut system = MnaSystem::new(n, 0);
                stamp_igbt_companion(&mut system, &node_map, &nodes, v, &params);
                system
            };
            let currents = |v: &DVector<f64>| {
                let system = stamp_at(v);
                &system.a.to_dense() * v - &system.b
            };
            let system = stamp_at(&v);
            let h = 1e-6;
            for &col in &idx {
                let mut vp = v.clone();
                let mut vm = v.clone();
                vp[col] += h;
                vm[col] -= h;
                let fd = (currents(&vp) - currents(&vm)) / (2.0 * h);
                for &row in &idx {
                    let stamped = system.a[(row, col)];
                    let tol = 1e-5 * stamped.abs().max(fd[row].abs()) + 1e-9;
                    assert!(
                        (stamped - fd[row]).abs() <= tol,
                        "at {bias:?}: dI[{row}]/dV[{col}] stamped {stamped:e}, finite difference {:e}",
                        fd[row]
                    );
                }
            }
        }
    }

    /// The linear pass (`v_prev = None`) stamps nothing for a nonlinear
    /// device or a capacitor (open at DC) and leaves an inductor to the
    /// caller, which shorts it with a 0 V source first: the system is the
    /// resistors' alone. A varactor is a 1 TΩ shunt, so its node keeps a
    /// DC path.
    #[test]
    fn linear_pass_stamps_only_the_linear_parts() {
        let r = |id: &str, a: &str| {
            CircuitElement::Resistor(Resistor {
                id: id.into(),
                nodes: [a.into(), "0".into()],
                resistance: 1e3,
            })
        };
        let resistors = vec![r("RA", "a"), r("RB", "b"), r("RC", "c")];
        let two = || -> [NodeId; 2] { ["a".into(), "b".into()] };
        let three = || -> [NodeId; 3] { ["a".into(), "b".into(), "c".into()] };
        let nonlinear = vec![
            CircuitElement::Diode(Diode {
                id: "D1".into(),
                nodes: two(),
                temperature: 300.15,
                model: None,
            }),
            CircuitElement::Led(Led {
                id: "LED1".into(),
                nodes: two(),
                color: "red".into(),
                temperature: 300.15,
            }),
            CircuitElement::ZenerDiode(ZenerDiode {
                id: "DZ1".into(),
                nodes: two(),
                vz: 5.1,
                temperature: 300.15,
            }),
            CircuitElement::SchottkyDiode(SchottkyDiode {
                id: "DS1".into(),
                nodes: two(),
                temperature: 300.15,
            }),
            CircuitElement::Photodiode(Photodiode {
                id: "PD1".into(),
                nodes: two(),
                irradiance: 1.0,
                temperature: 300.15,
            }),
            CircuitElement::Bjt(Bjt {
                id: "Q1".into(),
                nodes: three(),
                kind: BjtKind::Npn,
                bf: 100.0,
                temperature: 300.15,
                parasitic_caps: None,
                model: None,
            }),
            CircuitElement::Mosfet(Mosfet {
                id: "M1".into(),
                nodes: three(),
                kind: MosfetKind::Nmos,
                params: MosfetParams::default_nmos(),
                parasitic_caps: None,
                body_diode: None,
            }),
            CircuitElement::Igbt(Igbt {
                id: "Z1".into(),
                nodes: three(),
                params: sindr_devices::igbt::IgbtParams::default(),
            }),
            CircuitElement::Capacitor(Capacitor {
                id: "C1".into(),
                nodes: two(),
                capacitance: 1e-6,
            }),
            CircuitElement::Inductor(Inductor {
                id: "L1".into(),
                nodes: two(),
                inductance: 1e-3,
                saturation: None,
            }),
        ];
        let full = Circuit::new("0", [resistors.clone(), nonlinear].concat());
        let node_map = NodeMap::from_circuit(&full);
        let stamped = |circuit: &Circuit| {
            let mut system = MnaSystem::new(node_map.num_nodes(), 0);
            stamp_circuit(circuit, &mut system, &node_map, None).unwrap();
            system
        };
        let all = stamped(&full);
        let only_r = stamped(&Circuit::new("0", resistors.clone()));
        for i in 0..all.size() {
            assert_eq!(all.b[i], only_r.b[i], "b[{i}]");
            for j in 0..all.size() {
                assert_eq!(all.a[(i, j)], only_r.a[(i, j)], "A[{i},{j}]");
            }
        }

        let varactor = CircuitElement::Varactor(Varactor {
            id: "CV1".into(),
            nodes: two(),
            params: sindr_devices::varactor::VaractorParams::default(),
        });
        let with_varactor = stamped(&Circuit::new("0", [resistors, vec![varactor]].concat()));
        let (a, b) = (node_map.index("a").unwrap(), node_map.index("b").unwrap());
        assert_relative_eq!(
            with_varactor.a[(a, a)] - only_r.a[(a, a)],
            1e-12,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            with_varactor.a[(a, b)] - only_r.a[(a, b)],
            -1e-12,
            max_relative = 1e-12
        );
    }

    /// The elements the analyses lower to their parts refuse a direct
    /// stamp, and a zener stamps the diode companion with its breakdown.
    #[test]
    fn direct_stamps_of_lowered_elements() {
        let nodes = || -> [NodeId; 2] { ["a".into(), "0".into()] };
        let crystal = CircuitElement::Crystal(Crystal {
            id: "Y1".into(),
            nodes: nodes(),
            params: crate::circuit::CrystalParams::from_datasheet(1e6, 50.0, 5e-12),
        });
        let bead = CircuitElement::FerriteBead(FerriteBead {
            id: "FB1".into(),
            nodes: nodes(),
            params: crate::circuit::FerriteBeadParams::from_datasheet(600.0, 0.1),
        });
        for element in [crystal, bead] {
            let circuit = Circuit::new("0", vec![element]);
            let node_map = NodeMap::from_circuit(&circuit);
            let mut system = MnaSystem::new(node_map.num_nodes(), 0);
            let err = stamp_circuit(&circuit, &mut system, &node_map, None).unwrap_err();
            assert!(err.to_string().contains("stamped as its parts"), "{err}");
        }

        let zener = Circuit::new(
            "0",
            vec![CircuitElement::ZenerDiode(ZenerDiode {
                id: "Z1".into(),
                nodes: nodes(),
                vz: 5.1,
                temperature: 300.15,
            })],
        );
        let node_map = NodeMap::from_circuit(&zener);
        let v = DVector::from_element(1, -5.2);
        let mut system = MnaSystem::new(1, 0);
        stamp_circuit(&zener, &mut system, &node_map, Some(&v)).unwrap();
        let mut expected = MnaSystem::new(1, 0);
        let params = crate::device_params::zener_diode(5.1, 300.15);
        stamp_diode_companion(&mut expected, &node_map, &nodes(), &v, &params);
        assert_eq!(system.a[(0, 0)], expected.a[(0, 0)]);
        assert_eq!(system.b[0], expected.b[0]);
        assert!(
            system.a[(0, 0)] > 1e-3,
            "in breakdown: {}",
            system.a[(0, 0)]
        );
    }
}
