//! Pole-zero analysis, as SPICE's `.pz`.
//!
//! Linearises the circuit at its DC operating point, writes the small-signal
//! system as the matrix pencil `G + s·C`, and finds the poles and zeros of
//! the transfer function from an input node pair to an output node pair as
//! its generalised eigenvalues.
//!
//! # Method
//!
//! The poles are the roots of `det(G + s·C) = 0`, the circuit with its
//! input zeroed: shorted for a voltage gain, open for a transimpedance.
//! The zeros of `H(s) = cᵀ·(G + s·C)⁻¹·b` (input vector `b`, output vector
//! `c`) are the roots of the bordered pencil
//!
//! ```text
//!   det ⎡G + s·C   b⎤ = 0
//!       ⎣  cᵀ      0⎦
//! ```
//!
//! An MNA pencil has `C` singular (every resistor-only node, every source
//! branch), so it has infinite eigenvalues, often in Jordan blocks (a
//! capacitor across a source, an inductor in series with a current source).
//! Handed straight to QZ these perturb into large spurious finite
//! eigenvalues, so they are deflated first, exactly, with orthogonal
//! transformations: while `C` is singular, an SVD splits off the equations
//! with no `s` in them, and the unknowns are restricted to the null space
//! of those equations. Each step leaves a smaller pencil with the same
//! finite eigenvalues. Once `C` has full rank, QZ (faer's real generalised
//! eigensolver) gives the rest. Before any of this, `C` is scaled by
//! `ω₀ = ‖G‖/‖C‖` and both are row- and column-equilibrated, which changes
//! no eigenvalue but brings the entries to the same order.
//!
//! # Numerical envelope
//!
//! QZ is backward stable, so a well-separated pole or zero comes out to a
//! relative error of about `n·ε·κ`, with `κ` its condition number: 1e-12 or
//! better on passive networks and op-amp loops of a few dozen unknowns.
//! A repeated root of multiplicity `k` is only accurate to about `ε^(1/k)`
//! (a double root to ~1e-8), as with any eigensolver. The rank decisions
//! use a threshold of `64·n·ε` relative to the equilibrated pencil, so a
//! root more than about 1e13 times `ω₀` is treated as infinite and dropped.
//! Poles and zeros that cancel are both reported, as SPICE does.

use nalgebra::{DMatrix, DVector};
use num_complex::Complex64;

use crate::ac_analysis::{
    at_junction_temperatures, bias_point_with_result, incremental_inductors, linearize_at_dc,
    small_signal_pencil,
};
use crate::circuit::{Circuit, CircuitElement};
use crate::error::SimError;
use crate::node_map::NodeMap;

/// The transfer function a pole-zero analysis is of: SPICE's `vol` or
/// `cur`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum PzTransfer {
    /// `vol`: output voltage over the voltage across the input pair (V/V).
    /// The poles are those of the circuit with the input pair shorted.
    VoltageGain,
    /// `cur`: output voltage over a current driven into the input pair (Ω).
    /// The poles are those of the circuit with the input pair open.
    Transimpedance,
}

/// What a pole-zero analysis finds: SPICE's `pol`, `zer` or `pz`.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum PzFind {
    /// `pol`: poles only.
    Poles,
    /// `zer`: zeros only.
    Zeros,
    /// `pz`: both.
    Both,
}

/// Pole-zero analysis configuration: SPICE's
/// `.pz input input_reference output output_reference vol|cur pol|zer|pz`.
/// Build one with [`PzConfig::voltage_gain`] or
/// [`PzConfig::transimpedance`].
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct PzConfig {
    /// Positive input node.
    pub input: String,
    /// Negative input node (the ground node for a single-ended input).
    pub input_reference: String,
    /// Positive output node.
    pub output: String,
    /// Negative output node (the ground node for a single-ended output).
    pub output_reference: String,
    /// Voltage gain or transimpedance.
    pub transfer: PzTransfer,
    /// Poles, zeros or both. Default both.
    pub find: PzFind,
}

impl PzConfig {
    /// The voltage gain `V(output, output_reference) / V(input,
    /// input_reference)`, as SPICE's `.pz … vol pz`. When an independent
    /// voltage source sits across the input pair it is the input;
    /// otherwise the analysis drives the pair with one of its own.
    ///
    /// ```
    /// use sindr::pole_zero::{PzConfig, PzFind, PzTransfer};
    ///
    /// // SPICE `.pz in 0 out 0 vol pol`.
    /// let config = PzConfig::voltage_gain("in", "0", "out", "0").with_find(PzFind::Poles);
    /// assert_eq!(config.transfer, PzTransfer::VoltageGain);
    /// ```
    pub fn voltage_gain(
        input: impl Into<String>,
        input_reference: impl Into<String>,
        output: impl Into<String>,
        output_reference: impl Into<String>,
    ) -> Self {
        Self {
            input: input.into(),
            input_reference: input_reference.into(),
            output: output.into(),
            output_reference: output_reference.into(),
            transfer: PzTransfer::VoltageGain,
            find: PzFind::Both,
        }
    }

    /// The transimpedance `V(output, output_reference) / I`, with `I` a
    /// current driven into `input` and out of `input_reference`, as SPICE's
    /// `.pz … cur pz`.
    pub fn transimpedance(
        input: impl Into<String>,
        input_reference: impl Into<String>,
        output: impl Into<String>,
        output_reference: impl Into<String>,
    ) -> Self {
        Self {
            transfer: PzTransfer::Transimpedance,
            ..Self::voltage_gain(input, input_reference, output, output_reference)
        }
    }

    /// Finds `find`: poles, zeros or both.
    pub fn with_find(mut self, find: PzFind) -> Self {
        self.find = find;
        self
    }
}

/// Result of a pole-zero analysis.
///
/// Poles and zeros are in rad/s (divide by 2π for Hz), as SPICE prints
/// them, sorted by magnitude and then by imaginary part. Complex roots come
/// in conjugate pairs; a stable circuit's poles all have negative real
/// parts. Only finite roots are listed.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct PzResult {
    /// The poles (rad/s); empty when only zeros were asked for.
    pub poles: Vec<Complex64>,
    /// The zeros (rad/s); empty when only poles were asked for.
    pub zeros: Vec<Complex64>,
    /// Configuration used to produce this result.
    pub config: PzConfig,
}

impl PzResult {
    /// Whether every pole is in the open left half-plane.
    pub fn is_stable(&self) -> bool {
        self.poles.iter().all(|p| p.re < 0.0)
    }
}

/// Runs a pole-zero analysis, as SPICE's `.pz`.
///
/// # Algorithm
///
/// 1. Solve the DC operating point and linearise every nonlinear element
///    at it, as [`solve_ac`](crate::ac_analysis::solve_ac) does.
/// 2. Write the small-signal system as `G + s·C` (inductors, a battery's
///    RC pair and a diode's charge behind its RS get an extra unknown each,
///    so every stamp is affine in `s`).
/// 3. Find the poles and zeros as the finite generalised eigenvalues of
///    that pencil and of its bordered form; see the [module
///    documentation](self) for the method and its accuracy.
///
/// # Errors
///
/// - [`SimError::InvalidComponent`] if a node is not in the circuit, a
///   pair names the same node twice, or (for zeros) the output does not
///   respond to the input at all, such as a transimpedance whose input is
///   shorted by a voltage source. Node names are checked before the DC
///   solve
/// - Any [`SimError`] from the DC operating-point solve
/// - [`SimError::SingularMatrix`] if the linearised circuit is singular at
///   every frequency (a node with no small-signal path anywhere)
/// - [`SimError::InvalidSolution`] in the rare case that the eigensolver
///   does not converge
///
/// # Example
///
/// ```
/// use sindr::{Circuit, CircuitElement};
/// use sindr::pole_zero::{solve_pz, PzConfig};
///
/// // RC low-pass, 1 kΩ and 1 µF: one pole at −1/RC = −1000 rad/s.
/// let circuit = Circuit {
///     ground_node: "0".into(),
///     components: vec![
///         CircuitElement::VoltageSource {
///             id: "V1".into(),
///             nodes: ["in".into(), "0".into()],
///             voltage: 0.0,
///             waveform: None,
///         },
///         CircuitElement::Resistor {
///             id: "R1".into(),
///             nodes: ["in".into(), "out".into()],
///             resistance: 1_000.0,
///         },
///         CircuitElement::Capacitor {
///             id: "C1".into(),
///             nodes: ["out".into(), "0".into()],
///             capacitance: 1e-6,
///         },
///     ],
///     ..Default::default()
/// };
///
/// let pz = solve_pz(&circuit, &PzConfig::voltage_gain("in", "0", "out", "0"))?;
/// assert_eq!(pz.poles.len(), 1);
/// assert!((pz.poles[0].re + 1_000.0).abs() < 1e-9);
/// assert!(pz.zeros.is_empty());
/// # Ok::<(), sindr::SimError>(())
/// ```
pub fn solve_pz(circuit: &Circuit, config: &PzConfig) -> Result<PzResult, SimError> {
    solve_pz_with(circuit, config, &crate::SolverOptions::default())
}

/// [`solve_pz`] with `opts`.
pub(crate) fn solve_pz_with(
    circuit: &Circuit,
    config: &PzConfig,
    opts: &crate::SolverOptions,
) -> Result<PzResult, SimError> {
    let flat = crate::solver_circuit(circuit)?;
    let circuit: &Circuit = &flat;
    let ports = Ports::of(circuit, config)?;

    let (node_map, dc_solution, dc) = bias_point_with_result(circuit, opts)?;
    let circuit = &incremental_inductors(&at_junction_temperatures(circuit, &dc), &dc);
    let ss_models = linearize_at_dc(circuit, &node_map, &dc_solution);
    let pencil = small_signal_pencil(circuit, &node_map, &dc_solution, &ss_models)?;
    let (mut g, mut c) = (pencil.g, pencil.c);

    // The input vector b, after shorting a voltage input with a source of
    // our own unless one is already there.
    let b = match config.transfer {
        PzTransfer::VoltageGain => {
            let existing = circuit.components.iter().find_map(|e| match e {
                CircuitElement::VoltageSource { id, nodes, .. }
                    if same_pair(nodes, &config.input, &config.input_reference) =>
                {
                    pencil.vsource_branches.get(id).copied()
                }
                _ => None,
            });
            let k = match existing {
                Some(k) => k,
                None => {
                    let k = g.nrows();
                    (g, c) = (border(&g, None, None), border(&c, None, None));
                    for (node, sign) in [(ports.input, 1.0), (ports.input_reference, -1.0)] {
                        if let Some(i) = node {
                            g[(i, k)] += sign;
                            g[(k, i)] += sign;
                        }
                    }
                    k
                }
            };
            unit(g.nrows(), Some(k), None)
        }
        PzTransfer::Transimpedance => unit(g.nrows(), ports.input, ports.input_reference),
    };
    let out = unit(g.nrows(), ports.output, ports.output_reference);

    let poles = if config.find == PzFind::Zeros {
        Vec::new()
    } else {
        pencil_roots(&g, &c).map_err(|e| match e {
            PencilError::Singular => SimError::SingularMatrix,
            PencilError::NoConvergence => qz_failure(),
        })?
    };
    let zeros = if config.find == PzFind::Poles {
        Vec::new()
    } else {
        let gb = border(&g, Some(&b), Some(&out));
        let cb = border(&c, None, None);
        pencil_roots(&gb, &cb).map_err(|e| match e {
            PencilError::Singular => SimError::InvalidComponent(format!(
                "pole-zero: V({}, {}) does not respond to the input at ({}, {}), so the \
                 transfer function has no zeros",
                config.output, config.output_reference, config.input, config.input_reference
            )),
            PencilError::NoConvergence => qz_failure(),
        })?
    };

    Ok(PzResult {
        poles,
        zeros,
        config: config.clone(),
    })
}

/// The eigensolver didn't converge: only on a pencil far outside the
/// range sindr's values make.
fn qz_failure() -> SimError {
    SimError::InvalidSolution
}

/// Whether the two terminals `nodes` are the pair `(p, q)`, either way
/// round.
fn same_pair(nodes: &[String; 2], p: &str, q: &str) -> bool {
    (nodes[0] == p && nodes[1] == q) || (nodes[0] == q && nodes[1] == p)
}

/// The matrix rows of a configuration's four nodes; `None` is ground.
struct Ports {
    input: Option<usize>,
    input_reference: Option<usize>,
    output: Option<usize>,
    output_reference: Option<usize>,
}

impl Ports {
    fn of(circuit: &Circuit, config: &PzConfig) -> Result<Self, SimError> {
        let node_map = NodeMap::from_circuit(circuit);
        let index = |name: &str| match node_map.index(name) {
            None if name != circuit.ground_node => Err(SimError::InvalidComponent(format!(
                "pole-zero: node `{name}` is not in the circuit"
            ))),
            index => Ok(index),
        };
        for (p, q, what) in [
            (&config.input, &config.input_reference, "input"),
            (&config.output, &config.output_reference, "output"),
        ] {
            if p == q {
                return Err(SimError::InvalidComponent(format!(
                    "pole-zero: the {what} pair names `{p}` twice"
                )));
            }
        }
        Ok(Self {
            input: index(&config.input)?,
            input_reference: index(&config.input_reference)?,
            output: index(&config.output)?,
            output_reference: index(&config.output_reference)?,
        })
    }
}

/// `+1` at `p`, `−1` at `q`, in a vector of length `n`.
fn unit(n: usize, p: Option<usize>, q: Option<usize>) -> DVector<f64> {
    let mut v = DVector::zeros(n);
    if let Some(p) = p {
        v[p] += 1.0;
    }
    if let Some(q) = q {
        v[q] -= 1.0;
    }
    v
}

/// `m` with one more row and column: `column` down the new column, `row`
/// along the new row, zero where they're `None` and in the corner.
fn border(
    m: &DMatrix<f64>,
    column: Option<&DVector<f64>>,
    row: Option<&DVector<f64>>,
) -> DMatrix<f64> {
    let n = m.nrows();
    let mut out = DMatrix::zeros(n + 1, n + 1);
    out.view_mut((0, 0), (n, n)).copy_from(m);
    if let Some(b) = column {
        out.view_mut((0, n), (n, 1)).copy_from(b);
    }
    if let Some(c) = row {
        out.view_mut((n, 0), (1, n)).copy_from(&c.transpose());
    }
    out
}

/// Why [`pencil_roots`] found no roots.
#[derive(Debug, PartialEq)]
pub(crate) enum PencilError {
    /// `det(G + s·C)` is zero for every `s`.
    Singular,
    /// QZ did not converge.
    NoConvergence,
}

/// The finite roots of `det(G + s·C) = 0`, sorted by magnitude and then by
/// imaginary part. See the module documentation for the method.
pub(crate) fn pencil_roots(
    g: &DMatrix<f64>,
    c: &DMatrix<f64>,
) -> Result<Vec<Complex64>, PencilError> {
    let n = g.nrows();
    if n == 0 {
        return Ok(Vec::new());
    }
    // Frequency scaling: s = ω₀·σ with C·ω₀ the size of G.
    let (gn, cn) = (g.norm(), c.norm());
    let omega0 = if gn > 0.0 && cn > 0.0 { gn / cn } else { 1.0 };
    let (mut g, mut c) = equilibrate(g.clone(), c * omega0);
    let tol = 64.0 * n as f64 * f64::EPSILON * g.norm().max(c.norm());

    // Deflate the infinite eigenvalues: each pass removes the equations
    // without `s` and the unknowns they pin down.
    loop {
        let m = g.nrows();
        if m == 0 {
            return Ok(Vec::new());
        }
        let svd = c.clone().svd(true, true);
        let (Some(u), Some(v_t)) = (svd.u, svd.v_t) else {
            return Err(PencilError::NoConvergence);
        };
        let sv = &svd.singular_values;
        let dynamic: Vec<usize> = (0..m).filter(|&i| sv[i] > tol).collect();
        let algebraic: Vec<usize> = (0..m).filter(|&i| sv[i] <= tol).collect();
        if algebraic.is_empty() {
            break;
        }
        let v = v_t.transpose();
        let g_hat = u.transpose() * &g * &v;
        let c_hat = u.transpose() * &c * &v;
        // The equations with no `s`: N·x = 0. They must be independent,
        // or the pencil is singular.
        let mut padded = DMatrix::zeros(m, m);
        for (r, &i) in algebraic.iter().enumerate() {
            padded.row_mut(r).copy_from(&g_hat.row(i));
        }
        let svd_n = padded.svd(false, true);
        let Some(vn_t) = svd_n.v_t else {
            return Err(PencilError::NoConvergence);
        };
        let null: Vec<usize> = (0..m)
            .filter(|&i| svd_n.singular_values[i] <= tol)
            .collect();
        if null.len() != dynamic.len() {
            return Err(PencilError::Singular);
        }
        let z = DMatrix::from_fn(m, null.len(), |r, k| vn_t[(null[k], r)]);
        let rows = |a: &DMatrix<f64>| DMatrix::from_fn(dynamic.len(), m, |r, j| a[(dynamic[r], j)]);
        g = rows(&g_hat) * &z;
        c = rows(&c_hat) * &z;
    }

    // Complex QZ: every eigenvalue gets its own (α, β), with no 2×2
    // blocks to pair up.
    let m = g.nrows();
    let cplx = |x: &DMatrix<f64>| {
        faer::Mat::<faer::c64>::from_fn(m, m, |i, j| faer::c64::new(x[(i, j)], 0.0))
    };
    let (mut a, mut b) = (cplx(&g), cplx(&c));
    let mut alpha = faer::diag::Diag::<faer::c64>::zeros(m);
    let mut beta = faer::diag::Diag::<faer::c64>::zeros(m);
    let no = faer::linalg::evd::ComputeEigenvectors::No;
    let par = faer::Par::Seq;
    let mut buffer = faer::dyn_stack::MemBuffer::new(
        faer::linalg::gevd::gevd_scratch::<faer::c64>(m, no, no, par, Default::default()),
    );
    faer::linalg::gevd::gevd_cplx(
        a.as_mut(),
        b.as_mut(),
        alpha.as_mut(),
        beta.as_mut(),
        None,
        None,
        par,
        faer::dyn_stack::MemStack::new(&mut buffer),
        Default::default(),
    )
    .map_err(|_| PencilError::NoConvergence)?;
    let (alpha, beta) = (alpha.column_vector(), beta.column_vector());
    let mut roots: Vec<Complex64> = (0..m)
        .filter_map(|i| {
            let a = Complex64::new(alpha[i].re, alpha[i].im);
            let b = Complex64::new(beta[i].re, beta[i].im);
            // C has full rank here, so β is not small unless the root is
            // past what the rank threshold could tell from infinite.
            (b.norm() > tol * a.norm()).then(|| -(a / b) * omega0)
        })
        .filter(|s| s.is_finite())
        .collect();
    conjugate_symmetric(&mut roots, tol);
    roots.sort_by(|x, y| x.norm().total_cmp(&y.norm()).then(x.im.total_cmp(&y.im)));
    Ok(roots)
}

/// Makes the roots of a real pencil exactly conjugate-symmetric, as they
/// are in exact arithmetic: a root within `tol` (relative) of the real axis
/// is real, and each other root is paired with its nearest conjugate and
/// the pair averaged.
fn conjugate_symmetric(roots: &mut [Complex64], tol: f64) {
    let mut paired = vec![false; roots.len()];
    for i in 0..roots.len() {
        if paired[i] {
            continue;
        }
        let z = roots[i];
        if z.im.abs() <= tol * z.norm() {
            roots[i].im = 0.0;
            paired[i] = true;
            continue;
        }
        let partner = (0..roots.len())
            .filter(|&j| j != i && !paired[j] && roots[j].im * z.im < 0.0)
            .min_by(|&j, &k| {
                (roots[j] - z.conj())
                    .norm()
                    .total_cmp(&(roots[k] - z.conj()).norm())
            });
        if let Some(j) = partner {
            let mean = (z + roots[j].conj()) / 2.0;
            roots[i] = mean;
            roots[j] = mean.conj();
            paired[j] = true;
        }
        paired[i] = true;
    }
}

/// `(D_r·G·D_c, D_r·C·D_c)` with power-of-two diagonal scalings that bring
/// the largest entry of every row and column of the pair near 1. The
/// pencil's eigenvalues don't change.
fn equilibrate(mut g: DMatrix<f64>, mut c: DMatrix<f64>) -> (DMatrix<f64>, DMatrix<f64>) {
    let n = g.nrows();
    let pow2 = |x: f64| 2f64.powi(-(x.log2() / 2.0).round() as i32);
    for _ in 0..8 {
        for i in 0..n {
            let r = (0..n)
                .map(|j| g[(i, j)].abs().max(c[(i, j)].abs()))
                .fold(0.0, f64::max);
            if r > 0.0 {
                let f = pow2(r);
                g.row_mut(i).scale_mut(f);
                c.row_mut(i).scale_mut(f);
            }
        }
        for j in 0..n {
            let r = (0..n)
                .map(|i| g[(i, j)].abs().max(c[(i, j)].abs()))
                .fold(0.0, f64::max);
            if r > 0.0 {
                let f = pow2(r);
                g.column_mut(j).scale_mut(f);
                c.column_mut(j).scale_mut(f);
            }
        }
    }
    (g, c)
}
