//! Modified Nodal Analysis (MNA) matrix system.
//!
//! Holds the `(n+m) × (n+m)` MNA matrix and right-hand-side vector that every
//! analysis path eventually solves, where `n` is the number of non-ground
//! nodes and `m` is the number of branch-current unknowns (independent
//! voltage sources, controlled sources that introduce a branch, etc.).
//!
//! Components write themselves into this system via `stamp`;
//! the linear DC, Newton–Raphson, and transient pipelines all build an
//! [`MnaSystem`], stamp into it, and call [`MnaSystem::solve`].
//!
//! The matrix is stored densely for small systems and as a sparse list of
//! entries above [`DENSE_MAX`] unknowns, where it is factorised with a sparse
//! LU. A circuit's sparsity pattern is the same from one Newton iteration or
//! timestep to the next, so the sparse LU's symbolic analysis is reused while
//! the pattern doesn't change.
//!
//! Before a sparse factorisation, every unknown the system fixes on its own
//! is substituted out: a grounded voltage source's node, whose branch row
//! reads `v = V`, together with the source's branch current, found
//! afterwards from the node's row. A supply rail is a dense row and column,
//! and left in it fills the factor in; taken out, a board with one shared
//! rail factorises as cheaply as one with a supply per part. The rearrangement
//! is exact.

use std::cell::RefCell;
use std::collections::HashMap;
use std::ops::{Index, IndexMut};

use faer::linalg::solvers::Solve;
use faer::sparse::linalg::solvers::{Lu, SymbolicLu};
use faer::sparse::{SparseColMat, Triplet};
use nalgebra::{DMatrix, DVector};
use num_complex::Complex64;

use crate::error::SimError;

/// Largest system (unknowns) stored and solved densely. Above it the matrix
/// is sparse. From `benches/mna_solve.rs`, the crossover depends on the
/// circuit's shape: a ladder (three entries a row) goes sparse-faster near
/// 75 unknowns (41 µs vs 73 µs at 100), while a mesh (five a row) is still
/// faster dense at 100 (71 µs vs 85 µs). Below 60 dense wins on both by
/// 1.5–5×, and sparse is 50–250× faster at 1 000. 100 keeps mesh-like
/// circuits dense and costs a ladder tens of µs a solve.
pub const DENSE_MAX: usize = 100;

/// A scalar the MNA matrix can hold: `f64` for DC and transient,
/// [`Complex64`] for AC.
pub trait MnaScalar:
    nalgebra::ComplexField<RealField = f64> + faer::traits::ComplexField + Copy
{
    /// Whether the value is finite (both parts, for a complex number).
    fn is_finite_value(&self) -> bool;
}

impl MnaScalar for f64 {
    fn is_finite_value(&self) -> bool {
        self.is_finite()
    }
}

impl MnaScalar for Complex64 {
    fn is_finite_value(&self) -> bool {
        self.re.is_finite() && self.im.is_finite()
    }
}

/// The `size × size` MNA matrix: indexed as `a[(row, col)]` and written with
/// `+=` by the stamps, whatever its storage.
#[derive(Debug, Clone)]
pub struct MnaMatrix<T: MnaScalar = f64> {
    size: usize,
    storage: Storage<T>,
    /// Returned when reading an entry a sparse matrix doesn't hold.
    zero: T,
}

#[derive(Debug, Clone)]
enum Storage<T: MnaScalar> {
    Dense(DMatrix<T>),
    /// Entries in the order first written, with an index from position.
    Sparse {
        entries: Vec<(usize, usize, T)>,
        index: HashMap<(usize, usize), usize>,
    },
}

impl<T: MnaScalar> MnaMatrix<T> {
    /// A zero `size × size` matrix, dense if `size` ≤ [`DENSE_MAX`].
    pub fn zeros(size: usize) -> Self {
        if size <= DENSE_MAX {
            Self::dense(size)
        } else {
            Self::sparse(size)
        }
    }

    /// A zero `size × size` matrix stored densely.
    pub fn dense(size: usize) -> Self {
        Self {
            size,
            storage: Storage::Dense(DMatrix::from_element(size, size, T::zero())),
            zero: T::zero(),
        }
    }

    /// A zero `size × size` matrix stored sparsely.
    pub fn sparse(size: usize) -> Self {
        Self {
            size,
            storage: Storage::Sparse {
                entries: Vec::new(),
                index: HashMap::new(),
            },
            zero: T::zero(),
        }
    }

    /// Number of rows (and columns).
    pub fn size(&self) -> usize {
        self.size
    }

    /// The matrix as a dense nalgebra matrix.
    pub fn to_dense(&self) -> DMatrix<T> {
        match &self.storage {
            Storage::Dense(m) => m.clone(),
            Storage::Sparse { entries, .. } => {
                let mut m = DMatrix::from_element(self.size, self.size, T::zero());
                for &(r, c, v) in entries {
                    m[(r, c)] += v;
                }
                m
            }
        }
    }

    /// The transpose of the matrix, in the same storage.
    pub fn transpose(&self) -> Self {
        let storage = match &self.storage {
            Storage::Dense(m) => Storage::Dense(m.transpose()),
            Storage::Sparse { entries, .. } => {
                let entries: Vec<_> = entries.iter().map(|&(r, c, v)| (c, r, v)).collect();
                let index = entries
                    .iter()
                    .enumerate()
                    .map(|(k, &(r, c, _))| ((r, c), k))
                    .collect();
                Storage::Sparse { entries, index }
            }
        };
        Self {
            size: self.size,
            storage,
            zero: T::zero(),
        }
    }

    /// Solve `self · x = b`.
    ///
    /// # Errors
    ///
    /// [`SimError::SingularMatrix`] if the matrix is singular, or
    /// [`SimError::InvalidSolution`] if the solution contains NaN or Inf.
    pub fn solve(&self, b: &DVector<T>) -> Result<DVector<T>, SimError> {
        // A circuit with nothing but ground (an element from ground to
        // ground) has no unknowns; nalgebra's LU can't take a 0 × 0 matrix.
        if self.size == 0 {
            return Ok(DVector::from_element(0, T::zero()));
        }
        let solution = match &self.storage {
            Storage::Dense(m) => m.clone().lu().solve(b).ok_or(SimError::SingularMatrix)?,
            Storage::Sparse { entries, .. } => solve_sparse(self.size, entries, b)?,
        };
        if solution.iter().all(MnaScalar::is_finite_value) {
            Ok(solution)
        } else {
            Err(SimError::InvalidSolution)
        }
    }
}

impl<T: MnaScalar> Index<(usize, usize)> for MnaMatrix<T> {
    type Output = T;

    fn index(&self, (r, c): (usize, usize)) -> &T {
        match &self.storage {
            Storage::Dense(m) => &m[(r, c)],
            Storage::Sparse { entries, index } => {
                index.get(&(r, c)).map_or(&self.zero, |&k| &entries[k].2)
            }
        }
    }
}

impl<T: MnaScalar> IndexMut<(usize, usize)> for MnaMatrix<T> {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut T {
        assert!(r < self.size && c < self.size, "MNA index out of range");
        match &mut self.storage {
            Storage::Dense(m) => &mut m[(r, c)],
            Storage::Sparse { entries, index } => {
                let k = *index.entry((r, c)).or_insert_with(|| {
                    entries.push((r, c, T::zero()));
                    entries.len() - 1
                });
                &mut entries[k].2
            }
        }
    }
}

/// A cached sparse symbolic LU and the pattern (column pointers, row
/// indices) it was computed for.
type CachedSymbolic = (Vec<usize>, Vec<usize>, SymbolicLu<usize>);

thread_local! {
    /// The last sparse symbolic LU computed on this thread.
    static SYMBOLIC: RefCell<Option<CachedSymbolic>> = const { RefCell::new(None) };
}

fn solve_sparse<T: MnaScalar>(
    size: usize,
    entries: &[(usize, usize, T)],
    b: &DVector<T>,
) -> Result<DVector<T>, SimError> {
    let fixed = fixed_unknowns(size, entries);
    if fixed.is_empty() {
        return factor_and_solve(&to_csc(size, entries.iter().copied())?, b);
    }
    solve_with_fixed(size, entries, &fixed, b)
}

fn to_csc<T: MnaScalar>(
    size: usize,
    entries: impl Iterator<Item = (usize, usize, T)>,
) -> Result<SparseColMat<usize, T>, SimError> {
    let triplets: Vec<Triplet<usize, usize, T>> =
        entries.map(|(r, c, v)| Triplet::new(r, c, v)).collect();
    SparseColMat::<usize, T>::try_new_from_triplets(size, size, &triplets)
        .map_err(|_| SimError::InvalidSolution)
}

/// An unknown the system fixes on its own: `row`'s only entry is in column
/// `known`, so `x[known] = b[row] / value`. The row's partner unknown (`row`
/// as a column) appears in one equation only, `known`'s row, and is found
/// from it once everything else is solved.
///
/// This is how a voltage source to ground stamps: its branch row reads
/// `v_node = V` and its branch current enters only the node's KCL row. A
/// supply rail shared by many parts is a dense row and column, and
/// factorising with it fills the sparse LU in; taking it out first keeps
/// the factor as sparse as the circuit without the rail.
#[derive(Debug, Clone, Copy)]
struct Fixed<T> {
    known: usize,
    row: usize,
    /// The entry at (`row`, `known`).
    value: T,
}

/// The fixed unknowns, each with its own row, none sharing an unknown.
/// Exact rearrangement only: a second row fixing an unknown already taken
/// is left in, so a circuit that over-determines a node stays singular. An
/// entry written twice counts twice, which only means it isn't taken out.
fn fixed_unknowns<T: MnaScalar>(size: usize, entries: &[(usize, usize, T)]) -> Vec<Fixed<T>> {
    // Per row and per column: the number of non-zeros and the last one.
    let mut row_count = vec![0u32; size];
    let mut row_last = vec![(0usize, T::zero()); size];
    let mut col_count = vec![0u32; size];
    let mut col_last = vec![0usize; size];
    for &(r, c, v) in entries {
        if v != T::zero() {
            row_count[r] += 1;
            row_last[r] = (c, v);
            col_count[c] += 1;
            col_last[c] = r;
        }
    }
    let mut taken = vec![false; size];
    let mut fixed = Vec::new();
    for row in 0..size {
        let (known, value) = row_last[row];
        if row_count[row] == 1
            && col_count[row] == 1
            && col_last[row] == known
            && known != row
            && !taken[known]
            && !taken[row]
        {
            taken[known] = true;
            taken[row] = true;
            fixed.push(Fixed { known, row, value });
        }
    }
    fixed
}

/// Solves the system with the `fixed` unknowns substituted first.
fn solve_with_fixed<T: MnaScalar>(
    size: usize,
    entries: &[(usize, usize, T)],
    fixed: &[Fixed<T>],
    b: &DVector<T>,
) -> Result<DVector<T>, SimError> {
    const NONE: usize = usize::MAX;
    let mut x = DVector::from_element(size, T::zero());
    // For each unknown: its index in the reduced system, or NONE.
    let mut index = vec![0usize; size];
    // For each row that's some `known`'s: which entry of `fixed`.
    let mut slot = vec![NONE; size];
    let mut is_known = vec![false; size];
    for (i, f) in fixed.iter().enumerate() {
        x[f.known] = b[f.row] / f.value;
        index[f.known] = NONE;
        index[f.row] = NONE;
        is_known[f.known] = true;
        slot[f.known] = i;
    }
    let mut m = 0;
    for j in index.iter_mut() {
        if *j != NONE {
            *j = m;
            m += 1;
        }
    }
    let mut rhs = DVector::from_element(m, T::zero());
    for (j, &k) in index.iter().enumerate() {
        if k != NONE {
            rhs[k] = b[j];
        }
    }
    // One pass: the reduced system, the known unknowns moved to the right,
    // and each `known`'s own row gathered to find its partner from.
    let mut reduced = Vec::with_capacity(entries.len());
    let mut partner_acc: Vec<T> = fixed.iter().map(|f| b[f.known]).collect();
    let mut partner_coef = vec![T::zero(); fixed.len()];
    let mut partner_terms: Vec<(usize, usize, T)> = Vec::new();
    for &(r, c, v) in entries {
        let i = slot[r];
        if i != NONE {
            if c == fixed[i].row {
                partner_coef[i] += v;
            } else {
                partner_terms.push((i, c, v));
            }
        } else if index[r] != NONE {
            if is_known[c] {
                rhs[index[r]] -= v * x[c];
            } else if index[c] != NONE {
                reduced.push((index[r], index[c], v));
            }
        }
    }
    if m > 0 {
        let y = factor_and_solve(&to_csc(m, reduced.into_iter())?, &rhs)?;
        for (j, &k) in index.iter().enumerate() {
            if k != NONE {
                x[j] = y[k];
            }
        }
    }
    for (i, c, v) in partner_terms {
        partner_acc[i] -= v * x[c];
    }
    for (i, f) in fixed.iter().enumerate() {
        x[f.row] = partner_acc[i] / partner_coef[i];
    }
    Ok(x)
}

fn factor_and_solve<T: MnaScalar>(
    mat: &SparseColMat<usize, T>,
    b: &DVector<T>,
) -> Result<DVector<T>, SimError> {
    let size = mat.nrows();
    let symbolic = SYMBOLIC.with(|cache| -> Result<SymbolicLu<usize>, SimError> {
        let mut cache = cache.borrow_mut();
        let pattern = mat.symbolic();
        if let Some((col_ptr, row_idx, symbolic)) = cache.as_ref() {
            if col_ptr == pattern.col_ptr() && row_idx == pattern.row_idx() {
                return Ok(symbolic.clone());
            }
        }
        let symbolic = SymbolicLu::try_new(pattern).map_err(|_| SimError::SingularMatrix)?;
        *cache = Some((
            pattern.col_ptr().to_vec(),
            pattern.row_idx().to_vec(),
            symbolic.clone(),
        ));
        Ok(symbolic)
    })?;

    let lu =
        Lu::try_new_with_symbolic(symbolic, mat.as_ref()).map_err(|_| SimError::SingularMatrix)?;
    let mut x = faer::Mat::<T>::from_fn(size, 1, |i, _| b[i]);
    lu.solve_in_place(x.as_mut());
    Ok(DVector::from_fn(size, |i, _| x[(i, 0)]))
}

/// Modified Nodal Analysis system.
///
/// Holds the (n+m)x(n+m) conductance/constraint matrix `a` and the (n+m)
/// right-hand-side vector `b`, where n = number of non-ground nodes and
/// m = number of independent voltage sources.
pub struct MnaSystem {
    /// (n+m)x(n+m) conductance/constraint matrix (siemens S in the node block).
    pub a: MnaMatrix<f64>,
    /// (n+m) right-hand-side vector (amperes A in the node block, volts V in the source block).
    pub b: DVector<f64>,
}

impl MnaSystem {
    /// Create a zero-filled MNA system of the given dimensions, sparse above
    /// [`DENSE_MAX`] unknowns.
    pub fn new(num_nodes: usize, num_vsources: usize) -> Self {
        let size = num_nodes + num_vsources;
        Self {
            a: MnaMatrix::zeros(size),
            b: DVector::zeros(size),
        }
    }

    /// Total dimension of the system (n + m).
    #[cfg(test)]
    pub fn size(&self) -> usize {
        self.b.len()
    }

    /// Solve Ax = b: LU with partial pivoting, dense or sparse.
    ///
    /// Returns the solution vector on success.
    ///
    /// # Errors
    ///
    /// [`SimError::SingularMatrix`] if the matrix is singular, or
    /// [`SimError::InvalidSolution`] if the solution contains NaN or Inf.
    pub fn solve(&self) -> Result<DVector<f64>, SimError> {
        self.a.solve(&self.b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::DMatrix;

    /// Test Circuit 7: Single resistor V1=5V, R1=1kOhm.
    ///
    /// Matrix:
    ///   [ 0.001   1 ] [v1]   [ 0 ]
    ///   [ 1       0 ] [j1] = [ 5 ]
    ///
    /// Expected: v1 = 5.0, j_V1 = -0.005
    #[test]
    fn solve_2x2_single_resistor() {
        let mut sys = MnaSystem::new(1, 1);

        // Resistor R1=1kOhm between n1 and ground: G = 0.001
        sys.a[(0, 0)] = 0.001;

        // Voltage source V1=5V: n1 is positive, ground is negative
        sys.a[(0, 1)] = 1.0;
        sys.a[(1, 0)] = 1.0;
        sys.b[1] = 5.0;

        let x = sys.solve().expect("should solve valid 2x2 system");

        assert_relative_eq!(x[0], 5.0, epsilon = 1e-10);
        assert_relative_eq!(x[1], -0.005, epsilon = 1e-10);
    }

    /// Test Circuit 1: Voltage divider V1=10V, R1=1k, R2=2k.
    ///
    /// Matrix:
    ///   [  0.001   -0.001    1  ] [v1]   [  0 ]
    ///   [ -0.001    0.0015   0  ] [v2] = [  0 ]
    ///   [  1        0        0  ] [j1]   [ 10 ]
    ///
    /// Expected: v1=10.0, v2=20/3, j_V1=-10/3000
    #[test]
    fn solve_3x3_voltage_divider() {
        let mut sys = MnaSystem::new(2, 1);

        // R1=1k between n1 (idx 0) and n2 (idx 1): G = 0.001
        sys.a[(0, 0)] += 0.001;
        sys.a[(1, 1)] += 0.001;
        sys.a[(0, 1)] -= 0.001;
        sys.a[(1, 0)] -= 0.001;

        // R2=2k between n2 (idx 1) and ground: G = 0.0005
        sys.a[(1, 1)] += 0.0005;

        // V1=10V: positive at n1 (idx 0), negative at ground
        sys.a[(0, 2)] = 1.0;
        sys.a[(2, 0)] = 1.0;
        sys.b[2] = 10.0;

        let x = sys.solve().expect("should solve valid 3x3 system");

        assert_relative_eq!(x[0], 10.0, epsilon = 1e-10);
        assert_relative_eq!(x[1], 20.0 / 3.0, epsilon = 1e-10);
        assert_relative_eq!(x[2], -10.0 / 3000.0, epsilon = 1e-10);
    }

    #[test]
    fn solve_singular_matrix_returns_error() {
        let sys = MnaSystem::new(1, 1);
        // All zeros -> singular

        let result = sys.solve();
        assert!(result.is_err());

        match result.unwrap_err() {
            crate::error::SimError::SingularMatrix => {} // expected
            other => panic!("expected SingularMatrix, got: {other}"),
        }
    }

    #[test]
    fn new_creates_zero_filled_system() {
        let sys = MnaSystem::new(3, 2);

        assert_eq!(sys.size(), 5);
        assert_eq!(sys.a.size(), 5);
        assert_eq!(sys.b.len(), 5);
        assert_eq!(sys.a.to_dense(), DMatrix::zeros(5, 5));
    }

    #[test]
    fn solve_well_conditioned_returns_ok() {
        // Identity matrix * x = [1, 2, 3] -> x = [1, 2, 3]
        let mut sys = MnaSystem::new(3, 0);
        sys.a[(0, 0)] = 1.0;
        sys.a[(1, 1)] = 1.0;
        sys.a[(2, 2)] = 1.0;
        sys.b[0] = 1.0;
        sys.b[1] = 2.0;
        sys.b[2] = 3.0;

        let x = sys.solve().expect("identity system should solve");
        assert_relative_eq!(x[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(x[1], 2.0, epsilon = 1e-10);
        assert_relative_eq!(x[2], 3.0, epsilon = 1e-10);
    }

    /// A tridiagonal system with a voltage-source row (zero diagonal), solved
    /// dense and sparse: the same answer.
    /// Nothing but ground has no unknowns: an empty solution, not a panic
    /// in the LU (it underflowed `0 - 1`).
    #[test]
    fn a_system_with_no_unknowns_solves_to_nothing() {
        let empty = DVector::<f64>::from_element(0, 0.0);
        assert_eq!(MnaMatrix::<f64>::dense(0).solve(&empty).unwrap().len(), 0);
        assert_eq!(MnaMatrix::<f64>::sparse(0).solve(&empty).unwrap().len(), 0);
    }

    #[test]
    fn sparse_matches_dense() {
        let n = 50;
        let fill = |a: &mut MnaMatrix<f64>| {
            for i in 0..n {
                a[(i, i)] += 2.0 + i as f64 * 0.01;
                if i + 1 < n {
                    a[(i, i + 1)] -= 1.0;
                    a[(i + 1, i)] -= 1.0;
                }
            }
            a[(0, n)] = 1.0;
            a[(n, 0)] = 1.0;
        };
        let (mut dense, mut sparse) = (MnaMatrix::dense(n + 1), MnaMatrix::sparse(n + 1));
        fill(&mut dense);
        fill(&mut sparse);
        assert_eq!(dense.to_dense(), sparse.to_dense());
        let b = DVector::from_fn(n + 1, |i, _| if i == n { 5.0 } else { 1e-3 * i as f64 });
        let (x, y) = (dense.solve(&b).unwrap(), sparse.solve(&b).unwrap());
        for i in 0..=n {
            assert_relative_eq!(x[i], y[i], max_relative = 1e-12, epsilon = 1e-15);
        }
        // Again, through the cached symbolic factorisation.
        let y2 = sparse.solve(&b).unwrap();
        assert_eq!(y, y2);
    }

    /// `n` nodes on a chain to ground, every one also tied to node 0, a
    /// rail that `sources` grounded voltage sources hold (node 0 and nodes
    /// 1.. for the rest), and one floating source between the last two
    /// nodes. Unknowns: `n` nodes, then one branch per source.
    fn hub_system<T: MnaScalar>(
        n: usize,
        sources: usize,
        g: impl Fn(usize) -> T,
        a: &mut MnaMatrix<T>,
    ) -> DVector<T> {
        let one = T::one();
        for i in 1..n {
            // Each node to the rail, to ground and to its neighbour.
            let (gr, gg) = (g(i), g(i + n));
            a[(i, i)] += gr + gg;
            a[(0, 0)] += gr;
            a[(i, 0)] -= gr;
            a[(0, i)] -= gr;
            if i + 1 < n {
                let gc = g(i + 2 * n);
                a[(i, i)] += gc;
                a[(i + 1, i + 1)] += gc;
                a[(i, i + 1)] -= gc;
                a[(i + 1, i)] -= gc;
            }
        }
        let mut b = DVector::from_element(n + sources + 1, T::zero());
        for s in 0..sources {
            let (node, k) = (if s == 0 { 0 } else { 10 * s }, n + s);
            a[(node, k)] += one;
            a[(k, node)] += one;
            b[k] = g(s) * T::from_real(1e3 + s as f64);
        }
        // A floating source between the last two nodes: not grounded, so it
        // stays in the system.
        let (p, q, k) = (n - 2, n - 1, n + sources);
        a[(p, k)] += one;
        a[(k, p)] += one;
        a[(q, k)] -= one;
        a[(k, q)] -= one;
        b[k] = T::from_real(0.25);
        b
    }

    fn assert_hub_matches_dense<T: MnaScalar + std::fmt::Debug>(g: impl Fn(usize) -> T + Copy) {
        let (n, sources) = (150, 3);
        let size = n + sources + 1;
        let (mut dense, mut sparse) = (MnaMatrix::dense(size), MnaMatrix::sparse(size));
        let b = hub_system(n, sources, g, &mut dense);
        hub_system(n, sources, g, &mut sparse);
        let (x, y) = (dense.solve(&b).unwrap(), sparse.solve(&b).unwrap());
        for i in 0..size {
            let scale = x[i].abs().max(1e-12);
            assert!(
                (x[i] - y[i]).abs() <= 1e-10 * scale,
                "unknown {i}: {:?} vs {:?}",
                x[i],
                y[i]
            );
        }
        // And transposed, as the adjoint analyses solve it.
        let (xt, yt) = (
            dense.transpose().solve(&b).unwrap(),
            sparse.transpose().solve(&b).unwrap(),
        );
        for i in 0..size {
            let scale = xt[i].abs().max(1e-12);
            assert!(
                (xt[i] - yt[i]).abs() <= 1e-10 * scale,
                "transposed {i}: {:?} vs {:?}",
                xt[i],
                yt[i]
            );
        }
    }

    /// A rail held by grounded sources is taken out before factorising
    /// (it would fill the sparse LU in); the answer, the source currents
    /// included, is the dense LU's.
    #[test]
    fn grounded_sources_are_substituted_exactly() {
        assert_hub_matches_dense(|i| 1e-3 * (1.0 + (i % 7) as f64));
        assert_hub_matches_dense(|i| {
            Complex64::new(1e-3 * (1.0 + (i % 7) as f64), 1e-4 * (i % 5) as f64)
        });
    }

    /// Two sources fixing one node over-determine it: still singular, not
    /// one of them picked.
    #[test]
    fn a_node_fixed_twice_is_still_singular() {
        let n = 120;
        let mut a = MnaMatrix::<f64>::sparse(n + 2);
        for i in 0..n {
            a[(i, i)] += 1e-3;
        }
        for k in [n, n + 1] {
            a[(0, k)] = 1.0;
            a[(k, 0)] = 1.0;
        }
        let b = DVector::from_fn(n + 2, |i, _| if i >= n { 5.0 } else { 0.0 });
        assert!(matches!(a.solve(&b), Err(SimError::SingularMatrix)));
    }

    #[test]
    fn sparse_complex_matches_dense() {
        let n = 30;
        let j = Complex64::new(0.0, 1.0);
        let fill = |a: &mut MnaMatrix<Complex64>| {
            for i in 0..n {
                a[(i, i)] += Complex64::new(2.0, 0.0) + j * (i as f64);
                if i + 1 < n {
                    a[(i, i + 1)] -= Complex64::new(1.0, 0.0);
                    a[(i + 1, i)] -= Complex64::new(1.0, 0.0);
                }
            }
        };
        let (mut dense, mut sparse) = (MnaMatrix::dense(n), MnaMatrix::sparse(n));
        fill(&mut dense);
        fill(&mut sparse);
        let b = DVector::from_fn(n, |i, _| Complex64::new(1.0, i as f64));
        let (x, y) = (dense.solve(&b).unwrap(), sparse.solve(&b).unwrap());
        for i in 0..n {
            assert!((x[i] - y[i]).norm() < 1e-12 * x[i].norm().max(1.0));
        }
    }

    #[test]
    fn sparse_singular_matrix_returns_an_error() {
        let a = MnaMatrix::<f64>::sparse(3);
        assert!(a.solve(&DVector::zeros(3)).is_err());
    }

    /// Reading an entry a sparse matrix never stored is zero, as dense.
    #[test]
    fn sparse_reads_unstored_entries_as_zero() {
        let mut a = MnaMatrix::<f64>::sparse(3);
        a[(0, 0)] = 2.0;
        assert_eq!(a[(0, 0)], 2.0);
        assert_eq!(a[(2, 1)], 0.0);
        assert_eq!(a.to_dense(), {
            let mut d = DMatrix::zeros(3, 3);
            d[(0, 0)] = 2.0;
            d
        });
    }

    /// A solve that produces a NaN or an infinity is an error, not an
    /// answer, in either storage.
    #[test]
    fn non_finite_solution_is_invalid() {
        for mut a in [MnaMatrix::<f64>::dense(2), MnaMatrix::<f64>::sparse(2)] {
            a[(0, 0)] = 1.0;
            a[(1, 1)] = 1.0;
            let b = DVector::from_vec(vec![1.0, f64::INFINITY]);
            assert!(matches!(a.solve(&b), Err(SimError::InvalidSolution)));
        }
    }
}
