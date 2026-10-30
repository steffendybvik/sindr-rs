//! Modified Nodal Analysis (MNA) matrix system.
//!
//! Holds the `(n+m) × (n+m)` MNA matrix and right-hand-side vector that every
//! analysis path eventually solves, where `n` is the number of non-ground
//! nodes and `m` is the number of branch-current unknowns (independent
//! voltage sources, controlled sources that introduce a branch, etc.).
//!
//! Components write themselves into this system via [`stamp`](crate::stamp);
//! the linear DC, Newton–Raphson, and transient pipelines all build an
//! [`MnaSystem`], stamp into it, and call [`MnaSystem::solve`].
//!
//! The matrix is stored densely for small systems and as a sparse list of
//! entries above [`DENSE_MAX`] unknowns, where it is factorised with a sparse
//! LU. A circuit's sparsity pattern is the same from one Newton iteration or
//! timestep to the next, so the sparse LU's symbolic analysis is reused while
//! the pattern doesn't change.

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
/// is sparse. From `benches/mna_solve.rs`: dense and sparse cost about the
/// same at 100 unknowns (70 µs vs 35–80 µs), dense is 5–7× faster at 10, and
/// sparse is 50–250× faster at 1 000.
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

    /// Whether the matrix is stored sparsely.
    pub fn is_sparse(&self) -> bool {
        matches!(self.storage, Storage::Sparse { .. })
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

    /// Solve `self · x = b`.
    ///
    /// # Errors
    ///
    /// [`SimError::SingularMatrix`] if the matrix is singular, or
    /// [`SimError::InvalidSolution`] if the solution contains NaN or Inf.
    pub fn solve(&self, b: &DVector<T>) -> Result<DVector<T>, SimError> {
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
    let triplets: Vec<Triplet<usize, usize, T>> = entries
        .iter()
        .map(|&(r, c, v)| Triplet::new(r, c, v))
        .collect();
    let mat = SparseColMat::<usize, T>::try_new_from_triplets(size, size, &triplets)
        .map_err(|_| SimError::InvalidSolution)?;

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
    /// Number of non-ground nodes (n).
    pub num_nodes: usize,
    /// Number of independent voltage sources contributing branch-current unknowns (m).
    pub num_vsources: usize,
}

impl MnaSystem {
    /// Create a zero-filled MNA system of the given dimensions, sparse above
    /// [`DENSE_MAX`] unknowns.
    pub fn new(num_nodes: usize, num_vsources: usize) -> Self {
        let size = num_nodes + num_vsources;
        Self {
            a: MnaMatrix::zeros(size),
            b: DVector::zeros(size),
            num_nodes,
            num_vsources,
        }
    }

    /// Total dimension of the system (n + m).
    pub fn size(&self) -> usize {
        self.num_nodes + self.num_vsources
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
}
