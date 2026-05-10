//! Modified Nodal Analysis (MNA) matrix system.
//!
//! Holds the dense `(n+m) × (n+m)` MNA matrix and right-hand-side vector
//! that every analysis path eventually solves, where `n` is the number of
//! non-ground nodes and `m` is the number of branch-current unknowns
//! (independent voltage sources, controlled sources that introduce a branch,
//! etc.).
//!
//! Components write themselves into this system via [`stamp`](crate::stamp);
//! the linear DC, Newton–Raphson, and transient pipelines all build an
//! [`MnaSystem`], stamp into it, and call [`MnaSystem::solve`].

use nalgebra::{DMatrix, DVector};

use crate::error::SimError;

/// Modified Nodal Analysis system.
///
/// Holds the (n+m)x(n+m) conductance/constraint matrix `a` and the (n+m)
/// right-hand-side vector `b`, where n = number of non-ground nodes and
/// m = number of independent voltage sources.
pub struct MnaSystem {
    pub a: DMatrix<f64>,
    pub b: DVector<f64>,
    pub num_nodes: usize,
    pub num_vsources: usize,
}

impl MnaSystem {
    /// Create a zero-filled MNA system of the given dimensions.
    pub fn new(num_nodes: usize, num_vsources: usize) -> Self {
        let size = num_nodes + num_vsources;
        Self {
            a: DMatrix::zeros(size, size),
            b: DVector::zeros(size),
            num_nodes,
            num_vsources,
        }
    }

    /// Total dimension of the system (n + m).
    pub fn size(&self) -> usize {
        self.num_nodes + self.num_vsources
    }

    /// Solve Ax = b via LU decomposition with partial pivoting.
    ///
    /// Returns the solution vector on success, or a [`SimError`] if the
    /// matrix is singular or the solution contains NaN/Inf values.
    pub fn solve(&self) -> Result<DVector<f64>, SimError> {
        let solution = self
            .a
            .clone()
            .lu()
            .solve(&self.b)
            .ok_or(SimError::SingularMatrix)?;

        // Check every element for NaN or Infinity
        for i in 0..solution.len() {
            if solution[i].is_nan() || solution[i].is_infinite() {
                return Err(SimError::InvalidSolution);
            }
        }

        Ok(solution)
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
        assert_eq!(sys.a.nrows(), 5);
        assert_eq!(sys.a.ncols(), 5);
        assert_eq!(sys.b.len(), 5);
        assert_eq!(sys.a, DMatrix::zeros(5, 5));
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
}
