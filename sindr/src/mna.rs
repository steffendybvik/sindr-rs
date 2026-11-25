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
//! The matrix is stored densely for small systems and sparsely above
//! [`DENSE_MAX`] unknowns, where it is factorised with a sparse LU. A
//! circuit stamps the same entries in the same order on every Newton
//! iteration and timestep, so the first sparse solve records that order as a
//! layout: each write's slot in the compressed-column values, and the
//! pattern's symbolic LU. Later assemblies of that size add straight into the
//! slots and refactor numerically, with no hashing, sorting or symbolic
//! analysis. A write out of the recorded order switches that assembly back
//! to recording, and its solve leaves a new layout.
//!
//! Before a sparse factorisation, every unknown the system fixes on its own
//! is substituted out: a grounded voltage source's node, whose branch row
//! reads `v = V`, together with the source's branch current, found
//! afterwards from the node's row. A supply rail is a dense row and column,
//! and left in it fills the factor in; taken out, a board with one shared
//! rail factorises as cheaply as one with a supply per part. The rearrangement
//! is exact.

use std::any::{Any, TypeId};
use std::borrow::Cow;
use std::cell::{OnceCell, RefCell};
use std::collections::HashMap;
use std::hash::BuildHasherDefault;
use std::ops::{Index, IndexMut};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock, PoisonError};

use faer::dyn_stack::{MemBuffer, MemStack, StackReq};
use faer::sparse::linalg::lu::{factorize_symbolic_lu, LuRef, NumericLu, SymbolicLu};
use faer::sparse::{SparseColMatRef, SymbolicSparseColMat};
use faer::Conj;
use nalgebra::{DMatrix, DVector, Dyn, LU};
use num_complex::Complex64;

use crate::error::SimError;
use crate::node_map::Fnv;

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
    nalgebra::ComplexField<RealField = f64> + faer::traits::ComplexField + Copy + 'static
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

type EntryIndex = HashMap<(usize, usize), usize, BuildHasherDefault<Fnv>>;

#[derive(Debug, Clone)]
enum Storage<T: MnaScalar> {
    Dense(DMatrix<T>),
    /// Recording: entries in the order first written, with an index from
    /// position, and the entry each write went to. Solving it makes the
    /// [`Layout`] the next assembly replays.
    Sparse {
        entries: Vec<(usize, usize, T)>,
        index: EntryIndex,
        writes: Vec<usize>,
    },
    /// Replaying a layout: write `cursor` goes straight into its CSC slot.
    Replay {
        layout: Arc<Layout>,
        values: Vec<T>,
        cursor: usize,
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

    /// A zero `size × size` matrix stored sparsely. When the last sparse
    /// matrix of this size solved on this thread left a layout, writes
    /// follow it into place for as long as they come in its order.
    pub fn sparse(size: usize) -> Self {
        let storage = match last_layout::<T>(size) {
            Some(layout) => Storage::Replay {
                values: vec![T::zero(); layout.entries.len()],
                layout,
                cursor: 0,
            },
            None => Storage::Sparse {
                entries: Vec::new(),
                index: EntryIndex::default(),
                writes: Vec::new(),
            },
        };
        Self {
            size,
            storage,
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
            Storage::Replay { layout, values, .. } => {
                let mut m = DMatrix::from_element(self.size, self.size, T::zero());
                for &(r, c, s) in &layout.entries {
                    m[(r, c)] += values[s];
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
        self.factor()?.solve(b)
    }

    /// The LU factorisation of the matrix, to solve it and its transpose
    /// for any number of right-hand sides.
    ///
    /// # Errors
    ///
    /// [`SimError::SingularMatrix`] if a sparse matrix is singular. A
    /// singular dense matrix fails when solved.
    pub(crate) fn factor(&self) -> Result<Factor<'_, T>, SimError> {
        // A circuit with nothing but ground (an element from ground to
        // ground) has no unknowns; nalgebra's LU can't take a 0 × 0 matrix.
        if self.size == 0 {
            return Ok(Factor::Empty);
        }
        match &self.storage {
            Storage::Dense(m) => Ok(Factor::Dense {
                lu: m.clone().lu(),
                triangles: OnceCell::new(),
            }),
            Storage::Sparse {
                entries, writes, ..
            } => {
                let (layout, values) = Layout::build(self.size, entries, writes);
                keep_layout::<T>(&layout);
                SparseFactor::new(layout, Cow::Owned(values)).map(Factor::Sparse)
            }
            Storage::Replay {
                layout,
                values,
                cursor,
            } if *cursor == layout.writes.len() => {
                SparseFactor::new(layout.clone(), Cow::Borrowed(values)).map(Factor::Sparse)
            }
            // Fewer writes than its layout has: a pattern of its own.
            Storage::Replay {
                layout,
                values,
                cursor,
            } => {
                let (entries, _, writes) = replayed(layout, values, *cursor);
                let (layout, values) = Layout::build(self.size, &entries, &writes);
                keep_layout::<T>(&layout);
                SparseFactor::new(layout, Cow::Owned(values)).map(Factor::Sparse)
            }
        }
    }
}

/// An LU factorisation of an [`MnaMatrix`]: `A` and `Aᵀ` solve from the
/// same factor, so an adjoint costs a solve, not a second factorisation.
// Built on the stack for a solve or two: boxing the sparse one would cost
// an allocation a solve.
#[allow(clippy::large_enum_variant)]
pub(crate) enum Factor<'a, T: MnaScalar> {
    /// A system with no unknowns.
    Empty,
    Dense {
        lu: LU<T, Dyn, Dyn>,
        /// `L` and `U` unpacked, for the transpose solve, on its first use.
        triangles: OnceCell<(DMatrix<T>, DMatrix<T>)>,
    },
    Sparse(SparseFactor<'a, T>),
}

impl<T: MnaScalar> Factor<'_, T> {
    /// Solve `A · x = b`.
    pub(crate) fn solve(&self, b: &DVector<T>) -> Result<DVector<T>, SimError> {
        let x = match self {
            Factor::Empty => return Ok(DVector::from_element(0, T::zero())),
            Factor::Dense { lu, .. } => lu.solve(b).ok_or(SimError::SingularMatrix)?,
            Factor::Sparse(f) => f.solve(b, false)?,
        };
        finite(x)
    }

    /// Solve `Aᵀ · x = b`.
    pub(crate) fn solve_transpose(&self, b: &DVector<T>) -> Result<DVector<T>, SimError> {
        let x = match self {
            Factor::Empty => return Ok(DVector::from_element(0, T::zero())),
            // P·A = L·U, so Aᵀ = Uᵀ·Lᵀ·P.
            Factor::Dense { lu, triangles } => {
                let (l, u) = triangles.get_or_init(|| (lu.l(), lu.u()));
                let mut x = b.clone();
                if !u.tr_solve_upper_triangular_mut(&mut x)
                    || !l.tr_solve_lower_triangular_mut(&mut x)
                {
                    return Err(SimError::SingularMatrix);
                }
                lu.p().inv_permute_rows(&mut x);
                x
            }
            Factor::Sparse(f) => f.solve(b, true)?,
        };
        finite(x)
    }

    /// The factor with the matrix values it holds copied, so it outlives
    /// the matrix.
    pub(crate) fn into_owned(self) -> Factor<'static, T> {
        match self {
            Factor::Empty => Factor::Empty,
            Factor::Dense { lu, triangles } => Factor::Dense { lu, triangles },
            Factor::Sparse(f) => Factor::Sparse(f.into_owned()),
        }
    }
}

fn finite<T: MnaScalar>(x: DVector<T>) -> Result<DVector<T>, SimError> {
    if x.iter().all(MnaScalar::is_finite_value) {
        Ok(x)
    } else {
        Err(SimError::InvalidSolution)
    }
}

/// The first `cursor` writes of a replay as a recording: each entry with
/// the sum written to it so far, in the order first written.
fn replayed<T: MnaScalar>(
    layout: &Layout,
    values: &[T],
    cursor: usize,
) -> (Vec<(usize, usize, T)>, EntryIndex, Vec<usize>) {
    let mut entry_of_slot = vec![usize::MAX; layout.entries.len()];
    let mut entries = Vec::new();
    let mut index = EntryIndex::default();
    let mut writes = Vec::with_capacity(cursor);
    for (&(r, c), &s) in layout.writes[..cursor].iter().zip(&layout.write_slot) {
        if entry_of_slot[s] == usize::MAX {
            entry_of_slot[s] = entries.len();
            index.insert((r, c), entries.len());
            entries.push((r, c, values[s]));
        }
        writes.push(entry_of_slot[s]);
    }
    (entries, index, writes)
}

impl<T: MnaScalar> Index<(usize, usize)> for MnaMatrix<T> {
    type Output = T;

    fn index(&self, (r, c): (usize, usize)) -> &T {
        match &self.storage {
            Storage::Dense(m) => &m[(r, c)],
            Storage::Sparse { entries, index, .. } => {
                index.get(&(r, c)).map_or(&self.zero, |&k| &entries[k].2)
            }
            Storage::Replay { layout, values, .. } => {
                layout.slot(r, c).map_or(&self.zero, |s| &values[s])
            }
        }
    }
}

impl<T: MnaScalar> IndexMut<(usize, usize)> for MnaMatrix<T> {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut T {
        assert!(r < self.size && c < self.size, "MNA index out of range");
        // A write out of the layout's order: record from here on.
        if let Storage::Replay {
            layout,
            values,
            cursor,
        } = &self.storage
        {
            if layout.writes.get(*cursor) != Some(&(r, c)) {
                let (entries, index, writes) = replayed(layout, values, *cursor);
                self.storage = Storage::Sparse {
                    entries,
                    index,
                    writes,
                };
            }
        }
        match &mut self.storage {
            Storage::Dense(m) => &mut m[(r, c)],
            Storage::Sparse {
                entries,
                index,
                writes,
                ..
            } => {
                let k = *index.entry((r, c)).or_insert_with(|| {
                    entries.push((r, c, T::zero()));
                    entries.len() - 1
                });
                writes.push(k);
                &mut entries[k].2
            }
            Storage::Replay {
                layout,
                values,
                cursor,
            } => {
                let s = layout.write_slot[*cursor];
                *cursor += 1;
                &mut values[s]
            }
        }
    }
}

thread_local! {
    /// The layout the last shared sparse matrix solved on this thread left,
    /// one for real and one for complex matrices.
    static LAYOUTS: RefCell<[Option<Arc<Layout>>; 2]> = const { RefCell::new([None, None]) };
}

fn layout_kind<T: MnaScalar>() -> usize {
    usize::from(TypeId::of::<T>() != TypeId::of::<f64>())
}

fn last_layout<T: MnaScalar>(size: usize) -> Option<Arc<Layout>> {
    LAYOUTS.with(|cache| {
        cache.borrow()[layout_kind::<T>()]
            .as_ref()
            .filter(|layout| layout.size == size)
            .cloned()
    })
}

fn keep_layout<T: MnaScalar>(layout: &Arc<Layout>) {
    LAYOUTS.with(|cache| cache.borrow_mut()[layout_kind::<T>()] = Some(layout.clone()));
}

/// Where a sparse assembly's writes go: the order they came in, each one's
/// slot in the CSC value array, and the symbolic factorisations of that
/// pattern. An assembly whose writes come in the same order (every Newton
/// iteration and timestep of a circuit) adds straight into the slots: no
/// hashing, sorting, pattern compare or symbolic analysis.
#[derive(Debug)]
struct Layout {
    size: usize,
    writes: Vec<(usize, usize)>,
    write_slot: Vec<usize>,
    /// Each entry in the order first written: row, column and slot.
    entries: Vec<(usize, usize, usize)>,
    pattern: SymbolicSparseColMat<usize>,
    /// The whole system's symbolic LU, `None` if the analysis failed.
    symbolic: OnceLock<Option<Symbolic>>,
    /// The last reduction its fixed unknowns asked for.
    reduction: Mutex<Option<Arc<Reduction>>>,
}

impl Layout {
    /// The layout of a recording, with the values in its slots.
    fn build<T: MnaScalar>(
        size: usize,
        entries: &[(usize, usize, T)],
        writes: &[usize],
    ) -> (Arc<Self>, Vec<T>) {
        let mut order: Vec<usize> = (0..entries.len()).collect();
        order.sort_unstable_by_key(|&e| (entries[e].1, entries[e].0));
        let mut slot = vec![0; entries.len()];
        let mut col_ptr = vec![0usize; size + 1];
        let mut row_idx = Vec::with_capacity(entries.len());
        let mut values = Vec::with_capacity(entries.len());
        for (s, &e) in order.iter().enumerate() {
            let (r, c, v) = entries[e];
            slot[e] = s;
            col_ptr[c + 1] += 1;
            row_idx.push(r);
            values.push(v);
        }
        for c in 0..size {
            col_ptr[c + 1] += col_ptr[c];
        }
        let layout = Layout {
            size,
            writes: writes
                .iter()
                .map(|&k| (entries[k].0, entries[k].1))
                .collect(),
            write_slot: writes.iter().map(|&k| slot[k]).collect(),
            entries: entries
                .iter()
                .zip(&slot)
                .map(|(&(r, c, _), &s)| (r, c, s))
                .collect(),
            pattern: SymbolicSparseColMat::new_checked(size, size, col_ptr, None, row_idx),
            symbolic: OnceLock::new(),
            reduction: Mutex::new(None),
        };
        (Arc::new(layout), values)
    }

    /// The slot holding (`r`, `c`), if the pattern has it.
    fn slot(&self, r: usize, c: usize) -> Option<usize> {
        let pattern = self.pattern.as_ref();
        let col = pattern.col_ptr()[c]..pattern.col_ptr()[c + 1];
        pattern.row_idx()[col.clone()]
            .binary_search(&r)
            .ok()
            .map(|k| col.start + k)
    }

    /// The fixed unknowns, each with its own row, none sharing an unknown.
    /// Exact rearrangement only: a second row fixing an unknown already
    /// taken is left in, so a circuit that over-determines a node stays
    /// singular. An entry holding zero doesn't count.
    fn fixed_unknowns<T: MnaScalar>(&self, values: &[T]) -> Vec<Fixed<T>> {
        let size = self.size;
        // Per row and per column: the number of non-zeros and the last one.
        let mut row_count = vec![0u32; size];
        let mut row_last = vec![(0usize, T::zero()); size];
        let mut col_count = vec![0u32; size];
        let mut col_last = vec![0usize; size];
        for &(r, c, s) in &self.entries {
            let v = values[s];
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

    fn reduction_for<T: MnaScalar>(&self, fixed: &[Fixed<T>]) -> Arc<Reduction> {
        let mut cached = self
            .reduction
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if let Some(reduction) = cached.as_ref() {
            if reduction.fixes(fixed) {
                return reduction.clone();
            }
        }
        let reduction = Arc::new(Reduction::new(self, fixed));
        *cached = Some(reduction.clone());
        reduction
    }
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

const NONE: usize = usize::MAX;

/// A layout with a set of fixed unknowns substituted out: where each entry
/// of the full system goes, and the reduced system's pattern.
#[derive(Debug)]
struct Reduction {
    /// `(known, row)` of each fixed unknown.
    key: Vec<(usize, usize)>,
    /// For each unknown: its index in the reduced system, or [`NONE`].
    index: Vec<usize>,
    /// For each unknown that's some fixed one's `known`: which, or [`NONE`].
    fixed_of: Vec<usize>,
    pattern: SymbolicSparseColMat<usize>,
    symbolic: Option<Symbolic>,
    /// For each layout entry, in the order first written.
    actions: Vec<Action>,
}

#[derive(Debug, Clone, Copy)]
enum Action {
    /// Into this slot of the reduced matrix.
    Reduced(usize),
    /// Times the known unknown (column), moved to this reduced row's right.
    Rhs(usize, usize),
    /// The coefficient of fixed unknown `i`'s partner in its own row.
    PartnerCoef(usize),
    /// Another term of fixed unknown `i`'s row, times the unknown (column).
    PartnerTerm(usize, usize),
    /// The fixing entry itself, or one in a column nothing reads.
    Drop,
}

impl Reduction {
    fn new<T: MnaScalar>(layout: &Layout, fixed: &[Fixed<T>]) -> Self {
        let size = layout.size;
        let mut index = vec![0usize; size];
        // For each row that's some `known`'s: which entry of `fixed`.
        let mut slot = vec![NONE; size];
        let mut is_known = vec![false; size];
        for (i, f) in fixed.iter().enumerate() {
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
        let mut actions = Vec::with_capacity(layout.entries.len());
        // Reduced row, column and the action to point at its slot.
        let mut reduced = Vec::new();
        for &(r, c, _) in &layout.entries {
            let i = slot[r];
            let action = if i != NONE {
                if c == fixed[i].row {
                    Action::PartnerCoef(i)
                } else {
                    Action::PartnerTerm(i, c)
                }
            } else if index[r] == NONE {
                Action::Drop
            } else if is_known[c] {
                Action::Rhs(index[r], c)
            } else if index[c] != NONE {
                reduced.push((index[r], index[c], actions.len()));
                Action::Reduced(0)
            } else {
                Action::Drop
            };
            actions.push(action);
        }
        reduced.sort_unstable_by_key(|&(r, c, _)| (c, r));
        let mut col_ptr = vec![0usize; m + 1];
        let mut row_idx = Vec::with_capacity(reduced.len());
        for (s, &(r, c, a)) in reduced.iter().enumerate() {
            actions[a] = Action::Reduced(s);
            col_ptr[c + 1] += 1;
            row_idx.push(r);
        }
        for c in 0..m {
            col_ptr[c + 1] += col_ptr[c];
        }
        let pattern = SymbolicSparseColMat::new_checked(m, m, col_ptr, None, row_idx);
        let symbolic = if m > 0 { symbolic_lu(&pattern) } else { None };
        Self {
            key: fixed.iter().map(|f| (f.known, f.row)).collect(),
            index,
            fixed_of: slot,
            pattern,
            symbolic,
            actions,
        }
    }

    fn fixes<T>(&self, fixed: &[Fixed<T>]) -> bool {
        self.key.len() == fixed.len()
            && self
                .key
                .iter()
                .zip(fixed)
                .all(|(&(known, row), f)| known == f.known && row == f.row)
    }
}

/// A sparse matrix factorised: the whole system's LU, or with its fixed
/// unknowns substituted out first, the reduced system's.
pub(crate) struct SparseFactor<'a, T: MnaScalar> {
    layout: Arc<Layout>,
    values: Cow<'a, [T]>,
    reduced: Option<Reduced<T>>,
    /// The numeric LU and its symbolic LU's id; `None` when substituting
    /// leaves no unknowns.
    lu: Option<(u64, NumericLu<usize, T>)>,
}

/// The fixed unknowns a [`SparseFactor`] substituted out.
struct Reduced<T> {
    reduction: Arc<Reduction>,
    fixed: Vec<Fixed<T>>,
    /// Each fixed unknown's partner's coefficient in its row.
    partner_coef: Vec<T>,
}

impl<'a, T: MnaScalar> SparseFactor<'a, T> {
    fn new(layout: Arc<Layout>, values: Cow<'a, [T]>) -> Result<Self, SimError> {
        let fixed = layout.fixed_unknowns(&values);
        if fixed.is_empty() {
            let symbolic = layout
                .symbolic
                .get_or_init(|| symbolic_lu(&layout.pattern))
                .as_ref()
                .ok_or(SimError::SingularMatrix)?;
            let numeric = factorize(
                symbolic,
                SparseColMatRef::new(layout.pattern.as_ref(), &values),
            )?;
            let lu = Some((symbolic.id, numeric));
            return Ok(Self {
                layout,
                values,
                reduced: None,
                lu,
            });
        }
        let reduction = layout.reduction_for(&fixed);
        let mut reduced = vec![T::zero(); reduction.pattern.row_idx().len()];
        let mut partner_coef = vec![T::zero(); fixed.len()];
        for (&(_, _, s), &action) in layout.entries.iter().zip(&reduction.actions) {
            match action {
                Action::Reduced(k) => reduced[k] = values[s],
                Action::PartnerCoef(i) => partner_coef[i] += values[s],
                Action::Rhs(..) | Action::PartnerTerm(..) | Action::Drop => {}
            }
        }
        let lu = if reduction.pattern.nrows() > 0 {
            let symbolic = reduction
                .symbolic
                .as_ref()
                .ok_or(SimError::SingularMatrix)?;
            let numeric = factorize(
                symbolic,
                SparseColMatRef::new(reduction.pattern.as_ref(), &reduced),
            )?;
            Some((symbolic.id, numeric))
        } else {
            None
        };
        Ok(Self {
            layout,
            values,
            reduced: Some(Reduced {
                reduction,
                fixed,
                partner_coef,
            }),
            lu,
        })
    }

    fn into_owned(mut self) -> SparseFactor<'static, T> {
        SparseFactor {
            layout: self.layout.clone(),
            values: Cow::Owned(std::mem::take(&mut self.values).into_owned()),
            reduced: self.reduced.take(),
            lu: self.lu.take(),
        }
    }

    /// The symbolic LU the numeric one was factorised with.
    fn symbolic(&self) -> Option<&Symbolic> {
        match &self.reduced {
            Some(r) => r.reduction.symbolic.as_ref(),
            None => self.layout.symbolic.get().and_then(Option::as_ref),
        }
    }

    /// Solve the factorised system (the whole one or the reduced one), or
    /// its transpose.
    fn lu_solve(&self, b: &DVector<T>, transpose: bool) -> Result<DVector<T>, SimError> {
        match (self.symbolic(), &self.lu) {
            (Some(symbolic), Some((_, numeric))) => Ok(lu_solve(symbolic, numeric, b, transpose)),
            _ => Err(SimError::SingularMatrix),
        }
    }

    fn solve(&self, b: &DVector<T>, transpose: bool) -> Result<DVector<T>, SimError> {
        match &self.reduced {
            None => self.lu_solve(b, transpose),
            Some(r) if transpose => self.solve_reduced_transpose(r, b),
            Some(r) => self.solve_reduced(r, b),
        }
    }

    /// `A x = b` with the fixed unknowns substituted first.
    fn solve_reduced(&self, r: &Reduced<T>, b: &DVector<T>) -> Result<DVector<T>, SimError> {
        let (layout, values, reduction) = (&self.layout, &self.values, &r.reduction);
        let m = reduction.pattern.nrows();
        let mut x = DVector::from_element(layout.size, T::zero());
        for f in &r.fixed {
            x[f.known] = b[f.row] / f.value;
        }
        let mut rhs = DVector::from_element(m, T::zero());
        for (j, &k) in reduction.index.iter().enumerate() {
            if k != NONE {
                rhs[k] = b[j];
            }
        }
        // The known unknowns moved to the right.
        for (&(_, _, s), &action) in layout.entries.iter().zip(&reduction.actions) {
            if let Action::Rhs(row, c) = action {
                rhs[row] -= values[s] * x[c];
            }
        }
        if m > 0 {
            let y = self.lu_solve(&rhs, false)?;
            for (j, &k) in reduction.index.iter().enumerate() {
                if k != NONE {
                    x[j] = y[k];
                }
            }
        }
        let mut partner_acc: Vec<T> = r.fixed.iter().map(|f| b[f.known]).collect();
        for (&(_, _, s), &action) in layout.entries.iter().zip(&reduction.actions) {
            if let Action::PartnerTerm(i, c) = action {
                partner_acc[i] -= values[s] * x[c];
            }
        }
        for (i, f) in r.fixed.iter().enumerate() {
            x[f.row] = partner_acc[i] / r.partner_coef[i];
        }
        Ok(x)
    }

    /// `Aᵀ y = b` with the fixed unknowns substituted first. In `Aᵀ` the
    /// roles swap: the partner's column (one entry, in `known`'s row) fixes
    /// `y[known]`, and `known`'s column, the fixing entry's row, is where
    /// `y[row]` is found last.
    fn solve_reduced_transpose(
        &self,
        r: &Reduced<T>,
        b: &DVector<T>,
    ) -> Result<DVector<T>, SimError> {
        let (layout, values, reduction) = (&self.layout, &self.values, &r.reduction);
        let m = reduction.pattern.nrows();
        let mut y = DVector::from_element(layout.size, T::zero());
        for (f, &coef) in r.fixed.iter().zip(&r.partner_coef) {
            y[f.known] = b[f.row] / coef;
        }
        let mut rhs = DVector::from_element(m, T::zero());
        for (j, &k) in reduction.index.iter().enumerate() {
            if k != NONE {
                rhs[k] = b[j];
            }
        }
        // A known unknown's row terms, as columns of the reduced unknowns.
        for (&(_, c, s), &action) in layout.entries.iter().zip(&reduction.actions) {
            if let Action::PartnerTerm(i, _) = action {
                if reduction.index[c] != NONE {
                    rhs[reduction.index[c]] -= values[s] * y[r.fixed[i].known];
                }
            }
        }
        if m > 0 {
            let z = self.lu_solve(&rhs, true)?;
            for (j, &k) in reduction.index.iter().enumerate() {
                if k != NONE {
                    y[j] = z[k];
                }
            }
        }
        // Each known unknown's column, but for the fixing entry itself.
        let mut partner_acc: Vec<T> = r.fixed.iter().map(|f| b[f.known]).collect();
        for (&(row, c, s), &action) in layout.entries.iter().zip(&reduction.actions) {
            if matches!(action, Action::Rhs(..) | Action::PartnerTerm(..)) {
                let i = reduction.fixed_of[c];
                if i != NONE {
                    partner_acc[i] -= values[s] * y[row];
                }
            }
        }
        for (i, f) in r.fixed.iter().enumerate() {
            y[f.row] = partner_acc[i] / f.value;
        }
        Ok(y)
    }
}

impl<T: MnaScalar> Drop for SparseFactor<'_, T> {
    /// Hands the numeric LU back to the thread's [`Workspace`], for the
    /// next factorisation of the same pattern to refactor in place.
    fn drop(&mut self) {
        if let Some((id, numeric)) = self.lu.take() {
            with_workspace::<T, _>(|work| {
                work.numeric = numeric;
                work.symbolic = id;
            });
        }
    }
}

/// A symbolic LU with an id of its own, for [`Workspace`] to tell them apart.
#[derive(Debug)]
struct Symbolic {
    id: u64,
    lu: SymbolicLu<usize>,
}

fn symbolic_lu(pattern: &SymbolicSparseColMat<usize>) -> Option<Symbolic> {
    static NEXT: AtomicU64 = AtomicU64::new(1);
    let lu = factorize_symbolic_lu(pattern.as_ref(), Default::default()).ok()?;
    Some(Symbolic {
        id: NEXT.fetch_add(1, Ordering::Relaxed),
        lu,
    })
}

/// A numeric LU and scratch memory, kept between factorisations on a
/// thread so a refactor allocates nothing once they're big enough.
struct Workspace<T> {
    numeric: NumericLu<usize, T>,
    /// The id of the symbolic LU `numeric` was last factorised with: faer
    /// keeps some of a numeric LU's arrays at their old size, so one is
    /// only refactorised for the same symbolic analysis.
    symbolic: u64,
    stack: MemBuffer,
    stack_bytes: usize,
}

impl<T: MnaScalar> Workspace<T> {
    fn new() -> Self {
        Self {
            numeric: NumericLu::new(),
            symbolic: 0,
            stack: MemBuffer::new(StackReq::EMPTY),
            stack_bytes: 0,
        }
    }

    /// Scratch memory for `req`, grown if it's short.
    fn stack(&mut self, req: StackReq) -> &mut MemStack {
        let bytes = req.unaligned_bytes_required();
        if bytes > self.stack_bytes {
            self.stack = MemBuffer::new(req);
            self.stack_bytes = bytes;
        }
        MemStack::new(&mut self.stack)
    }
}

thread_local! {
    /// A [`Workspace`] for real and one for complex systems.
    static WORKSPACES: RefCell<[Option<Box<dyn Any>>; 2]> = const { RefCell::new([None, None]) };
}

/// Runs `f` on this thread's [`Workspace`] for `T`, taken out while it
/// runs, or on a fresh one if there's none (a thread shutting down).
fn with_workspace<T: MnaScalar, R>(f: impl FnOnce(&mut Workspace<T>) -> R) -> R {
    let kind = layout_kind::<T>();
    let mut work = WORKSPACES
        .try_with(|cache| {
            let taken = cache.try_borrow_mut().ok()?[kind].take()?;
            taken.downcast::<Workspace<T>>().ok()
        })
        .ok()
        .flatten()
        .unwrap_or_else(|| Box::new(Workspace::new()));
    let result = f(&mut work);
    let _ = WORKSPACES.try_with(|cache| {
        if let Ok(mut cache) = cache.try_borrow_mut() {
            cache[kind] = Some(work);
        }
    });
    result
}

/// The numeric LU of `mat`, refactored in place in the thread's workspace
/// when its last one had the same symbolic LU.
fn factorize<T: MnaScalar>(
    symbolic: &Symbolic,
    mat: SparseColMatRef<'_, usize, T>,
) -> Result<NumericLu<usize, T>, SimError> {
    with_workspace(|work: &mut Workspace<T>| {
        let par = faer::get_global_parallelism();
        let req = symbolic
            .lu
            .factorize_numeric_lu_scratch::<T>(par, Default::default());
        let mut numeric = std::mem::take(&mut work.numeric);
        if work.symbolic != symbolic.id {
            numeric = NumericLu::new();
        }
        // Taken: the factor gives it back, with its id, when dropped.
        work.symbolic = 0;
        symbolic
            .lu
            .factorize_numeric_lu(&mut numeric, mat, par, work.stack(req), Default::default())
            .map_err(|_| SimError::SingularMatrix)?;
        Ok(numeric)
    })
}

fn lu_solve<T: MnaScalar>(
    symbolic: &Symbolic,
    numeric: &NumericLu<usize, T>,
    b: &DVector<T>,
    transpose: bool,
) -> DVector<T> {
    with_workspace(|work: &mut Workspace<T>| {
        let size = b.len();
        let par = faer::get_global_parallelism();
        let req = symbolic.lu.solve_in_place_scratch::<T>(1, par);
        let lu = LuRef::new_unchecked(&symbolic.lu, numeric);
        let mut x = faer::Mat::<T>::from_fn(size, 1, |i, _| b[i]);
        if transpose {
            lu.solve_transpose_in_place_with_conj(Conj::No, x.as_mut(), par, work.stack(req));
        } else {
            lu.solve_in_place_with_conj(Conj::No, x.as_mut(), par, work.stack(req));
        }
        DVector::from_fn(size, |i, _| x[(i, 0)])
    })
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
    /// 10, 20, .. for the rest), a few one-way couplings, and one floating
    /// source between the last two nodes. Unknowns: `n` nodes, then one
    /// branch per source.
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
        // Couplings one way only, as a controlled source stamps, so that
        // Aᵀ ≠ A: into the rail's row, from a held node's column, between
        // two held nodes and between two free ones.
        for (r, c) in [(0, 5), (7, 10), (10, 0), (20, 30)] {
            a[(r, c)] += g(r + c) * T::from_real(0.5);
        }
        let mut b = DVector::from_element(n + sources + 1, T::zero());
        for s in 0..sources {
            let (node, k) = (if s == 0 { 0 } else { 10 * s }, n + s);
            // A branch row scaled by 2, so its entry differs from the
            // branch current's in the node row.
            a[(node, k)] += one;
            a[(k, node)] += one + one;
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
        // And transposed, as the adjoint analyses solve it: from the same
        // factor, against an LU of the transpose itself.
        let reference = dense.to_dense().transpose().lu().solve(&b).unwrap();
        for (what, yt) in [
            (
                "dense",
                dense.factor().unwrap().solve_transpose(&b).unwrap(),
            ),
            (
                "sparse",
                sparse.factor().unwrap().solve_transpose(&b).unwrap(),
            ),
        ] {
            for i in 0..size {
                let scale = reference[i].abs().max(1e-12);
                assert!(
                    (reference[i] - yt[i]).abs() <= 1e-10 * scale,
                    "{what} transposed {i}: {:?} vs {:?}",
                    reference[i],
                    yt[i]
                );
            }
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

    /// With nothing to substitute out, the transpose solves from the whole
    /// system's factor, and a factor solved again gives the same answer.
    #[test]
    fn a_factor_solves_its_transpose() {
        let n = 150;
        let mut a = MnaMatrix::<f64>::sparse(n);
        for i in 0..n {
            a[(i, i)] += 2e-3 + 1e-5 * i as f64;
            if i + 1 < n {
                a[(i, i + 1)] -= 1e-3;
                a[(i + 1, i)] -= 1.5e-3;
            }
            if i + 7 < n {
                a[(i, i + 7)] += 2e-4;
            }
        }
        let b = DVector::from_fn(n, |i, _| 1e-3 * (1.0 + (i % 11) as f64));
        let lu = a.factor().unwrap();
        assert!(matches!(&lu, Factor::Sparse(f) if f.reduced.is_none()));
        let reference = a.to_dense().transpose().lu().solve(&b).unwrap();
        let y = lu.solve_transpose(&b).unwrap();
        for i in 0..n {
            assert_relative_eq!(reference[i], y[i], max_relative = 1e-10);
        }
        let x = lu.solve(&b).unwrap();
        assert_eq!(lu.solve(&b).unwrap(), x);
        assert_eq!(a.solve(&b).unwrap(), x);
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

    /// A sparse ladder of `n` unknowns with a grounded source on node 0,
    /// written in a fixed order, scaled by `g`.
    fn ladder(n: usize, g: f64) -> (MnaMatrix<f64>, DVector<f64>) {
        let mut a = MnaMatrix::sparse(n + 1);
        for i in 0..n {
            a[(i, i)] += g * (2.0 + 0.01 * i as f64);
            if i + 1 < n {
                a[(i, i + 1)] -= g;
                a[(i + 1, i)] -= g;
            }
        }
        a[(0, n)] += 1.0;
        a[(n, 0)] += 1.0;
        let b = DVector::from_fn(n + 1, |i, _| if i == n { 5.0 } else { 0.0 });
        (a, b)
    }

    /// Each sparse solve against the dense LU of the same matrix.
    fn assert_matches_dense(a: &MnaMatrix<f64>, b: &DVector<f64>) {
        let (x, y) = (a.to_dense().lu().solve(b).unwrap(), a.solve(b).unwrap());
        for i in 0..b.len() {
            assert_relative_eq!(x[i], y[i], max_relative = 1e-10, epsilon = 1e-14);
        }
    }

    /// The second assembly in the same order replays the first one's
    /// layout, and solves the same as a fresh one.
    #[test]
    fn a_replayed_assembly_solves_as_a_fresh_one() {
        let (a, b) = ladder(150, 1e-3);
        let first = a.solve(&b).unwrap();
        let (a, b) = ladder(150, 1e-3);
        assert!(matches!(a.storage, Storage::Replay { .. }));
        assert_eq!(a.solve(&b).unwrap(), first);
        // Other values, same order: still replayed.
        let (a, b) = ladder(150, 2e-3);
        assert!(matches!(a.storage, Storage::Replay { .. }));
        assert_matches_dense(&a, &b);
    }

    /// A write out of the recorded order, or an assembly that stops short of
    /// it, records a pattern of its own and solves right.
    #[test]
    fn a_changed_pattern_records_again() {
        let (a, b) = ladder(150, 1e-3);
        a.solve(&b).unwrap();
        // An extra entry partway through: out of order.
        let (mut a, b) = ladder(150, 1e-3);
        a[(3, 7)] -= 1e-4;
        assert!(matches!(a.storage, Storage::Sparse { .. }));
        assert_matches_dense(&a, &b);
        // Its layout is the one kept now, and the next assembly in the
        // ladder's own order stops short of it.
        let (a, b) = ladder(150, 1e-3);
        assert!(
            matches!(a.storage, Storage::Replay { cursor, ref layout, .. }
            if cursor < layout.writes.len())
        );
        assert_matches_dense(&a, &b);
        // Reading back through a replay.
        let (a, _) = ladder(150, 1e-3);
        assert_eq!(a[(0, 0)], 2e-3);
        assert_eq!(a[(5, 9)], 0.0);
    }

    /// Systems of different sizes in turn each keep their factor right (a
    /// numeric LU reused for another size kept its old arrays).
    #[test]
    fn sizes_in_turn_solve_right() {
        for n in [150, 120, 150, 200, 120] {
            let (a, b) = ladder(n, 1e-3);
            assert_matches_dense(&a, &b);
            let (a, b) = ladder(n, 1e-3);
            assert_matches_dense(&a, &b);
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
