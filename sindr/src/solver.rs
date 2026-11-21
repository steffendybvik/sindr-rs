//! Solver options and the [`Solver`] that carries them into every analysis.
//!
//! The free functions ([`solve_circuit`](crate::solve_circuit),
//! [`solve_ac`](crate::solve_ac), …) run with [`SolverOptions::default`],
//! which holds SPICE's default tolerances. Build a [`Solver`] to change
//! them, or to seed the operating point with a `.nodeset`:
//!
//! ```
//! use sindr::{Circuit, CircuitElement, Solver, SolverOptions};
//!
//! let circuit = Circuit::new("0", vec![
//!     CircuitElement::VoltageSource { id: "V1".into(), nodes: ["vcc".into(), "0".into()], voltage: 5.0, waveform: None },
//!     CircuitElement::Resistor { id: "R1".into(), nodes: ["vcc".into(), "a".into()], resistance: 1_000.0 },
//!     CircuitElement::Diode { id: "D1".into(), nodes: ["a".into(), "0".into()], temperature: 300.15, model: None },
//! ]);
//!
//! let mut options = SolverOptions::default();
//! options.reltol = 1e-6;
//! options.max_iterations = 200;
//! options.nodeset.insert("a".into(), 0.7);
//!
//! let result = Solver::new(options).circuit(&circuit)?;
//! assert!((result.node_voltages["a"] - 0.7).abs() < 0.1);
//! # Ok::<(), sindr::SimError>(())
//! ```
//!
//! A SPICE deck's `.options` card fills
//! [`ParsedNetlist::options`](crate::spice::ParsedNetlist::options), and its
//! `.nodeset` cards fill the options' [`nodeset`](SolverOptions::nodeset).

use std::collections::HashMap;

use crate::ac_analysis::{AcConfig, AcResult};
use crate::circuit::Circuit;
use crate::dc_sweep::{DcSweepAxis, DcSweepResult, NestedDcSweepResult};
use crate::error::SimError;
use crate::noise::{NoiseConfig, NoiseResult};
use crate::results::SimulationResult;
use crate::sensitivity::{SensConfig, SensResult};
use crate::temp_sweep::TempSweepResult;
use crate::transfer_function::{TfConfig, TfResult};
use crate::transient::TransientConfig;

/// Tolerances, iteration budgets and convergence aids for every analysis.
///
/// [`Default`] gives SPICE's defaults, which every golden is checked
/// against. Change the fields you need on a default value; more may come,
/// so the struct can't be built with a literal outside this crate.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
#[non_exhaustive]
pub struct SolverOptions {
    /// Relative tolerance (SPICE `RELTOL`, default `1e-3`). Newton–Raphson
    /// has converged when every node voltage moved by at most
    /// `vntol + reltol·|V|` and every junction by at most
    /// `vntol + reltol·n·V_T`; a transient step's local truncation error may
    /// be up to `trtol·(reltol·|V| + vntol)`.
    pub reltol: f64,
    /// Absolute voltage tolerance in volts (SPICE `VNTOL`, default `1e-6`).
    pub vntol: f64,
    /// Newton–Raphson iterations per solve before it counts as not
    /// converged (SPICE `ITL1`, default `100`). Transient steps use the same
    /// budget.
    pub max_iterations: usize,
    /// Extra conductance to ground (S) on every node for each step of gmin
    /// stepping, tried in order when plain Newton–Raphson fails; a last
    /// step without it always follows. Empty skips gmin stepping. Default
    /// `[1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-8, 1e-10]`.
    pub gmin_steps: Vec<f64>,
    /// Ramp every independent source up from zero when gmin stepping fails
    /// too (default `true`).
    pub source_stepping: bool,
    /// Smallest fraction of the sources one source-stepping step may add
    /// before it gives up (default `1e-4`).
    pub min_source_step: f64,
    /// Transient truncation-error factor (SPICE `TRTOL`, default `7`).
    pub trtol: f64,
    /// Most timesteps a transient may take, counting the `tstep` grid and
    /// every waveform breakpoint (default `10_000_000`).
    pub max_transient_steps: u64,
    /// Starting guess for the DC operating point in volts, keyed by node
    /// name (SPICE `.nodeset`). Unknown node names are skipped. Only
    /// nonlinear circuits use it; a transient starts from its
    /// [`TransientConfig::initial_voltages`] instead.
    pub nodeset: HashMap<String, f64>,
}

impl Default for SolverOptions {
    fn default() -> Self {
        Self {
            reltol: 1e-3,
            vntol: 1e-6,
            max_iterations: 100,
            gmin_steps: vec![1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-8, 1e-10],
            source_stepping: true,
            min_source_step: 1e-4,
            trtol: 7.0,
            max_transient_steps: 10_000_000,
            nodeset: HashMap::new(),
        }
    }
}

impl SolverOptions {
    /// These options with `nodeset` as the starting guess.
    #[must_use]
    pub fn with_nodeset(mut self, nodeset: HashMap<String, f64>) -> Self {
        self.nodeset = nodeset;
        self
    }

    /// Checks every field is usable.
    ///
    /// # Errors
    ///
    /// [`SimError::InvalidComponent`] naming the first field that isn't:
    /// a tolerance that isn't finite and positive, no iterations, a gmin
    /// step that isn't finite and positive, a `min_source_step` outside
    /// `(0, 1)`, no transient steps, or a `nodeset` voltage that isn't finite.
    pub fn validate(&self) -> Result<(), SimError> {
        let invalid =
            |msg: String| Err(SimError::InvalidComponent(format!("solver options: {msg}")));
        for (name, value) in [
            ("reltol", self.reltol),
            ("vntol", self.vntol),
            ("trtol", self.trtol),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return invalid(format!("{name} must be finite and positive (got {value})"));
            }
        }
        if self.max_iterations == 0 {
            return invalid("max_iterations must be at least 1".into());
        }
        if let Some(g) = self
            .gmin_steps
            .iter()
            .find(|g| !(g.is_finite() && **g > 0.0))
        {
            return invalid(format!(
                "every gmin step must be finite and positive (got {g})"
            ));
        }
        if !(self.min_source_step > 0.0 && self.min_source_step < 1.0) {
            return invalid(format!(
                "min_source_step must be between 0 and 1 (got {})",
                self.min_source_step
            ));
        }
        if self.max_transient_steps == 0 {
            return invalid("max_transient_steps must be at least 1".into());
        }
        if let Some((node, v)) = self.nodeset.iter().find(|(_, v)| !v.is_finite()) {
            return invalid(format!(
                "nodeset voltage for '{node}' must be finite (got {v})"
            ));
        }
        Ok(())
    }
}

/// Runs analyses with one set of [`SolverOptions`].
///
/// `Solver::default()` is what the free functions use: `solve_circuit(&c)`
/// is `Solver::default().circuit(&c)`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Solver {
    options: SolverOptions,
}

impl Solver {
    /// A solver with `options`.
    pub fn new(options: SolverOptions) -> Self {
        Self { options }
    }

    /// The options this solver runs with.
    pub fn options(&self) -> &SolverOptions {
        &self.options
    }

    /// As [`solve_circuit`](crate::solve_circuit).
    ///
    /// # Errors
    ///
    /// As [`solve_circuit`](crate::solve_circuit), and
    /// [`SolverOptions::validate`].
    pub fn circuit(&self, circuit: &Circuit) -> Result<SimulationResult, SimError> {
        self.options.validate()?;
        crate::solve_circuit_with(circuit, &self.options)
    }

    /// As [`solve_op`](crate::solve_op).
    ///
    /// # Errors
    ///
    /// As [`solve_op`](crate::solve_op), and [`SolverOptions::validate`].
    pub fn op(&self, circuit: &Circuit) -> Result<SimulationResult, SimError> {
        self.options.validate()?;
        crate::solve_op_with(circuit, &self.options)
    }

    /// As [`solve_transient`](crate::solve_transient).
    ///
    /// # Errors
    ///
    /// As [`solve_transient`](crate::solve_transient), and
    /// [`SolverOptions::validate`].
    pub fn transient(
        &self,
        circuit: &Circuit,
        config: &TransientConfig,
    ) -> Result<SimulationResult, SimError> {
        self.options.validate()?;
        crate::transient::solve_transient_with(circuit, config, &self.options)
    }

    /// As [`solve_ac`](crate::solve_ac).
    ///
    /// # Errors
    ///
    /// As [`solve_ac`](crate::solve_ac), and [`SolverOptions::validate`].
    pub fn ac(&self, circuit: &Circuit, config: &AcConfig) -> Result<AcResult, SimError> {
        self.options.validate()?;
        crate::ac_analysis::solve_ac_with(circuit, config, &self.options)
    }

    /// As [`solve_noise`](crate::solve_noise).
    ///
    /// # Errors
    ///
    /// As [`solve_noise`](crate::solve_noise), and [`SolverOptions::validate`].
    pub fn noise(&self, circuit: &Circuit, config: &NoiseConfig) -> Result<NoiseResult, SimError> {
        self.options.validate()?;
        crate::noise::solve_noise_with(circuit, config, &self.options)
    }

    /// As [`solve_tf`](crate::solve_tf).
    ///
    /// # Errors
    ///
    /// As [`solve_tf`](crate::solve_tf), and [`SolverOptions::validate`].
    pub fn tf(&self, circuit: &Circuit, config: &TfConfig) -> Result<TfResult, SimError> {
        self.options.validate()?;
        crate::transfer_function::solve_tf_with(circuit, config, &self.options)
    }

    /// As [`solve_sensitivity`](crate::solve_sensitivity).
    ///
    /// # Errors
    ///
    /// As [`solve_sensitivity`](crate::solve_sensitivity), and
    /// [`SolverOptions::validate`].
    pub fn sensitivity(
        &self,
        circuit: &Circuit,
        config: &SensConfig,
    ) -> Result<SensResult, SimError> {
        self.options.validate()?;
        crate::sensitivity::solve_sensitivity_with(circuit, config, &self.options)
    }

    /// As [`dc_sweep`](fn@crate::dc_sweep). The nodeset seeds the first point.
    ///
    /// # Errors
    ///
    /// As [`dc_sweep`](fn@crate::dc_sweep), and [`SolverOptions::validate`].
    pub fn dc_sweep(
        &self,
        circuit: &Circuit,
        source_id: &str,
        start: f64,
        stop: f64,
        num_points: usize,
    ) -> Result<DcSweepResult, SimError> {
        self.options.validate()?;
        crate::dc_sweep::dc_sweep_with(circuit, source_id, start, stop, num_points, &self.options)
    }

    /// As [`dc_sweep_nested`](crate::dc_sweep_nested). The nodeset seeds the
    /// first point.
    ///
    /// # Errors
    ///
    /// As [`dc_sweep_nested`](crate::dc_sweep_nested), and
    /// [`SolverOptions::validate`].
    pub fn dc_sweep_nested(
        &self,
        circuit: &Circuit,
        inner: &DcSweepAxis,
        outer: &DcSweepAxis,
    ) -> Result<NestedDcSweepResult, SimError> {
        self.options.validate()?;
        crate::dc_sweep::dc_sweep_nested_with(circuit, inner, outer, &self.options)
    }

    /// As [`temperature_sweep`](crate::temperature_sweep).
    ///
    /// # Errors
    ///
    /// As [`temperature_sweep`](crate::temperature_sweep), and
    /// [`SolverOptions::validate`].
    pub fn temperature_sweep(
        &self,
        circuit: &Circuit,
        temp_start: f64,
        temp_end: f64,
        num_steps: usize,
    ) -> Result<TempSweepResult, SimError> {
        self.options.validate()?;
        crate::temp_sweep::temperature_sweep_with(
            circuit,
            temp_start,
            temp_end,
            num_steps,
            &self.options,
        )
    }
}
