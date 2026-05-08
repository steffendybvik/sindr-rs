# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0-alpha.4] - 2026-05-08

### Changed

- README tagline + crate description for `sindr` rewritten to
  "Rust circuit simulator. SPICE-style MNA solver with built-in
  semiconductor device models." (replaces "first-class semiconductor
  models", which overclaimed SPICE-grade fidelity).

### Removed

- **Breaking:** `SimError::UnsupportedCircuit(String)` variant. It was
  declared but never produced by the solver — pure dead code. Downstream
  pattern matches against this variant must be removed. If a future
  topology check needs to reject an unsupported configuration, a
  variant will be reintroduced with a real call site and a triggering
  test.

### Added

- README badges for CI, crates.io, docs.rs, and licence.
- `CLAUDE.md` at the repo root pointing to `AGENTS.md` as the canonical
  agent-orientation document.
- `.github/dependabot.yml` for weekly cargo + GitHub Actions updates.

## [0.1.0-alpha.3] - 2026-05-07

### Changed

- Declared MSRV (`rust-version = "1.87"`) on both crates. Required by
  `nalgebra 0.34.2` (which declares MSRV 1.87) and the transitive
  `nalgebra-macros 0.3.0` (which uses Rust edition 2024, requiring
  Cargo 1.85+).
- Pinned workspace path dependencies with explicit `version` fields so
  `cargo publish` can resolve `sindr-devices` to a registry coordinate
  when publishing `sindr`.
- One-time `rustfmt` sweep across the workspace.

### Added

- `[package.metadata.docs.rs]` blocks in both crates so docs.rs renders
  feature-gated items with `--cfg docsrs`.
- `NOTICE` shipped inside each crate's package (the root `NOTICE` is not
  picked up by `cargo package`).
- `CHANGELOG.md` (Keep a Changelog format).
- GitHub Actions CI workflow: `cargo fmt --check`, `cargo clippy -D warnings`,
  test matrix on stable + MSRV with `--all-features` and
  `--no-default-features`, `cargo doc -D warnings`.
- Quick voltage-divider example in the README.

## [0.1.0-alpha.2] - 2026-05-07

### Fixed

- Transient timestep array values were off-by-one against their time labels.
  Each snapshot is now stamped with its post-step time, so `(t, V)` pairs from
  `transient.timesteps` plot correctly. ([#4], [#5])

### Changed

- `transient.timesteps` no longer includes a `t=0` entry; samples now run from
  `dt` to slightly past `duration`. The math of the simulation itself was
  already correct — only the time-axis labelling changed.

## [0.1.0-alpha.1] - 2026-05

### Added

- Initial pre-release. MNA solver with Newton-Raphson convergence, transient
  (Backward Euler), AC analysis, DC sweep, temperature sweep. SPICE-style
  semiconductor models (diode, BJT, MOSFET, JFET, IGBT, varactor, zener,
  Schottky, LED, photodiode, photoresistor, thermistor).

[Unreleased]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.4...HEAD
[0.1.0-alpha.4]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.4
[0.1.0-alpha.3]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.3
[0.1.0-alpha.2]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.2
[0.1.0-alpha.1]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.1
[#4]: https://github.com/steffendybvik/sindr-rs/issues/4
[#5]: https://github.com/steffendybvik/sindr-rs/pull/5
