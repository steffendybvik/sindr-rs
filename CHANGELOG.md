# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/steffendybvik/sindr-rs/compare/v0.1.0-alpha.2...HEAD
[0.1.0-alpha.2]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.2
[0.1.0-alpha.1]: https://github.com/steffendybvik/sindr-rs/releases/tag/v0.1.0-alpha.1
[#4]: https://github.com/steffendybvik/sindr-rs/issues/4
[#5]: https://github.com/steffendybvik/sindr-rs/pull/5
