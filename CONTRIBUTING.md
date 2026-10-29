# Contributing to sindr

Thanks for helping. This page covers the local checks, the release rule CI
enforces, and where things live. [`AGENTS.md`](./AGENTS.md) goes deeper on the
architecture, how to add a circuit element or device model, and conventions.

## Toolchain

- **MSRV is Rust 1.87** (`rust-version` in both crate manifests). CI tests on
  1.87 and on stable, so don't use std or language features newer than 1.87.
- Edition 2021. No `unsafe`, and no `unwrap()`/`expect()` outside tests:
  return a `SimError`.

## Before you push

Run what CI runs:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
cargo test --workspace --no-default-features
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps --all-features
```

- `--all-features` turns on the `spice` parser and its end-to-end tests.
  `--no-default-features` checks the build without `serde` and `examples`.
- Both crates have `#![deny(missing_docs)]`: every public item needs a doc
  comment.
- The code blocks in `sindr-devices/README.md` run as doctests, so a stale
  README example fails `cargo test`.

## Tests

- Unit tests sit next to the code in `#[cfg(test)] mod tests`.
- Cross-cutting tests are in `sindr/tests/`. SPICE fixtures live in
  `sindr/tests/fixtures/`, and those tests need `--features spice`.
- Prefer checks against closed-form answers (a divider, an RC corner
  frequency, `1 − e^(−t/τ)`) over values copied from the current output.
- Parsers and evaluators get `proptest` properties (see `spice/si.rs` and
  `spice/param_eval.rs`). If one fails, proptest writes the shrunk case to
  `sindr/proptest-regressions/`; commit that file with the fix.
- The `coverage` CI job reports per-file line coverage in the job summary. It
  only reports and never fails a build. Locally:
  `cargo llvm-cov --workspace --all-features --summary-only`.

## Versions and the CHANGELOG

Every PR to `main` must bump the version. The `version bumped` CI job fails
unless:

1. `version` in both `sindr/Cargo.toml` and `sindr-devices/Cargo.toml`
   differs from `main`, and
2. `CHANGELOG.md` has a `## [<new version>]` section.

The two crates share one version. A bump touches:

- `version` in `sindr/Cargo.toml` and `sindr-devices/Cargo.toml`
- the `sindr` / `sindr-devices` entries under `[workspace.dependencies]` in
  the root `Cargo.toml`, plus `Cargo.lock`
- the version in install snippets and status lines: `README.md`,
  `sindr/README.md`, `sindr-devices/README.md`, `LIMITATIONS.md`, `AGENTS.md`,
  `sindr/src/lib.rs` and `sindr/src/spice/mod.rs`. Pre-release versions don't
  match a plain `"0.1"` requirement, so the snippets pin the full version.
- `CHANGELOG.md`: move the `[Unreleased]` entries into the new section and
  update the compare links at the bottom

Work for an upcoming release goes to a staging branch named after it (for
example `v0.1.0-alpha.7`) that already carries the bump. Branch from it and open
PRs into it. CI runs on those PRs, but the version-bump check only runs on the
staging branch's own PR into `main` when the release ships. Add CHANGELOG
entries under the staging version's section, and don't bump versions in
feature PRs.

Write CHANGELOG entries in the [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
style under `Added` / `Changed` / `Fixed` / `Removed`, and call out breaking
API changes.

## Commits and PRs

- One commit per logical change. Subject: `type(scope): what changed`,
  lowercase and imperative, with no trailing period.
- Breaking public-API changes are allowed during the alpha, but say so in the
  commit body and the CHANGELOG.

## License

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual-licensed under MIT or Apache-2.0, without any
additional terms or conditions.
