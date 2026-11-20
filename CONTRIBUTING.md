# Contributing to sindr

Thanks for helping. This page covers the local checks, the release rule CI
enforces, and where things live. [`AGENTS.md`](./AGENTS.md) goes deeper on the
architecture, how to add a circuit element or device model, and conventions.

## Toolchain

- **MSRV is Rust 1.89** (`rust-version` in both crate manifests). CI tests on
  1.89 and on stable, so don't use std or language features newer than 1.89.
- **Raise the MSRV only in a minor release** (0.x → 0.x+1 before 1.0), with
  a CHANGELOG line, never in a patch: `sindr = "0.2"` users get patches
  through `cargo update` and mustn't need a newer compiler for them.
- **Dependencies** must be licensed compatibly with MIT OR Apache-2.0 and free
  of security advisories: CI runs `cargo deny check` against `deny.toml`. To
  allow a new licence, add it there with the reason in the PR.
- **API breaks** follow the version: CI's semver job runs
  `cargo-semver-checks` against crates.io, so a patch release that breaks the
  public API fails it.
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
python3 scripts/gen_llms_full.py --check
```

- `--all-features` turns on the `spice` parser and its end-to-end tests.
  `--no-default-features` checks the build without `serde` and `examples`.
- Both crates have `#![deny(missing_docs)]`: every public item needs a doc
  comment.
- The code blocks in `sindr-devices/README.md` run as doctests, so a stale
  README example fails `cargo test`.
- `llms-full.txt` is generated from the READMEs, LIMITATIONS, doc comments
  and examples. If you changed any of them, run
  `python3 scripts/gen_llms_full.py` and commit the result.

## Tests

- Unit tests sit next to the code in `#[cfg(test)] mod tests`.
- Cross-cutting tests are in `sindr/tests/`. SPICE fixtures live in
  `sindr/tests/fixtures/`, and those tests need `--features spice`.
- Prefer checks against closed-form answers (a divider, an RC corner
  frequency, `1 − e^(−t/τ)`) over values copied from the current output.
- `sindr/tests/golden.rs` checks results against ngspice. To add a case or
  regenerate the references, see `sindr/tests/golden/README.md`. ngspice is
  needed only for regenerating, never to run the tests.
- Parsers and evaluators get `proptest` properties (see `spice/si.rs` and
  `spice/param_eval.rs`). If one fails, proptest writes the shrunk case to
  `sindr/proptest-regressions/`; commit that file with the fix.
- The `coverage` CI job reports per-file line coverage in the job summary,
  and fails if `stamp.rs`, `transient.rs`, `newton_raphson.rs`, `mna.rs` or
  the total drops below its floor in `scripts/coverage_floor.py`. Locally:

  ```bash
  cargo llvm-cov --workspace --all-features --summary-only > coverage.txt
  python3 scripts/coverage_floor.py coverage.txt
  ```

  Raise a floor when coverage rises. Lower one only with a reason in the PR.
- The SPICE parser has a fuzz target in `fuzz/` (cargo-fuzz, nightly). It
  feeds arbitrary text to `parse_str_with_options`, and for any deck that
  parses it validates the circuit and checks that `to_netlist`'s output
  parses again. Nothing may panic. Run it for five minutes before changing
  the parser:

  ```bash
  cargo install cargo-fuzz
  cd fuzz
  mkdir -p corpus/parse && cp seeds/parse/* corpus/parse/
  cargo +nightly fuzz run parse corpus/parse -- -dict=spice.dict -max_total_time=300
  ```

  A crash is saved under `fuzz/artifacts/parse/`. Fix it, then add the deck
  as a test in `sindr/tests/spice_errors.rs`. The seeds (one mode byte, 0
  for strict, then a deck) also run on stable in `fuzz_seed_decks_do_not_panic`,
  and the `fuzz` workflow runs the target for ten minutes every week.
- Benchmarks live in `sindr/benches/` (criterion). The `benchmarks` CI job
  runs `transient` and `newton` on a PR's base and on the PR, and fails if any
  case is more than 25 % slower. To check locally before pushing:

  ```bash
  cargo bench -p sindr --bench transient --bench newton -- --quick --save-baseline base
  # make the change
  cargo bench -p sindr --bench transient --bench newton -- --quick --save-baseline head
  python3 scripts/bench_compare.py base head
  ```

  When a change makes something deliberately slower, say why in the PR. If
  the numbers move a lot, update the table in `sindr/README.md`.

## Versions and the CHANGELOG

Versions are plain `0.x` until 1.0, with no prereleases. A release with a
breaking change bumps the minor version (`0.2.0` → `0.3.0`); one with only
fixes and additions bumps the patch (`0.2.0` → `0.2.1`). Cargo treats a 0.x
minor bump as breaking, so a caller on `sindr = "0.2"` is never broken by
`cargo update`.

Every PR to `main` must bump the version. The `version bumped` CI job fails
unless:

1. `version` in `sindr/Cargo.toml`, `sindr-core/Cargo.toml` and
   `sindr-devices/Cargo.toml` differs from `main` (a crate new on the
   branch is skipped), and
2. `CHANGELOG.md` has a `## [<new version>]` section.

The three crates share one version. A bump touches:

- `version` in `sindr/Cargo.toml`, `sindr-core/Cargo.toml` and
  `sindr-devices/Cargo.toml`
- the `sindr` / `sindr-core` / `sindr-devices` entries under `[workspace.dependencies]` in
  the root `Cargo.toml`, plus `Cargo.lock`
- the version in status lines: `README.md`, `LIMITATIONS.md`, `AGENTS.md`;
  and, on a minor bump, the `"0.x"` requirement in the install snippets of
  `README.md`, `sindr/README.md`, `sindr-core/README.md`,
  `sindr-devices/README.md`, `sindr/src/lib.rs` and `sindr/src/spice/mod.rs`
- `CHANGELOG.md`: move the `[Unreleased]` entries into the new section and
  update the compare links at the bottom

Work for an upcoming release goes to a staging branch named after it (for
example `v0.3.0`) that already carries the bump. Branch from it and open
PRs into it. CI runs on those PRs, but the version-bump check only runs on the
staging branch's own PR into `main` when the release ships. Add CHANGELOG
entries under the staging version's section, and don't bump versions in
feature PRs.

Write CHANGELOG entries in the [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
style under `Added` / `Changed` / `Fixed` / `Removed`, and call out breaking
API changes.

## Documentation and releases

[`docs/RELEASING.md`](./docs/RELEASING.md) lists where each capability is
documented (crate docs, READMEs, LIMITATIONS, `llms.txt`, examples), what a
PR that adds or changes a capability must update, and the per-release
checklist.

## Commits and PRs

- One commit per logical change. Subject: `type(scope): what changed`,
  lowercase and imperative, with no trailing period.
- Breaking public-API changes are allowed before 1.0, but say so in the
  commit body and the CHANGELOG, and the release they go into bumps the
  minor version.

## License

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual-licensed under MIT or Apache-2.0, without any
additional terms or conditions.
