# Releasing sindr

Every release keeps sindr's capabilities documented for two kinds of
reader: people reading rustdoc on docs.rs, and LLMs or agents generating
code against the API. Version mechanics (the staging branch, the bump, the
CHANGELOG section) are in [CONTRIBUTING.md](../CONTRIBUTING.md#versions-and-the-changelog).
This page is the documentation side.

## Where capabilities are documented

| Reader | Where they look | What has to be there |
|---|---|---|
| docs.rs | `sindr/src/lib.rs` crate docs | Quick start, the **Capabilities** tables (analyses with entry points, device models, convergence aids, SPICE subset), feature flags, link to LIMITATIONS |
| docs.rs | `sindr-core/src/lib.rs` crate docs | What the data model holds, quick example |
| docs.rs | `sindr-devices/src/lib.rs` crate docs | Device/model table, quick example |
| docs.rs | Every public item | A doc comment with units (V, A, Ω, F, H, s, K) and sign conventions; `# Errors` on fallible functions |
| GitHub, crates.io | `README.md`, `sindr/README.md`, `sindr-core/README.md`, `sindr-devices/README.md` | Install snippet, element reference, examples |
| Everyone | `LIMITATIONS.md` | What sindr does **not** do. This is what stops readers, and LLMs, assuming features that don't exist |
| Users and contributors | `ROADMAP.md` | The 1.0 gate and what comes next, in themes. No dates, no internal tracker ids, no names of users |
| LLMs | `llms.txt` | Short index: summary, key facts for generating code, links |
| LLMs | `llms-full.txt` | Generated: READMEs, LIMITATIONS, public API with docs, examples |
| Agents working on the repo | `AGENTS.md` | Orientation, "Common pitfalls when generating circuits via this API", "Things NOT to do" |
| Copy-paste learners | `sindr/examples/` | One small program per capability, named after it |

docs.rs builds with `--cfg docsrs` (see `[package.metadata.docs.rs]`), which
turns on `doc(cfg(...))`, so feature-gated items such as `sindr::spice` carry
a "crate feature" badge. A new feature-gated public item needs
`#[cfg_attr(docsrs, doc(cfg(feature = "...")))]` next to its `#[cfg]`.

## For every change that adds or changes a capability

In the same PR:

1. A doc comment with units and sign conventions on every new public item.
2. A runnable doctest on each new public entry point. `no_run` or `ignore`
   only with the reason stated next to the block (for example, it reads a
   file that doesn't exist where doctests run).
3. A CHANGELOG entry under the staging version, marked **Breaking** if it is.
4. A row in the crate-docs **Capabilities** tables, if it's a new analysis,
   device, convergence aid or SPICE card.
5. A LIMITATIONS update if the capability is partial, or if it removes a
   listed gap.
6. An example in `sindr/examples/` if it's a new capability rather than an
   extension of one.
7. Regenerate `llms-full.txt` (below).

## Release checklist

On the staging branch, before its PR into `main`:

1. **Versions.** Plain `0.x`, never a prerelease: a minor bump if the
   release has any breaking change, a patch bump otherwise. Do the bump as
   described in CONTRIBUTING, then check no snippet or status line still
   names the old version:
   `grep -rn "0\.2\.0\|\"0\.2\"" --include=*.md --include=*.rs .`
   (with the previous version in place of `0.1`).
2. **CHANGELOG.** The release section is complete, and breaking changes say
   what to change in calling code.
3. **Capabilities.** The crate-docs tables, `llms.txt` "Key facts" and
   LIMITATIONS agree with what shipped. Read the CHANGELOG's Added/Changed
   entries against each one.
4. **AGENTS.md.** Update "Common pitfalls when generating circuits via this
   API" and "Things NOT to do" for anything this release changed. `CLAUDE.md`
   only points to `AGENTS.md`; keep it that way.
5. **Accuracy.** `python3 scripts/accuracy_table.py` and commit
   `docs/ACCURACY.md` and the README table if they moved.
6. **llms-full.txt.** `python3 scripts/gen_llms_full.py`, commit the result.
   CI's `llms-full.txt` job fails if it's stale.
7. **Docs build.** These must pass (CI runs them too):

   ```bash
   cargo test --workspace --all-features      # includes doctests
   cargo test --workspace --no-default-features
   RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps --all-features
   RUSTDOCFLAGS="-D warnings --cfg docsrs" cargo +nightly doc --workspace --no-deps --all-features
   ```

8. **Read it.** Open `target/doc/sindr/index.html` and skim the front page,
   the `spice` badge and a few element docs as a docs.rs reader would. Skim
   `llms.txt` as an LLM would: could you write a working circuit from it?
9. **Publish** `sindr-devices` first, then `sindr-core`, then `sindr`. After docs.rs has built,
   check the published pages render and show the feature badges.
