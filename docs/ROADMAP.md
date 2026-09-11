# LNS-SAT publication roadmap

## Purpose

This roadmap turns the current research implementation into a credible, reproducible,
and pleasant-to-use public research artifact. It refines `RELEASE_PLAN.md` after the
first correctness and repository-hygiene checkpoint on `cleanup/pseudo-agent`.

The first public release is a command-line research solver, not yet a stable reusable
C++ library. The implementation should remain faithful to the published LNS-SAT
algorithm while making the active pseudo-agent approach understandable and testable.

## Guiding principles

- Protect scientific behavior with deterministic tests before restructuring code.
- Keep behavior changes, file moves, and artifact removal in separate commits.
- Treat every reported solution as valid only after independent verification.
- Keep MiniSAT as the only supported backend, behind a small replaceable interface.
- Record all inputs needed to replay a run, including the seed and code revision.
- Keep bulk experimental outputs outside the source repository.
- Prefer a small polished interface over exposing every historical experiment option.

## Completed baseline

The cleanup branch already provides:

- an independent complete MAPF solution verifier with deterministic tests;
- final verification that fails closed instead of returning invalid paths;
- structured statuses for solved, exhausted, invalid input, and invalid state;
- stable CLI exit codes for those statuses;
- transactional rollback for speculative `CurrentSolution` changes;
- explicit agent IDs during initial MDD construction;
- typed and tested neighborhood variants;
- public LNS-SAT (`1,2,3,4,...`) as the default radius schedule;
- a MiniSAT contradiction/UNSAT regression test;
- a fixed-seed end-to-end benchmark verification runner;
- architecture, algorithm-variant, issue, and future-improvement documentation; and
- removal and ignore rules for several gigabytes of generated logs and plots.

These changes are preserved in commits `51911f6` and `5142ad6`.

## Current release blockers

The post-5.3 audit fixes and their bounded regression evidence are documented in
[`CORRECTNESS_CHECKPOINT.md`](CORRECTNESS_CHECKPOINT.md). These checks address
instance selection, zone termination, final-unit slack, grid shape, deadlines,
path assumptions, and the combined pseudo-agent repair path.

1. The supported Make build still has checked-in build products.
2. The CLI cannot write a stable solution/result format and still redirects global output.
3. Human-readable progress output remains entangled with the algorithm outside the
   SAT/CNF backend diagnostics.
4. Source, legacy experiments, binaries, caches, and duplicate drivers remain mixed.
5. Run manifests still lack revision, compiler/build mode, backend version, and input checksums.
6. The root README, citation metadata, third-party notices, and CI are not release-ready.

## Milestone 1: lock down pseudo-agent correctness

### Goal

Protect the active research contribution before changing APIs or moving source files.

### Work

- Add a `LocalZoneState` validator or equivalent standalone validation function.
- Check unique segment IDs and valid real-agent ownership.
- Check `segment_index_by_id`, `original_to_segments`, and pseudo-ID mappings in both
  directions.
- Check ordered, non-overlapping segment intervals for every original agent.
- Check that segment path length agrees with entry and exit times.
- Check segment endpoints, MDD identity, time bounds, and path continuity.
- Validate state after construction, after zone/time-window refresh, and before solving.
- Add deterministic cases for one visit, re-entry, starting inside the zone, ending
  inside the zone, and reaching the global goal inside the zone.
- Add expansion cases in which a real agent gains, loses, or merges local segments.
- Test final reassembly and prove that path portions outside authorized segments do not
  change.

### Exit condition

Every pseudo-agent transformation is covered by deterministic tests and invalid local
state is rejected as `InvalidState` before it reaches SAT solving or global integration.

### Expected size

Three to five focused commits.

## Milestone 2: finish waiting-slack and grid correctness

### Goal

Make successful and unsuccessful local repair attempts obey explicit invariants.

### Work

- Test waiting-budget conservation after a successful local solve.
- Test UNSAT, multiple retries, early validation failure, rollback, and commit paths.
- Assert preservation of common makespan, goal suffix, and unaffected real-agent paths.
- Prefer constructing candidate waiting adjustments without mutating `CurrentSolution`.
- Commit the candidate only after a successful local solve and integration check.
- Define one walkability rule or grid abstraction shared by loading, MDD construction,
  zone construction, full-map fallback, and verification.
- Test boundaries and every terrain symbol accepted by the problem loader.

### Exit condition

Slack is consumed exactly once on success, failed attempts leave no state change, and
all solver components agree on which cells are traversable.

### Expected size

Three to four focused commits.

## Milestone 3: create validated solver configuration

**Status: configuration and cooperative deadlines implemented; replay manifest
incomplete.** Milestone 6 owns the result-format/CLI work needed for the manifest
requirement below. This milestone's full exit condition remains open.

### Goal

Replace long parameter lists and hidden limits with one replayable configuration.

### Work

- Add `SolveRequest` for the input instance and selected scenario slice.
- Add `SolverConfig` for seed, variant, makespan limit, lazy iteration limit, full-map
  threshold, wall-clock limit, and log level.
- Validate all configuration before starting the algorithm.
- Convert configuration parsing failures into `InvalidInput` without uncaught exceptions.
- Remove numeric policy literals from algorithm code where they affect public behavior.
- Record resolved configuration, backend, revision, compiler/build mode, and input
  checksums in machine-readable result metadata.

### Exit condition

A run can be reproduced from one explicit request/configuration object and its result
manifest; no public search limit is hidden in implementation code.

### Expected size

Two to four focused commits.

## Milestone 4: isolate MiniSAT and remove probSAT

**Status: completed on `cleanup/pseudo-agent`.**

### Goal

Support MiniSAT only while making a future backend replacement local and obvious.

### Completed work

- Added a narrow incremental `SatSolver` interface for reset, clause addition, solving,
  typed assumptions, model extraction, and per-call statistics.
- Added explicit `Sat`, `Unsat`, and `Error` results and preserved those distinctions
  through lazy, waiting, local-zone, and top-level solve outcomes.
- Isolated MiniSAT types and ownership inside one adapter implementation.
- The post-5.3 checkpoint adds `Interrupted` for deadlines, fixes literal-ID and
  absolute-time assumptions, and tests the real producer/adapter path.
- Injected the backend into lazy SAT solving and added a fake-backed protocol suite.
- Removed public solver selection from positional/configuration/batch inputs.
- Removed probSAT APIs from shared CNF/manager code and probSAT dependencies from the
  supported Make build and static archive. Historical backend sources remain outside the
  supported artifact for later repository organization.
- Routed MiniSAT clause diagnostics through a sink enabled only for `LogLevel::Debug` and
  removed unconditional CNF variable/model/stack dumps.

### Exit evidence

The supported build graph, archive member table, and archive symbols are checked by
`test_supported_build_dependencies.sh`. The incremental protocol tests cover suffix-only
clause loading, assumptions, exact UNSAT reset/retry, immediate error propagation, model
extraction, reset independence, and both-call statistics. The fixed-seed empty-8x8
verification still returns the baseline paths at makespan 6.

### Exit condition

Achieved: adding another backend now requires one `SatSolver` adapter and factory choice,
without changes to LNS, neighborhood, MDD, verifier, or CLI code.

### Delivered size

Six focused commits.

## Milestone 5: establish the build and source tree

**Milestone 5.2 completed:** the supported Make build no longer requires `-fpermissive`.
The local MiniSAT default-argument compatibility patch is documented in
[`lns_clean/BUILD.md`](../lns_clean/BUILD.md#local-minisat-compatibility-patch).

**Milestone 5.3 completed:** a root out-of-tree CMake build now mirrors the supported
Make sources, with separate bundled MiniSAT and project core targets, `lns-sat`, the
supported `run_experiments` batch runner, and the independent verification runner.
CTest covers the 14 existing C++ tests, both shell checks against CMake artifacts, and a
30-second fixed-seed smoke test (empty-8-8-even-1, four agents, index 0, seed 42,
verified makespan 8). Project warnings remain target-local and separate from bundled
MiniSAT. The Make workflow remains supported. See
[`lns_clean/BUILD.md`](../lns_clean/BUILD.md) for commands and target details.
Source moves, duplicate-runner consolidation, tracked-artifact removal, and Milestone
5.4 remain deferred.

### Goal

Provide one obvious clean build and one supported implementation.

### Work

- Add a root CMake build before moving files. **Completed in 5.3.**
- Define targets for the solver core, `lns-sat` executable, unit tests, and smoke test.
  **Completed in 5.3**, including the batch and independent verification runners.
- Build out of tree; removal of the `-fpermissive` requirement is completed in 5.2.
- Keep third-party compiler warnings separate from project warnings. **Completed for
  CMake in 5.3.**
- Consolidate the duplicate batch experiment runners.
- Remove tracked libraries, executables, object files, caches, notebook checkpoints,
  and `Zone.Identifier` files.
- Move MiniSAT under an explicit `third_party/` boundary with its license.
- Move the supported solver into `src/`, internal headers into an appropriate include
  tree, the executable into `app/`, and fixtures into `tests/fixtures/`.
- Archive or remove the older `lns/` implementation in a dedicated commit.
- Keep analysis scripts outside all solver build dependencies.

### Target build

    cmake -S . -B build
    cmake --build build
    ctest --test-dir build --output-on-failure

### Exit condition

A clean checkout builds and tests through the three documented commands without
checked-in build products or legacy solver dependencies.

### Expected size

Four to six focused commits.

## Milestone 6: build the public CLI and result formats

### Goal

Make the solver usable by a researcher who did not write it.

### Target interface

    lns-sat solve --map MAP --scenario SCEN --agents N [options]
    lns-sat verify --map MAP --scenario SCEN --solution FILE
    lns-sat --version

### Work

- Support scenario offset, seed, variant, time/makespan limits, output path, result
  format, and log level.
- Keep MiniSAT implicit while it is the only backend.
- Validate unknown flags, variants, numeric values, and configuration keys strictly.
- Print concise human-readable progress to stderr.
- Write solutions and machine-readable result manifests only to explicit outputs.
- Remove global `std::cout` redirection.
- Define and document a stable solution file format.
- Include status, makespan, runtime, seed, variant, revision, configuration, input
  checksums, and verification status in every result.
- Expose the independent verifier through the `verify` subcommand.
- Add `--help`, `--version`, CLI error tests, and one copy-paste smoke test.

### Exit condition

The README quick start produces a saved, verifier-approved solution on a tiny included
instance, and scripts can reliably distinguish every process outcome.

### Expected size

Three to five focused commits.

## Milestone 7: package reproducibility inputs and results

### Goal

Reproduce a small published result locally without storing bulk experiment output in Git.

### Work

- Keep one or two tiny maps/scenarios and expected results in the source repository.
- Record benchmark source, redistribution terms, attribution, and checksums.
- Provide scripts to download and validate larger benchmark inputs when appropriate.
- Audit the current map and scenario collections and retain only justified release data.
- Define the exact configurations that reproduce the selected paper result.
- Create an environment/build manifest.
- Package exact configurations, checksummed inputs, raw result tables, and required
  plotting scripts in a versioned archival artifact.
- Publish the full artifact through Zenodo or an equivalent DOI-bearing archive.

### Exit condition

One small result is reproducible from the source repository, and the complete paper
artifact can be downloaded and verified independently by checksum.

## Milestone 8: publication polish, CI, and release

### Goal

Turn the verified solver into a professional public research project.

### Work

- Replace the root README with project purpose, algorithm overview, quick start, CLI
  examples, reproducibility instructions, limitations, and citation.
- Add `CITATION.cff` and the published ICAART 2026 BibTeX entry.
- Add author and contact information.
- Audit the root license against MiniSAT, datasets, and retained third-party material.
- Add `THIRD_PARTY_NOTICES.md` and document any MiniSAT modifications.
- Add `CONTRIBUTING.md`, formatting rules, test expectations, and review conventions.
- Add GitHub Actions for build, unit tests, integration smoke solve, verifier, and README
  quick start.
- Add sanitizer coverage for deterministic fixtures.
- Decide whether to rewrite the approximately 387 MB historical Git data or publish a
  clean-history release snapshot after the source tree stabilizes.
- Tag the first release and link its source, paper, and reproducibility DOI.

### Exit condition

A tagged release builds from scratch, passes all tests and verification, provides correct
citation/licensing information, and links to a complete reproducibility artifact.

## Recommended next checkpoint

The original pseudo-agent/slack/grid checkpoint and the post-5.3 audit fixes now
have regression coverage; see `CORRECTNESS_CHECKPOINT.md` for before/after evidence
and remaining limits. Continue Milestone 5's source/artifact organization and
Milestone 6's CLI/result work, keeping correctness fixes separate from file moves.
Run the new regressions, including the independent tiny oracle, after source moves.

## Deliberately deferred improvements

The following ideas are valuable but are not first-release blockers:

- replacing initial MDD construction with direct single-path planning;
- exploiting initial MDD alternatives to reduce congestion;
- promising a stable reusable C++ library API;
- rewriting all historical analysis notebooks;
- supporting additional SAT backends;
- broad performance optimization before profiling the stable public build; and
- large-scale source renaming before pseudo-agent and slack behavior is protected.

See `FUTURE_IMPROVEMENTS.md` for the initial-path/MDD design discussion.

## Definition of done for the first public release

- Clean out-of-tree build succeeds from a fresh checkout.
- Default CLI behavior is published LNS-SAT with radius sequence `1,2,3,...`.
- All four documented neighborhood schedules are tested.
- Pseudo-agent segmentation, refresh, and reassembly are deterministically tested.
- Waiting-budget and grid invariants are enforced.
- Only MiniSAT is supported through a replaceable internal boundary.
- Every successful solve is independently verified.
- A solution and complete replay manifest can be written by the CLI.
- One tiny example and one documented paper-oriented workflow are reproducible.
- Bulk results live in a checksummed archival artifact rather than the source tree.
- README, citation, licensing, author/contact, CI, and release notes are complete.

## Scope estimate

The remaining first-release work is approximately 20 to 30 reviewable commits. A rough
planning estimate is three to five focused weeks, with the greatest uncertainty in
pseudo-agent edge cases and separating MiniSAT from the historical solver manager.
