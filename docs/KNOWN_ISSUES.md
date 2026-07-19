# Known issues and cleanup risks

## Purpose

This is a working audit of the code currently centered in `lns_clean/`. It exists to keep
documentation honest and to prevent a formatting refactor from hiding behavioral changes.
Items should move to tests or closed issues as they are resolved.

Severity labels mean:

- **P0**: can invalidate a reported solution or associate data with the wrong agent;
- **P1**: can change algorithm behavior, reproducibility, or supported interfaces;
- **P2**: release engineering, maintainability, or usability debt.

## P0: correctness and validation

### Final verification failure propagation (resolved on cleanup branch)

`SolutionVerifier` now independently checks expected agent IDs, common path horizon, path
geometry, start/goal positions, vertex conflicts, and edge-swap conflicts. The existing
`VerificationHelpers` API delegates to it.

`LNS.cpp` now rejects a failed final verification. `LNSResult` distinguishes solved,
bounded-search exhaustion, invalid input, and invalid internal state without relying on an
empty path map. The CLI maps those statuses to stable exit codes 0, 1, 2, and 3 respectively.
The SAT, slack, and zone layers use the same status vocabulary, so ordinary exhaustion can
expand the search while invalid input or internal state propagates upward immediately.

### Initial MDD agent identity (resolved on cleanup branch)

`create_mdds_with_waiting_time` now returns ordered `(agent_id, mdd)` records instead of
bare MDD pointers. `LNS.cpp` samples paths using the stored ID and rejects incomplete MDD
construction, so a skipped agent can no longer relabel later paths. A deterministic
regression covers successful IDs 0 and 2 with agent 1 deliberately omitted.

### Waiting-time accounting needs an invariant-based audit

Candidate segment-MDD construction can mutate the global `CurrentSolution` waiting-time
budget. `CurrentSolutionTransaction` now guarantees that every unsuccessful exit restores
paths and waiting budgets, including early validation returns, and a deterministic test
covers rollback and commit. It is still difficult to prove that successful attempts
consume slack exactly once.

Additional tests should assert conservation of each agent's path length, goal suffix, and
remaining waiting budget across real success, UNSAT, and multi-retry scenarios. Prefer
computing candidate state without mutation where practical, then committing it only after
a successful local solve.

### Pseudo-agent refresh and reassembly need focused tests

Zone expansion can change how many times a real path crosses the zone boundary. Stable
pseudo IDs, segment order, entry/exit times, MDDs, and collision metadata must all be
refreshed together. Tests are needed for an agent that:

- enters once;
- leaves and re-enters;
- begins or ends inside the zone;
- reaches its global goal inside the zone; and
- gains or loses a segment after spatial/time-window expansion.

The final splice must prove that changes remain inside the authorized segments and that
the reconstructed real-agent path is continuous.

### Walkability rules are not centralized

Different code paths infer walkability independently. For example, the full-map threshold
in `Solve_Local_Zone.cpp` counts only `'.'`, while Moving AI map handling may admit other
terrain symbols. This can produce inconsistent masks, thresholds, MDDs, and verification.
Define one `is_walkable(cell)` rule and use it everywhere.

## P1: algorithm behavior and reproducibility

### Public default variant (resolved on cleanup branch)

Neighborhood growth now uses a typed policy, and the default is the public LNS-SAT radius
sequence `1,2,3,4,...`. The three paper variants remain explicit selectable alternatives,
and solver and batch logs record the selected public name.

### Solver selection is misleading

The CLI and batch runner advertise `minisat` and `probsat`, and any positional solver name
other than `minisat` becomes a false boolean. The local implementation is not a cleanly
swappable two-backend system. The intended release should:

1. validate CLI values strictly;
2. support MiniSAT only;
3. define a narrow backend interface; and
4. make adding a different solver an isolated implementation task.

probSAT source, binaries, options, and build rules should not remain in the supported
artifact merely because historical experiments used them.

### Important limits and fallback thresholds are hardcoded

The maximum makespan increase, initial radius, expansion policy, full-map threshold, and
lazy-SAT iteration limits are embedded in implementation code or helper defaults. They
need named configuration fields, validation, documented defaults, and result metadata.

### Initial path sampling is reproducible only with complete run metadata

The seed is recorded, but exact replay also needs the code revision, variant, solver
version, compiler/build mode, map and scenario checksums, selected scenario offset, number
of agents, and all limits. A machine-readable run manifest should accompany results.

### Metrics may not describe the intended event

Experiment logging was added while the algorithm was changing. Counters and timings need
definitions and tests, especially around nested makespan, zone, waiting, and lazy-SAT
attempts. Duplicate includes and duplicated experiment runners are signs that this path
has not yet been consolidated.

## P2: repository and release engineering

### The root README describes an older project

It presents the repository primarily as an MDD constructor, documents probSAT as a
supported solver, and contains citation/contact TODOs. It should be replaced only after
the build and minimal CLI are stable enough for its commands to be tested.

### Generated and machine-local artifacts are committed

The repository contains large logs, plots, CSV results, notebooks and checkpoints,
compiled objects/libraries/executables, Python caches, and a backup of nested Git metadata.
These make cloning expensive and obscure the source artifact. They should be classified
before removal:

- source and small deterministic examples remain in Git;
- reproducibility inputs receive provenance and licensing notes;
- selected paper results move to a versioned archival release (for example Zenodo);
- regenerated plots and bulk logs stay out of the source repository; and
- binaries, object files, caches, and nested Git internals are removed from version
  control and covered by `.gitignore`.

Do not delete historical result data until it has been backed up and its archival location
has been verified.

### Build products and source are mixed

`lns_clean/BUILD.md` describes a static archive, while checked-in executables and objects
suggest several ad-hoc build paths. Establish one clean out-of-tree build, initially for
Linux with a documented C++ standard and MiniSAT dependency.

### Structured solver API (partially resolved on cleanup branch)

`LNS(...)` now returns status, paths, makespan, runtime, seed, and a diagnostic message.
The remaining API work is to separate progress logging from the algorithm and stabilize
configuration types before presenting this as a reusable library interface.

### Output and logging are entangled with the algorithm

The core prints the entire map, all agents, progress messages, and some final paths.
Configuration can redirect `std::cout` wholesale to a file. Replace this with explicit
verbosity levels and separate human-readable diagnostics from machine-readable results.

### Analysis code is not a release blocker

The current analysis scripts and notebooks are inconsistent and are intentionally outside
the first cleanup scope. Preserve them until result formats are stabilized; then select or
rewrite only the analyses needed to reproduce paper tables and figures.

## Questions to resolve before publication

- Preferred public author contact (email or project URL).
- Whether the under-review extended paper may be named publicly before acceptance.
- Which exact experiment configurations and result tables constitute the reproducibility
  package.
- Whether bundled MiniSAT should remain vendored or be fetched as a pinned dependency.
- Which Moving AI benchmark subset may be redistributed, with what attribution files.
