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

### Post-5.3 audit follow-up

`CORRECTNESS_CHECKPOINT.md` records reproduced failures and regression evidence for
exact agent counts, malformed scenario rows and map shapes, saturated disconnected
zones, the final slack retry, deadline propagation, path assumptions, and combined
pseudo-agent/slack/refresh/reassembly. These are addressed by the checkpoint;
its bounded tests do not prove arbitrary-instance completeness.

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

### Waiting-time accounting (resolved on cleanup branch)

`CurrentSolutionTransaction` restores paths and waiting budgets on every unsuccessful
exit, including early validation failures. Deterministic conservation tests cover a
successful repair, formula UNSAT, multiple retries, rollback, and commit, including goal
suffixes and unaffected paths.

### Pseudo-agent refresh and reassembly (resolved on cleanup branch)

`LocalZoneState` validation checks stable segment ownership, indices, ordering, time bounds,
paths, MDD identities, and bidirectional pseudo-agent mappings. Deterministic tests cover
one visit, re-entry, boundary starts/ends, goal-in-zone behavior, expansion refresh, and
final reassembly without modifying unauthorized path portions.

### Walkability rules (resolved on cleanup branch)

`Grid.h` provides the shared walkability rule used by loading, MDD construction, zone
construction, full-map fallback, and final verification. Boundary and supported terrain
symbol behavior is covered by deterministic characterization tests.

## P1: algorithm behavior and reproducibility

### Cooperative deadline return latency

The shared deadline prevents accepting late solutions and distinguishes interruption
from UNSAT. It does not guarantee return by the budget boundary. The 2026-09-12
Debug comparison observed 274–298 ms returns for a 100 ms limit on a valid 32-agent
case, always `Exhausted` with no paths. Use an external process timeout when needed;
do not use the configured budget as the measured runtime in experimental results.
See [VERIFICATION_SIGNOFF.md](VERIFICATION_SIGNOFF.md) for methods and measurements.
Tighter cancellation granularity can be separately profiled if required for experiments.

### Public default variant (resolved on cleanup branch)

Neighborhood growth now uses a typed policy, and the default is the public LNS-SAT radius
sequence `1,2,3,4,...`. The three paper variants remain explicit selectable alternatives,
and solver and batch logs record the selected public name.

### Solver selection (resolved on cleanup branch)

MiniSAT is now the only supported and implicit backend. Positional/configuration/batch
solver selection is rejected, and lazy solving depends on the typed incremental
`SatSolver` contract rather than MiniSAT or probSAT types. The supported build graph and
static archive are checked for probSAT dependencies. Historical backend-only source files
remain outside the supported artifact pending the source-tree cleanup milestone.

### Search limits are configured; result manifests remain incomplete

`SolverConfig` names and validates makespan increment/limit, lazy-SAT iteration limit,
full-map fallback threshold, wall-clock limit, seed, variant, and log level. A stable
machine-readable result manifest recording all resolved values is still missing.

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

### Structured solver API (mostly resolved on cleanup branch)

`LNS(...)` now accepts typed `SolveRequest`/`SolverConfig` values and returns status, paths,
makespan, runtime, seed, and a diagnostic message. Lazy SAT uses a typed backend boundary.
The remaining API work is to separate progress logging from the algorithm and present a
stable reusable-library surface.

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
