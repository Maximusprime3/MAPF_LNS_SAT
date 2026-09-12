# Known issues and cleanup risks

## Purpose

This is a working audit of the supported code in `src/` and `include/lnssat/`. It exists to keep
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
are retained under `archive/`, outside the supported artifact.

### Public run manifests (resolved in milestone 6)

The version 1 result records every resolved request/configuration value, exact input
hashes and scenario block, measured runtime, verified paths and build-time provenance.
See [RESULT_SCHEMA.md](RESULT_SCHEMA.md). The additive makespan limit retains its
legacy meaning; an optional absolute bound is separate.

### Initial path sampling is reproducible only with complete run metadata

All of these fields are now recorded in the public manifest, with explicit nulls for
unavailable source-archive Git metadata and an unidentified upstream MiniSAT version
(the exact bundled source is hashed). Retain matching inputs/source/toolchain: hashes
cannot reconstruct a dirty source tree or an unavailable input. Cross-platform bitwise
replay is not promised.

### Metrics may not describe the intended event

Experiment logging was added while the algorithm was changing. Counters and timings need
definitions and tests, especially around nested makespan, zone, waiting, and lazy-SAT
attempts. The batch runner has been consolidated; metric definitions and output
separation remain later work.

## P2: repository and release engineering

### Publication documentation remains incomplete

The root README now points to the supported build and layout. Citation/contact
metadata, release notices, and broader publication polish remain Milestone 8 work.

### Historical research data still needs an archival plan

Milestone 5 removes tracked executables, libraries, objects, bytecode caches,
notebook checkpoints, Zone.Identifier files, and nested Git backup metadata.
Primary notebooks, plots, CSV results, datasets, and intentional verification logs
remain preserved. Their provenance and archival selection belong to Milestone 7;
do not delete historical result data before verifying its backup/archive location.

### Source/build organization (resolved in Milestone 5)

`src/`, `include/lnssat/`, `app/`, `tests/`, and `third_party/minisat/` contain the
supported implementation. The old solver/backend experiments are in `archive/`.
Root CMake and the Make wrapper build from sources into selected output directories;
no tracked libraries or executables are required. See [BUILD.md](BUILD.md).

### Structured solver API (mostly resolved on cleanup branch)

`LNS(...)` now accepts typed `SolveRequest`/`SolverConfig` values and returns status, paths,
makespan, runtime, seed, and a diagnostic message. Lazy SAT uses a typed backend boundary.
The remaining API work is to separate progress logging from the algorithm and present a
stable reusable-library surface.

### Public output and logging (resolved in milestone 6)

Supported progress uses scoped verbosity and stderr sinks; there is no global stdout
redirection. Maps, paths and backend details use debug level. Results and diagnostic
files require explicit destinations; internal experimental CSV logging requires an
explicit directory and remains outside the public schema. Nested experimental metrics
still need the previously planned event-definition audit.

### Artifact and resource limits

Public input files are regular UTF-8-labeled paths with a 256 MiB per-file cap; JSON
has a 128-level nesting limit. The parser is not a hardened adversarial-input service.
Atomic publication detects ordinary write/flush/close failures, but does not provide
power-loss durability or protection against malicious concurrent directory changes.
Hard process termination can leave a private temporary file. Exit status is authoritative
when an unwritable destination prevents removal of an old artifact. No hard real-time
stopping guarantee is introduced; input hashing/loading and cleanup consume measured time.

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
