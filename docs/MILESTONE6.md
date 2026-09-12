# Milestone 6: public CLI and result formats

Date: 2026-09-12. Examined repository:
`/home/max/Documents/PhD/MAPF/LNS/MAPF_LNS_SAT_cleanup`.
Baseline: **91fd9e504c1f13d5cd1ac3bfa71eacbf2866684a**, clean working tree.
The requested baseline was verified, not assumed. Project `sources/` references
were not modified. Work was staged in a separate local checkout; unrelated files
in the requested repository are protected by per-file baseline checks on application.
Changes are left uncommitted for review. No release or experiment publication occurs.

## Observable contracts and design decisions

[CLI.md](CLI.md) was written before behavioral implementation. [RESULT_SCHEMA.md](RESULT_SCHEMA.md)
defines the closed version 1 schema, nullability, coordinates, selected scenario
block, verification and artifact lifecycle. JSON is sufficient for the initial
public format; no additional serialization dependency is introduced. Its dedicated
parser derives from the existing batch parser but adds strict integer preservation,
duplicate rejection, UTF-8/surrogate handling, bounded nesting and deterministic
serialization. The supported batch parser itself is unchanged.

The public commands are `solve`, `verify`, `--help` and `--version`. Named options
and complete flat INI configurations reject unknown/duplicate/malformed values;
CLI overrides valid INI values, independently of argument order. Positional and
`--config` legacy calls remain, preserving batch selection, seed and external-timeout
behavior. The batch runner's implementation is unchanged.

`scenario_index` remains a **block index**, with explicit `[index*N,(index+1)*N)`
provenance. `makespan_increase_limit` remains a maximum additive delta, not a count;
a separate optional absolute `makespan_bound` limits represented moves. Default
schedules, seed use, pseudo-agent transformations, waiting budgets and SAT calls are
preserved. The outer loop uses a wider additive counter and rejects unrepresentable
horizons before signed overflow; the verifier also widens coordinate-distance
arithmetic for malicious extreme submitted coordinates.

Input bytes are read once, hashed and parsed from the same in-memory snapshot.
The public adapter uses an LNS overload taking this parsed snapshot and the same
cooperative deadline. On success, the public adapter independently checks paths
again before saving them. The standalone verifier does not invoke the solve callback
or SAT. It ignores stored verification flags and recomputes makespan and validity.

Logging uses scoped per-thread sinks without changing global stream buffers.
Human progress goes to stderr; map/path/backend detail is debug-only. Quiet keeps
errors. `log=FILE` requests a checked diagnostic file explicitly. ExperimentLogger
has no default directory and requires explicit internal opt-in; historical nested
metrics remain outside the public schema and their event definitions are unchanged.

`LNSResult::runtime_ms` now measures the complete LNS call on every return, including
loading, final verification and cleanup. The public result separately records
`solver_ms` and `total_ms`; total includes input acquisition/hashing and public
verification but excludes serialization/publication. Two old tests asserting
exactly zero on early returns were updated to assert positive measured elapsed time.
The test-only delayed callback verifies elapsed time is not the requested budget.
Deadlines remain cooperative; no exact stopping latency is claimed.

Build-time metadata is refreshed by an always-run CMake prerequisite, with no Git
lookup at executable runtime. Dirty builds and archives also receive a supported
source checksum; the exact bundled MiniSAT tree is hashed. Unknown archive Git
metadata and the unverified upstream MiniSAT version are null. Compiler, selected
configuration and effective flags/toolchain settings, and target platform are recorded.
This meets milestone 3's manifest requirement. Retaining matching source/input files
and an environment remains necessary to replay; checksums cannot reconstruct them.

## Four focused review groups

| Group | Files and review intent |
| --- | --- |
| 1. Public interface and input snapshot | `app/main_clean_lns.cpp`, `app/PublicCli.cpp`, configuration headers/implementation, loader and manager stream overloads. Review strict options, legacy compatibility, path/I/O rules and byte identity. |
| 2. Result schema, verifier and provenance | `PublicResult.*`, `Json.h`, `Sha256.*`, CMake provenance generator/targets, small verifier arithmetic fix. Review fail-closed success, parsing, independence and build identity. |
| 3. Diagnostics and measured runtime | `Logging.h`, mechanical stream substitutions in core/header/MDD sources, `ExperimentLogger.*`, `LNS.*`. Review that search logic remains unchanged except the explicit optional bound/overflow guard. |
| 4. Contracts and regression evidence | New public format/CLI/provenance tests and test-only callback driver; existing early-runtime expectations and dependency-path false-positive fix; README, build/CLI/schema/issues/roadmap/example updates and this evidence. |

[Changed-file inventory](milestone6/changed-files.txt) enumerates the delivered files.
The groups are review guidance; no commits were manufactured solely to group changes.

## Requirement-to-test matrix

| Requirement / falsifiable claim | Evidence and observed outcome |
| --- | --- |
| Actual baseline and pre-existing failures | Fresh baseline Debug build: 20/21 CTest passed. Only dependency-path inspection failed because its case-insensitive regex matched the repository's **parent** `/LNS/`. With only the source-root prefix normalized, the original script and unchanged archives passed. This was not a solver failure. |
| Help/version, strict CLI and config | `test_public_cli`: missing options/values, unsupported format/backend/variant, duplicates/unknown keys, numeric syntax/range/overflow/nonfinite values, CLI-over-config precedence. Help/version stdout flush failures and relocated executable version also checked. |
| Scenario block selection and row sufficiency | Public CLI nonzero index with recorded rows 4–8, independent verify, oversized index and insufficient 32-row fixture rejection; legacy loader audit retained. |
| Solve outcomes 0/1/2/3 | Actual tiny solve; absolute-bound exhaustion; missing/malformed selection; test-only thrown backend failure and corrupt success callback. Internal exceptions return 3 and never publish paths; callback controls are absent from the shipped entrypoint. |
| Absolute bound vs additive policies | Bound 7 rejects the four-agent makespan-8 fixture; bound 8 with additive step 3 and maximum increase 0 succeeds at base 8. No default schedule reinterpretation. |
| JSON round-trip, escaping, deterministic order | `test_public_format`: empty/abc/million-a SHA-256 vectors, control bytes, escaped/raw Unicode and surrogate pairs, malformed/deep JSON; process tests check ascending IDs and accept reordered input arrays. |
| Closed schema and malformed submissions | Public CLI removes every required top-level field and rejects version/type/key/truncation/count/horizon/coordinate/duplicate-ID tampering. Unsupported versions and duplicate JSON keys return 2. |
| Independent saved-solution verification | Separate solve/save/verify processes; the test-only executable whose solve callback throws still verifies a good saved result. No search invocation is needed. |
| Semantic invalid submissions return 4 | Illegal/swapped/extreme coordinates, wrong IDs, checksum mismatch, inconsistent/reselected instance, incorrect makespan; explicit two-agent vertex and edge-swap fixtures. Stored `verification=failed` does not prevent independent acceptance of valid paths. |
| Quiet/info/debug and declared files only | Quiet has empty stdout/stderr on successful tiny solve; info omits full map; debug includes it. Directory inventory is unchanged without explicit output; no `logs/` directory. Legacy explicit diagnostic file works. |
| Artifact failure and stale success | Old solved documents replaced by exhausted/invalid/internal results with null solution; direct/canonical/link input aliases and nonregular/missing-parent destinations rejected. RLIMIT_FSIZE causes an actual write failure, exit 5, and no completed or temporary result. Flush/close/rename return values are checked; distinct kernel flush/close failures were not injected. |
| Input hashes bind consumed bytes | Test callback changes the map file **after loading**; solved manifest hashes the original bytes, verifies against the restored bytes, and rejects changed bytes. Both reads are attempted so available hashes survive one unreadable input. |
| Build provenance and archive fallback | Generator tests cover no-Git/null metadata, clean/dirty/changed commits, changed source hashes and no unnecessary header rewrite. Full source-only Make build reports null Git metadata, real compiler/configuration/platform and backend/source hashes. |
| Timing semantics and timeout outcomes | Delayed test callback with a 1 ms limit measures at least 10 ms, emits `wall_clock_limit`, no paths; existing real/fake backend deadline tests retain cooperative cancellation coverage. Early-return LNS timers are positive. |
| Algorithm and backend regressions | Full **24/24 CTest** in Debug and source-archive Make `-O2`, including **528 fixed-horizon oracle cases**, deadlines, pseudo-agents, waiting budgets, grid, typed backend and batch path/rejection tests. |
| Fixed-seed behavior preservation | **16 paired comparisons**: 14 solved, one exhausted and one invalid input. Status, independently verified validity, makespan and every returned path match exactly. No wall-clock search limits; 15-second external guards. |
| README quick start | Executed solve then independent verify: both exit 0, four agents, makespan 8. |

The public process harness reports **145 checked invocations**, plus two direct
stdout-failure invocations; it does not claim exhaustive argument/schema coverage.
All tests use temporary/build working directories and bounded process timeouts.
No new warning remains in the new public implementation; existing research-source
compiler warnings are retained rather than mixed into this milestone.

## Commands and retained observations

Observed environment: GCC 13.3.0, Linux x86_64, CMake 3.31.6. `cmake`/`ctest` were
not on PATH, so the existing local installation was used. No dependency downloads.
Exact baseline/Debug/Make and comparison commands are in
[commands.sh](milestone6/commands.sh). Retained compact transcripts:

- [baseline CTest](milestone6/baseline-ctest.txt), [baseline dependency diagnosis](milestone6/baseline-dependency-confirmation.txt);
- [Debug CTest](milestone6/debug-ctest.txt), [source-archive Make CTest](milestone6/make-ctest.txt);
- [paired outcomes/path fingerprints](milestone6/comparisons.json), [probe](milestone6/comparison.cpp), [comparison driver](milestone6/compare.py);
- [build provenance observations](milestone6/builds.json).

For the six default-variant eight-agent pairs, makespans are 8 on empty-8-8,
46 on random-32-32-10, and 69 on maze-32-32-2, for seeds 42 and 7 in each case.
Six alternate-variant pairs and two nonzero-block pairs also match at makespan 8.
One disconnected single-agent case is exhausted and requesting 33 rows from the
32-row fixture is invalid input on both revisions. Path fingerprints include
status/makespan/validity and all paths sorted by agent ID; timings are deliberately
excluded. These are bounded behavior checks, not statistical performance evidence.

## Conclusions and explicit limitations

**Public interface correctness:** the documented command, configuration, exit-code,
logging and artifact contracts have passing focused process evidence. The dependency
inspection's pre-existing parent-path false positive is fixed without removing its
backend/archive-symbol checks. Invalid invocations leave outputs untouched; when a
destination cannot be modified, exit 5 is authoritative and an old file may remain.

**Saved-solution validity:** successes pass the independent verifier before saving,
and a separate process recomputes validity. Adversarial geometry, identities, conflicts,
selection, checksums and makespan are rejected in the executed cases. This does not
prove that the JSON parser is a hardened untrusted-input service or cover all corruptions.

**Provenance and replay coverage:** the artifact captures current public inputs,
selection, limits/defaults, seed/variant, exact byte checksums and build identity.
Source archives are labeled honestly. Matching dirty source contents, input files,
compiler/libraries and environment must still be retained. Concurrent source edits
during a build, hostile directory races and power-loss durability are not supported.
Hard kills may leave private `.tmp.*` files; cooperative deadlines can overshoot.

**Regression evidence:** all 24 tests and 16 paired comparisons pass, preserving the
528-case oracle and prior transformation/deadline tests. No global solver completeness,
optimality, cross-platform bitwise replay, performance equivalence or reproduction
of paper results is established by this milestone. Broad experiments, nested metrics
redesign and release publication remain out of scope.
