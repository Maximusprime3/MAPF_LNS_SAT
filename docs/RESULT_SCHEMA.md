# LNS-SAT result document, version 1

`solve --output FILE` writes one UTF-8 JSON object, with a trailing newline. It
contains both the run manifest and, only on independently verified success, paths.
`verify --map MAP --scenario SCEN --solution FILE` consumes this object in a separate
process. It invokes the loader and `SolutionVerifier`, never LNS or SAT search.
There is no separate solution file to accidentally associate with another run.

## Required fields

Every object has exactly the fields specified here. Version 1 rejects unknown or
missing fields, duplicate JSON keys, unsupported versions, truncated/trailing data,
nonfinite numbers and invalid UTF-8. Integers must be integer JSON tokens: `1.0`,
`true` and `"1"` are not integer coordinates. General integers are signed 32-bit;
row-range endpoints are nonnegative signed 64-bit. Timing and the fallback fraction
are finite JSON numbers. Limits obey [CLI.md](CLI.md). Nulls are allowed only where
listed. Public input files must be regular files, at most 256 MiB each. The JSON
reader limits nesting to 128; it does not promise bounded memory for hostile input.

| Top-level field | Type and meaning |
| --- | --- |
| `schema_version` | Integer, exactly `1`. No implicit upgrade or downgrade. |
| `status` | `solved`, `exhausted`, `invalid_input`, `invalid_state`. |
| `diagnostic` | Human-readable string; wording is not a machine API. |
| `termination_reason` | `solved`, `search_exhausted`, `makespan_bound`, `wall_clock_limit`, `invalid_input`, `internal_failure`. `search_exhausted` includes missing initial paths and exhaustion of configured repair attempts; it is not a completeness claim. |
| `instance` | Selected input paths, hashes and exact scenario block (below). |
| `configuration` | Fully resolved solver values, including defaults (below). |
| `runtime` | `{ "total_ms": number, "solver_ms": number or null }`. Steady-clock milliseconds; see runtime boundaries in CLI.md. Null means LNS was not called. |
| `makespan` | Nonnegative integer number of moves for solved results, null otherwise. It includes represented goal waits, not merely the last non-wait move. |
| `verification` | `passed`, `failed`, `not_performed`. A timeout can have `passed` after verification but still carry no solution. Saved flags never authorize acceptance by `verify`. |
| `build` | Build-time provenance described below. |
| `solution` | Solution object for `solved`, null for every other status. |

`instance` has exactly `map_path`, `scenario_path` (absolute, lexically normalized
UTF-8 paths), `map_sha256`, `scenario_sha256` (64 lower-case hexadecimal characters,
or null if reading failed), `agents` (positive integer), `scenario_index`
(nonnegative integer), `first_row`, and `last_row_exclusive`.
The latter two describe **scenario entries**, excluding the optional version header
and empty lines; malformed entries are rejected rather than silently skipped.
`first_row = scenario_index * agents`; `last_row_exclusive = first_row + agents`.
The block must contain exactly the requested number of entries. No row-offset option
or reinterpretation of the legacy block index is introduced. Hashes cover whole
original files, including whitespace and unselected rows. The bytes are read once,
hashed, and parsed from the same immutable in-memory snapshot. Invalid map/scenario
syntax still yields hashes when reading succeeded. Solved documents require both hashes.

`configuration` has exactly `seed`, `variant`, `makespan_increment`,
`makespan_increase_limit`, `makespan_bound`, `lazy_iteration_limit`,
`full_map_fallback_threshold`, `wall_clock_limit_ms`, `log_level`, and `log`.
`makespan_bound` and `wall_clock_limit_ms` are null when unlimited; `log` is null when
no diagnostic file was requested, otherwise an absolute path. All other fields
are required and nonnull. `variant` is a canonical public name. This object can be
translated directly to the documented flat configuration keys; `instance` provides
map/scenario/count/index. Output destination and format do not affect search and
are chosen afresh for replay. The INI file itself must contain a complete valid
request; explicit CLI options override its values after validation.

## Solution conventions

`solution` has exactly:

- `coordinate_order`: the string `row,column`;
- `horizon`: a positive integer number of positions in **each** path;
- `agents`: an array of `{ "id": integer, "positions": [[row,column], ...] }`.

Origin is the top-left map cell `[0,0]`. Rows grow downward and columns rightward.
Moving AI scenario column fields are converted to this order by the existing loader.
Agent IDs are local to the selected block: exactly `0..agents-1`. Serialization orders
agents by ascending ID and positions by time; the verifier accepts any array order
but rejects duplicates. Object keys are emitted lexicographically. Runtime and
provenance can vary between runs, so entire documents are not byte-repeatable.

Timestep zero is the first position. A legal move changes one coordinate by one;
waiting repeats the same position. Only `.` and `G` terrain are walkable. Paths
must start at the corresponding scenario start and occupy the corresponding goal
at the final timestep. Earlier visits to and departures from a goal are allowed by
the existing verifier. After the common represented horizon, agents are understood
to remain at their goals. Makespan equals `horizon - 1`, including any represented
waits. Every represented timestep prohibits vertex sharing; consecutive timesteps
prohibit two agents traversing the same edge in opposite directions.

The verifier recomputes starts/goals, coverage, path geometry, common horizon,
makespan, vertex conflicts and edge swaps. It compares hashes and scenario-range
arithmetic. Recorded paths may be moved to another directory: **file bytes and
selection**, rather than recorded filenames, establish the instance. Stored flags,
build labels and makespan are not trusted as evidence of solution validity.
A checksum is an association/integrity check, not an authenticated signature.

Malformed structure/type/count/horizon, duplicate agents and unsupported schemas
return exit 2. Well-formed documents with no solution, incorrect ID coverage,
illegal paths, conflicts, incorrect recorded makespan, checksum mismatches, or
inconsistent selection return 4. Wrong selection that also makes input loading
impossible returns 2 with an input diagnostic. Checksum checks precede instance
parsing; a changed malformed map can therefore first report a checksum mismatch.
Neither outcome is an internal solver failure. Success returns 0; quiet suppresses
the success line but keeps invalid-submission diagnostics visible.

## Provenance and replay

`build` has exactly `revision`, `dirty`, `source_sha256`, `minisat`, `compiler`,
`configuration`, and `platform`.

- `revision` is the Git commit string and `dirty` a boolean sampled by an always-run
  build prerequisite. If Git metadata is unavailable (including a source archive),
  both are null. Untracked files count as dirty. The executable never consults the
  runtime working directory or Git checkout for this metadata.
- `source_sha256` identifies the supported source/header/app/CMake/vendor contents
  and root CMakeLists/Makefile. The generator sorts relative paths, concatenates
  `path:sha256(bytes)\n`, then hashes that manifest. Tests, docs, historical datasets
  and unsupported archive code are outside this build-source identity.
- `minisat` has `identity` (bundled MiniSAT with compatibility/deadline patches),
  `version` (null: no verified upstream revision identifier), and `source_sha256`
  (same manifest construction restricted to `third_party/minisat/`). This identifies
  the actual patched bundled source, not a guessed upstream version.
- `compiler` records CMake's compiler identity, version and path; `configuration`
  records selected build type, C++17, common/configuration compiler and linker
  flags, project warning policy, MiniSAT definition, thread dependency and toolchain
  settings; `platform` records CMake target system, version and processor.
  String fields and source hashes permit null for unavailable metadata in external
  producers, but this build populates them. The MiniSAT identity is always a string.

Keep the matching source snapshot (especially for a dirty build), input files,
compiler/toolchain and manifest to attempt replay. A hash identifies content but
does not contain or recover it; external libraries, OS scheduling and C++ random
implementation can affect replay. Source must not change while a build runs.
Archiving full environments and paper experiment inputs remains milestone 7.
This artifact captures all current public request/configuration values and the
required milestone 3 provenance; it does not promise cross-platform bitwise replay.

## Artifact lifecycle and limits

Only explicitly requested result/diagnostic files are written by this CLI. A result
is assembled completely before it is published through an exclusive private sibling
file, checked writes, fsync, checked close, and rename. See CLI.md for overwrite and
input-alias protection. If the directory cannot be modified, an old file may be
impossible to invalidate: exit 5 is authoritative and no new result is claimed.
No transactional guarantee spans diagnostic and JSON outputs together, and no
hard-crash/power-loss durability guarantee is made. A hard kill may leave a private
`.tmp.*` file; it is never a completed result at the requested destination.

Invalid invocations have no resolved request and do not emit documents. After valid
argument and destination preflight, invalid/unreadable instances, exhaustion and
internal failures emit manifests with no paths wherever the destination is writable.
These distinctions do not establish global completeness, optimality, exact deadline
latency, performance equivalence or reproduction of paper experiments.
