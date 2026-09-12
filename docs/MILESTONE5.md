# Milestone 5 source/build completion

Base: branch `cleanup/pseudo-agent`, HEAD
`80685c6b6dc357d849f9b0777f9ddae0b0e091a7`, initially clean. No branch/revision
mismatch was found. The shell lacked CMake on PATH; validation used the existing
local CMake 3.31.6 installation. The repository retains its CMake >=3.20 requirement.

## Layout and review groups

| Before | After |
| --- | --- |
| `lns_clean/*.cpp`, shared manager | `src/` |
| supported `cnf/*.cpp`, `mdd/*.cpp` | `src/cnf/`, `src/mdd/` |
| internal headers | `include/lnssat/`, with CNF/MDD/SAT/batch subtrees |
| single-run, supported batch, verification driver | `app/` |
| `lns_clean/tests/` and terrain fixtures | `tests/`, `tests/fixtures/` |
| `minisat/minisat-master/` vendor sources/license | `third_party/minisat/` |
| supported `minisat/minisat-wrapper.*` | `src/sat/`, `include/lnssat/sat/` |
| INI and batch JSON examples | `examples/`, `examples/batch/` |
| `lns/`, probSAT and obsolete drivers/wrappers/scripts | `archive/` |
| `lns_clean/BUILD.md` | `docs/BUILD.md` |

[File inventory](milestone5-file-inventory.tsv) lists all 162 relocations and 82
classified removals. Mechanical moves/include-path changes, runner lookup changes,
and historical/artifact changes are separate review groups. The initial handoff
was left uncommitted; the follow-up verification below prepares this milestone
for commit and push on `cleanup/pseudo-agent`.

The root-level duplicate `lns_clean/run_batch_experiments.cpp` was **zero bytes**.
The 1,014-line supported `lns_clean/run_experiments/run_batch_experiments.cpp` is
retained in `app/`. Its only behavioral change is resolving the sibling `lns-sat`
executable, independent of the working directory. Existing options, defaults,
scenario selection/order, logging, timeouts, dry runs, and rejection behavior are
preserved. The old CMake `main_clean_lns` copy is no longer needed.

CMake keeps explicit source lists, isolated test working directories, separate
vendor warnings, and out-of-tree outputs. Root Make and the old `lns_clean/`
entry point share that graph; documented Make targets remain. Default Make
optimization remains `-O2`, with assertions enabled. Tests add only a focused
batch path regression; all existing 20 CTest entries remain.

## Artifact classification and preservation

Removed files are compiled ELF executables/objects/shared libraries, static
archives, vendor dependency outputs, Python bytecode caches, notebook checkpoints,
Windows Zone.Identifier metadata, a nested Git backup, and the empty duplicate.
The inventory records the reason for each removal. A unique Python checkpoint
script is preserved as `archive/analysis/plot_solve_time_cdf_snapshot.py`.

Primary notebooks, source scripts, CSV/SVG research results, `data/`, both benchmark
collections, `minisat_output.txt`, `temp_200_agents.scen`, and intentional
`docs/verification/2026-09-12/` evidence are preserved. Small smoke input copies in
`tests/fixtures/` are byte-identical to the retained benchmark originals. This is
not dataset curation or a new solver replay manifest.

The entire older `lns/` implementation is archived byte for byte. Historical
probSAT sources and license are outside the supported graph. All retained MiniSAT
vendor files, including the `mkLit` default-argument compatibility patch and
termination callback, are byte-identical after relocation. Project-owned adapters
are outside the vendor directory. Analysis scripts remain build-independent.

## Validation

**Milestone 5 exit condition achieved.** Validation used a fresh source-only copy
at `/tmp/lnssat-m5-final-source`, containing the intended sources, fixtures and
retained evidence, with no generated/ignored build inputs. No solver code changed
after this validation. Final documentation records were then synchronized with it.

The shell used the existing local CMake 3.31.6 `bin/` on PATH, GCC 13.3.0, GNU Make,
and `CMAKE_BUILD_PARALLEL_LEVEL=4`. Exact root commands, from that fresh snapshot:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Configure/build succeeded; **21/21 tests passed** (all 20 baseline names plus
`test_batch_paths`). The baseline was established once at the requested revision
using `/tmp/lnssat-m5-baseline`: **20/20 passed**.

```sh
ctest --test-dir build -R '^smoke_lns_verification$|^test_tiny_mapf_oracle$' --output-on-failure -V
cd build
timeout 30s ./run_lns_verification ../tests/fixtures/empty-8-8.map ../tests/fixtures/empty-8-8-even-1.scen 4 0 42 lns-sat
./run_batch_experiments --map ../tests/fixtures/empty-8-8.map --scenario-dir ../tests/fixtures --num-agents 4 --experiments 1 --seed 42 --variant lns-sat --time-limit 10 --log-file batch-smoke.log
./run_batch_experiments --config ../examples/batch/tiny.json
sh ../tests/test_batch_backend_rejection.sh "$PWD/run_batch_experiments"
```

Both the CTest smoke (30-second limit) and separate externally bounded runner
reported `VERIFY_RESULT seed=42 variant=lns-sat status=valid agents=4 makespan=8`.
The oracle reported **528 agreements**. Both tiny batches completed one successful
experiment, and CLI/JSON backend-rejection checks passed. The added path test
also covers unrelated working directories, spaces, PATH, symlinks, and a missing
sibling solver. Existing deadline regressions passed unchanged.

From the snapshot root, with fresh Make outputs:

```sh
make all BUILD_DIR=/tmp/lnssat-m5-final-make BUILD_FLAGS=-j4
make test BUILD_DIR=/tmp/lnssat-m5-final-make BUILD_FLAGS=-j4
make -C lns_clean test_solver_configuration BUILD_DIR=/tmp/lnssat-m5-final-make BUILD_FLAGS=-j4
make -C lns_clean lns_verification_runner run_experiments BUILD_DIR=/tmp/lnssat-m5-final-make BUILD_FLAGS=-j4
```

Make configure/build succeeded and **21/21 tests passed**. The compatibility
individual test and both runner targets also succeeded.

Both compilation databases have **47 entries / 43 distinct sources**. Inspection
of every compile command and link file confirms no `-fpermissive`, probSAT,
legacy solver, or prebuilt source-tree archive dependency. Vendor compilation
has separate warning flags; Make retains `-O2` without `NDEBUG`. Archive-member
and symbol checks pass in both workflows. All vendor/archived bytes and the
smoke fixture copies were checked against the original paths.

A rename-aware diff review and `git diff -M --check` pass. Active build/code/docs
paths were checked; old paths remain only in explicit compatibility or historical
records. The intended source snapshot contains no compiled products, caches,
notebook checkpoint directories, or download metadata. Builds/tests wrote only
to task-specific output directories. Pre-existing ignored `lns_clean/build/`,
`lns_clean/logs/`, and root `logs/` in the user's checkout are left untouched.

Full task logs and separately grouped review diffs are retained outside the solver
repository in the task workspace at `work/milestone5-2026-09-12/`. The file inventory
in this document's directory provides a repository-local review index. No blockers
remain for Milestone 5; the original working tree and Git index are not used for
validation build products or intermediate staging.

## Pre-commit verification — 2026-09-12

A separate verification pass checked the actual working tree against the handoff:
all **1,094 intended files** matched its SHA-256 manifest, and every removed path
was accounted for by the **244-row inventory**. All **83 archived/vendor files**
matched their original Git blobs byte for byte. All 68 non-batch relocated C++
files differed only in include directives or CLI usage text; the batch runner's
executable lookup change was reviewed separately. No compiled products, caches,
notebook checkpoint directories or download metadata were present in the snapshot.

A new source-only snapshot was built using the three root CMake commands above:
**21/21 CTest tests passed**. A separate fresh `make test` build also passed
**21/21**, and the compatibility `lns_clean/` test and both runner targets passed.
The 20 baseline test names are retained, with only `test_batch_paths` added.
Both build graphs contain 47 compilation entries over 43 distinct sources, with
separate vendor warnings, no permissive flag or legacy/backend dependencies, and
only freshly built archives. Make retains `-O2` with assertions enabled.

The repeated oracle reported **528 agreements**. The independent smoke again
reported `status=valid agents=4 makespan=8` for seed 42 and variant `lns-sat`.
The documented tiny JSON batch succeeded; CTest exercised CLI/JSON paths, PATH,
symlinks, spaces, missing sibling solver rejection, obsolete backend rejection,
and deadline regressions. No implementation fixes were needed in this pass.

The reproduction script, fresh source hashes and full command logs are retained
in the task workspace at `work/milestone5-verification-2026-09-12/`.

## Scope boundary

Milestone 6 has not started. The public CLI/result format, global stdout handling,
complete replay manifest, dataset curation, CI and release packaging are unchanged
and deferred. The bounded correctness sign-off remains historical evidence, with
its limits and source hashes preserved. No new broad correctness audit or benchmark
campaign is part of this milestone.
