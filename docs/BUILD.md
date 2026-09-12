# Building LNS-SAT

The supported implementation is in `src/`, its internal headers in `include/lnssat/`,
and executable entry points in `app/`. CMake owns one explicit source graph; the root
Makefile and the compatibility `lns_clean/Makefile` delegate to it. MiniSAT is the
only supported backend. Analysis scripts and `archive/` are outside the build graph.

## Requirements

- A C++17 compiler, CMake 3.20 or newer, and a build tool such as GNU Make.
- A POSIX environment with `sh`, `ar`, and `nm`; the batch runner uses POSIX processes.
- GNU Make for the optional Make wrapper; Python 3 for CLI/provenance tests.
- No downloads or prebuilt source-tree libraries are needed. MiniSAT is bundled
  under `third_party/minisat/`, including its license.

## Root CMake/CTest workflow

From the repository root:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

An external build directory works identically, for example
`cmake -S /path/to/repository -B /tmp/lns-sat-build`. In-source configuration is
rejected before compiler detection. All objects, archives, executables, copied
fixtures, and CTest solver logs stay in the selected build directory.

| Target | Contents / output |
| --- | --- |
| `lns_minisat` | Bundled `Solver.cc` and `System.cc`; `liblns_minisat.a` |
| `lns_core` | Project solver, shared CNF/MDD/manager code and MiniSAT adapter; `liblns_core.a` |
| `lns-sat` | Public solve/verify CLI, with legacy positional/INI compatibility |
| `lns_cli` | CLI and artifact lifecycle support |
| `run_batch_experiments` | The consolidated supported batch driver |
| `run_lns_verification` | Independent end-to-end verification driver |
| `test_*` | 18 C++ regression executables plus shell/Python integration checks |

CTest runs **24 tests**: the 21 milestone 5 tests plus public JSON/SHA-256,
CLI subprocess, and build-provenance suites.
The tiny oracle retains **528 cases**. Deadline, backend rejection, dependency
inspection, and independent smoke tests remain enabled. C++ tests have isolated
build-directory working directories; terrain fixtures are copied there under the
relative paths those tests use. Smoke inputs live in `tests/fixtures/`.

Project targets use GNU C++17 and target-local `-Wall -Wextra -pedantic` with GCC/Clang.
MiniSAT has its own target without project warning flags and a `SYSTEM` include
path. The project-owned adapter keeps project warnings. Neither build adds
`-fpermissive`. Threads use `Threads::Threads`; MiniSAT supplies
`__STDC_FORMAT_MACROS`. `compile_commands.json` is emitted for inspection.

CMake does not force optimization: use `-DCMAKE_BUILD_TYPE=Release` or `Debug` as
needed. `-DBUILD_TESTING=OFF` omits tests but retains the verification driver.
For multi-configuration generators use `--config Debug`, `ctest -C Debug`, and
executables under `build/Debug/`.

## Smoke and single-run examples

```sh
ctest --test-dir build -R '^smoke_lns_verification$' --output-on-failure -V
```

The independent runner loads expected endpoints separately and verifies the result
for `empty-8-8-even-1`, **four agents, index 0, seed 42, variant lns-sat**. Expected
output includes `status=valid agents=4 makespan=8`. CTest enforces a **30-second**
external timeout; all other tests have 60-second limits.

The public quick start, from the repository root:

```sh
./build/lns-sat solve --map tests/fixtures/empty-8-8.map \
  --scenario tests/fixtures/empty-8-8-even-1.scen --agents 4 \
  --seed 42 --output build/tiny-result.json
./build/lns-sat verify --map tests/fixtures/empty-8-8.map \
  --scenario tests/fixtures/empty-8-8-even-1.scen --solution build/tiny-result.json
./build/lns-sat --version
```

Observed: both commands return 0, independently verified makespan 8. The result
contains paths, exact input hashes/selection, resolved configuration, measured
milliseconds and metadata from the code actually built. See [CLI.md](CLI.md) and
[RESULT_SCHEMA.md](RESULT_SCHEMA.md). No file appears unless explicitly requested.

From `build/`, `./lns-sat --config ../examples/example_config.ini` exercises the
retained complete INI configuration. Its paths remain working-directory-relative.
The legacy `MAP SCEN N INDEX [SEED [VARIANT]]` syntax remains for the batch runner.

## Batch examples and path rules

After the root build, from the repository root:

```sh
cd build
./run_batch_experiments --map ../tests/fixtures/empty-8-8.map \
  --scenario-dir ../tests/fixtures --num-agents 4 --experiments 1 \
  --seed 42 --variant lns-sat --time-limit 10 --log-file batch-smoke.log
```

Or run `./run_batch_experiments --config ../examples/batch/tiny.json` from `build/`.
The tiny JSON example uses config-relative input paths and writes its explicit
batch log into the default `build/` directory.

The runner finds **`lns-sat` beside its own executable**, including when invoked
through PATH or a symlink from an unrelated working directory. Keep the two built
executables together. The previous build-directory `main_clean_lns` compatibility
copy is no longer needed. CLI paths stay relative to the caller; JSON paths stay
relative to the configuration file. Solver progress now uses stderr (which the runner already captures), and the
solver creates no implicit CSV files. Explicit runner logs retain their existing paths.

Existing JSON examples are in `examples/batch/`; their paths still target the
unchanged benchmark collections. Scenario pattern expansion, natural ordering,
scenario slicing, variants, seeds, logging, dry runs, external time limits, and
obsolete backend-field rejection retain the supported runner's behavior.

## Supported Make workflow

From the root:

```sh
make all
make test
```

The default output directory is `build-make/`, with CMake Release configuration and `-O2` (without `NDEBUG`, preserving the
previous Make defaults). To
choose fresh external outputs and parallel compilation:

```sh
make all BUILD_DIR=/tmp/lns-sat-make BUILD_FLAGS=-j4
make test BUILD_DIR=/tmp/lns-sat-make BUILD_FLAGS=-j4
```

`make -C lns_clean all` and `make -C lns_clean test` remain compatibility entry
points and use the same root `build-make/` directory. No solver sources live there.
All documented individual `test_*` targets remain available, as do
`run_experiments`, `lns_verification_runner`, `asan`, and `clean`.

`make asan` builds into `build-make-asan/` with Debug/AddressSanitizer flags.
`make clean` removes the selected `BUILD_DIR`; use
`make clean BUILD_DIR=build-make-asan` to remove sanitizer outputs.
Override `CMAKE`, `CTEST`, `CMAKE_FLAGS`, or `BUILD_FLAGS` when needed. Use distinct
output directories for different configurations. Compilation/linker flags are
now supplied through CMake instead of a second independent Make source list.

## Local MiniSAT compatibility patch

In `third_party/minisat/minisat/core/SolverTypes.h`, the `mkLit` friend declaration
has no default argument; `sign = false` appears on the namespace-level inline
definition instead. This makes the declaration valid C++ without permissive
compilation. Literal encoding and the `mkLit(v)`, `mkLit(v, false)`, and
`mkLit(v, true)` forms are unchanged. Relocation preserves this file byte for byte.

## Local MiniSAT deadline hook

`third_party/minisat/minisat/core/Solver.h` provides a nullable termination callback
at its existing `withinBudget()` checks. The adapter in
`src/sat/minisat-wrapper.cpp` installs it only for a configured deadline and uses
`solveLimited` to preserve `l_Undef` as `Interrupted`. The callback runs on the
solver thread with no timer thread or asynchronous shared flag. The null callback
preserves unbounded behavior. All vendor files, including this hook, are moved
byte for byte. See [CORRECTNESS_CHECKPOINT.md](CORRECTNESS_CHECKPOINT.md) for the
cooperative deadline contract, which remains unchanged.

## Build-time provenance

Every build runs `cmake/BuildProvenance.cmake` before compiling `lns_cli`. It writes
a generated header only when provenance changes; this refreshes revision/dirty
metadata even on incremental builds. Source archives without their own `.git`
record null revision/dirty metadata and a supported-source checksum. The executable
never queries a checkout at runtime. Build flags are captured for the selected
configuration, including custom configurations. Do not edit source while compiling.

The supported Make defaults are still Release with `-O2` and assertions enabled;
Debug CMake and source-archive Make were both validated in [MILESTONE6.md](MILESTONE6.md).
The dependency inspection now strips literal source/build root prefixes before
checking forbidden relative paths, avoiding the pre-existing false positive for a
parent directory named `LNS` without weakening archive/symbol checks.
