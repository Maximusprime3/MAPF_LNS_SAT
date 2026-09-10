# Building LNS-SAT

The root CMake build supports the implementation in `lns_clean/` without moving sources
or using checked-in build products. The existing `lns_clean/Makefile` remains supported.

## Requirements

- A C++17-capable compiler (the Makefile uses `g++` with `-std=gnu++17`).
- CMake 3.20 or newer and a build tool such as Make or Ninja for the root workflow.
- `ar` and `nm` for static archives and dependency inspection tests.
- A POSIX-compatible environment with `sh` (and `make` for the Make workflow).
- The repository-bundled `cnf/`, `mdd/`, and MiniSAT sources.

MiniSAT is the only supported SAT backend. The build has no probSAT include, source,
object, archive-member, or symbol dependency.

## Root CMake/CTest workflow

From the repository root:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Use a new directory, or inspect an existing `build/` before configuring it. An external
directory works identically: `cmake -S /path/to/repository -B /tmp/lns-sat-build`.
In-source configuration is rejected. Objects, archives, executables, copied test fixtures,
and solver-generated test logs stay in the selected build directory; no prebuilt Make
artifacts are linked and CTest never invokes Make to build or inspect the solver.

The target structure is:

| Target | Contents / output |
| --- | --- |
| `lns_minisat` | Bundled `Solver.cc` and `System.cc`; `liblns_minisat.a` |
| `lns_core` | Supported local/shared sources and project-owned MiniSAT adapter; `liblns_core.a`, linked with `lns_minisat` and `Threads::Threads` |
| `lns-sat` | Existing single-run CLI, named `lns-sat` |
| `run_batch_experiments` | Only `lns_clean/run_experiments/run_batch_experiments.cpp` |
| `lns_batch_solver` | Build-directory `main_clean_lns` compatibility copy of `lns-sat` for the unchanged batch driver's executable lookup |
| `test_*` | The 14 existing C++ test executables |
| `run_lns_verification` | Existing independent end-to-end verification runner |

CTest registers **17 tests**: 14 C++ tests, the two existing shell checks, and one bounded
smoke solve. The dependency shell check inspects generated CMake target properties and
both CMake archives. Each C++ test has an explicit build-directory working directory;
the terrain fixtures retain their existing relative paths via build-directory copies.
The batch rejection check receives the absolute CMake runner path.

Run the independent smoke test alone with:

```bash
ctest --test-dir build -R '^smoke_lns_verification$' --output-on-failure -V
```

It uses absolute paths to existing `mapf-map/empty-8-8.map` and
`mapf-scen-even/scen-even/empty-8-8-even-1.scen`, four agents, scenario index 0,
seed **42**, and variant `lns-sat`. The runner loads the instance independently and
verifies the returned paths. The expected verified makespan is **8**. CTest enforces a
**30-second timeout**, with logs under `build/tests/smoke/`. Other tests have 60-second
timeouts. No solver limits, algorithms, or CLI semantics are changed.

To run the batch driver, first change into the CMake executable directory so its existing
lookup finds the freshly built compatibility copy, and supply absolute input paths:

```bash
cd build
./run_batch_experiments --map /absolute/path/to/map.map \
  --scenario-dir /absolute/path/to/scenarios --num-agents 4 --experiments 1
```

For multi-configuration generators, build with `--config Debug`, test with `-C Debug`,
and run executables from `build/Debug/`. `-DBUILD_TESTING=OFF` omits the CTest suite;
the verification runner remains buildable.

CMake requires C++17 (GNU extensions match Make), links threads through `Threads::Threads`,
and supplies `__STDC_FORMAT_MACROS` to MiniSAT and its adapter. Project warnings
(`-Wall -Wextra -pedantic` on GCC/Clang) are target-local. Bundled MiniSAT compiles in a
separate target without those project warning options, and its include directory is
`SYSTEM`; the project-owned adapter keeps project warnings. Neither build adds
`-fpermissive`. CMake uses standard build-type flags, selectable with
`-DCMAKE_BUILD_TYPE=Release` or `Debug`; no optimization level is forced by default.
`compile_commands.json` is emitted for Make/Ninja compilation inspection.

## Make compiler and linker flags

The Makefile currently uses:

- `-std=gnu++17` for the solver sources;
- `-O2`;
- `-Wall -Wextra -pedantic`;
- `-pthread`;
- `-D__STDC_FORMAT_MACROS` for the bundled MiniSAT headers; and
- `-MMD -MP` dependency tracking.

The supported Make build no longer requires `-fpermissive` (Milestone 5.2). CMake separates
third-party warning settings in Milestone 5.3. Removing checked-in build products and
reorganizing sources remain deferred.

## Local MiniSAT compatibility patch

In `minisat/minisat-master/minisat/core/SolverTypes.h`, the `mkLit` friend declaration
has no default argument; `sign = false` appears on the namespace-level inline definition
instead. This makes the declaration valid C++ without permissive compilation. The literal
encoding is unchanged, as are `mkLit(v)`, `mkLit(v, false)`, and `mkLit(v, true)`.

## Existing Make workflow

```bash
cd lns_clean
make all
make test
```

`make all` builds the archive, solver executable, and supported batch runner. `make test`
runs the deterministic unit/integration suite, including incremental SAT protocol tests,
obsolete-backend input rejection, and inspection of the supported build graph and archive.

```bash
make lns_verification_runner
make asan
make clean
```

The verification runner is used for bounded fixed-seed benchmark checks. `make asan`
rebuilds with AddressSanitizer. `make clean` removes the local build directory, archive,
solver executable, and batch runner.
