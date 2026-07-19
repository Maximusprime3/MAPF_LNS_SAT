# Building the refactored LNS components

The `Makefile` in this directory is the currently supported build. It produces the static
archive `liblns_clean.a`, the `main_clean_lns` solver, and the batch experiment runner.

## Requirements

- A C++17-capable compiler (the Makefile uses `g++` with `-std=gnu++17`).
- `ar` for producing the static library archive.
- A POSIX-compatible environment with `make` and `sh`.
- The repository-bundled `cnf/`, `mdd/`, and MiniSAT sources.

MiniSAT is the only supported SAT backend. The build has no probSAT include, source,
object, archive-member, or symbol dependency.

## Default compiler and linker flags

The Makefile currently uses:

- `-std=gnu++17` for the solver sources;
- `-O2`;
- `-Wall -Wextra -pedantic`;
- `-pthread`;
- `-D__STDC_FORMAT_MACROS` for the bundled MiniSAT headers;
- `-fpermissive`, retained temporarily for legacy project sources; and
- `-MMD -MP` dependency tracking.

Removing `-fpermissive`, separating third-party warnings, and replacing checked-in build
products belong to Milestone 5.

## Usage

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
