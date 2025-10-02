# Building the refactored LNS components

The `Makefile` in this directory builds a static archive named `liblns_clean.a` that bundles
all refactored LNS sources together with the shared SAT solver infrastructure used across the
repository.

## Requirements

- A C++17 capable compiler (the Makefile uses `g++` with `-std=gnu++17`).
- `gcc` for compiling the C-based probSAT sources.
- `ar` for producing the static library archive.
- POSIX-compatible build environment with `make`.
- The repository's bundled dependencies:
  - `cnf/` CNF construction sources.
  - `mdd/` MDD constructor sources.
  - `minisat/` MiniSAT wrapper and core sources.
  - `probSAT-master/` probSAT in-memory solver implementation.

## Default compiler and linker flags

The Makefile defaults to the following noteworthy flags:

- `-std=gnu++17` – enables C++17 language features required by the solver.
- `-O2` – applies a reasonable optimization level.
- `-Wall -Wextra -pedantic` – activates additional warnings during compilation.
- `-pthread` – links in POSIX threading support used by MiniSAT.
- `-D__STDC_FORMAT_MACROS` – ensures C99 format macros are available for older headers.
- `-fpermissive` – matches the flags historically used for the existing solver builds.
- Dependency tracking is enabled with `-MMD -MP`.

The probSAT C source is compiled with matching optimisation and macro definitions. All
compilation units are built with include paths that expose the repository root, CNF, MDD,
MiniSAT, and probSAT headers.

## Usage

```bash
cd lns_clean
make          # builds liblns_clean.a under lns_clean/
make clean    # removes the build/ directory and the static library
```

The resulting `liblns_clean.a` can then be linked into higher-level executables together with
any application that provides an entry point (e.g., experiment drivers) and the usual SAT
solver dependencies already packaged in the archive.