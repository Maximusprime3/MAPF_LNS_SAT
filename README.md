# LNS-SAT research solver

LNS-SAT is a SAT-based large-neighborhood-search solver for multi-agent path finding.
The supported implementation uses MiniSAT and independently verifies successful
solutions. This repository also retains historical research code and analysis data.

## Build and test

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Requirements, Make targets, smoke and batch examples are in
[docs/BUILD.md](docs/BUILD.md). A C++17 compiler, CMake >=3.20 and POSIX tools are
required; the test suite additionally uses Python 3.

## Solve and independently verify a tiny instance

Run these commands from the repository root after building:

```sh
./build/lns-sat solve --map tests/fixtures/empty-8-8.map \
  --scenario tests/fixtures/empty-8-8-even-1.scen --agents 4 \
  --seed 42 --output build/tiny-result.json
./build/lns-sat verify --map tests/fixtures/empty-8-8.map \
  --scenario tests/fixtures/empty-8-8-even-1.scen --solution build/tiny-result.json
```

Both commands return 0; the verifier prints `VERIFY_RESULT valid agents=4 makespan=8`
to stderr. This expected result was observed in the milestone 6 validation.
`--help` lists options; `--version` reports build-time provenance. `--log-level quiet`
suppresses progress. Results are written only with `--output`; there are no implicit
CSV files. Existing positional/INI callers, including the batch runner, remain supported.

Solve exits: 0 verified success, 1 bounded exhaustion, 2 invalid input, 3 internal
failure. Verify uses 4 for a well-formed invalid submission and 2 for malformed
input. Output failures use 5. Exhaustion/timeout does not prove global infeasibility.
See the [CLI contract](docs/CLI.md), [JSON schema](docs/RESULT_SCHEMA.md), and
[requirement-to-test evidence](docs/MILESTONE6.md).

## Repository structure

| Directory | Purpose |
| --- | --- |
| `src/` | Supported solver, shared CNF/MDD/manager code and MiniSAT adapter |
| `include/lnssat/` | Internal headers |
| `app/` | Single-run, batch, and independent verification entry points |
| `tests/`, `tests/fixtures/` | Regression tests and required small inputs |
| `third_party/minisat/` | Bundled MiniSAT source and license |
| `examples/` | INI and JSON batch configurations |
| `archive/` | Unsupported older solver/backend code and scripts |
| `Analysis/`, `Analysis2/` | Research scripts and notebooks, outside build dependencies |
| `mapf-map/`, `mapf-scen-even/` | Existing benchmark collections |
| `data/`, root CSV/SVG files | Retained historical research results |
| `docs/` | Architecture, roadmap, build and verification records |

The root Makefile uses the CMake graph. `lns_clean/Makefile` is a compatibility
entry point only. Generated binaries, libraries and caches are not source inputs.
Solver artifacts require explicit destinations. Batch log paths retain their existing rules.
Historical probSAT material is retained under `archive/` and is not a supported
backend. See [archive/README.md](archive/README.md).

## Project documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Roadmap](docs/ROADMAP.md)
- [Known issues and limitations](docs/KNOWN_ISSUES.md)
- [Milestone 6 public CLI and result evidence](docs/MILESTONE6.md)
- [Milestone 5 layout and validation](docs/MILESTONE5.md)
- [Bounded correctness verification](docs/VERIFICATION_SIGNOFF.md)

## License and publication metadata

Project license: [LICENSE](LICENSE). MiniSAT license:
[third_party/minisat/LICENSE](third_party/minisat/LICENSE). Retained historical
probSAT has its own license in [archive/probSAT-master/LICENSE](archive/probSAT-master/LICENSE).
Dataset attribution, citation/contact metadata and publication polish remain
tracked in the roadmap.
