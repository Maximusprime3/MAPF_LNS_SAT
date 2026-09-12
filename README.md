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
[docs/BUILD.md](docs/BUILD.md). The existing positional/configuration CLI remains in
use; the public CLI and replay-result format are later roadmap work.

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
Run solver and batch commands from an output directory to keep their logs there.
Historical probSAT material is retained under `archive/` and is not a supported
backend. See [archive/README.md](archive/README.md).

## Project documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Roadmap](docs/ROADMAP.md)
- [Known issues and limitations](docs/KNOWN_ISSUES.md)
- [Milestone 5 layout and validation](docs/MILESTONE5.md)
- [Bounded correctness verification](docs/VERIFICATION_SIGNOFF.md)

## License and publication metadata

Project license: [LICENSE](LICENSE). MiniSAT license:
[third_party/minisat/LICENSE](third_party/minisat/LICENSE). Retained historical
probSAT has its own license in [archive/probSAT-master/LICENSE](archive/probSAT-master/LICENSE).
Dataset attribution, citation/contact metadata and publication polish remain
tracked in the roadmap.
