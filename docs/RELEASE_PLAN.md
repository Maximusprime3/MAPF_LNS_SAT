# Release plan for LNS-SAT

> Historical checkpoint/planning record. Milestone 5 source organization is now
> complete; see [MILESTONE5.md](MILESTONE5.md) and [BUILD.md](BUILD.md). Paths and
> next steps below describe the recorded earlier revision.

## Target

The first public release should look and behave like a credible research software
artifact:

- it reproduces a small, documented subset of the published experiments;
- it provides a usable command-line solver for a Moving AI map/scenario pair;
- its default algorithm is LNS-SAT with radius sequence `1,2,3,4,...`;
- it contains the pseudo-agent approach from the active development branch;
- it supports MiniSAT through a replaceable internal solver boundary; and
- it does not promise a stable reusable C++ library API yet.

The work should proceed in small reviewable commits. Avoid large file moves and behavior
changes in the same commit.

## Phase 0: document the baseline

**Goal:** make the current implementation understandable before restructuring it.

- Maintain `ARCHITECTURE.md`, `ALGORITHM_VARIANTS.md`, and `KNOWN_ISSUES.md` against the
  current branch.
- Record the published paper and distinguish it from the under-review extended work.
- Identify the exact commit and experiment configurations that produced retained results.
- Tag or otherwise preserve the current working baseline before behavior changes.

**Exit condition:** a new contributor can trace one solve from CLI to final verification
and can see which behaviors are known risks.

## Phase 1: create a safety net

**Goal:** preserve intended behavior while making correctness failures visible.

- Implement one complete, independent MAPF solution verifier.
- Add tiny deterministic fixtures covering no conflict, vertex conflict, edge swap,
  waiting, unsatisfiable horizon, and two zone visits by one agent.
- Add tests for all four radius sequences.
- Add pseudo-agent split/refresh/reassembly tests.
- Capture one or two current end-to-end runs as characterization tests, clearly separated
  from assertions of correctness.
- Run memory/error tooling on the small fixtures where practical.

**Exit condition:** invalid paths cannot be reported as success, and the refactor has fast
tests for its highest-risk state transitions.

## Phase 2: define the supported source tree

**Goal:** establish one obvious implementation and build.

- Designate `lns_clean/` as the source baseline and remove the `_clean` label in a later,
  dedicated move (for example to `src/`).
- Separate executable entry points from reusable algorithm components.
- Consolidate the duplicate batch experiment drivers.
- Remove probSAT from the supported build and CLI.
- Introduce a minimal `SatSolver` interface and one MiniSAT implementation.
- Replace checked-in build products with an out-of-tree build.
- Decide between CMake and a cleaned Makefile; prefer CMake if installation, CI, and
  dependency discovery are priorities.

**Exit condition:** a clean checkout builds the solver in a new build directory using one
documented command, without relying on checked-in binaries.

## Phase 3: fix correctness risks without redesigning LNS-SAT

**Goal:** make the documented algorithm trustworthy.

- Preserve agent IDs explicitly during MDD construction.
- Make waiting-time candidate generation transactional.
- Centralize map bounds and walkability rules.
- Make pseudo-agent refresh atomic and validate all segment mappings.
- Return a structured solve result and propagate failure to the process exit status.
- Run the complete verifier before accepting or writing a solution.
- Convert hardcoded limits and the neighborhood policy into validated configuration.
- Set LNS-SAT (`1,2,3,...`) as the default and expose the three named variants.

**Exit condition:** the test suite covers each fixed issue and every successful CLI run
emits a verifier-approved solution.

## Phase 4: make the CLI usable

**Goal:** support a researcher who did not write the code.

Recommended command shape:

```text
lns-sat solve --map MAP --scenario SCEN --agents N [options]
lns-sat verify --map MAP --scenario SCEN --solution FILE
lns-sat --version
```

Key options should include scenario index, seed, variant, time/makespan limits, output
path, log level, and result format. MiniSAT should be the implicit backend; a backend flag
is unnecessary while only one backend is supported.

- Print concise progress to stderr and machine-readable results to an explicit file or
  stdout.
- Document exit codes for solved, unsolved/limit reached, invalid input, and internal
  error.
- Include resolved configuration, makespan, seed, runtime, revision, and verification
  status in every result.

**Exit condition:** the README quick start can be copied into a clean environment and
produces a verified solution on a tiny included example.

## Phase 5: package reproducibility data

**Goal:** support the paper without keeping gigabytes of generated data in the source
repository.

Recommended split:

1. **Source repository:** one or a few tiny maps/scenarios, expected outputs, experiment
   configuration schemas, and scripts that validate/download larger inputs.
2. **Benchmark source:** download the full Moving AI benchmark set from its canonical
   source when practical. If a small subset is redistributed, include the original
   attribution, license/data terms, source URL, and checksums.
3. **Archival artifact:** publish the exact experiment configurations, checksummed inputs,
   raw result tables needed for the paper, environment manifest, and plotting scripts as
   a versioned archive with a DOI.

Do not make the source repository depend on the current large `logs/` tree. Select the
minimal raw data that substantiates published claims and archive the rest separately.

**Exit condition:** a documented script can reproduce a small result locally, and the
complete paper artifact can be retrieved and verified by checksum.

## Phase 6: documentation, citation, and licensing

**Goal:** make reuse and attribution unambiguous.

- Replace the root README with project purpose, algorithm summary, quick start, CLI
  examples, reproducibility link, limitations, and citation.
- Add `CONTRIBUTING.md` with build/test/style expectations.
- Add `CITATION.cff` and a BibTeX entry for the published paper.
- Audit the top-level license against every bundled dependency and dataset.
- Document the MiniSAT license and any modifications.
- Add author/contact information after confirmation.

Published paper to cite:

> Max Frommknecht and Pavel Surynek. “SAT-Based Large Neighborhood Search for
> Multi-Agent Pathfinding.” ICAART 2026, Volume 1, pages 434–442.
> DOI: 10.5220/0014354400004052.

The extended paper, “Exploring SAT-Based Large Neighborhood Search Strategies for
Multi-Agent Path Finding,” is under peer review and must not be described as published.
Its final citation should be added only when stable publication metadata exists.

**Exit condition:** source, dependencies, data, and papers each have correct attribution
and no citation placeholders remain.

## Phase 7: continuous integration and release

**Goal:** make the public state repeatable.

- Build and run unit/smoke tests in GitHub Actions on a supported compiler matrix.
- Add formatting and static-analysis checks after the code layout stabilizes.
- Test the README quick start in CI.
- Produce a versioned source release and archival reproducibility artifact.
- Record known performance and correctness limitations in release notes.

**Exit condition:** a tagged release builds from scratch, passes its verifier/tests, and
links to a DOI-bearing reproducibility package.

## Suggested first implementation sequence

The next code changes should be deliberately narrow:

1. add the complete standalone verifier and tests;
2. add a typed `NeighborhoodVariant` with sequence tests;
3. make standard LNS-SAT the default;
4. preserve explicit agent IDs in initial MDD construction;
5. introduce structured solve status and correct CLI exit behavior;
6. isolate MiniSAT behind an interface; and only then
7. begin moving/renaming source files and removing archived artifacts.

This order protects scientific behavior before cosmetic restructuring.

## Definition of done for the first public release

- Clean checkout and documented build succeed without committed build products.
- The default CLI runs LNS-SAT, not IncreasingStep.
- All four documented radius schedules are tested.
- Only MiniSAT is supported, through a documented internal backend boundary.
- Every returned solution passes the independent full MAPF verifier.
- One small example is included and one paper-oriented reproduction workflow is
  documented.
- Bulk logs/plots are archived outside the source tree with checksums and provenance.
- README, license notices, author/contact, and citation metadata are complete.
- CI runs the build, unit tests, smoke solve, verifier, and README quick start.
