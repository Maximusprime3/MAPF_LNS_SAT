# Verification evidence — 2026-09-12

Read [the sign-off](../../VERIFICATION_SIGNOFF.md) for scope, results and limitations.
These small records support an engineering checkpoint; they are not paper benchmark
results or the future solver replay-manifest implementation.

## Files

- `measurements.jsonl`: 25 individual observations (15 map/schedule smoke runs,
  eight deadline runs and two invalid-request observations). `solve_ms` measures
  the LNS call, excluding the probe's independent load and final verification.
  `verified=false` for non-solved outcomes means there was no solution to verify.
  For the invalid 60-agent baseline request it means the requested path count failed.
- `inputs.json`: input locations and SHA-256 hashes.
- `environment.json`: revisions, compiler, platform, build mode and probe hash.
- `verification_probe.cpp`: one-off API probe; solver output goes to `solver.log`
  in its working directory and a compact JSON observation goes to stdout.
- `dense-60.scen`: synthetic cancellation fixture, not a published benchmark.
- `ctest.log`, `leak-tests.log`: full test-run transcripts.
- `sanitizer-provenance.json`: source comparison for reused sanitizer binaries.

## Reproduce

Use the evaluated revision for the current build and `1a052be` in a separate
checkout for the comparison. Both builds used the same GCC 13.3.0 Debug settings;
the bundled local CMake executable was version 3.31.6. Its build instructions were in `lns_clean/BUILD.md` (now `docs/BUILD.md`).

The commands below target the evaluated pre-relocation revision, preserving its
recorded probe and source hashes. For the current layout, see `docs/BUILD.md`;
compile this unchanged historical probe with `-I include/lnssat -I include` instead
of `-I lns_clean`.

From the evaluated repository root (replace paths to suit the local checkouts):

```sh
cmake -S . -B /tmp/lns-current -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/lns-current -j4
ctest --test-dir /tmp/lns-current --output-on-failure

c++ -std=c++17 -g -pthread -I lns_clean \
  docs/verification/2026-09-12/verification_probe.cpp \
  /tmp/lns-current/liblns_core.a /tmp/lns-current/liblns_minisat.a \
  -o /tmp/lns-current/verification-probe
```

Repeat the build for the baseline checkout with `-DBUILD_TESTING=OFF` and build
target `lns-sat`. Compile the same probe with the baseline include directory and
baseline archives. Do not mix headers and libraries from different revisions.

For each row in `measurements.jsonl`, run the appropriate probe in a separate
output directory, resolving `input` using `inputs.json`:

```sh
timeout 15s /tmp/lns-current/verification-probe \
  /absolute/path/to/map.map /absolute/path/to/scenario.scen \
  AGENTS SEED LIMIT_MS VARIANT
```

`LIMIT_MS=0` omits the internal deadline. Scenario group index is always zero;
the remaining solver settings are defaults from the evaluated revision. The input
hashes identify the exact instances. Timings will vary across machines and runs;
do not expect the precise pre-fix mixture of timeout and late success to repeat.
The probe reports invalid input with exit 65, which is its own loader failure code,
not the production CLI's status mapping. Its exit 2 flags a solved result failing
independent verification or requested path count. Other status details are in JSON.

For a new sanitizer build, configure Debug with
`-DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -D_GLIBCXX_ASSERTIONS"`
and `-DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"`, build, then run CTest
with `ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1` in an environment
without ptrace. This checkpoint reused the preceding sanitizer build after checking
source identity; it did not perform another sanitizer compilation.
