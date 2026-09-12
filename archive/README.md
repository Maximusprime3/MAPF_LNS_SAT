# Historical research code — unsupported

This directory preserves the older `lns/` implementation, probSAT sources and
license, obsolete CNF/backend adapters and test drivers, Python prototypes, and
historical build/run scripts. These files are retained for reference, byte for
byte, apart from their paths. Their internal paths describe the former layout;
they are not maintained build or usage instructions.

None is a dependency of root CMake or the supported Make wrapper. The only active
implementation is in `src/`, with internal headers in `include/lnssat/` and entry
points in `app/`. The project-owned supported MiniSAT adapter is
`src/sat/minisat-wrapper.cpp`; the vendor boundary is `third_party/minisat/`.

The older `lns/` archive is a pure relocation, separately listed in
`docs/milestone5-file-inventory.tsv`. Compiled artifacts and caches were removed,
not archived. The unique Python checkpoint script was preserved as
`analysis/plot_solve_time_cdf_snapshot.py`; primary research notebooks, plots,
CSV results, datasets, and correctness-verification evidence remain in place.

Use [the supported build instructions](../docs/BUILD.md). Historical probSAT code
is not an available backend and is not validated by the current test suite.
