#!/bin/sh
# Exact local paths/settings used for the 2026-09-12 validation.
# Requires retained baseline/staging/archive sources; do not blindly rerun baseline
# checks against a checkout that already contains the milestone 6 modifications.
set -eu
M6_REPO=/home/max/Documents/PhD/MAPF/LNS/MAPF_LNS_SAT_cleanup
M6_PROJECT=/home/max/.codex/.chatgpt-projects/g-p-68ef6b375c8c81919b0e53918a9e8787
M6_WORK=$M6_PROJECT/work/milestone6
M6_CMAKE=$M6_PROJECT/work/lnssat-fixes/cmake-3.31.6-linux-x86_64/bin/cmake
M6_CTEST=$M6_PROJECT/work/lnssat-fixes/cmake-3.31.6-linux-x86_64/bin/ctest

# Baseline, before edits (HEAD=91fd9e504c1f13d5cd1ac3bfa71eacbf2866684a):
# git -C "$M6_REPO" rev-parse HEAD
# git -C "$M6_REPO" status --short
# "$M6_CMAKE" -S "$M6_REPO" -B "$M6_WORK/baseline-build" -DCMAKE_BUILD_TYPE=Debug
# "$M6_CMAKE" --build "$M6_WORK/baseline-build" -j4
# "$M6_CTEST" --test-dir "$M6_WORK/baseline-build" --output-on-failure
# Observed 20/21; original dependency regex matched parent /LNS/. Replacing only
# the source-root prefix with <source>/ in the generated inspection text made the
# original inspection script pass against unchanged baseline archives.

# Debug implementation:
"$M6_CMAKE" -S "$M6_WORK/source" -B "$M6_WORK/build" -DCMAKE_BUILD_TYPE=Debug
"$M6_CMAKE" --build "$M6_WORK/build" -j4
"$M6_CTEST" --test-dir "$M6_WORK/build" --output-on-failure

# Independent source-only archive (no .git, existing artifacts or legacy data):
# Copy src/include/app/tests/third_party/cmake/lns_clean/examples/docs and
# CMakeLists.txt, Makefile, README.md, LICENSE into archive-source first.
make -C "$M6_WORK/archive-source" test CMAKE="$M6_CMAKE" CTEST="$M6_CTEST" \
    BUILD_DIR="$M6_WORK/make-build" BUILD_FLAGS=-j4
make -C "$M6_WORK/archive-source/lns_clean" test CMAKE="$M6_CMAKE" CTEST="$M6_CTEST" \
    BUILD_DIR="$M6_WORK/make-build" BUILD_FLAGS=-j4

# Exact fixed-seed/path comparison: probe built separately against each revision.
# Baseline probe was compiled before applying changes to M6_REPO.
# g++ -std=gnu++17 -I "$M6_REPO/include" "$M6_WORK/comparison.cpp" \
#   "$M6_WORK/baseline-build/liblns_core.a" "$M6_WORK/baseline-build/liblns_minisat.a" \
#   -pthread -o "$M6_WORK/baseline-comparison"
g++ -std=gnu++17 -I "$M6_WORK/source/include" "$M6_WORK/source/docs/milestone6/comparison.cpp" \
    "$M6_WORK/build/liblns_core.a" "$M6_WORK/build/liblns_minisat.a" \
    -pthread -o "$M6_WORK/current-comparison"
python3 "$M6_WORK/source/docs/milestone6/compare.py" "$M6_WORK/baseline-comparison" \
    "$M6_WORK/current-comparison" "$M6_WORK/source" "$M6_WORK/repeated-comparisons.json"

# README commands were executed from archive-source with build -> ../make-build.
cd "$M6_WORK/archive-source"
./build/lns-sat solve --map tests/fixtures/empty-8-8.map \
    --scenario tests/fixtures/empty-8-8-even-1.scen --agents 4 --seed 42 --output build/tiny-result.json
./build/lns-sat verify --map tests/fixtures/empty-8-8.map \
    --scenario tests/fixtures/empty-8-8-even-1.scen --solution build/tiny-result.json
