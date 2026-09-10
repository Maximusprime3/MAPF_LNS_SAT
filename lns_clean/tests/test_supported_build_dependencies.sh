#!/bin/sh
set -eu

# CMake supplies its generated target graph and both freshly built archives.
# The original single-archive invocation continues to inspect the Make graph.
task_dir=$(mktemp -d /tmp/lns-supported-build-test.XXXXXX)
trap 'rm -rf "$task_dir"' EXIT HUP INT TERM
if [ "${1:-}" = --cmake ]; then
    graph=${2:?usage: test_supported_build_dependencies.sh --cmake GRAPH ARCHIVE...}
    shift 2
    test -s "$graph"
    cp "$graph" "$task_dir/graph"
else
    make -s -pn all >"$task_dir/graph"
fi

if grep -Eiq 'probsat|CNFProbSAT' "$task_dir/graph"; then
    echo "supported build graph still contains a probSAT dependency" >&2
    exit 1
fi

test "$#" -gt 0
for archive do
    # Capture tool output first so a failed inspection cannot silently pass.
    "${AR:-ar}" t "$archive" >"$task_dir/members"
    "${NM:-nm}" -A "$archive" >"$task_dir/symbols"
    if grep -Eiq 'probsat|CNFProbSAT' "$task_dir/members"; then
        echo "supported archive still contains a probSAT object: $archive" >&2
        exit 1
    fi
    if grep -Eiq 'probsat|CNFProbSAT' "$task_dir/symbols"; then
        echo "supported archive still exposes a probSAT symbol: $archive" >&2
        exit 1
    fi
done

echo "Supported build dependency checks passed."
