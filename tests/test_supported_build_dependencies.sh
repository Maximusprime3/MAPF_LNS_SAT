#!/bin/sh
set -eu

# CMake supplies its generated target graph and both freshly built archives.
# The original single-archive invocation continues to inspect the Make graph.
task_dir=$(mktemp -d /tmp/lns-supported-build-test.XXXXXX)
trap 'rm -rf "$task_dir"' EXIT HUP INT TERM
if [ "${1:-}" = --cmake ]; then
    graph=${2:?usage: test_supported_build_dependencies.sh --cmake GRAPH ARCHIVE...}
    source_root=$3
    build_root=$4
    shift 4
    test -s "$graph"
    # Match repository-relative legacy components, not case-insensitive parent
    # directory names such as /home/user/LNS/current. Literal substitution only.
    awk -v src="$source_root/" -v build="$build_root/" '
        function strip(line, prefix, n) {
            while ((n = index(line, prefix)) > 0)
                line = substr(line, 1, n-1) "<root>/" substr(line, n+length(prefix))
            return line
        }
        { print strip(strip($0, src), build) }
    ' "$graph" >"$task_dir/graph"
else
    make -s -pn all >"$task_dir/graph"
fi

if grep -Eiq 'probsat|CNFProbSAT|(^|[ /;])archive/|(^|[ /;])lns/|-fpermissive' "$task_dir/graph"; then
    echo "supported build graph contains a forbidden backend, legacy path, or permissive flag" >&2
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
