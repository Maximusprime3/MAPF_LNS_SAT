#!/bin/sh
set -eu

archive=${1:?usage: test_supported_build_dependencies.sh ARCHIVE}

if make -s -pn all | grep -Eiq 'probsat|CNFProbSAT'; then
    echo "supported Makefile graph still contains a probSAT dependency" >&2
    exit 1
fi

if ar t "$archive" | grep -Eiq 'probsat|CNFProbSAT'; then
    echo "supported archive still contains a probSAT object" >&2
    exit 1
fi

if nm -A "$archive" | grep -Eiq 'probsat|CNFProbSAT'; then
    echo "supported archive still exposes a probSAT symbol" >&2
    exit 1
fi

echo "Supported build dependency checks passed."
