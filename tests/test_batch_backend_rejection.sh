#!/bin/sh
set -eu

runner=$1
task_dir=$(mktemp -d /tmp/lns-batch-backend-test.XXXXXX)
trap 'rm -rf "$task_dir"' EXIT HUP INT TERM

if "$runner" --solver minisat >"$task_dir/cli.log" 2>&1; then
    echo "FAIL: obsolete --solver option was accepted" >&2
    exit 1
fi
if ! grep -F "Unknown argument: --solver" "$task_dir/cli.log" >/dev/null; then
    echo "FAIL: obsolete --solver rejection was not deterministic" >&2
    exit 1
fi

printf '%s\n' '{"runs":[{"solver":"minisat"}]}' >"$task_dir/config.json"
if "$runner" --config "$task_dir/config.json" >"$task_dir/config.log" 2>&1; then
    echo "FAIL: obsolete JSON solver field was accepted" >&2
    exit 1
fi
if ! grep -F "Unknown key in configuration: solver" "$task_dir/config.log" >/dev/null; then
    echo "FAIL: obsolete JSON solver rejection was not deterministic" >&2
    exit 1
fi

echo "PASS: obsolete batch backend inputs are rejected"
