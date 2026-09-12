#!/bin/sh
set -eu
runner=$1
fixtures=$2
task_dir=$(mktemp -d /tmp/lns-batch-path-test.XXXXXX)
trap 'rm -rf "$task_dir"' EXIT HUP INT TERM
mkdir -p "$task_dir/working directory" "$task_dir/config directory" "$task_dir/input data" "$task_dir/missing solver"
cp "$fixtures/empty-8-8.map" "$fixtures/empty-8-8-even-1.scen" "$task_dir/input data/"
cd "$task_dir/working directory"

# CLI inputs remain relative to the caller. The solver comes from the runner's
# directory, with no source-root or working-directory executable fallback.
"$runner" --map '../input data/empty-8-8.map' --scenario-dir '../input data' \
    --num-agents 4 --experiments 1 --seed 42 --variant lns-sat \
    --time-limit 10 --log-file cli-solver.log >cli.log 2>&1
grep -F 'All requested experiments completed.' cli.log
grep -F 'Collision-free verified solution found at makespan 8' cli-solver.log

# JSON input paths remain relative to the configuration file. Invoke through
# PATH from an unrelated directory, with spaces in configuration and input paths.
cat >"$task_dir/config directory/batch.json" <<'JSON'
{"runs":[{"map":"../input data/empty-8-8.map","scenario_dir":"../input data","num_agents":4,"experiments":1,"seed":42,"variant":"lns-sat","time_limit_seconds":10,"log_file":"json-solver.log"}]}
JSON
PATH="$(dirname "$runner"):$PATH" run_batch_experiments \
    --config '../config directory/batch.json' >json.log 2>&1
grep -F 'All requested experiments completed.' json.log
grep -F 'Collision-free verified solution found at makespan 8' "$task_dir/config directory/json-solver.log"

# Symlinks still select the solver beside the real batch executable.
ln -s "$runner" "$task_dir/batch-link"
"$task_dir/batch-link" --map '../input data/empty-8-8.map' --scenario-dir '../input data' \
    --num-agents 4 --experiments 1 --time-limit 10 >symlink.log 2>&1
grep -F 'All requested experiments completed.' symlink.log

# A relocated runner without its sibling solver must fail clearly.
cp "$runner" "$task_dir/missing solver/run_batch_experiments"
if "$task_dir/missing solver/run_batch_experiments" \
    --map '../input data/empty-8-8.map' --scenario-dir '../input data' \
    --num-agents 4 --experiments 1 >missing.log 2>&1; then
    echo 'FAIL: missing sibling solver was accepted' >&2
    exit 1
fi
grep -F "Required solver executable 'lns-sat' not found or not executable beside the batch runner" missing.log
echo 'PASS: batch executable and CLI/JSON input lookup'
