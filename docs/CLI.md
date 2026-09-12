# Public CLI contract (schema 1)

The milestone 6 implementation follows this contract. `lns-sat solve --map MAP
--scenario SCEN --agents N [options]`, `lns-sat verify --map MAP --scenario SCEN
--solution FILE`, `lns-sat --help`, and `lns-sat --version` are the public entry points.
MiniSAT is implicit. Exhaustion, including timeout, is not proof of global UNSAT.

Solve options and defaults: `--scenario-index 0` (block index; selected scenario
entries are `[index*N, (index+1)*N)`), `--seed 42` (signed 32-bit), `--variant
lns-sat` (also `initial-radius-2`, `fixed-step-2`, `increasing-step`),
`--makespan-increment 1`, `--makespan-increase-limit 10`,
`--lazy-iteration-limit 10000000`, `--full-map-fallback-threshold 0.95`,
`--wall-clock-limit-ms MS` (positive integer milliseconds, default unlimited),
`--makespan-bound MOVES` (nonnegative absolute bound, default unlimited),
`--log-level info` (`quiet`, `info`, `debug`), `--format json`, `--output FILE`.
No output document is written unless `--output` is specified; `-` is a literal
filename, not stdout. Progress goes to stderr. Full maps/paths/backend details
are debug output. Quiet retains actionable errors. Help/version use stdout.

The old makespan loop tries base+0, base+increment, ... through base+increase_limit.
The limit is a maximum additive delta, **not a number of increases**. An absolute
bound is an additional ceiling; it never changes the base or the increment.

`solve --config FILE` uses the existing flat `key=value` file (blank lines and
whole-line `#` comments). Keys are `map`, `scenario`, `num_agents`,
`scenario_index`, `seed`, `variant`, `makespan_increment`,
`makespan_increase_limit`, `makespan_bound`, `lazy_iteration_limit`,
`full_map_fallback_threshold`, `wall_clock_limit_ms`, `log_level`, and `log`.
Defaults < configuration file < explicit CLI options, independent of option order.
The file must itself contain a complete valid request, even if CLI options would
override an invalid value.
Map, scenario, and agent count must be supplied after resolution. All relative
paths (including INI paths) resolve against the process working directory;
resolved absolute paths are recorded. Batch JSON retains config-relative paths.
Unknown keys/options, duplicate keys/options, missing/empty values, unsupported
formats, malformed/overflowing/nonfinite numbers, and invalid ranges are errors.
Integers use decimal syntax without a leading plus; fractions use JSON decimal
syntax. Named CLI and INI variants require canonical names; the positional legacy
parser retains its historical aliases. Variants are recorded canonically.

The legacy `MAP SCEN N INDEX [SEED [VARIANT]]` and `--config FILE` forms remain
supported for the batch runner. `log=FILE` explicitly requests a diagnostic text
file; it no longer redirects global stdout. No implicit CSV files are produced.
Experimental CSV logging is available only via an explicit internal logger directory.

Exit codes: solve 0 verified success, 1 bounded exhaustion, 2 invalid invocation or
instance, 3 internal failure. Verify 0 valid, 2 malformed document/invocation or
unreadable/malformed instance, 4 well-formed invalid submitted solution (including
checksum or instance-selection inconsistency). Either command uses 5 for output
I/O failure. Errors include diagnostics on stderr. Failed solves have no paths.

Output overwrites an existing regular file. Inputs (including configuration and
submitted solution) are protected against direct, canonical, symlink and hard-link
aliases. Nonregular outputs and symlink outputs are rejected. After valid argument
resolution and destination preflight, an old artifact is removed so that a failed
run cannot leave a stale success at its destination. A private sibling temporary
file is flushed, synced and closed before atomic rename. Failures leave no completed
new document. Argument/preflight errors leave existing files untouched; exit status
is authoritative. If a directory cannot be modified, an old file may be impossible
to remove; no new result is claimed on exit 5. There is no power-loss durability guarantee or hostile-directory
race protection. No parent directories are created automatically.

Timing uses steady-clock measured milliseconds. `total_ms` starts on entry to the
command handler and ends after input reads/hashes, loading, solve and final independent
verification, before serialization/output. `solver_ms` covers the complete LNS call,
including its validation/loading (when applicable), final verification and cleanup;
it is null when LNS was not invoked. The cooperative wall-clock deadline starts
before input acquisition for solve and is passed unchanged to LNS; return latency
can exceed the budget. Nested historical experimental timings are not redefined.
