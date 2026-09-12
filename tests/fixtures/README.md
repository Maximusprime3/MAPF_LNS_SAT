# Regression fixtures

`all-terrain.*` are the existing loader/grid/solve-result fixtures, moved from
`lns_clean/tests/fixtures/` without content changes.

`empty-8-8.map` and `empty-8-8-even-1.scen` are byte-identical copies of the existing
`mapf-map/empty-8-8.map` and
`mapf-scen-even/scen-even/empty-8-8-even-1.scen`. They make the smoke and batch path
regressions self-contained. The original benchmark collections remain unchanged;
dataset attribution/licensing curation remains Milestone 7 work.

The smoke selects four agents, scenario index 0, seed 42, variant `lns-sat`, and
independently verifies makespan 8. CTest supplies absolute paths for this driver
and copies terrain fixtures into the working directories of tests that use
relative `tests/fixtures/` paths.
