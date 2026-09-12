# Correctness checkpoint after Milestone 5.3

The audit of `1a052be` found gaps despite all 17 original CTest tests passing.
This checkpoint records the fixes and the scope of their regression evidence.

| Issue | Previous outcome | Current behavior and evidence |
| --- | --- | --- |
| Request/instance mismatch | A two-agent request could return a verified one-agent solution. Malformed scenario rows could silently shift a selected group. | The loader requires the requested count, rejects malformed scenario rows, and checks scenario/map dimensions. Partial groups return `InvalidInput` with no paths. |
| Saturated disconnected zone | An impossible corridor swap repeatedly expanded the same three cells because they covered only 50% of the map. | Once spatial expansion stops and the full time horizon is reached, one full-map attempt precedes makespan advancement. The disconnected regression terminates without a deadline. |
| Last waiting-slack unit | One unadjusted attempt exhausted a budget of one before trying the adjusted candidate. An unrelated agent's budget changed the outcome. | The attempt bound includes the initial attempt plus budget-consuming retries. The one-wait crossing solves with or without the unrelated agent, preserving its path and budget. |
| Grid shape | Ragged rows reached structures allocated using the first row's width. | Map headers and row dimensions are checked before search. Direct MDD construction rejects ragged grids; zone construction returns no zone. CRLF and existing terrain behavior remain supported. |
| Deadline enforcement | Lazy SAT and individual MiniSAT calls could exceed the deadline and return success. | Deadlines reach distance/MDD construction, clause loading, and MiniSAT's cooperative budget checks. Interruption is distinct from UNSAT, triggers no unrestricted retry, and cannot commit or return late paths. |
| Path assumptions | Literal IDs were interpreted as a 0/1 assignment; nonzero entry times produced missing or wrong assumptions. | Literal IDs are forwarded directly and looked up using each MDD's absolute entry time. A real-adapter test checks all six forced-path literals at both time 0 and time 5. |
| Pseudo-agent trust boundary | Self-consistent endpoints could disagree with the global boundary; nonempty MDD levels did not guarantee connectivity. | Validation checks global endpoint anchors, fixed MDD endpoints, reachability, dead ends, and legal adjacent-level edges. A combined slack/re-entry/refresh/reassembly regression checks the independent verifier and outside-path preservation. |

## Verification

Both supported builds include `test_audit_regressions`, `test_deadlines`, and
`test_tiny_mapf_oracle`, in addition to the existing tests. Run the documented
CMake/CTest commands or `make all` followed by `make test` at the repository root (the `lns_clean/` Make shim also works).

The tiny oracle enumerates joint states without using production MDD, CNF, grid,
or collision helpers. All 528 tested fixed-horizon cases cover two distinct agents
on a 2x2 square with every distinct start/goal pair and horizons from their
individual distance lower bound through four. Every returned SAT solution is also
checked by the independent verifier. This is bounded evidence, not a completeness
proof for arbitrary LNS neighborhoods or a reproduction of the paper benchmarks.

## Deadline contract

`wall_clock_limit_ms` is a cooperative deadline measured with a monotonic clock.
An expired run returns `Exhausted` with no result paths. MiniSAT reports
`Interrupted`, which is never interpreted as formula UNSAT or retried without
limits. An unsuccessful local attempt rolls back paths, budgets, and occupancy.

Cancellation checks run during distance/MDD expansion, during clause loading,
inside MiniSAT, between orchestration stages, and before accepting a result.
This is not a hard real-time process-kill guarantee: an individual allocation,
I/O operation, CNF-construction stage, propagation pass, verification pass, or
cleanup can finish before the next check. Use the batch runner's external process
timeout when a separate process must be terminated at a prescribed boundary.

## Remaining release work

- Milestone 3's complete replay manifest remains unfinished. Milestone 6 owns its
  CLI/result-format implementation, coordinated with Milestone 7's archival inputs.
  Do not describe Milestone 3's full exit condition as achieved before that exists.
- Log-level/output separation and metric definitions remain tracked release work.
- Path-assumption reuse retains the existing full-path policy and UNSAT fallback.
  Pinning a complete colliding model can require that fallback. A more selective
  reuse heuristic is a separate performance/algorithm change.
- The new fixtures do not establish large-instance performance equivalence, general
  LNS completeness, or exhaustive coverage of every pseudo-agent transformation.
