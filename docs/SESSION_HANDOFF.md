# Milestone_Cleanup session handoff — 2026-09-11

## Resume here

This branch continues the roadmap audit of milestones 1–4 after milestone 5.3,
starting from `1a052be` on `cleanup/pseudo-agent`. The user authorized fixes and
asked to pause, commit, and push this checkpoint as `Milestone_Cleanup`.
Read [CORRECTNESS_CHECKPOINT.md](CORRECTNESS_CHECKPOINT.md) for the seven issues,
before/after behavior, and evidence, then [ROADMAP.md](ROADMAP.md) for release order.

## Completed

- Reject incomplete requested scenario groups, malformed scenario rows, dimension
  mismatches, and malformed/ragged maps before solving.
- Terminate spatially saturated disconnected-zone expansion with a final full-map
  attempt; permit the retry after consuming the last waiting-slack unit.
- Forward path assumptions as SAT literals at the correct absolute MDD times.
- Propagate cooperative deadlines through construction, SAT, and orchestration;
  distinguish interruption from UNSAT, reject late results, and roll back local work.
- Strengthen pseudo-agent validation against global endpoints and disconnected or
  illegal MDD edges; cover combined slack, re-entry, refresh, and reassembly.
- Register three new test programs in both supported build workflows; correct the
  roadmap's milestone 3 completion claim and document remaining work.

## Verification at this checkpoint

- Clean out-of-tree CMake Debug build: **20/20 CTest tests passed**.
- Make workflow: **make -j2 test passed**, including all new regressions.
- AddressSanitizer + UndefinedBehaviorSanitizer Debug build with library assertions:
  **20/20 CTest tests passed** using `ASAN_OPTIONS=detect_leaks=0` and
  `UBSAN_OPTIONS=halt_on_error=1`. LeakSanitizer was unavailable under the sandbox's
  ptrace environment; leak checking is **not verified**.
- Independent joint-state enumeration agrees with lazy SAT on **528** two-agent,
  2x2-grid fixed-horizon cases. Every returned solution is independently verified.
- A real MiniSAT pigeonhole test exercises interruption; fake late SAT/UNSAT results
  check that neither late success nor an unrestricted fallback escapes the deadline.

Tests ran against an isolated copy of the source tree. Only source, tests, build
configuration, and documentation are included in the commit. Generated executables,
archives, objects, downloaded CMake, and local logs are not release artifacts.
No jobs remain intentionally running; no long benchmark campaign was started.

## Next steps, in order

1. Rebuild from this branch in a fresh out-of-tree directory using `lns_clean/BUILD.md`
   and run CTest. Read the new tests before changing deadline or pseudo-agent code.
2. Run bounded representative benchmarks with fixed inputs/seeds against `1a052be`:
   compare verified outcomes, makespan, runtime, and timeout behavior. Tiny exhaustive
   tests establish bounded correctness evidence, not large-instance performance or
   global LNS completeness. The proposed 60-agent CLI timing comparison was not run
   in this resumed session; do not present an unmeasured timing improvement as fact.
3. Continue the remaining milestone 5 organization work described in ROADMAP.md.
   Re-run regression tests after each source move; preserve the MiniSAT termination
   callback hook when relocating or updating the vendored solver.
4. Implement milestone 6 CLI/result output and the complete replay manifest, aligned
   with milestone 7 archival inputs. Milestone 3's full exit condition remains open
   until requested/resolved settings and reproducible inputs are recorded together.
5. Complete tracked logging/output separation and metric semantics. Some deadline
   exits do not yet retain every partial attempt metric; do not infer full accounting
   from the correctness tests.
6. Consider selective path-assumption reuse only as a separately measured optimization.
   The existing full-path reuse policy and explicit UNSAT fallback remain; pinning a
   colliding full model may require fallback and should not be mistaken for a new bug.
7. Run leak detection outside a ptrace environment and broaden pseudo-agent fixtures
   and benchmarks before claiming release-wide robustness.

Deadlines are cooperative, not hard process-kill boundaries. CNF construction,
propagation, verification, allocation, I/O, or cleanup can finish between checks.
Use the batch runner's external timeout where a hard process boundary is required.
