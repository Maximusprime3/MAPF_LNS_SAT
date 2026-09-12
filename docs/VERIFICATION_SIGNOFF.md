# Milestones 1–4: bounded verification sign-off

Date: 2026-09-12. Evaluated revision: `cdc3b03e8d4b10f25695787434c2f260f777653d`
on `Milestone_Cleanup`. Comparison revision: `1a052be8b326c7507c3782c5bdf96247c005aee7`
(milestone 5.3, before the correctness fixes).

## Decision and scope

The planned post-5.3 correctness verification checkpoint is complete. The reviewed
requirements of milestones 1, 2, and 4 have implementation and passing regression
evidence. Milestone 3's configuration and cooperative deadline behavior have passing
evidence, but **milestone 3 remains incomplete until the replay manifest is implemented**.
No new solver correctness defect was reproduced in this bounded follow-up.

This is an engineering verification checkpoint supporting publication preparation.
It does not establish general LNS completeness, exhaustive transformation coverage,
performance equivalence, or reproduction of the paper's experimental results. No
production algorithm, logging policy, or public interface was changed in this pass.

## Requirement-to-evidence review

| Requirement | Implementation / executable evidence | Assessment |
| --- | --- | --- |
| M1: segment identity, ownership, bidirectional indices, ordered intervals, path and MDD bounds | `validate_local_zone_state`; `test_local_zone_state_validation` | Covered, including rejection before integration. |
| M1: boundary anchors and connected, legal MDD paths | Validator; `test_audit_regressions` | Global anchors, dead ends and corrupted endpoints exercised. |
| M1: single visit, re-entry, starts/ends inside, global goal inside | `test_pseudo_agent_segmentation` | All named fixture categories covered. |
| M1: refresh gains/losses/merges segments; outside-path preservation | `test_pseudo_agent_refresh_reassembly`; `test_audit_regressions` | Time extension and spatial expansion covered; merging removes the obsolete pseudo-ID. Combined slack/re-entry/refresh/SAT/reassembly exercises shifted outside movement. This is not all possible combinations. |
| M2: consume slack once; preserve makespan, goal suffixes, unrelated paths | `test_waiting_budget_conservation`; `test_audit_regressions` | Success, multiple retries, final-unit retry, and shifted suffix covered. |
| M2: unsuccessful attempts leave no committed changes | `test_current_solution_transaction`; waiting-budget and deadline tests | UNSAT, early invalid state, expired deadline and commit/rollback paths covered. Implementation uses transactional speculative changes. |
| M2: terrain agreement and rectangular input contract | `test_grid_walkability`; `test_audit_regressions` | Terrain symbols, boundaries, malformed/ragged input, CRLF, and direct MDD/zone entry points covered. |
| M3: validated request/configuration and exposed search policies | Configuration tests; audit regressions; `test_neighborhood_variant` | Validation, selected instance size, defaults/overrides and four neighborhood policies covered. |
| M3: shared deadline, interruption distinct from UNSAT, no late success | `test_deadlines`; bounded comparisons below | Real MiniSAT interruption, fake late SAT/UNSAT, no unrestricted timeout retry, expired MDD and rollback covered. Return latency remains cooperative. |
| M3: complete replay manifest and effective output policy | `ROADMAP.md`, milestones 6–7 | Replay manifest unfinished; logging/output separation remains release work. The evidence files for this audit do not implement solver result manifests. |
| M4: MiniSAT boundary, incremental protocol, assumptions and errors | `test_sat_protocol_characterization`; `test_minisat_unsat_status`; audit/deadline tests | Producer-to-adapter literals at absolute times 0 and 5, UNSAT fallback, errors, interruption, reset/model/statistics covered. |
| M4: supported build excludes probSAT and rejects backend selection | `test_supported_build_dependencies`; `test_batch_backend_rejection` | Both pass against fresh CMake artifacts. Historical sources remain outside the supported build, pending milestone 5 cleanup. |

## Fresh build and memory checks

- A new out-of-tree **Debug** build of the committed checkout passed **20/20 CTest
  tests**, including the independent solution verifier, the 528-case tiny oracle,
  supported-build dependency checks and the fixed-seed smoke solve.
- The prior AddressSanitizer/UndefinedBehaviorSanitizer build passed **20/20 tests
  with leak detection enabled**, outside the ptrace sandbox:
  `ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1`.
  All 43 source files in that build's compilation database and all 50 source-copy
  headers were byte-compared with the evaluated checkout: no differences.
  Sanitizer binaries were reused, not freshly rebuilt in this pass.
- The Make suite passed at the preceding checkpoint; it was not repeated here.
- No sanitizer or leak error was reported in the executed fixtures. This is bounded
  dynamic-analysis evidence, not proof of absence of memory defects on other inputs.

The independent tiny oracle enumerates joint states for two agents on a 2×2 square,
over distinct start/goal pairs and horizons from individual lower bounds through four.
It uses no production MDD, CNF, grid or collision helpers. Its **528 agreements are
fixed-horizon checks of lazy SAT**, not exhaustive checks of the whole LNS algorithm.

## Bounded benchmark comparison

Both revisions were freshly built with GCC 13.3.0, C++17 and CMake Debug settings on
the same Linux host. The driver uses identical solver defaults, scenario group 0,
eight agents and seeds 42 and 7. It independently loads expected starts/goals and
verifies every returned solution and the requested path count. Each process has a
15-second external guard. Runs were sequential, with one observation per seed/map
and revision; runtime includes solver diagnostic output written to local files.

| Input map / scenario `*-even-1.scen` | Seeds | Pre-fix makespan | Current makespan | Verification |
| --- | --- | --- | --- | --- |
| `empty-8-8` | 42, 7 | 8, 8 | 8, 8 | All valid |
| `random-32-32-10` | 42, 7 | 46, 46 | 46, 46 | All valid |
| `maze-32-32-2` | 42, 7 | 69, 69 | 69, 69 | All valid |

The three alternative schedules (`initial-radius-2`, `fixed-step-2`,
`increasing-step`) also returned independently valid eight-agent solutions of
makespan 8 on `empty-8-8`, seed 42. Together with the default case, this exercises
all four public schedules end to end on one small instance.

All six paired outcomes and makespans agree. Current measured times were 9–100 ms;
the baseline range was 9–98 ms. These small Debug runs are smoke comparisons across
three map types, not statistical performance evidence or a large-instance campaign.
Exact paths and optimality were not compared.

## Deadline and input-selection checks

A 100 ms deadline was tested on the **valid 32-agent** `empty-8-8-even-1.scen`
instance, seed 42, three times per revision:

- Pre-fix: one `Exhausted` return at 124 ms and two independently valid `Solved`
  returns at 450 and 454 ms. The latter reproduce acceptance after the time budget.
- Current: three `Exhausted` returns, **zero paths**, at 274, 298 and 284 ms.
  Late success is prevented, but cleanup/unchecked work can still cause substantial
  return latency beyond the configured budget.

A separate, genuinely 60-agent synthetic scenario uses distinct starts and goals
sampled without replacement from the 8×8 grid with Python `Random(42)`. At 100 ms,
one run per revision returned `Exhausted` with zero paths: 162 ms pre-fix and 103 ms
current. This is a cancellation stress fixture; feasibility was not established.

The bundled `empty-8-8-even-1.scen` contains only 32 entries. Requesting 60 from it
is an invalid experiment: the old loader accepted it and returned 32 paths, while
the corrected loader rejects it. That exploratory request is retained as evidence
of input rejection and **excluded from valid deadline/performance comparisons**.
Earlier references to a 60-agent CLI comparison must not be treated as verified
60-agent timing evidence. This pass measured the solver API with an independent
probe, not CLI launch-to-exit timing.

The public contract remains cooperative cancellation, not a hard real-time upper
bound. Use a separate process timeout for a hard termination policy. The observed
32-agent overshoot is explicitly retained as a limitation; no precise stopping
latency or general speedup is claimed.

## Retained evidence and reproduction

[Evidence directory](verification/2026-09-12/README.md) contains the probe, synthetic
scenario, individual measurements, input SHA-256 hashes, environment information,
test transcripts, and sanitizer source-provenance check. It provides commands to
rebuild and rerun the checks. Raw verbose solver logs remain local build artifacts.

## Next release work

Resume milestone 5 source/artifact organization and rerun the regressions after
source moves. Then implement milestone 6 CLI/result formats, the complete replay
manifest coordinated with milestone 7 inputs, logging/output separation, and
documented metrics (including incomplete partial-attempt accounting on timeouts).
Preserve the MiniSAT termination callback when relocating the vendored solver.

Broad benchmark reproduction, release CI and expanded coverage belong to subsequent
publication preparation. This bounded checkpoint does not need another unrestricted
audit before continuing the roadmap.
