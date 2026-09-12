# Milestone_Cleanup session handoff — 2026-09-12

## Resume here

The bounded verification of milestones 1–4 after milestone 5.3 is complete.
Solver revision: `cdc3b03` on `Milestone_Cleanup`. This follow-up changes only
documentation and retains small verification evidence; no production code changed.

Read [VERIFICATION_SIGNOFF.md](VERIFICATION_SIGNOFF.md) for the requirement matrix,
measured outcomes, reproduction instructions and limitations. The seven original
fixes and before/after behavior remain in
[CORRECTNESS_CHECKPOINT.md](CORRECTNESS_CHECKPOINT.md).

## Completed evidence

- Fresh out-of-tree Debug build of the committed checkout: **20/20 CTest tests pass**.
- Leak-enabled AddressSanitizer + UndefinedBehaviorSanitizer: **20/20 pass** outside
  ptrace. Reused prior binaries after confirming all 43 compiled sources and 50
  source-copy headers match the checkout. The prior leak-check gap is now closed
  for these fixtures; this is not proof about every input.
- Independent tiny oracle: **528 fixed-horizon two-agent cases agree**.
- Six small, fixed-seed comparisons against `1a052be` on empty/random/maze maps:
  all solutions independently valid, matching makespans. Three alternative
  neighborhood schedules also pass small end-to-end checks.
- Valid 32-agent and synthetic 60-agent deadline probes: current returns
  `Exhausted` with no paths. The old revision can report solved after the budget.
- Requirement-to-test review covers named segmentation/refresh/reassembly,
  slack/rollback/grid, configuration and backend-boundary requirements.
- Make tests passed at the previous checkpoint; not repeated in this follow-up.

No benchmark or test job remains intentionally running. Raw verbose solver logs,
builds and temporary drivers are local artifacts. Small measurements, hashes,
probe source, synthetic fixture and test transcripts are under
`docs/verification/2026-09-12/`.

## Important limits

Milestone 3 is **not complete**: the full solver replay manifest is still missing.
Its implementation belongs to milestone 6, coordinated with milestone 7 inputs.
The manually retained audit records do not satisfy that product requirement.

Deadlines are cooperative. A 100 ms budget took 274–298 ms to return in the
32-agent Debug probe; the result was rejected correctly. Do not claim precise
stopping latency or use the requested time limit as measured runtime.

The bundled empty-8-8 scenario has only 32 entries. A request for 60 on that file
is invalid and is now rejected. This session used a separately recorded genuine
60-agent synthetic input for that cancellation check. Measurements use the solver
API; they do not establish CLI launch-to-exit timing.

These fixtures do not prove global LNS completeness, every pseudo-agent
transformation combination, performance equivalence, or paper-result reproduction.

## Next steps

1. Continue milestone 5 source/artifact organization in focused changes; rerun the
   regression suite after source moves and preserve MiniSAT's termination callback.
2. Implement milestone 6 CLI/result formats and replay manifest, aligned with
   milestone 7 archived inputs. Keep milestone 3 visibly open until that is done.
3. Complete logging/output separation and metric definitions. Some deadline exits
   still omit partial-attempt metrics; correctness tests do not establish full accounting.
4. Prepare the broader experimental reproduction and release CI from milestones
   7–8. Profile finer deadline cancellation only if experiment requirements need it.

Selective path-assumption reuse remains a separate, measured optimization. The
full-path policy and UNSAT fallback are intentional. No additional unrestricted
milestones 1–4 audit is required before continuing the roadmap.
