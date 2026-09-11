# LNS-SAT architecture

## Status and scope

This document describes the implementation currently centered in `lns_clean/` on the
`cleanup/pseudo-agent` branch. It is a description of the code as it
exists, not yet a claim that every component is release-ready.

The intended public project is **LNS-SAT**, a SAT-based large-neighborhood-search solver
for multi-agent path finding (MAPF). The current branch's main research feature is the
pseudo-agent representation used when one real agent enters a local zone more than once.

The older `lns/` implementation, experimental analysis scripts, generated logs, plots,
and checked-in binaries are outside the core architecture documented here.

## Problem and solution model

The solver consumes a Moving AI-style grid map and scenario file. For the selected
scenario slice, each agent has one start and one goal. A solution maps each integer agent
ID to a sequence of grid positions indexed by timestep.

The intended solution invariants are:

1. every requested agent has exactly one path;
2. each path starts at that agent's start and ends at its goal;
3. every move is either a wait or a move to an orthogonally adjacent walkable cell;
4. paths share a common planning horizon (makespan), padding at the goal when needed;
5. no two agents occupy the same vertex at the same timestep; and
6. no two agents swap across the same edge during one timestep.

`SolutionVerifier` independently enforces all six invariants before a successful result is returned.

## Entry points

### Single run

`lns_clean/main_clean_lns.cpp` is the current command-line entry point. It accepts either
positional arguments or an INI-like configuration file and calls `LNS(...)` in
`lns_clean/LNS.cpp`.

Current positional form:

```text
main_clean_lns <map_path> <scenario_path> <num_agents> <scenario_index> [seed] [variant]
```

Current configuration keys are `map`, `scenario`, `num_agents`, `scenario_index`,
`seed`, `variant`, `makespan_increment`, `makespan_increase_limit`,
`lazy_iteration_limit`, `full_map_fallback_threshold`, `wall_clock_limit_ms`,
`log_level`, and optional `log`. Unknown keys are rejected.

MiniSAT is the only supported backend and is intentionally implicit in the CLI. Obsolete
solver arguments and batch/configuration fields are rejected rather than silently mapped.

### Batch experiments

`lns_clean/run_experiments/run_batch_experiments.cpp` and the similarly named file at the
top of `lns_clean/` are research experiment drivers. They are not yet part of the stable
public CLI. Their behavior and duplication should be resolved during cleanup.

## End-to-end algorithm flow

The current orchestration in `lns_clean/LNS.cpp` follows this sequence:

1. **Load the instance.** `Load_LNSProblem` parses the map and the requested scenario
   slice into a grid, starts, and goals.
2. **Compute lower-bound data.** `SATSolverManager::compute_max_timesteps` produces
   per-agent distance maps and a base makespan.
3. **Build initial MDDs.** The solver creates one shortest-path MDD per agent, extended
   with goal waiting to reach the current makespan.
4. **Sample an initial solution.** One path is randomly sampled from each MDD using the
   configured seed. Paths are padded to the current makespan and waiting-time budgets are
   recorded.
5. **Detect global conflicts.** Vertex and edge conflicts are collected from the current
   solution.
6. **Select a neighborhood.** Conflicts at the earliest relevant time are grouped into
   spatial buckets. A bucket is selected and converted into a map-geometry-aware local
   zone.
7. **Build a local MAPF problem.** Every contiguous visit of a real agent inside the zone
   becomes a local segment. Later visits are represented by pseudo agents with stable
   segment IDs.
8. **Solve lazily with SAT.** MDD constraints are encoded into CNF. The SAT model is
   decoded into local paths, newly discovered vertex and edge collisions are added as
   clauses, and MiniSAT is invoked again until the local paths are collision-free or the
   local problem is unsatisfiable.
9. **Use available waiting slack.** If necessary, the local solver attempts to consume
   goal-waiting slack and rebuild affected segment MDDs.
10. **Integrate or expand.** A successful local repair is spliced into the global paths.
    Otherwise the spatial zone and time window are expanded according to the selected
    radius policy.
11. **Try the full instance.** Once the zone reaches the configured full-map fallback
    threshold, the code makes one full-map, full-time-window SAT attempt.
12. **Increase makespan.** If the full attempt fails, the outer loop increases the
    makespan by the configured increment and rebuilds the initial solution, up to the
    configured increase limit.
13. **Verify and return.** The final paths are passed through `VerificationHelpers`, which
    delegates to the independent complete verifier in `SolutionVerifier`. Verification
    failure is fail-closed and returned as a structured non-solved status with a
    diagnostic.

## Core modules

| Module | Current responsibility |
| --- | --- |
| `LNS.cpp` | Top-level makespan loop, conflict loop, neighborhood selection, metrics, and final return |
| `Load_LNSProblem.*` | Parse map/scenario input and select agents |
| `Current_Solution.*` | Own global paths, occupancy lookup, conflicts, waiting budgets, and local-path integration |
| `Local_Zone.*` | Build/select conflict buckets, mask maps, and expand a selected zone |
| `Local_Zone_Builder.*` | Grow reachable zones while respecting map geometry and corridors |
| `Create_Local_Problem.*` | Split real-agent paths into local segments, assign pseudo-agent IDs, and build segment MDDs |
| `Solve_Local_Zone.*` | Retry local repairs, expand the zone/time window, and perform the full-map fallback |
| `Waiting_time_Solve.*` | Coordinate waiting-slack attempts and update the global solution after success |
| `Lazy_SAT_Solve.*` | Backend-neutral incremental clause loading, assumptions, collision discovery, and result mapping |
| `SatSolver.h` | Typed backend contract for reset, clauses, assumptions, model, outcomes, and statistics |
| `SolutionVerifier.*` | Independently enforce agent coverage, common horizon, path geometry, start/goal, vertex-conflict, and edge-conflict invariants |
| `VerificationHelpers.*` | Compatibility wrappers used by existing solver call sites |
| `ExperimentLogger.*`, `Metrics.h` | Record experiment, makespan, zone, waiting, and lazy-iteration measurements |
| `mdd/` | Multi-value decision diagrams for time-expanded agent movement |
| `cnf/` | Translate MDD path choices and collision constraints into CNF |
| `minisat/minisat-wrapper.cpp` | The only supported `SatSolver` adapter; owns all MiniSAT-specific types |
| `minisat/minisat-master/` | Bundled MiniSAT implementation |
| `SATSolverManager.*` | Shared map, path, collision, and makespan utilities inherited from earlier code |

## Pseudo-agent representation

A real agent can cross the boundary of a local zone multiple times. Treating its entire
global path as one local path would allow the local solver to alter portions outside the
selected neighborhood. The pseudo-agent approach instead splits the path into contiguous
inside-zone visits:

- the first segment retains ownership by the real agent;
- each additional visit receives a unique pseudo-agent/segment ID;
- `original_id` links every segment back to the real agent;
- entry and exit positions/times constrain the segment at the zone boundary;
- the SAT solver operates on segment IDs; and
- successful segment paths are reassembled in time order into the real agent's global
  path.

`LocalZoneState` is the central data structure. It stores all `LocalSegment` objects,
indices by segment ID, segment order by original agent, and the stable mapping from real
agents to pseudo-agent IDs. Correctness depends on keeping those mappings stable whenever
the zone or time window is expanded.

## SAT boundary

`SatSolver` is the backend-neutral contract used by lazy solving:

```text
reset -> add appended clauses -> solve [with typed assumptions] -> Sat | Unsat | Interrupted | Error
                                                       -> model and per-call statistics
```

`Waiting_time_Solve` creates one fresh solver session per waiting attempt and injects it
into `Lazy_SAT_Solve`. The lazy loop loads only the suffix appended since the previous
iteration. If an assumption solve is explicitly `Unsat`, it resets once, reloads the full
formula, and retries without assumptions. Backend `Error` returns immediately and is never
treated as `Unsat`. Statistics and elapsed time include both calls when that retry occurs.

The MiniSAT adapter and factory are implemented in `minisat/minisat-wrapper.cpp`; MiniSAT
types do not cross that file boundary. Clause diagnostics use an injected sink enabled by
`LogLevel::Debug`. The supported Make target and archive have no probSAT include, source,
object, or symbol dependency. Historical probSAT-only files remain outside the supported
artifact until the source-tree milestone classifies legacy material.

## Randomness and reproducibility

The configured seed initializes the Mersenne Twister used to sample initial MDD paths and
is included in experiment metadata. Reproducibility still also depends on the exact input
files, code revision, compiler/build configuration, and SAT backend version. A release
experiment manifest should record all of them.

## Structured solver outcomes

Mutable algorithm state and solver conclusions are deliberately separate. `CurrentSolution`
owns the global paths, occupancy data, and waiting-time budgets; transactions restore that
state after unsuccessful speculative slack repairs. Result objects do not duplicate this
state.

Four boundaries report the shared statuses `Solved`, `Exhausted`, `InvalidInput`, and
`InvalidState`:

1. `LazySolveResult` describes one bounded lazy SAT/CNF solve.
2. `WaitingSolveResult` describes the sequence of SAT attempts using available waiting slack.
3. `LocalZoneResult` describes spatial zone expansion across slack attempts.
4. `LNSResult` describes the complete bounded solver invocation and owns verified final paths.

A formula or assumption conclusion of `Unsat` maps to bounded `Exhausted` search behavior;
a backend `Error` maps to `InvalidState` and propagates upward immediately. `Exhausted` may
cause the next slack amount, radius, or makespan to be tried, while `InvalidInput` and
`InvalidState` do not trigger an UNSAT retry. The CLI
maps the four statuses to exit codes 0, 1, 2, and 3 respectively.

## Architectural boundaries for cleanup

The first cleanup should preserve the algorithm while making these boundaries explicit:

```text
CLI/config
    -> problem loader
    -> LNS-SAT orchestration
       -> neighborhood policy
       -> local problem and pseudo-agent model
       -> SAT backend
       -> solution integration
    -> complete verifier
    -> result/metrics writer
```

Analysis notebooks and plotting scripts should consume stable result files, but should not
be dependencies of the solver executable.

## Post-5.3 correctness checkpoint

See `CORRECTNESS_CHECKPOINT.md` for the added input/termination/slack/grid tests,
absolute-time path assumptions, and combined pseudo-agent integration coverage.
`SatSolver::set_deadline` supplies a monotonic cooperative deadline. `Interrupted`
returns bounded exhaustion without an unrestricted retry. MDD construction polls
the same deadline; local transactions roll back on interruption, and late solutions
are rejected before commit or return.
