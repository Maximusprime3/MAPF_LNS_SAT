# LNS-SAT architecture

## Status and scope

This document describes the implementation currently centered in `lns_clean/` on the
`new_pseudo_agent_approach` line of development. It is a description of the code as it
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

The final public verifier should enforce all six invariants. The current verifier does not
yet enforce the complete list; see `KNOWN_ISSUES.md`.

## Entry points

### Single run

`lns_clean/main_clean_lns.cpp` is the current command-line entry point. It accepts either
positional arguments or an INI-like configuration file and calls `LNS(...)` in
`lns_clean/LNS.cpp`.

Current positional form:

```text
main_clean_lns <map_path> <scenario_path> <num_agents> <scenario_index> <solver> [seed]
```

Current configuration keys are `map`, `scenario`, `num_agents`, `scenario_index`,
`solver`, `seed`, `variant`, and optional `log`.

The CLI still advertises both MiniSAT and probSAT. The intended public artifact will ship
with MiniSAT only, behind a small solver interface so another SAT solver can be added
without changing the LNS algorithm.

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
11. **Try the full instance.** Once the zone covers at least 95% of cells counted as
    walkable, the code makes one full-map, full-time-window SAT attempt.
12. **Increase makespan.** If the full attempt fails, the outer loop increases the
    makespan by one and rebuilds the initial solution, up to a hard-coded limit.
13. **Verify and return.** The final paths are passed through `VerificationHelpers`, which
    delegates to the independent complete verifier in `SolutionVerifier`. Verification
    failure is fail-closed: callers receive an empty result rather than invalid paths.

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
| `Lazy_SAT_Solve.*` | Incremental collision discovery around the SAT/CNF solve |
| `SolutionVerifier.*` | Independently enforce agent coverage, common horizon, path geometry, start/goal, vertex-conflict, and edge-conflict invariants |
| `VerificationHelpers.*` | Compatibility wrappers used by existing solver call sites |
| `ExperimentLogger.*`, `Metrics.h` | Record experiment, makespan, zone, waiting, and lazy-iteration measurements |
| `mdd/` | Multi-value decision diagrams for time-expanded agent movement |
| `cnf/` | Translate MDD path choices and collision constraints into CNF |
| `minisat/` | Bundled MiniSAT implementation and in-memory wrapper |
| `SATSolverManager.*` | Shared path, collision, makespan, and SAT-solver utilities inherited from earlier code |

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

The algorithm conceptually needs a small backend contract:

```text
add clauses -> solve -> read model or UNSAT
```

The current implementation reaches MiniSAT through repository-wide CNF and solver-manager
classes. Solver selection leaks into the CLI and experiment tooling, while the actual
local solve is not cleanly backend-independent. A release-oriented refactor should define
one C++ interface at this boundary, implement it with MiniSAT, and remove probSAT from the
supported build. That preserves future solver replaceability without maintaining two
backends now.

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

`Exhausted` is an expected search outcome and may cause the next slack amount, radius, or
makespan to be tried. `InvalidInput` and `InvalidState` propagate upward immediately. The CLI
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
