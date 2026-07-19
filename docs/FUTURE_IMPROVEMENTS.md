# Future Improvements

This document collects promising changes that are intentionally outside the current
correctness-and-release cleanup. Items here should not silently become release blockers;
they are reminders for later design and performance work.

## Initial-solution construction

### Problem

The current initial-solution phase builds a complete shortest-path MDD for every agent and
then samples exactly one path from each MDD. If the only required output of this phase is
one initial path per agent, constructing and storing all alternatives in every MDD may be
unnecessarily expensive in both runtime and memory.

This is especially worth revisiting for large instances, where the initial MDDs may contain
many equivalent shortest-path alternatives even though most of that structure is discarded
after a single path is sampled.

### Possible direction A: construct one path directly

Compute one shortest path per agent without constructing a complete MDD. A direct search or
distance-guided walk could use the configured random generator to choose reproducibly among
equally short moves. This would keep the current initial-solution semantics while reducing
construction and allocation overhead.

Any implementation must preserve:

- explicit agent IDs;
- deterministic behavior for a fixed seed;
- shortest-path correctness;
- the existing goal-waiting and makespan-padding rules; and
- fail-closed behavior when an agent has no path.

### Possible direction B: make better use of the MDDs

Keep the MDDs, but use their alternative paths more deliberately. For example, initial paths
could be selected with awareness of already chosen agents, predicted vertex/edge congestion,
or path diversity. The MDDs could also be retained and reused by later repair stages if that
avoids rebuilding equivalent search structures.

This direction may cost more than direct single-path construction, but it could produce a
substantially better initial solution and reduce the number of conflicts that LNS-SAT must
repair.

### Evaluation needed before choosing

Compare both directions against the current implementation using fixed benchmark instances
and a documented seed set. Record at least:

- initial-solution runtime and peak memory;
- number of initial vertex and edge conflicts;
- total LNS-SAT runtime and number of repair attempts;
- final makespan and success rate; and
- repeatability when the same seed is rerun.

The right choice depends on the complete solve, not only on making initial path construction
faster. A cheaper initial path can be a regression if it creates enough additional conflicts
to make the repair phase significantly slower.

### Current decision

Deferred. Do not redesign initial-solution construction during the present cleanup. The
current priority is correctness, explicit failure propagation, reproducibility, documentation,
and a clean public command-line artifact.
