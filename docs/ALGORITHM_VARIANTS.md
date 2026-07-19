# LNS-SAT neighborhood variants

## Public naming

The canonical public algorithm name is **LNS-SAT**. Variant names describe only the
schedule used to expand the local-zone radius after an unsuccessful solve.

The initial radius applies to both the spatial conflict neighborhood and the surrounding
time-window offset in the current implementation. After each failed attempt, the next
radius is computed and the local problem is rebuilt.

## Intended variants

| Public name | Initial radius | Growth after failure | Attempted radii |
| --- | ---: | ---: | --- |
| **LNS-SAT** (default) | 1 | add 1 | `1, 2, 3, 4, 5, 6, 7, ...` |
| **InitialRadius2** | 2 | add 1 | `2, 3, 4, 5, 6, 7, ...` |
| **FixedStep2** | 1 | add 2 | `1, 3, 5, 7, 9, 11, ...` |
| **IncreasingStep** | 1 | add the failure count | `1, 2, 4, 7, 11, 16, ...` |

LNS-SAT is the intended command-line default and corresponds to the radius schedule used
for the published paper.

## Relationship to the current code

`solve_local_zone(...)` currently receives three policy values:

- `offset`: initial radius;
- `expansion_radius_step`: base step; and
- `ZoneExpansionGrowth`: either `FixedStep` or `DynamicStep`.

The variants map to those values as follows:

| Variant | `offset` | `expansion_radius_step` | growth enum |
| --- | ---: | ---: | --- |
| LNS-SAT | 1 | 1 | `FixedStep` |
| InitialRadius2 | 2 | 1 | `FixedStep` |
| FixedStep2 | 1 | 2 | `FixedStep` |
| IncreasingStep | 1 | 1 | `DynamicStep` |

Neighborhood growth is now represented by the typed `NeighborhoodVariant` policy. The
public LNS-SAT sequence is the default, while the three paper variants can be selected
explicitly through the command line, configuration file, integration verifier, or batch
runner.

The canonical command-line names are `lns-sat`, `initial-radius-2`, `fixed-step-2`, and
`increasing-step`.

## Formulae

Let `r_0` be the initial radius and let `k` count failed attempts starting at one.

- Fixed-step policies use `r_k = r_(k-1) + s`, where `s` is the configured step.
- IncreasingStep uses `r_k = r_(k-1) + k`.

The code calls the latter behavior `DynamicStep`. The public configuration should use the
paper-facing name `IncreasingStep`; the implementation enum can be renamed during cleanup.

## Full-map fallback and makespan

The variant controls neighborhood expansion only. Independently of the chosen schedule:

1. when the zone reaches at least 95% of cells counted as walkable, the implementation
   tries the full map over the full current time window;
2. if that attempt is unsatisfiable, the outer loop increases the makespan by one; and
3. a new initial solution is sampled for the larger makespan.

These fallback rules should be exposed as separate configuration values rather than being
encoded into the variant name.

## Historical labels

Repository paths and experiment logs contain labels such as `LNS+2`, `LNS++1`,
`LNSinit2`, and `LNSrerun`. They are useful provenance for old experiments but are not
clear public API names. Do not silently rename old result data; instead, add a small
metadata table when preserving it and use the four names above for all new runs.

## Proposed future CLI

The eventual CLI should make the policy explicit while retaining a sensible default:

```text
lns-sat solve INSTANCE_OPTIONS [--variant lns-sat|initial-radius-2|fixed-step-2|increasing-step]
```

If the variant is omitted, the implementation selects `lns-sat` and therefore uses radii
`1,2,3,4,...`. Solver and batch logs record the selected public variant name.
