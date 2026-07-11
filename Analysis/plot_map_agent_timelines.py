"""Visualise per-map agent completion curves for multiple solvers.

This helper complements :mod:`Analysis.plot_solve_time_cdf` by focusing on a
single map at a time.  It aggregates the bundled ``experiments.csv`` solver
logs, reconstructs the cumulative completion timeline for each agent count, and
plots the resulting step curves so that LNS-SAT and MDD-SAT runs can be compared
side by side.  The x-axis represents the elapsed wall time spent inside a single
map/agent configuration (capped at the 600 second experiment budget), while the
y-axis counts how many of the 100 attempted instances finished successfully.

Solver identity is now conveyed through colour (with MDD-SAT rendered in the
Okabe–Ito orange ``#E69F00`` and LNS-SAT in the Okabe–Ito blue ``#0072B2``), while
agent counts modulate colour grading and dash styles:

* 10 agents → lightest tint of the solver colour
* 20 agents → mid tint of the solver colour
* 50 agents → base solver colour
* 100 agents → base solver colour with dashed segments
* 200 agents → darker shade of the solver colour with dashed segments

The combination allows dense comparisons even when multiple solvers are plotted
for each agent count.  Axes are padded slightly below zero on both dimensions so
instantaneous completions and still-unsolved curves remain easy to inspect.

Typical usage combines the LNS-SAT and MDD-SAT roots shipped with this
repository::

    python Analysis/plot_map_agent_timelines.py \\
        --solver LNS-SAT lns_clean/logs/LNS \\
        --solver MDD-SAT lns_clean/logs/WholeSolve \\
        --output lns_wholesolve_map_timelines.png

Within a Jupyter notebook the high-level :func:`plot_solver_map_agent_timelines`
function can be imported directly.  By default the helper prints a tabular
summary of the per-map/agent statistics to standard output so the descriptive
figures are accompanied by the requested metrics.  Pass
``display_stats=False`` when calling the helper if you would like to suppress
the automatic print-out.  A typical invocation looks like::

    from pathlib import Path
    from Analysis.plot_map_agent_timelines import plot_solver_map_agent_timelines

    fig, axes = plot_solver_map_agent_timelines(
        {
            "LNS-SAT": [Path("lns_clean/logs/LNS")],
            "MDD-SAT": [Path("lns_clean/logs/WholeSolve")],
        },
        title="Per-map completion timelines",
    )

The returned figure and axes can be further customised using standard
``matplotlib`` APIs.  Colours encode the agent count while line styles
differentiate higher agent counts, mirroring the Okabe–Ito palette used throughout this
repository for colour-blind safe visualisations.  In addition to the visual
comparison, the helper can emit a per-map/agent statistical summary describing
the finishing time, number of completed instances, and descriptive statistics
for per-instance runtimes, CNF variables, and CNF clauses.  These summaries can
either be retrieved programmatically or exported from the command-line
interface alongside the generated figures.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import (
    Dict,
    Iterable,
    Mapping,
    MutableMapping,
    Optional,
    Sequence,
    Tuple,
    Union,
    cast,
)

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import colors as mcolors
from matplotlib.lines import Line2D
from pandas.api.types import is_bool_dtype, is_numeric_dtype

from Analysis.plot_solve_time_cdf import (
    OKABE_ITO_PALETTE,
    EXPERIMENT_TIME_LIMIT_S,
    EXPECTED_EXPERIMENTS_PER_COMBINATION,
    MAP_COLUMN_CANDIDATES,
    TIME_COLUMN_CANDIDATES,
    AGENT_COLUMN_CANDIDATES,
    STATUS_COLUMN_CANDIDATES,
    CNF_VARIABLE_COLUMN_CANDIDATES,
    CNF_CLAUSE_COLUMN_CANDIDATES,
    LogInput,
    _find_column,
    _find_optional_column,
    _iter_log_sources,
    _normalise_agent_label,
    _normalise_map_label,
    _normalise_statuses,
    _normalise_time_values,
)

__all__ = (
    "collect_solver_map_agent_timelines",
    "plot_solver_map_agent_timelines",
    "summarise_solver_map_agent_stats",
)

DEFAULT_SOLVER_COLOURS = {
    "LNS-SAT": "#0072B2",  # Okabe–Ito blue
    "LNS": "#0072B2",  # Backwards compatibility with previous label
    "WHOLESOLVE": "#E69F00",  # Backwards compatibility with previous label
    "MDD-SAT": "#E69F00",  # Okabe–Ito orange
}

AGENT_STYLE_OVERRIDES = {
    "10": {"linestyle": "-", "linewidth": 2.0},
    "20": {"linestyle": "-", "linewidth": 2.0},
    "50": {"linestyle": "--", "linewidth": 2.0},
    "100": {"linestyle": ":", "linewidth": 2.0},
    "200": {"linestyle": ":", "linewidth": 2.0},
}

AGENT_COLOUR_TRANSFORMS = {
    "10": ("lighten", 0.65),
    "20": ("lighten", 0.35),
    "50": ("identity", 0.0),
    "100": ("lighten", 0.35),
    "200": ("darken", 0.25),
}

DEFAULT_LINEWIDTH = 2.2


def _blend_with_white(color: str, amount: float) -> str:
    """Return ``color`` blended towards white by ``amount`` (0-1)."""

    base = np.array(mcolors.to_rgb(color))
    white = np.ones(3)
    amount = np.clip(amount, 0.0, 1.0)
    blended = base + (white - base) * amount
    return mcolors.to_hex(blended, keep_alpha=False)


def _blend_with_black(color: str, amount: float) -> str:
    """Return ``color`` blended towards black by ``amount`` (0-1)."""

    base = np.array(mcolors.to_rgb(color))
    black = np.zeros(3)
    amount = np.clip(amount, 0.0, 1.0)
    blended = base + (black - base) * amount
    return mcolors.to_hex(blended, keep_alpha=False)


def _resolve_agent_style(agent_label: str) -> Dict[str, object]:
    """Return plotting keyword overrides for a given agent count label."""

    if agent_label in AGENT_STYLE_OVERRIDES:
        return dict(AGENT_STYLE_OVERRIDES[agent_label])

    try:
        numeric = float(agent_label)
    except (TypeError, ValueError):
        return {"linestyle": "-", "linewidth": DEFAULT_LINEWIDTH}

    if math.isfinite(numeric) and numeric.is_integer():
        integer_label = str(int(numeric))
        if integer_label in AGENT_STYLE_OVERRIDES:
            return dict(AGENT_STYLE_OVERRIDES[integer_label])

    return {"linestyle": "-", "linewidth": DEFAULT_LINEWIDTH}


def _resolve_agent_colour(base_colour: str, agent_label: str) -> str:
    """Derive a colour for ``agent_label`` starting from ``base_colour``."""

    if not base_colour:
        return base_colour

    transform = AGENT_COLOUR_TRANSFORMS.get(agent_label)
    if transform is None:
        try:
            numeric = float(agent_label)
        except (TypeError, ValueError):
            return base_colour
        if math.isfinite(numeric) and numeric.is_integer():
            transform = AGENT_COLOUR_TRANSFORMS.get(str(int(numeric)))

    if not transform:
        return base_colour

    name, amount = transform
    if name == "lighten":
        return _blend_with_white(base_colour, float(amount))
    if name == "darken":
        return _blend_with_black(base_colour, float(amount))
    return base_colour


SolverSources = Mapping[str, Sequence[LogInput]]


def _ensure_iterable_sources(sources: Sequence[LogInput]) -> list[LogInput]:
    """Normalise ``sources`` into a plain list for iteration."""

    if isinstance(sources, list):
        return sources
    return list(sources)


def _build_step_timeline(
    times: Sequence[float],
    *,
    time_limit: Optional[float],
    expected: Optional[int],
) -> Tuple[np.ndarray, np.ndarray, int]:
    """Convert per-instance runtimes into a stepped completion timeline."""

    if not times:
        if time_limit is None:
            return np.asarray([0.0]), np.asarray([0]), 0
        return np.asarray([0.0, float(time_limit)]), np.asarray([0, 0]), 0

    array = np.asarray(times, dtype=float)
    finite_mask = np.isfinite(array)
    if not np.all(finite_mask):
        array = array[finite_mask]
    if array.size == 0:
        if time_limit is None:
            return np.asarray([0.0]), np.asarray([0]), 0
        return np.asarray([0.0, float(time_limit)]), np.asarray([0, 0]), 0

    if expected is not None and array.size > expected:
        array = array[:expected]

    cumulative = np.cumsum(array, dtype=float)
    if time_limit is not None:
        cumulative = np.minimum(cumulative, float(time_limit))

    times_out = np.concatenate(([0.0], cumulative))
    counts_out = np.arange(times_out.size, dtype=int)

    completed = int(counts_out[-1])
    if (
        time_limit is not None
        and (
            expected is None
            or completed < int(expected)
        )
    ):
        final_time = float(time_limit)
        if times_out[-1] < final_time:
            times_out = np.concatenate((times_out, [final_time]))
            counts_out = np.concatenate((counts_out, [completed]))

    return times_out, counts_out, completed


def _compute_agent_stats(
    times: Sequence[float],
    *,
    cnf_variables: Optional[Sequence[float]],
    cnf_clauses: Optional[Sequence[float]],
    time_limit: Optional[float],
    expected: Optional[int],
) -> Dict[str, object]:
    """Return descriptive statistics for a map/agent combination."""

    if times:
        times_array = np.asarray(list(times), dtype=float)
        if times_array.size:
            finite_mask = np.isfinite(times_array)
            times_array = times_array[finite_mask]
    else:
        times_array = np.asarray([], dtype=float)

    if expected is not None and times_array.size > int(expected):
        times_array = times_array[: int(expected)]

    solved_instances = int(times_array.size)
    total_time = float(times_array.sum()) if solved_instances else 0.0

    finish_time: float
    if expected is not None:
        expected_int = int(expected)
        if solved_instances >= expected_int:
            if time_limit is not None and math.isfinite(time_limit):
                finish_time = float(min(total_time, float(time_limit)))
            else:
                finish_time = total_time
        else:
            finish_time = float(time_limit) if time_limit is not None else total_time
    else:
        if time_limit is not None and math.isfinite(time_limit):
            finish_time = (
                float(time_limit)
                if solved_instances == 0
                else float(min(total_time, float(time_limit)))
            )
        else:
            finish_time = total_time

    if solved_instances:
        solve_time_mean = float(times_array.mean())
        solve_time_std = float(times_array.std(ddof=0))
    else:
        solve_time_mean = float("nan")
        solve_time_std = float("nan")

    def _prepare_metric(values: Optional[Sequence[float]]) -> np.ndarray:
        if not values:
            return np.asarray([], dtype=float)
        array = np.asarray(list(values), dtype=float)
        if array.size:
            finite_mask = np.isfinite(array)
            array = array[finite_mask]
        if expected is not None and array.size > int(expected):
            array = array[: int(expected)]
        return array

    cnf_variables_array = _prepare_metric(cnf_variables)
    cnf_clauses_array = _prepare_metric(cnf_clauses)

    if cnf_variables_array.size:
        cnf_variables_mean = float(cnf_variables_array.mean())
        cnf_variables_std = float(cnf_variables_array.std(ddof=0))
    else:
        cnf_variables_mean = float("nan")
        cnf_variables_std = float("nan")

    if cnf_clauses_array.size:
        cnf_clauses_mean = float(cnf_clauses_array.mean())
        cnf_clauses_std = float(cnf_clauses_array.std(ddof=0))
    else:
        cnf_clauses_mean = float("nan")
        cnf_clauses_std = float("nan")

    return {
        "finishing_time_s": finish_time,
        "solved_instances": solved_instances,
        "solve_time_mean_s": solve_time_mean,
        "solve_time_std_s": solve_time_std,
        "cnf_variables_mean": cnf_variables_mean,
        "cnf_variables_std": cnf_variables_std,
        "cnf_clauses_mean": cnf_clauses_mean,
        "cnf_clauses_std": cnf_clauses_std,
    }


def collect_solver_map_agent_timelines(
    solver_sources: SolverSources,
    *,
    success_statuses: Optional[Sequence[str]] = None,
    skip_status_filter: bool = False,
    expected_per_combination: Optional[int] = EXPECTED_EXPERIMENTS_PER_COMBINATION,
    time_limit_s: Optional[float] = EXPERIMENT_TIME_LIMIT_S,
) -> Dict[str, object]:
    """Aggregate per-map agent completion timelines for each solver.

    The returned dictionary now contains a ``"stats"`` entry with per-map and
    per-agent descriptive statistics that back the tabular summary emitted by
    :func:`summarise_solver_map_agent_stats`.
    """

    success_statuses = success_statuses or ("SAT", "SUCCESS")
    normalised_successes = _normalise_statuses(success_statuses)

    solver_order: list[str] = []
    map_order: list[str] = []
    map_seen_global: set[str] = set()
    agents_per_map: MutableMapping[str, set[Optional[str]]] = {}
    solver_timelines: Dict[str, Dict[str, Dict[str, list[float]]]] = {}
    solver_metric_values: Dict[
        str, Dict[str, Dict[str, Dict[str, list[float]]]]
    ] = {}

    for solver_label, sources in solver_sources.items():
        solver_label_str = str(solver_label)
        if solver_label_str not in solver_order:
            solver_order.append(solver_label_str)
        solver_data: Dict[str, Dict[str, list[float]]] = {}
        solver_metrics_data: Dict[str, Dict[str, Dict[str, list[float]]]] = {}

        per_solver_sources: list[Tuple[str, LogInput]] = [
            (solver_label_str, source) for source in _ensure_iterable_sources(sources)
        ]

        for raw_df, _source in _iter_log_sources(per_solver_sources):
            df_all = raw_df.copy()
            df_all["__original_order__"] = np.arange(len(df_all))

            map_column = _find_column(df_all.columns, MAP_COLUMN_CANDIDATES)
            time_column = _find_column(df_all.columns, TIME_COLUMN_CANDIDATES)

            agent_column = None
            for candidate in AGENT_COLUMN_CANDIDATES:
                if candidate in df_all.columns:
                    agent_column = candidate
                    break

            if agent_column is None:
                raise KeyError(
                    "Agent-count column not found in log; expected one of "
                    f"{AGENT_COLUMN_CANDIDATES}."
                )

            df_all["__map__"] = df_all[map_column].map(_normalise_map_label)
            df_all["__time__"] = _normalise_time_values(df_all[time_column], time_column)
            df_all["__agents__"] = df_all[agent_column].map(_normalise_agent_label)

            cnf_variables_column = _find_optional_column(
                df_all.columns, CNF_VARIABLE_COLUMN_CANDIDATES
            )
            cnf_clauses_column = _find_optional_column(
                df_all.columns, CNF_CLAUSE_COLUMN_CANDIDATES
            )
            df_all["__cnf_variables__"] = np.nan
            df_all["__cnf_clauses__"] = np.nan
            if cnf_variables_column is not None:
                df_all["__cnf_variables__"] = pd.to_numeric(
                    df_all[cnf_variables_column], errors="coerce"
                )
            if cnf_clauses_column is not None:
                df_all["__cnf_clauses__"] = pd.to_numeric(
                    df_all[cnf_clauses_column], errors="coerce"
                )

            df_attempts = df_all[
                df_all["__map__"].notna() & df_all["__agents__"].notna()
            ].copy()
            if df_attempts.empty:
                continue

            df_success = df_attempts
            if not skip_status_filter:
                status_column = None
                for candidate in STATUS_COLUMN_CANDIDATES:
                    if candidate in df_all.columns:
                        status_column = candidate
                        break
                if status_column is not None:
                    status_series = df_success[status_column]
                    if is_bool_dtype(status_series):
                        df_success = df_success[status_series.fillna(False)]
                    elif is_numeric_dtype(status_series):
                        df_success = df_success[status_series.fillna(0) != 0]
                    else:
                        normalised_status_series = (
                            status_series.astype(str).map(str.upper).str.strip()
                        )
                        df_success = df_success[
                            normalised_status_series.isin(normalised_successes)
                        ]

            df_success = df_success.dropna(subset=["__time__"])
            df_success = df_success[np.isfinite(df_success["__time__"])].copy()

            group_columns = ["__map__", "__agents__"]
            for (map_label, agent_label), attempts_df in df_attempts.groupby(
                group_columns, dropna=False, sort=False
            ):
                if map_label is None or agent_label is None:
                    continue

                map_label_str = str(map_label)
                agent_label_str = str(agent_label)

                if map_label_str not in map_seen_global:
                    map_order.append(map_label_str)
                    map_seen_global.add(map_label_str)

                agents_set = agents_per_map.setdefault(map_label_str, set())
                if agent_label_str not in agents_set:
                    agents_set.add(agent_label_str)

                solver_map_data = solver_data.setdefault(map_label_str, {})
                combo_times = solver_map_data.setdefault(agent_label_str, [])
                solver_map_metrics = solver_metrics_data.setdefault(map_label_str, {})
                combo_metrics = solver_map_metrics.setdefault(
                    agent_label_str,
                    {"cnf_variables": [], "cnf_clauses": []},
                )

                if df_success.empty:
                    continue

                success_subset = df_success.loc[
                    df_success.index.intersection(attempts_df.index)
                ]
                if success_subset.empty:
                    continue

                ordered = success_subset.sort_values("__original_order__")
                times = ordered["__time__"].to_numpy(dtype=float)
                if times.size == 0:
                    continue

                finite_mask = np.isfinite(times)
                if not np.all(finite_mask):
                    times = times[finite_mask]
                if times.size == 0:
                    continue

                combo_times.extend(times.tolist())

                cnf_variables_values = success_subset["__cnf_variables__"].to_numpy(
                    dtype=float
                )
                if cnf_variables_values.size:
                    finite_vars = cnf_variables_values[np.isfinite(cnf_variables_values)]
                    if finite_vars.size:
                        combo_metrics["cnf_variables"].extend(finite_vars.tolist())

                cnf_clauses_values = success_subset["__cnf_clauses__"].to_numpy(
                    dtype=float
                )
                if cnf_clauses_values.size:
                    finite_clauses = cnf_clauses_values[np.isfinite(cnf_clauses_values)]
                    if finite_clauses.size:
                        combo_metrics["cnf_clauses"].extend(finite_clauses.tolist())

        solver_timelines[solver_label_str] = solver_data
        solver_metric_values[solver_label_str] = solver_metrics_data

    if not map_order:
        raise ValueError("No map data was discovered; verify the solver log inputs.")

    agent_order_by_map: Dict[str, list[str]] = {}
    for map_label, agent_set in agents_per_map.items():
        numeric_agents: list[Tuple[float, str]] = []
        text_agents: list[str] = []
        for agent in agent_set:
            try:
                numeric_agents.append((float(agent), str(agent)))
            except (TypeError, ValueError):
                text_agents.append(str(agent))
        numeric_agents.sort()
        text_agents.sort()
        ordered_agents = [label for _, label in numeric_agents]
        ordered_agents.extend(text_agents)
        agent_order_by_map[map_label] = ordered_agents

    all_agent_labels: list[str] = []
    for map_label in map_order:
        for agent_label in agent_order_by_map.get(map_label, []):
            if agent_label not in all_agent_labels:
                all_agent_labels.append(agent_label)

    timelines_per_solver: Dict[str, Dict[str, Dict[str, Dict[str, object]]]] = {}
    stats_per_solver: Dict[str, Dict[str, Dict[str, Dict[str, object]]]] = {}
    for solver_label in solver_order:
        solver_map_data = solver_timelines.get(solver_label, {})
        solver_map_metrics = solver_metric_values.get(solver_label, {})
        map_timelines: Dict[str, Dict[str, Dict[str, object]]] = {}
        map_stats: Dict[str, Dict[str, Dict[str, object]]] = {}
        for map_label in map_order:
            agent_timelines: Dict[str, Dict[str, object]] = {}
            agent_stats: Dict[str, Dict[str, object]] = {}
            solver_agents = solver_map_data.get(map_label, {})
            solver_agent_metrics = solver_map_metrics.get(map_label, {})
            for agent_label in agent_order_by_map.get(map_label, []):
                times = solver_agents.get(agent_label, [])
                metrics = solver_agent_metrics.get(
                    agent_label, {"cnf_variables": [], "cnf_clauses": []}
                )
                cnf_variables = metrics.get("cnf_variables", [])
                cnf_clauses = metrics.get("cnf_clauses", [])
                times_out, counts_out, completed = _build_step_timeline(
                    times,
                    time_limit=time_limit_s,
                    expected=expected_per_combination,
                )
                stats_entry = _compute_agent_stats(
                    times,
                    cnf_variables=cnf_variables,
                    cnf_clauses=cnf_clauses,
                    time_limit=time_limit_s,
                    expected=expected_per_combination,
                )
                agent_timelines[agent_label] = {
                    "times": times_out,
                    "counts": counts_out,
                    "completed": completed,
                    "stats": stats_entry,
                }
                agent_stats[agent_label] = stats_entry
            map_timelines[map_label] = agent_timelines
            map_stats[map_label] = agent_stats
        timelines_per_solver[solver_label] = map_timelines
        stats_per_solver[solver_label] = map_stats

    return {
        "solver_order": solver_order,
        "map_order": map_order,
        "agent_order_by_map": agent_order_by_map,
        "all_agent_labels": all_agent_labels,
        "timelines": timelines_per_solver,
        "stats": stats_per_solver,
        "time_limit_s": time_limit_s,
        "expected_per_combination": expected_per_combination,
    }


def summarise_solver_map_agent_stats(aggregated: Mapping[str, object]) -> pd.DataFrame:
    """Return a tidy DataFrame describing per-map/agent solver statistics."""

    solver_order = cast(Sequence[str], aggregated.get("solver_order", []))
    map_order = cast(Sequence[str], aggregated.get("map_order", []))
    agent_order_by_map = cast(
        Mapping[str, Sequence[str]], aggregated.get("agent_order_by_map", {})
    )
    stats_per_solver = cast(
        Mapping[str, Mapping[str, Mapping[str, Dict[str, object]]]],
        aggregated.get("stats", {}),
    )
    all_agent_labels = cast(Sequence[str], aggregated.get("all_agent_labels", []))

    rows: list[Dict[str, object]] = []
    for solver_label in solver_order:
        solver_stats = stats_per_solver.get(solver_label, {})
        for map_label in map_order:
            agent_labels = agent_order_by_map.get(map_label, [])
            map_stats = solver_stats.get(map_label, {})
            for agent_label in agent_labels:
                stats_entry = map_stats.get(agent_label, {})
                rows.append(
                    {
                        "solver": solver_label,
                        "map": map_label,
                        "agents": agent_label,
                        "finishing_time_s": float(
                            stats_entry.get("finishing_time_s", float("nan"))
                        ),
                        "solved_instances": int(
                            stats_entry.get("solved_instances", 0) or 0
                        ),
                        "solve_time_mean_s": float(
                            stats_entry.get("solve_time_mean_s", float("nan"))
                        ),
                        "solve_time_std_s": float(
                            stats_entry.get("solve_time_std_s", float("nan"))
                        ),
                        "cnf_variables_mean": float(
                            stats_entry.get("cnf_variables_mean", float("nan"))
                        ),
                        "cnf_variables_std": float(
                            stats_entry.get("cnf_variables_std", float("nan"))
                        ),
                        "cnf_clauses_mean": float(
                            stats_entry.get("cnf_clauses_mean", float("nan"))
                        ),
                        "cnf_clauses_std": float(
                            stats_entry.get("cnf_clauses_std", float("nan"))
                        ),
                    }
                )

    columns = [
        "solver",
        "map",
        "agents",
        "finishing_time_s",
        "solved_instances",
        "solve_time_mean_s",
        "solve_time_std_s",
        "cnf_variables_mean",
        "cnf_variables_std",
        "cnf_clauses_mean",
        "cnf_clauses_std",
    ]

    if not rows:
        return pd.DataFrame(columns=columns)

    df = pd.DataFrame(rows, columns=columns)

    if solver_order:
        df["solver"] = pd.Categorical(df["solver"], categories=solver_order, ordered=True)
    if map_order:
        df["map"] = pd.Categorical(df["map"], categories=map_order, ordered=True)
    if all_agent_labels:
        df["agents"] = pd.Categorical(
            df["agents"], categories=all_agent_labels, ordered=True
        )

    df = df.sort_values(["solver", "map", "agents"]).reset_index(drop=True)

    return df


def _print_stats_dataframe(stats_df: pd.DataFrame) -> None:
    """Pretty-print the per-map statistics DataFrame to standard output."""

    if stats_df.empty:
        print("\nPer-map agent statistics: no data available.\n")
        return

    print("\nPer-map agent statistics:\n")
    with pd.option_context(
        "display.max_rows",
        None,
        "display.max_columns",
        None,
        "display.width",
        None,
    ):
        print(stats_df.to_string(index=False, float_format=lambda x: f"{x:0.3f}"))


def plot_solver_map_agent_timelines(
    solver_sources: SolverSources,
    *,
    success_statuses: Optional[Sequence[str]] = None,
    skip_status_filter: bool = False,
    palette: Sequence[str] = OKABE_ITO_PALETTE,
    expected_per_combination: Optional[int] = EXPECTED_EXPERIMENTS_PER_COMBINATION,
    time_limit_s: Optional[float] = EXPERIMENT_TIME_LIMIT_S,
    title: Optional[str] = None,
    preferred_map_order: Optional[Sequence[str]] = None,
    figsize: Optional[Tuple[float, float]] = None,
    return_stats: bool = False,
    display_stats: Optional[bool] = None,
) -> Union[
    Tuple[plt.Figure, np.ndarray],
    Tuple[plt.Figure, np.ndarray, pd.DataFrame],
]:
    """Plot per-map agent completion curves for the provided solvers.

    When ``return_stats`` is ``True`` an additional :class:`pandas.DataFrame`
    containing the per-map/agent statistics is returned alongside the figure
    and axes array.  The table is printed automatically when
    ``display_stats`` is left at its default ``None`` value and
    ``return_stats`` is ``False``; explicitly set ``display_stats`` to ``True``
    or ``False`` to override the default behaviour.
    """

    aggregated = collect_solver_map_agent_timelines(
        solver_sources,
        success_statuses=success_statuses,
        skip_status_filter=skip_status_filter,
        expected_per_combination=expected_per_combination,
        time_limit_s=time_limit_s,
    )

    map_order: Sequence[str] = aggregated["map_order"]
    if preferred_map_order is not None:
        available_maps = list(map_order)
        preferred_seen: set[str] = set()
        ordered_maps: list[str] = []
        for preferred in preferred_map_order:
            preferred_label = str(preferred)
            if (
                preferred_label in available_maps
                and preferred_label not in preferred_seen
            ):
                ordered_maps.append(preferred_label)
                preferred_seen.add(preferred_label)
        ordered_maps.extend(
            map_label for map_label in available_maps if map_label not in preferred_seen
        )
        aggregated["map_order"] = ordered_maps
        map_order = ordered_maps
    solver_order: Sequence[str] = aggregated["solver_order"]
    agent_order_by_map: Mapping[str, Sequence[str]] = aggregated["agent_order_by_map"]
    timelines: Mapping[str, Mapping[str, Mapping[str, Dict[str, object]]]] = aggregated[
        "timelines"
    ]
    all_agent_labels: Sequence[str] = aggregated["all_agent_labels"]
    time_limit = aggregated["time_limit_s"]
    stats_df = summarise_solver_map_agent_stats(aggregated)
    palette = tuple(palette) if palette is not None else ()

    if display_stats is None:
        display_stats = not return_stats
    if display_stats:
        _print_stats_dataframe(stats_df)

    if not map_order:
        raise ValueError("No map data available for plotting.")

    n_maps = len(map_order)
    if n_maps == 1:
        n_rows, n_cols = 1, 1
    else:
        n_cols = min(3, max(1, int(math.ceil(math.sqrt(n_maps)))))
        n_rows = int(math.ceil(n_maps / n_cols))

    if figsize is None:
        figsize = (n_cols * 4.5, n_rows * 3.5)

    fig, axes = plt.subplots(
        n_rows,
        n_cols,
        sharex=True,
        sharey=True,
        figsize=figsize,
        squeeze=False,
    )

    solver_colors: Dict[str, str] = {}
    palette_length = len(palette)
    for idx, solver_label in enumerate(solver_order):
        normalised = solver_label.strip().upper()
        colour = DEFAULT_SOLVER_COLOURS.get(normalised)
        if colour is None:
            if palette_length:
                colour = palette[idx % palette_length]
            else:
                colour = f"C{idx}"
        solver_colors[solver_label] = colour

    max_completed = 0
    max_time = 0.0

    for index, map_label in enumerate(map_order):
        row = index // n_cols
        col = index % n_cols
        ax = axes[row, col]
        ax.set_title(map_label)

        for solver_label in solver_order:
            solver_map_timelines = timelines.get(solver_label, {})
            agent_timelines = solver_map_timelines.get(map_label, {})
            for agent_label in agent_order_by_map.get(map_label, []):
                data = agent_timelines.get(agent_label)
                if not data:
                    continue
                times = data["times"]
                counts = data["counts"]
                completed = int(data.get("completed", 0))
                max_completed = max(max_completed, completed)

                if isinstance(times, np.ndarray) and times.size:
                    max_time = max(max_time, float(np.nanmax(times)))
                elif times:
                    max_time = max(max_time, float(times[-1]))

                style_kwargs = _resolve_agent_style(agent_label)
                base_colour = solver_colors.get(
                    solver_label,
                    palette[0] if palette_length else "#333333",
                )
                colour = _resolve_agent_colour(base_colour, agent_label)
                line_kwargs: Dict[str, object] = {
                    "color": colour,
                    "linewidth": style_kwargs.get("linewidth", DEFAULT_LINEWIDTH),
                    "drawstyle": "steps-post",
                }
                for key, value in style_kwargs.items():
                    if key == "linewidth":
                        continue
                    if value is not None:
                        line_kwargs[key] = value

                ax.plot(times, counts, **line_kwargs)

        ax.grid(True, which="both", linestyle="--", linewidth=0.8, alpha=0.5)

        if row == n_rows - 1:
            ax.set_xlabel("Elapsed time [s]")
        if col == 0:
            ax.set_ylabel("Completed experiments")

    total_axes = n_rows * n_cols
    for blank_index in range(len(map_order), total_axes):
        row = blank_index // n_cols
        col = blank_index % n_cols
        axes[row, col].set_visible(False)

    if expected_per_combination is not None:
        y_max = max(int(expected_per_combination), max_completed)
    else:
        y_max = max_completed
    if y_max <= 0:
        y_max = max_completed if max_completed > 0 else 1
    y_margin = max(1.0, 0.05 * y_max)

    if time_limit is not None and math.isfinite(time_limit):
        x_max = float(time_limit)
    else:
        x_max = max_time
    x_max = max(x_max, max_time)
    if x_max <= 0:
        x_max = max_time if max_time > 0 else 1.0
    x_margin = max(5.0, 0.05 * x_max)

    for ax_row in axes:
        for ax in ax_row:
            if not ax.get_visible():
                continue
            ax.set_ylim(-y_margin, y_max)
            ax.set_xlim(-x_margin, x_max)

    fig.supxlabel("Elapsed time within map/agent configuration [s]")
    fig.supylabel("Completed experiments")

    solver_handles = [
        Line2D(
            [0],
            [0],
            color=solver_colors.get(solver, "#333333"),
            linestyle="-",
            linewidth=2.6,
            label=solver,
        )
        for solver in solver_order
    ]
    agent_handles = []
    legend_base_colour = None
    if solver_order:
        legend_base_colour = solver_colors.get(solver_order[0])
    if not legend_base_colour:
        legend_base_colour = palette[0] if palette_length else "#333333"
    for agent in all_agent_labels:
        style_kwargs = _resolve_agent_style(agent)
        handle_kwargs: Dict[str, object] = {
            "color": _resolve_agent_colour(legend_base_colour, agent),
            "linewidth": style_kwargs.get("linewidth", DEFAULT_LINEWIDTH),
        }
        for key, value in style_kwargs.items():
            if key == "linewidth":
                continue
            if value is not None:
                handle_kwargs[key] = value
        agent_handles.append(
            Line2D(
                [0],
                [0],
                label=f"agents={agent}",
                **handle_kwargs,
            )
        )

    legend_handles = solver_handles + agent_handles
    if legend_handles:
        fig.legend(
            legend_handles,
            [handle.get_label() for handle in legend_handles],
            loc="upper center",
            ncol=max(1, min(len(legend_handles), 4)),
            frameon=False,
            bbox_to_anchor=(0.5, 1.02),
        )

    if title:
        fig.suptitle(title, y=0.98)

    fig.tight_layout()

    if return_stats:
        return fig, axes, stats_df

    return fig, axes


def _parse_solver_arguments(entries: Iterable[Sequence[str]]) -> Dict[str, list[LogInput]]:
    """Convert ``--solver`` CLI arguments into a mapping."""

    solver_sources: Dict[str, list[LogInput]] = {}
    for entry in entries:
        if len(entry) < 2:
            raise ValueError("Each --solver argument requires a label and at least one source path.")
        label = entry[0]
        sources = entry[1:]
        target_list = solver_sources.setdefault(label, [])
        target_list.extend(Path(source) if not isinstance(source, pd.DataFrame) else source for source in sources)
    return solver_sources


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Plot per-map agent completion timelines for multiple solver roots. "
            "Provide --solver LABEL PATH [PATH ...] entries to aggregate the "
            "experiments.csv files under each root."
        )
    )
    parser.add_argument(
        "--solver",
        action="append",
        nargs="+",
        metavar=("LABEL", "SOURCE"),
        help=(
            "Solver label followed by one or more CSV files or directories. "
            "Repeat this option to compare multiple solvers."
        ),
        required=True,
    )
    parser.add_argument(
        "--status",
        action="append",
        dest="success_statuses",
        help=(
            "Status value to treat as a successful solve (default: SAT, SUCCESS). "
            "Repeat to specify multiple values."
        ),
    )
    parser.add_argument(
        "--skip-status-filter",
        action="store_true",
        help="Do not filter rows by solve status.",
    )
    parser.add_argument(
        "--title",
        help="Optional plot title.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Path to save the generated figure.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figure instead of saving it to disk.",
    )
    parser.add_argument(
        "--stats-csv",
        type=Path,
        help=(
            "Optional path to write the per-map statistics as CSV. The summary is "
            "also printed to stdout."
        ),
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_argument_parser()
    args = parser.parse_args(argv)

    try:
        solver_sources = _parse_solver_arguments(args.solver)
    except ValueError as exc:
        parser.error(str(exc))

    fig, _axes, stats_df = plot_solver_map_agent_timelines(
        solver_sources,
        success_statuses=args.success_statuses,
        skip_status_filter=args.skip_status_filter,
        title=args.title,
        return_stats=True,
        display_stats=False,
    )

    if args.stats_csv:
        stats_df.to_csv(args.stats_csv, index=False)

    _print_stats_dataframe(stats_df)

    if args.output:
        fig.savefig(args.output, bbox_inches="tight")
    if args.show or not args.output:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())