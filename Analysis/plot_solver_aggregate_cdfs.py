"""Aggregate solver experiment statistics and plot grouped completion CDFs.

This module complements :mod:`Analysis.plot_solve_time_cdf` and
:mod:`Analysis.plot_map_agent_timelines` by providing an aggregate view across
all maps and agent counts.  It ingests the ``experiments.csv`` logs (and their
matching ``makespan_attempts.csv`` files when available), reconstructs the
per-instance runtimes, and derives cumulative completion timelines for three
scenarios:

* Overall progress across every map/agent combination
* Per-agent comparisons (e.g. 10/20/50/100/200 agents)
* Per-map comparisons across all agent counts

The resulting grouped timelines are visualised as cumulative distribution
functions (CDFs) that mirror the 600 second experiment window used in the
``lns_clean`` dataset.  Alongside the plots the helper computes descriptive
statistics that summarise solve runtimes, CNF/MDD construction costs, SAT solver
wall times, and CNF size metrics.  Totals for runtime, CNF/MDD construction, SAT
solving, and the zone-related counters are also reported so notebook users can
correlate the plotted progress with the underlying resource consumption.

Typical interactive usage::

    from pathlib import Path
    from Analysis.plot_solver_aggregate_cdfs import plot_solver_aggregate_cdfs

    results = plot_solver_aggregate_cdfs(
        {
            "LNS-SAT": [Path("lns_clean/logs/LNS")],
            "MDD-SAT": [Path("lns_clean/logs/WholeSolve")],
        },
        title_overall="Aggregate completion CDFs",
    )

The function returns tuples of ``(figure, axes)`` for the overall, per-agent,
and per-map CDF grids.  When ``return_stats=True`` an additional trio of
:pandas:`pandas.DataFrame` objects describing the overall, per-agent, and
per-map statistics is included in the return value.  By default the tables are
printed to standard output so that notebook users immediately see the numerical
summaries alongside the generated plots.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from functools import lru_cache
from pathlib import Path
from typing import Dict, Iterable, Mapping, MutableMapping, Optional, Sequence, Tuple, Union

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pandas.api.types import is_bool_dtype, is_numeric_dtype

from Analysis.plot_solve_time_cdf import (
    AGENT_COLUMN_CANDIDATES,
    CNF_CLAUSE_COLUMN_CANDIDATES,
    CNF_VARIABLE_COLUMN_CANDIDATES,
    EXPERIMENT_TIME_LIMIT_S,
    EXPECTED_EXPERIMENTS_PER_COMBINATION,
    LogInput,
    MAP_COLUMN_CANDIDATES,
    STATUS_COLUMN_CANDIDATES,
    TIME_COLUMN_CANDIDATES,
    _find_column,
    _find_optional_column,
    _iter_log_sources,
    _normalise_agent_label,
    _normalise_map_label,
    _normalise_statuses,
    _normalise_time_values,
    plot_cdfs,
)

OKABE_ITO_PALETTE = (
    "#0072B2",  # blue
    "#D55E00",  # vermillion
    "#009E73",  # bluish green
    "#CC79A7",  # reddish purple
    "#F0E442",  # yellow
    "#56B4E9",  # sky blue
    "#E69F00",  # orange
    "#000000",  # black
)

CNF_BUILD_COLUMN_CANDIDATES = (
    "total_cnf_build_ms",
    "cnf_build_time_ms",
    "cnf_construction_ms",
)
MDD_BUILD_COLUMN_CANDIDATES = (
    "total_mdd_build_ms",
    "mdd_build_time_ms",
)
SAT_SOLVER_COLUMN_CANDIDATES = (
    "total_lazy_solver_wall_ms",
    "total_lazy_solver_reported_ms",
    "total_lazy_wall_ms",
)
EXPERIMENT_ID_COLUMN_CANDIDATES = ("experiment_id", "id")
SCENARIO_INDEX_COLUMN_CANDIDATES = ("scenario_index", "scen_index", "instance_index")
SEED_COLUMN_CANDIDATES = ("seed", "random_seed")
MAP_FAMILY_ORDER = ("empty", "room", "berlin", "warehouse")
EXPECTED_COUNT_OVERRIDES = {
    ("empty-16-16", "50"): 50,
}
ATTEMPT_ZONE_USED_CANDIDATES = ("zones_attempted", "zones_used")
ATTEMPT_ZONE_EXPANDED_CANDIDATES = ("zones_solved", "zones_expanded")
ATTEMPT_WAITING_COLUMN_CANDIDATES = (
    "total_waiting_attempts",
    "waiting_attempts",
)

SolverSources = Mapping[str, Sequence[LogInput]]

__all__ = (
    "collect_solver_aggregate_data",
    "filter_solver_sources_to_common_experiments",
    "plot_solver_aggregate_cdfs",
)


def _ensure_iterable_sources(sources: Sequence[LogInput]) -> list[LogInput]:
    """Normalise ``sources`` into a list for iteration."""

    if isinstance(sources, list):
        return sources
    return list(sources)


def _map_family_rank(map_label: object) -> tuple[int, str]:
    """Sort maps as empty, room, Berlin, warehouse, then any other maps."""

    label = str(map_label)
    lower_label = label.lower()
    for rank, prefix in enumerate(MAP_FAMILY_ORDER):
        if lower_label.startswith(prefix):
            return rank, lower_label
    return len(MAP_FAMILY_ORDER), lower_label


def _normalise_expected_count_overrides(
    overrides: Optional[Mapping[Tuple[object, object], int]] = None,
) -> Dict[Tuple[str, str], int]:
    """Normalise map/agent expected-count overrides to collected labels."""

    normalised: Dict[Tuple[str, str], int] = {}
    for (map_label, agent_label), count in (overrides or {}).items():
        normalised[
            (_normalise_map_label(map_label), _normalise_agent_label(agent_label))
        ] = int(count)
    return normalised


def _scaled_time_limit(
    *,
    time_limit_s: Optional[float],
    expected_count: Optional[int],
    expected_per_combination: Optional[int],
) -> Optional[float]:
    """Scale the block time budget when a block has fewer expected scenarios."""

    if time_limit_s is None:
        return None
    if (
        expected_count is None
        or expected_per_combination is None
        or int(expected_per_combination) <= 0
    ):
        return float(time_limit_s)
    return float(time_limit_s) * (int(expected_count) / int(expected_per_combination))


@lru_cache(maxsize=None)
def _load_attempt_summaries(experiments_path: Optional[Union[str, Path]]) -> Dict[str, Dict[str, float]]:
    """Return per-experiment zone statistics from ``makespan_attempts.csv``."""

    if not experiments_path:
        return {}

    path = Path(experiments_path)
    if path.is_dir():
        attempts_path = path / "makespan_attempts.csv"
    else:
        attempts_path = path.with_name("makespan_attempts.csv")

    if not attempts_path.exists():
        return {}

    attempts_df = pd.read_csv(attempts_path)
    if attempts_df.empty:
        return {}

    try:
        experiment_column = _find_column(attempts_df.columns, EXPERIMENT_ID_COLUMN_CANDIDATES)
    except KeyError:
        return {}

    zones_used_column = _find_optional_column(
        attempts_df.columns, ATTEMPT_ZONE_USED_CANDIDATES
    )
    zones_expanded_column = _find_optional_column(
        attempts_df.columns, ATTEMPT_ZONE_EXPANDED_CANDIDATES
    )
    waiting_column = _find_optional_column(
        attempts_df.columns, ATTEMPT_WAITING_COLUMN_CANDIDATES
    )

    numeric_columns = {}
    for name, column in (
        ("zones_used", zones_used_column),
        ("zones_expanded", zones_expanded_column),
        ("waiting_attempts", waiting_column),
    ):
        if column is None:
            continue
        numeric_columns[name] = pd.to_numeric(
            attempts_df[column], errors="coerce"
        ).fillna(0.0)

    if not numeric_columns:
        return {}

    for key, series in numeric_columns.items():
        attempts_df[f"__{key}__"] = series.to_numpy(dtype=float)

    grouped = attempts_df.groupby(experiment_column, sort=False)
    summaries: Dict[str, Dict[str, float]] = {}
    for experiment_id, group in grouped:
        summary = {
            "zones_used": float(group.get("__zones_used__", 0.0).sum())
            if "__zones_used__" in group
            else 0.0,
            "zones_expanded": float(group.get("__zones_expanded__", 0.0).sum())
            if "__zones_expanded__" in group
            else 0.0,
            "waiting_attempts": float(group.get("__waiting_attempts__", 0.0).sum())
            if "__waiting_attempts__" in group
            else 0.0,
        }
        summaries[str(experiment_id)] = summary

    return summaries


def _build_series_for_sequence(
    sequence: Sequence[Dict[str, object]],
    *,
    time_limit: Optional[float],
    expected_per_combination: Optional[int],
) -> pd.Series:
    """Convert a combination sequence into a completion-time series."""

    if not sequence:
        series = pd.Series(dtype=float, name="completion_time_s")
        series.attrs["completed_total"] = 0
        series.attrs["timeline"] = [(0.0, 0)]
        series.attrs["total_combinations"] = 0
        if expected_per_combination is not None:
            series.attrs["expected_per_combination"] = int(expected_per_combination)
            series.attrs["expected_completion_total"] = 0
        series.attrs["time_horizon_s"] = 0.0
        return series

    completion_event_times: list[float] = []
    timeline: list[Tuple[float, int]] = [(0.0, 0)]
    running_total = 0
    elapsed = 0.0
    combination_totals: Dict[str, int] = {}
    block_records: list[Dict[str, object]] = []

    for entry in sequence:
        label = str(entry.get("label", "")) or "combination"
        times = entry.get("times") or []
        array = np.asarray(list(times), dtype=float)
        if array.size:
            finite_mask = np.isfinite(array)
            if not np.all(finite_mask):
                array = array[finite_mask]
        entry_expected = entry.get("expected", expected_per_combination)
        if entry_expected is not None and array.size > int(entry_expected):
            array = array[: int(entry_expected)]

        entry_time_limit = entry.get("time_limit_s", time_limit)

        cumulative = np.cumsum(array, dtype=float) if array.size else np.asarray([], dtype=float)
        if entry_time_limit is not None and cumulative.size:
            cumulative = np.minimum(cumulative, float(entry_time_limit))

        for value in cumulative:
            event_time = elapsed + float(max(value, 0.0))
            running_total += 1
            timeline.append((event_time, running_total))
            completion_event_times.append(event_time)

        completed = int(array.size)
        combination_totals[label] = completed

        if entry_time_limit is not None:
            if entry_expected is not None and completed < int(entry_expected):
                block_duration = float(entry_time_limit)
            else:
                block_duration = float(min(array.sum(), float(entry_time_limit))) if array.size else 0.0
        else:
            block_duration = float(array.sum()) if array.size else 0.0

        block_end = elapsed + max(block_duration, 0.0)
        if (
            entry_time_limit is not None
            and entry_expected is not None
            and completed < int(entry_expected)
        ):
            block_end = elapsed + float(entry_time_limit)

        if not timeline or timeline[-1][0] != block_end:
            timeline.append((block_end, running_total))

        block_records.append(
            {
                "map": entry.get("map"),
                "agents": entry.get("agents"),
                "label": label,
                "completed": completed,
                "expected": entry_expected,
                "time_limit_s": entry_time_limit,
                "block_start_s": float(elapsed),
                "block_end_s": float(block_end),
            }
        )

        elapsed = block_end

    total_combinations = len(sequence)
    horizon = None
    if time_limit is not None:
        horizon = sum(
            float(entry.get("time_limit_s", time_limit) or 0.0)
            for entry in sequence
        )
        if timeline and timeline[-1][0] < horizon:
            timeline.append((horizon, running_total))

    series = pd.Series(sorted(completion_event_times), dtype=float, name="completion_time_s")
    series.attrs["completed_total"] = int(running_total)
    series.attrs["timeline"] = timeline
    series.attrs["total_combinations"] = total_combinations
    if horizon is not None:
        series.attrs["time_horizon_s"] = float(horizon)
    expected_total = sum(
        int(entry.get("expected", expected_per_combination) or 0)
        for entry in sequence
    )
    if expected_per_combination is not None:
        series.attrs["expected_per_combination"] = int(expected_per_combination)
    if expected_total:
        series.attrs["expected_completion_total"] = int(expected_total)
    if combination_totals:
        series.attrs["combination_totals"] = combination_totals
    if block_records:
        series.attrs["combination_blocks"] = block_records

    return series


def _compute_group_statistics(
    instances: pd.DataFrame,
    *,
    group_column: Optional[str],
    solver_order: Sequence[str],
    group_order: Optional[Sequence[str]] = None,
) -> pd.DataFrame:
    """Return descriptive statistics for ``instances`` grouped by ``group_column``."""

    columns = ["solver"]
    if group_column:
        columns.append(group_column)
    columns.extend(
        [
            "solved_instances",
            "solve_time_mean_s",
            "solve_time_std_s",
            "cnf_build_mean_s",
            "cnf_build_std_s",
            "mdd_build_mean_s",
            "mdd_build_std_s",
            "sat_solver_mean_s",
            "sat_solver_std_s",
            "cnf_variables_mean",
            "cnf_variables_std",
            "cnf_clauses_mean",
            "cnf_clauses_std",
            "total_solve_time_s",
            "total_cnf_build_time_s",
            "total_mdd_build_time_s",
            "total_sat_solver_time_s",
            "total_zones_used",
            "total_zones_expanded",
            "total_waiting_attempts",
        ]
    )

    if instances.empty:
        return pd.DataFrame(columns=columns)

    group_fields = ["solver"]
    if group_column:
        group_fields.append(group_column)

    grouped = instances.groupby(group_fields, dropna=False, sort=False)
    rows = []
    for key, group in grouped:
        if group_column:
            solver_label, group_value = key
        else:
            solver_label, group_value = key, None

        runtime = group["runtime_s"].dropna()
        cnf_build = group["cnf_build_s"].dropna()
        mdd_build = group["mdd_build_s"].dropna()
        sat_solver = group["sat_solver_s"].dropna()
        cnf_variables = group["cnf_variables"].dropna()
        cnf_clauses = group["cnf_clauses"].dropna()

        solved_instances = int(runtime.size)
        solve_time_mean = float(runtime.mean()) if solved_instances else float("nan")
        solve_time_std = float(runtime.std(ddof=0)) if solved_instances else float("nan")
        cnf_build_mean = float(cnf_build.mean()) if cnf_build.size else float("nan")
        cnf_build_std = float(cnf_build.std(ddof=0)) if cnf_build.size else float("nan")
        mdd_build_mean = float(mdd_build.mean()) if mdd_build.size else float("nan")
        mdd_build_std = float(mdd_build.std(ddof=0)) if mdd_build.size else float("nan")
        sat_solver_mean = float(sat_solver.mean()) if sat_solver.size else float("nan")
        sat_solver_std = float(sat_solver.std(ddof=0)) if sat_solver.size else float("nan")
        cnf_variables_mean = float(cnf_variables.mean()) if cnf_variables.size else float("nan")
        cnf_variables_std = float(cnf_variables.std(ddof=0)) if cnf_variables.size else float("nan")
        cnf_clauses_mean = float(cnf_clauses.mean()) if cnf_clauses.size else float("nan")
        cnf_clauses_std = float(cnf_clauses.std(ddof=0)) if cnf_clauses.size else float("nan")

        rows.append(
            {
                "solver": solver_label,
                **({group_column: group_value} if group_column else {}),
                "solved_instances": solved_instances,
                "solve_time_mean_s": solve_time_mean,
                "solve_time_std_s": solve_time_std,
                "cnf_build_mean_s": cnf_build_mean,
                "cnf_build_std_s": cnf_build_std,
                "mdd_build_mean_s": mdd_build_mean,
                "mdd_build_std_s": mdd_build_std,
                "sat_solver_mean_s": sat_solver_mean,
                "sat_solver_std_s": sat_solver_std,
                "cnf_variables_mean": cnf_variables_mean,
                "cnf_variables_std": cnf_variables_std,
                "cnf_clauses_mean": cnf_clauses_mean,
                "cnf_clauses_std": cnf_clauses_std,
                "total_solve_time_s": float(group["runtime_s"].sum(skipna=True)),
                "total_cnf_build_time_s": float(group["cnf_build_s"].sum(skipna=True)),
                "total_mdd_build_time_s": float(group["mdd_build_s"].sum(skipna=True)),
                "total_sat_solver_time_s": float(group["sat_solver_s"].sum(skipna=True)),
                "total_zones_used": float(group["zones_used"].fillna(0.0).sum()),
                "total_zones_expanded": float(group["zones_expanded"].fillna(0.0).sum()),
                "total_waiting_attempts": float(group["waiting_attempts"].fillna(0.0).sum()),
            }
        )

    df = pd.DataFrame(rows, columns=columns)

    if solver_order:
        df["solver"] = pd.Categorical(df["solver"], categories=solver_order, ordered=True)

    if group_column and group_order:
        df[group_column] = pd.Categorical(df[group_column], categories=group_order, ordered=True)
        complete_index = pd.MultiIndex.from_product(
            [solver_order, group_order], names=["solver", group_column]
        )
        df = df.set_index(["solver", group_column]).reindex(complete_index)
        numeric_zero_fields = [
            "total_solve_time_s",
            "total_cnf_build_time_s",
            "total_mdd_build_time_s",
            "total_sat_solver_time_s",
            "total_zones_used",
            "total_zones_expanded",
            "total_waiting_attempts",
        ]
        for field in numeric_zero_fields:
            if field in df.columns:
                df[field] = df[field].fillna(0.0)
        if "solved_instances" in df.columns:
            df["solved_instances"] = df["solved_instances"].fillna(0).astype(int)
        df = df.reset_index()
    else:
        df = df.sort_values("solver").reset_index(drop=True)

    if "solved_instances" in df.columns:
        df["solved_instances"] = df["solved_instances"].astype(int)

    return df


def _print_stats_table(name: str, df: pd.DataFrame) -> None:
    """Pretty-print ``df`` with a section heading."""

    heading = f"\n{name}:"
    print(heading)
    if df.empty:
        print("  (no data)\n")
        return

    with pd.option_context(
        "display.max_rows",
        None,
        "display.max_columns",
        None,
        "display.width",
        None,
    ):
        print(df.to_string(index=False, float_format=lambda value: f"{value:0.3f}"))
        print()


def collect_solver_aggregate_data(
    solver_sources: SolverSources,
    *,
    success_statuses: Optional[Sequence[str]] = None,
    skip_status_filter: bool = False,
    expected_per_combination: Optional[int] = EXPECTED_EXPERIMENTS_PER_COMBINATION,
    time_limit_s: Optional[float] = EXPERIMENT_TIME_LIMIT_S,
    expected_from_attempts: bool = False,
    expected_count_overrides: Optional[Mapping[Tuple[object, object], int]] = None,
) -> Dict[str, object]:
    """Return aggregated completion timelines and statistics for each solver."""

    success_statuses = success_statuses or ("SAT", "SUCCESS")
    normalised_successes = _normalise_statuses(success_statuses)
    expected_count_overrides = _normalise_expected_count_overrides(
        {**EXPECTED_COUNT_OVERRIDES, **(expected_count_overrides or {})}
    )

    solver_order: list[str] = []
    map_order: list[str] = []
    map_seen: set[str] = set()
    agents_per_map: MutableMapping[str, set[str]] = defaultdict(set)
    solver_combo_times: Dict[str, Dict[Tuple[str, str], list[float]]] = defaultdict(dict)
    solver_combo_presence: Dict[str, set[Tuple[str, str]]] = defaultdict(set)
    solver_combo_expected: Dict[str, Dict[Tuple[str, str], int]] = defaultdict(dict)
    solver_combo_time_limit: Dict[str, Dict[Tuple[str, str], Optional[float]]] = defaultdict(dict)
    instance_records: list[Dict[str, object]] = []

    for solver_label, sources in solver_sources.items():
        solver_label_str = str(solver_label)
        if solver_label_str not in solver_order:
            solver_order.append(solver_label_str)

        per_solver_sources: list[Tuple[str, LogInput]] = [
            (solver_label_str, source) for source in _ensure_iterable_sources(sources)
        ]

        for raw_df, source in _iter_log_sources(per_solver_sources):
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
            scenario_index_column = _find_optional_column(
                df_all.columns, SCENARIO_INDEX_COLUMN_CANDIDATES
            )
            seed_column = _find_optional_column(df_all.columns, SEED_COLUMN_CANDIDATES)
            df_all["__scenario_order__"] = (
                pd.to_numeric(df_all[scenario_index_column], errors="coerce")
                if scenario_index_column
                else df_all.groupby(["__map__", "__agents__"], sort=False).cumcount()
            )
            df_all["__seed_order__"] = (
                pd.to_numeric(df_all[seed_column], errors="coerce").fillna(0)
                if seed_column
                else 0
            )

            cnf_variables_column = _find_optional_column(df_all.columns, CNF_VARIABLE_COLUMN_CANDIDATES)
            cnf_clauses_column = _find_optional_column(df_all.columns, CNF_CLAUSE_COLUMN_CANDIDATES)
            cnf_build_column = _find_optional_column(df_all.columns, CNF_BUILD_COLUMN_CANDIDATES)
            mdd_build_column = _find_optional_column(df_all.columns, MDD_BUILD_COLUMN_CANDIDATES)
            sat_solver_column = _find_optional_column(df_all.columns, SAT_SOLVER_COLUMN_CANDIDATES)
            experiment_id_column = _find_optional_column(df_all.columns, EXPERIMENT_ID_COLUMN_CANDIDATES)

            df_all["__cnf_variables__"] = (
                pd.to_numeric(df_all[cnf_variables_column], errors="coerce")
                if cnf_variables_column
                else np.nan
            )
            df_all["__cnf_clauses__"] = (
                pd.to_numeric(df_all[cnf_clauses_column], errors="coerce")
                if cnf_clauses_column
                else np.nan
            )
            df_all["__cnf_build__"] = (
                _normalise_time_values(df_all[cnf_build_column], cnf_build_column)
                if cnf_build_column
                else np.nan
            )
            df_all["__mdd_build__"] = (
                _normalise_time_values(df_all[mdd_build_column], mdd_build_column)
                if mdd_build_column
                else np.nan
            )
            df_all["__sat_solver__"] = (
                _normalise_time_values(df_all[sat_solver_column], sat_solver_column)
                if sat_solver_column
                else np.nan
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
                        normalised_status = (
                            status_series.astype(str).map(str.upper).str.strip()
                        )
                        df_success = df_success[
                            normalised_status.isin(normalised_successes)
                        ]

            df_success = df_success.dropna(subset=["__time__"])
            df_success = df_success[np.isfinite(df_success["__time__"])].copy()

            source_path = raw_df.attrs.get("source_path") if hasattr(raw_df, "attrs") else None
            if source_path is None and isinstance(source, (str, Path)):
                source_path = Path(source)
            attempt_summaries = _load_attempt_summaries(source_path)

            group_columns = ["__map__", "__agents__"]
            for (map_label, agent_label), attempts_df in df_attempts.groupby(
                group_columns, dropna=False, sort=False
            ):
                if map_label is None or agent_label is None:
                    continue

                map_label_str = str(map_label)
                agent_label_str = str(agent_label)

                if map_label_str not in map_seen:
                    map_order.append(map_label_str)
                    map_seen.add(map_label_str)
                agents_per_map[map_label_str].add(agent_label_str)
                solver_combo_presence[solver_label_str].add((map_label_str, agent_label_str))

                combo_key = (map_label_str, agent_label_str)
                override_expected = expected_count_overrides.get(combo_key)
                if expected_from_attempts:
                    observed = int(len(attempts_df))
                    if expected_per_combination is not None:
                        observed = min(observed, int(expected_per_combination))
                    expected_count = (
                        min(override_expected, observed)
                        if override_expected is not None
                        else observed
                    )
                    solver_combo_expected[solver_label_str][combo_key] = expected_count
                elif override_expected is not None:
                    expected_count = override_expected
                    solver_combo_expected[solver_label_str][combo_key] = expected_count
                else:
                    expected_count = expected_per_combination
                solver_combo_time_limit[solver_label_str][combo_key] = _scaled_time_limit(
                    time_limit_s=time_limit_s,
                    expected_count=expected_count,
                    expected_per_combination=expected_per_combination,
                )

                if df_success.empty:
                    solver_combo_times[solver_label_str].setdefault(combo_key, [])
                    continue

                success_subset = df_success.loc[
                    df_success.index.intersection(attempts_df.index)
                ]
                if success_subset.empty:
                    solver_combo_times[solver_label_str].setdefault(combo_key, [])
                    continue

                ordered = success_subset.sort_values(
                    ["__scenario_order__", "__seed_order__", "__original_order__"],
                    kind="mergesort",
                )
                times = ordered["__time__"].to_numpy(dtype=float)
                finite_mask = np.isfinite(times)
                if not np.all(finite_mask):
                    indices = np.nonzero(finite_mask)[0]
                    times = times[finite_mask]
                    ordered = ordered.iloc[indices]

                if expected_count is not None and times.size > int(expected_count):
                    limit = int(expected_count)
                    times = times[:limit]
                    ordered = ordered.iloc[:limit]

                times_list = times.tolist()
                solver_combo_times[solver_label_str].setdefault(combo_key, []).extend(times_list)

                cnf_build_values = ordered["__cnf_build__"].to_numpy(dtype=float, copy=True)
                mdd_build_values = ordered["__mdd_build__"].to_numpy(dtype=float, copy=True)
                sat_solver_values = ordered["__sat_solver__"].to_numpy(dtype=float, copy=True)
                cnf_variable_values = ordered["__cnf_variables__"].to_numpy(dtype=float, copy=True)
                cnf_clause_values = ordered["__cnf_clauses__"].to_numpy(dtype=float, copy=True)

                if experiment_id_column and experiment_id_column in ordered.columns:
                    experiment_ids = ordered[experiment_id_column].astype(str).tolist()
                else:
                    experiment_ids = [None] * len(times_list)

                for idx, runtime_value in enumerate(times_list):
                    raw_experiment_id = experiment_ids[idx]
                    if raw_experiment_id is None:
                        experiment_key = None
                    else:
                        experiment_key = raw_experiment_id.strip()
                        if not experiment_key or experiment_key.lower() == "nan":
                            experiment_key = None
                    zone_summary = attempt_summaries.get(experiment_key or "", {})

                    instance_records.append(
                        {
                            "solver": solver_label_str,
                            "map": map_label_str,
                            "agents": agent_label_str,
                            "runtime_s": float(runtime_value),
                            "cnf_build_s": float(cnf_build_values[idx]) if idx < len(cnf_build_values) else float("nan"),
                            "mdd_build_s": float(mdd_build_values[idx]) if idx < len(mdd_build_values) else float("nan"),
                            "sat_solver_s": float(sat_solver_values[idx]) if idx < len(sat_solver_values) else float("nan"),
                            "cnf_variables": float(cnf_variable_values[idx]) if idx < len(cnf_variable_values) else float("nan"),
                            "cnf_clauses": float(cnf_clause_values[idx]) if idx < len(cnf_clause_values) else float("nan"),
                            "zones_used": float(zone_summary.get("zones_used", 0.0)),
                            "zones_expanded": float(zone_summary.get("zones_expanded", 0.0)),
                            "waiting_attempts": float(zone_summary.get("waiting_attempts", 0.0)),
                        }
                    )

    if not solver_order or not map_order:
        raise ValueError("No solver experiment data found; verify the provided sources.")

    map_order = sorted(map_order, key=_map_family_rank)

    agent_order_by_map: Dict[str, list[str]] = {}
    global_agent_seen: set[str] = set()
    for map_label, agent_set in agents_per_map.items():
        numeric_agents: list[Tuple[float, str]] = []
        text_agents: list[str] = []
        for agent_label in agent_set:
            global_agent_seen.add(agent_label)
            try:
                numeric_agents.append((float(agent_label), agent_label))
            except (TypeError, ValueError):
                text_agents.append(agent_label)
        numeric_agents.sort()
        text_agents.sort()
        ordered = [label for _, label in numeric_agents]
        ordered.extend(text_agents)
        agent_order_by_map[map_label] = ordered

    numeric_agents_global: list[Tuple[float, str]] = []
    text_agents_global: list[str] = []
    for agent_label in global_agent_seen:
        try:
            numeric_agents_global.append((float(agent_label), agent_label))
        except (TypeError, ValueError):
            text_agents_global.append(agent_label)
    numeric_agents_global.sort()
    text_agents_global.sort()
    agent_order = [label for _, label in numeric_agents_global]
    agent_order.extend(text_agents_global)

    overall_series: Dict[str, pd.Series] = {}
    series_by_agent: Dict[str, Dict[str, pd.Series]] = {}
    series_by_map: Dict[str, Dict[str, pd.Series]] = {}

    def _combo_expected_and_time_limit(
        solver_expected: Mapping[Tuple[str, str], int],
        solver_time_limit: Mapping[Tuple[str, str], Optional[float]],
        combo_key: Tuple[str, str],
    ) -> tuple[Optional[int], Optional[float]]:
        expected = solver_expected.get(
            combo_key,
            expected_count_overrides.get(combo_key, expected_per_combination),
        )
        block_time_limit = solver_time_limit.get(
            combo_key,
            _scaled_time_limit(
                time_limit_s=time_limit_s,
                expected_count=expected,
                expected_per_combination=expected_per_combination,
            ),
        )
        return expected, block_time_limit

    for solver_label in solver_order:
        solver_times = solver_combo_times.get(solver_label, {})
        solver_presence = solver_combo_presence.get(solver_label, set())
        solver_expected = solver_combo_expected.get(solver_label, {})
        solver_time_limit = solver_combo_time_limit.get(solver_label, {})

        overall_sequence = []
        for map_label in map_order:
            agent_labels = agent_order_by_map.get(map_label, [])
            for agent_label in agent_labels:
                combo_key = (map_label, agent_label)
                times = solver_times.get(combo_key, [])
                if combo_key not in solver_presence:
                    times = times or []
                combo_expected, combo_time_limit = _combo_expected_and_time_limit(
                    solver_expected, solver_time_limit, combo_key
                )
                overall_sequence.append(
                    {
                        "map": map_label,
                        "agents": agent_label,
                        "label": f"{map_label} (agents={agent_label})",
                        "times": times,
                        "expected": combo_expected,
                        "time_limit_s": combo_time_limit,
                    }
                )
        overall_series[solver_label] = _build_series_for_sequence(
            overall_sequence,
            time_limit=time_limit_s,
            expected_per_combination=expected_per_combination,
        )

    for agent_label in agent_order:
        per_agent_mapping: Dict[str, pd.Series] = {}
        for solver_label in solver_order:
            solver_times = solver_combo_times.get(solver_label, {})
            solver_expected = solver_combo_expected.get(solver_label, {})
            solver_time_limit = solver_combo_time_limit.get(solver_label, {})
            sequence = []
            for map_label in map_order:
                if agent_label not in agent_order_by_map.get(map_label, []):
                    continue
                combo_key = (map_label, agent_label)
                times = solver_times.get(combo_key, [])
                combo_expected, combo_time_limit = _combo_expected_and_time_limit(
                    solver_expected, solver_time_limit, combo_key
                )
                sequence.append(
                    {
                        "map": map_label,
                        "agents": agent_label,
                        "label": map_label,
                        "times": times,
                        "expected": combo_expected,
                        "time_limit_s": combo_time_limit,
                    }
                )
            per_agent_mapping[solver_label] = _build_series_for_sequence(
                sequence,
                time_limit=time_limit_s,
                expected_per_combination=expected_per_combination,
            )
        series_by_agent[agent_label] = per_agent_mapping

    for map_label in map_order:
        per_map_mapping: Dict[str, pd.Series] = {}
        agent_labels = agent_order_by_map.get(map_label, [])
        for solver_label in solver_order:
            solver_times = solver_combo_times.get(solver_label, {})
            solver_expected = solver_combo_expected.get(solver_label, {})
            solver_time_limit = solver_combo_time_limit.get(solver_label, {})
            sequence = []
            for agent_label in agent_labels:
                combo_key = (map_label, agent_label)
                times = solver_times.get(combo_key, [])
                combo_expected, combo_time_limit = _combo_expected_and_time_limit(
                    solver_expected, solver_time_limit, combo_key
                )
                sequence.append(
                    {
                        "map": map_label,
                        "agents": agent_label,
                        "label": f"agents={agent_label}",
                        "times": times,
                        "expected": combo_expected,
                        "time_limit_s": combo_time_limit,
                    }
                )
            per_map_mapping[solver_label] = _build_series_for_sequence(
                sequence,
                time_limit=time_limit_s,
                expected_per_combination=expected_per_combination,
            )
        series_by_map[map_label] = per_map_mapping

    instances_df = pd.DataFrame(instance_records)

    stats_overall = _compute_group_statistics(
        instances_df,
        group_column=None,
        solver_order=solver_order,
    )
    stats_by_agent = _compute_group_statistics(
        instances_df,
        group_column="agents",
        solver_order=solver_order,
        group_order=agent_order,
    )
    stats_by_map = _compute_group_statistics(
        instances_df,
        group_column="map",
        solver_order=solver_order,
        group_order=map_order,
    )

    return {
        "solver_order": solver_order,
        "map_order": map_order,
        "agent_order": agent_order,
        "agent_order_by_map": agent_order_by_map,
        "overall_series": overall_series,
        "series_by_agent": series_by_agent,
        "series_by_map": series_by_map,
        "stats_overall": stats_overall,
        "stats_by_agent": stats_by_agent,
        "stats_by_map": stats_by_map,
        "time_limit_s": time_limit_s,
        "expected_per_combination": expected_per_combination,
    }


def _experiment_key_columns(df: pd.DataFrame) -> tuple[str, str, Optional[str], Optional[str]]:
    """Return columns used to identify the same scenario across solver reruns."""

    map_column = _find_column(df.columns, MAP_COLUMN_CANDIDATES)
    agent_column = _find_column(df.columns, AGENT_COLUMN_CANDIDATES)
    scenario_column = _find_optional_column(df.columns, SCENARIO_INDEX_COLUMN_CANDIDATES)
    seed_column = _find_optional_column(df.columns, SEED_COLUMN_CANDIDATES)
    return map_column, agent_column, scenario_column, seed_column


def _experiment_key_frame(df: pd.DataFrame) -> pd.DataFrame:
    """Build comparable experiment identifiers independent of timestamped ids."""

    map_column, agent_column, scenario_column, seed_column = _experiment_key_columns(df)
    key_df = pd.DataFrame(
        {
            "__map__": df[map_column].map(_normalise_map_label),
            "__agents__": df[agent_column].map(_normalise_agent_label),
            "__scenario_index__": (
                df[scenario_column].astype(str).str.strip()
                if scenario_column
                else df.groupby([map_column, agent_column], sort=False).cumcount().astype(str)
            ),
            "__seed__": (
                df[seed_column].astype(str).str.strip()
                if seed_column
                else ""
            ),
        },
        index=df.index,
    )
    return key_df


def filter_solver_sources_to_common_experiments(
    solver_sources: SolverSources,
) -> tuple[Dict[str, list[pd.DataFrame]], pd.DataFrame]:
    """Filter logs to the exact map/agent/scenario/seed experiments shared by all solvers.

    The raw ``experiment_id`` values end in run-specific timestamps, so this helper
    matches runs using stable columns: normalised map name, agent count,
    ``scenario_index`` (or per-combination row order as a fallback), and seed.
    It returns in-memory dataframes suitable for ``plot_solver_aggregate_cdfs`` plus
    a per-map/per-agent summary of how many common experiments remain.
    """

    loaded: Dict[str, list[pd.DataFrame]] = {}
    keys_by_solver: Dict[str, set[tuple[str, str, str, str]]] = {}

    for solver_label, sources in solver_sources.items():
        solver_label_str = str(solver_label)
        loaded[solver_label_str] = []
        solver_keys: set[tuple[str, str, str, str]] = set()
        per_solver_sources = [
            (solver_label_str, source) for source in _ensure_iterable_sources(sources)
        ]
        for raw_df, _source in _iter_log_sources(per_solver_sources):
            df = raw_df.copy()
            key_df = _experiment_key_frame(df)
            df["__common_key__"] = list(map(tuple, key_df.to_numpy(dtype=str)))
            solver_keys.update(df["__common_key__"])
            loaded[solver_label_str].append(df)
        keys_by_solver[solver_label_str] = solver_keys

    if not keys_by_solver:
        return loaded, pd.DataFrame(columns=["map", "agents", "common_experiments"])

    common_keys = set.intersection(*keys_by_solver.values())
    filtered: Dict[str, list[pd.DataFrame]] = {}
    for solver_label, frames in loaded.items():
        filtered[solver_label] = [
            frame.loc[frame["__common_key__"].isin(common_keys)]
            .drop(columns=["__common_key__"])
            .copy()
            for frame in frames
        ]

    summary = pd.DataFrame(
        [
            {"map": key[0], "agents": key[1], "common_experiments": 1}
            for key in common_keys
        ]
    )
    if not summary.empty:
        summary = (
            summary.groupby(["map", "agents"], as_index=False)["common_experiments"]
            .sum()
        )
        summary["__map_rank__"] = summary["map"].map(_map_family_rank)
        summary["__agents_numeric__"] = pd.to_numeric(summary["agents"], errors="coerce")
        summary = (
            summary.sort_values(["__map_rank__", "__agents_numeric__", "agents"])
            .drop(columns=["__map_rank__", "__agents_numeric__"])
            .reset_index(drop=True)
        )
    return filtered, summary


def plot_solver_aggregate_cdfs(
    solver_sources: SolverSources,
    *,
    success_statuses: Optional[Sequence[str]] = None,
    skip_status_filter: bool = False,
    expected_per_combination: Optional[int] = EXPECTED_EXPERIMENTS_PER_COMBINATION,
    time_limit_s: Optional[float] = EXPERIMENT_TIME_LIMIT_S,
    expected_from_attempts: bool = False,
    expected_count_overrides: Optional[Mapping[Tuple[object, object], int]] = None,
    palette: Sequence[str] = OKABE_ITO_PALETTE,
    title_overall: Optional[str] = None,
    title_by_agent: Optional[str] = None,
    title_by_map: Optional[str] = None,
    figsize_overall: Optional[Tuple[float, float]] = None,
    figsize_agents: Optional[Tuple[float, float]] = None,
    figsize_maps: Optional[Tuple[float, float]] = None,
    return_stats: bool = False,
    display_stats: Optional[bool] = None,
):
    """Plot overall, per-agent, and per-map completion CDFs for each solver."""

    aggregated = collect_solver_aggregate_data(
        solver_sources,
        success_statuses=success_statuses,
        skip_status_filter=skip_status_filter,
        expected_per_combination=expected_per_combination,
        time_limit_s=time_limit_s,
        expected_from_attempts=expected_from_attempts,
        expected_count_overrides=expected_count_overrides,
    )

    stats_overall: pd.DataFrame = aggregated["stats_overall"]
    stats_by_agent: pd.DataFrame = aggregated["stats_by_agent"]
    stats_by_map: pd.DataFrame = aggregated["stats_by_map"]

    if display_stats is None:
        display_stats = not return_stats

    if display_stats:
        _print_stats_table("Overall statistics", stats_overall)
        _print_stats_table("Per-agent statistics", stats_by_agent)
        _print_stats_table("Per-map statistics", stats_by_map)

    overall_series: Mapping[str, pd.Series] = aggregated["overall_series"]
    series_by_agent: Mapping[str, Mapping[str, pd.Series]] = aggregated["series_by_agent"]
    series_by_map: Mapping[str, Mapping[str, pd.Series]] = aggregated["series_by_map"]

    fig_overall, ax_overall = plt.subplots(figsize=figsize_overall or (7.5, 5.0))
    plot_cdfs(
        overall_series,
        ax=ax_overall,
        palette=palette,
        title=title_overall or "Aggregate completion CDF",
    )

    agent_labels: Sequence[str] = aggregated["agent_order"]
    n_agents = max(1, len(agent_labels))
    agent_cols = min(3, n_agents)
    agent_rows = int(np.ceil(n_agents / agent_cols))
    fig_agents, axes_agents = plt.subplots(
        agent_rows,
        agent_cols,
        figsize=figsize_agents or (6.5, 2.8 * agent_rows),
        squeeze=False,
    )
    axes_agents_flat = axes_agents.flatten()
    for idx, agent_label in enumerate(agent_labels):
        ax = axes_agents_flat[idx]
        plot_cdfs(
            series_by_agent.get(agent_label, {}),
            ax=ax,
            palette=palette,
            title=f"Agents: {agent_label}",
        )
    for idx in range(len(agent_labels), axes_agents_flat.size):
        axes_agents_flat[idx].axis("off")
    if title_by_agent:
        fig_agents.suptitle(title_by_agent, y=0.98)
    fig_agents.tight_layout()

    map_labels: Sequence[str] = aggregated["map_order"]
    if map_labels:
        map_cols = min(5, len(map_labels))
        map_rows = int(np.ceil(len(map_labels) / map_cols))
    else:
        map_cols = 1
        map_rows = 1
    fig_maps, axes_maps = plt.subplots(
        map_rows,
        map_cols,
        figsize=figsize_maps or (7.5, 2.6 * map_rows),
        squeeze=False,
    )
    axes_maps_flat = axes_maps.flatten()
    for idx, map_label in enumerate(map_labels):
        ax = axes_maps_flat[idx]
        plot_cdfs(
            series_by_map.get(map_label, {}),
            ax=ax,
            palette=palette,
            title=map_label,
        )
    for idx in range(len(map_labels), axes_maps_flat.size):
        axes_maps_flat[idx].axis("off")
    if title_by_map:
        fig_maps.suptitle(title_by_map, y=0.98)
    fig_maps.tight_layout()

    plots = (
        (fig_overall, ax_overall),
        (fig_agents, axes_agents),
        (fig_maps, axes_maps),
    )

    if return_stats:
        return (
            plots,
            stats_overall,
            stats_by_agent,
            stats_by_map,
        )

    return plots


def _parse_solver_arguments(entries: Iterable[Sequence[str]]) -> Dict[str, list[LogInput]]:
    """Convert ``--solver`` CLI arguments into a mapping."""

    solver_sources: Dict[str, list[LogInput]] = {}
    for entry in entries:
        if len(entry) < 2:
            raise ValueError(
                "Each --solver argument requires a label and at least one source path."
            )
        label = entry[0]
        sources = entry[1:]
        target = solver_sources.setdefault(label, [])
        for source in sources:
            target.append(Path(source) if not isinstance(source, pd.DataFrame) else source)
    return solver_sources

def _parse_expected_count_overrides(
    entries: Optional[Iterable[Sequence[str]]],
) -> Dict[Tuple[str, str], int]:
    """Convert ``--expected-count MAP AGENTS COUNT`` entries into overrides."""

    overrides: Dict[Tuple[str, str], int] = {}
    for entry in entries or []:
        if len(entry) != 3:
            raise ValueError(
                "Each --expected-count argument requires MAP AGENTS COUNT."
            )
        map_label, agent_label, count_text = entry
        try:
            count = int(count_text)
        except ValueError as exc:
            raise ValueError(
                f"Expected-count override for {map_label!r}/{agent_label!r} "
                f"must be an integer, got {count_text!r}."
            ) from exc
        if count < 0:
            raise ValueError(
                f"Expected-count override for {map_label!r}/{agent_label!r} "
                "must be non-negative."
            )
        overrides[(map_label, agent_label)] = count
    return overrides


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Plot aggregate completion CDFs for multiple solver logs. Provide "
            "--solver LABEL PATH [PATH ...] entries to analyse different solvers."
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
        "--common-experiments",
        action="store_true",
        help=(
            "Before plotting, keep only map/agent/scenario/seed experiments that "
            "exist for every solver. This is useful when newer reruns have fewer "
            "entries than the original LNS/WholeSolve logs."
        ),
    )
    parser.add_argument(
        "--expected-from-attempts",
        action="store_true",
        help=(
            "Use the number of attempted rows in each map/agent block as that "
            "block's expected completion count instead of assuming 100."
        ),
    )
    parser.add_argument(
        "--expected-count",
        action="append",
        nargs=3,
        metavar=("MAP", "AGENTS", "COUNT"),
        help=(
            "Override the expected scenario count for one map/agent block. "
            "The block time budget is scaled by COUNT/100; repeat for multiple "
            "exceptions. Built-in default: empty-16-16 50 50."
        ),
    )
    parser.add_argument(
        "--title-overall",
        help="Optional title for the overall CDF plot.",
    )
    parser.add_argument(
        "--title-agents",
        help="Optional title for the per-agent CDF grid.",
    )
    parser.add_argument(
        "--title-maps",
        help="Optional title for the per-map CDF grid.",
    )
    parser.add_argument(
        "--output-overall",
        type=Path,
        help="Path to save the overall CDF figure.",
    )
    parser.add_argument(
        "--output-agents",
        type=Path,
        help="Path to save the per-agent CDF grid.",
    )
    parser.add_argument(
        "--output-maps",
        type=Path,
        help="Path to save the per-map CDF grid.",
    )
    parser.add_argument(
        "--stats-csv-prefix",
        type=Path,
        help=(
            "Optional path prefix for CSV exports. Files with _overall, _agents, "
            "and _maps suffixes are written when provided."
        ),
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figures instead of saving them to disk.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_argument_parser()
    args = parser.parse_args(argv)

    try:
        solver_sources = _parse_solver_arguments(args.solver)
        expected_count_overrides = _parse_expected_count_overrides(args.expected_count)
    except ValueError as exc:
        parser.error(str(exc))
        return 2

    common_summary = None
    if args.common_experiments:
        solver_sources, common_summary = filter_solver_sources_to_common_experiments(
            solver_sources
        )

    plots, stats_overall, stats_by_agent, stats_by_map = plot_solver_aggregate_cdfs(
        solver_sources,
        success_statuses=args.success_statuses,
        skip_status_filter=args.skip_status_filter,
        expected_from_attempts=args.expected_from_attempts or args.common_experiments,
        expected_count_overrides=expected_count_overrides,
        title_overall=args.title_overall,
        title_by_agent=args.title_agents,
        title_by_map=args.title_maps,
        return_stats=True,
        display_stats=False,
    )

    (fig_overall, _), (fig_agents, _), (fig_maps, _) = plots

    if args.stats_csv_prefix:
        prefix = args.stats_csv_prefix
        stats_overall.to_csv(prefix.with_name(prefix.name + "_overall.csv"), index=False)
        stats_by_agent.to_csv(prefix.with_name(prefix.name + "_agents.csv"), index=False)
        stats_by_map.to_csv(prefix.with_name(prefix.name + "_maps.csv"), index=False)

    _print_stats_table("Overall statistics", stats_overall)
    if common_summary is not None:
        _print_stats_table("Common experiment counts", common_summary)
    _print_stats_table("Per-agent statistics", stats_by_agent)
    _print_stats_table("Per-map statistics", stats_by_map)

    if args.output_overall:
        fig_overall.savefig(args.output_overall, bbox_inches="tight")
    if args.output_agents:
        fig_agents.savefig(args.output_agents, bbox_inches="tight")
    if args.output_maps:
        fig_maps.savefig(args.output_maps, bbox_inches="tight")

    if args.show or not any(
        (args.output_overall, args.output_agents, args.output_maps)
    ):
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())