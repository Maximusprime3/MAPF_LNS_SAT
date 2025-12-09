"""Plot CDFs describing how many experiments complete within a given time.

This helper ingests one or more CSV log files containing solver runs and
reconstructs, for each solver, when individual experiments finished.  Each
experiment contributes the cumulative wall time consumed within its map/agent
configuration, capped by the 600 second per-configuration budget used in the
``lns_clean`` experiments.  Missing experiment files are treated as 600 seconds
with zero completions, producing a flat plateau to highlight solver failures.
The resulting completion timelines are visualised as cumulative distribution
functions (CDFs) that answer the question: *"How many instances did this solver
finish within ``x`` seconds?"*  The plot uses the Okabe–Ito colour palette,
which is widely recommended for colour-blind safe figures in scientific
publications.

Typical usage from the repository root::

    python Analysis/plot_solve_time_cdf.py data/solver_log_minisat.csv \
        data/solver_log_probsat.csv --output map_solve_time_cdf.png

For interactive work (e.g. in JupyterLab) the module exposes
``plot_solver_time_cdf``.  It accepts paths or already-loaded
``pandas.DataFrame`` objects, making it straightforward to stitch into data
analysis notebooks::

    import pandas as pd
    from Analysis.plot_solve_time_cdf import plot_solver_time_cdf

    minisat = pd.read_csv("data/solver_log_minisat.csv")
    probsat = pd.read_csv("data/solver_log_probsat.csv")
    ax = plot_solver_time_cdf([("MiniSAT", minisat), ("ProbSAT", probsat)])

A concrete example using the LNS-clean experiment logs bundled with this
repository combines every Berlin, Paris, empty, rooms, and warehouse map under
each solver root::

    from pathlib import Path
    from Analysis.plot_solve_time_cdf import plot_solver_time_cdf

    log_sources = [
        ("LNS", Path("lns_clean/logs/LNS")),
        ("WholeSolve", Path("lns_clean/logs/WholeSolve")),
    ]
    ax = plot_solver_time_cdf(
        log_sources,
        title="LNS-clean cumulative experiment completions",
    )

Passing a directory automatically discovers every ``experiments.csv`` file
below it, ensuring that the CDF reflects all map/agent combinations present in
the logs.  The helper returns the ``matplotlib`` axes object so that further
customisation can be applied inline.  The cumulative timeline spans the full
map/agent horizon (10 maps × 5 agent counts × 600 seconds in the bundled logs),
so the resulting figure reaches 30,000 seconds on the x-axis when every
combination is available.

By default only rows whose ``status`` column indicates a successful solve are
considered (values such as ``SAT`` or ``SUCCESS``).  This behaviour can be
overridden via the command-line flags documented below.  Gaps in the log – for
example when an experiment exceeds the 600 second per-configuration budget –
manifest as plateaus in the resulting CDF because no additional completions are
counted beyond that point.

The loader recognises common column names used across the repository.  Map
paths are automatically reduced to their base filenames and runtime columns that
end in ``_ms`` are converted from milliseconds to seconds so that the resulting
CDFs remain comparable across datasets.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable as IterableCollection
from pathlib import Path
from typing import (
    Dict,
    Iterable,
    Iterator,
    Mapping,
    MutableMapping,
    Optional,
    Sequence,
    Tuple,
    Union,
)

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pandas.api.types import is_bool_dtype, is_numeric_dtype

# Research-grade colour-blind palette (Okabe–Ito).
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

MAP_COLUMN_CANDIDATES = (
    "map_name",
    "map",
    "instance",
    "mapfile",
    "map_path",
    "mapfile_path",
)
TIME_COLUMN_CANDIDATES = (
    "total_time_s",
    "total_time",
    "solve_time",
    "solver_time",
    "runtime_s",
    "time",
    "total_runtime_ms",
)
AGENT_COLUMN_CANDIDATES = (
    "num_agents",
    "agents",
    "n_agents",
    "agent_count",
)
SOLVER_COLUMN_CANDIDATES = ("solver", "solver_name", "algorithm")
STATUS_COLUMN_CANDIDATES = ("status", "result", "outcome", "solved")
CNF_VARIABLE_COLUMN_CANDIDATES = (
    "total_cnf_variables",
    "cnf_variables",
    "num_cnf_variables",
)
CNF_CLAUSE_COLUMN_CANDIDATES = (
    "total_cnf_clauses",
    "cnf_clauses",
    "num_cnf_clauses",
)

LabelLike = Union[str, Path]
PathLike = Union[str, Path]
LogCollection = Iterable[Union[PathLike, pd.DataFrame]]

EXPERIMENT_TIME_LIMIT_S = 600.0
EXPECTED_EXPERIMENTS_PER_COMBINATION = 100

LogInput = Union[
    PathLike,
    pd.DataFrame,
    Tuple[LabelLike, pd.DataFrame],
    Tuple[pd.DataFrame, LabelLike],
    Tuple[LabelLike, LogCollection],
    Tuple[LogCollection, LabelLike],
]

__all__ = (
    "OKABE_ITO_PALETTE",
    "EXPERIMENT_TIME_LIMIT_S",
    "EXPECTED_EXPERIMENTS_PER_COMBINATION",
    "accumulate_solver_totals",
    "plot_cdfs",
    "plot_solver_time_cdf",
)


def _find_column(columns: Sequence[str], candidates: Sequence[str]) -> str:
    """Return the first matching column name from ``candidates``.

    Raises a ``KeyError`` if none of the candidate names are present.
    """

    for name in candidates:
        if name in columns:
            return name
    raise KeyError(f"None of the candidate columns {candidates!r} were found")


def _find_optional_column(
    columns: Sequence[str], candidates: Sequence[str]
) -> Optional[str]:
    """Return the first matching candidate name or ``None`` if absent."""

    for name in candidates:
        if name in columns:
            return name
    return None


def _normalise_statuses(values: Iterable[str]) -> set[str]:
    """Normalise status labels for case-insensitive comparisons."""

    return {value.strip().upper() for value in values}


def _load_csv(path: Path) -> pd.DataFrame:
    """Load a CSV file and annotate it with its source path for debugging."""

    df = pd.read_csv(path)
    df.attrs["source_path"] = path
    return df


def _is_iterable_collection(value: object) -> bool:
    """Return ``True`` when ``value`` is an iterable collection of sources."""

    return isinstance(value, IterableCollection) and not isinstance(
        value, (str, bytes, Path, pd.DataFrame)
    )


def _discover_log_files(path: Path) -> list[Path]:
    """Expand directories into the CSV files they contain.

    Directories are searched recursively for ``experiments.csv`` files.  When
    none are present the helper falls back to all ``*.csv`` files in the tree.
    ``FileNotFoundError`` is raised when no CSV files are found.
    """

    expanded = Path(path).expanduser()
    if expanded.is_file():
        return [expanded]
    if expanded.is_dir():
        experiment_csvs = sorted(
            (candidate.resolve() for candidate in expanded.rglob("experiments.csv") if candidate.is_file()),
            key=str,
        )
        if experiment_csvs:
            # Remove duplicates while preserving order.
            seen: set[Path] = set()
            unique: list[Path] = []
            for candidate in experiment_csvs:
                if candidate not in seen:
                    unique.append(candidate)
                    seen.add(candidate)
            return unique

        other_csvs = sorted(
            (candidate.resolve() for candidate in expanded.rglob("*.csv") if candidate.is_file()),
            key=str,
        )
        if not other_csvs:
            raise FileNotFoundError(
                f"No CSV log files were found under directory '{expanded}'."
            )

        seen = set()
        unique = []
        for candidate in other_csvs:
            if candidate not in seen:
                unique.append(candidate)
                seen.add(candidate)
        return unique

    raise FileNotFoundError(f"CSV log file '{expanded}' does not exist")


def _expand_source_collection(
    source: Union[PathLike, pd.DataFrame, LogCollection],
    *,
    label: Optional[Union[str, Path]] = None,
) -> Iterator[Tuple[pd.DataFrame, Optional[Union[str, Path]]]]:
    """Yield dataframes paired with ``label`` from the provided ``source``."""

    if isinstance(source, pd.DataFrame):
        yield source, label
        return

    if isinstance(source, (str, Path)):
        for csv_path in _discover_log_files(Path(source)):
            df = _load_csv(csv_path)
            yield df, label or csv_path
        return

    if _is_iterable_collection(source):
        for element in source:
            yield from _expand_source_collection(element, label=label)
        return

    raise TypeError(
        "Unsupported log source type. Provide CSV paths, directories, pandas "
        "DataFrames, or iterables containing those." 
    )


def _iter_log_sources(
    logs: Iterable[LogInput],
) -> Iterator[Tuple[pd.DataFrame, Optional[Union[str, Path]]]]:
    """Yield dataframes paired with an optional label describing their origin."""

    for item in logs:
        if isinstance(item, tuple):
            if len(item) != 2:
                raise ValueError(
                    "Tuples passed as log sources must contain exactly two elements "
                    "(label, dataframe/path)."
                )
            first, second = item
            if isinstance(first, pd.DataFrame) and isinstance(second, (str, Path)):
                yield first, second
                continue
            if isinstance(second, pd.DataFrame) and isinstance(first, (str, Path)):
                yield second, first
                continue
            if isinstance(first, (str, Path)) and _is_iterable_collection(second):
                for df, label in _expand_source_collection(second, label=first):
                    yield df, label
                continue
            if isinstance(second, (str, Path)) and _is_iterable_collection(first):
                for df, label in _expand_source_collection(first, label=second):
                    yield df, label
                continue
            if isinstance(first, (str, Path)) and isinstance(second, (str, Path)):
                load_errors = []
                for maybe_label, maybe_source in ((first, second), (second, first)):
                    try:
                        for df, label in _expand_source_collection(
                            maybe_source, label=maybe_label
                        ):
                            yield df, label
                    except (FileNotFoundError, OSError, TypeError) as exc:
                        load_errors.append(exc)
                        continue
                    else:
                        break
                else:
                    if load_errors:
                        raise load_errors[0]
                continue
            raise TypeError(
                "When using tuples as log sources, provide either a pandas DataFrame "
                "and a string/Path label or a label paired with CSV paths, directories, "
                "or iterables of such entries."
            )

        if isinstance(item, pd.DataFrame):
            label = item.attrs.get("solver_name") or item.attrs.get("source_path")
            yield item, label
            continue

        for df, label in _expand_source_collection(item):
            yield df, label


def _derive_default_solver_name(
    source: Optional[Union[str, Path]], fallback_index: int
) -> str:
    """Derive a human-readable solver label when none is provided."""

    if isinstance(source, Path):
        stem = source.stem
        return stem or str(source)
    if source is None:
        return f"solver_{fallback_index}"

    source_str = str(source)
    stem = Path(source_str).stem
    return stem or source_str or f"solver_{fallback_index}"


def _normalise_map_label(value: object) -> Optional[str]:
    """Convert map identifiers into concise, human-readable labels."""

    if pd.isna(value):
        return None

    if isinstance(value, Path):
        if value.suffix:
            return value.stem or value.name
        return value.name or str(value)

    text = str(value).strip()
    if not text:
        return None

    path = Path(text)
    if path.suffix:
        return path.stem or path.name
    return path.name or text


def _normalise_agent_label(value: object) -> Optional[str]:
    """Normalise agent-count fields for use in combination labels."""

    if pd.isna(value):
        return None

    if isinstance(value, (int, np.integer)):
        return str(int(value))

    if isinstance(value, (float, np.floating)):
        if np.isnan(value):
            return None
        if float(value).is_integer():
            return str(int(value))
        return str(value)

    text = str(value).strip()
    if not text:
        return None

    try:
        numeric = float(text)
    except ValueError:
        return text
    if np.isnan(numeric):
        return None
    if numeric.is_integer():
        return str(int(numeric))
    return text


def _format_combination_label(map_label: str, agent_label: Optional[str]) -> str:
    """Generate a compact label describing a map/agent configuration."""

    if agent_label:
        return f"{map_label} (agents={agent_label})"
    return map_label


def _normalise_time_values(values: pd.Series, column_name: str) -> pd.Series:
    """Convert solver runtimes to seconds when a millisecond column is detected."""

    numeric = pd.to_numeric(values, errors="coerce")
    column_lower = column_name.lower()
    if column_lower.endswith("_ms") or column_lower.endswith("milliseconds"):
        return numeric / 1000.0
    return numeric


def accumulate_solver_totals(
    log_sources: Iterable[LogInput],
    *,
    success_statuses: Optional[Sequence[str]] = None,
    skip_status_filter: bool = False,
) -> Dict[str, pd.Series]:
    """Aggregate cumulative experiment completion times for each solver.

    Parameters
    ----------
    log_sources:
        Iterable of CSV paths, directories containing ``experiments.csv``
        files, :class:`pandas.DataFrame` objects, or ``(label, source)`` tuples.
        Sources may themselves be iterables, allowing multiple CSV files to be
        associated with a single solver label.  When passing bare dataframes the
        helper will attempt to infer a solver label from
        ``df.attrs['solver_name']`` or ``df.attrs['source_path']``; tuples make
        the label explicit.
    success_statuses:
        Optional collection of status labels treated as a successful solve.  The
        default matches the command-line interface (``"SAT"`` and
        ``"SUCCESS"``).
    skip_status_filter:
        When ``True`` no filtering by solver status is applied.

    Returns
    -------
    Dict[str, pandas.Series]
        Mapping from solver label to a series of completion times (seconds).
        Each series stores the total number of completed experiments in
        ``series.attrs['completed_total']`` alongside per-map/agent counts in
        ``series.attrs['combination_totals']``.  Additional metadata captures the
        stepped timeline used for plotting (``series.attrs['timeline']``), the
        block-by-block schedule inferred from the logs
        (``series.attrs['combination_blocks']``), the combined map/agent time
        horizon (``series.attrs['time_horizon_s']``), and the expected number of
        experiments per configuration (``series.attrs['expected_per_combination']``).
    """

    solver_combination_details: MutableMapping[
        str, Dict[Tuple[str, Optional[str]], Dict[str, object]]
    ] = {}
    combination_labels: Dict[Tuple[str, Optional[str]], str] = {}
    map_labels_seen: set[str] = set()
    map_label_order: list[str] = []
    agent_labels_seen: set[Optional[str]] = set()
    agent_label_order: list[Optional[str]] = []
    agents_by_map: Dict[str, set[Optional[str]]] = {}
    success_statuses = success_statuses or ("SAT", "SUCCESS")
    normalised_successes = _normalise_statuses(success_statuses)

    for index, (raw_df, source) in enumerate(_iter_log_sources(log_sources), start=1):
        df_all = raw_df.copy()
        df_all["__original_order__"] = np.arange(len(df_all))

        map_column = _find_column(df_all.columns, MAP_COLUMN_CANDIDATES)
        time_column = _find_column(df_all.columns, TIME_COLUMN_CANDIDATES)

        agent_column: Optional[str] = None
        for candidate in AGENT_COLUMN_CANDIDATES:
            if candidate in df_all.columns:
                agent_column = candidate
                break

        try:
            solver_column = _find_column(df_all.columns, SOLVER_COLUMN_CANDIDATES)
        except KeyError:
            solver_column = "__solver_from_source__"
            df_all[solver_column] = _derive_default_solver_name(source, index)

        status_column = None
        if not skip_status_filter:
            for candidate in STATUS_COLUMN_CANDIDATES:
                if candidate in df_all.columns:
                    status_column = candidate
                    break
        df_all["__map__"] = df_all[map_column].map(_normalise_map_label)
        df_all["__time__"] = _normalise_time_values(df_all[time_column], time_column)
        if agent_column is not None:
            df_all["__agents__"] = df_all[agent_column].map(_normalise_agent_label)

        df_attempts = df_all[df_all["__map__"].notna()].copy()
        if agent_column is not None:
            df_attempts = df_attempts[df_attempts["__agents__"].notna()]

        if df_attempts.empty:
            continue

        df_success = df_attempts
        if not skip_status_filter and status_column is not None:
            status_series = df_success[status_column]
            if is_bool_dtype(status_series):
                df_success = df_success[status_series.fillna(False)]
            elif is_numeric_dtype(status_series):
                df_success = df_success[status_series.fillna(0) != 0]
            else:
                normalised_statuses = status_series.astype(str).map(str.upper).str.strip()
                df_success = df_success[normalised_statuses.isin(normalised_successes)]

        df_success = df_success.dropna(subset=["__time__"])
        df_success = df_success[np.isfinite(df_success["__time__"])].copy()

        group_columns = [solver_column, "__map__"]
        if agent_column is not None:
            group_columns.append("__agents__")

        for group_key, attempts_df in df_attempts.groupby(
            group_columns, dropna=False, sort=False
        ):
            if agent_column is not None:
                solver_name, map_label, agent_label = group_key
            else:
                solver_name, map_label = group_key
                agent_label = None

            solver_name = str(solver_name)
            map_label = str(map_label)
            agent_label_str = str(agent_label) if agent_label is not None else None

            combo_key = (map_label, agent_label_str)
            if map_label not in map_labels_seen:
                map_labels_seen.add(map_label)
                map_label_order.append(map_label)
            if agent_label_str not in agent_labels_seen:
                agent_labels_seen.add(agent_label_str)
                agent_label_order.append(agent_label_str)

            agents_for_map = agents_by_map.setdefault(map_label, set())
            agents_for_map.add(agent_label_str)

            combination_labels.setdefault(
                combo_key, _format_combination_label(map_label, agent_label_str)
            )

            solver_data = solver_combination_details.setdefault(solver_name, {})
            combo_details = solver_data.setdefault(
                combo_key,
                {
                    "times": [],
                    "completed": 0,
                },
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

            combo_details.setdefault("times", [])
            combo_details.setdefault("completed", 0)
            combo_details["times"].extend(times.tolist())
            combo_details["completed"] = int(combo_details["completed"]) + times.size

    if not combination_labels:
        return {}

    if not agent_labels_seen:
        agent_labels_seen.add(None)

    numeric_agent_labels: list[Tuple[float, str]] = []
    non_numeric_agents: list[str] = []
    for label in agent_labels_seen:
        if label is None:
            continue
        try:
            numeric_agent_labels.append((float(label), label))
        except (TypeError, ValueError):
            non_numeric_agents.append(label)

    if non_numeric_agents:
        ordered_agents = [
            value
            for value in agent_label_order
            if value in agent_labels_seen and value is not None
        ]
    else:
        numeric_agent_labels.sort()
        ordered_agents = [label for _, label in numeric_agent_labels]

    if None in agent_labels_seen and None not in ordered_agents:
        ordered_agents.append(None)

    expected_map_labels = map_label_order or sorted(map_labels_seen)

    expected_combinations: list[Tuple[str, Optional[str]]] = []
    for map_label in expected_map_labels:
        agent_options = agents_by_map.get(map_label, set())
        if not agent_options:
            agent_options = {None}
        if non_numeric_agents:
            ordered_for_map = [
                value for value in agent_label_order if value in agent_options
            ]
        else:
            ordered_for_map = [
                value for value in ordered_agents if value in agent_options
            ]
        if None in agent_options and None not in ordered_for_map:
            ordered_for_map.append(None)
        if not ordered_for_map:
            numeric_candidates: list[Tuple[float, Optional[str]]] = []
            text_candidates: list[str] = []
            for value in agent_options:
                if value is None:
                    continue
                try:
                    numeric_candidates.append((float(value), value))
                except (TypeError, ValueError):
                    text_candidates.append(value)
            numeric_candidates.sort()
            text_candidates.sort()
            ordered_for_map = [label for _, label in numeric_candidates]
            ordered_for_map.extend(text_candidates)
            if None in agent_options:
                ordered_for_map.append(None)
        for agent_label in ordered_for_map:
            expected_combinations.append((map_label, agent_label))

    for combo_key in expected_combinations:
        combination_labels.setdefault(
            combo_key, _format_combination_label(combo_key[0], combo_key[1])
        )

    expected_per_combo = EXPECTED_EXPERIMENTS_PER_COMBINATION
    time_limit = EXPERIMENT_TIME_LIMIT_S

    result: Dict[str, pd.Series] = {}
    total_combinations = len(expected_combinations)
    expected_completion_total = (
        expected_per_combo * total_combinations
        if expected_per_combo is not None
        else None
    )
    time_horizon = time_limit * total_combinations if time_limit is not None else None

    for solver_name, solver_data in solver_combination_details.items():
        completion_event_times: list[float] = []
        timeline: list[Tuple[float, int]] = [(0.0, 0)]
        running_total = 0
        elapsed = 0.0
        combination_totals: Dict[str, int] = {}
        block_records: list[Dict[str, object]] = []

        for map_label, agent_label in expected_combinations:
            combo_key = (map_label, agent_label)
            label = combination_labels[combo_key]
            details = solver_data.get(combo_key, {})
            completed = int(details.get("completed", 0)) if details else 0
            combination_totals[label] = completed

            times_list = details.get("times") if details else []
            times_array = (
                np.asarray(times_list, dtype=float)
                if isinstance(times_list, (list, tuple, np.ndarray))
                else np.asarray([], dtype=float)
            )
            if times_array.size:
                finite_mask = np.isfinite(times_array)
                if not np.all(finite_mask):
                    times_array = times_array[finite_mask]
            if times_array.size and expected_per_combo is not None:
                times_array = times_array[: expected_per_combo]

            cumulative = (
                np.cumsum(times_array)
                if times_array.size
                else np.asarray([], dtype=float)
            )
            if time_limit is not None and cumulative.size:
                cumulative = np.minimum(cumulative, float(time_limit))

            for value in cumulative:
                event_time = elapsed + float(max(value, 0.0))
                running_total += 1
                timeline.append((event_time, running_total))
                completion_event_times.append(event_time)

            sum_times = float(times_array.sum()) if times_array.size else 0.0
            if time_limit is not None:
                if (
                    expected_per_combo is not None
                    and completed < expected_per_combo
                ):
                    block_duration = float(time_limit)
                else:
                    block_duration = float(min(sum_times, float(time_limit)))
            else:
                block_duration = sum_times

            block_end_time = elapsed + max(block_duration, 0.0)
            if not timeline or timeline[-1][0] != block_end_time:
                timeline.append((block_end_time, running_total))

            block_records.append(
                {
                    "map": map_label,
                    "agents": agent_label,
                    "label": label,
                    "completed": completed,
                    "expected": expected_per_combo,
                    "block_start_s": float(elapsed),
                    "block_end_s": float(block_end_time),
                }
            )

            elapsed = block_end_time

        if time_horizon is not None and timeline[-1][0] < float(time_horizon):
            timeline.append((float(time_horizon), running_total))

        series = pd.Series(
            sorted(completion_event_times),
            dtype=float,
            name="completion_time_s",
        )
        series.attrs["completed_total"] = int(running_total)
        if combination_totals:
            series.attrs["combination_totals"] = combination_totals
        series.attrs["timeline"] = timeline
        if time_horizon is not None:
            series.attrs["time_horizon_s"] = float(time_horizon)
        if expected_completion_total is not None:
            series.attrs["expected_completion_total"] = int(expected_completion_total)
        series.attrs["total_combinations"] = total_combinations
        if expected_per_combo is not None:
            series.attrs["expected_per_combination"] = int(expected_per_combo)
        if block_records:
            series.attrs["combination_blocks"] = block_records

        result[solver_name] = series

    return result


def plot_cdfs(
    totals_per_solver: Mapping[str, pd.Series],
    *,
    palette: Sequence[str] = OKABE_ITO_PALETTE,
    ax: Optional[plt.Axes] = None,
    title: Optional[str] = None,
    logx: bool = False,
) -> plt.Axes:
    """Plot CDFs of cumulative experiment completions for each solver."""

    if not totals_per_solver:
        raise ValueError("No solver totals were provided; check the input CSV files.")

    if ax is None:
        _, ax = plt.subplots(figsize=(8, 5))

    max_time_horizon = 0.0

    for idx, (solver, totals) in enumerate(sorted(totals_per_solver.items())):
        timeline = totals.attrs.get("timeline")
        colour = palette[idx % len(palette)]
        completed_total = totals.attrs.get("completed_total", 0)
        expected_total = totals.attrs.get("expected_completion_total")
        label = solver
        if completed_total and expected_total:
            label = f"{solver} ({completed_total}/{expected_total} completed)"
        elif completed_total:
            label = f"{solver} ({completed_total} completed)"

        if timeline:
            times, counts = zip(*timeline)
            times_array = np.asarray(times, dtype=float)
            counts_array = np.asarray(counts, dtype=float)
        else:
            values = totals.to_numpy(dtype=float)
            sorted_values = np.sort(values)
            times_array = np.concatenate(([0.0], sorted_values)) if sorted_values.size else np.array([0.0])
            counts_array = np.arange(times_array.size, dtype=float)

        if logx and times_array.size > 0 and times_array[0] <= 0:
            times_array = times_array[1:]
            counts_array = counts_array[1:]

        if times_array.size == 0:
            continue

        ax.step(
            times_array,
            counts_array,
            where="post",
            label=label,
            color=colour,
            linewidth=2,
        )

        time_horizon = totals.attrs.get("time_horizon_s")
        if time_horizon is not None:
            max_time_horizon = max(max_time_horizon, float(time_horizon))

    ax.set_xlabel("Elapsed wall time across map/agent configurations [s]")
    ax.set_ylabel("Cumulative completed experiments")
    ax.set_ylim(bottom=0)
    if max_time_horizon > 0:
        ax.set_xlim(0, max_time_horizon)
    else:
        ax.set_xlim(left=0)
    if logx:
        ax.set_xscale("log")
    if title:
        ax.set_title(title)

    ax.grid(True, which="both", linestyle="--", linewidth=0.8, alpha=0.5)
    ax.legend(title="Solver")

    return ax


def plot_solver_time_cdf(
    log_sources: Iterable[LogInput],
    *,
    success_statuses: Optional[Sequence[str]] = None,
    skip_status_filter: bool = False,
    palette: Sequence[str] = OKABE_ITO_PALETTE,
    ax: Optional[plt.Axes] = None,
    title: Optional[str] = None,
    logx: bool = False,
) -> plt.Axes:
    """Convenience wrapper for generating experiment-completion CDF plots."""

    totals = accumulate_solver_totals(
        log_sources,
        success_statuses=success_statuses,
        skip_status_filter=skip_status_filter,
    )
    return plot_cdfs(
        totals,
        palette=palette,
        ax=ax,
        title=title,
        logx=logx,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot cumulative distribution functions (CDFs) of how many experiments "
            "complete within a given wall-clock time for one or more solver logs."
        )
    )
    parser.add_argument(
        "csv_logs",
        nargs="+",
        type=Path,
        help="Paths to CSV files or directories containing solver logs.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path where the resulting figure should be saved.",
    )
    parser.add_argument(
        "--title",
        type=str,
        help="Custom title for the plot.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the plot in an interactive window after generating it.",
    )
    parser.add_argument(
        "--logx",
        action="store_true",
        help="Use a logarithmic scale for the x-axis.",
    )
    parser.add_argument(
        "--skip-status-filter",
        action="store_true",
        help=(
            "Do not filter rows by solver status.  By default only rows marked "
            "as successfully solved (e.g. status == 'SAT') are used."
        ),
    )
    parser.add_argument(
        "--success-status",
        dest="success_statuses",
        action="append",
        help=(
            "Status label that indicates a successful solve.  Can be provided "
            "multiple times.  Defaults to 'SAT' and 'SUCCESS'."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    csv_paths = [path.expanduser().resolve() for path in args.csv_logs]
    for path in csv_paths:
        if not path.exists():
            raise FileNotFoundError(
                f"CSV log file or directory '{path}' does not exist"
            )

    success_statuses = args.success_statuses or ["SAT", "SUCCESS"]

    ax = plot_solver_time_cdf(
        csv_paths,
        success_statuses=success_statuses,
        skip_status_filter=args.skip_status_filter,
        title=args.title,
        logx=args.logx,
    )

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        ax.figure.savefig(args.output, dpi=300, bbox_inches="tight")

    if args.show or not args.output:
        plt.show()


if __name__ == "__main__":
    main()