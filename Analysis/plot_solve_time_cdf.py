"""Plot CDFs describing how many experiments complete within a given time.

This helper ingests one or more CSV log files containing solver runs and
reconstructs, for each solver, when individual experiments finished.  Each
experiment contributes the cumulative wall time consumed within its map/agent
configuration, allowing direct comparison between approaches such as LNS and
whole-solve strategies.  The resulting completion timelines are visualised as
cumulative distribution functions (CDFs) that answer the question: *"How many
instances did this solver finish within ``x`` seconds?"*  The plot uses the
Okabe–Ito colour palette, which is widely recommended for colour-blind safe
figures in scientific publications.

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
customisation can be applied inline.

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

LabelLike = Union[str, Path]
PathLike = Union[str, Path]
LogCollection = Iterable[Union[PathLike, pd.DataFrame]]

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
        Mapping from solver label to a sorted series of cumulative completion
        times (seconds).  Each series stores the total number of completed
        experiments in ``series.attrs['completed_total']`` and a mapping of
        per-map/agent counts in ``series.attrs['combination_totals']``.
    """

    totals: MutableMapping[str, list[float]] = {}
    combo_totals: MutableMapping[str, Dict[str, int]] = {}
    success_statuses = success_statuses or ("SAT", "SUCCESS")
    normalised_successes = _normalise_statuses(success_statuses)

    for index, (raw_df, source) in enumerate(_iter_log_sources(log_sources), start=1):
        df = raw_df.copy()
        df["__original_order__"] = np.arange(len(df))

        map_column = _find_column(df.columns, MAP_COLUMN_CANDIDATES)
        time_column = _find_column(df.columns, TIME_COLUMN_CANDIDATES)

        agent_column: Optional[str] = None
        for candidate in AGENT_COLUMN_CANDIDATES:
            if candidate in df.columns:
                agent_column = candidate
                break

        try:
            solver_column = _find_column(df.columns, SOLVER_COLUMN_CANDIDATES)
        except KeyError:
            solver_column = "__solver_from_source__"
            df[solver_column] = _derive_default_solver_name(source, index)

        status_column = None
        if not skip_status_filter:
            for candidate in STATUS_COLUMN_CANDIDATES:
                if candidate in df.columns:
                    status_column = candidate
                    break
            if status_column is not None:
                status_series = df[status_column]
                if is_bool_dtype(status_series):
                    df = df[status_series.fillna(False)]
                elif is_numeric_dtype(status_series):
                    df = df[status_series.fillna(0) != 0]
                else:
                    normalised_statuses = status_series.astype(str).map(str.upper).str.strip()
                    df = df[normalised_statuses.isin(normalised_successes)]

        df["__map__"] = df[map_column].map(_normalise_map_label)
        df["__time__"] = _normalise_time_values(df[time_column], time_column)
        if agent_column is not None:
            df["__agents__"] = df[agent_column].map(_normalise_agent_label)

        if agent_column is not None:
            df = df[df["__agents__"].notna()]

        df = df.dropna(subset=["__map__", "__time__"])
        if df.empty:
            continue

        group_columns = [solver_column, "__map__"]
        if agent_column is not None:
            group_columns.append("__agents__")

        for group_key, solver_df in df.groupby(group_columns, dropna=False, sort=False):
            if agent_column is not None:
                solver_name, map_label, agent_label = group_key
            else:
                solver_name, map_label = group_key
                agent_label = None

            solver_name = str(solver_name)
            map_label = str(map_label)
            ordered = solver_df.sort_values("__original_order__")
            times = ordered["__time__"].to_numpy(dtype=float)
            if times.size == 0:
                continue

            cumulative = np.cumsum(times)
            totals.setdefault(solver_name, []).extend(cumulative.tolist())

            combination_label = _format_combination_label(map_label, agent_label)
            combo_counts = combo_totals.setdefault(solver_name, {})
            combo_counts[combination_label] = combo_counts.get(combination_label, 0) + times.size

    result: Dict[str, pd.Series] = {}
    for solver_name, completion_times in totals.items():
        if not completion_times:
            continue
        series = pd.Series(sorted(completion_times), dtype=float, name="completion_time_s")
        series.attrs["completed_total"] = len(completion_times)
        if solver_name in combo_totals:
            series.attrs["combination_totals"] = dict(sorted(combo_totals[solver_name].items()))
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

    for idx, (solver, totals) in enumerate(sorted(totals_per_solver.items())):
        values = totals.to_numpy(dtype=float)
        if values.size == 0:
            continue

        sorted_values = np.sort(values)
        counts = np.arange(1, values.size + 1)
        colour = palette[idx % len(palette)]
        completed_total = totals.attrs.get("completed_total", values.size)
        label = solver
        if completed_total:
            label = f"{solver} ({completed_total} completed)"
        ax.step(sorted_values, counts, where="post", label=label, color=colour, linewidth=2)

    ax.set_xlabel("Elapsed wall time within configuration [s]")
    ax.set_ylabel("Cumulative completed experiments")
    ax.set_ylim(bottom=0)
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
