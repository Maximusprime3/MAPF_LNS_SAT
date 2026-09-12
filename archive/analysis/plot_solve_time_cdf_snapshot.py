"""Plot CDFs for the time required to solve all instances on each map.

This helper script ingests one or more CSV log files containing solver runs and
aggregates them by map to determine how long a solver needs (in total) to solve
all instances associated with that map.  The resulting per-map totals are then
visualised as cumulative distribution functions (CDFs).  The plot uses the
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
repository::

    from pathlib import Path
    from Analysis.plot_solve_time_cdf import plot_solver_time_cdf

    log_root = Path("lns_clean/logs/LNS")
    log_sources = [
        ("Berlin LNS", log_root / "Berlin" / "experiments.csv"),
        ("Paris LNS", log_root / "Paris" / "experiments.csv"),
        ("Warehouse LNS", log_root / "Warehouse2-2" / "experiments.csv"),
    ]
    ax = plot_solver_time_cdf(log_sources, title="LNS clean solve-time CDFs")

The helper returns the ``matplotlib`` axes object so that further customisation
can be applied inline.

By default only rows whose ``status`` column indicates a successful solve are
considered (values such as ``SAT`` or ``SUCCESS``).  This behaviour can be
overridden via the command-line flags documented below.

The loader recognises common column names used across the repository.  Map
paths are automatically reduced to their base filenames and runtime columns that
end in ``_ms`` are converted from milliseconds to seconds so that the resulting
CDFs remain comparable across datasets.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, Iterator, Mapping, MutableMapping, Sequence

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
SOLVER_COLUMN_CANDIDATES = ("solver", "solver_name", "algorithm")
STATUS_COLUMN_CANDIDATES = ("status", "result", "outcome", "solved")

LabelLike = str | Path

LogInput = (
    str
    | Path
    | pd.DataFrame
    | tuple[LabelLike, pd.DataFrame]
    | tuple[pd.DataFrame, LabelLike]
)

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


def _iter_log_sources(logs: Iterable[LogInput]) -> Iterator[tuple[pd.DataFrame, str | Path | None]]:
    """Yield dataframes paired with an optional label describing their origin."""

    for item in logs:
        if isinstance(item, tuple):
            if len(item) != 2:
                raise ValueError(
                    "Tuples passed as log sources must contain exactly two elements "
                    "(label, dataframe)."
                )
            first, second = item
            if isinstance(first, pd.DataFrame) and isinstance(second, (str, Path)):
                df, label = first, second
            elif isinstance(second, pd.DataFrame) and isinstance(first, (str, Path)):
                df, label = second, first
            else:
                raise TypeError(
                    "When using tuples as log sources, provide a pandas DataFrame and "
                    "a string or Path label."
                )
            yield df, label
            continue

        if isinstance(item, pd.DataFrame):
            label = item.attrs.get("solver_name") or item.attrs.get("source_path")
            yield item, label
            continue

        path = Path(item).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"CSV log file '{path}' does not exist")
        df = _load_csv(path)
        yield df, path


def _derive_default_solver_name(source: str | Path | None, fallback_index: int) -> str:
    """Derive a human-readable solver label when none is provided."""

    if isinstance(source, Path):
        stem = source.stem
        return stem or str(source)
    if source is None:
        return f"solver_{fallback_index}"

    source_str = str(source)
    stem = Path(source_str).stem
    return stem or source_str or f"solver_{fallback_index}"


def _normalise_map_label(value: object) -> str | None:
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


def _normalise_time_values(values: pd.Series, column_name: str) -> pd.Series:
    """Convert solver runtimes to seconds when a millisecond column is detected."""

    numeric = pd.to_numeric(values, errors="coerce")
    column_lower = column_name.lower()
    if column_lower.endswith("_ms") or column_lower.endswith("milliseconds"):
        return numeric / 1000.0
    return numeric


def _aggregate_times_by_map(
    data: pd.DataFrame,
    map_column: str,
    time_column: str,
) -> Mapping[str, float]:
    """Return total solve time per map for a single solver."""

    working = pd.DataFrame(
        {
            "__map__": data[map_column],
            "__time__": _normalise_time_values(data[time_column], time_column),
        }
    )
    working["__map__"] = working["__map__"].map(_normalise_map_label)
    working = working.dropna(subset=["__map__", "__time__"])

    grouped = working.groupby("__map__", dropna=True)["__time__"].sum()
    grouped = grouped[grouped.notna()]
    return {str(map_name): float(total_time) for map_name, total_time in grouped.items()}


def accumulate_solver_totals(
    log_sources: Iterable[LogInput],
    *,
    success_statuses: Sequence[str] | None = None,
    skip_status_filter: bool = False,
) -> Dict[str, pd.Series]:
    """Aggregate total solve times per map for each solver across solver logs.

    Parameters
    ----------
    log_sources:
        Iterable of CSV paths, :class:`pandas.DataFrame` objects, or
        ``(label, dataframe)`` tuples.  When passing bare dataframes the helper
        will attempt to infer a solver label from ``df.attrs['solver_name']`` or
        ``df.attrs['source_path']``; tuples make the label explicit.
    success_statuses:
        Optional collection of status labels treated as a successful solve.  The
        default matches the command-line interface (``"SAT"`` and
        ``"SUCCESS"``).
    skip_status_filter:
        When ``True`` no filtering by solver status is applied.
    """

    totals: MutableMapping[str, Dict[str, float]] = {}
    success_statuses = success_statuses or ("SAT", "SUCCESS")
    normalised_successes = _normalise_statuses(success_statuses)

    for index, (raw_df, source) in enumerate(_iter_log_sources(log_sources), start=1):
        df = raw_df.copy()

        map_column = _find_column(df.columns, MAP_COLUMN_CANDIDATES)
        time_column = _find_column(df.columns, TIME_COLUMN_CANDIDATES)

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

        for solver_name, solver_df in df.groupby(solver_column):
            per_map_totals = _aggregate_times_by_map(solver_df, map_column, time_column)
            if not per_map_totals:
                continue

            solver_totals = totals.setdefault(str(solver_name), {})
            for map_name, total_time in per_map_totals.items():
                solver_totals[map_name] = solver_totals.get(map_name, 0.0) + total_time

    return {
        solver: pd.Series(times).sort_values()
        for solver, times in totals.items()
        if times
    }


def plot_cdfs(
    totals_per_solver: Mapping[str, pd.Series],
    *,
    palette: Sequence[str] = OKABE_ITO_PALETTE,
    ax: plt.Axes | None = None,
    title: str | None = None,
    logx: bool = False,
) -> plt.Axes:
    """Plot CDFs of per-map total solve times for each solver."""

    if not totals_per_solver:
        raise ValueError("No solver totals were provided; check the input CSV files.")

    if ax is None:
        _, ax = plt.subplots(figsize=(8, 5))

    for idx, (solver, totals) in enumerate(sorted(totals_per_solver.items())):
        values = totals.to_numpy(dtype=float)
        if values.size == 0:
            continue

        sorted_values = np.sort(values)
        cdf = np.linspace(1 / values.size, 1.0, values.size)
        colour = palette[idx % len(palette)]
        ax.step(sorted_values, cdf, where="post", label=solver, color=colour, linewidth=2)

    ax.set_xlabel("Total time to solve all instances on map [s]")
    ax.set_ylabel("Cumulative fraction of maps")
    ax.set_ylim(0.0, 1.0)
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
    success_statuses: Sequence[str] | None = None,
    skip_status_filter: bool = False,
    palette: Sequence[str] = OKABE_ITO_PALETTE,
    ax: plt.Axes | None = None,
    title: str | None = None,
    logx: bool = False,
) -> plt.Axes:
    """Convenience wrapper for generating CDF plots in scripts or notebooks."""

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
            "Plot cumulative distribution functions (CDFs) of the time required "
            "to solve all instances on each map for one or more solver logs."
        )
    )
    parser.add_argument(
        "csv_logs",
        nargs="+",
        type=Path,
        help="Paths to CSV files containing solver logs.",
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
        if not path.is_file():
            raise FileNotFoundError(f"CSV log file '{path}' does not exist")

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

