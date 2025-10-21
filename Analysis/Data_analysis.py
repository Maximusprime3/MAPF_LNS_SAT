Analysis/load_lns_logs.py
New
+701-0
"""Utilities for loading LNS and Wholesolve log CSVs into pandas DataFrames.

The dataset expected by this module should live under ``lns_clean/logs`` with the
following structure::

    lns_clean/logs/
        LNS/
            <map_name>/
                <run>.csv
        Wholesolve/
            <map_name>/
                <run>.csv

Each CSV file is loaded and annotated with metadata derived from the folder
structure (``solver`` and ``map_name``) as well as heuristics that try to
extract ``run_id`` and ``agent_count`` from the file name.  The resulting
DataFrame can then be used for downstream exploratory analysis.

This script can also be executed directly from the command line.  When invoked
as a CLI it prints a short summary about the discovered files, optionally writes
an aggregated CSV/Parquet file, and (when ``--summary`` is provided) prints
per-solver mean/std statistics for time-related metrics as well as CNF clause
and variable counts aggregated by map/agent combinations and solver totals.
When LNS logs are present the summary also highlights runs that were solved by
the initial solution (i.e. without entering the LNS improvement loop) and, for
the remaining runs, reports zone-related, waiting-time, and spatial metrics that
characterise the behaviour of the LNS solver.
"""

from __future__ import annotations

import argparse
import logging
import re
from pathlib import Path
from typing import Iterable, Iterator, List, Optional, Sequence, Tuple

import pandas as pd

TIME_METRIC_KEYWORDS = ("time", "duration")
CNF_METRIC_KEYWORDS = ("cnf", "clause", "clauses", "variable", "variables")

INITIAL_SOLUTION_INDICATOR_KEYWORDS = (
    "initial_solution",
    "initial-solution",
    "initialonly",
    "initial_only",
    "solved_by_initial",
    "solvedinitial",
    "no_inner_loop",
)
ITERATION_KEYWORDS = ("iteration", "inner_loop")

ZONE_COUNT_MARKERS = (
    "count",
    "constructed",
    "construct",
    "build",
    "built",
    "create",
    "created",
    "num",
    "number",
    "total",
)
ZONE_SIZE_MARKERS = ("size", "width", "height")
ZONE_GROWTH_MARKERS = ("growth", "expand", "expansion")
SPATIAL_KEYWORDS = ("spatial", "space")

# Metadata columns that we append to each log DataFrame.  These columns are also
# ignored when automatically building summaries of numeric metrics.
METADATA_COLUMNS = {
    "solver",
    "map_name",
    "run_name",
    "run_id",
    "agent_count",
    "source_file",
}


def _iter_log_files(base_dir: Path) -> Iterator[Tuple[str, str, Path]]:
    """Yield ``(solver, map_name, csv_path)`` tuples for all CSV logs.

    Parameters
    ----------
    base_dir:
        Root directory that contains the ``LNS`` and ``Wholesolve`` subfolders.

    Yields
    ------
    tuple[str, str, Path]
        A tuple containing the solver name, map name, and absolute path to the
        CSV file.
    """

    base_dir = base_dir.expanduser().resolve()
    if not base_dir.exists():
        raise FileNotFoundError(f"Log directory '{base_dir}' does not exist")

    for solver_dir in sorted(p for p in base_dir.iterdir() if p.is_dir()):
        solver_name = solver_dir.name
        for map_dir in sorted(p for p in solver_dir.iterdir() if p.is_dir()):
            map_name = map_dir.name
            for csv_path in sorted(map_dir.rglob("*.csv")):
                if csv_path.is_file():
                    yield solver_name, map_name, csv_path


def _parse_run_metadata(
    file_stem: str,
    expected_agent_counts: Optional[Iterable[int]] = None,
) -> dict:
    """Extract run metadata from a CSV file name.

    Parameters
    ----------
    file_stem:
        The file name without the extension.
    expected_agent_counts:
        Optional iterable of agent counts that we expect to encounter.  This is
        used as a safeguard when heuristically parsing integers so that we do
        not accidentally interpret other numbers (e.g. coordinates embedded in
        the map name) as the agent count.

    Returns
    -------
    dict
        Dictionary containing zero or more of the keys ``run_name``,
        ``run_id``, and ``agent_count`` depending on what could be inferred.
    """

    metadata = {"run_name": file_stem}
    expected = set(expected_agent_counts or [])

    # Attempt to find an explicit ``run`` identifier (e.g. ``run3`` or
    # ``_run_3``) in the file name.
    run_match = re.search(r"run[-_]?(\d+)", file_stem, flags=re.IGNORECASE)
    if run_match:
        metadata["run_id"] = int(run_match.group(1))

    # Prefer patterns where the number is directly associated with the word
    # "agent" to avoid picking up unrelated numbers from the map name.
    agent_patterns: Sequence[re.Pattern[str]] = (
        re.compile(r"(?:^|[_-])(\d+)(?=agents?(?:[_-]|$))", re.IGNORECASE),
        re.compile(r"(?<=agents?[_-])(\d+)(?:(?=[_-])|$)", re.IGNORECASE),
        re.compile(r"agents?(\d+)", re.IGNORECASE),
    )
    for pattern in agent_patterns:
        match = pattern.search(file_stem)
        if match:
            candidate = int(match.group(1))
            if not expected or candidate in expected:
                metadata["agent_count"] = candidate
                return metadata

    if expected:
        # As a fallback, walk the tokens in reverse order and pick the first
        # integer that matches an expected agent count.  This accommodates file
        # names such as ``<map>_10.csv`` while still avoiding false positives.
        tokens = re.split(r"[_-]", file_stem)
        for token in reversed(tokens):
            if token.isdigit():
                candidate = int(token)
                if candidate in expected:
                    metadata["agent_count"] = candidate
                    break

    return metadata


def load_solver_logs(
    base_dir: Path | str = Path("lns_clean/logs"),
    expected_agent_counts: Optional[Iterable[int]] = (10, 20, 50, 100, 200),
    verbose: bool = False,
) -> pd.DataFrame:
    """Load all solver log CSVs found below ``base_dir`` into a DataFrame.

    Parameters
    ----------
    base_dir:
        Path to the directory that contains solver subdirectories (``LNS`` and
        ``Wholesolve``).
    expected_agent_counts:
        Optional iterable specifying the agent counts that should be considered
        valid when parsing metadata from file names.  Provide ``None`` to disable
        the safeguard entirely.
    verbose:
        When ``True`` log the path of each file as it is loaded.  This is
        primarily useful for debugging.

    Returns
    -------
    pandas.DataFrame
        A DataFrame containing the concatenated contents of all CSV logs with
        metadata columns appended.  If no files are discovered an empty
        DataFrame with the metadata columns is returned.
    """

    base_path = Path(base_dir)
    frames: List[pd.DataFrame] = []

    logger = logging.getLogger(__name__)
    if verbose:
        logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger.addHandler(handler)

    for solver, map_name, csv_path in _iter_log_files(base_path):
        if verbose:
            logger.info("Loading %s", csv_path)
        df = pd.read_csv(csv_path)
        df = df.copy()
        df["solver"] = solver
        df["map_name"] = map_name
        run_metadata = _parse_run_metadata(
            csv_path.stem, expected_agent_counts=expected_agent_counts
        )
        for key, value in run_metadata.items():
            df[key] = value
        df["source_file"] = str(csv_path.relative_to(base_path))
        frames.append(df)

    if not frames:
        return pd.DataFrame(columns=sorted(METADATA_COLUMNS))

    combined = pd.concat(frames, ignore_index=True, sort=False)
    return combined


def build_numeric_summary(df: pd.DataFrame) -> pd.DataFrame:
    """Aggregate numeric metrics grouped by solver/map/agent count.

    The function automatically detects numeric columns (excluding metadata) and
    computes ``mean``, ``median``, ``min``, ``max``, and ``count`` for each
    metric.  Grouping keys are chosen from ``solver``, ``map_name`` and
    ``agent_count`` if present.
    """

    if df.empty:
        return df

    numeric_columns = [
        col for col in df.select_dtypes(include=["number"]).columns
        if col not in METADATA_COLUMNS
    ]
    if not numeric_columns:
        return pd.DataFrame()

    grouping_columns = [col for col in ("solver", "map_name", "agent_count") if col in df]
    if not grouping_columns:
        grouping_columns = ["solver"] if "solver" in df else []

    grouped = (
        df.groupby(grouping_columns)[numeric_columns]
        .agg(["mean", "median", "min", "max", "count"])
    )
    grouped = grouped.reset_index()
    # Flatten the MultiIndex columns produced by ``agg``.
    grouped.columns = [
        "_".join(str(part) for part in col if part)
        for col in grouped.columns.to_flat_index()
    ]
    return grouped


def _select_metric_columns(df: pd.DataFrame, keywords: Sequence[str]) -> List[str]:
    numeric_columns = [
        col for col in df.select_dtypes(include=["number"]).columns
        if col not in METADATA_COLUMNS
    ]
    lower_keywords = tuple(keyword.lower() for keyword in keywords)
    selected = [
        col
        for col in numeric_columns
        if any(keyword in col.lower() for keyword in lower_keywords)
    ]
    return selected


def _flatten_multiindex(columns: Iterable) -> List[str]:
    flattened: List[str] = []
    for column in columns:
        if isinstance(column, tuple):
            flattened.append("_".join(str(part) for part in column if part))
        else:
            flattened.append(str(column))
    return flattened


def _groupby_with_dropna(df: pd.DataFrame, by: Sequence[str]):
    """Return a ``groupby`` object that keeps ``NaN`` categories when possible."""

    try:
        return df.groupby(by, dropna=False)
    except TypeError:  # pragma: no cover - compatibility for older pandas
        return df.groupby(by)


def _maybe_cast_group_columns(frame: pd.DataFrame, group_columns: Sequence[str]) -> pd.DataFrame:
    result = frame.copy()
    for column in group_columns:
        if column not in result.columns:
            continue
        series = result[column]
        if pd.api.types.is_float_dtype(series):
            non_na = series.dropna()
            if not non_na.empty and (non_na == non_na.astype(int)).all():
                result[column] = series.astype("Int64")
    return result


def _aggregate_mean_std(
    df: pd.DataFrame,
    group_columns: Sequence[str],
    metrics: Sequence[str],
) -> pd.DataFrame:
    if not metrics:
        return pd.DataFrame()
    if group_columns and any(column not in df.columns for column in group_columns):
        return pd.DataFrame()

    if group_columns:
        working = df.dropna(subset=[column for column in group_columns if column in df.columns])
        if working.empty:
            return pd.DataFrame()
        aggregated = (
            _groupby_with_dropna(working, list(group_columns))[list(metrics)]
            .agg(["mean", "std"])
            .reset_index()
        )
        aggregated.columns = _flatten_multiindex(aggregated.columns)
        aggregated = _maybe_cast_group_columns(aggregated, group_columns)
        aggregated = aggregated.sort_values(list(group_columns)).reset_index(drop=True)
        return aggregated

    aggregated = df[list(metrics)].agg(["mean", "std"]).transpose()
    aggregated = aggregated.rename_axis("metric").reset_index()
    aggregated = aggregated.rename(columns={"mean": "mean", "std": "std"})
    aggregated = aggregated.sort_values("metric").reset_index(drop=True)
    return aggregated


def _print_summary_section(title: str, frame: pd.DataFrame) -> None:
    if frame.empty:
        return
    print(title)
    with pd.option_context("display.max_rows", None, "display.max_columns", None):
        print(frame.to_string(index=False))
    print()


def _prepare_run_level_frame(df: pd.DataFrame) -> Optional[pd.DataFrame]:
    """Collapse a solver DataFrame so that each ``source_file`` appears once."""

    if "source_file" not in df.columns:
        return None
    if df.empty:
        return pd.DataFrame(columns=df.columns)
    return (
        df.sort_values(by="source_file")
        .drop_duplicates(subset="source_file", keep="last")
        .reset_index(drop=True)
    )


def _detect_initial_solution_mask(run_level: pd.DataFrame) -> Optional[pd.Series]:
    """Detect runs that were solved by the initial solution (zero LNS iterations)."""

    if run_level.empty:
        return pd.Series(dtype=bool)

    mask = pd.Series(False, index=run_level.index)
    found_indicator = False

    for column in run_level.columns:
        if column == "source_file":
            continue
        series = run_level[column]
        lower_name = column.lower()

        if pd.api.types.is_bool_dtype(series):
            if any(keyword in lower_name for keyword in INITIAL_SOLUTION_INDICATOR_KEYWORDS):
                mask |= series.fillna(False)
                found_indicator = True
                continue
            if any(keyword in lower_name for keyword in ITERATION_KEYWORDS):
                mask |= ~series.fillna(True)
                found_indicator = True
                continue

        if not pd.api.types.is_numeric_dtype(series):
            continue

        non_na = series.dropna()
        if any(keyword in lower_name for keyword in INITIAL_SOLUTION_INDICATOR_KEYWORDS):
            if not non_na.empty and set(non_na.unique()).issubset({0, 1}):
                mask |= series.fillna(0) > 0
                found_indicator = True
            continue

        if not any(keyword in lower_name for keyword in ITERATION_KEYWORDS):
            continue

        if non_na.empty:
            continue
        try:
            integer_like = (non_na == non_na.astype(int)).all()
        except ValueError:  # pragma: no cover - defensive fallback
            integer_like = False
        if integer_like:
            mask |= series.fillna(0) == 0
            found_indicator = True

    if not found_indicator:
        return None
    return mask


def _count_runs(
    frame: pd.DataFrame, group_columns: Sequence[str], count_label: str
) -> pd.DataFrame:
    if frame.empty:
        return pd.DataFrame(columns=[*group_columns, count_label])

    available = [column for column in group_columns if column in frame.columns]
    if not available:
        return pd.DataFrame({count_label: [len(frame)]})

    grouped = (
        _groupby_with_dropna(frame, available)["source_file"].nunique().reset_index(name=count_label)
    )
    grouped = _maybe_cast_group_columns(grouped, available)
    return grouped.sort_values(available).reset_index(drop=True)


def _print_initial_solution_overview(run_level: pd.DataFrame, mask: pd.Series) -> pd.DataFrame:
    """Print a summary of runs solved by the initial solution and return them."""

    if mask is None:
        print("Initial-solution detection skipped because no identifying columns were found.")
        return pd.DataFrame(columns=run_level.columns)

    initial_runs = run_level[mask]
    total_runs = len(run_level)
    solved_count = len(initial_runs)

    if total_runs:
        percentage = 100 * solved_count / total_runs
        print(
            f"Initial-solution runs detected: {solved_count} of {total_runs} "
            f"({percentage:.1f}%)."
        )
    else:
        print("No runs available for initial-solution detection.")

    if initial_runs.empty:
        print("No runs appear to have been solved by the initial solution.")
        return initial_runs

    print("Runs solved by the initial solution per map and agent count:")
    per_map_agent = _count_runs(initial_runs, ["map_name", "agent_count"], "initial_solution_runs")
    if not per_map_agent.empty:
        with pd.option_context("display.max_rows", None, "display.max_columns", None):
            print(per_map_agent.to_string(index=False))
        print()

    per_map = _count_runs(initial_runs, ["map_name"], "initial_solution_runs")
    if not per_map.empty:
        with pd.option_context("display.max_rows", None, "display.max_columns", None):
            print(per_map.to_string(index=False))
        print()

    return initial_runs


def _classify_lns_metric_columns(df: pd.DataFrame) -> List[Tuple[str, List[str]]]:
    numeric_columns = [
        col
        for col in df.select_dtypes(include=["number"]).columns
        if col not in METADATA_COLUMNS
    ]

    classification: List[Tuple[str, List[str]]] = [
        ("Zone counts", []),
        ("Average zone size", []),
        ("Average zone growth", []),
        ("Other zone metrics", []),
        ("Waiting metrics", []),
        ("Spatial metrics", []),
    ]
    buckets = {title: bucket for title, bucket in classification}

    for column in numeric_columns:
        lower = column.lower()
        if "wait" in lower:
            buckets["Waiting metrics"].append(column)
            continue
        if any(keyword in lower for keyword in SPATIAL_KEYWORDS):
            buckets["Spatial metrics"].append(column)
            continue
        if "zone" not in lower:
            continue
        if any(marker in lower for marker in ZONE_COUNT_MARKERS):
            buckets["Zone counts"].append(column)
        elif any(marker in lower for marker in ZONE_SIZE_MARKERS):
            buckets["Average zone size"].append(column)
        elif any(marker in lower for marker in ZONE_GROWTH_MARKERS):
            buckets["Average zone growth"].append(column)
        else:
            buckets["Other zone metrics"].append(column)

    return classification


def _print_lns_specific_statistics(df: pd.DataFrame) -> None:
    print("\n----- LNS-specific metrics -----")

    run_level = _prepare_run_level_frame(df)
    filtered_df = df

    if run_level is None or run_level.empty:
        print(
            "Initial-solution detection skipped because per-run metadata could not be prepared."
        )
    else:
        mask = _detect_initial_solution_mask(run_level)
        if mask is None:
            print(
                "Initial-solution detection skipped because no identifying columns were found."
            )
        else:
            initial_runs = _print_initial_solution_overview(run_level, mask)
            if not initial_runs.empty:
                excluded_sources = set(initial_runs["source_file"].dropna().astype(str))
                if excluded_sources:
                    filtered_df = df[~df["source_file"].astype(str).isin(excluded_sources)]
                    print(
                        f"Excluding {len(excluded_sources)} run(s) from LNS-specific metrics."
                    )
                else:
                    print(
                        "Initial-solution runs could not be matched to source files; "
                        "no rows were excluded."
                    )

    if filtered_df.empty:
        print("No LNS rows remain for metric aggregation.")
        return

    classified_columns = _classify_lns_metric_columns(filtered_df)
    if not any(columns for _, columns in classified_columns):
        print("No LNS-specific zone, waiting, or spatial metrics were detected.")
        return

    for title, columns in classified_columns:
        if not columns:
            continue
        _print_summary_section(
            f"Per map and agent count ({title})",
            _aggregate_mean_std(filtered_df, ["map_name", "agent_count"], columns),
        )
        _print_summary_section(
            f"Per map (all agent counts) ({title})",
            _aggregate_mean_std(filtered_df, ["map_name"], columns),
        )
        _print_summary_section(
            f"All runs combined ({title})",
            _aggregate_mean_std(filtered_df, [], columns),
        )


def print_solver_statistics(df: pd.DataFrame) -> None:
    """Print per-solver mean/std summaries for time and CNF metrics."""

    if "solver" not in df.columns:
        print("Cannot compute solver statistics because the 'solver' column is missing.")
        return

    time_metrics = _select_metric_columns(df, TIME_METRIC_KEYWORDS)
    cnf_metrics = _select_metric_columns(df, CNF_METRIC_KEYWORDS)

    if not time_metrics and not cnf_metrics:
        print("No matching time or CNF metrics were found for summarisation.")
        return

    for solver, solver_df in df.groupby("solver", sort=True):
        print(f"\n===== Solver: {solver} =====")
        _print_summary_section(
            "Per map and agent count (time metrics)",
            _aggregate_mean_std(solver_df, ["map_name", "agent_count"], time_metrics),
        )
        _print_summary_section(
            "Per map and agent count (CNF metrics)",
            _aggregate_mean_std(solver_df, ["map_name", "agent_count"], cnf_metrics),
        )
        _print_summary_section(
            "Per map (all agent counts) - time metrics",
            _aggregate_mean_std(solver_df, ["map_name"], time_metrics),
        )
        _print_summary_section(
            "Per map (all agent counts) - CNF metrics",
            _aggregate_mean_std(solver_df, ["map_name"], cnf_metrics),
        )
        _print_summary_section(
            "All runs combined (time metrics)",
            _aggregate_mean_std(solver_df, [], time_metrics),
        )
        _print_summary_section(
            "All runs combined (CNF metrics)",
            _aggregate_mean_std(solver_df, [], cnf_metrics),
        )

        if "lns" in solver.lower():
            _print_lns_specific_statistics(solver_df)


def _parse_expected_agent_counts(values: Optional[Sequence[str]]) -> Optional[List[int]]:
    if values is None:
        return (10, 20, 50, 100, 200)
    if not values:
        return None
    return [int(value) for value in values]


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "base_dir",
        nargs="?",
        default="lns_clean/logs",
        help="Directory that contains the solver log folders (default: %(default)s)",
    )
    parser.add_argument(
        "--expected-agent-counts",
        metavar="N",
        nargs="*",
        help=(
            "Optional whitelist of agent counts used when parsing file names. "
            "Provide an empty list to disable the safeguard."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path where the aggregated DataFrame should be written."
             " The format is inferred from the file extension (csv or parquet).",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="Print grouped summary statistics for numeric metrics.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print the path of each CSV file as it is processed.",
    )

    args = parser.parse_args(argv)

    expected_agent_counts = _parse_expected_agent_counts(args.expected_agent_counts)

    try:
        df = load_solver_logs(
            base_dir=args.base_dir,
            expected_agent_counts=expected_agent_counts,
            verbose=args.verbose,
        )
    except FileNotFoundError as exc:  # pragma: no cover - defensive UX
        parser.error(str(exc))
        return 2

    if df.empty:
        print("No CSV log files were discovered.")
        return 0

    print(f"Loaded {len(df)} rows from {df['source_file'].nunique()} CSV files.")
    print(f"Columns: {', '.join(df.columns)}")

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if args.output.suffix.lower() == ".csv":
            df.to_csv(args.output, index=False)
        elif args.output.suffix.lower() in {".parquet", ".pq"}:
            df.to_parquet(args.output, index=False)
        else:
            parser.error(
                "Unsupported output format. Use a .csv or .parquet/.pq extension."
            )
        print(f"Wrote aggregated data to {args.output}")

    if args.summary:
        print_solver_statistics(df)

    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())