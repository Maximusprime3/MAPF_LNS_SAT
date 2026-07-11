"""Latest standalone paper-ready per-map completion timelines for the new LNS variant data.

This script is a curated wrapper around :mod:`Analysis.plot_map_agent_timelines`
for the four current LNS variants:

* LNS
* LNS+2
* LNS++1
* LNSinit2

By default it plots only the maps that are present in every selected variant,
which keeps the comparison focused on common coverage.  It also intentionally
ignores obvious duplicate/backup experiment folders (for example
``Accidental_Berlin_again`` and timestamped backup directories) so the same
map/agent configuration is not counted twice.  The underlying
plotting code still reads the repository's standard ``experiments.csv`` files,
normalises map names, groups by agent count, and draws cumulative completion
step curves capped at the 600 second experiment budget.

Matplotlib is used deliberately: it remains the most reliable Python plotting
library for publication-quality vector output (PDF/SVG) and exact control over
fonts, line widths, legends, and journal column sizing.  The defaults below use
colour-blind-safe Okabe-Ito colours, a restrained grid, and save both PDF and
PNG unless overridden.

Typical usage from the repository root::

    python Analysis/plot_lns_variant_timelines_latest.py

To save to custom paths::

    python Analysis/plot_lns_variant_timelines_latest.py \
        --output-pdf lns_variants_timelines.pdf \
        --output-png lns_variants_timelines.png \
        --output-svg lns_variants_timelines.svg \
        --stats-csv lns_variants_timelines_stats.csv

For smaller figures, either shrink each map panel::

    python Analysis/plot_lns_variant_timelines_latest.py --panel-width 3 --panel-height 2.3

or force an exact final figure size::

    python Analysis/plot_lns_variant_timelines_latest.py --figsize 9 7
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Iterable, Mapping, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd

from Analysis.plot_map_agent_timelines import (
    LogInput,
    _print_stats_dataframe,
    plot_solver_map_agent_timelines,
)
from Analysis.plot_solve_time_cdf import (
    MAP_COLUMN_CANDIDATES,
    _find_column,
    _iter_log_sources,
    _normalise_map_label,
)

DEFAULT_SOLVER_ROOTS: Mapping[str, Path] = {
    "LNS": Path("lns_clean/logs/LNS"),
    "LNS+2": Path("lns_clean/logs/LNS+2"),
    "LNS++1": Path("lns_clean/logs/LNS++1"),
    "LNSinit2": Path("lns_clean/logs/LNSinit2"),
}

DEFAULT_EXCLUDE_PARTS = (
    "Accidental_Berlin_again",
    "backup_before_merge",
)

# Okabe-Ito-derived palette: blue, green, vermillion, purple.
LNS_VARIANT_PALETTE = ("#0072B2", "#009E73", "#D55E00", "#CC79A7")

DEFAULT_MAP_ORDER: Sequence[str] = (
    "room-64-64-8",
    "room-64-64-16",
    "room-32-32-4",
    "empty-48-48",
    "empty-32-32",
    "empty-16-16",
    "Berlin_1_256",
    "warehouse-20-40-10-2-2",
)


def _resolve_repo_path(path: Path) -> Path:
    """Resolve CLI/default paths from either repo root or Analysis/."""

    expanded = path.expanduser()
    if expanded.is_absolute() or expanded.exists():
        return expanded

    repo_relative = REPO_ROOT / expanded
    if repo_relative.exists():
        return repo_relative

    return expanded


def _load_solver_frames(
    solver_sources: Mapping[str, Sequence[LogInput]],
) -> dict[str, list[pd.DataFrame]]:
    """Load every source into DataFrames while preserving solver grouping."""

    loaded: dict[str, list[pd.DataFrame]] = {}
    for solver_label, sources in solver_sources.items():
        solver_frames: list[pd.DataFrame] = []
        for frame, source_label in _iter_log_sources(sources):
            frame = frame.copy()
            if source_label is not None:
                frame.attrs["source_path"] = source_label
            solver_frames.append(frame)
        loaded[str(solver_label)] = solver_frames
    return loaded


def _maps_in_frame(frame: pd.DataFrame) -> set[str]:
    """Return normalised map labels present in one experiment DataFrame."""

    map_column = _find_column(frame.columns, MAP_COLUMN_CANDIDATES)
    return {
        map_label
        for map_label in frame[map_column].map(_normalise_map_label).dropna()
        if map_label is not None
    }


def _common_maps_by_solver(loaded_sources: Mapping[str, Sequence[pd.DataFrame]]) -> set[str]:
    """Return maps that appear in at least one file for every solver variant."""

    common_maps: Optional[set[str]] = None
    for frames in loaded_sources.values():
        solver_maps: set[str] = set()
        for frame in frames:
            solver_maps.update(_maps_in_frame(frame))
        common_maps = solver_maps if common_maps is None else common_maps & solver_maps

    return common_maps or set()


def _filter_frames_to_maps(
    loaded_sources: Mapping[str, Sequence[pd.DataFrame]],
    maps_to_keep: set[str],
) -> dict[str, list[LogInput]]:
    """Return source DataFrames filtered to the selected normalised maps."""

    filtered_sources: dict[str, list[LogInput]] = {}
    for solver_label, frames in loaded_sources.items():
        filtered_frames: list[LogInput] = []
        for frame in frames:
            map_column = _find_column(frame.columns, MAP_COLUMN_CANDIDATES)
            normalised_maps = frame[map_column].map(_normalise_map_label)
            filtered = frame.loc[normalised_maps.isin(maps_to_keep)].copy()
            filtered.attrs.update(frame.attrs)
            if not filtered.empty:
                filtered_frames.append(filtered)
        filtered_sources[solver_label] = filtered_frames
    return filtered_sources


def select_solver_sources_by_map_mode(
    solver_sources: Mapping[str, Sequence[LogInput]],
    *,
    map_mode: str = "common",
) -> dict[str, list[LogInput]]:
    """Optionally restrict solver inputs to maps shared by all variants."""

    if map_mode == "union":
        return {label: list(sources) for label, sources in solver_sources.items()}
    if map_mode != "common":
        raise ValueError(f"Unsupported map mode: {map_mode!r}")

    loaded_sources = _load_solver_frames(solver_sources)
    common_maps = _common_maps_by_solver(loaded_sources)
    if not common_maps:
        raise ValueError("No common maps were found across all selected solver variants.")

    return _filter_frames_to_maps(loaded_sources, common_maps)


def _count_selected_maps(solver_sources: Mapping[str, Sequence[LogInput]]) -> int:
    """Count distinct normalised maps in the already selected solver inputs."""

    loaded_sources = _load_solver_frames(solver_sources)
    selected_maps: set[str] = set()
    for frames in loaded_sources.values():
        for frame in frames:
            selected_maps.update(_maps_in_frame(frame))
    return len(selected_maps)


def _default_figsize_for_map_count(
    n_maps: int,
    *,
    panel_width: float = 4.5,
    panel_height: float = 3.5,
) -> tuple[float, float]:
    """Match the compact per-map grid sizing requested for paper figures."""

    if n_maps < 1:
        raise ValueError("No map data available for sizing.")
    if n_maps == 1:
        n_rows, n_cols = 1, 1
    else:
        n_cols = min(3, max(1, int(math.ceil(math.sqrt(n_maps)))))
        n_rows = int(math.ceil(n_maps / n_cols))

    return (n_cols * panel_width, n_rows * panel_height)


def _configure_paper_style() -> None:
    """Apply conservative, publication-friendly Matplotlib defaults."""

    mpl.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "savefig.bbox": "tight",
            "font.size": 8,
            "axes.titlesize": 9,
            "axes.labelsize": 9,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "legend.fontsize": 8,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.8,
            "grid.linewidth": 0.5,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
        }
    )


def _discover_curated_experiment_csvs(
    root: Path,
    *,
    exclude_parts: Sequence[str] = DEFAULT_EXCLUDE_PARTS,
) -> list[Path]:
    """Return experiment CSVs below ``root`` while skipping duplicate folders."""

    root = _resolve_repo_path(root)
    if root.is_file():
        return [root]

    csvs: list[Path] = []
    for csv_path in sorted(root.rglob("experiments.csv")):
        path_text_parts = tuple(str(part) for part in csv_path.parts)
        if any(
            excluded in path_part
            for excluded in exclude_parts
            for path_part in path_text_parts
        ):
            continue
        csvs.append(csv_path)

    if not csvs:
        raise FileNotFoundError(f"No experiments.csv files found below {root}")
    return csvs


def build_default_solver_sources(
    *,
    include_duplicate_backups: bool = False,
) -> dict[str, list[LogInput]]:
    """Build the default four-variant solver source mapping."""

    exclude_parts: Sequence[str] = (
        () if include_duplicate_backups else DEFAULT_EXCLUDE_PARTS
    )
    return {
        label: _discover_curated_experiment_csvs(root, exclude_parts=exclude_parts)
        for label, root in DEFAULT_SOLVER_ROOTS.items()
    }


def _parse_solver_overrides(
    entries: Optional[Iterable[Sequence[str]]],
) -> Optional[dict[str, list[LogInput]]]:
    if not entries:
        return None

    solvers: dict[str, list[LogInput]] = {}
    for entry in entries:
        if len(entry) < 2:
            raise ValueError("Each --solver entry needs a label and at least one path.")
        label = entry[0]
        sources = [_resolve_repo_path(Path(value)) for value in entry[1:]]
        solvers.setdefault(label, []).extend(sources)
    return solvers


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Plot paper-ready per-map completion timelines for LNS, "
            "LNS+2, LNS++1, and LNSinit2."
        )
    )
    parser.add_argument(
        "--solver",
        action="append",
        nargs="+",
        metavar=("LABEL", "SOURCE"),
        help=(
            "Override defaults with a solver label followed by one or more "
            "CSV files or directories. Repeat per solver."
        ),
    )
    parser.add_argument(
        "--include-duplicate-backups",
        action="store_true",
        help=(
            "Include folders that are skipped by default because their names "
            "indicate accidental duplicates or backups."
        ),
    )
    parser.add_argument(
        "--output-pdf", type=Path, default=Path("lns_variant_map_timelines.pdf")
    )
    parser.add_argument(
        "--output-png", type=Path, default=Path("lns_variant_map_timelines.png")
    )
    parser.add_argument(
        "--output-svg", type=Path, default=Path("lns_variant_map_timelines.svg")
    )
    parser.add_argument(
        "--stats-csv",
        type=Path,
        default=Path("lns_variant_map_timelines_stats.csv"),
    )
    parser.add_argument("--title", default="LNS variant completion timelines")
    parser.add_argument(
        "--map-mode",
        choices=("common", "union"),
        default="common",
        help=(
            "Choose common to plot only maps present in every selected variant "
            "(default), or union to keep the older behaviour and plot every map "
            "found in any variant."
        ),
    )
    parser.add_argument(
        "--figsize",
        type=float,
        nargs=2,
        metavar=("WIDTH", "HEIGHT"),
        help=(
            "Override the full figure size in inches, for example "
            "--figsize 9 7. If omitted, size is computed from the "
            "number of maps and --panel-width/--panel-height."
        ),
    )
    parser.add_argument(
        "--panel-width",
        type=float,
        default=4.5,
        help="Width in inches for each map panel when --figsize is omitted.",
    )
    parser.add_argument(
        "--panel-height",
        type=float,
        default=3.5,
        help="Height in inches for each map panel when --figsize is omitted.",
    )
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--skip-status-filter", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_argument_parser()
    args = parser.parse_args(argv)

    if args.figsize is not None and any(value <= 0 for value in args.figsize):
        parser.error("Figure dimensions must be positive.")
    if args.panel_width <= 0 or args.panel_height <= 0:
        parser.error("Panel dimensions must be positive.")

    try:
        solver_sources = _parse_solver_overrides(args.solver)
    except ValueError as exc:
        parser.error(str(exc))

    if solver_sources is None:
        solver_sources = build_default_solver_sources(
            include_duplicate_backups=args.include_duplicate_backups
        )

    solver_sources = select_solver_sources_by_map_mode(
        solver_sources, map_mode=args.map_mode
    )

    figsize = (
        tuple(args.figsize)
        if args.figsize is not None
        else _default_figsize_for_map_count(
            _count_selected_maps(solver_sources),
            panel_width=args.panel_width,
            panel_height=args.panel_height,
        )
    )

    _configure_paper_style()
    fig, _axes, stats_df = plot_solver_map_agent_timelines(
        solver_sources,
        palette=LNS_VARIANT_PALETTE,
        title=args.title,
        preferred_map_order=DEFAULT_MAP_ORDER,
        figsize=figsize,
        return_stats=True,
        display_stats=False,
        skip_status_filter=args.skip_status_filter,
    )

    if args.stats_csv:
        stats_df.to_csv(args.stats_csv, index=False)
    _print_stats_dataframe(stats_df)

    if args.output_pdf:
        fig.savefig(args.output_pdf)
    if args.output_png:
        fig.savefig(args.output_png)
    if args.output_svg:
        fig.savefig(args.output_svg)
    if args.show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())