"""Paper-ready per-map completion timelines for the new LNS variant data.

This script is a curated wrapper around :mod:`Analysis.plot_map_agent_timelines`
for the four current LNS variants:

* LNS
* LNS+2
* LNS++1
* LNSinit2

It intentionally ignores obvious duplicate/backup experiment folders (for
example ``Accidental_Berlin_again`` and timestamped backup directories) so the
same map/agent configuration is not counted twice by default.  The underlying
plotting code still reads the repository's standard ``experiments.csv`` files,
normalises map names, groups by agent count, and draws cumulative completion
step curves capped at the 600 second experiment budget.

Matplotlib is used deliberately: it remains the most reliable Python plotting
library for publication-quality vector output (PDF/SVG) and exact control over
fonts, line widths, legends, and journal column sizing.  The defaults below use
colour-blind-safe Okabe-Ito colours, a restrained grid, and save both PDF and
PNG unless overridden.

Typical usage from the repository root::

    python Analysis/plot_lns_variant_timelines.py

To save to custom paths::

    python Analysis/plot_lns_variant_timelines.py \
        --output-pdf lns_variants_timelines.pdf \
        --output-png lns_variants_timelines.png \
        --stats-csv lns_variants_timelines_stats.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, Mapping, Optional, Sequence

import matplotlib as mpl
import matplotlib.pyplot as plt

from Analysis.plot_map_agent_timelines import (
    LogInput,
    _print_stats_dataframe,
    plot_solver_map_agent_timelines,
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

    root = root.expanduser()
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
        sources = [Path(value) for value in entry[1:]]
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
        "--stats-csv",
        type=Path,
        default=Path("lns_variant_map_timelines_stats.csv"),
    )
    parser.add_argument("--title", default="LNS variant completion timelines")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--skip-status-filter", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_argument_parser()
    args = parser.parse_args(argv)

    try:
        solver_sources = _parse_solver_overrides(args.solver)
    except ValueError as exc:
        parser.error(str(exc))

    if solver_sources is None:
        solver_sources = build_default_solver_sources(
            include_duplicate_backups=args.include_duplicate_backups
        )

    _configure_paper_style()
    fig, _axes, stats_df = plot_solver_map_agent_timelines(
        solver_sources,
        palette=LNS_VARIANT_PALETTE,
        title=args.title,
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
    if args.show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())