"""Create the common-experiment 5-way overall performance CDF.

This is the standalone script for the LNS-SAT, MDD-SAT, LNS++1, LNS+2,
and LNSinit2 comparison.  It keeps only scenarios that exist in every
solver log, orders the map families as empty -> room -> Berlin -> warehouse,
and respects the known short scenario-list exception for empty-16-16 with
50 agents (50 scenarios, therefore a 300 second block budget).

Run from the repository root::

    python Analysis/plot_common_5way_performance.py

By default this writes:

* ``overall_performance_common_5way.svg``
* ``per_agent_common_5way.svg``
* ``per_map_common_5way.svg``
* ``common_5way_stats_overall.csv``
* ``common_5way_stats_agents.csv``
* ``common_5way_stats_maps.csv``
* ``common_5way_stats_common_experiments.csv``
* ``common_5way_stats_depicted_scenario_blocks.csv``
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Iterable, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import matplotlib.pyplot as plt

from Analysis.plot_solver_aggregate_cdfs import (
    EXPECTED_COUNT_OVERRIDES,
    filter_solver_sources_to_common_experiments,
    plot_solver_aggregate_cdfs,
)

DEFAULT_SOLVERS = {
    "LNS-SAT": [Path("lns_clean/logs/LNS")],
    "MDD-SAT": [Path("lns_clean/logs/WholeSolve")],
    "LNS++1": [Path("lns_clean/logs/LNS++1")],
    "LNS+2": [Path("lns_clean/logs/LNS+2")],
    "LNSinit2": [Path("lns_clean/logs/LNSinit2")],
}

SCENARIO_BLOCK_ORDER = [
    ('empty-16-16', '10', 100, 600.0),
    ('empty-16-16', '20', 100, 600.0),
    ('empty-16-16', '50', 50, 300.0),
    ('empty-16-16', '100', 100, 600.0),
    ('empty-16-16', '200', 100, 600.0),
    ('empty-32-32', '10', 100, 600.0),
    ('empty-32-32', '20', 100, 600.0),
    ('empty-32-32', '50', 100, 600.0),
    ('empty-32-32', '100', 100, 600.0),
    ('empty-32-32', '200', 100, 600.0),
    ('empty-48-48', '10', 100, 600.0),
    ('empty-48-48', '20', 100, 600.0),
    ('empty-48-48', '50', 100, 600.0),
    ('empty-48-48', '100', 100, 600.0),
    ('empty-48-48', '200', 100, 600.0),
    ('room-32-32-4', '10', 100, 600.0),
    ('room-32-32-4', '20', 100, 600.0),
    ('room-32-32-4', '50', 100, 600.0),
    ('room-32-32-4', '100', 100, 600.0),
    ('room-32-32-4', '200', 100, 600.0),
    ('room-64-64-8', '10', 100, 600.0),
    ('room-64-64-8', '20', 100, 600.0),
    ('room-64-64-8', '50', 100, 600.0),
    ('room-64-64-8', '100', 100, 600.0),
    ('room-64-64-8', '200', 100, 600.0),
    ('room-64-64-16', '10', 100, 600.0),
    ('room-64-64-16', '20', 100, 600.0),
    ('room-64-64-16', '50', 100, 600.0),
    ('room-64-64-16', '100', 100, 600.0),
    ('room-64-64-16', '200', 100, 600.0),
    ('Berlin_1_256', '10', 100, 600.0),
    ('Berlin_1_256', '20', 100, 600.0),
    ('Berlin_1_256', '50', 100, 600.0),
    ('Berlin_1_256', '100', 100, 600.0),
    ('Berlin_1_256', '200', 100, 600.0),
    ('warehouse-10-20-10-2-2', '10', 100, 600.0),
    ('warehouse-10-20-10-2-2', '20', 100, 600.0),
    ('warehouse-10-20-10-2-2', '50', 100, 600.0),
    ('warehouse-10-20-10-2-2', '100', 100, 600.0),
    ('warehouse-10-20-10-2-2', '200', 100, 600.0),
    ('warehouse-20-40-10-2-2', '10', 100, 600.0),
    ('warehouse-20-40-10-2-2', '20', 100, 600.0),
    ('warehouse-20-40-10-2-2', '50', 100, 600.0),
    ('warehouse-20-40-10-2-2', '100', 100, 600.0),
    ('warehouse-20-40-10-2-2', '200', 100, 600.0),
]


def _parse_expected_count_overrides(
    entries: Iterable[Sequence[str]] | None,
) -> dict[tuple[str, str], int]:
    """Parse ``--expected-count MAP AGENTS COUNT`` overrides."""

    overrides: dict[tuple[str, str], int] = {}
    for entry in entries or []:
        if len(entry) != 3:
            raise ValueError("Expected --expected-count MAP AGENTS COUNT.")
        map_label, agent_label, count_text = entry
        count = int(count_text)
        if count < 0:
            raise ValueError("Expected-count overrides must be non-negative.")
        overrides[(map_label, agent_label)] = count
    return overrides


def _build_depicted_block_plan(common_summary, expected_count_overrides):
    """Return the ordered map/agent scenario blocks that will appear in the plot."""

    summary_lookup = {
        (str(row["map"]), str(row["agents"])): int(row["common_experiments"])
        for _, row in common_summary.iterrows()
    }
    rows = []
    seen_keys = set()
    for map_label, agent_label, default_expected, default_padding_s in SCENARIO_BLOCK_ORDER:
        key = (map_label, agent_label)
        if key not in summary_lookup:
            continue
        expected = int(expected_count_overrides.get(key, default_expected))
        common_scenarios = int(summary_lookup[key])
        depicted = min(common_scenarios, expected)
        padding_s = default_padding_s if depicted == expected else 600.0 * (depicted / 100.0)
        rows.append(
            {
                "map": map_label,
                "agents": agent_label,
                "common_scenarios": common_scenarios,
                "expected_scenarios": depicted,
                "padding_if_incomplete_s": padding_s,
            }
        )
        seen_keys.add(key)

    for key, common_scenarios in summary_lookup.items():
        if key in seen_keys:
            continue
        map_label, agent_label = key
        expected = int(expected_count_overrides.get(key, common_scenarios))
        depicted = min(int(common_scenarios), expected)
        rows.append(
            {
                "map": map_label,
                "agents": agent_label,
                "common_scenarios": int(common_scenarios),
                "expected_scenarios": depicted,
                "padding_if_incomplete_s": 600.0 * (depicted / 100.0),
            }
        )
    return rows


def build_parser() -> argparse.ArgumentParser:
    """Return the command-line parser for the standalone 5-way plot."""

    parser = argparse.ArgumentParser(
        description="Plot the common-experiment 5-way MAPF solver CDF."
    )
    parser.add_argument(
        "--output-overall",
        type=Path,
        default=Path("overall_performance_common_5way.svg"),
        help="Output path for the overall CDF figure.",
    )
    parser.add_argument(
        "--output-agents",
        type=Path,
        default=Path("per_agent_common_5way.svg"),
        help="Output path for the per-agent CDF grid.",
    )
    parser.add_argument(
        "--output-maps",
        type=Path,
        default=Path("per_map_common_5way.svg"),
        help="Output path for the per-map CDF grid.",
    )
    parser.add_argument(
        "--stats-prefix",
        type=Path,
        default=Path("common_5way_stats"),
        help="Prefix for CSV stats outputs.",
    )
    parser.add_argument(
        "--expected-count",
        action="append",
        nargs=3,
        metavar=("MAP", "AGENTS", "COUNT"),
        help=(
            "Add/override a short scenario-list exception. Built-in default: "
            "empty-16-16 50 50. Repeat for more exceptions."
        ),
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figures after writing outputs.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Generate the common-experiment 5-way plots and CSV summaries."""

    parser = build_parser()
    args = parser.parse_args(argv)

    expected_count_overrides = {
        **EXPECTED_COUNT_OVERRIDES,
        **_parse_expected_count_overrides(args.expected_count),
    }

    common_sources, common_summary = filter_solver_sources_to_common_experiments(
        DEFAULT_SOLVERS
    )
    depicted_block_plan = _build_depicted_block_plan(
        common_summary, expected_count_overrides
    )

    plots, stats_overall, stats_by_agent, stats_by_map = plot_solver_aggregate_cdfs(
        common_sources,
        expected_from_attempts=True,
        expected_count_overrides=expected_count_overrides,
        title_overall="Overall Performance",
        title_by_agent="Common-experiment performance by agent count",
        title_by_map="Common-experiment performance by map",
        return_stats=True,
        display_stats=False,
    )
    (fig_overall, _), (fig_agents, _), (fig_maps, _) = plots
    for figure in (fig_overall, fig_agents, fig_maps):
        for axis in figure.axes:
            if axis.has_data():
                axis.set_ylabel("Cumulative completed scenarios")

    args.output_overall.parent.mkdir(parents=True, exist_ok=True)
    args.output_agents.parent.mkdir(parents=True, exist_ok=True)
    args.output_maps.parent.mkdir(parents=True, exist_ok=True)
    args.stats_prefix.parent.mkdir(parents=True, exist_ok=True)

    fig_overall.savefig(args.output_overall, bbox_inches="tight")
    fig_agents.savefig(args.output_agents, bbox_inches="tight")
    fig_maps.savefig(args.output_maps, bbox_inches="tight")

    stats_overall.to_csv(
        args.stats_prefix.with_name(args.stats_prefix.name + "_overall.csv"),
        index=False,
    )
    stats_by_agent.to_csv(
        args.stats_prefix.with_name(args.stats_prefix.name + "_agents.csv"),
        index=False,
    )
    stats_by_map.to_csv(
        args.stats_prefix.with_name(args.stats_prefix.name + "_maps.csv"),
        index=False,
    )
    common_summary.to_csv(
        args.stats_prefix.with_name(args.stats_prefix.name + "_common_experiments.csv"),
        index=False,
    )
    block_plan_path = args.stats_prefix.with_name(
        args.stats_prefix.name + "_depicted_scenario_blocks.csv"
    )
    with block_plan_path.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "map",
                "agents",
                "common_scenarios",
                "expected_scenarios",
                "padding_if_incomplete_s",
            ],
        )
        writer.writeheader()
        writer.writerows(depicted_block_plan)

    print(f"Wrote {args.output_overall}")
    print(f"Wrote {args.output_agents}")
    print(f"Wrote {args.output_maps}")
    print(f"Wrote CSV stats with prefix {args.stats_prefix}")
    print(f"Wrote depicted scenario block order to {block_plan_path}")

    if args.show:
        plt.show()
    else:
        plt.close(fig_overall)
        plt.close(fig_agents)
        plt.close(fig_maps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())