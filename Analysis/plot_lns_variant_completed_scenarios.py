#!/usr/bin/env python3
"""Compare LNS variants by cumulative completed scenarios over padded slots.

This standalone script mirrors ``plot_completed_scenarios_over_time.py`` but
plots one cumulative curve per LNS variant.  The default comparison uses the
repository's LNS, LNS+2, LNS++1, and LNSinit2 log roots, assigns each discovered
map/agent combination a 10 minute slot, and overlays each variant's completed
scenario count over the shared requested map order.

Paris and the smaller warehouse map (warehouse-10-20-10-2-2) are excluded
from the default comparison so that the larger warehouse-20-40-10-2-2 map
remains in the intended common benchmark set.

Typical usage from the repository root::

    python Analysis/plot_lns_variant_completed_scenarios.py \
        --output lns_variant_completed_scenarios.png \
        --output-pdf lns_variant_completed_scenarios.pdf \
        --output-svg lns_variant_completed_scenarios.svg
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Mapping, Sequence

DEFAULT_VARIANT_ROOTS: Mapping[str, Path] = {
    "LNS": Path("lns_clean/logs/LNS"),
    "LNS+2": Path("lns_clean/logs/LNS+2"),
    "LNS++1": Path("lns_clean/logs/LNS++1"),
    "LNSinit2": Path("lns_clean/logs/LNSinit2"),
}

MAP_COLUMNS = ("map_name", "map", "instance", "mapfile", "map_path", "mapfile_path")
AGENT_COLUMNS = ("num_agents", "agents", "n_agents", "agent_count")
TIME_COLUMNS = (
    "total_time_s",
    "total_time",
    "solve_time",
    "solver_time",
    "runtime_s",
    "time",
    "total_runtime_ms",
)
STATUS_COLUMNS = ("solved", "status", "result", "outcome")
SCENARIO_COLUMNS = ("scenario_index", "instance_index", "instance", "scenario")
SUCCESS_LABELS = {"1", "true", "t", "yes", "y", "sat", "success", "solved", "ok"}
DEFAULT_EXCLUDE_PATTERNS = (
    "paris",
    "warehouse-10-20-10-2-2",
    "accidental_berlin_again",
    "backup_before_merge",
)
VARIANT_COLOURS = {
    "LNS": "#0072B2",
    "LNS+2": "#009E73",
    "LNS++1": "#D55E00",
    "LNSinit2": "#CC79A7",
}
MAP_ORDER_PATTERNS = (
    "room-64-64-8",
    "room-64-64-16",
    "room-32-32-4",
    "empty-48-48",
    "empty-32-32",
    "empty-16-16",
    "berlin",
    "warehouse",
)


@dataclass(frozen=True, order=True)
class ExperimentKey:
    """A map/agent combination that receives one shared padded slot."""

    map_name: str
    agent_sort: tuple[int, float | str]
    agents: str = field(compare=False)


@dataclass
class ScenarioRun:
    """One experiment-log row relevant to the completion timeline."""

    order: int
    runtime_s: float
    solved: bool


def _find_column(fieldnames: Sequence[str], candidates: Sequence[str]) -> str:
    for candidate in candidates:
        if candidate in fieldnames:
            return candidate
    raise KeyError(f"Expected one of {candidates!r}; found columns {fieldnames!r}")


def _find_optional_column(fieldnames: Sequence[str], candidates: Sequence[str]) -> str | None:
    for candidate in candidates:
        if candidate in fieldnames:
            return candidate
    return None


def _normalise_map(value: str) -> str:
    text = (value or "").strip()
    if not text:
        return "unknown-map"
    path = Path(text)
    return path.stem if path.suffix else (path.name or text)


def _normalise_agents(value: str) -> tuple[str, tuple[int, float | str]]:
    text = (value or "").strip()
    try:
        numeric = float(text)
    except ValueError:
        return text or "unknown", (1, text or "unknown")
    if math.isfinite(numeric) and numeric.is_integer():
        return str(int(numeric)), (0, numeric)
    return text, (0, numeric)


def _to_float(value: str) -> float:
    try:
        return float((value or "").strip())
    except ValueError:
        return math.nan


def _runtime_seconds(row: dict[str, str], time_column: str) -> float:
    value = _to_float(row.get(time_column, ""))
    if not math.isfinite(value):
        return math.nan
    lowered = time_column.lower()
    if lowered.endswith("_ms") or lowered.endswith("milliseconds"):
        return value / 1000.0
    return value


def _is_success(row: dict[str, str], status_column: str | None) -> bool:
    if status_column is None:
        return True
    return (row.get(status_column, "") or "").strip().lower() in SUCCESS_LABELS


def _scenario_order(row: dict[str, str], scenario_column: str | None, fallback: int) -> int:
    if scenario_column is None:
        return fallback
    value = _to_float(row.get(scenario_column, ""))
    if math.isfinite(value):
        return int(value)
    return fallback


def _is_excluded_text(text: str, exclude_patterns: Sequence[str]) -> bool:
    lowered = text.lower()
    return any(pattern.lower() in lowered for pattern in exclude_patterns)


def _map_order_index(map_name: str) -> int:
    lowered = map_name.lower()
    for index, pattern in enumerate(MAP_ORDER_PATTERNS):
        if pattern in lowered:
            return index
    return len(MAP_ORDER_PATTERNS)


def _experiment_sort_key(key: ExperimentKey) -> tuple[int, str, tuple[int, float | str]]:
    return (_map_order_index(key.map_name), key.map_name.lower(), key.agent_sort)


def discover_experiment_csvs(
    paths: Iterable[Path],
    *,
    exclude_patterns: Sequence[str],
) -> list[Path]:
    """Return experiment CSV files after applying path-based exclusions."""

    discovered: list[Path] = []
    for path in paths:
        expanded = path.expanduser()
        if expanded.is_file():
            candidates = [expanded]
        elif expanded.is_dir():
            candidates = sorted(expanded.rglob("experiments.csv"), key=lambda p: str(p))
        else:
            raise FileNotFoundError(f"Input path does not exist: {expanded}")
        for candidate in candidates:
            if _is_excluded_text(str(candidate), exclude_patterns):
                continue
            discovered.append(candidate)

    unique: list[Path] = []
    seen: set[Path] = set()
    for path in discovered:
        resolved = path.resolve()
        if resolved not in seen:
            unique.append(resolved)
            seen.add(resolved)
    if not unique:
        raise FileNotFoundError("No experiments.csv files were found in the provided inputs.")
    return unique


def load_variant_experiments(
    csv_paths: Iterable[Path],
    *,
    exclude_patterns: Sequence[str],
) -> dict[ExperimentKey, list[ScenarioRun]]:
    """Load one variant's rows grouped by map name and agent count."""

    grouped: dict[ExperimentKey, list[ScenarioRun]] = defaultdict(list)
    for csv_path in csv_paths:
        with csv_path.open(newline="") as handle:
            reader = csv.DictReader(handle)
            fieldnames = reader.fieldnames or []
            map_column = _find_column(fieldnames, MAP_COLUMNS)
            agent_column = _find_column(fieldnames, AGENT_COLUMNS)
            time_column = _find_column(fieldnames, TIME_COLUMNS)
            status_column = _find_optional_column(fieldnames, STATUS_COLUMNS)
            scenario_column = _find_optional_column(fieldnames, SCENARIO_COLUMNS)

            for fallback_order, row in enumerate(reader):
                map_name = _normalise_map(row.get(map_column, ""))
                if _is_excluded_text(map_name, exclude_patterns):
                    continue
                agents, agent_sort = _normalise_agents(row.get(agent_column, ""))
                runtime_s = _runtime_seconds(row, time_column)
                if not math.isfinite(runtime_s) or runtime_s < 0:
                    continue
                key = ExperimentKey(map_name=map_name, agent_sort=agent_sort, agents=agents)
                grouped[key].append(
                    ScenarioRun(
                        order=_scenario_order(row, scenario_column, fallback_order),
                        runtime_s=runtime_s,
                        solved=_is_success(row, status_column),
                    )
                )
    return dict(grouped)


def load_all_variants(
    variant_sources: Mapping[str, Sequence[Path]],
    *,
    exclude_patterns: Sequence[str],
) -> dict[str, dict[ExperimentKey, list[ScenarioRun]]]:
    """Load every requested variant into grouped experiment data."""

    loaded: dict[str, dict[ExperimentKey, list[ScenarioRun]]] = {}
    for label, sources in variant_sources.items():
        csv_paths = discover_experiment_csvs(sources, exclude_patterns=exclude_patterns)
        loaded[label] = load_variant_experiments(csv_paths, exclude_patterns=exclude_patterns)
    return loaded


def build_variant_timeline(
    grouped: dict[ExperimentKey, list[ScenarioRun]],
    schedule: Sequence[ExperimentKey],
    *,
    time_limit_s: float,
    expected_scenarios: int,
) -> tuple[list[float], list[int], dict[ExperimentKey, int]]:
    """Build one variant's cumulative curve over the shared schedule."""

    x_values = [0.0]
    y_values = [0]
    completed_total = 0
    solved_by_key: dict[ExperimentKey, int] = {}

    for slot_index, key in enumerate(schedule):
        start = slot_index * time_limit_s
        slot_end = start + time_limit_s
        within_slot = 0.0
        solved_in_slot = 0
        rows = sorted(grouped.get(key, ()), key=lambda item: item.order)

        for run in rows:
            if not run.solved or solved_in_slot >= expected_scenarios:
                continue
            within_slot += run.runtime_s
            event_time = min(start + within_slot, slot_end)
            completed_total += 1
            solved_in_slot += 1
            x_values.extend([event_time, event_time])
            y_values.extend([y_values[-1], completed_total])
            if event_time >= slot_end:
                break

        if x_values[-1] < slot_end:
            x_values.append(slot_end)
            y_values.append(completed_total)
        solved_by_key[key] = solved_in_slot

    return x_values, y_values, solved_by_key


def _parse_variant_overrides(entries: Sequence[Sequence[str]] | None) -> dict[str, list[Path]]:
    if not entries:
        return {label: [root] for label, root in DEFAULT_VARIANT_ROOTS.items()}
    variants: dict[str, list[Path]] = {}
    for entry in entries:
        if len(entry) < 2:
            raise ValueError("Each --variant entry needs a label and at least one source path.")
        variants.setdefault(entry[0], []).extend(Path(value) for value in entry[1:])
    return variants


def plot_comparison(
    loaded: Mapping[str, dict[ExperimentKey, list[ScenarioRun]]],
    *,
    outputs: Sequence[Path],
    title: str,
    time_limit_s: float,
    expected_scenarios: int,
) -> None:
    """Render all variant curves over one shared map/agent schedule."""

    import matplotlib.pyplot as plt

    if not outputs:
        raise ValueError("At least one output path must be provided.")

    schedule = sorted(
        {key for grouped in loaded.values() for key in grouped},
        key=_experiment_sort_key,
    )
    if not schedule:
        raise ValueError("No map/agent experiments remained after exclusions.")

    fig_height = max(6.0, 4.5 + 0.18 * len(schedule))
    fig, ax = plt.subplots(figsize=(14, fig_height))
    top_margin = min(0.55, 0.16 + 0.018 * len(schedule))
    fig.subplots_adjust(top=1.0 - top_margin)

    solved_summaries: dict[str, dict[ExperimentKey, int]] = {}
    for label, grouped in loaded.items():
        x_values, y_values, solved_by_key = build_variant_timeline(
            grouped,
            schedule,
            time_limit_s=time_limit_s,
            expected_scenarios=expected_scenarios,
        )
        solved_summaries[label] = solved_by_key
        ax.plot(
            x_values,
            y_values,
            label=label,
            color=VARIANT_COLOURS.get(label),
            linewidth=2.2,
        )

    for slot_index, key in enumerate(schedule):
        start = slot_index * time_limit_s
        end = start + time_limit_s
        ax.axvline(start, color="0.86", linewidth=0.8, zorder=0)
        ax.text(
            (start + end) / 2.0,
            -0.04,
            f"{key.map_name}\n{key.agents} agents",
            transform=ax.get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=8,
            rotation=45,
        )
    ax.axvline(len(schedule) * time_limit_s, color="0.86", linewidth=0.8, zorder=0)

    ax.set_title(title, pad=18)
    ax.set_xlabel("Elapsed padded experiment time (seconds)")
    ax.set_ylabel("Cumulative completed scenarios")
    ax.grid(True, which="both", axis="both", alpha=0.25)
    ax.legend(loc="upper left")

    padding_min = time_limit_s / 60.0
    entries = [
        f"{idx + 1}. {key.map_name}, {key.agents} agents, {padding_min:g} min"
        for idx, key in enumerate(schedule)
    ]
    wrapped_lines = [
        "Included experiments and padding (requested map order; Paris and warehouse-10-20-10-2-2 excluded):"
    ]
    line = ""
    for entry in entries:
        candidate = f"{line}   {entry}" if line else entry
        if len(candidate) > 115 and line:
            wrapped_lines.append(line)
            line = entry
        else:
            line = candidate
    if line:
        wrapped_lines.append(line)
    fig.text(0.01, 0.99, "\n".join(wrapped_lines), ha="left", va="top", fontsize=9)

    for output in outputs:
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print(
        "Wrote "
        + ", ".join(str(output) for output in outputs)
        + f" with {len(schedule)} included map/agent slot(s)."
    )
    for label, solved_by_key in solved_summaries.items():
        total = sum(solved_by_key.values())
        print(f"{label}: {total} completed scenarios")

    differing_slots = []
    for key in schedule:
        counts = {
            label: solved_by_key[key]
            for label, solved_by_key in solved_summaries.items()
        }
        if len(set(counts.values())) > 1:
            differing_slots.append((key, counts))

    if differing_slots:
        print("Differences in completed scenarios by experiment:")
        for key, counts in differing_slots:
            count_text = ", ".join(
                f"{label}={count}" for label, count in sorted(counts.items())
            )
            print(f"  {key.map_name}, {key.agents} agents: {count_text}")
    else:
        print("No differences in completed scenarios by experiment.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--variant",
        action="append",
        nargs="+",
        metavar=("LABEL", "SOURCE"),
        help="Override defaults with a variant label followed by one or more CSV files or directories. Repeat per variant.",
    )
    parser.add_argument("--output", type=Path, default=Path("lns_variant_completed_scenarios.png"), help="Output PNG/image path.")
    parser.add_argument("--output-pdf", type=Path, help="Optional PDF output path.")
    parser.add_argument("--output-svg", type=Path, help="Optional SVG output path.")
    parser.add_argument("--time-limit-s", type=float, default=600.0, help="Padded slot length per map/agent experiment.")
    parser.add_argument("--expected-scenarios", type=int, default=100, help="Expected scenario count per map/agent experiment.")
    parser.add_argument("--title", default="LNS variants: completed scenarios over padded experiment time", help="Plot title.")
    parser.add_argument(
        "--exclude-pattern",
        action="append",
        default=list(DEFAULT_EXCLUDE_PATTERNS),
        help="Case-insensitive text pattern to exclude from paths and map names. Repeat to add more exclusions.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        variant_sources = _parse_variant_overrides(args.variant)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    loaded = load_all_variants(variant_sources, exclude_patterns=args.exclude_pattern)
    outputs = [args.output]
    if args.output_pdf:
        outputs.append(args.output_pdf)
    if args.output_svg:
        outputs.append(args.output_svg)
    plot_comparison(
        loaded,
        outputs=outputs,
        title=args.title,
        time_limit_s=args.time_limit_s,
        expected_scenarios=args.expected_scenarios,
    )


if __name__ == "__main__":
    main()