#!/usr/bin/env python3
"""Plot cumulative completed MAPF scenarios over sequential experiments.

This script is intentionally self-contained: it reads one or more
``experiments.csv`` files, groups rows by map and agent count, orders those
experiments by map name and then agent count, and draws one cumulative curve over
all selected experiments.

Each map/agent experiment receives a fixed 10 minute (600 second) slot by
default.  Solved scenarios increase the curve at their cumulative runtime within
that slot.  If fewer than the expected number of scenarios are solved, the curve
stays flat until the slot reaches its 10 minute limit before the next map/agent
experiment starts.

Example:

    python Analysis/plot_completed_scenarios_over_time.py \
        lns_clean/logs/LNS \
        --output completed_scenarios_over_time.png
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib.pyplot as plt

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


@dataclass(frozen=True, order=True)
class ExperimentKey:
    """A map/agent combination that will be plotted as one time slot."""

    map_name: str
    agent_sort: tuple[int, float | str]
    agents: str = field(compare=False)


@dataclass
class ScenarioRun:
    """One row from an experiment log."""

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


def discover_experiment_csvs(paths: Iterable[Path]) -> list[Path]:
    """Return explicit CSVs or recursively discovered ``experiments.csv`` files."""

    discovered: list[Path] = []
    for path in paths:
        expanded = path.expanduser()
        if expanded.is_file():
            discovered.append(expanded)
            continue
        if expanded.is_dir():
            discovered.extend(sorted(expanded.rglob("experiments.csv"), key=lambda p: str(p)))
            continue
        raise FileNotFoundError(f"Input path does not exist: {expanded}")

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


def load_experiments(csv_paths: Iterable[Path]) -> dict[ExperimentKey, list[ScenarioRun]]:
    """Load all rows grouped by normalised map name and agent count."""

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


def build_timeline(
    grouped: dict[ExperimentKey, list[ScenarioRun]],
    *,
    time_limit_s: float,
    expected_scenarios: int,
) -> tuple[list[float], list[int], list[tuple[ExperimentKey, float, float, int]]]:
    """Build a single cumulative step timeline and experiment schedule."""

    x_values = [0.0]
    y_values = [0]
    elapsed = 0.0
    completed_total = 0
    schedule: list[tuple[ExperimentKey, float, float, int]] = []

    for key in sorted(grouped):
        start = elapsed
        slot_end = start + time_limit_s
        within_slot = 0.0
        solved_in_slot = 0
        rows = sorted(grouped[key], key=lambda item: item.order)

        for run in rows:
            if not run.solved or solved_in_slot >= expected_scenarios:
                continue
            within_slot += run.runtime_s
            event_time = min(start + within_slot, slot_end)
            if event_time < x_values[-1]:
                event_time = x_values[-1]
            completed_total += 1
            solved_in_slot += 1
            x_values.extend([event_time, event_time])
            y_values.extend([y_values[-1], completed_total])
            if event_time >= slot_end:
                break

        if x_values[-1] < slot_end:
            x_values.append(slot_end)
            y_values.append(completed_total)
        elapsed = slot_end
        schedule.append((key, start, slot_end, solved_in_slot))

    return x_values, y_values, schedule


def plot_timeline(
    x_values: Sequence[float],
    y_values: Sequence[int],
    schedule: Sequence[tuple[ExperimentKey, float, float, int]],
    *,
    output: Path,
    title: str,
    expected_scenarios: int,
    time_limit_s: float,
) -> None:
    """Render and save the cumulative scenario-completion plot."""

    fig_height = max(6.0, 4.5 + 0.18 * len(schedule))
    fig, ax = plt.subplots(figsize=(14, fig_height))
    top_margin = min(0.55, 0.16 + 0.018 * len(schedule))
    fig.subplots_adjust(top=1.0 - top_margin)

    ax.plot(x_values, y_values, color="#0072B2", linewidth=2.4)
    ax.set_title(title, pad=18)
    ax.set_xlabel("Elapsed padded experiment time (seconds)")
    ax.set_ylabel("Cumulative completed scenarios")
    ax.grid(True, which="both", axis="both", alpha=0.25)

    for key, start, end, solved in schedule:
        ax.axvline(start, color="0.85", linewidth=0.8, zorder=0)
        midpoint = (start + end) / 2.0
        ax.text(
            midpoint,
            -0.04,
            f"{key.map_name}\n{key.agents} agents",
            transform=ax.get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=8,
            rotation=45,
        )
    if schedule:
        ax.axvline(schedule[-1][2], color="0.85", linewidth=0.8, zorder=0)

    padding_min = time_limit_s / 60.0
    entries = [
        f"{idx + 1}. {key.map_name}, {key.agents} agents, {padding_min:g} min ({solved}/{expected_scenarios} solved)"
        for idx, (key, _start, _end, solved) in enumerate(schedule)
    ]
    wrapped_lines = ["Included experiments and padding (ordered by map):"]
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

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=200, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path, help="CSV files or directories containing experiments.csv logs.")
    parser.add_argument("--output", type=Path, default=Path("completed_scenarios_over_time.png"), help="Output image path.")
    parser.add_argument("--time-limit-s", type=float, default=600.0, help="Padded slot length per map/agent experiment.")
    parser.add_argument("--expected-scenarios", type=int, default=100, help="Expected scenario count per map/agent experiment.")
    parser.add_argument("--title", default="Completed scenarios over padded experiment time", help="Plot title.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_paths = discover_experiment_csvs(args.inputs)
    grouped = load_experiments(csv_paths)
    if not grouped:
        raise SystemExit("No usable experiment rows were found.")
    x_values, y_values, schedule = build_timeline(
        grouped,
        time_limit_s=args.time_limit_s,
        expected_scenarios=args.expected_scenarios,
    )
    plot_timeline(
        x_values,
        y_values,
        schedule,
        output=args.output,
        title=args.title,
        expected_scenarios=args.expected_scenarios,
        time_limit_s=args.time_limit_s,
    )
    print(f"Wrote {args.output} with {len(schedule)} included experiment(s).")


if __name__ == "__main__":
    main()