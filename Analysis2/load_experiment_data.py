"""Utilities for loading and grouping LNS experiment logs.

This helper focuses on the ``experiments.csv`` files produced by the LNS
experiments.  It provides small wrappers around :mod:`pandas` to aggregate the
raw CSV files per map and per agent count.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Mapping, MutableMapping, Tuple
import re

import pandas as pd

LogRoot = Path
MapName = str
NumAgents = int


@dataclass(frozen=True)
class MapAgentKey:
    """A composite key combining the map folder and number of agents."""

    map_name: MapName
    num_agents: NumAgents

    def __str__(self) -> str:  # pragma: no cover - convenience only
        return f"{self.map_name}-{self.num_agents}agents"


@dataclass(frozen=True)
class ActualMapAgentKey:
    """A composite key combining the actual map name and number of agents."""

    actual_map_name: str
    num_agents: NumAgents

    def __str__(self) -> str:  # pragma: no cover - convenience only
        return f"{self.actual_map_name}-{self.num_agents}agents"


def find_map_directories(log_root: LogRoot) -> Iterable[Path]:
    """Yield every map directory that contains an ``experiments.csv`` file.

    Parameters
    ----------
    log_root:
        Path pointing to the ``lns_clean/logs/LNS`` directory.
    """

    for child in sorted(log_root.iterdir()):
        if not child.is_dir():
            continue
        if (child / "experiments.csv").is_file():
            yield child


def load_map_experiments(map_dir: Path) -> pd.DataFrame:
    """Load the ``experiments.csv`` file for a single map directory."""

    experiment_path = map_dir / "experiments.csv"
    df = pd.read_csv(experiment_path)
    df = df.assign(map_folder=map_dir.name)
    return df


def extract_actual_map_name(map_path: str) -> str:
    """Extract the actual map name from a map file path.
    
    For example: '/path/to/empty-16-16.map' -> 'empty-16-16'
    """
    # Extract the filename from the path and remove the .map extension
    map_filename = Path(map_path).name
    return map_filename.replace('.map', '')


def load_all_experiments(log_root: LogRoot) -> Dict[MapName, pd.DataFrame]:
    """Read every map's ``experiments.csv`` file into a :class:`DataFrame`.

    The returned dictionary maps each map folder name to the associated DataFrame
    so that consumers can perform additional processing if needed.
    """

    map_frames: MutableMapping[MapName, pd.DataFrame] = {}
    for map_dir in find_map_directories(log_root):
        map_frames[map_dir.name] = load_map_experiments(map_dir)
    return dict(map_frames)


def split_by_map_and_agents(
    map_frames: Mapping[MapName, pd.DataFrame],
) -> Dict[MapAgentKey, pd.DataFrame]:
    """Split every map DataFrame into separate DataFrames per agent count."""

    split_frames: MutableMapping[MapAgentKey, pd.DataFrame] = {}
    for map_name, frame in map_frames.items():
        if "num_agents" not in frame.columns:
            raise KeyError(
                f"Expected a 'num_agents' column in experiments for {map_name}"
            )
        for num_agents, group in frame.groupby("num_agents", sort=True):
            key = MapAgentKey(map_name=map_name, num_agents=int(num_agents))
            split_frames[key] = group.reset_index(drop=True)
    return dict(split_frames)


def split_by_actual_map_and_agents(
    map_frames: Mapping[MapName, pd.DataFrame],
) -> Dict[ActualMapAgentKey, pd.DataFrame]:
    """Split every map DataFrame into separate DataFrames per actual map name and agent count."""

    split_frames: MutableMapping[ActualMapAgentKey, pd.DataFrame] = {}
    for map_name, frame in map_frames.items():
        if "num_agents" not in frame.columns:
            raise KeyError(
                f"Expected a 'num_agents' column in experiments for {map_name}"
            )
        if "map_path" not in frame.columns:
            raise KeyError(
                f"Expected a 'map_path' column in experiments for {map_name}"
            )
        
        # Add actual map name column
        frame_with_actual_map = frame.copy()
        frame_with_actual_map['actual_map_name'] = frame_with_actual_map['map_path'].apply(extract_actual_map_name)
        
        for (actual_map_name, num_agents), group in frame_with_actual_map.groupby(['actual_map_name', 'num_agents'], sort=True):
            key = ActualMapAgentKey(actual_map_name=actual_map_name, num_agents=int(num_agents))
            split_frames[key] = group.reset_index(drop=True)
    return dict(split_frames)


def load_and_split(log_root: LogRoot) -> Tuple[Dict[MapName, pd.DataFrame], Dict[MapAgentKey, pd.DataFrame]]:
    """Convenience function to load and split experiments in one call."""

    map_frames = load_all_experiments(log_root)
    split_frames = split_by_map_and_agents(map_frames)
    return map_frames, split_frames


def load_and_split_by_actual_map(log_root: LogRoot) -> Tuple[Dict[MapName, pd.DataFrame], Dict[ActualMapAgentKey, pd.DataFrame]]:
    """Convenience function to load and split experiments by actual map name in one call."""

    map_frames = load_all_experiments(log_root)
    split_frames = split_by_actual_map_and_agents(map_frames)
    return map_frames, split_frames


def _default_log_root() -> LogRoot:
    """Return the default log directory relative to this module."""

    module_root = Path(__file__).resolve().parent
    return module_root / "logs" / "LNS"


def main() -> None:  # pragma: no cover - convenience CLI helper
    """Print a short summary of the per-map/per-agent splits."""

    log_root = _default_log_root()
    map_frames, split_frames = load_and_split(log_root)

    print(f"Loaded experiments for {len(map_frames)} map(s) from {log_root}.")
    for map_name, frame in map_frames.items():
        agent_counts = sorted(frame["num_agents"].unique())
        print(f"- {map_name}: {len(frame)} rows across agent counts {agent_counts}")

    print("\nGenerated per-map/per-agent DataFrames:")
    for key, frame in split_frames.items():
        print(f"  * {key}: {len(frame)} rows")


if __name__ == "__main__":  # pragma: no cover
    main()