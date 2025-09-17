"""Visualization helper for MAPF LNS solver outputs.

This script converts textual agent paths produced by the LNS solver into an
animated visualization.  The map is rendered as a grid with black obstacles
and white walkable tiles.  Each agent receives a unique color and its complete
path is drawn as a line.  A per-timestep animation is then generated showing
the position of every agent along its path, with the current timestep rendered
in the upper-left corner of the figure.

Example usage::

    python tools/visualize_lns_solution.py \
        --map mapf-map/empty-8-8.map \
        --solution sample_solution.txt \
        --output animation.gif

The solution file is expected to contain the blocks printed by
``SATSolverManager::print_agent_paths``::

    === Agent Paths ===
    Agent 0 path:
      Time 0: (0, 0)
      Time 1: (0, 1)
      ...

By default the animation is saved as a GIF with one frame per second.  Saving
GIFs requires the ``pillow`` package.  If ``pillow`` is not available the
script can still dump individual PNG frames via ``--frames-dir``.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.artist import Artist
from matplotlib.colors import ListedColormap
import numpy as np


Position = Tuple[int, int]
AgentPaths = Dict[int, List[Position]]


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize MAPF LNS solutions")
    parser.add_argument("--map", required=True, type=Path, help="Path to .map file")
    parser.add_argument(
        "--solution",
        required=True,
        type=Path,
        help=(
            "Path to solution file produced by SATSolverManager::print_agent_paths "
            "(or another file using the same text format)."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output animation file.  GIF is recommended (requires pillow).",
    )
    parser.add_argument(
        "--frames-dir",
        type=Path,
        help=(
            "Optional directory to dump per-timestep PNG frames.  This is useful "
            "when GIF support is unavailable."
        ),
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=1.0,
        help="Frames per second for the animation (default: 1 frame == 1 second).",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=150,
        help="Dots-per-inch for saved figures (default: 150).",
    )
    parser.add_argument(
        "--title",
        type=str,
        default="MAPF LNS Solution",
        help="Optional title to display above the animation.",
    )
    return parser.parse_args()


def load_map(path: Path) -> np.ndarray:
    """Load an ASCII .map file into a numpy array.

    Obstacles are encoded as 1, walkable tiles as 0.
    """

    contents = path.read_text().splitlines()
    try:
        map_index = next(i for i, line in enumerate(contents) if line.strip().lower() == "map")
    except StopIteration as exc:
        raise ValueError(f"Map file {path} does not contain a 'map' header") from exc

    grid_lines = contents[map_index + 1 :]
    if not grid_lines:
        raise ValueError(f"Map file {path} contains no grid data")

    width = len(grid_lines[0].strip())
    grid = np.zeros((len(grid_lines), width), dtype=int)
    obstacle_chars = {"@", "T", "O", "S", "W", "#"}

    for r, line in enumerate(grid_lines):
        row = line.rstrip("\n")
        if len(row) != width:
            raise ValueError(
                f"Inconsistent row length in map {path}: expected {width}, got {len(row)}"
            )
        for c, char in enumerate(row):
            grid[r, c] = 1 if char in obstacle_chars else 0

    return grid


def parse_solution(path: Path) -> AgentPaths:
    """Parse agent paths from a solver log.

    The function expects lines that match ``Agent <id> path:`` followed by
    ``Time <t>: (<row>, <col>)`` entries.  Additional text in the file is
    ignored.
    """

    agent_re = re.compile(r"^\s*Agent\s+(\d+)\s+path\s*:", re.IGNORECASE)
    time_re = re.compile(r"^\s*Time\s+(\d+)\s*:\s*\(([-\d]+),\s*([-\d]+)\)")

    agent_paths: AgentPaths = {}
    current_agent: int | None = None
    current_path: List[Tuple[int, int]] = []

    def flush_current_agent() -> None:
        nonlocal current_agent, current_path
        if current_agent is None:
            return
        if not current_path:
            raise ValueError(f"Agent {current_agent} has no recorded positions in {path}")
        agent_paths[current_agent] = current_path
        current_agent = None
        current_path = []

    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            agent_match = agent_re.match(line)
            if agent_match:
                flush_current_agent()
                current_agent = int(agent_match.group(1))
                continue

            if current_agent is None:
                continue

            time_match = time_re.match(line)
            if time_match:
                timestep = int(time_match.group(1))
                row = int(time_match.group(2))
                col = int(time_match.group(3))

                if timestep != len(current_path):
                    if timestep < len(current_path):
                        raise ValueError(
                            f"Time steps for agent {current_agent} are not strictly increasing"
                        )
                    # Fill in missing intermediate timesteps by repeating the last position
                    if not current_path:
                        raise ValueError(
                            f"Agent {current_agent} starts at timestep {timestep}, expected 0"
                        )
                    last_row, last_col = current_path[-1]
                    for _ in range(len(current_path), timestep):
                        current_path.append((last_row, last_col))

                current_path.append((row, col))

    flush_current_agent()

    if not agent_paths:
        raise ValueError(f"No agent paths found in {path}")

    return agent_paths


def build_positions_per_timestep(agent_paths: AgentPaths) -> Tuple[List[int], List[List[Position]]]:
    agent_ids = sorted(agent_paths.keys())
    max_timesteps = max(len(path) for path in agent_paths.values())

    frame_positions: List[List[Position]] = []
    for timestep in range(max_timesteps):
        positions: List[Position] = []
        for agent_id in agent_ids:
            path = agent_paths[agent_id]
            if timestep < len(path):
                positions.append(path[timestep])
            else:
                positions.append(path[-1])
        frame_positions.append(positions)

    return agent_ids, frame_positions


def configure_axes(ax: plt.Axes, grid: np.ndarray, title: str) -> None:
    ax.imshow(grid, cmap=ListedColormap([[1, 1, 1], [0, 0, 0]]), origin="upper")
    rows, cols = grid.shape

    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="lightgray", linewidth=0.3)

    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title)


def plot_agent_paths(ax: plt.Axes, agent_paths: AgentPaths, agent_ids: Sequence[int], colors: Sequence[Tuple[float, float, float, float]]) -> None:
    for color, agent_id in zip(colors, agent_ids):
        path = agent_paths[agent_id]
        cols = [col + 0.5 for (_, col) in path]
        rows = [row + 0.5 for (row, _) in path]
        ax.plot(cols, rows, color=color, linewidth=2.0, alpha=0.75, zorder=2)


def create_animation(
    grid: np.ndarray,
    agent_paths: AgentPaths,
    agent_ids: Sequence[int],
    frame_positions: Sequence[Sequence[Position]],
    fps: float,
    title: str,
    dpi: int,
    output_path: Path,
    frames_dir: Path | None,
) -> None:
    rows, cols = grid.shape
    cmap = plt.cm.get_cmap("tab20", max(len(agent_ids), 1))
    colors = [cmap(i) for i in range(len(agent_ids))]

    fig, ax = plt.subplots(figsize=(max(cols / 4, 4), max(rows / 4, 4)))
    configure_axes(ax, grid, title)
    plot_agent_paths(ax, agent_paths, agent_ids, colors)

    # Prepare scatter points for dynamic agent positions.
    scatter = ax.scatter([], [], c=colors, s=80, edgecolors="black", linewidths=0.6, zorder=3)
    id_labels = [
        ax.text(0, 0, str(agent_id), color="black", fontsize=8, ha="center", va="center", zorder=4)
        for agent_id in agent_ids
    ]
    time_text = ax.text(
        0.02,
        0.95,
        "",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=12,
        color="black",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7),
    )

    def init() -> List[Artist]:
        scatter.set_offsets(np.empty((0, 2)))
        time_text.set_text("")
        for label in id_labels:
            label.set_visible(False)
        return [scatter, time_text, *id_labels]

    def update(frame_index: int) -> List[Artist]:
        positions = frame_positions[frame_index]
        offsets = np.array([[col + 0.5, row + 0.5] for row, col in positions])
        scatter.set_offsets(offsets)
        for label, (row, col) in zip(id_labels, positions):
            label.set_position((col + 0.5, row + 0.5))
            label.set_visible(True)
        time_text.set_text(f"t = {frame_index}")
        return [scatter, time_text, *id_labels]

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=len(frame_positions),
        init_func=init,
        interval=1000.0 / max(fps, 1e-9),
        blit=False,
    )

    if frames_dir is not None:
        frames_dir.mkdir(parents=True, exist_ok=True)
        for frame_index in range(len(frame_positions)):
            update(frame_index)
            fig.savefig(frames_dir / f"frame_{frame_index:04d}.png", dpi=dpi)
        # Reset animation to avoid duplicate drawing when saving video
        init()

    if output_path:
        suffix = output_path.suffix.lower()
        if suffix == ".gif":
            try:
                from matplotlib.animation import PillowWriter
            except ImportError as exc:  # pragma: no cover - exercised manually
                raise RuntimeError(
                    "Saving GIF files requires the 'pillow' package. "
                    "Install it with 'pip install pillow' or use --frames-dir"
                ) from exc
            writer = PillowWriter(fps=fps)
            anim.save(str(output_path), writer=writer, dpi=dpi)
        elif suffix in {".mp4", ".mkv", ".mov", ".avi"}:
            try:
                writer = animation.FFMpegWriter(fps=fps)
            except FileNotFoundError as exc:  # pragma: no cover - environment specific
                raise RuntimeError(
                    "Saving to video formats requires ffmpeg. "
                    "Install ffmpeg or choose a .gif output."
                ) from exc
            anim.save(str(output_path), writer=writer, dpi=dpi)
        else:
            raise ValueError(
                f"Unsupported output extension '{suffix}'. Use .gif or a common video extension."
            )

    plt.close(fig)


def main() -> None:
    args = parse_arguments()

    grid = load_map(args.map)
    agent_paths = parse_solution(args.solution)
    agent_ids, frame_positions = build_positions_per_timestep(agent_paths)

    if not agent_ids:
        raise RuntimeError("No agents to visualize")

    create_animation(
        grid=grid,
        agent_paths=agent_paths,
        agent_ids=agent_ids,
        frame_positions=frame_positions,
        fps=args.fps,
        title=args.title,
        dpi=args.dpi,
        output_path=args.output,
        frames_dir=args.frames_dir,
    )


if __name__ == "__main__":
    main()