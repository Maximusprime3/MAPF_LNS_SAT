from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from load_experiment_data import (
	ActualMapAgentKey,
	extract_actual_map_name,
	load_all_experiments,
)

# Match the experiment logic: per map/agent combination we expect up to 100 instances
# and a 600 second time budget. When fewer than 100 solves occurred, we treat the
# remaining as consuming the full 600 seconds for the combination when tallying
# the wall time needed to "finish" that combination.
EXPECTED_PER_COMBINATION = 100
TIME_LIMIT_PER_COMBINATION_S = 600.0


def compute_experiment_completion_timeline(map_frames: Dict[str, pd.DataFrame]) -> pd.DataFrame:
	"""Return a dataframe with experiment completion times across all maps and agent combinations.

	For each map/agent combination, we track when experiments complete within the 600s budget.
	Returns a timeline of cumulative experiment completions.
	"""

	all_completion_times: List[float] = []
	
	for map_folder, df in map_frames.items():
		if df.empty:
			continue
		if "map_path" not in df.columns or "num_agents" not in df.columns:
			continue
		# derive actual map name
		df = df.copy()
		df["actual_map_name"] = df["map_path"].map(extract_actual_map_name)
		# choose a runtime column in seconds
		runtime_col = None
		for candidate in ("total_runtime_ms", "total_time_s", "total_time", "runtime_s"):
			if candidate in df.columns:
				runtime_col = candidate
				break
		if runtime_col is None:
			continue
		# normalise to seconds
		runtime_s = pd.to_numeric(df[runtime_col], errors="coerce")
		if runtime_col.endswith("_ms"):
			runtime_s = runtime_s / 1000.0
		df["__runtime_s__"] = runtime_s
		# success filter: use 'solved' if present; otherwise include all
		if "solved" in df.columns:
			df_success = df[df["solved"].fillna(0) != 0].copy()
		else:
			df_success = df.copy()
		
		# group by actual map and agent count
		for (actual_map, num_agents), group in df.groupby(["actual_map_name", "num_agents"], sort=False):
			group_success = df_success[(df_success["actual_map_name"] == actual_map) & (df_success["num_agents"] == num_agents)]
			solve_times = group_success["__runtime_s__"].dropna().to_numpy(dtype=float)
			n_completed = int(solve_times.size)
			
			if n_completed == 0:
				# No successful experiments, skip this combination
				continue
			
			# Sort solve times and add them to the timeline
			solve_times = np.sort(solve_times)
			# Cap at expected number of experiments
			solve_times = solve_times[:EXPECTED_PER_COMBINATION]
			
			# Add completion times for this map/agent combination
			all_completion_times.extend(solve_times.tolist())

	if not all_completion_times:
		return pd.DataFrame(columns=["completion_time_s", "cumulative_experiments"])
	
	# Sort all completion times
	all_completion_times = sorted(all_completion_times)
	
	# Create timeline with cumulative counts
	timeline_data = []
	for i, time in enumerate(all_completion_times):
		timeline_data.append({
			"completion_time_s": time,
			"cumulative_experiments": i + 1
		})
	
	return pd.DataFrame(timeline_data)


def plot_experiment_completion_cdf(timeline_df: pd.DataFrame, *, title: str | None = None, logx: bool = False) -> plt.Axes:
	"""Plot CDF of experiment completion times showing cumulative number of experiments completed."""
	
	if timeline_df.empty:
		raise ValueError("No experiment completion data available to plot.")
	
	times = timeline_df["completion_time_s"].to_numpy(dtype=float)
	cumulative = timeline_df["cumulative_experiments"].to_numpy(dtype=int)
	
	fig, ax = plt.subplots(figsize=(8, 5))
	ax.step(times, cumulative, where="post", linewidth=2)
	ax.set_xlabel("Experiment completion time [s]")
	ax.set_ylabel("Cumulative number of experiments completed")
	ax.set_ylim(0, cumulative.max() if len(cumulative) > 0 else 1)
	if logx:
		# Avoid zero on log scale
		times_min = times[times > 0]
		if times_min.size:
			ax.set_xscale("log")
	if title:
		ax.set_title(title)
	ax.grid(True, which="both", linestyle="--", linewidth=0.8, alpha=0.5)
	return ax


def main() -> None:
	parser = argparse.ArgumentParser(description="Plot CDF of experiment completion times from LNS logs.")
	parser.add_argument("log_root", type=Path, nargs="?", default=Path("lns_clean/logs/LNS"), help="Root directory containing map folders with experiments.csv")
	parser.add_argument("--output", type=Path, help="Optional path to save the figure")
	parser.add_argument("--title", type=str, default="LNS: cumulative experiment completions (CDF)")
	parser.add_argument("--show", action="store_true", help="Show the plot interactively")
	parser.add_argument("--logx", action="store_true", help="Use log scale for the x-axis")
	args = parser.parse_args()

	map_frames = load_all_experiments(args.log_root)
	timeline = compute_experiment_completion_timeline(map_frames)
	if timeline.empty:
		raise SystemExit("No data available under the provided log root.")

	ax = plot_experiment_completion_cdf(timeline, title=args.title, logx=args.logx)
	if args.output:
		args.output.parent.mkdir(parents=True, exist_ok=True)
		ax.figure.savefig(args.output, dpi=300, bbox_inches="tight")
	if args.show or not args.output:
		plt.show()


if __name__ == "__main__":
	main()
