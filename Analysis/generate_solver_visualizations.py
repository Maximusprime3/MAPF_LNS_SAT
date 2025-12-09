"""Generate a portfolio of MAPF SAT-solver visualizations.

This script aggregates the CSV logs produced by the MAPF LNS + SAT experiments
and renders a collection of figures that highlight solver performance,
search dynamics, and outcome distributions.  The goal is to provide
publication-ready graphics that align with the "banger plots" suggested in the
analysis notes for an ICAART submission.

The script expects the following CSV files to be present under the repository's
``data/`` directory:

``solver_log_minisat.csv``
``solver_log_probsat.csv``
``solver_log_collisions_minisat.csv``
``solver_log_collisions_probsat.csv``
``solver_log_timesteps_minisat.csv``
``solver_log_timesteps_probsat.csv``

Each figure is saved under ``Analysis/figures`` with a descriptive filename.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
import seaborn as sns

import matplotlib.pyplot as plt

try:
    import plotly.graph_objects as go

    PLOTLY_AVAILABLE = True
except ImportError:  # pragma: no cover - plotly is optional.
    PLOTLY_AVAILABLE = False


sns.set_theme(context="talk", style="whitegrid")

DATA_DIR = Path("data")
OUTPUT_DIR = Path("Analysis") / "figures"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def _load_csvs() -> Dict[str, pd.DataFrame]:
    """Load the solver CSVs into dataframes indexed by a descriptive key."""

    csv_map = {
        "solver_log_minisat": DATA_DIR / "solver_log_minisat.csv",
        "solver_log_probsat": DATA_DIR / "solver_log_probsat.csv",
        "solver_log_collisions_minisat": DATA_DIR / "solver_log_collisions_minisat.csv",
        "solver_log_collisions_probsat": DATA_DIR / "solver_log_collisions_probsat.csv",
        "solver_log_timesteps_minisat": DATA_DIR / "solver_log_timesteps_minisat.csv",
        "solver_log_timesteps_probsat": DATA_DIR / "solver_log_timesteps_probsat.csv",
    }

    dataframes: Dict[str, pd.DataFrame] = {}
    missing: List[str] = []

    for key, path in csv_map.items():
        if not path.exists():
            missing.append(str(path))
            continue
        df = pd.read_csv(path)
        dataframes[key] = df

    if missing:
        raise FileNotFoundError(
            "Missing required solver logs: \n" + "\n".join(f" - {m}" for m in missing)
        )

    return dataframes


def _ensure_numeric(series: pd.Series) -> pd.Series:
    """Convert a mixed series to numeric, coercing errors to NaN."""

    return pd.to_numeric(series, errors="coerce")


def _parse_semicolon_series(series: pd.Series) -> List[List[float]]:
    """Split a semicolon-delimited string series into lists of floats."""

    parsed: List[List[float]] = []
    for value in series.fillna(""):
        if not value:
            parsed.append([])
            continue
        parts = [p for p in str(value).split(";") if p != ""]
        parsed.append([float(p) for p in parts])
    return parsed


def _explode_iteration_metric(
    df: pd.DataFrame,
    value_column: str,
    solver: str,
    iteration_label: str,
    add_columns: Sequence[str],
) -> pd.DataFrame:
    """Explode a per-iteration metric into long form for ridgeline plots."""

    rows: List[Dict[str, object]] = []
    per_iter = _parse_semicolon_series(df[value_column])
    for (_, row), values in zip(df.iterrows(), per_iter):
        base_info = {col: row[col] for col in add_columns}
        base_info["solver"] = solver
        for idx, val in enumerate(values):
            rows.append({**base_info, iteration_label: idx, "value": val})
    return pd.DataFrame(rows)


def _format_solver(solver: str) -> str:
    return "MiniSAT" if solver.lower().startswith("mini") else "probSAT"


def _prepare_solver_log(log_dfs: Dict[str, pd.DataFrame]) -> pd.DataFrame:
    """Merge solver-level logs into a single dataframe with common columns."""

    records: List[pd.DataFrame] = []
    for key, df in log_dfs.items():
        if not key.startswith("solver_log_"):
            continue
        if "collisions" in key or "timesteps" in key:
            continue
        solver_name = _format_solver(df["solver"].iloc[0]) if not df.empty else key
        df = df.copy()
        df["solver"] = solver_name
        df["cnf_size"] = df["cnf_vars_end"] + df["cnf_clauses_end"]
        df["total_solver_time_s"] = _ensure_numeric(df["total_solver_time_s"])
        records.append(df)
    return pd.concat(records, ignore_index=True)


# ---------------------------------------------------------------------------
# Visualization builders
# ---------------------------------------------------------------------------

def plot_performance_skyline(solver_log: pd.DataFrame) -> Path:
    """Scatter plots of total solver time vs CNF size with Pareto frontiers."""

    def compute_front(df: pd.DataFrame, x_col: str, y_col: str) -> pd.DataFrame:
        subset = df.sort_values(x_col)
        best = math.inf
        front_rows = []
        for _, row in subset.iterrows():
            if row[y_col] < best:
                front_rows.append(row[[x_col, y_col]].to_dict())
                best = row[y_col]
        return pd.DataFrame(front_rows)

    facet_col = "solver"
    palette = sns.color_palette("viridis", n_colors=solver_log["map_name"].nunique())

    g = sns.relplot(
        data=solver_log,
        x="cnf_size",
        y="total_solver_time_s",
        hue="map_name",
        style="num_agents",
        col=facet_col,
        kind="scatter",
        palette=palette,
        facet_kws={"sharex": False, "sharey": False},
        height=5,
        col_wrap=2,
    )

    for ax, (solver, df_solver) in zip(g.axes.flatten(), solver_log.groupby(facet_col)):
        frontier = compute_front(df_solver, "cnf_size", "total_solver_time_s")
        if not frontier.empty:
            ax.plot(
                frontier["cnf_size"],
                frontier["total_solver_time_s"],
                color="black",
                linewidth=2,
                label="Pareto frontier",
            )
            ax.legend(loc="best")
        ax.set_title(f"{solver}: Solve time vs CNF size")
        ax.set_xlabel("CNF vars + clauses")
        ax.set_ylabel("Total solver time (s)")
        ax.set_yscale("log")

    g.fig.suptitle("Performance skylines across solvers", fontsize=18, y=1.03)
    output_path = OUTPUT_DIR / "performance_skyline.png"
    g.fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(g.fig)
    return output_path


def plot_collision_spiral(collisions: pd.DataFrame) -> Path:
    """Polar spiral showing cumulative solve time per collision iteration."""

    solvers = sorted(collisions["solver"].unique())
    n_cols = len(solvers)
    fig, axes = plt.subplots(
        1,
        n_cols,
        subplot_kw={"projection": "polar"},
        figsize=(6 * n_cols, 6),
        squeeze=False,
    )

    for ax, solver in zip(axes[0], solvers):
        subset = collisions[collisions["solver"] == solver]
        groups = subset.groupby(["map_name", "num_agents", "timestep", "seed"])
        sampled_groups = list(groups)
        max_traces = 12
        if len(sampled_groups) > max_traces:
            sampled_groups = sampled_groups[:max_traces]

        for (map_name, num_agents, timestep, seed), run_df in sampled_groups:
            run_df = run_df.sort_values("collision_iter")
            if run_df.empty:
                continue
            theta = run_df["collision_iter"].to_numpy(dtype=float)
            # Spread the angle to create a spiral effect.
            theta_range = theta.max() - theta.min()
            norm = theta_range if theta_range > 0 else 1.0
            theta = (theta - theta.min()) / norm * 4 * math.pi
            radius = run_df["solver_time_s"].cumsum()
            label = f"{Path(map_name).name} | {num_agents} agents"
            ax.plot(theta, radius, label=label)

        ax.set_title(f"{solver}: Collision repair spiral")
        ax.set_theta_direction(-1)
        ax.set_theta_offset(math.pi / 2.0)
        ax.set_ylabel("Cumulative solver time (s)")
        ax.legend(loc="upper right", fontsize="small", frameon=True)

    fig.suptitle("Collision iteration spirals", fontsize=18)
    output_path = OUTPUT_DIR / "collision_iteration_spiral.png"
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return output_path


def plot_ridgeline_search_effort(minisat_log: pd.DataFrame, probsat_log: pd.DataFrame) -> Path:
    """Ridgeline-style KDEs of per-iteration search effort for both solvers."""

    mini_long = _explode_iteration_metric(
        minisat_log,
        value_column="decisions_per_iter",
        solver="MiniSAT",
        iteration_label="iteration",
        add_columns=["map_name", "num_agents"],
    )
    mini_long["metric"] = "Decisions"

    prob_long = _explode_iteration_metric(
        probsat_log,
        value_column="flips_per_iter",
        solver="probSAT",
        iteration_label="iteration",
        add_columns=["map_name", "num_agents"],
    )
    prob_long["metric"] = "Flips"

    combined = pd.concat([mini_long, prob_long], ignore_index=True)

    # Select representative agent counts to keep the plot readable.
    top_agent_counts = (
        combined.groupby("num_agents").size().sort_values(ascending=False).head(6).index
    )
    combined = combined[combined["num_agents"].isin(top_agent_counts)]

    combined["num_agents"] = (
        pd.Categorical(combined["num_agents"], categories=sorted(top_agent_counts), ordered=True)
    )

    g = sns.FacetGrid(
        combined,
        row="num_agents",
        col="solver",
        hue="num_agents",
        sharex=False,
        sharey=False,
        height=1.8,
        aspect=4,
    )
    g.map_dataframe(sns.kdeplot, x="value", fill=True, alpha=0.75)
    g.set_axis_labels("Per-iteration effort", "Density")
    g.add_legend(title="# agents")
    g.set_titles(col_template="{col_name}", row_template="{row_name} agents")
    g.fig.suptitle("Ridgeline view of solver search effort", fontsize=18, y=1.02)

    output_path = OUTPUT_DIR / "search_effort_ridgeline.png"
    g.fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(g.fig)
    return output_path


def plot_layered_timeline(timesteps: pd.DataFrame) -> Path:
    """Stacked area chart of solver workload over timesteps."""

    aggregated = (
        timesteps.groupby(["solver", "timestep"])[
            ["cnf_build_time_s", "total_solver_time_s", "num_collision_iterations"]
        ]
        .median()
        .reset_index()
    )

    solvers = aggregated["solver"].unique()
    fig, axes = plt.subplots(len(solvers), 1, figsize=(12, 5 * len(solvers)), sharex=True)
    if not isinstance(axes, np.ndarray):
        axes = np.array([axes])

    for ax, solver in zip(axes, solvers):
        df_solver = aggregated[aggregated["solver"] == solver].sort_values("timestep")
        if df_solver.empty:
            continue

        ax.stackplot(
            df_solver["timestep"],
            df_solver["cnf_build_time_s"],
            df_solver["total_solver_time_s"],
            labels=["CNF build", "Solver"],
            alpha=0.8,
        )
        ax2 = ax.twinx()
        ax2.plot(
            df_solver["timestep"],
            df_solver["num_collision_iterations"],
            color="black",
            linewidth=2,
            label="# collision iterations",
        )
        ax.set_title(f"{solver}: workload over timesteps")
        ax.set_ylabel("Time (s)")
        ax2.set_ylabel("Collision iterations")
        ax.legend(loc="upper left")
        ax2.legend(loc="upper right")

    axes[-1].set_xlabel("Timestep")
    fig.suptitle("Layered timeline of MAPF SAT solving", fontsize=18)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    output_path = OUTPUT_DIR / "layered_timeline.png"
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    return output_path


def plot_outcome_heatmap(collisions: pd.DataFrame) -> Path:
    """Heatmap of SAT success rate by collision iteration and agent count."""

    collisions = collisions.copy()
    collisions["is_sat"] = (collisions["status"].str.upper() == "SAT").astype(float)

    aggregated = (
        collisions.groupby(["solver", "num_agents", "collision_iter"])["is_sat"].mean().reset_index()
    )

    solvers = aggregated["solver"].unique()
    fig, axes = plt.subplots(1, len(solvers), figsize=(8 * len(solvers), 6), squeeze=False)

    for ax, solver in zip(axes[0], solvers):
        df_solver = aggregated[aggregated["solver"] == solver]
        pivot = df_solver.pivot_table(
            index="num_agents",
            columns="collision_iter",
            values="is_sat",
            fill_value=np.nan,
        )
        sns.heatmap(
            pivot.sort_index(),
            ax=ax,
            cmap="viridis",
            vmin=0,
            vmax=1,
            cbar_kws={"label": "SAT success rate"},
        )
        ax.set_title(f"{solver}: outcome matrix")
        ax.set_xlabel("Collision iteration")
        ax.set_ylabel("# agents")

    fig.suptitle("Solver outcome heatmaps", fontsize=18)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    output_path = OUTPUT_DIR / "outcome_heatmap.png"
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    return output_path


def plot_scaling_curves(timesteps: pd.DataFrame) -> Path:
    """Log-log scaling curves of solver time and decisions vs agent count."""

    aggregated = (
        timesteps.groupby(["solver", "map_name", "num_agents"])[
            ["total_solver_time_s", "decisions"]
        ]
        .median()
        .reset_index()
    )

    # Remove non-positive values before switching to log scale.
    aggregated = aggregated[
        (aggregated["total_solver_time_s"] > 0) & (aggregated["decisions"] > 0)
    ]

    metrics = ["total_solver_time_s", "decisions"]
    fig, axes = plt.subplots(1, len(metrics), figsize=(12, 5), sharey=False)

    for ax, metric in zip(axes, metrics):
        sns.lineplot(
            data=aggregated,
            x="num_agents",
            y=metric,
            hue="solver",
            style="map_name",
            markers=True,
            dashes=False,
            ax=ax,
        )
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_title(f"Scaling of {metric.replace('_', ' ')}")
        ax.set_xlabel("Number of agents")
        ax.set_ylabel(metric.replace("_", " "))

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3)
    fig.suptitle("Solver scaling with agent count", fontsize=18)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    output_path = OUTPUT_DIR / "agent_count_scaling.png"
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    return output_path


def plot_sankey(solver_log: pd.DataFrame) -> Optional[Path]:
    """Comparative Sankey diagram linking maps to performance tiers."""

    if not PLOTLY_AVAILABLE:
        print("Plotly is not installed; skipping Sankey diagram.")
        return None

    solver_log = solver_log.copy()
    output_paths: List[Path] = []

    tier_labels = ["Fast", "Medium", "Slow"]
    all_records = []

    for solver, df_solver in solver_log.groupby("solver"):
        times = df_solver["total_solver_time_s"].astype(float)
        try:
            categories = pd.qcut(times, q=[0, 1 / 3, 2 / 3, 1], labels=tier_labels)
        except ValueError:
            # Fallback to equal-width bins when quantile edges collapse.
            categories = pd.cut(times, bins=len(tier_labels), labels=tier_labels)
        if categories.isnull().any():
            categories = categories.fillna("Medium")
        df_solver = df_solver.assign(tier=categories)
        all_records.append(df_solver)

    categorized = pd.concat(all_records, ignore_index=True)

    aggregated = (
        categorized.dropna(subset=["tier"])
        .groupby(["solver", "map_name", "tier"])
        .size()
        .reset_index(name="count")
    )

    for solver, df_solver in aggregated.groupby("solver"):
        maps = df_solver["map_name"].unique().tolist()
        tiers = [f"{solver} - {tier}" for tier in ["Fast", "Medium", "Slow"]]
        node_labels = maps + tiers
        source = []
        target = []
        value = []

        map_index = {name: idx for idx, name in enumerate(node_labels)}
        for _, row in df_solver.iterrows():
            src = map_index[row["map_name"]]
            tgt = map_index[f"{solver} - {row['tier']}"]
            source.append(src)
            target.append(tgt)
            value.append(row["count"])

        fig = go.Figure(
            data=[
                go.Sankey(
                    arrangement="snap",
                    node=dict(label=node_labels, pad=20, thickness=20),
                    link=dict(source=source, target=target, value=value),
                )
            ]
        )
        fig.update_layout(
            title_text=f"{solver}: Map-to-performance Sankey", font=dict(size=14)
        )
        output_path = OUTPUT_DIR / f"sankey_{solver.lower()}.html"
        fig.write_html(str(output_path))
        output_paths.append(output_path)

    if output_paths:
        print(
            "Generated Sankey diagrams (HTML) for the following solvers:\n" +
            "\n".join(f" - {path}" for path in output_paths)
        )
        return output_paths[0]  # Return first path for reference.
    return None


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    dfs = _load_csvs()

    solver_log = _prepare_solver_log(dfs)

    minisat_log = dfs["solver_log_minisat"].copy()
    minisat_log["solver"] = "MiniSAT"
    probsat_log = dfs["solver_log_probsat"].copy()
    probsat_log["solver"] = "probSAT"

    collisions = pd.concat(
        [
            dfs["solver_log_collisions_minisat"].assign(solver="MiniSAT"),
            dfs["solver_log_collisions_probsat"].assign(solver="probSAT"),
        ],
        ignore_index=True,
    )

    timesteps = pd.concat(
        [
            dfs["solver_log_timesteps_minisat"].assign(solver="MiniSAT"),
            dfs["solver_log_timesteps_probsat"].assign(solver="probSAT"),
        ],
        ignore_index=True,
    )

    figure_generators = [
        ("Performance skyline", lambda: plot_performance_skyline(solver_log)),
        ("Collision iteration spiral", lambda: plot_collision_spiral(collisions)),
        (
            "Ridgeline search effort",
            lambda: plot_ridgeline_search_effort(minisat_log, probsat_log),
        ),
        ("Layered timeline", lambda: plot_layered_timeline(timesteps)),
        ("Outcome heatmap", lambda: plot_outcome_heatmap(collisions)),
        ("Agent count scaling", lambda: plot_scaling_curves(timesteps)),
        ("Sankey diagram", lambda: plot_sankey(solver_log)),
    ]

    generated: List[Tuple[str, Optional[Path]]] = []
    for label, fn in figure_generators:
        try:
            path = fn()
        except Exception as exc:  # pragma: no cover - surfaces issues for debugging.
            print(f"Failed to generate {label}: {exc}")
            continue
        generated.append((label, path))

    summary_lines = ["Generated visualizations:"]
    for label, path in generated:
        if path is None:
            summary_lines.append(f" - {label}: skipped (see log above)")
        else:
            summary_lines.append(f" - {label}: {path}")

    print("\n".join(summary_lines))


if __name__ == "__main__":
    main()