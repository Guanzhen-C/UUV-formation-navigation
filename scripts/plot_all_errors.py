"""Generate all AUV error plots in a single run.

This script loads the per-node Excel logs once, then produces: (1) relative
distance errors for all node pairs with RMSE annotations, and (2) separate
figures for each axis of position, velocity, and attitude errors (nine figures
total), also with RMSE annotations.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from auv_log_utils import load_all_nodes


PAIRS: List[Tuple[int, int]] = [
    (1, 2),
    (2, 3),
    (4, 5),
    (5, 6),
    (7, 8),
    (8, 9),
    (1, 4),
    (2, 5),
    (3, 6),
    (4, 7),
    (5, 8),
    (6, 9),
]


COMMON_COLUMNS = [
    "t_sec",
    "est_x_m",
    "est_y_m",
    "est_z_m",
    "gt_x_m",
    "gt_y_m",
    "gt_z_m",
    "est_vx_mps",
    "est_vy_mps",
    "est_vz_mps",
    "gt_vx_mps",
    "gt_vy_mps",
    "gt_vz_mps",
    "est_roll_deg",
    "est_pitch_deg",
    "est_yaw_deg",
    "gt_roll_deg",
    "gt_pitch_deg",
    "gt_yaw_deg",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--logs-root",
        type=Path,
        default=Path("logs/excel"),
        help="Root directory containing node Excel logs",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("logs/plots"),
        help="Directory where all resulting figures will be saved",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.6,
        help="Maximum allowed time difference (seconds) when aligning nodes",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figures interactively in addition to saving them",
    )
    return parser.parse_args()


def compute_rmse(values: np.ndarray) -> float:
    if values.size == 0:
        raise ValueError("Cannot compute RMSE of an empty array")
    return float(np.sqrt(np.mean(np.square(values))))


def compute_relative_error(
    df_a: pd.DataFrame,
    df_b: pd.DataFrame,
    node_a: int,
    node_b: int,
    tolerance: float,
) -> Tuple[np.ndarray, np.ndarray]:
    merged = pd.merge_asof(
        df_a,
        df_b,
        on="t_sec",
        direction="nearest",
        tolerance=tolerance,
        suffixes=(f"_{node_a}", f"_{node_b}"),
    )

    merged = merged.dropna(subset=[f"est_x_m_{node_b}"])
    if merged.empty:
        raise ValueError(f"No overlapping timestamps between nodes {node_a} and {node_b}")

    est_dx = merged[f"est_x_m_{node_a}"] - merged[f"est_x_m_{node_b}"]
    est_dy = merged[f"est_y_m_{node_a}"] - merged[f"est_y_m_{node_b}"]
    est_dz = merged[f"est_z_m_{node_a}"] - merged[f"est_z_m_{node_b}"]
    est_rel = np.sqrt(est_dx**2 + est_dy**2 + est_dz**2)

    gt_dx = merged[f"gt_x_m_{node_a}"] - merged[f"gt_x_m_{node_b}"]
    gt_dy = merged[f"gt_y_m_{node_a}"] - merged[f"gt_y_m_{node_b}"]
    gt_dz = merged[f"gt_z_m_{node_a}"] - merged[f"gt_z_m_{node_b}"]
    gt_rel = np.sqrt(gt_dx**2 + gt_dy**2 + gt_dz**2)

    error = est_rel - gt_rel
    time_hours = merged["t_sec"].to_numpy() / 3600.0
    return time_hours, error.to_numpy()


def plot_relative_distance_errors(
    node_frames: Dict[int, pd.DataFrame],
    tolerance: float,
    output_path: Path,
) -> None:
    pairs = list(PAIRS)
    n_pairs = len(pairs)
    n_cols = 3
    n_rows = (n_pairs + n_cols - 1) // n_cols

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 4 * n_rows), sharex=True)
    axes = np.atleast_1d(axes).ravel()

    for ax, (node_a, node_b) in zip(axes, pairs):
        time_hours, error = compute_relative_error(
            node_frames[node_a], node_frames[node_b], node_a, node_b, tolerance
        )
        rmse = compute_rmse(error)
        ax.plot(time_hours, error, linewidth=0.8)
        ax.set_title(f"Nodes {node_a} - {node_b} (RMSE {rmse:.3f} m)")
        ax.set_ylabel("Relative distance error (m)")
        ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)

    for ax in axes[n_pairs:]:
        ax.axis("off")

    fig.supxlabel("Time (hours)")
    fig.supylabel("Relative distance error (m)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    print(f"Saved figure to {output_path}")


def wrap_angle_deg(values: np.ndarray) -> np.ndarray:
    return (values + 180.0) % 360.0 - 180.0


def compute_position_components(df: pd.DataFrame) -> Dict[str, np.ndarray]:
    return {
        "x": df["est_x_m"].to_numpy() - df["gt_x_m"].to_numpy(),
        "y": df["est_y_m"].to_numpy() - df["gt_y_m"].to_numpy(),
        "z": df["est_z_m"].to_numpy() - df["gt_z_m"].to_numpy(),
    }


def compute_velocity_components(df: pd.DataFrame) -> Dict[str, np.ndarray]:
    return {
        "vx": df["est_vx_mps"].to_numpy() - df["gt_vx_mps"].to_numpy(),
        "vy": df["est_vy_mps"].to_numpy() - df["gt_vy_mps"].to_numpy(),
        "vz": df["est_vz_mps"].to_numpy() - df["gt_vz_mps"].to_numpy(),
    }


def compute_attitude_components(df: pd.DataFrame) -> Dict[str, np.ndarray]:
    return {
        "roll": wrap_angle_deg(
            df["est_roll_deg"].to_numpy() - df["gt_roll_deg"].to_numpy()
        ),
        "pitch": wrap_angle_deg(
            df["est_pitch_deg"].to_numpy() - df["gt_pitch_deg"].to_numpy()
        ),
        "yaw": wrap_angle_deg(
            df["est_yaw_deg"].to_numpy() - df["gt_yaw_deg"].to_numpy()
        ),
    }


def reorganise_components(
    components_by_node: Dict[int, Dict[str, np.ndarray]]
) -> Dict[str, Dict[int, np.ndarray]]:
    organised: Dict[str, Dict[int, np.ndarray]] = {}
    for node, comp_dict in components_by_node.items():
        for label, series in comp_dict.items():
            organised.setdefault(label, {})[node] = series
    return organised


def plot_node_component_figure(
    node_frames: Dict[int, pd.DataFrame],
    values: Dict[int, np.ndarray],
    ylabel: str,
    unit: str,
    output_path: Path,
) -> Dict[int, float]:
    nodes: List[int] = sorted(values.keys())
    n_nodes = len(nodes)
    n_cols = 3
    n_rows = (n_nodes + n_cols - 1) // n_cols

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 4 * n_rows), sharex=False)
    axes = np.atleast_1d(axes).ravel()

    rmses: Dict[int, float] = {}

    for ax, node in zip(axes, nodes):
        time_hours = node_frames[node]["t_sec"].to_numpy() / 3600.0
        series = values[node]
        rmse = compute_rmse(series)
        rmses[node] = rmse

        ax.plot(time_hours, series, linewidth=0.8)
        ax.set_title(f"Node {node} (RMSE {rmse:.3f} {unit})")
        ax.set_ylabel(f"{ylabel} ({unit})")
        ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)

    for ax in axes[n_nodes:]:
        ax.axis("off")

    fig.supxlabel("Time (hours)")
    fig.supylabel(f"{ylabel} ({unit})")
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    print(f"Saved figure to {output_path}")

    return rmses


def main() -> None:
    args = parse_args()

    nodes: Iterable[int] = range(1, 10)
    node_frames = load_all_nodes(args.logs_root, nodes, COMMON_COLUMNS)

    position_components_by_node = {
        node: compute_position_components(df) for node, df in node_frames.items()
    }
    velocity_components_by_node = {
        node: compute_velocity_components(df) for node, df in node_frames.items()
    }
    attitude_components_by_node = {
        node: compute_attitude_components(df) for node, df in node_frames.items()
    }

    position_components = reorganise_components(position_components_by_node)
    velocity_components = reorganise_components(velocity_components_by_node)
    attitude_components = reorganise_components(attitude_components_by_node)

    args.output_dir.mkdir(parents=True, exist_ok=True)

    rel_output = args.output_dir / "relative_distance_errors.png"
    plot_relative_distance_errors(node_frames, args.tolerance, rel_output)

    outputs = {
        ("position", "x"): ("Position error X", "m"),
        ("position", "y"): ("Position error Y", "m"),
        ("position", "z"): ("Position error Z", "m"),
        ("velocity", "vx"): ("Velocity error X", "m/s"),
        ("velocity", "vy"): ("Velocity error Y", "m/s"),
        ("velocity", "vz"): ("Velocity error Z", "m/s"),
        ("attitude", "roll"): ("Attitude error Roll", "deg"),
        ("attitude", "pitch"): ("Attitude error Pitch", "deg"),
        ("attitude", "yaw"): ("Attitude error Yaw", "deg"),
    }

    component_lookup = {
        "position": position_components,
        "velocity": velocity_components,
        "attitude": attitude_components,
    }

    rmse_reports: Dict[str, Dict[int, float]] = {}

    for (group, axis), (ylabel, unit) in outputs.items():
        values = component_lookup[group][axis]
        output_path = args.output_dir / f"{group}_error_{axis}.png"
        rmse_reports[f"{group}_{axis}"] = plot_node_component_figure(
            node_frames, values, ylabel, unit, output_path
        )

    for key, rmses in rmse_reports.items():
        group, axis = key.split("_")
        unit = outputs[(group, axis)][1]
        print(f"RMSE ({group} {axis}):")
        for node in sorted(rmses):
            print(f"  Node {node}: {rmses[node]:.4f} {unit}")
        print(f"  Mean RMSE: {np.mean(list(rmses.values())):.4f} {unit}")

    if args.show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()


