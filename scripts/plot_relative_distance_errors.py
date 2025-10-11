"""Plot relative distance errors between AUV nodes from Excel logs.

The script loads estimated and ground-truth positions for each node from the
`logs/excel/eca_a9_<node>` folders, aligns the timelines, computes the
difference between estimated and ground-truth pairwise distances, and writes a
multi-panel figure to disk.

Usage example:
    python scripts/plot_relative_distance_errors.py \
        --logs-root /home/cgz/catkin_ws20250927/logs/excel \
        --output-dir /home/cgz/catkin_ws20250927/logs/plots
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from auv_log_utils import load_node_dataframe


COLUMNS = [
    "t_sec",
    "est_x_m",
    "est_y_m",
    "est_z_m",
    "gt_x_m",
    "gt_y_m",
    "gt_z_m",
]

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--logs-root",
        type=Path,
        default=Path("logs/excel"),
        help="Root directory that contains eca_a9_<node> folders",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("logs/plots"),
        help="Directory where the resulting figure will be saved",
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
        help="Display the figure interactively in addition to saving it",
    )
    return parser.parse_args()



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

    required_columns = [
        f"est_x_m_{node_a}",
        f"est_y_m_{node_a}",
        f"est_z_m_{node_a}",
        f"gt_x_m_{node_a}",
        f"gt_y_m_{node_a}",
        f"gt_z_m_{node_a}",
        f"est_x_m_{node_b}",
        f"est_y_m_{node_b}",
        f"est_z_m_{node_b}",
        f"gt_x_m_{node_b}",
        f"gt_y_m_{node_b}",
        f"gt_z_m_{node_b}",
    ]

    merged = merged.dropna(subset=required_columns)
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
    error_values = error.to_numpy()
    time_hours = merged["t_sec"].to_numpy() / 3600.0
    finite_mask = np.isfinite(error_values) & np.isfinite(time_hours)
    if not finite_mask.any():
        raise ValueError(
            f"No finite relative distance errors between nodes {node_a} and {node_b}"
        )

    return time_hours[finite_mask], error_values[finite_mask]


def plot_errors(
    node_frames: Dict[int, pd.DataFrame],
    pairs: Iterable[Tuple[int, int]],
    tolerance: float,
) -> plt.Figure:
    def annotate_skip(ax: plt.Axes, message: str) -> None:
        ax.text(0.5, 0.5, message, ha="center", va="center", transform=ax.transAxes)
        ax.axis("off")

    pairs = list(pairs)
    n_pairs = len(pairs)
    n_cols = 3
    n_rows = (n_pairs + n_cols - 1) // n_cols if n_pairs else 1

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 4 * n_rows), sharex=True)
    axes = np.atleast_1d(axes).ravel()

    plotted_count = 0
    skipped_details: List[str] = []

    for ax, (node_a, node_b) in zip(axes, pairs):
        if node_a not in node_frames or node_b not in node_frames:
            reason = f"Skipping nodes {node_a}-{node_b}: missing data"
            skipped_details.append(reason)
            annotate_skip(ax, "数据缺失，已跳过")
            continue

        try:
            time_hours, error = compute_relative_error(
                node_frames[node_a], node_frames[node_b], node_a, node_b, tolerance
            )
        except Exception as exc:  # pragma: no cover - logging only
            reason = f"Skipping nodes {node_a}-{node_b}: {exc}"
            skipped_details.append(reason)
            annotate_skip(ax, "异常数据，已跳过")
            continue

        if time_hours.size == 0:
            reason = f"Skipping nodes {node_a}-{node_b}: empty after filtering"
            skipped_details.append(reason)
            annotate_skip(ax, "空数据，已跳过")
            continue

        ax.plot(time_hours, error, linewidth=0.8)
        ax.set_title(f"Nodes {node_a} - {node_b}")
        ax.set_ylabel("Error (m)")
        ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
        plotted_count += 1

    for ax in axes[n_pairs:]:
        ax.axis("off")

    if skipped_details:
        for entry in skipped_details:
            print(entry)

    if plotted_count == 0:
        fig.suptitle("未找到可绘制的相对误差数据")

    fig.supxlabel("Time (hours)")
    fig.supylabel("Relative distance error (m)")
    fig.tight_layout()
    return fig


def main() -> None:
    args = parse_args()

    nodes = sorted({node for pair in PAIRS for node in pair})
    node_frames: Dict[int, pd.DataFrame] = {}
    for node in nodes:
        try:
            node_frames[node] = load_node_dataframe(node, args.logs_root, COLUMNS)
        except Exception as exc:  # pragma: no cover - logging only
            print(f"Warning: skipping node {node}: {exc}")

    if not node_frames:
        print("Error: no nodes loaded successfully; aborting plot generation.")
        return

    available_pairs = [
        (node_a, node_b)
        for node_a, node_b in PAIRS
        if node_a in node_frames and node_b in node_frames
    ]

    if not available_pairs:
        print("Error: no valid node pairs available for plotting.")
        return

    figure = plot_errors(node_frames, available_pairs, args.tolerance)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.output_dir / "relative_distance_errors.png"
    figure.savefig(output_path, dpi=300)
    print(f"Saved figure to {output_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()

