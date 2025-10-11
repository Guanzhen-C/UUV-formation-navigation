"""Utility helpers for loading AUV Excel logs.

These helpers centralise the logic for reading the per-node Excel files,
handling missing directories, skipping unreadable files, and returning tidy
`pandas` DataFrames sorted by timestamp.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import pandas as pd


def _normalise_columns(columns: Sequence[str]) -> List[str]:
    unique = list(dict.fromkeys(columns))
    if "t_sec" not in unique:
        raise ValueError("'t_sec' column must be requested when loading logs")
    return unique


def load_node_dataframe(
    node: int,
    base_dir: Path,
    columns: Sequence[str],
) -> pd.DataFrame:
    """Load and concatenate all Excel logs for a single node.

    Parameters
    ----------
    node:
        Node identifier (1-based).
    base_dir:
        Root directory that contains `eca_a9_<node>` folders.
    columns:
        Columns that need to be present in the resulting DataFrame. The
        function will instruct `pandas.read_excel` to load exactly this set of
        columns.

    Returns
    -------
    pandas.DataFrame
        Concatenated, de-duplicated and time-sorted dataframe containing the
        requested columns.
    """

    requested_columns = _normalise_columns(columns)
    node_dir = base_dir / f"eca_a9_{node}"
    if not node_dir.exists():
        raise FileNotFoundError(f"Directory not found for node {node}: {node_dir}")

    frames: List[pd.DataFrame] = []
    for path in sorted(node_dir.glob("*.xlsx")):
        try:
            frames.append(
                pd.read_excel(path, engine="openpyxl", usecols=requested_columns)
            )
        except Exception as exc:  # pragma: no cover - logging only
            print(f"Warning: failed to read {path}: {exc}")
            continue

    if not frames:
        raise ValueError(f"No readable Excel files found for node {node} in {node_dir}")

    data = pd.concat(frames, ignore_index=True)
    data = data.drop_duplicates(subset="t_sec").sort_values("t_sec")
    return data.reset_index(drop=True)


def load_all_nodes(
    base_dir: Path,
    nodes: Iterable[int],
    columns: Sequence[str],
) -> Dict[int, pd.DataFrame]:
    """Load dataframes for all requested nodes."""

    return {node: load_node_dataframe(node, base_dir, columns) for node in nodes}


__all__ = ["load_node_dataframe", "load_all_nodes"]


