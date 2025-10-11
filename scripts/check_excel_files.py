#!/usr/bin/env python3
"""检查Excel日志文件是否可以被pandas/openpyxl读取。

脚本会遍历指定日志根目录下的Excel文件，尝试读取关键列，
并输出失败文件的列表，同时可选地写入报告文件。
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List, Tuple

import pandas as pd

# 需要检查的列，与plot_relative_distance_errors.py保持一致
COLUMNS: List[str] = [
    "t_sec",
    "est_x_m",
    "est_y_m",
    "est_z_m",
    "gt_x_m",
    "gt_y_m",
    "gt_z_m",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--logs-root",
        type=Path,
        default=Path("logs/excel"),
        help="包含日志Excel文件的根目录",
    )
    parser.add_argument(
        "--pattern",
        default="eca_a9_*/*.xlsx",
        help="在日志根目录下用于glob匹配的相对模式",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("scripts/bad_excel_files.txt"),
        help="将失败文件列表写入指定路径（默认: scripts/bad_excel_files.txt）",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="仅输出摘要信息，不逐条打印失败文件",
    )
    return parser.parse_args()


def check_excels(paths: Iterable[Path]) -> Tuple[int, List[Tuple[Path, str, str]]]:
    failures: List[Tuple[Path, str, str]] = []
    success_count = 0
    for path in sorted(paths):
        try:
            pd.read_excel(path, engine="openpyxl", usecols=COLUMNS, nrows=1)
        except Exception as exc:  # pragma: no cover - 主要用于打印信息
            failures.append((path, exc.__class__.__name__, str(exc)))
        else:
            success_count += 1
    return success_count, failures


def write_report(report_path: Path, failures: List[Tuple[Path, str, str]]) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("w", encoding="utf-8") as fh:
        for path, exc_name, message in failures:
            fh.write(f"{path}\t{exc_name}\t{message}\n")


def main() -> int:
    args = parse_args()

    if not args.logs_root.exists():
        print(f"日志根目录不存在：{args.logs_root}")
        return 2

    matched_files = list(args.logs_root.glob(args.pattern))
    if not matched_files:
        print(
            f"在{args.logs_root}下未匹配到任何文件，模式：{args.pattern}"
        )
        return 3

    success_count, failures = check_excels(matched_files)

    total = len(matched_files)
    fail_count = len(failures)

    if not args.quiet:
        for path, exc_name, message in failures:
            print(f"- {path}: {exc_name}: {message}")

    print("检查完成：")
    print(f"  总文件数：{total}")
    print(f"  成功读取：{success_count}")
    print(f"  失败读取：{fail_count}")

    if args.report:
        write_report(args.report, failures)
        print(f"失败文件列表已写入：{args.report}")

    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())


