#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Convert navigation evaluation rosbag logs to Excel files."""

import argparse
import sys
from pathlib import Path

import rosbag
import xlsxwriter


HEADERS = [
    't_sec',
    'est_x_m', 'est_y_m', 'est_z_m',
    'est_vx_mps', 'est_vy_mps', 'est_vz_mps',
    'est_roll_deg', 'est_pitch_deg', 'est_yaw_deg',
    'gt_x_m', 'gt_y_m', 'gt_z_m',
    'gt_vx_mps', 'gt_vy_mps', 'gt_vz_mps',
    'gt_roll_deg', 'gt_pitch_deg', 'gt_yaw_deg',
    'pos_error_m', 'vel_error_mps', 'att_error_deg'
]


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert navigation evaluation bag files into Excel logs.'
    )
    default_root = Path.home() / 'catkin_ws'
    parser.add_argument(
        '--bag-root',
        default=str(default_root / 'logs' / 'bag'),
        help='Directory containing bag files (default: %(default)s)'
    )
    parser.add_argument(
        '--excel-root',
        default=str(default_root / 'logs' / 'excel'),
        help='Directory to place generated Excel files (default: %(default)s)'
    )
    parser.add_argument(
        '--topic',
        default='/eskf/navigation_eval_data',
        help='Topic name used for evaluation records (default: %(default)s)'
    )
    return parser.parse_args()


def ensure_directory(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def read_bag_records(bag_path: Path, topic: str):
    rows = []
    robot_label = None

    try:
        with rosbag.Bag(str(bag_path), 'r') as bag:
            for _, msg, _ in bag.read_messages(topics=[topic]):
                data = list(msg.data)
                if not data:
                    continue
                if len(data) != len(HEADERS):
                    print(f'[WARN] {bag_path}: 数据长度{len(data)}不等于{len(HEADERS)}，忽略该条记录。', file=sys.stderr)
                    continue
                rows.append(data)
                if msg.layout.dim and msg.layout.dim[0].label:
                    robot_label = msg.layout.dim[0].label
    except rosbag.bag.ROSBagException as exc:
        print(f'[ERROR] 打开bag失败 {bag_path}: {exc}', file=sys.stderr)
        return [], None

    return rows, robot_label


def write_excel(records, excel_path: Path):
    ensure_directory(excel_path.parent)
    workbook = xlsxwriter.Workbook(str(excel_path))
    worksheet = workbook.add_worksheet('Data')

    for col, header in enumerate(HEADERS):
        worksheet.write(0, col, header)

    for row_idx, record in enumerate(records, start=1):
        for col_idx, value in enumerate(record):
            worksheet.write(row_idx, col_idx, value)

    workbook.close()


def convert_bag_to_excel(bag_path: Path, bag_root: Path, excel_root: Path, topic: str):
    try:
        relative = bag_path.relative_to(bag_root)
    except ValueError:
        relative = bag_path.name

    if not isinstance(relative, Path):
        relative = Path(relative)

    excel_path = (excel_root / relative).with_suffix('.xlsx')

    records, robot_label = read_bag_records(bag_path, topic)
    if not records:
        print(f'[WARN] {bag_path}: 未找到任何评估记录，跳过。', file=sys.stderr)
        return

    write_excel(records, excel_path)
    if robot_label:
        print(f'[INFO] 已生成Excel: {excel_path} (robot={robot_label})')
    else:
        print(f'[INFO] 已生成Excel: {excel_path}')


def main():
    args = parse_args()
    bag_root = Path(args.bag_root).expanduser()
    excel_root = Path(args.excel_root).expanduser()

    if not bag_root.exists():
        print(f'[ERROR] bag目录不存在: {bag_root}', file=sys.stderr)
        sys.exit(1)

    bag_files = sorted(bag_root.glob('**/*.bag'))
    if not bag_files:
        print('[WARN] 未找到任何bag文件，无需转换。', file=sys.stderr)
        return

    for bag_path in bag_files:
        convert_bag_to_excel(bag_path, bag_root, excel_root, args.topic)


if __name__ == '__main__':
    main()

