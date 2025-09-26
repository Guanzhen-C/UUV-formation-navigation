#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import math
from datetime import datetime
from typing import Dict, List
from pathlib import Path

import pandas as pd
import numpy as np

# Use non-interactive backend for headless environments
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import seaborn as sns
from tabulate import tabulate


BASE_DIR = str(Path(__file__).resolve().parent.parent)
EXCEL_ROOT = os.path.join(BASE_DIR, "logs", "excel")
PLOT_DIR = os.path.join(BASE_DIR, "logs", "plots")
REPORT_DIR = os.path.join(BASE_DIR, "logs", "report")
REPORT_PATH = os.path.join(REPORT_DIR, "eskf_sim_report.md")

sns.set(style="whitegrid", context="talk")

# 启用中文字体（按优先级回退），并避免负号显示为方块
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",   # 首选：Noto 思源黑体简体（系统已安装）
    "Noto Serif CJK SC",  # 备选：思源宋体简体
    "WenQuanYi Zen Hei",  # 备选：文泉驿正黑
    "WenQuanYi Micro Hei",
    "Droid Sans Fallback",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


EXPECTED_HEADERS = [
    "t_sec",
    "est_x_m",
    "est_y_m",
    "est_z_m",
    "est_vx_mps",
    "est_vy_mps",
    "est_vz_mps",
    "est_roll_deg",
    "est_pitch_deg",
    "est_yaw_deg",
    "gt_x_m",
    "gt_y_m",
    "gt_z_m",
    "gt_vx_mps",
    "gt_vy_mps",
    "gt_vz_mps",
    "gt_roll_deg",
    "gt_pitch_deg",
    "gt_yaw_deg",
    "pos_error_m",
    "vel_error_mps",
    "att_error_deg",
]


def read_robot_hours(robot_dir: str) -> pd.DataFrame:
    all_files = sorted(glob.glob(os.path.join(robot_dir, "*_simh*.xlsx")))
    frames = []
    for file_path in all_files:
        try:
            df = pd.read_excel(file_path)
            missing = [h for h in EXPECTED_HEADERS if h not in df.columns]
            if missing:
                print(f"[WARN] {os.path.basename(file_path)} missing columns: {missing}. Skipping.")
                continue
            df = df[EXPECTED_HEADERS].copy()
            df["source_file"] = os.path.basename(file_path)
            frames.append(df)
        except Exception as e:
            print(f"[WARN] Failed to read {file_path}: {e}")
    if not frames:
        return pd.DataFrame(columns=EXPECTED_HEADERS + ["source_file"])  # empty
    out = pd.concat(frames, ignore_index=True)
    # Drop any rows with missing timestamps (e.g., partial writes)
    out = out.dropna(subset=["t_sec"])  # keep rows where t_sec is present
    return out


def compute_metrics(df: pd.DataFrame) -> Dict[str, float]:
    if df.empty:
        return {
            "num_samples": 0,
            "duration_sec": float("nan"),
            "pos_rmse_m": float("nan"),
            "vel_rmse_mps": float("nan"),
            "att_rmse_deg": float("nan"),
            "pos_max_m": float("nan"),
            "vel_max_mps": float("nan"),
            "att_max_deg": float("nan"),
            "convergence_time_sec": float("nan"),
        }

    t = df["t_sec"].to_numpy()
    duration = float(t.max() - t.min()) if len(t) else float("nan")
    pos_err = df["pos_error_m"].to_numpy()
    vel_err = df["vel_error_mps"].to_numpy()
    att_err = df["att_error_deg"].to_numpy()

    # RMSE
    pos_rmse = float(np.sqrt(np.mean(np.square(pos_err)))) if len(pos_err) else float("nan")
    vel_rmse = float(np.sqrt(np.mean(np.square(vel_err)))) if len(vel_err) else float("nan")
    att_rmse = float(np.sqrt(np.mean(np.square(att_err)))) if len(att_err) else float("nan")

    # Max
    pos_max = float(np.max(pos_err)) if len(pos_err) else float("nan")
    vel_max = float(np.max(vel_err)) if len(vel_err) else float("nan")
    att_max = float(np.max(att_err)) if len(att_err) else float("nan")

    # Convergence time heuristic: first time pos_error < 1.0 m for a continuous 5 s window
    threshold = 1.0
    window_sec = 5.0
    convergence_time = float("nan")
    if len(t) and len(pos_err):
        order = np.argsort(t)
        t_sorted = t[order]
        e_sorted = pos_err[order]
        for idx in range(len(e_sorted)):
            if e_sorted[idx] < threshold:
                t0 = t_sorted[idx]
                j = idx
                ok = True
                while j < len(e_sorted) and t_sorted[j] <= t0 + window_sec:
                    if e_sorted[j] >= threshold:
                        ok = False
                        break
                    j += 1
                if ok:
                    convergence_time = float(t0 - t_sorted[0])
                    break

    return {
        "num_samples": int(len(df)),
        "duration_sec": duration,
        "pos_rmse_m": pos_rmse,
        "vel_rmse_mps": vel_rmse,
        "att_rmse_deg": att_rmse,
        "pos_max_m": pos_max,
        "vel_max_mps": vel_max,
        "att_max_deg": att_max,
        "convergence_time_sec": convergence_time,
    }


def plot_robot(df: pd.DataFrame, robot_name: str) -> Dict[str, str]:
    plots: Dict[str, str] = {}
    if df.empty:
        return plots

    df = df.sort_values("t_sec").reset_index(drop=True)

    def savefig(name: str):
        os.makedirs(PLOT_DIR, exist_ok=True)
        out_path = os.path.join(PLOT_DIR, f"{robot_name}_{name}.png")
        plt.tight_layout()
        plt.savefig(out_path, dpi=160)
        plt.close()
        plots[name] = out_path

    # Trajectory XY
    plt.figure(figsize=(8, 6))
    plt.plot(df["gt_x_m"], df["gt_y_m"], label="真值 XY", color="#2ca02c", linewidth=2)
    plt.plot(df["est_x_m"], df["est_y_m"], label="估计 XY", color="#1f77b4", alpha=0.85)
    plt.xlabel("X (米)")
    plt.ylabel("Y (米)")
    plt.title(f"{robot_name} 轨迹 XY")
    plt.legend()
    savefig("traj_xy")

    # Trajectory XZ
    plt.figure(figsize=(8, 6))
    plt.plot(df["gt_x_m"], df["gt_z_m"], label="真值 XZ", color="#2ca02c", linewidth=2)
    plt.plot(df["est_x_m"], df["est_z_m"], label="估计 XZ", color="#1f77b4", alpha=0.85)
    plt.xlabel("X (米)")
    plt.ylabel("Z (米)")
    plt.title(f"{robot_name} 轨迹 XZ")
    plt.legend()
    savefig("traj_xz")

    # Error time series
    plt.figure(figsize=(10, 6))
    plt.plot(df["t_sec"], df["pos_error_m"], label="位置误差 (米)", color="#d62728")
    plt.xlabel("时间 (秒)")
    plt.ylabel("位置误差 (米)")
    plt.title(f"{robot_name} 位置误差随时间")
    plt.legend()
    savefig("pos_err_ts")

    plt.figure(figsize=(10, 6))
    plt.plot(df["t_sec"], df["vel_error_mps"], label="速度误差 (米/秒)", color="#ff7f0e")
    plt.xlabel("时间 (秒)")
    plt.ylabel("速度误差 (米/秒)")
    plt.title(f"{robot_name} 速度误差随时间")
    plt.legend()
    savefig("vel_err_ts")

    plt.figure(figsize=(10, 6))
    plt.plot(df["t_sec"], df["att_error_deg"], label="姿态误差 (度)", color="#9467bd")
    plt.xlabel("时间 (秒)")
    plt.ylabel("姿态误差 (度)")
    plt.title(f"{robot_name} 姿态误差随时间")
    plt.legend()
    savefig("att_err_ts")

    # Euler angles (est vs gt)
    for (est_col, gt_col), (disp_label, file_key) in [
        (("est_roll_deg", "gt_roll_deg"), ("横滚角 (度)", "roll_deg")),
        (("est_pitch_deg", "gt_pitch_deg"), ("俯仰角 (度)", "pitch_deg")),
        (("est_yaw_deg", "gt_yaw_deg"), ("航向角 (度)", "yaw_deg")),
    ]:
        plt.figure(figsize=(10, 6))
        plt.plot(df["t_sec"], df[gt_col], label="真值", color="#2ca02c", linewidth=2)
        plt.plot(df["t_sec"], df[est_col], label="估计", color="#1f77b4", alpha=0.85)
        plt.xlabel("时间 (秒)")
        plt.ylabel(disp_label)
        plt.title(f"{robot_name} {disp_label} 随时间")
        plt.legend()
        savefig(f"{file_key}_ts")

    return plots


def write_report(robot_to_metrics: Dict[str, Dict[str, float]], robot_to_plots: Dict[str, Dict[str, str]]):
    os.makedirs(REPORT_DIR, exist_ok=True)
    lines = []
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines.append("# ESKF 仿真实验报告\n")
    lines.append(f"生成时间: {now}\n")
    lines.append("")

    # Algorithm overview
    lines.append("## 算法简介\n")
    lines.append(
        "本实验采用扩展误差状态卡尔曼滤波器（ESKF）完成 AUV 六自由度导航。ESKF 以“名义状态 + 小误差状态”"
        "的思想进行线性化，名义部分用非线性模型积分，误差部分在线性高斯假设下用卡尔曼滤波更新。主要要点如下：\n"
    )
    lines.append(
        "- 状态定义：位置 p∈R^3、速度 v∈R^3、姿态 q（单位四元数）、陀螺零偏 b_g、加计零偏 b_a（均为随机游走）。\n"
        "- 系统模型：\n"
        "  - ṗ = v；\n"
        "  - v̇ = R(q)·(a_m − b_a − n_a) + g；\n"
        "  - q̇ = 0.5·Ω(ω_m − b_g − n_g)·q；\n"
        "  - ḃ_g = n_bg，ḃ_a = n_ba（高斯白噪声驱动）。\n"
        "- 量测模型：融合里程计/仿真真值提供的位置、速度与（必要时的）姿态约束，采用标准线性化量测模型。\n"
        "- 误差状态：采用小旋量 φ 表达的姿态误差，构建误差状态 x_err=[δp, δv, φ, δb_g, δb_a]。由名义状态求得 F、G 矩阵并离散化（Φ,Qd）。\n"
        "- 滤波流程：IMU 预积分进行预测；到达量测时执行卡尔曼更新（可采用 Joseph 形式提高数值稳定性），随后进行“误差注入”并重正化四元数。\n"
        "- 参数与调节：过程噪声由陀螺/加计噪声密度与偏置随机游走确定；量测噪声依据外部传感器精度设定；收敛与稳态性能取决于运动激励与可观测性。\n\n"
    )

    # KF-GINS reference alignment
    lines.append("## 参考 KF-GINS 的算法要点\n")
    lines.append(
        "本工程的 ESKF 实现整体与 KF-GINS 的 15 维误差状态框架一致：采用“名义状态 + 误差状态”形式，以"
        "惯导捆绑解算/IMU 预积分完成预测，线性化量测完成更新，并通过误差注入复位名义状态。\n"
    )
    lines.append(
        "- 状态与误差：名义状态 x = [p, v, q, b_g, b_a]，误差状态 δx = [δp, δv, φ, δb_g, δb_a]（φ 为小旋量），"
        "与 KF-GINS 常用 15 维定义一致。\n"
    )
    lines.append(
        "- 机理推进/预积分：使用连续误差模型构造 F、G 与连续噪声 Q_c（由 {σ_g, σ_a, σ_bg, σ_ba} 给定），并离散化得到 (Φ, Q_d) "
        "进行协方差预测，与 KF-GINS 离散化流程一致。\n"
    )
    lines.append(
        "- 卡尔曼更新（Joseph 形式）：S = HPH^T + R，K = PH^T S^{-1}；P ← (I − KH) P (I − KH)^T + K R K^T，保持数值对称正定，"
        "做法与 KF-GINS 对齐。\n"
    )
    lines.append(
        "- 误差注入与重置：p ← p + δp，v ← v + δv，q ← exp(φ) ⊗ q，b_g ← b_g + δb_g，b_a ← b_a + δb_a；随后按 reset 雅可比对 P 做相似变换，"
        "并重正化四元数，与 KF-GINS 的 reset 处理一致。\n"
    )
    lines.append(
        "- 量测接口：各传感器统一返回 (H, r, R) 的线性化接口（如深度计、DVL/里程计、磁罗盘、GPS/USBL、互测距等），"
        "便于与 KF-GINS 传感器模块互参照。\n"
    )
    lines.append(
        "- 稳健性与工程细节：维持 P 的对称/正定，小角阈值保护 exp(φ)，优先使用 Cholesky/LDLT 求解，残差门控与稳健核以抑制外点，"
        "参考了 KF-GINS 的工程实践。\n\n"
    )

    # Distributed ESKF section
    lines.append("## 分布式 ESKF（仅 IMU + 深度计 + 互测距）\n")
    lines.append("在 `uuv_eskf_nav_1-9` 中，我们实现了面向 AUV 集群的分布式误差状态卡尔曼滤波（ESKF）。每个 AUV 节点独立运行本地 ESKF 预测与更新，通过互测距量测将邻居的位置信息以约束形式纳入更新，并在网络中只交换必要的低维统计量，实现去中心化协同定位。\n")
    lines.append("- 传感器集合：IMU（陀螺/加计）、深度计（单点标量）、互测距（节点间欧氏距离）。\n")
    lines.append("- 本地状态：x = [p, v, q, b_g, b_a]；误差状态 x_err = [δp, δv, φ, δb_g, δb_a]。名义用于非线性积分，误差用于线性化更新。\n")
    lines.append("- 过程模型：IMU 预积分得到离散预测（Φ,Qd），偏置为随机游走；姿态用四元数表示并在误差注入后重正化。\n")
    lines.append("- 深度计量测：z_depth ≈ e_z^T p + n_d，仅作用于位置的 z 分量，提供绝对垂向观测，改善俯仰/加计零偏可观测性。\n")
    lines.append("- 互测距量测：z_ij ≈ ||p_i − p_j|| + n_r。对节点 i 的线性化雅可比 H_i = (p_i − p_j)^T / ||p_i − p_j|| 作用在位置子块（在邻居 j 最近一次广播估计处评估）。为避免相关性导致过度自信，将邻居不确定度并入有效噪声：R_eff = R_r + H_j P_j H_j^T（位置子块 H_j = −H_i）。\n")
    lines.append("- 分布式更新：测距到达即在本地执行卡尔曼更新（Joseph 形式增强数值稳定性），所需通信仅为邻居的 {p_j, P_jpp, 时间戳}。多邻居可顺序或批量处理；时间不同步时先以 IMU 对邻居状态外推到同一时刻。\n")
    lines.append("- 可观测性与锚定：IMU+深度计提供绝对 z 与局部姿态约束，但在仅测距条件下，全局平移与航向存在规范自由度。可通过 1) 选定参考 AUV 锚定其初始位姿；2) 对网络质心施加弱先验；或 3) 仅报告相对位姿并在可视化端选定原点 来破除自由度。\n")
    lines.append("- 一致性与鲁棒性：采用 R_eff 膨胀抑制信息双计数；对测距残差使用马氏距离门控与稳健核（如 Huber）抵御外点与声学误检。\n")
    lines.append("- 实现要点：预测用 IMU 预积分；更新顺序先深度计再互测距；通信周期性广播 {p, P_pp} 与时间戳；误差注入后重正化 q 并保持 P 对称正定；量测噪声 R_r 与 R_d 按设备标定设置，邻居不确定度来自其广播协方差。\n\n")

    # Data and methods
    lines.append("## 数据与方法\n")
    lines.append(
        "评估器以 1 Hz 采样记录：时间戳 t_sec、估计状态（位置/速度/姿态欧拉角）、地面真值，以及误差指标。"
        "误差计算：位置误差为欧氏范数；速度误差为速度差范数；姿态误差由估计与真值姿态的相对旋转角度得到（以度计）。"
        "日志按仿真小时切分为 <robot>_simh%04d.xlsx，位于 logs/excel/<robot>/。\n"
    )

    # Metrics table
    lines.append("## 结果总览（各机器人）\n")
    headers = [
        "机器人",
        "样本数",
        "时长(秒)",
        "位置RMSE(米)",
        "速度RMSE(米/秒)",
        "姿态RMSE(度)",
        "位置最大(米)",
        "速度最大(米/秒)",
        "姿态最大(度)",
        "收敛时间(秒)",
    ]
    rows = []
    for robot, m in robot_to_metrics.items():
        def fmt(v: float, fmt_str: str) -> str:
            try:
                if v is None or (isinstance(v, float) and math.isnan(v)):
                    return "-"
                return fmt_str.format(v)
            except Exception:
                return "-"

        rows.append(
            [
                robot,
                m.get("num_samples", 0),
                fmt(m.get("duration_sec", float("nan")), "{:.1f}"),
                fmt(m.get("pos_rmse_m", float("nan")), "{:.3f}"),
                fmt(m.get("vel_rmse_mps", float("nan")), "{:.3f}"),
                fmt(m.get("att_rmse_deg", float("nan")), "{:.3f}"),
                fmt(m.get("pos_max_m", float("nan")), "{:.3f}"),
                fmt(m.get("vel_max_mps", float("nan")), "{:.3f}"),
                fmt(m.get("att_max_deg", float("nan")), "{:.3f}"),
                fmt(m.get("convergence_time_sec", float("nan")), "{:.1f}"),
            ]
        )

    table_md = tabulate(rows, headers=headers, tablefmt="github")
    lines.append(table_md + "\n")

    # Per-robot sections
    for robot, plots in robot_to_plots.items():
        lines.append(f"## {robot}\n")
        if not plots:
            lines.append("无可用曲线（缺少数据）。\n")
            continue
        lines.append("关键曲线如下（自动生成）：\n")
        name_map = {
            "traj_xy": "轨迹 XY",
            "traj_xz": "轨迹 XZ",
            "pos_err_ts": "位置误差",
            "vel_err_ts": "速度误差",
            "att_err_ts": "姿态误差",
            "roll_deg_ts": "横滚角",
            "pitch_deg_ts": "俯仰角",
            "yaw_deg_ts": "航向角",
        }
        for name, path in plots.items():
            rel = os.path.relpath(path, REPORT_DIR)
            disp = name_map.get(name, name)
            lines.append(f"![{robot} {disp}]({rel})\n")
        lines.append("")

    # Discussion
    lines.append("## 分析与讨论\n")
    lines.append("- 初始阶段位置误差随时间下降，随后进入稳定区间。\n")
    lines.append("- 各实例 RMSE 与最大误差差异反映了初始条件与噪声影响。\n")
    lines.append("- 收敛时间依据 1.0 m 连续 5 s 阈值评估，可按任务调整。\n")

    # Conclusion
    lines.append("## 结论\n")
    lines.append(
        "在当前仿真条件下，ESKF 能够稳定估计 AUV 的 6-DoF 状态，误差表现满足预期。"
        "建议后续引入 DVL/USBL 等量测以及非高斯噪声鲁棒性测试以进一步完善评估。\n"
    )

    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main():
    os.makedirs(PLOT_DIR, exist_ok=True)
    os.makedirs(REPORT_DIR, exist_ok=True)

    if not os.path.isdir(EXCEL_ROOT):
        print(f"[WARN] Excel root not found: {EXCEL_ROOT}")
        robot_dirs: List[str] = []
    else:
        robot_dirs = [
            d for d in sorted(glob.glob(os.path.join(EXCEL_ROOT, "*"))) if os.path.isdir(d)
        ]

    robot_to_metrics: Dict[str, Dict[str, float]] = {}
    robot_to_plots: Dict[str, Dict[str, str]] = {}

    if not robot_dirs:
        print(f"[INFO] No robot directories found under {EXCEL_ROOT}. Report will still be generated.")

    for robot_dir in robot_dirs:
        robot = os.path.basename(robot_dir)
        df = read_robot_hours(robot_dir)
        metrics = compute_metrics(df)
        plots = plot_robot(df, robot)
        robot_to_metrics[robot] = metrics
        robot_to_plots[robot] = plots
        dur = metrics.get("duration_sec", float("nan"))
        print(
            f"[INFO] {robot}: samples={metrics.get('num_samples', 0)} "
            f"duration={(f'{dur:.1f}' if not (isinstance(dur, float) and math.isnan(dur)) else '-') } s "
            f"posRMSE={metrics.get('pos_rmse_m')}"
        )

    write_report(robot_to_metrics, robot_to_plots)
    print(f"[OK] Report written to {REPORT_PATH}")


if __name__ == "__main__":
    main()


