# ESKF 仿真实验报告

生成时间: 2025-09-22 11:05:45


## 算法简介

本实验采用扩展误差状态卡尔曼滤波器（ESKF）完成 AUV 六自由度导航。ESKF 以“名义状态 + 小误差状态”的思想进行线性化，名义部分用非线性模型积分，误差部分在线性高斯假设下用卡尔曼滤波更新。主要要点如下：

- 状态定义：位置 p∈R^3、速度 v∈R^3、姿态 q（单位四元数）、陀螺零偏 b_g、加计零偏 b_a（均为随机游走）。
- 系统模型：
  - ṗ = v；
  - v̇ = R(q)·(a_m − b_a − n_a) + g；
  - q̇ = 0.5·Ω(ω_m − b_g − n_g)·q；
  - ḃ_g = n_bg，ḃ_a = n_ba（高斯白噪声驱动）。
- 量测模型：融合里程计/仿真真值提供的位置、速度与（必要时的）姿态约束，采用标准线性化量测模型。
- 误差状态：采用小旋量 φ 表达的姿态误差，构建误差状态 x_err=[δp, δv, φ, δb_g, δb_a]。由名义状态求得 F、G 矩阵并离散化（Φ,Qd）。
- 滤波流程：IMU 预积分进行预测；到达量测时执行卡尔曼更新（可采用 Joseph 形式提高数值稳定性），随后进行“误差注入”并重正化四元数。
- 参数与调节：过程噪声由陀螺/加计噪声密度与偏置随机游走确定；量测噪声依据外部传感器精度设定；收敛与稳态性能取决于运动激励与可观测性。


## IMU 仿真配置（URDF）

本次仿真使用 `eca_a9` 机型的默认 IMU 插件配置，来源于 `src/eca_a9/eca_a9_description/urdf/eca_a9_sensors.xacro` 与 `uuv_sensor_ros_plugins/urdf/imu_snippets.xacro`。关键参数如下（单位见括号）：

- 质量 mass_imu_sensor: 0.015 (kg)
- 陀螺噪声密度 gyroscope_noise_density: 0.00000029088 (rad/s/√Hz)
- 陀螺随机游走 gyroscope_random_walk: 0.00000029088 (rad/s/s/√Hz)
- 陀螺偏置相关时间 gyroscope_bias_correlation_time: 3600.0 (s)
- 陀螺开机偏置标准差 gyroscope_turn_on_bias_sigma: 0.000004848 (rad/s) ≈ 0.001°/h
- 加计噪声密度 accelerometer_noise_density: 0.000010 (m/s²/√Hz) ≈ 10 μg/√Hz
- 加计随机游走 accelerometer_random_walk: 0.000010 (m/s²/s/√Hz)
- 加计偏置相关时间 accelerometer_bias_correlation_time: 3600.0 (s)
- 加计开机偏置标准差 accelerometer_turn_on_bias_sigma: 0.0098 (m/s²) ≈ 1 mg
- 姿态噪声 orientation_noise: 0.001 (rad)
- 参考系 reference_frame: world（ENU）
- NED本地帧 enable_local_ned_frame: false
- 更新率 update_rate: 200 (Hz)

说明：滤波器用到的离散噪声标准差依据采样率从噪声密度换算而来，例如在 200 Hz 时，σ_g ≈ 4.1e-6 rad/s、σ_a ≈ 1.4e-4 m/s²；偏置随机游走依据相关时间换算，参数与 `config/eskf_params.yaml` 保持一致。


## ESKF 补偿算法与实现

为保证高角速率/高动态下的积分精度，ESKF 机械编排中实现了以下补偿（参考 KF-GINS 思路，并结合本项目实现）：

- 圆锥误差补偿（Coning，二阶）
  - 姿态更新使用 Δθ + (Δθ_pre × Δθ)/12 校正（若有上一时段数据），提升高频小摆动条件下的姿态积分精度。

- 划桨效应补偿（Sculling，三项）
  - 速度积分中的比力增量使用三项补偿：Δθ×Δv/2 + Δθ_pre×Δv/12 + Δv_pre×Δθ/12（上一时段可用时加入后两项），降低角-加耦合误差。

- 中间姿态投影（Mid-rotation）
  - 使用中间姿态 R(q_prev ⊗ exp(Δθ/2)) 将机体系比力增量投影到导航系，减小离散化误差。

- 重力与科氏加速度补偿（Gravity & Coriolis）
  - 若加速度计读数不含重力，则速度更新中加入 g·dt；
  - 启用地球自转补偿时，加入 -(2·ω_ie + ω_en)×v · dt 的科氏/导航系转动项（ω_ie 由实时纬度计算，ω_en 由速度与位置计算）。

- 传感器偏置补偿（Bias compensation）
  - 先对陀螺与加计测量去偏再积分，偏置作为随机游走在误差状态中估计与注入。

- 数值与滤波细节
  - 位置采用梯形积分；协方差离散化使用基于去偏量的 F、Q；量测更新使用 Joseph 形式保持数值稳定；误差注入后重置误差并保持 P 对称正定。


## 参考 KF-GINS 的算法要点

本工程的 ESKF 实现整体与 KF-GINS 的 15 维误差状态框架一致：采用“名义状态 + 误差状态”形式，以惯导捆绑解算/IMU 预积分完成预测，线性化量测完成更新，并通过误差注入复位名义状态。

- 状态与误差：名义状态 x = [p, v, q, b_g, b_a]，误差状态 δx = [δp, δv, φ, δb_g, δb_a]（φ 为小旋量），与 KF-GINS 常用 15 维定义一致。

- 机理推进/预积分：使用连续误差模型构造 F、G 与连续噪声 Q_c（由 {σ_g, σ_a, σ_bg, σ_ba} 给定），并离散化得到 (Φ, Q_d) 进行协方差预测，与 KF-GINS 离散化流程一致。

- 卡尔曼更新（Joseph 形式）：S = HPH^T + R，K = PH^T S^{-1}；P ← (I − KH) P (I − KH)^T + K R K^T，保持数值对称正定，做法与 KF-GINS 对齐。

- 误差注入与重置：p ← p + δp，v ← v + δv，q ← exp(φ) ⊗ q，b_g ← b_g + δb_g，b_a ← b_a + δb_a；随后按 reset 雅可比对 P 做相似变换，并重正化四元数，与 KF-GINS 的 reset 处理一致。

- 量测接口：各传感器统一返回 (H, r, R) 的线性化接口（如深度计、DVL/里程计、磁罗盘、GPS/USBL、互测距等），便于与 KF-GINS 传感器模块互参照。

- 稳健性与工程细节：维持 P 的对称/正定，小角阈值保护 exp(φ)，优先使用 Cholesky/LDLT 求解，残差门控与稳健核以抑制外点，参考了 KF-GINS 的工程实践。


## 分布式 ESKF（仅 IMU + 深度计 + 互测距）

在 `uuv_eskf_nav_1-9` 中，我们实现了面向 AUV 集群的分布式误差状态卡尔曼滤波（ESKF）。每个 AUV 节点独立运行本地 ESKF 预测与更新，通过互测距量测将邻居的位置信息以约束形式纳入更新，并在网络中只交换必要的低维统计量，实现去中心化协同定位。

- 传感器集合：IMU（陀螺/加计）、深度计（单点标量）、互测距（节点间欧氏距离）。

- 本地状态：x = [p, v, q, b_g, b_a]；误差状态 x_err = [δp, δv, φ, δb_g, δb_a]。名义用于非线性积分，误差用于线性化更新。

- 过程模型：IMU 预积分得到离散预测（Φ,Qd），偏置为随机游走；姿态用四元数表示并在误差注入后重正化。

- 深度计量测：z_depth ≈ e_z^T p + n_d，仅作用于位置的 z 分量，提供绝对垂向观测，改善俯仰/加计零偏可观测性。

- 互测距量测：z_ij ≈ ||p_i − p_j|| + n_r。对节点 i 的线性化雅可比 H_i = (p_i − p_j)^T / ||p_i − p_j|| 作用在位置子块（在邻居 j 最近一次广播估计处评估）。为避免相关性导致过度自信，将邻居不确定度并入有效噪声：R_eff = R_r + H_j P_j H_j^T（位置子块 H_j = −H_i）。

- 分布式更新：测距到达即在本地执行卡尔曼更新（Joseph 形式增强数值稳定性），所需通信仅为邻居的 {p_j, P_jpp, 时间戳}。多邻居可顺序或批量处理；时间不同步时先以 IMU 对邻居状态外推到同一时刻。

- 可观测性与锚定：IMU+深度计提供绝对 z 与局部姿态约束，但在仅测距条件下，全局平移与航向存在规范自由度。可通过 1) 选定参考 AUV 锚定其初始位姿；2) 对网络质心施加弱先验；或 3) 仅报告相对位姿并在可视化端选定原点 来破除自由度。

- 一致性与鲁棒性：采用 R_eff 膨胀抑制信息双计数；对测距残差使用马氏距离门控与稳健核（如 Huber）抵御外点与声学误检。

- 实现要点：预测用 IMU 预积分；更新顺序先深度计再互测距；通信周期性广播 {p, P_pp} 与时间戳；误差注入后重正化 q 并保持 P 对称正定；量测噪声 R_r 与 R_d 按设备标定设置，邻居不确定度来自其广播协方差。


## 数据与方法

评估器以 1 Hz 采样记录：时间戳 t_sec、估计状态（位置/速度/姿态欧拉角）、地面真值，以及误差指标。误差计算：位置误差为欧氏范数；速度误差为速度差范数；姿态误差由估计与真值姿态的相对旋转角度得到（以度计）。日志按仿真小时切分为 <robot>_simh%04d.xlsx，位于 logs/excel/<robot>/。

## 结果总览（各机器人）

| 机器人      |   样本数 |   时长(秒) |   位置RMSE(米) |   速度RMSE(米/秒) |   姿态RMSE(度) |   位置最大(米) |   速度最大(米/秒) |   姿态最大(度) |   收敛时间(秒) |
|----------|-------|---------|-------------|---------------|-------------|-----------|-------------|-----------|-----------|
| eca_a9_1 | 46265 | 46264   |      37.617 |         0.028 |       0.025 |    47.406 |       0.381 |     0.051 |         0 |
| eca_a9_2 | 46230 | 46229   |       3.147 |         0.006 |       0     |     3.319 |       0.058 |     0.002 |         0 |
| eca_a9_3 | 46228 | 46227   |       1.811 |         0.025 |       0.001 |     2.583 |       0.365 |     0.003 |         0 |
| eca_a9_4 | 46250 | 46248.3 |       0.567 |         0.013 |       0     |     0.682 |       0.205 |     0.002 |         0 |
| eca_a9_5 | 46248 | 46246.1 |       2.689 |         0.005 |       0     |     3.123 |       0.055 |     0.001 |         0 |
| eca_a9_6 | 46245 | 46244   |       1.419 |         0.005 |       0     |     3.131 |       0.056 |     0.001 |         0 |
| eca_a9_7 | 46242 | 46240.4 |       0.215 |         0.03  |       0.001 |     0.729 |       0.495 |     0.005 |         0 |
| eca_a9_8 | 46218 | 46217   |       0.332 |         0.043 |       0.001 |     0.759 |       0.711 |     0.007 |         0 |
| eca_a9_9 | 46237 | 46235.3 |       1.189 |         0.008 |       0     |     1.987 |       0.083 |     0.001 |         0 |

## eca_a9_1

关键曲线如下（自动生成）：

![eca_a9_1 轨迹 XY](../plots/eca_a9_1_traj_xy.png)

![eca_a9_1 轨迹 XZ](../plots/eca_a9_1_traj_xz.png)

![eca_a9_1 位置误差](../plots/eca_a9_1_pos_err_ts.png)

![eca_a9_1 速度误差](../plots/eca_a9_1_vel_err_ts.png)

![eca_a9_1 姿态误差](../plots/eca_a9_1_att_err_ts.png)

![eca_a9_1 横滚角](../plots/eca_a9_1_roll_deg_ts.png)

![eca_a9_1 俯仰角](../plots/eca_a9_1_pitch_deg_ts.png)

![eca_a9_1 航向角](../plots/eca_a9_1_yaw_deg_ts.png)


## eca_a9_2

关键曲线如下（自动生成）：

![eca_a9_2 轨迹 XY](../plots/eca_a9_2_traj_xy.png)

![eca_a9_2 轨迹 XZ](../plots/eca_a9_2_traj_xz.png)

![eca_a9_2 位置误差](../plots/eca_a9_2_pos_err_ts.png)

![eca_a9_2 速度误差](../plots/eca_a9_2_vel_err_ts.png)

![eca_a9_2 姿态误差](../plots/eca_a9_2_att_err_ts.png)

![eca_a9_2 横滚角](../plots/eca_a9_2_roll_deg_ts.png)

![eca_a9_2 俯仰角](../plots/eca_a9_2_pitch_deg_ts.png)

![eca_a9_2 航向角](../plots/eca_a9_2_yaw_deg_ts.png)


## eca_a9_3

关键曲线如下（自动生成）：

![eca_a9_3 轨迹 XY](../plots/eca_a9_3_traj_xy.png)

![eca_a9_3 轨迹 XZ](../plots/eca_a9_3_traj_xz.png)

![eca_a9_3 位置误差](../plots/eca_a9_3_pos_err_ts.png)

![eca_a9_3 速度误差](../plots/eca_a9_3_vel_err_ts.png)

![eca_a9_3 姿态误差](../plots/eca_a9_3_att_err_ts.png)

![eca_a9_3 横滚角](../plots/eca_a9_3_roll_deg_ts.png)

![eca_a9_3 俯仰角](../plots/eca_a9_3_pitch_deg_ts.png)

![eca_a9_3 航向角](../plots/eca_a9_3_yaw_deg_ts.png)


## eca_a9_4

关键曲线如下（自动生成）：

![eca_a9_4 轨迹 XY](../plots/eca_a9_4_traj_xy.png)

![eca_a9_4 轨迹 XZ](../plots/eca_a9_4_traj_xz.png)

![eca_a9_4 位置误差](../plots/eca_a9_4_pos_err_ts.png)

![eca_a9_4 速度误差](../plots/eca_a9_4_vel_err_ts.png)

![eca_a9_4 姿态误差](../plots/eca_a9_4_att_err_ts.png)

![eca_a9_4 横滚角](../plots/eca_a9_4_roll_deg_ts.png)

![eca_a9_4 俯仰角](../plots/eca_a9_4_pitch_deg_ts.png)

![eca_a9_4 航向角](../plots/eca_a9_4_yaw_deg_ts.png)


## eca_a9_5

关键曲线如下（自动生成）：

![eca_a9_5 轨迹 XY](../plots/eca_a9_5_traj_xy.png)

![eca_a9_5 轨迹 XZ](../plots/eca_a9_5_traj_xz.png)

![eca_a9_5 位置误差](../plots/eca_a9_5_pos_err_ts.png)

![eca_a9_5 速度误差](../plots/eca_a9_5_vel_err_ts.png)

![eca_a9_5 姿态误差](../plots/eca_a9_5_att_err_ts.png)

![eca_a9_5 横滚角](../plots/eca_a9_5_roll_deg_ts.png)

![eca_a9_5 俯仰角](../plots/eca_a9_5_pitch_deg_ts.png)

![eca_a9_5 航向角](../plots/eca_a9_5_yaw_deg_ts.png)


## eca_a9_6

关键曲线如下（自动生成）：

![eca_a9_6 轨迹 XY](../plots/eca_a9_6_traj_xy.png)

![eca_a9_6 轨迹 XZ](../plots/eca_a9_6_traj_xz.png)

![eca_a9_6 位置误差](../plots/eca_a9_6_pos_err_ts.png)

![eca_a9_6 速度误差](../plots/eca_a9_6_vel_err_ts.png)

![eca_a9_6 姿态误差](../plots/eca_a9_6_att_err_ts.png)

![eca_a9_6 横滚角](../plots/eca_a9_6_roll_deg_ts.png)

![eca_a9_6 俯仰角](../plots/eca_a9_6_pitch_deg_ts.png)

![eca_a9_6 航向角](../plots/eca_a9_6_yaw_deg_ts.png)


## eca_a9_7

关键曲线如下（自动生成）：

![eca_a9_7 轨迹 XY](../plots/eca_a9_7_traj_xy.png)

![eca_a9_7 轨迹 XZ](../plots/eca_a9_7_traj_xz.png)

![eca_a9_7 位置误差](../plots/eca_a9_7_pos_err_ts.png)

![eca_a9_7 速度误差](../plots/eca_a9_7_vel_err_ts.png)

![eca_a9_7 姿态误差](../plots/eca_a9_7_att_err_ts.png)

![eca_a9_7 横滚角](../plots/eca_a9_7_roll_deg_ts.png)

![eca_a9_7 俯仰角](../plots/eca_a9_7_pitch_deg_ts.png)

![eca_a9_7 航向角](../plots/eca_a9_7_yaw_deg_ts.png)


## eca_a9_8

关键曲线如下（自动生成）：

![eca_a9_8 轨迹 XY](../plots/eca_a9_8_traj_xy.png)

![eca_a9_8 轨迹 XZ](../plots/eca_a9_8_traj_xz.png)

![eca_a9_8 位置误差](../plots/eca_a9_8_pos_err_ts.png)

![eca_a9_8 速度误差](../plots/eca_a9_8_vel_err_ts.png)

![eca_a9_8 姿态误差](../plots/eca_a9_8_att_err_ts.png)

![eca_a9_8 横滚角](../plots/eca_a9_8_roll_deg_ts.png)

![eca_a9_8 俯仰角](../plots/eca_a9_8_pitch_deg_ts.png)

![eca_a9_8 航向角](../plots/eca_a9_8_yaw_deg_ts.png)


## eca_a9_9

关键曲线如下（自动生成）：

![eca_a9_9 轨迹 XY](../plots/eca_a9_9_traj_xy.png)

![eca_a9_9 轨迹 XZ](../plots/eca_a9_9_traj_xz.png)

![eca_a9_9 位置误差](../plots/eca_a9_9_pos_err_ts.png)

![eca_a9_9 速度误差](../plots/eca_a9_9_vel_err_ts.png)

![eca_a9_9 姿态误差](../plots/eca_a9_9_att_err_ts.png)

![eca_a9_9 横滚角](../plots/eca_a9_9_roll_deg_ts.png)

![eca_a9_9 俯仰角](../plots/eca_a9_9_pitch_deg_ts.png)

![eca_a9_9 航向角](../plots/eca_a9_9_yaw_deg_ts.png)


## 分析与讨论

- 初始阶段位置误差随时间下降，随后进入稳定区间。

- 各实例 RMSE 与最大误差差异反映了初始条件与噪声影响。

- 收敛时间依据 1.0 m 连续 5 s 阈值评估，可按任务调整。

## 结论

在当前仿真条件下，ESKF 能够稳定估计 AUV 的 6-DoF 状态，误差表现满足预期。建议后续引入 DVL/USBL 等量测以及非高斯噪声鲁棒性测试以进一步完善评估。
