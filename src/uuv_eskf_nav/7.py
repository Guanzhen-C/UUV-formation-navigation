#!/usr/bin/env python3
import numpy as np
import cv2  # 用于保存 16-bit PNG

def generate_distinct_dense_map():
    # ================= 配置参数 =================
    ncols = 257         
    nrows = 257         
    real_width = 1000.0
    real_height = 1000.0
    cellsize_x = real_width / (ncols - 1)
    
    xllcorner = -real_width / 2.0
    yllcorner = -real_height / 2.0
    nodata_value = -9999
    base_depth = -105.0 
    
    print(f"Generating Unique Terrain: {ncols}x{nrows}")
    print("Mode: Anti-Aliasing Dense Ripples (Optimized for Particle Filter)")
    
    # 创建网格
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv, yv = np.meshgrid(x, y)
    
    # 辅助坐标
    center_x = real_width / 2.0
    center_y = real_height / 2.0
    dx = xv - center_x
    dy = yv - center_y
    dist_to_center = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx)
    
    # ================= 1. 基础大尺度特征 (保持导航的大局观) =================
    # 保持原有的低频山脉，提供全局梯度
    z_hills = 6.0 * np.sin(xv / 100.0) * np.cos(yv / 100.0)
    
    z_unique_1 = 3.0 * np.sin(xv / 40.0 + np.pi/3) * np.cos(yv / 35.0)
    
    # 轨迹标记点 (400m) - 强烈的路标
    for angle_marker in [0, np.pi/3, 2*np.pi/3, np.pi, 4*np.pi/3, 5*np.pi/3]:
        dist_to_trajectory = np.abs(dist_to_center - 400.0) + np.abs(angle - angle_marker) * 50.0
        mask = dist_to_trajectory < 15.0
        if np.any(mask):
            z_unique_1[mask] += (4.0 * np.sin(angle_marker) + 5.0) * np.exp(-dist_to_trajectory[mask]**2 / 200.0)
    
    z_radial_asym = 2.0 * np.sin(7 * angle + xv / 60.0) * np.exp(-(np.abs(dist_to_center - 400.0)) / 30.0)
    
    # 随机指纹 (Fingerprints) - 局部唯一性最强的特征
    np.random.seed(42)
    for i in range(15):
        fx = center_x + 350 * np.cos(2 * np.pi * i / 15)
        fy = center_y + 350 * np.sin(2 * np.pi * i / 15)
        d = np.sqrt((xv - fx)**2 + (yv - fy)**2)
        mask = d < 40
        h = np.random.uniform(3, 6) * (-1)**i
        z_unique_1[mask] += h * np.exp(-d[mask]**2 / (2 * np.random.uniform(8, 20)**2))
        
    z_canyon = -12.0 * np.exp(-(np.abs((xv + yv)/2.0 - 500.0) / 80.0)**2)
    
    # ================= [核心修改] 2. 具有唯一性的高频条纹 =================
    # 目标：保持5米级的密集感，但消除周期性重复
    
    # A. 基础高频波 (主波) - 5.0米波长
    wave_base = dist_to_center / 5.0
    
    # B. 干涉波 (干扰波) - 5.5米波长
    # 两个波叠加会产生 "拍 (Beat)"，形成 1/(1/5 - 1/5.5) = 55米的包络周期
    # 这意味着每隔55米，条纹会经历一次 "清晰 -> 模糊/平坦 -> 清晰" 的变化
    wave_interfere = dist_to_center / 5.5
    
    # C. 角度调制 (Orientation Lock)
    # 让干扰波的相位随角度变化。这样，在半径相同但角度不同时，波形叠加的结果不同。
    # 系数 3.0 意味着转一圈会有3个相位周期，打破旋转对称性。
    angle_mod = angle * 3.0
    
    # D. 组合
    # 主波振幅 3.0，干扰波振幅 2.0。永远不会完全抵消，保留条纹感，但峰值高度会变。
    z_ridges = 3.0 * np.sin(wave_base) + 2.0 * np.sin(wave_interfere + angle_mod)
    
    # E. 引入非线性调频 (Chirp)
    # 让波长随距离极缓慢地增加，防止远处的波形和近处完全一样
    # dist^1.1 使得越远波长拉得越开一点点
    chirp_dist = np.power(dist_to_center, 1.05) 
    # 重算一个超高频细节叠加，增加随机性
    z_ridges += 1.0 * np.sin(chirp_dist / 4.0)
    
    # ================= 3. 细节与后处理 =================
    z_small_hills = 2.5 * np.sin(xv / 25.0 + yv / 30.0) * np.sin(yv / 20.0 + xv / 35.0)
    z_detail = 1.2 * np.sin(xv / 12.0) * np.sin(yv / 12.0) * np.sin(xv*yv / 1000.0)
    
    edge_x = np.minimum(xv, real_width - xv)
    edge_y = np.minimum(yv, real_height - yv)
    z_edge = np.zeros_like(xv)
    z_edge[edge_x < 50] += 2.0 * np.sin(edge_x[edge_x < 50] / 10.0)
    z_edge[edge_y < 50] += 2.0 * np.cos(edge_y[edge_y < 50] / 8.0)

    # 组合
    z_total = base_depth + z_hills + z_canyon + z_ridges + z_small_hills + z_detail + z_unique_1 + z_radial_asym + z_edge
    
    z_total = np.minimum(z_total, -10.0)
    z_out = np.flipud(z_total)

    # ================= 保存 =================
    asc_filename = "gazebo_terrain.asc"
    header = (
        f"ncols        {ncols}\n"
        f"nrows        {nrows}\n"
        f"xllcorner    {xllcorner}\n"
        f"yllcorner    {yllcorner}\n"
        f"cellsize     {cellsize_x:.6f}\n"
        f"NODATA_value {nodata_value}\n"
    )
    with open(asc_filename, 'w') as f:
        f.write(header)
        np.savetxt(f, z_out, fmt='%.4f', delimiter=' ')
    print(f"Saved ASC map to: {asc_filename}")

    png_filename = "gazebo_terrain.png"
    min_elev, max_elev = np.min(z_out), np.max(z_out)
    elev_range = max_elev - min_elev
    
    print("-" * 30)
    print("【Uniqueness Upgrade】")
    print("  1. Applied Beat Interference (Dual Frequency) to break radial symmetry.")
    print("  2. Applied Angular Phase Modulation to break rotational symmetry.")
    print(f"  Range: {elev_range:.4f}")
    print("-" * 30)

    z_norm = (z_out - min_elev) / elev_range if elev_range > 0 else np.zeros_like(z_out)
    cv2.imwrite(png_filename, (z_norm * 65535).astype(np.uint16))
    print(f"Saved 16-bit PNG to: {png_filename}")

if __name__ == "__main__":
    generate_distinct_dense_map()
