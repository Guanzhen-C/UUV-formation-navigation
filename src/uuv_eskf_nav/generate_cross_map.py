#!/usr/bin/env python3
import numpy as np
import cv2

def generate_terrain():
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
    
    print(f"Generating terrain: {ncols}x{nrows}")
    
    # 物理坐标网格
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv, yv = np.meshgrid(x, y)
    
    # 将坐标中心化到 (0,0) 以方便计算象限
    cx = xv - real_width / 2.0
    cy = yv - real_height / 2.0
    
    # ================= 基础地形 =================
    z_total = np.zeros_like(xv) + base_depth
    
    # 1. 基础波浪 (保持一点点，避免太生硬)
    z_total += 2.0 * np.sin(xv / 150.0) * np.cos(yv / 150.0)

    # ================= 核心逻辑：正交山脊拼接 =================
    # 目标：让山脊走向总是沿着径向（从中心向外辐射），或者总是垂直于圆周切向。
    
    # 角度坐标 (-pi 到 pi)
    angle = np.arctan2(cy, cx)
    
    # 定义两个正交的山脊系统
    # System A: 走向沿着 y = x (45度) -> 垂直于 y = -x
    # 这种山脊在西北(135度)和东南(-45度)象限，AUV 切向运动时会平行于它 -> 不好
    # AUV 在圆周运动。
    # 在第一象限(45度)，切向是135度。如果山脊走向是45度(y=x)，则互相垂直。完美。
    # 在第二象限(135度)，切向是225度。如果山脊走向是135度(y=-x)，则互相垂直。完美。
    
    # 所以：
    # 第一、三象限（东北、西南）：需要 y=x 走向的山脊。
    # 第二、四象限（西北、东南）：需要 y=-x 走向的山脊。
    
    # 为了实现平滑过渡，我们可以使用 sin(2*angle) 来混合
    # sin(2*theta): 0->0, 45->1, 90->0, 135->-1
    
    # Ridge 1 (走向 y=x): 沿 (x-y) 方向变化
    # 波长 60m, 幅度 8m
    z_ridge_diag1 = 8.0 * np.sin((cx - cy) / 60.0 * 2 * np.pi)
    
    # Ridge 2 (走向 y=-x): 沿 (x+y) 方向变化
    z_ridge_diag2 = 8.0 * np.sin((cx + cy) / 60.0 * 2 * np.pi)
    
    # Mask 1: 第一、三象限 (x*y > 0) -> 想要 Ridge 1 (y=x走向)
    # Mask 2: 第二、四象限 (x*y < 0) -> 想要 Ridge 2 (y=-x走向)
    
    # 使用软混合避免接缝
    # weight 1 在 1,3 象限为 1，在 2,4 象限为 0
    # angle 0-90 (1象限): sin(2*a) > 0. 
    # 实际上直接判断 x*y 的符号最简单
    
    mask_1 = (cx * cy) > 0
    mask_2 = (cx * cy) < 0
    
    # 混合
    z_ridges = np.zeros_like(xv)
    z_ridges[mask_1] = z_ridge_diag1[mask_1]
    z_ridges[mask_2] = z_ridge_diag2[mask_2]
    
    # 处理轴线附近的过渡 (避免突变)
    # 使用一个平滑过渡带
    blend_width = 50.0
    blend_mask = (np.abs(cx) < blend_width) | (np.abs(cy) < blend_width)
    # 在过渡带简单叠加或取平均
    z_ridges[blend_mask] = (z_ridge_diag1[blend_mask] + z_ridge_diag2[blend_mask]) / 2.0
    
    z_total += z_ridges
    
    # ================= 辅助特征 =================
    
    # 增加一些高频噪声防止局部平坦
    z_noise = np.random.normal(0, 0.5, xv.shape)
    z_total += z_noise
    
    # 限制深度
    z_total = np.clip(z_total, -180.0, -20.0)
    
    # NO FLIPUD (Based on previous findings)
    z_out = z_total

    # ================= 保存 .ASC =================
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
    print(f"Saved {asc_filename}")

    # ================= 保存 .PNG =================
    png_filename = "gazebo_terrain.png"
    min_elev = np.min(z_out)
    max_elev = np.max(z_out)
    elev_range = max_elev - min_elev
    
    gazebo_pose_z = (min_elev + max_elev) / 2.0
    
    print("-" * 30)
    print("【Gazebo 关键配置】(请更新 model.sdf):")
    print(f"  Size Z:    {elev_range:.4f}")
    print(f"  Pose Z:    {gazebo_pose_z:.4f}")
    print("-" * 30)

    z_norm = (z_out - min_elev) / elev_range
    z_16bit = (z_norm * 65535).astype(np.uint16)
    cv2.imwrite(png_filename, z_16bit)
    print(f"Saved {png_filename}")

if __name__ == "__main__":
    generate_terrain()
