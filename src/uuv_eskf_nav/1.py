#!/usr/bin/env python3
import numpy as np
import cv2

def generate_q1_rotated():
    # ================= 配置 =================
    ncols, nrows = 257, 257
    real_width, real_height = 1000.0, 1000.0
    xllcorner, yllcorner = -real_width/2.0, -real_height/2.0
    cellsize = real_width / (ncols - 1)
    nodata_value = -9999
    
    print("Generating Map: Source Quadrant 1 (Top-Right) Rotated")
    
    # 原始坐标网格
    x_orig = np.linspace(0, real_width, ncols)
    y_orig = np.linspace(0, real_height, nrows)
    xv_raw, yv_raw = np.meshgrid(x_orig, y_orig)
    
    # 将坐标原点移到中心 (物理坐标)
    center_x, center_y = real_width / 2.0, real_height / 2.0
    xv_phys = xv_raw - center_x
    yv_phys = yv_raw - center_y
    
    # ================= 核心逻辑：坐标映射 =================
    xv, yv = xv_phys.copy(), yv_phys.copy() # 用于计算地形的坐标
    
    # 1. 第二象限 (x<0, y>0) -> 顺时针转90度到 Q1 -> (y, -x)
    mask_q2 = (xv_phys < 0) & (yv_phys >= 0)
    xv[mask_q2], yv[mask_q2] = yv_phys[mask_q2], -xv_phys[mask_q2]
    
    # 2. 第三象限 (x<0, y<0) -> 旋转180度到 Q1 -> (-x, -y)
    mask_q3 = (xv_phys < 0) & (yv_phys < 0)
    xv[mask_q3], yv[mask_q3] = -xv_phys[mask_q3], -yv_phys[mask_q3]
    
    # 3. 第四象限 (x>0, y<0) -> 逆时针转90度到 Q1 -> (-y, x)
    mask_q4 = (xv_phys >= 0) & (yv_phys < 0)
    xv[mask_q4], yv[mask_q4] = -yv_phys[mask_q4], xv_phys[mask_q4]
    
    # 还原回原来的 Grid 坐标系用于代入公式
    xv = xv + center_x
    yv = yv + center_y

    # ================= 地形生成逻辑 =================
    dist_to_center = np.sqrt((xv - center_x)**2 + (yv - center_y)**2)
    angle = np.arctan2(yv - center_y, xv - center_x)
    
    z_hills = 6.0 * np.sin(xv / 100.0) * np.cos(yv / 100.0)
    z_unique_1 = 3.0 * np.sin(xv / 40.0 + np.pi/3) * np.cos(yv / 35.0)
    z_unique_2 = 2.5 * np.sin(xv / 25.0) * np.cos(yv / 45.0 + np.pi/4)
    
    for angle_marker in [0, np.pi/3, 2*np.pi/3, np.pi, 4*np.pi/3, 5*np.pi/3]:
        dist_to_trajectory = np.abs(dist_to_center - 400.0) + np.abs(angle - angle_marker) * 50.0
        feature_mask = dist_to_trajectory < 15.0
        if np.any(feature_mask):
            feature_height = 4.0 * np.sin(angle_marker) + 5.0
            z_unique_1[feature_mask] += feature_height * np.exp(-dist_to_trajectory[feature_mask]**2 / (2 * 10.0**2))
            
    z_radial_asym = 2.0 * np.sin(7 * angle + xv / 60.0) * np.exp(-(np.abs(dist_to_center - 400.0)) / 30.0)
    
    np.random.seed(42)
    for i in range(15):
        feature_x = center_x + 350 * np.cos(2 * np.pi * i / 15)
        feature_y = center_y + 350 * np.sin(2 * np.pi * i / 15)
        feature_radius = np.random.uniform(8, 20)
        feature_height = np.random.uniform(3, 6) * (-1)**i
        dist_to_feature = np.sqrt((xv - feature_x)**2 + (yv - feature_y)**2)
        feature_mask = dist_to_feature < feature_radius * 2
        z_unique_1[feature_mask] += feature_height * np.exp(-dist_to_feature[feature_mask]**2 / (2 * feature_radius**2))
        
    center_line = (xv + yv) / 2.0
    dist_from_center_line = np.abs(center_line - (real_width+real_height)/2.0)
    z_canyon = -12.0 * np.exp(-(dist_from_center_line / 80.0)**2)
    z_ridges = 4.0 * np.sin((xv - yv + xv*yv/50000.0) / 70.0)
    z_small_hills = 2.5 * np.sin(xv / 25.0 + yv / 30.0) * np.sin(yv / 20.0 + xv / 35.0)
    z_detail = 1.2 * np.sin(xv / 12.0) * np.sin(yv / 12.0) * np.sin(xv*yv / 1000.0)
    
    edge_x = np.minimum(xv, real_width - xv)
    edge_y = np.minimum(yv, real_height - yv)
    z_edge_features = np.zeros_like(xv)
    z_edge_features[edge_x < 50] += 2.0 * np.sin(edge_x[edge_x < 50] / 10.0)
    z_edge_features[edge_y < 50] += 2.0 * np.cos(edge_y[edge_y < 50] / 8.0)

    z_total = -105.0 + z_hills + z_canyon + z_ridges + z_small_hills + z_detail + z_unique_1 + z_unique_2 + z_radial_asym + z_edge_features
    z_total = np.minimum(z_total, -10.0)
    z_out = np.flipud(z_total)

    # ================= 1. 保存 .ASC =================
    asc_filename = "gazebo_terrain.asc"
    header = (
        f"ncols        {ncols}\n"
        f"nrows        {nrows}\n"
        f"xllcorner    {xllcorner}\n"
        f"yllcorner    {yllcorner}\n"
        f"cellsize     {cellsize:.6f}\n"
        f"NODATA_value {nodata_value}\n"
    )
    with open(asc_filename, 'w') as f:
        f.write(header)
        np.savetxt(f, z_out, fmt='%.4f', delimiter=' ')
    print(f"Saved ASC map to: {asc_filename}")

    # ================= 2. 保存 .PNG =================
    png_filename = "gazebo_terrain.png"
    min_elev = np.min(z_out)
    max_elev = np.max(z_out)
    elev_range = max_elev - min_elev
    
    # 归一化并保存 16-bit PNG
    z_norm = (z_out - min_elev) / elev_range
    z_16bit = (z_norm * 65535).astype(np.uint16)
    cv2.imwrite(png_filename, z_16bit)
    print(f"Saved 16-bit PNG to: {png_filename}")

if __name__ == "__main__":
    generate_q1_rotated()
