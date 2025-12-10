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
    
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv, yv = np.meshgrid(x, y)
    
    # ================= 原始特征 (保持不变) =================
    # 地图中心 (128, 128) 对应物理坐标 (0, 0)
    center_x = real_width / 2.0
    center_y = real_height / 2.0
    dist_to_center = np.sqrt((xv - center_x)**2 + (yv - center_y)**2)
    angle = np.arctan2(yv - center_y, xv - center_x)
    
    # 1. 原始大波浪 (东西走向为主)
    # sin(x)*cos(y) 其实已经是网格状了，但可能周期太长，局部显平
    z_hills = 6.0 * np.sin(xv / 100.0) * np.cos(yv / 100.0)
    
    # 2. 非对称的海底山丘
    z_unique_1 = 3.0 * np.sin(xv / 40.0 + np.pi/3) * np.cos(yv / 35.0)
    z_unique_2 = 2.5 * np.sin(xv / 25.0) * np.cos(yv / 45.0 + np.pi/4)
    
    # 3. 原始地标 (6个)
    for angle_marker in [0, np.pi/3, 2*np.pi/3, np.pi, 4*np.pi/3, 5*np.pi/3]:
        dist_to_trajectory = np.abs(dist_to_center - 400.0) + np.abs(angle - angle_marker) * 50.0
        feature_mask = dist_to_trajectory < 15.0
        if np.any(feature_mask):
            feature_height = 4.0 * np.sin(angle_marker) + 5.0
            z_unique_1[feature_mask] += feature_height * np.exp(-dist_to_trajectory[feature_mask]**2 / (2 * 10.0**2))
    
    # 4. 原始径向特征
    z_radial_asym = 2.0 * np.sin(7 * angle + xv / 60.0) * np.exp(-(np.abs(dist_to_center - 400.0)) / 30.0)
    
    # 5. 原始随机特征 (15个)
    np.random.seed(42)
    for i in range(15):
        feature_x = center_x + 350 * np.cos(2 * np.pi * i / 15)
        feature_y = center_y + 350 * np.sin(2 * np.pi * i / 15)
        feature_radius = np.random.uniform(8, 20)
        feature_height = np.random.uniform(3, 6) * (-1)**i
        
        dist_to_feature = np.sqrt((xv - feature_x)**2 + (yv - feature_y)**2)
        feature_mask = dist_to_feature < feature_radius * 2
        z_feature = feature_height * np.exp(-dist_to_feature**2 / (2 * feature_radius**2))
        z_unique_1[feature_mask] += z_feature[feature_mask]
    
    # 6. 海底峡谷
    center_line = (xv + yv) / 2.0
    map_center_scalar = (real_width / 2.0 + real_height / 2.0) / 2.0
    dist_from_center_line = np.abs(center_line - map_center_scalar)
    z_canyon = -12.0 * np.exp(-(dist_from_center_line / 80.0)**2)
    
    # 7. 山脊
    z_ridges = 4.0 * np.sin((xv - yv + xv*yv/50000.0) / 70.0)
    
    # 8. 中等尺度
    z_small_hills = 2.5 * np.sin(xv / 25.0 + yv / 30.0) * np.sin(yv / 20.0 + xv / 35.0)
    
    # 9. 高频细节
    z_detail = 1.2 * np.sin(xv / 12.0) * np.sin(yv / 12.0) * np.sin(xv*yv / 1000.0)
    
    # 10. 边界
    edge_x = np.minimum(xv, real_width - xv)
    edge_y = np.minimum(yv, real_height - yv)
    z_edge_features = np.zeros_like(xv)
    z_edge_features[edge_x < 50] += 2.0 * np.sin(edge_x[edge_x < 50] / 10.0)
    z_edge_features[edge_y < 50] += 2.0 * np.cos(edge_y[edge_y < 50] / 8.0)

    # ================= NEW: 正交叠加层 (Grid Grid!) =================
    # 在原有基础上，强制叠加一组高频、高幅度的正交波浪
    # 这就像在华夫饼上又横着切了几刀
    
    # 南北向山脊 (高度随 X 变化)
    # 波长 80m (确保 AUV 几秒钟就能跨过一个)，幅度 5m
    z_north_south_ridges = 5.0 * np.sin(xv / 80.0 * 2 * np.pi) 
    
    # 东西向山脊 (高度随 Y 变化)
    # 波长 100m，幅度 4m
    z_east_west_ridges = 4.0 * np.sin(yv / 100.0 * 2 * np.pi)
    
    # 对角线切割 (彻底消除死角)
    z_diagonal_chop = 3.0 * np.sin((xv + yv) / 60.0 * 2 * np.pi)

    # 组合
    z_total = base_depth + z_hills + z_canyon + z_ridges + z_small_hills + z_detail + \
              z_unique_1 + z_unique_2 + z_radial_asym + z_edge_features + \
              z_north_south_ridges + z_east_west_ridges + z_diagonal_chop
              
    z_total = np.minimum(z_total, -10.0)
    z_total = np.minimum(z_total, -10.0)
    
    # NO FLIPUD based on previous debugging
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
    
    # Gazebo Pose Z (Center)
    gazebo_pose_z = (min_elev + max_elev) / 2.0
    
    print("-" * 30)
    print("【Gazebo 关键配置】(请更新 model.sdf):")
    print(f"  Size Z:    {elev_range:.4f}")
    # 既然我们已经不再翻转，且希望对齐，这里我们输出 min/max 中心
    # 但为了保险，还是建议用 base_depth (-105.0) 如果这证明是稳健的。
    # 不过由于加了大幅度叠加，base_depth 可能不再是均值了。
    print(f"  Pose Z:    {gazebo_pose_z:.4f} (Min/Max Center)")
    print(f"  (Base Depth was: {base_depth})")
    print("-" * 30)

    z_norm = (z_out - min_elev) / elev_range
    z_16bit = (z_norm * 65535).astype(np.uint16)
    cv2.imwrite(png_filename, z_16bit)
    print(f"Saved {png_filename}")

if __name__ == "__main__":
    generate_terrain()
