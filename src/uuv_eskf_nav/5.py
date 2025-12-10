#!/usr/bin/env python3
import numpy as np
import cv2  # 用于保存 16-bit PNG

def generate_dense_ring_map():
    # ================= 配置参数 (保持 generate_map.py 原致) =================
    ncols = 257         
    nrows = 257         
    
    real_width = 1000.0
    real_height = 1000.0
    
    cellsize_x = real_width / (ncols - 1)
    cellsize_y = real_height / (nrows - 1)
    
    xllcorner = -real_width / 2.0
    yllcorner = -real_height / 2.0
    
    nodata_value = -9999
    base_depth = -105.0 
    
    print(f"Generating Terrain: {ncols}x{nrows}")
    print("Mode: Original Layout + Dense Concentric Ripples")
    
    # 创建网格
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv, yv = np.meshgrid(x, y)
    
    # 辅助坐标：中心点和距离 (用于生成同心条纹)
    center_x = real_width / 2.0
    center_y = real_height / 2.0
    dist_to_center = np.sqrt((xv - center_x)**2 + (yv - center_y)**2)
    angle = np.arctan2(yv - center_y, xv - center_x)
    
    # ================= 1. 地形特征 (保持 generate_map.py 一致) =================
    
    # [保持] 1. 大波浪 (海底山脉)
    z_hills = 6.0 * np.sin(xv / 100.0) * np.cos(yv / 100.0)
    
    # [保持] 2. 非对称的海底山丘
    z_unique_1 = 3.0 * np.sin(xv / 40.0 + np.pi/3) * np.cos(yv / 35.0)
    z_unique_2 = 2.5 * np.sin(xv / 25.0) * np.cos(yv / 45.0 + np.pi/4)
    
    # [保持] 3. 在圆形轨迹上创建独特的标记点 (400m 半径)
    for angle_marker in [0, np.pi/3, 2*np.pi/3, np.pi, 4*np.pi/3, 5*np.pi/3]:
        dist_to_trajectory = np.abs(dist_to_center - 400.0) + np.abs(angle - angle_marker) * 50.0
        feature_mask = dist_to_trajectory < 15.0
        if np.any(feature_mask):
            feature_height = 4.0 * np.sin(angle_marker) + 5.0
            z_unique_1[feature_mask] += feature_height * np.exp(-dist_to_trajectory[feature_mask]**2 / (2 * 10.0**2))
    
    # [保持] 4. 创建径向不对称特征
    z_radial_asym = 2.0 * np.sin(7 * angle + xv / 60.0) * np.exp(-(np.abs(dist_to_center - 400.0)) / 30.0)
    
    # [保持] 5. 随机高斯指纹特征
    np.random.seed(42)
    n_unique_features = 15
    for i in range(n_unique_features):
        feature_x = center_x + 350 * np.cos(2 * np.pi * i / n_unique_features)
        feature_y = center_y + 350 * np.sin(2 * np.pi * i / n_unique_features)
        feature_radius = np.random.uniform(8, 20)
        feature_height = np.random.uniform(3, 6) * (-1)**i
        
        dist_to_feature = np.sqrt((xv - feature_x)**2 + (yv - feature_y)**2)
        feature_mask = dist_to_feature < feature_radius * 2
        z_feature = feature_height * np.exp(-dist_to_feature**2 / (2 * feature_radius**2))
        z_unique_1[feature_mask] += z_feature[feature_mask]
    
    # [保持] 6. 海底峡谷 (对角线峡谷)
    # 虽然条纹变成了圆形，但我们保留这个大的对角线峡谷，以符合"其余部分保持一致"的要求
    center_line = (xv + yv) / 2.0
    map_center = (real_width / 2.0 + real_height / 2.0) / 2.0
    dist_from_center_line = np.abs(center_line - map_center)
    z_canyon = -12.0 * np.exp(-(dist_from_center_line / 80.0)**2)
    
    # ================= [修改重点] 7. 条纹/山脊 (Ridges) =================
    # 原代码: z_ridges = 4.0 * np.sin((xv - yv + ...) / 70.0)  <-- 对角线，低频
    # 修改后: 使用 dist_to_center 实现周向同心圆；使用 /15.0 实现高频密集
    
    # 频率: 15.0 (原为70.0)，这意味着条纹会非常密集
    z_ridges = 4.0 * np.sin(dist_to_center / 15.0) 
    
    # ================= 8. 细节与边界 (保持 generate_map.py 一致) =================
    
    # [保持] 8. 中等尺度特征
    z_small_hills = 2.5 * np.sin(xv / 25.0 + yv / 30.0) * np.sin(yv / 20.0 + xv / 35.0)
    
    # [保持] 9. 高频细节
    z_detail = 1.2 * np.sin(xv / 12.0) * np.sin(yv / 12.0) * np.sin(xv*yv / 1000.0)
    
    # [保持] 10. 边界增强
    edge_x = np.minimum(xv, real_width - xv)
    edge_y = np.minimum(yv, real_height - yv)
    z_edge_features = np.zeros_like(xv)
    z_edge_features[edge_x < 50] += 2.0 * np.sin(edge_x[edge_x < 50] / 10.0)
    z_edge_features[edge_y < 50] += 2.0 * np.cos(edge_y[edge_y < 50] / 8.0)

    # 组合
    z_total = base_depth + z_hills + z_canyon + z_ridges + z_small_hills + z_detail + z_unique_1 + z_unique_2 + z_radial_asym + z_edge_features
    
    # 限制与翻转
    z_total = np.minimum(z_total, -10.0)
    z_out = np.flipud(z_total)

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
    print(f"Saved ASC map to: {asc_filename}")

    # ================= 保存 .PNG =================
    png_filename = "gazebo_terrain.png"
    
    min_elev = np.min(z_out)
    max_elev = np.max(z_out)
    elev_range = max_elev - min_elev
    
    print("-" * 30)
    print("【Map Stats】")
    print(f"  Min Elev: {min_elev:.4f}")
    print(f"  Max Elev: {max_elev:.4f}")
    print(f"  Range:    {elev_range:.4f}")
    print("-" * 30)

    if elev_range == 0:
        z_norm = np.zeros_like(z_out)
    else:
        z_norm = (z_out - min_elev) / elev_range
    
    z_16bit = (z_norm * 65535).astype(np.uint16)
    cv2.imwrite(png_filename, z_16bit)
    print(f"Saved 16-bit PNG to: {png_filename}")

if __name__ == "__main__":
    generate_dense_ring_map()
