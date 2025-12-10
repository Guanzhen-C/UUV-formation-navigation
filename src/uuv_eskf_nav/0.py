#!/usr/bin/env python3
import numpy as np
import cv2

def generate_octant_rotated():
    # ================= 配置参数 =================
    ncols = 257
    nrows = 257
    real_width = 1000.0
    real_height = 1000.0
    
    cellsize = real_width / (ncols - 1)
    xllcorner = -real_width / 2.0
    yllcorner = -real_height / 2.0
    nodata_value = -9999
    base_depth = -105.0
    
    print(f"Generating 8-fold Rotated Terrain (Pinwheel): {ncols}x{nrows}")
    print("Logic: Q1 Top-Left Triangle (45-90 deg) -> Rotated 7 times to fill 360 deg")
    
    # 1. 创建物理坐标网格
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv_raw, yv_raw = np.meshgrid(x, y)
    
    center_x = real_width / 2.0
    center_y = real_height / 2.0
    
    # 2. 计算极坐标
    dx = xv_raw - center_x
    dy = yv_raw - center_y
    dist = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx) # 范围 -pi 到 pi
    
    # 3. 核心坐标映射逻辑
    # 目标：将全图任意位置的像素映射回“源区域”
    # 源区域：第一象限左上三角形 (Q1 Top-Left)。
    # 对应角度：45度 到 90度 (pi/4 到 pi/2)
    
    # 将角度归一化到 0 到 2pi
    angle_norm = np.mod(angle, 2 * np.pi)
    
    # 定义扇区宽度 (45度)
    sector_width = np.pi / 4.0
    
    # 计算在该扇区内的偏移量 (0 到 45度)
    # 这会把 0-45, 45-90, 90-135... 全部重叠到 0-45 范围内
    angle_offset = np.mod(angle_norm, sector_width)
    
    # 将其平移到源扇区范围 (45-90度)
    # 这样全图所有点都会去查 45-90 度这个区域的数据
    angle_src = angle_offset + np.pi / 4.0
    
    # 4. 计算源坐标 (用于代入地形公式)
    xv_calc = center_x + dist * np.cos(angle_src)
    yv_calc = center_y + dist * np.sin(angle_src)
    
    # 为了让 generate_map.py 的公式正常工作，我们需要 xv, yv
    xv = xv_calc
    yv = yv_calc
    
    # 重新计算映射后的 angle (虽然大部分公式用 xv/yv 即可，但有些用 angle)
    # 注意：这里的 angle 已经是 angle_src 了
    angle_for_calc = angle_src 

    # ================= 地形特征生成 (使用映射后的坐标) =================
    
    # 1. 大波浪
    z_hills = 6.0 * np.sin(xv / 100.0) * np.cos(yv / 100.0)
    
    # 2. 非对称山丘
    z_unique_1 = 3.0 * np.sin(xv / 40.0 + np.pi/3) * np.cos(yv / 35.0)
    z_unique_2 = 2.5 * np.sin(xv / 25.0) * np.cos(yv / 45.0 + np.pi/4)
    
    # 3. 轨迹标记 (注意：源区域是 45-90度)
    # 我们只保留源区域内的标记，然后它会被自动复制
    for angle_marker in [0, np.pi/3, 2*np.pi/3, np.pi, 4*np.pi/3, 5*np.pi/3]:
        # 计算相对于源坐标的距离
        dist_to_trajectory = np.abs(dist - 400.0) + np.abs(angle_for_calc - angle_marker) * 50.0
        feature_mask = dist_to_trajectory < 15.0
        if np.any(feature_mask):
            feature_height = 4.0 * np.sin(angle_marker) + 5.0
            z_unique_1[feature_mask] += feature_height * np.exp(-dist_to_trajectory[feature_mask]**2 / (2 * 10.0**2))
    
    # 4. 径向特征
    z_radial_asym = 2.0 * np.sin(7 * angle_for_calc + xv / 60.0) * np.exp(-(np.abs(dist - 400.0)) / 30.0)
    
    # 5. 随机特征 (Fingerprints)
    np.random.seed(42)
    n_unique_features = 15
    for i in range(n_unique_features):
        feature_x = center_x + 350 * np.cos(2 * np.pi * i / n_unique_features)
        feature_y = center_y + 350 * np.sin(2 * np.pi * i / n_unique_features)
        feature_radius = np.random.uniform(8, 20)
        feature_height = np.random.uniform(3, 6) * (-1)**i
        
        # 计算映射后的坐标到特征点的距离
        dist_to_feature = np.sqrt((xv - feature_x)**2 + (yv - feature_y)**2)
        feature_mask = dist_to_feature < feature_radius * 2
        z_feature = feature_height * np.exp(-dist_to_feature**2 / (2 * feature_radius**2))
        z_unique_1[feature_mask] += z_feature[feature_mask]
    
    # 6. 峡谷
    center_line = (xv + yv) / 2.0
    map_center = (real_width / 2.0 + real_height / 2.0) / 2.0
    dist_from_center_line = np.abs(center_line - map_center)
    z_canyon = -12.0 * np.exp(-(dist_from_center_line / 80.0)**2)
    
    # 7. 山脊
    z_ridges = 4.0 * np.sin((xv - yv + xv*yv/50000.0) / 70.0)
    
    # 8. 细节
    z_small_hills = 2.5 * np.sin(xv / 25.0 + yv / 30.0) * np.sin(yv / 20.0 + xv / 35.0)
    z_detail = 1.2 * np.sin(xv / 12.0) * np.sin(yv / 12.0) * np.sin(xv*yv / 1000.0)
    
    # 9. 边界特征 (也会被旋转)
    edge_x = np.minimum(xv, real_width - xv)
    edge_y = np.minimum(yv, real_height - yv)
    z_edge_features = np.zeros_like(xv)
    z_edge_features[edge_x < 50] += 2.0 * np.sin(edge_x[edge_x < 50] / 10.0)
    z_edge_features[edge_y < 50] += 2.0 * np.cos(edge_y[edge_y < 50] / 8.0)

    # ================= 组合 =================
    z_total = base_depth + z_hills + z_canyon + z_ridges + z_small_hills + z_detail + z_unique_1 + z_unique_2 + z_radial_asym + z_edge_features
    z_total = np.minimum(z_total, -10.0)
    
    # 翻转
    z_out = np.flipud(z_total)

    # ================= 保存 .ASC =================
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

    # ================= 保存 .PNG =================
    png_filename = "gazebo_terrain.png"
    min_elev = np.min(z_out)
    max_elev = np.max(z_out)
    elev_range = max_elev - min_elev
    
    print("-" * 30)
    print("【Gazebo 配置】")
    print(f"  Min/Max: {min_elev:.2f} / {max_elev:.2f}")
    print(f"  Pose Z:  {(min_elev+max_elev)/2.0:.2f}")
    print("-" * 30)

    if elev_range == 0:
        z_norm = np.zeros_like(z_out)
    else:
        z_norm = (z_out - min_elev) / elev_range
        
    z_16bit = (z_norm * 65535).astype(np.uint16)
    cv2.imwrite(png_filename, z_16bit)
    print(f"Saved 16-bit PNG to: {png_filename}")

if __name__ == "__main__":
    generate_octant_rotated()
