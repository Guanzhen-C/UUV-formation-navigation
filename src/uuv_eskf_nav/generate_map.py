#!/usr/bin/env python3
import numpy as np
import cv2  # 需要 opencv-python 库来保存 16-bit 图片

def generate_terrain():
    # ================= 配置参数 =================
    # Gazebo 最佳实践: 使用 2^n + 1 的尺寸
    # 256 + 1 = 257。这比 200x200 稍微精细一点点
    ncols = 257         
    nrows = 257         
    
    # 物理尺寸 (米)
    real_width = 1000.0
    real_height = 1000.0
    
    # 自动计算新的分辨率
    cellsize_x = real_width / (ncols - 1) # 注意 -1，因为是点间距
    cellsize_y = real_height / (nrows - 1)
    
    # 地图中心位于 (0,0)，所以左下角是 -500
    xllcorner = -real_width / 2.0
    yllcorner = -real_height / 2.0
    
    nodata_value = -9999
    base_depth = -105.0 
    
    print(f"Generating terrain: {ncols}x{nrows} pixels")
    print(f"Physical Size: {real_width}m x {real_height}m")
    
    # 创建网格
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv, yv = np.meshgrid(x, y)
    
    # ================= 地形特征 (增强圆形轨迹区域独特特征) =================
    # 地图中心 (128, 128) 对应物理坐标 (0, 0)
    # 400米半径圆在网格坐标系中约为 102 像素半径
    
    # 计算到中心点的距离
    center_x = real_width / 2.0
    center_y = real_height / 2.0
    dist_to_center = np.sqrt((xv - center_x)**2 + (yv - center_y)**2)
    angle = np.arctan2(yv - center_y, xv - center_x)  # 角度坐标
    
    # 1. 大波浪 (海底山脉) - 保持通用起伏
    z_hills = 6.0 * np.sin(xv / 100.0) * np.cos(yv / 100.0)
    
    # 2. 非对称的海底山丘 - 创建独特的位置标识
    # 用不同频率和相位的正弦波创建不重复的地形模式
    z_unique_1 = 3.0 * np.sin(xv / 40.0 + np.pi/3) * np.cos(yv / 35.0)
    z_unique_2 = 2.5 * np.sin(xv / 25.0) * np.cos(yv / 45.0 + np.pi/4)
    
    # 3. 在圆形轨迹上创建独特的标记点
    # 定义特定角度的突出地形特征，作为"路标"
    for angle_marker in [0, np.pi/3, 2*np.pi/3, np.pi, 4*np.pi/3, 5*np.pi/3]:  # 6个方向的标记
        # 在圆形轨迹上创建独特的地形特征
        dist_to_trajectory = np.abs(dist_to_center - 400.0) + np.abs(angle - angle_marker) * 50.0
        feature_mask = dist_to_trajectory < 15.0  # 小范围的独特特征
        if np.any(feature_mask):
            # 创建独特的地形突起，每个方向高度不同
            feature_height = 4.0 * np.sin(angle_marker) + 5.0  # 每个角度不同高度
            z_unique_1[feature_mask] += feature_height * np.exp(-dist_to_trajectory[feature_mask]**2 / (2 * 10.0**2))
    
    # 4. 创建径向不对称特征
    z_radial_asym = 2.0 * np.sin(7 * angle + xv / 60.0) * np.exp(-(np.abs(dist_to_center - 400.0)) / 30.0)
    
    # 5. 创建独特的海底突起和凹陷 - 作为地理"指纹"
    # 用高斯函数创建局部的、非重复的地形特征
    np.random.seed(42)
    n_unique_features = 15  # 15个独特地标
    for i in range(n_unique_features):
        # 随机位置，但集中在圆形轨迹附近
        feature_x = center_x + 350 * np.cos(2 * np.pi * i / n_unique_features)  # 沿圆形分布
        feature_y = center_y + 350 * np.sin(2 * np.pi * i / n_unique_features)
        feature_radius = np.random.uniform(8, 20)  # 随机半径
        feature_height = np.random.uniform(3, 6) * (-1)**i  # 交替正负高度
        
        dist_to_feature = np.sqrt((xv - feature_x)**2 + (yv - feature_y)**2)
        feature_mask = dist_to_feature < feature_radius * 2
        z_feature = feature_height * np.exp(-dist_to_feature**2 / (2 * feature_radius**2))
        z_unique_1[feature_mask] += z_feature[feature_mask]
    
    # 6. 海底峡谷 (保持原有对角线峡谷，增加特征多样性)
    center_line = (xv + yv) / 2.0
    map_center = (real_width / 2.0 + real_height / 2.0) / 2.0
    dist_from_center_line = np.abs(center_line - map_center)
    z_canyon = -12.0 * np.exp(-(dist_from_center_line / 80.0)**2)
    
    # 7. 非对称的海底山脊
    z_ridges = 4.0 * np.sin((xv - yv + xv*yv/50000.0) / 70.0)  # 加入非线性项增加独特性
    
    # 8. 中等尺度特征 - 增加更多细节
    z_small_hills = 2.5 * np.sin(xv / 25.0 + yv / 30.0) * np.sin(yv / 20.0 + xv / 35.0)
    
    # 9. 高频细节 - 增加纹理特征
    z_detail = 1.2 * np.sin(xv / 12.0) * np.sin(yv / 12.0) * np.sin(xv*yv / 1000.0)
    
    # 10. 边界增强 - 在地图边缘创建参考特征
    edge_x = np.minimum(xv, real_width - xv)  # 距离左右边界的距离
    edge_y = np.minimum(yv, real_height - yv)  # 距离上下边界的距离
    z_edge_features = np.zeros_like(xv)
    z_edge_features[edge_x < 50] += 2.0 * np.sin(edge_x[edge_x < 50] / 10.0)
    z_edge_features[edge_y < 50] += 2.0 * np.cos(edge_y[edge_y < 50] / 8.0)

    # 组合并限制 - 确保地形不会超过海面
    z_total = base_depth + z_hills + z_canyon + z_ridges + z_small_hills + z_detail + z_unique_1 + z_unique_2 + z_radial_asym + z_edge_features
    z_total = np.minimum(z_total, -10.0)  # 确保至少有10米水深
    z_total = np.minimum(z_total, -10.0)
    
    # 上下翻转 (匹配 Gazebo 和常规地图习惯)
    z_out = np.flipud(z_total)

    # ================= 1. 保存 .ASC (给粒子滤波) =================
    asc_filename = "gazebo_terrain.asc"
    header = (
        f"ncols        {ncols}\n"
        f"nrows        {nrows}\n"
        f"xllcorner    {xllcorner}\n"
        f"yllcorner    {yllcorner}\n"
        f"cellsize     {cellsize_x:.6f}\n" # 使用高精度
        f"NODATA_value {nodata_value}\n"
    )
    with open(asc_filename, 'w') as f:
        f.write(header)
        np.savetxt(f, z_out, fmt='%.4f', delimiter=' ')
    print(f"Saved ASC map to: {asc_filename}")

    # ================= 2. 保存 .PNG (给 Gazebo) =================
    png_filename = "gazebo_terrain.png"
    
    min_elev = np.min(z_out)
    max_elev = np.max(z_out)
    elev_range = max_elev - min_elev
    
    print("-" * 30)
    print("【Gazebo 配置参数】(请记下这些数字):")
    print(f"  Size X/Y:  {real_width}")
    print(f"  Size Z:    {elev_range:.4f} (高度差)")
    print(f"  Pose Z:    {min_elev + elev_range/2.0:.4f} (中心点高度)")
    print("-" * 30)

    # 归一化到 0.0 - 1.0
    z_norm = (z_out - min_elev) / elev_range
    
    # 转换为 16-bit 整数 (0 - 65535)
    z_16bit = (z_norm * 65535).astype(np.uint16)
    
    # 保存为 16-bit PNG
    cv2.imwrite(png_filename, z_16bit)
    print(f"Saved 16-bit PNG to: {png_filename}")

if __name__ == "__main__":
    generate_terrain()
