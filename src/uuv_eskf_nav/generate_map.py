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
    base_depth = -100.0 
    
    print(f"Generating terrain: {ncols}x{nrows} pixels")
    print(f"Physical Size: {real_width}m x {real_height}m")
    
    # 创建网格
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv, yv = np.meshgrid(x, y)
    
    # ================= 地形特征 (增强海底地形起伏) =================
    # 1. 大波浪 (海底山脉) - 增加幅度但仍保持海底环境
    z_hills = 12.0 * np.sin(xv / 120.0) * np.cos(yv / 120.0)
    
    # 2. 海底峡谷 (增加更深的沟壑)
    center_line = (xv + yv) / 2.0
    map_center = (real_width / 2.0 + real_height / 2.0) / 2.0
    dist_from_center = np.abs(center_line - map_center)
    z_canyon = -20.0 * np.exp(-(dist_from_center / 100.0)**2)
    
    # 3. 海底山脊 (增加线性地形特征)
    z_ridges = 6.0 * np.sin((xv - yv) / 80.0)
    
    # 4. 小规模海底丘陵 (增加中等尺度特征)
    z_small_hills = 4.0 * np.sin(xv / 40.0) * np.sin(yv / 40.0)
    
    # 5. 细节 (保持海底地形细节)
    z_detail = 1.8 * np.sin(xv / 20.0) * np.sin(yv / 20.0)

    # 组合并限制 - 确保地形不会超过海面
    z_total = base_depth + z_hills + z_canyon + z_ridges + z_small_hills + z_detail
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
