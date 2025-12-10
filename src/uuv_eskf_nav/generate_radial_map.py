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
    
    print(f"Generating RADIAL terrain: {ncols}x{nrows}")
    
    # 创建网格
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv, yv = np.meshgrid(x, y)
    
    # 转换为极坐标 (相对于中心)
    center_x = real_width / 2.0
    center_y = real_height / 2.0
    dx = xv - center_x
    dy = yv - center_y
    
    dist_to_center = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx) # -pi to pi
    
    # ================= 径向特征生成 (Radial Features) =================
    # 核心思想：所有特征都沿着径向分布 (Spokes)，确保圆周运动总是垂直切割它们
    
    z_total = np.zeros_like(xv) + base_depth
    
    # 1. 主径向波浪 (Major Radial Waves) - 非对称、非均匀
    # 频率调制 (FM): 让条带疏密不均
    # 基础频率 40，加上一个低频变化分量
    freq_mod = 40.0 + 10.0 * np.sin(3.0 * angle) 
    # 幅度调制 (AM): 让条带高低不一
    amp_mod = 8.0 + 3.0 * np.cos(5.0 * angle)
    
    z_radial_main = amp_mod * np.sin(freq_mod * angle) 
    z_total += z_radial_main
    
    # 2. 次级径向纹理 (Secondary Radial Texture) - 扭曲
    # 添加“漩涡”效果：相位随距离变化 (dist_to_center)
    swirl = dist_to_center / 300.0 
    z_radial_detail = 4.0 * np.sin(80.0 * angle + swirl) 
    z_total += z_radial_detail
    
    # 3. 非对称径向山脊 - 随机化
    # 叠加一个非周期的噪声项到角度上
    angle_noise = np.sin(17.0 * angle) * 0.2
    freq_angle = 60.0 + dist_to_center / 50.0 
    z_radial_complex = 3.0 * np.sin(freq_angle * (angle + angle_noise))
    z_total += z_radial_complex
    
    # 4. 400m 轨迹处的特殊增强 (Trajectory Enhancement) - 巨型山脉式起伏
    # 移除高频“搓衣板”，只保留大幅度的低频起伏，更自然，但依然保证强烈梯度
    dist_from_traj = np.abs(dist_to_center - 400.0)
    traj_mask = dist_from_traj < 80.0 
    
    # 复合波形: 保留大幅度低频成分
    # 幅度 18.0 (主) + 8.0 (调制) + 5.0 (对称破坏)
    z_traj_unique = 15.0 * np.sin(120 * angle) + 8.0 * np.sin(23 * angle) + 5.0 * np.cos(7 * angle)
    
    # 强力叠加
    #z_total[traj_mask] += z_traj_unique[traj_mask]
    
    # 5. 随机局部特征 (保留原有的随机点，增加全局定位能力)
    np.random.seed(42)
    for i in range(30): # 增加到30个
        # 随机极坐标位置
        r = np.random.uniform(50, 480)
        theta = np.random.uniform(-np.pi, np.pi)
        
        fx = center_x + r * np.cos(theta)
        fy = center_y + r * np.sin(theta)
        
        dist = np.sqrt((xv - fx)**2 + (yv - fy)**2)
        rad = np.random.uniform(10, 25)
        h = np.random.uniform(6, 12) * (1 if np.random.random() > 0.5 else -1)
        
        mask = dist < rad * 2
        z_total[mask] += h * np.exp(-dist[mask]**2 / (2 * (rad/2)**2))

    # 6. 高频噪声 (High Frequency Noise)
    # 最后的纹理层，防止任何地方绝对平坦
    z_noise = np.random.normal(0, 0.5, xv.shape)
    z_total += z_noise

    # ================= 后处理 =================
    
    # 限制深度
    z_total = np.clip(z_total, -180.0, -20.0)
    
    # NO FLIPUD based on debugging results (Assuming Map Server expects Bottom-Left origin logic for Y)
    # If Map Server expects v=-y (Top-Left origin), and our Y is Bottom-Up, we need to be careful.
    # Standard: meshgrid Y is Bottom-Up. Image Top is Row 0. 
    # Without flipud: Row 0 is Y=0 (Bottom). Image Top = Bottom.
    # Map Server v=-y: Y grows (North), v decreases (Up in image). 
    # If Image Top is Bottom, then moving North moves towards Bottom in physical, but Up in Image. 
    # This matches!
    # Wait: If Image Top (Row 0) is Physical Bottom (Y=0).
    # AUV moves North (Y increases).
    # Map Server: v = -y. As y increases, v becomes more negative. 
    # Grid Sample: -1 is Top, +1 is Bottom.
    # So v becoming negative means moving towards Top.
    # So AUV moving North looks at South data. -> FLIPPED!
    
    # So we DO need FLIPUD if Map Server uses v=-y.
    # With FLIPUD: Row 0 is Y=Max (Top). Image Top = Physical North.
    # AUV moves North -> Look at Image Top -> Look at North data. -> CORRECT.
    
    # Let's restore flipud because my logic deduction says it's needed for v=-y.
    # The previous divergence might have been due to terrain features, not flipud.
    
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
