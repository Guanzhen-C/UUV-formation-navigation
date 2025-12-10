#!/usr/bin/env python3
import numpy as np
import cv2

def generate_combined_terrain():
    # ================= Configuration =================
    ncols = 257
    nrows = 257
    real_width = 1000.0
    real_height = 1000.0
    
    cellsize_x = real_width / (ncols - 1)
    
    xllcorner = -real_width / 2.0
    yllcorner = -real_height / 2.0
    
    nodata_value = -9999
    
    # Common base depth (used in both scripts, though they add to it differently)
    base_depth = -105.0 
    
    print(f"Generating COMBINED terrain: {ncols}x{nrows}")
    
    # Create Grid
    x = np.linspace(0, real_width, ncols)
    y = np.linspace(0, real_height, nrows)
    xv, yv = np.meshgrid(x, y)
    
    # Center coordinates
    center_x = real_width / 2.0
    center_y = real_height / 2.0
    
    # Polar coordinates for features
    dx = xv - center_x
    dy = yv - center_y
    dist_to_center = np.sqrt(dx**2 + dy**2)
    angle = np.arctan2(dy, dx)
    
    # ================= Logic A: Radial Map (For Q1 & Q3) =================
    # Source: generate_radial_map.py
    
    z_radial = np.zeros_like(xv) + base_depth
    
    # 1. Major Radial Waves
    freq_mod = 40.0 + 10.0 * np.sin(3.0 * angle)
    amp_mod = 8.0 + 3.0 * np.cos(5.0 * angle)
    z_radial += amp_mod * np.sin(freq_mod * angle)
    
    # 2. Secondary Radial Texture
    swirl = dist_to_center / 300.0
    z_radial += 4.0 * np.sin(80.0 * angle + swirl)
    
    # 3. Asymmetric Radial Ridges
    angle_noise = np.sin(17.0 * angle) * 0.2
    freq_angle = 60.0 + dist_to_center / 50.0
    z_radial += 3.0 * np.sin(freq_angle * (angle + angle_noise))
    
    # 4. Trajectory Enhancement (The 400m ring unique to radial map)
    z_traj_unique = 15.0 * np.sin(120 * angle) + 8.0 * np.sin(23 * angle) + 5.0 * np.cos(7 * angle)
    # Note: The original script calculated this but commented out the application:
    # #z_total[traj_mask] += z_traj_unique[traj_mask]
    # We will assume we follow the active code, so this is skipped or applied if you uncommented it.
    # I will stick to the provided file content which had it commented out.
    
    # 5. Random Local Features (Radial style)
    np.random.seed(42) # Set seed for consistency
    for i in range(30):
        r = np.random.uniform(50, 480)
        theta = np.random.uniform(-np.pi, np.pi)
        fx = center_x + r * np.cos(theta)
        fy = center_y + r * np.sin(theta)
        dist = np.sqrt((xv - fx)**2 + (yv - fy)**2)
        rad = np.random.uniform(10, 25)
        h = np.random.uniform(6, 12) * (1 if np.random.random() > 0.5 else -1)
        mask = dist < rad * 2
        z_radial[mask] += h * np.exp(-dist[mask]**2 / (2 * (rad/2)**2))
        
    # 6. High Freq Noise
    z_radial += np.random.normal(0, 0.5, xv.shape)
    
    # Radial Clipping
    z_radial = np.clip(z_radial, -180.0, -20.0)

    # ================= Logic B: Standard Map (For Q2 & Q4) =================
    # Source: generate_map.py
    
    # 1. Hills
    z_hills = 6.0 * np.sin(xv / 100.0) * np.cos(yv / 100.0)
    
    # 2. Asymmetric Hills
    z_unique_1 = 3.0 * np.sin(xv / 40.0 + np.pi/3) * np.cos(yv / 35.0)
    z_unique_2 = 2.5 * np.sin(xv / 25.0) * np.cos(yv / 45.0 + np.pi/4)
    
    # 3. Trajectory Markers (Standard Map style)
    for angle_marker in [0, np.pi/3, 2*np.pi/3, np.pi, 4*np.pi/3, 5*np.pi/3]:
        dist_to_trajectory = np.abs(dist_to_center - 400.0) + np.abs(angle - angle_marker) * 50.0
        feature_mask = dist_to_trajectory < 15.0
        if np.any(feature_mask):
            feature_height = 4.0 * np.sin(angle_marker) + 5.0
            z_unique_1[feature_mask] += feature_height * np.exp(-dist_to_trajectory[feature_mask]**2 / (2 * 10.0**2))
            
    # 4. Radial Asym (Standard Map style)
    z_radial_asym = 2.0 * np.sin(7 * angle + xv / 60.0) * np.exp(-(np.abs(dist_to_center - 400.0)) / 30.0)
    
    # 5. Unique Fingerprints
    # Note: Re-seeding here to match generate_map logic independently
    np.random.seed(42) 
    n_unique_features = 15
    z_features_accum = np.zeros_like(xv)
    for i in range(n_unique_features):
        feature_x = center_x + 350 * np.cos(2 * np.pi * i / n_unique_features)
        feature_y = center_y + 350 * np.sin(2 * np.pi * i / n_unique_features)
        feature_radius = np.random.uniform(8, 20)
        feature_height = np.random.uniform(3, 6) * (-1)**i
        
        dist_to_feature = np.sqrt((xv - feature_x)**2 + (yv - feature_y)**2)
        feature_mask = dist_to_feature < feature_radius * 2
        z_features_accum[feature_mask] += feature_height * np.exp(-dist_to_feature[feature_mask]**2 / (2 * feature_radius**2))

    # 6. Canyons
    center_line = (xv + yv) / 2.0
    map_center_val = (real_width / 2.0 + real_height / 2.0) / 2.0
    dist_from_center_line = np.abs(center_line - map_center_val)
    z_canyon = -12.0 * np.exp(-(dist_from_center_line / 80.0)**2)

    # 7. Ridges
    z_ridges = 4.0 * np.sin((xv - yv + xv*yv/50000.0) / 70.0)

    # 8. Small Hills
    z_small_hills = 2.5 * np.sin(xv / 25.0 + yv / 30.0) * np.sin(yv / 20.0 + xv / 35.0)

    # 9. Details
    z_detail = 1.2 * np.sin(xv / 12.0) * np.sin(yv / 12.0) * np.sin(xv*yv / 1000.0)

    # 10. Edge Features
    edge_x = np.minimum(xv, real_width - xv)
    edge_y = np.minimum(yv, real_height - yv)
    z_edge_features = np.zeros_like(xv)
    z_edge_features[edge_x < 50] += 2.0 * np.sin(edge_x[edge_x < 50] / 10.0)
    z_edge_features[edge_y < 50] += 2.0 * np.cos(edge_y[edge_y < 50] / 8.0)
    
    # Combine Map
    z_map = base_depth + z_hills + z_canyon + z_ridges + z_small_hills + z_detail + \
            z_unique_1 + z_unique_2 + z_radial_asym + z_edge_features + z_features_accum
            
    z_map = np.minimum(z_map, -10.0)

    # ================= COMBINE BY QUADRANT =================
    # Quadrant Definition:
    # Q1: Top-Right (x > center, y > center) -> Radial
    # Q2: Top-Left (x < center, y > center) -> Map
    # Q3: Bottom-Left (x < center, y < center) -> Radial
    # Q4: Bottom-Right (x > center, y < center) -> Map
    
    # Create masks
    mask_q1 = (xv >= center_x) & (yv >= center_y)
    mask_q2 = (xv < center_x) & (yv >= center_y)
    mask_q3 = (xv < center_x) & (yv < center_y)
    mask_q4 = (xv >= center_x) & (yv < center_y)
    
    z_combined = np.zeros_like(xv)
    
    # Apply logic
    z_combined[mask_q1] = z_radial[mask_q1]
    z_combined[mask_q3] = z_radial[mask_q3]
    
    z_combined[mask_q2] = z_map[mask_q2]
    z_combined[mask_q4] = z_map[mask_q4]
    
    # ================= Post-Process =================
    # Flip for image/map server convention
    z_out = np.flipud(z_combined)
    
    # ================= Save .ASC =================
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

    # ================= Save .PNG =================
    png_filename = "gazebo_terrain.png"
    min_elev = np.min(z_out)
    max_elev = np.max(z_out)
    elev_range = max_elev - min_elev
    
    gazebo_pose_z = (min_elev + max_elev) / 2.0
    
    print("-" * 30)
    print("【Combined Map Configuration】:")
    print(f"  Min Elev:  {min_elev:.4f}")
    print(f"  Max Elev:  {max_elev:.4f}")
    print(f"  Size Z:    {elev_range:.4f}")
    print(f"  Pose Z:    {gazebo_pose_z:.4f}")
    print("-" * 30)

    if elev_range == 0:
        z_norm = np.zeros_like(z_out)
    else:
        z_norm = (z_out - min_elev) / elev_range
        
    z_16bit = (z_norm * 65535).astype(np.uint16)
    cv2.imwrite(png_filename, z_16bit)
    print(f"Saved {png_filename}")

if __name__ == "__main__":
    generate_combined_terrain()
