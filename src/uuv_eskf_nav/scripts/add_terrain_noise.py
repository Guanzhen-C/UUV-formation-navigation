#!/usr/bin/env python3
import numpy as np
import cv2
from terrain_map_server import TerrainMapServer
import os

def generate_fractal_noise(shape, scale=10.0, octaves=4, persistence=0.5, lacunarity=2.0):
    """
    Generate fractal noise using superimposed sinusoids.
    shape: (rows, cols)
    scale: base scale of the features
    """
    rows, cols = shape
    noise = np.zeros(shape)
    
    x = np.linspace(0, scale, cols)
    y = np.linspace(0, scale, rows)
    X, Y = np.meshgrid(x, y)
    
    frequency = 1.0
    amplitude = 1.0
    
    for _ in range(octaves):
        # Superimpose random sinusoids
        angle = np.random.uniform(0, 2*np.pi)
        # Rotate coordinates to avoid grid artifacts
        X_rot = X * np.cos(angle) - Y * np.sin(angle)
        Y_rot = X * np.sin(angle) + Y * np.cos(angle)
        
        noise += amplitude * np.sin(X_rot * frequency) * np.sin(Y_rot * frequency)
        
        frequency *= lacunarity
        amplitude *= persistence
        
    return noise

def add_noise_to_terrain(input_asc, output_asc_name, noise_magnitude=5.0):
    print(f"Reading {input_asc}...")
    server = TerrainMapServer(input_asc)
    
    rows = server.nrows
    cols = server.ncols
    
    # 1. Generate High-Freq Noise
    print("Generating synthetic terrain details...")
    # We want high frequency details.
    # Scale factor determines how "dense" the bumps are.
    # Higher scale = more bumps.
    noise = generate_fractal_noise((rows, cols), scale=50.0, octaves=6)
    
    # Normalize noise to -1..1
    noise = (noise - np.min(noise)) / (np.max(noise) - np.min(noise)) * 2.0 - 1.0
    
    # Scale by magnitude (e.g. +/- 5 meters)
    noise_map = noise * noise_magnitude
    
    # 2. Add to original elevation
    # Only add noise to valid data
    new_elevation = server.elevation_grid.copy()
    mask = server.valid_mask
    
    new_elevation[mask] += noise_map[mask]
    
    # 3. Save as new ASC file
    output_dir = os.path.dirname(input_asc)
    output_path = os.path.join(output_dir, output_asc_name)
    
    print(f"Saving enhanced terrain to {output_path}...")
    with open(output_path, 'w') as f:
        # Write Header
        f.write(f"ncols         {cols}\n")
        f.write(f"nrows         {rows}\n")
        f.write(f"xllcorner     {server.header['xllcorner']}\n")
        f.write(f"yllcorner     {server.header['yllcorner']}\n")
        f.write(f"cellsize      {server.header['cellsize']}\n")
        f.write(f"NODATA_value  {server.nodata}\n")
        
        # Write Data
        for i in range(rows):
            # ASC data is space separated
            line_data = new_elevation[i, :]
            # Format to avoid scientific notation clutter if possible, but float is fine
            line_str = " ".join([f"{val:.2f}" for val in line_data])
            f.write(line_str + "\n")
            
    return output_path

if __name__ == "__main__":
    base_path = "/home/cgz/catkin_ws/src/uuv_eskf_nav/gebco_2025_n40.4838_s39.4745_w131.501_e132.4997.asc"
    output_name = "terrain_with_noise.asc"
    
    new_asc_path = add_noise_to_terrain(base_path, output_name, noise_magnitude=15.0) # +/- 15m noise
    
    # Now generate PNG for Gazebo
    server_new = TerrainMapServer(new_asc_path)
    # Force resize to 513x513 for Gazebo
    server_new.generate_heightmap_png() 
