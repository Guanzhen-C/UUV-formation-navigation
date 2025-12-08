#!/usr/bin/env python3
import numpy as np
import cv2
import os
import math
import torch
import torch.nn.functional as F

class TerrainMapServerGPU:
    def __init__(self, asc_file_path, device='cuda'):
        self.file_path = asc_file_path
        self.device = device
        self.header = {}
        self.elevation_tensor = None # (1, 1, H, W) on GPU
        self.min_elev = 0.0
        self.max_elev = 0.0
        self.resolution_x = 0.0 
        self.resolution_y = 0.0
        self.real_width = 0.0
        self.real_height = 0.0
        
        self.load_asc()

    def load_asc(self):
        print(f"[GPU] Loading terrain from {self.file_path}...")
        if not os.path.exists(self.file_path):
            raise FileNotFoundError(f"ASC file not found: {self.file_path}")

        data_lines = []
        with open(self.file_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 2 and parts[0].lower() in ['ncols', 'nrows', 'xllcorner', 'yllcorner', 'cellsize', 'nodata_value']:
                    self.header[parts[0].lower()] = float(parts[1])
                else:
                    data_lines.append(np.fromstring(line, sep=' '))

        self.ncols = int(self.header['ncols'])
        self.nrows = int(self.header['nrows'])
        self.cellsize = self.header['cellsize']
        self.nodata = self.header.get('nodata_value', -9999)

        raw_data = np.concatenate(data_lines)
        elevation_grid = raw_data.reshape((self.nrows, self.ncols))
        
        # Handle NoData
        valid_mask = elevation_grid != self.nodata
        valid_data = elevation_grid[valid_mask]
        
        if len(valid_data) == 0:
            raise ValueError("Map contains no valid data!")

        self.min_elev = np.min(valid_data)
        self.max_elev = np.max(valid_data)
        
        # Fill nodata with min_elev
        elevation_grid[~valid_mask] = self.min_elev
        
        # Upload to GPU
        # grid_sample expects (N, C, H, W). Here N=1, C=1.
        # Also, grid_sample coordinates (-1, 1) assume:
        # (-1, -1) is Top-Left corner of the first pixel
        # (1, 1) is Bottom-Right corner of the last pixel
        self.elevation_tensor = torch.from_numpy(elevation_grid).float().to(self.device)
        self.elevation_tensor = self.elevation_tensor.unsqueeze(0).unsqueeze(0) # (1, 1, H, W)
        
        # Resolution
        lat_min = self.header['yllcorner']
        avg_lat = lat_min + (self.nrows * self.cellsize) / 2.0
        self.resolution_y = 111320.0 * self.cellsize
        self.resolution_x = 111320.0 * self.cellsize * math.cos(math.radians(avg_lat))
        
        self.real_width = self.ncols * self.resolution_x
        self.real_height = self.nrows * self.resolution_y

        print(f"[GPU] Map Loaded on {self.device}: {self.ncols}x{self.nrows}")
        print(f"Resolution: {self.resolution_x:.2f}m x {self.resolution_y:.2f}m")

    def get_elevation_batch(self, x_metric, y_metric):
        """
        Batch query elevation using GPU grid_sample.
        x_metric, y_metric: Tensor (N,) or (N, M) in metric coordinates (center origin)
        Returns: Tensor of elevations (same shape)
        """
        # 1. Normalize coordinates to [-1, 1]
        # Center (0,0) -> (0,0) in UV if map is centered
        # Left Edge (-W/2) -> -1
        # Right Edge (W/2) -> 1
        # Top Edge (H/2) -> -1 (Note: grid_sample Y is usually Top-Down -1..1)
        # But our metric Y is usually ENU (Up/North is positive).
        # Let's check ASC order. ASC rows usually start from Top (North).
        # So Row 0 (Top) corresponds to Y_max. Row N (Bottom) is Y_min.
        # In grid_sample, y=-1 is Top, y=1 is Bottom.
        # So if metric Y increases (North), we are moving towards Top, so grid_sample Y should decrease towards -1.
        
        u = x_metric / (self.real_width / 2.0)
        v = -y_metric / (self.real_height / 2.0) # Flip Y
        
        # Stack to (Total_Points, 2)
        uv = torch.stack((u, v), dim=-1)
        
        # Reshape for grid_sample: (1, 1, Total_Points, 2)
        # We treat all query points as a single "row" of pixels in the output
        # Batch=1 matches the map batch size.
        grid = uv.view(1, 1, -1, 2)
        
        # Output: (1, 1, 1, Total_Points)
        sampled = F.grid_sample(self.elevation_tensor, grid, align_corners=False, padding_mode='border')
        
        # Reshape back to input shape
        return sampled.view(x_metric.shape)

