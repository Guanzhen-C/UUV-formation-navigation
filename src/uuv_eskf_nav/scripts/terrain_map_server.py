#!/usr/bin/env python3
import numpy as np
import cv2
import os
import math

class TerrainMapServer:
    def __init__(self, asc_file_path):
        self.file_path = asc_file_path
        self.header = {}
        self.elevation_grid = None
        self.valid_mask = None
        self.min_elev = 0.0
        self.max_elev = 0.0
        self.resolution_x = 0.0 # meters per cell
        self.resolution_y = 0.0 # meters per cell
        
        self.load_asc()

    def load_asc(self):
        print(f"Loading terrain from {self.file_path}...")
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
        self.elevation_grid = raw_data.reshape((self.nrows, self.ncols))
        
        # Handle NoData and Statistics
        self.valid_mask = self.elevation_grid != self.nodata
        valid_data = self.elevation_grid[self.valid_mask]
        
        if len(valid_data) == 0:
            raise ValueError("Map contains no valid data!")

        self.min_elev = np.min(valid_data)
        self.max_elev = np.max(valid_data)
        
        # Lat/Lon to Meters approximation
        # Assuming average latitude from filename or header. 
        # GEBCO header usually has xllcorner (lon) and yllcorner (lat).
        # But yllcorner is bottom-left. Average lat is yllcorner + (nrows*cellsize)/2
        lat_min = self.header['yllcorner']
        avg_lat = lat_min + (self.nrows * self.cellsize) / 2.0
        
        self.resolution_y = 111320.0 * self.cellsize # 1 deg lat approx 111.32 km
        self.resolution_x = 111320.0 * self.cellsize * math.cos(math.radians(avg_lat))

        print(f"Map Loaded: {self.ncols}x{self.nrows}")
        print(f"Resolution: {self.resolution_x:.2f}m x {self.resolution_y:.2f}m per cell")
        print(f"Depth Range: {self.min_elev:.2f}m to {self.max_elev:.2f}m")

    def generate_heightmap_png(self, output_path=None):
        """
        Generates a 16-bit PNG for Gazebo.
        Resizes to 257x257 (2^n + 1) to avoid Gazebo errors.
        """
        if output_path is None:
            output_path = self.file_path.replace('.asc', '.png')
        
        # Target size for Gazebo (Must be 2^n + 1)
        TARGET_SIZE = 513
        
        # Fill nodata
        filled_grid = self.elevation_grid.copy()
        filled_grid[~self.valid_mask] = self.min_elev
        
        # Normalize to 0-1.0
        normalized = (filled_grid - self.min_elev) / (self.max_elev - self.min_elev)
        
        # Resize using Cubic interpolation for smoothness
        # cv2.resize takes (width, height)
        resized_grid = cv2.resize(normalized, (TARGET_SIZE, TARGET_SIZE), interpolation=cv2.INTER_CUBIC)
        
        # Scale to 16-bit
        png_data = (resized_grid * 65535).astype(np.uint16)
        
        cv2.imwrite(output_path, png_data)
        print(f"Heightmap saved to {output_path} (Resized to {TARGET_SIZE}x{TARGET_SIZE})")
        
        # Calculate Physical Size
        # We want the physical size of the terrain to remain the same as the original data.
        # Even though we resized the image, the <size> tag in Gazebo defines the physical extent.
        real_width = self.ncols * self.resolution_x
        real_height = self.nrows * self.resolution_y
        real_depth = self.max_elev - self.min_elev
        
        return {
            'filename': output_path,
            'size': [real_width, real_height, real_depth],
            'pos': [0, 0, self.min_elev + real_depth/2.0]
        }

    def get_elevation(self, x_metric, y_metric):
        """
        Bilinear interpolation for elevation at metric coordinates (x, y).
        Coordinate system: Origin (0,0) is at the CENTER of the map (Gazebo style).
        """
        # Convert metric (center origin) to grid indices (top-left origin)
        # Gazebo X (East) -> Grid Col
        # Gazebo Y (North) -> Grid Row (inverted usually for images, check ASC order)
        
        # ASC Data order: usually Row 0 is Top (North), Row N is Bottom (South).
        # So Y increases upwards (North), Row index increases downwards.
        
        real_w = self.ncols * self.resolution_x
        real_h = self.nrows * self.resolution_y
        
        # Map Center (0,0) corresponds to:
        # Col = ncols / 2
        # Row = nrows / 2
        
        u = (x_metric + real_w / 2.0) / self.resolution_x
        v = (real_h / 2.0 - y_metric) / self.resolution_y # Inverted Y for image coords
        
        # Check bounds
        if u < 0 or u >= self.ncols - 1 or v < 0 or v >= self.nrows - 1:
            return self.min_elev # Return default or handle out of bounds
            
        # Bilinear Interpolation
        x0 = int(u)
        y0 = int(v)
        x1 = x0 + 1
        y1 = y0 + 1
        
        dx = u - x0
        dy = v - y0
        
        h00 = self.elevation_grid[y0, x0]
        h10 = self.elevation_grid[y0, x1]
        h01 = self.elevation_grid[y1, x0]
        h11 = self.elevation_grid[y1, x1]
        
        return (1 - dy) * ((1 - dx) * h00 + dx * h10) + dy * ((1 - dx) * h01 + dx * h11)

if __name__ == "__main__":
    # Test script
    import sys
    asc_path = "src/uuv_eskf_nav/gebco_2025_n40.4838_s39.4745_w131.501_e132.4997.asc"
    if len(sys.argv) > 1:
        asc_path = sys.argv[1]
        
    server = TerrainMapServer(asc_path)
    params = server.generate_heightmap_png()
    
    print("\n--- Gazebo Configuration ---")
    print(f"<size>{params['size'][0]:.2f} {params['size'][1]:.2f} {params['size'][2]:.2f}</size>")
    print(f"<pos>0 0 {server.min_elev + params['size'][2]/2.0:.2f}</pos>") 
    # NOTE: Gazebo heightmap Z pos moves the CENTER of the heightmap.
    # If min is -1000 and max is -500, height is 500. 
    # Center of model is at 250 (relative to its bottom).
    # We want bottom to be at min_elev.
