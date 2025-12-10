#!/usr/bin/env python3
import numpy as np

def check_gradient():
    # Load map
    data = np.loadtxt('gazebo_terrain.asc', skiprows=6)
    # Generate map logic uses np.flipud, so row 0 is Top (y=500)
    # If saved without flipud, row 0 is Bottom (y=-500).
    # Assuming current asc matches generate_map.py's output.
    
    # Map params
    real_width = 1000.0
    nrows, ncols = data.shape
    cell_size = real_width / (ncols - 1)
    
    # Target Point
    px, py = 193.21, 349.87
    
    # Convert to Grid indices
    # Assuming center at (500, 500) in grid coordinates if map is 0..1000
    # Center of map is (0,0) physically.
    # Grid X: -500 -> 0, 500 -> 256
    # col = (x + 500) / 1000 * 256
    col = int((px + 500.0) / 1000.0 * (ncols - 1))
    
    # Grid Y: 
    # If flipud was used: row 0 is y=500. row = (500 - y) / 1000 * 256
    row = int((500.0 - py) / 1000.0 * (nrows - 1))
    
    # 1. Calculate Terrain Gradient (dz/dx, dz/dy)
    # Simple central difference
    z_x_plus = data[row, min(col+1, ncols-1)]
    z_x_minus = data[row, max(col-1, 0)]
    dz_dx = (z_x_plus - z_x_minus) / (2 * cell_size)
    
    z_y_plus = data[max(row-1, 0), col] # Row-1 is Up (y+)
    z_y_minus = data[min(row+1, nrows-1), col] # Row+1 is Down (y-)
    dz_dy = (z_y_plus - z_y_minus) / (2 * cell_size)
    
    grad_mag = np.sqrt(dz_dx**2 + dz_dy**2)
    grad_dir = np.arctan2(dz_dy, dz_dx) # Rad (-pi, pi)
    
    # 2. Calculate AUV Heading (Tangential to circle)
    # Counter-Clockwise circle. Tangent at (x,y) is (-y, x)
    vel_x = -py
    vel_y = px
    heading = np.arctan2(vel_y, vel_x)
    
    # 3. Calculate Angle between Gradient and Heading
    # Ideally we want them to be parallel (0 or 180 deg) for max observability along path.
    # If they are perpendicular (90 deg), we have zero observability along path.
    diff_angle = np.abs(grad_dir - heading)
    if diff_angle > np.pi: diff_angle = 2*np.pi - diff_angle
    
    # Convert to degrees for display
    diff_deg = np.degrees(diff_angle)
    
    print("-" * 30)
    print(f"Analysis at ({px:.2f}, {py:.2f})")
    print(f"Terrain Gradient: ({dz_dx:.3f}, {dz_dy:.3f}), Mag: {grad_mag:.3f}")
    print(f"AUV Heading Vec:  ({vel_x:.1f}, {vel_y:.1f})")
    print(f"Angle difference: {diff_deg:.1f} degrees")
    print("-" * 30)
    
    if grad_mag < 0.1:
        print(">> CONCLUSION: Terrain is FLAT. Zero observability.")
    elif 70 < diff_deg < 110:
        print(">> CONCLUSION: Gradient is PERPENDICULAR to motion.")
        print(">> AUV is moving along a ridge/valley (Contour line).")
        print(">> Tangential observability is POOR.")
    else:
        print(">> CONCLUSION: Gradient has component along motion.")
        print(">> Observability should be OK.")

if __name__ == "__main__":
    check_gradient()
