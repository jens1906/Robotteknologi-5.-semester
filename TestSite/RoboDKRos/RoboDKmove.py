#!/usr/bin/env python3
"""
RoboDK Back-and-Forth Parallel Lines Path Generator
This script creates a scanning pattern with parallel lines connected by curves.
"""

from robodk.robolink import *    # RoboDK API
from robodk.robomath import *    # Robot toolbox
import math

def create_parallel_lines_path(start_x=100, start_y=100, z_height=100, 
                              start_rx=0, start_ry=0, start_rz=0,
                              line_length=200, line_spacing=50, num_lines=5, 
                              rotation_angle=0):
    """Create a back-and-forth parallel lines scanning pattern with rounded ends
    
    Args:
        start_x, start_y, z_height: Starting position coordinates
        start_rx, start_ry, start_rz: Starting orientation in degrees (Euler angles)
        line_length: Length of each parallel line
        line_spacing: Distance between parallel lines
        num_lines: Number of parallel lines in the pattern
        rotation_angle: Additional rotation angle for the path pattern in degrees
    """
    
    points = []
    
    for line_num in range(num_lines):
        # Calculate local Y position for this line (before rotation)
        local_y = line_num * line_spacing
        
        if line_num % 2 == 0:
            # Even lines: go from left to right (in local coordinates)
            # Add points along the line (for smooth movement)
            for i in range(21):  # 21 points per line for smoothness
                ratio = i / 20.0
                local_x = line_length * ratio
                
                # Apply rotation and translation with full 6-DOF
                rotated_point = rotate_and_translate_point_6dof(
                    local_x, local_y, 0, 0, 0, 0,  # local position and orientation
                    start_x, start_y, z_height, start_rx, start_ry, start_rz, 
                    rotation_angle
                )
                points.append(rotated_point)
            
            # Add rounded end (semicircle) at the right side - only if there's a next line
            if line_num < num_lines - 1:
                next_local_y = (line_num + 1) * line_spacing
                
                # Position the arc center so the semicircle connects current line to next line
                arc_center_local_x = line_length
                arc_center_local_y = (local_y + next_local_y) / 2.0
                
                # Calculate the actual radius needed to connect the two lines
                actual_radius = abs(next_local_y - local_y) / 2.0
                
                # Create semicircle from current line to next line
                start_angle = -90  # Start pointing downward from current line
                end_angle = 90     # End pointing downward to next line
                
                for i in range(1, 19):  # 18 points for semicircle
                    angle = start_angle + ((end_angle - start_angle) * i / 18.0)
                    local_x = arc_center_local_x + actual_radius * math.cos(math.radians(angle))
                    local_y_arc = arc_center_local_y + actual_radius * math.sin(math.radians(angle))
                    
                    # Apply rotation and translation with full 6-DOF
                    rotated_point = rotate_and_translate_point_6dof(
                        local_x, local_y_arc, 0, 0, 0, 0,
                        start_x, start_y, z_height, start_rx, start_ry, start_rz,
                        rotation_angle
                    )
                    points.append(rotated_point)
        
        else:
            # Odd lines: go from right to left (in local coordinates)
            # Add points along the line (right to left)
            for i in range(21):
                ratio = i / 20.0
                local_x = line_length - (line_length * ratio)
                
                # Apply rotation and translation with full 6-DOF
                rotated_point = rotate_and_translate_point_6dof(
                    local_x, local_y, 0, 0, 0, 0,
                    start_x, start_y, z_height, start_rx, start_ry, start_rz,
                    rotation_angle
                )
                points.append(rotated_point)
            
            # Add rounded end (semicircle) at the left side - only if there's a next line
            if line_num < num_lines - 1:
                next_local_y = (line_num + 1) * line_spacing
                
                # Position the arc center so the semicircle connects current line to next line
                arc_center_local_x = 0
                arc_center_local_y = (local_y + next_local_y) / 2.0
                
                # Calculate the actual radius needed to connect the two lines
                actual_radius = abs(next_local_y - local_y) / 2.0
                
                # Create semicircle from current line to next line
                start_angle = -90  # Start pointing downward from current line
                end_angle = 90     # End pointing downward to next line
                
                for i in range(1, 19):  # 18 points for semicircle
                    angle = start_angle + ((end_angle - start_angle) * i / 18.0)
                    local_x = arc_center_local_x - actual_radius * math.cos(math.radians(angle))
                    local_y_arc = arc_center_local_y + actual_radius * math.sin(math.radians(angle))
                    
                    # Apply rotation and translation with full 6-DOF
                    rotated_point = rotate_and_translate_point_6dof(
                        local_x, local_y_arc, 0, 0, 0, 0,
                        start_x, start_y, z_height, start_rx, start_ry, start_rz,
                        rotation_angle
                    )
                    points.append(rotated_point)
    
    print(points)
    return points

def rotate_and_translate_point_6dof(local_x, local_y, local_z, local_rx, local_ry, local_rz,
                                   start_x, start_y, start_z, start_rx, start_ry, start_rz,
                                   additional_rotation=0):
    """Apply 6-DOF transformation to a point with position and orientation"""
    
    # First apply the additional rotation around Z-axis (for pattern rotation)
    if additional_rotation != 0:
        angle_rad = math.radians(additional_rotation)
        rotated_x = local_x * math.cos(angle_rad) - local_y * math.sin(angle_rad)
        rotated_y = local_x * math.sin(angle_rad) + local_y * math.cos(angle_rad)
        local_x = rotated_x
        local_y = rotated_y
    
    # Create transformation matrix using RoboDK's pose functions
    local_pose = KUKA_2_Pose([local_x, local_y, local_z, local_rx, local_ry, local_rz])
    start_pose = KUKA_2_Pose([start_x, start_y, start_z, start_rx, start_ry, start_rz])
    
    # Apply transformation
    final_pose = start_pose * local_pose
    
    # Convert back to KUKA format [X, Y, Z, RX, RY, RZ]
    final_coords = Pose_2_KUKA(final_pose)
    
    return final_coords

def main():
    # Connect to RoboDK
    RDK = Robolink()
    
    # Check if RoboDK is running
    try:
        station = RDK.ActiveStation()
        if station is None:
            print("RoboDK is not running or no station is active. Please start RoboDK.")
            return
    except:
        print("RoboDK is not running. Please start RoboDK.")
        return
    
    print("Connected to RoboDK")
    
    # Parameters for the scanning pattern with full 6-DOF control
    start_x = 300        # Starting X coordinate
    start_y = -600        # Starting Y coordinate
    start_z = -100        # Starting Z coordinate
    start_rx = 180         # Starting RX rotation (degrees)
    start_ry = 0         # Starting RY rotation (degrees)
    start_rz = 45        # Starting RZ rotation (degrees)
    
    line_length = 200    # Length of each parallel line
    line_spacing = 40    # Distance between parallel lines
    num_lines = 6        # Number of parallel lines
    path_rotation = 0    # Additional rotation of the scanning pattern (degrees)
    
    print(f"Creating back-and-forth parallel lines pattern with full 6-DOF control:")
    print(f"  - Start position: X={start_x}, Y={start_y}, Z={start_z}")
    print(f"  - Start orientation: RX={start_rx}°, RY={start_ry}°, RZ={start_rz}°")
    print(f"  - Line length: {line_length}mm")
    print(f"  - Line spacing: {line_spacing}mm") 
    print(f"  - Number of lines: {num_lines}")
    print(f"  - Path rotation: {path_rotation}°")
    
    # Generate the scanning pattern points
    scan_points = create_parallel_lines_path(
        start_x, start_y, start_z, 
        start_rx, start_ry, start_rz,
        line_length, line_spacing, num_lines, path_rotation
    )
    
    print(f"Generated {len(scan_points)} points for scanning pattern")
    
    # Create the curve in RoboDK
    curve = RDK.AddCurve(scan_points)
    curve.setName(f"6DOF_ScanPattern_RZ{start_rz}deg")
    
    if curve and curve.Valid():
        print(f"\n=== 6-DOF Scanning Pattern Created ===")
        print(f"✓ Successfully created curve: {curve.Name()}")
        print(f"✓ Pattern type: 6-DOF oriented parallel lines with rounded ends")
        print(f"✓ Total points: {len(scan_points)}")
        print("✓ Curve is now available in RoboDK station")
        
        # Print the pattern description
        print(f"\nPattern details:")
        print(f"  - Full 6-DOF start pose: ({start_x}, {start_y}, {start_z}, {start_rx}°, {start_ry}°, {start_rz}°)")
        print(f"  - {num_lines} parallel lines with {line_length}mm length each")
        print(f"  - {line_spacing}mm spacing between lines")
        print(f"  - Additional path rotation: {path_rotation}°")
        
        print(f"\nOrientation control:")
        print(f"  - RX: Rotation around X-axis (roll)")
        print(f"  - RY: Rotation around Y-axis (pitch)")
        print(f"  - RZ: Rotation around Z-axis (yaw)")
        
    else:
        print("Failed to create scanning pattern curve")

if __name__ == "__main__":
    main()