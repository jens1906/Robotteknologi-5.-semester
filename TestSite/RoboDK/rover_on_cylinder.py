"""
RoboDK Rover on Cylinder Setup
Creates a cylinder, loads the IdealRover, and positions the rover on the cylinder's surface
"""

from robodk import robolink, robomath
import math
import os

# Import our custom functions
from RDK_CylGen import make_smooth_cylinder
from load_rover import load_ideal_rover

RDK = robolink.Robolink()

def position_rover_on_cylinder(rover_item, cylinder_diameter_m, cylinder_height_m, angle_deg=0, height_position=0.5, rover_offset_mm=200):
    """
    Position the rover on the outside surface of a cylinder with proper orientation
    
    Args:
        rover_item: RoboDK item object for the rover
        cylinder_diameter_m (float): Cylinder diameter in meters
        cylinder_height_m (float): Cylinder height in meters
        angle_deg (float): Angle around cylinder (0-360 degrees)
        height_position (float): Height position (0.0=bottom, 1.0=top)
        rover_offset_mm (float): Distance from rover center to bottom surface (mm)
    
    Returns:
        bool: Success status
    """
    if not rover_item or not rover_item.Valid():
        print("✗ Invalid rover item")
        return False
    
    # Convert to mm and calculate position
    cylinder_radius_mm = (cylinder_diameter_m * 1000) / 2
    height_mm = cylinder_height_m * 1000 * height_position
    
    # Calculate position on cylinder surface + offset for rover bottom to be tangent
    angle_rad = math.radians(angle_deg)
    surface_radius_mm = cylinder_radius_mm + rover_offset_mm
    
    x = surface_radius_mm * math.cos(angle_rad)
    y = surface_radius_mm * math.sin(angle_rad)
    z = height_mm
    
    # Create proper orientation for rover on cylinder surface
    # The rover should be oriented so its bottom is tangent to the cylinder
    
    # Calculate the angle to tilt the rover so its bottom surface is tangent to cylinder
    # This is the angle between the rover's center-to-surface vector and the vertical
    tilt_angle_rad = math.atan2(rover_offset_mm, cylinder_radius_mm)
    tilt_angle_deg = math.degrees(tilt_angle_rad)
    
    # Create rotation matrix for proper orientation
    # First rotate around Z to face the correct direction around the cylinder
    rotation_z = robomath.rotz(angle_deg + 90)  # +90 to align rover forward direction
    
    # Then tilt around the local Y-axis to make bottom tangent to cylinder surface
    rotation_y = robomath.roty(-tilt_angle_deg)
    
    # Combine position and orientation
    pose = robomath.transl(x, y, z) * rotation_z * rotation_y
    
    # Apply the pose to the rover
    rover_item.setPose(pose)
    
    print(f"✓ Positioned rover on cylinder surface:")
    print(f"  Position: [{x:.1f}, {y:.1f}, {z:.1f}] mm")
    print(f"  Angle: {angle_deg}°, Height: {height_position*100:.1f}%")
    print(f"  Surface radius: {surface_radius_mm:.1f}mm, Tilt: {tilt_angle_deg:.1f}°")
    print(f"  Rover offset: {rover_offset_mm}mm from center to bottom")
    
    return True

def adjust_rover_positioning_test(cylinder_diameter=6.0, cylinder_height=10.0):
    """
    Test different rover offset values to find the best positioning
    """
    print("=== Rover Positioning Test ===\n")
    
    # Create cylinder
    cylinder = make_smooth_cylinder(cylinder_diameter, cylinder_height, "PositionTestCylinder")
    if not cylinder:
        return
    
    # Test different offset values
    test_offsets = [100, 150, 200, 250, 300]  # mm
    
    for i, offset in enumerate(test_offsets):
        rover_name = f"TestRover_Offset{offset}"
        print(f"\nTesting rover with {offset}mm offset...")
        
        # Load rover
        rover = load_ideal_rover(rover_name)
        if rover:
            # Position at different angles for each test
            angle = i * 72  # Distribute around cylinder
            position_rover_on_cylinder(rover, cylinder_diameter, cylinder_height, 
                                     angle, 0.5, offset)
    
    print(f"\n✓ Created {len(test_offsets)} test rovers with different offsets")
    print("Check RoboDK to see which positioning looks best!")

def create_rover_on_cylinder_scene(cylinder_diameter=6.0, cylinder_height=10.0, 
                                 rover_angle=0, rover_height_ratio=0.5,
                                 cylinder_name="MainCylinder", rover_name="SurfaceRover"):
    """
    Create complete scene with cylinder and rover positioned on its surface
    
    Args:
        cylinder_diameter (float): Cylinder diameter in meters
        cylinder_height (float): Cylinder height in meters
        rover_angle (float): Angle around cylinder for rover placement (degrees)
        rover_height_ratio (float): Height position on cylinder (0.0-1.0)
        cylinder_name (str): Name for cylinder object
        rover_name (str): Name for rover object
    
    Returns:
        tuple: (cylinder_item, rover_item) or (None, None) if failed
    """
    print("=== Creating Rover on Cylinder Scene ===\n")
    
    # Step 1: Create the cylinder
    print("1. Creating ultra-smooth cylinder...")
    cylinder_item = make_smooth_cylinder(cylinder_diameter, cylinder_height, cylinder_name)
    
    if not cylinder_item:
        print("✗ Failed to create cylinder")
        return None, None
    
    # Step 2: Load the rover
    print("\n2. Loading IdealRover.stl...")
    rover_item = load_ideal_rover(rover_name)
    
    if not rover_item:
        print("✗ Failed to load rover")
        return cylinder_item, None
    
    # Step 3: Position rover on cylinder surface
    print("\n3. Positioning rover on cylinder surface...")
    success = position_rover_on_cylinder(rover_item, cylinder_diameter, cylinder_height, 
                                       rover_angle, rover_height_ratio, rover_offset_mm=200)
    
    if success:
        print("\n=== Scene Complete! ===")
        print(f"✓ Cylinder: {cylinder_diameter}m × {cylinder_height}m")
        print(f"✓ Rover positioned at {rover_angle}° angle, {rover_height_ratio*100:.1f}% height")
        print("✓ Ready for simulation!")
    else:
        print("✗ Failed to position rover")
    
    return cylinder_item, rover_item

def create_multiple_rover_positions(cylinder_diameter=6.0, cylinder_height=10.0, num_rovers=4):
    """
    Create a cylinder with multiple rovers positioned around it
    
    Args:
        cylinder_diameter (float): Cylinder diameter in meters
        cylinder_height (float): Cylinder height in meters
        num_rovers (int): Number of rovers to place around cylinder
    
    Returns:
        tuple: (cylinder_item, list_of_rover_items)
    """
    print(f"=== Creating Cylinder with {num_rovers} Rovers ===\n")
    
    # Create cylinder
    cylinder_item = make_smooth_cylinder(cylinder_diameter, cylinder_height, "MultiRoverCylinder")
    
    if not cylinder_item:
        return None, []
    
    rover_items = []
    angle_step = 360.0 / num_rovers
    
    # Create rovers at different angles
    for i in range(num_rovers):
        angle = i * angle_step
        height_ratio = 0.3 + (i * 0.4 / (num_rovers - 1))  # Distribute heights from 30% to 70%
        
        print(f"\nCreating rover {i+1}/{num_rovers} at {angle:.1f}°...")
        rover_item = load_ideal_rover(f"Rover_{i+1}")
        
        if rover_item:
            position_rover_on_cylinder(rover_item, cylinder_diameter, cylinder_height, 
                                     angle, height_ratio, rover_offset_mm=200)
            rover_items.append(rover_item)
        else:
            print(f"✗ Failed to create rover {i+1}")
    
    print(f"\n✓ Created {len(rover_items)}/{num_rovers} rovers successfully!")
    return cylinder_item, rover_items

def animate_rover_around_cylinder(rover_item, cylinder_diameter_m, cylinder_height_m, 
                                height_ratio=0.5, num_positions=36):
    """
    Create animation positions for rover moving around cylinder with proper surface tangency
    
    Args:
        rover_item: RoboDK rover item
        cylinder_diameter_m (float): Cylinder diameter in meters
        cylinder_height_m (float): Cylinder height in meters
        height_ratio (float): Height position on cylinder
        num_positions (int): Number of positions for animation
    
    Returns:
        list: List of poses for animation
    """
    if not rover_item or not rover_item.Valid():
        return []
    
    poses = []
    angle_step = 360.0 / num_positions
    
    print(f"Generating {num_positions} animation positions with proper surface tangency...")
    
    # Same calculations as in position_rover_on_cylinder
    cylinder_radius_mm = (cylinder_diameter_m * 1000) / 2
    rover_offset_mm = 150  # Same offset as positioning function
    surface_radius_mm = cylinder_radius_mm + rover_offset_mm
    height_mm = cylinder_height_m * 1000 * height_ratio
    tilt_angle = math.degrees(math.atan2(rover_offset_mm, cylinder_radius_mm))
    
    for i in range(num_positions):
        angle = i * angle_step
        angle_rad = math.radians(angle)
        
        # Calculate position with proper offset
        x = surface_radius_mm * math.cos(angle_rad)
        y = surface_radius_mm * math.sin(angle_rad)
        z = height_mm
        
        # Create pose with proper orientation (tangent to surface)
        rotation_z = robomath.rotz(angle + 90)
        rotation_y = robomath.roty(-tilt_angle)
        pose = robomath.transl(x, y, z) * rotation_z * rotation_y
        
        poses.append(pose)
    
    print(f"✓ Generated {len(poses)} animation poses with surface tangency")
    return poses

# Example usage and demonstrations
if __name__ == "__main__":
    print("RoboDK Rover on Cylinder Setup\n")
    
    # Demonstrate different setups
    choice = input("Choose setup:\n1. Single rover on cylinder\n2. Multiple rovers\n3. Animation demo\n4. Test different rover positions\nEnter choice (1-4): ").strip()
    
    if choice == "1":
        # Single rover setup
        print("\n--- Single Rover Setup ---")
        cylinder, rover = create_rover_on_cylinder_scene(
            cylinder_diameter=6.0,
            cylinder_height=10.0,
            rover_angle=45,
            rover_height_ratio=0.6,
            cylinder_name="TestCylinder",
            rover_name="TestRover"
        )
        
    elif choice == "2":
        # Multiple rovers
        print("\n--- Multiple Rovers Setup ---")
        cylinder, rovers = create_multiple_rover_positions(
            cylinder_diameter=8.0,
            cylinder_height=12.0,
            num_rovers=6
        )
        
    elif choice == "3":
        # Animation demo
        print("\n--- Animation Demo Setup ---")
        cylinder, rover = create_rover_on_cylinder_scene(6.0, 10.0, 0, 0.5, "AnimCylinder", "AnimRover")
        
        if rover:
            poses = animate_rover_around_cylinder(rover, 6.0, 10.0, 0.5, 36)
            print(f"Animation ready with {len(poses)} positions!")
            print("Use the poses list to animate the rover in RoboDK")
    
    elif choice == "4":
        # Position testing
        print("\n--- Rover Position Testing ---")
        adjust_rover_positioning_test(6.0, 10.0)
            
    else:
        # Default demo
        print("\n--- Default Demo ---")
        cylinder, rover = create_rover_on_cylinder_scene()
    
    print("\n=== Setup Complete! ===")
    print("Your RoboDK workspace now contains the rover positioned on the cylinder surface.")