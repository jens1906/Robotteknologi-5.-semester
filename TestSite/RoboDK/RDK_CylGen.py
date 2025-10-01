"""
Simple Ultra-Smooth Cylinder Creator for RoboDK
Just the essentials - no extra objects or experimental features
"""

from robodk import robolink
import math

RDK = robolink.Robolink()

def make_smooth_cylinder(diameter, height, name="SmoothCylinder"):
    """
    Create an ultra-smooth cylinder using 720-sided geometry
    
    Args:
        diameter (float): Cylinder diameter in meters
        height (float): Cylinder height in meters  
        name (str): Name for the cylinder object
    
    Returns:
        Item: RoboDK item object for the created cylinder
    """
    # Remove old object if it exists
    item = RDK.Item(name, robolink.ITEM_TYPE_OBJECT)
    if item.Valid():
        RDK.Delete(item)
    
    # Convert from meters to millimeters (RoboDK uses mm)
    diameter_mm = diameter * 1000
    height_mm = height * 1000
    
    # Ultra-high resolution for smooth appearance
    n_sides = 720  # 0.5 degrees per side = visually perfect
    radius = diameter_mm / 2
    
    print(f"Creating ultra-smooth cylinder: {name} ({diameter}m x {height}m = {diameter_mm}mm x {height_mm}mm, {n_sides} sides)")
    
    # Generate vertices
    triangle_vertices = []
    bottom_center = [0, 0, 0]
    top_center = [0, 0, height_mm]
    
    # Circle vertices
    bottom_vertices = []
    top_vertices = []
    
    for i in range(n_sides):
        angle = 2 * math.pi * i / n_sides
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        bottom_vertices.append([x, y, 0])
        top_vertices.append([x, y, height_mm])
    
    # Bottom face (fan from center)
    for i in range(n_sides):
        next_i = (i + 1) % n_sides
        triangle_vertices.append(bottom_center)
        triangle_vertices.append(bottom_vertices[i])
        triangle_vertices.append(bottom_vertices[next_i])
    
    # Top face (fan from center, reversed winding)
    for i in range(n_sides):
        next_i = (i + 1) % n_sides
        triangle_vertices.append(top_center)
        triangle_vertices.append(top_vertices[next_i])
        triangle_vertices.append(top_vertices[i])
    
    # Side walls
    for i in range(n_sides):
        next_i = (i + 1) % n_sides
        
        # Triangle 1
        triangle_vertices.append(bottom_vertices[i])
        triangle_vertices.append(bottom_vertices[next_i])
        triangle_vertices.append(top_vertices[i])
        
        # Triangle 2
        triangle_vertices.append(bottom_vertices[next_i])
        triangle_vertices.append(top_vertices[next_i])
        triangle_vertices.append(top_vertices[i])
    
    # Create the shape in RoboDK
    new_item = RDK.AddShape(triangle_vertices)
    
    if new_item.Valid():
        new_item.setName(name)
        print(f"✓ Success! Created {name} with {len(triangle_vertices) // 3} triangles")
        return new_item
    else:
        print("✗ Failed to create cylinder")
        return None

# Example usage
if __name__ == "__main__":
    # Clean, simple cylinder creation - now using meters as input!
    cylinder = make_smooth_cylinder(6, 10, "MySmoothCylinder")  # 6m diameter, 10m height
    
    if cylinder:
        print("Cylinder created successfully!")
        print("- Input in meters, automatically converted to mm for RoboDK")
        print("- Visually indistinguishable from perfect smoothness")
        print("- No extra objects or curves")
        print("- Ready to use in your RoboDK project")