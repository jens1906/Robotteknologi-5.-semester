"""
RoboDK STL Loader - Load IdealRover.stl into RoboDK
"""

from robodk import robolink
import os

RDK = robolink.Robolink()

def load_ideal_rover(name="IdealRover", position=None, scale_factor=0.1):
    """
    Load the IdealRover.stl file into RoboDK workspace
    
    Args:
        name (str): Name for the rover object in RoboDK
        position (list): [x, y, z] position in mm (optional)
    
    Returns:
        Item: RoboDK item object for the loaded rover
    """
    # Path to the STL file (relative to this script)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    stl_path = os.path.join(current_dir, "IdealRover.stl")
    
    # Check if STL file exists
    if not os.path.exists(stl_path):
        print(f"✗ Error: STL file not found at {stl_path}")
        return None
    
    # Remove old object if it exists
    item = RDK.Item(name, robolink.ITEM_TYPE_OBJECT)
    if item.Valid():
        RDK.Delete(item)
        print(f"Removed existing {name}")
    
    print(f"Loading STL file: {stl_path}")
    
    try:
        # Load the STL file into RoboDK
        rover_item = RDK.AddFile(stl_path)
        
        if rover_item.Valid():
            rover_item.setName(name)
            # Apply uniform scale if factor != 1.0 (only on fresh load)
            if scale_factor and abs(scale_factor - 1.0) > 1e-6:
                try:
                    rover_item.Scale([scale_factor, scale_factor, scale_factor])
                    print(f"✓ Scaled {name} by factor {scale_factor}")
                except Exception as e:
                    print(f"⚠ Failed to scale rover: {e}")
            
            # Set position if provided
            if position:
                from robodk import robomath
                pose = robomath.transl(position[0], position[1], position[2])
                rover_item.setPose(pose)
                print(f"✓ Positioned rover at: {position} mm")
            
            print(f"✓ Success! Loaded {name} from STL file")
            print(f"  File: {os.path.basename(stl_path)}")
            return rover_item
        else:
            print("✗ Failed to load STL file into RoboDK")
            return None
            
    except Exception as e:
        print(f"✗ Error loading STL file: {e}")
        return None

def load_rover_and_cylinder():
    """
    Load both the IdealRover.stl and create a smooth cylinder for demonstration
    """
    print("=== Loading RoboDK Objects ===\n")
    
    # Load the rover
    print("1. Loading IdealRover.stl...")
    rover = load_ideal_rover("IdealRover", position=[0, 0, 0])
    
    # Import the cylinder function from RDK_CylGen
    print("\n2. Creating smooth cylinder...")
    try:
        from RDK_CylGen import make_smooth_cylinder
        cylinder = make_smooth_cylinder(0.1, 0.2, "DemoCylinder")
        
        if cylinder:
            # Position cylinder next to rover
            from robodk import robomath
            pose = robomath.transl(200, 0, 0)  # 200mm to the right
            cylinder.setPose(pose)
            print("✓ Positioned cylinder at [200, 0, 0] mm")
            
    except ImportError:
        print("Note: RDK_CylGen not available - skipping cylinder creation")
    except Exception as e:
        print(f"Error creating cylinder: {e}")
    
    print("\n=== Complete ===")
    if rover:
        print("✓ IdealRover loaded successfully")
    print("Objects are now ready in your RoboDK workspace!")

def get_stl_info():
    """
    Display information about the IdealRover.stl file
    """
    current_dir = os.path.dirname(os.path.abspath(__file__))
    stl_path = os.path.join(current_dir, "IdealRover.stl")
    
    if os.path.exists(stl_path):
        file_size = os.path.getsize(stl_path)
        print(f"STL File Information:")
        print(f"  Path: {stl_path}")
        print(f"  Size: {file_size:,} bytes ({file_size/1024:.1f} KB)")
        print(f"  Status: Ready to load")
    else:
        print(f"STL file not found: {stl_path}")

# Example usage
if __name__ == "__main__":
    print("RoboDK STL Loader - IdealRover\n")
    
    # Show file info
    get_stl_info()
    print()
    
    # Load the rover
    rover = load_ideal_rover("IdealRover")
    
    if rover:
        print("\nRover loaded successfully!")
        print("You can now see the IdealRover in your RoboDK workspace.")
        
        # Ask if user wants to load both rover and cylinder
        print("\nTo load both rover and cylinder, run:")
        print("load_rover_and_cylinder()")
    else:
        print("\nFailed to load rover. Please check:")
        print("1. RoboDK is running")
        print("2. IdealRover.stl exists in the same folder")
        print("3. File is not corrupted")