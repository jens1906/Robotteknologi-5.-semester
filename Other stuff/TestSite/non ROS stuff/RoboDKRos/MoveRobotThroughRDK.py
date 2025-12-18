from robodk.robolink import *
from robodk.robomath import *
import time

#-------------------------------------------
# Connect to RoboDK
#-------------------------------------------
RDK = Robolink()

# Get the robot (change the name if needed)
robot = RDK.Item('UR3e', ITEM_TYPE_ROBOT)

if not robot.Valid():
    print("Robot not found! Make sure RoboDK is running and the robot is loaded.")
    print("Available robots:")
    for item in RDK.ItemList(ITEM_TYPE_ROBOT):
        print(f"  - {item.Name()}")
    exit()
else:
    robot.Connect()
    print(f"Connected to robot: {robot.Name()}")

#-------------------------------------------
# Optional: move robot to a start position
#-------------------------------------------
joint_target = [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
#robot.MoveL(joint_target)

#-------------------------------------------
# Move to a base pose if desired
#-------------------------------------------
target_pose = Mat([
    [1.0, 0.0, 0.0, 100],
    [0.0, 1.0, 0.0, -300],
    [0.0, 0.0, 1.0, 400.0],
    [0.0, 0.0, 0.0, 1.0]
])
#robot.MoveL(target_pose)

#---------------------------------
# Get the curve object and project
#---------------------------------
#---------------------------------
# Find the real curve object inside RoboDK
#---------------------------------
def find_valid_curve(rdk: Robolink, name: str):
    """Recursively search for a valid object containing curve geometry."""
    # Try to find the item directly first
    item = rdk.Item(name, ITEM_TYPE_OBJECT)
    if item.Valid():
        print(f"✅ Found object directly: {item.Name()}")
        return item

    # If not found directly, search through all objects
    candidates = rdk.ItemList(ITEM_TYPE_OBJECT)
    for c in candidates:
        if not c.Valid():
            continue
            
        try:
            c_name = c.Name()
        except:
            continue

        if name.lower() in c_name.lower():
            print(f"✅ Found object by partial name: {c_name}")
            return c

    raise Exception(f"❌ No valid curve geometry found matching '{name}'")

curve_object = find_valid_curve(RDK, 'Curve_with_correct_tilt')

if not curve_object.Valid():
    raise Exception("Curve object 'Curve_with_correct_tilt' not found!")

# ---------------------------------
# Count points in the curve
# ---------------------------------
try:
    # FEATURE_CURVE is usually 2
    points = curve_object.GetPoints(FEATURE_CURVE)
    # points is a list of lists of points (curves)
    total_points = 0
    for curve_points in points:
        total_points += len(curve_points)
    print(f"ℹ️ Curve object contains {len(points)} curves with a total of {total_points} points.")
except Exception as e:
    print(f"⚠️ Could not retrieve points from curve object: {e}")

# Create or get the Curve Follow project
curve_follow_project = RDK.Item('CurveFollow', ITEM_TYPE_MACHINING)
if not curve_follow_project.Valid():
    curve_follow_project = RDK.AddMachiningProject("CurveFollow")

#---------------------------------
# Configure the curve follow project
#---------------------------------
print("Generating program...")
start_gen_time = time.time()

# Enable collision checking globally
RDK.setCollisionActive(COLLISION_ON)

# Configure parameters for collision avoidance
# CheckCollisions=1: Enable collision checking
# RotZ_Range=180: Allow tool rotation around Z axis +/- 180 degrees to avoid collisions/singularities
# RotZ_Step=10: Step size for rotation search
machining_params = "ReorderAuto=0;CheckCollisions=1;RotZ_Range=180;RotZ_Step=10"

# This sets the path parameters for the curve-follow project automatically
prog, status = curve_follow_project.setMachiningParameters(part=curve_object, params=machining_params)

if not prog.Valid():
    print(f"⚠️ Warning: Program generation failed or incomplete. Status: {status}")
    # Try to update/generate
    curve_follow_project.Update()
    prog = curve_follow_project.getLink(ITEM_TYPE_PROGRAM)

end_gen_time = time.time()
print(f"⏱️ Program generation took {end_gen_time - start_gen_time:.4f} seconds.")

if prog.Valid():
    print(f"✅ Generated program: {prog.Name()}")
else:
    print("❌ Failed to generate program from curve follow project.")

# Link the robot to the project
curve_follow_project.setLink(robot)

# Update the project to ensure everything is calculated
print("Updating machining project...")
curve_follow_project.Update()

# Retrieve the program again just in case
prog = curve_follow_project.getLink(ITEM_TYPE_PROGRAM)

if not prog.Valid():
    print("❌ No valid program linked to the machining project! Cannot run.")
else:
    print(f"✅ Program linked to project: {prog.Name()}")
    
    #---------------------------------
    # Simulate in RoboDK
    #---------------------------------
    print("Simulating curve follow...")
    
    # Ensure we are in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    
    # Run the program, not the project, to avoid ambiguity
    prog.RunProgram()
    
    # Wait for the simulation to finish
    while prog.Busy():
        time.sleep(0.1)
        
    print("Simulation complete.")

print("Done.")



