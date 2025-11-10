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
    candidates = rdk.ItemList(ITEM_TYPE_OBJECT)
    for c in candidates:
        if name.lower() in c.Name().lower():
            # Make sure it actually has curves
            linked_curves = c.ObjectLink()
            if len(linked_curves) > 0 or c.Valid():
                print(f"✅ Found valid curve object: {c.Name()}")
                return c
    raise Exception(f"❌ No valid curve geometry found matching '{name}'")

curve_object = find_valid_curve(RDK, '6DOF_ScanPattern_RZ45deg')

if not curve_object.Valid():
    raise Exception("Curve object '6DOF_ScanPattern_RZ45deg' not found!")

# Create or get the Curve Follow project
curve_follow_project = RDK.Item('CurveFollow', ITEM_TYPE_PROGRAM)
if not curve_follow_project.Valid():
    curve_follow_project = RDK.AddMachiningProject("CurveFollow")

#---------------------------------
# Configure the curve follow project
#---------------------------------
# This sets the path parameters for the curve-follow project automatically
prog, status = curve_follow_project.setMachiningParameters(part=curve_object, params="ReorderAuto=0")

# Link the robot to the project
curve_follow_project.setLink(robot)

# Optionally set the tool and reference frame if needed:
# curve_follow_project.setPoseTool(robot.PoseTool())
# curve_follow_project.setPoseFrame(robot.PoseFrame())

#---------------------------------
# Simulate first in RoboDK
#---------------------------------
print("Simulating curve follow...")
curve_follow_project.RunProgram()  # runs the project in simulation mode
while curve_follow_project.Busy():
    time.sleep(0.1)

#---------------------------------
# Run the same path on the real robot
#---------------------------------
print("Running on real robot...")
robot.Connect()  # make sure connection is alive
curve_follow_project.RunCode('', True)  # run directly on connected robot

print("Done.")
