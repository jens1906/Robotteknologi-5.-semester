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

<<<<<<< Updated upstream
# --- Common settings ---
RDK.setSimulationSpeed(1)
#RDK.setCollisionActive(COLLISION_ON)
=======
#-------------------------------------------
# Optional: move robot to a start position
#-------------------------------------------
joint_target = [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
#robot.MoveL(joint_target)
>>>>>>> Stashed changes

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

<<<<<<< Updated upstream

# --- Example poses ---
first = transl(-100, -300, 600)
second = transl(100, -300, 500)
third = transl(100, -300, 400)

#get all objects in the station
all_items = RDK.ItemList(list_names=True)

"""
pts = CurvePlotter.build_curve_with_normals(300, -600, -100, L=200, S=40, N=6, rot_deg=180, tilt_deg=0)
print(pts)

if 'Curve' in all_items:
    RDK.Item('Curve').Delete()

RDK.AddCurve(pts).setName("Curve")
"""

def get_global_curve_poses(curve_item):
    """
    Converts all XYZijk curve points into Pose matrices in the parent frame.
    Returns a list of global Pose() matrices, with inverted Z direction.
    """
    pose_frame = curve_item.Pose()
    points, _ = curve_item.GetPoints(FEATURE_CURVE)

    poses_global = []

    for p in points:
        x, y, z, i, j, k = p

        # --- Invert the Z-axis direction ---
        z_axis = normalize3([-i, -j, -k])

        # Pick a reference axis
        ref = [1, 0, 0]
        if abs(dot(z_axis, ref)) > 0.99:
            ref = [0, 1, 0]

        # Orthogonal axes
        x_axis = normalize3(cross(ref, z_axis))
        y_axis = cross(z_axis, x_axis)

        # Build local pose matrix
        pose_local = Mat([
            [x_axis[0], y_axis[0], z_axis[0], x],
            [x_axis[1], y_axis[1], z_axis[1], y],
            [x_axis[2], y_axis[2], z_axis[2], z],
            [0, 0, 0, 1]
        ])

        # Transform to global coordinates
        pose_global = pose_frame * pose_local
        poses_global.append(pose_global)

    return poses_global

global_poses = get_global_curve_poses(RDK.Item('Curve'))[0:10]

#replace the rotation part of currentpose with rot_matrix
currentpose = Mat(robot.Pose())
currentpose[0:3, 0:3] = first[0:3, 0:3]
neworientation = currentpose

target_joints = robot.SolveIK_All(neworientation)
currentjoints = robot.Joints()
for joint in target_joints:
    col = robot.MoveJ_Test(currentjoints, joint)
    robot.setJoints(currentjoints)
    if col == 0:
        print("Found valid joint solution:", joint)
        break
robot.MoveJ(joint, blocking=True)
robot.MoveL(first, blocking=True)



start = time.time()
valid_joint_solutions = []

# Start from the robot's current position
totalstartjoing = robot.Joints()

for pose in global_poses:
    #print("Processing pose:", pose)
    #robot.MoveL(pose, blocking=True)
    break
    



end = time.time()

print(f"Found {len(valid_joint_solutions)} valid joint solutions in {end - start:.2f} seconds.")

#for joint in valid_joint_solutions:
#    print("Moving to joint solution:", joint)
#    robot.MoveJ(joint, blocking=True)


#reset to home

#for pose in global_poses:
#    print("Moving to pose:", pose) 
#    safe_move(robot, pose, 'J')
#    #robot.MoveJ(pose, blocking=True)


"""
Curve = RDK.Item('Curve')

path_settings = RDK.AddMachiningProject("AutoCurveFollow settings")
path_settings.setSpeed(20) #set speed to 100 mm/s

MachiningUpdate = {
    "VisibleNormals": 0,
    "AutoUpdate": 1,          # auto update path after changes
    "AvoidCollisions": 1,     # enable collision avoidance
    "Algorithm": 1,           # allow tool orientation changes
    "FollowAngleOn": 1,
    "FollowAngle": 45,        # degrees allowed to follow path
    "FollowRealignOn": 1,
    "FollowRealign": 10,      # degrees to realign if collision
    "RotZ_Range": 180,        # allow rotation around tool axis
    "RotZ_Step": 20,          # step for rotation search
    "SpeedOperation": 50,
    "SpeedRapid": 1000,
    "PointApproach": 20,
    "RapidApproachRetract": 1
}
status = path_settings.setParam("Machining", MachiningUpdate)

# --- Program events (optional) ---
ProgEventsUpdate = {
    "ToolChange": "ChangeTool(%1)",
    "CallPathStart": "ArcStart(1)",
    "CallPathStartOn": 1,
    "CallPathFinish": "ArcEnd()",
    "CallPathFinishOn": 1
}
status = path_settings.setParam("ProgEvents", ProgEventsUpdate)

# --- Generate robot program ---
prog, status = path_settings.setMachiningParameters(
    part=Curve,
    params="ReorderAuto=1"
)

# --- Get the actual robot program ---
robot_prog = path_settings.getLink(robolink.ITEM_TYPE_PROGRAM)

prog.RunCode()
"""



# --- Disconnect if connected to real robot ---
if connected:
    print("🔌 Disconnecting from real robot...")
    #robot.Disconnect()
    print("✅ Disconnected.")
=======
print("Done.")
>>>>>>> Stashed changes
