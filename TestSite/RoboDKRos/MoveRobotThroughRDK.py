from robodk.robolink import *
from robodk.robomath import *
import robolink
import CurvePlotter 
import numpy as np
import time

# --- Connect to RoboDK ---
RDK = Robolink()
robot = RDK.Item('UR3e', ITEM_TYPE_ROBOT)

# --- Try to connect to the real robot ---
connected = False
if connected:
    try:
        # Try to connect to the robot
        #robot.Connect()
        status, status_msg = robot.ConnectedState()
        if status == ROBOTCOM_READY:
            connected = True
            RDK.setRunMode(RUNMODE_RUN_ROBOT)
            print("✅ Connected to the real robot.")
        else:
            RDK.setRunMode(RUNMODE_SIMULATE)
            print("⚠️ Could not connect to the real robot. Running in simulation mode.")
    except Exception as e:
        RDK.setRunMode(RUNMODE_SIMULATE)
        print(f"⚠️ Connection failed: {e}\nRunning in simulation mode.")

# --- Common settings ---
#RDK.setSimulationSpeed(1)
#RDK.setCollisionActive(COLLISION_ON)

test = first = transl(-100, -300, 600)
print(test)
robot.MoveJ(test, blocking=True)



"""
print("-" * 50)


# --- Safe move wrapper ---
def safe_move(robot: Item, target: Mat, move_type: str = 'J'):
    if type(target) == Mat:
        pass
    elif type(target) is list:
        target = robot.SolveFK(target)

    move_type = move_type.upper()
    if move_type not in ['J', 'L', 'C']:
        raise ValueError("Invalid move_type. Use 'J', 'L', or 'C'.")

    joints_current = robot.Joints()
    joints_target = robot.SolveIK(target)

    if not joints_target:
        print("❌ No IK solution found!")
        return False

    # Path test before execution
    if move_type == 'J':
        test_failed = robot.MoveJ_Test(joints_current, joints_target)
    elif move_type == 'L':
        for joints in joints_target:
            pose_target = robot.SolveFK(joints)
            test_failed = robot.MoveL_Test(joints_current, pose_target)
            if test_failed == -1:
                print("❌ No valid path found for linear move, switching to joint move.")
                safe_move(robot, target, 'J')
                return True
    else:
        print("MoveC not implemented for poses")
        return False

    if test_failed != 0:
        print("⚠️ No valid path found or collision predicted! Move aborted.")
        robot.setJoints(joints_current)  # Reset to current joints after test
        return False

    # Execute move
    robot.setJoints(joints_current)  # Reset to current joints after test
    if move_type == 'J':
        robot.MoveJ(joints_target, blocking=True)
    elif move_type == 'L':
        robot.MoveL(joints_target, blocking=True)

    print("✅ Move completed successfully.")
    return True


# --- Example poses ---
first = transl(-100, -300, 600)
second = transl(100, -300, 500)
third = transl(100, -300, 400)




#print("Moving to first pose...")
#safe_move(robot, first, 'J')

#print("Moving to joint pose...")
#safe_move(robot, third, 'L')


#get all objects in the station
all_items = RDK.ItemList(list_names=True)

#pts = CurvePlotter.build_curve_with_normals(300, -600, -100, L=200, S=40, N=6, rot_deg=180, tilt_deg=0)
#print(pts)

#if 'Curve' in all_items:
    #RDK.Item('Curve').Delete()

#RDK.AddCurve(pts).setName("Curve")

#select the curve '6DOF_ScanPattern_RZ45deg' in robodk

#get first point for Curve 

def get_global_curve_poses(curve_item):
    Converts all XYZijk curve points into Pose matrices in the parent frame.
    Returns a list of global Pose() matrices, with inverted Z direction.

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

global_poses = get_global_curve_poses(RDK.Item('Curve'))

#set joints home 0.000000, -90.000000, -90.000000, 0.000000, 90.000000, 0.000000
robot.setJoints([0, -90, -90, 0, 90, 0])

robot.MoveJ(first, blocking=True)


valid_joint_solutions = []

for pose in global_poses:
    RDK.Render(False)
    # Solve IK for this pose
    targetJoints_list = robot.SolveIK_All(pose)
    currentJoints = robot.Joints()
    
    for joint in targetJoints_list:
        # Check if the move would cause a collision
        colres = robot.MoveJ_Test(currentJoints, joint)
        if colres == 0:  # No collision
            valid_joint_solutions.append(joint)
            robot.setJoints(currentJoints)
            RDK.Render(True)
            robot.MoveJ(joint, blocking=True)
            break  # Found a valid one, skip the rest

#for joint in valid_joint_solutions:
#    print("Moving to joint solution:", joint)
#    robot.MoveJ(joint, blocking=True)


#reset to home

#for pose in global_poses:
#    print("Moving to pose:", pose) 
#    safe_move(robot, pose, 'J')
#    #robot.MoveJ(pose, blocking=True)



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
    robot.Disconnect()
    print("✅ Disconnected.")
