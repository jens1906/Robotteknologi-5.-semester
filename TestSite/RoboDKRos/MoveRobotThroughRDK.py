from robodk.robolink import *
from robodk.robomath import *
import robolink
import CurvePlotter 
import numpy as np
import time

time.sleep(2)

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
RDK.setSimulationSpeed(2)
RDK.setCollisionActive(COLLISION_ON)

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

    if test_failed:
        print("⚠️ No valid path found or collision predicted! Move aborted.")
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
Curve = RDK.Item('Curve')
points = Curve.GetPoints(FEATURE_CURVE)[0]
print(points[0])
print(points[1])

print("********")

for point in Curve.GetPoints(FEATURE_CURVE)[0]:
    print("Point:", point)
    if point == Curve.GetPoints(FEATURE_CURVE)[0][0]:
        continue
    pose = transl(point[0], point[1], point[2])
    print("Moving to point:", pose)
    robot.MoveL(pose, blocking=True)


#first = robot.MoveL(Curve.Pose(0), blocking=False)
#print(first)




#safe_move(robot, first, 'J')

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
    robot.Disconnect()
    print("✅ Disconnected.")
