from robodk.robolink import *
from robodk.robomath import *
import numpy as np
import time

# Connect to RoboDK
RDK = Robolink()
robot = RDK.Item('UR3e', ITEM_TYPE_ROBOT)
robot.Connect()

RDK.setSimulationSpeed(1)
robot.setSpeed(100)  # deg/s
robot.setSpeed(100, 100)  # linear mm/s, optional joint speed limit
robot.setAcceleration(1000)  # mm/s^2
robot.setSpeedJoints(1000)  # deg/s
robot.setAccelerationJoints(1000)

RDK.setRunMode(RUNMODE_RUN_ROBOT)
RDK.setCollisionActive(COLLISION_ON)

print("-" * 50)

def safe_move(robot: Item, target: Mat, move_type: str = 'J'):
    if type(target) == Mat:
        pass
    elif type(target) is list:
        target = robot.SolveFK(target)

    move_type = move_type.upper()
    if move_type not in ['J', 'L', 'C']:
        raise ValueError("Invalid move_type. Use 'J', 'L', or 'C'.")
    
    joints_current = robot.Joints()  # current robot position
    joints_target = robot.SolveIK(target)

    if not joints_target:
        print("❌ No IK solution found!")
        return False

    # ---- PATH TEST using **exact joint vector** for execution ----
    if move_type == 'J':
        test_failed = robot.MoveJ_Test(joints_current, joints_target)
    elif move_type == 'L':
        # Use FK of the **exact same joints** to avoid numerical differences
        for joints in joints_target:
            pose_target = robot.SolveFK(joints)
            test_failed = robot.MoveL_Test(joints_current, pose_target)
            if test_failed == -1:
                print("❌ No valid path found for linear move found, resulting to joint move.")
                safe_move(robot,target,'J')
                return True

    elif move_type == 'C':
        print("MoveC not implemented for poses")
        return False

    if test_failed:
        print("⚠️ No valid path found or collision predicted! Move aborted.")
        return False

    # ---- Execute using the **same joint vector** ----
    if move_type == 'J':
        robot.MoveJ(joints_target, blocking=True)
    elif move_type == 'L':
        robot.MoveL(joints_target, blocking=True)

    print("✅ Move completed successfully.")
    return True

# Example joint move
joint_target = [0.0, -90.0, -90.0, 0.0, 90.0, 0.0]
#safe_move(robot, RDK, joint_target, 'J')

# Example pose move
first = Mat([
    [1, 0, 0, 100],
    [0, 1, 0, -300],
    [0, 0, 1, 600],
    [0, 0, 0, 1]
])
second = Mat([
    [1, 0, 0, 100],
    [0, 1, 0, -300],
    [0, 0, 1, 500],
    [0, 0, 0, 1]
])

third = Mat([
    [1, 0, 0, 100],
    [0, 1, 0, -300],
    [0, 0, 1, 400],
    [0, 0, 0, 1]
])

print("Moving to first pose...")
safe_move(robot, first, 'J')

print("Moving to joint pose...")
safe_move(robot, third, 'L')
