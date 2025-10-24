#!/usr/bin/env python3
"""
RoboDK Robot Movement Script
This script connects to RoboDK, creates a new point, and moves the robot to that point.
"""

from robodk.robolink import *    # RoboDK API
from robodk.robomath import *    # Robot toolbox
import time

def main():
    # Connect to RoboDK
    RDK = Robolink()
    
    # Check if RoboDK is running
    if not RDK.Valid():
        print("RoboDK is not running. Please start RoboDK.")
        return
    
    print("Connected to RoboDK")
    
    # Get the robot (first robot in the station)
    robot = RDK.Item('', ITEM_TYPE_ROBOT)
    if not robot.Valid():
        print("No robot found in the station")
        return
    
    print(f"Robot found: {robot.Name()}")
    
    # Define a new target point (X, Y, Z, RX, RY, RZ in mm and degrees)
    # You can modify these coordinates as needed
    target_position = [500, 200, 300, 0, 0, 90]  # X, Y, Z, Rx, Ry, Rz
    
    # Create a pose from the target position
    target_pose = KUKA_2_Pose(target_position)
    
    # Create a new target in RoboDK
    target_name = "NewTarget"
    target = RDK.AddTarget(target_name)
    target.setPose(target_pose)
    
    print(f"Created new target: {target_name}")
    print(f"Target position: X={target_position[0]}, Y={target_position[1]}, Z={target_position[2]}")
    print(f"Target orientation: RX={target_position[3]}, RY={target_position[4]}, RZ={target_position[5]}")
    
    # Check if the target is reachable
    if robot.SolveIK(target_pose) is None:
        print("Warning: Target may not be reachable by the robot")
    else:
        print("Target is reachable")
    
    # Move the robot to the target
    print("Moving robot to target...")
    try:
        # Set the robot to move to the target
        robot.MoveJ(target)
        print("Robot movement completed successfully")
        
        # Optional: Get current robot position for verification
        current_pose = robot.Pose()
        current_xyz = Pose_2_KUKA(current_pose)
        print(f"Robot current position: X={current_xyz[0]:.2f}, Y={current_xyz[1]:.2f}, Z={current_xyz[2]:.2f}")
        
    except Exception as e:
        print(f"Error moving robot: {e}")
    
    print("Script completed")

if __name__ == "__main__":
    main()