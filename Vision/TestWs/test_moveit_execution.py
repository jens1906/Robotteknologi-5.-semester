#!/usr/bin/env python3
"""
Test MoveIt trajectory planning and execution directly without RViz
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import time

def main():
    rclpy.init()
    
    # Create MoveItPy instance
    print("Initializing MoveItPy...")
    moveit = MoveItPy(node_name="test_moveit_execution")
    
    # Get planning component
    arm = moveit.get_planning_component("ur_manipulator")
    print("Got planning component for ur_manipulator")
    
    # Get current state
    robot_model = moveit.get_robot_model()
    robot_state = moveit.get_planning_scene_monitor().current_state
    print(f"Current joint positions: {robot_state.joint_positions}")
    
    # Set a simple goal - move joint 1 by 0.5 radians
    print("\nSetting goal state...")
    arm.set_start_state_to_current_state()
    
    # Create goal state
    goal_state = RobotState(robot_model)
    joint_positions = robot_state.joint_positions.copy()
    joint_positions[0] += 0.3  # Move shoulder_pan_joint by 0.3 radians
    goal_state.joint_positions = joint_positions
    arm.set_goal_state(robot_state=goal_state)
    
    # Plan
    print("Planning...")
    plan_result = arm.plan()
    
    if plan_result:
        print(f"✓ Planning succeeded!")
        trajectory = plan_result.trajectory
        print(f"  Trajectory has {len(trajectory.joint_trajectory.points)} waypoints")
        
        # Check timestamps
        if len(trajectory.joint_trajectory.points) >= 2:
            t0 = trajectory.joint_trajectory.points[0].time_from_start.sec + \
                 trajectory.joint_trajectory.points[0].time_from_start.nanosec * 1e-9
            t1 = trajectory.joint_trajectory.points[1].time_from_start.sec + \
                 trajectory.joint_trajectory.points[1].time_from_start.nanosec * 1e-9
            print(f"  First waypoint time: {t0:.6f}s")
            print(f"  Second waypoint time: {t1:.6f}s")
            print(f"  Time difference: {t1-t0:.6f}s")
            
            if t1 > t0:
                print("  ✓ Timestamps are strictly increasing!")
            else:
                print("  ✗ ERROR: Timestamps not increasing!")
                return
        
        # Execute
        print("\nExecuting trajectory...")
        robot = moveit.get_planning_component("ur_manipulator")
        result = robot.execute(blocking=True)
        
        if result:
            print("✓ Execution SUCCEEDED!")
        else:
            print("✗ Execution FAILED!")
    else:
        print("✗ Planning failed!")
    
    moveit.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
