#!/usr/bin/env python3
"""
Collision-Aware Robot Movement Interface

This script provides a simple Python API for moving the UR3e robot with
full collision checking against the testsetup, the robot itself, and the camera.

All movements go through MoveIt, ensuring proper collision detection with:
- The testsetup/testplate
- Robot self-collision
- RealSense camera attached to the end-effector
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    WorkspaceParameters
)
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import math


class CollisionAwareMover(Node):
    """
    A ROS2 node that provides collision-aware movement capabilities
    for the UR3e robot through MoveIt.
    """
    
    def __init__(self):
        super().__init__('collision_aware_mover')
        
        # MoveIt action client
        self._move_group_client = ActionClient(
            self, 
            MoveGroup, 
            '/move_action'
        )
        
        # Joint state subscriber to track current position
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        self._current_joint_state = None
        self._planning_group = "ur_manipulator"
        
        self.get_logger().info("Waiting for MoveIt action server...")
        self._move_group_client.wait_for_server()
        self.get_logger().info("✅ Connected to MoveIt - Collision checking enabled!")
        
    def _joint_state_callback(self, msg):
        """Store the current joint state"""
        self._current_joint_state = msg
        
    def move_to_joint_positions(self, joint_positions, velocity_scaling=0.1, acceleration_scaling=0.1):
        """
        Move to specified joint positions with collision checking.
        
        Args:
            joint_positions: List of 6 joint angles in radians [j1, j2, j3, j4, j5, j6]
            velocity_scaling: Speed factor (0.0-1.0), default 0.1 for safety
            acceleration_scaling: Acceleration factor (0.0-1.0), default 0.1 for safety
            
        Returns:
            True if movement succeeded, False if planning failed (collision detected)
        """
        if len(joint_positions) != 6:
            self.get_logger().error("Must provide exactly 6 joint positions!")
            return False
            
        # Wait for current state
        if self._current_joint_state is None:
            self.get_logger().warn("Waiting for joint states...")
            time.sleep(1.0)
            if self._current_joint_state is None:
                self.get_logger().error("No joint states available!")
                return False
        
        # Create motion plan request
        goal_msg = MoveGroup.Goal()
        
        # Set planning group
        goal_msg.request.group_name = self._planning_group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = acceleration_scaling
        
        # Set workspace bounds (adjust to your testsetup)
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.min_corner = Point(x=-1.0, y=-1.0, z=-0.5)
        goal_msg.request.workspace_parameters.max_corner = Point(x=1.0, y=1.0, z=1.5)
        
        # Define joint constraints for target position
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        constraints = Constraints()
        for joint_name, target_pos in zip(joint_names, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_pos
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        # Set start state to current state
        goal_msg.request.start_state.joint_state = self._current_joint_state
        goal_msg.request.start_state.is_diff = False
        
        # Set planner
        goal_msg.planning_options.plan_only = False  # Plan AND execute
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        self.get_logger().info(f"Planning movement to: {[f'{p:.3f}' for p in joint_positions]}")
        
        # Send goal and wait for result
        future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Movement rejected by MoveIt!")
            return False
            
        self.get_logger().info("Planning accepted, waiting for execution...")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("✅ Movement completed successfully!")
            return True
        else:
            self.get_logger().error(f"❌ Movement failed! Error code: {result.error_code.val}")
            self.get_logger().error("Possible reasons: collision detected, IK failed, or unreachable pose")
            return False
            
    def move_to_named_target(self, target_name):
        """
        Move to a named target defined in SRDF (e.g., 'home').
        
        Args:
            target_name: Name of the target pose (e.g., 'home', 'up')
            
        Returns:
            True if movement succeeded, False otherwise
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self._planning_group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Set named target
        constraints = Constraints()
        constraints.name = target_name
        goal_msg.request.goal_constraints.append(constraints)
        
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.planning_options.plan_only = False
        
        self.get_logger().info(f"Moving to named target: {target_name}")
        
        future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"❌ Named target '{target_name}' rejected!")
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        return result.error_code.val == 1


def main():
    """Example usage of the CollisionAwareMover"""
    rclpy.init()
    
    mover = CollisionAwareMover()
    
    try:
        # Example 1: Move to home position
        print("\n=== Example 1: Moving to home position ===")
        time.sleep(1)  # Wait for initial joint states
        
        home_position = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        success = mover.move_to_joint_positions(home_position)
        
        if success:
            print("✅ Reached home position")
            time.sleep(2)
        else:
            print("❌ Failed to reach home - collision or planning failure!")
            
        # Example 2: Try a small movement
        print("\n=== Example 2: Small rotation of joint 1 ===")
        test_position = [0.3, -1.57, 1.57, -1.57, -1.57, 0.0]
        success = mover.move_to_joint_positions(test_position)
        
        if success:
            print("✅ Movement completed safely")
        else:
            print("❌ Movement blocked - would have caused collision!")
            
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    finally:
        mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
