#!/usr/bin/env python3
"""
Simple script to move the robot to a single point using MoveIt.
This actually executes the movement, not just checks if it's reachable.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes
)
from shape_msgs.msg import SolidPrimitive
import sys


class SimpleMoveNode(Node):
    def __init__(self):
        super().__init__('simple_move_node')
        
        # Create action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ MoveGroup action server is available')
    
    def move_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Move the robot to a target pose."""
        
        self.get_logger().info('='*60)
        self.get_logger().info('MOVING ROBOT TO TARGET POSITION')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Target: pos=[{x:.3f}, {y:.3f}, {z:.3f}]')
        self.get_logger().info(f'        quat=[{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]')
        
        # Create goal message
        goal_msg = MoveGroup.Goal()
        
        # Set up the motion plan request
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 1.0
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Create target pose constraint
        pose_constraint = PositionConstraint()
        pose_constraint.header.frame_id = 'world'
        pose_constraint.link_name = 'tool0'
        
        # Set target position with small tolerance
        pose_constraint.target_point_offset.x = 0.0
        pose_constraint.target_point_offset.y = 0.0
        pose_constraint.target_point_offset.z = 0.0
        
        # Create bounding volume (small sphere around target)
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]  # 1mm tolerance
        bounding_volume.primitives.append(sphere)
        
        from geometry_msgs.msg import Pose
        sphere_pose = Pose()
        sphere_pose.position.x = x
        sphere_pose.position.y = y
        sphere_pose.position.z = z
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)
        
        pose_constraint.constraint_region = bounding_volume
        pose_constraint.weight = 1.0
        
        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.orientation.x = qx
        orientation_constraint.orientation.y = qy
        orientation_constraint.orientation.z = qz
        orientation_constraint.orientation.w = qw
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        # Add constraints to goal
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pose_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        # Set planning pipeline
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False  # Plan AND execute
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5
        
        self.get_logger().info('Sending goal to MoveGroup...')
        
        # Send goal
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done():
            self.get_logger().error('Failed to send goal (timeout)')
            return False
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by MoveGroup')
            return False
        
        self.get_logger().info('✓ Goal accepted, planning and executing...')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        
        self.get_logger().info('Waiting for motion to complete (this may take a while)...')
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=60.0)
        
        if not get_result_future.done():
            self.get_logger().error('Motion execution timed out')
            return False
        
        result = get_result_future.result().result
        error_code = result.error_code.val
        
        self.get_logger().info('='*60)
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('✓ SUCCESS! Robot moved to target position!')
            self.get_logger().info('='*60)
            return True
        else:
            self.get_logger().error(f'✗ FAILED with error code: {error_code}')
            self.get_logger().error('='*60)
            return False


def main():
    rclpy.init()
    
    node = SimpleMoveNode()
    
    print("\n" + "="*60)
    print("SIMPLE ROBOT MOVEMENT TEST")
    print("="*60)
    print("\nThis will move the robot to a target position.")
    print("Make sure the robot is in a safe starting configuration!")
    print("")
    
    # Target position - adjust these coordinates
    x = 0.3   # Forward
    y = 0.0   # Left/Right
    z = 0.4   # Up/Down
    
    # Orientation: tool pointing down
    # Quaternion for 180° rotation around X-axis (pointing down)
    qx = 1.0
    qy = 0.0
    qz = 0.0
    qw = 0.0
    
    print(f"Target position: [{x}, {y}, {z}]")
    print(f"Target orientation: pointing down")
    print("")
    
    try:
        input("Press Enter to move the robot, or Ctrl+C to abort...")
    except KeyboardInterrupt:
        print("\nAborted by user")
        node.destroy_node()
        rclpy.shutdown()
        return

    success = node.move_to_pose(x, y, z, qx, qy, qz, qw)
    
    if success:
        print("\n✓ Robot successfully moved to target!")
    else:
        print("\n✗ Failed to move robot. Check the logs above for details.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
