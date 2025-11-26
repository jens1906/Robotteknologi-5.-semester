#!/usr/bin/env python3
"""
Simple test script to move robot to a single target position.
Tests basic MoveIt functionality without complex path planning.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
import sys


class TestMover(Node):
    def __init__(self):
        super().__init__('test_mover')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Test Mover - Single Point')
        self.get_logger().info('='*60)
        
        # Create action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ MoveGroup action ready')
    
    def move_to_position(self, x, y, z, orientation='down'):
        """
        Move to a target position with specified orientation.
        
        Args:
            x, y, z: Target position in meters
            orientation: 'down' (tool pointing down) or 'forward' (tool pointing forward)
        """
        self.get_logger().info(f'Moving to position: [{x:.3f}, {y:.3f}, {z:.3f}]')
        self.get_logger().info(f'Tool orientation: {orientation}')
        
        # Create goal
        goal_msg = MoveGroup.Goal()
        
        # Workspace parameters
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        # Planning parameters
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # Create target pose
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from moveit_msgs.msg import BoundingVolume
        from shape_msgs.msg import SolidPrimitive
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'world'
        position_constraint.link_name = 'tool0'
        position_constraint.weight = 1.0
        
        # Small sphere around target
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]  # 1mm tolerance
        bounding_volume.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position.x = x
        sphere_pose.position.y = y
        sphere_pose.position.z = z
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)
        
        position_constraint.constraint_region = bounding_volume
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.weight = 0.1  # Low weight - prioritize position
        
        if orientation == 'down':
            # Tool pointing straight down (Z-axis of tool0 pointing down)
            # This is identity quaternion (no rotation from world frame)
            orientation_constraint.orientation.x = 0.0
            orientation_constraint.orientation.y = 0.0
            orientation_constraint.orientation.z = 0.0
            orientation_constraint.orientation.w = 1.0
        else:  # forward
            # Tool pointing forward (rotate 90° around Y)
            from scipy.spatial.transform import Rotation as R
            rot = R.from_euler('y', 90, degrees=True)
            quat = rot.as_quat()  # [x, y, z, w]
            orientation_constraint.orientation.x = quat[0]
            orientation_constraint.orientation.y = quat[1]
            orientation_constraint.orientation.z = quat[2]
            orientation_constraint.orientation.w = quat[3]
        
        # Very relaxed orientation tolerance
        orientation_constraint.absolute_x_axis_tolerance = 3.14
        orientation_constraint.absolute_y_axis_tolerance = 3.14
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        
        # Add constraints
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        
        # Planning options
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        # Send goal
        self.get_logger().info('Sending move goal...')
        send_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        if not send_future.done() or not send_future.result().accepted:
            self.get_logger().error('❌ Goal rejected')
            return False
        
        goal_handle = send_future.result()
        self.get_logger().info('✓ Goal accepted, planning and executing...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        if not result_future.done():
            self.get_logger().error('❌ Motion timed out')
            return False
        
        result = result_future.result().result
        
        from moveit_msgs.msg import MoveItErrorCodes
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('✅ SUCCESS! Robot reached target position')
            return True
        else:
            error_codes = {
                -1: 'FAILURE',
                -2: 'PLANNING_FAILED',
                -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
                -31: 'NO_IK_SOLUTION',
            }
            error_name = error_codes.get(result.error_code.val, f'ERROR_{result.error_code.val}')
            self.get_logger().error(f'❌ Motion failed: {error_name}')
            return False


def main():
    rclpy.init()
    node = TestMover()
    
    # Target position from line simulator (UPDATED: Y is now -0.300 based on log)
    target_x = -0.150
    target_y = -0.300  # Updated from 0.300 to -0.300
    target_z = 0.200
    
    node.get_logger().info('')
    node.get_logger().info('TARGET POSITION:')
    node.get_logger().info(f'  X = {target_x:7.3f} m')
    node.get_logger().info(f'  Y = {target_y:7.3f} m')
    node.get_logger().info(f'  Z = {target_z:7.3f} m')
    node.get_logger().info('')
    node.get_logger().info('This matches the first point from line_simulator output')
    node.get_logger().info('')
    node.get_logger().info('Press Enter to start motion...')
    input()
    
    # Try with tool pointing down (simplest orientation)
    success = node.move_to_position(target_x, target_y, target_z, orientation='down')
    
    if success:
        node.get_logger().info('')
        node.get_logger().info('='*60)
        node.get_logger().info('✅ TEST PASSED - Robot reached target!')
        node.get_logger().info('='*60)
    else:
        node.get_logger().info('')
        node.get_logger().info('='*60)
        node.get_logger().info('❌ TEST FAILED - Could not reach target')
        node.get_logger().info('='*60)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
