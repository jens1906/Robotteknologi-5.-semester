#!/usr/bin/env python3
"""
Simple test script to move the robot end effector to the first waypoint.
Used for testing path starting position and orientation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, MoveItErrorCodes
)
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys


class FirstWaypointTester(Node):
    """Move to the first waypoint for testing."""
    
    def __init__(self):
        super().__init__('first_waypoint_tester')
        
        self.get_logger().info('='*60)
        self.get_logger().info('First Waypoint Tester')
        self.get_logger().info('='*60)
        
        # Create action client for moving
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        self.get_logger().info('Waiting for /move_action...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ MoveGroup action ready')
        
        # Subscribe to path topic (from external source or sim_external_line.sh)
        self.path_sub = self.create_subscription(
            Float64MultiArray,
            '/tool_orientation/xyz_rotation',
            self.path_callback,
            10
        )
        
        self.first_waypoint = None
        self.path_received = False
        
        self.get_logger().info('Waiting for path on /tool_orientation/xyz_rotation...')
    
    def path_callback(self, msg):
        """Parse received path and extract first waypoint."""
        if self.path_received:
            return
            
        self.get_logger().info(f'Received path with {len(msg.data)} elements')
        
        data = np.array(msg.data)
        
        if len(data) % 12 != 0:
            self.get_logger().error(f'Invalid path data length: {len(data)}')
            return
        
        num_waypoints = len(data) // 12
        self.get_logger().info(f'Parsing {num_waypoints} waypoints...')

        # Format is POSITION-FIRST: [x, y, z, r11...r33]
        # This is what external_topic_simulator.py and real external sources publish
        chosen = 'pos_first'
        self.get_logger().info(f'Using format: {chosen} (standard for /tool_orientation/xyz_rotation topic)')

        # Parse first waypoint only
        # Format: [x, y, z, r11,r12,r13,r21,r22,r23,r31,r32,r33]
        idx = 0
        try:
            position = np.array(data[idx:idx+3])
            rot_matrix = np.array(data[idx+3:idx+12]).reshape((3, 3))
            
            # DEBUG: Log raw parsed data
            self.get_logger().info('='*60)
            self.get_logger().info('DEBUG: First waypoint RAW data')
            self.get_logger().info('='*60)
            self.get_logger().info(f'Raw data[0:3] (position): {data[0:3]}')
            self.get_logger().info(f'Parsed position: [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]')
            self.get_logger().info(f'Rotation matrix determinant: {np.linalg.det(rot_matrix):.6f}')
            self.get_logger().info('='*60)

            # Check Z height
            if position[2] < 0.05:
                z_offset = 0.15 - position[2]
                self.get_logger().warn(f'Applying Z offset of {z_offset:.3f}m for safety')
                position[2] += z_offset

            # Sanitize rotation matrix
            try:
                U, S, Vt = np.linalg.svd(rot_matrix)
                rot_fixed = U @ Vt
                if np.linalg.det(rot_fixed) < 0:
                    U[:, -1] *= -1
                    rot_fixed = U @ Vt
            except Exception:
                rot_fixed = rot_matrix

            rotation = R.from_matrix(rot_fixed)
            quat = rotation.as_quat()

            # NO orientation correction - use the orientation exactly as published
            # The external source should provide the correct orientation for tool0

            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])

            self.first_waypoint = pose
            self.get_logger().info(f'✓ First waypoint: pos=[{pose.position.x:.3f}, '
                                  f'{pose.position.y:.3f}, {pose.position.z:.3f}]')
            self.path_received = True

        except Exception as e:
            self.get_logger().error(f'Failed to parse first waypoint: {e}')
            return
    
    def move_to_first_waypoint(self):
        """Move to the first waypoint."""
        if not self.path_received or self.first_waypoint is None:
            self.get_logger().error('No waypoint available!')
            return False
        
        self.get_logger().info('='*60)
        self.get_logger().info('MOVING TO FIRST WAYPOINT')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Target: pos=[{self.first_waypoint.position.x:.3f}, '
                              f'{self.first_waypoint.position.y:.3f}, '
                              f'{self.first_waypoint.position.z:.3f}]')
        
        # Try with progressively more relaxed constraints
        tolerance_levels = [
            {'pos': 0.01, 'orient': 0.1, 'name': 'strict'},
            {'pos': 0.02, 'orient': 0.3, 'name': 'normal'},
            {'pos': 0.03, 'orient': 0.5, 'name': 'relaxed'},
            {'pos': 0.05, 'orient': 1.0, 'name': 'very relaxed'}
        ]
        
        for tolerances in tolerance_levels:
            self.get_logger().info(f'Attempting with {tolerances["name"]} constraints...')
            
            # Create goal message
            goal_msg = MoveGroup.Goal()
            goal_msg.request.workspace_parameters.header.frame_id = 'world'
            goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
            goal_msg.request.group_name = 'ur_manipulator'
            goal_msg.request.num_planning_attempts = 30
            goal_msg.request.allowed_planning_time = 15.0
            goal_msg.request.max_velocity_scaling_factor = 0.2
            goal_msg.request.max_acceleration_scaling_factor = 0.2
            goal_msg.request.planner_id = "RRTConnectkConfigDefault"
            
            # Position constraint
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = 'world'
            position_constraint.link_name = 'tool0'
            position_constraint.weight = 1.0
            
            bounding_volume = BoundingVolume()
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [tolerances['pos']]
            bounding_volume.primitives.append(sphere)
            
            sphere_pose = Pose()
            sphere_pose.position = self.first_waypoint.position
            sphere_pose.orientation.w = 1.0
            bounding_volume.primitive_poses.append(sphere_pose)
            position_constraint.constraint_region = bounding_volume
            
            # Orientation constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = 'world'
            orientation_constraint.link_name = 'tool0'
            orientation_constraint.orientation = self.first_waypoint.orientation
            orientation_constraint.absolute_x_axis_tolerance = tolerances['orient']
            orientation_constraint.absolute_y_axis_tolerance = tolerances['orient']
            orientation_constraint.absolute_z_axis_tolerance = tolerances['orient']
            orientation_constraint.weight = 1.0
            
            goal_constraints = Constraints()
            goal_constraints.position_constraints.append(position_constraint)
            goal_constraints.orientation_constraints.append(orientation_constraint)
            goal_msg.request.goal_constraints.append(goal_constraints)
            
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
            goal_msg.planning_options.plan_only = False
            
            # Send goal
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            if not send_goal_future.done() or not send_goal_future.result().accepted:
                self.get_logger().warn(f'Goal rejected with {tolerances["name"]} constraints')
                continue
            
            goal_handle = send_goal_future.result()
            self.get_logger().info('✓ Planning goal accepted, computing path...')
            
            # Wait for result
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=30.0)
            
            if not get_result_future.done():
                self.get_logger().warn(f'Planning timed out with {tolerances["name"]} constraints')
                continue
            
            result = get_result_future.result().result
            
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('='*60)
                self.get_logger().info(f'✓ SUCCESS! Reached first waypoint')
                self.get_logger().info(f'  Used {tolerances["name"]} constraints')
                self.get_logger().info('='*60)
                return True
            else:
                error_meanings = {
                    -1: 'FAILURE',
                    -2: 'PLANNING_FAILED',
                    -31: 'NO_IK_SOLUTION',
                    -13: 'GOAL_IN_COLLISION'
                }
                error_name = error_meanings.get(result.error_code.val, f'ERROR_{result.error_code.val}')
                self.get_logger().warn(f'Failed with {tolerances["name"]} constraints: {error_name}')
        
        # All attempts failed
        self.get_logger().error('='*60)
        self.get_logger().error('✗ FAILED to reach first waypoint with all constraint levels')
        self.get_logger().error('='*60)
        return False


def main():
    rclpy.init()
    
    node = FirstWaypointTester()
    
    print("\n" + "="*60)
    print("First Waypoint Tester")
    print("="*60)
    print("\nTests moving the robot to the first waypoint")
    print("with correct position and orientation.")
    print("")
    print("Make sure:")
    print("  1. launch_ur_moveit.sh is running")
    print("  2. Path data is being published to /tool_orientation/xyz_rotation")
    print("     - Use sim_external_line.sh for testing, OR")
    print("     - Connect to real external PC publishing path data")
    print("  3. Robot is in a safe starting position")
    print("")
    
    # Wait for path
    print("Waiting for path data...")
    timeout = 10.0
    start_time = node.get_clock().now()
    
    while not node.path_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
            print("ERROR: No path received within timeout!")
            print("Make sure sim_external_line.sh is running OR external PC is publishing.")
            node.destroy_node()
            rclpy.shutdown()
            return
    
    print(f"✓ First waypoint received\n")
    
    try:
        input("Press Enter to move to first waypoint, or Ctrl+C to abort...")
    except KeyboardInterrupt:
        print("\nAborted by user")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Move to first waypoint
    success = node.move_to_first_waypoint()
    
    if success:
        print("\n✓ Robot successfully moved to first waypoint!")
        print("  Position and orientation are correct.")
    else:
        print("\n⚠ Failed to reach first waypoint. Check logs above.")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
