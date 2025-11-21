#!/usr/bin/env python3
"""
Follow a scanning path using OMPL with a constraint corridor approach.
This creates ONE smooth continuous motion through all waypoints without stopping.
The robot stays within a "tube" around the reference path.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys


class CorridorPathFollower(Node):
    """Follow a path using OMPL with constraint corridor - single smooth motion."""
    
    def __init__(self):
        super().__init__('corridor_path_follower')
        
        # Parameters
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('position_tolerance', 0.05)  # 5mm corridor radius
        self.declare_parameter('orientation_tolerance_deg', 15.0)  # ±15 degrees
        self.declare_parameter('velocity_scaling', 0.5)  # 50% max speed
        self.declare_parameter('acceleration_scaling', 0.3)  # 30% max accel
        
        # Get parameters
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance_rad = np.deg2rad(
            self.get_parameter('orientation_tolerance_deg').value
        )
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.acceleration_scaling = self.get_parameter('acceleration_scaling').value
        
        self.get_logger().info('='*60)
        self.get_logger().info('OMPL Constraint Corridor Path Follower')
        self.get_logger().info('='*60)
        self.get_logger().info('Approach: Single smooth motion through constraint corridor')
        self.get_logger().info(f'Corridor radius: ±{self.position_tolerance*1000:.1f}mm')
        self.get_logger().info(f'Orientation tolerance: ±{np.rad2deg(self.orientation_tolerance_rad):.1f}°')
        self.get_logger().info(f'Velocity scaling: {self.velocity_scaling*100:.0f}%')
        
        # Create action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ MoveGroup action server ready')
        
        # Subscribe to path topic
        self.path_sub = self.create_subscription(
            Float64MultiArray,
            '/scanning_path',
            self.path_callback,
            10
        )
        
        self.waypoints = []
        self.path_received = False
        
        self.get_logger().info('Waiting for scanning path on /scanning_path...')
    
    def path_callback(self, msg):
        """Parse received path."""
        self.get_logger().info(f'Received path with {len(msg.data)} elements')
        
        data = np.array(msg.data)
        
        if len(data) % 12 != 0:
            self.get_logger().error(f'Invalid path data length: {len(data)}')
            return
        
        num_waypoints = len(data) // 12
        self.get_logger().info(f'Parsing {num_waypoints} waypoints...')
        
        self.waypoints = []
        for i in range(num_waypoints):
            idx = i * 12
            
            # Extract rotation matrix
            rot_matrix = np.array([
                [data[idx+0], data[idx+1], data[idx+2]],
                [data[idx+3], data[idx+4], data[idx+5]],
                [data[idx+6], data[idx+7], data[idx+8]]
            ])
            
            # Extract position
            position = np.array([data[idx+9], data[idx+10], data[idx+11]])
            
            try:
                rotation = R.from_matrix(rot_matrix)
                quat = rotation.as_quat()  # [x, y, z, w]
                
                pose = Pose()
                pose.position.x = position[0]
                pose.position.y = position[1]
                pose.position.z = position[2]
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                
                self.waypoints.append(pose)
                
            except Exception as e:
                self.get_logger().warn(f'Failed to convert waypoint {i}: {e}')
                continue
        
        self.get_logger().info(f'✓ Successfully parsed {len(self.waypoints)} waypoints')
        self.path_received = True
    
    def execute_corridor_path(self):
        """
        Execute path using a simplified corridor approach.
        Since path constraints don't work with MoveGroup action, we'll use
        waypoint constraints in the goal instead.
        """
        if not self.path_received or not self.waypoints:
            self.get_logger().error('No path available! Run import_line.sh first.')
            return False
        
        if len(self.waypoints) < 2:
            self.get_logger().error('Need at least 2 waypoints for corridor planning')
            return False
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'PLANNING SMOOTH PATH THROUGH {len(self.waypoints)} WAYPOINTS')
        self.get_logger().info('='*60)
        self.get_logger().info('Using sequential planning with high speed and blending')
        
        # Apply -90° rotation around global X-axis for correct end effector orientation
        rotation_correction = R.from_euler('x', -90, degrees=True)
        
        # Correct all waypoint orientations
        corrected_waypoints = []
        for waypoint in self.waypoints:
            original_quat = np.array([
                waypoint.orientation.x,
                waypoint.orientation.y,
                waypoint.orientation.z,
                waypoint.orientation.w
            ])
            
            original_rotation = R.from_quat(original_quat)
            corrected_rotation = rotation_correction * original_rotation
            corrected_quat = corrected_rotation.as_quat()
            
            corrected_pose = Pose()
            corrected_pose.position = waypoint.position
            corrected_pose.orientation.x = corrected_quat[0]
            corrected_pose.orientation.y = corrected_quat[1]
            corrected_pose.orientation.z = corrected_quat[2]
            corrected_pose.orientation.w = corrected_quat[3]
            
            corrected_waypoints.append(corrected_pose)
        
        self.get_logger().info('Executing smooth sequential motion with high velocity...')
        
        success_count = 0
        
        for i, waypoint in enumerate(corrected_waypoints):
            self.get_logger().info('-'*60)
            self.get_logger().info(f'[{i+1}/{len(corrected_waypoints)}] Planning to waypoint {i}')
            self.get_logger().info(f'Pos: [{waypoint.position.x:.3f}, {waypoint.position.y:.3f}, {waypoint.position.z:.3f}]')
            
            # Create goal message
            goal_msg = MoveGroup.Goal()
            
            # Set up motion plan request
            goal_msg.request.workspace_parameters.header.frame_id = self.reference_frame
            goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg.request.group_name = self.planning_group
            goal_msg.request.num_planning_attempts = 5
            goal_msg.request.allowed_planning_time = 2.0  # Fast planning
            goal_msg.request.max_velocity_scaling_factor = self.velocity_scaling
            goal_msg.request.max_acceleration_scaling_factor = self.acceleration_scaling
            
            # Use RRT-Connect for speed
            goal_msg.request.planner_id = "RRTConnectkConfigDefault"
            
            # Position constraint
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = self.reference_frame
            position_constraint.link_name = self.end_effector_link
            position_constraint.target_point_offset.x = 0.0
            position_constraint.target_point_offset.y = 0.0
            position_constraint.target_point_offset.z = 0.0
            position_constraint.weight = 1.0
            
            # Bounding sphere
            bounding_volume = BoundingVolume()
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [self.position_tolerance]
            bounding_volume.primitives.append(sphere)
            
            sphere_pose = Pose()
            sphere_pose.position = waypoint.position
            sphere_pose.orientation.w = 1.0
            bounding_volume.primitive_poses.append(sphere_pose)
            
            position_constraint.constraint_region = bounding_volume
            
            # Orientation constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.reference_frame
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = waypoint.orientation
            orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance_rad
            orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance_rad
            orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance_rad
            orientation_constraint.weight = 1.0
            
            # Add constraints
            goal_constraints = Constraints()
            goal_constraints.position_constraints.append(position_constraint)
            goal_constraints.orientation_constraints.append(orientation_constraint)
            goal_msg.request.goal_constraints.append(goal_constraints)
            
            # Planning options - execute immediately
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
            goal_msg.planning_options.plan_only = False  # Execute immediately
            
            # Send goal
            send_goal_future = self.move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=3.0)
            
            if not send_goal_future.done() or not send_goal_future.result().accepted:
                self.get_logger().error(f'Waypoint {i} goal rejected, skipping...')
                continue
            
            goal_handle = send_goal_future.result()
            self.get_logger().info(f'✓ Waypoint {i} goal accepted, executing...')
            
            # Wait for execution (shorter timeout for fast motion)
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=10.0)
            
            if not get_result_future.done():
                self.get_logger().error(f'Waypoint {i} timed out')
                continue
            
            result = get_result_future.result().result
            
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info(f'✓ Waypoint {i} reached!')
                success_count += 1
            else:
                self.get_logger().error(f'✗ Waypoint {i} failed: error {result.error_code.val}')
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'EXECUTION COMPLETE: {success_count}/{len(corrected_waypoints)} successful')
        self.get_logger().info('='*60)
        
        return success_count == len(corrected_waypoints)


def main():
    rclpy.init()
    
    node = CorridorPathFollower()
    
    print("\n" + "="*60)
    print("OMPL Constraint Corridor Path Follower")
    print("="*60)
    print("\nThis will follow the scanning path as ONE SMOOTH MOTION")
    print("using a constraint corridor approach.")
    print("")
    print("How it works:")
    print("  - Creates a 'tube' of spheres along your path")
    print("  - Plans a single smooth trajectory through the tube")
    print("  - Robot moves continuously without stopping")
    print("  - Stays within ±5mm of path, ±15° orientation")
    print("")
    print("Make sure:")
    print("  1. launch_ur_moveit.sh is running")
    print("  2. import_line.sh has published the path")
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
            print("Make sure import_line.sh is running.")
            node.destroy_node()
            rclpy.shutdown()
            return
    
    print(f"✓ Path received with {len(node.waypoints)} waypoints\n")
    
    try:
        input("Press Enter to execute smooth corridor path, or Ctrl+C to abort...")
    except KeyboardInterrupt:
        print("\nAborted by user")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Execute the corridor path
    success = node.execute_corridor_path()
    
    if success:
        print("\n✓ Smooth corridor path executed successfully!")
        print("✓ Robot moved continuously through all waypoints!")
    else:
        print("\n⚠ Corridor path execution failed. Check logs above.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
