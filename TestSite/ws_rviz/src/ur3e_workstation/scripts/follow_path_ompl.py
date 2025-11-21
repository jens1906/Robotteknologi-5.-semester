#!/usr/bin/env python3
"""
Follow a scanning path using OMPL RRT-Connect planner with relaxed constraints.
This allows for position tolerance (±0.5cm) and orientation tolerance (±15°).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
    PlanningOptions
)
from shape_msgs.msg import SolidPrimitive
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys


class OMPLPathFollower(Node):
    """Follow a path using OMPL RRT-Connect with smooth continuous motion."""
    
    def __init__(self):
        super().__init__('ompl_path_follower')
        
        # Parameters
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('position_tolerance', 0.0001)  # 0.5cm = 5mm
        self.declare_parameter('orientation_tolerance_deg', 0.1)  # ±15 degrees
        self.declare_parameter('velocity_scaling', 0.8)  # 30% max speed
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
        self.get_logger().info('OMPL Path Follower - Smooth Continuous Motion')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Position tolerance: ±{self.position_tolerance*1000:.1f}mm')
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
    
    def move_to_waypoint_ompl(self, waypoint_idx, pose):
        """
        Move to a waypoint using OMPL RRT-Connect planner with relaxed constraints.
        """
        self.get_logger().info('-'*60)
        self.get_logger().info(f'Planning to waypoint {waypoint_idx}')
        self.get_logger().info(f'Target: pos=[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]')
        self.get_logger().info(f'        ori=[{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, '
                              f'{pose.orientation.z:.3f}, {pose.orientation.w:.3f}]')
        
        # Create goal message
        goal_msg = MoveGroup.Goal()
        
        # Set up motion plan request
        goal_msg.request.workspace_parameters.header.frame_id = self.reference_frame
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3  # 30% speed
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # IMPORTANT: Specify OMPL RRT-Connect planner
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"  # OMPL RRT-Connect
        
        # Create position constraint with tolerance sphere
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.reference_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.weight = 1.0
        
        # Create bounding sphere with position tolerance
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]  # 5mm radius
        bounding_volume.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position.x = pose.position.x
        sphere_pose.position.y = pose.position.y
        sphere_pose.position.z = pose.position.z
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)
        
        position_constraint.constraint_region = bounding_volume
        
        # Create orientation constraint with tolerance
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.reference_frame
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation.x = pose.orientation.x
        orientation_constraint.orientation.y = pose.orientation.y
        orientation_constraint.orientation.z = pose.orientation.z
        orientation_constraint.orientation.w = pose.orientation.w
        
        # Set orientation tolerances (±15 degrees in radians)
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance_rad
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance_rad
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance_rad
        orientation_constraint.weight = 1.0
        
        # Add constraints to goal
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        # Set planning options
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False  # Plan AND execute
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        
        self.get_logger().info('Sending goal with OMPL RRT-Connect...')
        
        # Send goal
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done():
            self.get_logger().error('Failed to send goal (timeout)')
            return False
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveGroup')
            return False
        
        self.get_logger().info('✓ Goal accepted, planning with RRT-Connect...')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=30.0)
        
        if not get_result_future.done():
            self.get_logger().error('Planning/execution timed out')
            return False
        
        result = get_result_future.result().result
        error_code = result.error_code.val
        
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'✓ Waypoint {waypoint_idx} reached successfully!')
            return True
        else:
            self.get_logger().error(f'✗ Failed with error code: {error_code}')
            return False
    
    def execute_path_smooth(self):
        """
        Execute path by planning to each waypoint but with smooth blending.
        Uses relaxed constraints and executes quickly without stopping.
        Applies -90° rotation around global X-axis for correct end effector orientation.
        """
        if not self.path_received or not self.waypoints:
            self.get_logger().error('No path available! Run import_line.sh first.')
            return False
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'EXECUTING SMOOTH PATH THROUGH {len(self.waypoints)} WAYPOINTS')
        self.get_logger().info('='*60)
        self.get_logger().info('Strategy: Sequential planning with minimal stop time')
        self.get_logger().info('Optimization: Each segment uses RRT-Connect for speed')
        self.get_logger().info('Orientation correction: -90° around global X-axis')
        
        # Create rotation correction: -90° around global X-axis
        rotation_correction = R.from_euler('x', +90, degrees=True)
        
        success_count = 0
        
        for i, waypoint in enumerate(self.waypoints):
            # Apply -90° rotation around global X-axis to the waypoint orientation
            original_quat = np.array([
                waypoint.orientation.x,
                waypoint.orientation.y,
                waypoint.orientation.z,
                waypoint.orientation.w
            ])
            
            # Convert to rotation object
            original_rotation = R.from_quat(original_quat)
            
            # Apply correction: rotate -90° around global X-axis
            corrected_rotation = rotation_correction * original_rotation
            corrected_quat = corrected_rotation.as_quat()
            
            # Create corrected waypoint
            corrected_waypoint = Pose()
            corrected_waypoint.position = waypoint.position
            corrected_waypoint.orientation.x = corrected_quat[0]
            corrected_waypoint.orientation.y = corrected_quat[1]
            corrected_waypoint.orientation.z = corrected_quat[2]
            corrected_waypoint.orientation.w = corrected_quat[3]
            
            self.get_logger().info('-'*60)
            self.get_logger().info(f'[{i+1}/{len(self.waypoints)}] Moving to waypoint {i}')
            self.get_logger().info(f'Target: pos=[{corrected_waypoint.position.x:.3f}, {corrected_waypoint.position.y:.3f}, {corrected_waypoint.position.z:.3f}]')
            self.get_logger().info(f'Original quat: [{original_quat[0]:.3f}, {original_quat[1]:.3f}, {original_quat[2]:.3f}, {original_quat[3]:.3f}]')
            self.get_logger().info(f'Corrected quat: [{corrected_quat[0]:.3f}, {corrected_quat[1]:.3f}, {corrected_quat[2]:.3f}, {corrected_quat[3]:.3f}]')
            
            # Create goal message
            goal_msg = MoveGroup.Goal()
            
            # Set up motion plan request
            goal_msg.request.workspace_parameters.header.frame_id = self.reference_frame
            goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg.request.group_name = self.planning_group
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 3.0  # Faster planning
            goal_msg.request.max_velocity_scaling_factor = self.velocity_scaling
            goal_msg.request.max_acceleration_scaling_factor = self.acceleration_scaling
            
            # Use RRT-Connect
            goal_msg.request.planner_id = "RRTConnectkConfigDefault"
            
            # Create position constraint with tolerance
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
            sphere_pose.position.x = corrected_waypoint.position.x
            sphere_pose.position.y = corrected_waypoint.position.y
            sphere_pose.position.z = corrected_waypoint.position.z
            sphere_pose.orientation.w = 1.0
            bounding_volume.primitive_poses.append(sphere_pose)
            
            position_constraint.constraint_region = bounding_volume
            
            # Create orientation constraint with CORRECTED orientation
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.reference_frame
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = corrected_waypoint.orientation
            orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance_rad
            orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance_rad
            orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance_rad
            orientation_constraint.weight = 1.0
            
            # Add constraints
            goal_constraints = Constraints()
            goal_constraints.position_constraints.append(position_constraint)
            goal_constraints.orientation_constraints.append(orientation_constraint)
            goal_msg.request.goal_constraints.append(goal_constraints)
            
            # Planning options - execute immediately for smooth motion
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
            
            # Wait for execution
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=15.0)
            
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
        self.get_logger().info(f'EXECUTION COMPLETE: {success_count}/{len(self.waypoints)} successful')
        self.get_logger().info('='*60)
        
        return success_count == len(self.waypoints)


def main():
    rclpy.init()
    
    node = OMPLPathFollower()
    
    print("\n" + "="*60)
    print("OMPL RRT-Connect Path Follower")
    print("="*60)
    print("\nThis will follow the scanning path using OMPL RRT-Connect.")
    print("Constraints:")
    print("  - Position tolerance: ±5mm")
    print("  - Orientation tolerance: ±15°")
    print("  - Collision avoidance: Enabled")
    print("\nMake sure:")
    print("  1. launch_ur_moveit.sh is running")
    print("  2. import_line.sh has published the path")
    print("  3. Robot is in a safe starting position")
    print("")
    
    # Wait for path to be received
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
        input("Press Enter to start path execution, or Ctrl+C to abort...")
    except KeyboardInterrupt:
        print("\nAborted by user")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Execute the path as ONE smooth continuous motion
    success = node.execute_path_smooth()
    
    if success:
        print("\n✓ Smooth path executed successfully through all waypoints!")
    else:
        print("\n⚠ Path execution failed. Check logs above for details.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
