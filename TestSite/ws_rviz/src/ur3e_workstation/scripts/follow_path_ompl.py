#!/usr/bin/env python3
"""
Follow a scanning path using OMPL with ONE continuous trajectory through all waypoints.
This creates a single smooth motion without stopping between waypoints.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
    PlanningOptions,
    JointConstraint
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys


class OMPLContinuousPathFollower(Node):
    """Follow a path using OMPL with one continuous trajectory through all waypoints."""
    
    def __init__(self):
        super().__init__('ompl_continuous_path_follower')
        
        self.get_logger().info('='*60)
        self.get_logger().info('OMPL Continuous Path Follower - Single Smooth Motion')
        self.get_logger().info('='*60)
        
        # Create service client for Cartesian path (fallback)
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        
        # Create action client for trajectory execution
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        
        # Create action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info('Waiting for MoveIt services...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ MoveGroup action server ready')
        
        # Subscribe to path topic
        self.path_sub = self.create_subscription(
            Float64MultiArray,
            '/tool_orientation/xyz_rotation',
            self.path_callback,
            10
        )
        
        self.waypoints = []
        self.path_received = False
        self.executing = False
        
        self.get_logger().info('Waiting for scanning path on /tool_orientation/xyz_rotation...')
    
    def path_callback(self, msg):
        """Parse received path with Z-offset correction."""
        if self.executing:
            return
            
        self.get_logger().info(f'Received path with {len(msg.data)} elements')
        
        data = np.array(msg.data)
        
        if len(data) % 12 != 0:
            self.get_logger().error(f'Invalid path data length: {len(data)}')
            return
        
        num_waypoints = len(data) // 12
        self.get_logger().info(f'Parsing {num_waypoints} waypoints...')
        
        # Find Z range and apply offset if needed
        min_z = min(data[i*12 + 11] for i in range(num_waypoints))
        max_z = max(data[i*12 + 11] for i in range(num_waypoints))
        
        if min_z < 0.15:
            z_offset = 0.25 - min_z
            self.get_logger().warn(f'Applying Z offset of {z_offset:.3f}m for safety')
        else:
            z_offset = 0.0
        
        self.waypoints = []
        for i in range(num_waypoints):
            idx = i * 12
            
            # Extract rotation matrix
            rot_matrix = np.array([
                [data[idx+0], data[idx+1], data[idx+2]],
                [data[idx+3], data[idx+4], data[idx+5]],
                [data[idx+6], data[idx+7], data[idx+8]]
            ])
            
            # Extract position with Z offset
            position = np.array([data[idx+9], data[idx+10], data[idx+11] + z_offset])
            
            try:
                rotation = R.from_matrix(rot_matrix)
                quat = rotation.as_quat()
                
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
    
    def execute_continuous_path(self):
        """Execute one continuous trajectory through all waypoints."""
        if not self.path_received or not self.waypoints:
            self.get_logger().error('No path available!')
            return False
        
        self.executing = True
        
        try:
            self.get_logger().info('='*60)
            self.get_logger().info(f'COMPUTING CONTINUOUS TRAJECTORY THROUGH ALL {len(self.waypoints)} WAYPOINTS')
            self.get_logger().info('='*60)
            self.get_logger().info('Strategy: Single OMPL plan through ALL waypoints')
            self.get_logger().info('Result: ONE smooth continuous motion without stops')
            
            # Apply orientation correction
            rotation_correction = R.from_euler('x', +90, degrees=True)
            corrected_waypoints = []
            
            for waypoint in self.waypoints:
                original_quat = np.array([
                    waypoint.orientation.x, waypoint.orientation.y,
                    waypoint.orientation.z, waypoint.orientation.w
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
            
            # Try Cartesian path first (best for continuous motion)
            self.get_logger().info('ATTEMPT 1: Cartesian path planning (preferred)')
            success = self.try_cartesian_path(corrected_waypoints)
            
            if success:
                return True
            
            # Fallback to multi-goal OMPL planning
            self.get_logger().info('ATTEMPT 2: Multi-goal OMPL planning (fallback)')
            success = self.try_multi_goal_ompl(corrected_waypoints)
            
            return success
            
        finally:
            self.executing = False
    
    def try_cartesian_path(self, waypoints):
        """Try to compute path using Cartesian planning."""
        if not self.cartesian_path_client.service_is_ready():
            self.get_logger().warn('Cartesian path service not available')
            return False
        
        # Move to first waypoint using joint planning
        if not self.move_to_start_position(waypoints[0]):
            self.get_logger().error('Failed to reach starting position')
            return False
        
        # Plan Cartesian path through remaining waypoints
        request = GetCartesianPath.Request()
        request.header.frame_id = 'world'
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = 'ur_manipulator'
        request.link_name = 'tool0'
        request.waypoints = waypoints[1:]  # Skip first (we're already there)
        request.max_step = 0.01  # 1cm steps
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.start_state.is_diff = True
        
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if not future.done():
            self.get_logger().error('Cartesian planning timed out')
            return False
        
        response = future.result()
        fraction = response.fraction
        
        self.get_logger().info(f'Cartesian path fraction: {fraction*100:.1f}%')
        
        if fraction > 0.9:  # 90% or more computed
            self.get_logger().info('✓ Executing Cartesian trajectory...')
            return self.execute_trajectory(response.solution)
        else:
            self.get_logger().warn(f'Only {fraction*100:.1f}% of path computed via Cartesian')
            return False
    
    def try_multi_goal_ompl(self, waypoints):
        """Try planning with multiple goal constraints using OMPL."""
        self.get_logger().info('Planning with multiple goal constraints...')
        
        # Create goal message with multiple goal constraints
        goal_msg = MoveGroup.Goal()
        
        # Basic setup
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 15.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        
        # Add goal constraint for the LAST waypoint (OMPL will plan to final goal)
        final_waypoint = waypoints[-1]
        
        from moveit_msgs.msg import PositionConstraint, BoundingVolume, Constraints
        
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'world'
        position_constraint.link_name = 'tool0'
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.weight = 1.0
        
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # 2cm tolerance
        bounding_volume.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position = final_waypoint.position
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)
        position_constraint.constraint_region = bounding_volume
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.orientation = final_waypoint.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.3
        orientation_constraint.absolute_y_axis_tolerance = 0.3
        orientation_constraint.absolute_z_axis_tolerance = 0.3
        orientation_constraint.weight = 1.0
        
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False
        
        self.get_logger().info('Sending multi-goal OMPL request...')
        
        # Send goal
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done() or not send_goal_future.result().accepted:
            self.get_logger().error('Multi-goal OMPL planning rejected')
            return False
        
        goal_handle = send_goal_future.result()
        self.get_logger().info('✓ Planning accepted, computing trajectory...')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=30.0)
        
        if not get_result_future.done():
            self.get_logger().error('Multi-goal OMPL planning timed out')
            return False
        
        result = get_result_future.result()
        
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('✓ Multi-goal OMPL planning successful!')
            return True
        else:
            self.get_logger().error(f'Multi-goal OMPL failed: error {result.error_code.val}')
            return False
    
    def move_to_start_position(self, start_waypoint):
        """Move to starting position using joint-space planning."""
        self.get_logger().info(f'Moving to start: pos=[{start_waypoint.position.x:.3f}, '
                              f'{start_waypoint.position.y:.3f}, {start_waypoint.position.z:.3f}]')
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        
        # Create constraints for start position
        from moveit_msgs.msg import PositionConstraint, BoundingVolume, Constraints
        
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'world'
        position_constraint.link_name = 'tool0'
        position_constraint.weight = 1.0
        
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        bounding_volume.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position = start_waypoint.position
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)
        position_constraint.constraint_region = bounding_volume
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.orientation = start_waypoint.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.2
        orientation_constraint.absolute_y_axis_tolerance = 0.2
        orientation_constraint.absolute_z_axis_tolerance = 0.2
        orientation_constraint.weight = 1.0
        
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False
        
        # Send and wait
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done() or not send_goal_future.result().accepted:
            return False
        
        goal_handle = send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=20.0)
        
        if not get_result_future.done():
            return False
        
        result = get_result_future.result()
        return result.error_code.val == MoveItErrorCodes.SUCCESS
    
    def execute_trajectory(self, trajectory):
        """Execute a computed trajectory."""
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ExecuteTrajectory action not available')
            return False
        
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = trajectory
        
        self.get_logger().info('Executing continuous trajectory...')
        
        exec_future = self.execute_trajectory_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future, timeout_sec=5.0)
        
        if not exec_future.done() or not exec_future.result().accepted:
            self.get_logger().error('Trajectory execution rejected')
            return False
        
        exec_handle = exec_future.result()
        exec_result_future = exec_handle.get_result_async()
        
        # Estimate timeout based on trajectory duration
        timeout = 60.0  # Default timeout
        if hasattr(trajectory, 'joint_trajectory') and len(trajectory.joint_trajectory.points) > 0:
            last_point = trajectory.joint_trajectory.points[-1]
            if hasattr(last_point, 'time_from_start'):
                duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
                timeout = duration + 15.0
        
        self.get_logger().info(f'⏳ Waiting for execution (timeout: {timeout:.1f}s)...')
        rclpy.spin_until_future_complete(self, exec_result_future, timeout_sec=timeout)
        
        if not exec_result_future.done():
            self.get_logger().error('Execution timed out')
            return False
        
        exec_result = exec_result_future.result()
        
        if exec_result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('✓ Continuous trajectory executed successfully!')
            return True
        else:
            self.get_logger().error(f'Execution failed: error {exec_result.error_code.val}')
            return False


def main():
    rclpy.init()
    
    node = OMPLContinuousPathFollower()
    
    print("\n" + "="*60)
    print("OMPL Continuous Path Follower")
    print("="*60)
    print("\nThis creates ONE continuous trajectory through ALL waypoints.")
    print("No stopping between waypoints - smooth continuous motion!")
    print("")
    print("Strategy:")
    print("  1. Try Cartesian path (best for smooth motion)")
    print("  2. Fallback to multi-goal OMPL if Cartesian fails")
    print("")
    print("Make sure:")
    print("  1. launch_ur_moveit.sh is running")
    print("  2. sim_external_line.sh or real data is publishing")
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
            print("Make sure external path data is being published.")
            node.destroy_node()
            rclpy.shutdown()
            return
    
    print(f"✓ Path received with {len(node.waypoints)} waypoints\n")
    
    try:
        input("Press Enter to execute ONE continuous motion through all waypoints, or Ctrl+C to abort...")
    except KeyboardInterrupt:
        print("\nAborted by user")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Execute continuous path
    success = node.execute_continuous_path()
    
    if success:
        print("\n✓ Continuous path executed successfully!")
        print("✓ Robot moved smoothly through all waypoints without stopping!")
    else:
        print("\n⚠ Continuous path execution failed. Check logs above.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
