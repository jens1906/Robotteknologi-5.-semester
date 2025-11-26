#!/usr/bin/env python3
"""
Follow a scanning path using MoveIt's compute_cartesian_path service.
This creates smooth Cartesian motion through waypoints using the recommended method.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    DisplayTrajectory, Constraints, PositionConstraint, 
    OrientationConstraint, BoundingVolume, MoveItErrorCodes
)
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import sys
import copy


def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in radians."""
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot = np.abs(np.dot(q1, q2))
    dot = np.clip(dot, -1.0, 1.0)
    return 2.0 * np.arccos(dot)


class CartesianPathFollowerNew(Node):
    """Follow a path using MoveIt's compute_cartesian_path method."""
    
    def __init__(self):
        super().__init__('cartesian_path_follower_new')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Cartesian Path Follower (New Method)')
        self.get_logger().info('='*60)
        
        # Create service client for Cartesian path computation
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )
        
        self.get_logger().info('Waiting for /compute_cartesian_path service...')
        if not self.cartesian_path_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Cartesian path service not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ Cartesian path service ready')
        
        # Create action client for trajectory execution
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        
        self.get_logger().info('Waiting for /execute_trajectory action...')
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ExecuteTrajectory action not available!')
            sys.exit(1)
        
        self.get_logger().info('✓ ExecuteTrajectory action ready')
        
        # Create action client for moving to first waypoint
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
        
        # Create publisher for trajectory visualization
        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory,
            '/display_planned_path',
            10
        )
        
        # Subscribe to path topic (from external source or sim_external_line.sh)
        self.path_sub = self.create_subscription(
            Float64MultiArray,
            '/tool_orientation/xyz_rotation',
            self.path_callback,
            10
        )
        
        self.waypoints = []
        self.path_received = False
        
        self.get_logger().info('Waiting for scanning path on /tool_orientation/xyz_rotation...')
    
    def path_callback(self, msg):
        """Parse received path data."""
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

        self.waypoints = []
        positions = []

        # Parse positions first for Z-offset calculation
        for i in range(num_waypoints):
            idx = i * 12
            try:
                # Position-first format: [x, y, z, r11...r33]
                position = np.array(data[idx:idx+3])
                positions.append(position)
            except Exception as e:
                self.get_logger().warn(f'Parsing waypoint {i} failed: {e}')
                positions.append(np.array([np.nan, np.nan, np.nan]))

        # Compute Z-offset for safety
        min_z = float('inf')
        max_z = float('-inf')
        for p in positions:
            try:
                min_z = min(min_z, float(p[2]))
                max_z = max(max_z, float(p[2]))
            except Exception:
                continue

        z_offset = 0.0
        if min_z < 0.05:
            desired_min = 0.15
            z_offset = desired_min - min_z
            if z_offset > 0.6:
                self.get_logger().error(f'Computed Z offset {z_offset:.3f}m is unexpectedly large')
                self.waypoints = []
                self.path_received = True
                return
            self.get_logger().warn(f'Applying Z offset of {z_offset:.3f}m for safety')
        else:
            self.get_logger().info(f'Waypoints Z range [{min_z:.3f}, {max_z:.3f}] looks good')

        # Parse and create Pose objects
        for i in range(num_waypoints):
            idx = i * 12
            try:
                # Position-first format: [x, y, z, r11...r33]
                position = np.array(data[idx:idx+3])
                rot_matrix = np.array(data[idx+3:idx+12]).reshape((3, 3))

                position = position.copy()
                position[2] = position[2] + z_offset

                # Sanitize rotation matrix using SVD
                try:
                    U, S, Vt = np.linalg.svd(rot_matrix)
                    rot_fixed = U @ Vt
                    if np.linalg.det(rot_fixed) < 0:
                        U[:, -1] *= -1
                        rot_fixed = U @ Vt
                except Exception:
                    rot_fixed = rot_matrix

                det_val = np.linalg.det(rot_fixed)
                if det_val <= 0.0 or not np.isfinite(det_val):
                    raise ValueError(f'Non-positive determinant: {det_val}')

                rotation = R.from_matrix(rot_fixed)
                quat = rotation.as_quat()

                pose = Pose()
                pose.position.x = float(position[0])
                pose.position.y = float(position[1])
                pose.position.z = float(position[2])
                pose.orientation.x = float(quat[0])
                pose.orientation.y = float(quat[1])
                pose.orientation.z = float(quat[2])
                pose.orientation.w = float(quat[3])

                self.waypoints.append(pose)

            except Exception as e:
                self.get_logger().warn(f'Failed to convert waypoint {i}: {e}')
                continue

        self.get_logger().info(f'✓ Successfully parsed {len(self.waypoints)} waypoints')
        if z_offset > 0:
            self.get_logger().info(f'✓ Applied {z_offset:.3f}m Z-offset for safety')
        self.path_received = True
    
    def smooth_orientations(self, waypoints, max_angle_deg=20.0):
        """
        Insert intermediate waypoints where orientation changes are too large.
        Uses SLERP for smooth orientation transitions.
        """
        if len(waypoints) < 2:
            return waypoints
        
        max_angle_rad = np.deg2rad(max_angle_deg)
        smoothed = [copy.deepcopy(waypoints[0])]
        
        for i in range(len(waypoints) - 1):
            curr_pose = waypoints[i]
            next_pose = waypoints[i + 1]
            
            # Extract quaternions
            q_curr = np.array([curr_pose.orientation.x, curr_pose.orientation.y,
                              curr_pose.orientation.z, curr_pose.orientation.w])
            q_next = np.array([next_pose.orientation.x, next_pose.orientation.y,
                              next_pose.orientation.z, next_pose.orientation.w])
            
            # Compute angular distance
            angle_dist = quaternion_angular_distance(q_curr, q_next)
            
            # If orientation change is too large, insert intermediate waypoints
            if angle_dist > max_angle_rad:
                num_subdivisions = int(np.ceil(angle_dist / max_angle_rad))
                
                self.get_logger().info(f'Waypoint {i}->{i+1}: Large orientation change '
                                      f'({np.rad2deg(angle_dist):.1f}°) - inserting {num_subdivisions} '
                                      f'intermediate points')
                
                # Create SLERP interpolator
                rotations = R.from_quat([q_curr, q_next])
                times = np.array([0.0, 1.0])
                slerp = Slerp(times, rotations)
                
                # Interpolate positions linearly
                p_curr = np.array([curr_pose.position.x, curr_pose.position.y, curr_pose.position.z])
                p_next = np.array([next_pose.position.x, next_pose.position.y, next_pose.position.z])
                
                # Insert intermediate waypoints
                for j in range(1, num_subdivisions + 1):
                    t = j / (num_subdivisions + 1)
                    
                    # Interpolate orientation with SLERP
                    interp_rot = slerp([t])
                    interp_quat = interp_rot.as_quat()[0]
                    
                    # Interpolate position linearly
                    interp_pos = (1 - t) * p_curr + t * p_next
                    
                    # Create intermediate pose
                    interp_pose = Pose()
                    interp_pose.position.x = float(interp_pos[0])
                    interp_pose.position.y = float(interp_pos[1])
                    interp_pose.position.z = float(interp_pos[2])
                    interp_pose.orientation.x = float(interp_quat[0])
                    interp_pose.orientation.y = float(interp_quat[1])
                    interp_pose.orientation.z = float(interp_quat[2])
                    interp_pose.orientation.w = float(interp_quat[3])
                    
                    smoothed.append(interp_pose)
            
            # Add the next waypoint
            smoothed.append(copy.deepcopy(next_pose))
        
        self.get_logger().info(f'Orientation smoothing: {len(waypoints)} -> {len(smoothed)} waypoints')
        return smoothed
    
    def execute_cartesian_path(self):
        """
        Compute and execute Cartesian path using MoveIt's compute_cartesian_path service.
        """
        if not self.path_received or not self.waypoints:
            self.get_logger().error('No path available! Run import_line.sh first.')
            return False
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'COMPUTING CARTESIAN PATH (NEW METHOD)')
        self.get_logger().info(f'Through {len(self.waypoints)} waypoints')
        self.get_logger().info('='*60)
        
        try:
            # NO orientation correction - use orientations exactly as published
            # The external source should provide correct orientations for tool0
            
            # Parse all waypoints
            corrected_waypoints = []
            for i, waypoint in enumerate(self.waypoints):
                # Use waypoint as-is, no rotation correction
                corrected_waypoints.append(waypoint)
            
            # Smooth large orientation changes
            self.get_logger().info('Analyzing orientation changes...')
            corrected_waypoints = self.smooth_orientations(corrected_waypoints, max_angle_deg=15.0)
            
            # STEP 1: Move to first waypoint using MoveGroup action
            self.get_logger().info('='*60)
            self.get_logger().info('STEP 1: Moving to first waypoint')
            self.get_logger().info('='*60)
            
            if not self.move_to_first_waypoint(corrected_waypoints[0]):
                self.get_logger().error('Failed to reach first waypoint')
                return False
            
            self.get_logger().info('✓ First waypoint reached!')
            
            # Small delay to ensure robot is settled
            import time
            time.sleep(1.0)
            
            # STEP 2: Compute Cartesian path through remaining waypoints
            self.get_logger().info('='*60)
            self.get_logger().info('STEP 2: Computing Cartesian path')
            self.get_logger().info('='*60)
            
            # Use remaining waypoints (skip first since we're already there)
            cartesian_waypoints = corrected_waypoints[1:]
            
            self.get_logger().info(f'Computing Cartesian path through {len(cartesian_waypoints)} waypoints...')
            self.get_logger().info('Using 1cm resolution (max_step=0.01)...')
            
            # Create GetCartesianPath request
            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            request.header.stamp = self.get_clock().now().to_msg()
            request.group_name = 'ur_manipulator'
            request.link_name = 'tool0'
            request.waypoints = cartesian_waypoints
            request.max_step = 0.01  # 1cm resolution
            request.jump_threshold = 0.0  # Disabled for flexibility
            request.avoid_collisions = True
            request.path_constraints = Constraints()  # Empty for maximum flexibility
            request.start_state.is_diff = True
            
            # Call service
            self.get_logger().info('Calling Cartesian path service...')
            future = self.cartesian_path_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if not future.done():
                self.get_logger().error('✗ Cartesian path computation timed out')
                return False
            
            response = future.result()
            fraction = response.fraction
            
            self.get_logger().info(f'Cartesian path fraction achieved: {fraction*100:.1f}%')
            
            if fraction < 0.1:
                self.get_logger().error(f'✗ Could only compute {fraction*100:.1f}% of the path')
                self.get_logger().error('Possible issues:')
                self.get_logger().error('  - Waypoints unreachable (IK failures)')
                self.get_logger().error('  - Orientation changes too large')
                self.get_logger().error('  - Path goes through singularities')
                self.get_logger().error('  - Collision detected')
                return False
            elif fraction < 0.95:
                self.get_logger().warn(f'⚠ Computed {fraction*100:.1f}% of Cartesian path')
                self.get_logger().info('Will execute computed portion')
            else:
                self.get_logger().info(f'✓ Successfully computed full Cartesian path ({fraction*100:.1f}%)!')
            
            # Display trajectory in RViz
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory.append(response.solution)
            self.display_trajectory_publisher.publish(display_trajectory)
            
            self.get_logger().info('✓ Trajectory published to RViz for visualization')
            
            # STEP 3: Execute the plan
            self.get_logger().info('='*60)
            self.get_logger().info('STEP 3: EXECUTING CARTESIAN TRAJECTORY')
            self.get_logger().info('='*60)
            self.get_logger().info('Robot will move smoothly through waypoints...')
            
            # Execute using ExecuteTrajectory action
            exec_goal = ExecuteTrajectory.Goal()
            exec_goal.trajectory = response.solution
            
            exec_future = self.execute_trajectory_client.send_goal_async(exec_goal)
            rclpy.spin_until_future_complete(self, exec_future, timeout_sec=5.0)
            
            if not exec_future.done() or not exec_future.result().accepted:
                self.get_logger().error('✗ Trajectory execution goal rejected')
                return False
            
            exec_handle = exec_future.result()
            self.get_logger().info('✓ Trajectory accepted, executing...')
            
            # Wait for execution to complete
            exec_result_future = exec_handle.get_result_async()
            
            # Estimate duration for timeout
            if len(response.solution.joint_trajectory.points) > 0:
                last_point = response.solution.joint_trajectory.points[-1]
                duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
                timeout = duration + 10.0
            else:
                timeout = 30.0
            
            self.get_logger().info(f'⏳ Waiting for execution (timeout: {timeout:.1f}s)...')
            rclpy.spin_until_future_complete(self, exec_result_future, timeout_sec=timeout)
            
            if not exec_result_future.done():
                self.get_logger().error('✗ Trajectory execution timed out')
                return False
            
            exec_result = exec_result_future.result().result
            
            if exec_result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('='*60)
                self.get_logger().info('✓ Cartesian path executed successfully!')
                self.get_logger().info('='*60)
                return True
            else:
                self.get_logger().error(f'✗ Execution failed with error code: {exec_result.error_code.val}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error during Cartesian path execution: {e}')
            import traceback
            traceback.print_exc()
            return False
    
    def move_to_first_waypoint(self, first_waypoint):
        """Move to the first waypoint using MoveGroup action with relaxed constraints."""
        self.get_logger().info(f'Target: pos=[{first_waypoint.position.x:.3f}, '
                              f'{first_waypoint.position.y:.3f}, {first_waypoint.position.z:.3f}]')
        
        # Try with progressively more relaxed constraints
        tolerance_levels = [
            {'pos': 0.01, 'orient': 0.1, 'name': 'strict'},      # 1cm, ~6°
            {'pos': 0.02, 'orient': 0.3, 'name': 'normal'},      # 2cm, ~17°
            {'pos': 0.03, 'orient': 0.5, 'name': 'relaxed'},     # 3cm, ~29°
            {'pos': 0.05, 'orient': 1.0, 'name': 'very relaxed'} # 5cm, ~57°
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
            sphere_pose.position = first_waypoint.position
            sphere_pose.orientation.w = 1.0
            bounding_volume.primitive_poses.append(sphere_pose)
            position_constraint.constraint_region = bounding_volume
            
            # Orientation constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = 'world'
            orientation_constraint.link_name = 'tool0'
            orientation_constraint.orientation = first_waypoint.orientation
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
                self.get_logger().warn(f'Planning goal rejected with {tolerances["name"]} constraints')
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
                self.get_logger().info(f'✓ Successfully reached first waypoint with {tolerances["name"]} constraints!')
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
        self.get_logger().error('Failed to reach first waypoint with all constraint levels')
        self.get_logger().error('Possible issues:')
        self.get_logger().error('  - Waypoint orientation not achievable by robot')
        self.get_logger().error('  - Position outside robot workspace')
        self.get_logger().error('  - Robot starting position causes collision')
        return False


def main():
    rclpy.init()
    
    node = CartesianPathFollowerNew()
    
    print("\n" + "="*60)
    print("Cartesian Path Follower (New Method)")
    print("="*60)
    print("\nUses MoveIt's compute_cartesian_path() service")
    print("for smooth continuous Cartesian motion.")
    print("")
    print("Advantages:")
    print("  ✓ Simple and recommended API")
    print("  ✓ Smooth continuous motion")
    print("  ✓ Maintains end-effector orientation")
    print("  ✓ 1cm interpolation resolution")
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
    
    print(f"✓ Path received with {len(node.waypoints)} waypoints\n")
    
    try:
        input("Press Enter to execute Cartesian path, or Ctrl+C to abort...")
    except KeyboardInterrupt:
        print("\nAborted by user")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Execute the Cartesian path
    success = node.execute_cartesian_path()
    
    if success:
        print("\n✓ Cartesian path executed successfully!")
    else:
        print("\n⚠ Cartesian path execution failed. Check logs above.")
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
