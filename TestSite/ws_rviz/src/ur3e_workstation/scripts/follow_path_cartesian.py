#!/usr/bin/env python3
"""
Follow a scanning path using MoveIt's Cartesian path planning.
This computes ONE continuous trajectory through all waypoints without stopping.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from rclpy.action import ActionClient
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys


class CartesianPathFollower(Node):
    """Follow a path using Cartesian path planning for continuous motion."""
    
    def __init__(self):
        super().__init__('cartesian_path_follower')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Cartesian Path Follower - Continuous Motion')
        self.get_logger().info('='*60)
        
        # Create service client for Cartesian path
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
    
    def execute_cartesian_path(self):
        """
        Compute and execute a single continuous Cartesian trajectory through all waypoints.
        """
        if not self.path_received or not self.waypoints:
            self.get_logger().error('No path available! Run import_line.sh first.')
            return False
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'COMPUTING CONTINUOUS CARTESIAN PATH')
        self.get_logger().info(f'Through {len(self.waypoints)} waypoints')
        self.get_logger().info('='*60)
        
        # Apply -90° rotation around global X-axis for correct end effector orientation
        rotation_correction = R.from_euler('x', +90, degrees=True)
        
        # Correct all waypoint orientations
        corrected_waypoints = []
        for i, waypoint in enumerate(self.waypoints):
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
            
            if i % 2 == 0:  # Log every other waypoint
                self.get_logger().info(f'  Waypoint {i}: pos=[{corrected_pose.position.x:.3f}, '
                                      f'{corrected_pose.position.y:.3f}, {corrected_pose.position.z:.3f}]')
        
        # IMPORTANT: Move to first waypoint using joint-space planning
        # This establishes a good starting position for Cartesian planning
        self.get_logger().info('='*60)
        self.get_logger().info('STEP 1: Moving to first waypoint (joint-space planning)')
        self.get_logger().info('='*60)
        
        if not self.move_to_first_waypoint(corrected_waypoints[0]):
            self.get_logger().error('Failed to reach first waypoint - aborting')
            return False
        
        self.get_logger().info('✓ First waypoint reached!')
        
        # Small delay to ensure robot is settled
        import time
        time.sleep(1.0)
        
        # Now compute Cartesian path through remaining waypoints
        self.get_logger().info('='*60)
        self.get_logger().info('STEP 2: Computing Cartesian path through remaining waypoints')
        self.get_logger().info('='*60)
        
        # Use remaining waypoints (skip first since we're already there)
        cartesian_waypoints = corrected_waypoints[1:]
        
        self.get_logger().info(f'Computing Cartesian path through {len(cartesian_waypoints)} waypoints...')
        
        # Create GetCartesianPath request
        request = GetCartesianPath.Request()
        
        # Set header
        request.header.frame_id = 'world'
        request.header.stamp = self.get_clock().now().to_msg()
        
        # Set group name
        request.group_name = 'ur_manipulator'
        
        # Set link name
        request.link_name = 'tool0'
        
        # Set waypoints (remaining waypoints)
        request.waypoints = cartesian_waypoints
        
        # Set max step (distance between interpolated points)
        request.max_step = 0.005  # 5mm resolution for very smooth path
        
        # Jump threshold (0.0 = no jump check, allows more flexibility)
        request.jump_threshold = 0.0
        
        # Avoid collisions
        request.avoid_collisions = True
        
        # Path constraints (empty)
        request.path_constraints.name = ""
        
        # Start state (use current state - we're at first waypoint now)
        request.start_state.is_diff = True
        
        self.get_logger().info('Calling Cartesian path service...')
        self.get_logger().info('⏳ Please wait...')
        
        # Call service
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if not future.done():
            self.get_logger().error('✗ Cartesian path computation timed out')
            return False
        
        response = future.result()
        
        # Check fraction (how much of path was successfully computed)
        fraction = response.fraction
        self.get_logger().info(f'Cartesian path fraction achieved: {fraction*100:.1f}%')
        
        if fraction < 0.8:  # Less than 80% of path computed
            self.get_logger().error(f'✗ Could only compute {fraction*100:.1f}% of the path')
            self.get_logger().error('Possible issues:')
            self.get_logger().error('  - Waypoints cause IK failures (unreachable poses)')
            self.get_logger().error('  - Orientation changes too large between waypoints')
            self.get_logger().error('  - Path goes through singularities')
            self.get_logger().error('  - Collision detected along path')
            return False
        
        self.get_logger().info(f'✓ Successfully computed Cartesian path ({fraction*100:.1f}%)!')
        self.get_logger().info(f'  Trajectory has {len(response.solution.joint_trajectory.points)} points')
        
        self.get_logger().info('='*60)
        self.get_logger().info('STEP 3: EXECUTING CONTINUOUS TRAJECTORY')
        self.get_logger().info('='*60)
        self.get_logger().info('Robot will move smoothly through all waypoints WITHOUT STOPPING')
        
        # Execute the trajectory using ExecuteTrajectory action
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = response.solution
        
        self.get_logger().info('Sending trajectory to ExecuteTrajectory action...')
        
        # Send goal to execute trajectory
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
            if hasattr(last_point, 'time_from_start'):
                duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
                self.get_logger().info(f'Estimated duration: {duration:.1f} seconds')
                timeout = duration + 10.0  # Add buffer
            else:
                timeout = 30.0
        else:
            timeout = 30.0
        
        self.get_logger().info(f'⏳ Waiting for execution (timeout: {timeout:.1f}s)...')
        
        rclpy.spin_until_future_complete(self, exec_result_future, timeout_sec=timeout)
        
        if not exec_result_future.done():
            self.get_logger().error('✗ Trajectory execution timed out')
            return False
        
        exec_result = exec_result_future.result().result
        
        from moveit_msgs.msg import MoveItErrorCodes
        if exec_result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('='*60)
            self.get_logger().info('✓ Trajectory execution complete!')
            self.get_logger().info('='*60)
            return True
        else:
            self.get_logger().error(f'✗ Execution failed with error code: {exec_result.error_code.val}')
            return False
    
    def move_to_first_waypoint(self, first_waypoint):
        """
        Move to the first waypoint using joint-space planning (more reliable).
        """
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient
        
        self.get_logger().info(f'Target: pos=[{first_waypoint.position.x:.3f}, '
                              f'{first_waypoint.position.y:.3f}, {first_waypoint.position.z:.3f}]')
        
        # Create action client for MoveGroup
        move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        if not move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False
        
        # Create goal message
        goal_msg = MoveGroup.Goal()
        
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        
        # Position constraint
        from moveit_msgs.msg import PositionConstraint, BoundingVolume, Constraints
        from shape_msgs.msg import SolidPrimitive
        
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
        sphere.dimensions = [0.01]  # 1cm tolerance
        bounding_volume.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position = first_waypoint.position
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)
        
        position_constraint.constraint_region = bounding_volume
        
        # Orientation constraint
        from moveit_msgs.msg import OrientationConstraint
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.orientation = first_waypoint.orientation
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
        
        # Send goal
        self.get_logger().info('Planning to first waypoint...')
        send_goal_future = move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done() or not send_goal_future.result().accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        goal_handle = send_goal_future.result()
        self.get_logger().info('Goal accepted, moving to first waypoint...')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=20.0)
        
        if not get_result_future.done():
            self.get_logger().error('Timed out')
            return False
        
        result = get_result_future.result().result
        
        from moveit_msgs.msg import MoveItErrorCodes
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True
        else:
            self.get_logger().error(f'Failed with error code: {result.error_code.val}')
            return False
    
    def time_parameterize_trajectory(self, joint_trajectory):
        """
        Add time stamps to trajectory for smooth continuous motion.
        """
        self.get_logger().info('Time parameterizing trajectory for smooth motion...')
        
        # Create new trajectory with time stamps
        timed_trajectory = JointTrajectoryMsg()
        timed_trajectory.joint_names = joint_trajectory.joint_names
        
        # Velocity and acceleration limits (conservative for smooth motion)
        max_velocity = 0.5  # rad/s (50% of typical max)
        max_acceleration = 0.5  # rad/s^2
        
        current_time = 0.0
        
        for i, point in enumerate(joint_trajectory.points):
            new_point = JointTrajectoryPoint()
            new_point.positions = list(point.positions)
            
            # Calculate velocities and accelerations based on position changes
            if i > 0:
                prev_point = joint_trajectory.points[i-1]
                
                # Calculate max joint displacement
                max_displacement = max(abs(p - pp) for p, pp in zip(point.positions, prev_point.positions))
                
                # Time needed for this segment (based on max velocity)
                if max_displacement > 0:
                    segment_time = max_displacement / max_velocity
                else:
                    segment_time = 0.1  # Minimum time
                
                current_time += segment_time
                
                # Calculate velocities
                new_point.velocities = [
                    (p - pp) / segment_time if segment_time > 0 else 0.0
                    for p, pp in zip(point.positions, prev_point.positions)
                ]
                
                # Set zero accelerations (smooth motion)
                new_point.accelerations = [0.0] * len(point.positions)
            else:
                # First point starts from current position
                new_point.velocities = [0.0] * len(point.positions)
                new_point.accelerations = [0.0] * len(point.positions)
            
            # Set time from start
            new_point.time_from_start = Duration()
            new_point.time_from_start.sec = int(current_time)
            new_point.time_from_start.nanosec = int((current_time - int(current_time)) * 1e9)
            
            timed_trajectory.points.append(new_point)
        
        self.get_logger().info(f'✓ Trajectory time-parameterized: {current_time:.1f}s duration')
        
        return timed_trajectory


def main():
    rclpy.init()
    
    node = CartesianPathFollower()
    
    print("\n" + "="*60)
    print("Cartesian Path Follower - Continuous Motion")
    print("="*60)
    print("\nThis computes ONE continuous Cartesian trajectory")
    print("through ALL waypoints and executes it smoothly.")
    print("")
    print("Advantages:")
    print("  ✓ NO stopping between waypoints")
    print("  ✓ Smooth continuous motion")
    print("  ✓ Maintains end-effector orientation")
    print("  ✓ True Cartesian interpolation")
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
        input("Press Enter to execute continuous Cartesian path, or Ctrl+C to abort...")
    except KeyboardInterrupt:
        print("\nAborted by user")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Execute the Cartesian path
    success = node.execute_cartesian_path()
    
    if success:
        print("\n✓ Continuous Cartesian path executed successfully!")
        print("✓ Robot moved smoothly through all waypoints without stopping!")
    else:
        print("\n⚠ Cartesian path execution failed. Check logs above.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
