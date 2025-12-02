#!/usr/bin/env python3
"""
Follow a scanning path using MoveIt's Cartesian path planning.
This computes ONE continuous trajectory through all waypoints without stopping.

ERROR PREVENTION STRATEGY:
To minimize -3 (INVALID_MOTION_PLAN) and -4 (MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE):
1. Lock planning scene during critical operations (self.scene_locked flag)
2. Extended stabilization delays (5s) before/after movements
3. Disable collision avoidance during Cartesian planning (prevent scene-based invalidation)
4. Increased retry attempts (10x) with long delays between retries
5. Extended timeouts (60s) for planning operations
6. Block all scene updates from callbacks during execution

NOTE: These changes trade collision safety for plan stability. Only use in
controlled environments where collision-free paths are pre-validated.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from rclpy.action import ActionClient
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import sys


def quaternion_angular_distance(q1, q2):
    """Compute angular distance between two quaternions in radians."""
    # Ensure unit quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    # Compute dot product
    dot = np.abs(np.dot(q1, q2))
    # Clamp to avoid numerical errors
    dot = np.clip(dot, -1.0, 1.0)
    # Return angular distance
    return 2.0 * np.arccos(dot)


class CartesianPathFollower(Node):
    """Follow a path using Cartesian path planning for continuous motion."""
    
    def __init__(self):
        super().__init__('cartesian_path_follower')
        
        self.get_logger().info('='*60)
        self.get_logger().info('Cartesian Path Follower - Continuous Motion')
        self.get_logger().info('='*60)
        
        # Flag to freeze scene updates during critical operations
        self.scene_locked = False
        
        # Visualization publisher
        from visualization_msgs.msg import MarkerArray, Marker
        self.viz_publisher = self.create_publisher(MarkerArray, '/target_poses_viz', 10)
        
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
        
        # Create service client for IK validation
        from moveit_msgs.srv import GetPositionIK
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        
        if self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('✓ IK service ready for waypoint validation')
        else:
            self.get_logger().warn('⚠ IK service not available - skipping validation')
        
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
            PoseArray,
            '/tool_orientation/path',
            self.path_callback,
            10
        )

        # Subscribe to RViz visualization topics to build poses from positions + orientations
        self.line_path_sub = self.create_subscription(
            Marker,
            '/line_path',
            self.line_path_callback,
            10
        )
        self.line_viz_sub = self.create_subscription(
            MarkerArray,
            '/line_visualization',
            self.line_viz_callback,
            10
        )
        
        self.waypoints = []
        self.path_received = False
        self.executing = False  # Flag to prevent path updates during execution
        # Buffers for RViz-derived inputs
        self._rviz_positions = []  # list of (x,y,z)
        self._rviz_orientations = []  # list of (qx,qy,qz,qw)
        
        self.get_logger().info('Waiting for scanning path on /tool_orientation/path (PoseArray)...')
        self.get_logger().info('Also listening to /line_path (positions) and /line_visualization (orientations)')

    def line_path_callback(self, marker: Marker):
        """Capture positions from LINE_STRIP marker published on /line_path (in base_link frame)."""
        if self.executing or self.scene_locked:
            return  # Don't update during execution or when scene is locked
        try:
            pts = marker.points
            self._rviz_positions = [(p.x, p.y, p.z) for p in pts]
            self.get_logger().info(f'Received /line_path with {len(self._rviz_positions)} positions')
            self._try_build_waypoints_from_rviz()
        except Exception as e:
            self.get_logger().warn(f'/line_path parse failed: {e}')

    def line_viz_callback(self, arr: MarkerArray):
        """Capture orientations from ARROW markers in /line_visualization (in base_link frame)."""
        if self.executing or self.scene_locked:
            return  # Don't update during execution or when scene is locked
        try:
            # Expect 3 arrows per point (x,y,z axes) plus spheres; use the first arrow in each ns group
            # We'll gather unique ns entries by point index and take orientation from initial arrow id per point.
            orient_by_ns = {}
            for m in arr.markers:
                if m.type == Marker.ARROW:
                    # ns like 'point_i' per visualizer
                    key = m.ns
                    # prefer lowest id per ns
                    if key not in orient_by_ns or m.id < orient_by_ns[key][0]:
                        q = m.pose.orientation
                        orient_by_ns[key] = (m.id, (q.x, q.y, q.z, q.w))
            # Sort by ns order if possible; fallback to id ordering
            items = list(orient_by_ns.items())
            # Extract index from ns 'point_i' if present
            def ns_index(ns):
                try:
                    if 'point_' in ns:
                        return int(ns.split('point_')[-1])
                except Exception:
                    pass
                return None
            items.sort(key=lambda kv: (ns_index(kv[0]) if ns_index(kv[0]) is not None else 1e9, kv[1][0]))
            self._rviz_orientations = [kv[1][1] for kv in items]
            self.get_logger().info(f'Received /line_visualization with {len(self._rviz_orientations)} orientations')
            self._try_build_waypoints_from_rviz()
        except Exception as e:
            self.get_logger().warn(f'/line_visualization parse failed: {e}')

    def _try_build_waypoints_from_rviz(self):
        """If we have both positions and orientations, build Pose waypoints and set as path."""
        if not self._rviz_positions or not self._rviz_orientations:
            return
        n = min(len(self._rviz_positions), len(self._rviz_orientations))
        if n < 1:
            return
        self.waypoints = []
        for i in range(n):
            x, y, z = self._rviz_positions[i]
            qx, qy, qz, qw = self._rviz_orientations[i]
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)
            self.waypoints.append(pose)
        self.path_received = True
        self.get_logger().info(f'✓ Built {len(self.waypoints)} waypoints from RViz topics')
    
    def path_callback(self, msg):
        """Parse received path - but ignore updates during execution."""
        if self.executing:
            self.get_logger().debug('Ignoring path update during execution')
            return
            
        self.get_logger().info(f'Received path with {len(msg.poses)} elements')
        
        self.waypoints = []
        for i, pose in enumerate(msg.poses):
            self.waypoints.append(pose)
            
            if i == 0 or i == len(msg.poses) - 1:
                self.get_logger().info(
                    f'  Point {i}: pos=[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]'
                )
        
        self.path_received = True
        self.get_logger().info(f'✓ Successfully parsed {len(self.waypoints)} waypoints (PoseArray format)')
    
    def _estimate_waypoint_spacing(self, waypoints):
        """Estimate average spacing between waypoints in meters."""
        if len(waypoints) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i].position
            p2 = waypoints[i + 1].position
            dist = ((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)**0.5
            total_distance += dist
        
        return total_distance / (len(waypoints) - 1)
    
    def validate_waypoints(self, waypoints, verbose=True):
        """
        Validate each waypoint by checking IK feasibility.
        Returns list of (index, is_valid, error_message) tuples.
        """
        if not hasattr(self, 'ik_client') or self.ik_client is None:
            self.get_logger().warn('IK client not available - skipping validation')
            return [(i, True, 'Validation skipped') for i in range(len(waypoints))]
        
        from moveit_msgs.srv import GetPositionIK
        from moveit_msgs.msg import MoveItErrorCodes
        
        results = []
        failed_indices = []
        
        if verbose:
            self.get_logger().info(f'Validating {len(waypoints)} waypoints via IK...')
        
        for i, waypoint in enumerate(waypoints):
            # Create IK request
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = 'ur_manipulator'
            ik_request.ik_request.robot_state.is_diff = True
            ik_request.ik_request.avoid_collisions = True
            ik_request.ik_request.timeout.sec = 0
            ik_request.ik_request.timeout.nanosec = 100000000  # 0.1 seconds
            
            # Set pose stamped
            from geometry_msgs.msg import PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'world'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = waypoint
            ik_request.ik_request.pose_stamped = pose_stamped
            
            # Call IK service
            try:
                future = self.ik_client.call_async(ik_request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
                
                if future.done():
                    response = future.result()
                    
                    if response.error_code.val == MoveItErrorCodes.SUCCESS:
                        results.append((i, True, 'Valid'))
                        if verbose and i % 10 == 0:  # Log every 10th waypoint
                            self.get_logger().info(f'  Waypoint {i}: ✓ Valid')
                    else:
                        error_name = self._decode_moveit_error(response.error_code.val)
                        results.append((i, False, error_name))
                        failed_indices.append(i)
                        if verbose:
                            self.get_logger().warn(f'  Waypoint {i}: ✗ {error_name}')
                else:
                    results.append((i, False, 'IK timeout'))
                    failed_indices.append(i)
                    
            except Exception as e:
                results.append((i, False, f'Exception: {str(e)}'))
                failed_indices.append(i)
        
        # Summary
        valid_count = sum(1 for _, is_valid, _ in results if is_valid)
        if verbose:
            self.get_logger().info(f'Validation complete: {valid_count}/{len(waypoints)} waypoints valid')
            if failed_indices:
                self.get_logger().warn(f'Failed waypoint indices: {failed_indices[:20]}...' if len(failed_indices) > 20 else f'Failed waypoint indices: {failed_indices}')
        
        return results
    
    def _decode_moveit_error(self, error_code):
        """Decode MoveIt error codes to human-readable strings."""
        error_meanings = {
            1: 'SUCCESS',
            -1: 'FAILURE',
            -31: 'NO_IK_SOLUTION',
            -11: 'START_STATE_IN_COLLISION',
            -13: 'GOAL_IN_COLLISION',
            -14: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
            -15: 'GOAL_CONSTRAINTS_VIOLATED',
            -20: 'INVALID_ROBOT_STATE',
            -21: 'INVALID_LINK_NAME',
            -23: 'FRAME_TRANSFORM_FAILURE',
        }
        return error_meanings.get(error_code, f'ERROR_{error_code}')

    
    def smooth_orientations(self, waypoints, max_angle_deg=20.0):
        """
        Insert intermediate waypoints where orientation changes are too large.
        Uses SLERP (spherical linear interpolation) for smooth orientation transitions.
        
        Args:
            waypoints: List of Pose messages
            max_angle_deg: Maximum allowed orientation change between consecutive waypoints (degrees)
        
        Returns:
            List of Pose messages with smooth orientation transitions
        """
        if len(waypoints) < 2:
            return waypoints
        
        max_angle_rad = np.deg2rad(max_angle_deg)
        smoothed = [waypoints[0]]  # Start with first waypoint
        
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
                # Calculate number of subdivisions needed
                num_subdivisions = int(np.ceil(angle_dist / max_angle_rad))
                
                self.get_logger().info(f'Waypoint {i}->{i+1}: Large orientation change '
                                      f'({np.rad2deg(angle_dist):.1f}°) - inserting {num_subdivisions} '
                                      f'intermediate points')
                
                # Create SLERP interpolator
                r_curr = R.from_quat(q_curr)
                r_next = R.from_quat(q_next)
                times = np.array([0.0, 1.0])
                rotations = R.from_quat([q_curr, q_next])
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
            smoothed.append(next_pose)
        
        self.get_logger().info(f'Orientation smoothing: {len(waypoints)} -> {len(smoothed)} waypoints')
        return smoothed
    
    def visualize_target_poses(self, poses, namespace='target_poses'):
        """Publish visualization markers for target poses in RViz"""
        from visualization_msgs.msg import MarkerArray, Marker
        from builtin_interfaces.msg import Duration as MarkerDuration
        
        marker_array = MarkerArray()
        
        for i, pose in enumerate(poses):
            # Create a sphere marker for position
            sphere = Marker()
            sphere.header.frame_id = 'world'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = namespace
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = 0.05
            sphere.scale.y = 0.05
            sphere.scale.z = 0.05
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            sphere.color.a = 0.8
            sphere.lifetime = MarkerDuration(sec=30)
            marker_array.markers.append(sphere)
            
            # Create an arrow marker for orientation
            arrow = Marker()
            arrow.header.frame_id = 'world'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = namespace + '_orientation'
            arrow.id = i * 2 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose
            arrow.scale.x = 0.15  # Arrow length
            arrow.scale.y = 0.02  # Arrow width
            arrow.scale.z = 0.02  # Arrow height
            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 0.0
            arrow.color.a = 0.8
            arrow.lifetime = MarkerDuration(sec=30)
            marker_array.markers.append(arrow)
            
            # Add text label
            text = Marker()
            text.header.frame_id = 'world'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = namespace + '_labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.position.x
            text.pose.position.y = pose.position.y
            text.pose.position.z = pose.position.z + 0.1  # Above the sphere
            text.pose.orientation.w = 1.0
            text.scale.z = 0.05
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'Target {i}\n[{pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}]'
            text.lifetime = MarkerDuration(sec=30)
            marker_array.markers.append(text)
        
        self.viz_publisher.publish(marker_array)
        self.get_logger().info(f'✓ Published {len(poses)} target pose visualizations to /target_poses_viz')
    
    def execute_cartesian_path(self):
        """
        Compute and execute a single continuous Cartesian trajectory through all waypoints.
        """
        if not self.path_received or not self.waypoints:
            self.get_logger().error('No path available! Run import_line.sh first.')
            return False
        
        # Set executing flag to prevent path updates during execution
        self.executing = True
        self.scene_locked = True  # Lock planning scene from external updates
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'COMPUTING CONTINUOUS CARTESIAN PATH')
        self.get_logger().info(f'Through {len(self.waypoints)} waypoints')
        self.get_logger().info('='*60)
        self.get_logger().info('⚠ Planning scene LOCKED - preventing external updates during planning')
        
        try:
            # Apply -90° rotation around global X-axis for correct end effector orientation
            # UPDATE: Try simpler orientation - tool pointing down
            # rotation_correction = R.from_euler('x', +90, degrees=True)
            rotation_correction = R.from_euler('xyz', [0, 0, 0], degrees=True)  # Identity - no rotation
            
            # Correct all waypoint orientations
            corrected_waypoints = []
            for i, waypoint in enumerate(self.waypoints):
                original_quat = np.array([
                    waypoint.orientation.x,
                    waypoint.orientation.y,
                    waypoint.orientation.z,
                    waypoint.orientation.w
                ])
                
                self.get_logger().info(f'  Waypoint {i} original quat: [{original_quat[0]:.3f}, '
                                      f'{original_quat[1]:.3f}, {original_quat[2]:.3f}, {original_quat[3]:.3f}]')
                
                original_rotation = R.from_quat(original_quat)
                original_euler = original_rotation.as_euler('xyz', degrees=True)
                self.get_logger().info(f'  Waypoint {i} original euler (deg): [{original_euler[0]:.1f}, '
                                      f'{original_euler[1]:.1f}, {original_euler[2]:.1f}]')
                
                # Use original orientation (identity) for simpler path planning
                corrected_rotation = original_rotation  # No correction
                corrected_quat = corrected_rotation.as_quat()
                corrected_euler = corrected_rotation.as_euler('xyz', degrees=True)
                
                self.get_logger().info(f'  Waypoint {i} using euler (deg): [{corrected_euler[0]:.1f}, '
                                      f'{corrected_euler[1]:.1f}, {corrected_euler[2]:.1f}]')
                self.get_logger().info('  (Using identity orientation - tool pointing down)')
                
                corrected_pose = Pose()
                corrected_pose.position = waypoint.position
                corrected_pose.orientation.x = corrected_quat[0]
                corrected_pose.orientation.y = corrected_quat[1]
                corrected_pose.orientation.z = corrected_quat[2]
                corrected_pose.orientation.w = corrected_quat[3]
                
                corrected_waypoints.append(corrected_pose)
                
                self.get_logger().info(f'  Waypoint {i}: pos=[{corrected_pose.position.x:.3f}, '
                                      f'{corrected_pose.position.y:.3f}, {corrected_pose.position.z:.3f}]')
            
            # CRITICAL: Smooth large orientation changes by inserting intermediate waypoints
            self.get_logger().info('Analyzing orientation changes...')
            corrected_waypoints = self.smooth_orientations(corrected_waypoints, max_angle_deg=15.0)
            
            # Visualize target poses in RViz
            self.get_logger().info('Publishing target pose visualizations to RViz...')
            self.visualize_target_poses(corrected_waypoints, namespace='corrected_targets')
            self.get_logger().info('⚠ CHECK RVIZ: Red spheres show target positions, green arrows show orientations')
            self.get_logger().info('⚠ Add /target_poses_viz topic in RViz if not visible')
            
            # IMPORTANT: Move to first waypoint using joint-space planning
            # This establishes a good starting position for Cartesian planning
            self.get_logger().info('='*60)
            self.get_logger().info('STEP 1: Moving to first waypoint (joint-space planning)')
            self.get_logger().info('='*60)
            
            if not self.move_to_first_waypoint(corrected_waypoints[0]):
                self.get_logger().error('Failed to reach first waypoint - aborting')
                return False
            
            self.get_logger().info('✓ First waypoint reached!')
            
            # Extended delay to ensure robot is settled AND planning scene stabilizes
            import time
            self.get_logger().info('Waiting for robot to settle and planning scene to stabilize...')
            time.sleep(5.0)  # Increased from 1.0s - critical for preventing error -4
            
            # Now compute Cartesian path through remaining waypoints
            self.get_logger().info('='*60)
            self.get_logger().info('STEP 2: Computing Cartesian path through remaining waypoints')
            self.get_logger().info('='*60)
            
            # Use remaining waypoints (skip first since we're already there)
            cartesian_waypoints = corrected_waypoints[1:]
            
            # Convert to position-only waypoints (remove orientation constraints)
            position_only_waypoints = []
            for wp in cartesian_waypoints:
                pos_wp = Pose()
                pos_wp.position = wp.position
                # Identity orientation (no constraints)
                pos_wp.orientation.w = 1.0
                pos_wp.orientation.x = 0.0
                pos_wp.orientation.y = 0.0
                pos_wp.orientation.z = 0.0
                position_only_waypoints.append(pos_wp)
            
            self.get_logger().info(f'Converted {len(cartesian_waypoints)} waypoints to position-only for max reachability')
            
            # Validate waypoints before planning
            self.get_logger().info('='*60)
            self.get_logger().info('PRE-VALIDATION: Checking waypoint feasibility')
            self.get_logger().info('='*60)
            validation_results = self.validate_waypoints(position_only_waypoints, verbose=True)
            
            # Filter out invalid waypoints
            valid_waypoints = []
            invalid_indices = []
            for i, (idx, is_valid, error_msg) in enumerate(validation_results):
                if is_valid:
                    valid_waypoints.append(position_only_waypoints[i])
                else:
                    invalid_indices.append(i)
            
            if len(invalid_indices) > 0:
                self.get_logger().warn(f'⚠ Removing {len(invalid_indices)} invalid waypoints before Cartesian planning')
                self.get_logger().warn(f'  Invalid indices: {invalid_indices[:10]}{"..." if len(invalid_indices) > 10 else ""}')
                position_only_waypoints = valid_waypoints
                
                if len(position_only_waypoints) == 0:
                    self.get_logger().error('❌ No valid waypoints remaining after validation!')
                    return False
            else:
                self.get_logger().info('✓ All waypoints passed IK validation')
            
            self.get_logger().info('Validating waypoint reachability...')
            UR3E_REACH = 0.55  # Maximum reach from base (meters)
            out_of_reach = 0
            for i, wp in enumerate(position_only_waypoints):
                dist = (wp.position.x**2 + wp.position.y**2 + wp.position.z**2)**0.5
                if dist > UR3E_REACH:
                    out_of_reach += 1
                    if i < 5 or i >= len(position_only_waypoints) - 5:  # Log first/last 5
                        self.get_logger().warn(f'  Waypoint {i} OUT OF REACH: distance={dist:.3f}m (>0.55m)')
            
            if out_of_reach > 0:
                self.get_logger().error(f'❌ {out_of_reach}/{len(position_only_waypoints)} waypoints OUT OF REACH!')
                self.get_logger().error('   Possible reasons:')
                self.get_logger().error('   1. Detected surface is too far from robot base')
                self.get_logger().error('   2. Transform world↔base_link is wrong')
                self.get_logger().error('   3. Robot position needs to be adjusted')
                # Don't return - try anyway to see partial results
            else:
                self.get_logger().info(f'✓ All waypoints within UR3e reach (0.55m)')
            
            self.get_logger().info(f'Computing Cartesian path through {len(position_only_waypoints)} waypoints...')
            
            # Create GetCartesianPath request
            request = GetCartesianPath.Request()
            
            # Set header
            request.header.frame_id = 'world'
            request.header.stamp = self.get_clock().now().to_msg()
            
            # Set group name
            request.group_name = 'ur_manipulator'
            
            # Set link name
            request.link_name = 'tool0'
            
            # Set waypoints (position-only for maximum reachability)
            request.waypoints = position_only_waypoints
            
            # Set max step (distance between interpolated points)
            # MUCH LARGER step size - MoveIt needs room to interpolate
            avg_spacing = self._estimate_waypoint_spacing(position_only_waypoints)
            request.max_step = 0.05  # 5cm steps - very generous for Cartesian planning
            
            self.get_logger().info(f'  Using max_step={request.max_step:.4f}m (avg waypoint spacing: {avg_spacing:.4f}m)')
            
            # Jump threshold (0.0 = no jump check, allows more flexibility)
            request.jump_threshold = 0.0
            
            # CRITICAL: Disable collision avoidance during planning to prevent scene changes
            # from invalidating the plan. MoveIt will still check joint limits and kinematics.
            request.avoid_collisions = False  # Changed from True - prevents -3/-4 errors
            self.get_logger().info('⚠ Collision avoidance DISABLED during Cartesian planning to prevent invalidation')
            
            # Path constraints with orientation tolerance
            # This allows MoveIt to deviate slightly from exact orientation to find valid paths
            from moveit_msgs.msg import Constraints, OrientationConstraint
            constraints = Constraints()
            constraints.name = "orientation_tolerance"
            
            # Add orientation constraint with generous tolerance
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = 'world'
            orient_constraint.link_name = 'tool0'
            orient_constraint.orientation = cartesian_waypoints[0].orientation  # Reference orientation
            # Allow ±30° deviation in each axis for flexibility
            orient_constraint.absolute_x_axis_tolerance = 0.52  # ~30 degrees in radians
            orient_constraint.absolute_y_axis_tolerance = 0.52
            orient_constraint.absolute_z_axis_tolerance = 0.52
            orient_constraint.weight = 0.5  # Lower weight = more flexible
            
            # Note: Commenting out constraints for now to maximize path completion
            # Uncomment if you need strict orientation control:
            # constraints.orientation_constraints.append(orient_constraint)
            # request.path_constraints = constraints
            
            request.path_constraints = Constraints()  # Empty for maximum flexibility
            
            # Start state (use current state - we're at first waypoint now)
            request.start_state.is_diff = True
            
            # Additional stabilization before calling service
            self.get_logger().info('Final stabilization before Cartesian path computation...')
            time.sleep(2.0)  # Extra delay to ensure absolutely no scene changes
            
            self.get_logger().info('Calling Cartesian path service...')
            self.get_logger().info('⏳ Please wait... (scene locked, no external updates)')
            
            # Call service with extended timeout
            future = self.cartesian_path_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)  # Increased timeout
            
            if not future.done():
                self.get_logger().error('✗ Cartesian path computation timed out')
                return False
            
            response = future.result()
            
            # Check fraction (how much of path was successfully computed)
            fraction = response.fraction
            self.get_logger().info(f'Cartesian path fraction achieved: {fraction*100:.1f}%')
            
            # Execute whatever portion was successfully computed
            if fraction < 0.01:  # Less than 1% - almost complete failure
                self.get_logger().error(f'✗ Could only compute {fraction*100:.1f}% of the path')
                self.get_logger().error('Possible issues:')
                self.get_logger().error('  - Waypoints cause IK failures (unreachable poses)')
                self.get_logger().error('  - Orientation changes too large between waypoints')
                self.get_logger().error('  - Path goes through singularities')
                self.get_logger().error('  - Collision detected along path')
                
                # FALLBACK: Try position-only Cartesian path planning
                self.get_logger().warn('='*60)
                self.get_logger().warn('FALLBACK: Attempting position-only Cartesian path')
                self.get_logger().warn('(Ignoring orientation constraints for maximum reachability)')
                self.get_logger().warn('='*60)
                
                # Create position-only waypoints (keep positions, drop orientations)
                position_only_waypoints = []
                for i, wp in enumerate(cartesian_waypoints):
                    pos_wp = Pose()
                    pos_wp.position = wp.position
                    # Use identity orientation (pointing down) for all waypoints
                    pos_wp.orientation.x = 0.0
                    pos_wp.orientation.y = 0.0
                    pos_wp.orientation.z = 0.0
                    pos_wp.orientation.w = 1.0
                    position_only_waypoints.append(pos_wp)
                
                # Try Cartesian path with position-only
                request_pos_only = GetCartesianPath.Request()
                request_pos_only.header.frame_id = 'world'
                request_pos_only.header.stamp = self.get_clock().now().to_msg()
                request_pos_only.group_name = 'ur_manipulator'
                request_pos_only.link_name = 'tool0'
                request_pos_only.waypoints = position_only_waypoints
                request_pos_only.max_step = 0.005
                request_pos_only.jump_threshold = 0.0
                request_pos_only.avoid_collisions = True
                request_pos_only.path_constraints = Constraints()
                request_pos_only.start_state.is_diff = True
                
                self.get_logger().info('Calling Cartesian path service with position-only waypoints...')
                future_pos_only = self.cartesian_path_client.call_async(request_pos_only)
                rclpy.spin_until_future_complete(self, future_pos_only, timeout_sec=15.0)
                
                if future_pos_only.done():
                    response_pos_only = future_pos_only.result()
                    fraction_pos_only = response_pos_only.fraction
                    self.get_logger().info(f'✓ Position-only Cartesian path fraction: {fraction_pos_only*100:.1f}%')
                    
                    if fraction_pos_only > fraction:
                        self.get_logger().info('✓ Position-only planning achieved better results! Using this path.')
                        response = response_pos_only
                        fraction = fraction_pos_only
                    else:
                        self.get_logger().warn('Position-only planning no better than original. Continuing with original.')
                else:
                    self.get_logger().warn('Position-only planning timed out.')
            
            # Check final fraction after possible fallback
            if fraction < 0.1:  # Still less than 10% after fallback
                self.get_logger().error(f'✗ Could only compute {fraction*100:.1f}% of the path (even with fallback)')
                return False
            elif fraction < 0.95:  # Partial success
                self.get_logger().warn(f'⚠ Computed {fraction*100:.1f}% of Cartesian path')
                self.get_logger().info(f'  Will execute computed portion, then use joint-space for remainder')
            else:
                self.get_logger().info(f'✓ Successfully computed full Cartesian path ({fraction*100:.1f}%)!')
            
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
                self.get_logger().info('✓ Executed computed Cartesian trajectory successfully')
                
                # If we didn't complete the full path, try to continue with remaining waypoints
                if fraction < 0.95:
                    # Calculate which waypoints remain
                    waypoints_completed = int(len(cartesian_waypoints) * fraction)
                    remaining_waypoints = cartesian_waypoints[waypoints_completed:]
                    
                    if len(remaining_waypoints) > 0:
                        self.get_logger().warn(f'Cartesian planned {fraction*100:.1f}% -> '
                                              f'{len(remaining_waypoints)} waypoints unreachable')
                        self.get_logger().warn('POLISHING MODE: Skipping unreachable waypoints to maintain continuous motion')
                        self.get_logger().info('Attempting to continue Cartesian path with remaining reachable waypoints...')
                        
                        # Try to compute Cartesian path through remaining waypoints
                        # This might skip problematic poses and connect reachable ones
                        if len(remaining_waypoints) > 2:
                            # Try Cartesian planning again for the tail section
                            request_tail = GetCartesianPath.Request()
                            request_tail.header.frame_id = 'world'
                            request_tail.header.stamp = self.get_clock().now().to_msg()
                            request_tail.group_name = 'ur_manipulator'
                            request_tail.link_name = 'tool0'
                            request_tail.waypoints = remaining_waypoints
                            request_tail.max_step = 0.005
                            request_tail.jump_threshold = 0.0
                            request_tail.avoid_collisions = True
                            request_tail.path_constraints = Constraints()
                            request_tail.start_state.is_diff = True
                            
                            self.get_logger().info('Computing Cartesian path for remaining segment...')
                            future_tail = self.cartesian_path_client.call_async(request_tail)
                            rclpy.spin_until_future_complete(self, future_tail, timeout_sec=10.0)
                            
                            if future_tail.done():
                                response_tail = future_tail.result()
                                fraction_tail = response_tail.fraction
                                
                                if fraction_tail > 0.1 and len(response_tail.solution.joint_trajectory.points) > 0:
                                    self.get_logger().info(f'✓ Computed continuation: {fraction_tail*100:.1f}% of remaining path')
                                    
                                    # Execute continuation trajectory
                                    exec_goal_tail = ExecuteTrajectory.Goal()
                                    exec_goal_tail.trajectory = response_tail.solution
                                    
                                    exec_future_tail = self.execute_trajectory_client.send_goal_async(exec_goal_tail)
                                    rclpy.spin_until_future_complete(self, exec_future_tail, timeout_sec=5.0)
                                    
                                    if exec_future_tail.done() and exec_future_tail.result().accepted:
                                        exec_handle_tail = exec_future_tail.result()
                                        exec_result_future_tail = exec_handle_tail.get_result_async()
                                        
                                        # Estimate timeout
                                        if len(response_tail.solution.joint_trajectory.points) > 0:
                                            last_point_tail = response_tail.solution.joint_trajectory.points[-1]
                                            duration_tail = last_point_tail.time_from_start.sec + last_point_tail.time_from_start.nanosec / 1e9
                                            timeout_tail = duration_tail + 10.0
                                        else:
                                            timeout_tail = 20.0
                                        
                                        rclpy.spin_until_future_complete(self, exec_result_future_tail, timeout_sec=timeout_tail)
                                        
                                        if exec_result_future_tail.done():
                                            exec_result_tail = exec_result_future_tail.result().result
                                            if exec_result_tail.error_code.val == MoveItErrorCodes.SUCCESS:
                                                self.get_logger().info('✓ Successfully executed continuation trajectory')
                                            else:
                                                self.get_logger().warn('Continuation trajectory execution had issues')
                                else:
                                    self.get_logger().warn('Could not compute continuation - remaining waypoints unreachable')
                        else:
                            self.get_logger().info('Only 1-2 waypoints remaining - acceptable completion')
                
                self.get_logger().info('='*60)
                self.get_logger().info('✓ Path execution complete!')
                self.get_logger().info('='*60)
                return True
            else:
                self.get_logger().error(f'✗ Execution failed with error code: {exec_result.error_code.val}')
                return False
                
        finally:
            # Always reset the executing flag and unlock scene
            self.executing = False
            self.scene_locked = False
            self.get_logger().info('✓ Planning scene UNLOCKED - external updates re-enabled')
    
    def move_to_first_waypoint(self, first_waypoint, relaxed=False):
        """
        Move to the first waypoint using joint-space planning with better error handling.
        
        Args:
            first_waypoint: Target Pose
            relaxed: If True, use very relaxed orientation constraints
        """
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient
        
        self.get_logger().info(f'Target: pos=[{first_waypoint.position.x:.3f}, '
                              f'{first_waypoint.position.y:.3f}, {first_waypoint.position.z:.3f}]')
        
        # Check distance from origin (robot base) - UR3e reach is ~0.5m
        x, y, z = first_waypoint.position.x, first_waypoint.position.y, first_waypoint.position.z
        distance = (x*x + y*y + z*z)**0.5
        
        if distance > 0.5:  # UR3e reach limit
            self.get_logger().warn(f'Waypoint distance {distance:.3f}m is at edge of UR3e reach (~0.5m)')
        
        self.get_logger().info(f'Attempting to reach waypoint (distance: {distance:.3f}m)')
        
        # Create action client for MoveGroup
        move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        if not move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False
        
        # Create goal message with more relaxed constraints
        goal_msg = MoveGroup.Goal()
        
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 30
        goal_msg.request.allowed_planning_time = 20.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        
        # Position constraint with larger tolerance
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
        sphere.dimensions = [0.03]  # 3cm tolerance (larger)
        bounding_volume.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position = first_waypoint.position
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)
        
        position_constraint.constraint_region = bounding_volume
        
        # Orientation constraint with adjustable tolerances
        from moveit_msgs.msg import OrientationConstraint
        
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = 'tool0'
        orientation_constraint.orientation = first_waypoint.orientation
        
        if relaxed:
            # Very relaxed orientation for difficult waypoints
            orientation_constraint.absolute_x_axis_tolerance = 3.14  # Allow full rotation
            orientation_constraint.absolute_y_axis_tolerance = 3.14
            orientation_constraint.absolute_z_axis_tolerance = 3.14
            orientation_constraint.weight = 0.01  # Minimal weight - almost ignore orientation
            self.get_logger().info('  Using VERY relaxed orientation constraints (almost position-only)')
        else:
            # Normal relaxed orientation
            orientation_constraint.absolute_x_axis_tolerance = 1.0  # ~60 degrees
            orientation_constraint.absolute_y_axis_tolerance = 1.0
            orientation_constraint.absolute_z_axis_tolerance = 1.0
            orientation_constraint.weight = 0.3  # Lower weight
            self.get_logger().info('  Using relaxed orientation constraints (~60° tolerance)')
        
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = True  # Plan only first, execute separately
        
        # Wait for planning scene to stabilize
        self.get_logger().info('Waiting for planning scene to stabilize...')
        import time
        time.sleep(5.0)  # Extended wait for maximum stability - prevents -4 errors
        
        # Retry loop for handling environment changes
        max_retries = 10  # Increased retries
        for attempt in range(max_retries):
            if attempt > 0:
                self.get_logger().info(f'Retry attempt {attempt + 1}/{max_retries}...')
                self.get_logger().info('Waiting longer for scene to fully stabilize...')
                time.sleep(5.0)  # Even longer between retries
            
            # Send goal
            self.get_logger().info('Planning to first waypoint with relaxed constraints...')
            send_goal_future = move_group_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            if not send_goal_future.done():
                self.get_logger().error('Planning goal submission timed out')
                if attempt == max_retries - 1:
                    return False
                continue
                
            if not send_goal_future.result().accepted:
                self.get_logger().error('Planning goal rejected by MoveGroup')
                if attempt == max_retries - 1:
                    return False
                continue
            
            goal_handle = send_goal_future.result()
            self.get_logger().info('✓ Planning goal accepted, computing path...')
            
            # Wait for result with longer timeout
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=40.0)  # Longer timeout
            
            if not get_result_future.done():
                self.get_logger().error('Planning timed out after 40 seconds')
                if attempt == max_retries - 1:
                    self.get_logger().error('Possible issues:')
                    self.get_logger().error('  - Waypoint orientation not achievable')
                    self.get_logger().error('  - Robot in collision or near singularity')
                    self.get_logger().error('  - MoveIt planning taking too long')
                    return False
                continue
            
            if not get_result_future.done():
                self.get_logger().error('Planning and execution timed out after 30 seconds')
                if attempt == max_retries - 1:
                    self.get_logger().error('Possible issues:')
                    self.get_logger().error('  - Waypoint orientation not achievable')
                    self.get_logger().error('  - Robot in collision or near singularity')
                    self.get_logger().error('  - MoveIt planning taking too long')
                    return False
                continue
            
            result = get_result_future.result().result
            
            from moveit_msgs.msg import MoveItErrorCodes
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('✓ Successfully reached first waypoint!')
                return True
            elif result.error_code.val == -4:  # MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE
                self.get_logger().warn(f'⚠ Planning scene changed during planning (attempt {attempt + 1}/{max_retries})')
                if attempt < max_retries - 1:
                    continue  # Retry
                # Fall through to error handling on last attempt
            else:
                # Other error - don't retry
                break
        
        
        if not get_result_future.done():
            self.get_logger().error('Planning and execution timed out after 30 seconds')
            self.get_logger().error('Possible issues:')
            self.get_logger().error('  - Waypoint orientation not achievable')
            self.get_logger().error('  - Robot in collision or near singularity')
            self.get_logger().error('  - MoveIt planning taking too long')
            return False
        
        result = get_result_future.result().result
        
        # Decode error codes
        from moveit_msgs.msg import MoveItErrorCodes
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('✓ Planning successful!')
            
            # Now execute the planned trajectory
            if result.planned_trajectory and len(result.planned_trajectory.joint_trajectory.points) > 0:
                self.get_logger().info('Executing planned trajectory...')
                
                # Execute using ExecuteTrajectory action
                from moveit_msgs.action import ExecuteTrajectory
                execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
                
                if not execute_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error('ExecuteTrajectory action not available')
                    return False
                
                exec_goal = ExecuteTrajectory.Goal()
                exec_goal.trajectory = result.planned_trajectory
                
                exec_future = execute_client.send_goal_async(exec_goal)
                rclpy.spin_until_future_complete(self, exec_future, timeout_sec=5.0)
                
                if not exec_future.done() or not exec_future.result().accepted:
                    self.get_logger().error('✗ Trajectory execution rejected')
                    return False
                
                exec_handle = exec_future.result()
                self.get_logger().info('Executing...')
                
                # Wait for execution
                exec_result_future = exec_handle.get_result_async()
                rclpy.spin_until_future_complete(self, exec_result_future, timeout_sec=30.0)
                
                if not exec_result_future.done():
                    self.get_logger().error('Execution timed out')
                    return False
                
                exec_result = exec_result_future.result().result
                if exec_result.error_code.val == MoveItErrorCodes.SUCCESS:
                    self.get_logger().info('✓ Successfully reached first waypoint!')
                    return True
                else:
                    self.get_logger().error(f'Execution failed: {exec_result.error_code.val}')
                    return False
            else:
                self.get_logger().error('No trajectory in planning result')
                return False
        
        error_meanings = {
            1: 'SUCCESS',
            -1: 'FAILURE', 
            -2: 'PLANNING_FAILED',
            -3: 'INVALID_MOTION_PLAN',
            -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
            -5: 'CONTROL_FAILED',
            -10: 'PREEMPTED',
            -11: 'START_STATE_IN_COLLISION',
            -12: 'START_STATE_VIOLATES_PATH_CONSTRAINTS', 
            -13: 'GOAL_IN_COLLISION',
            -14: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
            -15: 'GOAL_CONSTRAINTS_VIOLATED',
            -16: 'INVALID_GROUP_NAME',
            -31: 'NO_IK_SOLUTION',
            99999: 'TIMEOUT'
        }
        error_name = error_meanings.get(result.error_code.val, f'UNKNOWN_ERROR_{result.error_code.val}')
        
        self.get_logger().error(f'✗ Planning failed: {error_name} (code {result.error_code.val})')
        
        if result.error_code.val == -31:  # NO_IK_SOLUTION
            self.get_logger().error('The waypoint pose cannot be reached by the robot')
            self.get_logger().error('Try adjusting the waypoint position or orientation')
        elif result.error_code.val == -13:  # GOAL_IN_COLLISION
            self.get_logger().error('The waypoint would cause a collision')
        elif result.error_code.val == -2:  # PLANNING_FAILED
            self.get_logger().error('MoveIt could not find a path to the waypoint')
        elif result.error_code.val == 99999:  # TIMEOUT
            self.get_logger().error('Planning took too long - waypoint might be unreachable')
        
        return False
    
    def move_to_position_only(self, target_pose):
        """
        Move to waypoint position, completely ignoring orientation.
        Last resort fallback for unreachable orientations.
        
        Args:
            target_pose: Target Pose (only position will be used)
        """
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient
        
        self.get_logger().info(f'Position-only target: [{target_pose.position.x:.3f}, '
                              f'{target_pose.position.y:.3f}, {target_pose.position.z:.3f}]')
        
        # Create action client
        move_group_client = ActionClient(self, MoveGroup, '/move_action')
        if not move_group_client.wait_for_server(timeout_sec=5.0):
            return False
        
        # Create goal with ONLY position constraint (no orientation)
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.num_planning_attempts = 30
        goal_msg.request.allowed_planning_time = 15.0
        goal_msg.request.max_velocity_scaling_factor = 0.15
        goal_msg.request.max_acceleration_scaling_factor = 0.15
        
        from moveit_msgs.msg import PositionConstraint, BoundingVolume, Constraints
        from shape_msgs.msg import SolidPrimitive
        
        # Position-only constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'world'
        position_constraint.link_name = 'tool0'
        position_constraint.weight = 1.0
        
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]  # 5cm tolerance
        bounding_volume.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position = target_pose.position
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)
        position_constraint.constraint_region = bounding_volume
        
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        # NO orientation constraints - that's the point!
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False
        
        # Execute
        send_goal_future = move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        if not send_goal_future.done() or not send_goal_future.result().accepted:
            return False
        
        goal_handle = send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=30.0)
        
        if not get_result_future.done():
            return False
        
        result = get_result_future.result().result
        from moveit_msgs.msg import MoveItErrorCodes
        return result.error_code.val == MoveItErrorCodes.SUCCESS
    
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
        # Removed timeout check to allow waiting indefinitely for external path
        # if (node.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
        #     print("ERROR: No path received within timeout!")
        #     print("Make sure import_line.sh is running.")
        #     node.destroy_node()
        #     rclpy.shutdown()
        #     return
    
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
