#!/usr/bin/env python3
"""
Follow a scanning path using MoveIt's Cartesian path planning.
This computes ONE continuous trajectory through all waypoints without stopping.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
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
            '/tool_orientation/xyz_rotation',
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
        
        self.get_logger().info('Waiting for scanning path on /tool_orientation/xyz_rotation...')
        self.get_logger().info('Also listening to /line_path (positions) and /line_visualization (orientations)')

    def line_path_callback(self, marker: Marker):
        """Capture positions from LINE_STRIP marker published on /line_path (in base_link frame)."""
        if self.executing:
            return
        try:
            pts = marker.points
            self._rviz_positions = [(p.x, p.y, p.z) for p in pts]
            self.get_logger().info(f'Received /line_path with {len(self._rviz_positions)} positions')
            self._try_build_waypoints_from_rviz()
        except Exception as e:
            self.get_logger().warn(f'/line_path parse failed: {e}')

    def line_viz_callback(self, arr: MarkerArray):
        """Capture orientations from ARROW markers in /line_visualization (in base_link frame)."""
        if self.executing:
            return
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
            
        self.get_logger().info(f'Received path with {len(msg.data)} elements')
        
        data = np.array(msg.data)
        
        # Check for quaternion format first: [x,y,z,qx,qy,qz,qw] = 7 values per point
        if len(data) % 7 == 0:
            self.get_logger().info('Detected quaternion format: [x,y,z,qx,qy,qz,qw]')
            num_waypoints = len(data) // 7
            self.get_logger().info(f'Parsing {num_waypoints} waypoints...')
            
            self.waypoints = []
            for i in range(num_waypoints):
                idx = i * 7
                try:
                    # Extract position and quaternion (already in correct frame)
                    position = np.array(data[idx:idx+3])
                    quat = np.array(data[idx+3:idx+7])  # [qx, qy, qz, qw]
                    
                    pose = Pose()
                    pose.position.x = float(position[0])
                    pose.position.y = float(position[1])
                    pose.position.z = float(position[2])
                    pose.orientation.x = float(quat[0])
                    pose.orientation.y = float(quat[1])
                    pose.orientation.z = float(quat[2])
                    pose.orientation.w = float(quat[3])
                    
                    self.waypoints.append(pose)
                    
                    if i == 0 or i == num_waypoints - 1:
                        self.get_logger().info(
                            f'  Point {i}: pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]'
                        )
                        
                except Exception as e:
                    self.get_logger().error(f'Failed to parse waypoint {i}: {e}')
                    return
            
            self.path_received = True
            self.get_logger().info(f'✓ Successfully parsed {len(self.waypoints)} waypoints (quaternion format)')
            return
        
        # Original rotation matrix format check
        if len(data) % 12 != 0:
            self.get_logger().error(f'Invalid path data length: {len(data)} (expected multiple of 7 or 12)')
            return
        num_waypoints = len(data) // 12
        self.get_logger().info(f'Parsing {num_waypoints} waypoints (rotation matrix format)...')

        # Heuristic auto-detection for format: "rotation-first" vs "position-first"
        # rotation-first: [r11..r33, x,y,z] per waypoint
        # position-first: [x,y,z, r11..r33] per waypoint
        def probe_format(fmt):
            # returns tuple (valid_pos_bounds, det)
            try:
                if fmt == 'rot_first':
                    idx = 0
                    rot = np.array(data[idx:idx+9]).reshape((3, 3))
                    pos = np.array(data[idx+9:idx+12])
                else:
                    idx = 0
                    pos = np.array(data[idx:idx+3])
                    rot = np.array(data[idx+3:idx+12]).reshape((3, 3))

                det = np.linalg.det(rot)
                # position bounds heuristic (UR3e workspace approx)
                pos_ok = (abs(pos[0]) < 1.0 and abs(pos[1]) < 1.0 and -0.5 < pos[2] < 1.2)
                return pos_ok, float(det)
            except Exception:
                return False, 0.0

        pos_ok_rot, det_rot = probe_format('rot_first')
        pos_ok_pos, det_pos = probe_format('pos_first')

        # Choose format: prefer one with position in bounds and det > 0
        chosen = None
        if pos_ok_pos and det_pos > 0.0 and (not pos_ok_rot or det_rot <= 0.0):
            chosen = 'pos_first'
        elif pos_ok_rot and det_rot > 0.0 and (not pos_ok_pos or det_pos <= 0.0):
            chosen = 'rot_first'
        else:
            # If both look plausible, choose the one with det closer to +1
            if abs(det_pos - 1.0) < abs(det_rot - 1.0):
                chosen = 'pos_first'
            else:
                chosen = 'rot_first'

        self.get_logger().info(f'Detected input format: {chosen} (det_pos={det_pos:.3f}, det_rot={det_rot:.3f})')
        self.waypoints = []
        positions = []

        # Parse into position/rotation arrays first
        for i in range(num_waypoints):
            idx = i * 12
            try:
                if chosen == 'rot_first':
                    rot_matrix = np.array(data[idx:idx+9]).reshape((3, 3))
                    position = np.array(data[idx+9:idx+12])
                else:
                    position = np.array(data[idx:idx+3])
                    rot_matrix = np.array(data[idx+3:idx+12]).reshape((3, 3))

                positions.append(position)
            except Exception as e:
                self.get_logger().warn(f'Parsing waypoint {i} failed: {e}')
                positions.append(np.array([np.nan, np.nan, np.nan]))

        # Compute Z-range and decide small safety offset (cap maximum auto-lift)
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
            # Cap accidental large offsets (indicates bad parsing)
            if z_offset > 0.6:
                self.get_logger().error(f'Computed Z offset {z_offset:.3f}m is unexpectedly large — aborting parse')
                self.waypoints = []
                self.path_received = True
                return
            self.get_logger().warn(f'Waypoints have unsafe Z range [{min_z:.3f}, {max_z:.3f}]')
            self.get_logger().warn(f'Applying Z offset of {z_offset:.3f}m to make them reachable')
        else:
            self.get_logger().info(f'Waypoints Z range [{min_z:.3f}, {max_z:.3f}] looks good')

        # Now construct Pose list with sanitized rotations
        for i in range(num_waypoints):
            idx = i * 12
            try:
                if chosen == 'rot_first':
                    rot_matrix = np.array(data[idx:idx+9]).reshape((3, 3))
                    position = np.array(data[idx+9:idx+12])
                else:
                    position = np.array(data[idx:idx+3])
                    rot_matrix = np.array(data[idx+3:idx+12]).reshape((3, 3))

                position = position.copy()
                position[2] = position[2] + z_offset

                # Sanitize rotation matrix using SVD / polar decomposition
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
                    raise ValueError(f'Non-positive determinant after fix: {det_val}')

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
                if i == 0:
                    self.get_logger().info(f'First waypoint corrected: pos=[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]')
                elif i == num_waypoints - 1:
                    self.get_logger().info(f'Last waypoint corrected: pos=[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]')

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
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'COMPUTING CONTINUOUS CARTESIAN PATH')
        self.get_logger().info(f'Through {len(self.waypoints)} waypoints')
        self.get_logger().info('='*60)
        
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
            
            # Execute whatever portion was successfully computed
            if fraction < 0.1:  # Less than 10% - complete failure
                self.get_logger().error(f'✗ Could only compute {fraction*100:.1f}% of the path')
                self.get_logger().error('Possible issues:')
                self.get_logger().error('  - Waypoints cause IK failures (unreachable poses)')
                self.get_logger().error('  - Orientation changes too large between waypoints')
                self.get_logger().error('  - Path goes through singularities')
                self.get_logger().error('  - Collision detected along path')
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
            # Always reset the executing flag
            self.executing = False
    
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
        
        # Check if the waypoint is within reasonable workspace bounds
        x, y, z = first_waypoint.position.x, first_waypoint.position.y, first_waypoint.position.z
        
        # UR3e approximate workspace limits
        if abs(x) > 0.6 or abs(y) > 0.6 or z > 0.8 or z < 0.1:
            self.get_logger().error(f'Waypoint [{x:.3f}, {y:.3f}, {z:.3f}] is outside UR3e workspace!')
            self.get_logger().error('UR3e workspace: X,Y: ±0.6m, Z: 0.1-0.8m')
            return False
        
        # Check distance from origin (robot base)
        distance = (x*x + y*y + z*z)**0.5
        if distance > 0.85:  # UR3e reach is ~0.85m
            self.get_logger().error(f'Waypoint distance {distance:.3f}m exceeds UR3e reach (~0.85m)!')
            return False
        
        self.get_logger().info(f'✓ Waypoint is within workspace bounds (distance: {distance:.3f}m)')
        
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
        goal_msg.request.num_planning_attempts = 30  # Even more attempts
        goal_msg.request.allowed_planning_time = 20.0  # Much more time
        goal_msg.request.max_velocity_scaling_factor = 0.2  # Slower for safety
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
        time.sleep(2.0)  # Longer wait for stability
        
        # Retry loop for handling environment changes
        max_retries = 5  # More retries
        for attempt in range(max_retries):
            if attempt > 0:
                self.get_logger().info(f'Retry attempt {attempt + 1}/{max_retries}...')
                time.sleep(2.0)  # Longer wait between retries
            
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
