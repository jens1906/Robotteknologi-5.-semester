#!/usr/bin/env python3

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import Float64MultiArray, ByteMultiArray
    from geometry_msgs.msg import PoseArray, Pose
    from scipy.spatial.transform import Rotation as R
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 packages not available. Running in standalone mode.")

def normalize(v, epsilon=1e-12):
    #Normalize vector to unit length
    norm = np.linalg.norm(v)
    if norm < epsilon:
        return np.zeros_like(v)
    return v / norm

def compute_velocity(path_xyz, dt=0.1):
    #Compute velocity at each point using numerical differentiation
    #Args: path_xyz (N, 3) array, dt time step
    #Returns: velocities (N, 3) array
    n_points = len(path_xyz)
    velocities = np.zeros((n_points, 3))
    
    #Handle edge case of single point
    if n_points == 1:
        return velocities
    
    for i in range(n_points):
        if i == 0:
            #Forward difference for first point
            velocities[i] = (path_xyz[i+1] - path_xyz[i]) / dt
        elif i == n_points - 1:
            #Backward difference for last point
            velocities[i] = (path_xyz[i] - path_xyz[i-1]) / dt
        else:
            #Central difference for middle points
            velocities[i] = (path_xyz[i+1] - path_xyz[i-1]) / (2 * dt)
    
    return velocities

def compute_normal_from_neighbors(path_xyz, index, neighbor_range=3):
    #Estimate surface normal at a point using neighboring points
    #Fits a local plane and returns the normal to that plane
    #Args: path_xyz (N,3) array, index current point, neighbor_range neighbors on each side
    #Returns: normal (3,) unit vector
    n_points = len(path_xyz)
    
    #Get neighbor indices
    start = max(0, index - neighbor_range)
    end = min(n_points, index + neighbor_range + 1)
    
    #Get local points
    local_points = path_xyz[start:end]
    
    if len(local_points) < 3:
        #Not enough neighbors, estimate from velocity direction
        #Normal is perpendicular to path tangent
        if index > 0 and index < n_points - 1:
            tangent = path_xyz[index + 1] - path_xyz[index - 1]
        elif index == 0 and n_points > 1:
            tangent = path_xyz[1] - path_xyz[0]
        elif index == n_points - 1 and n_points > 1:
            tangent = path_xyz[index] - path_xyz[index - 1]
        else:
            return np.array([0, 0, 1])  #Fallback for single point
        
        #Create perpendicular vector in XY plane if possible
        tangent_norm = normalize(tangent)
        if abs(tangent_norm[2]) < 0.99:  #Not vertical
            normal = np.array([0, 0, 1])
        else:  #Nearly vertical tangent
            normal = np.array([1, 0, 0])
        return normalize(normal)
    
    #Center the points
    centroid = np.mean(local_points, axis=0)
    centered = local_points - centroid
    
    #Check if points are too close together (degenerate case)
    max_distance = np.max(np.linalg.norm(centered, axis=1))
    if max_distance < 1e-10:
        #Points are essentially identical, estimate from path direction
        if index > 0:
            tangent = normalize(path_xyz[index] - path_xyz[index - 1])
        elif index < n_points - 1:
            tangent = normalize(path_xyz[index + 1] - path_xyz[index])
        else:
            tangent = np.array([1, 0, 0])
        
        #Normal perpendicular to tangent, prefer upward direction
        if abs(tangent[2]) < 0.99:
            normal = np.array([0, 0, 1])
        else:
            normal = np.array([1, 0, 0])
        return normalize(normal)
    
    #Use SVD to find the normal (smallest singular value direction)
    try:
        _, s, vh = np.linalg.svd(centered, full_matrices=False)
        
        #Check if the data is degenerate (collinear points)
        if len(s) < 3 or s[-1] < 1e-10 * s[0]:
            #Points are collinear - normal should be perpendicular to the line
            #Use the dominant direction (first singular vector) as tangent
            tangent = vh[0, :]
            
            #Create a normal perpendicular to tangent
            #Try Z-up first, if tangent is too vertical, use X
            if abs(tangent[2]) < 0.99:
                normal = np.array([0, 0, 1]) - tangent[2] * tangent
            else:
                normal = np.array([1, 0, 0]) - tangent[0] * tangent
            
            normal = normalize(normal)
            
            #Ensure upward preference
            if normal[2] < 0:
                normal = -normal
            
            return normal
        
        normal = vh[-1, :]  #Last row is normal direction
        
    except np.linalg.LinAlgError:
        #SVD failed to converge, estimate from local geometry
        if index > 0 and index < n_points - 1:
            #Use two vectors to compute cross product
            v1 = path_xyz[index] - path_xyz[index - 1]
            v2 = path_xyz[index + 1] - path_xyz[index]
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm > 1e-10:
                normal = normal / norm
            else:
                normal = np.array([0, 0, 1])
        else:
            normal = np.array([0, 0, 1])
    
    #Ensure normal points "upward" (positive z component)
    if normal[2] < 0:
        normal = -normal
    
    return normalize(normal)


def orientation_matrix_from_path(velocity, normal):
    #Compute orientation matrix at a point
    #Args: velocity (3,) vector, normal (3,) vector
    #Returns: R (3, 3) rotation matrix
    
    #Tool z-axis points into surface (opposite of normal)
    ez = normalize(-normal)
    
    #Tool feed direction (tangent to path)
    vel_norm = np.linalg.norm(velocity)
    if vel_norm < 1e-10:
        #Zero velocity (stationary point or duplicate)
        #Use a default feed direction perpendicular to normal
        if abs(ez[2]) < 0.99:  #Normal not vertical
            e_gamma = np.array([1, 0, 0])  #Default to X direction
        else:  #Normal is vertical
            e_gamma = np.array([0, 1, 0])  #Default to Y direction
        e_gamma = normalize(e_gamma - np.dot(e_gamma, ez) * ez)  #Make perpendicular to ez
    else:
        e_gamma = velocity / vel_norm
    
    #Tool y-axis perpendicular to tool axis and feed direction
    ey_cross = np.cross(-ez, e_gamma)
    ey_norm = np.linalg.norm(ey_cross)
    
    if ey_norm < 1e-10:
        #e_gamma and ez are parallel/antiparallel
        #Choose arbitrary perpendicular direction
        if abs(ez[0]) < 0.99:
            ey = normalize(np.cross(ez, np.array([1, 0, 0])))
        else:
            ey = normalize(np.cross(ez, np.array([0, 1, 0])))
    else:
        ey = ey_cross / ey_norm
    
    #Tool x-axis completes right-handed frame
    ex = np.cross(ey, ez)
    ex = normalize(ex)  #Ensure unit length
    
    #Construct rotation matrix (columns are basis vectors)
    R = np.column_stack([ex, ey, ez])
    
    return R

def apply_off_surface_offset(positions, orientations, on_surface, offset_distance=0.05):
    #Apply offset to positions where robot should be off surface
    #Args: positions (N,3) array, orientations (N,3,3) array of rotation matrices
    #      on_surface (N,) boolean array, offset_distance in meters (default 5 cm)
    #Returns: adjusted_positions (N,3) array with offsets applied
    
    adjusted_positions = positions.copy()
    
    for i in range(len(positions)):
        if not on_surface[i]:
            #Get the surface normal from the orientation matrix
            #The tool z-axis (3rd column) points INTO the surface (negative normal)
            #So the surface normal (outward) is the negative of the z-axis
            tool_z_axis = orientations[i][:, 2]  # Third column
            surface_normal = -tool_z_axis  # Flip to get outward normal
            
            #Move position along surface normal by offset_distance
            adjusted_positions[i] = positions[i] + surface_normal * offset_distance
    
    return adjusted_positions

def compute_orientations_from_xyz(path_xyz, dt=0.1, neighbor_range=3, smooth_orientations=True):
    #Main function: compute tool orientations from cartesian XYZ points
    #Args: path_xyz (N,3) array of [x,y,z] in sequence, dt time step, neighbor_range for normal estimation
    #      smooth_orientations if True, check for discontinuous jumps and smooth them
    #Returns: positions (N,3) array, orientations (N,3,3) array of rotation matrices
    n_points = len(path_xyz)
    
    #Step 1: Compute velocities from consecutive points
    velocities = compute_velocity(path_xyz, dt)
    
    #Step 2: Initialize output arrays
    orientations = np.zeros((n_points, 3, 3))
    
    #Step 3: Process each point
    for i in range(n_points):
        velocity = velocities[i]
        
        #Estimate surface normal from neighboring points
        normal = compute_normal_from_neighbors(path_xyz, i, neighbor_range)
        
        #Compute orientation matrix
        R = orientation_matrix_from_path(velocity, normal)
        
        #Check for discontinuous orientation change
        if smooth_orientations and i > 0:
            #Compute angle difference between consecutive orientations
            R_prev = orientations[i-1]
            R_diff = R_prev.T @ R
            trace_diff = np.trace(R_diff)
            angle_diff = np.arccos(np.clip((trace_diff - 1) / 2, -1, 1))
            
            #If orientation changes by more than 90 degrees, likely a sign flip
            if angle_diff > np.pi / 2:
                #Check if flipping the normal reduces the discontinuity
                R_flipped = orientation_matrix_from_path(velocity, -normal)
                R_diff_flipped = R_prev.T @ R_flipped
                trace_flipped = np.trace(R_diff_flipped)
                angle_flipped = np.arccos(np.clip((trace_flipped - 1) / 2, -1, 1))
                
                if angle_flipped < angle_diff:
                    R = R_flipped
        
        orientations[i] = R
    
    return path_xyz, orientations


if ROS2_AVAILABLE:
    class ToolOrientationNode(Node):
        def __init__(self):
            super().__init__('tool_orientation_node')
            
            #Publisher for trajectory with quaternions (unified format)
            self.trajectory_pub = self.create_publisher(
                PoseArray,
                '/tool_orientation/path',
                10
            )
            
            #Subscriber for input path points
            self.path_sub = self.create_subscription(
                Float64MultiArray,
                '/parameterization/xyz_path',
                self.path_callback,
                10
            )
            
            #Subscriber for on_surface boolean array
            self.on_surface_sub = self.create_subscription(
                ByteMultiArray,
                '/path/on_surface',
                self.on_surface_callback,
                10
            )
            
            #Parameters
            self.declare_parameter('dt', 0.1)
            self.declare_parameter('neighbor_range', 3)
            self.declare_parameter('off_surface_height', 0.05)  # 5 cm in meters
            self.declare_parameter('frame_id', 'world')  # Coordinate frame for visualization
            
            #Storage for received data
            self.path_xyz = None
            self.on_surface = None
            self.data_processed = False  # Flag to avoid reprocessing same data
            
            self.get_logger().info('Tool Orientation Node initialized')
            self.get_logger().info('Publishing to: /tool_orientation/path (PoseArray with quaternions)')
        
        def on_surface_callback(self, msg):
            #Callback for receiving on_surface boolean array
            self.on_surface = np.array(msg.data, dtype=bool)
            self.data_processed = False
            self.try_process_data()
        
        def path_callback(self, msg):
            #Callback for receiving path points and computing orientations
            #Expected input: Float64MultiArray with [x1, y1, z1, x2, y2, z2, ...]
            n_points = len(msg.data) // 3
            self.path_xyz = np.array(msg.data).reshape(n_points, 3)
            self.data_processed = False
            self.get_logger().info(f'Received {n_points} path points')
            self.try_process_data()
        
        def try_process_data(self):
            #Process data only when both path and on_surface are available
            if self.path_xyz is None or self.on_surface is None or self.data_processed:
                return
            
            #Check that arrays are aligned
            if len(self.path_xyz) != len(self.on_surface):
                self.get_logger().error(
                    f'Array size mismatch: path has {len(self.path_xyz)} points, '
                    f'on_surface has {len(self.on_surface)} points'
                )
                return
            
            #Extract parameters
            dt = self.get_parameter('dt').value
            neighbor_range = self.get_parameter('neighbor_range').value
            off_surface_height = self.get_parameter('off_surface_height').value
            
            #Compute and publish
            self.compute_and_publish_orientations(
                self.path_xyz, 
                self.on_surface, 
                dt, 
                neighbor_range, 
                off_surface_height
            )
            
            #Mark data as processed
            self.data_processed = True
        
        def compute_and_publish_orientations(self, path_xyz, on_surface, dt=0.1, neighbor_range=3, off_surface_height=0.05):
            #Compute orientations and publish positions + rotation matrices

            #INPUT: path_xyz is in MILLIMETERS (from camera/corrosion detection)
            #OUTPUT: positions should be in METERS (for robot)
            
            self.get_logger().info(f'Input path_xyz range: X=[{np.min(path_xyz[:,0]):.1f}, {np.max(path_xyz[:,0]):.1f}] mm')
            self.get_logger().info(f'                      Y=[{np.min(path_xyz[:,1]):.1f}, {np.max(path_xyz[:,1]):.1f}] mm')
            self.get_logger().info(f'                      Z=[{np.min(path_xyz[:,2]):.1f}, {np.max(path_xyz[:,2]):.1f}] mm')
            
            #First compute base orientations for all points (input in mm)
            positions_mm, orientations = compute_orientations_from_xyz(path_xyz, dt, neighbor_range)
            
            # Convert off_surface_height from meters to millimeters for consistent units
            off_surface_height_mm = off_surface_height * 1000.0  # m to mm
            
            #Apply off-surface offsets where needed (in mm)
            adjusted_positions_mm = apply_off_surface_offset(positions_mm, orientations, on_surface, off_surface_height_mm)
            
            # CONVERT FROM MILLIMETERS TO METERS for robot output
            adjusted_positions = adjusted_positions_mm / 1000.0  # mm to m
            
            #Count how many points are off-surface
            n_off_surface = np.sum(~on_surface)
            self.get_logger().info(
                f'Applied off-surface offset to {n_off_surface}/{len(positions_mm)} points '
                f'(offset: {off_surface_height*1000:.1f} mm = {off_surface_height*100:.1f} cm)'
            )
            
            self.get_logger().info(f'Output positions range: X=[{np.min(adjusted_positions[:,0]):.4f}, {np.max(adjusted_positions[:,0]):.4f}] m')
            self.get_logger().info(f'                        Y=[{np.min(adjusted_positions[:,1]):.4f}, {np.max(adjusted_positions[:,1]):.4f}] m')
            self.get_logger().info(f'                        Z=[{np.min(adjusted_positions[:,2]):.4f}, {np.max(adjusted_positions[:,2]):.4f}] m')
            
            #Create PoseArray message with positions + quaternions
            trajectory_msg = PoseArray()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.header.frame_id = self.get_parameter('frame_id').value
            
            for i in range(len(adjusted_positions)):
                pose = Pose()
                #Position in meters
                pose.position.x = float(adjusted_positions[i][0])
                pose.position.y = float(adjusted_positions[i][1])
                pose.position.z = float(adjusted_positions[i][2])
                
                #Convert rotation matrix to quaternion
                try:
                    rot = R.from_matrix(orientations[i])
                    quat = rot.as_quat()  # Returns [x, y, z, w]
                    pose.orientation.x = float(quat[0])
                    pose.orientation.y = float(quat[1])
                    pose.orientation.z = float(quat[2])
                    pose.orientation.w = float(quat[3])
                except Exception as e:
                    self.get_logger().warn(f'Failed to convert orientation at point {i}: {e}')
                    #Default to identity quaternion if conversion fails
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = 0.0
                    pose.orientation.w = 1.0
                
                trajectory_msg.poses.append(pose)
            
            self.trajectory_pub.publish(trajectory_msg)
            self.get_logger().info(f'Published {len(adjusted_positions)} waypoints with quaternions')
            
            return adjusted_positions, orientations

    class ToolOrientationDebuggerNode(Node):
        def __init__(self):
            super().__init__('tool_orientation_debugger')

            # Parameters mirror generator node for consistency
            self.declare_parameter('dt', 0.1)
            self.declare_parameter('neighbor_range', 3)
            self.declare_parameter('off_surface_height', 0.05)
            self.declare_parameter('frame_id', 'world')
            self.declare_parameter('axis_tolerance_deg', 5.0)
            self.declare_parameter('offset_tolerance_m', 0.001)
            self.declare_parameter('transverse_tolerance_m', 0.001)

            # Subscribers
            self.path_sub = self.create_subscription(
                Float64MultiArray,
                '/parameterization/xyz_path',
                self.path_callback,
                10
            )
            self.on_surface_sub = self.create_subscription(
                ByteMultiArray,
                '/path/on_surface',
                self.on_surface_callback,
                10
            )
            self.pose_sub = self.create_subscription(
                PoseArray,
                '/tool_orientation/path',
                self.pose_callback,
                10
            )

            # Publisher for corrected poses
            self.debug_pub = self.create_publisher(
                PoseArray,
                '/tool_orientation/debug_path',
                10
            )

            self.path_xyz = None
            self.on_surface = None
            self.latest_pose_msg = None
            self.last_processed_stamp = None

            self.get_logger().info('Tool Orientation Debugger Node initialized')
            self.get_logger().info('Listening on /tool_orientation/path for validation')

        def path_callback(self, msg):
            n_points = len(msg.data) // 3
            if n_points == 0:
                self.get_logger().warn('Debugger received empty xyz path')
                return
            self.path_xyz = np.array(msg.data, dtype=float).reshape(n_points, 3)
            self.get_logger().info(f'Debugger stored {n_points} path points')
            self.try_validate()

        def on_surface_callback(self, msg):
            self.on_surface = np.array(msg.data, dtype=bool)
            if self.path_xyz is not None and len(self.on_surface) != len(self.path_xyz):
                self.get_logger().warn(
                    f'Debugger on_surface size mismatch (path: {len(self.path_xyz)}, flags: {len(self.on_surface)})'
                )
            self.try_validate()

        def pose_callback(self, msg):
            self.latest_pose_msg = msg
            self.try_validate()

        def try_validate(self):
            if self.path_xyz is None or self.on_surface is None or self.latest_pose_msg is None:
                return

            if len(self.on_surface) != len(self.path_xyz):
                self.get_logger().error('Cannot validate: on_surface array length does not match path length')
                return

            if len(self.latest_pose_msg.poses) != len(self.path_xyz):
                self.get_logger().error(
                    f'Cannot validate: PoseArray has {len(self.latest_pose_msg.poses)} poses, '
                    f'but path has {len(self.path_xyz)} points'
                )
                return

            current_stamp = (
                self.latest_pose_msg.header.stamp.sec,
                self.latest_pose_msg.header.stamp.nanosec
            )
            if current_stamp == self.last_processed_stamp:
                return

            try:
                self.validate_and_publish(self.latest_pose_msg)
                self.last_processed_stamp = current_stamp
            except Exception as exc:
                self.get_logger().error(f'Validation failure: {exc}')

        def validate_and_publish(self, pose_msg):
            dt = float(self.get_parameter('dt').value)
            neighbor_range = int(self.get_parameter('neighbor_range').value)
            off_surface_height = float(self.get_parameter('off_surface_height').value)
            axis_tol_deg = float(self.get_parameter('axis_tolerance_deg').value)
            offset_tol_m = float(self.get_parameter('offset_tolerance_m').value)
            transverse_tol_m = float(self.get_parameter('transverse_tolerance_m').value)

            n_points = len(pose_msg.poses)
            base_positions_m = self.path_xyz / 1000.0
            velocities = compute_velocity(self.path_xyz, dt)
            expected_tangents = self._compute_tangents(velocities)
            _, expected_orientations = compute_orientations_from_xyz(
                self.path_xyz,
                dt,
                neighbor_range
            )
            expected_normals = -expected_orientations[:, 2]

            corrected_positions = base_positions_m.copy()
            corrected_positions[~self.on_surface, 2] += off_surface_height

            actual_positions = np.zeros((n_points, 3), dtype=float)
            actual_rotations = np.zeros((n_points, 3, 3), dtype=float)
            for i, pose in enumerate(pose_msg.poses):
                actual_positions[i] = np.array([
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ], dtype=float)
                try:
                    actual_rotations[i] = R.from_quat([
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w
                    ]).as_matrix()
                except ValueError as exc:
                    self.get_logger().warn(
                        f'Pose {i}: invalid quaternion ({exc}); using identity for validation'
                    )
                    actual_rotations[i] = np.eye(3)

            errors_found = False
            axis_error_count = 0
            offset_error_count = 0

            for i in range(n_points):
                normal = expected_normals[i]
                position = actual_positions[i]

                # Check Z-axis alignment
                expected_z = expected_orientations[i][:, 2]
                actual_z = actual_rotations[i][:, 2]
                angle_z = self._axis_angle_deg(actual_z, expected_z)
                if angle_z > axis_tol_deg:
                    errors_found = True
                    axis_error_count += 1
                    self._log_axis_error(i, position, normal, 'Z', angle_z)

                # Check X-axis alignment with path tangent
                actual_x = actual_rotations[i][:, 0]
                expected_tangent = expected_tangents[i]
                angle_x = self._axis_angle_deg(actual_x, expected_tangent)
                if angle_x > axis_tol_deg:
                    errors_found = True
                    axis_error_count += 1
                    self._log_axis_error(i, position, normal, 'X', angle_x)

                # Check offset along +Z
                translation_delta = actual_positions[i] - base_positions_m[i]
                xy_error = np.linalg.norm(translation_delta[:2])
                if self.on_surface[i]:
                    if abs(translation_delta[2]) > offset_tol_m or xy_error > transverse_tol_m:
                        errors_found = True
                        offset_error_count += 1
                        self._log_offset_error(i, position, translation_delta, off_surface_height, on_surface=True)
                else:
                    z_error = abs(translation_delta[2] - off_surface_height)
                    if z_error > offset_tol_m or xy_error > transverse_tol_m:
                        errors_found = True
                        offset_error_count += 1
                        self._log_offset_error(i, position, translation_delta, off_surface_height, on_surface=False)

            if errors_found:
                self.get_logger().warn(
                    f'Validation detected {axis_error_count} axis deviations and '
                    f'{offset_error_count} offset deviations across {n_points} poses'
                )
            else:
                self.get_logger().info(f'Validation passed for all {n_points} poses')

            # Publish corrected PoseArray for visualization
            debug_msg = PoseArray()
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = pose_msg.header.frame_id or self.get_parameter('frame_id').value

            for i in range(n_points):
                pose = Pose()
                pose.position.x = float(corrected_positions[i, 0])
                pose.position.y = float(corrected_positions[i, 1])
                pose.position.z = float(corrected_positions[i, 2])

                quat = R.from_matrix(expected_orientations[i]).as_quat()
                pose.orientation.x = float(quat[0])
                pose.orientation.y = float(quat[1])
                pose.orientation.z = float(quat[2])
                pose.orientation.w = float(quat[3])

                debug_msg.poses.append(pose)

            self.debug_pub.publish(debug_msg)
            self.get_logger().info(
                f'Published corrected PoseArray with {n_points} poses on /tool_orientation/debug_path'
            )

        def _axis_angle_deg(self, actual_vec, expected_vec, epsilon=1e-10):
            actual = normalize(actual_vec)
            expected = normalize(expected_vec)
            dot_product = np.clip(np.dot(actual, expected), -1.0, 1.0)
            return np.degrees(np.arccos(dot_product))

        def _compute_tangents(self, velocities, epsilon=1e-12):
            tangents = np.zeros_like(velocities)
            fallback = np.array([1.0, 0.0, 0.0])
            n_points = len(velocities)
            for i in range(n_points):
                speed = np.linalg.norm(velocities[i])
                if speed > epsilon:
                    tangents[i] = velocities[i] / speed
                    fallback = tangents[i]
                    continue

                # Try next point for a non-zero tangent
                next_idx = i + 1
                selected = None
                while next_idx < n_points:
                    next_speed = np.linalg.norm(velocities[next_idx])
                    if next_speed > epsilon:
                        selected = velocities[next_idx] / next_speed
                        break
                    next_idx += 1

                if selected is not None:
                    tangents[i] = selected
                    fallback = selected
                else:
                    tangents[i] = fallback

            return tangents

        def _log_axis_error(self, index, position, normal, axis_label, angle_deg):
            self.get_logger().warn(
                f'Pose {index}: position=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}) m, '
                f'normal=({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}), '
                f'axis {axis_label} deviates by {angle_deg:.2f} degrees'
            )

        def _log_offset_error(self, index, position, delta, off_surface_height, on_surface):
            if on_surface:
                descriptor = 'on-surface'
                expected = 0.0
            else:
                descriptor = 'off-surface'
                expected = off_surface_height
            self.get_logger().warn(
                f'Pose {index} ({descriptor}): position=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}) m, '
                f'delta=({delta[0]:.4f}, {delta[1]:.4f}, {delta[2]:.4f}) m, '
                f'expected +Z offset {expected:.4f} m'
            )

    def main(args=None):
        rclpy.init(args=args)
        tool_node = ToolOrientationNode()
        debugger_node = ToolOrientationDebuggerNode()

        executor = MultiThreadedExecutor()
        executor.add_node(tool_node)
        executor.add_node(debugger_node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            tool_node.destroy_node()
            debugger_node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()
else:
    if __name__ == '__main__':
        print("ROS2 mode disabled. Import this module to use compute_orientations_from_xyz() function.")