#!/usr/bin/env python3

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile,
        QoSReliabilityPolicy,
        QoSDurabilityPolicy,
    )
    from std_msgs.msg import Float64MultiArray, ByteMultiArray
    from geometry_msgs.msg import PoseArray, Pose
    from scipy.spatial.transform import Rotation as R
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 packages not available. Running in standalone mode.")

NORMAL_FLIP_THRESHOLD_RAD = np.deg2rad(60.0)

def normalize(v, epsilon=1e-12):
    #Normalize vector to unit length
    norm = np.linalg.norm(v)
    if norm < epsilon:
        return np.zeros_like(v)
    return v / norm

def normalize_or_none(v, epsilon=1e-12):
    norm = np.linalg.norm(v)
    if norm < epsilon:
        return None
    return v / norm

def quaternion_axis(quat_xyzw, axis_index=1):
    #Return axis vector from quaternion (default Y axis)
    try:
        rot = R.from_quat(quat_xyzw)
        matrix = rot.as_matrix()
        axis_vec = matrix[:, axis_index]
        return normalize(axis_vec)
    except Exception:
        fallback = np.array([0.0, 1.0, 0.0]) if axis_index == 1 else np.array([0.0, 0.0, 1.0])
        return fallback

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

def compute_normal_from_neighbors(path_xyz, index, neighbor_range=3, prev_normal=None):
    #Estimate surface normal at a point using neighboring points
    #Fits a local plane and returns the normal to that plane
    #Args: path_xyz (N,3) array, index current point, neighbor_range neighbors on each side
    #Returns: normal (3,) unit vector
    n_points = len(path_xyz)

    def fallback_normal():
        if prev_normal is not None:
            return prev_normal
        return np.array([0, 0, 1])
    
    #Get neighbor indices
    start = max(0, index - neighbor_range)
    end = min(n_points, index + neighbor_range + 1)
    
    #Get local points
    local_points = path_xyz[start:end]
    
    if len(local_points) < 3:
        return fallback_normal()
    
    #Center the points
    centroid = np.mean(local_points, axis=0)
    centered = local_points - centroid
    
    #Check if points are too close together (degenerate case)
    max_distance = np.max(np.linalg.norm(centered, axis=1))
    if max_distance < 1e-10:
        return fallback_normal()
    
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
            reference = prev_normal if prev_normal is not None else np.array([0, 0, 1])
            helper = np.cross(tangent, np.cross(reference, tangent))
            if np.linalg.norm(helper) < 1e-10:
                helper = reference
            return normalize(helper)
        
        normal = vh[-1, :]  #Last row is normal direction
        
    except np.linalg.LinAlgError:
        #SVD failed to converge, estimate from local geometry
        if index > 0 and index < n_points - 1:
            v1 = path_xyz[index] - path_xyz[index - 1]
            v2 = path_xyz[index + 1] - path_xyz[index]
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm > 1e-10:
                normal = normal / norm
            else:
                normal = fallback_normal()
        else:
            normal = fallback_normal()
    
    normal = normalize(normal)

    if np.linalg.norm(normal) < 1e-10:
        if prev_normal is not None:
            return prev_normal
        return np.array([0, 0, 1])

    if prev_normal is not None:
        alignment = np.dot(normal, prev_normal)
        if alignment < 0:
            normal = -normal

    return normal


def orientation_matrix_locked_roll(velocity, normal, prev_ey=None, lock_roll=False):
    #Compute orientation matrix where X follows the path and roll about X can be locked
    #Args: velocity (3,), normal (3,), prev_ey optional previous y-axis, lock_roll flag
    #Returns: (R, ey) rotation matrix and resulting y-axis for reuse

    #Tool X-axis follows velocity (feed direction)
    vel_norm = np.linalg.norm(velocity)
    if vel_norm < 1e-10:
        ex = np.array([1.0, 0.0, 0.0])
    else:
        ex = velocity / vel_norm

    #Tool Z-axis still points into the surface (opposite normal)
    ez_target = normalize(-normal)
    ez_proj = ez_target - np.dot(ez_target, ex) * ex
    ez_norm = np.linalg.norm(ez_proj)
    if ez_norm < 1e-10:
        #Normal nearly parallel to X, pick helper axis
        helper = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(helper, ex)) > 0.95:
            helper = np.array([0.0, 1.0, 0.0])
        ez_proj = helper - np.dot(helper, ex) * ex
        ez_norm = np.linalg.norm(ez_proj)
        if ez_norm < 1e-10:
            ez_proj = np.array([0.0, 0.0, 1.0])
    ez = ez_proj / np.linalg.norm(ez_proj)

    #Baseline Y-axis from Z and X (ensures right-handed frame)
    ey_base = np.cross(ez, ex)
    ey_norm = np.linalg.norm(ey_base)
    if ey_norm < 1e-10:
        helper = np.array([0.0, 1.0, 0.0])
        ey_base = helper - np.dot(helper, ex) * ex
        ey_norm = np.linalg.norm(ey_base)
        if ey_norm < 1e-10:
            ey_base = np.array([0.0, 1.0, 0.0])
    ey_base = ey_base / np.linalg.norm(ey_base)

    ey = ey_base

    if lock_roll and prev_ey is not None:
        ey_proj = prev_ey - np.dot(prev_ey, ex) * ex
        proj_norm = np.linalg.norm(ey_proj)
        if proj_norm > 1e-6:
            ey_candidate = ey_proj / proj_norm
            if np.dot(ey_candidate, ey_base) < 0:
                ey_candidate = -ey_candidate
            ey = ey_candidate

    #Final orthonormalization to guarantee right-handed frame
    ex = normalize(ex)
    ey = normalize(ey - np.dot(ey, ex) * ex)
    ez = normalize(np.cross(ex, ey))

    R = np.column_stack([ex, ey, ez])
    return R, ey

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

def compute_orientations_from_xyz(path_xyz, dt=0.1, neighbor_range=6, smooth_orientations=True, lock_roll=False, initial_roll_ey=None):
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
    prev_normal = None
    prev_ey = normalize_or_none(initial_roll_ey) if initial_roll_ey is not None else None
    for i in range(n_points):
        velocity = velocities[i]
        
        #Estimate surface normal from neighboring points
        normal = compute_normal_from_neighbors(path_xyz, i, neighbor_range, prev_normal)

        if prev_normal is not None:
            dot = float(np.clip(np.dot(normal, prev_normal), -1.0, 1.0))
            angle = np.arccos(dot)
            if angle > NORMAL_FLIP_THRESHOLD_RAD:
                normal = prev_normal
        
        #Compute orientation matrix
        R, ey_vector = orientation_matrix_locked_roll(velocity, normal, prev_ey, lock_roll)
        
        #Enforce consistent surface-facing Z-axis
        if smooth_orientations and i > 0:
            prev_z = orientations[i-1][:, 2]
            curr_z = R[:, 2]
            if np.dot(prev_z, curr_z) < 0:
                normal = -normal
                R, ey_vector = orientation_matrix_locked_roll(velocity, normal, prev_ey, lock_roll)
        
        orientations[i] = R
        prev_normal = normal
        prev_ey = ey_vector
    
    return path_xyz, orientations


if ROS2_AVAILABLE:
    class ToolOrientationNode(Node):
        def __init__(self):
            super().__init__('tool_orientation_node')
            
            #Publisher for trajectory with quaternions (unified format)
            trajectory_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.trajectory_pub = self.create_publisher(
                PoseArray,
                '/tool_orientation/path',
                trajectory_qos
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
            self.declare_parameter('neighbor_range', 6)
            self.declare_parameter('off_surface_height', 0.05)  # 5 cm in meters
            self.declare_parameter('frame_id', 'world')  # Coordinate frame for visualization
            self.declare_parameter('lock_roll', True)
            self.declare_parameter('initial_roll_quaternion', [0.707, 0.0, 0.0, -0.707])
            
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
        
        def get_initial_roll_axis(self):
            quat_value = self.get_parameter('initial_roll_quaternion').value
            try:
                if len(quat_value) != 4:
                    raise ValueError('Quaternion must have 4 components')
                quat = [float(q) for q in quat_value]
            except Exception:
                quat = [-0.707, 0.0, 0.0, 0.707]
            return quaternion_axis(quat, axis_index=1)

        def compute_and_publish_orientations(self, path_xyz, on_surface, dt=0.1, neighbor_range=3, off_surface_height=0.05):
            #Compute orientations and publish positions + rotation matrices

            #INPUT: path_xyz is in MILLIMETERS (from camera/corrosion detection)
            #OUTPUT: positions should be in METERS (for robot)
            
            self.get_logger().info(f'Input path_xyz range: X=[{np.min(path_xyz[:,0]):.1f}, {np.max(path_xyz[:,0]):.1f}] mm')
            self.get_logger().info(f'                      Y=[{np.min(path_xyz[:,1]):.1f}, {np.max(path_xyz[:,1]):.1f}] mm')
            self.get_logger().info(f'                      Z=[{np.min(path_xyz[:,2]):.1f}, {np.max(path_xyz[:,2]):.1f}] mm')
            
            #First compute base orientations for all points (input in mm)
            lock_roll = self.get_parameter('lock_roll').value
            positions_mm, orientations = compute_orientations_from_xyz(
                path_xyz,
                dt,
                neighbor_range,
                smooth_orientations=True,
                lock_roll=lock_roll,
                initial_roll_ey=self.get_initial_roll_axis() if lock_roll else None,
            )
            
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


    def main(args=None):
        rclpy.init(args=args)
        tool_node = ToolOrientationNode()

        try:
            rclpy.spin(tool_node)
        except KeyboardInterrupt:
            pass
        finally:
            tool_node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()
else:
    if __name__ == '__main__':
        print("ROS2 mode disabled. Import this module to use compute_orientations_from_xyz() function.")