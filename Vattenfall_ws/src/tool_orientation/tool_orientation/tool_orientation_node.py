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


def orientation_matrix_min_twist(velocity, normal, prev_ex=None):
    #Compute orientation matrix using a rotation-minimizing frame
    #Args: velocity (3,) vector, normal (3,) vector, prev_ex optional previous x-axis
    #Returns: R (3,3) rotation matrix minimizing twist along path

    #Tool z-axis points into surface (opposite of normal)
    ez = normalize(-normal)

    #Determine feed direction projected onto plane perpendicular to ez
    vel_norm = np.linalg.norm(velocity)
    if vel_norm < 1e-10:
        fallback = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(fallback, ez)) > 0.95:
            fallback = np.array([0.0, 1.0, 0.0])
        e_gamma = fallback
    else:
        e_gamma = velocity / vel_norm
    e_gamma = e_gamma - np.dot(e_gamma, ez) * ez
    if np.linalg.norm(e_gamma) < 1e-10:
        #Velocity parallel to ez; pick arbitrary tangent direction
        e_gamma = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(e_gamma, ez)) > 0.95:
            e_gamma = np.array([0.0, 1.0, 0.0])
        e_gamma = e_gamma - np.dot(e_gamma, ez) * ez

    #Temporary Y-axis from cross product between -ez (surface normal) and feed
    ey_temp = np.cross(-ez, e_gamma)
    ey_norm = np.linalg.norm(ey_temp)
    if ey_norm < 1e-10:
        helper = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(helper, ez)) > 0.95:
            helper = np.array([0.0, 1.0, 0.0])
        ey_temp = np.cross(-ez, helper)
        ey_norm = np.linalg.norm(ey_temp)
        if ey_norm < 1e-10:
            ey_temp = np.array([0.0, 1.0, 0.0])
    ey_temp = ey_temp / np.linalg.norm(ey_temp)

    #Default x-axis from temporary frame
    ex_default = np.cross(ey_temp, ez)
    ex_default = normalize(ex_default)
    ey_default = ey_temp

    ex = ex_default
    ey = ey_default

    #Apply rotation-minimizing projection of previous x-axis if available
    if prev_ex is not None:
        proj = prev_ex - np.dot(prev_ex, ez) * ez
        proj_norm = np.linalg.norm(proj)
        if proj_norm > 1e-6:
            ex_candidate = proj / proj_norm
            ey_candidate = np.cross(-ez, ex_candidate)
            ey_candidate_norm = np.linalg.norm(ey_candidate)
            if ey_candidate_norm > 1e-6:
                ex = ex_candidate
                ey = ey_candidate / ey_candidate_norm

    #Ensure orthonormality without altering ez direction
    ex = normalize(ex)
    ey = np.cross(-ez, ex)
    ey_norm = np.linalg.norm(ey)
    if ey_norm < 1e-10:
        ey = ey_default
    else:
        ey = ey / ey_norm
    ex = normalize(np.cross(ey, ez))

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
    prev_normal = None
    prev_ex = None
    for i in range(n_points):
        velocity = velocities[i]
        
        #Estimate surface normal from neighboring points
        normal = compute_normal_from_neighbors(path_xyz, i, neighbor_range, prev_normal)
        
        #Compute orientation matrix
        R = orientation_matrix_min_twist(velocity, normal, prev_ex)
        
        #Enforce consistent surface-facing Z-axis
        if smooth_orientations and i > 0:
            prev_z = orientations[i-1][:, 2]
            curr_z = R[:, 2]
            if np.dot(prev_z, curr_z) < 0:
                normal = -normal
                R = orientation_matrix_min_twist(velocity, normal, prev_ex)
        
        orientations[i] = R
        prev_normal = normal
        prev_ex = R[:, 0]
    
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