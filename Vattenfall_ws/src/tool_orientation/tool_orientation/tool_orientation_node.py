#!/usr/bin/env python3

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, MultiArrayDimension
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


def rotation_matrix_to_ijk(R):
    #Compute ijk from rotation matrix
    #Args: R(3,3) rotation matrix
    #Returns: ijk vector(3)
    #Dont need to comment math here, it is normal stuff from internet
    angle = np.arccos((np.trace(R) - 1) / 2)
    
    if np.abs(angle) < 1e-10:
        return np.array([0.0, 0.0, 0.0])
    
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(angle))
    
    ijk = axis * angle
    return ijk


def orientation_matrix_from_path(x, y, z, velocity, normal):
    #Compute orientation matrix at a point
    #Args: x,y,z position, velocity (3,) vector, normal (3,) vector
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
        x, y, z = path_xyz[i]
        velocity = velocities[i]
        
        #Estimate surface normal from neighboring points
        normal = compute_normal_from_neighbors(path_xyz, i, neighbor_range)
        
        #Compute orientation matrix
        R = orientation_matrix_from_path(x, y, z, velocity, normal)
        
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
                R_flipped = orientation_matrix_from_path(x, y, z, velocity, -normal)
                R_diff_flipped = R_prev.T @ R_flipped
                trace_flipped = np.trace(R_diff_flipped)
                angle_flipped = np.arccos(np.clip((trace_flipped - 1) / 2, -1, 1))
                
                if angle_flipped < angle_diff:
                    R = R_flipped
        
        """
        If ijk is needed, then run this to convert to ijk!
        ijk = rotation_matrix_to_ijk(R)
        orientations[i] = ijk
        """

        orientations[i] = R
    
    return path_xyz, orientations


if ROS2_AVAILABLE:
    class ToolOrientationNode(Node):
        def __init__(self):
            super().__init__('tool_orientation_node')
            
            #Publisher for positions and rotation matrices
            self.trajectory_pub = self.create_publisher(
                Float64MultiArray,
                '/tool_orientation/xyz_rotation',
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
                Float64MultiArray,
                '/path/on_surface',
                self.on_surface_callback,
                10
            )
            
            #Parameters
            self.declare_parameter('dt', 0.1)
            self.declare_parameter('neighbor_range', 3)
            self.declare_parameter('off_surface_height', 0.05)  # 5 cm in meters
            
            #Storage for received data
            self.path_xyz = None
            self.on_surface = None
            self.data_processed = False  # Flag to avoid reprocessing same data
            
            self.get_logger().info('Tool Orientation Node initialized')
            self.get_logger().info('Waiting for path on topic: /parameterization/xyz_path')
            self.get_logger().info('Waiting for on_surface data on topic: /path/on_surface')
        
        def on_surface_callback(self, msg):
            #Callback for receiving on_surface boolean array
            #Expected input: Float64MultiArray with [bool1, bool2, ...] as floats (0.0 or 1.0)
            self.on_surface = np.array(msg.data, dtype=bool)
            self.data_processed = False  # New data received, reset flag
            self.get_logger().info(f'Received on_surface data with {len(self.on_surface)} points')
            
            #Try to process if we have both path and on_surface data
            self.try_process_data()
        
        def path_callback(self, msg):
            #Callback for receiving path points and computing orientations
            #Expected input: Float64MultiArray with [x1, y1, z1, x2, y2, z2, ...]
            self.get_logger().info(f'Received path on topic: /parameterization/xyz_path')
            
            #Reshape data to (N, 3)
            n_points = len(msg.data) // 3
            self.path_xyz = np.array(msg.data).reshape(n_points, 3)
            self.data_processed = False  # New data received, reset flag
            
            self.get_logger().info(f'Received path with {n_points} points')
            
            #Try to process if we have both path and on_surface data
            self.try_process_data()
        
        def try_process_data(self):
            #Process data only when both path and on_surface are available
            if self.path_xyz is None:
                self.get_logger().info('Waiting for path data...')
                return
            
            if self.on_surface is None:
                self.get_logger().info('Waiting for on_surface data...')
                return
            
            #Skip if data already processed
            if self.data_processed:
                self.get_logger().debug('Data already processed, skipping...')
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
            #First compute base orientations for all points
            positions, orientations = compute_orientations_from_xyz(path_xyz, dt, neighbor_range)
            
            #Apply off-surface offsets where needed
            adjusted_positions = apply_off_surface_offset(positions, orientations, on_surface, off_surface_height)
            
            #Count how many points are off-surface
            n_off_surface = np.sum(~on_surface)
            self.get_logger().info(
                f'Applied off-surface offset to {n_off_surface}/{len(positions)} points '
                f'(offset: {off_surface_height*100:.1f} cm)'
            )
            
            #Create Float64MultiArray message
            trajectory_msg = Float64MultiArray()
            
            #Setup dimensions: [n_points, 12] where each row is [x, y, z, r11, r12, r13, r21, r22, r23, r31, r32, r33]
            dim_points = MultiArrayDimension()
            dim_points.label = "points"
            dim_points.size = len(adjusted_positions)
            dim_points.stride = len(adjusted_positions) * 12
            
            dim_data = MultiArrayDimension()
            dim_data.label = "data"
            dim_data.size = 12
            dim_data.stride = 12
            
            trajectory_msg.layout.dim = [dim_points, dim_data]
            trajectory_msg.layout.data_offset = 0
            
            #Flatten data: [x, y, z, rotation_matrix_flattened]
            for i in range(len(adjusted_positions)):
                trajectory_msg.data.append(float(adjusted_positions[i][0]))
                trajectory_msg.data.append(float(adjusted_positions[i][1]))
                trajectory_msg.data.append(float(adjusted_positions[i][2]))
                
                #Flatten rotation matrix (row-major)
                R_flat = orientations[i].flatten()
                for val in R_flat:
                    trajectory_msg.data.append(float(val))
            
            self.trajectory_pub.publish(trajectory_msg)
            self.get_logger().info(f'Published {len(adjusted_positions)} waypoints with rotation matrices')
            
            return adjusted_positions, orientations

    def main(args=None):
        rclpy.init(args=args)
        node = ToolOrientationNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()
else:
    if __name__ == '__main__':
        print("ROS2 mode disabled. Import this module to use compute_orientations_from_xyz() function.")

#Compute orientations
#   positions, orientations = compute_orientations_from_xyz(path_xyz)
    
#Display results
"""
    print("Tool Orientation Computation Complete")
    print(f"Number of waypoints: {len(positions)}")
    print(f"\nExample - Point 25:")
    print(f"  Position: {positions[25]}")
    print(f"  Orientation matrix:\n{orientations[25]}")
    print(f"  Determinant (should be ~1): {np.linalg.det(orientations[25]):.6f}")
    
    #Save results
    np.save('positions_xyz.npy', positions)
    np.save('orientations_xyz.npy', orientations)
    print(f"\nSaved:")
    print(f"  - positions_xyz.npy: {positions.shape}")
    print(f"  - orientations_xyz.npy: {orientations.shape}")
"""