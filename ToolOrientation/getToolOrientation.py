import numpy as np

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
        #Not enough neighbors, return default upward normal
        return np.array([0, 0, 1])
    
    #Center the points
    centroid = np.mean(local_points, axis=0)
    centered = local_points - centroid
    
    #Use SVD to find the normal (smallest singular value direction)
    _, _, vh = np.linalg.svd(centered)
    normal = vh[-1, :]  #Last row is normal direction
    
    #Ensure normal points "upward" (positive z component)
    if normal[2] < 0:
        normal = -normal
    
    return normalize(normal)


def rotation_matrix_to_ijk(R):
    #Compute ijk from rotation matrix
    #Args: R(3,3) rotation matrix
    #Returns: ijk vector(3)
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
    ez = -normal
    
    #Tool feed direction (tangent to path)
    e_gamma = normalize(velocity)
    
    #Tool y-axis perpendicular to tool axis and feed direction
    ey = normalize(np.cross(-ez, e_gamma))
    
    #Tool x-axis completes right-handed frame
    ex = np.cross(ey, ez)
    
    #Construct rotation matrix (columns are basis vectors)
    R = np.column_stack([ex, ey, ez])
    
    return R

def compute_orientations_from_xyz(path_xyz, dt=0.1, neighbor_range=3):
    #Main function: compute tool orientations from cartesian XYZ points
    #Args: path_xyz (N,3) array of [x,y,z] in sequence, dt time step, neighbor_range for normal estimation
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
        
        """
        If ijk is needed, then run this to convert to ijk!
        ijk = rotation_matrix_to_ijk(R)
        orientations[i] = ijk
        """

        orientations[i] = R
    
    return path_xyz, orientations

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