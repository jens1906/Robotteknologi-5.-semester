import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

def create_curved_surface(point1, point2, max_z_point, num_points=50, u_range=(0, 1), v_range=(0, 1)):
    """
    Create a curved rectangular surface between two 3D points passing through a max z point.
    
    Parameters:
    point1: tuple (x, y, z) - first corner of rectangle
    point2: tuple (x, y, z) - opposite corner of rectangle
    max_z_point: tuple (x, y, z) - point with maximum z value
    num_points: number of points along the curve
    u_range: tuple (min, max) - range of u parameter to include (0 to 1)
    v_range: tuple (min, max) - range of v parameter to include (0 to 1)
    """
    # Create parameter arrays with specified ranges
    u = np.linspace(u_range[0], u_range[1], num_points)  # along the curve
    v = np.linspace(v_range[0], v_range[1], num_points)  # across the rectangle width
    U, V = np.meshgrid(u, v)
    
    # Define the four corners of the rectangle
    p1 = np.array(point1)
    p2 = np.array(point2)
    p_max = np.array(max_z_point)
    
    # Calculate the other two corners to form a rectangle
    p3 = np.array([point2[0], point1[1], point1[2]])
    p4 = np.array([point1[0], point2[1], point1[2]])
    
    # Interpolate between corners to get rectangle edges at each u position
    edge1_x = (1-U) * p1[0] + U * p3[0]
    edge1_y = (1-U) * p1[1] + U * p3[1]
    edge1_z = (1-U) * p1[2] + U * p3[2]
    
    edge2_x = (1-U) * p4[0] + U * p2[0]
    edge2_y = (1-U) * p4[1] + U * p2[1]
    edge2_z = (1-U) * p4[2] + U * p2[2]
    
    # Interpolate across the width to get base rectangle
    base_X = (1-V) * edge1_x + V * edge2_x
    base_Y = (1-V) * edge1_y + V * edge2_y
    base_Z = (1-V) * edge1_z + V * edge2_z
    
    # Use quadratic Bezier curve through all three points (p1, p_max, p2)
    bezier_X = (1-U)**2 * p1[0] + 2*(1-U)*U * p_max[0] + U**2 * p2[0]
    bezier_Y = (1-U)**2 * p1[1] + 2*(1-U)*U * p_max[1] + U**2 * p2[1]
    bezier_Z = (1-U)**2 * p1[2] + 2*(1-U)*U * p_max[2] + U**2 * p2[2]
    
    # Calculate displacement from flat rectangle to curved path
    displacement_X = bezier_X - ((1-U) * p1[0] + U * p2[0])
    displacement_Y = bezier_Y - ((1-U) * p1[1] + U * p2[1])
    displacement_Z = bezier_Z - ((1-U) * p1[2] + U * p2[2])
    
    # Apply displacement to create curved surface
    X = base_X + displacement_X
    Y = base_Y + displacement_Y
    Z = base_Z + displacement_Z
    
    return X, Y, Z, u, v

def cubic_bezier(tau, b0, b1, b2, b3):
    return (1-tau)**3 * b0 + 3 * (1-tau)**2 * tau * b1 + 3 * (1-tau) * tau**2 * b2 + tau**3 * b3

def get_start_end(lines, idx):
    # even: start = u[0], end = u[-1]; odd: reversed
    if idx % 2 == 0:
        start = np.array([lines[0][idx][0],  lines[1][idx][0],  lines[2][idx][0]])
        end   = np.array([lines[0][idx][-1], lines[1][idx][-1], lines[2][idx][-1]])
    else:
        start = np.array([lines[0][idx][-1], lines[1][idx][-1], lines[2][idx][-1]])
        end   = np.array([lines[0][idx][0],  lines[1][idx][0],  lines[2][idx][0]])
    return start, end

def path_planning_on_surface(point1, point2, max_z_point, u_range=(0, 1), v_range=(0, 1), d=0.1, b=0.05):
    start_time = time.time()
    
    # Create the curved surface
    num_points = 50
    X, Y, Z, u_arr, v_arr = create_curved_surface(point1, point2, max_z_point, num_points, u_range, v_range)
    
    # Get the surface equation function
    p1 = np.array(point1)
    p2 = np.array(point2)
    p_max = np.array(max_z_point)
    p3 = np.array([point2[0], point1[1], point1[2]])
    p4 = np.array([point1[0], point2[1], point1[2]])
    
    def surface_point(u_val, v_val):
        """Calculate X, Y, Z for given u, v parameters"""
        # Rectangle edges
        edge1 = (1-u_val) * p1 + u_val * p3
        edge2 = (1-u_val) * p4 + u_val * p2
        
        # Base rectangle point
        base = (1-v_val) * edge1 + v_val * edge2
        
        # Bezier curve point
        bezier = (1-u_val)**2 * p1 + 2*(1-u_val)*u_val * p_max + u_val**2 * p2
        
        # Displacement
        linear = (1-u_val) * p1 + u_val * p2
        displacement = bezier - linear
        
        # Final point
        return base + displacement
    
    # Setup plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(X, Y, Z, alpha=0.3, cmap='viridis', edgecolor='none')
    
    # Create path lines at regular v intervals
    v_min, v_max = v_range
    spacing = np.arange(v_min, v_max + d * 0.5, d)
    
    u_min, u_max = u_range
    u = np.linspace(u_min, u_max, num_points)
    
    lines = [[] for _ in range(3)]
    
    # Plot path lines
    for v_k in spacing:
        x_line = []
        y_line = []
        z_line = []
        
        for u_val in u:
            point = surface_point(u_val, v_k)
            x_line.append(point[0])
            y_line.append(point[1])
            z_line.append(point[2])
        
        x_line = np.array(x_line)
        y_line = np.array(y_line)
        z_line = np.array(z_line)
        
        lines[0].append(x_line)
        lines[1].append(y_line)
        lines[2].append(z_line)
        ax.plot3D(x_line, y_line, z_line, 'k--', alpha=0.6, linewidth=1)
    
    # Interconnection of lines with Bezier curves
    tau = np.linspace(0, 1, num=50)
    
    for i in range(len(spacing)-1):
        k_start, k_end = get_start_end(lines, i)
        k1_start, k1_end = get_start_end(lines, i+1)
        
        v_k  = k_end - k_start
        v_k1 = k1_end - k1_start
        
        norm_vk  = np.linalg.norm(v_k)
        norm_vk1 = np.linalg.norm(v_k1)
        if norm_vk == 0 or norm_vk1 == 0:
            continue
        
        rho_k_d  = v_k  / norm_vk
        rho_k1_d = v_k1 / norm_vk1
        
        b0 = k_end
        b1 = k_end + b * rho_k_d
        b2 = k1_start - b * rho_k1_d
        b3 = k1_start
        
        curve_3d = cubic_bezier(tau[:, None], b0, b1, b2, b3)
        ax.plot3D(curve_3d[:, 0], curve_3d[:, 1], curve_3d[:, 2], 'r-', alpha=0.8, linewidth=2)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Path Planning on Curved Rectangular Surface')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_zlim(0, 8)
    
    end_time = time.time()
    print(f"Execution Time: {end_time - start_time:.4f} seconds")
    plt.show()

# Example usage
point1 = (0, 0, 0)
point2 = (10, 10, 0)
max_z_point = (5, 5, 3)

# Path planning on the cut-out section
path_planning_on_surface(
    point1, point2, max_z_point, 
    u_range=(0.2, 0.8), 
    v_range=(0.3, 0.7), 
    d=0.05,  # spacing between path lines
    b=0.025  # bezier curve parameter
)