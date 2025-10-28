import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point

def create_blob_outline(u_len, v_len, n_points=100):
    """Create an organic blob-like outline similar to the image"""
    theta = np.linspace(0, 2*np.pi, n_points)
    a = u_len / 2
    b = v_len / 4
    
    r = np.sqrt((a * np.cos(theta))**2 + (b * np.sin(theta))**2)
    r = r * (1 + 0.15 * np.sin(3*theta) + 0.1 * np.sin(5*theta) + 0.08 * np.cos(7*theta))

    blob_x, blob_y = r * np.cos(theta), r * np.sin(theta)

    len_x = np.max(blob_x) - np.min(blob_x)
    len_y = np.max(blob_y) - np.min(blob_y)
    
    return blob_x, blob_y, len_x, len_y

def cubic_bezier(tau, b0, b1, b2, b3):
    return (1-tau)**3 * b0 + 3 * (1-tau)**2 * tau * b1 + 3 * (1-tau) * tau**2 * b2 + tau**3 * b3

def get_start_end(lines, idx):
    # even: start = u[0], end = u[-1]; odd: reversed
    if idx % 2 == 0:
        start = np.array([lines[0][idx][0], lines[1][idx][0]])
        end = np.array([lines[0][idx][-1], lines[1][idx][-1]])
    else:
        start = np.array([lines[0][idx][-1], lines[1][idx][-1]])
        end = np.array([lines[0][idx][0], lines[1][idx][0]])
    return start, end

def create_continuous_path(lines, d, n=50):
    """Create a continuous path combining lines and Bezier interconnections"""
    b = d / 2 # Curving factor
     
    tau = np.linspace(0, 1, n)
    path = []

    for i in range(len(lines[0])):
        k_start, k_end = get_start_end(lines, i)
        
        if i % 2 == 0: # shift direction for zig-zag
            line_points = np.column_stack([lines[0][i], lines[1][i]])
        else:
            line_points = np.column_stack([lines[0][i][::-1], lines[1][i][::-1]])
        
        path.extend(line_points)
        
        # Add interconnection to next line (if not last line)
        if i < len(lines[0]) - 1:
            k1_start, k1_end = get_start_end(lines, i+1)

            norm_vk = np.linalg.norm(k_end - k_start)
            norm_vk1 = np.linalg.norm(k1_end - k1_start)
            
            if norm_vk != 0 and norm_vk1 != 0:
                # Normalised direction vectors
                rho_k_d = (k_end - k_start) / norm_vk
                rho_k1_d = (k1_end - k1_start) / norm_vk1

                b0 = k_end
                b1 = k_end + b * rho_k_d
                b2 = k1_start - b * rho_k1_d
                b3 = k1_start

                path.extend(cubic_bezier(tau[:, None], b0, b1, b2, b3))

    return np.array(path)

def path_planning(u_len, v_len, d, n=50):
    u = np.linspace(-u_len/2, u_len/2, n)

    v_start = -v_len/2 + d/2
    v_end = v_len/2 - d/2
    line_positions = np.arange(v_start, v_end + d * 0.5, d)    
    lines = [[u for _ in line_positions], [np.full_like(u, v) for v in line_positions]]
    
    path = create_continuous_path(lines, d)

    return path


v_len = 10.0
u_len = 24.0
d = 1

blob_x, blob_y, len_x, len_y = create_blob_outline(u_len, v_len)

path = path_planning(len_x, len_y, d)

print(f"Total points in path: {len(path)}")

# Plot
plt.figure(figsize=(10, 6))
plt.fill(blob_x, blob_y, color='#FF6B6B', alpha=0.3, label='Blob outline')
plt.plot(blob_x, blob_y, color='#FF6B6B', linewidth=2)
plt.plot(path[:, 0], path[:, 1], 'b-', linewidth=1.5, alpha=0.5, label='Original path')
plt.plot(path[0, 0], path[0, 1], 'go', markersize=7, label='Start')
plt.plot(path[-1, 0], path[-1, 1], 'ro', markersize=7, label='End')
plt.xlabel('u (x)')
plt.ylabel('v (y)')
plt.title(f'Path Planning: Lines along u with {d} spacing in v')
plt.grid(True, alpha=0.3)
plt.legend()
plt.axis('equal')
plt.show()