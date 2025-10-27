import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

start = time.time()

def cubic_bezier(tau, b0, b1, b2, b3):
    return (1-tau)**3 * b0 + 3 * (1-tau)**2 * tau * b1 + 3 * (1-tau) * tau**2 * b2 + tau**3 * b3

def get_start_end(lines,idx):
        # even: start = u[0], end = u[-1]; odd: reversed
        if idx % 2 == 0:
            start = np.array([lines[0][idx][0],  lines[1][idx][0],  lines[2][idx][0]])
            end   = np.array([lines[0][idx][-1], lines[1][idx][-1], lines[2][idx][-1]])
        else:
            start = np.array([lines[0][idx][-1], lines[1][idx][-1], lines[2][idx][-1]])
            end   = np.array([lines[0][idx][0],  lines[1][idx][0],  lines[2][idx][0]])
        return start, end

def path_planning(u,v, d, b): 
    # Create the surface
    U, V = np.meshgrid(u, v)
    X = -np.sin(U) * np.cos(V)
    Y = -np.sin(V) * np.cos(U)
    Z = -(1 - np.sqrt(1 - np.sin(U)**2 * np.cos(V)**2 - np.sin(V)**2 * np.cos(U)**2))
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(X, Y, Z, alpha=0.7, cmap='viridis', edgecolor='none')

    spacing = np.arange(u[0], u[-1] + d * 0.5, d) # a to b with step d (d*0.5 to include endpoint)

    lines = [[] for _ in range(3)]
    # Plot path lines
    for v_k in spacing:
        x_line = -np.sin(u) * np.cos(v_k)
        y_line = -np.sin(v_k) * np.cos(u)
        z_line = -(1 - np.sqrt(1 - np.sin(u)**2 * np.cos(v_k)**2 - np.sin(v_k)**2 * np.cos(u)**2))
        lines[0].append(x_line)
        lines[1].append(y_line)
        lines[2].append(z_line)
        ax.plot3D(x_line, y_line, z_line, 'k--', alpha=0.6)

    # Interconnection of lines
    tau = np.linspace(0, 1, num=50) # num = points along the bezier curve
    print(tau)

    for i in range(len(spacing)-1):
        k_start, k_end = get_start_end(lines,i)
        k1_start, k1_end = get_start_end(lines,i+1)

        v_k  = k_end - k_start
        v_k1 = k1_end - k1_start

        norm_vk  = np.linalg.norm(v_k)
        norm_vk1 = np.linalg.norm(v_k1)
        if norm_vk == 0 or norm_vk1 == 0:
            continue

        # normalize with vector norm (avoid elementwise abs and divide-by-zero)
        rho_k_d  = v_k  / norm_vk
        rho_k1_d = v_k1 / norm_vk1

        b0 = k_end
        b1 = k_end + b * rho_k_d
        b2 = k1_start - b * rho_k1_d
        b3 = k1_start

        # ensure tau broadcasts to shape (len(tau), 1) -> cubic_bezier returns (len(tau), 3)
        curve_3d = cubic_bezier(tau[:, None], b0, b1, b2, b3)

        # plot columns (x,y,z)
        ax.plot3D(curve_3d[:, 0], curve_3d[:, 1], curve_3d[:, 2], 'r-', alpha=0.8)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Path Planning on Curved Surface')
    fig.colorbar(surf, ax=ax, shrink=0.5)
    end = time.time()
    print(f"Execution Time: {end - start} seconds")
    plt.show()

u_len = 1.0
v_len = 1.0

d = 0.01 # d is the spacing between path lines
b = d/2 # b affect the curvature of bezier curve

u = np.linspace(-u_len/2, u_len/2, 50)
v = np.linspace(-v_len/2, v_len/2, 50)

path_planning(u, v, d, b)
