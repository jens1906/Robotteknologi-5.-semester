import numpy as np

def normalize(v, epsilon=1e-12):
    """
    Normalizes a vector to a unit vector.

    Args:
        v (np.ndarray): The vector to normalize.
        epsilon (float): A small number to prevent division by zero.

    Returns:
        np.ndarray: The normalized unit vector.
    """
    norm = np.linalg.norm(v)
    if norm < epsilon:
        return np.zeros_like(v)
    return v / norm

def derivative_of_unit_vector(h, h_dot):
    """
    Computes the derivative of a unit vector h/||h||.
    This implements the formula: d/dt(h/||h||) = (I - u @ u.T) @ h_dot / ||h||
    where u = h/||h||. This is equivalent to eq. (31) from the paper.

    Args:
        h (np.ndarray): The vector.
        h_dot (np.ndarray): The time derivative of the vector h.

    Returns:
        np.ndarray: The time derivative of the unit vector.
    """
    norm_h = np.linalg.norm(h)
    if norm_h < 1e-12:
        return np.zeros_like(h)
    
    u = h / norm_h
    u = u.reshape(-1, 1)
    I = np.identity(u.shape[0])
    
    proj_matrix = I - u @ u.T
    h_dot = h_dot.reshape(-1, 1)
    u_dot = (proj_matrix @ h_dot) / norm_h
    
    return u_dot.flatten()

def fz(x, y, params):
    """
    Computes the surface height z for a given (x, y) based on paraboloid parameters.
    fz(x,y) = cxx*x**2 + 2*cxy*x*y + cyy*y**2 + px*x + py*y + q

    Args:
        x (float): x-coordinate.
        y (float): y-coordinate.
        params (dict): Dictionary with paraboloid coefficients 
                       ('cxx', 'cxy', 'cyy', 'px', 'py', 'q').

    Returns:
        float: The surface height z.
    """
    cxx, cxy, cyy = params['cxx'], params['cxy'], params['cyy']
    px, py, q = params['px'], params['py'], params['q']
    
    return cxx*x**2 + 2*cxy*x*y + cyy*y**2 + px*x + py*y + q

def partials(x, y, params):
    """
    Computes the first and second partial derivatives of the paraboloid surface fz.

    Args:
        x (float): x-coordinate.
        y (float): y-coordinate.
        params (dict): Dictionary with paraboloid coefficients.

    Returns:
        tuple: A tuple containing (fx, fy, fxx, fxy, fyx, fyy).
    """
    cxx, cxy, cyy = params['cxx'], params['cxy'], params['cyy']
    px, py = params['px'], params['py']

    fx = 2 * cxx * x + 2 * cxy * y + px
    fy = 2 * cxy * x + 2 * cyy * y + py

    fxx = 2 * cxx
    fxy = 2 * cxy
    fyx = 2 * cxy
    fyy = 2 * cyy
    
    return fx, fy, fxx, fxy, fyx, fyy

def surface_normal(x, y, params):
    """
    Computes the outward-pointing unit normal vector to the surface.
    The surface is defined by f = [x, y, fz(x,y)]^T.
    The normal is computed as n = normalize(∂f/∂x × ∂f/∂y), which points outwards.
    This corresponds to eq. (27) in the paper.

    Args:
        x (float): x-coordinate on the surface.
        y (float): y-coordinate on the surface.
        params (dict): Parameters for the surface function fz.

    Returns:
        np.ndarray: The 3D unit normal vector n.
    """
    fx, fy, _, _, _, _ = partials(x, y, params)
    
    df_dx = np.array([1, 0, fx])
    df_dy = np.array([0, 1, fy])
    
    normal_vec = np.cross(df_dx, df_dy)
    
    return normalize(normal_vec)

def orientation_matrix(x, y, r_dot, params):
    """
    Computes the end-effector orientation matrix R.
    This follows eq. (28) from the paper.
    - ez = -n (tool axis aligned with negative normal)
    - e_gamma = T = r_dot / ||r_dot|| (tangent to the path)
    - ey = normalize(-ez × e_gamma)
    - ex = ey × ez

    Args:
        x (float): x-coordinate.
        y (float): y-coordinate.
        r_dot (np.ndarray): The velocity vector of the tool center point.
        params (dict): Parameters for the surface function fz.

    Returns:
        np.ndarray: The 3x3 orientation matrix R.
    """
    n = surface_normal(x, y, params)
    
    ez = -n
    e_gamma = normalize(r_dot)
    ey = normalize(np.cross(-ez, e_gamma))
    ex = np.cross(ey, ez)
    
    R = np.column_stack([ex, ey, ez])
    
    return R

def angular_velocity(x, y, r_dot, r_ddot, x_dot, y_dot, x_ddot, y_ddot, params):
    """
    Computes the angular velocity vector omega of the tool frame.
    This implementation follows the steps outlined in the prompt, based on
    eqs. (29) and (31) from the paper.

    Args:
        x, y (float): Current position.
        r_dot, r_ddot (np.ndarray): Tool center point velocity and acceleration.
        x_dot, y_dot (float): Velocity components in the xy-plane.
        x_ddot, y_ddot (float): Acceleration components in the xy-plane.
        params (dict): Parameters for the surface function fz.

    Returns:
        np.ndarray: The angular velocity vector omega.
    """
    fx, fy, fxx, fxy, fyx, fyy = partials(x, y, params)
    h_n = np.array([-fx, -fy, 1])
    n = normalize(h_n)

    h_n_dot = np.array(
        [-(fxx * x_dot + fxy * y_dot),
         -(fyx * x_dot + fyy * y_dot),
         0]
    )
    n_dot = derivative_of_unit_vector(h_n, h_n_dot)

    ez = -n
    e_dot_z = -n_dot

    e_gamma = normalize(r_dot)
    e_dot_gamma = derivative_of_unit_vector(r_dot, r_ddot)

    h_y = np.cross(-ez, e_gamma)
    ey = normalize(h_y)
    
    h_y_dot = np.cross(-e_dot_z, e_gamma) + np.cross(-ez, e_dot_gamma)
    e_dot_y = derivative_of_unit_vector(h_y, h_y_dot)

    ex = np.cross(ey, ez)
    e_dot_x = np.cross(e_dot_y, ez) + np.cross(ey, e_dot_z)

    R = np.column_stack([ex, ey, ez])
    R_dot = np.column_stack([e_dot_x, e_dot_y, e_dot_z])

    S = R_dot @ R.T
    omega = np.array([S[2, 1], S[0, 2], S[1, 0]])
    
    return omega
if __name__ == "__main__":
    paraboloid_params = {
        'cxx': -2.0, 'cxy': 1.0, 'cyy': -3.0,
        'px': 1.0, 'py': 1.0, 'q': 5.0
    }

    t = 1.0
    x0, y0 = 0.5, 0.5
    vx, vy = 0.1, 0.0

    x = x0 + vx * t
    y = y0 + vy * t

    x_dot = vx
    y_dot = vy
    fx, fy, _, _, _, _ = partials(x, y, paraboloid_params)
    z_dot = fx * x_dot + fy * y_dot
    r_dot = np.array([x_dot, y_dot, z_dot])

    x_ddot = 0.0
    y_ddot = 0.0
    _, _, fxx, fxy, fyx, fyy = partials(x, y, paraboloid_params)
    z_ddot = (fxx * x_dot + fxy * y_dot) * x_dot + \
             (fyx * x_dot + fyy * y_dot) * y_dot
    r_ddot = np.array([x_ddot, y_ddot, z_ddot])
    
    R = orientation_matrix(x, y, r_dot, paraboloid_params)
    omega = angular_velocity(x, y, r_dot, r_ddot, x_dot, y_dot, x_ddot, y_ddot, paraboloid_params)

    print("--- Example: Tool on a Paraboloid Surface ---")
    print(f"Time t = {t:.2f}s")
    print(f"Position (x, y) = ({x:.3f}, {y:.3f})")
    print(f"Velocity (x_dot, y_dot) = ({x_dot:.3f}, {y_dot:.3f}) m/s")
    print(f"Tool Center Velocity r_dot = {np.round(r_dot, 4)}")
    print(f"Tool Center Acceleration r_ddot = {np.round(r_ddot, 4)}\n")

    print("Orientation Matrix R (ex, ey, ez):")
    print(np.round(R, 4))
    print("\nAngular Velocity ω (in base frame):")
    print(np.round(omega, 4))
    print("---------------------------------------------")
