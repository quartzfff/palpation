import numpy as np
# Fix for NumPy 2.0+ compatibility with older SciPy
if not hasattr(np, 'Inf'):
    np.Inf = np.inf
if not hasattr(np, 'float_'):
    np.float_ = float

from scipy.optimize import fsolve


class VirtuosoKinematics:
    def __init__(self, kappa=60.0, tool_len=15.0):
        # Physical Constants (in meters)
        self.kappa = kappa        # Curvature (1/R)
        self.tool_len = tool_len  # Extension of tool beyond arc tip
        self.collar_len = 0.005   # 5mm guide collar
        self.r_tube = 1000/kappa   # Equivalent to 1000/60 mm in meters
        self.c_mm = 0.0003        # 0.3mm clearance
        
    def solve_ik(self, p_target):
        """
        Calculates Joints [ir, or, it, ot, tool] from [x, y, z]
        p_target should be in meters.
        """
        x, y, z = p_target
        R = 1.0 / self.kappa
        
        # 1. Azimuth (Rotation)
        # Note: Added small epsilon to avoid atan2(0,0)
        theta = float(np.arctan2(x, -y))
        r_target = np.sqrt(x**2 + y**2)
        z_target = z

        # 2. Solver for d1 (Insertion Depth / it)
        def find_err(d1):
            L = max(d1 - self.collar_len, 0)
            alpha = L / R
            L_in = max(0, self.collar_len - d1)
            
            # Clearance angle math
            term = L_in**2 + 2*self.c_mm*self.r_tube - self.c_mm**2
            theta_c = 2 * np.arctan((L_in - np.sqrt(max(0, term))) / (self.c_mm - 2*self.r_tube))
            
            cc, sc = np.cos(theta_c), np.sin(theta_c)
            r_p = cc * r_target - sc * z_target
            z_p = sc * r_target + cc * z_target
            
            r_arc = R * (1 - np.cos(alpha))
            z_arc = R * np.sin(alpha)
            
            if abs(np.sin(alpha)) > 1e-7:
                s_geo = (r_p - r_arc) / np.sin(alpha)
                return (z_arc + s_geo * np.cos(alpha)) - z_p
            return r_p - r_arc

        # Solve d1. Initial guess 0.05m
        d1_sol, info, ier, msg = fsolve(find_err, 0.05, full_output=True)
        if ier != 1:
            raise ValueError(f"IK Convergence failed: {msg}")
            
        d1 = float(d1_sol[0])
        
        # 3. Solve for d2 (Translation / ot)
        L = max(d1 - self.collar_len, 0)
        alpha = L / R
        L_in = max(0, self.collar_len - d1)
        term = L_in**2 + 2*self.c_mm*self.r_tube - self.c_mm**2
        theta_c = 2 * np.arctan((L_in - np.sqrt(max(0, term))) / (self.c_mm - 2*self.r_tube))
        
        r_p = np.cos(theta_c) * r_target - np.sin(theta_c) * z_target
        r_arc = R * (1 - np.cos(alpha))
        
        if abs(np.sin(alpha)) > 1e-7:
            s_geo = (r_p - r_arc) / np.sin(alpha)
        else:
            # Straight configuration case
            z_p = np.sin(theta_c) * r_target + np.cos(theta_c) * z_target
            s_geo = z_p

        d2 = float(s_geo + L - self.tool_len)

        # Output format for ROS: [ir, or, it, ot, tool]
        return [theta, theta, d1, d2, 0.0]

    def check_grid_feasibility(self, grid, push_depth, push_dir):
        """Checks every point in grid and logs failures."""
        failures = []
        for i, pt in enumerate(grid):
            # Check base point
            try:
                self.solve_ik(pt[:3])
            except Exception as e:
                failures.append(f"Point {i} Surface: {e}")
            
            # Check depth point
            p_push = np.array(pt[:3]) + push_depth * push_dir
            try:
                self.solve_ik(p_push)
            except Exception as e:
                failures.append(f"Point {i} Push: {e}")
        
        return failures