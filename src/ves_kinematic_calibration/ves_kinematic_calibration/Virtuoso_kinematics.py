import numpy as np
from scipy.optimize import fsolve


class VirtuosoKinematics:
    def __init__(self, kappa=60.0 / 1000.0, tool_len=0.0):
        # Everything in MILLIMETERS
        self.kappa = kappa  # 0.06 (1/mm)
        self.tool_len = tool_len  # Extension in mm
        self.collar_len = 5.0  # 5mm guide collar
        self.r_tube = 1 / kappa  # Radius in mm (e.g., 16.66mm)
        self.c_mm = 0.3  # 0.3mm clearance

    def solve_ik(self, p_target):
        # Ensure p_target is mm. If you pass meters, multiply by 1000 here.
        x, y, z = p_target
        R = 1.0 / self.kappa

        theta = float(np.arctan2(x, -y))
        r_target = np.sqrt(x ** 2 + y ** 2)
        z_target = z

        def find_err(d1):
            # L is the length of the tube sticking out of the collar
            L = max(d1 - self.collar_len, 0)
            alpha = L / R

            # L_in is how much is retracted inside the collar
            L_in = max(0, self.collar_len - d1)

            # Clearance angle math (Using unified mm)
            term = L_in ** 2 + 2 * self.c_mm * self.r_tube - self.c_mm ** 2
            # Use max(0, term) to avoid nan from tiny negatives
            theta_c = 2 * np.arctan((L_in - np.sqrt(max(0, term))) / (self.c_mm - 2 * self.r_tube))

            cc, sc = np.cos(theta_c), np.sin(theta_c)
            r_p = cc * r_target - sc * z_target
            z_p = sc * r_target + cc * z_target

            r_arc = R * (1 - np.cos(alpha))
            z_arc = R * np.sin(alpha)

            if abs(np.sin(alpha)) > 1e-7:
                s_geo = (r_p - r_arc) / np.sin(alpha)
                return (z_arc + s_geo * np.cos(alpha)) - z_p
            else:
                return r_p - r_arc

        # Initial guess 10.0mm (closer to the typical workspace than 0.05mm)
        d1_sol, info, ier, msg = fsolve(find_err, 10.0, full_output=True)

        if ier != 1:
            # If fsolve fails, try a different start or a scalar minimizer
            raise ValueError(f"IK Convergence failed: {msg}")

        d1 = float(d1_sol[0])

        # Calculate d2 based on the solved d1
        L = max(d1 - self.collar_len, 0)
        alpha = L / R
        L_in = max(0, self.collar_len - d1)
        term = L_in ** 2 + 2 * self.c_mm * self.r_tube - self.c_mm ** 2
        theta_c = 2 * np.arctan((L_in - np.sqrt(max(0, term))) / (self.c_mm - 2 * self.r_tube))

        cc, sc = np.cos(theta_c), np.sin(theta_c)
        r_p = cc * r_target - sc * z_target
        r_arc = R * (1 - np.cos(alpha))

        if abs(np.sin(alpha)) > 1e-7:
            s_geo = (r_p - r_arc) / np.sin(alpha)
        else:
            z_p = sc * r_target + cc * z_target
            s_geo = z_p

        d2 = float(s_geo + L - self.tool_len)

        return [0.0, theta, d1, d2, 0.0]

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