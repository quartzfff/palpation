class VirtuosoKinematics:
    def __init__(self, kappa=60.0, tool_len=0.0):
        # All inputs here should be in METERS
        self.kappa = kappa  # 60.0 (1/m)
        self.tool_len = tool_len  # Extension in meters
        self.collar_len = 0.005  # 5mm guide collar
        self.r_tube = 1.0 / kappa  # Radius in meters
        self.c_m = 0.0003  # 0.3mm clearance in meters

    def solve_ik(self, p_target):
        """
        Input: p_target [x, y, z] in METERS
        Output: [ir, or, it, ot, tool] in METERS/RADIANS
        """
        # Internal conversion to MM for numerical stability during solving
        target_mm = np.array(p_target) * 1000.0
        kappa_mm = self.kappa / 1000.0
        R_mm = 1.0 / kappa_mm
        collar_mm = self.collar_len * 1000.0
        r_tube_mm = self.r_tube * 1000.0
        c_mm = self.c_m * 1000.0
        tool_mm = self.tool_len * 1000.0

        x, y, z = target_mm
        theta = float(np.arctan2(x, -y))
        r_target = np.sqrt(x ** 2 + y ** 2)
        z_target = z

        def find_err(d1):
            L = max(d1 - collar_mm, 0)
            alpha = L / R_mm
            L_in = max(0, collar_mm - d1)

            # Clearance math
            term = L_in ** 2 + 2 * c_mm * r_tube_mm - c_mm ** 2
            theta_c = 2 * np.arctan((L_in - np.sqrt(max(0, term))) / (c_mm - 2 * r_tube_mm))

            cc, sc = np.cos(theta_c), np.sin(theta_c)
            r_p = cc * r_target - sc * z_target
            z_p = sc * r_target + cc * z_target

            r_arc = R_mm * (1 - np.cos(alpha))
            z_arc = R_mm * np.sin(alpha)

            if abs(np.sin(alpha)) > 1e-7:
                s_geo = (r_p - r_arc) / np.sin(alpha)
                return (z_arc + s_geo * np.cos(alpha)) - z_p
            return r_p - r_arc

        # Solve in MM. Initial guess 10mm
        d1_sol, info, ier, msg = fsolve(find_err, 10.0, full_output=True)
        if ier != 1:
            raise ValueError(f"IK Convergence failed: {msg}")

        d1_mm = float(d1_sol[0])

        # Calculate d2 in MM
        L = max(d1_mm - collar_mm, 0)
        alpha = L / R_mm
        L_in = max(0, collar_mm - d1_mm)
        term = L_in ** 2 + 2 * c_mm * r_tube_mm - c_mm ** 2
        theta_c = 2 * np.arctan((L_in - np.sqrt(max(0, term))) / (c_mm - 2 * r_tube_mm))

        cc, sc = np.cos(theta_c), np.sin(theta_c)
        r_p = cc * r_target - sc * z_target
        r_arc = R_mm * (1 - np.cos(alpha))

        if abs(np.sin(alpha)) > 1e-7:
            s_geo = (r_p - r_arc) / np.sin(alpha)
        else:
            z_p = sc * r_target + cc * z_target
            s_geo = z_p

        d2_mm = float(s_geo + L - tool_mm)

        # RETURN EVERYTHING CONVERTED BACK TO METERS
        # [inner_rot, outer_rot, inner_trans_m, outer_trans_m, tool_m]
        return [0.0, theta, d1_mm / 1000.0, d2_mm / 1000.0, 0.0]

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