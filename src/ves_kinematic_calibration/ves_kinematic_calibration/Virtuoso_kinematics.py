import numpy as np
from scipy.optimize import minimize_scalar

class VirtuosoKinematics:
    def __init__(self, kappa=28.5, tool_len=0.015):
        self.kappa = kappa
        self.tool_len = tool_len
        self.collar_len = 0.005 
        self.c_m = 0.0003        
        self.r_tube_m = 1.0 / kappa if kappa > 0 else 1000.0

    def solve_ik(self, p_target):
        # 1. MM for stability
        target_mm = np.array(p_target).flatten() * 1000.0
        kappa_mm = self.kappa / 1000.0
        R_mm = 1.0 / kappa_mm
        collar_mm = self.collar_len * 1000.0
        c_mm = self.c_m * 1000.0
        tool_mm = self.tool_len * 1000.0
        r_mm = self.r_tube_m * 1000.0
        
        D1_MAX_MM = 100.0 

        # 2. Solve Azimuth (FLIPPED CONVENTION)
        # Using (-x, -y) flips the 0 and 3.14 positions.
        theta_base = float(np.arctan2(-target_mm[0], -target_mm[1]))
        
        # Radial distance is invariant to rotation
        r_target_xy = np.sqrt(target_mm[0]**2 + target_mm[1]**2)
        z_target = target_mm[2]

        # 3. Cost Function
        def physical_cost_exact(d1):
            L = max(d1 - collar_mm, 0.0)
            a = L / R_mm
            if L > 0:
                r_arc = R_mm * (1.0 - np.cos(a))
                z_arc = R_mm * np.sin(a)
                t_hat = np.array([np.sin(a), np.cos(a)])
            else:
                r_arc, z_arc = 0.0, 0.0
                t_hat = np.array([0.0, 1.0])

            L_in = max(0.0, collar_mm - d1)
            radicand = max(0.0, L_in**2 + 2*c_mm*r_mm - c_mm**2)
            theta_c = 2.0 * np.arctan((L_in - np.sqrt(radicand)) / (c_mm - 2.0 * r_mm))

            # Simplified 2D Projection
            r_t = r_target_xy * np.cos(theta_c) - z_target * np.sin(theta_c)
            z_t = r_target_xy * np.sin(theta_c) + z_target * np.cos(theta_c)

            v = np.array([r_t - r_arc, z_t - z_arc])
            s_proj = v[0] * t_hat[0] + v[1] * t_hat[1]
            s_geo_final = max(s_proj, collar_mm + tool_mm)

            p_calc = np.array([r_arc, z_arc]) + s_geo_final * t_hat
            err_sq = np.sum((p_calc - np.array([r_t, z_t]))**2)

            d2_implied = d1 + s_geo_final - collar_mm - tool_mm
            penalty = (d1 - d2_implied)**2 * 1000 if d2_implied < d1 else 0.0
            return float(err_sq + penalty)

        # 4. Search
        res = minimize_scalar(physical_cost_exact, bounds=(0.0, D1_MAX_MM), method='bounded')
        d1_star = float(res.x)
        min_err = np.sqrt(res.fun)

        # 5. Reconstruction
        L_f = max(d1_star - collar_mm, 0.0)
        a_f = L_f / R_mm
        L_in_f = max(0.0, collar_mm - d1_star)
        rad_f = max(0.0, L_in_f**2 + 2*c_mm*r_mm - c_mm**2)
        tc_f = 2.0 * np.arctan((L_in_f - np.sqrt(rad_f)) / (c_mm - 2.0 * r_mm))
        
        r_t_f = r_target_xy * np.cos(tc_f) - z_target * np.sin(tc_f)
        z_t_f = r_target_xy * np.sin(tc_f) + z_target * np.cos(tc_f)
        
        r_arc_f = R_mm * (1.0 - np.cos(a_f)) if L_f > 0 else 0.0
        z_arc_f = R_mm * np.sin(a_f) if L_f > 0 else 0.0
        t_hat_f = np.array([np.sin(a_f), np.cos(a_f)]) if L_f > 0 else np.array([0.0, 1.0])
        
        s_geo_star = max((r_t_f - r_arc_f) * t_hat_f[0] + (z_t_f - z_arc_f) * t_hat_f[1], collar_mm + tool_mm)
        d2_star = s_geo_star + d1_star - collar_mm - tool_mm

        if min_err > 0.5:
            raise ValueError(f"Unreachable: Error {min_err:.2f}mm")

        # Return [theta1, theta2, d1_m, d2_m, tool_m]
        return [theta_base, theta_base, d1_star / 1000.0, d2_star / 1000.0, self.tool_len]
    def check_grid_feasibility(self, grid, push_depth, push_dir):
        """Checks every point in grid and logs failures."""
        failures = []
        for i, pt in enumerate(grid):
            # Surface point
            try:
                self.solve_ik(pt[:3])
            except Exception as e:
                failures.append(f"Pt {i} Surface: {e}")

            # Push point
            try:
                p_push = np.array(pt[:3]) + push_depth * np.array(push_dir)
                self.solve_ik(p_push)
            except Exception as e:
                failures.append(f"Pt {i} Push: {e}")
        return failures