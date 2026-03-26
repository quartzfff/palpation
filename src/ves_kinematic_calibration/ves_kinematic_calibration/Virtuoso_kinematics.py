import numpy as np
from scipy.optimize import minimize_scalar


class VirtuosoKinematics:
    def __init__(self, kappa=28.5, tool_len=0.015):
        # All inputs in METERS
        self.kappa = kappa
        self.tool_len = tool_len
        self.collar_len = 0.005
        self.c_m = 0.0003
        # Physical tube radius matching opts.r_mm = 1000/kappa in MATLAB
        self.r_tube_m = 1.0 / kappa

    def solve_ik(self, p_target):
        """
        Input: p_target [x, y, z] in METERS
        Output: [theta1, theta2, d1_m, d2_m, tool_m]
        """
        # 1. Convert to MM for numerical stability (matching MATLAB)
        target_mm = np.array(p_target).flatten() * 1000.0
        kappa_mm = self.kappa / 1000.0
        R_mm = 1.0 / kappa_mm
        collar_mm = self.collar_len * 1000.0
        c_mm = self.c_m * 1000.0
        tool_mm = self.tool_len * 1000.0
        r_mm = self.r_tube_m * 1000.0
        D1_MAX_MM = 20.0

        # 2. Step 1: Solve Azimuth (theta1)
        theta1 = float(np.arctan2(target_mm[0], -target_mm[1]))
        theta2 = theta1

        # 3. Define Internal Cost Function
        def physical_cost_exact(d1):
            # A. Arc Geometry
            L = max(d1 - collar_mm, 0.0)
            a = L / R_mm
            if L > 0:
                r_arc = R_mm * (1.0 - np.cos(a))
                z_arc = R_mm * np.sin(a)
                t_hat = np.array([np.sin(a), np.cos(a)])
            else:
                r_arc, z_arc = 0.0, 0.0
                t_hat = np.array([0.0, 1.0])

            # B. Clearance Angle
            L_in = max(0.0, collar_mm - max(d1, 0.0))
            radicand = max(0.0, L_in ** 2 + 2 * c_mm * r_mm - c_mm ** 2)
            theta_c = 2.0 * np.arctan((L_in - np.sqrt(radicand)) / (c_mm - 2.0 * r_mm))

            # C. Project Target back to Planar Frame
            cz, sz = np.cos(theta1), np.sin(theta1)
            cc, sc = np.cos(theta_c), np.sin(theta_c)

            Rz_inv = np.array([[cz, sz, 0], [-sz, cz, 0], [0, 0, 1]])
            Rx_inv = np.array([[1, 0, 0], [0, cc, sc], [0, -sc, cc]])

            p_planar = Rx_inv @ (Rz_inv @ target_mm)

            x_planar_err = p_planar[0]
            r_t = -p_planar[1]
            z_t = p_planar[2]

            # D. Tangent Projection
            v = np.array([r_t - r_arc, z_t - z_arc])
            s_proj = v[0] * t_hat[0] + v[1] * t_hat[1]

            s_min = collar_mm + tool_mm
            s_geo_final = max(s_proj, s_min)

            # E. Residual & Penalty
            p_calc = np.array([r_arc, z_arc]) + s_geo_final * t_hat
            err_vec = p_calc - np.array([r_t, z_t])

            d2_implied = d1 + s_geo_final - collar_mm - tool_mm
            penalty = 0.0
            if d2_implied < d1:
                penalty = (d1 - d2_implied) ** 2 * 1000 + 100

            return float(np.sum(err_vec ** 2) + x_planar_err ** 2 + penalty)

        # 4. Step 2: 1D Bounded Search
        res = minimize_scalar(
            physical_cost_exact,
            bounds=(0.0, D1_MAX_MM),
            method='bounded',
            options={'xatol': 1e-10}
        )

        d1_star = float(res.x)
        min_cost = float(res.fun)

        # 5. Step 3: Reconstruction
        # Get s_geo_star for the optimal d1
        # To avoid re-writing logic, we temporarily re-calculate s_geo_star
        L_final = max(d1_star - collar_mm, 0.0)
        a_final = L_final / R_mm
        L_in_final = max(0.0, collar_mm - d1_star)
        rad_final = max(0.0, L_in_final ** 2 + 2 * c_mm * r_mm - c_mm ** 2)
        theta_c_final = 2.0 * np.arctan((L_in_final - np.sqrt(rad_final)) / (c_mm - 2.0 * r_mm))

        cz, sz = np.cos(theta1), np.sin(theta1)
        cc, sc = np.cos(theta_c_final), np.sin(theta_c_final)
        p_planar = np.array([[1, 0, 0], [0, cc, sc], [0, -sc, cc]]) @ \
                   (np.array([[cz, sz, 0], [-sz, cz, 0], [0, 0, 1]]) @ target_mm)

        r_t, z_t = -p_planar[1], p_planar[2]
        r_arc = R_mm * (1.0 - np.cos(a_final)) if L_final > 0 else 0.0
        z_arc = R_mm * np.sin(a_final) if L_final > 0 else 0.0
        t_hat = np.array([np.sin(a_final), np.cos(a_final)]) if L_final > 0 else np.array([0.0, 1.0])

        s_geo_star = max((r_t - r_arc) * t_hat[0] + (z_t - z_arc) * t_hat[1], collar_mm + tool_mm)
        d2_star = s_geo_star + d1_star - collar_mm - tool_mm

        # 6. Step 4: Final Validation
        if d2_star < (d1_star - 1e-3) or np.sqrt(min_cost) > 0.5:
            raise ValueError(f"Unreachable Target: Position error {np.sqrt(min_cost):.4f} mm exceeds tolerance.")

        # RETURN EVERYTHING IN METERS [theta1, theta2, d1_m, d2_m, tool_m]
        return [0, theta2, d1_star / 1000.0, d2_star / 1000.0]

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