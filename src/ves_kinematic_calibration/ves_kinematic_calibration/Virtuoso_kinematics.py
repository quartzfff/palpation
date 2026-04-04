import numpy as np
from scipy.optimize import minimize_scalar

class VirtuosoKinematics:
    def __init__(self, kappa_ik=28.5, kappa_fk=28.5, tool_len=0.015):
        # Separation of IK and FK curvatures
        self.kappa_ik = kappa_ik        # Curvature used for solving IK
        self.kappa_fk = kappa_fk        # Curvature used for Forward Kinematics
        
        self.tool_len = tool_len        # m (Default tool length)
        self.collar_len = 0.005         # m
        self.c_m = 0.0003               # m (clearance)
        
        # Tube radii based on respective curvatures
        self.r_tube_ik = 1.0 / kappa_ik if kappa_ik > 0 else 1000.0
        self.r_tube_fk = 1.0 / kappa_fk if kappa_fk > 0 else 1000.0

    def solve_fk(self, q, manual_tool_len=None):
        """
        Forward Kinematics using the FK-specific curvature.
        :param q: [theta1, theta2, d1, d2, (optional_tool)]
        :param manual_tool_len: If provided, overrides self.tool_len
        """
        d1 = q[2] * 1000.0
        d2 = q[3] * 1000.0
        theta1 = q[0]
        theta2 = q[1]
        
        # Use FK-specific parameters
        kappa_mm = self.kappa_fk / 1000.0
        R = 1.0 / kappa_mm if kappa_mm > 0 else 1e6
        r_mm = self.r_tube_fk * 1000.0
        
        # Determine tool length to use
        t_val = manual_tool_len if manual_tool_len is not None else self.tool_len
        t = t_val * 1000.0
        
        collar = self.collar_len * 1000.0
        c_mm = self.c_m * 1000.0

        # --- Step 1: Physical Arc Geometry ---
        L = max(d1 - collar, 0.0)
        a = L / R
        
        if L > 0:
            r_arc = R * (1.0 - np.cos(a))
            z_arc = R * np.sin(a)
            t_hat = np.array([np.sin(a), np.cos(a)])
        else:
            r_arc, z_arc = 0.0, 0.0
            t_hat = np.array([0.0, 1.0])

        d2_eff = max(d2, d1)
        s_geo = (d2_eff - d1) + collar + t

        r_nom = r_arc + s_geo * t_hat[0]
        z_nom = z_arc + s_geo * t_hat[1]

        p_nom = np.array([r_nom * np.sin(theta1), -r_nom * np.cos(theta1), z_nom])

        # --- Step 2: Clearance Angle (Tilt) ---
        L_inside = max(0.0, collar - max(d1, 0.0))
        radicand = max(0.0, L_inside**2 + 2*c_mm*r_mm - c_mm**2)
        theta_c = 2.0 * np.arctan((L_inside - np.sqrt(radicand)) / (c_mm - 2.0 * r_mm))

        # --- Step 3: Rotation Matrices ---
        cz, sz = np.cos(theta1), np.sin(theta1)
        ca, sa = np.cos(a), np.sin(a)
        cc, sc = np.cos(theta_c), np.sin(theta_c)
        cr, sr = np.cos(theta2 - theta1), np.sin(theta2 - theta1)

        Rz1 = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        Rx_c = np.array([[1, 0, 0], [0, cc, -sc], [0, sc, cc]])
        Rx_bend = np.array([[1, 0, 0], [0, ca, sa], [0, -sa, ca]])
        Rz_rel = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])

        R_tip = Rz1 @ Rx_c @ Rx_bend @ Rz_rel
        p_world = Rz1 @ (Rx_c @ (Rz1.T @ p_nom))

        return p_world / 1000.0, R_tip

    def solve_ik(self, p_target, manual_tool_len=None):
        """
        Inverse Kinematics using the IK-specific curvature.
        """
        target_mm = np.array(p_target).flatten() * 1000.0
        
        # Use IK-specific parameters
        kappa_mm = self.kappa_ik / 1000.0
        R_mm = 1.0 / kappa_mm
        r_mm = self.r_tube_ik * 1000.0
        
        t_val = manual_tool_len if manual_tool_len is not None else self.tool_len
        tool_mm = t_val * 1000.0
        
        collar_mm = self.collar_len * 1000.0
        c_mm = self.c_m * 1000.0
        D1_MAX_MM = 100.0 

        theta_base = float(np.arctan2(-target_mm[0], -target_mm[1]))
        r_target_xy = np.sqrt(target_mm[0]**2 + target_mm[1]**2)
        z_target = target_mm[2]

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

        res = minimize_scalar(physical_cost_exact, bounds=(0.0, D1_MAX_MM), method='bounded')
        d1_star = float(res.x)
        min_err = np.sqrt(res.fun)

        if min_err > 0.5:
            raise ValueError(f"Unreachable: Error {min_err:.2f}mm")

        # Re-calculating d2 for return
        L_f = max(d1_star - collar_mm, 0.0)
        a_f = L_f / R_mm
        L_in_f = max(0.0, collar_mm - d1_star)
        rad_f = max(0.0, L_in_f**2 + 2*c_mm*r_mm - c_mm**2)
        tc_f = 2.0 * np.arctan((L_in_f - np.sqrt(rad_f)) / (c_mm - 2.0 * r_mm))
        r_t_f = r_target_xy * np.cos(tc_f) - z_target * np.sin(tc_f)
        z_t_f = r_target_xy * np.sin(tc_f) + z_target * np.cos(tc_f)
        t_hat_f = np.array([np.sin(a_f), np.cos(a_f)]) if L_f > 0 else np.array([0.0, 1.0])
        r_arc_f = R_mm * (1.0 - np.cos(a_f)) if L_f > 0 else 0.0
        z_arc_f = R_mm * np.sin(a_f) if L_f > 0 else 0.0
        
        s_geo_star = max((r_t_f - r_arc_f) * t_hat_f[0] + (z_t_f - z_arc_f) * t_hat_f[1], collar_mm + tool_mm)
        d2_star = s_geo_star + d1_star - collar_mm - tool_mm

        return [theta_base, theta_base, d1_star / 1000.0, d2_star / 1000.0, t_val]