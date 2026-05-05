import numpy as np
from scipy.optimize import minimize_scalar


class VirtuosoKinematics:
    def __init__(self, kappa_ik=28.5, kappa_fk=28.5, tool_len=0.015):
        self.kappa_ik = kappa_ik
        self.kappa_fk = kappa_fk

        self.tool_len = tool_len        # m
        self.collar_len = 0.005         # m, first 5 mm straight outer tube
        self.c_m = 0.0003               # m, clearance

        self.r_tube_ik = 1.0 / kappa_ik if kappa_ik > 0 else 1000.0
        self.r_tube_fk = 1.0 / kappa_fk if kappa_fk > 0 else 1000.0

    def _outer_geometry_mm(self, d1_mm, R_mm, collar_mm):
        """
        Outer tube geometry:
        d1 = 0–5 mm: straight
        d1 > 5 mm: first 5 mm straight, then curved
        """
        L_curve = max(d1_mm - collar_mm, 0.0)
        a = L_curve / R_mm

        if L_curve > 0:
            r_arc = R_mm * (1.0 - np.cos(a))
            z_arc = collar_mm + R_mm * np.sin(a)
            t_hat = np.array([np.sin(a), np.cos(a)])
        else:
            r_arc = 0.0
            z_arc = d1_mm
            t_hat = np.array([0.0, 1.0])

        return r_arc, z_arc, t_hat, a

    def solve_fk(self, q, manual_tool_len=None):
        """
        Forward Kinematics.
        q = [theta1, theta2, d1, d2]
        d1, d2 in meters.
        """

        theta1 = q[0]
        theta2 = q[1]
        d1 = q[2] * 1000.0
        d2 = q[3] * 1000.0

        kappa_mm = self.kappa_fk / 1000.0
        R_mm = 1.0 / kappa_mm if kappa_mm > 0 else 1e6
        r_mm = self.r_tube_fk * 1000.0

        tool_m = manual_tool_len if manual_tool_len is not None else self.tool_len
        tool_mm = tool_m * 1000.0

        collar_mm = self.collar_len * 1000.0
        c_mm = self.c_m * 1000.0

        # --- Outer tube geometry ---
        r_arc, z_arc, t_hat, a = self._outer_geometry_mm(d1, R_mm, collar_mm)

        # --- Inner tube extension ---
        # Inner tube is straight.
        # d2 is total inner tube insertion from base.
        # Extension beyond outer tube tip = d2 - d1.
        d2_eff = max(d2, d1)
        s_geo = (d2_eff - d1) + tool_mm

        r_nom = r_arc + s_geo * t_hat[0]
        z_nom = z_arc + s_geo * t_hat[1]

        p_nom = np.array([
            r_nom * np.sin(theta1),
            -r_nom * np.cos(theta1),
            z_nom
        ])

        # --- Clearance angle ---
        L_inside = max(0.0, collar_mm - max(d1, 0.0))
        radicand = max(0.0, L_inside**2 + 2.0 * c_mm * r_mm - c_mm**2)

        if abs(c_mm - 2.0 * r_mm) < 1e-12:
            theta_c = 0.0
        else:
            theta_c = 2.0 * np.arctan(
                (L_inside - np.sqrt(radicand)) / (c_mm - 2.0 * r_mm)
            )

        # --- Rotation matrices ---
        cz, sz = np.cos(theta1), np.sin(theta1)
        ca, sa = np.cos(a), np.sin(a)
        cc, sc = np.cos(theta_c), np.sin(theta_c)
        cr, sr = np.cos(theta2 - theta1), np.sin(theta2 - theta1)

        Rz1 = np.array([
            [cz, -sz, 0.0],
            [sz,  cz, 0.0],
            [0.0, 0.0, 1.0]
        ])

        Rx_c = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cc, -sc],
            [0.0, sc,  cc]
        ])

        Rx_bend = np.array([
            [1.0, 0.0, 0.0],
            [0.0, ca,  sa],
            [0.0, -sa, ca]
        ])

        Rz_rel = np.array([
            [cr, -sr, 0.0],
            [sr,  cr, 0.0],
            [0.0, 0.0, 1.0]
        ])

        R_tip = Rz1 @ Rx_c @ Rx_bend @ Rz_rel
        p_world = Rz1 @ (Rx_c @ (Rz1.T @ p_nom))

        return p_world / 1000.0, R_tip

    def solve_ik(self, p_target, manual_tool_len=None):
        """
        Inverse Kinematics.
        Returns [theta1, theta2, d1, d2, tool_len]
        d1, d2 in meters.
        """

        target_mm = np.array(p_target).flatten() * 1000.0

        kappa_mm = self.kappa_ik / 1000.0
        R_mm = 1.0 / kappa_mm if kappa_mm > 0 else 1e6
        r_mm = self.r_tube_ik * 1000.0

        tool_m = manual_tool_len if manual_tool_len is not None else self.tool_len
        tool_mm = tool_m * 1000.0

        collar_mm = self.collar_len * 1000.0
        c_mm = self.c_m * 1000.0

        D1_MAX_MM = 20.0
        D2_MAX_MM = 40.0

        theta_base = float(np.arctan2(target_mm[0], -target_mm[1]))
        r_target_xy = np.sqrt(target_mm[0]**2 + target_mm[1]**2)
        z_target = target_mm[2]

        def physical_cost_exact(d1):
            r_arc, z_arc, t_hat, a = self._outer_geometry_mm(
                d1, R_mm, collar_mm
            )

            # Clearance inverse rotation
            L_inside = max(0.0, collar_mm - d1)
            radicand = max(0.0, L_inside**2 + 2.0 * c_mm * r_mm - c_mm**2)

            if abs(c_mm - 2.0 * r_mm) < 1e-12:
                theta_c = 0.0
            else:
                theta_c = 2.0 * np.arctan(
                    (L_inside - np.sqrt(radicand)) / (c_mm - 2.0 * r_mm)
                )

            r_t = r_target_xy * np.cos(theta_c) - z_target * np.sin(theta_c)
            z_t = r_target_xy * np.sin(theta_c) + z_target * np.cos(theta_c)

            v = np.array([r_t - r_arc, z_t - z_arc])
            s_proj = v[0] * t_hat[0] + v[1] * t_hat[1]

            # Important correction:
            # no + collar_mm here.
            s_geo = max(s_proj, tool_mm)

            p_calc = np.array([r_arc, z_arc]) + s_geo * t_hat
            err_sq = np.sum((p_calc - np.array([r_t, z_t]))**2)

            # d2 = d1 + extension beyond outer tube
            d2_implied = d1 + s_geo - tool_mm

            penalty = 0.0

            if d2_implied < d1:
                penalty += 1000.0 * (d1 - d2_implied)**2

            if d2_implied < 0.0:
                penalty += 1000.0 * d2_implied**2

            if d2_implied > D2_MAX_MM:
                penalty += 1000.0 * (d2_implied - D2_MAX_MM)**2

            return float(err_sq + penalty)

        res = minimize_scalar(
            physical_cost_exact,
            bounds=(0.0, D1_MAX_MM),
            method='bounded'
        )

        d1_star = float(res.x)
        min_err = np.sqrt(res.fun)

        if min_err > 0.5:
            raise ValueError(f"Unreachable: Error {min_err:.2f} mm")

        # Recompute d2
        r_arc, z_arc, t_hat, a = self._outer_geometry_mm(
            d1_star, R_mm, collar_mm
        )

        L_inside = max(0.0, collar_mm - d1_star)
        radicand = max(0.0, L_inside**2 + 2.0 * c_mm * r_mm - c_mm**2)

        if abs(c_mm - 2.0 * r_mm) < 1e-12:
            theta_c = 0.0
        else:
            theta_c = 2.0 * np.arctan(
                (L_inside - np.sqrt(radicand)) / (c_mm - 2.0 * r_mm)
            )

        r_t = r_target_xy * np.cos(theta_c) - z_target * np.sin(theta_c)
        z_t = r_target_xy * np.sin(theta_c) + z_target * np.cos(theta_c)

        s_geo = max(
            (r_t - r_arc) * t_hat[0] + (z_t - z_arc) * t_hat[1],
            tool_mm
        )

        d2_star = d1_star + s_geo - tool_mm
        d2_star = np.clip(d2_star, d1_star, D2_MAX_MM)

        return [
            theta_base,
            0.0,
            d1_star / 1000.0,
            d2_star / 1000.0,
            tool_m
        ]