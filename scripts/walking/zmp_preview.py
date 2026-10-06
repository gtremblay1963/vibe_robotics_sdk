"""Planificateur de COM par suivi de ZMP (preview control, Kajita/Wieber).

Remplace les deux QP contraints de fsm.WalkingFSM.update_mpc par un moindres
carres sans contrainte : toujours soluble, gain precalcule une seule fois.
"""
import copy
import numpy as np
from classes import WalkState, FootType
from fsm import WalkingFSM


class ZmpPreview:
    def __init__(self, h, dt, horizon, r_jerk=1e-6, g=9.81):
        n = int(round(horizon / dt))
        A = np.array([[1., dt, dt * dt / 2.], [0., 1., dt], [0., 0., 1.]])
        B = np.array([dt ** 3 / 6., dt * dt / 2., dt])
        c = np.array([1., 0., -h / g])
        Px = np.zeros((n, 3))
        Pu = np.zeros((n, n))
        Ak = np.eye(3)
        cols = []                      # cols[j] = A^j B
        for k in range(n):
            cols.append(Ak @ B)
            Ak = A @ Ak                # A^(k+1)
            Px[k] = c @ Ak
            for j in range(k + 1):
                Pu[k, j] = c @ cols[k - j]
        K = np.linalg.solve(Pu.T @ Pu + r_jerk * np.eye(n), Pu.T)
        self.k0 = K[0]                 # premiere commande seulement
        self.Px = Px
        self.n = n
        self.dt = dt

    def jerk(self, x0, zref):
        return float(self.k0 @ (zref - self.Px @ x0))


class WalkingFSMPreview(WalkingFSM):
    horizon = 1.5          # s
    r_jerk = 1e-6
    stop_time = 0.3        # s, retour du ZMP au centre a l'arret
    zmp_inset = 0.0        # m, ZMP de reference rentre vers l'interieur du pied
    steering_deg = 15.0    # degres de rotation par pas a fond de stick

    def reset_standing(self):
        super().reset_standing()
        self.footstep_generator.steering_strength = np.deg2rad(self.steering_deg)
        h = float(self.stance.com.position[2])
        self.preview = ZmpPreview(h, self.dt, self.horizon, self.r_jerk)
        self.zmp_now = self._mid()
        self.dsp_from = self._mid()
        self.dsp_total = self.dsp_duration
        self.zmp_ref_xy = self.zmp_now.copy()

    # --- utilitaires -------------------------------------------------
    def _mid(self):
        return 0.5 * (self.stance.left_foot.position[:2] + self.stance.right_foot.position[:2])

    def _zmp_of(self, pos_xy, side):
        if self.zmp_inset == 0.0:
            return np.asarray(pos_xy, dtype=float).copy()
        th = self.footstep_generator.ref_theta
        right = np.array([np.cos(th), -np.sin(th)])
        s = -1.0 if side == FootType.RIGHT else 1.0     # vers l'interieur
        return np.asarray(pos_xy, dtype=float) + s * self.zmp_inset * right

    def _segments(self):
        """Liste (duree, debut, fin) du ZMP de reference a partir de maintenant."""
        segs = []
        if self.state == WalkState.STAND or self.stance_foot is None:
            m = self._mid()
            return [(1e3, m, m)]
        st = self._zmp_of(self.stance_foot.position[:2], self.stance_foot.side)
        if self.state == WalkState.DSP:
            r = max(self.rem_time, 0.0)
            segs.append((r, self.zmp_now.copy(), st))
            segs.append((self.ssp_duration, st, st))
        else:
            segs.append((max(self.rem_time, 0.0), st, st))
        tgt_side = self.swing_foot.side
        tgt = self._zmp_of(self.swing_target.position[:2], tgt_side)
        if np.linalg.norm(self.cmd) < 0.01:
            m = 0.5 * (self.stance_foot.position[:2] + self.swing_target.position[:2])
            segs.append((self.stop_time, st, m))
            segs.append((1e3, m, m))
            return segs
        gen = copy.copy(self.footstep_generator)
        cur, cur_side = st, self.stance_foot.side
        nxt, nxt_side = tgt, tgt_side
        t = sum(s[0] for s in segs)
        while t < self.horizon + self.dt:
            segs.append((self.dsp_duration, cur, nxt))
            segs.append((self.ssp_duration, nxt, nxt))
            t += self.dsp_duration + self.ssp_duration
            new = gen.get_next_footstep(self.cmd, nxt_side)       # pied oppose a nxt
            cur, cur_side = nxt, nxt_side
            nxt_side = FootType.LEFT if cur_side == FootType.RIGHT else FootType.RIGHT
            nxt = self._zmp_of(new.position[:2], nxt_side)
        return segs

    def _zref(self):
        segs = self._segments()
        n, dt = self.preview.n, self.dt
        out = np.zeros((n, 2))
        i, t0 = 0, 0.0
        for k in range(n):
            t = (k + 1) * dt
            while i < len(segs) - 1 and t > t0 + segs[i][0]:
                t0 += segs[i][0]
                i += 1
            d, a, b = segs[i]
            s = 1.0 if d <= 1e-9 else min(max((t - t0) / d, 0.0), 1.0)
            out[k] = a + s * (b - a)
        return out

    # --- remplace le MPC --------------------------------------------
    def update_mpc(self, dsp_duration, ssp_duration):
        self.preview_time = 0.0

    def run_com_mpc(self):
        zref = self._zref()
        com = self.stance.com
        j = np.zeros(3)
        for ax in (0, 1):
            x0 = np.array([com.position[ax], com.velocity[ax], com.acceleration[ax]])
            j[ax] = self.preview.jerk(x0, zref[:, ax])
        com.integrate_constant_jerk(j, self.dt)
        self.zmp_now = zref[0].copy()
        self.zmp_ref_xy = zref[0].copy()
        self.com_xy = com.position[:2].copy()
        self.goal_xy = zref[0].copy()

    def run_standing(self):
        if self.start_walking:
            self.start_walking = False
            return self.start_double_support()
        self.run_com_mpc()
