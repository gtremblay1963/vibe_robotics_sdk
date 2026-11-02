"""Postures de jambes de Victor a l'arret : pied leve (equilibre sur une jambe) et accroupi.

Le controleur de marche tient les jambes a la pose debout quand le robot est arrete. Ici, on calcule d'autres
poses d'arret, que demo.py rejoint lentement, articulation par articulation (STAND_SLEW).

- Pied leve : calcule au demarrage avec le modele de l'IK de marche (moindres carres sur pinocchio) :
  1. « appui » : les deux pieds au sol, le centre de masse deplace de APPUI_DX vers le pied d'appui ;
  2. « pre »   : centre de masse au-dessus du pied d'appui (PIED_DX), l'autre pied a peine decolle (5 mm) ;
  3. « leve »  : meme centre de masse, pied leve de PIED_H.
  Le retour refait le chemin a l'envers. Verifie en simulation (MuJoCo, modele short_new, le 2026-11-02) :
  le robot tient sur un pied, torse incline d'environ 8 deg sur le cote (prevu par le calcul).
- Accroupi : poses enregistrees avec pose_server (accroupi_1, _2, _3), deja essayees sur le robot.
"""
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

APPUI_DX = 0.05     # m : deplacement du centre de masse, deux pieds au sol (le pied d'appui est a ~0,089 m)
PIED_DX = 0.09      # m : centre de masse au-dessus du pied d'appui, pied leve
PIED_DZ = -0.01     # m : genoux un peu plies pendant l'equilibre
PIED_H = 0.03       # m : le pied leve monte de PIED_H par rapport au bassin, en pliant la jambe (hanche, genou,
                    # cheville) : il decolle meme si la jambe d'appui flechit sous le poids (essai du 2026-11-02 :
                    # en inclinant seulement le bassin, le pied restait au sol)
PRE_H = 0.005       # m : pied a peine decolle (le poids passe sur l'autre pied)
BRAS_ECART = 25.0   # deg : bras du cote du pied d'appui ecarte (le torse penche de ce cote : la main pousserait
                    # sur la jambe)
BRAS_RECUL = -15.0  # deg : bras du cote du pied leve, un peu vers l'arriere (non ecarte) : sinon la main touche le
                    # genou plie (simulation du 2026-11-02)
TORSE_ROULIS = 12.0 # deg : le torse penche vers le pied d'appui (essai du 2026-11-02 : torse reste droit, le bassin
                    # se decale sans s'incliner et le pied ne peut pas decoller). Moitie de l'angle pour « appui ».

# Accroupi : angles des 12 articulations des jambes (deg, convention du modele), tires des poses du robot.
ACCROUPI = {
    'accroupi_1': [-9.5, -0.5, -6.1, 45.1, 34.9, -0.2, 9.5, 0.5, 6.1, 44.9, -34.7, 0.2],
    'accroupi_2': [-15.6, -1.6, -9.8, 69.3, 53.7, -0.5, 15.6, 1.5, 9.8, 69.2, -53.6, 0.4],
    'accroupi_3': [-17.1, -1.8, -10.6, 75.9, 59.2, -0.6, 17.1, 1.8, 10.6, 75.8, -59.1, 0.6],
}

# Chemin entre la pose debout ('normal') et chaque posture demandee (le retour = le chemin a l'envers).
CHEMINS = {
    'normal': [],
    'pied_gauche': ['appui_droit', 'pre_gauche', 'pied_gauche'],
    'pied_droit': ['appui_gauche', 'pre_droit', 'pied_droit'],
    'penche_droite': ['appui_droit'],
    'penche_gauche': ['appui_gauche'],
    'accroupi_peu': ['accroupi_1'],
    'accroupi': ['accroupi_2'],
    'accroupi_bas': ['accroupi_3'],
}
# Bras pendant l'equilibre, par etape : {cote : angles imposes} ; les autres angles restent ceux d'avant.
BRAS_EQUILIBRE = {
    'appui_droit': {'droit': {'ecart': BRAS_ECART}},
    'pre_gauche': {'droit': {'ecart': BRAS_ECART}, 'gauche': {'avant': BRAS_RECUL}},
    'pied_gauche': {'droit': {'ecart': BRAS_ECART}, 'gauche': {'avant': BRAS_RECUL}},
    'appui_gauche': {'gauche': {'ecart': BRAS_ECART}},
    'pre_droit': {'gauche': {'ecart': BRAS_ECART}, 'droit': {'avant': BRAS_RECUL}},
    'pied_droit': {'gauche': {'ecart': BRAS_ECART}, 'droit': {'avant': BRAS_RECUL}},
}
# Pliage de la jambe levee (pied en l'air, sans charge) : peut aller plus vite que les transferts de poids.
PLIAGES = ({'pre_gauche', 'pied_gauche'}, {'pre_droit', 'pied_droit'})
# Pose sure si le robot penche trop sur un pied : le pied leve se repose, le poids reste sur l'autre.
REPLI = {'pre_gauche': 'appui_droit', 'pied_gauche': 'appui_droit',
         'pre_droit': 'appui_gauche', 'pied_droit': 'appui_gauche'}


def chemin(depuis, vers):
    """Etapes pour aller de la posture interne actuelle 'depuis' a la posture demandee 'vers'.
    On passe toujours par la pose debout ('normal'), sauf pour continuer ou reprendre un meme chemin."""
    aller = []
    if depuis != 'normal':
        for p, etapes in CHEMINS.items():
            if depuis in etapes:
                i = etapes.index(depuis)
                if p == vers:
                    return etapes[i + 1:]
                aller = list(reversed(etapes[:i])) + ['normal']
                break
        else:
            aller = ['normal']
    return aller + CHEMINS[vers]


def calculer(robot):
    """Renvoie {nom de posture interne : 12 angles (rad)} pour le robot (instance de Robot / Demo)."""
    m, d = robot.model, robot.robot.data
    t0 = robot._get_targets()
    lf0, rf0 = t0.left_foot_pose.position.copy(), t0.right_foot_pose.position.copy()
    lo, hi = robot.joint_lower, robot.joint_upper
    from robot import Rz, Rx, TORSO_PITCH_TRIM_DEG
    rb_des = Rz(-t0.heading) @ Rx(-np.radians(TORSO_PITCH_TRIM_DEG))

    def q_of(x):
        q = np.zeros(m.nq)
        q[:3] = x[:3]
        q[3:7] = Rot.from_rotvec(x[3:6]).as_quat()
        q[7:] = x[6:]
        return q

    def plier(x_pre, cote, h):
        """A partir de la posture « pre », seule la jambe levee bouge : son pied monte de h (pied a plat)."""
        fid, rf_flat = ((robot._ik_left_fid, robot._R_L_flat) if cote == 'gauche'
                        else (robot._ik_right_fid, robot._R_R_flat))
        j = slice(6 + 6, 6 + 12) if cote == 'gauche' else slice(6, 12)   # 12 articulations : droite puis gauche
        q = q_of(x_pre)
        pin.forwardKinematics(m, d, q)
        pin.updateFramePlacements(m, d)
        cible = d.oMf[fid].translation + np.array([0, 0, h])

        def res(v):
            x = x_pre.copy()
            x[j] = v
            q = q_of(x)
            pin.forwardKinematics(m, d, q)
            pin.updateFramePlacements(m, d)
            ecart = d.oMf[fid].translation - cible
            e = [ecart[0] * 1000, ecart[1] * 100, ecart[2] * 1000]   # le pied peut reculer un peu (jambe pliee)
            e += list(pin.log3(rf_flat @ d.oMf[fid].rotation.T) * 30)   # pied en l'air : a peu pres a plat
            e += list((v - x_pre[j]) * 2)
            return np.array(e)

        jl = slice(j.start - 6, j.stop - 6)
        lo_j, hi_j = lo[jl].copy(), hi[jl].copy()
        lo_j[3] = max(lo_j[3], 0.0)          # genou plie dans le sens naturel (positif, comme pour s'accroupir)
        lo_j[5], hi_j[5] = max(lo_j[5], -np.radians(38)), min(hi_j[5], np.radians(38))   # cheville cote : butee reelle 40
        v0 = x_pre[j].copy()
        v0[3] = 0.3                          # depart genou plie : evite la solution « genou a l'envers »
        s = least_squares(res, np.clip(v0, lo_j + 1e-6, hi_j - 1e-6), bounds=(lo_j, hi_j))
        x = x_pre.copy()
        x[j] = s.x
        return np.clip(x[6:], lo, hi)

    def resoudre(dx, dz=0.0, leve=None, h=0.0, brut=False, roulis=0.0):
        """roulis : inclinaison du torse vers la droite (deg, + = droite) ; None = libre."""
        lf = lf0 + (np.array([0, 0, h]) if leve == 'gauche' else 0)
        rf = rf0 + (np.array([0, 0, h]) if leve == 'droit' else 0)
        com_t = t0.com_pos + np.array([dx, 0, dz])

        def res(x):
            q = q_of(x)
            pin.forwardKinematics(m, d, q)
            pin.updateFramePlacements(m, d)
            pin.centerOfMass(m, d, q)
            e = []
            for fid, p, rf_flat in ((robot._ik_left_fid, lf, robot._R_L_flat), (robot._ik_right_fid, rf, robot._R_R_flat)):
                e += list((d.oMf[fid].translation - p) * 1000)
                e += list(pin.log3(rf_flat @ d.oMf[fid].rotation.T) * 300)
            e += list((d.com[0] - com_t) * 2000)
            err = pin.log3(rb_des @ d.oMf[robot._ik_base_fid].rotation.T)
            e += [err[2] * 300, err[0] * 300]              # cap et tangage du torse tenus
            if roulis is not None:                          # inclinaison laterale du torse imposee
                haut = d.oMf[robot._ik_base_fid].rotation[:, 2]
                e += [(haut[0] - np.sin(np.radians(roulis))) * 1000]
            e += list((x[6:] - robot.default_q[7:]) * 2)
            return np.array(e)

        x0 = np.r_[robot.default_q[:3], Rot.from_quat(robot.default_q[3:7]).as_rotvec(), robot.default_q[7:]]
        lb, ub = np.r_[[-1] * 6, lo], np.r_[[1] * 6, hi]
        for k in (6 + 3, 6 + 9):                 # genoux : pliés dans le sens naturel seulement (pas « a l'envers »)
            lb[k] = max(lb[k], np.radians(-3))
        s = least_squares(res, np.clip(x0, lb + 1e-6, ub - 1e-6), bounds=(lb, ub))
        return s.x if brut else np.clip(s.x[6:], lo, hi)

    out = {'normal': robot.default_q[7:].copy()}
    for cote, s in (('gauche', +1.0), ('droit', -1.0)):      # x > 0 = cote droit : pied gauche leve -> poids a droite
        appui = 'appui_droit' if cote == 'gauche' else 'appui_gauche'
        out[appui] = resoudre(s * APPUI_DX, roulis=s * TORSE_ROULIS / 2)
        x_pre = resoudre(s * PIED_DX, PIED_DZ, cote, PRE_H, brut=True, roulis=s * TORSE_ROULIS)
        out['pre_' + cote] = np.clip(x_pre[6:], lo, hi)
        out['pied_' + cote] = plier(x_pre, cote, PIED_H)
    for nom, deg in ACCROUPI.items():
        out[nom] = np.radians(deg)
    return out
