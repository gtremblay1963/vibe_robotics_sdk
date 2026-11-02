"""Actions de Victor : liste fermee, validation et execution tick par tick dans demo.py.

Une action est un dict, par exemple {'type': 'avancer', 'pas': 5}. Les actions possibles :
  avancer / reculer   pas (1..MAX_PAS) ou metres (0..MAX_METRES)
  cote                direction 'gauche' | 'droite', pas
  tourner             degres (+ = vers la gauche, - = vers la droite), |degres| <= MAX_DEGRES
  bras                cote 'gauche' | 'droit' | 'deux', et un ou plusieurs angles en degres :
                      avant (epaule vers l'avant, 0 = le long du corps, 90 = horizontal devant),
                      ecart (epaule vers l'exterieur, 90 = horizontal sur le cote),
                      coude (0 = tendu, 90 = plie a angle droit), rotation (du bras sur lui-meme ; bras ecarte
                      a 90 et coude plie, rotation 90 = avant-bras vers le haut), pince (0 = fermee, 100 = ouverte, en %),
                      duree en s. Les angles non donnes restent comme ils sont.
  pose                nom d'une pose des bras (poses.json, sans servo de jambe), duree en s
  attendre            secondes
  stop                arrete tout et vide la file
Le code ne fait confiance a personne : chaque action est verifiee avant d'etre mise en file.
"""
import json
import threading
import time
from collections import deque

import numpy as np

from classes import WalkState
import postures_jambes as PJ

MAX_PAS = 20
MAX_METRES = 3.0
MAX_DEGRES = 360.0
MAX_ATTENTE = 30.0
METRES_PAR_PAS = 0.05     # a etalonner : distance reelle / nombre de pas
POSE_DUREE = (0.5, 10.0)  # s
STAND_PAUSE = 0.6         # s debout et immobile entre deux actions
# Securite d'equilibre : bras leves (avant ou ecart au-dela de ce seuil, d'un cote ou de l'autre),
# Victor perd l'equilibre en marchant. Avant tout deplacement (phrase ou manette), il baisse les bras.
BRAS_LEVES_DEG = 20.0
MOUVEMENTS = ('avancer', 'reculer', 'cote', 'tourner')
JAMBES_TENUE = 0.15       # s immobile dans chaque etape d'une posture de jambes avant la suivante
JAMBES_NOMS = {'normal': 'debout', 'pied_gauche': 'pied gauche leve', 'pied_droit': 'pied droit leve',
               'penche_gauche': 'penche a gauche', 'penche_droite': 'penche a droite',
               'accroupi_peu': 'un peu accroupi', 'accroupi': 'accroupi', 'accroupi_bas': 'accroupi bas'}
TURN_CMD = 0.5            # commande de rotation (moitie du stick)
TURN_TOL = 4.0            # deg acceptes a la fin d'une rotation
TURN_TRIES = 4
TURN_SETTLE = 1.0         # s d'immobilite avant de mesurer le cap
LEG_IDS = set(range(7, 13)) | set(range(19, 25))
# Articulations des bras : (indice dans le modele = numero du servo, signe) pour passer de la convention
# « humaine » (avant +, ecart vers l'exterieur +, coude plie +, pince ouverte +) a celle du modele.
# Signes tires du modele 3D (le robot reel suit la meme convention, verifiee sur les jambes).
BRAS = {
    'droit':  {'avant': (1, +1.0), 'ecart': (2, -1.0), 'rotation': (3, +1.0), 'coude': (4, +1.0), 'pince': (6, -1.0)},
    'gauche': {'avant': (13, +1.0), 'ecart': (14, +1.0), 'rotation': (15, +1.0), 'coude': (16, -1.0), 'pince': (18, -1.0)},   # pince gauche inversee depuis son remplacement
}
# Tete (servo 25 = indice 0 du modele) : rotation gauche-droite seulement. L'axe du modele pointe vers le bas,
# donc + dans le modele = vers la droite ; convention « humaine » : + = vers la gauche.
# Le robot tourne la tete de 180 deg de chaque cote (le modele s'arrete a +-57). Le mouvement interpole l'angle :
# il ne fait jamais le tour par l'arriere, le retour se fait toujours en sens inverse (cables).
TETE = (0, -1.0)
TETE_MAX = 175.0          # deg de chaque cote (servo : +-180 au maximum)
# Limites prudentes (degres ; pince en % d'ouverture) : a elargir apres mesure sur le robot.
BRAS_LIMITES = {'avant': (-45.0, 90.0), 'ecart': (0.0, 90.0), 'rotation': (-90.0, 90.0), 'coude': (0.0, 90.0),
                'pince': (0.0, 100.0)}
PINCE_MAX_DEG = 60.0      # ouverture a 100 % (a mesurer)
BRAS_VITESSE = 90.0       # deg/s au plus
# Regle de securite des bras (appliquee quoi que demande l'IA) : pres du corps (avant < PRES_CORPS),
# un bras dont la pince est ouverte reste ecarte d'au moins ECART_PAR_PINCE x ouverture (10 deg a 100 %,
# 5 deg a 50 %, rien si la pince est fermee), sinon la pince accroche la jambe. Ordre des etapes :
# fermer la pince -> s'ecarter si besoin -> bouger le bras -> ouvrir la pince.
PRES_CORPS = 30.0         # deg d'avant en dessous desquels le bras longe la jambe
ECART_PAR_PINCE = 8.0     # deg d'ecart requis pour une pince ouverte a 100 %
ETAPE_MIN = 0.4           # s, duree minimale d'une etape de mouvement de bras
STEP_RAD = 2 * np.pi / 4096


def charger_poses_bras(path, sign_change, sim_of_id):
    """Poses du serveur de poses (pas des servos) -> {nom: {indice sim: radians}}, bras/pinces/tete seulement."""
    try:
        with open(path) as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return {}
    poses = {}
    for name, p in raw.items():
        ids = [int(i) for i in p]
        if not ids or any(i in LEG_IDS for i in ids):
            continue
        poses[name] = {int(sim_of_id[i]): (int(v) - 2048) * STEP_RAD * float(sign_change[sim_of_id[i]])
                       for i, v in ((int(k), v) for k, v in p.items())}
    return poses


def valider(a, poses):
    """Renvoie (action propre, None) ou (None, raison du refus)."""
    if not isinstance(a, dict):
        return None, 'action illisible'
    t = a.get('type')
    try:
        if t in ('avancer', 'reculer', 'cote'):
            if t == 'cote' and a.get('direction') not in ('gauche', 'droite'):
                return None, 'direction du pas de cote manquante'
            if a.get('metres') is not None:
                m = float(a['metres'])
                if not 0 < m <= MAX_METRES:
                    return None, f'distance hors limites (max {MAX_METRES} m)'
                pas = max(1, int(round(m / METRES_PAR_PAS)))
            else:
                pas = int(a.get('pas', 0))
            if not 1 <= pas <= MAX_PAS:
                return None, f'nombre de pas hors limites (1 a {MAX_PAS})'
            out = {'type': t, 'pas': pas}
            if t == 'cote':
                out['direction'] = a['direction']
            return out, None
        if t == 'tourner':
            deg = float(a.get('degres', 0))
            if not 1 <= abs(deg) <= MAX_DEGRES:
                return None, f'angle hors limites (1 a {MAX_DEGRES:.0f} deg)'
            return {'type': t, 'degres': deg}, None
        if t == 'jambes':
            p = a.get('posture')
            if p not in PJ.CHEMINS:
                return None, f'jambes : posture inconnue ({p})'
            return {'type': t, 'posture': p}, None
        if t == 'tete':
            deg = float(a.get('degres', 0))
            if abs(deg) > TETE_MAX:
                return None, f'tete : {deg:.0f} deg hors limites (max {TETE_MAX:.0f} de chaque cote)'
            return {'type': t, 'degres': deg, 'duree': float(np.clip(float(a.get('duree', 1.0)), *POSE_DUREE))}, None
        if t == 'bras':
            cote = a.get('cote')
            if cote not in ('gauche', 'droit', 'deux'):
                return None, 'bras : cote manquant (gauche, droit ou deux)'
            out = {'type': t, 'cote': cote, 'duree': float(np.clip(float(a.get('duree', 1.5)), *POSE_DUREE))}
            for k, (lo, hi) in BRAS_LIMITES.items():
                if a.get(k) is not None:
                    v = float(a[k])
                    if not lo <= v <= hi:
                        return None, f'bras : {k} {v:.0f} hors limites ({lo:.0f} a {hi:.0f})'
                    out[k] = v
            if len(out) == 3:
                return None, 'bras : aucun angle donne'
            return out, None
        if t == 'pose':
            nom = a.get('nom')
            if nom not in poses:
                return None, f'pose inconnue : {nom}'
            d = float(a.get('duree', 1.5))
            return {'type': t, 'nom': nom, 'duree': float(np.clip(d, *POSE_DUREE))}, None
        if t == 'attendre':
            s = float(a.get('secondes', 0))
            if not 0 < s <= MAX_ATTENTE:
                return None, 'attente hors limites'
            return {'type': t, 'secondes': s}, None
        if t == 'stop':
            return {'type': 'stop'}, None
    except (TypeError, ValueError):
        return None, f'valeur invalide pour {t}'
    return None, f'action inconnue : {t}'


def a_faire(a):
    return {'avancer': 'avancer', 'reculer': 'reculer', 'cote': 'aller de cote', 'tourner': 'tourner'}[a['type']]


def decrire(a):
    t = a['type']
    if t in ('avancer', 'reculer'):
        return f"{t} de {a['pas']} pas"
    if t == 'cote':
        return f"{a['pas']} pas de cote vers la {a['direction']}"
    if t == 'tourner':
        return f"tourner de {abs(a['degres']):.0f} deg vers la {'gauche' if a['degres'] > 0 else 'droite'}"
    if t == 'bras':
        quoi = ', '.join(f"{k} {a[k]:.0f}" + ('%' if k == 'pince' else '') for k in BRAS_LIMITES if k in a)
        return f"bras {a['cote']} : {quoi}"
    if t == 'tete':
        d = a['degres']
        return 'tete droit devant' if abs(d) < 0.5 else f"tete a {abs(d):.0f} deg vers la {'gauche' if d > 0 else 'droite'}"
    if t == 'pose':
        return f"pose {a['nom']} ({a['duree']:.1f} s)"
    if t == 'attendre':
        return f"attendre {a['secondes']:.1f} s"
    if t == 'jambes':
        return 'jambes : ' + JAMBES_NOMS[a['posture']]
    return t


class Executeur:
    """File d'actions executee a chaque tick de la boucle de marche. Sans action : la manette commande."""

    def __init__(self, poses, arm_idxs, q0):
        self.poses = poses
        self.arm_idxs = list(arm_idxs)
        self.arm_q = np.array([q0[i] for i in self.arm_idxs], dtype=float)
        self.lock = threading.Lock()
        self.file = deque()
        self.cur = None
        self.journal = deque(maxlen=30)
        self.turn_gain = 15.0          # deg tournes par pas pour une commande de 1 (appris)
        self.ecart_demande = {'droit': 0.0, 'gauche': 0.0}   # ecart voulu, sans l'ecart de securite des pinces
        self._stand_t = 0.0
        # Jambes (lues et ecrites par demo.py) : posture interne visee, posture atteinte, repli rapide en cours
        self.posture_cible = 'normal'
        self.jambes_atteinte = 'normal'
        self.repli_rapide = False
        self._avant_ref = None          # bras : angle « avant » d'avant la posture de jambes, rendu au retour debout

    # --- appele par l'interface (autre fil) ---
    def ajouter(self, actions):
        with self.lock:
            for a in actions:
                if a['type'] == 'stop':
                    self._stop_locked('demande')
                else:
                    self.file.append(a)
        self.log('recu : ' + ', '.join(decrire(a) for a in actions))

    def arreter(self, raison):
        with self.lock:
            had = self.cur is not None or self.file
            self._stop_locked(raison)
        if had:
            self.log(f'ARRET ({raison})')

    def _stop_locked(self, raison):
        self.file.clear()
        self.cur = None

    def log(self, txt):
        self.journal.append(time.strftime('%H:%M:%S ') + txt)
        print('[victor] ' + txt)

    def etat(self):
        with self.lock:
            return {'en_cours': decrire(self.cur['a']) if self.cur else '',
                    'file': [decrire(a) for a in self.file], 'journal': list(self.journal)}

    @property
    def actif(self):
        return self.cur is not None or bool(self.file)

    # --- appele par la boucle de marche ---
    def tick(self, dt, state, yaw):
        """Renvoie la commande de marche [avant, cote, rotation], ou None si rien a faire."""
        standing = state == WalkState.STAND
        self._stand_t = self._stand_t + dt if standing else 0.0
        with self.lock:
            if self.cur is None:
                if not self.file or self._stand_t < STAND_PAUSE:
                    return None if not self.file else np.zeros(3)
                a = self.file.popleft()
                if a['type'] in MOUVEMENTS and self.posture_cible != 'normal':
                    self.file.appendleft(a)                   # le deplacement attend que Victor soit debout
                    a = self._debout()
                    self.log(f'securite : retour debout avant de {a_faire(self.file[0])}')
                elif a['type'] in ('bras', 'pose') and self.posture_cible in PJ.REPLI:
                    self.file.appendleft(a)                   # sur un pied, bouger les bras deplace le poids : pied pose
                    a = self._debout()
                    self.log('securite : pied pose avant de bouger les bras')
                elif (a['type'] in MOUVEMENTS or (a['type'] == 'jambes' and a['posture'].startswith('pied')
                                                   and self.posture_cible == 'normal')) and self.bras_leves():
                    self.file.appendleft(a)                   # bras leves : equilibre compromis, on les baisse d'abord
                    a = self._baisser()
                    self.log('securite : bras baisses avant de ' +
                             (a_faire(self.file[0]) if self.file[0]['type'] in MOUVEMENTS else 'lever un pied'))
                if a['type'] in ('bras', 'pose'):
                    self._avant_ref = None            # nouvelle position des bras voulue : elle sera gardee
                self.cur = {'a': a, 'n': 0, 'prev': state, 't': 0.0, 'phase': 'go'}
                self.log('debut : ' + decrire(a))
            c = self.cur
            a = c['a']
            t = a['type']
            c['t'] += dt
            if t in ('avancer', 'reculer', 'cote'):
                if state == WalkState.SSP and c['prev'] != WalkState.SSP:
                    c['n'] += 1
                c['prev'] = state
                if c['n'] < a['pas']:
                    if t == 'avancer':
                        return np.array([1.0, 0.0, 0.0])
                    if t == 'reculer':
                        return np.array([-1.0, 0.0, 0.0])
                    return np.array([0.0, 1.0 if a['direction'] == 'gauche' else -1.0, 0.0])
                return self._finir_quand_arrete(standing)
            if t == 'attendre':
                return self._finir_quand_arrete(c['t'] >= a['secondes'])
            if t == 'jambes':
                return self._jambes(c, a, dt)
            if t in ('pose', 'bras', 'tete'):
                if 'plan' not in c:
                    c['plan'] = (self._plan_bras(a) if t == 'bras' else
                                 [self._cible_tete(a)] if t == 'tete' else [self._cible_pose(a)])
                    c['k'], c['start'], c['t'] = 0, self.arm_q.copy(), 0.0
                tgt = c['plan'][c['k']]
                big = float(np.degrees(np.abs(tgt - c['start']).max()))
                dur = max(ETAPE_MIN, big / BRAS_VITESSE, a['duree'] if len(c['plan']) == 1 else 0.0)
                s = min(1.0, c['t'] / dur)
                s = 10 * s**3 - 15 * s**4 + 6 * s**5
                self.arm_q = c['start'] + (tgt - c['start']) * s
                if s >= 1.0 and c['k'] + 1 < len(c['plan']):
                    c['k'], c['start'], c['t'] = c['k'] + 1, self.arm_q.copy(), 0.0
                    return np.zeros(3)
                return self._finir_quand_arrete(s >= 1.0)
            if t == 'tourner':
                return self._tourner(c, a, dt, state, standing, yaw)
        return None

    def bras_leves(self):
        e = self.etat_bras()
        return any(abs(e[c]['avant']) > BRAS_LEVES_DEG or e[c]['ecart'] > BRAS_LEVES_DEG for c in e)

    @staticmethod
    def _baisser():
        return {'type': 'bras', 'cote': 'deux', 'duree': 1.5, 'avant': 0.0, 'ecart': 0.0,
                'rotation': 0.0, 'coude': 0.0, 'securite': True}

    @staticmethod
    def _debout():
        return {'type': 'jambes', 'posture': 'normal', 'securite': True}

    def pret_a_marcher(self):
        """Manette : True si on peut marcher. Sinon, remet Victor debout et baisse les bras (annule le reste),
        et renvoie False."""
        with self.lock:
            if (self.cur is not None and self.cur['a'].get('securite')) or (self.file and self.file[0].get('securite')):
                return False                              # retour debout ou descente des bras deja en cours
            debout, bras = self.posture_cible == 'normal', not self.bras_leves()
            if debout and bras:
                return True
            self.file.clear()
            if not debout:
                self.file.append(self._debout())
            if not bras:
                self.file.append(self._baisser())
            self.cur = None
        self.log('securite : ' + ('retour debout' if not debout else 'bras baisses') + ' avant de marcher a la manette')
        return False

    def _jambes(self, c, a, dt):
        """Suit les etapes d'une posture de jambes : demo.py rejoint self.posture_cible et signale l'arrivee
        dans self.jambes_atteinte."""
        if 'etapes' not in c:
            c['etapes'], c['k'], c['tenue'] = PJ.chemin(self.posture_cible, a['posture']), 0, 0.0
            if self._avant_ref is None:
                e = self.etat_bras()
                self._avant_ref = {cote: e[cote]['avant'] for cote in BRAS}
        if c['k'] >= len(c['etapes']):
            if self.posture_cible == 'normal':
                self._avant_ref = None
            return self._finir_quand_arrete(True)
        cible = c['etapes'][c['k']]
        self.posture_cible = cible
        self._bras_equilibre(cible, dt)
        if self.jambes_atteinte == cible and not self.repli_rapide:
            c['tenue'] += dt
            if c['tenue'] >= JAMBES_TENUE:
                c['k'], c['tenue'] = c['k'] + 1, 0.0
        else:
            c['tenue'] = 0.0
        return np.zeros(3)

    @property
    def jambes_actives(self):
        """Vrai pendant un changement de posture de jambes (demo.py va alors plus vite)."""
        c = self.cur
        return c is not None and c['a']['type'] == 'jambes'

    def _bras_equilibre(self, etape, dt):
        """Bras pendant l'equilibre (PJ.BRAS_EQUILIBRE) : bras du cote d'appui ecarte, bras du cote du pied leve
        un peu en arriere ; ailleurs, ecart voulu et angle « avant » d'avant la posture. A BRAS_VITESSE."""
        e = self.etat_bras()
        impose = PJ.BRAS_EQUILIBRE.get(etape, {})
        tgt = self.arm_q
        for cote in BRAS:
            but = dict(e[cote], ecart=self.ecart_demande[cote])
            if self._avant_ref is not None:
                but['avant'] = self._avant_ref[cote]
            but.update(impose.get(cote, {}))
            tgt = self._vers_q(tgt, cote, but)
        pas = np.radians(BRAS_VITESSE) * dt
        self.arm_q = self.arm_q + np.clip(tgt - self.arm_q, -pas, pas)

    def urgence_pied(self, raison):
        """Le robot penche trop sur un pied : on repose tout de suite le pied leve (le poids reste sur l'autre)."""
        with self.lock:
            repli = PJ.REPLI.get(self.posture_cible)
            if repli is None or self.repli_rapide:
                return
            self.file.clear()
            self.cur = None
            self.posture_cible = repli
            self.repli_rapide = True
        self.log(f'URGENCE : pied repose ({raison})')

    def _cible_tete(self, a):
        tgt = self.arm_q.copy()
        if TETE[0] in self.arm_idxs:
            tgt[self.arm_idxs.index(TETE[0])] = TETE[1] * np.radians(a['degres'])
        return tgt

    def _cible_pose(self, a):
        tgt = self.arm_q.copy()
        for k, i in enumerate(self.arm_idxs):
            if i in self.poses[a['nom']]:
                tgt[k] = self.poses[a['nom']][i]
        return tgt

    def etat_bras(self, q=None):
        """Angles « humains » actuels de chaque bras, relus depuis les consignes."""
        q = self.arm_q if q is None else q
        pos = {i: k for k, i in enumerate(self.arm_idxs)}
        out = {}
        for cote, art in BRAS.items():
            e = {}
            for nom, (i, sgn) in art.items():
                deg = float(np.degrees(sgn * q[pos[i]])) if i in pos else 0.0
                e[nom] = deg * 100.0 / PINCE_MAX_DEG if nom == 'pince' else deg
            out[cote] = e
        return out

    def _vers_q(self, q, cote, e):
        q = q.copy()
        pos = {i: k for k, i in enumerate(self.arm_idxs)}
        for nom, (i, sgn) in BRAS[cote].items():
            if i in pos:
                deg = e[nom] * PINCE_MAX_DEG / 100.0 if nom == 'pince' else e[nom]
                q[pos[i]] = sgn * np.radians(deg)
        return q

    def _plan_bras(self, a):
        """Etapes du mouvement (une ou deux) apres application des regles de securite."""
        avant = self.etat_bras()
        par_cote = {}                                # suite d'etats a atteindre, par cote
        for cote in (('droit', 'gauche') if a['cote'] == 'deux' else (a['cote'],)):
            old = dict(avant[cote])
            new = dict(old)
            new.update({k: a[k] for k in BRAS_LIMITES if k in a})
            # ecart final = ecart voulu, ou l'ecart de securite si la pince est ouverte pres du corps ;
            # pince refermee, le bras revient a l'ecart voulu
            dem = a['ecart'] if 'ecart' in a else self.ecart_demande[cote]
            self.ecart_demande[cote] = dem
            req = new['pince'] * ECART_PAR_PINCE / 100.0 if new['avant'] < PRES_CORPS else 0.0
            new['ecart'] = max(dem, req)
            if req > dem + 0.5:
                self.log(f'securite : bras {cote} garde {req:.0f} deg d\'ecart (pince ouverte {new["pince"]:.0f} %)')
            suite = []
            cur = dict(old)
            if new['pince'] < cur['pince']:              # fermeture : d'abord la pince, puis le bras
                cur = dict(cur, pince=new['pince'])
                suite.append(cur)
            # pendant le trajet, la pince garde l'ouverture de cur : s'ecarter d'abord si le trajet longe le corps
            need = cur['pince'] * ECART_PAR_PINCE / 100.0 if min(cur['avant'], new['avant']) < PRES_CORPS else 0.0
            if cur['ecart'] < need - 0.5:
                cur = dict(cur, ecart=need)
                suite.append(cur)
            # coude et rotation : en montant, l'epaule d'abord ; en descendant, le coude et la rotation d'abord
            # (l'avant-bras ne balaie pas le corps ni la jambe)
            bouge_coude = abs(new['coude'] - cur['coude']) > 1.0 or abs(new['rotation'] - cur['rotation']) > 1.0
            if bouge_coude and new['avant'] + new['ecart'] > cur['avant'] + cur['ecart'] + 1.0:
                cur = dict(cur, avant=new['avant'], ecart=new['ecart'])
                suite.append(cur)
            elif bouge_coude and new['avant'] + new['ecart'] < cur['avant'] + cur['ecart'] - 1.0:
                cur = dict(cur, coude=new['coude'], rotation=new['rotation'])
                suite.append(cur)
            cur = dict(cur, **{k: new[k] for k in ('avant', 'ecart', 'rotation', 'coude')})
            suite.append(cur)
            if new['pince'] > cur['pince']:              # ouverture : seulement une fois le bras en place
                cur = dict(cur, pince=new['pince'])
                suite.append(cur)
            par_cote[cote] = suite
        n = max(len(s) for s in par_cote.values())
        plan = []
        for k in range(n):
            q = self.arm_q
            for cote in BRAS:
                e = par_cote[cote][min(k, len(par_cote[cote]) - 1)] if cote in par_cote else avant[cote]
                q = self._vers_q(q, cote, e)
            if not plan or np.abs(q - plan[-1]).max() > 1e-6:
                plan.append(q)
        return plan

    def _finir_quand_arrete(self, done):
        if done:
            self.log('fin : ' + decrire(self.cur['a']))
            self.cur = None
            self._stand_t = 0.0
        return np.zeros(3)

    def _tourner(self, c, a, dt, state, standing, yaw):
        """Rotation sur place mesuree au cap de la centrale (+ = gauche), par essais successifs."""
        if c['phase'] == 'go' and 'target' not in c:
            if yaw is None:                               # sans centrale : estimation seulement
                c['target'] = None
                c['rest'] = a['degres']
            else:
                c['target'] = yaw + a['degres']
            c['tries'] = TURN_TRIES
            c['steps'] = None
        if c['phase'] == 'go':
            if c['steps'] is None:
                if not standing:
                    return np.zeros(3)
                err = c['rest'] if c['target'] is None else c['target'] - yaw   # + = encore a tourner a gauche
                if abs(err) <= TURN_TOL:
                    return self._finir_quand_arrete(True)
                c['cmd'] = float(np.copysign(TURN_CMD, -err))             # commande + = tourne a droite
                c['steps'] = max(1, int(np.ceil(0.9 * abs(err) / (self.turn_gain * TURN_CMD))))
                c['err0'], c['n'], c['prev'] = err, 0, state
            if state == WalkState.SSP and c['prev'] != WalkState.SSP:
                c['n'] += 1
            c['prev'] = state
            if c['n'] < c['steps']:
                return np.array([0.0, 0.0, c['cmd']])
            c['phase'], c['settle'] = 'settle', 0.0
            return np.zeros(3)
        # 'settle' : attendre l'arret, mesurer, apprendre, recommencer si besoin
        c['settle'] = c['settle'] + dt if standing else 0.0
        if c['settle'] < TURN_SETTLE:
            return np.zeros(3)
        if c['target'] is None:                           # sans centrale : un seul essai
            return self._finir_quand_arrete(True)
        err = c['target'] - yaw
        turned = c['err0'] - err
        if c['n'] > 0 and abs(turned) > 1.0:
            self.turn_gain = float(np.clip(abs(turned) / (c['n'] * TURN_CMD), 3.0, 30.0))
        c['tries'] -= 1
        if abs(err) <= TURN_TOL or c['tries'] <= 0:
            if abs(err) > TURN_TOL:
                self.log(f'rotation : ecart restant {err:+.0f} deg')
            return self._finir_quand_arrete(True)
        c['phase'], c['steps'] = 'go', None
        return np.zeros(3)
