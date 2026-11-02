"""Serveur web de poses, a lancer sur le Raspberry Pi (demo.py et run_receiver*.py arretes).

    python pose_server.py            puis ouvrir http://192.168.1.25:8080 (portable ou telephone)
    python pose_server.py --simule   sans robot, pour essayer l'interface (sur n'importe quel ordinateur)

Enregistre des poses a la main (servos relaches puis tenus), compose des sequences et les rejoue
en douceur. La manette branchee a l'ordinateur qui affiche la page est lue par le navigateur.
Les poses sont dans poses.json et les sequences dans sequences.json, a cote de ce script.
L'arret fige le robot ou il est : il ne coupe jamais le couple.
"""
import argparse
import json
import os
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
POSE_FILE = os.path.join(HERE, 'poses.json')
SEQ_FILE = os.path.join(HERE, 'sequences.json')
PAGE_FILE = os.path.join(HERE, 'pose_web.html')

LEGS = [7, 8, 9, 10, 11, 12, 19, 20, 21, 22, 23, 24]
ARMS = [1, 2, 3, 4, 5, 6, 13, 14, 15, 16, 17, 18]
LEG_PAIRS = [(7, 19, 'Hanche flexion'), (8, 20, 'Hanche ecartement'), (9, 21, 'Hanche rotation'),
             (10, 22, 'Genou'), (11, 23, 'Cheville avant-arriere'), (12, 24, 'Cheville cote')]
STEP_DEG = 360.0 / 4096
MAX_SPEED_DEG = 90.0      # vitesse max d'un servo pendant un mouvement (deg/s)
MIN_DURATION = 0.3        # s
RATE = 50.0               # Hz


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_json(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=1, ensure_ascii=False)
    os.replace(tmp, path)


def clean_name(s):
    s = str(s or '').strip()
    return s if re.fullmatch(r'[\w\- ]{1,40}', s) else None


def group_steps(steps):
    """[[pose, duree, en_meme_temps], ...] -> [((pose, ...), duree), ...]
    Une etape marquee en_meme_temps se joue avec la precedente ; le groupe prend la plus longue duree."""
    groups = []
    for st in steps:
        name, d = st[0], float(st[1])
        together = bool(st[2]) if len(st) > 2 else False
        if together and groups:
            names, d0 = groups[-1]
            groups[-1] = (names + (name,), max(d0, d))
        else:
            groups.append(((name,), d))
    return groups


def part_name(pose):
    ids = {int(i) for i in pose}
    if ids == set(LEGS):
        return 'jambes'
    if ids == set(ARMS):
        return 'bras'
    return 'tout' if len(ids) >= len(LEGS) + len(ARMS) else f'{len(ids)} servos'


class FakeManager:
    """Robot simule pour --simule : les servos rejoignent instantanement leur consigne."""
    def __init__(self):
        self.ids = sorted(LEGS + ARMS + [25])
        self.motor_order = np.zeros(26, dtype=np.int32)
        for k, i in enumerate(self.ids):
            self.motor_order[i] = k
        self.sign_change = np.ones(len(self.ids))
        self.q = np.full(len(self.ids), 2048.0)
        self.relaxed = set()

    def get_raw_state(self):
        return self.q.copy(), np.zeros_like(self.q)

    def set_raw_positions(self, q, v, a):
        for i in self.ids:
            if i not in self.relaxed:
                self.q[self.motor_order[i]] = q[self.motor_order[i]]

    def disable_torque(self, ids):
        self.relaxed.update(ids)

    def enable_torque(self, i):
        self.relaxed.discard(i)


class Robot:
    def __init__(self, mgr, enable_torque, imu):
        self.mgr = mgr
        self.enable_torque = enable_torque
        self.imu = imu
        self.order = mgr.motor_order
        self.all_ids = [i for i in sorted(LEGS + ARMS + [25])]
        self.lock = threading.RLock()          # acces au bus des servos
        self.stop_evt = threading.Event()
        self.worker = None
        self.relaxed = set()
        self.action = ''                       # mouvement en cours, pour l'affichage
        self.msg = 'pret'
        self.poses = load_json(POSE_FILE)
        self.sequences = load_json(SEQ_FILE)
        self.goal = self.read()
        self.send(self.goal)

    # --- bus des servos ---
    def read(self):
        with self.lock:
            q, _ = self.mgr.get_raw_state()
        return np.round(q).astype(np.int32)

    def send(self, g):
        with self.lock:
            self.mgr.set_raw_positions(g.copy(), 0, 50)

    def hold_here(self):
        with self.lock:
            self.goal = self.read()
            self.send(self.goal)
            for i in sorted(self.relaxed):
                self.enable_torque(i)
            self.relaxed.clear()

    @property
    def moving(self):
        return self.worker is not None and self.worker.is_alive()

    # --- actions ---
    def relax(self, ids):
        if self.moving:
            return False, 'mouvement en cours'
        with self.lock:
            self.mgr.disable_torque(ids)
            self.relaxed.update(ids)
        return True, f'relache : {len(ids)} servos'

    def hold(self):
        if self.moving:
            return False, 'mouvement en cours'
        self.hold_here()
        return True, 'tous les servos sont tenus'

    def stop(self):
        self.stop_evt.set()
        if self.worker is not None:
            self.worker.join(timeout=1.0)
        if not self.relaxed:
            self.hold_here()
        self.msg = 'ARRET : robot fige'
        return True, self.msg

    def pose_array(self, names):
        """Cible : position actuelle, remplacee par les servos de chaque pose (la derniere l'emporte)."""
        g = self.read()
        for name in names:
            for i, v in self.poses[name].items():
                g[self.order[int(i)]] = int(v)
        return g

    def _move(self, target, duration):
        self.hold_here()
        start = self.goal.astype(float)
        target = target.astype(float)
        duration = max(duration, MIN_DURATION,
                       np.abs(target - start).max() * STEP_DEG / MAX_SPEED_DEG)
        n = max(1, int(duration * RATE))
        t0 = time.time()
        for k in range(1, n + 1):
            if self.stop_evt.is_set():
                return False
            s = k / n
            s = 10 * s**3 - 15 * s**4 + 6 * s**5
            self.goal = np.round(start + (target - start) * s).astype(np.int32)
            self.send(self.goal)
            time.sleep(max(0.0, t0 + k / RATE - time.time()))
        return True

    def _run(self, steps, label):
        try:
            for names, d in steps:
                self.action = f'{label} : {" + ".join(names)} ({d:.1f} s)'
                if not self._move(self.pose_array(names), d):
                    self.msg = 'ARRET : robot fige'
                    return
            self.msg = f'{label} termine'
        except Exception as e:                  # le robot reste tenu a sa derniere consigne
            self.msg = f'erreur : {e}'
        finally:
            self.action = ''

    def start(self, steps, label):
        if self.moving:
            return False, 'mouvement deja en cours'
        missing = [n for names, _ in steps for n in names if n not in self.poses]
        if missing:
            return False, f'pose inconnue : {missing[0]}'
        self.stop_evt.clear()
        self.worker = threading.Thread(target=self._run, args=(steps, label), daemon=True)
        self.worker.start()
        return True, f'{label} demarre'

    def record(self, name, ids=None):
        """Enregistre la position actuelle de ids (tout le corps par defaut). Une pose partielle
        ne bouge que ses servos : les autres restent ou ils sont quand on la joue."""
        ids = ids or self.all_ids
        if not name:
            k = 1
            while f'pose_{k:02d}' in self.poses:
                k += 1
            name = f'pose_{k:02d}'
        q = self.read()
        self.poses[name] = {str(i): int(q[self.order[i]]) for i in ids}
        save_json(POSE_FILE, self.poses)
        part = '' if len(ids) == len(self.all_ids) else f' ({len(ids)} servos)'
        return True, f'pose "{name}" enregistree{part}'

    def state(self):
        q = self.read()
        sc = getattr(self.mgr, 'sign_change', np.ones(len(q)))
        deg = lambda i: round(float((q[self.order[i]] - 2048) * STEP_DEG * sc[self.order[i]]), 1)
        s = self.imu.get(max_age=0.5) if self.imu is not None else None
        return {
            'relaxed': sorted(self.relaxed),
            'moving': self.moving,
            'action': self.action,
            'msg': self.msg,
            'imu': None if s is None else {'tangage': round(s[0], 1), 'roulis': round(s[1], 1)},
            'legs': [[n, deg(d), deg(g)] for d, g, n in LEG_PAIRS],
            'poses': sorted(self.poses),
            'pose_parts': {n: part_name(p) for n, p in self.poses.items()},
            'sequences': self.sequences,
        }


def make_handler(robot):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype + '; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                with open(PAGE_FILE, 'rb') as f:
                    self.reply(200, f.read(), 'text/html')
            elif self.path == '/api/state':
                self.reply(200, robot.state())
            else:
                self.reply(404, {'ok': False, 'msg': 'introuvable'})

        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0) or 0)
            try:
                p = json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                p = {}
            try:
                ok, msg = self.dispatch(self.path, p)
            except Exception as e:
                ok, msg = False, f'erreur : {e}'
            if ok and self.path != '/api/stop':
                robot.msg = msg
            self.reply(200, {'ok': ok, 'msg': msg})

        def dispatch(self, path, p):
            if path == '/api/stop':
                return robot.stop()
            if path == '/api/relax':
                grp = p.get('group')
                ids = {'jambes': LEGS, 'bras': ARMS, 'tout': robot.all_ids}.get(grp)
                return robot.relax(ids) if ids else (False, 'groupe inconnu')
            if path == '/api/hold':
                return robot.hold()
            if path == '/api/record':
                name = p.get('name') or ''
                if name and not clean_name(name):
                    return False, 'nom invalide (lettres, chiffres, espace, - et _ ; 40 max)'
                ids = {'tout': None, 'jambes': LEGS, 'bras': ARMS}.get(p.get('group', 'tout'), None)
                return robot.record(clean_name(name) if name else '', ids)
            if path == '/api/update':
                name = p.get('name')
                if name not in robot.poses:
                    return False, 'pose inconnue'
                ok, _ = robot.record(name, [int(i) for i in robot.poses[name]])
                return ok, f'pose "{name}" mise a jour ({part_name(robot.poses[name])})'
            if path == '/api/rename':
                old, new = p.get('old'), clean_name(p.get('new'))
                if old not in robot.poses or not new:
                    return False, 'nom invalide'
                if new in robot.poses:
                    return False, f'"{new}" existe deja'
                robot.poses[new] = robot.poses.pop(old)
                for steps in robot.sequences.values():
                    for st in steps:
                        if st[0] == old:
                            st[0] = new
                save_json(POSE_FILE, robot.poses)
                save_json(SEQ_FILE, robot.sequences)
                return True, f'"{old}" renommee en "{new}"'
            if path == '/api/delete':
                name = p.get('name')
                if name not in robot.poses:
                    return False, 'pose inconnue'
                used = [s for s, steps in robot.sequences.items() if any(st[0] == name for st in steps)]
                if used:
                    return False, f'pose utilisee dans la sequence "{used[0]}"'
                del robot.poses[name]
                save_json(POSE_FILE, robot.poses)
                return True, f'pose "{name}" supprimee'
            if path == '/api/goto':
                return robot.start([((p.get('name'),), float(p.get('duration', 2.0)))], 'pose')
            if path == '/api/seq/save':
                name = clean_name(p.get('name'))
                steps = [[str(st[0]), max(MIN_DURATION, float(st[1])), bool(st[2]) if len(st) > 2 and k else False]
                         for k, st in enumerate(p.get('steps', []))]
                if not name:
                    return False, 'nom de sequence invalide'
                if any(st[0] not in robot.poses for st in steps):
                    return False, 'la sequence contient une pose inconnue'
                robot.sequences[name] = steps
                save_json(SEQ_FILE, robot.sequences)
                return True, f'sequence "{name}" enregistree ({len(steps)} etapes)'
            if path == '/api/seq/delete':
                if robot.sequences.pop(p.get('name'), None) is None:
                    return False, 'sequence inconnue'
                save_json(SEQ_FILE, robot.sequences)
                return True, 'sequence supprimee'
            if path == '/api/seq/play':
                steps = robot.sequences.get(p.get('name'))
                if not steps:
                    return False, 'sequence vide ou inconnue'
                return robot.start(group_steps(steps), f'sequence {p.get("name")}')
            return False, 'commande inconnue'
    return Handler


def couper_servos(handler_of, ids):
    """Servos en panne : limite de couple a 0 et couple coupe. Ils ne forcent plus et ne chauffent plus,
    meme si une consigne de position leur arrive (la limite a 0 reste jusqu'a la mise hors tension)."""
    for i in ids:
        h = handler_of(i)
        before, res, err = h.read2ByteTxRx(i, 48)
        if res != 0:
            print(f'servo {i} : hors service, ne repond pas (com {res})')
            continue
        if 0 <= before <= 1000:
            h.write2ByteTxRx(i, 48, 0)
        h.write1ByteTxRx(i, 40, 0)
        after, res, err = h.read2ByteTxRx(i, 48)
        print(f'servo {i} : HORS SERVICE, couple coupe (limite {before / 10:.0f} % -> {after / 10:.0f} %)')


def liste_ids(texte):
    return [int(x) for x in texte.replace(' ', '').split(',') if x]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', type=str, default='sundaya1_real_config_short.yaml')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--simule', action='store_true', help='sans robot, pour essayer l interface')
    ap.add_argument('--hors-service', default='',
                    help="servos sans couple, ex. 4 ou 4,16 ('' = aucun, une fois le servo remplace)")
    args = ap.parse_args()

    if args.simule:
        mgr = FakeManager()
        enable = mgr.enable_torque
        imu = None
    else:
        from viberobotics.configs.config import load_config
        from viberobotics.motor.motor_controller_manager import MotorControllerManager
        from viberobotics.motor.ftservo_python_sdk.scservo_sdk.sms_sts import SMS_STS_TORQUE_ENABLE
        cfg = load_config(args.config)
        mgr = MotorControllerManager(
            n_motors=cfg.real_config.n_motors,
            motor_mapping=cfg.real_config.motor_controllers,
            calibration_file=cfg.real_config.calibration_file,
            remote=False, sender=False)
        ctrl_of = {int(i): c for c in mgr.controllers for i in c.motor_ids}

        def enable(i):
            ctrl_of[i].packetHandler.write1ByteTxRx(i, SMS_STS_TORQUE_ENABLE, 1)
        couper_servos(lambda i: ctrl_of[i].packetHandler, liste_ids(args.hors_service))
        imu = None
        try:
            sys.path.insert(0, os.path.join(HERE, 'walking'))
            from imu_remote import ImuClient
            imu = ImuClient('127.0.0.1')
        except Exception as e:
            print(f'centrale non disponible ({e})')

    robot = Robot(mgr, enable, imu)
    srv = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(robot))
    print(f'Serveur de poses : http://<adresse du Pi>:{args.port}   (Ctrl+C pour quitter, les servos restent tenus)')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        robot.stop()
    print('fin : les servos restent tenus')


if __name__ == '__main__':
    main()
