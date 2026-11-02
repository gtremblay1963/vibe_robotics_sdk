"""A lancer sur le Raspberry Pi, robot NON pilote par le portable (demo.py et run_receiver*.py arretes).
Sert a enregistrer des poses a la main puis a les rejouer en douceur (s'asseoir, se relever, ...).

Au depart, tous les servos (jambes, bras, tete) tiennent leur position actuelle.

Commandes (taper puis Entree) :
  l jambes | l bras | l tout | l 10 22    relacher (couple coupe) : le membre se bouge a la main
  t                tenir : les servos relaches se bloquent a leur position actuelle
  e assis          enregistrer la position actuelle des 25 servos sous le nom "assis"
  v assis 3        aller a la pose "assis" en 3 s (2 s par defaut), en douceur
  s debout:2 accroupi:3 assis:2      sequence de poses (duree en s apres les deux-points)
  p                liste des poses enregistrees
  x assis          supprimer la pose "assis"
  m                angles actuels, jambe droite / jambe gauche cote a cote, et inclinaison du torse
  q                quitter (les servos restent tenus)
Ctrl+C pendant un mouvement : arret immediat, le robot reste ou il est.
ATTENTION : relacher les jambes d'un robot debout le fait s'effondrer. Tenez-le avant "l jambes".
Les poses sont dans poses.json, a cote de ce script.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

from viberobotics.configs.config import load_config
from viberobotics.motor.motor_controller_manager import MotorControllerManager
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.sms_sts import SMS_STS_TORQUE_ENABLE

LEGS = [7, 8, 9, 10, 11, 12, 19, 20, 21, 22, 23, 24]
ARMS = [1, 2, 3, 4, 5, 6, 13, 14, 15, 16, 17, 18]
LEG_PAIRS = [(7, 19, 'hanche flexion'), (8, 20, 'hanche ecartement'), (9, 21, 'hanche rotation'),
             (10, 22, 'genou'), (11, 23, 'cheville avant-arriere'), (12, 24, 'cheville cote')]
STEP_DEG = 360.0 / 4096
MAX_SPEED_DEG = 90.0      # vitesse max d'un servo pendant un mouvement (deg/s)
RATE = 50.0               # Hz
POSE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'poses.json')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default='sundaya1_real_config_short.yaml')
    args = parser.parse_args()
    cfg = load_config(args.config)
    mgr = MotorControllerManager(
        n_motors=cfg.real_config.n_motors,
        motor_mapping=cfg.real_config.motor_controllers,
        calibration_file=cfg.real_config.calibration_file,
        remote=False, sender=False)
    order = mgr.motor_order
    all_ids = sorted(int(i) for c in mgr.controllers for i in c.motor_ids)
    ctrl_of = {int(i): c for c in mgr.controllers for i in c.motor_ids}

    imu = None
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'walking'))
        from imu_remote import ImuClient
        imu = ImuClient('127.0.0.1')
    except Exception as e:
        print(f'centrale non disponible ({e})')

    poses = json.load(open(POSE_FILE)) if os.path.exists(POSE_FILE) else {}
    relaxed = set()

    def read():
        q, _ = mgr.get_raw_state()
        return np.round(q).astype(np.int32)

    goal = read()

    def send(g):
        mgr.set_raw_positions(g.copy(), 0, 50)

    def hold_here():
        """Bloque tous les servos a leur position mesuree (et remet le couple des servos relaches)."""
        nonlocal goal
        goal = read()
        send(goal)
        for i in sorted(relaxed):
            ctrl_of[i].packetHandler.write1ByteTxRx(i, SMS_STS_TORQUE_ENABLE, 1)
        relaxed.clear()

    def imu_txt():
        s = imu.get(max_age=0.5) if imu is not None else None
        return 'centrale absente' if s is None else f'torse : tangage {s[0]:+5.1f}  roulis {s[1]:+5.1f}'

    def move_to(target, duration):
        """Mouvement en douceur (profil sans a-coup) de la position actuelle vers target."""
        nonlocal goal
        hold_here()
        start = goal.astype(float)
        target = target.astype(float)
        need = np.abs(target - start).max() * STEP_DEG / MAX_SPEED_DEG
        if need > duration:
            print(f'  duree allongee a {need:.1f} s (vitesse max {MAX_SPEED_DEG:.0f} deg/s)')
            duration = need
        n = max(1, int(duration * RATE))
        t0 = time.time()
        try:
            for k in range(1, n + 1):
                s = k / n
                s = 10 * s**3 - 15 * s**4 + 6 * s**5
                goal = np.round(start + (target - start) * s).astype(np.int32)
                send(goal)
                time.sleep(max(0.0, t0 + k / RATE - time.time()))
        except KeyboardInterrupt:
            hold_here()
            print('\n  ARRET : le robot reste dans sa position actuelle')
            return False
        return True

    def pose_array(name):
        p = poses[name]
        g = read()
        for i, v in p.items():
            g[order[int(i)]] = int(v)
        return g

    def parse_ids(words):
        if not words:
            return []
        if words[0] == 'jambes':
            return LEGS
        if words[0] == 'bras':
            return ARMS
        if words[0] == 'tout':
            return all_ids
        ids = [int(w) for w in words if w.isdigit() and int(w) in ctrl_of]
        return ids

    send(goal)
    time.sleep(0.3)
    print(__doc__)
    while True:
        try:
            line = input(f'[{len(relaxed)} relache(s)] > ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        w = line.split()
        if not w:
            continue
        cmd, rest = w[0], w[1:]
        if cmd == 'q':
            break
        elif cmd == 'l':
            ids = parse_ids(rest)
            if not ids:
                print('exemples : l jambes   l bras   l tout   l 10 22')
                continue
            if any(i in LEGS for i in ids):
                if input('Le robot est-il tenu ou couche ? (les jambes vont lacher) o/n : ').strip().lower() != 'o':
                    print('annule')
                    continue
            mgr.disable_torque(ids)
            relaxed.update(ids)
            print(f'relache : {ids}')
        elif cmd == 't':
            hold_here()
            print('tous les servos sont tenus')
        elif cmd == 'e' and len(rest) == 1:
            q = read()
            poses[rest[0]] = {str(i): int(q[order[i]]) for i in all_ids}
            json.dump(poses, open(POSE_FILE, 'w'), indent=1)
            print(f'pose "{rest[0]}" enregistree   ({imu_txt()})')
        elif cmd == 'v' and rest and rest[0] in poses:
            d = float(rest[1]) if len(rest) > 1 else 2.0
            if move_to(pose_array(rest[0]), d):
                print(f'arrive a "{rest[0]}"   ({imu_txt()})')
        elif cmd == 's' and rest:
            steps = []
            for item in rest:
                name, _, d = item.partition(':')
                if name not in poses:
                    print(f'pose inconnue : {name}')
                    steps = []
                    break
                steps.append((name, float(d) if d else 2.0))
            for name, d in steps:
                print(f'  -> {name} ({d:.1f} s)')
                if not move_to(pose_array(name), d):
                    break
            else:
                if steps:
                    print(f'sequence terminee   ({imu_txt()})')
        elif cmd == 'p':
            print(', '.join(sorted(poses)) or 'aucune pose')
        elif cmd == 'x' and rest and rest[0] in poses:
            del poses[rest[0]]
            json.dump(poses, open(POSE_FILE, 'w'), indent=1)
            print(f'pose "{rest[0]}" supprimee')
        elif cmd == 'm':
            q = read()
            sc = mgr.sign_change
            deg = lambda i: (q[order[i]] - 2048) * STEP_DEG * sc[order[i]]
            print(f'{"":24s} {"droite":>8s} {"gauche":>8s}   (deg, convention du modele)')
            for d, g, name in LEG_PAIRS:
                print(f'{name:24s} {deg(d):+8.1f} {deg(g):+8.1f}')
            print(imu_txt())
        else:
            print('commande inconnue ou pose inexistante (p = liste des poses)')
    print('fin : les servos restent tenus')


if __name__ == '__main__':
    main()
