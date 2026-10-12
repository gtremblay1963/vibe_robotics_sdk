"""A lancer sur le Raspberry Pi, robot NON pilote par le portable (demo.py arrete,
run_receiver*.py arrete). Sert a calibrer le zero des servos des jambes.

Chaque servo tient sa position. On en choisit un et on le fait avancer ou reculer
pas a pas (1 pas = 0,088 deg). Quand l'articulation est exactement a sa position de
reference, la commande z enregistre cette position comme nouveau zero DANS le servo
(c'est ce zero qu'utilisent demo.py et run_receiver*.py).
Jambes : servos 7 a 12 et 19 a 24. Bras : servos 1 a 6 et 13 a 18.

Commandes (taper puis Entree) :
  11          choisir le servo 11
  + / -       1 pas        ++ / --   10 pas        +25 / -7   nombre de pas choisi
  0           ramener le servo choisi a son zero actuel
  a           ramener les 12 servos des jambes a leur zero actuel
  b           ramener les 12 servos des bras a leur zero actuel
  p           tableau des servos du groupe choisi (jambes ou bras)
  z           ce point devient le nouveau zero du servo choisi (confirmation demandee)
  q           quitter (les servos restent tenus)
Si imu_sender.py tourne sur le Pi, le tangage et le roulis du torse sont affiches.
Chaque nouveau zero est note dans calibration_log.txt (pour pouvoir revenir en arriere).
"""
import argparse
import os
import sys
import time

import numpy as np

from viberobotics.configs.config import load_config
from viberobotics.motor.motor_controller_manager import MotorControllerManager
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.sms_sts import SMS_STS_TORQUE_ENABLE

LEG_NAMES = {
    7: 'D hanche flexion (axe incline)', 8: 'D hanche ecartement', 9: 'D hanche rotation',
    10: 'D genou', 11: 'D cheville avant-arriere', 12: 'D cheville cote',
    19: 'G hanche flexion (axe incline)', 20: 'G hanche ecartement', 21: 'G hanche rotation',
    22: 'G genou', 23: 'G cheville avant-arriere', 24: 'G cheville cote',
}
ARM_NAMES = {
    1: 'D epaule avant-arriere', 2: 'D epaule ecartement', 3: 'D bras rotation',
    4: 'D coude', 5: 'D poignet rotation', 6: 'D pince',
    13: 'G epaule avant-arriere', 14: 'G epaule ecartement', 15: 'G bras rotation',
    16: 'G coude', 17: 'G poignet rotation', 18: 'G pince',
}
NAMES = {**LEG_NAMES, **ARM_NAMES}
STEP_DEG = 360.0 / 4096
MAX_STEPS = 50          # pas maximum par commande, par securite


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
    leg = mgr.controllers_mapping['leg']
    arm = mgr.controllers_mapping['arm']
    ids = [int(i) for i in leg.motor_ids]
    arm_ids = [int(i) for i in arm.motor_ids if int(i) in ARM_NAMES]
    order = mgr.motor_order

    def handler_of(i):
        return (arm if i in ARM_NAMES else leg).packetHandler

    def group_of(i):
        return arm_ids if i in ARM_NAMES else ids

    imu = None
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'walking'))
        from imu_remote import ImuClient
        imu = ImuClient('127.0.0.1')
    except Exception as e:
        print(f'centrale non disponible ({e})')

    # Tous les servos tiennent leur position actuelle.
    goal, _ = mgr.get_raw_state()
    goal = np.round(goal).astype(np.int32)

    def send():
        mgr.set_raw_positions(goal.copy(), 0, 20)

    def measured(i):
        q, _ = mgr.get_raw_state()
        return int(q[order[i]])

    def imu_txt():
        s = imu.get(max_age=0.5) if imu is not None else None
        return '' if s is None else f'   torse : tangage {s[0]:+5.1f}  roulis {s[1]:+5.1f}'

    def show(i):
        g = int(goal[order[i]])
        print(f'servo {i:2d} {NAMES[i]:30s} consigne {g:4d}  mesure {measured(i):4d}  '
              f'ecart au zero {(g - 2048) * STEP_DEG:+6.2f} deg{imu_txt()}')

    send()
    time.sleep(0.5)
    print(__doc__)
    sel = None
    log = open('calibration_log.txt', 'a')
    while True:
        try:
            cmd = input(f'[servo {sel if sel else "-"}] > ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd == 'q':
            break
        if cmd == 'p':
            for i in (group_of(sel) if sel else ids):
                show(i)
            continue
        if cmd in ('a', 'b'):
            grp = ids if cmd == 'a' else arm_ids
            for i in grp:
                goal[order[i]] = 2048
            send()
            time.sleep(0.8)
            for i in grp:
                show(i)
            continue
        if cmd.isdigit() and int(cmd) in NAMES:
            sel = int(cmd)
            show(sel)
            continue
        if sel is None:
            print('choisissez d\'abord un servo (jambes 7 a 12, 19 a 24 ; bras 1 a 6, 13 a 18)')
            continue
        k = order[sel]
        if cmd in ('+', '-', '++', '--') or (cmd[:1] in '+-' and cmd[1:].isdigit()):
            n = {'+': 1, '-': -1, '++': 10, '--': -10}.get(cmd)
            if n is None:
                n = int(cmd)
            n = int(np.clip(n, -MAX_STEPS, MAX_STEPS))
            goal[k] += n
            send()
            time.sleep(0.15)
            show(sel)
        elif cmd == '0':
            goal[k] = 2048
            send()
            time.sleep(0.4)
            show(sel)
        elif cmd == 'z':
            delta = int(goal[k]) - 2048
            ok = input(f'Nouveau zero du servo {sel} ici ({delta * STEP_DEG:+.2f} deg par rapport a l\'ancien) ? o/n : ')
            if ok.strip().lower() != 'o':
                print('annule')
                continue
            mgr.zero_motors([sel])                         # position actuelle -> 2048 dans le servo
            time.sleep(0.3)
            handler_of(sel).write1ByteTxRx(sel, SMS_STS_TORQUE_ENABLE, 1)
            goal[k] = 2048
            send()
            time.sleep(0.3)
            log.write(f'{time.strftime("%Y-%m-%d %H:%M:%S")} servo {sel} ({NAMES[sel]}) : '
                      f'nouveau zero a {delta:+d} pas ({delta * STEP_DEG:+.2f} deg) de l\'ancien\n')
            log.flush()
            print('nouveau zero enregistre')
            show(sel)
        else:
            print('commande inconnue')
    log.close()
    print('fin : les servos restent tenus a leur position')


if __name__ == '__main__':
    main()
