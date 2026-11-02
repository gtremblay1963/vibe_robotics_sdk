"""A lancer sur le Raspberry Pi a la place de run_receiver.py.

Meme fonction (recoit les consignes du portable et pilote les servos), plus un
enregistrement : apres chaque consigne, lit la position, la vitesse et la charge
reelles des 12 servos des jambes et les ecrit dans legs_log_AAAAMMJJ_HHMMSS.csv.

Colonnes : t (s), puis pour chaque servo de jambe (numero du servo) :
c<id> = consigne (pas), p<id> = position lue (pas), v<id> = vitesse lue (pas/s),
l<id> = charge lue (0,1 %, signee). 4096 pas = 1 tour.
"""
import argparse
import time

import numpy as np

from viberobotics.configs.config import load_config
from viberobotics.motor.motor_controller_manager import MotorControllerManager
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.group_sync_read import GroupSyncRead
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.sms_sts import (
    COMM_SUCCESS, SMS_STS_PRESENT_POSITION_L, SMS_STS_PRESENT_SPEED_L, SMS_STS_PRESENT_LOAD_L,
    SMS_STS_TORQUE_ENABLE)


ANKLE_PITCH_IDS = (11, 23)   # chevilles avant-arriere (droite, gauche)
ARM_IDS = (1, 2, 3, 4, 5, 13, 14, 15, 16, 17)
GRIPPER_IDS = (6, 18)
TORQUE_LIMIT_ADDR = 48       # STS3215 : limite de couple en RAM, 0 a 1000 (0,1 %), remise a 100 % a la mise sous tension
HORS_SERVICE = ''            # servos en panne, sans couple (ex. '4') ; coude droit remplace le 2026-10-28


def couper_servos(handler_of, ids):
    """Servos en panne : limite de couple a 0 et couple coupe. Ils ne forcent plus et ne chauffent plus,
    meme si une consigne de position leur arrive (la limite a 0 reste jusqu'a la mise hors tension)."""
    for i in ids:
        h = handler_of(i)
        before, res, err = h.read2ByteTxRx(i, TORQUE_LIMIT_ADDR)
        if res != COMM_SUCCESS:
            print(f'servo {i} : hors service, ne repond pas (com {res})')
            continue
        if 0 <= before <= 1000:
            h.write2ByteTxRx(i, TORQUE_LIMIT_ADDR, 0)
        h.write1ByteTxRx(i, SMS_STS_TORQUE_ENABLE, 0)
        after, res, err = h.read2ByteTxRx(i, TORQUE_LIMIT_ADDR)
        print(f'servo {i} : HORS SERVICE, couple coupe (limite {before / 10:.0f} % -> {after / 10:.0f} %)')


def liste_ids(texte):
    return [int(x) for x in texte.replace(' ', '').split(',') if x]


def set_torque_limit(manager, ids, percent):
    """Limite la force des servos : en cas de choc, le servo cale au lieu de casser la piece.
    Ecrit seulement si la valeur relue avant a l'allure attendue (sinon, l'adresse serait fausse)."""
    handler = manager.controllers_mapping['arm'].packetHandler
    for i in ids:
        before, res, err = handler.read2ByteTxRx(i, TORQUE_LIMIT_ADDR)
        if res != COMM_SUCCESS or not 100 <= before <= 1000:
            print(f'servo {i} : limite de couple non modifiee (lecture {before}, com {res}) ; verifier l adresse')
            continue
        handler.write2ByteTxRx(i, TORQUE_LIMIT_ADDR, int(percent * 10))
        after, res, err = handler.read2ByteTxRx(i, TORQUE_LIMIT_ADDR)
        print(f'servo {i} : limite de couple {before / 10:.0f} % -> {after / 10:.0f} %')


def set_ankle_gain(manager, kp, kd):
    """Raideur des deux chevilles avant-arriere (valeur d'origine des STS3215 : P = 32, D = 32)."""
    handler = manager.controllers_mapping['leg'].packetHandler
    for i in ANKLE_PITCH_IDS:
        handler.WriteKpKd(i, kp, kd)
        rkp, rkd, res, err = handler.ReadKpKd(i)
        print(f'cheville servo {i} : P = {rkp}, D = {rkd} (com {res}, err {err})')


def run(manager, log_path, acc=50):
    leg = manager.controllers_mapping['leg']
    ids = [int(i) for i in leg.motor_ids]
    idxs = manager.motor_order[leg.motor_ids]
    handler = leg.packetHandler
    # Une seule lecture groupee de 6 octets par servo : position, vitesse, charge.
    reader = GroupSyncRead(handler, SMS_STS_PRESENT_POSITION_L, 6)
    for i in ids:
        reader.addParam(i)

    def read_legs():
        if reader.txRxPacket() != COMM_SUCCESS:
            return None
        pos, speed, load = [], [], []
        for i in ids:
            if not reader.isAvailable(i, SMS_STS_PRESENT_POSITION_L, 6):
                return None
            pos.append(reader.getData(i, SMS_STS_PRESENT_POSITION_L, 2))
            speed.append(handler.scs_tohost(reader.getData(i, SMS_STS_PRESENT_SPEED_L, 2), 15))
            load.append(handler.scs_tohost(reader.getData(i, SMS_STS_PRESENT_LOAD_L, 2), 10))
        return pos + speed + load

    n, failed, read_s = 0, 0, 0.0
    with open(log_path, 'w') as log:
        log.write('t,' + ','.join(f'{k}{i}' for k in 'cpvl' for i in ids) + '\n')
        print(f'Enregistrement des jambes dans {log_path} (acceleration {acc})')
        while True:
            try:
                q = manager.remote_socket.recv()
                manager.set_raw_positions(q, 0, acc)
            except Exception as e:
                print(f"Error in receiver loop: {e}")
                break
            t = time.time()
            try:
                state = read_legs()
            except Exception:
                state = None
            read_s += time.time() - t
            if state is None:
                failed += 1
                continue
            cmd = [int(v) for v in np.round(np.asarray(q)[idxs])]
            log.write(f'{t:.3f},' + ','.join(str(int(v)) for v in cmd + state) + '\n')
            n += 1
            if n % 30 == 0:
                log.flush()
    print(f'{n} lignes enregistrees, {failed} lectures ratees, '
          f'lecture moyenne {1000 * read_s / max(n + failed, 1):.1f} ms')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default='sundaya1_real_config_short.yaml')
    parser.add_argument('--acc', type=int, default=50, help='acceleration des servos (50 = valeur d origine)')
    parser.add_argument('--ankle-kp', type=int, default=64, help='raideur P des chevilles avant-arriere (origine : 32, 64 valide le 11 oct.)')
    parser.add_argument('--ankle-kd', type=int, default=32, help='amortissement D des chevilles (origine : 32)')
    parser.add_argument('--bras-couple', type=float, default=100, help='limite de couple des bras en %% (100 = aucune limite)')
    parser.add_argument('--pince-couple', type=float, default=30, help='limite de couple des pinces en %% (100 = aucune limite)')
    parser.add_argument('--hors-service', default=HORS_SERVICE,
                        help="servos sans couple, ex. 4 ou 4,16 ('' = aucun, une fois le servo remplace)")
    args = parser.parse_args()

    cfg = load_config(args.config)

    motor_controller_manager = MotorControllerManager(
        n_motors=cfg.real_config.n_motors,
        motor_mapping=cfg.real_config.motor_controllers,
        calibration_file=cfg.real_config.calibration_file,
        remote=True,
        sender=False,
    )
    if args.ankle_kp is not None:
        set_ankle_gain(motor_controller_manager, args.ankle_kp, args.ankle_kd)
    set_torque_limit(motor_controller_manager, ARM_IDS, args.bras_couple)
    set_torque_limit(motor_controller_manager, GRIPPER_IDS, args.pince_couple)
    ctrl_of = {int(i): c for c in motor_controller_manager.controllers for i in c.motor_ids}
    couper_servos(lambda i: ctrl_of[i].packetHandler, liste_ids(args.hors_service))
    run(motor_controller_manager, time.strftime('legs_log_%Y%m%d_%H%M%S.csv'), args.acc)
