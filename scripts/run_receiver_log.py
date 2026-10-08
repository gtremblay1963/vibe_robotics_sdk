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
    COMM_SUCCESS, SMS_STS_PRESENT_POSITION_L, SMS_STS_PRESENT_SPEED_L, SMS_STS_PRESENT_LOAD_L)


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
    args = parser.parse_args()

    cfg = load_config(args.config)

    motor_controller_manager = MotorControllerManager(
        n_motors=cfg.real_config.n_motors,
        motor_mapping=cfg.real_config.motor_controllers,
        calibration_file=cfg.real_config.calibration_file,
        remote=True,
        sender=False,
    )
    run(motor_controller_manager, time.strftime('legs_log_%Y%m%d_%H%M%S.csv'), args.acc)
