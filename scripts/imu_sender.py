"""A lancer sur le Raspberry Pi : lit la BNO055 en I2C et envoie tangage et roulis
du torse par UDP a qui les demande (le portable, via scripts/walking/imu_remote.py).

Montage de la carte : X vers la droite du robot, Y vers l'avant, Z vers le haut.
Paquet envoye (6 flottants) : horodatage Pi (s), tangage (deg, + = avant),
roulis (deg, + = droite du robot), vitesse de tangage (deg/s), vitesse de roulis (deg/s),
nombre d'erreurs I2C, cap cumule depuis le demarrage (deg, + = vers la gauche).
"""
import argparse
import math
import socket
import struct
import time

from smbus2 import SMBus

ADDR = 0x28
REG_CHIP_ID, REG_GYRO, REG_QUAT, REG_GRAVITY, REG_OPR_MODE = 0x00, 0x14, 0x20, 0x2E, 0x3D
MODE_CONFIG, MODE_IMU = 0x00, 0x08
PACKET = struct.Struct('<7d')
SUBSCRIBER_TIMEOUT = 3.0   # s sans nouvelle demande avant d'arreter d'envoyer
MAX_JUMP_DEG = 2.0         # ecart tolere entre le saut d'angle mesure et celui predit par le gyroscope
MAX_REJECTS = 5            # rejets consecutifs avant de reprendre la mesure telle quelle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=9001)
    parser.add_argument('--bus', type=int, default=1)
    parser.add_argument('--rate', type=float, default=60.0, help='envois par seconde')
    parser.add_argument('--print', action='store_true', help='affiche les angles')
    args = parser.parse_args()

    bus = SMBus(args.bus)

    def rd(reg, n):
        return bytes(bus.read_i2c_block_data(ADDR, reg, n))

    chip = rd(REG_CHIP_ID, 1)[0]
    if chip != 0xA0:
        raise SystemExit(f'BNO055 introuvable (identifiant 0x{chip:02X}, attendu 0xA0)')
    bus.write_byte_data(ADDR, REG_OPR_MODE, MODE_CONFIG)
    time.sleep(0.05)
    bus.write_byte_data(ADDR, REG_OPR_MODE, MODE_IMU)
    time.sleep(0.1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.setblocking(False)
    print(f'BNO055 prete, en attente de demandes sur le port UDP {args.port}')

    subscriber, last_hello, errors, n = None, 0.0, 0, 0
    last, rejects = None, 0   # derniere mesure acceptee : (instant, tangage, roulis)
    yaw, last_heading = 0.0, None   # cap cumule depuis le demarrage (deg, + = vers la gauche)
    period = 1.0 / args.rate
    next_t = time.monotonic()
    while True:
        try:
            while True:
                _, addr = sock.recvfrom(64)
                if addr != subscriber:
                    print(f'envoi vers {addr[0]}:{addr[1]}')
                subscriber, last_hello = addr, time.monotonic()
        except BlockingIOError:
            pass

        try:
            # Le premier octet d'un bloc est parfois corrompu sur le bus I2C du Pi :
            # on commence un registre plus tot et on jette cet octet.
            gx, gy, gz = (v / 100.0 for v in struct.unpack('<hhh', rd(REG_GRAVITY - 1, 7)[1:]))
            wx, wy, wz = (v / 16.0 for v in struct.unpack('<hhh', rd(REG_GYRO - 1, 7)[1:]))
            qw, qx, qy, qz = (v / 16384.0 for v in struct.unpack('<hhhh', rd(REG_QUAT - 1, 9)[1:]))
            now = time.monotonic()
            pitch = math.degrees(math.atan2(-gy, gz))
            roll = math.degrees(math.atan2(-gx, gz))
            pitch_rate, roll_rate = -wx, wy
            heading = math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
            ok = (8.8 < math.sqrt(gx * gx + gy * gy + gz * gz) < 10.8
                  and 0.9 < qw * qw + qx * qx + qy * qy + qz * qz < 1.1)
            if ok and last is not None and rejects < MAX_REJECTS:
                dt = now - last[0]
                turn = (heading - last_heading + 180.0) % 360.0 - 180.0
                ok = (abs(pitch - last[1] - pitch_rate * dt) < MAX_JUMP_DEG
                      and abs(roll - last[2] - roll_rate * dt) < MAX_JUMP_DEG
                      and abs(turn - wz * dt) < MAX_JUMP_DEG)
            if not ok:
                rejects += 1
                raise OSError('mesure incoherente')
            if last_heading is not None:
                yaw += (heading - last_heading + 180.0) % 360.0 - 180.0
            last, rejects, last_heading = (now, pitch, roll), 0, heading
        except OSError:
            errors += 1
        else:
            if subscriber is not None and time.monotonic() - last_hello < SUBSCRIBER_TIMEOUT:
                sock.sendto(PACKET.pack(time.time(), pitch, roll, pitch_rate, roll_rate, errors, yaw), subscriber)
            n += 1
            if args.print and n % 20 == 0:
                print(f'tangage {pitch:+6.1f}  roulis {roll:+6.1f}  cap {yaw:+7.1f}  erreurs {errors}')

        next_t += period
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()


if __name__ == '__main__':
    main()
