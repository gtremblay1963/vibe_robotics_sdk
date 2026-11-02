"""Test d'un seul servo STS3215 (a lancer sur le Pi, demo.py / run_receiver*.py / pose_server.py ARRETES).

  python scripts/test_servo.py --id 4            # coude droit (bus des bras)
  python scripts/test_servo.py --id 16           # coude gauche, pour comparer
  python scripts/test_servo.py --id 9 --port /dev/sundaya1_legs   # un servo de jambe

Commandes (taper puis Entree) :
  t           tenir (moteur alimente, garde sa position actuelle)
  r           relacher (moteur libre, se tourne a la main)
  +10 / -10   avancer / reculer de 10 degres (max 90 par commande)
  a 30        aller a +30 deg du zero (a 0 = zero)
  i           etat : position, charge, courant, tension, temperature, erreurs
  b 45        balayage lent de +-45 deg autour de la position actuelle, charge mesuree
              tous les 3 deg : un frottement anormal = charge plus haute ou par pics
  s           suivi : moteur relache, la position s'affiche pendant qu'on le tourne a la main
              (verifie le capteur) ; Entree pour arreter
  q           quitter (le moteur est relache)
Force limitee par defaut a 30 % (--couple) : en cas de blocage, il cale au lieu de casser.
"""
import argparse
import select
import sys
import time

from viberobotics.motor.ftservo_python_sdk.scservo_sdk.port_handler import PortHandler
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.sms_sts import (
    sms_sts, SMS_STS_TORQUE_ENABLE, SMS_STS_PRESENT_POSITION_L, SMS_STS_PRESENT_LOAD_L,
    SMS_STS_PRESENT_CURRENT_L, SMS_STS_PRESENT_VOLTAGE, SMS_STS_PRESENT_TEMPERATURE)
from viberobotics.motor.ftservo_python_sdk.scservo_sdk.scservo_def import COMM_SUCCESS

TORQUE_LIMIT_ADDR = 48          # meme adresse que run_receiver_log.py (0-1000, RAM)
DEG = 360.0 / 4096              # degres par pas
VITESSE = 300                   # pas/s (~26 deg/s) : lent, pour bien sentir et mesurer
ACC = 30
ERREURS = {0: 'tension', 1: 'capteur', 2: 'temperature', 3: 'courant', 5: 'surcharge'}


class Servo:
    def __init__(self, port, sid):
        self.port = PortHandler(port)
        if not self.port.openPort() or not self.port.setBaudRate(1000000):
            sys.exit(f'impossible d\'ouvrir {port} (un autre programme l\'utilise ?)')
        self.h = sms_sts(self.port)
        self.id = sid

    def lire2(self, adr):
        v, res, err = self.h.read2ByteTxRx(self.id, adr)
        return (v if res == COMM_SUCCESS else None), err

    def lire1(self, adr):
        v, res, err = self.h.read1ByteTxRx(self.id, adr)
        return (v if res == COMM_SUCCESS else None), err

    def position(self):
        v, _ = self.lire2(SMS_STS_PRESENT_POSITION_L)
        return v

    def charge(self):
        """Charge en % du couple maximal, signee (bit 10 = sens)."""
        v, _ = self.lire2(SMS_STS_PRESENT_LOAD_L)
        if v is None:
            return None
        return (-1 if v & 0x400 else 1) * (v & 0x3FF) / 10.0

    def courant_ma(self):
        v, _ = self.lire2(SMS_STS_PRESENT_CURRENT_L)
        if v is None:
            return None
        return (-1 if v & 0x8000 else 1) * (v & 0x7FFF) * 6.5

    def couple(self, on):
        self.h.write1ByteTxRx(self.id, SMS_STS_TORQUE_ENABLE, 1 if on else 0)

    def aller(self, pas):
        pas = max(0, min(4095, int(pas)))
        self.h.WritePosEx(self.id, pas, VITESSE, ACC)
        return pas

    def attendre(self, cible, delai=6.0):
        t0 = time.time()
        while time.time() - t0 < delai:
            p = self.position()
            if p is not None and abs(p - cible) <= 8:
                return p
            time.sleep(0.05)
        return self.position()

    def etat(self):
        p = self.position()
        tension, err = self.lire1(SMS_STS_PRESENT_VOLTAGE)
        temp, _ = self.lire1(SMS_STS_PRESENT_TEMPERATURE)
        couple_on, _ = self.lire1(SMS_STS_TORQUE_ENABLE)
        lim, _ = self.lire2(TORQUE_LIMIT_ADDR)
        errs = [n for b, n in ERREURS.items() if err & (1 << b)] if err else []
        pos_txt = f'{p} ({(p - 2048) * DEG:+.1f} deg)' if p is not None else '?'
        print(f'  position {pos_txt}  charge {self.charge()} %  courant {self.courant_ma()} mA  '
              f'tension {tension / 10 if tension else "?"} V  temperature {temp} C  '
              f'moteur {"TENU" if couple_on else "libre"}  limite {lim / 10 if lim else "?"} %  '
              f'erreurs : {", ".join(errs) if errs else "aucune"}')


def balayage(s, amplitude):
    p0 = s.position()
    if p0 is None:
        print('  pas de reponse du servo')
        return
    s.couple(True)
    n = int(amplitude / DEG)
    pas3 = int(3 / DEG)
    res = {'aller': [], 'retour': []}
    for sens, a, b in (('aller', p0, p0 + n), ('retour', p0 + n, p0 - n), ('aller', p0 - n, p0)):
        cle = 'aller' if sens == 'aller' else 'retour'
        d = pas3 if b > a else -pas3
        for cible in range(a, b + d, d):
            s.aller(cible)
            time.sleep(0.12)
            c = s.charge()
            if c is not None:
                p = s.position()
                if p is not None:
                    res[cle].append((round((p - 2048) * DEG), c))
    for cle, val in res.items():
        if not val:
            continue
        ch = [abs(c) for _, c in val]
        pire = max(val, key=lambda x: abs(x[1]))
        print(f'  {cle:6s} : charge moyenne {sum(ch) / len(ch):5.1f} %   max {max(ch):5.1f} % (a {pire[0]:+d} deg)')
    print('  detail (deg : charge %) : ' + '  '.join(f'{a:+d}:{c:.0f}' for a, c in res['retour']))
    s.aller(p0)


def suivi(s):
    s.couple(False)
    print('  moteur relache : tournez-le a la main. Entree pour arreter.')
    while True:
        p = s.position()
        print(f'\r  position {p} ({(p - 2048) * DEG:+7.1f} deg)   ' if p is not None else '\r  pas de lecture   ',
              end='', flush=True)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            sys.stdin.readline()
            print()
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--id', type=int, default=4, help='numero du servo (coude droit = 4, coude gauche = 16)')
    ap.add_argument('--port', default='/dev/sundaya1_arms', help='bus des bras (jambes : /dev/sundaya1_legs)')
    ap.add_argument('--couple', type=float, default=30.0, help='force maximale en %% (defaut 30)')
    args = ap.parse_args()

    s = Servo(args.port, args.id)
    model, res, err = s.h.ping(args.id)
    if res != COMM_SUCCESS:
        sys.exit(f'le servo {args.id} ne repond pas sur {args.port} : cable, alimentation ou numero ?')
    print(f'servo {args.id} trouve (modele {model})')
    lim, _ = s.lire2(TORQUE_LIMIT_ADDR)
    if lim is not None and 100 <= lim <= 1000:
        s.h.write2ByteTxRx(args.id, TORQUE_LIMIT_ADDR, int(args.couple * 10))
    else:
        print(f'  limite de couple non modifiee (lecture {lim}) : attention, force complete')
    s.couple(False)
    s.etat()
    print("Commandes : t tenir | r relacher | +10/-10 deg | a 30 | i etat | b 45 balayage | s suivi | q quitter")
    cible = s.position() or 2048
    try:
        while True:
            cmd = input(f'[servo {args.id}] > ').strip().lower()
            if cmd == 'q':
                break
            elif cmd == 't':
                cible = s.position() or cible
                s.aller(cible)
                s.couple(True)
                s.etat()
            elif cmd == 'r':
                s.couple(False)
                s.etat()
            elif cmd == 'i':
                s.etat()
            elif cmd[:1] in '+-' and cmd[1:].replace('.', '', 1).isdigit():
                d = max(-90.0, min(90.0, float(cmd)))
                cible = s.aller((s.position() or cible) + d / DEG)
                s.couple(True)
                s.attendre(cible)
                s.etat()
            elif cmd.startswith('a ') and cmd[2:].strip().lstrip('+-').replace('.', '', 1).isdigit():
                cible = s.aller(2048 + float(cmd[2:]) / DEG)
                s.couple(True)
                s.attendre(cible)
                s.etat()
            elif cmd.startswith('b'):
                amp = float(cmd[1:]) if cmd[1:].strip().replace('.', '', 1).isdigit() else 45.0
                balayage(s, min(amp, 90.0))
                s.etat()
            elif cmd == 's':
                suivi(s)
                s.etat()
            elif cmd:
                print('commande inconnue (t, r, +10, -10, a 30, i, b 45, s, q)')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        s.couple(False)
        print('\nmoteur relache')
        s.port.closePort()


if __name__ == '__main__':
    main()
