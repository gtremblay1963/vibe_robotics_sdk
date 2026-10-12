from robot import Robot
from classes import WalkState
from viberobotics.configs.config import load_config
from viberobotics.motor.motor_controller_manager import MotorControllerManager
from viberobotics.utils.utils import get_asset_path
import pygame
from enum import Enum
import numpy as np
import time
from loop_rate_limiters import RateLimiter
import argparse
from imu_remote import ImuClient


# Real robot only. Positive values shift each step to the robot's left.
# The straight walk drifts right, so both defaults push left, in meters,
# at full stick. 5 mm was below the 1 cm IK tolerance and did not move
# the motors. Set the flag to False to walk with no lateral compensation.
LATERAL_COMP_ENABLED = False
LATERAL_COMP_FORWARD_LEFT_M = 0.03
LATERAL_COMP_BACKWARD_LEFT_M = 0.05

# Arret : le robot recentre son poids avant de revenir a la pose debout.
STAND_SETTLE_MAX = 1.5   # s, duree maximale du recentrage
STAND_SLEW = 0.004       # rad par tick, vitesse du retour a la pose debout (0,13 rad/s)

# Protection : la marche s'arrete si le torse s'incline trop par rapport a la pose
# mesuree au demarrage. Marche au stick : jamais plus de 10 deg pendant 0,15 s en marche normale.
SAFETY_TILT_DEG = 10.0
SAFETY_TIME = 0.15       # s au-dela du seuil avant l'arret

# Bouton RB, robot arrete : retour au cap pris au demarrage (appui sur Start).
HOME_TOL_DEG = 3.0       # ecart accepte a la fin
HOME_MAX_CMD = 0.5       # commande de rotation maximale (moitie du stick)
HOME_MIN_CMD = 0.15      # commande minimale, pour que le robot fasse vraiment un pas
HOME_TRIES = 5           # nombre maximal d'essais si l'ecart reste trop grand
HOME_SETTLE = 1.0        # s d'immobilite avant de mesurer l'ecart

# Poussee a l'arret : si le torse penche de plus de PUSH_ON_DEG vers l'avant (ou l'arriere)
# par rapport a la pose de reference, le robot fait des pas dans ce sens jusqu'a se redresser.
PUSH_ON_DEG = 7.0        # declenchement (reference -2 affiche -> +5 affiche)
PUSH_LEAD = 0.25         # s : on anticipe l'inclinaison avec sa vitesse (declenche plus tot si ca bascule vite)
PUSH_MIN_DEG = 3.0       # ... mais jamais sous cet ecart reel
PUSH_TIME = 0.03         # s au-dela du seuil avant de partir (1 cycle)
# Pas de rattrapage rapides : temps et longueur de pas propres a ce mode.
PUSH_DSP = 0.08          # s deux pieds au sol (marche normale : 0,3)
PUSH_SSP = 0.3           # s par pas (marche normale : 0,6)
PUSH_STEP = 0.05         # m par pas (marche normale : 0,04)
PUSH_BIG_DEG = 11.0      # au-dela de cet ecart pendant le premier pas, un deuxieme pas
PUSH_REARM = 1.5         # s immobile apres un rattrapage avant de pouvoir en declencher un autre
PUSH_CMD = 1.0           # commande d'avance pendant les pas (1 = pas complets de 4 cm)
SAFETY_PUSH_DEG = 25.0   # seuil hors marche au stick : debout et pendant les pas de rattrapage

class JoystickButton(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    START = 7

class Demo(Robot):
    def __init__(self, enable_teleop=False, imu=None):
        super().__init__()
        self.enable_teleop = enable_teleop
        # Centrale inertielle : lecture et enregistrement seulement, aucune correction.
        self.imu = imu
        self._rb_prev = False
        self.imu_log = None
        if imu is not None:
            name = time.strftime('imu_log_%Y%m%d_%H%M%S.csv')
            self.imu_log = open(name, 'w')
            self.imu_log.write('t,etat,cmd_avant,cmd_cote,cmd_rot,tangage,roulis,vit_tangage,vit_roulis,age,'
                               'tangage_prevu,roulis_prevu,rejets,calcul_ms,cap,mode\n')
            print(f'centrale inertielle : enregistrement dans {name}')

    def leg_targets(self, last_legs, stand_legs, dt):
        """Consignes des 12 articulations des jambes pour ce tick."""
        fsm = self.fsm
        if fsm.state != WalkState.STAND:
            self._settle_t = 0.0
        elif self._settle_t is not None:
            self._settle_t += dt
            com = fsm.stance.com
            mid = 0.5 * (fsm.stance.left_foot.position[:2] + fsm.stance.right_foot.position[:2])
            centered = (np.linalg.norm(com.position[:2] - mid) < 0.003
                        and np.linalg.norm(com.velocity[:2]) < 0.01)
            if centered or self._settle_t > STAND_SETTLE_MAX:
                self._settle_t = None
        if self._settle_t is not None:      # en marche, ou recentrage juste apres l'arret
            self.q, success = self.ik(self._get_targets())
            return self.safe_leg_q(self.q, last_legs, success)
        self.q = self.default_q.copy()
        # Toutes les articulations avancent ensemble, a la meme fraction du trajet restant.
        diff = stand_legs - last_legs
        biggest = float(np.abs(diff).max())
        return last_legs + diff * min(1.0, STAND_SLEW / biggest) if biggest > 0 else stand_legs.copy()

    def reset_supervisor(self):
        """Oublie la pose de reference : elle sera reprise au prochain demarrage."""
        self._ref, self._ref_samples = None, []
        self._ref_wait = 1.5     # s : laisser le robot finir de se redresser avant de prendre la reference
        self._safety_t, self._safety_on = 0.0, False
        self._home = None
        self._push_dir, self._push_t = 0, 0.0
        self._push_steps, self._push_count, self._push_prev, self._push_rearm = 0, 0, None, 0.0
        self._ep_prev, self._ep_rate = None, 0.0
        self.set_step_timing(fast=False)
        self._home_gain = 15.0   # deg tournes par pas pour une commande de 1 (corrige par la mesure)

    def set_step_timing(self, fast):
        """Temps et longueur des pas : rapides pour le rattrapage, normaux sinon (pris a l'arret seulement)."""
        wc = self.walk_config
        self.fsm.dsp_duration = PUSH_DSP if fast else wc.dsp_duration
        self.fsm.ssp_duration = PUSH_SSP if fast else wc.ssp_duration
        self.fsm.footstep_generator.step_length = PUSH_STEP if fast else wc.step_length
        self._push_fast = fast

    def toggle_home(self):
        """Bouton RB : lance ou annule le retour au cap de depart."""
        if self._home is not None:
            self._home = None
            print('retour au cap : annule')
        elif self.imu is None or self._ref is None:
            print('retour au cap : impossible, centrale absente ou pas encore prete')
        elif self._safety_on:
            print('retour au cap : impossible pendant un arret de securite')
        elif self._push_dir != 0:
            print('retour au cap : impossible pendant les pas de rattrapage')
        else:
            self._home, self._home_tries, self._home_t, self._home_steps = 'turn', HOME_TRIES, 0.0, None
            print('retour au cap : en cours')

    def supervise(self, cmd, dt):
        """Protection anti-chute et retour au cap. Renvoie (commande, mode)."""
        sample = self.imu.get() if self.imu is not None else None
        if sample is None:
            if self._home is not None:
                self._home = None
                print('retour au cap : annule, plus de mesure de la centrale')
            return cmd, 'marche'
        pitch, roll = sample[0], sample[1]
        yaw = self.imu.yaw
        standing = self.fsm.state == WalkState.STAND
        # Pose de reference : moyenne sur 0,5 s debout, au demarrage.
        if self._ref is None:
            if standing and self._ref_wait > 0.0:
                self._ref_wait -= dt
            elif standing:
                self._ref_samples.append((pitch, roll, yaw))
                if len(self._ref_samples) >= int(0.5 / dt):
                    self._ref = np.mean(self._ref_samples, axis=0)
                    print(f'pose de reference : tangage {self._ref[0]:+.1f}, roulis {self._ref[1]:+.1f}, cap {self._ref[2]:+.1f}')
            return cmd, 'marche'
        ep = pitch - self._ref[0]                                  # + = penche vers l'avant
        if self._ep_prev is not None:                              # vitesse de bascule, lissee
            self._ep_rate += 0.3 * ((ep - self._ep_prev) / dt - self._ep_rate)
        self._ep_prev = ep
        if self._push_fast and self._push_dir == 0 and standing:   # fin des pas rapides : temps normaux
            self.set_step_timing(fast=False)
        # Protection anti-chute (seuil releve pendant les pas de rattrapage).
        tilt = max(abs(ep), abs(roll - self._ref[1]))
        limit = SAFETY_TILT_DEG if np.any(cmd != 0.0) else SAFETY_PUSH_DEG   # seuil bas seulement en marche au stick
        self._safety_t = self._safety_t + dt if tilt > limit else 0.0
        if self._safety_t >= SAFETY_TIME and not self._safety_on:
            self._safety_on, self._home, self._push_dir = True, None, 0
            print(f'ARRET DE SECURITE : inclinaison {tilt:.0f} deg. Relachez les sticks pour reprendre.')
        if self._safety_on:
            if tilt < SAFETY_TILT_DEG / 2 and not np.any(cmd != 0.0):
                self._safety_on = False
                print('arret de securite leve')
            return np.zeros(3), 'securite'
        # Pas de rattrapage apres une poussee : 1 pas rapide (2 si la poussee est forte), puis arret.
        if np.any(cmd != 0.0) or self._home is not None:
            if self._push_dir != 0:
                print('pas de rattrapage : annules')
            self._push_dir, self._push_t = 0, 0.0
        elif self._push_dir == 0:
            # Pas de nouveau declenchement tant que le robot n'est pas reste immobile PUSH_REARM s :
            # l'arret du rattrapage fait lui-meme pencher le robot (sinon : va-et-vient sans fin).
            self._push_rearm = self._push_rearm - dt if standing else PUSH_REARM
            ep_pred = ep + PUSH_LEAD * self._ep_rate
            if (standing and self._push_rearm <= 0.0 and abs(ep_pred) > PUSH_ON_DEG
                    and abs(ep) > PUSH_MIN_DEG and ep * ep_pred > 0):
                self._push_t += dt
                if self._push_t >= PUSH_TIME:
                    self._push_dir, self._push_t = (1 if ep > 0 else -1), 0.0
                    self._push_steps = 2 if abs(ep_pred) > PUSH_BIG_DEG else 1
                    self._push_count, self._push_prev = 0, self.fsm.state
                    self.set_step_timing(fast=True)
                    print(f'poussee : {ep:+.1f} deg ({self._ep_rate:+.0f} deg/s), pas rapide vers ' + ('l\'avant' if ep > 0 else 'l\'arriere'))
            else:
                self._push_t = 0.0
        if self._push_dir != 0:
            if self.fsm.state == WalkState.SSP and self._push_prev != WalkState.SSP:
                self._push_count += 1
            self._push_prev = self.fsm.state
            if self._push_steps == 1 and abs(ep) > PUSH_BIG_DEG and ep * self._push_dir > 0:
                self._push_steps = 2
                print(f'poussee forte ({ep:+.1f} deg) : deuxieme pas')
            if self._push_count >= self._push_steps:
                # Assez de pas : on relache ; le planificateur fait le pas de fermeture et s'arrete.
                self._push_dir, self._push_rearm = 0, PUSH_REARM
                print(f'pas de rattrapage : {self._push_count} pas, ecart {ep:+.1f} deg')
                return np.zeros(3), 'marche'
            return np.array([self._push_dir * PUSH_CMD, 0.0, 0.0]), 'pousse'
        # Retour au cap : annule des que l'on touche un stick.
        if self._home is not None and np.any(cmd != 0.0):
            self._home = None
            print('retour au cap : annule par le stick')
        if self._home is None:
            return cmd, 'marche'
        err = (yaw - self._ref[2] + 180.0) % 360.0 - 180.0   # + = tourne a gauche du cap de depart
        # Pendant les pas, le cap oscille de +-5 deg : on ne mesure l'ecart qu'a l'arret,
        # puis on fait un nombre de pas calcule, et on corrige au besoin a l'essai suivant.
        if self._home == 'turn':
            if self._home_steps is None:                      # debut d'un essai : robot immobile
                if not standing:
                    return np.zeros(3), 'retour'              # attendre la fin du dernier pas
                if abs(err) <= HOME_TOL_DEG:
                    self._home = None
                    print(f'retour au cap : termine, ecart {err:+.1f} deg')
                    return np.zeros(3), 'marche'
                c = min(HOME_MAX_CMD, max(HOME_MIN_CMD, abs(err) / self._home_gain))
                self._home_c = float(np.copysign(c, err))     # + = tourne a droite (cap qui diminue)
                self._home_steps = int(np.ceil(0.8 * abs(err) / (self._home_gain * c)))   # vise 80 % : mieux vaut un essai de plus que depasser
                self._home_err0, self._home_count, self._home_prev = err, 0, self.fsm.state
                print(f'retour au cap : ecart {err:+.1f} deg, {self._home_steps} pas a {self._home_c:+.2f}')
            if self.fsm.state == WalkState.SSP and self._home_prev != WalkState.SSP:
                self._home_count += 1
            self._home_prev = self.fsm.state
            if self._home_count < self._home_steps:
                return np.array([0.0, 0.0, self._home_c]), 'retour'
            self._home, self._home_t = 'settle', 0.0
            return np.zeros(3), 'retour'
        # 'settle' : attendre l'arret complet, mesurer, apprendre combien le robot tourne vraiment.
        self._home_t = self._home_t + dt if standing else 0.0
        if self._home_t >= HOME_SETTLE:
            turned = self._home_err0 - err
            if self._home_count > 0 and abs(turned) > 1.0:
                self._home_gain = float(np.clip(abs(turned) / (self._home_count * abs(self._home_c)), 3.0, 30.0))
            if abs(err) <= HOME_TOL_DEG:
                self._home = None
                print(f'retour au cap : termine, ecart {err:+.1f} deg')
            elif self._home_tries > 1:
                self._home, self._home_tries, self._home_steps = 'turn', self._home_tries - 1, None
            else:
                self._home = None
                print(f'retour au cap : abandon, ecart {err:+.1f} deg')
        return np.zeros(3), 'retour'

    def planned_tilt(self):
        """Inclinaison du torse prevue par l'IK, en degres : (tangage, + = avant ; roulis, + = droite)."""
        x, y, z, w = self.q[3:7]
        gx, gy, gz = 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
        return np.degrees(np.arctan2(-gy, gz)), np.degrees(np.arctan2(-gx, gz))

    def wait_for_button(self, button_id):
        while self.joystick.get_button(button_id) == 0:
            for _ in pygame.event.get():
                pass
    
    def get_button(self, button: JoystickButton):
        for _ in pygame.event.get():
            pass
        return self.joystick.get_button(button.value) == 1
    
    def wait_for_start(self):
        self.wait_for_button(JoystickButton.START.value)
        while self.get_button(JoystickButton.START):
            time.sleep(0.01)

    def get_current_button(self):
        for _ in pygame.event.get():
            pass
        for button in JoystickButton:
            if button == JoystickButton.START:
                continue
            if self.joystick.get_button(button.value) == 1:
                return button
        return None
    
    def deploy_controller(self, motor_manager: MotorControllerManager, stand_qpos: np.ndarray):
        try:
            dt = 0.03
            rate_limiter = RateLimiter(frequency=1 / dt, warn=True)
            self.fsm.reset_standing()
            # Kept at 0. A left-yaw trim made the robot sway and did not
            # stop the right drift. Do not raise this until the hip is solid.
            self.fsm.footstep_generator.straight_yaw_trim = 0.0
            self.fsm.footstep_generator.lateral_comp_enabled = LATERAL_COMP_ENABLED
            self.fsm.footstep_generator.lateral_comp_forward = LATERAL_COMP_FORWARD_LEFT_M
            self.fsm.footstep_generator.lateral_comp_backward = LATERAL_COMP_BACKWARD_LEFT_M
            if LATERAL_COMP_ENABLED:
                print(
                    'lateral comp left: '
                    f'forward {LATERAL_COMP_FORWARD_LEFT_M * 100:.1f} cm, '
                    f'backward {LATERAL_COMP_BACKWARD_LEFT_M * 100:.1f} cm'
                )
            else:
                print('lateral comp off')
            self.q = self.default_q.copy()
            self._settle_t = None
            leg_idxs = motor_manager.get_sim_idxs('leg')
            # Depart : la pose envoyee par run() (articulations a 0). Pose debout visee : celle de l'IK,
            # torse redresse (TORSO_PITCH_TRIM_DEG dans robot.py). Le passage se fait au pas de STAND_SLEW.
            last_legs = np.array(stand_qpos[leg_idxs], dtype=float)
            stand_legs = self.default_q[7:].copy()
            ignore_cmd_until = time.perf_counter() + 0.5
            self.reset_supervisor()
            tick = 0
            t_report = time.perf_counter()
            while True:
                t_tick = time.perf_counter()
                current_button = self.get_current_button()
                rb = current_button == JoystickButton.RB
                if rb and not self._rb_prev:
                    self.toggle_home()
                self._rb_prev = rb
                if current_button is not None and not rb:
                    return current_button
                cmd = self.get_joystick_cmd()
                if time.perf_counter() < ignore_cmd_until:
                    cmd = np.zeros(3)
                cmd, mode = self.supervise(np.array(cmd, dtype=float), dt)
                self.fsm.set_cmd(cmd)
                self.fsm.on_tick()

                q_full = np.array(stand_qpos, dtype=float).copy()
                last_legs = self.leg_targets(last_legs, stand_legs, dt)
                q_full[leg_idxs] = last_legs

                motor_manager.set_positions(q_full, 0, 30)
                calc_ms = (time.perf_counter() - t_tick) * 1000.0   # duree du calcul de ce tick
                rate_limiter.sleep()

                imu_txt = ''
                if self.imu is not None:
                    sample = self.imu.get()
                    if sample is None:
                        imu_txt = '  centrale : pas de mesure'
                    else:
                        pitch, roll, pitch_rate, roll_rate, age = sample
                        imu_txt = f'  tangage {pitch:+5.1f}  roulis {roll:+5.1f}  cap {self.imu.yaw:+6.1f}  (age {age * 1000:.0f} ms)'
                        c = self.fsm.cmd
                        plan_pitch, plan_roll = self.planned_tilt()
                        self.imu_log.write(
                            f'{time.perf_counter():.3f},{self.fsm.state.name},{c[0]:.2f},{c[1]:.2f},{c[2]:.2f},'
                            f'{pitch:.2f},{roll:.2f},{pitch_rate:.2f},{roll_rate:.2f},{age:.3f},'
                            f'{plan_pitch:.2f},{plan_roll:.2f},{self.imu.i2c_errors},{calc_ms:.1f},{self.imu.yaw:.2f},{mode}\n')

                tick += 1
                if tick % 15 == 0:
                    now = time.perf_counter()
                    hz = 15.0 / max(now - t_report, 1e-6)
                    t_report = now
                    print(f'{hz:.1f} Hz  state {self.fsm.state.name}  {mode}  cmd {self.fsm.cmd}{imu_txt}')
                    if self.imu_log is not None:
                        self.imu_log.flush()
        except KeyboardInterrupt:
            motor_manager.disable_torque()
    
    def teleop(self, 
               leader_motor_manager: MotorControllerManager, 
               follower_motor_manager: MotorControllerManager):
        leader_motor_manager.disable_torque()
        follower_motor_manager.set_positions(np.zeros(follower_motor_manager.n_motors), 0, 30)
        time.sleep(1.)
        raw_zero_q = follower_motor_manager.get_raw_state()[0]
        while True:
            if self.get_button(JoystickButton.START):
                follower_motor_manager.set_positions(np.zeros(follower_motor_manager.n_motors), 0, 30)
                time.sleep(1.)
                break
            q, _ = leader_motor_manager.get_raw_state()
            full_q = raw_zero_q.copy()
            full_q[follower_motor_manager.get_sim_idxs('arm')] = q
            follower_motor_manager.set_raw_positions(full_q, 0, 30)
            time.sleep(0.03)
    
    def run(self, is_remote, host, config_path='sundaya1_real_config_short.yaml'):
        cfg = load_config(config_path)
        motor_manager = MotorControllerManager(
            cfg.real_config.n_motors, 
            cfg.real_config.motor_controllers, 
            cfg.real_config.calibration_file, 
            mode=0,
            remote=is_remote,
            sender=is_remote,
            host=host,
        )
        input('start>')
        if self.enable_teleop:
            leader_cfg = load_config('sundaya1_real_config_short_arm.yaml')
            leader_motor_manager = MotorControllerManager(
                leader_cfg.real_config.n_motors, 
                leader_cfg.real_config.motor_controllers, 
                leader_cfg.real_config.calibration_file, 
                mode=0
            )
        motor_manager.set_positions(cfg.default_qpos, 0, 30)
        self.wait_for_start()
        print('standing — push the stick to walk')
        while True:
            button = self.deploy_controller(motor_manager, cfg.default_qpos)
            print(f'Button {button} pressed')
            self.fsm.reset_standing()
            self.q = self.default_q.copy()
            motor_manager.set_positions(cfg.default_qpos, 0, 30)
            if button == JoystickButton.A:
                motor_manager.play_recording(get_asset_path('motions/waving_motion.json'))
                print('motion done, press START to stand, then push the stick to walk')
                self.wait_for_start()
            elif button == JoystickButton.B:
                motor_manager.play_recording(get_asset_path('motions/lay_down_motion.json'))
                will_exit = True
                while True:
                    if self.get_button(JoystickButton.START):
                        will_exit = False
                        break
                    elif self.get_button(JoystickButton.X):
                        break
                    time.sleep(0.1)
                if will_exit:
                    print('end demo')
                    break
                else:
                    motor_manager.play_recording(get_asset_path('motions/stand_up_motion.json'))
            elif button == JoystickButton.Y:
                if self.enable_teleop:
                    self.teleop(leader_motor_manager, motor_manager)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--remote', action='store_true', help='Whether to run in remote mode')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Host IP for remote mode')
    parser.add_argument('--teleop', action='store_true', help='Whether to run teleoperation demo')
    parser.add_argument('--config', type=str, default='sundaya1_real_config_short.yaml', help='Path to configuration file')
    parser.add_argument('--imu', action='store_true', help='Lire la centrale inertielle du Pi (scripts/imu_sender.py doit y tourner)')
    args = parser.parse_args()
    
    imu = ImuClient(args.host) if args.imu else None
    demo = Demo(enable_teleop=args.teleop, imu=imu)
    demo.run(is_remote=args.remote, host=args.host, config_path=args.config)