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
        self.imu_log = None
        if imu is not None:
            name = time.strftime('imu_log_%Y%m%d_%H%M%S.csv')
            self.imu_log = open(name, 'w')
            self.imu_log.write('t,etat,cmd_avant,cmd_cote,cmd_rot,tangage,roulis,vit_tangage,vit_roulis,age,'
                               'tangage_prevu,roulis_prevu,rejets,calcul_ms,cap\n')
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
            stand_legs = np.array(stand_qpos[leg_idxs], dtype=float)
            last_legs = stand_legs.copy()
            ignore_cmd_until = time.perf_counter() + 0.5
            tick = 0
            t_report = time.perf_counter()
            while True:
                t_tick = time.perf_counter()
                current_button = self.get_current_button()
                if current_button is not None:
                    return current_button
                cmd = self.get_joystick_cmd()
                if time.perf_counter() < ignore_cmd_until:
                    cmd = np.zeros(3)
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
                            f'{plan_pitch:.2f},{plan_roll:.2f},{self.imu.i2c_errors},{calc_ms:.1f},{self.imu.yaw:.2f}\n')

                tick += 1
                if tick % 15 == 0:
                    now = time.perf_counter()
                    hz = 15.0 / max(now - t_report, 1e-6)
                    t_report = now
                    print(f'{hz:.1f} Hz  state {self.fsm.state.name}  cmd {self.fsm.cmd}{imu_txt}')
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