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


class JoystickButton(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    START = 7

class Demo(Robot):
    def __init__(self, enable_teleop=False):
        super().__init__()
        self.enable_teleop = enable_teleop

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
            self.q = self.default_q.copy()
            leg_idxs = motor_manager.get_sim_idxs('leg')
            last_legs = np.array(stand_qpos[leg_idxs], dtype=float).copy()
            tick = 0
            t_report = time.perf_counter()
            while True:
                current_button = self.get_current_button()
                if current_button is not None:
                    return current_button
                self.fsm.set_cmd(self.get_joystick_cmd())
                self.fsm.on_tick()

                q_full = np.array(stand_qpos, dtype=float).copy()
                if self.fsm.state != WalkState.STAND:
                    self.q, success = self.ik(self._get_targets())
                    last_legs = self.safe_leg_q(self.q, last_legs, success)
                    q_full[leg_idxs] = last_legs
                else:
                    last_legs = np.array(stand_qpos[leg_idxs], dtype=float).copy()
                    self.q = self.default_q.copy()

                motor_manager.set_positions(q_full, 0, 30)
                rate_limiter.sleep()

                tick += 1
                if tick % 15 == 0:
                    now = time.perf_counter()
                    hz = 15.0 / max(now - t_report, 1e-6)
                    t_report = now
                    print(f'{hz:.1f} Hz  state {self.fsm.state.name}  cmd {self.fsm.cmd}')
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
    args = parser.parse_args()
    
    demo = Demo(enable_teleop=args.teleop)
    demo.run(is_remote=args.remote, host=args.host, config_path=args.config)