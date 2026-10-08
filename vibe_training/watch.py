"""Preview an experimental trained policy in MuJoCo; simulation only."""
import argparse
from pathlib import Path
import time
import mujoco.viewer
from stable_baselines3 import PPO
from vibe_env import VibeEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sdk', default=str(Path(__file__).resolve().parent.parent))
    parser.add_argument('--model', default=str(Path(__file__).resolve().parent / 'runs/essai5/best_trajectory.zip'))
    parser.add_argument('--stage', choices=['stepping', 'walking'], default='stepping')
    parser.add_argument('--forward', type=float, default=None)
    parser.add_argument('--yaw', type=float, default=0.)
    args = parser.parse_args()
    path = Path(args.model)
    if not path.is_file():
        parser.error(f'Modele appris introuvable : {path}. Utiliser --model avec un fichier enregistre.')
    forward = args.forward if args.forward is not None else (0. if args.stage == 'stepping' else 0.10)
    env = VibeEnv(args.sdk, command=[forward, 0., args.yaw], randomize=False, stage=args.stage)
    agent = PPO.load(str(path), device='cpu')
    obs, _ = env.reset(seed=0)
    if agent.observation_space.shape != (51,):
        parser.error('Ce visualiseur attend une politique V5 : 51 observations.')
    print('Fermer la fenetre pour quitter. Redemarrage automatique apres une chute.')
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.distance = 1.4
        viewer.cam.elevation = -15
        report_at = time.perf_counter()
        while viewer.is_running():
            start = time.perf_counter()
            action, _ = agent.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            viewer.cam.lookat[:] = env.data.xpos[env.base]
            viewer.sync()
            if time.perf_counter() - report_at >= 2.:
                print(f"vitesse filtree {info.get('forward_speed', 0.):.3f} m/s; "
                      f"distance {info.get('distance_forward', 0.):.3f} m; "
                      f"vitesse moyenne {info.get('mean_forward_speed', 0.):.3f} m/s; "
                      f"derive {info.get('lateral_distance', 0.):.3f} m; "
                      f"cap {info.get('heading_error_deg', 0.):.1f} deg; "
                      f"verticalite {info.get('upright', 0.):.2f}")
                print(f"PIEDS : droit {100 * info.get('right_lift_m', 0.):.1f} cm; "
                      f"gauche {100 * info.get('left_lift_m', 0.):.1f} cm; "
                      f"appuis alternes {100 * info.get('right_swing_success', 0.):.0f}% / "
                      f"{100 * info.get('left_swing_success', 0.):.0f}%")
                report_at = time.perf_counter()
            if terminated or truncated or info.get('failed'):
                if info.get('failed'):
                    print('Chute : nouvel essai.' if info.get('fallen') else 'Deplacement insuffisant : nouvel essai.')
                obs, _ = env.reset()
            time.sleep(max(0., env.dt - (time.perf_counter() - start)))
    env.close()


if __name__ == '__main__':
    main()
