"""Train a new simulation policy; does not connect to the real robot."""

import argparse
import json
from pathlib import Path

import torch
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CheckpointCallback,
    EvalCallback,
    CallbackList,
)
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
)
from stable_baselines3.common.logger import configure

from vibe_env import VibeEnv
from episode_rules import verify_rules


def verify_absorbing_env(sdk):
    """Force failure state to verify its integration, without training."""

    test = VibeEnv(
        sdk,
        command=[0.0, 0.0, 0.0],
        randomize=False,
        stage="stepping",
    )

    try:
        test.reset(seed=0)

        test.rules.steps = 38
        test.elapsed = test.rules.elapsed
        test.rules.fail()

        test.failure_info = {
            "fallen": True,
            "stalled": False,
            "distance_forward": 0.0,
            "mean_forward_speed": 0.0,
            "lateral_distance": 0.0,
            "heading_error_deg": 0.0,
            "mean_slip_speed": 0.0,
        }

        test.swing_successes[:] = 1.0
        test.swing_trials[:] = 1.0

        q = test.data.qpos.copy()
        sim_time = test.data.time

        penalties = 0

        while not test.rules.done:
            obs, reward, terminated, truncated, info = test.step(
                np.zeros(12)
            )

            if (
                reward != -1.0
                or truncated
                or terminated != test.rules.done
            ):
                raise AssertionError(
                    "Traitement dechec incoherent."
                )

            if (
                obs.shape != (51,)
                or obs[-1] != 1.0
            ):
                raise AssertionError(
                    "Observation dechec incorrecte."
                )

            penalties += 1

        if (
            penalties != 362
            or test.elapsed != 12.0
            or not np.isclose(
                info["survival_time"],
                1.14,
            )
        ):
            raise AssertionError(
                "Penalites restantes ou duree incorrectes."
            )

        if (
            test.data.time != sim_time
            or not np.array_equal(
                q,
                test.data.qpos,
            )
        ):
            raise AssertionError(
                "La physique continue apres lechec."
            )

        if (
            max(
                info["right_swing_success"],
                info["left_swing_success"],
            )
            > 0.02
        ):
            raise AssertionError(
                "Un seul appui a encore un pourcentage trompeur."
            )

        print(
            "CONTROLE ECHEC : "
            "362 penalites restantes, "
            "physique figee, "
            "taux dappui calcule sur 12 s.",
            flush=True,
        )

    finally:
        test.close()


class ProgressReport(BaseCallback):
    """Report real distance in a fixed forward trial, not only its reward."""

    def __init__(
        self,
        sdk,
        frequency,
        output,
        stage,
    ):
        super().__init__()

        self.stage = stage

        command = (
            [0.0, 0.0, 0.0]
            if stage == "stepping"
            else [0.1, 0.0, 0.0]
        )

        self.test = VibeEnv(
            sdk,
            command=command,
            randomize=False,
            stage=stage,
        )

        self.frequency = frequency
        self.output = Path(output)
        self.best_key = None

    def _on_training_start(self):
        print(
            "Evaluation initiale de la politique recuperee :",
            flush=True,
        )

        self._evaluate()

    def _on_step(self):
        if self.n_calls % self.frequency:
            return True

        self._evaluate()

        return True

    def _evaluate(self):
        obs, _ = self.test.reset(
            seed=1234
        )

        finished = False

        while not finished:
            action, _ = self.model.predict(
                obs,
                deterministic=True,
            )

            obs, _, terminated, truncated, info = self.test.step(
                action
            )

            finished = (
                terminated
                or truncated
            )

        reason = (
            "chute"
            if info["fallen"]
            else (
                "immobilite"
                if info["stalled"]
                else "12 secondes atteintes"
            )
        )

        print(
            f"\nDEPLACEMENT : "
            f"{info['distance_forward']:.3f} m "
            f"pendant {info['survival_time']:.2f} s de survie; "
            f"vitesse moyenne {info['mean_forward_speed']:.3f} m/s; "
            f"derive laterale {info['lateral_distance']:.3f} m; "
            f"ecart cap {info['heading_error_deg']:.1f} deg; "
            f"glissement moyen {info['mean_slip_speed']:.3f} m/s; "
            f"fin : {reason}\n",
            flush=True,
        )

        right = info.get(
            "right_swing_success",
            0.0,
        )

        left = info.get(
            "left_swing_success",
            0.0,
        )

        print(
            f"PIEDS : "
            f"leve droit {100 * info.get('right_lift_m', 0):.1f} cm; "
            f"leve gauche {100 * info.get('left_lift_m', 0):.1f} cm; "
            f"appuis alternes reussis droit {100 * right:.0f}% / "
            f"gauche {100 * left:.0f}%; "
            f"evaluation {self.test.elapsed:.2f} s; "
            f"fin : {reason}",
            flush=True,
        )

        print(
            f"SURVIE : "
            f"{info['survival_time']:.2f} / 12.00 s; "
            f"cycles complets observes avant echec "
            f"{info['cycles_observed_alive']}; "
            f"pourcentages PIEDS calcules sur TOUTE l'evaluation.",
            flush=True,
        )

        self.logger.record(
            "movement/distance_m",
            info["distance_forward"],
        )

        self.logger.record(
            "movement/mean_speed_m_s",
            info["mean_forward_speed"],
        )

        self.logger.record(
            "movement/stalled",
            float(info["stalled"]),
        )

        self.logger.record(
            "movement/fallen",
            float(info["fallen"]),
        )

        self.logger.record(
            "movement/heading_error_deg",
            info["heading_error_deg"],
        )

        self.logger.record(
            "movement/mean_slip_speed",
            info["mean_slip_speed"],
        )

        self.logger.record(
            "feet/right_swing_success",
            right,
        )

        self.logger.record(
            "feet/left_swing_success",
            left,
        )

        self.logger.record(
            "movement/survival_s",
            info["survival_time"],
        )

        score = (
            min(
                info["distance_forward"],
                1.2,
            )
            - 2.0
            * abs(
                info["lateral_distance"]
            )
            - 0.3
            * abs(
                np.radians(
                    info["heading_error_deg"]
                )
            )
            - 3.0
            * info["mean_slip_speed"]
            - float(
                info["fallen"]
            )
            - 0.5
            * float(
                info["stalled"]
            )
        )

        if self.stage == "stepping":
            score = (
                2.0
                * min(
                    right,
                    left,
                )
                + 0.5
                * (
                    right
                    + left
                )
                - 3.0
                * info["mean_slip_speed"]
                - float(
                    info["fallen"]
                )
                - abs(
                    info["distance_forward"]
                )
                - abs(
                    info["lateral_distance"]
                )
                - 0.3
                * abs(
                    np.radians(
                        info["heading_error_deg"]
                    )
                )
            )

        key = (
            not info["failed"],
            round(
                info["survival_time"],
                6,
            ),
            score,
        )

        if (
            self.best_key is None
            or key > self.best_key
        ):
            self.best_key = key

            self.model.save(
                str(
                    self.output
                    / "best_trajectory"
                )
            )

            print(
                "Meilleure trajectoire conservee : "
                "best_trajectory.zip "
                "(a verifier visuellement).",
                flush=True,
            )

        valid = (
            not info["failed"]
            and info["cycles_observed_alive"] >= 6
            and min(right, left) >= 0.6
            and abs(
                info["distance_forward"]
            )
            < 0.10
            and abs(
                info["lateral_distance"]
            )
            < 0.05
            and abs(
                info["heading_error_deg"]
            )
            < 15.0
            and info["mean_slip_speed"]
            < 0.05
        )

        print(
            "CRITERE APPUIS : "
            + (
                "atteint, verification visuelle requise"
                if valid
                else "non atteint"
            ),
            flush=True,
        )

    def _on_training_end(self):
        self.test.close()


def transfer_actor(source_path, target):
    """Keep old motor policy, add zero-weight heading inputs, reset value net."""

    source = PPO.load(
        source_path,
        device="cpu",
    )

    if (
        source.observation_space.shape
        not in (
            (47,),
            (49,),
            (51,),
        )
        or source.action_space.shape != (12,)
    ):
        raise ValueError(
            "--warmstart attend 47, 49 ou 51 observations et 12 actions."
        )

    old = source.policy.state_dict()
    new = target.policy.state_dict()

    copied = 0

    for key in new:
        if not (
            key.startswith(
                "mlp_extractor.policy_net."
            )
            or key.startswith(
                "action_net."
            )
            or key == "log_std"
        ):
            continue

        if key not in old:
            raise ValueError(
                f"Parametre du controleur manquant : {key}"
            )

        if (
            new[key].shape
            == old[key].shape
        ):
            new[key] = old[key].clone()

        elif (
            key
            == "mlp_extractor.policy_net.0.weight"
            and new[key].shape
            == (128, 51)
            and old[key].shape
            in (
                (128, 47),
                (128, 49),
            )
        ):
            new[key] = torch.zeros_like(
                new[key]
            )

            new[key][
                :,
                :old[key].shape[1],
            ] = old[key]

        else:
            raise ValueError(
                f"Architecture incompatible : {key}"
            )

        copied += 1

    if copied != 7:
        raise ValueError(
            f"Architecture inattendue : "
            f"{copied} parametres du controleur."
        )

    target.policy.load_state_dict(
        new,
        strict=True,
    )

    nobs = (
        source.observation_space.shape[0]
    )

    sample = (
        np.random.default_rng(0)
        .normal(
            0.0,
            0.2,
            (
                16,
                nobs,
            ),
        )
        .astype(
            np.float32
        )
    )

    old_action, _ = source.predict(
        sample,
        deterministic=True,
    )

    extended = np.concatenate(
        [
            sample,
            np.zeros(
                (
                    16,
                    51 - nobs,
                ),
                dtype=np.float32,
            ),
        ],
        axis=1,
    )

    new_action, _ = target.predict(
        extended,
        deterministic=True,
    )

    if not np.allclose(
        old_action,
        new_action,
        atol=1e-5,
        rtol=1e-5,
    ):
        raise ValueError(
            "Le transfert ne preserve pas les commandes; "
            "entrainement annule."
        )

    print(
        "Reseau recupere; equivalence des sorties normalisees verifiee.",
        flush=True,
    )

    print(
        "Entrees ajoutees : temps restant et etat dechec. "
        "Amplitude des genoux identique a V4.",
        flush=True,
    )

    print(
        "Evaluation de recompense et optimiseur reinitialises "
        "pour le nouvel objectif.",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--sdk",
        default=str(
            Path(__file__)
            .resolve()
            .parent
            .parent
        ),
    )

    parser.add_argument(
        "--steps",
        type=int,
        default=50_000,
    )

    parser.add_argument(
        "--envs",
        type=int,
        default=2,
    )

    parser.add_argument(
        "--out",
        default=str(
            Path(__file__)
            .resolve()
            .parent
            / "runs/essai8"
        ),
    )

    parser.add_argument(
        "--audit-only",
        action="store_true",
        help=(
            "Verifications et evaluation initiale "
            "sans apprentissage"
        ),
    )

    parser.add_argument(
        "--stage",
        choices=[
            "stepping",
            "walking",
        ],
        default="stepping",
    )

    parser.add_argument(
        "--resume",
        help=(
            "Reprendre une politique V8 "
            "de la meme etape"
        ),
    )

    parser.add_argument(
        "--warmstart",
        help=(
            "Recuperer les commandes dune "
            "politique existante (.zip)"
        ),
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42,
    )

    args = parser.parse_args()

    print(
        "CONTROLE DU SCORE :",
        verify_rules(),
        flush=True,
    )

    if (
        args.envs < 1
        or args.steps < 1
    ):
        parser.error(
            "--envs et --steps doivent etre positifs"
        )

    if (
        args.resume
        and args.warmstart
    ):
        parser.error(
            "Choisir --resume ou --warmstart, pas les deux."
        )

    for filename in (
        args.resume,
        args.warmstart,
    ):
        if (
            filename
            and not Path(filename).is_file()
        ):
            parser.error(
                f"Sauvegarde introuvable : {filename}"
            )

    sdk = Path(
        args.sdk
    ).resolve()

    out = Path(
        args.out
    ).resolve()

    out.mkdir(
        parents=True,
        exist_ok=True,
    )

    torch.set_num_threads(1)

    verify_absorbing_env(
        sdk
    )

    command = (
        [0.0, 0.0, 0.0]
        if args.stage == "stepping"
        else [0.1, 0.0, 0.0]
    )

    probe = VibeEnv(
        sdk,
        command=command,
        stage=args.stage,
    )

    print(
        "Verification du nouveau programme : "
        "interface Gymnasium et mapping moteurs.",
        flush=True,
    )

    check_env(
        probe,
        warn=True,
    )

    print(
        f"Modele complet : {probe.model.nu} moteurs, "
        f"apprentissage : 12 jambes; "
        f"hauteur initiale {probe.spawn_height:.3f} m.",
        flush=True,
    )

    probe.close()

    config = {
        "sdk": str(sdk),
        "seed": args.seed,
        "envs": args.envs,
        "steps": args.steps,
        "stage": args.stage,
        "reward_version": 8,
        "warmstart": args.warmstart,
        "simulation_only": True,
        "control_dt": 0.03,
        "provisional_torque_limit_Nm": 2.0,
        "provisional_target_slew_rad_s": 2.0,
    }

    (
        out
        / "config.json"
    ).write_text(
        json.dumps(
            config,
            indent=2,
        ),
        encoding="utf-8",
    )

    def make_env():
        return Monitor(
            VibeEnv(
                sdk,
                command=command,
                stage=args.stage,
            )
        )

    env = (
        DummyVecEnv(
            [make_env]
        )
        if args.envs == 1
        else SubprocVecEnv(
            [
                make_env
                for _ in range(
                    args.envs
                )
            ],
            start_method="spawn",
        )
    )

    env.seed(
        args.seed
    )

    def make_eval():
        return Monitor(
            VibeEnv(
                sdk,
                command=command,
                randomize=False,
                stage=args.stage,
            )
        )

    eval_env = (
        DummyVecEnv(
            [make_eval]
        )
        if args.envs == 1
        else SubprocVecEnv(
            [make_eval],
            start_method="spawn",
        )
    )

    eval_env.seed(
        1234
    )

    callbacks = CallbackList(
        [
            CheckpointCallback(
                save_freq=max(
                    25_000
                    // args.envs,
                    1,
                ),
                save_path=str(
                    out
                    / "checkpoints"
                ),
                name_prefix="marche",
            ),
            EvalCallback(
                eval_env,
                best_model_save_path=str(
                    out
                    / "best"
                ),
                log_path=str(
                    out
                    / "evaluation"
                ),
                eval_freq=max(
                    25_000
                    // args.envs,
                    1,
                ),
                n_eval_episodes=4,
                deterministic=True,
            ),
            ProgressReport(
                sdk,
                max(
                    25_000
                    // args.envs,
                    1,
                ),
                out,
                args.stage,
            ),
        ]
    )

    if args.resume:
        agent = PPO.load(
            args.resume,
            env=env,
            device="cpu",
            tensorboard_log=str(
                out
                / "tensorboard"
            ),
        )

    else:
        agent = PPO(
            "MlpPolicy",
            env,
            device="cpu",
            verbose=1,
            seed=args.seed,
            n_steps=1024,
            batch_size=128,

            # V8 : apprentissage volontairement plus lent
            # pour conserver davantage la politique V5.
            learning_rate=1e-4,

            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=0.002,
            clip_range=0.2,
            target_kl=0.015,

            policy_kwargs={
                "net_arch": {
                    "pi": [
                        128,
                        128,
                    ],
                    "vf": [
                        128,
                        128,
                    ],
                }
            },

            tensorboard_log=str(
                out
                / "tensorboard"
            ),
        )

        if args.warmstart:
            transfer_actor(
                args.warmstart,
                agent,
            )

    if args.audit_only:
        agent.set_logger(
            configure(
                str(
                    out
                    / "audit"
                ),
                ["stdout"],
            )
        )

        report = ProgressReport(
            sdk,
            1,
            out,
            args.stage,
        )

        report.init_callback(
            agent
        )

        try:
            report._evaluate()

        finally:
            report.test.close()
            env.close()
            eval_env.close()

        print(
            "AUDIT TERMINE : aucune mise a jour du reseau.",
            flush=True,
        )

        return

    print(
        "Apprentissage en cours. "
        "Ctrl+C enregistre la progression. "
        "Aucun moteur reel commande.",
        flush=True,
    )

    try:
        agent.learn(
            total_timesteps=args.steps,
            callback=callbacks,
            reset_num_timesteps=not bool(
                args.resume
            ),
        )

    except KeyboardInterrupt:
        print(
            "\nInterruption : sauvegarde de la progression.",
            flush=True,
        )

    finally:
        agent.save(
            str(
                out
                / "last_model"
            )
        )

        env.close()
        eval_env.close()

        print(
            f"Resultat enregistre : "
            f"{out / 'last_model.zip'}",
            flush=True,
        )


if __name__ == "__main__":
    main()