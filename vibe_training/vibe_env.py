"""Experimental, simulation-only locomotion environment for Sunday A1 short.

No SDK controller, hardware connection, or existing gait is used.
Body +Y is forward, +X is lateral. Observations use joint state and IMU-like
signals; exact linear velocity and ground contacts are used only for rewards.
"""

from pathlib import Path

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from episode_rules import EpisodeRules


LEGS = tuple(
    f"{side}_{joint}"
    for side in ("right", "left")
    for joint in (
        "hip_roll",
        "hip_pitch",
        "hip_yaw",
        "knee",
        "ankle_pitch",
        "ankle_roll",
    )
)


class VibeEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, sdk, command=None, randomize=True, stage="stepping"):
        if stage not in ("stepping", "walking"):
            raise ValueError("Etape attendue : stepping ou walking.")

        self.stage = stage

        self.scene = (
            Path(sdk).resolve()
            / "viberobotics/assets/mujoco/SundayA1_short_new/scene.xml"
        )

        if not self.scene.is_file():
            raise FileNotFoundError(f"Modele introuvable : {self.scene}")

        self.model = mujoco.MjModel.from_xml_path(str(self.scene))
        self.model.opt.timestep = 0.002
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.data = mujoco.MjData(self.model)

        self.dt = 0.03
        self.substeps = 15
        self.randomize = randomize

        self.fixed_command = (
            None if command is None else np.asarray(command, dtype=float)
        )

        self.base = self._id(mujoco.mjtObj.mjOBJ_BODY, "body")
        self.floor = self._id(mujoco.mjtObj.mjOBJ_GEOM, "floor")

        self.feet = [
            self._id(
                mujoco.mjtObj.mjOBJ_BODY,
                f"{side}_foot0120",
            )
            for side in ("right", "left")
        ]

        self.joints = np.array(
            [
                self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in LEGS
            ]
        )

        self.qadr = self.model.jnt_qposadr[self.joints]
        self.vadr = self.model.jnt_dofadr[self.joints]

        self.aids = []

        for jid in self.joints:
            hits = np.flatnonzero(
                self.model.actuator_trnid[:, 0] == jid
            )

            if len(hits) != 1:
                raise ValueError(
                    "Chaque articulation doit avoir exactement un actionneur."
                )

            self.aids.append(int(hits[0]))

        self.aids = np.asarray(self.aids)

        if self.model.nu != 25 or self.model.nq != 32:
            raise ValueError(
                "Ce fichier attend le modele complet a 25 articulations."
            )

        if np.any(
            self.model.actuator_biastype[self.aids]
            != mujoco.mjtBias.mjBIAS_AFFINE
        ):
            raise ValueError("Actionneurs de position attendus.")

        self.action_scale = np.full(12, 0.65)
        self.action_scale[[3, 9]] = 1.2

        self.low = -self.action_scale.copy()
        self.high = self.action_scale.copy()

        for i, jid in enumerate(self.joints):
            if self.model.jnt_limited[jid]:
                self.low[i] = max(
                    self.low[i],
                    self.model.jnt_range[jid, 0],
                )

                self.high[i] = min(
                    self.high[i],
                    self.model.jnt_range[jid, 1],
                )

        # Valeurs provisoires de simulation, pas des caracteristiques STS3215 mesurees.
        self.model.actuator_forcelimited[:] = 1
        self.model.actuator_forcerange[:] = [-2.0, 2.0]

        self.nominal_gain = self.model.actuator_gainprm.copy()
        self.nominal_bias = self.model.actuator_biasprm.copy()
        self.nominal_friction = self.model.geom_friction.copy()

        self.action_space = spaces.Box(
            -1.0,
            1.0,
            (12,),
            np.float32,
        )

        self.observation_space = spaces.Box(
            -np.inf,
            np.inf,
            (51,),
            np.float32,
        )

        # V8 : conserver toute la geometrie de collision de chaque pied.
        # La hauteur sera le point le plus bas de toute la semelle.
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        bottoms = []
        self.sole_vertices = []

        for bid in self.feet:
            world_points = []

            for gid in np.flatnonzero(self.model.geom_bodyid == bid):
                if self.model.geom_contype[gid] == 0:
                    continue

                if self.model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
                    continue

                mesh = self.model.geom_dataid[gid]
                start = self.model.mesh_vertadr[mesh]
                count = self.model.mesh_vertnum[mesh]

                verts = self.model.mesh_vert[start:start + count]

                geom_rot = self.data.geom_xmat[gid].reshape(3, 3)

                world = (
                    verts @ geom_rot.T
                    + self.data.geom_xpos[gid]
                )

                world_points.append(world)

            if not world_points:
                raise ValueError(
                    "Maillage de collision du pied manquant."
                )

            points = np.concatenate(world_points)

            bottoms.append(
                float(points[:, 2].min())
            )

            body_rot = self.data.xmat[bid].reshape(3, 3)

            local_points = (
                points - self.data.xpos[bid]
            ) @ body_rot

            self.sole_vertices.append(local_points)

        # Pose neutre : point le plus bas des pieds a 2 mm du sol.
        self.spawn_height = float(
            0.002 - min(bottoms)
        )

        self.period = 1.1

        self.reset(seed=0)

    def _id(self, kind, name):
        idx = mujoco.mj_name2id(
            self.model,
            kind,
            name,
        )

        if idx < 0:
            raise ValueError(
                f"Element MuJoCo manquant : {name}"
            )

        return idx

    def _kinematics(self):
        rotation = self.data.xmat[self.base].reshape(3, 3)

        vel = np.zeros(6)

        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_BODY,
            self.base,
            vel,
            1,
        )

        gravity = rotation.T @ np.array(
            [0.0, 0.0, -1.0]
        )

        return vel, gravity

    def _obs(self):
        if self.rules.failed:
            return np.concatenate(
                (
                    np.zeros(49),
                    [
                        self.rules.remaining,
                        1.0,
                    ],
                )
            ).astype(np.float32)

        velocity, gravity = self._kinematics()

        phase = (
            2.0
            * np.pi
            * self.elapsed
            / self.period
        )

        return np.concatenate(
            (
                velocity[:3] * 0.25,
                gravity,
                self.data.qpos[self.qadr],
                self.data.qvel[self.vadr] * 0.05,
                self.previous_action,
                self.command,
                [
                    np.sin(phase),
                    np.cos(phase),
                ],
                [
                    np.sin(self._heading_error()),
                    np.cos(self._heading_error()) - 1.0,
                ],
                [
                    self.rules.remaining,
                    0.0,
                ],
            )
        ).astype(np.float32)

    def _reference_axes(self):
        angle = self.command[2] * self.elapsed

        c = np.cos(angle)
        s = np.sin(angle)

        forward = (
            np.array(
                [
                    [c, -s],
                    [s, c],
                ]
            )
            @ self.initial_forward
        )

        lateral = np.array(
            [
                forward[1],
                -forward[0],
            ]
        )

        return forward, lateral

    def _heading_error(self):
        desired, _ = self._reference_axes()

        current = self.data.xmat[
            self.base
        ].reshape(3, 3)[:2, 1]

        return float(
            np.arctan2(
                desired[0] * current[1]
                - desired[1] * current[0],
                desired @ current,
            )
        )

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        mujoco.mj_resetData(
            self.model,
            self.data,
        )

        self.model.actuator_gainprm[:] = self.nominal_gain
        self.model.actuator_biasprm[:] = self.nominal_bias
        self.model.geom_friction[:] = self.nominal_friction

        if self.randomize:
            factor = self.np_random.uniform(
                0.9,
                1.1,
            )

            self.model.actuator_gainprm[:, 0] *= factor
            self.model.actuator_biasprm[:, 1:] *= factor

            self.model.geom_friction[:, 0] *= (
                self.np_random.uniform(
                    0.85,
                    1.15,
                )
            )

        self.data.qpos[2] = self.spawn_height

        if self.randomize:
            self.data.qpos[self.qadr] = (
                self.np_random.uniform(
                    -0.015,
                    0.015,
                    12,
                )
            )

        self.data.ctrl[:] = 0.0

        self.previous_action = np.zeros(12)
        self.target = np.zeros(12)

        self.elapsed = 0.0

        self.rules = EpisodeRules()
        self.failure_info = None

        if self.fixed_command is not None:
            self.command = self.fixed_command.copy()

        else:
            mode = self.np_random.choice(
                4,
                p=[
                    0.1,
                    0.45,
                    0.25,
                    0.2,
                ],
            )

            fwd = (
                0.0
                if mode in (0, 3)
                else self.np_random.uniform(
                    0.04,
                    0.16,
                )
                * (
                    1
                    if mode == 1
                    else -1
                )
            )

            yaw = (
                self.np_random.uniform(
                    -0.5,
                    0.5,
                )
                if (
                    mode == 3
                    or self.np_random.random() < 0.3
                )
                else 0.0
            )

            self.command = np.array(
                [
                    fwd,
                    0.0,
                    yaw,
                ]
            )

        mujoco.mj_forward(
            self.model,
            self.data,
        )

        self.start_xy = self.data.xpos[
            self.base,
            :2,
        ].copy()

        forward = self.data.xmat[
            self.base
        ].reshape(3, 3)[:2, 1].copy()

        self.initial_forward = (
            forward
            / max(
                np.linalg.norm(forward),
                1e-8,
            )
        )

        self.initial_lateral = np.array(
            [
                self.initial_forward[1],
                -self.initial_forward[0],
            ]
        )

        self.filtered_speed = 0.0
        self.filtered_lateral_speed = 0.0

        self.slip_total = 0.0
        self.step_count = 0

        self.max_lift = np.zeros(2)
        self.swing_trials = np.zeros(2)
        self.swing_successes = np.zeros(2)

        self.progress_check_time = 0.0
        self.progress_check_distance = 0.0

        return self._obs(), {}

    def step(self, action):
        action = np.asarray(
            action,
            dtype=float,
        )

        if (
            action.shape != (12,)
            or not np.all(np.isfinite(action))
        ):
            raise ValueError(
                "Action attendue : 12 valeurs finies."
            )

        self.rules.advance()
        self.elapsed = self.rules.elapsed

        if self.rules.failed:
            phase = (
                self.elapsed
                / self.period
            ) % 1.0

            for i, start in enumerate(
                (
                    0.1,
                    0.6,
                )
            ):
                if start < phase < start + 0.4:
                    wanted = (
                        0.002
                        + 0.023
                        * np.sin(
                            np.pi
                            * (
                                phase - start
                            )
                            / 0.4
                        )
                    )

                    if wanted > 0.015:
                        self.swing_trials[i] += 1

            info = self._full_info(
                self.failure_info
            )

            return (
                self._obs(),
                -1.0,
                self.rules.done,
                False,
                info,
            )

        action = np.clip(
            action,
            -1.0,
            1.0,
        )

        desired = np.clip(
            self.action_scale * action,
            self.low,
            self.high,
        )

        self.target += np.clip(
            desired - self.target,
            -2.0 * self.dt,
            2.0 * self.dt,
        )

        self.data.ctrl[:] = 0.0
        self.data.ctrl[self.aids] = self.target

        previous_xy = self.data.xpos[
            self.base,
            :2,
        ].copy()

        forward, lateral = self._reference_axes()

        mujoco.mj_step(
            self.model,
            self.data,
            nstep=self.substeps,
        )

        mujoco.mj_forward(
            self.model,
            self.data,
        )

        if (
            not np.all(np.isfinite(self.data.qpos))
            or not np.all(np.isfinite(self.data.qvel))
        ):
            self.rules.fail()

            self.failure_info = {
                "numerical_failure": True,
                "distance_forward": 0.0,
                "mean_forward_speed": 0.0,
                "fallen": True,
                "stalled": False,
                "lateral_distance": 0.0,
                "heading_error_deg": 0.0,
                "mean_slip_speed": 0.0,
            }

            return (
                self._obs(),
                -1.0,
                self.rules.done,
                False,
                self._full_info(
                    self.failure_info
                ),
            )

        velocity, gravity = self._kinematics()

        instant_speed = float(
            (
                self.data.xpos[
                    self.base,
                    :2,
                ]
                - previous_xy
            )
            @ forward
            / self.dt
        )

        alpha = (
            1.0
            - np.exp(
                -self.dt / 0.5
            )
        )

        self.filtered_speed += (
            alpha
            * (
                instant_speed
                - self.filtered_speed
            )
        )

        instant_lateral = float(
            (
                self.data.xpos[
                    self.base,
                    :2,
                ]
                - previous_xy
            )
            @ lateral
            / self.dt
        )

        self.filtered_lateral_speed += (
            alpha
            * (
                instant_lateral
                - self.filtered_lateral_speed
            )
        )

        measured = np.array(
            [
                self.filtered_speed,
                self.filtered_lateral_speed,
                velocity[2],
            ]
        )

        error = measured - self.command

        tolerance = max(
            0.025,
            0.25 * abs(self.command[0]),
        )

        tracking = np.exp(
            -(
                error[0]
                / tolerance
            )
            ** 2
        )

        turning = np.exp(
            -error[2] ** 2
            / 0.25
        )

        # V8 : glissement aux vrais points de contact MuJoCo.
        contacts = np.zeros(
            2,
            dtype=bool,
        )

        bad_contact = False
        contact_slip_sq = []
        foot_yaw_rate = np.zeros(2)

        for c in self.data.contact[
            : self.data.ncon
        ]:
            other = (
                c.geom2
                if c.geom1 == self.floor
                else (
                    c.geom1
                    if c.geom2 == self.floor
                    else -1
                )
            )

            if other < 0:
                continue

            bid = int(
                self.model.geom_bodyid[
                    other
                ]
            )

            if bid in self.feet:
                i = self.feet.index(bid)

                contacts[i] = True

                foot_vel = np.zeros(6)

                mujoco.mj_objectVelocity(
                    self.model,
                    self.data,
                    mujoco.mjtObj.mjOBJ_XBODY,
                    bid,
                    foot_vel,
                    0,
                )

                contact_pos = np.array(
                    c.pos,
                    dtype=float,
                )

                r = (
                    contact_pos
                    - self.data.xpos[bid]
                )

                point_velocity = (
                    foot_vel[3:]
                    + np.cross(
                        foot_vel[:3],
                        r,
                    )
                )

                contact_slip_sq.append(
                    float(
                        np.sum(
                            point_velocity[:2] ** 2
                        )
                    )
                )

                foot_yaw_rate[i] = abs(
                    float(
                        foot_vel[2]
                    )
                )

            else:
                bad_contact = True

        slip = (
            float(
                np.mean(
                    contact_slip_sq
                )
            )
            if contact_slip_sq
            else 0.0
        )

        self.slip_total += np.sqrt(slip)
        self.step_count += 1

        # V8 : hauteur = point le plus bas de toute la semelle.
        clearance = []

        for i, bid in enumerate(
            self.feet
        ):
            rot = self.data.xmat[
                bid
            ].reshape(3, 3)

            world_points = (
                self.sole_vertices[i]
                @ rot.T
                + self.data.xpos[bid]
            )

            clearance.append(
                float(
                    world_points[:, 2].min()
                )
            )

        moving = (
            self.stage == "stepping"
            or np.linalg.norm(
                self.command
            )
            > 0.01
        )

        phase = (
            self.elapsed
            / self.period
        ) % 1.0

        desired_contact = (
            np.array(
                [
                    not (
                        0.1
                        < phase
                        < 0.5
                    ),
                    not (
                        0.6
                        < phase
                        < 1.0
                    ),
                ]
            )
            if moving
            else np.ones(
                2,
                dtype=bool,
            )
        )

        desired_height = np.full(
            2,
            0.002,
        )

        for i, start in enumerate(
            (
                0.1,
                0.6,
            )
        ):
            if start < phase < start + 0.4:
                u = (
                    phase - start
                ) / 0.4

                desired_height[i] += (
                    0.023
                    * np.sin(
                        np.pi * u
                    )
                )

            if not desired_contact[i]:
                self.max_lift[i] = max(
                    self.max_lift[i],
                    clearance[i],
                )

            if desired_height[i] > 0.015:
                self.swing_trials[i] += 1

                self.swing_successes[i] += float(
                    clearance[i] > 0.010
                    and not contacts[i]
                    and contacts[1 - i]
                )

        contact_reward = float(
            np.mean(
                contacts
                == desired_contact
            )
        )

        swing = ~desired_contact

        lift_reward = (
            float(
                np.mean(
                    np.exp(
                        -(
                            (
                                np.asarray(
                                    clearance
                                )[swing]
                                - 0.025
                            )
                            / 0.02
                        )
                        ** 2
                    )
                )
            )
            if np.any(swing)
            else 0.0
        )

        upright = float(
            -gravity[2]
        )

        height = float(
            self.data.xpos[
                self.base,
                2,
            ]
        )

        fallen = (
            upright < 0.65
            or height
            < self.spawn_height * 0.65
            or bad_contact
        )

        smoothness = float(
            np.mean(
                (
                    action
                    - self.previous_action
                )
                ** 2
            )
        )

        effort = float(
            np.mean(
                self.data.actuator_force[
                    self.aids
                ]
                ** 2
            )
        )

        delta = (
            self.data.xpos[
                self.base,
                :2,
            ]
            - self.start_xy
        )

        distance = float(
            delta
            @ self.initial_forward
        )

        lateral_distance = float(
            delta
            @ self.initial_lateral
        )

        heading_error = self._heading_error()

        stalled = False

        if self.stage == "stepping":
            stationary_contacts = float(
                np.mean(
                    np.ones(
                        2,
                        dtype=bool,
                    )
                    == desired_contact
                )
            )

            height_score = 0.0

            if np.any(swing):
                actual = np.asarray(
                    clearance
                )[swing]

                wanted = (
                    desired_height[swing]
                )

                baseline = np.full_like(
                    wanted,
                    0.002,
                )

                height_score = float(
                    np.mean(
                        np.exp(
                            -(
                                (
                                    actual
                                    - wanted
                                )
                                / 0.008
                            )
                            ** 2
                        )
                        - np.exp(
                            -(
                                (
                                    baseline
                                    - wanted
                                )
                                / 0.008
                            )
                            ** 2
                        )
                    )
                )

            # V8 :
            # retour proche de V5/V6, mais avec un peu plus de poids
            # sur la vraie hauteur de semelle.
            reward = (
                4.0
                * (
                    contact_reward
                    - stationary_contacts
                )
                + 4.0
                * height_score
            )

            if not np.any(contacts):
                reward -= 5.0

            # V8 :
            # petit bonus seulement au coeur de la phase de levee.
            # Pas de penalite lorsque le pied redescend normalement.
            high_swing = (
                swing
                & (
                    desired_height
                    > 0.015
                )
            )

            if np.any(high_swing):
                successful_high_swing = []

                for i in np.flatnonzero(high_swing):
                    successful_high_swing.append(
                        float(
                            clearance[i] > 0.010
                            and not contacts[i]
                            and contacts[1 - i]
                        )
                    )

                reward += (
                    1.5
                    * float(
                        np.mean(
                            successful_high_swing
                        )
                    )
                )

            reward -= (
                0.8
                * min(
                    float(
                        np.sum(
                            delta ** 2
                        )
                    )
                    / 0.12 ** 2,
                    4.0,
                )
            )

            reward -= (
                0.25
                * min(
                    slip
                    / 0.03 ** 2,
                    4.0,
                )
            )

            # Penaliser le pivot du pied porteur.
            support = (
                desired_contact
                & contacts
            )

            if (
                np.any(support)
                and np.any(swing)
            ):
                support_yaw = float(
                    np.max(
                        foot_yaw_rate[
                            support
                        ]
                    )
                )

                reward -= (
                    0.6
                    * min(
                        (
                            support_yaw
                            / 0.35
                        )
                        ** 2,
                        4.0,
                    )
                )

            # Autoriser un petit transfert lateral.
            lateral_excess = max(
                0.0,
                abs(
                    lateral_distance
                )
                - 0.04,
            )

            reward -= (
                1.0
                * min(
                    (
                        lateral_excess
                        / 0.05
                    )
                    ** 2,
                    4.0,
                )
            )

        elif abs(self.command[0]) > 0.01:
            sign = float(
                np.sign(
                    self.command[0]
                )
            )

            ratio = float(
                np.clip(
                    sign
                    * self.filtered_speed
                    / abs(
                        self.command[0]
                    ),
                    -1.0,
                    1.5,
                )
            )

            gate = float(
                np.clip(
                    (
                        ratio - 0.3
                    )
                    / 0.7,
                    0.0,
                    1.0,
                )
            )

            stationary_tracking = np.exp(
                -(
                    self.command[0]
                    / tolerance
                )
                ** 2
            )

            reward = (
                4.0
                * (
                    tracking
                    - stationary_tracking
                )
                + ratio
                + gate
                * (
                    0.3 * upright
                    + 0.3 * turning
                    + 0.5 * contact_reward
                    + 0.3 * lift_reward
                )
            )

            if (
                self.elapsed
                - self.progress_check_time
                >= 4.0
            ):
                progress = (
                    sign
                    * (
                        distance
                        - self.progress_check_distance
                    )
                )

                stalled = (
                    progress < 0.04
                )

                self.progress_check_time = self.elapsed
                self.progress_check_distance = distance

        else:
            reward = (
                0.8 * turning
                + 0.5 * upright
                + 0.4 * contact_reward
            )

        tilt_weight = (
            1.8
            if self.stage == "stepping"
            else 1.2
        )

        reward -= (
            tilt_weight
            * float(
                np.sum(
                    gravity[:2] ** 2
                )
            )
            + 0.05
            * float(
                np.sum(
                    velocity[:2] ** 2
                )
            )
            + 0.03 * smoothness
            + 0.005 * effort
            + 2.0 * slip
        )

        lateral_weight = (
            0.15
            if self.stage == "stepping"
            else 2.0
        )

        reward -= (
            lateral_weight
            * min(
                (
                    (
                        self.filtered_lateral_speed
                        - self.command[1]
                    )
                    / 0.05
                )
                ** 2,
                2.0,
            )
        )

        if abs(self.command[2]) < 0.01:
            reward -= (
                0.25
                * min(
                    (
                        lateral_distance
                        / 0.05
                    )
                    ** 2,
                    4.0,
                )
            )

        heading_weight = (
            1.5
            if self.stage == "stepping"
            else 0.75
        )

        reward -= (
            heading_weight
            * min(
                (
                    heading_error
                    / 0.25
                )
                ** 2,
                4.0,
            )
        )

        self.previous_action = action.copy()

        info = {
            "forward_speed": float(
                measured[0]
            ),
            "yaw_speed": float(
                measured[2]
            ),
            "upright": upright,
            "tracking_error": float(
                np.linalg.norm(
                    error
                )
            ),
            "fallen": bool(
                fallen
            ),
            "stalled": bool(
                stalled
            ),
            "distance_forward": distance,
            "mean_forward_speed": (
                distance
                / self.elapsed
            ),
            "lateral_distance": lateral_distance,
            "heading_error_deg": float(
                np.degrees(
                    heading_error
                )
            ),
            "mean_slip_speed": (
                self.slip_total
                / max(
                    self.step_count,
                    1,
                )
            ),
            "right_lift_m": float(
                self.max_lift[0]
            ),
            "left_lift_m": float(
                self.max_lift[1]
            ),
            "right_swing_success": float(
                self.swing_successes[0]
                / max(
                    self.swing_trials[0],
                    1,
                )
            ),
            "left_swing_success": float(
                self.swing_successes[1]
                / max(
                    self.swing_trials[1],
                    1,
                )
            ),
        }

        if fallen or stalled:
            self.rules.fail()
            self.failure_info = info.copy()

        info = self._full_info(info)

        return (
            self._obs(),
            self.rules.score(
                float(
                    reward
                )
            ),
            self.rules.done,
            False,
            info,
        )

    def _full_info(self, info):
        info = dict(info)

        info.update(
            {
                "failed": self.rules.failed,
                "survival_time": self.rules.survival,
                "evaluation_time": self.elapsed,
                "cycles_observed_alive": int(
                    self.rules.survival
                    / self.period
                ),
                "right_swing_success": float(
                    self.swing_successes[0]
                    / max(
                        self.swing_trials[0],
                        1,
                    )
                ),
                "left_swing_success": float(
                    self.swing_successes[1]
                    / max(
                        self.swing_trials[1],
                        1,
                    )
                ),
                "right_lift_m": float(
                    self.max_lift[0]
                ),
                "left_lift_m": float(
                    self.max_lift[1]
                ),
            }
        )

        return info