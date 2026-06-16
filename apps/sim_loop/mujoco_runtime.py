from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

DM8009P_RATED_TORQUE = 20.0
M3508_HEXROLL_RATED_TORQUE = 2.46


@dataclass(slots=True)
class MujocoRuntimeConfig:
    model_path: Path
    leg_kp: float
    leg_kd: float
    wheel_kd: float


class MujocoRuntime:
    def __init__(self, cfg: MujocoRuntimeConfig) -> None:
        self.cfg = cfg
        if not cfg.model_path.exists():
            raise FileNotFoundError(f"MJCF model not found: {cfg.model_path}")
        self.model = self._build_model(cfg.model_path)
        self.data = mujoco.MjData(self.model)
        self.joint_names = (
            "lf0_Joint",
            "lf1_Joint",
            "rf0_Joint",
            "rf1_Joint",
            "l_wheel_Joint",
            "r_wheel_Joint",
        )
        self.actuator_names = tuple(f"{name}_motor" for name in self.joint_names)
        self.actuator_ids = self._resolve_actuators(self.actuator_names)
        self.joint_ids = self._resolve_joints(self.joint_names)
        self._refresh_joint_cache()

    @staticmethod
    def _build_model(model_path: Path) -> mujoco.MjModel:
        spec = mujoco.MjSpec.from_file(str(model_path))
        actuator_defs = (
            ("lf0_Joint_motor", "lf0_Joint", DM8009P_RATED_TORQUE),
            ("lf1_Joint_motor", "lf1_Joint", DM8009P_RATED_TORQUE),
            ("rf0_Joint_motor", "rf0_Joint", DM8009P_RATED_TORQUE),
            ("rf1_Joint_motor", "rf1_Joint", DM8009P_RATED_TORQUE),
            ("l_wheel_Joint_motor", "l_wheel_Joint", M3508_HEXROLL_RATED_TORQUE),
            ("r_wheel_Joint_motor", "r_wheel_Joint", M3508_HEXROLL_RATED_TORQUE),
        )
        for name, target, force_limit in actuator_defs:
            act = spec.add_actuator()
            act.name = name
            act.target = target
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.dyntype = mujoco.mjtDyn.mjDYN_NONE
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.biastype = mujoco.mjtBias.mjBIAS_NONE
            act.gainprm[0] = 1.0
            act.forcelimited = True
            act.forcerange[:] = np.array([-force_limit, force_limit])
            act.ctrllimited = False
            act.inheritrange = 0.0
        return spec.compile()

    def _resolve_actuators(self, names: tuple[str, ...]) -> tuple[int, ...]:
        resolved: list[int] = []
        for name in names:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if idx < 0:
                raise ValueError(f"missing actuator: {name}")
            resolved.append(int(idx))
        return tuple(resolved)

    def _resolve_joints(self, names: tuple[str, ...]) -> tuple[int, ...]:
        resolved: list[int] = []
        for name in names:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if idx < 0:
                raise ValueError(f"missing joint: {name}")
            resolved.append(int(idx))
        return tuple(resolved)

    def _refresh_joint_cache(self) -> None:
        self.joint_qpos = np.asarray(
            [self.model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=np.int64
        )
        self.joint_qvel = np.asarray(
            [self.model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=np.int64
        )

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[2] = 0.22
        self.data.qpos[3:7] = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        mujoco.mj_forward(self.model, self.data)

    def apply_target(self, target_pos: np.ndarray, target_wheel_vel: np.ndarray) -> np.ndarray:
        dof_pos = self.data.qpos[self.joint_qpos]
        dof_vel = self.data.qvel[self.joint_qvel]
        leg_err = target_pos - dof_pos[:4]
        leg_torque = self.cfg.leg_kp * leg_err - self.cfg.leg_kd * dof_vel[:4]
        wheel_torque = self.cfg.wheel_kd * (target_wheel_vel - dof_vel[4:])
        ctrl = np.concatenate([leg_torque, wheel_torque]).astype(np.float64, copy=False)
        self.data.ctrl[:] = ctrl
        return ctrl

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
