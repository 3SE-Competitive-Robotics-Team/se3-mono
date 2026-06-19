from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .protocol import PolicyStateFrame, PolicyTargetFrame

DM8009P_RATED_TORQUE = 20.0
DM8009P_NO_LOAD_SPEED = 160.0 * 2.0 * np.pi / 60.0
M3508_HEXROLL_RATED_TORQUE = 2.46
M3508_HEXROLL_NO_LOAD_SPEED = 482.0 * 19.0 / 14.0 * 2.0 * np.pi / 60.0
LEG_FRICTIONLOSS = 0.2
WHEEL_FRICTIONLOSS = 0.1
JOINT_DAMPING = 0.05
JOINT_ARMATURE = 0.01


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
        self.torque_limits = np.asarray(
            [
                DM8009P_RATED_TORQUE,
                DM8009P_RATED_TORQUE,
                DM8009P_RATED_TORQUE,
                DM8009P_RATED_TORQUE,
                M3508_HEXROLL_RATED_TORQUE,
                M3508_HEXROLL_RATED_TORQUE,
            ],
            dtype=np.float64,
        )
        self.no_load_speeds = np.asarray(
            [
                DM8009P_NO_LOAD_SPEED,
                DM8009P_NO_LOAD_SPEED,
                DM8009P_NO_LOAD_SPEED,
                DM8009P_NO_LOAD_SPEED,
                M3508_HEXROLL_NO_LOAD_SPEED,
                M3508_HEXROLL_NO_LOAD_SPEED,
            ],
            dtype=np.float64,
        )

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
            joint = spec.joint(target)
            joint.damping[0] = JOINT_DAMPING
            joint.armature = JOINT_ARMATURE
            joint.frictionloss = WHEEL_FRICTIONLOSS if "wheel" in target else LEG_FRICTIONLOSS
            joint.actfrcrange[:] = np.array([-force_limit, force_limit])
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
        requested = np.concatenate([leg_torque, wheel_torque]).astype(np.float64, copy=False)
        ctrl = self._clamp_torque_speed(requested, dof_vel)
        self.data.ctrl[:] = ctrl
        return ctrl

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def state_frame(
        self,
        *,
        seq: int,
        tick_ms: int,
        target: PolicyTargetFrame,
        target_age_ms: int,
        target_valid: bool,
        ctrl: np.ndarray,
    ) -> PolicyStateFrame:
        dof_pos = self.data.qpos[self.joint_qpos]
        dof_vel = self.data.qvel[self.joint_qvel]
        base_quat = np.asarray(self.data.qpos[3:7], dtype=np.float64)
        base_ang_vel_body = np.asarray(self.data.qvel[3:6], dtype=np.float64)
        projected_gravity = _project_gravity_body(base_quat)
        return PolicyStateFrame(
            seq=seq,
            tick_ms=tick_ms,
            target_seq=target.seq,
            target_age_ms=min(max(target_age_ms, 0), 0xFFFF),
            target_valid=1 if target_valid else 0,
            rc_switch_r=1,
            output_enabled=1,
            base_ang_vel_body=_f32x3(base_ang_vel_body),
            projected_gravity=_f32x3(projected_gravity),
            joint_pos=_f32x4(dof_pos[:4]),
            joint_vel=_f32x4(dof_vel[:4]),
            wheel_pos=_f32x2(dof_pos[4:]),
            wheel_vel=_f32x2(dof_vel[4:]),
            target_joint_pos=target.joint_pos,
            hip_torque=_f32x4(ctrl[:4]),
            wheel_torque=_f32x2(ctrl[4:]),
            wheel_motor_torque=_f32x2(ctrl[4:]),
        )

    def _clamp_torque_speed(self, requested: np.ndarray, dof_vel: np.ndarray) -> np.ndarray:
        speed_margin = np.maximum(0.0, 1.0 - np.abs(dof_vel) / self.no_load_speeds)
        limits = self.torque_limits * speed_margin
        return np.clip(requested, -limits, limits)


def _project_gravity_body(quat: np.ndarray) -> np.ndarray:
    mat = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(mat, quat)
    rot = mat.reshape(3, 3)
    return rot.T @ np.asarray([0.0, 0.0, -1.0], dtype=np.float64)


def _f32x2(values: np.ndarray) -> tuple[float, float]:
    return (float(values[0]), float(values[1]))


def _f32x3(values: np.ndarray) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def _f32x4(values: np.ndarray) -> tuple[float, float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))
