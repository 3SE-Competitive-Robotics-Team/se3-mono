"""Headless sim smoke tests — verify each robot model loads, stands, and does not fall."""

from pathlib import Path

import numpy as np

from sim_loop.mujoco_runtime import MujocoRuntime, MujocoRuntimeConfig
from sim_loop.zoo import get_robot


def _model_path() -> Path:
    return get_robot("serial_leg_dev").sim.model_path


def _config() -> MujocoRuntimeConfig:
    return MujocoRuntimeConfig(
        model_path=_model_path(),
        leg_kp=40.0,
        leg_kd=2.0,
        wheel_kd=0.5,
    )


def test_serial_leg_model_loads() -> None:
    """The MJCF model must compile without error."""
    rt = MujocoRuntime(_config())
    assert rt.model is not None
    assert rt.data is not None


def test_serial_leg_stands_with_pd_hold() -> None:
    """Run 200 PD-hold steps; the robot must not fall over."""
    rt = MujocoRuntime(_config())
    rt.reset()
    # Hold the default joint positions with PD torques.
    q0 = rt.data.qpos[rt.joint_qpos].copy()
    for _ in range(200):
        q = rt.data.qpos[rt.joint_qpos]
        dq = rt.data.qvel[rt.joint_qvel]
        torque = np.zeros(6, dtype=np.float64)
        torque[:4] = rt.cfg.leg_kp * (q0[:4] - q[:4]) - rt.cfg.leg_kd * dq[:4]
        torque[4:] = rt.cfg.wheel_kd * (-dq[4:])
        rt.data.ctrl[list(rt.actuator_ids)] = torque
        rt.step()

    base_z = rt.data.qpos[2]
    assert base_z > 0.10, f"robot fell over: base_z={base_z:.3f}"
    assert np.all(np.isfinite(rt.data.qpos)), "qpos contains NaN/Inf"
    assert np.all(np.isfinite(rt.data.qvel)), "qvel contains NaN/Inf"
