from __future__ import annotations

import os
import socket
import stat
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .mujoco_runtime import MujocoRuntime, MujocoRuntimeConfig
from .protocol import PolicyTargetFrame, unpack_policy_target


@dataclass(slots=True)
class SimLoopConfig:
    model_path: Path
    socket_path: Path
    max_steps: int
    rate_hz: float
    leg_kp: float
    leg_kd: float
    wheel_kd: float


class SimLoopRuntime:
    def __init__(self, cfg: SimLoopConfig) -> None:
        self.cfg = cfg
        self.mj = MujocoRuntime(
            MujocoRuntimeConfig(
                model_path=cfg.model_path,
                leg_kp=cfg.leg_kp,
                leg_kd=cfg.leg_kd,
                wheel_kd=cfg.wheel_kd,
            )
        )
        self.latest_target = PolicyTargetFrame(0, (0.0, 0.0, 0.0, 0.0), (0.0, 0.0))
        self._lock = threading.Lock()
        self._running = False

    def run(self) -> None:
        self._prepare_socket()
        self.mj.reset()
        self._running = True
        thread = threading.Thread(target=self._recv_loop, daemon=True)
        thread.start()
        period = 1.0 / max(self.cfg.rate_hz, 1.0)
        next_tick = time.monotonic()
        steps = 0
        try:
            while self._running:
                if self.cfg.max_steps > 0 and steps >= self.cfg.max_steps:
                    break
                with self._lock:
                    target = self.latest_target
                ctrl = self.mj.apply_target(
                    np.asarray(target.joint_pos, dtype=np.float64),
                    np.asarray(target.wheel_vel, dtype=np.float64),
                )
                self.mj.step()
                if steps % 20 == 0:
                    print(
                        f"sim step={steps} seq={target.seq} ctrl={ctrl.tolist()} "
                        f"qpos={self.mj.data.qpos[2]:.3f}"
                    )
                steps += 1
                next_tick += period
                now = time.monotonic()
                if next_tick > now:
                    time.sleep(next_tick - now)
                else:
                    next_tick = now
        finally:
            self._running = False
            self._cleanup_socket()

    def _prepare_socket(self) -> None:
        if self.cfg.socket_path.exists():
            _unlink_socket_path(self.cfg.socket_path, require_socket=True)
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.bind(str(self.cfg.socket_path))
        self._socket.settimeout(0.1)

    def _cleanup_socket(self) -> None:
        try:
            self._socket.close()
        finally:
            if self.cfg.socket_path.exists():
                _unlink_socket_path(self.cfg.socket_path, require_socket=False)

    def _recv_loop(self) -> None:
        while self._running:
            try:
                data, _ = self._socket.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError:
                break
            try:
                target = unpack_policy_target(data)
            except ValueError as exc:
                print(f"drop packet: {exc}")
                continue
            with self._lock:
                self.latest_target = target


def _unlink_socket_path(path: Path, *, require_socket: bool) -> None:
    mode = path.lstat().st_mode
    if not stat.S_ISSOCK(mode):
        if require_socket:
            raise RuntimeError(f"refusing to unlink non-socket path: {path}")
        return
    os.unlink(path)
