from __future__ import annotations

import os
import socket
import stat
import threading
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import mujoco.viewer
import numpy as np

from .mujoco_runtime import MujocoRuntime, MujocoRuntimeConfig
from .protocol import (
    MSG_COMMAND,
    MSG_TARGET,
    PolicyCommandFrame,
    PolicyTargetFrame,
    pack_policy_state,
    unpack_policy_command,
    unpack_policy_target,
)
from .rerun_viewer import RerunSimViewer, RerunSimViewerConfig

TARGET_TIMEOUT_S = 0.10


class SimViewer(Protocol):
    def is_running(self) -> bool: ...

    def sync(
        self,
        *,
        step: int,
        command: PolicyCommandFrame,
        target: PolicyTargetFrame,
        ctrl: np.ndarray,
    ) -> None: ...


@dataclass(slots=True)
class SimLoopConfig:
    model_path: Path
    socket_path: Path
    max_steps: int
    rate_hz: float
    leg_kp: float
    leg_kd: float
    wheel_kd: float
    viewer: str
    rerun_address: str | None
    rerun_save: Path | None
    rerun_memory_limit: str


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
        self.latest_command = PolicyCommandFrame(0, (0.0, 0.0, 0.0, 0.0, 0.22, 0.0, 0.0, 0.0))
        self.latest_target_time = time.monotonic()
        self._peer_path: str | None = None
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
        viewer_ctx = self._viewer_context()
        try:
            with viewer_ctx as viewer:
                while self._running and viewer.is_running():
                    if self.cfg.max_steps > 0 and steps >= self.cfg.max_steps:
                        break
                    with self._lock:
                        command = self.latest_command
                        target = self.latest_target
                        target_time = self.latest_target_time
                        peer_path = self._peer_path
                    target_age_s = max(0.0, time.monotonic() - target_time)
                    target_valid = target.seq != 0 and target_age_s <= TARGET_TIMEOUT_S
                    if target_valid:
                        ctrl = self.mj.apply_target(
                            np.asarray(target.joint_pos, dtype=np.float64),
                            np.asarray(target.wheel_vel, dtype=np.float64),
                        )
                    else:
                        ctrl = self.mj.disable_actuators()
                    self.mj.step()
                    viewer.sync(step=steps, command=command, target=target, ctrl=ctrl)
                    if peer_path is not None:
                        state = self.mj.state_frame(
                            seq=steps & 0xFFFFFFFF,
                            tick_ms=int(self.mj.data.time * 1000.0) & 0xFFFFFFFF,
                            target=target,
                            target_age_ms=int(target_age_s * 1000.0),
                            target_valid=target_valid,
                            ctrl=ctrl,
                        )
                        try:
                            self._socket.sendto(pack_policy_state(state), peer_path)
                        except ValueError as exc:
                            print(f"drop state: {exc}")
                        except OSError as exc:
                            print(f"drop state: {exc}")
                            with self._lock:
                                if self._peer_path == peer_path:
                                    self._peer_path = None
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
            thread.join(timeout=1.0)

    @contextmanager
    def _viewer_context(self) -> Iterator[SimViewer]:
        if self.cfg.viewer == "mujoco":
            with mujoco.viewer.launch_passive(self.mj.model, self.mj.data) as viewer:
                yield MujocoSimViewer(viewer)
            return
        if self.cfg.viewer == "rerun":
            viewer = RerunSimViewer(
                RerunSimViewerConfig(
                    address=self.cfg.rerun_address,
                    save_path=self.cfg.rerun_save,
                    memory_limit=self.cfg.rerun_memory_limit,
                )
            )
            viewer.log_model(self.mj.model)
            try:
                yield RerunRuntimeViewer(viewer, self.mj.model, self.mj.data)
            finally:
                viewer.close()
            return
        yield NullSimViewer()

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
                data, peer_path = self._socket.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError:
                break
            try:
                msg_type = data[2] if len(data) >= 3 else None
                if msg_type == MSG_TARGET:
                    target = unpack_policy_target(data)
                    with self._lock:
                        self.latest_target = target
                        self.latest_target_time = time.monotonic()
                        if isinstance(peer_path, str) and peer_path:
                            self._peer_path = peer_path
                    continue
                if msg_type == MSG_COMMAND:
                    command = unpack_policy_command(data)
                    with self._lock:
                        self.latest_command = command
                        if isinstance(peer_path, str) and peer_path:
                            self._peer_path = peer_path
                    continue
                raise ValueError(f"unexpected message type {msg_type}")
            except ValueError as exc:
                print(f"drop packet: {exc}")
                continue


def _unlink_socket_path(path: Path, *, require_socket: bool) -> None:
    mode = path.lstat().st_mode
    if not stat.S_ISSOCK(mode):
        if require_socket:
            raise RuntimeError(f"refusing to unlink non-socket path: {path}")
        return
    os.unlink(path)


class NullSimViewer:
    def is_running(self) -> bool:
        return True

    def sync(
        self,
        *,
        step: int,
        command: PolicyCommandFrame,
        target: PolicyTargetFrame,
        ctrl: np.ndarray,
    ) -> None:
        return


class MujocoSimViewer:
    def __init__(self, viewer: object) -> None:
        self._viewer = viewer

    def is_running(self) -> bool:
        return bool(self._viewer.is_running())

    def sync(
        self,
        *,
        step: int,
        command: PolicyCommandFrame,
        target: PolicyTargetFrame,
        ctrl: np.ndarray,
    ) -> None:
        self._viewer.sync()


class RerunRuntimeViewer:
    def __init__(self, viewer: RerunSimViewer, model: object, data: object) -> None:
        self._viewer = viewer
        self._model = model
        self._data = data

    def is_running(self) -> bool:
        return True

    def sync(
        self,
        *,
        step: int,
        command: PolicyCommandFrame,
        target: PolicyTargetFrame,
        ctrl: np.ndarray,
    ) -> None:
        self._viewer.log_state(
            self._model,
            self._data,
            step=step,
            command=command,
            target=target,
            ctrl=ctrl,
        )
