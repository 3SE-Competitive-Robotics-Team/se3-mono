from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

DEFAULT_SOCKET_PATH = Path("/tmp/se3_sim_loop.sock")
DEFAULT_RATE_HZ = 500.0
DEFAULT_LEG_KP = 40.0
DEFAULT_LEG_KD = 2.0
DEFAULT_WHEEL_KD = 0.5
DEFAULT_ROBOT_ID = "serial_leg_dev"

_REPO_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_MODEL_PATH = (
    _REPO_ROOT / "assets" / "robots" / "serial_leg" / "mjcf" / "serialleg_fourbar_surrogate_train.xml"
)


@dataclass(frozen=True, slots=True)
class SimProfile:
    model_path: Path
    socket_path: Path
    rate_hz: float
    leg_kp: float
    leg_kd: float
    wheel_kd: float


@dataclass(frozen=True, slots=True)
class RobotProfile:
    id: str
    name: str
    sim: SimProfile


_ROBOTS = {
    DEFAULT_ROBOT_ID: RobotProfile(
        id=DEFAULT_ROBOT_ID,
        name="Serial Leg Dev",
        sim=SimProfile(
            model_path=_DEFAULT_MODEL_PATH,
            socket_path=DEFAULT_SOCKET_PATH,
            rate_hz=DEFAULT_RATE_HZ,
            leg_kp=DEFAULT_LEG_KP,
            leg_kd=DEFAULT_LEG_KD,
            wheel_kd=DEFAULT_WHEEL_KD,
        ),
    ),
}


def list_robots() -> tuple[RobotProfile, ...]:
    return tuple(_ROBOTS.values())


def get_robot(robot_id: str) -> RobotProfile:
    try:
        return _ROBOTS[robot_id]
    except KeyError as exc:
        raise KeyError(f"unknown sim_loop robot profile: {robot_id}") from exc


__all__ = [
    "DEFAULT_LEG_KD",
    "DEFAULT_LEG_KP",
    "DEFAULT_RATE_HZ",
    "DEFAULT_ROBOT_ID",
    "DEFAULT_SOCKET_PATH",
    "DEFAULT_WHEEL_KD",
    "RobotProfile",
    "SimProfile",
    "get_robot",
    "list_robots",
]
