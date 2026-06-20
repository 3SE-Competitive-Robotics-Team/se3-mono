from __future__ import annotations

import argparse
from pathlib import Path

from .runtime import SimLoopConfig, SimLoopRuntime
from .zoo import DEFAULT_ROBOT_ID, get_robot, list_robots


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run SE3 MuJoCo sim loop")
    parser.add_argument(
        "--robot",
        choices=[profile.id for profile in list_robots()],
        default=DEFAULT_ROBOT_ID,
        help="Robot profile id",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=None,
        help="MJCF model file path; defaults to the selected robot profile",
    )
    parser.add_argument(
        "--socket-path",
        type=Path,
        default=None,
        help="Override the selected robot profile socket path",
    )
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=None, help="Override profile sim rate")
    parser.add_argument("--kp", type=float, default=None, help="Override profile leg kp")
    parser.add_argument("--kd", type=float, default=None, help="Override profile leg kd")
    parser.add_argument("--wheel-kd", type=float, default=None, help="Override profile wheel kd")
    parser.add_argument(
        "--viewer",
        choices=["none", "rerun", "mujoco"],
        default="none",
        help="Viewer mode. Use 'rerun' on macOS; 'mujoco' requires mjpython on macOS.",
    )
    parser.add_argument("--rerun-address", default=None)
    parser.add_argument("--rerun-save", type=Path, default=None)
    parser.add_argument("--rerun-memory-limit", default="1GB")
    return parser


def build_config(args: argparse.Namespace) -> SimLoopConfig:
    profile = get_robot(args.robot).sim
    return SimLoopConfig(
        model_path=args.model if args.model is not None else profile.model_path,
        socket_path=args.socket_path if args.socket_path is not None else profile.socket_path,
        max_steps=args.max_steps,
        rate_hz=args.rate_hz if args.rate_hz is not None else profile.rate_hz,
        leg_kp=args.kp if args.kp is not None else profile.leg_kp,
        leg_kd=args.kd if args.kd is not None else profile.leg_kd,
        wheel_kd=args.wheel_kd if args.wheel_kd is not None else profile.wheel_kd,
        viewer=args.viewer,
        rerun_address=args.rerun_address,
        rerun_save=args.rerun_save,
        rerun_memory_limit=args.rerun_memory_limit,
    )


def main() -> int:
    args = build_parser().parse_args()
    runtime = SimLoopRuntime(build_config(args))
    runtime.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
