from __future__ import annotations

import argparse
from pathlib import Path

from .runtime import SimLoopConfig, SimLoopRuntime


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run SE3 MuJoCo sim loop")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("/Users/flamingo/Projects/robomaster/se3_rl/assets/robots/serialleg/mjcf/serialleg_fidelity_cylinder_wheels.xml"),
    )
    parser.add_argument("--socket-path", type=Path, default=Path("/tmp/se3_sim_loop.sock"))
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--kp", type=float, default=40.0)
    parser.add_argument("--kd", type=float, default=2.0)
    parser.add_argument("--wheel-kd", type=float, default=0.5)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    runtime = SimLoopRuntime(
        SimLoopConfig(
            model_path=args.model,
            socket_path=args.socket_path,
            max_steps=args.max_steps,
            rate_hz=args.rate_hz,
            leg_kp=args.kp,
            leg_kd=args.kd,
            wheel_kd=args.wheel_kd,
        )
    )
    runtime.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
