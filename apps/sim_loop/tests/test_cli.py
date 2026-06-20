from pathlib import Path

from sim_loop.cli import build_config, build_parser
from sim_loop.zoo import DEFAULT_ROBOT_ID, get_robot, list_robots


def test_default_robot_profile_matches_rust_zoo_defaults() -> None:
    profile = get_robot(DEFAULT_ROBOT_ID)

    assert DEFAULT_ROBOT_ID == "serial_leg_dev"
    assert profile.name == "Serial Leg Dev"
    assert (
        profile.sim.model_path
        == Path(__file__).resolve().parents[3]
        / "assets"
        / "robots"
        / "serial_leg"
        / "mjcf"
        / "serialleg_fourbar_surrogate_train.xml"
    )


def test_build_config_uses_robot_profile_defaults() -> None:
    args = build_parser().parse_args([])

    assert args.robot == DEFAULT_ROBOT_ID
    assert args.model is None
    assert args.socket_path is None
    assert args.rate_hz is None
    assert args.kp is None
    assert args.kd is None
    assert args.wheel_kd is None

    profile = get_robot(DEFAULT_ROBOT_ID).sim
    cfg = build_config(args)

    assert cfg.model_path == profile.model_path
    assert cfg.socket_path == profile.socket_path
    assert cfg.rate_hz == profile.rate_hz
    assert cfg.leg_kp == profile.leg_kp
    assert cfg.leg_kd == profile.leg_kd
    assert cfg.wheel_kd == profile.wheel_kd
    assert cfg.max_steps == 0
    assert cfg.viewer == "none"


def test_build_config_prefers_explicit_cli_overrides(tmp_path: Path) -> None:
    model_path = tmp_path / "override.xml"
    socket_path = tmp_path / "sim.sock"

    args = build_parser().parse_args(
        [
            "--robot",
            DEFAULT_ROBOT_ID,
            "--model",
            str(model_path),
            "--socket-path",
            str(socket_path),
            "--max-steps",
            "42",
            "--rate-hz",
            "240.0",
            "--kp",
            "55.0",
            "--kd",
            "3.5",
            "--wheel-kd",
            "0.9",
            "--viewer",
            "rerun",
            "--rerun-address",
            "127.0.0.1:9876",
            "--rerun-save",
            str(tmp_path / "trace.rrd"),
            "--rerun-memory-limit",
            "2GB",
        ]
    )

    cfg = build_config(args)

    assert cfg.model_path == model_path
    assert cfg.socket_path == socket_path
    assert cfg.max_steps == 42
    assert cfg.rate_hz == 240.0
    assert cfg.leg_kp == 55.0
    assert cfg.leg_kd == 3.5
    assert cfg.wheel_kd == 0.9
    assert cfg.viewer == "rerun"
    assert cfg.rerun_address == "127.0.0.1:9876"
    assert cfg.rerun_save == tmp_path / "trace.rrd"
    assert cfg.rerun_memory_limit == "2GB"


def test_list_robots_contains_default_profile() -> None:
    robots = list_robots()

    assert robots
    assert robots[0] == get_robot(DEFAULT_ROBOT_ID)
