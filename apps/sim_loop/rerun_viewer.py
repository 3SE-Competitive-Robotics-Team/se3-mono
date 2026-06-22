from __future__ import annotations

from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from .protocol import PolicyCommandFrame, PolicyTargetFrame

_TF = "tf#"
_VISUAL_CONTYPE = 0
_VISUAL_CONAFFINITY = 0
_COLLISION_GROUP = 3
_ACTION_LEG_NAMES = (
    "left_front_joint_pos",
    "left_back_joint_pos",
    "right_front_joint_pos",
    "right_back_joint_pos",
)
_ACTION_WHEEL_NAMES = (
    "left_wheel_vel",
    "right_wheel_vel",
)
_COMMAND_NAMES = (
    "vx_mps",
    "yaw_rate_rad_s",
    "pitch_rad",
    "roll_rad",
    "height_m",
    "jump_enabled",
    "jump_target_height_m",
    "jump_phase",
)
_COMMAND_COLORS = (
    (39, 119, 217, 255),
    (235, 145, 55, 255),
    (44, 174, 191, 255),
    (216, 91, 80, 255),
    (84, 118, 172, 255),
    (139, 103, 209, 255),
    (34, 153, 112, 255),
    (180, 185, 195, 255),
)
_CTRL_NAMES = (
    "left_front_hip_torque",
    "left_back_hip_torque",
    "right_front_hip_torque",
    "right_back_hip_torque",
    "left_wheel_torque",
    "right_wheel_torque",
)
_WHEEL_RADIUS_M = 0.06
_DEFAULT_GEOM_RGBA = np.asarray((0.5, 0.5, 0.5, 1.0), dtype=np.float32)

_GEOM_COLORS = {
    "base": (84, 118, 172, 255),
    "left_thigh": (34, 153, 112, 255),
    "left_calf": (44, 174, 191, 255),
    "left_wheel": (39, 119, 217, 255),
    "right_thigh": (235, 145, 55, 255),
    "right_calf": (216, 91, 80, 255),
    "right_wheel": (139, 103, 209, 255),
    "default": (180, 185, 195, 255),
}


@dataclass(slots=True)
class RerunSimViewerConfig:
    app_id: str = "se3_sim_loop"
    spawn: bool = True
    address: str | None = None
    save_path: Path | None = None
    memory_limit: str = "1GB"


class RerunSimViewer:
    def __init__(self, cfg: RerunSimViewerConfig) -> None:
        self.cfg = cfg
        _disable_rerun_atexit_shutdown()
        self.recording = rr.RecordingStream(cfg.app_id, make_default=True)
        sinks = []
        if cfg.address:
            sinks.append(rr.GrpcSink(_rerun_grpc_url(cfg.address)))
        elif cfg.spawn:
            rr.spawn(
                connect=False,
                detach_process=True,
                memory_limit=cfg.memory_limit,
                server_memory_limit=cfg.memory_limit,
            )
            sinks.append(rr.GrpcSink(_rerun_grpc_url(None)))
        if cfg.save_path is not None:
            cfg.save_path.parent.mkdir(parents=True, exist_ok=True)
            sinks.append(rr.FileSink(str(cfg.save_path), write_footer=False))
        if sinks:
            self.recording.set_sinks(*sinks)
        self.recording.send_blueprint(_blueprint(), make_active=True, make_default=True)
        self.body_paths: list[str] = []
        self.geom_paths: dict[int, str] = {}

    def close(self) -> None:
        self.recording.flush(timeout_sec=5.0)

    def log_model(self, model: mujoco.MjModel) -> None:
        _log_timeseries_styles()
        self.body_paths = [_body_path(model, body_id) for body_id in range(model.nbody)]
        for geom_id in range(model.ngeom):
            if not _should_log_visual_geom(model, geom_id):
                continue
            body_id = int(model.geom_bodyid[geom_id])
            body_name = _name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            geom_name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            path = f"/world/visual_geometries/{body_name}/{geom_name}"
            self.geom_paths[geom_id] = path
            _log_geom(model, geom_id, path, body_frame=f"{_TF}/{self.body_paths[body_id]}")

    def log_state(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        step: int,
        command: PolicyCommandFrame,
        target: PolicyTargetFrame,
        ctrl: np.ndarray,
    ) -> None:
        if not self.body_paths:
            self.log_model(model)
        rr.set_time("time", duration=float(data.time))
        rr.set_time("step", sequence=int(step))
        for body_id, path in enumerate(self.body_paths):
            rr.log(
                path,
                rr.Transform3D(
                    translation=np.asarray(data.xpos[body_id], dtype=np.float32),
                    quaternion=rr.Quaternion(xyzw=_quat_wxyz_to_xyzw(data.xquat[body_id])),
                ),
            )
        rr.log("/metrics/base_height", rr.Scalars(float(data.qpos[2])))
        for name, value in zip(_COMMAND_NAMES, command.command, strict=True):
            rr.log(f"/metrics/command/{name}", rr.Scalars(float(value)))
        for name, value in zip(_ACTION_LEG_NAMES, target.joint_pos, strict=True):
            rr.log(f"/metrics/action/joint_pos/{name}", rr.Scalars(float(value)))
        for name, value in zip(_ACTION_WHEEL_NAMES, target.wheel_vel, strict=True):
            rr.log(
                f"/metrics/action/wheel_vel_mps/{name}", rr.Scalars(float(value) * _WHEEL_RADIUS_M)
            )
            rr.log(f"/metrics/action/wheel_vel_rad_s/{name}", rr.Scalars(float(value)))
        for name, value in zip(_CTRL_NAMES, ctrl, strict=True):
            rr.log(f"/metrics/ctrl/{name}", rr.Scalars(float(value)))


def _rerun_grpc_url(address: str | None) -> str:
    if address is None:
        return "rerun+http://127.0.0.1:9876/proxy"
    if "://" in address:
        return address
    return f"rerun+http://{address}/proxy"


def _disable_rerun_atexit_shutdown() -> None:
    unregister_shutdown = getattr(rr, "unregister_shutdown", None)
    if callable(unregister_shutdown):
        with suppress(ValueError):
            unregister_shutdown()


def _blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", name="MuJoCo"),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="/metrics/base_height", name="Base height"),
                rrb.TimeSeriesView(
                    origin="/metrics/command",
                    name="Semantic command",
                ),
                rrb.TimeSeriesView(origin="/metrics/action/joint_pos", name="Action joint target"),
                rrb.TimeSeriesView(origin="/metrics/action/wheel_vel_mps", name="Action wheel m/s"),
                rrb.TimeSeriesView(
                    origin="/metrics/action/wheel_vel_rad_s",
                    name="Action wheel rad/s",
                    visible=False,
                ),
                rrb.TimeSeriesView(origin="/metrics/ctrl", name="Motor torque"),
            ),
            column_shares=[3, 2],
        ),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
        collapse_panels=True,
        auto_layout=False,
        auto_views=False,
    )


def _log_timeseries_styles() -> None:
    for name, color in zip(_COMMAND_NAMES, _COMMAND_COLORS, strict=True):
        rr.log(
            f"/metrics/command/{name}",
            rr.SeriesLines(colors=[color], names=[name]),
            static=True,
        )


def _log_geom(model: mujoco.MjModel, geom_id: int, path: str, *, body_frame: str) -> None:
    geom_type = int(model.geom_type[geom_id])
    size = np.asarray(model.geom_size[geom_id], dtype=np.float32)
    pos = np.asarray(model.geom_pos[geom_id], dtype=np.float32)
    quat = _quat_wxyz_to_xyzw(model.geom_quat[geom_id])
    color = _geom_color(model, geom_id)

    rr.log(path, rr.CoordinateFrame(body_frame), static=True)
    rr.log(path, rr.InstancePoses3D(translations=[pos], quaternions=[quat]), static=True)

    if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id >= 0:
            rr.log(path, _mesh_asset(model, mesh_id, color), static=True)
            return
    if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
        rr.log(
            path,
            rr.Boxes3D(half_sizes=[size], colors=[color], fill_mode=rr.components.FillMode.Solid),
            static=True,
        )
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        rr.log(
            path,
            rr.Ellipsoids3D(
                half_sizes=[[float(size[0]), float(size[0]), float(size[0])]],
                colors=[color],
                fill_mode=rr.components.FillMode.Solid,
            ),
            static=True,
        )
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        rr.log(
            path,
            rr.Cylinders3D(
                lengths=[float(size[1] * 2.0)],
                radii=[float(size[0])],
                colors=[color],
                fill_mode=rr.components.FillMode.Solid,
            ),
            static=True,
        )
    elif geom_type == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        rr.log(
            path,
            rr.Capsules3D(
                lengths=[float(size[1] * 2.0)],
                radii=[float(size[0])],
                translations=[[0.0, 0.0, -float(size[1])]],
                colors=[color],
                fill_mode=rr.components.FillMode.Solid,
            ),
            static=True,
        )
    else:
        rr.log(path, rr.Points3D([[0.0, 0.0, 0.0]], colors=[color], radii=[0.01]), static=True)


def _body_path(model: mujoco.MjModel, body_id: int) -> str:
    return f"/world/bodies/{_name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)}"


def _name(model: mujoco.MjModel, obj_type: mujoco.mjtObj, obj_id: int) -> str:
    name = mujoco.mj_id2name(model, obj_type, obj_id)
    return name if name else f"{obj_type.name.lower()}_{obj_id}"


def _geom_color(model: mujoco.MjModel, geom_id: int) -> tuple[int, int, int, int]:
    rgba = np.asarray(model.geom_rgba[geom_id], dtype=np.float32)
    if _has_custom_rgba(rgba):
        return tuple(int(v) for v in np.clip(rgba * 255.0, 0, 255))
    return _semantic_geom_color(_geom_descriptor(model, geom_id))


def _has_custom_rgba(rgba: np.ndarray) -> bool:
    return bool(np.any(rgba > 0.0) and not np.allclose(rgba, _DEFAULT_GEOM_RGBA))


def _geom_descriptor(model: mujoco.MjModel, geom_id: int) -> str:
    parts = [_name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)]
    mesh_id = int(model.geom_dataid[geom_id])
    if mesh_id >= 0:
        parts.append(_name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id))
    return " ".join(parts).lower()


def _semantic_geom_color(name: str) -> tuple[int, int, int, int]:
    side = _geom_side(name)
    if "wheel" in name and side is not None:
        return _GEOM_COLORS[f"{side}_wheel"]
    if "thigh" in name and side is not None:
        return _GEOM_COLORS[f"{side}_thigh"]
    if "calf" in name and side is not None:
        return _GEOM_COLORS[f"{side}_calf"]
    if "base" in name:
        return _GEOM_COLORS["base"]
    return _GEOM_COLORS["default"]


def _geom_side(name: str) -> str | None:
    if any(
        token in name
        for token in ("left", "lf_", "lf0", "lf1", "l_wheel", "visual_lf", "visual_l_")
    ):
        return "left"
    if any(
        token in name
        for token in ("right", "rf_", "rf0", "rf1", "r_wheel", "visual_rf", "visual_r_")
    ):
        return "right"
    return None


def _quat_wxyz_to_xyzw(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float32).reshape(4)
    return np.asarray([q[1], q[2], q[3], q[0]], dtype=np.float32)


def _mesh_asset(model: mujoco.MjModel, mesh_id: int, color: tuple[int, int, int, int]) -> rr.Mesh3D:
    vert_adr = int(model.mesh_vertadr[mesh_id])
    vert_num = int(model.mesh_vertnum[mesh_id])
    face_adr = int(model.mesh_faceadr[mesh_id])
    face_num = int(model.mesh_facenum[mesh_id])
    scale = np.asarray(model.mesh_scale[mesh_id], dtype=np.float32).reshape(1, 3)
    vertices = np.asarray(model.mesh_vert[vert_adr : vert_adr + vert_num], dtype=np.float32) * scale
    faces = np.asarray(model.mesh_face[face_adr : face_adr + face_num], dtype=np.uint32)
    vertex_colors = np.repeat(np.asarray(color, dtype=np.uint8).reshape(1, 4), vert_num, axis=0)
    return rr.Mesh3D(vertex_positions=vertices, triangle_indices=faces, vertex_colors=vertex_colors)


def _should_log_visual_geom(model: mujoco.MjModel, geom_id: int) -> bool:
    is_plane = int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_PLANE)
    return not is_plane and not _is_collision_geom(model, geom_id)


def _is_collision_geom(model: mujoco.MjModel, geom_id: int) -> bool:
    contype = int(model.geom_contype[geom_id])
    conaffinity = int(model.geom_conaffinity[geom_id])
    group = int(model.geom_group[geom_id])
    return (
        contype != _VISUAL_CONTYPE
        or conaffinity != _VISUAL_CONAFFINITY
        or group == _COLLISION_GROUP
    )
