# se3-mono

RoboMaster 机器人运行时代码 monorepo。

设计文档见 [docs/robot-monorepo-design.md](docs/robot-monorepo-design.md)。

## 当前模块

| 路径 | 说明 |
| --- | --- |
| `apps/auto_aim` | 自瞄运行时应用，负责视频输入、ONNX 推理流水线、估计器、发控和 CAN 收发编排。 |
| `crates/auto_aim_core` | 自瞄核心库，包含几何、PnP、YOLO 后处理、目标估计、发控、能量机关和通讯协议。 |
| `apps/locomotion` / `crates/locomotion_core` | locomotion 策略运行时、ONNX 推理、观测构造、动作解码和 CDC/sim transport。 |
| `crates/se3_command` | 跨运行时共享的操作命令类型，当前包含底盘、云台和跳跃命令。 |
| `crates/se3_input` | 跨平台输入设备封装，当前支持 XInput 手柄采样和死区处理。 |
| `crates/se3_log` | 共享日志初始化，默认写本地 `logs/`，部署机存在 `/var/opt/se3/logs` 时写部署日志目录。 |
| `crates/se3_ort_ep` | ONNX Runtime execution provider 选择策略。 |
| `crates/zoo` | 机器人、策略、仿真和命令源的类型化默认配置注册表。 |
| `tools/replay_telemetry` | telemetry 回放验证工具。 |
| `tools/visualize_cdc_state` | CDC 状态 Web 可视化工具。 |
| `cfg/` | 当前迁移阶段的运行配置。 |
| `docs/` | 设计文档、迁移说明和模块维护文档。 |

## 自瞄

自瞄模块已经迁移到 `apps/auto_aim` 和 `crates/auto_aim_core`。主线是单进程、多 Tokio task 的实时流水线：视频帧进入预处理后按运行时 route 分发，装甲板 route 走检测、PnP、目标选择、YPD tracker 和普通发控，能量机关 route 走关键点检测、目标解算、相位跟踪和专用发控。发控线程固定 250 Hz 消费最新目标和反馈，再输出 CAN 控制帧。

自瞄文档见：

- [自瞄模块说明](docs/auto_aim/README.md)
- [系统架构与发控主线](docs/auto_aim/architecture.md)

## 资源文件

模型、视频、图片、日志和 Rerun 记录不进 git，统一放在仓库根目录下的资源目录：

| 路径 | 用途 |
| --- | --- |
| `model/armor/Armor.onnx` | 装甲板 ONNX 模型。 |
| `model/engine_mechanism/EngineMechanism.onnx` | 能量机关 ONNX 模型。 |
| `model/recovery/model_4999_recovery_gru.onnx` | `serial_leg_dev` 默认 locomotion policy checkpoint。 |
| `videos/offline_capture_bundle.avi` | `auto_aim` 默认离线输入视频。 |
| `imgs/` | 本地调试图片。 |
| `logs/` | 运行日志和 Rerun 记录。 |
| `rerun-log/` | Rerun 调试记录。 |

这些目录已写入 `.gitignore`。编译、clippy 和单元测试不要求资源存在；运行 `cargo run -p auto_aim --release` 时，配置里引用的模型和默认视频必须存在；运行 `cargo run -p locomotion -- --robot serial_leg_dev` 时，需要默认 policy checkpoint 存在，或者用 `--checkpoint <ONNX>` 显式覆盖。

## 常用检查

```bash
cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets -- -D warnings
cargo test --locked --workspace --all-targets
```

仿真循环通过根目录的 Python 启动脚本运行，根目录 `pyproject.toml` 管理依赖和脚本入口。默认 `--robot serial_leg_dev` 会使用仓库内的四连杆 surrogate MJCF、500 Hz 仿真频率和 `/tmp/se3_sim_loop.sock`：

```bash
uv run se3-sim-loop
cargo run -p locomotion -- --transport sim --checkpoint <ONNX> --max-steps 100
```

默认仿真 socket 为 `/tmp/se3_sim_loop.sock`，locomotion 客户端 socket 为 `/tmp/se3_locomotion.sock`；可分别用 `se3-sim-loop --socket-path`、`locomotion --sim-socket-path` 和 `locomotion --sim-client-socket-path` 覆盖。

locomotion 默认从 `zoo` 读取 `serial_leg_dev` 的 robot/policy 配置。命令源默认为固定静止命令，也可以接 XInput 手柄：

```bash
cargo run -p locomotion -- --list-gamepads
cargo run -p locomotion -- --command-source xinput --gamepad auto --checkpoint <ONNX>
```

policy command 当前为 8 维：`vx_mps`、`yaw_rate_rad_s`、`pitch_rad`、`roll_rad`、`height_m`、`jump_enabled`、`jump_target_height_m`、`jump_phase`。仿真 Rerun viewer 会按这些语义字段记录曲线。

## prek 预提交

提交前自动运行 CI 检查，避免推送后 CI 报错：

```bash
# 安装 prek（仅首次）
brew install prek

# 在仓库内安装 git hook
prek install

# 手动运行所有检查
prek run --all-files
```

`prek.toml` 已配置以下检查，与 CI 保持一致：`cargo fmt`、`cargo clippy`、`cargo test`、`ruff check`、`pytest`。
