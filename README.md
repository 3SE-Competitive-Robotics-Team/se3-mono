# se3-mono

RoboMaster 机器人运行时代码 monorepo。

设计文档见 [docs/robot-monorepo-design.md](docs/robot-monorepo-design.md)。

## 当前模块

| 路径 | 说明 |
| --- | --- |
| `apps/auto_aim` | 自瞄运行时应用，负责视频输入、ONNX 推理流水线、估计器、发控和 CAN 收发编排。 |
| `crates/auto_aim_core` | 自瞄核心库，包含几何、PnP、YOLO 后处理、目标估计、发控、能量机关和通讯协议。 |
| `apps/locomotion` / `crates/locomotion_core` | locomotion 运行时和模型推理、恢复控制逻辑。 |
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
| `videos/offline_capture_bundle.avi` | `auto_aim` 默认离线输入视频。 |
| `imgs/` | 本地调试图片。 |
| `log/` | 运行日志。 |
| `rerun-log/` | Rerun 调试记录。 |

这些目录已写入 `.gitignore`。编译、clippy 和单元测试不要求资源存在；运行 `cargo run -p auto_aim --release` 时，配置里引用的模型和默认视频必须存在。

## 常用检查

```bash
cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets -- -D warnings
cargo test --locked --workspace --all-targets
```

仿真循环通过根目录的 Python 启动脚本运行，命令为 `uv run se3-sim-loop --model <MJCF>`，根目录 `pyproject.toml` 管理依赖和脚本入口。
