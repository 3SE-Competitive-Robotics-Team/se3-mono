# 自瞄模块

`auto_aim` 是 RoboMaster 自瞄运行时应用，核心逻辑放在 `crates/auto_aim_core`，进程入口和异步流水线放在 `apps/auto_aim`。这份文档记录迁移到 `se3-mono` 之后的目录、资源和运行方式。

## 代码结构

| 路径 | 职责 |
| --- | --- |
| `apps/auto_aim` | 自瞄进程入口，负责初始化配置、共享日志、ONNX Runtime session、队列和 Tokio task。 |
| `crates/auto_aim_core/src/rbt_base` | 几何、弹道、EKF、IPPE PnP、排序等基础算法。 |
| `crates/auto_aim_core/src/rbt_infra` | 配置、错误、异步 latest queue 和 Rerun 辅助。 |
| `crates/auto_aim_core/src/rbt_mod` | 装甲板识别与解算、能量机关、估计器、发控、通讯、运行时路由。 |
| `crates/se3_log` | 共享日志初始化，`auto_aim` 和 `locomotion` 共用。 |
| `crates/se3_ort_ep` | ONNX Runtime execution provider 选择策略，供自瞄和 locomotion 共用。 |
| `cfg/rbt_cfg.toml` | 自瞄运行配置。模型路径、推理 EP、相机内参、估计器参数、能量机关参数都从这里读。 |

## 当前主线

当前实现是一个单进程、多 Tokio task 的异步流水线。视频帧进入预处理 task 后，根据 `RuntimeRouter` 的 route 分发到装甲板流水线或能量机关流水线。普通自瞄 route 会经过装甲板推理、YOLO 后处理、PnP 解算、目标选择、YPD tracker 和发控；能量机关 route 会经过能量机关推理、关键点解码、目标解算、相位跟踪和专用发控。

发控闭环固定在 `control_loop_250hz` 中运行，但发控决策本身在 `crates/auto_aim_core/src/rbt_mod/rbt_fire_control` 和 `crates/auto_aim_core/src/rbt_mod/rbt_energy_mechanism/fire_control.rs` 里。app 层只做调度、队列消费、反馈读取、CAN payload 构造和周期日志。

更完整的 task、队列和发控关系见 [architecture.md](architecture.md)。

## 资源目录

自瞄需要模型、视频和调试输出，这些文件通常很大，不进 git。仓库通过 `.gitignore` 忽略下面几类资源：

| 路径 | 用途 |
| --- | --- |
| `model/armor/Armor.onnx` | 装甲板 ONNX 模型。 |
| `model/armor/` | 装甲板引擎缓存目录，比如 TensorRT engine。 |
| `model/engine_mechanism/EngineMechanism.onnx` | 能量机关 ONNX 模型。 |
| `model/engine_mechanism/` | 能量机关引擎缓存目录。 |
| `videos/offline_capture_bundle.avi` | 默认离线输入视频，`apps/auto_aim` 启动时会检查这个文件。 |
| `imgs/` | 本地调试图片目录。 |
| `rerun-log/` | Rerun 调试记录输出目录。 |
| `logs/` | 运行日志目录。 |

如果只做编译和单元测试，不需要准备这些资源。运行 `auto_aim` 主程序时，模型文件和默认视频必须存在。

## 配置

主要配置在 `cfg/rbt_cfg.toml`：

- `[general_cfg]` 控制弹速、CAN 接口、是否启用 CAN、离线默认任务模式。
- `[detector_cfg]` 控制相机尺寸、推理尺寸和 ONNX Runtime execution provider。
- `[detector_cfg.armor]` 和 `[detector_cfg.energy_mechanism]` 分别配置两个模型路径、engine 目录和后处理阈值。
- `[cam_cfg]` 保存相机内参。
- `[estimator_cfg]` 保存目标丢失等待、图像中心、装甲板跳变开火保护和 YPD 几何恢复参数。运动状态分类使用硬编码常量。
- `[energy_mechanism_cfg]` 保存能量机关 tracker、aimer 和 MPC 参数。

`detector_cfg.ort_ep = "auto"` 时，运行时会按平台和可见库选择 EP。macOS 默认 CoreML；Linux aarch64 在 CUDA、TensorRT 或 cuDNN 库可见时优先走硬件加速，否则回退；其他平台按 `se3_ort_ep` 的策略选择。

日志由 `se3_log` 统一初始化。开发机默认写仓库根目录 `logs/`，机器人部署环境中如果存在 `/var/opt/se3/logs`，文件日志写到该部署目录；也可以用 `SE3_LOG_DIR` 覆盖。

## 构建、检查和运行

常用检查命令：

```bash
cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets -- -D warnings
cargo test --locked --workspace --all-targets
```

只构建自瞄：

```bash
cargo build -p auto_aim --release
```

运行自瞄：

```bash
cargo run -p auto_aim --release
```

如果使用依赖本机动态库的 ONNX Runtime EP，需要先让动态库路径对进程可见。不同平台路径不一样，按部署机器上的 ONNX Runtime、OpenVINO、CUDA、TensorRT 安装位置配置 `LD_LIBRARY_PATH` 或系统等价机制。

## 维护边界

- 运行时 task 和队列编排放在 `apps/auto_aim/src/rbt_threads.rs`。
- 目标选择、tracker 状态和发控事实信号放在 `rbt_estimator`。
- 普通自瞄发控决策放在 `rbt_fire_control`。
- 能量机关检测、跟踪和发控放在 `rbt_energy_mechanism`。
- 任务模式到 route 的映射放在 `rbt_mode_context`，共享 route 状态放在 `rbt_runtime_router`。
- 机器人参数放配置，不放 Cargo feature。
