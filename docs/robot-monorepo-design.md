# 机器人部署 Monorepo 设计文档

## 状态

草案。当前落地入口以 `README.md` 和 `Cargo.toml` 为准：仓库已经有 `apps/locomotion`、`apps/auto_aim` 和 Python `apps/sim_loop`，早期草案里的 `control` / `auto_strike` 名称只代表当时的架构占位。

## 背景

这个仓库用于维护部署到 RoboMaster 机器人的运行时代码。当前已经落地的核心进程是：

- `locomotion`，负责 locomotion policy 推理闭环、CDC/sim transport 和运动目标输出。
- `auto_aim`，负责自瞄、目标跟踪、发控和 CAN 收发编排。

后续可能继续加入 `nav`、`vision_record`、`diagnostics` 等进程。它们最终会部署到同一台机器人上，由机器人开机后统一拉起和守护。

项目还有一个关键约束：队伍会维护多台不同机器人。不同机器人可能使用不同底盘、不同云台、不同相机、不同串口设备、不同电机 ID、不同 PID 参数，也可能在 `locomotion` 或 `auto_aim` 的某些算法策略上存在差异。

项目还有另一条独立约束：这些进程可能部署到不同计算平台。比如 x86_64 的 Intel NUC、aarch64 的 Jetson Orin NX、地瓜类 ARM Linux 板卡，后续也可能出现新的边缘计算盒。它们的 CPU 架构、Linux 发行版、glibc 版本、GPU/NPU 推理运行时、系统库和交叉编译工具链都可能不同。

这份文档要解决的问题是：如何用一个 Rust workspace 管理这些进程，同时让机器人差异和计算平台差异都可配置、可测试、可部署，不把仓库拆成一堆难维护的机器人专属分支或平台专属分支。

## 设计目标

### 进程边界稳定

仓库里的进程应该按运行时职责划分，而不是按机器人型号划分。`locomotion`、`auto_aim`、后续 `nav` 都是长期稳定的进程入口。新增机器人时，不应该复制出 `locomotion_infantry_a`、`locomotion_hero` 这种平行应用。

### 多机器人差异可管理

机器人差异分成三类处理：

- 参数差异进配置，例如串口、CAN 设备、电机 ID、PID、相机内参、外参、模型路径、云台限位、弹速参数。
- 硬件协议差异进 driver crate，例如不同相机 SDK、不同下位机协议、不同传感器读取方式。
- 运动学或策略差异进 adapter 或 strategy crate，例如麦轮底盘、舵轮底盘、英雄机器人特殊发射策略。

### 同一套代码支持多台机器人部署

尽量让同一个平台上的同一套二进制可以通过不同配置运行在不同机器人上。只有确实受计算平台、硬件 SDK 或编译依赖限制的能力，才使用 Cargo feature 做编译期开关。

### 计算平台差异独立建模

机器人型号和计算平台是两个维度。`infantry_a` 可以跑在 Orin NX，也可以换到 Intel NUC；`hero` 也可能使用同一类计算平台。CPU 架构、target triple、linker、sysroot、CUDA/TensorRT/NPU SDK、系统库路径和部署包格式，应该进入平台配置，不应该塞进机器人配置或 driver crate。

### 部署结果可追溯

每次部署应该能回答这几个问题：

- 部署的是哪个 git commit。
- 部署到了哪台机器人。
- 拉起了哪些进程。
- 每个进程使用了哪份配置。
- 运行失败时去哪里看日志。

### 本地开发和机器人部署共用一套入口

开发机上跑仿真、回放、单元测试和集成测试，应该复用正式进程的核心逻辑。不要维护一套只在本地能跑的测试入口，再维护一套机器人上专用入口。

## 非目标

这份设计暂时不解决以下问题：

- 不规定 IPC 最终必须使用 Zenoh、DDS、ROS 2、TCP、Unix domain socket 或共享内存。
- 不设计完整的机器人状态机和比赛策略。
- 不绑定具体相机、下位机、推理框架和模型格式。
- 不一次性规定所有计算平台的完整交叉编译工具链。
- 不要求一次性把所有未来机器人抽象完整。

这里先把仓库边界和演进规则定住，避免早期结构选错导致后面每加一台机器人都要重构。

## 依据

设计参考了以下一手资料：

- Rust Cargo workspace 官方文档：workspace 用于共同管理一组 packages，支持共享 `Cargo.lock`、统一 `target` 目录，并可以用 `cargo check --workspace` 对整个 workspace 运行检查。
- Rust Cargo features 官方文档：feature 用于条件编译和可选依赖，适合表达编译期能力，不适合承载大量运行时机器人配置。
- Rust rustc platform support 官方文档：Rust 用 target triple 标识编译目标，例如 `x86_64-unknown-linux-gnu` 和 `aarch64-unknown-linux-gnu`。
- rustup cross compilation 官方文档：交叉编译到其他平台需要安装对应 target，通常还需要额外 linker 或平台 SDK。
- Cargo configuration 官方文档：Cargo 支持按 `target.<triple>` 配置 linker、runner、rustflags 等目标相关参数。
- systemd service 官方文档：`.service` unit 描述一个由 systemd 控制和守护的进程，适合作为机器人上进程拉起和重启的基础。

## 总体方案

仓库采用一个 Rust workspace。workspace 外再保留运行配置和平台配置。整体分成五类目录：

- `apps/` 放最终运行的进程入口。
- `crates/` 放通用业务逻辑、公共类型、配置加载、通信抽象。
- `drivers/` 放硬件驱动、协议适配和 SDK 绑定。
- `robots/` 放每台机器人的运行配置和部署清单。
- `platforms/` 放计算平台、交叉编译和部署包差异。

推荐目录如下：

```text
se3-mono/
  Cargo.toml
  README.md

  apps/
    control/
      Cargo.toml
      src/main.rs
    auto_strike/
      Cargo.toml
      src/main.rs
    nav/
      Cargo.toml
      src/main.rs

  crates/
    se3_common/
      Cargo.toml
      src/lib.rs
    se3_config/
      Cargo.toml
      src/lib.rs
    se3_bus/
      Cargo.toml
      src/lib.rs
    locomotion_core/
      Cargo.toml
      src/lib.rs
    auto_strike_core/
      Cargo.toml
      src/lib.rs
    nav_core/
      Cargo.toml
      src/lib.rs

  drivers/
    rm_can/
      Cargo.toml
      src/lib.rs
    gimbal_serial/
      Cargo.toml
      src/lib.rs
    camera_hik/
      Cargo.toml
      src/lib.rs
    camera_mock/
      Cargo.toml
      src/lib.rs

  platforms/
    x86_64-linux/
      platform.toml
    orin_nx/
      platform.toml
    d_robotics/
      platform.toml

  robots/
    infantry_a/
      robot.toml
      control.toml
      auto_strike.toml
      systemd/
        se3-control.service
        se3-auto-strike.service
    infantry_b/
      robot.toml
      control.toml
      auto_strike.toml
    hero/
      robot.toml
      control.toml
      auto_strike.toml

  tools/
    deploy/
    build/
    replay/
    calibration/

  docs/
    robot-monorepo-design.md
```

这个结构的核心判断是：进程按职责稳定存在，机器人差异通过 `robots/` 和 adapter 注入，计算平台差异通过 `platforms/` 和构建脚本注入，driver 只负责接硬件和 SDK。

## Cargo workspace 设计

根目录 `Cargo.toml` 使用 virtual workspace，不在根目录放 root package。

```toml
[workspace]
resolver = "3"
members = [
  "apps/*",
  "crates/*",
  "drivers/*",
]

[workspace.package]
edition = "2024"
version = "0.1.0"
license = "UNLICENSED"

[workspace.dependencies]
anyhow = "1"
serde = { version = "1", features = ["derive"] }
toml = "0.8"
tracing = "0.1"
tracing-subscriber = "0.3"
```

`robots/` 和 `platforms/` 不进入 workspace。它们是运行和构建配置，不是 Rust package。这样可以避免配置目录被 Cargo 当成 crate，也能让机器人配置、平台配置独立演进。

`apps/*` 是二进制进程，依赖核心库和 driver。`crates/*` 和 `drivers/*` 是库，不直接负责启动线程、解析 CLI 或读环境变量，除非它们本身就是工具库。

## 进程划分

### `apps/locomotion`

`locomotion` 是运动控制进程。它负责：

- 读取机器人配置，加载 ONNX 策略模型。
- 通过 USB CDC 与 STM32 电机控制器通信（或通过 Unix socket 与 sim_loop 仿真通信）。
- 以固定 50 Hz 频率运行策略推理闭环：接收传感器状态帧 → 策略推理 → 解码动作 → 发送关节目标值。
- 上报运动状态、策略统计和关键 telemetry。
- 支持 `--dry-run` 模式（无硬件推理仅用于冒烟测试和 benchmark）。

`locomotion` 不包含自瞄图像处理逻辑。

### `apps/auto_strike`

`auto_strike` 是自瞄进程。它负责：

- 初始化相机、模型、图像预处理和推理 pipeline。
- 识别装甲板或目标。
- 跟踪目标状态。
- 解算云台目标角度、预测量和开火条件。
- 向 `locomotion` 发布瞄准请求或打击请求。
- 记录关键帧、检测结果、跟踪状态和延迟指标。

`auto_strike` 不应该直接控制电机。它最多输出命令意图，由 `locomotion` 根据机器人状态、模式和安全边界决定是否执行。

### `apps/nav`

`nav` 暂时作为未来进程预留。它负责导航、路径规划、定位融合和目标点选择。`nav` 不应该直接驱动底盘，而是向 `locomotion` 发布运动目标或速度请求。

### `apps/sim_loop`

`sim_loop` 是 MuJoCo 物理仿真工具（Python 实现），用于在开发机上替代真实硬件进行 locomotion 策略闭环测试。

- 加载机器人的 MJCF 模型，运行 PD 控制驱动虚拟关节。
- 接收 Rust 侧发送的关节目标帧，物理步进后回传传感器状态帧。
- 与 `apps/locomotion` 组成完整仿真闭环，无需物理机器人即可验证策略行为。
- 支持 MuJoCo 原生渲染和 Rerun 可视化。
- 通过 `zoo` crate 共享机器人模型定义（关节名、电机参数、MJCF 路径），不使用独立的配置来源。
- `sim_loop` 是开发工具，不部署到机器人上。

## crate 边界

### `se3_common`

放跨进程共享的基础类型：

- 时间戳、坐标系、角度、速度、姿态。
- 机器人 ID、进程 ID、模式枚举。
- 通用错误类型和结果类型。
- telemetry 事件结构。

这里不能放任何具体业务流程。`se3_common` 应该足够小，避免变成所有东西都能塞进去的杂物箱。

### `se3_config`

负责配置模型和加载逻辑：

- 解析 `robots/<name>/robot.toml`。
- 按进程读取 `control.toml`、`auto_strike.toml`。
- 校验必填字段、路径存在性、数值范围。
- 输出强类型配置结构。

配置校验应该尽早失败。机器人启动时发现配置不合法，比运行一半才出现奇怪控制行为更容易处理。

### `se3_bus`

负责进程间通信抽象：

- 定义 topic、message、publisher、subscriber。
- 屏蔽底层 IPC 选择。
- 支持本地开发时使用 mock 或 in-process bus。
- 支持机器人部署时切到真实传输层。

早期可以先用简单 TCP、Unix domain socket 或本机消息队列实现。后续如果换 Zenoh、DDS 或 ROS 2，只改 `se3_bus` 和少量接入层。

### `locomotion_core`

负责控制逻辑：

- 控制模式状态机。
- 底盘速度解算。
- 云台控制策略。
- 发射安全边界。
- 传感器状态融合。
- 对 driver trait 的调用。

`locomotion_core` 不直接依赖具体硬件 crate。它依赖 trait，由 `apps/locomotion` 组装具体实现。

### `auto_strike_core`

负责自瞄核心逻辑：

- 图像输入抽象。
- detector、tracker、predictor、solver 的接口和组合。
- 目标选择策略。
- 开火条件判断。
- 输出给 `control` 的命令模型。

`auto_strike_core` 不直接依赖具体相机 SDK。相机实现放在 `drivers/camera_*`，通过 trait 注入。

## 依赖方向

推荐依赖方向如下：

```text
apps/locomotion
  -> locomotion_core
  -> se3_config
  -> se3_bus
  -> se3_common
  -> drivers/*

apps/auto_strike
  -> auto_strike_core
  -> se3_config
  -> se3_bus
  -> se3_common
  -> drivers/*

locomotion_core
  -> se3_common

auto_strike_core
  -> se3_common

drivers/*
  -> se3_common
```

不推荐的依赖方向：

- `locomotion_core` 依赖 `apps/locomotion`。
- `auto_strike_core` 依赖 `apps/auto_strike`。
- `locomotion_core` 直接依赖具体相机或串口 SDK。
- `auto_strike_core` 直接控制电机。
- `se3_common` 反向依赖任何业务 crate。

依赖方向保持单向，测试、替换硬件和拆进程都会简单很多。

## 多机器人差异如何处理

### 第一层：配置差异

绝大多数机器人差异应该先尝试放进配置。

适合放进配置的内容：

- 机器人名称和类型。
- CAN 接口名。
- 串口路径。
- 电机 ID。
- 相机序列号。
- 相机内参和畸变参数。
- 相机到云台、云台到车体的外参。
- 云台限位。
- PID 参数。
- 底盘几何参数。
- 模型文件路径。
- 弹速、摩擦轮转速、发射延迟。
- 进程启用状态。
- 日志等级。

不适合放进配置的内容：

- 大段业务逻辑。
- 复杂条件分支。
- 会改变代码控制流结构的脚本。
- 硬件协议解析代码。

配置应该表达事实和参数，不应该变成另一套编程语言。

### 第二层：driver 差异

当差异来自硬件协议或 SDK 时，放进 driver crate。

比如：

- `drivers/camera_hik` 封装海康相机 SDK。
- `drivers/camera_mock` 提供回放和测试相机。
- `drivers/rm_can` 封装 CAN 通信和电机反馈。
- `drivers/gimbal_serial` 封装云台串口协议。

driver crate 只解决和硬件通信有关的问题，不负责决定机器人策略。

### 第三层：adapter 或 strategy 差异

当机器人在结构或策略上真的不同，再引入 adapter 或 strategy。

比如：

- 步兵使用麦轮底盘。
- 英雄使用不同发射机构。
- 工程机器人后续可能需要机械臂控制。
- 某台机器人需要不同的目标选择策略。

这类差异可以通过 trait 表达：

```rust
pub trait Chassis {
    fn set_velocity(&mut self, command: ChassisCommand) -> anyhow::Result<()>;
    fn state(&self) -> anyhow::Result<ChassisState>;
}

pub trait Gimbal {
    fn aim(&mut self, command: AimCommand) -> anyhow::Result<()>;
    fn state(&self) -> anyhow::Result<GimbalState>;
}

pub trait TargetSelector {
    fn select(&mut self, candidates: &[Target]) -> Option<TargetId>;
}
```

`apps/locomotion` 或 `apps/auto_strike` 根据配置组装具体实现：

```rust
let config = se3_config::load_robot_config(args.robot)?;

let chassis: Box<dyn Chassis> = match config.control.chassis.kind {
    ChassisKind::Mecanum => Box::new(MecanumChassis::new(config.control.chassis)?),
    ChassisKind::Swerve => Box::new(SwerveChassis::new(config.control.chassis)?),
};
```

早期不要为了想象中的未来机器人提前抽象太多。先把明确存在的差异抽出来，后续出现第二个实现时再稳定 trait。

## 为什么不按机器人拆 app

不推荐：

```text
apps/
  control_infantry_a/
  control_infantry_b/
  control_hero/
  auto_strike_infantry_a/
  auto_strike_hero/
```

这种结构一开始看起来直接，但很快会出现问题：

- 公共逻辑复制。
- bug fix 要改多份。
- 每台机器人行为逐渐漂移。
- 部署脚本需要记住更多二进制名称。
- 测试矩阵膨胀。

推荐保留稳定进程名：

```text
apps/
  control/
  auto_strike/
```

机器人差异通过启动参数选择：

```bash
control --robot robots/infantry_a/robot.toml
auto_strike --robot robots/infantry_a/robot.toml
```

这样部署系统只需要知道机器人名称，不需要知道每台机器人对应哪个专属二进制。

## 计算平台差异如何处理

机器人差异和计算平台差异要分开建模。

机器人差异回答的是：这台车的底盘、云台、相机、电机、外参、PID、策略是什么。

计算平台差异回答的是：代码要编译成什么目标，链接哪些系统库，启用哪些平台能力，部署包要带哪些运行时文件。

这两个维度最终组成一次部署：

```text
deploy target = robot + platform

infantry_a + orin_nx
infantry_a + intel_nuc
hero + orin_nx
```

### 第一层：Rust target triple

Rust 交叉编译的第一层是 target triple。当前可以先假设常见 Linux 目标：

```text
x86_64-unknown-linux-gnu
aarch64-unknown-linux-gnu
```

Intel NUC 通常对应 `x86_64-unknown-linux-gnu`。Jetson Orin NX 和地瓜类 ARM Linux 板卡通常对应 `aarch64-unknown-linux-gnu`，但它们不能只靠同一个 target triple 完全描述。它们的 GPU/NPU、系统库、SDK、驱动版本和部署包内容可能不同，所以还需要平台配置。

### 第二层：平台配置

每类计算平台放一份 `platform.toml`：

```text
platforms/
  intel_nuc/
    platform.toml
  orin_nx/
    platform.toml
  d_robotics/
    platform.toml
```

示例：

```toml
name = "orin_nx"
target = "aarch64-unknown-linux-gnu"

[build]
linker = "aarch64-linux-gnu-gcc"
features = ["camera_hik", "tensorrt"]
sysroot = "/opt/sysroots/orin_nx"

[runtime]
os = "ubuntu"
accelerator = "tensorrt"
library_paths = [
  "/usr/lib/aarch64-linux-gnu",
  "/usr/local/cuda/lib64",
]

[deploy]
package = "linux-systemd"
install_dir = "/opt/se3"
```

Intel NUC 可以是：

```toml
name = "intel_nuc"
target = "x86_64-unknown-linux-gnu"

[build]
features = ["camera_hik", "openvino"]

[runtime]
os = "ubuntu"
accelerator = "openvino"

[deploy]
package = "linux-systemd"
install_dir = "/opt/se3"
```

地瓜类平台可以先保守命名为 `d_robotics`，等具体板卡和 SDK 定下来后再细分：

```toml
name = "d_robotics"
target = "aarch64-unknown-linux-gnu"

[build]
linker = "aarch64-linux-gnu-gcc"
features = ["camera_hik", "d_robotics_npu"]
sysroot = "/opt/sysroots/d_robotics"

[runtime]
os = "linux"
accelerator = "d_robotics_npu"

[deploy]
package = "linux-systemd"
install_dir = "/opt/se3"
```

### 第三层：机器人引用平台

`robot.toml` 里只引用平台名，不展开平台细节：

```toml
name = "infantry_a"
kind = "infantry"
team_color = "blue"

[deploy]
platform = "orin_nx"
host = "192.168.1.31"
user = "robot"
install_dir = "/opt/se3"
```

如果同一台机器人换计算盒，只改 `platform` 和必要的硬件配置。机器人结构配置不应该因为 x86_64 或 aarch64 改写一遍。

### driver 和平台的关系

`drivers/` 仍然按硬件或 SDK 能力命名，不按计算平台命名。

推荐：

```text
drivers/
  camera_hik/
  camera_mock/
  inference_tensorrt/
  inference_openvino/
  inference_d_robotics/
  rm_can/
```

不推荐：

```text
drivers/
  orin_nx_camera/
  nuc_inference/
  infantry_a_orin_driver/
```

如果某个 SDK 只在一个平台上存在，也按 SDK 或能力命名，而不是按板卡命名。比如 Orin NX 上使用 TensorRT，crate 叫 `inference_tensorrt`。地瓜平台使用自己的 NPU runtime，crate 可以叫 `inference_d_robotics`。这样以后同一个 SDK 换到另一块板卡时，代码边界不需要改名。

### 构建命令由平台生成

部署脚本根据 `robot.toml` 找到平台，再根据 `platform.toml` 生成构建命令：

```bash
cargo build --release \
  --target aarch64-unknown-linux-gnu \
  -p control \
  -p auto_strike \
  --features "camera_hik tensorrt"
```

如果平台需要 linker，可以由脚本生成 `.cargo/config.toml`，或者在 CI 的 Cargo 配置里设置：

```toml
[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"
```

`rustup target add aarch64-unknown-linux-gnu` 只安装 Rust 标准库，不等于系统 SDK、linker、CUDA、TensorRT、NPU runtime 都准备好了。这些外部依赖要在平台文档或构建镜像里单独固定。

### 构建产物按平台分开

本地和 CI 的产物目录应该包含 target triple 或 platform 名：

```text
target/
  x86_64-unknown-linux-gnu/
  aarch64-unknown-linux-gnu/

dist/
  infantry_a-orin_nx/
  infantry_a-intel_nuc/
  hero-orin_nx/
```

同一个机器人如果分别构建 Orin NX 和 Intel NUC 版本，部署包不能共用。`VERSION` 里必须记录 robot、platform、target triple 和 features。

## Cargo feature 使用规则

Cargo feature 只用于编译期能力开关和可选依赖。

适合用 feature 的场景：

- 是否编译某个相机 SDK。
- 是否启用 CUDA、TensorRT、OpenVINO。
- 是否启用地瓜平台的 NPU runtime。
- 是否启用 replay 工具。
- 是否启用仿真后端。
- 是否启用某个需要系统库的 driver。

不适合用 feature 的场景：

- 选择 `infantry_a` 或 `infantry_b`。
- 配置 PID。
- 切换电机 ID。
- 配置相机外参。
- 选择当前机器人颜色。
- 调整打击阈值。

推荐 feature 示例：

```toml
[features]
default = ["camera_mock"]
camera_hik = ["dep:camera_hik"]
camera_mock = ["dep:camera_mock"]
tensorrt = ["dep:tensorrt_runtime"]
```

机器人型号不放进 feature。否则每新增一台机器人都会产生新的编译组合，最后很难确认线上二进制到底包含了哪些能力。

## 配置设计

### `robot.toml`

`robot.toml` 是一台机器人的入口配置。它描述机器人是谁、启用哪些进程、每个进程使用哪份配置。

```toml
name = "infantry_a"
kind = "infantry"
team_color = "blue"

[deploy]
platform = "orin_nx"
host = "192.168.1.31"
user = "robot"
install_dir = "/opt/se3"

[processes.control]
enabled = true
bin = "control"
config = "control.toml"
systemd_unit = "se3-control.service"

[processes.auto_strike]
enabled = true
bin = "auto_strike"
config = "auto_strike.toml"
systemd_unit = "se3-auto-strike.service"
```

`robot.toml` 只做索引和部署描述，不放太多进程内部参数。

### `control.toml`

`control.toml` 描述运动控制需要的硬件和控制参数。

```toml
[runtime]
log_level = "info"
control_hz = 1000

[can]
interface = "can0"
bitrate = 1000000

[chassis]
kind = "mecanum"
wheel_radius_m = 0.076
half_length_m = 0.19
half_width_m = 0.17

[chassis.motors.front_left]
id = 1
inverted = false

[chassis.motors.front_right]
id = 2
inverted = true

[chassis.pid.velocity]
kp = 8.0
ki = 0.0
kd = 0.2

[gimbal]
kind = "serial"
device = "/dev/ttyUSB0"
baudrate = 115200
yaw_min_deg = -160.0
yaw_max_deg = 160.0
pitch_min_deg = -20.0
pitch_max_deg = 35.0

[shooter]
enabled = true
feed_motor_id = 7
friction_left_id = 8
friction_right_id = 9
default_bullet_speed_mps = 16.0
```

### `auto_strike.toml`

`auto_strike.toml` 描述自瞄进程需要的图像、模型、解算和输出参数。

```toml
[runtime]
log_level = "info"
pipeline_hz_limit = 240

[camera]
kind = "hik"
serial = "DA1234567"
width = 1440
height = 1080
fps = 240
exposure_us = 3000

[camera.intrinsics]
fx = 1400.0
fy = 1400.0
cx = 720.0
cy = 540.0
distortion = [0.0, 0.0, 0.0, 0.0, 0.0]

[extrinsics.camera_to_gimbal]
translation_m = [0.03, 0.0, 0.06]
rotation_quat_xyzw = [0.0, 0.0, 0.0, 1.0]

[detector]
kind = "onnx"
model = "/opt/se3/models/armor_detector.onnx"
confidence_threshold = 0.45
nms_threshold = 0.50

[tracker]
kind = "ekf"
max_lost_frames = 8

[solver]
bullet_speed_mps = 16.0
gravity_mps2 = 9.80665

[output]
topic = "control/aim_request"
fire_policy = "when_confident"
```

## 配置校验规则

配置加载后必须做强校验，不能只靠反序列化成功。

建议校验内容：

- 文件路径必须存在。
- 串口、CAN 接口允许启动前不存在，但要给出清楚错误。
- 数值范围必须合理，例如频率大于 0、置信度在 0 到 1 之间。
- 电机 ID 不能重复。
- 云台限位必须满足 `min < max`。
- 相机内参矩阵不能全 0。
- 必填进程配置必须存在。
- `robot.toml` 中启用的进程必须有对应配置文件。
- `robot.toml` 引用的平台必须能在 `platforms/<name>/platform.toml` 中找到。
- `platform.toml` 中的 target triple、features、linker 和 sysroot 必须和构建环境匹配。

配置错误要在进程启动阶段失败，并输出具体字段路径：

```text
invalid config robots/infantry_a/control.toml:
  chassis.motors.front_left.id duplicates chassis.motors.front_right.id
```

## 运行时通信

进程间通信先定义消息模型，再选择传输实现。

建议先定义这些方向：

```text
auto_strike -> control
  aim_request
  fire_request
  target_state

control -> auto_strike
  robot_state
  gimbal_state
  shooter_state
  game_state

nav -> control
  velocity_request
  path_follow_request

control -> nav
  odometry
  chassis_state
```

早期消息可以用 Rust struct 表达：

```rust
pub struct AimRequest {
    pub target_id: Option<TargetId>,
    pub yaw_rad: f32,
    pub pitch_rad: f32,
    pub confidence: f32,
    pub fire_allowed: bool,
    pub timestamp_ns: u64,
}
```

`se3_bus` 只暴露 publish 和 subscribe 接口。底层用什么协议，可以留到实现阶段决定。

## 安全边界

所有会影响机器人运动和发射的动作，最终都要经过 `control`。

规则如下：

- `auto_strike` 可以请求瞄准和开火，但不能直接写电机。
- `nav` 可以请求移动，但不能直接写底盘电机。
- `control` 要根据当前模式、遥控器状态、裁判系统状态和硬件状态决定是否执行。
- `control` 对外发布执行结果，方便上层判断请求是否生效。

这样可以把危险动作收敛到一个进程里，便于加急停、失联保护、限位保护和比赛状态保护。

## systemd 部署模型

机器人上推荐用 systemd 管理长期进程。每个 app 一个 service。

示例：

```ini
[Unit]
Description=SE3 control service
After=network.target

[Service]
Type=simple
WorkingDirectory=/opt/se3
ExecStart=/opt/se3/bin/control --robot /opt/se3/robots/infantry_a/robot.toml
Restart=on-failure
RestartSec=1
User=robot

[Install]
WantedBy=multi-user.target
```

`auto_strike` 类似：

```ini
[Unit]
Description=SE3 auto strike service
After=network.target se3-control.service
Wants=se3-control.service

[Service]
Type=simple
WorkingDirectory=/opt/se3
ExecStart=/opt/se3/bin/auto_strike --robot /opt/se3/robots/infantry_a/robot.toml
Restart=on-failure
RestartSec=1
User=robot

[Install]
WantedBy=multi-user.target
```

也可以使用模板 service：

```ini
[Service]
ExecStart=/opt/se3/bin/control --robot /opt/se3/robots/%i/robot.toml
```

然后启动：

```bash
systemctl enable --now se3-control@infantry_a.service
systemctl enable --now se3-auto-strike@infantry_a.service
```

模板方式适合多台机器人共用同一份 service 文件。固定 service 适合每台机器人部署包完全展开后使用。

## 部署包结构

部署到机器人上的目录建议固定：

```text
/opt/se3/
  bin/
    control
    auto_strike
  robots/
    infantry_a/
      robot.toml
      control.toml
      auto_strike.toml
  models/
    armor_detector.onnx
  systemd/
    se3-control.service
    se3-auto-strike.service
  VERSION
```

`VERSION` 写入：

```text
git_commit = "..."
build_time = "..."
robot = "infantry_a"
platform = "orin_nx"
target = "aarch64-unknown-linux-gnu"
profile = "release"
features = "camera_hik,tensorrt"
```

部署脚本应该上传完整目录，再切换当前版本。后续如果需要回滚，可以保留：

```text
/opt/se3/releases/<commit>/
/opt/se3/current -> /opt/se3/releases/<commit>/
```

service 的 `WorkingDirectory` 和 `ExecStart` 指向 `/opt/se3/current`。

## 本地开发流程

常用命令：

```bash
cargo check --workspace
cargo test --workspace
cargo run -p control -- --robot robots/infantry_a/robot.toml
cargo run -p auto_strike -- --robot robots/infantry_a/robot.toml
```

本地没有真实硬件时，使用 mock driver 或 replay 输入：

```bash
cargo run -p auto_strike --features camera_mock -- \
  --robot robots/infantry_a/robot.toml \
  --camera-source replay:data/matches/sample_001
```

`camera_mock`、`can_mock` 这类能力可以作为 feature 或普通 crate 存在。选择标准是：如果它只影响运行时选择，普通 crate 加配置就够了；如果它引入了额外系统依赖或明显增加编译成本，再放进 feature。

## 测试策略

### 单元测试

单元测试覆盖纯逻辑：

- 底盘速度解算。
- PID 更新。
- 云台限位。
- 目标选择。
- 弹道解算。
- 配置校验。

这类测试不依赖硬件。

### 配置测试

每台机器人至少有一个配置加载测试：

```rust
#[test]
fn infantry_a_config_is_valid() {
    let config = se3_config::load_robot_config("robots/infantry_a/robot.toml").unwrap();
    config.validate().unwrap();
}
```

这样修改配置时，CI 可以提前发现字段缺失、路径写错、ID 冲突。

### 回放测试

`auto_strike` 应该支持用录制视频或图像序列跑回放测试。回放测试不追求每帧完全一致，但要检查：

- pipeline 能跑完。
- 延迟统计在可接受范围。
- 关键场景有目标输出。
- 不出现崩溃和明显错误状态。

### 硬件在环测试

机器人接硬件后，需要保留一组 smoke test：

- 能打开 CAN。
- 能读取电机反馈。
- 能打开相机。
- 能读取云台状态。
- 能启动 `control` 并进入安全 idle。
- 能启动 `auto_strike` 并发布检测状态。

这些测试可以先放在 `tools/diagnostics`，不一定一开始就进 CI。

## 日志和可观测性

每个进程统一使用 `tracing` 输出结构化日志。

日志至少包含：

- robot 名称。
- process 名称。
- git commit。
- 配置路径。
- 关键硬件初始化结果。
- 主循环频率。
- IPC 连接状态。
- 错误码和错误上下文。

关键指标：

- `control` 主循环频率和最大延迟。
- CAN 发送失败次数。
- 电机反馈超时次数。
- 云台状态超时次数。
- `auto_strike` 图像输入 FPS。
- 推理耗时。
- 跟踪状态。
- 输出命令延迟。

systemd 下可以先用 `journalctl` 看日志：

```bash
journalctl -u se3-control.service -f
journalctl -u se3-auto-strike.service -f
```

后续如果需要图形化观测，再把 telemetry 接到单独 dashboard。

## 错误处理原则

启动阶段错误直接失败：

- 配置非法。
- 必需模型不存在。
- 必需硬件无法打开。
- IPC 初始化失败。

运行阶段错误按风险分级：

- 瞬时相机帧失败，记录 warning，继续运行。
- CAN 连续发送失败，进入降级或停止控制。
- 云台状态超时，停止接收自动瞄准请求。
- `auto_strike` 目标丢失，输出无目标状态，不请求开火。
- 配置热更新失败，保留旧配置并记录错误。

所有错误都要带上下文。不要只返回 `failed`、`invalid` 这类模糊信息。

## 命名约定

进程名使用短横线或下划线需要统一。Rust crate 名建议使用下划线：

- crate：`auto_strike`
- binary：`auto_strike`
- service：`se3-auto-strike.service`
- topic：`auto_strike/target_state`

如果外部系统更习惯短横线，可以只在 systemd service 和命令别名使用短横线。Rust 内部保持下划线。

机器人目录名使用稳定小写 snake case：

```text
robots/infantry_a
robots/infantry_b
robots/hero
```

不要把临时外号写进目录名。外号可以放到配置字段里。

## 新增一台机器人怎么做

新增 `infantry_c` 的流程：

1. 复制一份最接近的机器人配置。
2. 修改 `robot.toml` 中的名称、host、启用进程。
3. 修改 `control.toml` 中的硬件接口、电机 ID、PID 和机械参数。
4. 修改 `auto_strike.toml` 中的相机、模型、内参、外参和解算参数。
5. 运行配置测试。
6. 本地用 mock 或 replay 启动 `control` 和 `auto_strike`。
7. 部署到机器人。
8. 在机器人上运行 smoke test。
9. 通过 systemd 启动正式进程。

如果新增配置时发现必须改代码，先判断差异属于 driver、adapter 还是 strategy，再放到对应层。

## 新增一个进程怎么做

新增 `nav` 的流程：

1. 新建 `apps/nav`。
2. 新建 `crates/nav_core`。
3. 在 `se3_common` 中补充必要共享消息。
4. 在 `se3_bus` 中增加 topic 定义。
5. 在每台需要导航的机器人 `robot.toml` 中增加进程配置。
6. 添加 `robots/<name>/nav.toml`。
7. 添加 systemd service。
8. 添加本地启动和配置校验测试。

新增进程时不要把它塞进现有 `control`。只要它有独立生命周期、独立故障模式、独立资源消耗，就适合成为单独进程。

## 什么时候拆 crate

可以拆 crate 的信号：

- 这部分逻辑可以独立测试。
- 多个 app 会复用它。
- 它有明确的依赖边界。
- 它的编译依赖和其他模块明显不同。
- 它需要被 mock 或替换。

不建议拆 crate 的信号：

- 只是为了目录看起来更分层。
- 只有一个文件、一个调用方、没有独立测试价值。
- 抽象名字很大，但实际只包了一层转发。
- trait 只有一个实现，而且短期看不到第二个实现。

先从少量 crate 开始，等重复和边界变清楚再拆。

## 初始落地建议

第一阶段只建最小可用结构：

```text
apps/locomotion
apps/auto_strike
crates/se3_common
crates/se3_config
crates/locomotion_core
crates/auto_strike_core
drivers/camera_mock
robots/infantry_a
```

这一阶段目标：

- `cargo check --workspace` 通过。
- 两个 app 都能读取 `robots/infantry_a/robot.toml`。
- 配置校验能发现明显错误。
- `control` 能以 mock driver 进入 idle。
- `auto_strike` 能以 mock camera 跑一条空 pipeline。

第二阶段接入真实硬件：

- 添加 CAN driver。
- 添加云台串口 driver。
- 添加真实相机 driver。
- 补 smoke test。
- 机器人上用 systemd 拉起进程。

第三阶段完善工程化：

- 加部署脚本。
- 加 release 目录和回滚。
- 加回放测试。
- 加 telemetry。
- 加多机器人配置测试。

## 关键取舍

### 机器人差异默认进配置

这样同一套 app 可以部署到多台机器人，调参不需要重新编译。只有硬件 SDK 或协议不同，才进入 driver 或 feature。

### 计算平台差异默认进 `platforms/`

机器人结构、控制参数和部署地址放在 `robots/`。CPU 架构、target triple、linker、sysroot、GPU/NPU runtime 和平台 feature 放在 `platforms/`。这样换计算盒时不会复制整套机器人配置，新增机器人时也不用重复平台构建参数。

### `control` 作为危险动作的唯一执行者

自瞄和导航都输出请求，不直接写电机。这样急停、限位、裁判系统状态、遥控器接管都能集中处理。

### workspace 管代码，`robots/` 和 `platforms/` 管部署事实

Cargo workspace 负责构建和测试 Rust package。`robots/` 保存机器人配置和部署清单，`platforms/` 保存交叉编译和平台运行时配置。三者职责分开，仓库会更清楚。

### trait 等第二个实现出现后再稳定

早期可以先定义小 trait，但不要为了未来想象写一整套复杂 plugin system。真实差异出现后再抽象，接口会更准。

## 风险和应对

### 配置越来越大

应对方式：

- 按进程拆配置。
- `robot.toml` 只做索引。
- 配置结构强类型化。
- 加配置测试。
- 给字段写清楚单位，例如 `_m`、`_deg`、`_mps`。

### trait 过早抽象

应对方式：

- 每个 trait 只覆盖当前确实需要替换的行为。
- 避免大而全的 `Robot` trait。
- 避免把所有 driver 都塞进一个统一对象。

### 进程间协议频繁变化

应对方式：

- 先在 `se3_common` 中定义消息结构。
- topic 名称保持稳定。
- 消息字段新增优先兼容旧字段。
- 重大变更在同一个 PR 中同步修改发送方和接收方。

### 部署和源码状态对不上

应对方式：

- 部署包写入 `VERSION`。
- 进程启动日志打印 git commit。
- systemd service 固定指向 `/opt/se3/current`。
- 保留 release 目录，支持回滚。

### 平台依赖散落在脚本里

应对方式：

- 每个平台必须有 `platform.toml`。
- 构建脚本只读取平台配置，不硬编码 Orin NX、Intel NUC 或地瓜平台参数。
- 平台 SDK 和系统库版本写进平台文档或构建镜像。
- 部署包 `VERSION` 记录 platform、target triple 和 features。

## 待确认问题

这些问题不影响先搭仓库，但实现前需要逐步确定：

- 机器人主控系统架构和 CPU 平台清单。
- x86_64 和 aarch64 的目标 triple。
- Orin NX、Intel NUC、地瓜平台分别使用哪些 linker、sysroot 和系统库。
- 是否使用构建镜像固化交叉编译环境。
- 相机 SDK 和推理运行时选择。
- IPC 使用哪种实现。
- 是否需要和 ROS 2 或现有上位机工具兼容。
- 部署是从开发机推送，还是机器人主动拉取。
- 比赛时是否允许配置热更新。
- 日志是否只用 journald，还是要接 dashboard。

## 结论

推荐方案是：一个 Rust workspace，稳定保留 `locomotion`、`auto_aim` 等进程入口；机器人差异默认放到 `robots/<name>` 配置；计算平台差异放到 `platforms/<name>` 配置；硬件协议和 SDK 绑定放到 `drivers/*`；确实不同的运动学和策略放到 adapter 或 strategy。部署时每台机器人选择自己的 `robot.toml`，构建脚本根据其中的平台引用选择 `platform.toml`，systemd 负责拉起和守护各个进程。

这个方案能支撑当前的 `locomotion` 和 `auto_aim`，也能自然扩展到后续 `nav`。新增机器人主要是新增 `robots/` 配置，新增计算平台主要是新增 `platforms/` 配置，只有真实硬件、SDK 或策略差异出现时才新增代码边界。
