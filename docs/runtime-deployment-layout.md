# SE3 运行时部署目录与原生依赖管理方案

## 状态

设计草案。本文定义机器人上 SE3 运行时代码、配置、日志、模型和原生动态库的放置方式。当前仓库里的运行时进程包括 `auto_aim` 和 `locomotion`；设计上仍保留后续 `control`、`nav`（导航）等进程扩展空间。ONNX Runtime 是当前最具体的例子，后续相机 SDK、NPU SDK、TensorRT 插件和其他 `.so` 也按同一套规则管理。

## 要解决的问题

机器人上的运行环境会同时包含 Rust 进程、机器人配置、模型文件、平台 SDK、推理运行时和 systemd service。如果这些文件散落在 `/usr/lib`、用户 home、临时目录和启动脚本旁边，后续排查会很麻烦：

- 不知道某个 `.so` 是系统包提供的，还是项目手工放进去的。
- 更新 ONNX Runtime 或相机 SDK 时难以回滚。
- 不同机器人平台的库版本混在一起。
- systemd 启动失败时，只能从动态链接器报错里猜路径。
- 机器人重装系统后，很难判断缺的是二进制、配置、模型还是原生库。

这份方案的目标是把 SE3 自己管理的文件集中到清楚的位置，让部署、检查、升级和回滚都有固定入口。

## 依据

本文参考以下一手资料：

- FHS 3.0：`/opt` 用于安装附加应用软件包，包的静态文件应放在 `/opt/<package>` 或 `/opt/<provider>` 下。
- FHS 3.0：`/etc/opt/<subdir>` 用于 `/opt` 软件包的主机特定配置。
- FHS 3.0：`/var/opt/<subdir>` 用于 `/opt` 软件包运行中会变化的数据，`/var/log` 用于日志。
- Linux `ld.so` 手册：动态链接器会按 RPATH、`LD_LIBRARY_PATH`、RUNPATH、`/etc/ld.so.cache`、默认系统库路径等顺序寻找共享库；带斜杠的库路径会被当作路径直接加载。
- `ort` 文档：`load-dynamic` 支持运行时通过 `ORT_DYLIB_PATH` 或程序显式路径加载 ONNX Runtime 动态库。

## 总体布局

SE3 在机器人上的默认安装前缀是：

```text
/opt/se3
```

推荐目录：

```text
/opt/se3/
  bin/
    control
    auto_aim
    locomotion
  lib/
    onnxruntime -> onnxruntime-1.24.2
    onnxruntime-1.24.2/
      include/
      lib/
        libonnxruntime.so
  cfg/
    rbt_cfg.toml
  models/
    yolo/
    locomotion/
  share/
    systemd/
      se3-control.service
      se3-auto-aim.service
      se3-locomotion.service

/etc/opt/se3/
  env/
    control.env
    auto_aim.env
    locomotion.env
  rbt_cfg.local.toml

/var/opt/se3/
  logs/
  cache/
  state/
```

`/opt/se3` 放随部署包一起发布、可以整体替换的内容，包括二进制、私有库、模型、默认配置和 service 模板。`/etc/opt/se3` 放每台机器本地维护的配置覆盖和环境变量文件。`/var/opt/se3` 放运行中会变化的数据，比如日志、缓存、状态快照和回放输出。

早期可以只落地 `/opt/se3/bin`、`/opt/se3/lib`、`/opt/se3/cfg` 和 `/var/opt/se3/logs`。目录边界先定住，具体子目录按进程逐步补齐。

## 为什么不用 `/usr/lib`

不要把 SE3 自己下载或打包的 `.so` 放进 `/usr/lib`。

`/usr/lib` 属于系统发行版和包管理器管理的库目录。手动把 ONNX Runtime、相机 SDK 或 NPU SDK 放进去，会把项目私有依赖变成全局依赖，容易影响系统里其他程序。升级时也很难判断某个库是 apt、厂商 SDK 还是 SE3 部署脚本放进去的。

SE3 需要的是应用私有运行时。应用私有库放在 `/opt/se3/lib`，由 SE3 的启动配置显式指定路径。这样问题更容易定位：找不到库就是 SE3 部署包没准备好，不是系统库搜索路径碰巧扫不到。

## 原生动态库管理规则

每个大型原生依赖用独立目录管理：

```text
/opt/se3/lib/<name>-<version>/
```

当前生效版本用不带版本号的 symlink 指向：

```text
/opt/se3/lib/<name> -> <name>-<version>
```

ONNX Runtime 的例子：

```text
/opt/se3/lib/
  onnxruntime -> onnxruntime-1.24.2
  onnxruntime-1.24.2/
    include/
    lib/
      libonnxruntime.so
```

这个结构有几个好处：

- 升级时先安装新目录，再切 symlink。
- 回滚时把 symlink 指回旧目录。
- 多个平台可以保留不同构建产物，不覆盖彼此。
- `include/` 和 `lib/` 保留上游包结构，后续编译、诊断和人工检查都方便。

不建议把项目根目录整体加入 `LD_LIBRARY_PATH`。如果某个库支持显式路径加载，优先使用显式路径。ONNX Runtime 通过 `ort` 的 `load-dynamic` 加载时，运行前设置：

```bash
export ORT_DYLIB_PATH=/opt/se3/lib/onnxruntime/lib/libonnxruntime.so
```

需要依赖动态链接器搜索路径的库，可以先用 systemd 的 per-service 环境变量限制范围，不要写入全局 `/etc/ld.so.conf`。只有当某个厂商 SDK 明确要求系统级安装时，才考虑把它做成平台安装步骤。

## 配置、模型和日志

### 配置文件放置

SE3 的默认配置（`rbt_cfg.toml`）随部署包放在：

```text
/opt/se3/cfg/
  rbt_cfg.toml
```

部署包里的配置是比赛/测试用的基准版本，一次部署即完整说明当前运行的是哪套配置。

每台机器本地差异放在 `/etc/opt/se3/rbt_cfg.local.toml`。比如某台机器更换了相机、串口名或临时调整参数，只覆盖差异字段，不修改 `/opt/se3/cfg/` 下的基准文件。启动时先读 `/opt/se3/cfg/rbt_cfg.toml`，再按约定合并 `/etc/opt/se3/rbt_cfg.local.toml` 中的覆盖项。早期如果还没有配置合并逻辑，可以先只使用 `/opt/se3/cfg/rbt_cfg.toml`，但不要把本地覆盖写进 `/usr` 或脚本里。

模型文件放在 `/opt/se3/models`：

```text
/opt/se3/models/
  yolo/
    armor-detector.onnx
  locomotion/
    policy.onnx
```

日志优先交给 systemd journal。Rust 运行时通过 `se3_log` 统一初始化文件日志：默认开发环境写仓库根目录 `log/`，如果部署机上存在 `/var/opt/se3/logs` 则写到部署日志目录，也可以用 `SE3_LOG_DIR` 显式覆盖。需要落盘文件时，目标目录是：

```text
/var/opt/se3/logs/
```

缓存、回放中间产物和运行状态放到 `/var/opt/se3/cache` 或 `/var/opt/se3/state`。这些目录可以清理或轮转，不应该影响二进制和原生库。

## systemd 启动约定

systemd unit 不应该依赖登录 shell 的环境变量。每个进程需要的动态库路径写在 service 或 `EnvironmentFile` 里。

示例：

```ini
[Service]
EnvironmentFile=-/etc/opt/se3/env/auto_aim.env
ExecStart=/opt/se3/bin/auto_aim --cfg /opt/se3/cfg/rbt_cfg.toml
```

`/etc/opt/se3/env/auto_aim.env`：

```text
ORT_DYLIB_PATH=/opt/se3/lib/onnxruntime/lib/libonnxruntime.so
```

## 依赖部署流程

原生依赖（ONNX Runtime、相机 SDK、NPU SDK 等）应在部署阶段安装到位，不在机器人启动时下载或编译。

仓库提供 `tools/deploy/` 下的部署脚本，负责将构建产物、配置、模型和原生库推送到 `/opt/se3`。部署流程分两步：

1. **安装原生依赖**：按目标平台准备 ONNX Runtime 包，安装到 `/opt/se3/lib/onnxruntime-<version>`，切 symlink。
2. **同步应用文件**：将二进制、配置、模型、systemd unit 同步到位。

部署完成后检查：

```bash
test -x /opt/se3/bin/control
test -x /opt/se3/bin/auto_aim
test -x /opt/se3/bin/locomotion
test -f /opt/se3/lib/onnxruntime/lib/libonnxruntime.so
test -f /opt/se3/cfg/rbt_cfg.toml
```

部署失败就停止，不进业务逻辑。`ExecStartPre` 不做安装，只做存在性检查：

```ini
[Service]
ExecStartPre=/bin/sh -c 'test -f /opt/se3/lib/onnxruntime/lib/libonnxruntime.so || (echo "onnxruntime missing" && exit 1)'
ExecStart=/opt/se3/bin/auto_aim --cfg /opt/se3/cfg/rbt_cfg.toml
```

### 环境变量约定

部署脚本可按平台设置环境变量文件。`/etc/opt/se3/env/auto_aim.env`：

```text
ORT_DYLIB_PATH=/opt/se3/lib/onnxruntime/lib/libonnxruntime.so
```

默认使用 Microsoft 官方 CPU 包只适合作为最小可用路径。Orin NX、TensorRT、CUDA、OpenVINO、NPU 等平台能力需要平台专属 ONNX Runtime 包，在部署阶段由部署脚本选择对应的下载源。Rust 侧打开 Cargo feature 不会自动让 `libonnxruntime.so` 拥有对应 execution provider。

## ONNX Runtime 兼容性约束

`ort` 自带的 `download-binaries` feature 会下载 pyke 提供的 ONNX Runtime 预构建二进制。`ort` 官方平台说明写明：Linux 预构建二进制需要 `glibc >= 2.39` 和 `libstdc++ >= 13.2`，对应 Ubuntu 24.04 及以上、Debian 13 Trixie 及以上。

机器人部署不要默认依赖这条路径。Ubuntu 22.04、Debian 12、Jetson Orin NX 常见 JetPack 6.0 rootfs 都低于这个要求。更稳的做法是关闭 `download-binaries`，按目标平台准备 ONNX Runtime，再用显式路径加载。

这条规则也适用于其他原生库。`.so` 能不能运行，取决于它构建时使用的 glibc、libstdc++、CUDA、TensorRT、NPU SDK 和目标系统是否匹配。

## 部署检查

部署脚本至少检查这些内容：

```bash
test -x /opt/se3/bin/control
test -x /opt/se3/bin/auto_aim
test -x /opt/se3/bin/locomotion
test -f /opt/se3/lib/onnxruntime/lib/libonnxruntime.so
test -f /opt/se3/cfg/rbt_cfg.toml
test -d /opt/se3/models
```

系统 ABI 可以这样查：

```bash
ldd --version
strings /usr/lib/*/libstdc++.so.6 | grep GLIBCXX_ | sort -V | tail -n 1
```

进程启动前，service 使用 `ExecStartPre` 检查必需库。检查失败就停止启动，不进入业务逻辑。

## 后续演进

后续可以在这个布局上继续补：

- `apps/nav`：导航进程。
- `platforms/<name>/platform.toml`：声明该平台需要哪些原生库、模型和环境变量。
- `tools/check-runtime.sh`：统一检查二进制、配置、模型、原生库和 systemd unit。
- 版本化发布目录：需要原子回滚时，再引入 `/opt/se3/releases/<commit>` 和 `/opt/se3/current`。

当前先保持一层 `/opt/se3`，把库目录和检查脚本做扎实。

## 参考资料

- [Filesystem Hierarchy Standard 3.0](https://specifications.freedesktop.org/fhs/latest/)
- [FHS `/opt`](https://specifications.freedesktop.org/fhs/latest/opt.html)
- [FHS `/etc/opt`](https://specifications.freedesktop.org/fhs/latest/etc.html)
- [FHS `/var/opt`](https://specifications.freedesktop.org/fhs/latest/varOpt.html)
- [Linux `ld.so` manual](https://man7.org/linux/man-pages/man8/ld.so.8.html)
- [`ort` platform support](https://ort.pyke.io/setup/platforms)
- [`ort` linking guide](https://ort.pyke.io/setup/linking)
