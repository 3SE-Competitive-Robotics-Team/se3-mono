# SerialLeg Recovery Rust Port

这个目录是 `Serialleg_deploy_python` 的 Rust 版移植。

Recovery 策略网络推理使用 Rust `ort-rs` 加载 ONNX，不再使用 Rust 手写 NPZ 推理路径。默认执行器为 `--ort-ep auto`，可显式传 `cpu`、`coreml`、`openvino` 或 `tensorrt`。

## 架构分层

- `apps/locomotion`: 真机 recovery 策略进程入口。
- `apps/replay_telemetry`: 回放 NX telemetry，并用同一个 ORT policy 复算动作。
- `apps/visualize_cdc_state`: CDC/remote/synthetic 状态可视化入口。
- `crates/locomotion_core`: SerialLeg 机器人参数、四连杆映射、观测构造、动作解码、CDC、帧协议、ORT policy、runtime、replay 和 viewer 逻辑。
- `scripts`: 面向操作员的启动、导出和 NX 时间同步脚本。
- 模型和机器人资源不放入本仓库；运行时通过 `--checkpoint` 或 `SE3_RECOVERY_CHECKPOINT` 指向外部 ONNX。

对应关系：

- `src/se3_shared/*` -> `crates/locomotion_core/src/*`
- `src/se3_deploy/protocol.py` -> `crates/locomotion_core/src/protocol.rs`
- `src/se3_deploy/observation.py` -> `crates/locomotion_core/src/recovery_observation.rs`
- `src/se3_deploy/onnx_policy.py` -> `crates/locomotion_core/src/ort_policy.rs`
- `src/se3_deploy/export_onnx.py` -> 外部 ONNX artifact，由 `SE3_RECOVERY_CHECKPOINT` 指定
- `src/se3_deploy/recovery_runtime.py` -> `crates/locomotion_core/src/recovery_runtime.rs`
- `src/se3_deploy/replay_telemetry.py` -> `crates/locomotion_core/src/replay_telemetry.rs`
- `src/se3_deploy/visualize_cdc_state.py` -> `crates/locomotion_core/src/visualize_cdc_state.rs`
- `run_recovery.sh` -> `scripts/run_recovery.sh`
- `scripts/fix_time_and_pull.sh` -> `scripts/fix_time_and_pull.sh`
- `scripts/visualize_cdc_state.sh` -> `scripts/visualize_cdc_state.sh`

原仓库中 `DEPLOY_MANIFEST.md`、`workflow.md`、`requirements-runtime.txt` 的 Rust 版信息已合并到本文档。Python runtime 依赖文件不再作为 Rust 运行时依赖复制。

## Rust 部署清单

- Source Python port: `../Serialleg_deploy_python`
- Runtime entry: `./scripts/run_recovery.sh`
- Policy artifact: 外部 ONNX 文件，通过 `SE3_RECOVERY_CHECKPOINT=/path/to/model.onnx` 或 `--checkpoint /path/to/model.onnx` 指定。
- Policy runtime: Rust `ort-rs` / ONNX Runtime
- Runtime EP: `SE3_ORT_EP=auto|cpu|coreml|openvino|tensorrt`
- Time sync helper: `scripts/fix_time_and_pull.sh`
- CDC visualizer: `scripts/visualize_cdc_state.sh`
- CDC visualizer URL: `http://<nx-ip>:8081` 或 `ssh -L 8081:127.0.0.1:8081 serialleg-nx`
- Local viewer URL: `http://127.0.0.1:8097`
- USB CDC device: `auto`，优先 `/dev/ttyACM*`，其次 `/dev/ttyUSB*`
- Policy rate: `50 Hz`
- STM32 protocol: `A5 5A` framed CDC protocol, version `1`
- STM32 control loop expectation: `1 kHz` local PD / output loop

启动：

```bash
SE3_RECOVERY_CHECKPOINT=/path/to/model_4999_recovery_gru.onnx ./scripts/run_recovery.sh --dry-run --max-steps 2
```

调试工具：

```bash
./scripts/visualize_cdc_state.sh --synthetic --no-mjcf-render --viewer-port 8097
cargo run -p replay_telemetry -- logs/telemetry/example.jsonl --checkpoint /path/to/model_4999_recovery_gru.onnx
```

## NX 调试 Workflow

默认环境：

- NX SSH: `serialleg-nx`
- NX runtime: `/home/amov/project/se3_wheel_leg_nx_runtime`
- NX relay: `http://192.168.137.100:8081`
- 本机 viewer: `http://127.0.0.1:8097`

1. 对齐 NX 时间：

```bash
ssh serialleg-nx "cd /home/amov/project/se3_wheel_leg_nx_runtime; ./scripts/fix_time_and_pull.sh --time-only; date -Is"
date -Is
```

2. 启动 NX CDC relay：

```bash
ssh serialleg-nx "cd /home/amov/project/se3_wheel_leg_nx_runtime; pkill -f 'visualize_cdc_state' 2>/dev/null || true; nohup ./scripts/visualize_cdc_state.sh --local-cdc --no-mjcf-render >/tmp/se3_cdc_visualizer.log 2>&1 &"
```

3. 检查 NX relay：

```bash
curl -s http://192.168.137.100:8081/snapshot
```

4. 启动本机 viewer：

```bash
NO_PROXY="localhost,127.0.0.1,::1,192.168.137.100" \
no_proxy="localhost,127.0.0.1,::1,192.168.137.100" \
./scripts/visualize_cdc_state.sh --remote-url http://192.168.137.100:8081 --host 127.0.0.1 --viewer-port 8097
```

打开 `http://127.0.0.1:8097` 后，`source` 应为 `remote`，`hz` 应接近 `50`，`joint_pos`、`joint_vel`、`wheel`、`Observation Slices` 应持续刷新。

## 常见异常

- `source=remote` 且 `connected=false`: 本机 viewer 连到了 NX relay，但 NX relay 没读到 STM32 CDC。查看 `ssh serialleg-nx "tail -80 /tmp/se3_cdc_visualizer.log"`。
- 无法访问 `http://192.168.137.100:8081`: NX relay 未启动，或本机到 NX 网络不通。
- `target_age_ms` 持续增长: NX 没有持续下发 target。检查 recovery runtime、遥控总开关和 STM32 target 接收。

限制：

- `export_npz.py` / `export_onnx.py` 依赖训练仓库里的 `se3_sim2sim`、PyTorch 和 ONNX，Rust 版通过 `scripts/export_recovery_npz.sh` 和 `scripts/export_recovery_onnx.sh` 保留对应入口；运行时推理已切换为 ORT。
- Rust `visualize_cdc_state` 覆盖 CDC/remote/synthetic relay、JSON snapshot、SSE 和 canvas fallback；MuJoCo MJCF 渲染在当前 Rust 轻量运行时中返回 disabled。
- Python 原仓库的 Rerun telemetry 可视化没有 1:1 复制；Rust 版保留数值 replay 和误差报告。
- Python 原仓库的 `cli.py` 聚合命令没有按同名 CLI 复刻；Rust 版按 monorepo 约定拆成 workspace app。
- 原 manifest 中记录过 `model_5999_recovery_gru.npz`，之前验证使用的是外部 `model_4999_recovery_gru.onnx`。模型文件当前不进入本仓库。
