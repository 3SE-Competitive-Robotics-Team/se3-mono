# 核心链路 Gap 清单

对照参考仓库 `vivsionn/`（C++）与当前 Rust 实现。

## 实施口径

- 提交粒度：修复时按 4 个逻辑提交拆分，而不是 19 个独立提交：solver/PnP/yaw/outpost 光心，estimator/tracker/jump/movement，fire-control/timing，detector/NMS/class mapping。
- Gimbal pose：严格对齐 C++，把 gimbal pose 绑定到当前图像帧里，再随 solve 链路传入 solver；不要从 control loop 侧临时取最新 `feedback_queue`。
- IPPE 坐标系修正：#5 暂不实现。Rust 当前是自实现 IPPE，点定义和 OpenCV `SOLVEPNP_IPPE` 链路不同，先保留为待验证项，避免直接照抄 `R_ippe * M`。
- 单板半径：按 C++ tracker 口径处理。C++ 初始化时把半径写入 tracker state，普通 4 板初值 `0.20m`，outpost 3 板初值 `0.2765m`，后续由 tracker/EKF 更新并通过 `getArmorRadius()` 暴露；Rust 修复时也应让半径成为估计器状态/快照的一部分，再被单板中心回推使用，不让 solver 直接访问 tracker。

| # | 模块 | C++ (`vivsionn/`) | Rust (`se3-mono/`) | Gap | 状态 |
|---|------|-------------------|---------------------|-----|------|
| 1 | Outpost yaw | `AngleSolver.cpp:169-175` — outpost 先用 `rvec` 算法线，失败再回退到 `yaw_absolute`，两条路径都要当前图像帧的 `FrameMeta.poseEuler`；`loadMeta()` 直接把 `poseEuler` 存进 `_gimbal_pose` | `rbt_solver.rs:104` 只做 PnP 解算，`RbtSolver` 没有随帧 gimbal pose 入参；Rust 的 `gimbal_yaw/pitch` 目前只在 control 侧的 `SensData` 里使用 | 按 C++ 补“图像帧携带 gimbal pose → solver 使用”的链路 | 已完成：`RbtFrame` 携带 `GimbalPose`，preprocess 绑定当前反馈，solver 使用随帧 pose。 |
| 2 | 普通装甲板 yaw | `AngleSolver.cpp:176-177` — 普通装甲板直接做 `yaw_absolute - _gimbal_pose.yaw`，把相机系 yaw 转成机器人/云台参考系 | solver 仍只产 camera-frame yaw，没有随帧 `gimbal_yaw` 入参；`feedback_queue` 里的 `SensData.gimbal_yaw/pitch` 不能作为替代 | 缺少随帧 gimbal pose 驱动的云台参考系变换 | 已完成：普通装甲板 yaw 按随帧 gimbal yaw 转到云台/机体系口径。 |
| 3 | yaw 枚举优化 | `AngleSolver.cpp:383-445` — IPPE 后 tvec 固定、枚举 rvec（±80°/2°→0.1°）最小化重投影误差 | `rbt_ippe.rs` — 直接用 IPPE 两个候选选优 | 完全缺失 | 已完成：IPPE 后固定 tvec，按 C++ 两段 yaw 搜索优化重投影。 |
| 4 | 装甲板 3D 模型点尺寸 | `AngleSolver.h:28-37` + `AngleSolver.cpp:161-163` — 用 `armor.type` 选 Large 225mm / Small 135mm；`generalDeclaration.h:181-189` 显示默认 Small，但 `armor.number == 1` 且类型未显式给出时强制 Large | `rbt_ippe.rs:6-20` 只定义了一组 135x55mm 世界点，没有按敌机种类或装甲板类型切换 | 大型装甲板仍按小板几何在解 | 已完成：YOLO 输出携带 armor type，Hero/Large 使用 225mm，其余 Small 使用 135mm。 |
| 5 | IPPE 坐标系修正 | `AngleSolver.cpp:285-298` — C++ 先按 IPPE 规范点集解 `solvePnP(..., SOLVEPNP_IPPE)`，再做 `R_orig = R_ippe * M` 把结果转回原始相机坐标系 | `rbt_ippe.rs:109-166` 自己构造世界点、归一化点和 `Isometry3`，没有同名的 `R * M` 后处理 | 暂不实现；先作为坐标定义待验证项保留 | 暂不实现：按用户确认保留为坐标定义待验证项。 |
| 6 | 首发射击提前量符号 | `param.yaml` — `first_shot_advance_ms = -10.0`（延迟 0.06s） | `shot_phase.rs:19` — `+10.0`（提前 0.04s） | 符号反了，时间差 0.02s | 已完成：默认 `first_shot_advance_ms=-10.0`，负值走延迟语义。 |
| 7 | 单板半径 | `ypd_angle_tracker.cpp:462-475` 初始化 tracker state 半径：普通 4 板 `0.20m`，outpost 3 板 `0.2765m`；`robotestimator.cpp:297-299` / `sync_robot_msg_from_tracker()` 再用 `getArmorRadius()` 把 tracker 内半径写回估计快照 | `rbt_solver.rs:209-212` 只用 `r = 200.0` 处理单板中心回推，solver 既不持有 tracker 引用，也没半径入参 | 按 C++ 把半径放进 tracker/estimator 状态与快照，再给单板中心回推使用 | 已完成：tracker state/snapshot 保持半径；单板回推使用普通 200mm / outpost 276.5mm 初值。 |
| 8 | Outpost 半径初值 | `ypd_angle_tracker.cpp:463` — 三板直接设 `276.5mm` + prior sigma 18mm | `handle_single_armor` — 常量存在但不使用，走 200mm | 未使用 | 已完成：outpost 单板回推使用 276.5mm。 |
| 9 | 检测类别映射 | `mt_detector_tensorrt.cpp:512-515` — idx=5→Infantry5, idx=6→num=6 | `rbt_yolo.rs:273-282` — idx=5→None, idx=6→Outpost8 | Infantry5 丢失，idx=6 归类错误 | 已完成：新增 `Infantry5`，idx=5 映射 Infantry5，idx=6 映射 Outpost。 |
| 10 | 中性色装甲板过期 | `robotestimator.cpp:725-731` — 20 帧后清空 batch | `rbt_enemy_select.rs` — 无 neutral 概念 | 灰色/紫色永不过期 | 已完成：`DetectedArmor` 携带 neutral flag，estimator 按 20 帧 grace 过滤中性色主观测。 |
| 11 | Same-number ignore | `robotestimator.cpp:236-243` — `IGNORE_SAMENUM_CONDITION_SWITCH` 控制同号匹配 | `EnemySelectHandler` — 按 EnemyId 选，不比较 armor.number | 缺失 | 部分对齐：新增配置字段；Rust 当前按 `EnemyId` 分桶，无法完整复刻 C++ 跨 number batch 匹配，只在同一 solved enemy 内放宽 type 过滤。 |
| 12 | Observation jump 判定 | `robotestimator.cpp:776-782` — 比较的是 `primary_observation_pos` 和 `_last_obs_armor_pos` 的 3D 距离，阈值 `>0.15m`，且要求 `tracker_state != DETECTING`；这是装甲板观测点跳变，不是车体中心跳变 | `rbt_estimator.rs:290-302` 现在是 `single_or_double` 从 false 变 true 的“装甲板数量变化”判定 | 语义不同，Rust 这条还没对齐 C++ 的空间跳变 | 已完成：fire block 与 geometry recovery 改为空间 3D armor observation jump。 |
| 13 | Jump fire block | `robotestimator.cpp:948-951` — `FIRE_BLOCK_ON_ARMORJUMP` + block frames | `rbt_estimator.rs:292-298` — 有 config 但因 #12 判定错误 | 误触发 | 已完成：jump fire hold 基于 #12 的空间跳变触发。 |
| 14 | Static bypass 条件 | `firecontrol.cpp:194-196` — 需 `AIM_COMMAND_CTRL_MODE==2` 且 `movement==STATIC` | `controller.rs:179` — 无条件旁路 MPC | 条件过宽 | 已完成：当前 Rust controller 本身是二阶 MPC 路径，static bypass 仅对 `STATIC` 生效；无 legacy mode 时不额外造模式。 |
| 15 | Movement 粒度 | `robotestimator.cpp:1166-1229` 明确分 4 态：`spin_count > 10` 进 spinning；静态分支里 `v_xy >= 0.2` 连续 5 帧才算 `TRANSLATION`，否则 `STATIC`；旋转分支里 `linear_speed > 0.2` 才是 `TRANSPIN`，否则 `SPINNING` | `rbt_estimator.rs:445-457` 只有 `Static / Dynamic` 两态，阈值也只做静/动切分 | 旋转/横移这层分类还没保住 | 已完成：导出 `STATIC/TRANSLATION/SPINNING/TRANSPIN` 四态，按 C++ 阈值和连续计数。 |
| 16 | YawPlanner 裸默认值 | `param.yaml` — ENTER=55° LEAVE=20° | `yaw_planner.rs:52-53` — `Default` trait 写 50°/30° | 裸值错（构造时被覆盖） | 已完成：YawPlanner 默认 ENTER=55° / LEAVE=20°。 |
| 17 | 己方颜色过滤时机 | `mt_detector_tensorrt.cpp:409-415` — NMS 之后过滤 | `rbt_yolo.rs:182-185` — NMS 之前过滤 | 己方板无法抑制邻接敌方板 | 已完成：己方颜色过滤移动到 NMS 之后。 |
| 18 | post-NMS 二重过滤 | `mt_detector_tensorrt.cpp:468-524` — 只 score≥0.5→NMS | `rbt_yolo.rs:216-221` — NMS 后 confidence≥0.5 | 当前同值无害 | 已完成：移除 NMS 后二次 confidence 过滤。 |
| 19 | Outpost 光心偏移 | `param.yaml:39-41` 里 `AIMING_CX = -160`, `AIMING_CY = -300`；`AngleSolverParamsInit()` 会把它们加到主点上，而 outpost 分支又显式用 `cx - AIMING_CX`, `cy - AIMING_CY` 作为瞄准光心 | `rbt_estimator.rs:439-442` 只用 `image_center_x/y` 做普通中心分数，`rbt_solver.rs` / `yaw_planner.rs` 没有 outpost 专用偏移 | 这是固定的像素级瞄准偏置，不是裁剪偏移；Rust 里还没对应实现 | 已完成：outpost 选择评分使用 `cx - AIMING_CX`, `cy - AIMING_CY` 默认偏移。 |
