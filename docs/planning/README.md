# MoveIt 2 运动规划模块 —— 常建烁 10/11 工作说明

> 负责人：常建烁（MoveIt 2 Motion Planning）｜分支：`feature/moveit-planner`
> 适用范围：**Mock / Fake Hardware，只规划、不执行**。所有输出标记 `simulation_only=true`，
> 未经现场授权与标定，不得用于真实机械臂。

> **10/12 最新验收**：V1 CAD 规划结果见 [V1_FEASIBILITY.md](V1_FEASIBILITY.md)；
> 空关节集合拒绝修复、MoveIt 退出缺陷的对照实验与临时规避见
> [MOVE_GROUP_SHUTDOWN_AUDIT.md](MOVE_GROUP_SHUTDOWN_AUDIT.md)。下文首轮结果仅作历史记录。

## 1. 一句话说明

把状态机（PickPlace FSM）"下一步要去哪"的请求，交给 MoveIt 2 算出一条**完整的、带时间的关节轨迹**，
并按 FSM 规定的格式交回去。对应任务书第 03 节，以及接口约定里"FSM → 规划"和"规划 → 执行"两条。

```text
PickPlace FSM ──PlanTarget（阶段/动作/提环 6D 位姿/工位）──▶ MoveItPlannerPort（本模块）
                                                                    │ 查动作规则、TCP 补偿、TF 变换、起点串接、校验
                                                                    ▼
                                                              MoveIt 2（move_group）
                                                                    │
PickPlace FSM ◀──PlanResult（每个动作一条完整 TimedTrajectory）─────┘
       └──▶ MotionPort（孙嘉明）——整条轨迹一次提交，不拆点
```

## 2. 我做了什么

| # | 内容 | 位置 |
|---|---|---|
| 1 | **核对实际配置**：规划组 `manipulator`、planning frame `world`（运行时确认）、末端 `ee_link`、6 关节名与顺序、控制器与 Action、SRDF/URDF 来源与版本；发现并记录 4 处不一致（`tool0` 不存在、launch 默认型号 `aubo_i5`、MoveIt 与控制器关节顺序不同、URDF 与控制器限位不同） | [PLANNING_HANDOFF.md §1](PLANNING_HANDOFF.md) |
| 2 | **本机复现 Fake 环境**：Ubuntu 24.04/WSL2 安装 Jazzy + MoveIt 2；独立工作空间按固定提交构建 AUBO 驱动并打团队归档补丁（SHA256 与归档一致）；未触碰 NUC 只读环境 | `src/mtc_moveit_planner/scripts/` |
| 3 | **规划包 `mtc_moveit_planner`**：实现 `PlannerPort`（不改任何团队接口） | [`src/mtc_moveit_planner/`](../../src/mtc_moveit_planner/README.md) |
| 4 | **固定 6D Pose 规划证据**：规划 0.042 s，轨迹 15.7 s / 159 点（时间、位置、速度、加速度），终点误差 0.1 mm | [evidence/…/fixed_pose_plan_*.yaml](evidence/20261011_075828/) |
| 5 | **单块电池动作分段**：预抓取/接近 → 试提 → 抬起 → 搬运到 T0 → 下降 → 落座 → 撤离，逐段规划 | [PLANNING_HANDOFF.md §4](PLANNING_HANDOFF.md) |
| 6 | **测试**：29 个 GTest，其中 4 个用 main 上**真实的 PickPlaceFsm** 跑通 V1/V2 单块电池 | `src/mtc_moveit_planner/test/` |
| 7 | **一键复现脚本**：启动 Fake 控制 + move_group → 只读检查 → 导出 TF 图 → 规划 → 安全退出 | `scripts/run_planning_evidence.sh` |
| 8 | **交接文档与风险清单** | [PLANNING_HANDOFF.md](PLANNING_HANDOFF.md) |

## 2A. 第二轮（孙嘉明反馈后）

| 内容 | 结果 |
|---|---|
| 同步最新 main（Phase 2A），全新构建，原 29 项 GTest 全过 → 开 PR #3 | ✅ |
| **规划场景**：占位桌面障碍物 + 试提后末端挂电池碰撞体 | ✅ 目标放进桌面会被拒绝；V2 全流程在有桌面、带电池时仍 7 段成功 |
| **起点一致性**：`check_trajectory_start()` + `replan_primitive()`，给执行适配在发轨迹前用 | ✅ |
| **V1 规划**：以起点为种子求逆解 + 关节空间插值（逐点碰撞校验） | 预对准 116–196 s → 约 40 s，仍超看门狗（瓶颈是 90° 转向 × 0.1 rad/s），按要求保留为已知阻断（"可重复"在第三轮复测中被推翻，已修正，见 2B） |
| 测试 | 35 个 GTest 全过（新增起点检查 3、载荷/重规划 3） |

## 2B. 第三轮（#4 评审意见）

| 内容 | 结果 |
|---|---|
| 基于最新 main（#3、#6 已合并）全新构建复测 | ✅ 40 个 GTest 全过（原 35 + 新增 5）；6 个包合计 424 项 0 失败 |
| **起点检查拒绝重复关节名**：轨迹或实测状态里有重名 / 空名直接判不通过，不再"取第一个"；重规划收到重名起点返回 `INVALID_START_STATE` | ✅ |
| **场景对象 ID 隔离**：本模块写入的对象统一叫 `mtc_moveit_planner/…`，只增删自己前缀的对象；别的模块叫 `table` 的东西不会被覆盖或删掉 | ✅ 单元测试 + Fake move_group 实测反证 |
| **V1 时长稳定**：复测发现上一轮"可重复"不对（同一请求有时 123 s，因为逆解随机落到另一组解）。改成固定 27 个种子求多组解、按预计时长挑最短的 | 最终代码连续 5 次都是 39.59 s，规划耗时约 0.4–0.5 s；仍超看门狗，保留为已知阻断，没有放宽安全参数 |

## 2C. 第四轮：V1 可行性 + 舌规 CAD

| 内容 | 结果 |
|---|---|
| 末端限速时间最优参数化（上限不变） | 所有关节插值路径更快 |
| 按 `舌规V1.STEP` 建工具模型：真实 TCP、工具碰撞体、"钩口朝上"、插入行程 60 mm | 法兰朝下时舌头本就水平，**V1 不再需要大角度转向** |
| V1 单块电池（状态机不改） | 7 段全部成功，预对准 13.31 s < 24 s ✅ |
| 舌规装法 | 舌头朝插入方向 13.3 s；差 90° 约 20 s；装反（180°）不可行 |
| 更正 | 之前"要转 180°、必须改状态机"是占位 TCP 造成的错误结论 |

详见 [V1_FEASIBILITY.md](V1_FEASIBILITY.md)。

## 3. 规划模块保证的事（对 FSM 的契约）

- 每个动作返回**一条完整轨迹**，按动作名一一对应；一段失败则整体失败，不返回半套结果。
- 关节顺序**按名字重排**为 FSM 顺序（MoveIt 的顺序不可信）。
- 速度/加速度缩放 0.1（上限 0.2）；`timeout_ms` 一定长于轨迹时长；轨迹超过对应状态看门狗 80% 时直接拒绝。
- 位姿必须能经 TF 变换到 `world`，变换不了就拒绝，**绝不直接抄 XYZ**。
- 控制器限位（foreArm ±2.41、wrist1 ±2.66、wrist3 ±2.23）作为约束交给 MoveIt，并在输出前逐点复核。
- 失败统一返回 `PLANNING_FAILED(200)`，`message` 写明原因（如 `[TF_UNAVAILABLE] side_insert: …`），不静默重试。
- 规划异步进行：第一次调用返回"未完成"，完成后再返回结果，不阻塞 FSM 线程，同一请求不重复规划。
- 钩住、卸载、退钩等**接触动作归工具模块（ToolPort）**，规划模块拒绝为它们出轨迹。

## 4. 今晚结果

| 项目 | 结果 |
|---|---|
| 配置核验 | ✅ PASS |
| 固定 6D Pose 规划 | ✅ PASS |
| PlannerPort 转换层 + 29 个测试 | ✅ PASS |
| V2 单块电池 7 段规划 | ✅ PASS（仅仿真意图） |
| V1 单块电池 | ❌ FAIL：预对准限速 0.02 m/s，需 91–196 s，超过 30 s 看门狗 |
| 真实机械臂 | NOT RUN（未授权，本模块不执行任何轨迹） |

## 5. 怎么复现

```bash
cd ~/meituan/meituan_challenge_ws
bash src/mtc_moveit_planner/scripts/setup_ros_jazzy.sh   # 一次性系统依赖（需要 sudo）
bash src/mtc_moveit_planner/scripts/build_driver_ws.sh   # AUBO 驱动（独立目录，先核对版本指纹）
bash src/mtc_moveit_planner/scripts/build_planner.sh     # FSM Core + 规划包 + 测试
MTC_RMW=rmw_cyclonedds_cpp bash src/mtc_moveit_planner/scripts/run_planning_evidence.sh
```

驱动源码的获取与打补丁步骤见 [PLANNING_HANDOFF.md §6](PLANNING_HANDOFF.md)。
`MTC_RMW=rmw_cyclonedds_cpp` 仅因本机 WSL2 镜像网络下 Fast DDS 不稳定；NUC 上按 SOP 使用默认 Fast DDS。

## 6. 还没解决、需要别人配合的

| 问题 | 找谁 |
|---|---|
| V1 已可行（按 CAD，13.31 s）；需确认舌规安装方向、谁执行 60 mm 插入 / 退钩，见 [V1_FEASIBILITY.md](V1_FEASIBILITY.md) | 孙嘉明 / 现场 |
| `test_lift` / `retreat` 起点依赖工具动作假设：执行前调用 `check_trajectory_start()`，不通过则 `replan_primitive()` | 孙嘉明（执行适配接入） |
| PLAN_ACQUIRE 看门狗只有 2 s | 孙嘉明 |
| 舌规 TCP 已按 CAD 估计（ESTIMATED）；需现场核对安装方向与贴合面；T0/P1–P3 位姿、提环轴向、相机外参 | 韩明赫 |
| 场景只有占位桌面 + 电池；缺工具外形、相机、工位支架的实测尺寸 | 常建烁（现场按实测补） |
| 统一启动顺序中加入 `move_group` 与本模块 | 张叔阳 |
