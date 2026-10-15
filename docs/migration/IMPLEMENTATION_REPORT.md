# IMPLEMENTATION_REPORT —— 实施与验收记录

> 项目：美团第四届低空经济与具身智能挑战赛｜AUBO S3 机械臂 ROS 2 工程
> 阶段：DSH V3 提示词 P0–P6
> 日期：2026-10-09（Asia/Hong_Kong）
> 工作空间：`/home/robot/meituan_challenge_ws`
> **安全边界：全程离线开发；未连接任何真实设备；未执行 `git push`。**

---

## 0. 一页结论

| 阶段 | 内容 | 结果 |
|---|---|---|
| P0 | 只读盘点与取证 | **PASS**（含旧工程缺失取证） |
| P1 | 仓库确认、目录初始化、文档归位、`.gitignore` 修正 | **PASS** |
| P2 | 复制旧四包 | **NOT RUN**（本机无源，见 §3） |
| P2 | 首次最小编译 | **NOT RUN**（本机无 ROS 2 Jazzy） |
| P3 | ROSIDL 接口契约 | **PASS**（静态自检；官方生成器 `NOT RUN`） |
| P4 | 双层 FSM、工具策略、执行层、安全层 | **PASS**（离线单测） |
| P5 | SDK 桥接（disabled stub） | **PASS**（离线单测；真实通路禁止） |
| P6 | 回归与验收 | **PARTIAL**（离线单元 255/255、静态自检 79/79、Mock 端到端 15/15 全 PASS；`colcon test` 与仿真回归 `NOT RUN`） |

**一句话：** 新工程可独立构建（依赖层面无旧工程绝对路径），双层状态机与 Mock 执行链路在离线环境可运行并通过单元测试；**实机抓放、完整轨迹执行、仿真回归均未验证**。

---

## 1. 工作空间与版本控制

| 项 | 值 |
|---|---|
| 当前路径 | `/home/robot/meituan_challenge_ws`（2026-10-09 用户指示迁至此处，见 §1.1） |
| `realpath` | `/home/robot/meituan_challenge_ws` |
| `git rev-parse --show-toplevel` | `/home/robot/meituan_challenge_ws` |
| remote | `origin https://github.com/F1u0rite/meituan_challenge_ws.git`（fetch/push） |
| 克隆基线 HEAD | `24f08a3143051ba0cdc89378ef234e740eb78ac2`（"Add initial files"） |
| 独立 `.git` | GitHub clone 自带，**未 `git init`、未重新克隆、未删除/替换**；路径变更采用整体移动，提交历史完整 |
| 旧工程写入 | **0 次**（旧工程路径在本机不存在，见 §3） |

### 1.1 路径变更历史（如实记录，含历史命令的真实路径）

**第一阶段（P1，原定路径）：** V3 提示词与 README 原稿要求固定工作目录为 `/home/meituan_challenge_ws`。该路径在 `/home` 根目录下，属主为 `root`（`drwxr-xr-x`），普通账户不可写。经用户**明确授权**后执行了最小必要操作（**此处记录的是当时实际执行的命令与当时的路径**）：

```bash
sudo mkdir -p /home/meituan_challenge_ws
sudo chown aaet:aaet /home/meituan_challenge_ws
sudo chmod 755 /home/meituan_challenge_ws
```

**未**对 `/home` 整体 `chmod`，**未**改动其他系统路径；之后以 `aaet` 身份克隆并完成 P0–P6（4 个 commit）。

**第二阶段（2026-10-09 用户指示，路径迁移）：** 用户反馈 `/home` 下入口不便操作，指示把工程整体合并到 `/home/robot/meituan_challenge_ws`。由于 `/home` 根目录的删除/移动对普通账户受限，执行了：

```bash
# 1) 先备份目标位置原有的临时克隆（仅基线 7 个文件，无任何开发内容）
mv /home/robot/meituan_challenge_ws /home/robot/meituan_challenge_ws_temp_clone_backup
# 2) 同文件系统内整体移动完整工程（保留 .git 历史）
sudo mv /home/meituan_challenge_ws /home/robot/meituan_challenge_ws
```

**核验结果：** `realpath` 与新路径一致；`git rev-parse --show-toplevel` 精确返回新路径；HEAD 仍为最终提交；`git status --porcelain` 为空；`find -not -user aaet` 无输出（全部归 `aaet`）；旧路径已不存在。

**该路径变更偏离** V3 提示词 §1.1“禁止改建到 `~/meituan_challenge_ws`”，属**用户显式豁免**。`docs/prompts/DSH_AUBO_S3_独立Workspace重构提示词_v3.md` 原文与哈希**未被修改**（`6f058481…`），偏离已登记在仓库 `README.md` §2.1。

**说明：** 位于 `/home/robot/meituan_challenge_ws_temp_clone_backup` 的临时克隆备份在本次迁移中**保留未删除**，仅含基线 7 个文件，可用于核对或删除。

---

## 2. 目录树（实际）

```text
/home/robot/meituan_challenge_ws/
├── .gitignore                     # ROS 2/colcon 缺口补全 + 源码误忽略修复
├── LICENSE
├── README.md
├── config/
│   ├── manipulation.yaml          # 新编写，数值含「待标定」注释
│   ├── motion_profiles.yaml
│   ├── safety.yaml                # 设备身份白名单留空占位
│   ├── tool_v1.yaml
│   └── tool_v2.yaml
├── docs/
│   ├── architecture/
│   │   ├── AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md   # 原文未改
│   │   ├── AUBO_S3_ROS2_状态机与接口技术设计_v0.1.docx # 原文未改
│   │   ├── CONTROL_FSM_AND_ROS2_INTERFACES.md          # 入口说明
│   │   ├── FSM_IMPLEMENTATION.md                       # FSM 落地说明
│   │   └── TF_CONVENTIONS.md                           # 坐标/单位约定
│   ├── migration/
│   │   ├── SOURCE_AUDIT.md
│   │   ├── MIGRATION_MAP.md
│   │   ├── INTERFACE_CHANGELOG.md
│   │   └── IMPLEMENTATION_REPORT.md   # 本文件
│   ├── prompts/DSH_AUBO_S3_独立Workspace重构提示词_v3.md  # 原文未改
│   └── reference/AUBO_S3.md                              # 原文未改
├── scripts/offline_selfcheck.py   # 无 ROS 2 环境下的静态自检
├── src/
│   ├── mtc_interfaces/            # ROSIDL 契约（新建）
│   ├── mtc_motion_execution/      # 运动执行 + Mock 后端（新建）
│   ├── mtc_aubo_bridge/           # SDK 桥接，默认 disabled（新建）
│   ├── mtc_task/                  # 任务 FSM（新建）
│   ├── mtc_manipulation/          # 抓放 FSM（新建）
│   ├── mtc_tool/                  # V1/V2 工具策略（新建）
│   ├── mtc_safety/                # 联锁与故障监控（新建）
│   ├── mtc_bringup/               # 统一启动、配置加载与 Mock 组合（新建）
│   ├── mtc_motion_planning/       # 占位（无 package.xml）
│   ├── mtc_description/           # 占位（无 package.xml）
│   └── mtc_simulation/            # 占位（无 package.xml）
└── tests/                         # 跨包离线端到端测试
```

**占位目录刻意不含 `package.xml` / `CMakeLists.txt`**，因此不会被 colcon 识别为功能包——避免“空包被当作已迁移完成”的误导。

---

## 3. 迁移结果（摘要）

完整映射见 [`MIGRATION_MAP.md`](./MIGRATION_MAP.md)。

| 源 | 目标 | 操作 | 原因 |
|---|---|---|---|
| `/home/operator/meituan_challenge/src/mtc_interfaces` | `src/mtc_interfaces` | **未复制** | 源缺失（全盘无命中） |
| `/home/operator/meituan_challenge/src/mtc_motion_planning` | `src/mtc_motion_planning` | **未复制** | 源缺失；**规划算法未被重写** |
| `/home/operator/meituan_challenge/src/mtc_description` | `src/mtc_description` | **未复制** | 源缺失；不臆造 URDF/网格 |
| `/home/operator/meituan_challenge/src/mtc_simulation` | `src/mtc_simulation` | **未复制** | 源缺失；仿真回归 `NOT RUN` |
| 仓库根 4 份文档 | `docs/{reference,architecture,prompts}/` | `git mv` | 内容不变，哈希前后一致 |
| `/home/robot/aubo_s3_nuc_smoke/hardware/*` | 未复制 | **只读参考** | 抽取 SDK/RTDE/停止处理经验；不复制裸法兰零 TCP 假设与 J6 固定目标 |

**为什么没有“迁移前后哈希对照”：** 源文件不存在，无法产生对照。为避免用文档记述冒充代码证据，**未做任何推测性哈希**。

**关键保留声明：**
- 旧工程的**运动规划与动力学成果未被重写**——`mtc_motion_planning` 保留为占位，未新建替代规划器；
- **未重写 `PlanMotion.action`**，未将其字段臆测后写入新接口；
- **未把离散 `moveJoint` 调用当作完整轨迹控制器**：后端能力显式声明 `timed_trajectory_supported=false`，且有测试断言“一个请求只产生一次后端调用”。

---

## 4. 接口契约

完整清单与逐字段差异见 [`INTERFACE_CHANGELOG.md`](./INTERFACE_CHANGELOG.md)。

### 4.1 新增接口

- **Action 3 个**：`ExecuteTask` / `PickPlace` / `ExecuteJointMove`
- **Message 6 个**：`BatteryDetection` / `BatteryDetectionArray` / `ToolState` / `TaskState` / `RobotState` / `ErrorCodes`
- **Service 3 个**：`GetMotionCapabilities` / `TriggerUnlock` / `RequestStop`

### 4.2 相对 V0.1 草案的变更（全部为追加或语义明确化，无破坏性改动）

| 编号 | 变更 | 类型 |
|---|---|---|
| C1 | `ExecuteTask.Goal` 追加 `placement_stability_sec` | 追加（兼容） |
| C2 | `ExecuteTask.Feedback` 追加 `substate` | 追加（兼容） |
| C3 | `PickPlace.Goal` 追加 `placement_stability_sec` | 追加（兼容） |
| C4 | 新增 `ErrorCodes.msg`（只含常量） | 新增 |
| C5 | `ExecuteJointMove` 增加 `COMMAND_OK/REJECTED/UNKNOWN` 常量 | 追加（兼容） |
| C6 | `execution_state` 取值集合文档化为执行状态枚举 | 语义明确化 |

### 4.3 未定义的既有接口

`PlanMotion.action`、`Latch.srv` **未定义、未重定义**——源缺失，且设计文档明文要求“不擅自重定义”。新接口使用不同名称，**不存在同名不同义冲突**。

### 4.4 验证

| 项 | 状态 |
|---|---|
| 离线静态自检（分隔符/字段/常量/CMake 一致性/package.xml/Python 语法/gitignore） | **PASS 57 / FAIL 0** |
| `colcon build --packages-select mtc_interfaces` | **NOT RUN** |
| `ros2 interface show` | **NOT RUN** |

---

## 5. 状态机与安全语义（实现要点）

完整说明见 [`FSM_IMPLEMENTATION.md`](../architecture/FSM_IMPLEMENTATION.md)。

| 层 | 包 | 状态机 | 源码入口 |
|---|---|---|---|
| 任务层 | `mtc_task` | `INIT→SELF_CHECK→WAIT_TASK→VALIDATE_TASK→SCAN_SCENE→EXECUTE_ITEM→RECORD_RESULT→FINISHED`（+ `RECOVERY`/`CANCEL_PENDING`/`TASK_FAILED`/`FAULT`/`CANCELLED`） | `mtc_task/task_fsm.py`（无 rclpy） |
| 抓放层 | `mtc_manipulation` | `RESOLVE_TARGET→…→VERIFY_PLACED→SUCCESS`（+ `RECOVERY`/`FAILED`/`FAULT`/`CANCEL_PENDING`/`CANCELLED`） | `mtc_manipulation/pick_place_fsm.py`（无 rclpy） |
| 工具层 | `mtc_tool` | V1/V2 策略 + 证据等级 | `mtc_tool/strategy.py`、`manager.py` |
| 执行层 | `mtc_motion_execution` | `IDLE/ACCEPTED/MOVING/SETTLED/CANCELLING/STOPPED/STOP_UNKNOWN/TIMEOUT/REJECTED/FAULT` | `mtc_motion_execution/executor_core.py` |
| 安全层 | `mtc_safety` | `DISCONNECTED/NOT_READY/READY/MOVING/STOP_REQUESTED/STOPPED/FAULT` | `mtc_safety/interlocks.py` |

**已被测试锁定的关键安全语义：**

1. `placement_verified` 仅由 `VERIFY_PLACED` 置位；
2. 未验证挂载禁止运输（`ATTACH_NOT_VERIFIED=310`）；
3. 未确认落座禁止解锁（`SEAT_NOT_VERIFIED=400`），且**不触发 IO**；
4. V2 解锁失败/结果未知 → `FAULT` + `RELEASE_NOT_VERIFIED=420`，**禁止强行抽出**；
5. `TriggerUnlock.accepted=true` **不等于**已解锁；
6. V1 路径**零电磁 IO 依赖**（断言调用次数为 0）；
7. 运动状态未知 → `FAULT`(230) **禁止自动重发**；仅“从未开始运动”的 `REJECTED` 允许立即重发；
8. 取消 ≠ 停稳：须等 `stop_confirmed`，否则 `CANCEL_NOT_CONFIRMED=520`；
9. 停止请求送达 ≠ 已确认停稳；状态未知时**停止确认本身不足以解除锁定**，需 `state_reconciled` 或人工 `clear_by_human(note)`；
10. 稳定性观察未满 `placement_stability_sec`（默认 3.0 s）不得标记成功；
11. 后端能力不足即**拒绝**，不做“变形执行”；未知后端名一律落到 `disabled`；
12. `joint_names` 必须与 `target_rad` 索引一一对应且与配置一致，**禁止假定固定 J1..J6 顺序**。

---

## 6. 测试结果（真实输出）

所有测试均为**离线纯 Python**（不 `import rclpy`、不连接设备），可在无 ROS 2 环境运行。

### 6.1 各包离线单元测试与集成测试（实际执行，2026-10-09）

全部为纯 Python 3（**不 import rclpy**、不连接任何设备），可在无 ROS 2 环境的机器上复现。

| 测试套件 | 命令 | 通过 / 总数 | 退出码 |
|---|---|---|---|
| `mtc_motion_execution` | `python3 test/test_backends.py` | **20 / 20** | 0 |
| `mtc_aubo_bridge` | `python3 test/test_bridge_core.py` | **25 / 25** | 0 |
| `mtc_task` | `python3 -m pytest test/ -q` | **56 / 56** | 0 |
| `mtc_manipulation` | `python3 -m pytest test/ -q` | **45 / 45** | 0 |
| `mtc_tool` | `python3 -m pytest test/ -q` | **39 / 39** | 0 |
| `mtc_safety` | `python3 -m pytest test/ -q` | **70 / 70** | 0 |
| **单元测试合计** | — | **255 / 255** | — |
| 离线静态自检 | `python3 scripts/offline_selfcheck.py` | **79 / 79** | 0 |
| 跨包 Mock 端到端 | `python3 tests/test_mock_end_to_end.py` | **15 / 15** | 0 |

**端到端最终输出（原文）：**

```text
用例总数 15：PASS 15 / FAIL 0 / SKIP 0
E2E RESULT: PASS 15 / FAIL 0 / SKIP 0
全部 PASS 均为 Mock PASS，非实机抓放成功。
```

**端到端覆盖的必测故障用例（设计 §10.1）：** 三色序列槽位映射、并发第二个 Task/Motion Goal 拒绝、颜色错误/重复 ID/位姿过期不运动、规划成功但执行失败不记成功、`moveJoint` 超时且可能仍在动不自动重发、试提未跟随禁止运输、未落座禁止 V2 解锁、解锁已受理但未解除不撤离、取消携带中 Action 的不确定负载保护、能力/模式变化使旧规划作废、非目标电池进入扫掠区被拒、稳定性未满 3 秒不得成功、V1 不依赖电磁 IO、配置交叉校验、联锁守卫。

### 6.2 过程中发现并修复的真实缺陷

| 编号 | 缺陷 | 严重度 | 修复 | 回归测试 |
|---|---|---|---|---|
| B1 | **FSM 未校验感知颜色**：`_h_resolve_target` 只用 `found/unique/stale`，`pose.color` 从未与 `expected_color` 比对；注释声称会校验但未实现，导致“期望蓝色、实际红色”仍会走到抓取动作 | **高**（抓错颜色/抓错物体） | 在 FSM 层加入颜色双重校验：颜色不匹配或 `COLOR_UNKNOWN=0` 均返回 `TARGET_NOT_FOUND=110`；`FakePerception` 同步按颜色不符视同“未找到” | `test_color_mismatch_blocks_motion_and_reports_110`、`test_unknown_color_blocks_motion`、`test_matching_color_still_reaches_planning` |
| B2 | 端到端测试用例 08/09 用 `PickPlaceState`（普通 `Enum`）与 `str` 比较，断言恒为 `False`（信息打印与实际值一致，掩盖了判定失败） | 中（测试有效性） | 测试侧统一比较 `.value` | 用例 08/09 现为 PASS |
| B3 | 端到端用例 12 假设稳定性窗口起点早于实现（实现以**首次有效观测**起算），断言与设计语义不符 | 低（测试期望） | 测试改为按实测累计值断言，并验证“恰好达标后才成功” | 用例 12 现为 PASS |
| B4 | 端到端用例 05 期望超时错误码 `220`，实现在**运动在途超时**（机器人可能仍在动）时保守返回 `MOTION_STATUS_UNKNOWN=230` | 低（测试期望） | 判定实现更保守、符合 V0.1 §6.4 语义，改为断言 `230` 并记录差异 | 用例 05 现为 PASS |
| B5 | 端到端用例 11 的规划拒绝记录为列表 `['sweep_zone_occupied:...']`，测试以“包含子串”方式断言失败 | 低（测试期望） | 测试改为对列表逐项判断子串 | 用例 11 现为 PASS |

> **注：** B2–B5 为测试自身缺陷/期望偏差，**不是**被掩盖的产品缺陷；B1 是真实安全缺陷，已修复并有回归测试。B4 的处理原则是“更保守的实现优先于测试期望”。

### 6.3 关键安全语义的测试锚点

| 安全语义 | 覆盖测试 |
|---|---|
| 颜色不匹配/未知（`COLOR_UNKNOWN`）禁止运动 | `mtc_manipulation` 回归测试（B1） |
| 未验证挂载禁止运输（310） | `mtc_manipulation`、端到端用例 06 |
| 未落座禁止解锁（400），且不触发 IO | `mtc_manipulation`、`mtc_tool`、端到端用例 07 |
| 解锁受理 ≠ 已解锁，无独立证据不得撤离（410/420） | `mtc_tool`、`mtc_aubo_bridge`、端到端用例 08 |
| V1 全程零电磁 IO | `mtc_tool`、端到端用例 13 |
| 运动状态未知禁止重发（230） | `mtc_motion_execution`、`mtc_safety`、`mtc_aubo_bridge`、端到端用例 05/15 |
| 取消 ≠ 停稳，未确认停稳不得宣告安全停止（520） | `mtc_motion_execution`、`mtc_safety`、`mtc_manipulation`、端到端用例 09 |
| 稳定性未满 3 秒不得标记成功 | `mtc_manipulation`、端到端用例 12 |
| 后端能力不足即拒绝，未知后端名落到 disabled | `mtc_motion_execution`、`mtc_aubo_bridge` |
| 设备身份白名单留空即拒绝建立运动通路 | `mtc_aubo_bridge`、端到端用例 14/15 |
| 状态未知后停止确认本身不足以解锁 | `mtc_aubo_bridge`（`state_reconciled` 语义） |

### 6.1 未执行项（`NOT RUN`，不得表述为通过）

| 项 | 原因 |
|---|---|
| `colcon build` / `colcon test` / `colcon test-result` | 本机仅有 ROS 2 **Humble**（`ROS_DISTRO=humble` 已预置），无 Jazzy；Humble 安装在 Noble 属非支持组合；且按项目约定不做编译验证 |
| `ros2 interface show` | 同上 |
| Gazebo 仿真回归 | 无 `mtc_simulation` 源码、无 Jazzy、无可用 Gazebo 基线 |
| 真实 AUBO S3 任何操作 | **未获现场授权**；安全默认禁止 |

### 6.4 集成一致性核验（本轮新增，均已固化为自检项）

离线单测与端到端测试覆盖不到“安装/装配层面”的断裂，因此补充了四类静态核验，
并全部纳入 `scripts/offline_selfcheck.py`，避免日后回归：

| 核验项 | 方法 | 结果 |
|---|---|---|
| **入口点可解析性** | 解析各 `setup.py` 的 `console_scripts`，检查目标模块文件是否存在、函数是否定义 | 8/8 存在。**修复 1 处真实断裂**：`mtc_aubo_bridge` 声明 `aubo_bridge = mtc_aubo_bridge.bridge_node:main`，但 `bridge_node.py` 从未创建（安装后 `ros2 run` 必然失败）——已补齐该节点外壳 |
| **错误码镜像一致性** | 实际导入各纯 Python 模块，比对常量数值与 `ErrorCodes.msg` | `mtc_tool/codes.py` 18/18、`mtc_safety/interlocks.py` 18/18、`mtc_manipulation/pick_place_fsm.py` 18/18、`mtc_aubo_bridge/bridge_core.py` 5/5（该模块只用到 5 个码）全部一致 |
| **关节名一致性** | 比对 6 处配置/实现文件中的 `*_joint` 名称集合 | 6/6 完全一致（`shoulder_joint`/`upperArm_joint`/`foreArm_joint`/`wrist1_joint`/`wrist2_joint`/`wrist3_joint`） |
| **文档链接与齐备性** | 核对 V3 §2 要求的架构与迁移文档 | `CONTROL_FSM_AND_ROS2_INTERFACES.md`、`TF_CONVENTIONS.md`、`FSM_IMPLEMENTATION.md` 与四份迁移文档全部存在 |

**同时修复一处文档与代码不一致：** `mock_bringup.launch.py` 的注释声称
`mtc_safety` 的节点外壳“尚未落地”，并将 `start_safety_supervisor` 默认置为 `False`。
该外壳实际已存在，现已改为默认启动并把就绪性说明更新为实测结论（`mtc_aubo_bridge`
的 `bridge_node` 已落地但不属于 Mock 组合，故不在该 launch 中启动）。

**依赖可用性（实测）：** 在现有 ROS 2 Humble 中 `std_msgs`、`geometry_msgs`、
`builtin_interfaces`、`rosidl_default_generators`、`sensor_msgs`、`ament_cmake`
均可用；`PyYAML 6.0.1` 可用（`mtc_bringup` 配置加载与校验依赖它）。

**launch 引用核验：** `mock_bringup.launch.py` 的 `COMPOSITION` 表引用的 5 个可执行文件
（`motion_executor`/`task_executor`/`pick_place_server`/`tool_manager`/`safety_supervisor`）
均已在对应包中声明且入口点可解析；`real_bringup.launch.py` 与 `sim_bringup.launch.py`
不含任何 `Node` 启动，只打印拒绝/NOT RUN 说明，符合安全边界。


---

## 7. 安全与合规

| 约束 | 执行情况 |
|---|---|
| 旧工程只读 | ✅ 未写入、未修改（旧工程本机不存在，取证见 `SOURCE_AUDIT.md` §4） |
| 不重新初始化 Git | ✅ 使用 clone 自带 `.git` |
| 不擅自更换工程路径 | ✅ 最终落在 `/home/robot/meituan_challenge_ws` |
| 不控制真实机械臂 | ✅ 全程离线；`mtc_aubo_bridge` 默认 disabled；无任何设备连接 |
| 不触发末端电磁铁 | ✅ `DisabledUnlockIo` / `DisabledSdkWorker` 拒绝一切 IO 调用 |
| 不提交凭据 | ✅ 新增文件中无任何凭据；`.gitignore` 已排除 `.env`/`*.local.*`/`secrets/`/`credentials/`/`*.pem`/`*.key`/`*.token` |
| 不 `git push` | ✅ 仅本地提交 `3145cbd`，未 push、未改写远端历史 |

### 7.1 ⚠️ 未处置的安全风险（需人工决策）

公开仓库 `docs/reference/AUBO_S3.md` 第 39–47 行等位置含**明文设备凭据**（SSH/sudo/SDK 登录口令等），该文档第 7 行自述“不上传公开 GitHub”，但仓库实际为 **Public**。

按用户决策，本次**暂不脱敏**，仅登记风险：
- **未**把任何凭据值复制到新文件、代码、配置或日志；
- 建议后续：轮换受影响凭据、评估 Git 历史暴露、必要时生成脱敏副本。

---

## 8. 已知风险与限制

| 编号 | 风险 | 影响 |
|---|---|---|
| R1 | 旧四包源码不可得 | 迁移、哈希对照、旧接口字段核实均无法完成 |
| R2 | 无 ROS 2 Jazzy | 全部编译/接口生成/`colcon test` 未经执行 |
| R3 | `PlanMotion.action` / `Latch.srv` 字段未核实 | 新旧接口最终对齐需二次评审 |
| R4 | 公开仓库含明文凭据 | 凭据暴露风险（见 §7.1） |
| R5 | 真实 S3 轨迹执行能力未验证 | 不得声明支持 `FollowJointTrajectory` |
| R6 | 工具 TCP/质量/碰撞几何与 V2 电磁参数未标定 | 配置数值为占位值，禁止用于真实运动 |
| R7 | 感知节点（`mtc_perception`）未实现 | FSM 通过注入端口消费感知结果；无真实视觉链路 |
| R8 | `mtc_tool` 与 `mtc_manipulation` 各自定义了工具策略接口 | 存在重复定义，需在接口冻结前统一（见 §10） |

---

## 9. 离线复现命令（无危险动作）

```bash
# 1) 静态自检（接口结构、包元数据、Python 语法、gitignore）
cd /home/robot/meituan_challenge_ws
python3 scripts/offline_selfcheck.py

# 2) 各包单元测试（不需要 ROS 2）
(cd src/mtc_motion_execution && python3 test/test_backends.py)
(cd src/mtc_aubo_bridge && python3 test/test_bridge_core.py)
(cd src/mtc_tool && python3 -m pytest test/ -q)
(cd src/mtc_safety && python3 -m pytest test/ -q)

# 3) 跨包离线端到端（Mock）
python3 tests/test_mock_end_to_end.py

# 4) 配置校验
PYTHONPATH=src/mtc_bringup python3 -m mtc_bringup.validate_config
```

**Jazzy 环境下的完整验收（待现场或具备 Jazzy 的机器执行）：**

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon list
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

---

## 10. 下一步需要现场授权的实机验收清单（**均未执行**）

> 以下每一项都需要**单独明确授权**，并在现场完成安全检查。远程操作不等于实体急停。

1. **环境一致性**：确认 ARCS / SDK / 接口版本与交接记录一致或记录差异；
2. **只读连接**：验证身份读取、关节状态、控制模式、告警接入（不运动）；
3. **RTDE 时效**：标定反馈新鲜度上限与停稳判据（不得照搬 J6 历史数值）；
4. **单关节小范围运动**：低风险姿态、低速、受监护，逐关节验证目标运动能力；
5. **停止实测**：验证停止请求实际效果与“停稳确认”判据（**软件停止不替代急停**）；
6. **工具标定**：V1/V2 的 TCP、质量、重心、碰撞几何、退钩净空；
7. **V2 电气**：DO 通道、电压/电流、脉冲时长、解锁反馈来源、断电保持策略；
8. **视觉与 TF**：相机安装确认、手眼标定、`ring_frame` 轴向定义；
9. **V1 无视觉取放**：固定标定位姿的完整抓放闭环；
10. **V2 接入**：机械锁止 + 解锁联锁 + I/O 证据；
11. **故障路径实测**：断线、取消、抓取失败、掉落等路径。

**统一前置：** 完成以上任一实机动作前，必须先通过 `identity_check()`（设备身份白名单）与 `health_check()`（RPC/RTDE 可用且反馈新鲜）。

---

*本报告中的 PASS/FAIL 均来自实际执行并保存输出；未执行项一律标记 `NOT RUN`，不虚构通过。*
