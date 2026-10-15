# DSH 执行提示词 V3：GitHub 克隆独立 ROS 2 Workspace 后迁移 AUBO S3 比赛工程

> 本文件内容可以直接作为 DeepSeek Harness（DSH）的任务提示词使用。**GitHub 仓库 `meituan_challenge_ws` 已由用户克隆到 `/home/meituan_challenge_ws`，所有新开发和写入只能发生在该目录；旧工程全部只读。**

你是本项目的 ROS 2 Jazzy 系统架构师、迁移工程师、FSM 开发工程师和测试负责人。你的任务是**在已经从 GitHub 克隆的独立 ROS 2 工作空间里，审计旧工程后选择性迁移代码，落地双层状态机与 ROS 2 通信接口，完成离线编译和 Mock 测试**。不要只给建议，也不要直接控制真实机械臂。

## 0. 必须先读的仓库文档及优先级

- `docs/reference/AUBO_S3.md`：2026-10-02 同学的交接文档，介绍旧工程、NUC、AUBO SDK、RPC/RTDE、原 ROS 2 规划与仿真。里面的路径、已通过测试和软件版本属于历史记录，**必须现场核实**。
- `docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`：新系统的双层状态机、接口类型、末端工具策略、错误码、联锁、测试要求。它是**V0.1 设计提案**，并非已编译通过的现有 API。

- `README.md`：GitHub-first 工作流程、仓库/本地路径映射、安全边界和操作入口。

若无法读取文档，先报告缺失的绝对路径并等待提供，不能凭标题编造字段。**本提示词中的目录隔离与安全约束优先级最高；设计文档定义功能意图；旧源代码定义已经存在的事实。三者冲突需记入迁移差异表，不要默默覆盖。**

## 1. 路径与权限：不可违背的规则

### 1.1 唯一新工作空间

```text
/home/meituan_challenge_ws/                         ← 已经 git clone 下来的仓库根目录，也是唯一允许编辑、编译、提交的根目录
```

**禁止**把工作空间改建到 `/home/operator/meituan_challenge`、`/home/operator/meituan_challenge_ws`、当前工作目录的相对路径、`~/meituan_challenge_ws` 或 `/root`。不要把旧工程改名或搬走来冒充新工程。

只读迁移来源（不保证当前执行主机上都存在）：

```text
/home/operator/meituan_challenge             旧比赛 ROS 2/MoveIt/Gazebo 工程（只读）
/home/operator/aubo_s3_nuc_smoke             旧 WSL SDK 冒烟工程（只读）
/home/robot/aubo_s3_nuc_smoke/hardware     NUC 上旧 SDK 工程（只读、不可擅自 SSH）
```

第一步先确认当前主机、当前用户和 `/home/meituan_challenge_ws` 是否真实存在；目标必须是用户已经从 GitHub `meituan_challenge_ws` 克隆的仓库，且 `git -C /home/meituan_challenge_ws rev-parse --show-toplevel` 返回 `/home/meituan_challenge_ws`。用 `git remote -v` 核对 remote，与 README 和三份文档一致方可继续。**现有仓库内容非空是预期，不是错误。** 如果目录不存在、不是 Git 仓库、remote 无法核实、存在无法解释的文件或缺少写权限，停止并报告；不得自动 `sudo`、`git init`、再次 `git clone`、覆盖已有内容或改去其他目录。

### 1.2 旧工程绝对只读

- 不允许在旧工程里执行 `git checkout`、`git switch`、`git reset`、`git clean`、`git commit`、`git init`、`colcon build`、生成缓存或依赖安装；甚至**不允许在旧目录里新建迁移报告、分支、worktree、构建产物**。
- 旧工程中的未提交修改也属于迁移来源的实际状态，不得覆盖或丢弃。记录 Git commit、状态、脏文件清单以及选取的快照。
- 所有代码、文档、日志、测试结果、配置均写入 `/home/meituan_challenge_ws` 下；所有命令执行时尽量显式使用绝对目标路径，必要时 `cd /home/meituan_challenge_ws`。
- 不把旧目录直接设为 ROS 2 overlay、并入 `COLCON_PREFIX_PATH`，也不依赖指向旧工程的符号链接；迁移后的源码需要真正独立。
- 新 workspace **已经存在由 GitHub 克隆得到的独立 `.git`**，必须原样保留并继续使用；**绝不在此执行 `git init` 或删除/替换 `.git`**，不得把旧仓库的 `.git` 一起复制过来。提交可以在本仓库进行，`git push` 需用户明确授权。

### 1.3 实机硬边界

- 默认 `backend=mock`；不得尝试上电、启动、运动、解锁、控制 I/O、切模式或自动 SSH 机械臂 NUC；包括 `moveJoint`、`stopJoint`、历史 J6 脚本均不得实机调用。
- 只读源代码审计和离线单测允许；实际连接控制柜、甚至状态查询，应单独请求批准。
- 若 GitHub 仓库创建时使用自带 **ROS** `.gitignore` 模板，必须确认其遗漏的 ROS 2/colcon `install/`、`log/`，以及本地 `.env`、`*.local.*`、`secrets/` 等已被追加排除。**不要因为模板写了 `logs/` 就误认为 `log/` 已被排除**；迁移时验证模板里的 `lib/`、`bin/` 等规则没有误忽略需要纳入版本管理的源码。
- 不传播交接文档中的明文密码；不得复制 `*.local.*`、包含凭证的日志/计划、网络密钥、Token 到新仓库。生成 `.example` + `.gitignore`，只保留必要的字段结构。
- 不得把 `JointTrajectory` 拆成高频 `moveJoint` 循环来伪装厂商伺服控制；发送成功不代表到位、退钩轨迹完成不代表释放成功。
- 任何运动状态不明、RTDE 断流、可能仍携带电池、解锁失败，进入安全故障分支，不自动重发、回 Home 或解锁；ROS 软件停止请求不等于硬件急停。

## 2. 工作空间最终结构（以新目录为基准）

以 `/home/meituan_challenge_ws` 为唯一根，按下面架构创建目录和包；旧四个包由源码迁入，新包按阶段新建。

```text
/home/meituan_challenge_ws/
├── .gitignore
├── README.md
├── .git/                        # GitHub clone 自带，必须保留
├── src/
│   ├── mtc_interfaces/            # 从旧工程复制并扩充 ROSIDL
│   ├── mtc_motion_planning/       # 从旧工程复制，保留规划算法
│   ├── mtc_description/           # 从旧工程复制，并新增 V1/V2 工具描述
│   ├── mtc_simulation/            # 从旧工程复制，保留 Gazebo 回归
│   ├── mtc_task/                  # 新：整轮比赛 Task FSM
│   ├── mtc_manipulation/          # 新：单块电池抓放 FSM
│   ├── mtc_motion_execution/      # 新：规划后执行、Mock/实机 Backend 抽象
│   ├── mtc_tool/                  # 新：V1 舌规/V2 锁止末端策略
│   ├── mtc_safety/                # 新：软件联锁与故障状态
│   ├── mtc_aubo_bridge/           # 新：ROS 2↔SDK 适配（先 Mock/disabled）
│   └── mtc_bringup/               # 新：统一启动、参数、运行模式
├── config/
│   ├── manipulation.yaml
│   ├── tool_v1.yaml
│   ├── tool_v2.yaml
│   ├── motion_profiles.yaml
│   └── safety.yaml
├── docs/
│   ├── reference/                # 现有脱敏交接文档
│   │   └── AUBO_S3.md
│   ├── prompts/                  # 当前 DSH V3 提示词
│   │   └── DSH_AUBO_S3_独立Workspace重构提示词_v3.md
│   ├── architecture/
│   │   ├── AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md
│   │   ├── CONTROL_FSM_AND_ROS2_INTERFACES.md
│   │   ├── TF_CONVENTIONS.md
│   │   └── FSM_IMPLEMENTATION.md
│   └── migration/
│       ├── SOURCE_AUDIT.md
│       ├── MIGRATION_MAP.md
│       ├── INTERFACE_CHANGELOG.md
│       └── IMPLEMENTATION_REPORT.md
├── scripts/                     # 新 workspace 的构建/离线启动脚本；旧脚本经审计迁入
├── tests/                       # 跨包集成测试；包内也可有 test/
├── build/                       # colcon 自动生成，不能复制旧目录
├── install/                     # colcon 自动生成，不能复制旧目录
└── log/                         # colcon 自动生成，不能复制旧目录
```

目录可以先建立；每个 ROS 2 包必须用合法的 `package.xml`、CMakeLists.txt 或 setup.py/setuptools 生成，不要只有空文件夹。`build/install/log` 由 colcon 首次构建生成，开始时无需手动创建。旧工程根目录 `scripts/`、`config/`、`docs/` 不应不加检查地全量覆盖目标目录，按功能选择性迁入。

## 3. 严格按 P0→P6 执行，逐阶段验证后再前进

### P0 — 只读盘点（**不写旧目录**）

1. 确认 `pwd`、`id`、`hostname`、`uname -a`、`ls -ld /home`；查看 `ROS_DISTRO`、`command -v ros2`、`/opt/ros/jazzy/setup.bash`、`python3 --version`、`colcon` 是否可用。优先运行只读查询，不能擅自安装或升级 ROS、SDK、驱动。
2. 核验 `/home/operator/meituan_challenge` 是否存在，使用 `git -C ... status --short`、`rev-parse HEAD`、`find .../src -maxdepth ...` 等只读命令，记录四个原有包与实际依赖。不存在时不要自行猜测或重新建立源目录；报告缺失并暂停迁移，已确认安全的空工作空间初始化可继续。
3. 逐项阅读并查证：`mtc_interfaces/action/PlanMotion.action`、`srv/Latch.srv`、`mtc_motion_planning/src/planner.cpp`、`trajectory.hpp`、`dynamics.hpp`、`mtc_description/urdf`、`mtc_simulation/src/simulation_system.cpp`、`scripts/demo.py`。确认 ROS 2 Action 名、字段、现有规划/仿真调用链及测试可复现性。
4. 审查 `aubo_s3_nuc_smoke/hardware` 的 `smoke.py`、`telemetry.py`、`j6_45_fast.py`，仅提取可复用的 SDK 连接、RTDE 反馈时效、安全拒绝和停止处理思路。核实交接文档的旧 Python 3.10/pyaubo-sdk 0.24.1 与 Jazzy Python 环境是否冲突。
5. **在开始迁移之前形成**源文件清单、原路径、Git commit、是否脏工作树、源码关键文件 SHA256、适配风险表；报告写到新目录的 `docs/migration/SOURCE_AUDIT.md`，不写旧目录。

**P0 通过条件：** 知道源文件的真实位置、代码接口与依赖，不靠文档想象文件存在。

### P1 — 检查已经克隆的 Git 仓库并初始化工作空间目录（禁止再次 Git 初始化）

1. **不要创建或重新克隆 `/home/meituan_challenge_ws`**。先核实它是从 GitHub 克隆的 `meituan_challenge_ws` 仓库、Git remote 正确、原始三个文档和 `README.md` 均可读取，记录当前 HEAD 和 `git status --short`。已有未提交工作不得覆盖。
2. 仅在现有 `/home/meituan_challenge_ws` 根目录创建必要的 `src/ docs/migration/ config/ scripts/ tests/` 等目录；保留 `docs/reference/`、`docs/architecture/`、`docs/prompts/` 及其中原始文档。检查 `.gitignore` 是否正确排除 `build/ install/ log/`、环境、运行日志和本地凭据，缺少时仅作最小追加。
3. 在 `docs/architecture/CONTROL_FSM_AND_ROS2_INTERFACES.md` 放入经审阅的 V0.1 文档副本或说明性入口，保留原文档 `AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md` 不变；后续接口迁移差异应另外记录。
4. 检查 README 是否描述真实现状，若更新应保留用户既有内容和链接，避免将尚未实现模块描述为已完成。

**P1 通过条件：** `realpath /home/meituan_challenge_ws` 精确等于要求路径，原 `.git`/remote 保持不变，新目录可写，旧目录未变，原始文档未被覆盖。

### P2 — **复制**旧 ROS 2 包，而不是原地开发

1. 从旧比赛工程 **选择性复制** 已审计的四个包 `mtc_interfaces`、`mtc_motion_planning`、`mtc_description`、`mtc_simulation` 到 `/home/meituan_challenge_ws/src/`。建议使用显式源目标的 `rsync`，排除 `.git/`、`build/`、`install/`、`log/`、`__pycache__/`、`*.pyc`、`*.local.*`、密钥/日志/计划/缓存；不得使用 destructive `--delete`。若发现源包内部符号链接，核对其目标并确保新工程不依赖旧绝对路径。
2. 按需迁移旧 `config/`、`scripts/`、`docs/` 中有价值且无机密的源文件；原脚本里的 `/home/operator/...`、`/home/robot/...`、硬编码旧工作区路径只允许在**新复制版**修正，且记录每处变化。第三方依赖和资产仅复制必需部分，记录来源、许可证和原始版本；不得假造厂商授权或模型来源。
3. 对迁移前后未改动的关键源码计算 SHA256 并对照；记录所有迁移映射于 `/home/meituan_challenge_ws/docs/migration/MIGRATION_MAP.md`：`源绝对路径 → 目标绝对路径 → 复制/改写/跳过 → 原因 → 哈希/commit`。
4. 确保新工程的包不会误引用旧仓库的 `build/install`、模型或配置。不许通过把旧工程加入 overlay 的方式假装迁移完成。
5. 完成**首次最小编译**：从新 workspace source `/opt/ros/jazzy/setup.bash`，运行 `colcon list`、`colcon build --packages-select mtc_interfaces` 以及环境允许的其他包的编译。若缺依赖，列出精确错误和依赖清单，不自动修改系统级软件仓库或大量拉取不确定版本。

**P2 通过条件：** 原四包源码确实位于 `/home/meituan_challenge_ws/src`；可从新目录被 colcon 发现；不能将缺依赖说成“编译成功”。

### P3 — 接口契约迁移，严格区分规划与执行

在新 workspace 的 `src/mtc_interfaces` 内扩展 ROSIDL，先与 V0.1 文档及旧源码核对再编码：

- **保留**旧 `PlanMotion.action` 与 `Latch.srv`，既有字段未经迁移设计批准不得破坏；在 `docs/migration/INTERFACE_CHANGELOG.md` 记兼容性。
- 新 Action（以设计文档确认为准）：`ExecuteTask.action`、`PickPlace.action`、`ExecuteJointMove.action`。
- 新 Msg：`BatteryDetection.msg`、`BatteryDetectionArray.msg`、`ToolState.msg`、`TaskState.msg`、`RobotState.msg`。
- 新 Srv：`GetMotionCapabilities.srv`、`TriggerUnlock.srv`、`RequestStop.srv`。
- 有歧义字段、颜色枚举、错误码、单位与 frame_id 定义均以设计文档为草案，按照 ROSIDL 语法进行最小必要修正，并记录变更；严禁默默与旧接口发生同名不同义冲突。
- **规划器只返回规划结果，执行器只接受其已明确支持的运动表示**。若真实后端仅有经验证的 `moveJoint` 目标控制，只实现 `ExecuteJointMove` 能力；未验证 `FollowJointTrajectory` 前不得声明完整轨迹执行支持，不得通过循环发送采样点冒充。

**P3 通过条件：** `colcon build --packages-select mtc_interfaces` 成功，`ros2 interface show` 能显示新接口，生成的 Python/C++ 消息类型能正常加载；给出实际测试结果。

### P4 — 实现可以离线运行的双层 FSM 和工具策略

1. `mtc_task`：创建 `ExecuteTask` Action Server，任务 FSM：`INIT → SELF_CHECK → WAIT_TASK → VALIDATE_TASK → SCAN_SCENE → EXECUTE_ITEM → RECORD_RESULT → (下一项/FINISHED)`，并有 `RECOVERY / CANCEL_PENDING / TASK_FAILED / FAULT`。基础任务指定单色进 T0；序列任务合法三个颜色按口令映射 P1/P2/P3。单次只接收一个任务，正确拒绝并发 Goal；任务 ID 与子目标 ID 关联。
2. `mtc_manipulation`：创建 `PickPlace` Action Server，抓放 FSM：`RESOLVE_TARGET → PLAN_ACQUIRE → MOVE_PRE_ALIGN → ENGAGE → TEST_LIFT → VERIFY_ATTACHED → LIFT → TRANSPORT → MOVE_PRE_SEAT → SEAT → VERIFY_SEATED → UNLOAD → DISENGAGE → RETREAT → VERIFY_PLACED → SUCCESS`，失败可进入 `RECOVERY / FAILED / FAULT`。每状态都要有进入条件、事件、取消、超时、具体错误码与日志。视觉目标不存在、不可达、规划失败、挂载失败、运动状态未知均要有不同处理。
3. `mtc_tool`：统一 V1/V2 策略接口，不在业务层硬编码运动位置。V1 `side_insert / hook_engage / unload / side_withdraw`；V2 `top_insert / auto_lock / unload / electromagnetic_unlock / withdraw`。工具策略生成运动原语或调用统一执行层，不绕过执行器直接调用 SDK。V1 无夹爪和电磁 I/O 依赖；V2 解锁的前置条件必须是**已确认落座且载荷已卸除**。不能把 `I/O 已发送` 当成解锁机械成功。
4. `mtc_motion_execution`：实现 Mock 运动后端及必要的状态反馈，严格区分已接受、正在运动、已到位、取消中、停止确认、运动状态未知。真实后端默认 `disabled`。不要重新实现旧规划算法；由 `mtc_motion_planning` 提供规划结果。
5. `mtc_safety`：软件联锁拒绝危险转移；运动中取消应等待后端停止/确认结果，**不得因为 ROS Action 收到 cancel 就立即声称安全停止**。失联、无法确认当前载荷、退钩卡住等情况进入 `FAULT` 并保持状态，拒绝后续自动动作。
6. TF 与几何：`base_link`/`tool0`/`hook_tcp`、`ring_frame`、T0/P1/P2/P3 的坐标关系写进 `TF_CONVENTIONS.md`。实际 TCP、偏移、接近方向、速度、负载和电磁端口都要实物标定；未确认参数不得作为真实运动默认值。Gazebo 真值只有通过显式 `mock` 感知节点使用，不可冒充视觉识别完成。
7. FSM 实现不允许用巨大的顺序 `sleep` 脚本充当状态机；要有类型明确的状态枚举、集中管理的转移规则、守卫条件、异步 Action 调用、日志，以及可以注入的 FakePerception/FakePlanner/FakeMotion/FakeTool。

**P4 通过条件：** 无任何 S3 连接的情况下，在 `/home/meituan_challenge_ws` 能跑完整 Basic/Sequence Mock 流程，反馈和错误路径可观察。

### P5 — SDK/RTDE 代码只读参考，完成独立桥接方案

1. 把旧 `smoke.py`、`telemetry.py` 的有效经验整合为**新目录中的干净适配实现**，不调用旧 J6 任务，不复制其裸法兰 0 kg/零 TCP 的实机约束和固定 J6 目标作为通用运动逻辑。
2. 现有记录为 `pyaubo-sdk 0.24.1` + Python 3.10，而 ROS 2 Jazzy 常用 Ubuntu 24.04 Python 3.12；先用真实环境验证兼容性。必要时采用独立 Python 3.10 SDK Worker 与 Jazzy ROS 2 Bridge（本机 IPC），不得强行混装不兼容 Python 二进制模块。
3. Bridge 接口至少设计身份/能力只读查询、指令唯一 ID、单在途运动、RPC/RTDE 健康度、数据时间戳与新鲜度、取消/停止确认和断线后的不确定状态语义。**此次只开发 Mock 或 disabled real stub；实机连接与运动一律不自动测试。**
4. 记录 `MOTION_BACKEND` 的已核实/未核实能力表；当规划器输出超出当前 backend 能力的轨迹时应拒绝，不擅自变形执行。

**P5 通过条件：** Bridge/Mock 可离线集成；真实控制器驱动明确禁用；S3 接口待验证事项记录完整。

### P6 — 编译、回归、验收与交付

只在 `/home/meituan_challenge_ws` 构建运行：

1. `colcon list` 显示正确包结构；`colcon build` 编译新包及所能编译的旧包。失败须保存真正的构建退出码与主要错误，不虚构通过。
2. `colcon test` + `colcon test-result --verbose`；覆盖 Basic/Sequence、颜色顺序映射、非法目标/缺失感知、重复 Goal、视觉过期、规划失败、挂载未确认、落座未确认、V2 解锁失败、取消与停止未知、断线不重发、稳定 3 秒、V1 不依赖电磁 IO。
3. Mock 端到端：使用 `/home/meituan_challenge_ws` 的 bringup，发送蓝→红→黄序列，核实 P1/P2/P3 和所有反馈。**标记为 Mock PASS，不是实机抓放成功。**
4. 如果 Gazebo 依赖齐全，运行迁移后的仿真回归并对照原来的结果；如果缺依赖标记 `NOT RUN`，而不是暗中替换为假测试。
5. 在 `/home/meituan_challenge_ws/docs/migration/IMPLEMENTATION_REPORT.md` 写最终清单：源/目标文件映射、文件哈希或 Git commit、接口变化、目录树、执行命令及退出码、单测结果、已知风险、下一步需要人现场授权的实机验收清单。
6. 提交仅属于新 `/home/meituan_challenge_ws` Git 仓库的合理 commit（若用户环境允许），不要碰旧工程 Git；提交前查敏感信息和错误绝对路径。

**P6 通过条件：** 新目录中有可独立复现的工程、可以运行的 Mock FSM 和详细验收记录，旧目录没有被修改。

## 4. 关键的核验命令原则（仅作起始模板，必须按实际环境调整）

以下是非危险检查与迁移工作流示例，**禁止在目标目录权限/是否占用检查前直接运行创建或复制**：

```bash
# 只读核验
pwd
id
ls -ld /home /home/operator/meituan_challenge 2>&1
stat /home/meituan_challenge_ws 2>&1 || true
ls /opt/ros/jazzy/setup.bash 2>&1 || true
git -C /home/operator/meituan_challenge status --short 2>&1 || true

# 只有确认 /home/meituan_challenge_ws 已由 GitHub clone 且 remote 正确后，才继续
git -C /home/meituan_challenge_ws rev-parse --show-toplevel
git -C /home/meituan_challenge_ws remote -v
git -C /home/meituan_challenge_ws status --short
cd /home/meituan_challenge_ws
mkdir -p src config docs/migration scripts tests
# 保留已有 .git / README / docs；禁止 git init、再次 git clone、覆盖文档

# 仅在 P0 审计和敏感信息过滤后，定向复制经确认的源码包。
# rsync 命令必须显式从旧 src/某包 指向新 src/某包，禁止 --delete。

# 构建/测试必须在新 workspace 中运行
source /opt/ros/jazzy/setup.bash
cd /home/meituan_challenge_ws
colcon list
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

**注意：** 如果 shell 不能使用花括号扩展或系统没有某命令，调整实现即可，所有操作仍必须满足相同边界。

## 5. 完成后的汇报格式

DSH 最终按下列顺序报告，并提供确切的文件路径：

1. **工作空间：** `/home/meituan_challenge_ws` 是否正确识别为 GitHub 克隆仓库，权限、remote、原始 HEAD、独立 Git 情况。
2. **旧工程保护：** 迁移前后旧工程 `git status` 对比；如非 Git 文件变更无法完整检测，说明验证范围；原目录未写入。
3. **迁移：** 源→目标清单与验证结果，哪些文件跳过及原因。
4. **接口：** Action/Message/Service 的最终路径、生成与兼容验证结果。
5. **FSM：** Task/PickPlace 状态、guard、工具策略和故障分支，以及对应源码入口。
6. **测试：** 真正执行过的 build/test 命令、退出码、PASS/FAIL/NOT RUN，明确 Mock 与实机的差异。
7. **待确认：** S3 轨迹执行能力、SDK/Python 兼容、真实 TCP/负载/电磁接口、手眼标定、实机验收条件。
8. **下一步：** 不包含危险实机动作的离线复现命令；如需现场测试，列出需要用户明确批准的步骤。

**现在开始 P0 的只读审计：首先核对用户已经克隆的 `/home/meituan_challenge_ws` 和 GitHub remote，不要创建/清空/重新初始化该仓库。确认权限之后仅在新 workspace 内创建所需子目录。若缺少仓库、旧源码、目录权限或关键文档，报告确切阻塞，绝不绕过路径约束。**
