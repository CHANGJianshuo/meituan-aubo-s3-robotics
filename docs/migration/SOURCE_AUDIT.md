# SOURCE_AUDIT —— P0 只读源与环境审计

> 阶段：V3 P0（只读盘点）
> 执行时间：2026-10-09（Asia/Hong_Kong）
> 执行者：DSH（ROS 2 架构师 / 迁移工程师）
> 约束执行情况：**未修改、未删除、未移动任何旧工程文件**；旧工程源码在本机不存在，本审计对其缺失做了实测取证。

---

## 1. 结论摘要

| 审计项 | 期望（按 V3 提示词） | 实测结果 | 判定 |
|---|---|---|---|
| 唯一新工作空间 | 当时为 `/home/meituan_challenge_ws`（V3 指定）；**现为 `/home/robot/meituan_challenge_ws`** | 已由 GitHub 克隆建立，`rev-parse --show-toplevel` 精确返回当时路径（见 §3.1.1 的路径迁移记录） | ✅ 符合 |
| 旧比赛工程 | `/home/operator/meituan_challenge` | **不存在**（`/home` 下仅有 `aaet`） | ❌ 缺失 → 迁移阻塞 |
| 旧 WSL 冒烟工程 | `/home/operator/aubo_s3_nuc_smoke` | **不存在** | ❌ 缺失 |
| NUC 旧 SDK 工程 | `/home/robot/aubo_s3_nuc_smoke/hardware` | **存在**（非 Git 仓库，含 `smoke.py`/`telemetry.py`/`j6_45_fast.py` 等） | ⚠️ 只读参考可用 |
| ROS 2 Jazzy | `/opt/ros/jazzy/setup.bash` | **不存在**；仅有 `/opt/ros/humble`，且无 ROS apt 源 | ❌ 不满足设计基线 |
| 旧四包源码 | `mtc_interfaces` 等四包 | **全盘未找到任何副本** | ❌ 无法执行 P2 源码迁移 |

**结论：** P0 的“源文件真实位置、代码接口与依赖”无法通过旧源码核实，只能通过交接文档记述核实。按用户明确决策，本次采用**无源迁移模式**：只新建包与接口，旧四包保留占位并在报告中记 `NOT RUN`，不臆造旧接口字段、不重定义同名类型。

---

## 2. 环境实测快照

| 项 | 实测值 |
|---|---|
| 主机名 | `asusNUC05` |
| 用户 | `aaet` (uid=1000)，属组含 `sudo` |
| 操作系统 | Ubuntu 24.04.4 LTS (Noble Numbat) |
| 内核 | `6.8.0-100-generic` x86_64 |
| Python | 3.12.3（`/usr/bin/python3`） |
| colcon | `/usr/bin/colcon`（版本命令无输出，命令存在） |
| ROS 发行版目录 | `/opt/ros/humble`（仅有 Humble） |
| `ROS_DISTRO` 环境变量 | `humble`（**已预置在环境中**） |
| ROS apt 源 | 未配置（`/etc/apt/sources.list.d/` 无 `packages.ros.org`） |
| 磁盘 | `/` 938G，已用 29G，可用 862G |
| 网络 | GitHub HTTPS 可达（`api.github.com` 200） |

### 2.1 ROS 2 Jazzy 缺失的影响

设计文档与 README 的基线是 **Ubuntu 24.04 + ROS 2 Jazzy**，但：

- Humble 是 Jammy (22.04) 发行版，被安装到 Noble；这属于**混装状态**，不应作为 Jazzy 的替代基线；
- `ROS_DISTRO=humble` 已预置，任何 `ros2`/`colcon` 命令都会落在 Humble 上；
- 无 ROS apt 源，安装 Jazzy 需要 sudo 且需要联网添加源。

**处理决定（已获用户确认）：** 本次**不执行编译验证**，只交付源码与文档。所有 `colcon build` / `ros2 interface show` 结论在 `IMPLEMENTATION_REPORT.md` 中标记为 `NOT RUN`，并给出复现命令。

---

## 3. 新工作空间核验（P1 证据）

```text
realpath                          : /home/robot/meituan_challenge_ws
git rev-parse --show-toplevel     : /home/robot/meituan_challenge_ws
remote origin (fetch/push)        : https://github.com/F1u0rite/meituan_challenge_ws.git
初始 HEAD                         : 24f08a3143051ba0cdc89378ef234e740eb78ac2
初始 HEAD 提交信息                : "Add initial files"
初始 git status --short           : 干净（无未提交修改）
分支                              : main（跟踪 origin/main）
.git 来源                         : GitHub clone 自带，**未执行 git init、未重新克隆、未删除/替换 .git**
```

### 3.1 目录创建权限处理（P1 阶段的 sudo 使用；另见 §3.1.1 的路径迁移）

`/home` 属主为 `root`（`drwxr-xr-x`），普通账户无法在其中建目录。经用户明确授权后执行了**最小必要**操作（**记录当时真实路径**：P1 阶段的工作空间为 V3 提示词指定的 `/home/meituan_challenge_ws`）：

```bash
sudo mkdir -p /home/meituan_challenge_ws
sudo chown aaet:aaet /home/meituan_challenge_ws
sudo chmod 755 /home/meituan_challenge_ws
```

**未**对 `/home` 整体 `chmod`；**未**改动其他任何系统路径。建目录后即以普通用户 `aaet` 身份克隆与写入。

### 3.1.1 后续路径迁移（2026-10-09，用户指示）

用户随后指示把工程迁至 `/home/robot/meituan_challenge_ws`，理由是 `/home` 根目录下的入口不便操作。由于 `/home` 根目录的移动对普通账户受限，再次使用了一次 sudo（**真实执行的命令**）：

```bash
# 1) 备份目标位置原有的临时克隆（仅基线 7 个文件，无开发内容）
mv /home/robot/meituan_challenge_ws /home/robot/meituan_challenge_ws_temp_clone_backup
# 2) 同文件系统内整体移动完整工程（保留 .git 历史与全部提交）
sudo mv /home/meituan_challenge_ws /home/robot/meituan_challenge_ws
```

迁移后核验：`realpath` 与 `git rev-parse --show-toplevel` 均返回新路径；`git status --porcelain` 为空；`find -not -user aaet` 无输出（全部归 `aaet`）；旧路径已不存在。原始文档哈希未变（见 §3.2）。详细记录见 `IMPLEMENTATION_REPORT.md` §1.1。

> §3.2 起的哈希基线采集于 P1 阶段（原路径）；路径变化不影响文件哈希值。

### 3.2 初始文档哈希（迁移前基线）

| 文件（迁移前根目录路径） | SHA256 |
|---|---|
| `README.md` | `0c4f49089b90b20f34aaf2d8bcf3572c3aa0254ff8e24d884fa35832b71e21ff` |
| `.gitignore`（原始模板，51 行） | `1f196a09e41e72f15b9b7d246c3bd1ffbccec06da16c91d2dcc23dfbc89819fb` |
| `AUBO_S3.md` | `a5cf0980cd5e911f29bc416c776e452c4e8a3a25616ac47fa90d15cd2edd7a59` |
| `AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md` | `802c1986626d9c3e722fabf981b5855e5331408e247eb4942b2d3083d1917155` |
| `AUBO_S3_ROS2_状态机与接口技术设计_v0.1.docx` | `cfbd40e48c680ae6a689df2226fc74615c898058e1ef72e3fb7fc43089b30f85` |
| `DSH_AUBO_S3_独立Workspace重构提示词_v3.md` | `6f058481d0c2fa7eeb451a2d80b9b80cc1c24789855d2eef832e833b90192fcf` |

文档迁移后哈希**保持不变**（`git mv` 只改路径不改内容）；`docs/reference/AUBO_S3.md` 迁移后哈希仍为 `a5cf0980...`，可证明文档内容未被改写。

---

## 4. 旧工程缺失的取证过程

以下均为只读命令，未写入任何被检查的目录：

| 检查 | 命令要点 | 结果 |
|---|---|---|
| 目录是否存在 | `ls -la /home/` | 仅 `aaet`，无 `chang` 用户、无 `meituan_challenge` |
| 旧工程路径 | `ls -la /home/operator/meituan_challenge` | `No such file or directory` |
| 全盘包查找 | `find / -xdev -name mtc_interfaces -o -name mtc_motion_planning -o -name PlanMotion.action -o -name trajectory.hpp` | 无任何命中 |
| 归档查找 | `find /home/robot -maxdepth 3 -name '*.tar*' -o -name '*.zip' -o -name '*.7z'` | 仅 miniforge3 的第三方包缓存，无工程归档 |
| 挂载点 | `df -h` | 单一根分区，无外挂工程盘 |

**重要：** 全盘查找命中的 `trajectory.hpp` 是 `/usr/include/kdl/trajectory.hpp`（系统 KDL 库头文件），**不是**旧工程的 `mtc_motion_planning/include/.../trajectory.hpp`，两者无关。

### 4.1 交接文档记述的旧工程事实（**文档证据，非代码证据**）

来源：`docs/reference/AUBO_S3.md`（1034 行）

| 项 | 文档记述 |
|---|---|
| 旧工程根 | `/home/operator/meituan_challenge` |
| 四个包 | `mtc_motion_planning`、`mtc_interfaces`、`mtc_description`、`mtc_simulation` |
| 关键源文件 | `src/planner.cpp`、`trajectory.hpp`、`dynamics.hpp`、`mtc_simulation/src/simulation_system.cpp`、`scripts/demo.py` |
| 既有接口 | `PlanMotion.action`：`start_state` / `target_pose` / `goal_state` / `mode` / `touching_object` / `velocity_scaling` → `success` / `trajectory` / `minimum_clearance` / `maximum_torque_ratio` / `diagnostics_json`；另有 `Latch.srv` |
| 脚本 | `build.sh`、`env.sh`、`run_demo.sh`、`run_demo.py`、`launch_sim.py`、`launch_planner.py`、`capture_frames` |
| 构建 | colcon / CMake |
| 依赖 | MoveIt、Gazebo、Ruckig、B 样条、KDL、CycloneDDS |

**可信度声明：** 上表是交接文档的记述，**未逐文件核实**。设计文档 V0.1 §4.2 亦明确声明“`PlanMotion.action` 与 `Latch.srv` 的真实字段尚未获得源文件，不擅自重定义”。因此本次**不生成**这两个接口的臆测字段版本，详见 `INTERFACE_CHANGELOG.md` §3。

### 4.2 旧 SDK/NUC 工程（本机存在，只读参考）

路径 `/home/robot/aubo_s3_nuc_smoke`，**非 Git 仓库**（无 `.git`），因此无法记录 commit；如需可追溯性应记录文件哈希。

| 关注文件 | 用途 |
|---|---|
| `hardware/smoke.py` | SDK 连接、状态查询、日志 |
| `hardware/telemetry.py` | RTDE 反馈读取 |
| `hardware/j6_45_fast.py`、`hardware/j6_45_fast.py` 系列 | 历史 J6 目标运动实测脚本 |
| `hardware/verify_final.py` | 结果判读 |
| `hardware/.motion.lock`、`hardware/attempts/` | 本地互斥与尝试记录（**非防篡改**，不能替代现场控制权检查） |

**迁移策略：** 只读参考，抽取“连接/身份查询/RTDE 时效/停止与拒绝处理”的工程经验，**不复制**裸法兰 0 kg / 零 TCP 假设与固定 J6 目标，也不把 `j6_45*` 脚本扩展为比赛运动执行器（与设计文档 §11 一致）。

### 4.3 历史 SDK 与 Jazzy Python 的兼容性风险

| 项 | 交接文档记述 |
|---|---|
| 实机 Python | 3.10 |
| 仿真 Python | 3.11 |
| SDK | `pyaubo-sdk 0.24.1` |
| 控制柜 | ARCS `0.25.6-alpha.1+c24e317c`，接口代码 `24000` / 接口 `0.24.0` |
| 通信 | RPC `30004`（登录/状态/`moveJoint`/`stopJoint`），RTDE `30010`（100 Hz，只读） |
| 本机 Python | **3.12.3** |

**风险：** Python 3.10 的 SDK 与 3.12 的 rclpy **不能默认混装于同一进程**。设计文档 §6.2 给出路线 A（原生 ROS 2 驱动）/路线 B（独立 Python 3.10 SDK Worker + IPC Bridge）。本次实现采用**路线 B 的 disabled stub** 形态，真实进程桥接需现场实测后决定。

**已核实的历史实测边界：** 仅验证过**单关节（J6）固定终点目标运动**，**未**验证完整 `FollowJointTrajectory`、笛卡尔伺服、抓放闭环、手眼标定。

---

## 5. 敏感信息审计

| 项 | 结果 |
|---|---|
| `AUBO_S3.md` 自述 | 第 7 行明确写有“这是一份含现场账号密码的本地手册……不上传公开 GitHub” |
| 实际仓库可见性 | **Public**（`api.github.com` 返回 `"private": false`、`"visibility": "public"`） |
| 明文凭据分布 | 第 39–47 行速查表含 SSH / sudo / Wi-Fi / SDK / 校园网多类凭据；另见第 95、173、210、223、266、277、282 行 |
| 本次处置 | 按用户决策**暂不脱敏**，仅在本文与 `IMPLEMENTATION_REPORT.md` 记录风险；**未**把任何凭据值复制到新文件、代码、配置或日志 |
| 已采取的防护 | `.gitignore` 追加 `.env`、`*.local.*`、`secrets/`、`credentials/`、`*.pem`、`*.key`、`*.token` 等排除规则 |

**⚠️ 待人工处置（超出本次授权范围）：**
1. 受影响凭据应轮换（SSH/sudo 口令、SDK 登录口令、控制柜相关口令）；
2. 公开仓库的历史暴露应评估（删除文件不等于清除 Git 历史）；
3. 脱敏副本的生成可在获得授权后进行。

> 本文件及仓库内其他新增文件**均未记录**任何凭据明文值。

---

## 6. `.gitignore` 审计与修正

原模板为 GitHub 自带的 **ROS (catkin)** 模板，存在两处与 ROS 2 场景相关的缺口，实测证据如下：

| 检查路径 | 原始模板判定 | 命中的规则 |
|---|---|---|
| `install/` | **未被忽略** ❌ | 无（模板只有 catkin 的 `devel/`、`logs/`） |
| `log/` | **未被忽略** ❌ | 无（模板的 `logs/` 不覆盖 `log/`） |
| `build/` | 已忽略 ✅ | `build/` |
| `src/mtc_task/mtc_task/lib/helper.py` | **被忽略，误伤源码** ❌ | `lib/`（匹配任意层级） |
| `src/mtc_bringup/bin/tool.py` | **被忽略，误伤源码** ❌ | `bin/`（匹配任意层级） |
| `config/safety.yaml` | 未被忽略 ✅ | — |
| `docs/migration/SOURCE_AUDIT.md` | 未被忽略 ✅ | — |

**修正内容（仅追加，不删除原规则）：**
1. 追加 ROS 2/colcon 相关排除：`install/`、`log/`、`build_isolated/`、`install_isolated/`、`log_isolated/`、`COLCON_IGNORE` 等；
2. 追加 Python 环境与缓存、本地私有配置与凭据、运行时日志与录制产物、编辑器目录的排除；
3. 追加**放行规则** `!**/bin/`、`!**/bin/**`、`!**/lib/`、`!**/lib/**`，修复 catkin 时代规则误伤 ROS 2 源码树的问题。

修正后复验：`src/mtc_task/mtc_task/lib/helper.py` 与 `src/mtc_bringup/bin/tool.py` 已被放行（命中 `!**/lib/**` / `!**/bin/**`），`install/`、`log/`、`build/` 仍被正确忽略。

---

## 7. 遗留风险与待确认项

| 编号 | 风险 | 影响 | 建议 |
|---|---|---|---|
| R1 | 旧四包源码在本机不可得 | P2 源码迁移、哈希对照、旧接口字段核实**无法完成** | 在具备旧工程的机器上重新执行 P0/P2，或提供源码归档 |
| R2 | 无 ROS 2 Jazzy，`ROS_DISTRO=humble` 预置 | 所有编译/接口生成验证 `NOT RUN`；Humble 混装于 Noble 属非支持组合 | 安装 Jazzy 后执行本报告 §8 的复现命令 |
| R3 | `PlanMotion.action` / `Latch.srv` 字段未核实 | 不得臆造字段；新接口与旧接口的最终对齐需二次评审 | 拿到源码后按 `INTERFACE_CHANGELOG.md` §3 执行对齐 |
| R4 | 公开仓库含明文凭据 | 实验室设备凭据已公开暴露 | 轮换凭据、评估历史清理（需用户授权） |
| R5 | 真实 S3 轨迹执行能力未验证 | 不得声明支持 `FollowJointTrajectory` | 现场授权后做小范围能力验证（设计文档 §6.1 表） |
| R6 | 工具 TCP/质量/碰撞几何、V2 电磁电气参数未标定 | 不得用示意值驱动真实运动 | 实物标定后填写 `config/tool_v*.yaml` |
| R7 | 相机型号/手眼标定未确认 | `ring_frame` 实际来源未定 | 现场确认 D435i 安装与标定结果 |

---

## 8. Jazzy 环境下的复现命令（P6 验收待执行）

```bash
# 前置：安装 ROS 2 Jazzy 后
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon list
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

**注意：** 当前环境 `ROS_DISTRO=humble` 已预置，执行上述命令前必须确认已显式切换到 Jazzy，否则会在 Humble 上产生误导性结果。

---

*本审计未在任何旧目录写入文件；所有命令均为只读或仅作用于 `/home/robot/meituan_challenge_ws`。*
