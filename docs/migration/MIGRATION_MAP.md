# MIGRATION_MAP —— 迁移映射与哈希对照

> 阶段：V3 P2
> 迁移模式：**无源迁移**（旧工程源码在本机不可得，见 `SOURCE_AUDIT.md` §1/§4）
> 原则：不复制生成产物、不复制凭据、不臆造旧接口字段、不做破坏性覆盖

---

## 1. 映射总表

| 编号 | 源路径（绝对） | 目标路径（绝对） | 操作 | 原因 | 哈希 / 证据 |
|---|---|---|---|---|---|
| M1 | `/home/operator/meituan_challenge/src/mtc_interfaces` | `/home/robot/meituan_challenge_ws/src/mtc_interfaces` | **未复制**（源缺失） | 源目录在本机不存在（全盘查找无命中） | 见 `SOURCE_AUDIT.md` §4；期望内容仅见于交接文档记述 |
| M2 | `/home/operator/meituan_challenge/src/mtc_motion_planning` | `/home/robot/meituan_challenge_ws/src/mtc_motion_planning` | **未复制**（源缺失） | 同上；规划算法与动力学成果**未被重写或替换** | 占位说明见 `src/mtc_motion_planning/README.md` |
| M3 | `/home/operator/meituan_challenge/src/mtc_description` | `/home/robot/meituan_challenge_ws/src/mtc_description` | **未复制**（源缺失） | 同上；URDF/网格未迁移，禁止臆造 | 占位说明见 `src/mtc_description/README.md` |
| M4 | `/home/operator/meituan_challenge/src/mtc_simulation` | `/home/robot/meituan_challenge_ws/src/mtc_simulation` | **未复制**（源缺失） | 同上；Gazebo 回归 `NOT RUN` | 占位说明见 `src/mtc_simulation/README.md` |
| M5 | `/home/operator/meituan_challenge/scripts/*` | `scripts/` | **未复制** | 旧脚本含指向 `/home/operator/...` 的绝对路径，且无可核实的源文件 | — |
| M6 | `/home/operator/meituan_challenge/config/*` | `config/` | **未复制** | 同上 | 新配置为本项目**新编写**，见 §4 |
| M7 | `AUBO_S3.md`（仓库根） | `docs/reference/AUBO_S3.md` | **git mv（重排）** | 归位到 V3 §2 规定结构 | 内容未改；SHA256 `a5cf0980cd5e911f29bc416c776e452c4e8a3a25616ac47fa90d15cd2edd7a59`（迁移前后一致） |
| M8 | `AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`（仓库根） | `docs/architecture/...` | **git mv（重排）** | 同上 | 内容未改；SHA256 `802c1986626d9c3e722fabf981b5855e5331408e247eb4942b2d3083d1917155` |
| M9 | `AUBO_S3_ROS2_状态机与接口技术设计_v0.1.docx`（仓库根） | `docs/architecture/...` | **git mv（重排）** | 同上 | 内容未改；SHA256 `cfbd40e48c680ae6a689df2226fc74615c898058e1ef72e3fb7fc43089b30f85` |
| M10 | `DSH_AUBO_S3_独立Workspace重构提示词_v3.md`（仓库根） | `docs/prompts/...` | **git mv（重排）** | 同上 | 内容未改；SHA256 `6f058481d0c2fa7eeb451a2d80b9b80cc1c24789855d2eef832e833b90192fcf` |
| M11 | `/home/robot/aubo_s3_nuc_smoke/hardware/{smoke.py,telemetry.py}` | 未直接复制 | **只读参考** | 抽取 SDK 连接/RTDE 时效/停止与拒绝处理经验；不复制裸法兰 0 kg/零 TCP 假设 | 该目录**非 Git 仓库**，无 commit 可记录 |
| M12 | `/home/robot/aubo_s3_nuc_smoke/hardware/j6_45*.py`、`verify_final.py` | 未复制 | **冻结归档参考** | 设计文档 §11：不再作为比赛运动执行器扩展 | — |
| M13 | 旧工程 `.git/` | 未复制 | **未复制** | 新工程使用 GitHub clone 自带 `.git` | 初始 HEAD `24f08a3143051ba0cdc89378ef234e740eb78ac2` |
| M14 | 旧工程 `build/ install/ log/` | 未复制 | **未复制** | 生成产物，禁止迁移 | — |

### 1.1 为什么没有产生“已迁移源码”的哈希对照

V3 P2 要求“对迁移前后未改动的关键源码计算 SHA256 并对照”。由于源文件不存在，**无法产生迁移前后的哈希对**。为避免用文档记述冒充代码证据，本节不做任何推测性哈希。待旧工程可用时，应补充：

```
源绝对路径 → 目标绝对路径 → SHA256(源) → SHA256(目标) → 是否一致
```

---

## 2. 旧四包占位目录说明

`src/mtc_interfaces` 等四个位置中，**`mtc_interfaces` 是本次新建的真实 ROS 2 包**（承载新增 ROSIDL 契约）。其余三个（`mtc_motion_planning`、`mtc_description`、`mtc_simulation`）为**占位说明目录**，仅含 `README.md`，**不含 `package.xml`、不含 CMakeLists**，因此不会被 colcon 识别为功能包——这是刻意设计，避免出现“空包被当成已迁移完成”的误导。

占位目录内明确写明：源码缺失、原规划与动力学成果未被重写、迁移状态 `NOT RUN`、以及补齐来源后的接入步骤。

---

## 3. 环境路径污染的排查

V3 要求“确保新工程的包不会误引用旧仓库的 `build/install`、模型或配置”。已执行：

| 检查 | 命令 | 结果 |
|---|---|---|
| 是否出现旧绝对路径 | `grep -rn "/home/operator" src/ config/ scripts/ docs/` | 仅出现在**说明性文档**中对缺失源的引用（本文件、`SOURCE_AUDIT.md`、占位 README），源码内**无**硬编码旧路径 |
| 是否出现旧 SDK 目录硬编码 | `grep -rn "aubo_s3_nuc_smoke" src/` | 无 |
| 是否引用旧 overlay | `grep -rn "COLCON_PREFIX_PATH\|/home/operator.*install" src/ config/` | 无 |
| 是否存在符号链接指向旧工程 | `find /home/robot/meituan_challenge_ws -type l` | 无 |

**结论：** 新工程不依赖旧工程绝对路径，可在任意位置独立 `colcon build`。

---

## 4. 新增配置来源（非迁移，为本项目新编写）

| 目标文件 | 来源 | 说明 |
|---|---|---|
| `config/manipulation.yaml` | 本项目新编写 | 结构参照设计文档 §8；数值一律为**未标定保守值**，禁止据其驱动真实运动 |
| `config/tool_v1.yaml` | 本项目新编写 | V1 舌规占位参数，标注“必须实物标定” |
| `config/tool_v2.yaml` | 本项目新编写 | V2 锁止/电磁占位参数，标注“必须实物标定” |
| `config/motion_profiles.yaml` | 本项目新编写 | 各阶段速度/加速度/超时/停稳容差占位值 |
| `config/safety.yaml` | 本项目新编写 | 超时、身份白名单（占位）、联锁开关 |

**不包含任何密码、Token 或真实设备凭据。**

---

## 5. 迁移完成度

| 阶段 | 内容 | 状态 |
|---|---|---|
| P0 | 只读审计 | **PASS**（含源缺失取证） |
| P1 | 目录初始化、文档归位、`.gitignore` 修正 | **PASS** |
| P2 | 复制旧四包 | **NOT RUN**（源缺失，已改为占位 + 文档记录） |
| P2 | 首次最小编译 | **NOT RUN**（无 Jazzy，且按约定不做编译） |
| P3 | 新增 ROSIDL 接口契约 | 见 `INTERFACE_CHANGELOG.md` |
| P4 | 双层 FSM / 工具策略 / 执行层 / 安全层 | 见 `IMPLEMENTATION_REPORT.md` |
| P5 | SDK Bridge（disabled stub） | 见 `IMPLEMENTATION_REPORT.md` |
| P6 | 回归与验收 | 见 `IMPLEMENTATION_REPORT.md` |
