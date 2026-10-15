# mtc_interfaces

ROSIDL 接口契约包（Action / Message / Service）。**只定义接口，不含任何真实设备连接能力。**

> 目标架构依据：[`docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`](../../docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md)
> 接口差异与兼容性：[`docs/migration/INTERFACE_CHANGELOG.md`](../../docs/migration/INTERFACE_CHANGELOG.md)

---

## 1. 接口清单

### Action

| 文件 | 用途 | 绑定名称 |
|---|---|---|
| `action/ExecuteTask.action` | 整轮比赛任务 | `/mtc/task/execute` |
| `action/PickPlace.action` | 单块电池抓放 | `/mtc/manipulation/pick_place` |
| `action/ExecuteJointMove.action` | 六关节目标移动（MVP，不承诺轨迹跟踪） | `/mtc/motion/execute_joint_move` |

### Message

| 文件 | 用途 |
|---|---|
| `msg/BatteryDetection.msg` | 单块电池颜色、本体与提环位姿 |
| `msg/BatteryDetectionArray.msg` | 场景全部识别结果 |
| `msg/ToolState.msg` | 末端工具状态与证据等级 |
| `msg/TaskState.msg` | 任务状态快照 |
| `msg/RobotState.msg` | 规约后的设备状态 |
| `msg/ErrorCodes.msg` | 错误码常量容器（**本次新增**，非 V0.1 原文） |

### Service

| 文件 | 用途 | 绑定名称 |
|---|---|---|
| `srv/GetMotionCapabilities.srv` | 查询后端**已实测启用**的能力 | `/mtc/motion/capabilities` |
| `srv/TriggerUnlock.srv` | V2 解锁脉冲请求（受理 ≠ 已解锁） | `/mtc/tool/trigger_unlock` |
| `srv/RequestStop.srv` | 软件停止请求（≠ 紧急停止） | `/mtc/motion/request_stop` |

---

## 2. 关键语义约定

| 约定 | 说明 |
|---|---|
| `placement_verified` | 只有 `VERIFY_PLACED` 通过后才允许为 `true` |
| `ToolState.state` vs `verified` | **两个独立维度**：`STATE_ATTACHED` + `EVIDENCE_COMMAND_ONLY` 时 `verified=false` |
| `TriggerUnlock.accepted` | 只表示脉冲请求受理，**不代表电池已释放** |
| `RequestStop.request_delivered` | 只表示请求送达执行层；`stop_confirmed` 必须另行确认 |
| `GetMotionCapabilities` | 只声明**当前已连接且实测启用**的能力，不因 SDK 文档宣称而置 `true` |
| 颜色枚举 | `COLOR_UNKNOWN=0 / RED=1 / BLUE=2 / YELLOW=3 / GREEN=4`；其他消息引用同一数值集合 |
| `joint_names` | 与 `target_rad` **按索引一一对应**，禁止假定固定 J1..J6 顺序 |
| 单位 | 位置 m、姿态四元数、关节 rad、`*_ms` 毫秒、`*_sec` 秒 |

---

## 3. 未包含的接口（源缺失，刻意不定义）

`action/PlanMotion.action` 与 `srv/Latch.srv` 属于旧工程既有契约，但旧工程源码在本机**不存在**（取证见 [`SOURCE_AUDIT.md`](../../docs/migration/SOURCE_AUDIT.md) §4）。

按设计文档 V0.1 §4.2 与 §12 的要求，**不擅自重定义这两个已存在类型**，因此本包不提供它们，新接口一律使用不同名称以避免同名不同义冲突。待拿到真实源码后按 [`INTERFACE_CHANGELOG.md`](../../docs/migration/INTERFACE_CHANGELOG.md) §5 的对齐清单处理。

---

## 4. 构建与验证

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon build --packages-select mtc_interfaces
source install/setup.bash
ros2 interface show mtc_interfaces/action/ExecuteTask
```

**当前状态：`NOT RUN`。** 本机只有 ROS 2 Humble（`ROS_DISTRO=humble` 已预置），无 Jazzy，按项目约定不做编译验证。替代性静态自检见 [`scripts/offline_selfcheck.py`](../../scripts/offline_selfcheck.py)：

```bash
python3 scripts/offline_selfcheck.py
```

该脚本检查分隔符数量、字段与常量语法、`CMakeLists.txt` 与磁盘接口文件的一致性、`package.xml` 合法性、Python 语法与 `.gitignore` 误忽略——**不能替代** `rosidl` 代码生成。
