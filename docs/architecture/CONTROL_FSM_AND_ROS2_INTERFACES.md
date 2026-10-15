# 控制状态机与 ROS 2 接口（入口说明）

> 本文件是**索引入口**，不是设计正文。

## 目标架构依据

正式设计文档为（**内容未被修改**）：

- [`AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`](./AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md)
  - SHA256：`802c1986626d9c3e722fabf981b5855e5331408e247eb4942b2d3083d1917155`
  - 状态：**V0.1 设计评审稿**，不是已编译验证的现有 API
- 同名 `.docx` 为同一文档的 Word 版本，内容未改动。

## 相关文档

| 文档 | 作用 |
|---|---|
| [`TF_CONVENTIONS.md`](./TF_CONVENTIONS.md) | 坐标树、TCP、工位与单位约定（数值待标定） |
| [`FSM_IMPLEMENTATION.md`](./FSM_IMPLEMENTATION.md) | 双层 FSM 的**落地实现**说明（状态、事件、守卫、错误路径） |
| [`../migration/INTERFACE_CHANGELOG.md`](../migration/INTERFACE_CHANGELOG.md) | 接口相对 V0.1 的逐字段差异与兼容性策略 |
| [`../migration/IMPLEMENTATION_REPORT.md`](../migration/IMPLEMENTATION_REPORT.md) | 实施、测试与验收记录 |

## 冲突处理原则（V3 §0）

> 安全与路径隔离约束**优先**；源代码代表**现存事实**；架构文档代表**目标设计**；差异必须记录，不能默默覆盖。

本次实施中出现的实际冲突与处理：

| 冲突 | 处理 | 记录位置 |
|---|---|---|
| V0.1 要求保留 `PlanMotion.action` / `Latch.srv`，但旧源码在本机不存在 | **不重定义**、不臆造字段；新接口改用不同名称；对齐清单待补 | `INTERFACE_CHANGELOG.md` §3、§5 |
| V0.1 §7.2 使用 `/mtc/safety/stop_motion`，本实现绑定 `/mtc/motion/request_stop` | 语义相同，作为**显式偏差**登记；可由 bringup 重映射对齐 | `INTERFACE_CHANGELOG.md` §4.1 |
| 设计基线为 ROS 2 Jazzy，本机仅有 Humble（`ROS_DISTRO=humble` 预置） | 按用户决策**不做编译验证**，全部标记 `NOT RUN` | `SOURCE_AUDIT.md` §2.1 |
| README 要求先脱敏再上传，实际公开仓库含明文凭据 | 按用户决策**暂不脱敏**，仅记录风险；未复制任何凭据值 | `SOURCE_AUDIT.md` §5 |
