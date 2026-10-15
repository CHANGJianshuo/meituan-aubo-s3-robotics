# mtc_motion_planning —— 规划器（迁移占位）

本目录是**迁移占位**，不是已迁移完成的功能包。

## 为什么是空的

按 V3 P2 要求，本包应从旧工程 `/home/operator/meituan_challenge/src/mtc_motion_planning`
**选择性复制**而来。但该路径在本机**不存在**，全盘亦未找到
`planner.cpp` / `trajectory.hpp` / `dynamics.hpp` 的任何副本
（取证过程见 `docs/migration/SOURCE_AUDIT.md` §4）。

## 刻意不做的三件事

1. **没有 `package.xml` / `CMakeLists.txt`**：避免被 colcon 识别为“已迁移完成”的空包。
2. **没有重写规划算法**：旧工程的运动规划与动力学成果是保留对象，未经核实与确认不得重写（V3 §三 明确要求）。
3. **没有臆造 `PlanMotion.action`**：旧接口字段未获源码核实，新接口改用 `ExecuteJointMove` 等不同名称，
   兼容性说明见 `docs/migration/INTERFACE_CHANGELOG.md` §3。

## 补齐后如何接入

1. 在有旧工程的机器上执行只读审计（记录 commit、脏工作树、源码 SHA256）；
2. 用显式源/目标的 `rsync` 复制，排除 `.git/ build/ install/ log/ __pycache__/ *.local.*`；
3. 计算迁移前后哈希并写入 `docs/migration/MIGRATION_MAP.md`；
4. 逐字段对照 `PlanMotion.action` 真实字段，更新 `INTERFACE_CHANGELOG.md` §5 清单；
5. 修正复制版中指向 `/home/operator/...` 的硬编码绝对路径，并逐处记录。

## 当前状态

**迁移状态：NOT RUN**（源缺失）。本工程在无此包的情况下仍可独立 `colcon build`（不依赖旧绝对路径）。
