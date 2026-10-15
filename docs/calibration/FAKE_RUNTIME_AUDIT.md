# 常建烁提供的模型 / TF 材料核验与规划交接

2026-10-12。对齐团队 main `f7eaa2c0d2887aeada4fd635112c63c237962ff7`；范围仅 Calibration / TF / TCP。

## 1. 材料来源与结论

`share_for_hanminghe` 的六个文件已逐字节对照 main 归档的 Git blob：URDF、SRDF、三份 TF 日志，以及 TF 图 PDF，全部一致。因此直接复用 [规划证据目录](../planning/evidence/20261011_075828/)，不重复复制。Windows checkout 的文本可能转为 CRLF，内容核验另用 LF 规范化哈希。

PDF [frames_2026-10-11_07.59.05.pdf](../planning/evidence/20261011_075828/frames_2026-10-11_07.59.05.pdf) 已渲染目视核验，SHA256 为 `da6a1c749e4c17b34471aed25aa1dadf4625f57c09867397cfdef1b12bf52ab1`。图中有机器人链，无相机和工具 TCP：

`world → base_link → shoulder_Link → upperArm_Link → foreArm_Link → wrist1_Link → wrist2_Link → wrist3_Link → ee_link`。

**结论：模型与这批 fake TF 读数相互一致，可以作为离线集成依据；没有取得新的实物相机外参、V1 TCP 或工位测量。** URDF 的 hardware 是 `mock_components/GenericSystem`，六关节 position 初值均为 0。这批材料的证据类型记为 `FAKE_RUNTIME_CAPTURE`，不将其转写为真实清单中的 MEASURED。

## 2. 可复算的模型 / 日志一致性

[audit_fake_model.py](../../scripts/calibration/audit_fake_model.py) 用独立 4×4 矩阵实现读取 URDF origin，按 URDF `Rz(yaw) Ry(pitch) Rx(roll)` 合成六关节为 0 时的 FK，不复用待审查的 `geometry.py`。同时检查 mock 初值、SRDF 链、模型固定旋转及三份日志里的全部有效样本。输出保留 LF 规范化 SHA256，适应 Windows checkout 换行；本轮共享原文件与 Git blob 的比较另为逐字节比较。

```bash
python3 -B scripts/calibration/audit_fake_model.py
python3 -B scripts/calibration/run_offline_validation.py
```

完整结果：[fake_model_audit.json](validation/fake_model_audit.json)。日志 translation / quaternion 只打印三位小数；平移分量比较容差为 0.500001 mm，旋转矩阵分量容差为 0.003。这是日志舍入容差，**不是实际定位或姿态精度验收阈值**。

| 检查 | 本轮结果 | 限制 |
| --- | --- | --- |
| SRDF manipulator 链 | `base_link → ee_link` | SRDF 的 base 不等于 MoveIt planning frame |
| MoveIt planning frame | `world` | 采用常的 [运行报告 §1](../planning/PLANNING_HANDOFF.md)，本机没有重新启动 ROS |
| `T_world_base` | 固定单位阵；5 个日志样本一致 | 当前 fake 模型定义，不能推广到有移动底座或另加 world 偏移的部署 |
| `T_wrist3_ee` | 零平移，绕 Z **+π/2** | 从展开 URDF 核验；仍不证明物理法兰标记 F 的方向 |
| 六关节 0 时 `T_base_ee` | p≈`[0,-0.2562,0.8072]` m，q(xyzw)=`[0.5,-0.5,0.5,0.5]` | 是本批 fake 初值的计算；没有随图像同步采集的实机 joint_states |
| base→ee / world→ee 日志 | 各 5 / 7 个样本均与 FK 一致；位置最大分量差 0.2 mm | 两份日志时刻不同，不能当成运动状态的同步验证 |
| TF 图时间 0 / rate 10000 | world→base、wrist3→ee 是固定边，图中时间为 0 | 固定 TF 的零时间不等于观测 Pose 的有效零时间，也不表示真实发布频率为 10 kHz |

日志开头的 `Waiting for transform / frame does not exist` 后面已有成功样本，属于启动阶段记录；它们不证明当前仍缺机器人 TF。图与日志也不能证明采集图像时刻的历史 TF 已可用。

新增回归测试会故意删掉 EE 的 90° 旋转、改反 TF 平移符号、去掉有效日志样本、改错 SRDF tip，均须发现不一致。脚本只生成诊断：`motion_authorized=false`、`real_calibration_verified=false`。

## 3. 交给规划层的参数方向与所有权

当前 [planner.yaml](../../src/mtc_moveit_planner/config/planner.yaml) 的 TCP、T0/P1/P2/P3 仍是 `PLACEHOLDER`，并强制仿真。真实清单 [site_inventory.json](../../config/calibration/site_inventory.json) 是待测记录，不是可以直接加载的 ROS 参数文件；两者不会自动同步。

| 标定交付 | 当前规划层接收方式 | 待确认 |
| --- | --- | --- |
| V1 TCP | `tcp.xyz` / `tcp.quat_xyzw` = **`T_ee_link_TCP`**，单位 m、xyzw | 记录工具工作点和三轴；若实测以 wrist3 为父系，先乘 `inverse(T_wrist3_ee)` |
| T0/P1/P2/P3 | `slots.<name>.frame_id` / `xyz` / `status` | 规划目前把它当作**落座 TCP 位置**，不是电池中心或桌面中心 |
| 相机安装外参 | 相机根 frame 的安装 TF，由集成层统一发布 | 保存父系、实际 optical frame、标定版本；相机内部 TF 由驱动发布 |
| 相机提环观测 | 完整位姿、来源 frame 和 ROS 采集时间 | 统一 ring / V1 插入轴；不把未知朝向设为单位四元数 |

`MoveItPlannerPort` 已做 `T_planning_EE_goal = T_planning_TCP_goal × inverse(T_EE_TCP)`。离线工具的 EE goal 仅供复核，**不要再把补偿后的 EE goal 当作 ring/TCP 目标传给它**，否则会重复补偿。当前 RING_STANDOFF 规则沿 ring 的局部 +Z 退让，并用其朝向作为目标 TCP 朝向；若 ring 与 V1 工作轴不一致，应由视觉/规划明确目标映射后接入，不能直接改变标定记录来凑数。

当前工位规划保持当前抓取姿态，使用工位位置，**不使用独立的工位目标朝向**。需要固定落座朝向时交常建烁接入；记录表仍应保存完整工位位姿及其对象语义，不能声称现有规划器已消费这些朝向。

若标定在物理设计 F 中：先核验 `T_wrist3_F`，再算 `T_ee_camera = inverse(T_wrist3_ee) × T_wrist3_F × T_F_camera`，TCP 同理。固定旋转可核验，`T_wrist3_F` 仍 UNKNOWN；共享 URDF 和 TF 图没有提供法兰孔位 / 标记 +X 的实物证据。

## 4. 动态观测时间的集成阻断

main 的 [move_group_backend.cpp](../../src/mtc_moveit_planner/src/move_group_backend.cpp) 在 `transform_to_planning_frame()` 中以 ROS 时间 0 查询 latest，因为 `Pose6D.stamp_s` 属于 FSM 时基。这对当前静态 fake 例子可以成立，**对随腕运动的 D435i 采集帧不能宣称已解决同步**。

真实接入前，由视觉 / 孙嘉明 / 常建烁确认：保留 ROS 采集 stamp；按这个时刻查完整动态链；FSM 单调 Clock 只用于看门狗。如果适配器无法带 ROS stamp 穿过 Core，可在进入 Core 前用 TF 按观测时间完成光学系→规划系变换，同时保存原始 stamp / frame / 标定版本，并单独把年龄检查交给一致时基的适配器。不得拿 FSM 秒值直接填 ROS stamp，也不得拿 latest 掩盖缺失历史 TF。

本分支不修改其他人的规划 / FSM / IDL。诊断拒绝规则也尚未自动接入它们。负责人合并软件与文档后，真实执行仍需相机外参、V1 TCP、台面/T0、ring 轴向及误差 / 年龄阈值，以及历史 TF 与失败传播的现场验收。
