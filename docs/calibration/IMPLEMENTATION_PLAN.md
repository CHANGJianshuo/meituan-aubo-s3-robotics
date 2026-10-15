# D435i / V1 Calibration P0 Implementation Plan

用户已确认：D435i 固定在末端法兰附近，10/12 使用 V1 舌规；无真实采集数据，先完成任务书第 05 节今晚可完成项。沿用此前交付设计，直接执行，不扩大到 FSM、视觉算法或规划。

**Goal:** 完成可复现的离线坐标转换校验、TF/标定交接、已测/待测表和现场 SOP。
**Architecture:** Python 标准库实现 SE(3) 与明确的变换方向、测量状态和时间校验；JSON 合成夹具只作离线证据。现场检查提供只读 ROS 命令，不发布伪造 TF 或运动目标。
**Tech Stack:** Python >=3.10 标准库；团队集成环境 Ubuntu 24.04 / ROS 2 Jazzy。

## Constraints

- 数据状态为 MEASURED / ESTIMATED / UNKNOWN；UNKNOWN 的数值为 null。
- 位移 m，四元数 xyzw；RPY/关节角 rad；时间戳保留采集时刻且时基一致。
- 保留完整 6D Pose；TF 缺失/未知、非单位四元数、过期/未来观测、时间不匹配拒绝。
- 腕部相机基座外参随关节运动变化；末端至相机的安装外参固定。
- 不修改 mtc_interfaces、C++ Core 或驱动工作空间；不连接实机。
- 本地独立分支 feat/calibration-tf-v1；不推送、不合并。

## Tasks

- [x] 写行为测试并运行，记录未实现时的失败。文件：tests/calibration/test_calibration.py；入口 python -m unittest discover -s tests/calibration -v。
- [x] 实现 scripts/calibration/geometry.py：Transform.compose/apply/inverse；必须通过非零旋转+平移、姿态与逆变换测试。
- [x] 实现 scripts/calibration/contract.py：Pose、FrameGraph、transform_pose；验证 UNKNOWN、缺路径、多父、循环、时间/单位/有效标志的拒绝。
- [x] 实现 scripts/calibration/check_calibration.py：JSON 输入与可读/JSON 输出；接受输出只作诊断，非运动 Goal；拒绝返回 exit 2，解析错误返回 exit 2。
- [x] 保存 config/calibration/site_inventory.json 与 examples/calibration/synthetic_case.json。真实清单保留 null；合成数据输出 simulation_only=true。
- [x] 编写根目录 TF_AND_CALIBRATION_HANDOFF.md、docs/calibration/MODEL_FRAME_AUDIT.md、FIELD_SOP.md、measurement_inventory.csv、handeye_samples.csv、tcp_samples.csv、validation_points.csv。证据包括官方固定提交及源码哈希。
- [x] 验证所有测试、CLI 正常/拒绝样例、文档链接、CSV/JSON、Python 编译和变更边界；保存 VALIDATION_REPORT.md 和 results.json。
- [x] 完成自审后提交本地分支，生成便于转交的本地压缩包与今晚汇报模板。ROS/实物部分标 NOT RUN。
- [x] 独立复核（Claude，2026-10-11）后修正：17.5 mm 基准改为 1/4-20 中心线并在推导输出中列出假设；设计输入要求 camera_model（D435f/D435if 为 4.7 mm）；两个 CLI 对深嵌套 JSON/意外异常返回 exit 2；OUTPUT_ERROR 带 motion_authorized=false；schema_version 严格为整数 1；时间戳上限 int64 ns；补外参方向/列主序、四元数顺序、tf2_echo 方向说明；新增 run_offline_validation.py 重建证据。先写测试观察失败，再实现。

不把缺少实物数据当成实现失败；实物标定仍需现场数据。离线工具不实现动态 TF 插值，动态夹具要求与观测时间精确匹配；运行环境使用 tf2 按观测时刻求值。
