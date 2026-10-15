# V1 被动舌规仿真

ROS 2 Jazzy / Gazebo Harmonic，使用原始电池 STEP、基础/顺序任务底图，以及现有 C++ `PickPlaceFsm` 与 MoveIt 后端。V1 靠舌片、倒钩和电池把手之间的接触承托；仿真观察插件不写物体位姿、不创建挂载约束，也不施加抓取力。

## 构建与运行

在 Ubuntu 24.04 / WSL 中运行。Ubuntu 22.04 旧工程作为资产来源，不修改旧工程或实机参考工作区。

```bash
sudo apt-get update
sudo apt-get install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control nlohmann-json3-dev
# 已有 AUBO S3 description / MoveIt 配置 overlay 时：
export MTC_DRIVER_SETUP=/home/operator/meituan/aubo_s3_mock_ws/install_no_symlink/setup.bash
bash src/mtc_simulation/scripts/build_v1.sh
source install/v1_sim/setup.bash

# 基础搬运：A -> T0
ros2 run mtc_simulation run_v1.py --task basic --order A --pin-moveit-plugins --output outputs/v1_basic
# 顺序抓放：C -> P1、A -> P2、B -> P3（CAB 是演示输入）
ros2 run mtc_simulation run_v1.py --task sequence --order CAB --timeout 2400 --pin-moveit-plugins --output outputs/v1_sequence
# 加 --gui 显示 Gazebo；只看场景，不发任务：
ros2 run mtc_simulation run_v1.py --task basic --gui --observe-only --timeout 3600 --output outputs/v1_scene
```

每次使用新的输出目录。默认独立 ROS 域 86，启动前拒绝已有节点的域；可用 `--domain` 更换。Gazebo transport 使用每次独立的 partition。脚本只清理自己创建的进程组。默认 Fast DDS / UDPv4（可通过已有环境变量覆盖）；运行时打印 FSM 阶段。`--timeout` 为墙钟上限，不是仿真时间。

`--pin-moveit-plugins` 显式启用当前 MoveIt 2.12.4 的进程内插件保留退出修复，原始系统库不修改；原因见 `docs/planning/MOVE_GROUP_SHUTDOWN_AUDIT.md`。子进程异常退出也会使总验收失败。

运行产生 `commands.jsonl`（实际发送的完整轨迹，标记仅仿真）、`events.jsonl`（FSM、规划和执行）、`physics.jsonl`（物理位姿）、`result.json`、独立计算的 `acceptance.json`、`trajectories.png`（安装 matplotlib 时）、`overview.mp4` 和相机快照。失败返回非零；ROS launch 进程返回 0 不会被当作任务成功。

## 输入与边界

- 原始资产及 SHA-256 见 `assets/provenance.json`。电池尺寸 70 × 80 × 80 mm；把手开口约 50 × 18 mm。原始 STEP 和碰撞分解保留。
- AUBO S3 来自官方模型提交 `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`，原始清单见 `assets/aubo_source_manifest.json`；上游 package.xml 声明 BSD。
- V1 几何与 `mtc_moveit_planner/config/planner.yaml` 的 CAD 名义尺寸与 TCP 计算一致；本场景相对 `ee_link` 选用 270°（-90°）名义安装角（`wrist3_Link → ee_link` 另有 +90° 固定变换），由 `config/*.json` 明确记录，区别于早期规划样例的 0°；工具质量暂定 0.25 kg，电池质量 0.4 kg、摩擦系数 0.8 都是仿真假设。
- 顺序场景从已通过碰撞检查的上方待机位启动（TCP 在 C 左侧预对准点上方，z=1.00 m）；基础场景保留原始初始关节值。完整初始关节值写入每次运行的 `scenario.json`，这不是实机回零动作。
- 物理场景统一用 `world`；底座安装高度 0.651 m，桌面 0.650 m。基础/顺序任务的搬运 TCP 高度分别为 1.00/0.95 m。抓取 TCP 在电池底面上方 60 mm 的开口内；不沿用旧 V2 位于把手横梁实体内部的 74 mm 抓取点。
- 右列从 +X 侧预对准（60 mm），左列从 -X 侧预对准（45 mm），避免横穿另一列电池。左侧舌尖到把手边缘的名义间隙为 2 mm，实机需复测安装误差。左列电池在空中转向 90°，放置后沿 -Y 退出 60 mm；右列放置后沿 +X 退出 60 mm，避免 P1 左侧退出的自碰撞。试提后检查电池实际上升、工具接触和桌面脱离；搬运中监测相对位姿。落座、卸载和退出均检查物理反馈。规划场景保留目标把手横梁，仅在试提规划时放行 `v1_hook` 与该横梁的接触，其他碰撞关系保留。
- 位姿输入明确是 Gazebo 真值，固定全景相机只用于观察，尚未建模腕部相机。未验证真实视觉识别、手眼标定、TCP 实测、真实力控或电机力矩能力。
- 执行是 `gz_ros2_control` 位置伺服。现有 GenericSystem / 实机适配器不放宽；MoveIt 使用无控制器插件，只提供规划。唯一 FJT 执行者属于本仿真包。
- `gz_ros2_control` 1.2.20 的硬件查询可能不填写 `plugin_name`。同时校验唯一的活动 `V1GazeboSystem`、Gazebo 实体中实际实例化的控制插件、每次运行的 token，以及新鲜物理反馈。WSL 仿真采用 2 s 墙钟反馈超时，同时保留 0.3 s 仿真位姿时效检查；超时取消轨迹并失败，不宣称已停稳。
- 现有 FSM 的 `verify_test_lift=false` 在这里使挂载证据在 `VERIFY_ATTACHED`、即实际试提运动后读取；挂载验证仍为必需。仿真完整场景预规划的 `PLAN_ACQUIRE` 上限单独设为 30 s，`MOVE_PRE_ALIGN` 设为 180 s 以容纳原限速下的舌规转向；运动速度上限和公共契约不变。
- 验收目标为中心误差 <15 mm、底面高度误差 <2 mm、竖直、连续稳定至少 3 s。电池外接圆直径约 106.3 mm，不能完整进入直径 80 mm 内圈；本验收不声称比赛内圈得分。

实机前仍需实测并替换工具安装/TCP、相机与工作台坐标、负载质量及摩擦参数，完成低速空载和试提验证。仿真入口不加载真实 AUBO 驱动。

模型依据、验收证据与实机交接项见 [V1 仿真报告](../../docs/simulation/V1_SIMULATION_REPORT.md)。
