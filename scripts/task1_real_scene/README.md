# 当前现场的任务一

本目录仅连接已有模块：控制柜只读状态、原 V1 安装方向识别、与控制柜一致的机器人模型、
现场场景生成，以及成功轨迹导出。任务执行仍使用 `mtc_simulation` 的接触物理与 C++ FSM，
规划仍使用 `mtc_moveit_planner`。没有向真实机械臂发送运动命令。

2026-10-14 的输入与运行证据位于 `outputs/task1_real_scene_20261014/`。

后续几何复核的新场景位于 `outputs/task1_real_scene_20261014/revision_01/`，仅保留红电池。
`revise_scene.py` 根据当前把手、T0 轮廓及用户确认的同高支撑面建立独立版本；
当前坐标仍为候选，不能把原点测量或仿真验收当成实机验证。
`insertion_tcp_height_above_support_m` 在新场景取 0.0565，使原 V1 的名义上下间隙各约 4 mm；
旧场景默认 0.060 保持历史行为。`save_simulation_trajectory.py` 可保存通过验收的完整轨迹与采样，
明确标记实机坐标验证待完成，不附带硬件启动器。下面输入约定与原导出命令描述的是历史场景。

新版本规划 `revision_01/plan_red_01` 与原样回放 `revision_01/replay_red_01` 均通过；
保存文件在 `revision_01/saved_trajectory/`，结果见
[当前结果](../../outputs/task1_real_scene_20261014/revision_01/RESULT.md)。
复现新版本时使用 `revision_01/inputs_red_only/scenario.json`；回放添加
`--replay-commands outputs/task1_real_scene_20261014/revision_01/plan_red_01/commands.jsonl`，
并选用空输出目录。原始相机视频按收到的帧数编码，可能快于仿真时间；
`retime_video.py --run <成功运行目录>` 可按画面中的时钟精确匹配、补帧，另存正常速度视频。

## 输入约定

- `world` 与真实 `base_link` 完全重合，所有长度以米表示。
- 红色把手点是横梁**顶面**，按用户指定固定；电池底面为该点减去 80 mm，
  插入工作点为该点减去 20 mm。保留原始点数据和它的手眼标定引用。
- T0 点沿 `base_link` 的 Z 轴减去 20 mm，X/Y 不变。
  T0 与电池支撑面的 2.155 mm 高差用局部支撑面表示，不整体平移旧场景。
- 当前实机关节角是仿真初值。初值来自时间重叠的控制柜与 RGB-D 采集；没有复用历史过渡段。
- 原 V1、44 mm 舌片由用户确认。比较四种离散安装角的 CAD 投影，识别为
  `wrist3_Link → tool = Rz(+90°)`，即 `ee_link → tool = Rz(0°)`。
  舌片伸出方向是法兰 +Y，连接杆沿法兰 +Z，倒钩朝法兰 −Z。
  CAD 工作点相对法兰为 `[0, 0.045, 0.1105]` m，工作点姿态为 `Rz(90°) Ry(90°)`。
  这是安装方向识别与名义 CAD TCP，不冒充独立的 TCP 实测标定。
- 机器人名义模型与控制柜 FK 存在毫米级差异。`fit_kinematics.py` 使用只读的虚拟 FK 查询
  匹配关节原点，并用保留配置验证。它验证数学模型一致性，不代表机器人绝对物理精度。
- 工具、把手的接触碰撞形状保留；电池下半部以包围原网格的盒体做碰撞计算，
  可避免每块 7636 个三角面的平面接触开销。原 CAD 视觉、质量、惯量与把手碰撞体保留。
- 其他电池的近似位置和铝型材包围盒来自图像。模型假设与来源记录在 `scenario.json`。

## 可复现流程

`capture_robot_state.py` 在 NUC 的现有 SDK 环境运行，只调用状态和配置读取、虚拟 FK 计算。
凭据通过 stdin 输入，不写入输出；不要把凭据放进命令行或版本库。
相机使用已有 `capture_rgbd.py` 的 `request_capture.json` 接口保存 RGB/原始深度/对齐深度。
原始采集位置由 `inputs/captured_state.json` 的 `camera_capture_dir` 指向。

已有输入后，在工作区根目录执行：

```bash
python3 scripts/task1_real_scene/fit_kinematics.py \
  --urdf src/mtc_simulation/assets/arm.urdf \
  --reference outputs/task1_real_scene_20261014/inputs/kinematics_reference.json \
  --output outputs/task1_real_scene_20261014/inputs

python3 scripts/task1_real_scene/inspect_mount.py \
  --inputs outputs/task1_real_scene_20261014/inputs \
  --point /home/operator/meituan/eye_to_hand_calib/data/measurements/red_handle_20261014_104532/point_result.json

python3 scripts/task1_real_scene/build_scene.py \
  --inputs outputs/task1_real_scene_20261014/inputs \
  --measurements /home/operator/meituan/eye_to_hand_calib/data/measurements/red_handle_20261014_104532

source /opt/ros/jazzy/setup.bash
source /home/operator/meituan/aubo_s3_mock_ws/install_no_symlink/setup.bash
source install/v1_sim/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 LP_NUM_THREADS=4

ros2 run mtc_simulation run_v1.py --task basic --order A \
  --scenario outputs/task1_real_scene_20261014/inputs/scenario.json \
  --domain 91 --pin-moveit-plugins --timeout 1200 --output outputs/new_task1_plan

# 用原始关节轨迹复现；不会重新规划或静默改轨迹。
ros2 run mtc_simulation run_v1.py --task basic --order A \
  --scenario outputs/task1_real_scene_20261014/inputs/scenario.json \
  --replay-commands outputs/new_task1_plan/commands.jsonl \
  --domain 91 --pin-moveit-plugins --timeout 1200 --output outputs/new_task1_replay

python3 scripts/task1_real_scene/export_package.py \
  --run outputs/new_task1_replay \
  --inputs outputs/task1_real_scene_20261014/inputs \
  --output outputs/new_task1_hardware_package
```

仿真输出目录必须为空。保留失败运行的记录，不能将旧成功标记复制到新轨迹。
新构建需要按本工作区的 colcon 配置重新安装 `mtc_simulation`。

## 验证与实机接口

验收读取独立的物理轨迹：确认试提稳定至少 2 秒，电池主体在 T0 矩形内，
底面高度、竖直程度与静止速度符合要求，释放后稳定至少 3 秒；同时检查非预期碰撞。
保存的轨迹包含所有关节路点、速度、加速度、时间和阶段名称，默认时间倍率为 1。

导出器选择复用兄弟工作区 `scripts/task_one_hardware/` 的轨迹数学、播放器与异常测试，
按内容哈希记录来源，并复制到独立的交付目录。它不会复用历史任务路径或增加未验证的起点过渡。
控制柜均匀位置采样与仿真五次插值的区别保存在插值检查中，不宣称原生控制器已实测一致。

采集时控制柜为断电/急停，关节位置不变且 `isSteady=true`，但 J2 速度值非零。
这不妨碍用读取到的位置建立仿真；真正播放时，原播放器仍要求正常使能与有效静止反馈，
并检查身份、限位、起点、TCP、负载和轨迹哈希。实际工具负载与名义 TCP 仍须对应现场。

## 实机起点变化时的衔接验证

`verify_start_alignment.py` 只在显式隔离的 ROS domain 中工作，核对 Gazebo 硬件组件与
每次运行的观察器 token 后，检查不超过 1° 的独立衔接路径，并发送给仿真控制器。
它逐 10 ms 检查碰撞，验证接触记录和终点，不连接实机。
本次 4 秒衔接的输入、仿真验证及实机记录保存在
`outputs/task1_real_scene_20261014/physical_run_20261014T171937Z/`。
原任务文件保持不变；其首次实机执行在第 1 段约 62 秒遇到控制柜急停，未完成任务。
