# 现场任务二：红 → 黄 → 蓝，舌规朝向保持不变

当前口令为 **A → P1、B → P2、C → P3**。A/B/C/D 对应红/黄/蓝/绿。
绿色电池留在场景中；已放置电池也继续参与后续避障。

复用任务一的控制柜匹配模型、原 V1 工具和安装变换、56.5 mm 插入高度、
接触承托 FSM、只读采集接口、原样回放和轨迹导出数学。
任务二重新测量物体与目标位置，并重新规划各段；不搬用任务一的关节路径、
J6 微转或 −Y 3 mm 修正量。图像坐标使用任务一已成功对中的参考关系作候选估计，
不构成新的手眼或绝对位置标定。

## 数据与入口

最新的全流程包支持显式 `--all-tracking-guard-deg 0.6`，将全部 30 段六轴应用层跟踪阈值统一为 0.6°。
必须配套 `task2_all_0p6_v1` 播放器、完全一致的六轴阈值及 `--tracking-request` 固定哈希记录；
不能与 J4 单轴选项同时使用。默认仍为统一 0.5°，已生成的历史包和单次台账不变。
Web UI 显示实际六轴阈值；原轨迹、速度、起点与停稳检查、控制柜保护配置保持原值。

用户随后要求“取消阈值”。对应入口为 `--tracking-monitor-only`，只取消六轴实际/目标位置差的
软件停机阈值，RTDE 实际值及目标值继续完整记录。它要求 `task2_tracking_monitor_v1` 播放器和
明确固定哈希请求，不能与数字阈值选项同时使用。Web UI 显示“全程跟踪误差仅记录，软件阈值已取消”。
控制柜保护、轨迹包络、起点/终点、停稳、身份/TCP/负载、独占锁及单次台账检查仍保留。

本次工作目录为 `outputs/task2_real_scene_20261014/`。原任务一数据和实机网页保持原状。

- `capture_scene.py`：向现有相机采集进程请求 RGB-D，读取本机操作台的停稳反馈，
  保存图像、时序和文件哈希。只执行 HTTP GET 和相机快照，不提交运动。
- `register_view.py`：复用任务一的 RGB-D 视角配准实现，另存结果与实现文件哈希。
- `estimate_scene.py`：在支撑平面上拟合 P1/P2/P3 外圆、识别四个把手顶面；
  当前 ROI 对应已核对画面。底图、相机或物块移动后必须重新采集和核对 ROI/标签。
- `build_scene.py`：创建独立场景，保留完整工具/把手碰撞体；从采集关节角和匹配模型
  计算当前舌规姿态，写入 `fixed_tcp_orientation_xyzw`。

`mtc_simulation` 在此配置下采用垂直抬高、水平移动、垂直下降的笛卡尔接近路径，
整段均保持姿态，不允许回退到自由空间转腕。后续抓取、抬升、搬运和退出使用同一姿态。
原任务一和历史场景未设置该参数时保留原规划行为。

## 复现当前候选场景

从工作区根目录执行，输出目录必须是新的。源数据保留在本机历史任务一目录中。

```bash
python3 scripts/task2_real_scene/capture_scene.py \
  --preview /home/operator/meituan/eye_to_hand_calib/data/measurements/red_handle_20261014_104532 \
  --output outputs/task2_new/capture --frames 30

python3 scripts/task2_real_scene/register_view.py \
  --implementation outputs/webui_laptop_20261014/hardware_integration/register_current_view.py \
  --capture-record outputs/task2_new/capture/capture_record.json \
  --output outputs/task2_new/registration

python3 scripts/task2_real_scene/estimate_scene.py \
  --capture-record outputs/task2_new/capture/capture_record.json \
  --registration outputs/task2_new/registration/camera_registration_candidate.json \
  --task1-scenario outputs/task1_real_scene_20261014/revision_01/physical_attempt_20261014T184851Z/half_time_01/scenario.json \
  --anchor-report outputs/webui_laptop_20261014/hardware_integration/obstacle_estimate.json \
  --anchor-view outputs/webui_laptop_20261014/hardware_integration/camera_registration_candidate.json \
  --output outputs/task2_new/points --order ABC

python3 scripts/task2_real_scene/build_scene.py \
  --reference outputs/task1_real_scene_20261014/revision_01/physical_attempt_20261014T184851Z/half_time_01/scenario.json \
  --measured outputs/task2_new/points/measured_scene.json \
  --output outputs/task2_new/inputs
```

先检查 `points/annotated_scene.png` 中颜色、槽位和轮廓；候选坐标仍需实机对中验证。
如果识别结果出现物体重叠或尺寸异常，应复核图像和模型，不缩小碰撞体、平移电池
或放宽碰撞阈值来获得通过结果。

```bash
source /opt/ros/jazzy/setup.bash
source /home/operator/meituan/aubo_s3_mock_ws/install_no_symlink/setup.bash
source install/v1_sim/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 LP_NUM_THREADS=4

ros2 run mtc_simulation run_v1.py --task sequence --order ABC \
  --scenario outputs/task2_new/inputs/scenario.json --domain 92 \
  --pin-moveit-plugins --timeout 2400 --output outputs/task2_new/plan

ros2 run mtc_simulation run_v1.py --task sequence --order ABC \
  --scenario outputs/task2_new/inputs/scenario.json --domain 92 \
  --replay-commands outputs/task2_new/plan/commands.jsonl \
  --pin-moveit-plugins --timeout 2400 --output outputs/task2_new/replay

python3 scripts/task1_real_scene/save_simulation_trajectory.py \
  --run outputs/task2_new/replay --output outputs/task2_new/saved_trajectory
```

导出器复用任务一实现，支持按物体区分的 30 段，输出 `task_two_trajectory.json`、
CSV、原始命令、均匀位置采样和验证记录。原样回放必须使用全部相同命令，不能重规划。

## 验收与现场衔接

验收检查每块电池真实试提、落位后至少 3 秒稳定、30 段及三块完成次序、绿色电池和
尚未抓取电池没有被移动、已放置电池在后续动作中保持落位，以及非预期接触。
固定姿态另由 Gazebo 独立 TCP 四元数轨迹检查，全程偏差不得超过 0.1°。

模型与相机估计通过后，仍需要以当前现场姿态核对新抓取点。此目录的仿真与导出命令
不连接机器人。当前任务一网页仅支持红 → T0，不可把任务二数据替换进去冒充可执行任务。
实机准备需使用单独的任务二执行包并保留原播放器的身份、起点、限位、停稳、
独占锁及单次执行检查；每块的实际试提和放置观察也必须单独确认。

`prepare_commissioning.py` 可从已通过原样回放的保存目录生成九个单次分段包，
每块三个包，起点逐段衔接。它复用任务一 `player_settle_v2` 的统一 0.5° 跟踪检查，
不移植任务一某一条路径特有的 J3 滞后放宽策略。构建过程只写文件：

```bash
python3 scripts/task2_real_scene/prepare_commissioning.py \
  --saved outputs/task2_new/saved_trajectory \
  --manifest-reference outputs/webui_laptop_20261014/hardware_integration/execution_package/manifest.json \
  --adapter outputs/webui_laptop_20261014/hardware_integration/player_settle_v2 \
  --output outputs/task2_new/commissioning
```

每包在 NUC 的 `./run.sh offline` 不连接硬件，`inspect` 只读核对，`execute` 执行
该包内阶段并停止。现场尚未验证前，不连续启动各包；没有自动串联九包的脚本。

本次现场入口为 `field_session.py --root outputs/task2_real_scene_20261014`。
它在用户终端读取 SDK 凭据，仅在本进程内存保留，逐包复用原任务一
`remote_bridge.py` 的心跳、断连停止、独占执行及单次台账。登录后默认等待，
每次只受理一个与下一包一致、包含已复核证据哈希的新请求；不会自动串联或重试。
状态和各段实机回执保存在任务目录的 `field_session/`。
在会话终端按 Ctrl+C，或创建 `field_session/stop.request`，会停止该会话拥有的动作。
使用 `--session-name` 创建新的会话目录时，停止文件也必须放在该目录。

`verify_placement.py` 检查已落位颜色、候选目标中心和至少 3 秒的连续图像稳定性。
例如 `--placed AB` 检查红、黄两块；最终使用 `--placed ABC`。输入为
`--capture-record`、`--points`，结果写入新的 `--output` 文件。检查会验证采集文件哈希；
直立、把手承托和退刀间隙仍需要查看相机图像，图像中心估计不代表绝对标定精度。

SDK 收尾异常不能作为重新执行同一包的理由。若已取得所有成功段回执、包完成记录、
独立停稳反馈以及现场图像，可将这些证据和哈希组成恢复检查点，通过
`--resume-checkpoint <检查点.json> --session-name <新目录>` 接续尚未执行的下一包。
恢复检查要求已完成包构成连续前缀，最后反馈与保存终点一致，当前停稳姿态与下一包起点一致。
原失败会话和执行台账保留，不重建计划 ID。

仅当已收到包完成记录、无停止/失败事件，且进程以 255 退出并明确报告
`RTDE send TIMEOUT` 时，会话可进入 `awaiting_completed_reconciliation`，保持登录并等待
独立核对。将新检查点的路径、哈希和新鲜时间戳作为 `reconcile_completed` 请求写入
该会话的 `reconcile.json`，只能确认刚完成的一个包。确认后仍需另一个带现场证据的
下一包请求才会移动。其他异常及最后一包的收尾异常均终止会话、清除凭据，保留记录供核验。

`verify_scene_unchanged.py` 用于运动前核对底图、四块电池和起点是否仍与规划一致。
进入物体附近后工具可能遮挡色块，此时不能把遮挡当作物体位移，也不能据此宣告抓取；
应保留停稳 RGB-D、执行回执和实际试提/落位观察，再决定下一包。

本次实际进度、验收与已知限制见 [现场结果记录](../../outputs/task2_real_scene_20261014/RESULT.md)。

## 完整流程与平滑二倍速

在已完成分段实机核对后，完整流程使用一个包含 30 段的包，由原播放器自动推进。
人工将三块放回起始区后重新采集点位，并从当前真实关节姿态生成完整基线仿真；
基线完成后，复用任务一 `retime_successful_trajectory.py --motion-ratio 0.5` 平滑重定时。

2026-10-15 的 Web UI 提速运行在蓝块接近段触发 J4 > 0.5° 保护。用户随后明确要求提高阈值并复位重跑，新增限定版本 `player_j4_06`：只将 J4 应用层跟踪阈值设为 0.6°，其他五轴仍为 0.5°，控制柜配置不变。`prepare_fullflow.py --j4-tracking-guard-deg 0.6 --tracking-request <不可变请求记录>` 必须同时使用匹配的播放器；默认仍生成原 0.5° 版本。新版本保留触发超差的完整 RTDE 帧及关节、实测误差和阈值。每次新包使用新 plan_id，旧失败记录和单次台账保留，仍由用户在 Web UI 提交。
该模式支持完整 ABC 序列，保持各段端点和内部停点，起停速度、加速度为零；
以五次样条和渐变时钟限制速度、加速度与加加速度，必要时细化样条采样，
使工具参考点偏离原路径不超过 0.1 mm。若指定时长无法满足约束则拒绝生成。
合并相距不足 1 ms 的导出时刻，避免过短插值区间放大数值误差；
使用 Bernstein 系数包络检查每个完整区间的导数上界，执行包构建时独立复算。

二倍速指该路径累计运动时间减半，自动停稳和接触核验等待另计。速度写入新的轨迹时间，
播放器仍用时间倍率 1；不会提高控制柜速度滑块或放宽跟踪保护。
新轨迹必须完成全流程原样仿真回放、固定朝向与三块落位验收，再导出保存目录。
`prepare_fullflow.py` 校验平滑输入、回放原始命令和导出轨迹完全相符后，生成单个执行包：

```bash
python3 scripts/task2_real_scene/prepare_fullflow.py \
  --saved <平滑回放保存目录> --retiming <平滑验证目录> \
  --manifest-reference outputs/task2_real_scene_20261014/commissioning/01_A_P1_approach/manifest.json \
  --adapter outputs/webui_laptop_20261014/hardware_integration/player_settle_v2 \
  --output <新工作目录>/commissioning
```

部署和离线检查通过后，可继续使用 `field_session.py --root <新工作目录>`。
此次唯一的执行请求包含全部 30 段，仍保留断连停止、独占锁、单次台账和异常不重试。
实机运行前重新核对源物块、底图、关节起点；运行中记录相机画面和运动反馈，结束后独立验收。

用户明确要求保留原始起点和原轨迹时，复用原完整回放的 30 段及时间参数，仅生成新的单次执行标识与已请求的跟踪策略。`prepare_return_to_start.py` 为独立空载复位组合已经验收的接近路径和原接近路径的逆向回放，统一放慢到一半速度，逐段核对时间反演、起止停稳、朝向和导数界限；此输出还必须独立完成场景碰撞与 Gazebo 回放验收，不能仅凭数学检查上实机。复位授权与完整任务提交分开：本次用户授权助手直接复位，原 30 段任务仍由用户在 Web UI 启动。
