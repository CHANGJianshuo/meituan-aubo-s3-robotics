# Phase 2B：完整 FollowJointTrajectory 的 Fake Hardware 执行端

本包提供 `mtc::trajectory::FjtMotionPort`，继承既有
`mtc::manipulation::MotionPort`，以一条ROS Action Goal提交完整时间参数化轨迹。
默认执行禁用；只支持明确核实的 `mock_components/GenericSystem`。不包含真实
AUBO SDK、RPC/RTDE、工具IO或MoveIt Planner，不改变Phase 2A的三个Mock Server。

2026-10-12 Action Mutex Backport：锁外出站队列与迟到响应生命周期修复已完成。
原478项身份保留，新增8项，486项GTest与486项UBSan通过；9项并发/身份测试重复40轮通过。
TSan启动因unexpected memory mapping受阻（完整回归NOT RUN）；原生GenericSystem仍因
严格Status缺失保持BLOCKED、Goal0。独立模块验收不代表完整联合执行完成。详见
[Action mutex回移验收报告](../../docs/migration/PHASE2B_ACTION_MUTEX_BACKPORT_GATE.md)。

以下为此前Ownership阶段的历史结果：

2026-10-12 Ownership Final Review：本轮新增15项真实ROS异步测试，原463项身份
全部保留；478项GTest及478项UBSan通过，TSan NOT RUN。FJT包为30项校验 +
61项ROS Action测试。当前原生GenericSystem控制器未提供初始实时Status证据，
执行预检 **BLOCKED、零Goal**；不能沿用上一轮的四项执行PASS作为本轮结果。
MoveIt共存仍因第二个FJT客户端及Status证据不足保持BLOCKED。详见
[所有权最终复核报告](../../docs/migration/PHASE2B_OWNERSHIP_FINAL_REVIEW.md)。
[上一轮集成门禁报告](../../docs/migration/PHASE2B_FINAL_INTEGRATION_GATE.md)和
[首次验收报告](../../docs/migration/PHASE2B_TRAJECTORY_EXECUTION_REPORT.md)仅为历史证据。

## API 与执行所有权

| API | 行为 |
|---|---|
| `FjtMotionPort(node, config)` | 外部负责spin该Node；Config必须给齐六关节限位。执行默认禁用，显式开启仍需运行时Fake证据 |
| `execute_trajectory(command)` | 首次校验并异步提交；在途返回`std::nullopt`，后续tick轮询同一command_id，不重发；有明确结论才返回MotionResult |
| `execute_joint_move(command)` | 明确拒绝单点入口，不将完整轨迹拆分 |
| `cancel_command(command_id, reason)` | 仅操作本实例持有的对应Goal handle/UUID；错ID或旧ID不发送任何取消 |
| `request_stop(reason)` | 固定本实例当前ID，再调用关联取消；没有关联身份便拒绝，不调用全局Stop |
| `snapshot()` | 加锁副本：ID/UUID、Goal和Cancel计数、Feedback、终态、观察窗口、外部在途Goal及FAULT |

异步回调属于同一MutuallyExclusive CallbackGroup；Port内部mutex保护状态及命令
所有权。回调不访问FSM，不等待完整轨迹，不使用detached thread。调用者继续保持
FSM单线程事件所有权。Goal Response、Feedback、Result和Cancel响应均检查原Session
及UUID；超时后仍保留原所有权，迟到受理也只取消原Goal。

不同ID并发请求拒绝；同ID内容改变拒绝；已完成ID重读历史结果，不再派发。
历史MotionResult是执行记录；新的Stop查询会重新检查当前反馈，不能永久复用历史
停稳。超时、通信/反馈失效和控制器异常保持FAULT锁定，没有自动复位或人工解除API。
进程内同一domain/Action只允许一个Port。未确认结束或FAULT时销毁，端点保持隔离；
不能通过重建Port绕过锁定。

析构仅对自有Goal做best-effort关联取消，不等待或声称停稳。未回来的Goal Response
回调只会取消它自己的handle。正常宿主退出前应保持Executor运行，取得终态及停止
证据，再移除Node；断联或强制退出可能无法完成取消，本包保留隔离并报告不确定性。
重启进程不构成真实设备恢复证明。软件Stop不是安全急停。

## 执行前门禁与轨迹范围

运行时通过 `controller_manager_msgs` 查询硬件组件和控制器：

- 唯一硬件组件必须匹配配置名称，plugin必须为`mock_components/GenericSystem`且Active，具备六关节position/velocity状态接口。
- 目标JointTrajectoryController和joint_state_broadcaster必须Active；控制器按名称拥有全部六个position接口，无其他控制器占用这些关节。
- FJT Server及控制查询服务必须唯一，目标FJT Client只有本实例，没有直接`joint_trajectory` Topic命令发布者，JointState发布者唯一。
- Action Status未收到、非法或本地接收时间超过proof_max_age时拒绝派发；内部Status订阅使用volatile，不把transient-local缓存当成当前证据。标准Status是事件驱动，无发布时刻字段；本地接收年龄不是绝对新鲜度或跨进程锁。
- Status中的外部在途Goal阻止派发，即使原Client已经退出；不会发送新Goal抢占它。待Goal Response期间的UUID全部视为Unknown，保留Session及观测记录，取得原请求Handle后才比对归属；不按count==1猜测Own。
- Status有效但归属尚未确定时，Session有界等待Response，ready=false，不再发Goal，也不取消未知UUID。证据丢失或超时则FAULT保留；迟到Handle仅取消原Goal，迟到Result不清锁。
- Result终结但Status仍Active时不允许后继Goal。历史终态不当作在途。Snapshot新增ownership/status_received/status_fresh/unknown_active_goals/status_messages；foreign_active_goals兼容字段保守包含尚未关联的Unknown。该结构布局变化，下游C++消费者需要重新编译。
- JointState必须包含完整且唯一的六关节名称、有限位置/速度、有效持续递增的ROS时间戳。位置符合配置限位；按名称重排，拒绝缺字段、旧帧和回退。
- 初次执行前须观察新鲜静止窗口；当前位置与首点按名称比较，默认偏差容差0.01rad，超差拒绝，绝不自动跳到起点。

门禁是受信任、隔离的软件ROS图检查，不是抵抗恶意伪造服务的身份认证或实体安全
系统。需要专用本机domain、已审查的Fake Launch及稳定的控制器配置；禁止在真实
驱动或其他运动客户端所在图中使用。发现不符时拒绝执行或关联取消并锁定。

输入必须有非空command_id、trajectory_id和`simulation_only=true`；后者不能独立
授权发令。只支持完整六关节轨迹，至少两点，首点`t=0`，时间严格递增且在ROS
纳秒转换后仍可区分。原frame_id及header stamp保留；stamp为0表示立即开始，非零
stamp必须在当前ROS时间之后且不超过配置允许的启动延迟。零JointState stamp无效。

每点positions、可选velocities/accelerations按名称映射到配置的控制器顺序。
导数字段需各点一致；加速度不能单独存在。NaN/Inf、重复/缺失关节、维度不匹配、
非法缩放、超限和不足的timeout均拒绝。默认最大轨迹60秒、最大10000点、最大
timeout120000ms；timeout必须覆盖轨迹、启动延迟和停稳窗口。关节限位无默认实机
授权；未配置便拒绝。缩放字段只做输入上限校验，不二次缩放或重定时已规划轨迹。

为避免路点合法但JTC插值超限，转换器以Bernstein控制点凸包保守检查整段linear/
cubic/quintic spline的位置、速度和加速度；可能拒绝实际可行但无法由该保守边界
证明的轨迹。不提供碰撞检查、Cartesian控制或接触控制；`requires_contact`请求
显式拒绝。这里不是完整PickPlace接触工序验收。

Jazzy官方说明见 [JTC轨迹表示](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)
及 [JTC接口和抢占](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)。

## 成功与停止证据

成功必须同时具有匹配UUID的终态SUCCEEDED、控制器SUCCESSFUL、有效关联Feedback、
最终关节到位，以及终态后持续新鲜JointState的静止窗口。检查速度和相邻位置的
变化率；观察窗口须由实际新样本覆盖，ROS stamp跨度与steady接收时间跨度均达到
settle_duration，不能靠持续Timer/tick假装静止。默认窗口0.2秒，必须正值。

Cancel ACK仅记录请求响应，不能确认停止；匹配Goal终态CANCELED加独立静止窗口
才形成正常取消停止结论。ABORTED或其他异常保持锁定；超时会发一次关联取消，
返回未知/超时结果并禁止重发。窗口失效、停止更新或状态不明确均不能生成成功。

这些证据来自理想化GenericSystem和模拟控制器，未经过真实编码器、设备队列、
制动、硬实时或实体安全系统验证，不能推广成实机安全证明。

## 编译与测试

在主工作空间根目录、仅加载Jazzy的新终端执行，不清空历史目录：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/meituan_challenge_ws
colcon build --packages-up-to mtc_trajectory_execution_cpp
source install/setup.bash
colcon test --packages-select mtc_trajectory_execution_cpp
colcon test-result --test-result-base build/mtc_trajectory_execution_cpp --all --verbose

# 当前六个C++包都需有本轮结果，身份核验才完整
colcon build --packages-up-to mtc_ros2_action_cpp mtc_trajectory_execution_cpp
colcon test --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp \
  mtc_motion_execution_cpp mtc_ros2_action_cpp mtc_trajectory_execution_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build
```

核验脚本固定45f497c的原463项身份，包含Planner；核对当前源码声明、
二进制注册清单和实际XML，拒绝未注册、未完成、
遗漏、重复、FAIL和SKIP。历史Phase 2A脚本固定20个可执行文件，不用于当前26个集合。
全目录test-result会包含保留的历史TSan失败，须与本轮各包结果分开报告。

Level 1为纯转换测试；Level 2使用真正的rclcpp_action Client/Server、状态Topic和
硬件/控制器查询服务替身，4线程Executor与真实两线程Goal竞争。另有原FSM首个
运动阶段的注入测试，以及异步取消未确认时保留FAULT的契约测试。测试里的感知/
工具夹具仅供软件测试，未接实体IO；不是安装后的公共API。

## GenericSystem 单独复验 SOP

**本轮执行验收BLOCKED。** 以下启动步骤仍是只读Fake预检，不能将GenericSystem
插件确认当成Status/执行权确认。当前原生JTC启动后不发初始空Status且非周期心跳，
修复后的Port会拒绝执行；不得预热发Goal、注入空Status或放宽门禁。
先以`topology`查看status_received/status_fresh/客户端数量。只有后续经审核的证据
契约具备后才能重新验收下列运动模式，本轮没有实现该Phase 2C环境。

三个终端全部使用以下环境；218是本轮示例domain，先确认空图。同一终端只加载
这一套AUBO安装目录；不构建或修改参考工作空间：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot/aubo_driver_validation_ws/install_no_symlink/setup.bash
source /home/robot/meituan_challenge_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=218
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/mtc_phase2b_generic_logs
```

终端1先检查空图与Fake Xacro。检查失败立即停止，不将参数改为实机模式：

```bash
ros2 node list --no-daemon --spin-time 2
python3 - <<'PY'
from pathlib import Path
import subprocess, xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
share = Path(get_package_share_directory('aubo_description'))
assert str(share).startswith('/home/robot/aubo_driver_validation_ws/install_no_symlink/')
assert (share / 'urdf/aubo_S3.urdf.xacro').is_file(), '禁止在只读目录生成模型'
xml = subprocess.check_output([
    'xacro', str(share / 'urdf/xacro/inc/aubo_ros2.xacro'),
    'aubo_type:=aubo_S3', 'use_fake_hardware:=true'], text=True)
assert [p.text.strip() for p in ET.fromstring(xml).findall('ros2_control/hardware/plugin')] == ['mock_components/GenericSystem']
assert not subprocess.check_output(['ros2', 'node', 'list', '--no-daemon', '--spin-time', '2'], text=True).strip(), 'domain非空'
print('PASS: GenericSystem且空domain')
PY
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_S3 use_fake_hardware:=true \
  initial_joint_controller:=joint_trajectory_controller launch_rviz:=false
```

终端1保持运行。启动日志必须显示GenericSystem加载和两个controller Active。
此Launch的默认Fake参数为false，绝不裸启动。完整驱动背景见
[部署手册](../../docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)。

终端2检查控制器、状态和Action，再运行显式软件测试轨迹：

```bash
ros2 control list_hardware_components -c /controller_manager -v
ros2 control list_controllers -c /controller_manager
ros2 action info /joint_trajectory_controller/follow_joint_trajectory -t
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --timeout 5
ros2 run mtc_trajectory_execution_cpp fake_trajectory_probe --verify-generic-system success
ros2 run mtc_trajectory_execution_cpp fake_trajectory_probe --verify-generic-system mismatch
ros2 run mtc_trajectory_execution_cpp fake_trajectory_probe --verify-generic-system cancel
ros2 run mtc_trajectory_execution_cpp fake_trajectory_probe --verify-generic-system identity
```

probe必须有显式验证参数，否则退出且不执行。它从新鲜JointState读取起点，构造
一条0/1/2秒的三点轨迹、非零终点和不同关节输入顺序；不覆盖控制器关节状态。
使用窄软件测试范围±0.5rad、速度0.2rad/s、加速度0.5rad/s²，均不是实机标定值。
起点不在该范围则拒绝，不自动回零。success预期一个Goal和独立到位/停稳；
mismatch预期明确起点偏差错误、零Goal；cancel预期一个Goal、一次UUID关联Cancel，
初始stop_confirmed=false，最终取得终态及停稳证据后才true。

终端3可只读观察 `ros2 topic hz /joint_states`，确认持续更新，Ctrl+C退出监视。
当前验证环境报告部分URDF限位表达式无法解析、manager未启用命令限位，以及
没有FIFO权限。本包强制自己的配置限位和spline边界，不修改驱动来掩盖问题；
这些环境限制必须记录，不能声称已完成实机限位或硬实时验收。

等待probe终结，再在终端1 Ctrl+C，检查 `ros2 node list --no-daemon --spin-time 2`
中本次节点已退出。不要全局kill其他成员进程；正常关闭不是安全急停。

## 后续 FSM / Planner 集成边界

通过既有PickPlaceFsm构造函数显式注入共享MotionPort即可使用轨迹执行入口。
Phase 2A的AsyncMockMotion和三个默认Mock Server保持原样，本包不提供后端参数
来自动替换它们。本轮测试只证明原FSM首个运动阶段能派发整条轨迹，不宣称完成
完整抓放或Task → PickPlace → MoveIt → FJT闭环。

**取消桥接仍是联合集成阻断项。** 原FSM对同步`request_stop()`只调用并缓存一次，
若返回未确认立即FAULT(520)。直接注入本异步Port将保留这一行为；不能阻塞该FSM
事件线程等待取消。后续宿主应先关联取消、在FSM外非阻塞等待Stop证据，冻结新的
运动派发，确认后才交给FSM处理；逾期强制FAULT。该宿主尚未实现，本轮不改Core。

已通过合入`origin/main`取得PR #3的Planner代码（main基线`f7eaa2c`），本轮未修改
Planner源码或其成员分支。PR #4已关闭但未合并，不能假定具备其场景/起点功能。
Planner仅规划，不调用execute；默认move_group仍加载可执行的控制器客户端。
`exclusive_graph()`保持单客户端要求，不能把多客户端简单当成单一执行权。
需要独立planning-only配置和指定执行管理者后再验收联合路径；图检查不是身份认证。

Planner输出的command_id/trajectory_id由FSM派发时赋值。其stamp_s是生成时间
（可能来自SteadyClock），FJT非零stamp则是ROS时域的计划启动时间。两者尚未统一；
不能静默改为0、取消起点或spline校验。接触轨迹仍显式拒绝。

完成结果仅允许原始命令的完全相同内容重读。适配器为在途及完成命令保存不可变
MotionCommand副本，逐字段比较原始顺序、路点、ID、时戳、scaling、timeout、Pose和
安全字段；相同ID内容变化返回REJECTED，零额外Goal/Cancel，不解除FAULT。
完成账本仍限10000条、不自动淘汰；轮询比较成本随路点数增长。

`fake_trajectory_probe --verify-generic-system topology`只检查图和健康状态，不发Goal。
共存场景会打印clients=2、ready=0，退出码2表示BLOCKED。`identity`提交一次完整
轨迹，再检查缓存重读及三种内容变更拒绝，Goal计数保持1、Cancel计数0。
没有接入真实硬件；TSan本轮NOT RUN，普通多线程或UBSan通过不等于排除所有竞态。
