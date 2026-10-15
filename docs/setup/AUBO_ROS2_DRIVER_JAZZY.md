# AUBO S3 ROS 2 Jazzy 驱动部署与运行手册

核对日期：2026-10-11。适用范围：**独立 AUBO S3 Fake Hardware / MoveIt 2 Mock 环境**。
项目安排：10 月 12 日开始团队联调，10 月 15 日提交比赛视频；日期安排不构成真实设备操作授权。

FSM Core v0.1 已合入 `main`。独立驱动的 Mock 控制器、FollowJointTrajectory 和 MoveIt
Plan → Execute 已有成功运行记录；**完整 FSM + MoveIt 2 集成仍在进行**，新 C++ Action
外壳、Planner/MotionAdapter 和抓放闭环尚未完成。

本任务只编写文档、只读检查和静态 Mock 核验，没有重编译或启动验证工作空间，
没有连接控制柜、调用真实 RPC/RTDE、发送实体运动或触发工具 IO。

## 1. 工作空间与版本指纹

| 位置 | 用途和边界 |
|---|---|
| `/home/robot/meituan_challenge_ws` | 主开发仓库；FSM源码、文档、验收报告 |
| `/home/robot/aubo_driver_validation_ws` | **只读**驱动验证工作空间；不可在其中clone、patch、build或修改配置 |
| `/home/robot/aubo_driver_validation_ws/src/aubo_ros2_driver` | 实际官方驱动源码根目录 |
| `/home/robot/aubo_driver_validation_ws/install_no_symlink` | 已验证的安装overlay；本文快速启动使用此目录 |
| `/home/robot/aubo_driver_validation_ws/build_no_symlink` | 历史成功构建目录 |

源码指纹已通过本地Git只读命令核对：

| 仓库 | remote | branch / commit |
|---|---|---|
| 驱动父仓库 | `https://github.com/AuboRobot/aubo_ros2_driver.git` | `jazzy` / `53aab83735236f0c262ecb11e04ef2600241d9d9` |
| `aubo_description` 子模块 | `https://github.com/AuboRobot/aubo_description.git` | 固定提交 `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`，detached HEAD |

验证环境不是纯上游checkout：父仓库显示 `m aubo_description`；子模块有一处
已跟踪Xacro修改及运行生成的模型文件。修复和生成物已在
[驱动Fake Hardware修复审计](../migration/AUBO_DRIVER_FAKE_HARDWARE_FIX_AUDIT.md)
及[版本指纹](../migration/patches/driver_version_fingerprint.txt)归档。
本轮没有改动或清理这些文件，也没有读取个人Shell历史或设备秘密配置。

### 环境依赖与实际版本

| 项目 | 本机只读核对结果 |
|---|---|
| Ubuntu / CPU | Ubuntu 24.04.4 LTS / x86_64 |
| ROS 2 | Jazzy，加载 `/opt/ros/jazzy/setup.bash` 后 `ROS_DISTRO=jazzy` |
| 编译工具 | GCC 13.3.0、CMake 3.28.3、Python 3.12.3 |
| colcon / rosdep | `python3-colcon-common-extensions` 0.3.0-100 / `python3-rosdep` 0.27.0-1 |
| ros2_control | `controller_manager` / `hardware_interface` 4.48.1 |
| ros2_controllers | `joint_trajectory_controller` 4.42.1 |
| MoveIt 2 | `ros-jazzy-moveit` / `moveit_ros_move_group` / `moveit_configs_utils` 2.12.4 |
| RViz / Xacro | RViz 14.1.24 / Xacro 2.1.1 |
| RMW | 本机默认 `rmw_fastrtps_cpp`；以下SOP明确使用它 |
| 驱动 / MoveIt配置 / description包 | 2.3.2 / 0.3.0 / 1.3.3（package.xml值，以Git提交为源码身份依据） |
| SDK构建依赖 | 驱动CMake FetchContent使用 `aubo_sdk-0.24.1-rc.3-Linux_x86_64+318754d.zip`；编译依赖不等于调用真实SDK |

这些是本机快照，不是所有队友机器已安装相同版本的保证。
Ubuntu 24.04的Jazzy安装和source方法见
[ROS 2官方安装说明源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)。
已有Jazzy环境无需重复改APT源或系统配置。

## 2. 从源码部署：独立新工作空间

此节供新机器准备环境；**本轮没有执行安装、clone、patch或build**。
不得将下面新部署目录替换为只读验证目录。先安装Jazzy、配置其APT源，再安装
构建/运行依赖；系统安装命令仅供新开发机使用，不在现有验证机自动执行：

```bash
sudo apt update
sudo apt install git build-essential cmake libboost-dev \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros-base ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-moveit ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-rmw-fastrtps-cpp
source /opt/ros/jazzy/setup.bash
```

示例新目录为 `/home/robot/aubo_s3_mock_ws`，它是下面命令将创建的目录，**不是
已经存在或已经验证的工作空间**。目标存在时停止，先检查，不覆盖：

```bash
(
  set -e
  AUBO_TEAM_WS=/home/robot/aubo_s3_mock_ws
  test ! -e "$AUBO_TEAM_WS"
  mkdir -p "$AUBO_TEAM_WS/src"
  git clone --branch jazzy https://github.com/AuboRobot/aubo_ros2_driver.git \
    "$AUBO_TEAM_WS/src/aubo_ros2_driver"
  git -C "$AUBO_TEAM_WS/src/aubo_ros2_driver" switch --detach \
    53aab83735236f0c262ecb11e04ef2600241d9d9
  git -C "$AUBO_TEAM_WS/src/aubo_ros2_driver" submodule update --init --recursive
  git -C "$AUBO_TEAM_WS/src/aubo_ros2_driver" submodule status
)
```

预期子模块提交为上节的47fa5e02。不要只跟踪未来可能变化的branch HEAD。

### 必须先满足的Mock修复前置条件

上游固定提交的hardware分支被注释且使用旧插件/参数名称，**直接从上述纯上游
checkout构建并启动不能复现已验证Mock环境**。部署快照必须包含已审查的
[Mock修复补丁](../migration/patches/aubo_description_fake_hardware_jazzy.patch)：
Fake分支为 `mock_components/GenericSystem`，参数为 `mock_sensor_commands` 和
`position_state_following_offset`，Xacro条件实际生效。

本任务禁止修改驱动配置，因此这里只提供核验，不提供或执行应用补丁的命令。
维护者需在另行允许配置变更的部署流程中准备该修复快照；**新机器补丁安装及
全新部署：NEEDS VERIFICATION**。已经验证的安装目录可直接进入第4节。
以下命令在新副本中比较归档修复，纯上游checkout不一致是预期的阻断结果：

```bash
cmp /home/robot/aubo_s3_mock_ws/src/aubo_ros2_driver/aubo_description/urdf/xacro/inc/aubo.ros2_control.xacro \
  /home/robot/meituan_challenge_ws/docs/migration/patches/aubo.ros2_control.xacro.locally_fixed
```

必须先取得一致的审核快照，不能跳过失败或在只读验证目录就地修复。
本轮确认归档文件与验证环境源码和安装副本SHA256一致；静态展开只产生GenericSystem。

### rosdep依赖与构建

如果机器尚未初始化rosdep，可先按
[官方rosdep教程源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Intermediate/Rosdep.rst)
初始化；已经初始化时不重复执行 `sudo rosdep init`。
下面只安装5个所需包的依赖，不把Gazebo和其他未验收示例纳入部署：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/robot/aubo_s3_mock_ws
rosdep update --rosdistro jazzy
rosdep install --from-paths \
  src/aubo_ros2_driver/aubo_description \
  src/aubo_ros2_driver/aubo_dashboard_msgs \
  src/aubo_ros2_driver/aubo_msgs \
  src/aubo_ros2_driver/aubo_ros2_driver \
  src/aubo_ros2_driver/aubo_moveit_config \
  --ignore-src --rosdistro jazzy -y \
  -t build -t buildtool -t build_export -t buildtool_export -t exec

# 普通复制安装，不使用--symlink-install；先编依赖，再编驱动和MoveIt配置
colcon build --packages-select aubo_description aubo_dashboard_msgs aubo_msgs \
  --build-base build_no_symlink --install-base install_no_symlink \
  --event-handlers console_direct+
source install_no_symlink/setup.bash
colcon build --packages-select aubo_ros2_driver \
  --build-base build_no_symlink --install-base install_no_symlink \
  --event-handlers console_direct+
colcon build --packages-select aubo_moveit_config \
  --build-base build_no_symlink --install-base install_no_symlink
source install_no_symlink/setup.bash
```

构建参数来自历史成功命令；将前三个依赖包合为一次选择是同依赖集的重建入口，
本轮未重建新机器。驱动C++标准为17；历史命令未额外传CMake优化参数，源码在
未指定build type时选择RelWithDebInfo。不要宣称历史使用了自定义Release参数。
首次配置需访问官方SDK下载地址；网络/SDK获取失败属于构建问题，不需要连接机器人。

本轮只读rosdep check发现验证机缺 `ros-jazzy-joint-state-publisher-gui`；它是manifest
exec_depend，但上述已验证的两个Launch均不启动GUI JointState Publisher。
新机器rosdep应补齐依赖；本轮未安装，不能将依赖完整性检查写成PASS。

## 3. 文件、模型与控制器对应关系

下表相对驱动源码根目录；安装后对应在各包 `share/<package>/` 中，均已核对存在。

| 包 / 文件 | 用途 |
|---|---|
| `aubo_ros2_driver/launch/aubo_control.launch.py` | ros2_control_node、robot_state_publisher、两类controller spawner；可选RViz |
| `aubo_ros2_driver/config/aubo_controllers.yaml` | 控制频率200Hz、控制器类型、关节名与接口 |
| `aubo_moveit_config/launch/aubo_moveit.launch.py` | move_group及可选RViz；**不启动controller_manager** |
| `aubo_moveit_config/config/moveit_controllers.yaml` | MoveIt控制器映射到FollowJointTrajectory |
| `aubo_moveit_config/config/aubo_robot.srdf` | 通用SRDF，group `manipulator`，chain `base_link → ee_link` |
| `aubo_moveit_config/config/kinematics.yaml` | KDL求解器配置 |
| `aubo_moveit_config/config/ompl_planning.yaml` | OMPL pipeline |
| `aubo_moveit_config/config/joint_limits.yaml` | 速度/加速度配置；不是实机标定安全限值证明 |
| `aubo_moveit_config/config/moveit.rviz` | RViz MotionPlanning显示配置 |
| `aubo_description/urdf/aubo_S3.urdf` | S3模型来源 |
| `aubo_description/urdf/aubo_S3.urdf.xacro` | 构建/Launch生成的模型副本；不单独提供ros2_control段 |
| `aubo_description/urdf/xacro/inc/aubo_ros2.xacro` | 总入口，组装模型和ros2_control |
| `aubo_description/urdf/xacro/inc/aubo_ros2_macro.xacro` | S3模型及 `ee_link` 等组装 |
| `aubo_description/urdf/xacro/inc/aubo.ros2_control.xacro` | GenericSystem与真实硬件分支选择，已归档修复点 |

控制manager为 `/controller_manager`，硬件组件名为 `auboHardwareInterface`。
**组件名看起来像实机，但只有plugin name才能区分**：本手册要求它为
`mock_components/GenericSystem`。该插件是理想化命令/状态镜像，不是物理动力学
仿真或实机安全证明。[ros2_control官方Mock说明](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)

| controller | type | 本文预期状态 |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | active |
| `forward_command_controller_position` | `forward_command_controller/ForwardCommandController` | YAML存在，SOP不加载，不用于验证 |

关节命令顺序为：`shoulder_joint`、`upperArm_joint`、`foreArm_joint`、`wrist1_joint`、
`wrist2_joint`、`wrist3_joint`。命令接口position，状态接口position/velocity，
`allow_partial_joints_goal=false`。MoveIt YAML中upperArm/foreArm的列举顺序不同；
必须按名称映射，不能直接假定所有数组顺序相同。
Action为 `/joint_trajectory_controller/follow_joint_trajectory`，类型为
`control_msgs/action/FollowJointTrajectory`。

还存在 `aubo_client.launch.py`、`aubo_viewer.launch.py`、`aubo_gazebo.launch.py`、
`ros_joints_plan/launch/joints_plan.launch.py` 等入口；**存在不等于本手册验证通过**，
尤其不启动client、SDK示例或校准脚本。

## 4. 每个终端的环境加载与隔离

快速启动使用现有安装overlay，**不在验证目录重新编译**。每个终端分别执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot/aubo_driver_validation_ws/install_no_symlink/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=91
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/aubo_s3_mock_logs
```

91是本SOP选用的隔离domain，不是历史运行参数或官方指定值。启动前确认此domain
没有其他ROS应用；占用时先协调另一个空domain，并在所有终端保持一致。
LOCALHOST限制自动发现，清除静态peers；它不能代替Fake插件核验，也不能隔离
同主机同domain的其他驱动。[ROS 2官方发现配置说明源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst)

日志写入 `/tmp`，不写只读工作空间，也不永久修改 `.bashrc`。
新机器使用自己的审核overlay替代第2行；不要同时source两个AUBO安装目录。
本机确认 `ros2 pkg prefix aubo_description` 等指向 `install_no_symlink/<package>`。

```bash
echo "$ROS_DISTRO"
ros2 pkg prefix aubo_ros2_driver
ros2 pkg prefix aubo_description
ros2 pkg prefix aubo_moveit_config
ros2 node list --no-daemon --spin-time 2
```

预期jazzy、三个包来自同一overlay；未启动时node list为空。
发现已有节点或来源目录不符，停止并检查；不要直接停掉不属于本SOP的进程。

## 5. Fake Hardware启动及启动前核验

**控制Launch默认 `use_fake_hardware` 为false、默认型号为aubo_i5；不得裸启动。**
始终显式指定 `aubo_type:=aubo_S3`、`use_fake_hardware:=true`。
先执行下面只读核验。已存在模型副本很重要：Launch的ensure函数在缺文件时会
写入description share；只读参考环境不能让它走生成路径。

```bash
python3 - <<'PY'
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

share = Path(get_package_share_directory('aubo_description'))
assert (share / 'urdf/aubo_S3.urdf.xacro').is_file(), '缺少生成模型；停止，不在参考目录生成'
xml = subprocess.check_output([
    'xacro', str(share / 'urdf/xacro/inc/aubo_ros2.xacro'),
    'aubo_type:=aubo_S3', 'use_fake_hardware:=true'], text=True)
plugins = [p.text.strip() for p in ET.fromstring(xml).findall('ros2_control/hardware/plugin')]
assert plugins == ['mock_components/GenericSystem'], plugins
print('PASS：S3只选择GenericSystem')
PY
```

只有核验PASS且空domain前置检查完成后，终端1启动：

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_S3 use_fake_hardware:=true \
  initial_joint_controller:=joint_trajectory_controller launch_rviz:=false
```

终端1保持运行。预期日志出现 `update rate is 200 Hz`、
`Loaded hardware ... from plugin 'mock_components/GenericSystem'`，以及两controller
configure/activate成功。发现其他plugin、错误型号或失败，停止该Mock启动并定位，
不得改成真实模式来绕过错误。200Hz配置/启动日志不等于硬实时性能认证。

## 6. ros2_control、JointState和TF检查

终端3加载第4节环境后执行以下只读命令：

```bash
ros2 control list_hardware_components -c /controller_manager -v
ros2 control list_controllers -c /controller_manager
ros2 control list_hardware_interfaces -c /controller_manager
ros2 param get /controller_manager update_rate
ros2 topic info /joint_states -v
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --timeout 10
ros2 topic hz /joint_states
```

预期只有一个hardware component，plugin为GenericSystem且active；两controller
active；六个position命令接口available/claimed，position/velocity状态接口available；
update_rate=200；JointState包含六个名称及有限位置，时间戳持续更新。
JointState消息顺序按name解析，不能只按索引套用控制YAML。
`topic hz` 持续运行，观察后Ctrl+C仅退出监视；实际消息频率不必等于manager配置频率。

```bash
ros2 topic list -t
ros2 run tf2_ros tf2_echo world base_link
```

观察world/base_link固定TF后Ctrl+C，再检查末端链：

```bash
ros2 run tf2_ros tf2_echo base_link ee_link
```

URDF/SRDF实际frame为world、base_link、ee_link，大小写必须匹配。
tf2_echo是只读监视，观察后Ctrl+C；初始发现等待与持续缺失要区分。
若缺TF，检查 `/robot_state_publisher`、JointState和模型加载，不能凭RViz模型显示
就认定完整TF/运动反馈已正常。

## 7. MoveIt 2 / RViz启动

终端1及第6节核验正常后，终端2加载第4节环境并执行：

```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
  aubo_type:=aubo_S3 launch_rviz:=true
```

终端2保持运行，管理move_group与RViz。该Launch只有description/MoveIt配置/
型号/RViz等参数，**没有use_fake_hardware的DeclareLaunchArgument**，不要编造
该参数；它的description使用总Xacro默认Fake值，实际执行仍连接终端1的controller。
本机默认通用 `config/aubo_robot.srdf`，不是把aubo_i5 SRDF文件名当作S3配置。

预期 `/move_group` 出现，RViz加载MotionPlanning，group选 `manipulator`，
Planning Scene Topic为 `/monitored_planning_scene`，模型和当前JointState一致。
若没有图形桌面，可用 `launch_rviz:=false` 启动move_group，但无GUI的Plan/Execute
专用验证工具本任务没有实现，标为 **NEEDS VERIFICATION**，不能只因节点启动而PASS。

## 8. FollowJointTrajectory与Plan → Execute验证方法

先在终端3检查接口，不发送Goal：

```bash
ros2 action list -t
ros2 action info /joint_trajectory_controller/follow_joint_trajectory -t
ros2 interface show control_msgs/action/FollowJointTrajectory
```

预期controller Action server只有一个、类型正确。必须同时确认第6节仅有
GenericSystem；只看到Action名称不证明它指向Mock。

在核验后的隔离环境，通过RViz测试完整链路：

1. 在MotionPlanning选择 `manipulator`，Start State取当前状态；确认没有其他
   controller或程序在修改关节，保留全部六关节目标，不用旧规划起点。
2. 在Joints界面设一个可规划的小变化，例如仅肩关节相对当前值改变0.05rad，
   其余保持当前值。这是Mock检查量，不是实机运动参数或正式抓放轨迹。
3. 点击Plan，核对终端2规划成功和显示轨迹；规划失败时不点击Execute。
4. 再次确认Mock硬件及唯一Action server，然后Execute刚生成的轨迹。
5. 联合核对终端1的 `Received new action goal` / `Goal reached, success!`、
   终端2的 `Completed trajectory execution with status SUCCEEDED`，及JointState
   最终位置按名称接近规划目标。记录起点、目标、结果、实际日志和Mock插件。

RViz的Execute经MoveIt controller manager发到上述FollowJointTrajectory，验证
完整轨迹提交；controller仅accepted不能算PASS。为避免复制到错误ROS图时误发，
本手册/SOP不提供裸 `ros2 action send_goal`、关节command topic发布或SDK运动命令。
单独绕过MoveIt的CLI轨迹client本轮不重建，**NEEDS VERIFICATION**；历史直接
FollowJointTrajectory成功由controller运行日志证明，不能凭文件名或接口存在证明。

此验证只说明Mock执行链路可用，不能证明真实停稳、负载、接触、工具释放或放置
稳定。FSM成功仍须独立感知/工具证据与严格停稳契约。

## 9. 常见编译和启动问题

| 现象 | 检查和处理 |
|---|---|
| package not found / 查到另一overlay | 重开终端，按第4节source；核对三个pkg prefix。不要混用install和install_no_symlink |
| description symlink安装失败 | 历史11:32日志确有 `Could not create symlink ... link0.DAE`；新副本改用独立普通复制安装目录，保留旧目录，不清空验证工作空间 |
| 只select驱动却找不到aubo消息/description | 先按第2节构建依赖并source相同install base；不要用不存在的依赖安装目录继续构建 |
| SDK下载或动态库失败 | 检查构建的 `_deps/aubo_sdk-src` 和已安装lib；历史使用上述SDK release。不要运行SDK示例验证下载成功 |
| hardware plugin为空 / fake_components找不到 | 原始上游缺陷；核对归档修复和静态插件展开。停止部署，维护者在另行授权流程处理，不改只读驱动配置 |
| spawner等待controller_manager | 检查终端1、同domain/RMW、模型和plugin错误；Launch有2秒延迟、120秒manager等待，不重复启动第二个manager |
| controller inactive或接口未claimed | 用第6节只读命令检查；不自动切换其他controller或启用forward command路径 |
| JointState/TF无数据 | 检查broadcaster、robot_state_publisher、S3模型、name映射及时间；核对额外JointState Publisher的来源，由其启动者处理重复发布 |
| RViz场景不更新 | 检查 `/monitored_planning_scene`、group、TF、JointState和终端2；不能只看RViz显示就认为执行成功 |
| Invalid Trajectory: start point deviates ... 0.01 | 历史确有此错误；从当前状态重新Plan，确认期间无其他运动。不扩大allowed_start_tolerance或跳过起点核验 |
| 退出时move_group不退出或exit code -11 | 历史Ctrl+C后5秒升级SIGTERM并出现-11；正常退出可靠性 **NEEDS VERIFICATION**。保留本次日志，不把它记为干净退出；不用全局pkill/killall清场 |
| rosdep check缺GUI依赖 | 验证机缺joint-state-publisher-gui；当前Launch不用它，依赖完整性仍不是PASS。新部署按rosdep补齐，不在本任务安装系统包 |

## 10. 10 月 12 日团队联调快速启动

**A：已验证的独立Fake环境。B：尚未完成的FSM集成。C：真实硬件须现场单独授权。**
下面SOP仅执行A；不因10月12日到来自动进入C。10月15日视频目标不能代替验收证据。

| 步骤 | 终端与操作 | 预期结果 | 出错检查 |
|---|---|---|---|
| 1 | 打开三个新的Bash终端，编号1/2/3；每个执行第4节全部source和隔离设置 | jazzy，同一overlay/domain/RMW，日志在/tmp | 第4节echo、pkg prefix、node list |
| 2 | 若使用现有验证机，**跳过编译**；若是新机器，先由部署维护者完成第2节修复快照/依赖/构建后再开始SOP | 已安装5包、模型存在；不写验证目录 | package前缀、构建日志、cmp；前置不完整则NEEDS VERIFICATION并停止 |
| 3 | 终端3执行第5节静态核验和第4节空domain检查 | 只含GenericSystem、没有其他节点 | 修复快照/overlay/domain；不跳过失败 |
| 4 | 终端1执行第5节Fake控制Launch，**保持运行** | 200Hz启动、GenericSystem、两controller active | 终端1日志、第6节controller/hardware检查 |
| 5 | 终端3执行第6节检查；topic hz和tf2_echo观察后Ctrl+C | 有限且更新的六关节反馈、world/base/ee TF | node/topic/control命令；Fake插件不对时停止A环境 |
| 6 | 终端2执行第7节MoveIt/RViz Launch，**保持运行** | move_group、RViz、manipulator、当前模型一致 | 终端2日志、node list、场景topic、JointState |
| 7 | 终端3查第8节Action信息；通过Mock身份/唯一server检查后才在RViz Plan → Execute | controller目标完成、MoveIt SUCCEEDED、反馈到位 | 旧起点先重Plan；accepted不记为通过 |
| 8 | 保存/tmp下本次日志、测试目标和Mock身份；按下面顺序结束 | 自己启动的节点退出，其他节点不受影响 | 自己Launch的退出日志、同domain node list |

### 正常结束Mock环境

1. 不再创建新Plan/Execute，等待当前Mock轨迹结果；不发送实机Stop或IO。
2. 在终端3对topic hz/tf2_echo等只读监视Ctrl+C。
3. 在终端2对MoveIt Launch Ctrl+C，等待其子进程退出；不要只关闭RViz窗口就
   假定move_group也结束。历史退出异常按第9节登记，不能冒充PASS。
4. 在终端1对控制Launch Ctrl+C，等待controller和GenericSystem deactivate/shutdown。
5. 终端3执行 `ros2 node list --no-daemon --spin-time 2`，预期本SOP节点不再出现。
   若仍有节点，核对自己启动的PID及所属Launch，保留错误；不执行全局清理命令。

终端1/2在验证期间必须保持运行，终端3可退出单个监视而不影响服务。
Ctrl+C或退出软件不是实机安全急停；C类操作不在本文SOP中，未给出真实模式、
设备地址、账号、校准连接、SDK运动或实体IO命令。

## 11. 与FSM MotionAdapter的集成边界

当前四包API和安全契约见 [FSM开发者指南](../../README_FSM.md) 与
[最终安全验收报告](../migration/CPP_FSM_FINAL_SAFETY_GATE.md)。未来集成关系为：

```text
TaskFsm → ActionRequest → PickPlaceFsm
                            ├─ PlannerPort → MoveIt规划结果（待接入）
                            └─ MotionPort / MotionAdapter（待接入ROS Action）
                                   → 一条FollowJointTrajectory Goal
                                   → GenericSystem（仅Mock阶段）
```

* Task/PickPlace仍由单事件线程拥有；ROS回调入串行队列，长执行由worker处理，
  不能用当前同步MotionAdapterPort阻塞FSM事件线程等待长轨迹。
* PlannerPort必须返回完整 `PlanResult.commands`，保留6D Pose/frame/stamp/约束。
  多点时间轨迹作为一次FollowJointTrajectory提交，不拆成多个ExecuteJointMove。
* 当前MotionExecutorCore只提供同步关节运动执行入口，MotionAdapterPort整条轨迹
  路径继承明确拒绝；新ROS异步轨迹adapter/Planner/Action外壳不是已经完成的功能。
* Goal取消保留原command_id，调用 `cancel_goal(id, reason)`；无ID
  `supervisory_stop(reason)` 是监督停止，不能用于迟到Goal取消。后端需处理
  accepted/cancel/result交错、重复取消、迟到结果及worker生命周期。
* 停稳需command关联、执行状态、队列与连续新鲜反馈证据；Action成功/取消受理
  不直接构造StopEvidence。未知/超时未停稳继续锁定，不自动重发或自动人工复位。
* 纯逻辑MockMotionAdapter使用共享ManualClock；ROS GenericSystem环境仍有实际
  ROS反馈时间，集成时需统一单调时基并明确ROS stamp转换，不能虚拟推进真实等待。
* controller名字/六关节名称按本手册映射；GUI默认限值、模型和0.05rad测试量均
  不是已审核实机约束。工具IO、感知和放置稳定性核验仍需独立闭环。

尚无可复制的完整FSM + MoveIt启动命令：**NEEDS VERIFICATION**，不得编造
`ros2 launch mtc_*_cpp ...`。详细待办见
[第二阶段集成计划](../migration/PHASE2_ROS2_INTEGRATION_PLAN.md)。

## 12. 已验证、失败与未验证的证据清单

| 项目 | 状态 | 证据与限制 |
|---|---|---|
| 源码/子模块/安装文件核对 | PASS（本轮只读） | 上述完整commit、安装文件存在且Launch与源码一致 |
| S3 Fake Xacro | PASS（本轮静态） | 执行本文核验代码，展开仅1个GenericSystem、六关节，world/base_link/ee_link存在；归档/源码/安装修复相同 |
| 文档链接、路径及命令参数 | PASS（本轮静态） | 31个本地链接、14个Bash代码块语法、18个源码路径、5个安装包；控制器/SRDF和CLI帮助核验通过 |
| 普通复制安装的5包构建 | PASS（历史日志） | 11:42:02 description、11:43:03 dashboard、11:43:25 msgs、11:43:59 driver、12:04:03 MoveIt；colcon返回0 |
| description symlink安装 | FAIL（历史） | 11:32:15构建返回1，link0.DAE symlink错误；不能把所有历史build都记为成功 |
| Fake硬件及controller激活 | PASS（历史运行） | GenericSystem加载、200Hz、两controller activate |
| 直接FollowJointTrajectory | PASS（历史运行） | controller日志Received/Accepted Goal及Goal reached success，不仅接口存在 |
| MoveIt Plan → Execute | PASS（历史运行） | 规划成功、controller成功、move_group SUCCEEDED、RViz Execute request success相互印证 |
| 旧规划起点执行 | FAIL（历史负面） | 起点超出0.01而ABORTED；需当前状态重新规划 |
| MoveIt正常退出 | NEEDS VERIFICATION | 已有SIGINT未及时退出、升级SIGTERM后-11记录，不宣称干净退出 |
| manifest依赖完整性 | FAIL（本轮只读check） | 缺joint_state_publisher_gui；当前Launch不用它，但仍需说明 |
| 驱动自动测试 | NOT RUN / 未取得证据 | build目录未找到test_results XML，不声称colcon test通过 |
| 本轮动态Mock图启动/运动复跑 | NOT RUN | 本轮未重启Launch或发Goal；动态PASS引用明确历史日志 |
| 新机器全新部署/GUI之外的专用验证client | NEEDS VERIFICATION | 配置修复前置与新环境复验仍需落实 |
| Gazebo、ros_joints_plan及其他示例 | NOT RUN | 文件存在但不在本手册验收范围 |
| 完整C++ FSM + MoveIt / ROS Action抓放闭环 | NOT RUN / 尚未完成 | 第一阶段核心通过不等于第二阶段完成 |
| 真实AUBO S3、实体工具IO/负载/标定 | NOT RUN | 未授权，本轮禁止 |

历史编译证据位于只读工作空间 `log/build_2026-10-11_*/logger_all.log` 及各包的
`command.log` / `stderr.log`，本轮核对实际命令和返回码，没有重写日志。
运行证据为ROS进程日志，不是个人Shell历史：

* `/home/robot/.ros/log/ros2_control_node_33423_1791692562616.log`
* `/home/robot/.ros/log/ros2_control_node_34608_1791693746781.log`
* `/home/robot/.ros/log/move_group_34716_1791693771783.log`
* `/home/robot/.ros/log/2026-10-11-12-42-51-665002-asusNUC05-34713/launch.log`

关键实际输出摘要（省略设备连接字段）：

```text
Loaded hardware 'auboHardwareInterface' from plugin 'mock_components/GenericSystem'
update rate is 200 Hz
Goal reached, success!
Motion plan was computed successfully.
Controller 'joint_trajectory_controller' successfully finished
Completed trajectory execution with status SUCCEEDED ...
Execution completed: SUCCEEDED

Invalid Trajectory: start point deviates from current robot state more than 0.01 ...
Execution completed: ABORTED
process[move_group-1] failed to terminate '5' seconds after receiving 'SIGINT'
process has died ... exit code -11
```

这些本机日志路径不是其他队友机器必然存在的文件；本报告只保存安全摘要，
没有复制设备凭据或整份潜在敏感日志。复验时保存自己的新日志和判定。

## 13. 安全与维护约定

只读验证环境是基线，不能为部署手册清理或覆盖build/install/log，也不修改驱动
配置。所有新机器源码安装发生在独立目录；所需修复由部署维护者另行审核，
不能假定纯上游版本已含本地Mock修复。未知命令或前置条件标NEEDS VERIFICATION。

SOP始终显式Fake模式；不启动真实硬件分支、client、SDK测试、校准连接或实体IO。
只要插件/overlay/domain/Action server身份不符就停止验证。普通Stop、Ctrl+C、
Action canceled或进程退出均不能替代真实停稳确认和安全急停。

10月12日的C类现场联调须单独授权与现场安全流程，本文不提供可误触发实机的
命令。视频中Mock演示、FSM逻辑测试和实机演示必须分别说明，不混用验证结论。
