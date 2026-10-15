#!/usr/bin/env bash
# 一键采集规划证据（Mock / Fake Hardware，只规划不执行）：
#   1) 空 ROS 域检查 + 静态 Xacro 核验（只允许 mock_components/GenericSystem）
#   2) 启动 Fake 控制 launch 与 move_group（无 RViz），各自独立进程组
#   3) 只读采集：hardware/controllers/nodes/topics/actions/joint_states/TF 图/URDF/SRDF
#   4) 运行 plan_fixed_pose（含场景 ID 隔离反证、碰撞反证）与 plan_battery_sequence（V1、V2）
#   5) 只结束本脚本启动的进程组；不使用全局 pkill/killall
# 不发送任何 FollowJointTrajectory goal，不连接真实设备。
#
# 用法：bash run_planning_evidence.sh [输出目录]
set -uo pipefail
set -m   # 每个后台 launch 自成进程组：PGID == $!，退出时只结束自己的进程组
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$SCRIPT_DIR/env.sh"
OUT="${1:-$MTC_WS/docs/planning/evidence/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
SUMMARY="$OUT/SUMMARY.txt"
: > "$SUMMARY"
note() { echo "$*" | tee -a "$SUMMARY"; }

CTRL_PGID=""
MG_PGID=""
stop_group() {
  local pgid="$1" name="$2"
  [ -z "$pgid" ] && return 0
  if kill -0 -- "-$pgid" 2>/dev/null; then
    # Signal the launch parent once; launch forwards SIGINT to its children.
    # Signalling the whole group also makes launch forward a second SIGINT.
    kill -INT "$pgid" 2>/dev/null
    for _ in $(seq 1 30); do kill -0 -- "-$pgid" 2>/dev/null || break; sleep 0.5; done
    if kill -0 -- "-$pgid" 2>/dev/null; then
      note "FAIL: $name did not exit 15 s after SIGINT; sending SIGTERM to its own process group"
      kill -TERM -- "-$pgid" 2>/dev/null
      sleep 3
    fi
  fi
  # Reap this script's launch child so its final lifecycle records are flushed.
  if kill -0 -- "-$pgid" 2>/dev/null; then
    note "FAIL: $name survived SIGTERM; sending SIGKILL to its own process group"
    kill -KILL -- "-$pgid" 2>/dev/null
  fi
  wait "$pgid" 2>/dev/null || true
}
cleanup() {
  stop_group "$MG_PGID" "move_group launch"
  stop_group "$CTRL_PGID" "control launch"
}
trap cleanup EXIT

note "== mtc_moveit_planner evidence $(date '+%F %T %Z') =="
note "ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
note "aubo_ros2_driver prefix: $(ros2 pkg prefix aubo_ros2_driver)"
note "mtc_moveit_planner prefix: $(ros2 pkg prefix mtc_moveit_planner)"
note "driver commit: $(git -C "$AUBO_WS/src/aubo_ros2_driver" rev-parse HEAD)"
note "aubo_description commit: $(git -C "$AUBO_WS/src/aubo_ros2_driver/aubo_description" rev-parse HEAD)"
note "fake patch sha256: $(sha256sum "$AUBO_WS/src/aubo_ros2_driver/aubo_description/urdf/xacro/inc/aubo.ros2_control.xacro" | cut -d' ' -f1)"
note "team repo: $(git -C "$MTC_WS" rev-parse --abbrev-ref HEAD) @ $(git -C "$MTC_WS" rev-parse HEAD)"

# ---- 1. 前置检查 ----------------------------------------------------------
EXISTING="$(ros2 node list --no-daemon --spin-time 2 2>/dev/null)"
if [ -n "$EXISTING" ]; then
  note "ABORT: ROS domain $ROS_DOMAIN_ID is not empty:"; note "$EXISTING"; exit 1
fi
python3 - "$OUT" <<'PY' | tee -a "$SUMMARY"
import subprocess, sys, xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
out = Path(sys.argv[1])
share = Path(get_package_share_directory('aubo_description'))
xml = subprocess.check_output(['xacro', str(share / 'urdf/xacro/inc/aubo_ros2.xacro'),
                               'aubo_type:=aubo_S3', 'use_fake_hardware:=true'], text=True)
(out / 'aubo_S3_fake_expanded.urdf').write_text(xml)
root = ET.fromstring(xml)
plugins = [p.text.strip() for p in root.findall('ros2_control/hardware/plugin')]
assert plugins == ['mock_components/GenericSystem'], plugins
links = [l.get('name') for l in root.findall('link')]
joints = [(j.get('name'), j.get('type'), j.find('parent').get('link'), j.find('child').get('link'))
          for j in root.findall('joint')]
print('PASS static xacro: plugin=mock_components/GenericSystem')
print('links:', links)
for j in joints:
    print('joint: %s (%s) %s -> %s' % j)
PY
[ "${PIPESTATUS[0]}" -eq 0 ] || { note "ABORT: static xacro check failed"; exit 1; }
cp "$(ros2 pkg prefix aubo_moveit_config)/share/aubo_moveit_config/config/aubo_robot.srdf" "$OUT/"
note "srdf sha256: $(sha256sum "$OUT/aubo_robot.srdf" | cut -d' ' -f1)"

# ---- 2. 启动 Fake 控制与 move_group ----------------------------------------
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_S3 use_fake_hardware:=true \
  initial_joint_controller:=joint_trajectory_controller launch_rviz:=false \
  > "$OUT/control_launch.log" 2>&1 &
CTRL_PGID=$!
ok=0
for _ in $(seq 1 60); do
  if [ "$(grep -c "Configured and activated" "$OUT/control_launch.log")" -ge 2 ]; then ok=1; break; fi
  sleep 1
done
[ "$ok" -eq 1 ] || { note "ABORT: controllers not active within 60 s (see control_launch.log)"; exit 1; }
note "PASS controllers active"

# Optional, explicit workaround for the upstream MoveIt 2.12.4 plugin-unload
# shutdown crash. Scoped to this launch subtree; never enabled by default.
# See docs/planning/MOVE_GROUP_SHUTDOWN_AUDIT.md before using it.
MG_ENV=(env)
case "${MTC_MOVE_GROUP_PIN_PLUGINS:-0}" in
  0) ;;
  1)
    PIN_LIBS="$(ros2 pkg prefix moveit_simple_controller_manager)/lib/libmoveit_simple_controller_manager.so:$(ros2 pkg prefix moveit_ros_move_group)/lib/libmoveit_move_group_default_capabilities.so"
    MG_ENV+=("LD_PRELOAD=$PIN_LIBS")
    note "WARN: diagnostic plugin-retention workaround enabled for move_group launch only; upstream shutdown defect remains"
    ;;
  *) note "ABORT: MTC_MOVE_GROUP_PIN_PLUGINS must be 0 or 1"; exit 1 ;;
esac
"${MG_ENV[@]}" ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_S3 launch_rviz:=false \
  > "$OUT/move_group_launch.log" 2>&1 &
MG_PGID=$!
ok=0
for _ in $(seq 1 90); do
  if grep -q "You can start planning now" "$OUT/move_group_launch.log"; then ok=1; break; fi
  sleep 1
done
[ "$ok" -eq 1 ] || { note "ABORT: move_group not ready within 90 s (see move_group_launch.log)"; exit 1; }
note "PASS move_group ready"

# ---- 3. 只读采集 ------------------------------------------------------------
# ros2 CLI 在 WSL2 mirrored 网络下连接本机 daemon 端口会超时，改用 rclpy 只读快照。
timeout 60 python3 "$SCRIPT_DIR/ros_graph_snapshot.py" > "$OUT/ros_graph.txt" 2>&1
grep -q "plugin=mock_components/GenericSystem state=active" "$OUT/ros_graph.txt" \
  && note "PASS runtime hardware component: mock_components/GenericSystem (active)" \
  || { note "ABORT: runtime plugin is not mock_components/GenericSystem (see ros_graph.txt)"; exit 1; }
grep -q "joint_trajectory_controller type=joint_trajectory_controller/JointTrajectoryController state=active" "$OUT/ros_graph.txt" \
  && note "PASS joint_trajectory_controller active" || note "WARN: joint_trajectory_controller not active"
for pair in "world base_link" "base_link ee_link" "world ee_link"; do
  # shellcheck disable=SC2086
  timeout 6 ros2 run tf2_ros tf2_echo $pair > "$OUT/tf2_echo_${pair// /_to_}.txt" 2>&1
done
( cd "$OUT" && timeout 30 ros2 run tf2_tools view_frames > view_frames.log 2>&1 )
ls "$OUT"/frames_*.pdf >/dev/null 2>&1 && note "PASS TF graph exported: $(basename "$(ls "$OUT"/frames_*.pdf | head -1)")" \
  || note "WARN: view_frames produced no PDF (see view_frames.log)"

# ---- 4. 规划工具 ------------------------------------------------------------
run_tool() {
  local name="$1"; shift
  ros2 launch mtc_moveit_planner plan_tool.launch.py output_dir:="$OUT" "$@" > "$OUT/$name.log" 2>&1
  # ros2 launch 的返回码不反映节点结果，以工具自身的结论行为准。
  if grep -qE "PLAN OK|SEQUENCE PLAN OK" "$OUT/$name.log"; then
    note "PASS $name"
  else
    note "FAIL $name: $(grep -oE "PLAN FAILED.*|SEQUENCE PLAN FAILED.*|\[(ACQUIRE|TRANSPORT)\] ok=false.*" "$OUT/$name.log" | head -2 | tr '\n' ' ')"
  fi
}
# 场景 ID 隔离反证：先写入其他模块的同名对象与本模块遗留对象，再由规划工具 apply_scene。
timeout 60 python3 "$SCRIPT_DIR/scene_isolation_probe.py" seed > "$OUT/scene_isolation.log" 2>&1
run_tool plan_fixed_pose program:=plan_fixed_pose
if timeout 60 python3 "$SCRIPT_DIR/scene_isolation_probe.py" check >> "$OUT/scene_isolation.log" 2>&1; then
  note "PASS scene_isolation: foreign objects kept, stale mtc_moveit_planner/* removed (see scene_isolation.log)"
else
  note "FAIL scene_isolation (see scene_isolation.log)"
fi
timeout 60 python3 "$SCRIPT_DIR/scene_isolation_probe.py" cleanup >> "$OUT/scene_isolation.log" 2>&1
# 碰撞反证：目标在桌面内部，必须失败（失败 = PASS）。
ros2 launch mtc_moveit_planner plan_tool.launch.py output_dir:="$OUT/collision_probe" program:=plan_fixed_pose \
  extra_params_file:="$(ros2 pkg prefix mtc_moveit_planner)/share/mtc_moveit_planner/config/collision_probe_override.yaml" \
  > "$OUT/collision_probe.log" 2>&1
if grep -q "PLAN FAILED" "$OUT/collision_probe.log"; then
  note "PASS collision_probe: goal inside table rejected ($(grep -oE 'PLAN FAILED \[[A-Z_]*\][^>]*' "$OUT/collision_probe.log" | head -1 | cut -c1-160))"
else
  note "FAIL collision_probe: goal inside table was NOT rejected"
fi
# 工具碰撞反证：法兰离桌面 9 cm、舌规会插入桌面；工具模型生效时必须失败（失败 = PASS）。
ros2 launch mtc_moveit_planner plan_tool.launch.py output_dir:="$OUT/tool_collision_probe" program:=plan_fixed_pose \
  extra_params_file:="$(ros2 pkg prefix mtc_moveit_planner)/share/mtc_moveit_planner/config/tool_collision_probe_override.yaml" \
  > "$OUT/tool_collision_probe.log" 2>&1
if grep -q "PLAN FAILED" "$OUT/tool_collision_probe.log"; then
  note "PASS tool_collision_probe: hook tool would enter the table and the goal was rejected"
else
  note "FAIL tool_collision_probe: goal accepted although the hook tool would enter the table"
fi
run_tool plan_battery_sequence_v1 program:=plan_battery_sequence
run_tool plan_battery_sequence_v2 program:=plan_battery_sequence \
  extra_params_file:="$(ros2 pkg prefix mtc_moveit_planner)/share/mtc_moveit_planner/config/sequence_v2_override.yaml"

# ---- 5. 退出 ----------------------------------------------------------------
cleanup
CTRL_PGID=""; MG_PGID=""
for launch_log in move_group_launch control_launch; do
  if python3 "$SCRIPT_DIR/check_launch_shutdown.py" "$OUT/$launch_log.log" > "$OUT/${launch_log}_shutdown.txt"; then
    note "PASS $launch_log clean shutdown"
  else
    note "FAIL $launch_log clean shutdown (see ${launch_log}_shutdown.txt)"
  fi
done
if LEFT="$(ros2 node list --no-daemon --spin-time 2 2>/dev/null)"; then
  if [ -z "$LEFT" ]; then note "PASS ROS domain empty after cleanup"; else note "FAIL nodes still present:"; note "$LEFT"; fi
else
  note "FAIL unable to query ROS graph after cleanup"
fi
note "evidence directory: $OUT"
# Planning and shutdown failures must be reflected in the script exit status.
! grep -q '^FAIL' "$SUMMARY"
