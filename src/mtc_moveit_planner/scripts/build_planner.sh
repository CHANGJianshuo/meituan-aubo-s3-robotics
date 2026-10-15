#!/usr/bin/env bash
# 构建并测试 FSM Core 4 包 + mtc_moveit_planner（独立 build/install/log 目录，不覆盖他人产物）。
set -euo pipefail
: "${AUBO_WS:=$HOME/meituan/aubo_s3_mock_ws}"
: "${MTC_WS:=$HOME/meituan/meituan_challenge_ws}"
set +u; source /opt/ros/jazzy/setup.bash; source "$AUBO_WS/install_no_symlink/setup.bash"; set -u
cd "$MTC_WS"
colcon --log-base log/planner build --build-base build/planner --install-base install/planner \
  --packages-up-to mtc_moveit_planner --event-handlers console_cohesion+
set +u; source install/planner/setup.bash; set -u
colcon --log-base log/planner test --build-base build/planner --install-base install/planner \
  --packages-select mtc_core_cpp mtc_task_cpp mtc_motion_execution_cpp mtc_manipulation_cpp \
  mtc_moveit_planner --event-handlers console_cohesion+ || true
colcon test-result --test-result-base build/planner --verbose
