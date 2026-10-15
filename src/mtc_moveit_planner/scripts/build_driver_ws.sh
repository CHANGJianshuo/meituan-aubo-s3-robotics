#!/usr/bin/env bash
# 在独立工作空间构建 AUBO 驱动 5 包（docs/setup/AUBO_ROS2_DRIVER_JAZZY.md §2）。
# 前置：$AUBO_WS/src/aubo_ros2_driver 为 jazzy@53aab83 + aubo_description@47fa5e0，
#       且已打上 docs/migration/patches/aubo_description_fake_hardware_jazzy.patch。
# 只在新目录中构建；绝不在 /home/robot/aubo_driver_validation_ws 中执行。
set -euo pipefail
: "${AUBO_WS:=$HOME/meituan/aubo_s3_mock_ws}"
: "${MTC_WS:=$HOME/meituan/meituan_challenge_ws}"
DRV="$AUBO_WS/src/aubo_ros2_driver"
case "$AUBO_WS" in /home/robot/aubo_driver_validation_ws*) echo "refuse: read-only validation ws"; exit 1;; esac

test "$(git -C "$DRV" rev-parse HEAD)" = 53aab83735236f0c262ecb11e04ef2600241d9d9
test "$(git -C "$DRV/aubo_description" rev-parse HEAD)" = 47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1
cmp "$DRV/aubo_description/urdf/xacro/inc/aubo.ros2_control.xacro" \
    "$MTC_WS/docs/migration/patches/aubo.ros2_control.xacro.locally_fixed"
echo "source fingerprint + fake-hardware patch: PASS"

set +u; source /opt/ros/jazzy/setup.bash; set -u
cd "$AUBO_WS"
colcon build --packages-select aubo_description aubo_dashboard_msgs aubo_msgs \
  --build-base build_no_symlink --install-base install_no_symlink --event-handlers console_direct+
set +u; source install_no_symlink/setup.bash; set -u
colcon build --packages-select aubo_ros2_driver \
  --build-base build_no_symlink --install-base install_no_symlink --event-handlers console_direct+
colcon build --packages-select aubo_moveit_config \
  --build-base build_no_symlink --install-base install_no_symlink
echo "driver workspace build: PASS"
