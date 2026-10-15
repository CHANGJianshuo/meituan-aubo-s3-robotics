# shellcheck shell=bash
# 隔离的 Mock 环境变量（与 docs/setup/AUBO_ROS2_DRIVER_JAZZY.md §4 一致）。
# 用法：source env.sh   （需先设置 AUBO_WS 与 MTC_WS，或使用默认路径）
: "${AUBO_WS:=$HOME/meituan/aubo_s3_mock_ws}"
: "${MTC_WS:=$HOME/meituan/meituan_challenge_ws}"
set +u
source /opt/ros/jazzy/setup.bash
source "$AUBO_WS/install_no_symlink/setup.bash"
: "${MTC_INSTALL:=$MTC_WS/install/planner}"
if [ -f "$MTC_INSTALL/setup.bash" ]; then
  source "$MTC_INSTALL/setup.bash"
fi
# SOP 默认 Fast DDS。本机 WSL2 networkingMode=mirrored 下 Fast DDS 本机发现不稳定，
# 可用 MTC_RMW=rmw_cyclonedds_cpp 覆盖（需 ros-jazzy-rmw-cyclonedds-cpp）。
export RMW_IMPLEMENTATION="${MTC_RMW:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/aubo_s3_mock_logs}"
