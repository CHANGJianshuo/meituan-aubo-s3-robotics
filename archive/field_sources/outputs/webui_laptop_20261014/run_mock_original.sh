#!/usr/bin/env bash
set -eo pipefail
MTC_UI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$MTC_UI_DIR"
source /opt/ros/jazzy/setup.bash
source "$MTC_UI_DIR/install/setup.bash"
export ROS_DOMAIN_ID=99
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR="$MTC_UI_DIR/ros_logs"
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export CYCLONEDDS_URI="file://$MTC_UI_DIR/cyclonedds.xml"
exec /usr/bin/python3 -m mtc_cpp_mock_bringup.bringup \
  --namespace /mtc_laptop_mock --port 8767 --output-dir "$1" \
  --preview-dir /home/operator/meituan/eye_to_hand_calib/data/measurements/red_handle_20261014_104532
