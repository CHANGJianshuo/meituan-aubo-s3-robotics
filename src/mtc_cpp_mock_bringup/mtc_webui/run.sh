#!/usr/bin/env bash
# Optional wrapper for a configured workspace; no machine-specific default path.
set -eo pipefail
: "${MTC_UI_SETUP:?Set MTC_UI_SETUP to this candidate workspace install/setup.bash}"
source /opt/ros/jazzy/setup.bash
source "$MTC_UI_SETUP"
exec ros2 run mtc_cpp_mock_bringup web_console "$@"
