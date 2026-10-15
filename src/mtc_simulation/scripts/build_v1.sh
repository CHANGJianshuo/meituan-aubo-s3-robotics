#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
driver_setup="${MTC_DRIVER_SETUP:-/home/operator/meituan/aubo_s3_mock_ws/install_no_symlink/setup.bash}"
if [[ ! -f "$driver_setup" ]]; then
  echo "Set MTC_DRIVER_SETUP to the AUBO description/MoveIt configuration overlay." >&2
  exit 1
fi
source "$driver_setup"
cd "$root"
colcon build --build-base build/v1_sim --install-base install/v1_sim \
  --packages-up-to mtc_simulation --parallel-workers 4 --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
