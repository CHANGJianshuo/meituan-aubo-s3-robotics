#!/usr/bin/env bash
set -euo pipefail
task_package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_python=/home/robot/miniforge3/envs/aubo-s3-hardware/bin/python
if [[ ! -x "$task_python" ]]; then
  echo "Missing NUC Python environment: $task_python" >&2
  exit 1
fi
exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 \
  "$task_python" "$task_package_dir/run_hardware.py" "$@"
