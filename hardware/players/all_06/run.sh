#!/usr/bin/env bash
set -euo pipefail
task_package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_python="${MTC_HARDWARE_PYTHON:-python3}"
exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 \
  "$task_python" "$task_package_dir/run_hardware.py" "$@"
