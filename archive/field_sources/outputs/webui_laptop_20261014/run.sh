#!/usr/bin/env bash
set -eo pipefail
MTC_UI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$MTC_UI_DIR"
export PYTHONPATH="$MTC_UI_DIR/source/src/mtc_cpp_mock_bringup${PYTHONPATH:+:$PYTHONPATH}"
MTC_RUN_DIR="$1"
shift
exec /usr/bin/python3 -m mtc_webui.server --port 8767 --output-dir "$MTC_RUN_DIR" \
  --hardware-config "$MTC_UI_DIR/hardware_integration/hardware_config.json" "$@"
