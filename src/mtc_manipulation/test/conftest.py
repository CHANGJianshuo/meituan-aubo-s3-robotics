"""Path helper for offline tests.

当前机器只有 ROS 2 Humble、且本次任务禁止 colcon build，
因此测试必须能在不解包、不安装的情况下直接以源码导入 mtc_manipulation：
本 conftest 只把本功能包源码根目录加入 sys.path，不导入任何 ROS 模块。
"""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
for candidate in (PACKAGE_ROOT, PACKAGE_ROOT.parent):
    if (candidate / "mtc_manipulation" / "pick_place_fsm.py").is_file():
        if str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))
        break
