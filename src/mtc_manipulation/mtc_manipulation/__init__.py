"""mtc_manipulation：单块电池抓放状态机（第二层 PickPlace FSM）。

模块边界（重要）：
    - pick_place_fsm.py  纯 Python 核心，**不导入 rclpy**；可离线单元测试。
    - tool_strategy.py   V1/V2 工具策略与运动原语，纯 Python，不导入 rclpy。
    - fakes.py           Fake 端口，纯 Python，供测试与上层 Mock 集成。
    - pick_place_server.py  唯一允许导入 rclpy 的模块（ROS 2 Action Server 外壳）。

未编译验证：本包面向 ROS 2 Jazzy + mtc_interfaces，但当前机器只有 ROS 2 Humble，
且本次任务禁止 colcon build；因此 ROS 侧字段以源码审阅为准，未做运行时验证。
"""

__all__ = [
    "pick_place_fsm",
    "tool_strategy",
    "fakes",
    "pick_place_server",
]

__version__ = "0.1.0"
