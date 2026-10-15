# RobotState.msg（拟新增，来源：状态机设计 V0.1 §4.7）
# 由适配层规约后的信息。底层 SDK 原始状态码必须另行记日志，
# 不得用布尔值丢弃关键安全原因。

std_msgs/Header header
bool communication_ok
bool robot_ready
bool motion_active
bool safety_normal
bool stop_confirmed
bool simulation_mode
string controller_mode
string detail
