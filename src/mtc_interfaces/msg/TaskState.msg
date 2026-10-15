# TaskState.msg（拟新增，来源：状态机设计 V0.1 §4.7）

std_msgs/Header header
string task_id
string state
string substate
uint8 completed_count
uint8 total_count
uint16 last_error_code
string detail
