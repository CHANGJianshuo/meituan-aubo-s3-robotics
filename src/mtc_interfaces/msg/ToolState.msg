# ToolState.msg（拟新增，来源：状态机设计 V0.1 §4.6）
# 末端工具状态。state 是当前估计，verified 是独立维度，二者不得混为一谈。

uint8 STATE_UNKNOWN=0
uint8 STATE_DETACHED=1
uint8 STATE_ENGAGING=2
uint8 STATE_ATTACHED=3
uint8 STATE_RELEASING=4
uint8 STATE_FAULT=5

uint8 EVIDENCE_NONE=0
uint8 EVIDENCE_COMMAND_ONLY=1
uint8 EVIDENCE_GEOMETRY=2
uint8 EVIDENCE_SENSOR_OR_VISION=3

std_msgs/Header header
uint8 state
uint8 evidence_level    # 支撑当前 state 的证据等级
bool verified           # 是否已被独立证据确认
string tool_type        # passive_hook_v1 / magnetic_latch_v2
string object_id
string detail
