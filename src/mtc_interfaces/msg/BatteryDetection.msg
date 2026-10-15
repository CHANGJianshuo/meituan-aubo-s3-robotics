# BatteryDetection.msg（拟新增，来源：状态机设计 V0.1 §4.5）
# 单块电池的颜色、本体位姿与提环抓取参考坐标系位姿。

uint8 COLOR_UNKNOWN=0
uint8 COLOR_RED=1
uint8 COLOR_BLUE=2
uint8 COLOR_YELLOW=3
uint8 COLOR_GREEN=4

string object_id                    # 稳定唯一 ID；同一物理电池跨帧应保持一致
uint8 color
geometry_msgs/PoseStamped body_pose # 电池主体中心
geometry_msgs/PoseStamped ring_pose # 提环局部抓取参考坐标系；轴向定义见 TF_CONVENTIONS.md
float32 confidence
bool ring_pose_valid                # false 时禁止使用 ring_pose 生成运动目标
