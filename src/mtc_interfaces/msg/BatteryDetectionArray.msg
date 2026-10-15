# BatteryDetectionArray.msg（拟新增，来源：状态机设计 V0.1 §4.5）
# 场景中当前全部电池识别结果。感知节点按 RELIABLE + KEEP_LAST(5) 发布。

std_msgs/Header header
BatteryDetection[] detections
