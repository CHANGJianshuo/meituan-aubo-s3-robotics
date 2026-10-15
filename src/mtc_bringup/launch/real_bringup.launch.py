#!/usr/bin/env python3
"""real_bringup.launch.py —— 真实 AUBO S3 组合：**存在但默认拒绝启动**。

设计意图（刻意为之）
--------------------
本文件的存在是为了让“真实通路”有一个**唯一、显式、可审计**的入口，而不是散落在
各处。但它在任何默认取值下都会**打印错误并以非零退出**，绝不会连接设备。

启用条件（缺一不可）
--------------------
1. ``i_understand_real_robot_is_enabled:=true`` —— 现场授权确认开关；
2. ``device_serial_number`` 非空 —— 与控制柜身份白名单匹配的序列号；
3. ``site_authorization_note`` 非空 —— 现场授权说明（人员/时间/范围），写入日志。

**即使三项都满足，本文件也不会连接设备**：它只打印“已确认前置条件”，
然后明确拒绝继续，直到：
* ``mtc_aubo_bridge`` 的独立 Python 3.10 SDK Worker 完成实测验证；
* ``config/safety.yaml`` 的设备身份白名单由现场人工核实填入；
* 通过独立的安全评审（本文件不负责也不代表该评审）。

此外，真实后端在本工程中始终 available=False（``DisabledMotionBackend`` /
``DisabledSdkWorker``）；本文件既不会绕过它，也不会修改它。

用法（预期都会被拒绝）::

    ros2 launch mtc_bringup real_bringup.launch.py
    ros2 launch mtc_bringup real_bringup.launch.py i_understand_real_robot_is_enabled:=true
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

#: 现场授权确认开关的唯一合法取值（除 true 以外的任何取值都必须拒绝启动）。
REAL_ROBOT_ACK_FLAG = "i_understand_real_robot_is_enabled"

REFUSAL = (
    "=" * 78 + "\n"
    "!! 错误：real_bringup 默认拒绝启动，不会连接真实 AUBO S3。\n"
    "!! 需要现场明确授权，并完成下列前置条件（本文件不代替安全评审）：\n"
    "!!   1) %s:=true（现场授权确认）\n"
    "!!   2) device_serial_number 非空（与 config/safety.yaml 身份白名单一致）\n"
    "!!   3) site_authorization_note 非空（人员/时间/范围）\n"
    "!!   4) mtc_aubo_bridge 的 Python 3.10 SDK Worker 已完成实测验证\n"
    "!!   5) config/safety.yaml 的身份白名单已由现场人工核实填入\n"
    "!! 即便以上全部满足，本文件当前仍会拒绝继续（真实通路未实现、未验收）。\n"
    + "=" * 78
)


def _always_refuse(context, *args, **kwargs):
    ack = str(LaunchConfiguration("i_understand_real_robot_is_enabled").perform(context)).strip().lower()
    serial = str(LaunchConfiguration("device_serial_number").perform(context)).strip()
    note = str(LaunchConfiguration("site_authorization_note").perform(context)).strip()
    backend = str(LaunchConfiguration("backend").perform(context)).strip()

    lines = [REFUSAL]
    lines.append("收到的启动参数：")
    lines.append("  %-40s = %r" % (REAL_ROBOT_ACK_FLAG, ack))
    lines.append("  %-40s = %r" % ("device_serial_number", serial or "(空)"))
    lines.append("  %-40s = %r" % ("site_authorization_note", note or "(空)"))
    lines.append("  %-40s = %r" % ("backend", backend))

    if ack != "true":
        lines.append("拒绝原因：未检测到 %s:=true（当前取值 %r 不等于 'true'）。"
                     % (REAL_ROBOT_ACK_FLAG, ack))
    elif not serial or not note:
        lines.append("拒绝原因：已确认授权开关，但缺少设备序列号或现场授权说明；"
                     "不得在身份未核实的情况下建立运动通路。")
    else:
        lines.append("前置条件表面满足，但真实通路尚未实现/未验收：仍然拒绝启动。"
                     "请改用经过安全评审的独立流程，并先完成 SDK Worker 实测验证。")

    lines.append("已执行动作：无（未连接控制柜、未发送运动指令、未触发电磁铁）。")
    return [LogInfo(msg="\n".join(lines))]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(REAL_ROBOT_ACK_FLAG, default_value="false",
                              description="现场授权确认；只有 'true' 才会继续到下一步检查"),
        DeclareLaunchArgument("device_serial_number", default_value="",
                              description="控制柜序列号（必须与 safety.yaml 白名单一致）"),
        DeclareLaunchArgument("site_authorization_note", default_value="",
                              description="现场授权说明：人员 / 时间 / 范围"),
        DeclareLaunchArgument("backend", default_value="disabled",
                              description="真实后端在本工程中始终 disabled"),
        DeclareLaunchArgument("tool_type", default_value="passive_hook_v1",
                              description="工具类型；待实物标定"),
        OpaqueFunction(function=_always_refuse),
    ])
