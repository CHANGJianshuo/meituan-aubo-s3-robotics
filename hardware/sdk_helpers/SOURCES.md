# 来源与验证记录

查阅日期：2026-09-07。示例参数属于本测试的工程选择，不是厂家认证限值。

- [官方 SDK 安装](https://developer.aubo-robotics.cn/guide_sdk/4_dev_preparation/2_installation.html)：官方指向 PyPI 的 pyaubo_sdk，支持按版本安装。
- [官方 PC 与机器人网络设置](https://developer.aubo-robotics.cn/guide_sdk/5_quick_start/1_communication_set_up.html)：有线连接控制柜、同网段、RPC 端口检查。
- [官方 JSON-RPC 应用说明](https://developer.aubo-robotics.cn/application_notes/33-tutorial-for-the-json-rpc-api/index.html)：moveJoint 参数顺序、rad/rad/s/rad/s² 单位及返回码。
- [官方 MotionControl API](https://docs.aubo-robotics.cn/arcs_api/zh/classarcs_1_1common__interface_1_1MotionControl.html)：队列、执行 ID、运动方法。在线页是 0.26.0，本机 Python SDK 是 0.24.1；已核对本程序调用的类/方法存在，具体控制版本的兼容性仍需确认。
- [官方 ROS 2 驱动](https://github.com/AuboRobot/aubo_ros2_driver)：2026-09-07 查询 HEAD `85684075d6ff06c5385e39611208e99ebf0f94c6`；README 明确建议依据控制器校准补偿生成实机 URDF。
- [微软 WSL 网络说明](https://learn.microsoft.com/en-us/windows/wsl/networking)：NAT、mirrored 网络及防火墙相关说明。

本次只安装、导入官方 SDK，检查其 Python API，并运行本地离线测试。没有给任何机器人 IP 发送连接或运动请求。没有改动 Windows/WSL 网络配置、机器人参数或原比赛仓库。

## 本次本机验证

- `python -m unittest -v test_smoke`：23 项离线测试通过，覆盖越限、无效输入、硬件身份、非静止、网络/反馈异常、禁止重发、禁止自动返回与停止尝试。
- `smoke.py dry-run`：通过，不连接网络。
- 未填写 IP 的模板运行运动命令：按预期拒绝。
- WSL 实际模式：`mirrored`；原比赛仓库保持干净。

未验证：控制器固件与 SDK 的实际兼容性、控制器 runtime 准备流程、实际最小可执行速度/位移、真机停止行为及现场碰撞空间。
